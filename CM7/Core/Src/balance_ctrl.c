/**
 ******************************************************************************
 * @file    balance_ctrl.c
 * @brief   Stage-1 balance controller: LQR state feedback -> torque (M7).
 ******************************************************************************
 */

#include "balance_ctrl.h"
#include "main.h"
#include "shared_state.h"
#include "state_est.h"
#include "odrive.h"
#include "as5047p.h"
#include "rail_limits.h"
#include <math.h>

/* ---- Module state -------------------------------------------------------- */
static const float s_K_base[4] = BAL_K_BASE;

static bool      s_running   = false;
static float     s_x_ref     = 0.0f;   /* regulation target (rail frame, rev) */
static float     s_u_hold    = 0.0f;   /* torque held between 1 kHz updates */
static uint32_t  s_run_ticks = 0u;     /* 1 kHz ticks since Start (== ms) */
static bal_end_t s_last_end  = BAL_END_NONE;

/* Fader-during-balance input (C15, see header). Volatile: written from the
 * main loop, read on the 1 kHz tick; a single aligned float store is
 * atomic. Compiled unconditionally (trivial); CONSUMED only under the
 * BAL_FADER_STEER / BAL_FADER_POKE flags. */
static volatile float s_fader_in = 0.0f;

void BalanceCtrl_SetFaderInput(float norm)
{
    if (norm >  1.0f) norm =  1.0f;
    if (norm < -1.0f) norm = -1.0f;
    s_fader_in = norm;
}

static inline float clampf(float v, float lim)
{
    if (v >  lim) return  lim;
    if (v < -lim) return -lim;
    return v;
}

/* Cut power and hand the mode back to IDLE (coast — for a falling pendulum
 * a freewheeling cart is kinder than a torque fight or a hard stop). ISR
 * context: controller_mode/motor_command writes need no masking here, the
 * TIM7 ISR is the highest-priority writer. */
static void abort_run(bal_end_t why)
{
#if CTRL_INPUT_ACCEL
    (void)Odrive_SetInputVel(0.0f, 0.0f);   /* zero target, then disarm */
#else
    (void)Odrive_SetInputTorque(0.0f);
#endif
    (void)Odrive_SetAxisState(ODRIVE_AXIS_STATE_IDLE);
    g_shared.motor_command   = 0.0f;
    g_shared.controller_mode = CTRL_MODE_IDLE;   /* pot_ctrl resyncs on this */
    s_u_hold   = 0.0f;
    s_running  = false;
    s_last_end = why;
}

bool BalanceCtrl_ArmOK(void)
{
    if (!StateEst_IsZeroed())    return false;
    if (!RailLimits_IsHomed())   return false;
    if (RailLimits_Active())     return false;

    state_est_t e;
    StateEst_Get(&e);

    if (fabsf(e.theta)     >= BAL_ARM_THETA_RAD) return false;
    if (fabsf(e.theta_dot) >= BAL_ARM_THETA_DOT) return false;
    if (fabsf(e.x_dot)     >= BAL_ARM_XDOT)      return false;

    if (e.x < RailLimits_SoftMin() + BAL_ARM_X_MARGIN_REV) return false;
    if (e.x > RailLimits_SoftMax() - BAL_ARM_X_MARGIN_REV) return false;

    return true;
}

void BalanceCtrl_Start(void)
{
    s_x_ref     = 0.5f * (RailLimits_SoftMin() + RailLimits_SoftMax());
    s_u_hold    = 0.0f;
    s_run_ticks = 0u;
    s_fader_in  = 0.0f;   /* C15: no carried-over deflection on a new run */
    s_last_end  = BAL_END_NONE;
    s_running   = true;
}

float BalanceCtrl_Tick(bool est_tick)
{
    if (!s_running) return 0.0f;
    if (!est_tick)  return s_u_hold;   /* hold between estimator updates */

    s_run_ticks++;

    /* ---- Fault aborts (sensor/actuator health — arch review F1) ------ *
     * Checked BEFORE the state envelope: when these fire, the state data
     * is exactly what can no longer be trusted. A frozen theta or x sits
     * inside the envelope forever. */
    if (!AS5047P_AngleFresh(0u)) {
        abort_run(BAL_END_ENC);
        return 0.0f;
    }
    if (!Odrive_EstimatesFresh()) {
        g_shared.m7_fault_flags |= M7_FAULT_ODRIVE_AXIS;
        abort_run(BAL_END_CART);
        return 0.0f;
    }
    if (s_run_ticks > ODRIVE_AXIS_SETTLE_MS &&   /* 1 kHz ticks == ms */
        (Odrive_GetAxisError() != 0u ||
         Odrive_GetAxisState() != ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL)) {
        g_shared.m7_fault_flags |= M7_FAULT_ODRIVE_AXIS;
        abort_run(BAL_END_AXIS);
        return 0.0f;
    }

    state_est_t e;
    StateEst_Get(&e);   /* same-ISR snapshot: the masking is a no-op cost */

    /* ---- Abort envelope --------------------------------------------- */
    if (fabsf(e.theta) > BAL_ABORT_THETA_RAD) { abort_run(BAL_END_THETA); return 0.0f; }
    if (fabsf(e.x_dot) > BAL_ABORT_XDOT)      { abort_run(BAL_END_XDOT);  return 0.0f; }
    if (e.x < RailLimits_SoftMin() || e.x > RailLimits_SoftMax()) {
        abort_run(BAL_END_RAIL);
        return 0.0f;
    }

#if BAL_FADER_STEER
    /* ---- C15-B: fader steers the regulation target ------------------- *
     * Reference motion only — u is untouched, the envelope above and the
     * clamp below are exactly the fixed-setpoint controller's. dt = 1 ms
     * (this branch runs on the 1 kHz estimator tick). */
    s_x_ref += s_fader_in * BAL_STEER_XREF_RATE_REV_S * 0.001f;
    {
        float lo = RailLimits_SoftMin() + BAL_ARM_X_MARGIN_REV;
        float hi = RailLimits_SoftMax() - BAL_ARM_X_MARGIN_REV;
        if (s_x_ref < lo) s_x_ref = lo;
        if (s_x_ref > hi) s_x_ref = hi;
    }
#endif

    /* ---- LQR: u = -K x  (K = flash base + live GUI deltas) ----------- */
    float K0 = s_K_base[0] + g_shared.lqr_gain_delta[0];
    float K1 = s_K_base[1] + g_shared.lqr_gain_delta[1];
    float K2 = s_K_base[2] + g_shared.lqr_gain_delta[2];
    float K3 = s_K_base[3] + g_shared.lqr_gain_delta[3];

    float u = -(K0 * (e.x - s_x_ref)
              + K1 * e.x_dot
              + K2 * e.theta
              + K3 * e.theta_dot);

#if BAL_FADER_POKE
    /* ---- C15-A: bounded additive disturbance ------------------------- *
     * Injected BEFORE the final clamp: at full deflection the controller
     * still holds (1 - BAL_POKE_U_FRAC) of its authority, and the total
     * can never exceed BAL_U_MAX. */
    u += s_fader_in * (BAL_POKE_U_FRAC * BAL_U_MAX);
#endif

    u = clampf(u, BAL_U_MAX);   /* Nm or rev/s^2 per control_input.h */

    s_u_hold = u;
    g_shared.motor_command = u;   /* telemetry: "Motor cmd" = live torque */
    return u;
}

bool BalanceCtrl_IsRunning(void) { return s_running; }

void BalanceCtrl_Stop(void)
{
    if (s_running) {
        s_running  = false;
        s_u_hold   = 0.0f;
        s_last_end = BAL_END_EXTERN;
    }
}

bal_end_t BalanceCtrl_LastEnd(void) { return s_last_end; }
