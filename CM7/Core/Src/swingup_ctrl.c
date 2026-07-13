/**
 ******************************************************************************
 * @file    swingup_ctrl.c
 * @brief   Energy-based swing-up -> LQR catch handoff (M7, stage 1).
 ******************************************************************************
 */

#include "swingup_ctrl.h"
#include "main.h"
#include "shared_state.h"
#include "state_est.h"
#include "balance_ctrl.h"
#include "odrive.h"
#include "as5047p.h"
#include "rail_limits.h"
#include <math.h>

#define G_MPS2          9.80665f
#define SWING_TICK_HZ   1000u          /* 1 kHz estimator cadence */

/* ---- Module state --------------------------------------------------------- */
static bool        s_running  = false;
static uint32_t    s_ticks    = 0u;    /* 1 kHz ticks since Start */
static float       s_x_mid    = 0.0f;  /* rail centre (rev), captured at Start */
static float       s_u_hold   = 0.0f;
static swing_end_t s_last_end = SWING_END_NONE;

static inline float clampf(float v, float lim)
{
    if (v >  lim) return  lim;
    if (v < -lim) return -lim;
    return v;
}

/* Cut power, hand the mode back to IDLE (coast). ISR context. */
static void abort_run(swing_end_t why)
{
#if CTRL_INPUT_ACCEL
    (void)Odrive_SetInputVel(0.0f, 0.0f);   /* zero target, then disarm */
#else
    (void)Odrive_SetInputTorque(0.0f);
#endif
    (void)Odrive_SetAxisState(ODRIVE_AXIS_STATE_IDLE);
    g_shared.motor_command   = 0.0f;
    g_shared.controller_mode = CTRL_MODE_IDLE;   /* pot_ctrl resyncs */
    s_u_hold   = 0.0f;
    s_running  = false;
    s_last_end = why;
}

bool SwingupCtrl_ArmOK(void)
{
    /* No model, no swing-up: the energy law needs ml/Jt from the ID fit. */
    if (SWING_ML_KGM <= 0.0f || SWING_JT_KGM2 <= 0.0f) return false;

    if (!StateEst_IsZeroed())  return false;
    if (!RailLimits_IsHomed()) return false;
    if (RailLimits_Active())   return false;

    state_est_t e;
    StateEst_Get(&e);

    /* Cart centred-ish and still; the pendulum may hang anywhere. */
    if (fabsf(e.x_dot) >= SWING_ARM_XDOT)                  return false;
    if (e.x < RailLimits_SoftMin() + SWING_ARM_X_MARGIN)   return false;
    if (e.x > RailLimits_SoftMax() - SWING_ARM_X_MARGIN)   return false;

    return true;
}

void SwingupCtrl_Start(void)
{
    s_x_mid    = 0.5f * (RailLimits_SoftMin() + RailLimits_SoftMax());
    s_ticks    = 0u;
    s_u_hold   = 0.0f;
    s_last_end = SWING_END_NONE;
    s_running  = true;
}

float SwingupCtrl_Tick(bool est_tick)
{
    if (!s_running) return 0.0f;
    if (!est_tick)  return s_u_hold;   /* hold between estimator updates */

    s_ticks++;

    /* ---- Fault aborts (sensor/actuator health — arch review F1) ------ *
     * Before the state envelope: a frozen theta or x sits inside the
     * envelope forever, and a self-disarmed axis still heartbeats. */
    if (!AS5047P_AngleFresh(0u)) {
        abort_run(SWING_END_ENC);
        return 0.0f;
    }
    if (!Odrive_EstimatesFresh()) {
        g_shared.m7_fault_flags |= M7_FAULT_ODRIVE_AXIS;
        abort_run(SWING_END_CART);
        return 0.0f;
    }
    if (s_ticks > ODRIVE_AXIS_SETTLE_MS &&   /* 1 kHz ticks == ms */
        (Odrive_GetAxisError() != 0u ||
         Odrive_GetAxisState() != ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL)) {
        g_shared.m7_fault_flags |= M7_FAULT_ODRIVE_AXIS;
        abort_run(SWING_END_AXIS);
        return 0.0f;
    }

    state_est_t e;
    StateEst_Get(&e);

    /* ---- Abort envelope ---------------------------------------------- */
    if (e.x < RailLimits_SoftMin() || e.x > RailLimits_SoftMax()) {
        abort_run(SWING_END_RAIL);
        return 0.0f;
    }
    if (fabsf(e.x_dot) > SWING_XDOT_MAX) {
        abort_run(SWING_END_XDOT);
        return 0.0f;
    }
    if (s_ticks > (SWING_TIMEOUT_MS * (SWING_TICK_HZ / 1000u))) {
        abort_run(SWING_END_TIMEOUT);
        return 0.0f;
    }

    /* ---- Catch: inside the basin -> hand off to the LQR --------------- *
     * Same tick: Start() re-seeds the balance slew/reference, and the mode
     * switch makes control_loop dispatch BalanceCtrl_Tick next cycle. The
     * torque returned below still goes out this tick (keeps the stream and
     * the ODrive watchdog continuous across the handoff).                 */
    if (fabsf(e.theta)     < SWING_CATCH_THETA &&
        fabsf(e.theta_dot) < SWING_CATCH_THETA_DOT) {
        BalanceCtrl_Start();
        g_shared.controller_mode = CTRL_MODE_BALANCE;
        s_running  = false;
        s_last_end = SWING_END_CAUGHT;
        return s_u_hold;   /* one bridging frame; LQR owns the next tick */
    }

    /* ---- Energy pump + cart containment ------------------------------- */
    float E = 0.5f * SWING_JT_KGM2 * e.theta_dot * e.theta_dot
            + G_MPS2 * SWING_ML_KGM * (cosf(e.theta) - 1.0f);

    float u = SWING_KE * (0.0f - E) * e.theta_dot * cosf(e.theta)
            - SWING_KX  * (e.x - s_x_mid)
            - SWING_KXD * e.x_dot;

    u = clampf(u, SWING_U_MAX);   /* Nm or rev/s^2 per control_input.h */

    s_u_hold = u;
    g_shared.motor_command = u;   /* telemetry mirror */
    return u;
}

bool SwingupCtrl_IsRunning(void) { return s_running; }

void SwingupCtrl_Stop(void)
{
    if (s_running) {
        __disable_irq();
        s_running  = false;
        s_u_hold   = 0.0f;
        s_last_end = SWING_END_EXTERN;
        __enable_irq();
    }
}

swing_end_t SwingupCtrl_LastEnd(void) { return s_last_end; }
