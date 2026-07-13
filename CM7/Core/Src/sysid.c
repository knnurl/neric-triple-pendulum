/**
 ******************************************************************************
 * @file    sysid.c
 * @brief   System-ID excitation generator (M7).
 ******************************************************************************
 */

#include "sysid.h"
#include "main.h"
#include "shared_state.h"
#include "state_est.h"
#include "odrive.h"
#include "as5047p.h"
#include "rail_limits.h"
#include <math.h>

#ifndef M_TWOPI_F
#define M_TWOPI_F   6.28318530717958647692f
#endif

#define SYSID_TICK_DT   (1.0f / 1000.0f)   /* 1 kHz estimator tick */

/* ---- Latched run parameters (Start) -------------------------------------- */
static sysid_type_t s_type = SYSID_FREE;
static float s_amp  = 0.0f;
static float s_dur  = 0.0f;
static float s_f0   = 0.1f;
static float s_f1   = 10.0f;

/* ---- Run state (ISR) ------------------------------------------------------ */
static bool     s_running = false;
static uint32_t s_n       = 0u;       /* 1 kHz ticks since start */
static float    s_phase   = 0.0f;     /* chirp phase accumulator, rad */
static float    s_u_hold  = 0.0f;

/* PRBS: 15-bit maximal LFSR (x^15 + x^14 + 1), clocked at the chip rate. */
static uint16_t s_lfsr       = 0x5A5Au;
static uint32_t s_chip_ticks = 25u;   /* estimator ticks per PRBS chip */
static uint32_t s_chip_cnt   = 0u;
static float    s_prbs_u     = 0.0f;

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* End the run. Driving types: zero the torque and idle the axis. Hand the
 * mode back to IDLE so pot_ctrl's EXTERNAL state resyncs. ISR context. */
static void finish(void)
{
    if (s_type != SYSID_FREE) {
#if CTRL_INPUT_ACCEL
        (void)Odrive_SetInputVel(0.0f, 0.0f);   /* zero target, then disarm */
#else
        (void)Odrive_SetInputTorque(0.0f);
#endif
        (void)Odrive_SetAxisState(ODRIVE_AXIS_STATE_IDLE);
    }
    g_shared.motor_command   = 0.0f;
    g_shared.controller_mode = CTRL_MODE_IDLE;
    s_u_hold  = 0.0f;
    s_running = false;
}

bool SysId_Start(void)
{
    uint8_t t = g_shared.sysid_type;
    if (t > (uint8_t)SYSID_STEP) return false;

    /* Driving signals move the cart: require the rail frame so the soft-band
     * abort guard means something. Free swing is unpowered — always OK. */
    if (t != (uint8_t)SYSID_FREE && !RailLimits_IsHomed()) return false;

    s_type = (sysid_type_t)t;
    s_amp  = clampf(g_shared.sysid_amp_nm, 0.0f, SYSID_U_MAX);  /* Nm or rev/s^2 */
    s_dur  = clampf(g_shared.sysid_dur_s,  0.1f, SYSID_DUR_MAX_S);
    s_f0   = clampf(g_shared.sysid_f0_hz,  SYSID_F_MIN_HZ, SYSID_F_MAX_HZ);
    s_f1   = clampf(g_shared.sysid_f1_hz,  SYSID_F_MIN_HZ, SYSID_F_MAX_HZ);

    /* PRBS chip rate 2*f1 -> bandwidth ~f1 (clamped to >= 2 ticks). */
    float chip_s = 0.5f / s_f1;
    s_chip_ticks = (uint32_t)(chip_s / SYSID_TICK_DT);
    if (s_chip_ticks < 2u) s_chip_ticks = 2u;

    s_n        = 0u;
    s_phase    = 0.0f;
    s_chip_cnt = 0u;
    s_lfsr     = 0x5A5Au;
    s_prbs_u   = s_amp;
    s_u_hold   = 0.0f;
    s_running  = true;
    return true;
}

bool SysId_IsRunning(void)  { return s_running; }
bool SysId_IsDriving(void)  { return s_running && (s_type != SYSID_FREE); }

float SysId_Tick(bool est_tick)
{
    if (!s_running) return 0.0f;
    if (!est_tick)  return s_u_hold;

    float t = (float)s_n * SYSID_TICK_DT;
    s_n++;

    /* ---- Termination / guards ---------------------------------------- */
    if (t >= s_dur) { finish(); return 0.0f; }

    /* Fault guards (arch review F1). A stale link-1 encoder ends ANY run —
     * theta frozen at last-known makes the captured data garbage, and a
     * driving run is unsafe on top. The cart/axis checks only apply to
     * driving types: a free swing is unpowered and fits theta alone. */
    if (!AS5047P_AngleFresh(0u)) {
        finish();
        return 0.0f;
    }
    if (s_type != SYSID_FREE) {
        if (!Odrive_EstimatesFresh()) {
            g_shared.m7_fault_flags |= M7_FAULT_ODRIVE_AXIS;
            finish();
            return 0.0f;
        }
        if (s_n > ODRIVE_AXIS_SETTLE_MS &&   /* 1 kHz ticks == ms */
            (Odrive_GetAxisError() != 0u ||
             Odrive_GetAxisState() != ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL)) {
            g_shared.m7_fault_flags |= M7_FAULT_ODRIVE_AXIS;
            finish();
            return 0.0f;
        }

        state_est_t e;
        StateEst_Get(&e);
        if (e.x < RailLimits_SoftMin() || e.x > RailLimits_SoftMax() ||
            fabsf(e.x_dot) > SYSID_XDOT_MAX) {
            finish();
            return 0.0f;
        }
    }

    /* ---- Signal generation -------------------------------------------- */
    float u = 0.0f;
    switch (s_type) {

    case SYSID_CHIRP: {
        /* Incremental phase keeps float precision over long runs. */
        float f_inst = s_f0 + (s_f1 - s_f0) * (t / s_dur);
        s_phase += M_TWOPI_F * f_inst * SYSID_TICK_DT;
        if (s_phase > M_TWOPI_F) s_phase -= M_TWOPI_F;
        u = s_amp * sinf(s_phase);
        break;
    }

    case SYSID_PRBS:
        if (s_chip_cnt == 0u) {
            /* Galois step, taps 15 & 14 (maximal length 32767). */
            uint16_t bit = (uint16_t)(((s_lfsr >> 0) ^ (s_lfsr >> 1)) & 1u);
            s_lfsr = (uint16_t)((s_lfsr >> 1) | (bit << 14));
            s_prbs_u = (s_lfsr & 1u) ? s_amp : -s_amp;
            s_chip_cnt = s_chip_ticks;
        }
        s_chip_cnt--;
        u = s_prbs_u;
        break;

    case SYSID_STEP:
        u = s_amp;
        break;

    case SYSID_FREE:
    default:
        u = 0.0f;
        break;
    }

    s_u_hold = u;
    g_shared.motor_command = u;   /* telemetry mirror */
    return u;
}

void SysId_Stop(void)
{
    /* Operator stop from main-loop context — caller (pot_ctrl exit_to_idle)
     * owns the axis IDLE transition; just stop generating. */
    __disable_irq();
    s_running = false;
    s_u_hold  = 0.0f;
    __enable_irq();
}
