/**
 ******************************************************************************
 * @file    sysid.h
 * @brief   System-ID excitation generator (M7): bounded torque test signals.
 *
 *  Runs under CTRL_MODE_SYSID. On each 1 kHz estimator tick it produces the
 *  excitation torque u(t); control_loop.c streams it via Set_Input_Torque at
 *  5 kHz and pushes (tick, theta, theta_dot, x, x_dot, u) into the shared
 *  idlog ring for the M4 -> UDP:5007 streamer. MATLAB (id_logger.m) starts
 *  runs and captures the data for grey-box fitting.
 *
 *  Signal types (g_shared.sysid_type):
 *    0 FREE   — u = 0, axis stays IDLE (unpowered). Log a hand-started free
 *               swing: link inertia, CoM, joint damping. Needs no homing.
 *    1 CHIRP  — amp * sin(phase), instantaneous frequency swept linearly
 *               f0 -> f1 over the duration. The workhorse for grey-box ID.
 *    2 PRBS   — +/-amp pseudo-random binary sequence (15-bit LFSR), chip
 *               rate 2*f1 (bandwidth ~f1). Good for friction/backlash.
 *    3 STEP   — constant amp for the duration (abort guards end it early if
 *               the cart runs). Cart mass + Coulomb/viscous friction.
 *
 *  Safety: amplitude/duration re-clamped here regardless of what M4 passed;
 *  driving types (1-3) require a homed rail and abort when the cart leaves
 *  the soft band or overspeeds. RailLimits + the ODrive vel_limit remain the
 *  outer layers. Free-swing skips the guards — the axis is unpowered.
 ******************************************************************************
 */

#ifndef SYSID_H
#define SYSID_H

#include <stdint.h>
#include <stdbool.h>
#include "control_input.h"   /* torque vs accel input path selection */

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Types --------------------------------------------------------------- */
typedef enum {
    SYSID_FREE  = 0,
    SYSID_CHIRP = 1,
    SYSID_PRBS  = 2,
    SYSID_STEP  = 3
} sysid_type_t;

/* ---- Firmware-side hard limits (M4 requests are re-clamped to these) ---- */
#define SYSID_TORQUE_MAX_NM   0.6f    /* excitation ceiling, Nm (torque path) */

/* Accel-input path (CTRL_INPUT_ACCEL=1): excitation amplitude is a cart
 * accel [rev/s^2]. Note the accel paradigm mostly obviates driven ID —
 * free swings alone fit the remaining (pendulum) parameters — but driven
 * types stay available for validation. PLACEHOLDER clamp. */
#define SYSID_ACCEL_MAX_REV_S2  25.0f

#if CTRL_INPUT_ACCEL
#define SYSID_U_MAX   SYSID_ACCEL_MAX_REV_S2
#else
#define SYSID_U_MAX   SYSID_TORQUE_MAX_NM
#endif
#define SYSID_DUR_MAX_S       60.0f
#define SYSID_XDOT_MAX        6.0f    /* rev/s — abort a runaway cart */
#define SYSID_F_MIN_HZ        0.05f
#define SYSID_F_MAX_HZ        50.0f

/* ---- API ----------------------------------------------------------------- */

/* Validate + latch a run from the g_shared.sysid_* params. Returns false if
 * refused (unknown type, or a driving type without a homed rail). Main-loop
 * context (pot_ctrl), BEFORE switching controller_mode to SYSID. */
bool  SysId_Start(void);

/* True while a run is active. */
bool  SysId_IsRunning(void);

/* True if the active run commands torque (type != FREE): control_loop must
 * stream Set_Input_Torque; for FREE it must leave the axis alone (explicit
 * watchdog-feed path). */
bool  SysId_IsDriving(void);

/* Per-cycle update from the control-loop ISR while mode == SYSID.
 * est_tick = true on the 1 kHz estimator ticks. Returns the torque to send
 * (0 for FREE). On completion/abort it has already idled the axis (driving
 * types) and set controller_mode = IDLE. */
float SysId_Tick(bool est_tick);

/* Operator stop (main-loop context). Caller owns the axis transition. */
void  SysId_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSID_H */
