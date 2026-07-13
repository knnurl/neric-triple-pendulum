/**
 ******************************************************************************
 * @file    swingup_ctrl.h
 * @brief   Energy-based swing-up for the single link (M7, stage 1).
 *
 *  Astrom–Furuta energy shaping. Pendulum energy about the pivot, zero at
 *  upright rest (hanging rest = -2*g*ml):
 *
 *      E = 1/2 * Jt * theta_dot^2 + g*ml*(cos(theta) - 1)
 *
 *  Cart force pumps energy fastest along theta_dot*cos(theta), so:
 *
 *      u = SWING_KE * (0 - E) * theta_dot * cos(theta)     (energy pump)
 *        - SWING_KX * (x - x_mid) - SWING_KXD * x_dot      (cart containment)
 *
 *  clamped to SWING_TORQUE_MAX_NM. The pump term fades as E -> 0, so the
 *  pendulum arrives at the top slow; when it enters the capture basin
 *  (|theta| < SWING_CATCH_THETA and |theta_dot| < SWING_CATCH_THETA_DOT)
 *  the module hands off IN THE SAME TICK to the balance LQR:
 *  BalanceCtrl_Start() + controller_mode = BALANCE. The axis is already in
 *  TORQUE/PASSTHROUGH closed loop, so the handoff is a pure software switch.
 *
 *  Model constants SWING_ML / SWING_JT come from the system-ID fit
 *  (id_fit.m: params.ml, params.Jt). They ship as 0.0f and the arm gate
 *  REFUSES to start until real values are pasted in — swing-up without a
 *  model is flailing. Gains are pre-tuned offline in swingup_design.m
 *  against the same fitted model.
 *
 *  Safety envelope (mirrors balance/sysid):
 *    ARM   — upright zeroed + homed + model params set + cart mid-rail +
 *            cart still. Pendulum may start anywhere (hanging is normal;
 *            already-upright just catches immediately).
 *    ABORT — cart leaves the soft band, |x_dot| runaway, or
 *            SWING_TIMEOUT_MS without capture: torque 0, axis IDLE, mode
 *            IDLE. RailLimits hard cut + switches remain the outer layers.
 *            Fault aborts (arch review 2026-07-11): also aborts on link-1
 *            encoder stale, encoder-estimate stream stale, and — after
 *            ODRIVE_AXIS_SETTLE_MS — axis error / axis left CLOSED_LOOP
 *            (a frozen state is invisible to the envelope above).
 *
 *  Contexts: ArmOK/Start from the main loop (pot_ctrl); Tick from the TIM7
 *  ISR (math on the 1 kHz estimator tick, torque held between, streamed at
 *  5 kHz = ODrive watchdog feed).
 ******************************************************************************
 */

#ifndef SWINGUP_CTRL_H
#define SWINGUP_CTRL_H

#include <stdint.h>
#include <stdbool.h>
#include "control_input.h"   /* torque vs accel input path selection */

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Model constants (paste from id_fit results — 0.0 refuses to arm) --- */
#define SWING_ML_KGM          0.0f    /* params.ml  [kg m]   — SET FROM ID */
#define SWING_JT_KGM2         0.0f    /* params.Jt  [kg m^2] — SET FROM ID */

/* ---- Tunables (pre-tune in swingup_design.m, trim on bench) ------------- */
#define SWING_KE              2.0f    /* energy pump gain [Nm / (J rad/s)] */
#define SWING_KX              0.05f   /* cart centering  [Nm / rev] */
#define SWING_KXD             0.02f   /* cart damping    [Nm / (rev/s)] */
#define SWING_TORQUE_MAX_NM   0.6f    /* output clamp, Nm (torque path) */

/* Accel-input path (CTRL_INPUT_ACCEL=1): u is cart accel [rev/s^2]; the
 * classic Åström pump is naturally acceleration-based, so the law above
 * keeps its structure — but SWING_KE/KX/KXD change units and MUST come
 * from the accel variant of swingup_design.m. PLACEHOLDER clamp. */
#define SWING_ACCEL_MAX_REV_S2  30.0f

#if CTRL_INPUT_ACCEL
#define SWING_U_MAX   SWING_ACCEL_MAX_REV_S2
#else
#define SWING_U_MAX   SWING_TORQUE_MAX_NM
#endif

/* Capture basin: hand off to the LQR inside this window. Slightly wider in
 * rate than the balance ARM gate — the LQR grabs a moving pendulum better
 * than the static gate implies; verify the margin in swingup_design.m. */
#define SWING_CATCH_THETA     0.17f   /* ~10 deg */
#define SWING_CATCH_THETA_DOT 1.5f    /* rad/s */

/* Envelope */
#define SWING_ARM_X_MARGIN    1.5f    /* rev inside each soft limit at start */
#define SWING_ARM_XDOT        0.5f    /* rev/s — cart still at start */
#define SWING_XDOT_MAX        8.0f    /* rev/s — runaway abort */
#define SWING_TIMEOUT_MS      30000u  /* no capture -> give up */

/* ---- API ----------------------------------------------------------------- */
bool  SwingupCtrl_ArmOK(void);

/* Reset run state. Call right before switching controller_mode to SWINGUP
 * (axis already in TORQUE/PASSTHROUGH closed loop). */
void  SwingupCtrl_Start(void);

/* Per-cycle update from the control-loop ISR while mode == SWINGUP.
 * est_tick = true on 1 kHz estimator ticks. Returns clamped torque. On
 * capture it has switched controller_mode to BALANCE (keep streaming — the
 * balance tick takes over next cycle); on abort it has idled the axis and
 * set mode IDLE. */
float SwingupCtrl_Tick(bool est_tick);

bool  SwingupCtrl_IsRunning(void);

/* Operator/mode-change stop (main-loop context). Caller owns the axis. */
void  SwingupCtrl_Stop(void);

/* Why the last run ended. */
typedef enum {
    SWING_END_NONE = 0,
    SWING_END_CAUGHT,    /* handed off to BALANCE */
    SWING_END_RAIL,      /* left the soft band */
    SWING_END_XDOT,      /* cart runaway */
    SWING_END_TIMEOUT,   /* no capture within SWING_TIMEOUT_MS */
    SWING_END_EXTERN,    /* stopped externally */
    /* Fault aborts (arch review 2026-07-11, batch B) — append-only: */
    SWING_END_ENC,       /* link-1 encoder stale — theta frozen at last-known */
    SWING_END_AXIS,      /* ODrive axis_error set or axis left CLOSED_LOOP */
    SWING_END_CART       /* encoder-estimate stream stale — x/x_dot frozen */
} swing_end_t;
swing_end_t SwingupCtrl_LastEnd(void);

#ifdef __cplusplus
}
#endif

#endif /* SWINGUP_CTRL_H */
