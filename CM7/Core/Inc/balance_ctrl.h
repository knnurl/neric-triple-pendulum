/**
 ******************************************************************************
 * @file    balance_ctrl.h
 * @brief   Stage-1 balance controller: LQR state feedback -> motor torque (M7).
 *
 *  u = -( K[0]*(x - x_ref) + K[1]*x_dot + K[2]*theta + K[3]*theta_dot )
 *
 *  State order [x, x_dot, theta, theta_dot] — matches state_est.h. x_ref is
 *  the middle of the rail's soft band (drift-free regulation target). Gains
 *  are BAL_K_BASE + g_shared.lqr_gain_delta[0..3] so the GUI's +K/-K spinner
 *  path tunes them live. BAL_K_BASE ships all-zero: with K = 0 the loop
 *  outputs 0 Nm — safe to arm before the system-ID gains exist.
 *
 *  Safety envelope (per Eltohamy & Kuo 1997: links held within ~20 deg,
 *  saturation monitored):
 *    ARM gate  — upright zero captured, rail homed, |theta| < BAL_ARM_THETA,
 *                |theta_dot| / |x_dot| small, cart inside the soft band with
 *                margin. Checked by pot_ctrl BEFORE entering BALANCE.
 *    ABORT     — |theta| > BAL_ABORT_THETA (pendulum lost), |x_dot| runaway,
 *                or cart leaves the soft band: torque 0, axis IDLE (coast),
 *                controller_mode -> IDLE. RailLimits' hard-limit cut and the
 *                physical switches remain the outer layers.
 *    FAULT ABORT (arch review 2026-07-11) — the state envelope cannot see a
 *                FROZEN state: a dead encoder holds the last-known theta and
 *                a self-disarmed ODrive keeps sending heartbeats. So the
 *                tick also aborts on: link-1 encoder stale
 *                (AS5047P_AngleFresh), encoder-estimate stream stale
 *                (Odrive_EstimatesFresh), and — after ODRIVE_AXIS_SETTLE_MS
 *                of run time — axis_error != 0 or axis not in CLOSED_LOOP.
 *    CLAMP     — |u| <= BAL_TORQUE_MAX_NM every tick, always.
 *
 *  Contexts: ArmOK/Start from main loop (pot_ctrl); Tick from the TIM7 ISR.
 *  The LQR math runs on the 1 kHz estimator tick; the returned torque is
 *  held between updates and streamed at 5 kHz (which feeds the ODrive
 *  watchdog).
 ******************************************************************************
 */

#ifndef BALANCE_CTRL_H
#define BALANCE_CTRL_H

#include <stdint.h>
#include <stdbool.h>
#include "control_input.h"   /* torque vs accel input path selection */

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Tunables (bench placeholders — revisit after system ID) ------------ */
#define BAL_TORQUE_MAX_NM     0.8f     /* hard output clamp, Nm (torque path) */

/* Accel-input path (CTRL_INPUT_ACCEL=1): u is cart acceleration [rev/s^2].
 * PLACEHOLDER — set from the accel variant of the lqr_design sweep before
 * ever flipping the flag. K units change with the path too: BAL_K_BASE
 * must come from the matching design variant (Nm-per-state vs
 * rev/s^2-per-state) — mixed units balance nothing. */
#define BAL_ACCEL_MAX_REV_S2  40.0f

/* Active output clamp for the selected input path. */
#if CTRL_INPUT_ACCEL
#define BAL_U_MAX   BAL_ACCEL_MAX_REV_S2
#else
#define BAL_U_MAX   BAL_TORQUE_MAX_NM
#endif

/* ---- Fader-during-balance demo interactions (C15) — DORMANT by default --
 * Two independent, compile-time-gated ways the physical fader (PA3 pot)
 * can interact with a RUNNING balance. Both ship 0 so the first-balance
 * campaign runs the boring fixed-setpoint controller; flip ONLY after U7
 * passes. When 0, the pot_ctrl feed and the consumption below compile out
 * entirely — zero object-code change to the untested controller.
 *
 *  BAL_FADER_STEER (option B, recommended for the public CHALLENGE demo):
 *      fader deflection moves the regulation target: x_ref += norm *
 *      BAL_STEER_XREF_RATE_REV_S * dt, clamped inside the soft band with
 *      the arm margin. The LQR TRACKS a moving setpoint while balancing —
 *      "drive the pendulum around". Never touches u: envelope, clamps and
 *      rail layers all unchanged; an untrackable demand just grows theta
 *      into the normal abort.
 *
 *  BAL_FADER_POKE (option A, EXPERT-grade disturbance-rejection demo):
 *      fader deflection adds a bounded disturbance to the control output:
 *      u += norm * (BAL_POKE_U_FRAC * BAL_U_MAX), BEFORE the final BAL_U_MAX
 *      clamp — the controller always keeps >= (1 - frac) authority.
 *
 * Both are fed by pot_ctrl only while ctrl_src == POT, behind a
 * zero-crossing gate (the fader must pass through centre once after each
 * balance entry before it goes live — grabbing it mid-run cannot slam an
 * offset in). Units are normalized [-1, +1], so both work identically on
 * the torque and accel input paths. */
#ifndef BAL_FADER_STEER
#define BAL_FADER_STEER   0
#endif
#ifndef BAL_FADER_POKE
#define BAL_FADER_POKE    0
#endif

#define BAL_STEER_XREF_RATE_REV_S  1.0f   /* x_ref rate at full deflection */
#define BAL_POKE_U_FRAC            0.25f  /* poke ceiling as fraction of BAL_U_MAX */

#define BAL_ARM_THETA_RAD     0.14f    /* ~8 deg  — must start near upright */
#define BAL_ARM_THETA_DOT     0.6f     /* rad/s   — and nearly still */
#define BAL_ARM_XDOT          0.5f     /* rev/s */
#define BAL_ARM_X_MARGIN_REV  1.0f     /* min distance inside each soft limit */

#define BAL_ABORT_THETA_RAD   0.26f    /* ~15 deg — pendulum lost, cut + coast */
#define BAL_ABORT_XDOT        8.0f     /* rev/s   — cart runaway */

/* Baseline LQR gains (flash). All-zero until system ID + lqr() produce real
 * ones; live-tune via lqr_gain_delta meanwhile. Units: Nm per (rev, rev/s,
 * rad, rad/s) respectively. */
#define BAL_K_BASE            { 0.0f, 0.0f, 0.0f, 0.0f }

/* ---- API ----------------------------------------------------------------- */

/* All arm-gate conditions currently met? (main-loop context) */
bool  BalanceCtrl_ArmOK(void);

/* Reset internal state and capture x_ref. Call right before switching
 * controller_mode to BALANCE (after the ODrive is in TORQUE/PASSTHROUGH
 * closed loop). */
void  BalanceCtrl_Start(void);

/* Per-cycle update; call from the control-loop ISR while mode == BALANCE.
 * est_tick = true on the 1 kHz estimator ticks. Returns the torque to send
 * (clamped). On an abort it has already idled the axis and set
 * controller_mode = IDLE — the same tick's dispatch must re-read the mode
 * or simply send the returned 0 Nm; both are safe. */
float BalanceCtrl_Tick(bool est_tick);

bool  BalanceCtrl_IsRunning(void);

/* Fader-during-balance input (C15): normalized centered deflection in
 * [-1, +1], 0 = centred/no effect. Written by pot_ctrl (main loop, 100 Hz,
 * single atomic float store); consumed on the 1 kHz balance tick ONLY when
 * BAL_FADER_STEER / BAL_FADER_POKE are compiled in — otherwise it lands in
 * a variable nothing reads. Reset to 0 by BalanceCtrl_Start(). */
void  BalanceCtrl_SetFaderInput(float norm);

/* Operator/mode-change stop (main-loop context). Does NOT touch the ODrive —
 * the caller owns the axis transition (e.g. pot_ctrl's exit_to_idle). */
void  BalanceCtrl_Stop(void);

/* Why the last run ended (telemetry/debug). 0 = none/still running. */
typedef enum {
    BAL_END_NONE = 0,
    BAL_END_THETA,      /* |theta| exceeded BAL_ABORT_THETA_RAD */
    BAL_END_XDOT,       /* cart velocity runaway */
    BAL_END_RAIL,       /* cart left the soft band */
    BAL_END_EXTERN,     /* mode switched away externally (operator stop) */
    /* Fault aborts (arch review 2026-07-11, batch B) — append-only: */
    BAL_END_ENC,        /* link-1 encoder stale — theta frozen at last-known */
    BAL_END_AXIS,       /* ODrive axis_error set or axis left CLOSED_LOOP */
    BAL_END_CART        /* encoder-estimate stream stale — x/x_dot frozen */
} bal_end_t;
bal_end_t BalanceCtrl_LastEnd(void);

#ifdef __cplusplus
}
#endif

#endif /* BALANCE_CTRL_H */
