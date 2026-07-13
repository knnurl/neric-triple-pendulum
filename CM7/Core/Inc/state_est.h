/**
 ******************************************************************************
 * @file    state_est.h
 * @brief   Stage-1 state estimator: [x, x_dot, theta, theta_dot] (M7).
 *
 *  Produces the 4-state estimate the balance LQR and system-ID logger
 *  consume:
 *
 *    x         cart position (motor rev; RAIL frame once homed, raw motor
 *              frame before that — flagged in the estimate struct)
 *    x_dot     cart velocity (rev/s), straight from the ODrive's encoder
 *              estimate broadcasts (already filtered on the ODrive)
 *    theta     link-1 angle (rad), 0 = upright, wrapped to (-pi, +pi]
 *    theta_dot link-1 angular velocity (rad/s): wrap-aware finite difference
 *              at the 1 kHz encoder tick, smoothed by a 2nd-order Butterworth
 *              low-pass (STATE_EST_LPF_HZ)
 *
 *  Design note (Eltohamy & Kuo 1997): with near noise-free digital encoders,
 *  direct differentiation + light filtering beat observers on their triple
 *  rig. The AS5047P (14-bit @ 1 kHz) is in that regime, so stage 1 uses
 *  FD + LPF; the interface hides the internals so a Kalman filter can slot
 *  in for stages 2/3 without touching callers.
 *
 *  Upright zero: theta is measured relative to an offset captured while the
 *  operator holds link 1 vertical (GUI "Zero θ" -> cmd_zero_upright). RAM
 *  only — re-zero after every boot. Estimates are not "valid" until zeroed.
 *
 *  Contexts: StateEst_Tick runs in the TIM7 control-loop ISR (1 kHz decimated
 *  tick). StateEst_ZeroUpright runs in main-loop context (pot_ctrl) and
 *  briefly masks IRQs to swap the offset + reset the velocity filter.
 ******************************************************************************
 */

#ifndef STATE_EST_H
#define STATE_EST_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Configuration ------------------------------------------------------ */

/* theta_dot low-pass corner. 40 Hz keeps ~8x margin below the 1 kHz sample
 * rate while passing the pendulum dynamics (plant time constants are tens
 * of ms — Eltohamy's triple: 68 ms). */
#define STATE_EST_LPF_HZ        40.0f

/* Estimator tick rate = control loop / encoder decimation (control_loop.c). */
#define STATE_EST_RATE_HZ       1000.0f

/* Angle sign: +1 if a positive AS5047P angle change corresponds to the link
 * tipping toward the rail's FAR end (+x). CHECK ON BENCH; flip to -1.0f if
 * the plot in the GUI moves the wrong way. */
#define STATE_EST_THETA_SIGN    (+1.0f)

/* ---- Estimate ------------------------------------------------------------ */
typedef struct {
    float x;            /* cart position, rev (rail frame if rail_frame) */
    float x_dot;        /* cart velocity, rev/s */
    float theta;        /* link-1 angle, rad, 0 = upright, (-pi, +pi] */
    float theta_dot;    /* link-1 angular velocity, rad/s (filtered) */
    bool  zeroed;       /* upright offset has been captured this boot */
    bool  rail_frame;   /* x/x_dot are in the homed rail frame */
} state_est_t;

/* ---- API ----------------------------------------------------------------- */

/* Compute filter coefficients. Call once from USER CODE 2. */
void StateEst_Init(void);

/* Advance the estimate. Call from the control-loop ISR on each decimated
 * (1 kHz) encoder tick, AFTER the AS5047P read. Uses g_shared.link_angle_rad[0]
 * and the ODrive pos/vel shadows; also mirrors x/x_dot into
 * g_shared.cart_position / cart_velocity for telemetry. */
void StateEst_Tick(void);

/* Capture the upright zero at the current link-1 angle (operator holds the
 * link vertical). Main-loop context. */
void StateEst_ZeroUpright(void);

bool StateEst_IsZeroed(void);

/* Snapshot the latest estimate (IRQ-safe: masks the control ISR briefly). */
void StateEst_Get(state_est_t *out);

#ifdef __cplusplus
}
#endif

#endif /* STATE_EST_H */
