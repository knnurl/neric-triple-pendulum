/**
 ******************************************************************************
 * @file    state_est.c
 * @brief   Stage-1 state estimator: FD + Butterworth LPF on theta_dot (M7).
 ******************************************************************************
 */

#include "state_est.h"
#include "main.h"
#include "shared_state.h"
#include "odrive.h"
#include "rail_limits.h"
#include <math.h>

#ifndef M_PI_F
#define M_PI_F      3.14159265358979323846f
#define M_TWOPI_F   6.28318530717958647692f
#endif

/* ---- Module state (written in ISR context unless noted) ----------------- */
static state_est_t s_est = {0};

static float s_theta_offset = 0.0f;   /* raw angle at upright (ZeroUpright) */
static bool  s_zeroed       = false;
static float s_theta_prev   = 0.0f;   /* previous zeroed angle, for FD */
static bool  s_have_prev    = false;

/* 2nd-order Butterworth LPF (Direct Form I) on the raw FD velocity. */
static float s_b0, s_b1, s_b2, s_a1, s_a2;   /* a0 normalised to 1 */
static float s_vx1, s_vx2, s_vy1, s_vy2;     /* filter delay line */

/* Wrap to (-pi, +pi]. */
static inline float wrap_pi(float a)
{
    while (a >  M_PI_F) a -= M_TWOPI_F;
    while (a <= -M_PI_F) a += M_TWOPI_F;
    return a;
}

void StateEst_Init(void)
{
    /* Bilinear-transform Butterworth biquad, fc = STATE_EST_LPF_HZ at
     * fs = STATE_EST_RATE_HZ (Q = 1/sqrt(2)). Computed at init so the
     * corner is a one-line #define change. */
    const float K  = tanf(M_PI_F * STATE_EST_LPF_HZ / STATE_EST_RATE_HZ);
    const float Q  = 0.70710678f;
    const float n  = 1.0f / (1.0f + K / Q + K * K);

    s_b0 = K * K * n;
    s_b1 = 2.0f * s_b0;
    s_b2 = s_b0;
    s_a1 = 2.0f * (K * K - 1.0f) * n;
    s_a2 = (1.0f - K / Q + K * K) * n;

    s_vx1 = s_vx2 = s_vy1 = s_vy2 = 0.0f;
}

void StateEst_Tick(void)
{
    const float dt = 1.0f / STATE_EST_RATE_HZ;

    /* ---- Cart states: ODrive encoder estimates ---------------------- */
    float motor_pos = Odrive_GetMotorPos();
    bool  homed     = RailLimits_IsHomed();
    s_est.x          = homed ? RailLimits_MotorToRail(motor_pos) : motor_pos;
    s_est.x_dot      = Odrive_GetMotorVel();   /* frame offset is constant */
    s_est.rail_frame = homed;

    /* Mirror for the 100 Hz telemetry packet (fields were previously dead). */
    g_shared.cart_position = s_est.x;
    g_shared.cart_velocity = s_est.x_dot;

    /* ---- Link-1 angle ------------------------------------------------ */
    float raw   = g_shared.link_angle_rad[0];
    float theta = wrap_pi(STATE_EST_THETA_SIGN * (raw - s_theta_offset));
    s_est.theta  = theta;
    s_est.zeroed = s_zeroed;

    /* ---- theta_dot: wrap-aware FD + Butterworth LPF ------------------- */
    if (s_have_prev) {
        float v_raw = wrap_pi(theta - s_theta_prev) / dt;

        float v_f = s_b0 * v_raw + s_b1 * s_vx1 + s_b2 * s_vx2
                  - s_a1 * s_vy1 - s_a2 * s_vy2;
        s_vx2 = s_vx1;  s_vx1 = v_raw;
        s_vy2 = s_vy1;  s_vy1 = v_f;

        s_est.theta_dot = v_f;
    } else {
        s_have_prev     = true;
        s_est.theta_dot = 0.0f;
    }
    s_theta_prev = theta;
}

void StateEst_ZeroUpright(void)
{
    /* Main-loop context: swap the offset and restart the velocity filter
     * atomically w.r.t. the control ISR (a step in theta must not shock
     * theta_dot through the FD). */
    float raw = g_shared.link_angle_rad[0];
    __disable_irq();
    s_theta_offset = raw;
    s_zeroed       = true;
    s_have_prev    = false;
    s_vx1 = s_vx2 = s_vy1 = s_vy2 = 0.0f;
    __enable_irq();
}

bool StateEst_IsZeroed(void) { return s_zeroed; }

void StateEst_Get(state_est_t *out)
{
    __disable_irq();
    *out = s_est;
    __enable_irq();
}
