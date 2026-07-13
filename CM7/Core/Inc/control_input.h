/**
 ******************************************************************************
 * @file    control_input.h
 * @brief   Compile-time selection of the control INPUT PATH for the external
 *          controllers (balance / swing-up / sysid). M7 only.
 *
 *  CTRL_INPUT_ACCEL = 0  (default) — TORQUE input:
 *      controller output u is a torque [Nm]; the dispatch streams
 *      Set_Input_Torque; the axis is armed in TORQUE/PASSTHROUGH.
 *      The plant model must identify cart mass + belt viscous + Coulomb
 *      friction (id_fit stages 2/3) — both papers' hardest fit.
 *
 *  CTRL_INPUT_ACCEL = 1  — ACCELERATION input (Kaheman et al. 2022 style):
 *      controller output u is a cart acceleration [rev/s^2]; the dispatch
 *      integrates u into a velocity command at 5 kHz and streams
 *      Set_Input_Vel; the axis is armed in VELOCITY/PASSTHROUGH. The
 *      ODrive's velocity loop rejects cart mass and friction as
 *      disturbances, so the CART-SIDE PARAMETERS DROP OUT OF THE MODEL —
 *      pendulum parameters (ml, Jt, bp) from free-swing fits alone
 *      suffice. Trade: the inner loop's bandwidth and current limit
 *      replace the direct force authority; saturation becomes opaque
 *      (an unachievable accel just lags) — hence the conservative clamps.
 *
 *  Decision rule (STATUS_11_07 §8): flip to 1 only if the U6 held-out
 *  validation misses the 80 % gate twice with residuals implicating the
 *  friction terms — and ONLY with gains from the accel variant of
 *  lqr_design.m (K units differ: Nm-per-state vs rev/s^2-per-state; the
 *  flag comment in balance_ctrl.h repeats this). Behavioural difference
 *  worth knowing for drills: with K = 0, the torque path leaves the motor
 *  limp (0 Nm), while the accel path holds ZERO VELOCITY — an armed axis
 *  actively resists hand motion.
 *
 *  Wire/telemetry impact: none (layouts unchanged). g_shared.motor_command
 *  and the idlog 'u' column carry rev/s^2 instead of Nm when flipped —
 *  the MATLAB fit/design scripts must be told which units they are seeing.
 *
 *  Same idiom as ENC_READ_NONBLOCKING: #ifndef default so the syntax
 *  matrix compiles both paths with -DCTRL_INPUT_ACCEL=0/1.
 ******************************************************************************
 */

#ifndef CONTROL_INPUT_H
#define CONTROL_INPUT_H

#ifndef CTRL_INPUT_ACCEL
#define CTRL_INPUT_ACCEL   0
#endif

/* ODrive controller mode the external controllers arm with (pot_ctrl's
 * three arm sites). Token substitution — use sites include odrive.h. */
#if CTRL_INPUT_ACCEL
#define CTRL_ODRV_CTRL_MODE   ODRIVE_CTRL_MODE_VELOCITY
#else
#define CTRL_ODRV_CTRL_MODE   ODRIVE_CTRL_MODE_TORQUE
#endif

/* Accel path only: hard clamp on the integrated velocity command streamed
 * by the dispatch (rev/s). This is the integrator's own safety bound —
 * independent of it, the measured-|x_dot| abort envelopes (8 rev/s) and
 * the ODrive's vel_limit (U2) still stand behind it.
 * PLACEHOLDER — retune from the accel-mode design sweep before flipping. */
#define CTRL_VEL_CMD_MAX_REV_S   5.0f

#endif /* CONTROL_INPUT_H */
