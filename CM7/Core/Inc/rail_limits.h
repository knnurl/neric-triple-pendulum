/**
 ******************************************************************************
 * @file    rail_limits.h
 * @brief   Rail geometry, homing, and layered travel limits (M7).
 *
 *  The cart runs on a finite linear rail with a hard-stop switch at each end
 *  (NC to GND, pull-up, falling-edge EXTI — not wired yet, see
 *  RAIL_HOME_SWITCH_WIRED). The home-side switch doubles as the rail-zero
 *  reference. All positions in this module are in ODrive revolutions.
 *
 *  Layered limits, per rail end (center -> end):
 *
 *              pot / slider travel maps to [soft_min .. soft_max]
 *     home    |============= usable =============|            far
 *     switch  soft    swHard              swHard   soft      switch
 *     (0 ref) min     min                  max     max      (travel)
 *       |------|-------|~~~~~~~~~~~~~~~~~~~~|-------|----------|
 *      phys   clamp    power cut           power   clamp      phys
 *      Estop  +taper   + auto re-arm       cut+    +taper     Estop
 *                        inward            rearm
 *
 *  - SOFT: setpoints are clamped to [soft_min, soft_max] and the commanded
 *    position is slew-limited, with the slew rate tapering down inside
 *    RAIL_TAPER_ZONE_REV of a soft limit (eases in instead of slamming).
 *  - SW HARD: if the measured position crosses [hard_min, hard_max] in an
 *    active mode, power is cut (axis IDLE), M7_FAULT_LIMIT_HIT set, and
 *    after a dwell the axis re-arms holding just inside the limit.
 *  - PHYS: EXTI on the end switches cuts power via CAN Estop independently
 *    of the encoder. Auto re-arms only if the switch releases; a switch
 *    still pressed after the dwell (cart parked on the wall, broken wire)
 *    latches until Clear Errors. Hardened 2026-07-11 (arch review F7):
 *    a physical trip while UN-homed latches (no rail frame to re-arm
 *    into); a physical edge during the CLEAR/REARM recovery re-trips
 *    (Estop + fresh dwell) instead of being swallowed; and both switch
 *    LEVELS are re-sampled before returning to NORMAL — EXTI is
 *    edge-triggered and would never re-fire for a held switch.
 *
 *  Frames: rail_pos = motor_pos - home_ref. Home (0) is where the home
 *  switch triggers; far end is RAIL_TRAVEL_REV (measured constant).
 *
 *  Calibration procedure (RAIL_HOME_SWITCH_WIRED == 0, manual zero):
 *   1. Jog the cart to the home end, click Home in the GUI (zeroes here).
 *   2. Jog to the far end, read "ODrive pos (rev)" -> RAIL_TRAVEL_REV,
 *      rebuild. Repeat only if the mechanics change.
 *  (The ODrive S1 onboard absolute encoder needs no index search — motor and
 *   encoder-offset calibration are saved on the ODrive and applied at boot.)
 *  Once the switches are wired, set RAIL_HOME_SWITCH_WIRED to 1 and Home
 *  creeps onto the switch instead.
 ******************************************************************************
 */

#ifndef RAIL_LIMITS_H
#define RAIL_LIMITS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Build-time configuration ------------------------------------------ */

/* 0 = switches not wired: Home zeroes at the current cart position and the
 * EXTI/creep machinery is compiled out. 1 = switches wired on the pins
 * below: Home creeps onto the home switch; both switches trip via EXTI. */
#define RAIL_HOME_SWITCH_WIRED   1

/* Rail geometry (revolutions, rail frame: 0 = home switch trigger point). */
#define RAIL_TRAVEL_REV          12.0f   /* home -> far end; MEASURE ON BENCH */
#define RAIL_SOFT_MARGIN_REV     0.5f    /* soft limit inset from each end */
#define RAIL_HARD_MARGIN_REV     0.15f   /* sw hard limit inset from each end */
#define RAIL_BACKOFF_REV         0.30f   /* re-arm hold point inside a hard limit */

/* Soft-approach taper: slew rate scales down linearly across this zone
 * before a soft limit, bottoming out at RAIL_TAPER_MIN of full rate. */
#define RAIL_SLEW_REV_S          10.0f   /* full-speed position slew */
#define RAIL_TAPER_ZONE_REV      1.0f
#define RAIL_TAPER_MIN           0.15f

/* Hard-limit recovery timing. */
#define RAIL_DWELL_MS            300u    /* power-off dwell after a trip */
#define RAIL_CLEAR_GAP_MS        50u     /* Clear_Errors -> re-arm gap (phys trip) */
#define RAIL_REARM_HOLD_MS       300u    /* hold at backoff before resuming */

/* Homing (used when RAIL_HOME_SWITCH_WIRED == 1). Creep is negative motor
 * direction by default — flip the sign if home is the other way. */
#define RAIL_HOME_CREEP_REV_S    (-0.5f)
#define RAIL_HOME_TIMEOUT_MS     30000u

/* End-switch pins (set as pull-up inputs in the .ioc; RailLimits_Init re-inits
 * them as EXTI). Home = PF14 (EXTI14), Far = PE11 (EXTI11) — both lines fall in
 * the 10..15 group, so they share the single EXTI15_10_IRQn vector/handler.
 * (Home was PE9/EXTI9_5; moved to PF14 after PE9 wouldn't read on the bench.) */
#define RAIL_SW_HOME_PORT        GPIOF
#define RAIL_SW_HOME_PIN         GPIO_PIN_14
#define RAIL_SW_FAR_PORT         GPIOE
#define RAIL_SW_FAR_PIN          GPIO_PIN_11

/* ---- Rail state (mirrored into g_shared.rail_state for telemetry) ------ */
#define RAIL_STATE_UNHOMED       0u
#define RAIL_STATE_OK            1u
#define RAIL_STATE_SOFT_CLAMP    2u   /* setpoint being clamped/tapered */
#define RAIL_STATE_HARD_TRIP     3u   /* power cut / re-arm in progress */
#define RAIL_STATE_LATCHED       4u   /* phys switch trip, awaiting Clear Errors */

/* ---- API ---------------------------------------------------------------- */

/* Configure end-switch GPIO/EXTI (no-op when not wired). Call from USER
 * CODE 2 after Odrive_Init. */
void RailLimits_Init(void);

/* Home the rail. Blocking (call from main-loop context like index search),
 * abortable by a remote IDLE/SAFE_STOP command. Wired: creeps onto the home
 * switch, records zero, backs off RAIL_BACKOFF_REV. Unwired: zeroes at the
 * current position (operator jogs the cart to the home end first).
 * Returns true when homed. */
bool RailLimits_StartHoming(uint32_t timeout_ms);

bool RailLimits_IsHomed(void);

/* Frame conversions (only meaningful once homed). */
float RailLimits_MotorToRail(float motor_pos);
float RailLimits_RailToMotor(float rail_pos);

/* Soft-limit band the pot / slider maps onto. */
float RailLimits_SoftMin(void);
float RailLimits_SoftMax(void);

/* Clamp `rail_desired` to the soft band and advance *rail_cmd_io toward it
 * under the tapered slew limit (call at the pot_ctrl 100 Hz tick; dt_s is
 * the tick period). Returns the MOTOR-frame target to send to the ODrive. */
float RailLimits_ClampTarget(float rail_desired, float *rail_cmd_io, float dt_s);

/* Hard-limit supervisor. Call every control-loop cycle from the TIM7 ISR,
 * BEFORE the CAN dispatch (re-arm rewrites motor_command/controller_mode
 * so the same tick sends the corrected command). */
void RailLimits_Tick(float motor_pos, uint32_t now_ms);

/* True while a hard-limit cut/recovery is in progress — pot_ctrl suspends
 * its own setpoint output and re-arms its match gate on the falling edge. */
bool RailLimits_Active(void);

/* Release a latched physical-switch trip (wire into the Clear Errors path). */
void RailLimits_ClearLatch(void);

#ifdef __cplusplus
}
#endif

#endif /* RAIL_LIMITS_H */
