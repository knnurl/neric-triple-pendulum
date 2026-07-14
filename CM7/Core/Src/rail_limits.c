/**
 ******************************************************************************
 * @file    rail_limits.c
 * @brief   Rail homing + layered travel limits (M7). See rail_limits.h.
 *
 *  Switch wiring convention (when RAIL_HOME_SWITCH_WIRED == 1):
 *      NC switch between pin and GND, internal pull-up.
 *      Healthy (not pressed): switch closed -> pin LOW.
 *      Pressed OR wire broken: circuit open -> pull-up -> pin HIGH.
 *      EXTI trips on the RISING edge — fail-safe on a broken wire.
 *
 *  Concurrency:
 *      - RailLimits_Tick and the EXTI handlers both run at NVIC preempt
 *        priority 0 (same as TIM7), so they never preempt each other —
 *        the supervisor state machine needs no locking.
 *      - RailLimits_StartHoming / ClampTarget run in main-loop context;
 *        s_homing gates the ISR-side supervisor off while homing.
 ******************************************************************************
 */

#include "rail_limits.h"
#include "main.h"
#include "shared_state.h"
#include "odrive.h"
#include <math.h>

/* ---- Derived geometry (rail frame) -------------------------------------- */
#define SOFT_MIN   (RAIL_SOFT_MARGIN_REV)
#define SOFT_MAX   (RAIL_TRAVEL_REV - RAIL_SOFT_MARGIN_REV)
#define HARD_MIN   (RAIL_HARD_MARGIN_REV)
#define HARD_MAX   (RAIL_TRAVEL_REV - RAIL_HARD_MARGIN_REV)

/* ---- Supervisor state machine ------------------------------------------- */
typedef enum {
    RL_NORMAL = 0,
    RL_DWELL,       /* power cut; waiting RAIL_DWELL_MS */
    RL_CLEAR,       /* phys trip: Clear_Errors sent, waiting RAIL_CLEAR_GAP_MS */
    RL_REARM,       /* closed-loop re-armed at backoff; waiting RAIL_REARM_HOLD_MS */
    RL_LATCHED      /* phys switch still pressed after dwell; needs Clear Errors */
} rl_state_t;

static volatile rl_state_t s_state       = RL_NORMAL;
static volatile uint32_t   s_state_t0    = 0u;      /* ms timestamp of entry */
static volatile bool       s_trip_phys   = false;   /* physical switch (vs sw pos) */
static volatile bool       s_trip_at_max = false;   /* which end tripped */

static volatile bool  s_homed    = false;
static volatile bool  s_homing   = false;
static volatile float s_home_ref = 0.0f;   /* motor pos at rail zero */

/* Set by ClampTarget (main loop) when the setpoint is being clamped; held
 * for a short window so the telemetry indicator doesn't depend on position
 * mode still calling ClampTarget. */
static volatile uint32_t s_soft_clamp_until_ms = 0u;
#define SOFT_CLAMP_HOLD_MS 100u

/* ========================================================================
 *  Helpers
 * ===================================================================== */

static inline bool sw_pressed(GPIO_TypeDef *port, uint16_t pin)
{
#if RAIL_HOME_SWITCH_WIRED
    return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET;
#else
    (void)port; (void)pin;
    return false;
#endif
}

/* Active modes = the control loop is streaming motor commands. Hard limits
 * only trip (and re-arm) in these; in IDLE/SAFE_STOP the motor is unpowered
 * and the operator may be hand-moving the cart. */
static inline bool mode_is_active(void)
{
    uint8_t m = g_shared.controller_mode;
    return (m == CTRL_MODE_POT_POSITION || m == CTRL_MODE_POT_VELOCITY ||
            m == CTRL_MODE_SWINGUP      || m == CTRL_MODE_BALANCE);
}

static void publish_rail_state(void)
{
    uint8_t rs;
    if (s_state == RL_LATCHED)      rs = RAIL_STATE_LATCHED;
    else if (s_state != RL_NORMAL)  rs = RAIL_STATE_HARD_TRIP;
    else if (!s_homed)              rs = RAIL_STATE_UNHOMED;
    else if ((int32_t)(HAL_GetTick() - s_soft_clamp_until_ms) < 0)
                                    rs = RAIL_STATE_SOFT_CLAMP;
    else                            rs = RAIL_STATE_OK;
    g_shared.rail_state = rs;
}

#if RAIL_HOME_SWITCH_WIRED
/* Remote IDLE/SAFE_STOP abort check for the blocking homing loop (same
 * pattern as pot_ctrl's index-search abort). Only used by the switch-homing
 * routine; the manual-zero fallback (RAIL_HOME_SWITCH_WIRED == 0) doesn't
 * creep, so these two helpers are scoped to the wired build to avoid
 * -Wunused-function. */
static bool remote_stop_requested(uint32_t seq_at_entry)
{
    if (g_shared.command_seq == seq_at_entry) return false;
    /* command_mode is a sticky payload byte (F10): without the one-shot
     * cmd_set_mode flag, any unrelated command arriving mid-homing while
     * the byte still held a stale IDLE would abort the homing run. Do NOT
     * clear the flag here — pot_ctrl consumes it after homing returns and
     * performs the actual exit_to_idle. */
    if (!g_shared.cmd_set_mode) return false;
    uint8_t m = g_shared.command_mode;
    return (m == CTRL_MODE_SAFE_STOP || m == CTRL_MODE_IDLE);
}

/* Atomically update the shared command pair (mirrors pot_ctrl's
 * write_cmd_pair). Homing drives the motor THROUGH the control loop rather
 * than issuing Odrive_SetInput* directly, so the control loop keeps feeding
 * the ODrive watchdog — a direct SetInputVel from here would be zeroed every
 * 2 ms by the IDLE-mode watchdog feed in control_loop.c. */
static void set_cmd(uint8_t mode, float cmd)
{
    __disable_irq();
    g_shared.motor_command   = cmd;
    g_shared.controller_mode = mode;
    __enable_irq();
}
#endif /* RAIL_HOME_SWITCH_WIRED */

/* ========================================================================
 *  Init
 * ===================================================================== */

void RailLimits_Init(void)
{
#if RAIL_HOME_SWITCH_WIRED
    __HAL_RCC_GPIOD_CLK_ENABLE();   /* PD15 home */
    __HAL_RCC_GPIOE_CLK_ENABLE();   /* PE11 far, PE13 homing ref */

    /* Home (PD15) and far (PE11) are on different ports, so init each
     * separately. Both EXTI lines (15 and 11) fall in the 10..15 group and
     * share the single EXTI15_10 vector. */
    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_IT_RISING;   /* pressed/broken -> HIGH -> trip */
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin   = RAIL_SW_HOME_PIN;
    HAL_GPIO_Init(RAIL_SW_HOME_PORT, &g);
    g.Pin   = RAIL_SW_FAR_PIN;
    HAL_GPIO_Init(RAIL_SW_FAR_PORT, &g);

    /* Homing reference (PE13): plain polled input with pull-up, NOT an EXTI —
     * it is only read during the homing creep and never cuts power. (Its port,
     * GPIOE, is already clocked above for the far switch.) */
    g.Mode  = GPIO_MODE_INPUT;
    g.Pin   = RAIL_SW_HOME_REF_PIN;
    HAL_GPIO_Init(RAIL_SW_HOME_REF_PORT, &g);

    /* Configuring the pins as EXTI latches a spurious pending bit from the
     * config transition (and the pull-up settling). Clear the EXTI line
     * pending flags and the NVIC pending IRQ BEFORE enabling, or the ISR
     * fires once at boot and hard-trips regardless of the actual switch level
     * — the "always latches at boot" symptom. */
    __HAL_GPIO_EXTI_CLEAR_IT(RAIL_SW_HOME_PIN);
    __HAL_GPIO_EXTI_CLEAR_IT(RAIL_SW_FAR_PIN);
    HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);

    /* Same preempt priority as TIM7 (0): can't preempt an in-flight tick,
     * runs immediately after — <=200 us latency, backed by the sw limit.
     * PD15 (line 15) and PE11 (line 11) both live on the EXTI15_10 vector. */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* Boot sanity: a switch already pressed (cart parked on a wall) or a
     * broken wire latches immediately — position control refuses to arm
     * until it's resolved and Clear Errors is issued. */
    if (sw_pressed(RAIL_SW_HOME_PORT, RAIL_SW_HOME_PIN) ||
        sw_pressed(RAIL_SW_FAR_PORT,  RAIL_SW_FAR_PIN)) {
        g_shared.m7_fault_flags |= M7_FAULT_LIMIT_HIT;
        s_state = RL_LATCHED;
    }
#endif
    publish_rail_state();
}

/* ========================================================================
 *  Homing
 * ===================================================================== */

bool RailLimits_IsHomed(void) { return s_homed; }

bool RailLimits_StartHoming(uint32_t timeout_ms)
{
    if (s_state != RL_NORMAL) return false;   /* resolve trips first */

#if RAIL_HOME_SWITCH_WIRED
    uint32_t seq0 = g_shared.command_seq;
    s_homing = true;    /* gate the SOFTWARE travel-limit supervisor off during
                         * the creep; the physical end switches stay armed. */

    /* Creep toward the inboard homing switch, streamed via the control loop
     * (see set_cmd — keeps the ODrive watchdog fed). */
    Odrive_SetControllerMode(ODRIVE_CTRL_MODE_VELOCITY, ODRIVE_INPUT_MODE_PASSTHROUGH);
    Odrive_SetAxisState(ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL);
    set_cmd(CTRL_MODE_POT_VELOCITY, RAIL_HOME_CREEP_REV_S);

    bool hit = false;
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < timeout_ms) {
        if (remote_stop_requested(seq0)) break;
        if (sw_pressed(RAIL_SW_HOME_REF_PORT, RAIL_SW_HOME_REF_PIN)) { hit = true; break; }
        /* An end switch tripping (home-end overshoot, or far-end from a wrong
         * creep direction) cuts power via EXTI -> RL_DWELL; s_state leaves
         * RL_NORMAL and the creep aborts here. */
        if (s_state != RL_NORMAL) break;
    }

    if (!hit) {
        set_cmd(CTRL_MODE_IDLE, 0.0f);
        Odrive_SetAxisState(ODRIVE_AXIS_STATE_IDLE);
        s_homing = false;
        publish_rail_state();
        return false;
    }

    s_home_ref = Odrive_GetMotorPos();
    s_homed    = true;

    /* Back off the switch (streamed as a position hold), then idle. */
    Odrive_SetControllerMode(ODRIVE_CTRL_MODE_POSITION, ODRIVE_INPUT_MODE_PASSTHROUGH);
    set_cmd(CTRL_MODE_POT_POSITION, RailLimits_RailToMotor(RAIL_BACKOFF_REV));
    HAL_Delay(700);
    set_cmd(CTRL_MODE_IDLE, 0.0f);
    Odrive_SetAxisState(ODRIVE_AXIS_STATE_IDLE);

    s_homing = false;
#else
    /* Manual zero: operator has jogged the cart to the home end. */
    (void)timeout_ms;
    s_home_ref = Odrive_GetMotorPos();
    s_homed    = true;
#endif

    publish_rail_state();
    return s_homed;
}

/* ========================================================================
 *  Frames / soft clamp
 * ===================================================================== */

float RailLimits_MotorToRail(float motor_pos) { return motor_pos - s_home_ref; }
float RailLimits_RailToMotor(float rail_pos)  { return rail_pos + s_home_ref; }

float RailLimits_SoftMin(void) { return SOFT_MIN; }
float RailLimits_SoftMax(void) { return SOFT_MAX; }

float RailLimits_ClampTarget(float rail_desired, float *rail_cmd_io, float dt_s)
{
    /* 1. Clamp the request to the soft band. */
    float d = rail_desired;
    if (d < SOFT_MIN) d = SOFT_MIN;
    if (d > SOFT_MAX) d = SOFT_MAX;
    if (d != rail_desired) {
        s_soft_clamp_until_ms = HAL_GetTick() + SOFT_CLAMP_HOLD_MS;
    }

    /* 2. Tapered slew toward it: full rate mid-rail, scaling down linearly
     *    across RAIL_TAPER_ZONE_REV when moving toward a soft limit. */
    float cur  = *rail_cmd_io;
    float step = RAIL_SLEW_REV_S * dt_s;

    float dist_to_limit = (d > cur) ? (SOFT_MAX - cur) : (cur - SOFT_MIN);
    float taper = dist_to_limit / RAIL_TAPER_ZONE_REV;
    if (taper < RAIL_TAPER_MIN) taper = RAIL_TAPER_MIN;
    if (taper > 1.0f)           taper = 1.0f;
    step *= taper;

    float dv = d - cur;
    if      (dv >  step) cur += step;
    else if (dv < -step) cur -= step;
    else                 cur  = d;

    *rail_cmd_io = cur;
    return RailLimits_RailToMotor(cur);
}

/* ========================================================================
 *  Hard-limit supervisor (TIM7 ISR context)
 * ===================================================================== */

bool RailLimits_Active(void)
{
    return (s_state != RL_NORMAL);
}

void RailLimits_ClearLatch(void)
{
    if (s_state == RL_LATCHED) {
        s_state = RL_NORMAL;
        publish_rail_state();
    }
}

/* Common trip entry. ISR-safe (EXTI and TIM7 share preempt priority 0). */
static void hard_trip(bool phys, bool at_max)
{
    if (s_state != RL_NORMAL) {
        /* Recovery already in flight. A PHYSICAL edge during RL_CLEAR /
         * RL_REARM means the cart reached a wall while we were re-arming
         * (or still coasting) — cut power again and restart the dwell.
         * Swallowing it here left the re-arm drive live into a pressed
         * switch (arch review F7b). No mode_is_active() gate on the
         * re-trip: during recovery the axis is supervisor-owned, and a
         * redundant Estop to an idle axis is harmless. In RL_DWELL /
         * RL_LATCHED power is already cut — nothing to add. */
        if (phys && (s_state == RL_CLEAR || s_state == RL_REARM)) {
            (void)Odrive_Estop();
            g_shared.m7_fault_flags |= M7_FAULT_LIMIT_HIT;
            s_trip_phys   = true;
            s_trip_at_max = at_max;
            s_state_t0    = HAL_GetTick();
            s_state       = RL_DWELL;
            publish_rail_state();
        }
        return;
    }
    /* Both end switches stay armed even during homing: the creep now targets
     * the separate inboard homing switch (RAIL_SW_HOME_REF), so a home-end
     * trip here means the creep overshot — a real emergency to cut. (This is
     * the old s_homing home-suppression, deliberately removed with the
     * dedicated homing switch.) */
    if (!phys && !s_homed)          return;  /* rail frame meaningless unhomed */
    if (!mode_is_active())          return;  /* unpowered; nothing to cut */

    if (phys) {
        (void)Odrive_Estop();                /* strongest cut for a real wall hit */
    } else {
        (void)Odrive_SetAxisState(ODRIVE_AXIS_STATE_IDLE);
    }
    g_shared.m7_fault_flags |= M7_FAULT_LIMIT_HIT;

    s_trip_phys   = phys;
    s_trip_at_max = at_max;
    s_state_t0    = HAL_GetTick();
    s_state       = RL_DWELL;
    publish_rail_state();
}

void RailLimits_Tick(float motor_pos, uint32_t now_ms)
{
    switch (s_state) {

    case RL_NORMAL: {
        if (s_homed && !s_homing && mode_is_active()) {
            float rail = motor_pos - s_home_ref;
            if      (rail <= HARD_MIN) hard_trip(false, false);
            else if (rail >= HARD_MAX) hard_trip(false, true);
        }
        publish_rail_state();   /* also refreshes soft-clamp/unhomed states */
        break;
    }

    case RL_DWELL: {
        if ((now_ms - s_state_t0) < RAIL_DWELL_MS) break;

        if (s_trip_phys) {
            /* Only recover if the switch actually released; otherwise the
             * cart is parked on the wall or a wire broke — stay latched. */
            bool still = s_trip_at_max
                       ? sw_pressed(RAIL_SW_FAR_PORT,  RAIL_SW_FAR_PIN)
                       : sw_pressed(RAIL_SW_HOME_PORT, RAIL_SW_HOME_PIN);
            if (still) {
                s_state = RL_LATCHED;
                publish_rail_state();
                break;
            }
            /* Un-homed physical trip: no rail frame exists, so the rearm
             * path's RailToMotor() would aim the recovery drive at a
             * garbage home reference (arch review F7a). Latch instead —
             * operator clears errors, then homes. */
            if (!s_homed) {
                s_state = RL_LATCHED;
                publish_rail_state();
                break;
            }
            /* Estop latched an error on the ODrive — clear it first. */
            (void)Odrive_ClearErrors();
            s_state_t0 = now_ms;
            s_state    = RL_CLEAR;
            break;
        }

        /* Software trip: re-arm straight away. */
        goto rearm;
    }

    case RL_CLEAR: {
        if ((now_ms - s_state_t0) < RAIL_CLEAR_GAP_MS) break;
        goto rearm;
    }

    case RL_REARM: {
        if ((now_ms - s_state_t0) < RAIL_REARM_HOLD_MS) break;
        /* EXTI is edge-triggered: a switch that is STILL pressed here would
         * never re-fire after we return to NORMAL — the supervisor would
         * resume with the cart parked on a wall, or a wire broken
         * mid-recovery, and no fault (arch review F7c). Level re-sample
         * both switches and latch rather than resume. (sw_pressed() folds
         * to false when RAIL_HOME_SWITCH_WIRED == 0.) */
        if (sw_pressed(RAIL_SW_HOME_PORT, RAIL_SW_HOME_PIN) ||
            sw_pressed(RAIL_SW_FAR_PORT,  RAIL_SW_FAR_PIN)) {
            (void)Odrive_SetAxisState(ODRIVE_AXIS_STATE_IDLE);
            s_state = RL_LATCHED;
            publish_rail_state();
            break;
        }
        /* Recovery complete ("momentary" cut). pot_ctrl sees Active() fall
         * and re-arms its match gate so control can't drive straight back
         * into the wall. */
        g_shared.m7_fault_flags &= ~(uint32_t)M7_FAULT_LIMIT_HIT;
        s_state = RL_NORMAL;
        publish_rail_state();
        break;
    }

    case RL_LATCHED:
    default:
        break;

    rearm: {
        /* Hold just inside the tripped limit. Rewrite the shared command
         * pair BEFORE re-arming so this same tick's CAN dispatch (which
         * runs after RailLimits_Tick) sends the corrected target, not the
         * stale one that caused the trip. */
        float backoff_rail = s_trip_at_max ? (HARD_MAX - RAIL_BACKOFF_REV)
                                           : (HARD_MIN + RAIL_BACKOFF_REV);
        g_shared.motor_command   = RailLimits_RailToMotor(backoff_rail);
        g_shared.controller_mode = CTRL_MODE_POT_POSITION;

        (void)Odrive_SetControllerMode(ODRIVE_CTRL_MODE_POSITION,
                                       ODRIVE_INPUT_MODE_PASSTHROUGH);
        (void)Odrive_SetAxisState(ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL);

        s_state_t0 = now_ms;
        s_state    = RL_REARM;
        publish_rail_state();
        break;
    }
    }
}

/* ========================================================================
 *  EXTI handlers (compiled only when the switches are wired)
 * ===================================================================== */

#if RAIL_HOME_SWITCH_WIRED
/* Both end switches share the EXTI15_10 vector now: home = PD15 (line 15),
 * far = PE11 (line 11). One handler services both, checking each pin's flag.
 *
 * Level-confirm before tripping: clear the flag, then only hard_trip if the
 * line is ACTUALLY high (pressed/open/broken). A real end-stop hit holds the
 * line high for as long as the cart is at the wall, so this never misses a
 * genuine trip — but it rejects the spurious edge captured when the pin is
 * first muxed to EXTI at init (the "latches at boot while grounded" bug),
 * without depending on the pending-bit clear landing on the right dual-core
 * register. */
void EXTI15_10_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(RAIL_SW_HOME_PIN) != 0u) {   /* PD15 = home */
        __HAL_GPIO_EXTI_CLEAR_IT(RAIL_SW_HOME_PIN);
        if (sw_pressed(RAIL_SW_HOME_PORT, RAIL_SW_HOME_PIN)) {
            hard_trip(true, false);
        }
    }
    if (__HAL_GPIO_EXTI_GET_IT(RAIL_SW_FAR_PIN) != 0u) {    /* PE11 = far */
        __HAL_GPIO_EXTI_CLEAR_IT(RAIL_SW_FAR_PIN);
        if (sw_pressed(RAIL_SW_FAR_PORT, RAIL_SW_FAR_PIN)) {
            hard_trip(true, true);
        }
    }
}
#endif
