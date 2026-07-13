/**
 ******************************************************************************
 * @file    pot_ctrl.c
 * @brief   Button + potentiometer manual control on M7 (Step "button/pot").
 *          Ported from ODrive_Pot_SW reference; simplified to single input.
 ******************************************************************************
 */

#include "pot_ctrl.h"
#include "main.h"
#include "shared_state.h"
#include "odrive.h"
#include "control_input.h"   /* CTRL_ODRV_CTRL_MODE: torque vs velocity arming */
#include "rail_limits.h"
#include "state_est.h"
#include "balance_ctrl.h"
#include "swingup_ctrl.h"
#include "sysid.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

extern ADC_HandleTypeDef hadc1;

/* ========================================================================
 *  Tunables
 * ===================================================================== */

/* Button timing (ms) */
#define BTN_DEBOUNCE_MS       20u
#define BTN_DOUBLE_CLICK_MS   400u
#define BTN_SHORT_HOLD_MS     1200u
#define BTN_LONG_HOLD_MS      4000u

/* Position control. Targets are ABSOLUTE rail positions: the pot's travel
 * maps onto [RailLimits_SoftMin() .. RailLimits_SoftMax()], so the pot end
 * stops correspond to the rail end stops (minus the soft margin). Requires
 * a Home first (rail_limits.h). */
#define POS_HYSTERESIS_REV    0.018f     /* output deadband for ADC noise */
#define POS_ARM_BAND_REV      0.25f      /* match gate: source must come to the cart */

/* Velocity control */
#define VEL_MAX               10.0f      /* rev/s */
#define VEL_RAMP_RATE         20.0f      /* rev/s² */
#define VEL_CENTRE_DB_PCT     4U         /* ±5% around midpoint */
#define LOOP_DT_S             0.010f     /* 10 ms tick period */
#define VEL_TO_POS_BRAKE_MS   500u

/* Remote (MATLAB/ESP32) velocity zero-crossing safety gate: a setpoint left
 * non-zero from a previous session must pass back through ~0 before
 * VELOCITY_CONTROL starts applying it. (Position uses the position-match
 * gate POS_ARM_BAND_REV instead — the source must come to the cart.) */
#define REMOTE_VEL_ZERO_EPS   (VEL_MAX * (float)VEL_CENTRE_DB_PCT / 100.0f)  /* rev/s */

/* ADC */
#define ADC_FULL_SCALE        65535U
#define ADC_DEADBAND_PCT      2U
#define ADC_DEADBAND_LO       ((ADC_FULL_SCALE * ADC_DEADBAND_PCT) / 100U)
#define ADC_DEADBAND_HI       (ADC_FULL_SCALE - ADC_DEADBAND_LO)
#define ADC_FILTER_LEN        32U

/* Derived */
#define ADC_CENTRE            (ADC_FULL_SCALE / 2U)
#define VEL_CENTRE_DB         ((ADC_FULL_SCALE * VEL_CENTRE_DB_PCT) / 100U)
#define VEL_CENTRE_DB_LO      (ADC_CENTRE - VEL_CENTRE_DB)
#define VEL_CENTRE_DB_HI      (ADC_CENTRE + VEL_CENTRE_DB)

/* LED ports/pins (Nucleo-H745ZI-Q) */
#define LED_GREEN_PORT        GPIOB
#define LED_GREEN_PIN         GPIO_PIN_0
#define LED_YELLOW_PORT       GPIOE
#define LED_YELLOW_PIN        GPIO_PIN_1
#define LED_RED_PORT          GPIOB
#define LED_RED_PIN           GPIO_PIN_14

/* Button (Nucleo blue button — active HIGH when pressed) */
#define BTN_PORT              GPIOC
#define BTN_PIN               GPIO_PIN_13

/* ========================================================================
 *  Internal types & state
 * ===================================================================== */

typedef enum {
    BTN_FSM_IDLE,
    BTN_FSM_DEBOUNCE,
    BTN_FSM_PRESSED,
    BTN_FSM_RELEASED_WAIT,
    BTN_FSM_SECOND_PRESS,
    BTN_FSM_RELEASED_WAIT_2,
    BTN_FSM_THIRD_PRESS,
    BTN_FSM_AFTER_SHORT_HOLD,
    BTN_FSM_AFTER_LONG_HOLD
} btn_fsm_t;

typedef enum {
    BTN_EVENT_NONE,
    BTN_EVENT_SINGLE_CLICK,
    BTN_EVENT_DOUBLE_CLICK,
    BTN_EVENT_TRIPLE_CLICK,
    BTN_EVENT_SHORT_HOLD,
    BTN_EVENT_LONG_HOLD
} btn_event_t;

static pot_sys_state_t s_state          = POT_STATE_IDLE;

/* ADC rolling-average filter */
static uint16_t s_filter_ring[ADC_FILTER_LEN] = {0};
static uint32_t s_filter_sum   = 0;
static uint8_t  s_filter_idx   = 0;
static bool     s_filter_primed = false;

/* Button FSM */
static btn_fsm_t s_btn_fsm = BTN_FSM_IDLE;
static uint32_t  s_btn_ts  = 0;

/* Position control state (rail frame unless noted) */
static bool  s_pos_centre_ok  = false;   /* position-match gate armed */
static float s_last_sent_pos  = 0.0f;    /* motor frame, CAN dedup */
static float s_rail_desired   = 0.0f;    /* latest request from active source */
static float s_rail_cmd       = 0.0f;    /* slewed command (RailLimits_ClampTarget io) */
/* Velocity control state */
static bool  s_vel_centre_ok  = false;
static float s_vel_current    = 0.0f;

/* Control-source (g_shared.ctrl_src, M4-owned, set from the MATLAB GUI):
 * previous reading, for edge detection on REMOTE->POT handover (forces the
 * pot to re-centre so control doesn't jump). */
static bool  s_remote_active_prev = false;

/* RailLimits hard-limit recovery: previous Active() reading, for the
 * falling-edge resync after an auto re-arm. */
static bool  s_rl_active_prev = false;

/* ========================================================================
 *  Forward declarations
 * ===================================================================== */
static btn_event_t button_fsm_tick(bool raw_pressed);
static uint16_t    filter_push(uint16_t sample);
static float       map_adc_position(uint16_t adc);
static float       map_adc_velocity(uint16_t adc);
static void        update_leds(pot_sys_state_t state, bool waiting_centre);
static void        write_cmd_pair(uint8_t mode, float cmd);
static void        arm_closed_loop(void);
static void        enter_position_control(bool already_closed_loop);
static void        enter_velocity_control(void);
static void        exit_to_idle(void);

/* ========================================================================
 *  Public API
 * ===================================================================== */

void PotCtrl_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* LEDs: push-pull outputs, low speed */
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin   = LED_GREEN_PIN;   HAL_GPIO_Init(LED_GREEN_PORT,  &g);
    g.Pin   = LED_YELLOW_PIN;  HAL_GPIO_Init(LED_YELLOW_PORT, &g);
    g.Pin   = LED_RED_PIN;     HAL_GPIO_Init(LED_RED_PORT,    &g);

    HAL_GPIO_WritePin(LED_GREEN_PORT,  LED_GREEN_PIN,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_YELLOW_PORT, LED_YELLOW_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RED_PORT,    LED_RED_PIN,    GPIO_PIN_RESET);

    /* Button: input, no pull (board has external pull-down for Blue Button) */
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    g.Pin  = BTN_PIN;
    HAL_GPIO_Init(BTN_PORT, &g);

    /* Make sure mode/command in shared state start safe. */
    write_cmd_pair(CTRL_MODE_IDLE, 0.0f);

    update_leds(s_state, false);
}

pot_sys_state_t PotCtrl_GetState(void) { return s_state; }

void PotCtrl_Tick(void)
{
    /* ---- Control source: g_shared.ctrl_src, set from the MATLAB GUI's
     * POT/REMOTE switch (0 = POT, default/safe; 1 = REMOTE). Read first,
     * before the remote-command handler below, so a command from the
     * INACTIVE source can be gated out symmetrically with the physical
     * pot's per-tick block further down — otherwise the inactive side
     * could still slip a setpoint through and move the motor. ---------- */
    bool remote_active = (g_shared.ctrl_src != 0u);
    if (remote_active != s_remote_active_prev) {
        /* Control source just changed hands (either direction). Whichever
         * side now has authority is likely nowhere near the other side's
         * last value — re-arm the zero-crossing gate so it must pass back
         * through ~0 before it takes over, instead of slamming the target
         * to whatever it happens to already be sitting at. */
        s_pos_centre_ok = false;
        s_vel_centre_ok = false;
        update_leds(s_state, true);
    }
    s_remote_active_prev = remote_active;

    /* ---- Rail hard-limit recovery resync ------------------------------- *
     * While RailLimits handles a trip it owns the motor (power cut, then
     * re-arm holding just inside the limit, with controller_mode forced to
     * POT_POSITION). On the falling edge we adopt that state: match gates
     * re-armed so neither source can drive straight back into the wall,
     * and the slew command resynced to the backoff hold point.            */
    bool rl_active = RailLimits_Active();
    if (s_rl_active_prev && !rl_active) {
        BalanceCtrl_Stop();   /* a hard trip ends any balance/swing-up/ID run */
        SwingupCtrl_Stop();
        SysId_Stop();
        s_state         = POT_STATE_POSITION_CONTROL; /* holding at backoff */
        s_pos_centre_ok = false;
        s_vel_centre_ok = false;
        s_rail_cmd      = RailLimits_MotorToRail(g_shared.motor_command);
        s_rail_desired  = s_rail_cmd;
        s_last_sent_pos = g_shared.motor_command;
        update_leds(s_state, true);
    }
    s_rl_active_prev = rl_active;

    /* ---- Remote commands from MATLAB / ESP32 (via M4 shared state) ---- *
     * command_seq is bumped by M4 on every accepted command. We track the
     * last value we consumed; when it advances, a new command arrived.    */
    static uint32_t s_last_cmd_seq = 0u;
    if (g_shared.command_seq != s_last_cmd_seq) {
        s_last_cmd_seq = g_shared.command_seq;

        /* Clear-errors request: only M7 can actually clear anything here —
         * the ODrive's latched error state lives on the far side of FDCAN,
         * and m7_fault_flags is M7-owned. M4's own apply_clear_errors()
         * only zeroes its local m4_fault_flags; this is what makes the
         * button actually clear ODrive errors. Allowed in any state. */
        if (g_shared.cmd_clear_errors) {
            g_shared.cmd_clear_errors = 0u;
            (void)Odrive_ClearErrors();
            g_shared.m7_fault_flags = 0u;
            RailLimits_ClearLatch();   /* release a latched end-switch trip */
        }

        /* Home request: establish rail zero (rail_limits.h). Blocking like
         * index search, abortable by a remote IDLE/SAFE_STOP. Only from a
         * non-moving state. */
        if (g_shared.cmd_home) {
            g_shared.cmd_home = 0u;
            if (s_state == POT_STATE_IDLE && !RailLimits_Active()) {
                (void)RailLimits_StartHoming(RAIL_HOME_TIMEOUT_MS);
                update_leds(s_state, false);
            }
        }

        /* Zero-upright request: capture the pendulum vertical reference for
         * the state estimator (operator is holding link 1 upright). Any
         * non-moving state; instant. */
        if (g_shared.cmd_zero_upright) {
            g_shared.cmd_zero_upright = 0u;
            if (s_state == POT_STATE_IDLE || s_state == POT_STATE_EXTERNAL) {
                StateEst_ZeroUpright();
            }
        }

        /* System-ID run request: params already in g_shared.sysid_*.
         * SysId_Start re-clamps them and refuses driving types un-homed.
         * Driving types need the axis armed in torque mode FIRST — the
         * excitation clock only starts once controller_mode goes SYSID. */
        if (g_shared.cmd_sysid) {
            g_shared.cmd_sysid = 0u;
            if (s_state == POT_STATE_IDLE && !RailLimits_Active()
                && SysId_Start()) {
                if (SysId_IsDriving()) {
                    Odrive_SetControllerMode(CTRL_ODRV_CTRL_MODE,
                                             ODRIVE_INPUT_MODE_PASSTHROUGH);
                    arm_closed_loop();
                }
                write_cmd_pair(CTRL_MODE_SYSID, 0.0f);
                s_state = POT_STATE_EXTERNAL;
                update_leds(s_state, false);
            }
        }

        /* ---- Mode request: ONE-SHOT (arch review F10, fixed 07-11) ---- *
         * command_mode is a sticky payload byte; the request EVENT is
         * cmd_set_mode (M4 sets 1 with the payload, we clear on
         * consumption — same pattern as the other cmd_* flags). This
         * block previously re-ran on EVERY command_seq bump with whatever
         * command_mode still held, so any unrelated command (gain delta,
         * zero-upright, clear-errors) replayed the last mode request —
         * flipping POSITION back to VELOCITY, or killing a button-started
         * session with a stale IDLE. */
        if (g_shared.cmd_set_mode) {
            g_shared.cmd_set_mode = 0u;
            uint8_t req = g_shared.command_mode;

            if (req == CTRL_MODE_IDLE || req == CTRL_MODE_SAFE_STOP) {
                /* --- Remote IDLE / E-STOP: always honoured regardless of
                 * ctrl_src — this is a stop, not a competing setpoint. -- */
                exit_to_idle();
                s_state = POT_STATE_IDLE;
                update_leds(s_state, false);

            } else if (req == CTRL_MODE_POT_POSITION) {
                /* --- Mode selection works regardless of ctrl_src
                 * (orchestration, not "who drives the value"); the
                 * continuous setpoint is applied in the block below,
                 * slewed/clamped through RailLimits and gated on the
                 * position-match band. Needs a rail zero: refuse until
                 * homed (GUI shows rail_state UNHOMED). ----------------- */
                if (RailLimits_IsHomed() &&
                    (s_state == POT_STATE_IDLE ||
                     s_state == POT_STATE_VELOCITY_CONTROL)) {
                    /* IDLE arms closed loop; VELOCITY_CONTROL is already closed
                     * loop, so don't re-arm (mirrors the POS->VEL transition
                     * below). Without the VELOCITY_CONTROL case the mode switch
                     * was silently refused and the position setpoint leaked into
                     * the still-active velocity loop — pos slider drove velocity. */
                    enter_position_control(s_state == POT_STATE_VELOCITY_CONTROL);
                    s_state = POT_STATE_POSITION_CONTROL;
                    update_leds(s_state, true);
                }

            } else if (req == CTRL_MODE_SWINGUP) {
                /* --- Energy swing-up. Only from IDLE through its arm gate
                 * (model params set + zeroed + homed + cart centred/still —
                 * the pendulum itself may hang anywhere). Captures into
                 * BALANCE by itself. Refused requests are ignored, like
                 * BALANCE below. ----------------------------------------- */
                if (s_state == POT_STATE_IDLE && SwingupCtrl_ArmOK()) {
                    Odrive_SetControllerMode(CTRL_ODRV_CTRL_MODE,
                                             ODRIVE_INPUT_MODE_PASSTHROUGH);
                    arm_closed_loop();
                    SwingupCtrl_Start();
                    write_cmd_pair(CTRL_MODE_SWINGUP, 0.0f);
                    s_state = POT_STATE_EXTERNAL;
                    update_leds(s_state, false);
                }

            } else if (req == CTRL_MODE_BALANCE) {
                /* --- Balance (LQR torque loop). Only from IDLE, and only
                 * through the full arm gate: upright zeroed + homed + link
                 * near-vertical and still + cart mid-rail. A refused
                 * request is simply ignored — the GUI mode pill stays put,
                 * telling the operator the gate isn't satisfied. -------- */
                if (s_state == POT_STATE_IDLE && BalanceCtrl_ArmOK()) {
                    Odrive_SetControllerMode(CTRL_ODRV_CTRL_MODE,
                                             ODRIVE_INPUT_MODE_PASSTHROUGH);
                    arm_closed_loop();
                    BalanceCtrl_Start();
                    write_cmd_pair(CTRL_MODE_BALANCE, 0.0f);
                    s_state = POT_STATE_EXTERNAL;
                    update_leds(s_state, false);
                }

            } else if (req == CTRL_MODE_POT_VELOCITY) {
                /* --- Remote mode-select (see POT_POSITION note above) -- */
                if (s_state == POT_STATE_IDLE) {
                    /* Enter closed loop first, then switch to velocity mode. */
                    arm_closed_loop();
                }
                if (s_state == POT_STATE_IDLE ||
                    s_state == POT_STATE_POSITION_CONTROL) {
                    enter_velocity_control();   /* sets s_vel_centre_ok=false */
                    s_state = POT_STATE_VELOCITY_CONTROL;
                    update_leds(s_state, true);
                }
            }
        }

        /* ---- Continuous remote setpoint -------------------------------- *
         * Decoupled from the one-shot mode request: applies on ANY accepted
         * command while REMOTE owns the value, to whatever mode is active.
         * setpoint holds its last value between SET_SETPOINT commands, so
         * re-applying it is idempotent; handovers stay guarded by the
         * position-match / zero-crossing gates (re-armed on the ctrl_src
         * edge at the top of this tick). */
        if (remote_active) {
            float sp = g_shared.setpoint;
            /* Apply sp ONLY to the mode it was sent FOR. The GUI sends
             * SET_MODE(POS|VEL) immediately before each SET_SETPOINT, so
             * command_mode tags which slider produced sp. When a mode switch
             * is refused (e.g. VELOCITY->POSITION while unhomed) s_state and
             * command_mode disagree; without this guard a position setpoint
             * leaks into the still-active velocity loop and the pos slider
             * drives velocity. (Reading the sticky command_mode as a setpoint
             * TAG here is distinct from acting on it as a mode request — the
             * F10 one-shot rule; the mode switch above still gates on
             * cmd_set_mode.) */
            uint8_t sp_for = g_shared.command_mode;
            if (s_state == POT_STATE_POSITION_CONTROL &&
                sp_for == CTRL_MODE_POT_POSITION) {
                /* sp is an ABSOLUTE rail position (rev from home). */
                s_rail_desired = sp;
            } else if (s_state == POT_STATE_VELOCITY_CONTROL &&
                       sp_for == CTRL_MODE_POT_VELOCITY) {
                if (!s_vel_centre_ok && fabsf(sp) < REMOTE_VEL_ZERO_EPS) {
                    s_vel_centre_ok = true;
                }
                if (s_vel_centre_ok) {
                    float vel = sp;
                    if (vel >  VEL_MAX) vel =  VEL_MAX;
                    if (vel < -VEL_MAX) vel = -VEL_MAX;
                    s_vel_current = vel;
                    write_cmd_pair(CTRL_MODE_POT_VELOCITY, vel);
                }
                update_leds(s_state, !s_vel_centre_ok);
            }
        }
    }

    /* ---- Read pot --------------------------------------------------- */
    uint16_t filtered_adc = (uint16_t)ADC_CENTRE;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
        filtered_adc = filter_push((uint16_t)HAL_ADC_GetValue(&hadc1));
    }
    HAL_ADC_Stop(&hadc1);

    /* ---- Read button & advance FSM --------------------------------- */
    bool raw_pressed = (HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN) == GPIO_PIN_SET);
    btn_event_t evt  = button_fsm_tick(raw_pressed);

    /* ---- Handle button events -------------------------------------- */
    switch (evt) {
    case BTN_EVENT_SINGLE_CLICK:
        if (s_state == POT_STATE_IDLE && RailLimits_IsHomed()) {
            /* Position control needs a rail zero — Home first (velocity
             * mode is available un-homed for jogging to the home end). */
            enter_position_control(false);
            s_state = POT_STATE_POSITION_CONTROL;
            update_leds(s_state, true);
        }
        break;

    case BTN_EVENT_DOUBLE_CLICK:
        if (s_state == POT_STATE_POSITION_CONTROL) {
            enter_velocity_control();
            s_state = POT_STATE_VELOCITY_CONTROL;
            update_leds(s_state, true);
        }
        else if (s_state == POT_STATE_VELOCITY_CONTROL && RailLimits_IsHomed()) {
            enter_position_control(true);
            s_state = POT_STATE_POSITION_CONTROL;
            update_leds(s_state, true);
        }
        break;

    case BTN_EVENT_SHORT_HOLD:
        if (s_state == POT_STATE_POSITION_CONTROL ||
            s_state == POT_STATE_VELOCITY_CONTROL ||
            s_state == POT_STATE_EXTERNAL) {       /* physical balance/ID abort */
            exit_to_idle();
            s_state = POT_STATE_IDLE;
            update_leds(s_state, false);
        }
        break;

    case BTN_EVENT_LONG_HOLD:
        exit_to_idle();
        Odrive_ClearErrors();
        s_state = POT_STATE_IDLE;
        update_leds(s_state, false);
        break;

    default:
        break;
    }

    /* ---- Per-state runtime control --------------------------------- */
    switch (s_state) {

    case POT_STATE_POSITION_CONTROL: {
        /* RailLimits owns the motor during a hard-limit cut/recovery. */
        if (rl_active) break;

        /* Desired target comes from whichever source has authority. The pot
         * updates it every tick; the remote slider updated it in the command
         * handler above. Both are ABSOLUTE rail positions. */
        if (!remote_active) {
            s_rail_desired = map_adc_position(filtered_adc);
        }

        /* Position-match gate: after mode entry (or a source handover /
         * limit recovery) the active source must come to the cart before it
         * takes over — prevents a jump to wherever the pot knob / slider
         * happens to be sitting. Until then we hold the entry position. */
        float rail_now = RailLimits_MotorToRail(Odrive_GetMotorPos());
        if (!s_pos_centre_ok) {
            if (fabsf(s_rail_desired - rail_now) < POS_ARM_BAND_REV) {
                s_pos_centre_ok = true;
                s_rail_cmd      = rail_now;   /* slew starts from the cart */
                update_leds(s_state, false);
            }
            break;
        }

        /* Soft clamp + tapered slew toward the desired target. */
        float target = RailLimits_ClampTarget(s_rail_desired, &s_rail_cmd,
                                              LOOP_DT_S);

        float diff = target - s_last_sent_pos;
        if (diff < 0.0f) diff = -diff;

        if (diff >= POS_HYSTERESIS_REV) {
            write_cmd_pair(CTRL_MODE_POT_POSITION, target);
            s_last_sent_pos = target;
        }
        break;
    }

    case POT_STATE_VELOCITY_CONTROL: {
        if (rl_active) break;
        if (remote_active) break;

        if (!s_vel_centre_ok) {
            if (filtered_adc > VEL_CENTRE_DB_LO && filtered_adc < VEL_CENTRE_DB_HI) {
                s_vel_centre_ok = true;
                update_leds(s_state, false);
            }
            break;
        }
        float target = map_adc_velocity(filtered_adc);
        /* Software ramp */
        float dv_max = VEL_RAMP_RATE * LOOP_DT_S;
        float dv     = target - s_vel_current;
        if      (dv >  dv_max) s_vel_current += dv_max;
        else if (dv < -dv_max) s_vel_current -= dv_max;
        else                   s_vel_current  = target;

        write_cmd_pair(CTRL_MODE_POT_VELOCITY, s_vel_current);
        break;
    }

    case POT_STATE_EXTERNAL:
#if BAL_FADER_STEER || BAL_FADER_POKE
        /* ---- C15: fader input to a RUNNING balance (dormant unless the
         * flags in balance_ctrl.h are compiled in). Physical fader only
         * (ctrl_src == POT); zero-crossing gate re-arms on every balance
         * entry — the fader must pass through centre once before it goes
         * live, so grabbing it mid-run cannot slam an offset in. The
         * normalized mapping reuses map_adc_velocity's centre deadband. */
        {
            static bool s_fader_live     = false;
            static bool s_fader_bal_prev = false;
            bool bal_now = (g_shared.controller_mode == CTRL_MODE_BALANCE) &&
                           !remote_active && !rl_active;
            if (bal_now && !s_fader_bal_prev) {
                s_fader_live = false;            /* fresh run: re-centre first */
            }
            s_fader_bal_prev = bal_now;

            if (!bal_now) {
                s_fader_live = false;
                BalanceCtrl_SetFaderInput(0.0f);
            } else if (!s_fader_live) {
                if (filtered_adc > VEL_CENTRE_DB_LO &&
                    filtered_adc < VEL_CENTRE_DB_HI) {
                    s_fader_live = true;
                }
                BalanceCtrl_SetFaderInput(0.0f);
            } else {
                BalanceCtrl_SetFaderInput(map_adc_velocity(filtered_adc) / VEL_MAX);
            }
        }
#endif
        /* balance_ctrl / swingup_ctrl / sysid owns the motor. All hand
         * controller_mode back to IDLE when they finish or abort (swing-up
         * hands to BALANCE on capture — still "ours"; rail_limits rewrites
         * to POT_POSITION during a hard-limit recovery, caught by the
         * rl_active resync above) — fall back to IDLE when ours is gone. */
        if (g_shared.controller_mode != CTRL_MODE_BALANCE &&
            g_shared.controller_mode != CTRL_MODE_SWINGUP &&
            g_shared.controller_mode != CTRL_MODE_SYSID) {
            BalanceCtrl_Stop();
            SwingupCtrl_Stop();
            SysId_Stop();
            s_state = POT_STATE_IDLE;
            update_leds(s_state, false);
        }
        break;

    case POT_STATE_IDLE:
    default:
        /* No active command; mode already IDLE. */
        break;
    }

    /* NOTE: do NOT write g_shared.command_mode here — that field is M4-owned
     * (MATLAB/ESP32 command input). M7's active mode is already visible to
     * telemetry via the M7-owned controller_mode field. */
}

/* ========================================================================
 *  Helpers
 * ===================================================================== */

/* Atomically update controller_mode + motor_command so the TIM7 ISR never
 * sees a stale pair. Critical section is ~100 ns. */
static void write_cmd_pair(uint8_t mode, float cmd)
{
    __disable_irq();
    g_shared.motor_command   = cmd;
    g_shared.controller_mode = mode;
    __enable_irq();
}

/* Rolling-average ADC filter. */
static uint16_t filter_push(uint16_t sample)
{
    s_filter_sum -= s_filter_ring[s_filter_idx];
    s_filter_ring[s_filter_idx] = sample;
    s_filter_sum += sample;
    s_filter_idx = (s_filter_idx + 1U) % ADC_FILTER_LEN;
    if (!s_filter_primed) {
        if (s_filter_idx == 0U) s_filter_primed = true;
        else                    return sample;
    }
    return (uint16_t)(s_filter_sum / ADC_FILTER_LEN);
}

/* Map filtered ADC to an ABSOLUTE rail position: pot end stops correspond
 * to the rail's soft limits (pot-min = soft_min = home end). If the bench
 * shows the pot working backwards, invert norm here (norm = 1.0f - norm). */
static float map_adc_position(uint16_t adc)
{
    float norm;
    if (adc <= ADC_DEADBAND_LO)      norm = 0.0f;   /* pin edges to the ends */
    else if (adc >= ADC_DEADBAND_HI) norm = 1.0f;
    else                             norm = (float)adc / (float)ADC_FULL_SCALE;

    float lo = RailLimits_SoftMin();
    float hi = RailLimits_SoftMax();
    return lo + norm * (hi - lo);
}

/* Map filtered ADC to velocity in rev/s, centred at zero. */
static float map_adc_velocity(uint16_t adc)
{
    if (adc > VEL_CENTRE_DB_LO && adc < VEL_CENTRE_DB_HI) return 0.0f;
    float norm = ((float)adc - (float)ADC_CENTRE) / (float)ADC_CENTRE;
    float v = norm * VEL_MAX;
    if (v >  VEL_MAX) v =  VEL_MAX;
    if (v < -VEL_MAX) v = -VEL_MAX;
    return v;
}

/* LEDs:
 *   green  = active control mode
 *   yellow = position control
 *   red    = NEEDS_INDEX (steady) or waiting-centre (blink not implemented;
 *            here it's just an extra-bright indicator using both red/yellow). */
static void update_leds(pot_sys_state_t state, bool waiting_centre)
{
    GPIO_PinState g = GPIO_PIN_RESET, y = GPIO_PIN_RESET, r = GPIO_PIN_RESET;
    switch (state) {
    case POT_STATE_IDLE:                                   break;
    case POT_STATE_POSITION_CONTROL:
        g = GPIO_PIN_SET; y = GPIO_PIN_SET;
        if (waiting_centre) { r = GPIO_PIN_SET; }
        break;
    case POT_STATE_VELOCITY_CONTROL:
        g = GPIO_PIN_SET;
        if (waiting_centre) { r = GPIO_PIN_SET; }
        break;
    case POT_STATE_EXTERNAL:            /* balance / sysid active */
        g = GPIO_PIN_SET; r = GPIO_PIN_SET;
        break;
    }
    HAL_GPIO_WritePin(LED_GREEN_PORT,  LED_GREEN_PIN,  g);
    HAL_GPIO_WritePin(LED_YELLOW_PORT, LED_YELLOW_PIN, y);
    HAL_GPIO_WritePin(LED_RED_PORT,    LED_RED_PIN,    r);
}

/* ========================================================================
 *  ODrive state-transition helpers
 * ===================================================================== */

/* Arm the ODrive into closed-loop control.
 *
 * The ODrive S1 uses its onboard (absolute) magnetic encoder: motor and
 * encoder-offset calibration are performed once in odrivetool and saved to
 * the ODrive, so there is NO index search — the axis goes straight from IDLE
 * to CLOSED_LOOP_CONTROL. We only clear any stale latched error first (e.g.
 * WATCHDOG_TIMER_EXPIRED carried over from a previous boot); without this the
 * Set_Axis_State would be rejected and the axis would silently stay DISARMED.
 * Called from PotCtrl_Tick (main-loop context), so the short HAL_Delay is
 * safe. */
static void arm_closed_loop(void)
{
    (void)Odrive_ClearErrors();
    HAL_Delay(20);   /* let the ODrive process the clear before the next cmd */
    Odrive_SetAxisState(ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL);
}

static void enter_position_control(bool already_closed_loop)
{
    /* Hold the cart where it is; the position-match gate releases control
     * to the active source once its target comes to the cart. */
    float entry_pos = Odrive_GetMotorPos();
    s_last_sent_pos = entry_pos;
    s_rail_cmd      = RailLimits_MotorToRail(entry_pos);
    s_rail_desired  = s_rail_cmd;
    s_pos_centre_ok = false;

    Odrive_SetControllerMode(ODRIVE_CTRL_MODE_POSITION, ODRIVE_INPUT_MODE_PASSTHROUGH);

    if (!already_closed_loop) {
        arm_closed_loop();
    }

    write_cmd_pair(CTRL_MODE_POT_POSITION, entry_pos);
}

static void enter_velocity_control(void)
{
    Odrive_SetControllerMode(ODRIVE_CTRL_MODE_VELOCITY, ODRIVE_INPUT_MODE_PASSTHROUGH);
    /* Closed-loop should already be active from prior position state. */

    s_vel_centre_ok = false;
    s_vel_current   = 0.0f;
    write_cmd_pair(CTRL_MODE_POT_VELOCITY, 0.0f);
}

static void exit_to_idle(void)
{
    /* Stop any external controller first so no fresh torque lands between
     * the mode write and the axis-IDLE command below. */
    BalanceCtrl_Stop();
    SwingupCtrl_Stop();
    SysId_Stop();

    write_cmd_pair(CTRL_MODE_IDLE, 0.0f);   /* stop control_loop.c TX */
    Odrive_SetAxisState(ODRIVE_AXIS_STATE_IDLE);

    s_pos_centre_ok = false;
    s_vel_centre_ok = false;
    s_vel_current   = 0.0f;
}

/* ========================================================================
 *  Button FSM (ported verbatim from reference, simplified naming)
 * ===================================================================== */

static btn_event_t button_fsm_tick(bool raw_pressed)
{
    btn_event_t event = BTN_EVENT_NONE;
    uint32_t    now   = HAL_GetTick();

    switch (s_btn_fsm) {

    case BTN_FSM_IDLE:
        if (raw_pressed) {
            s_btn_ts  = now;
            s_btn_fsm = BTN_FSM_DEBOUNCE;
        }
        break;

    case BTN_FSM_DEBOUNCE:
        if (!raw_pressed) {
            s_btn_fsm = BTN_FSM_IDLE;
        } else if ((now - s_btn_ts) >= BTN_DEBOUNCE_MS) {
            s_btn_fsm = BTN_FSM_PRESSED;
            s_btn_ts  = now;
        }
        break;

    case BTN_FSM_PRESSED:
        if (!raw_pressed) {
            s_btn_fsm = BTN_FSM_RELEASED_WAIT;
            s_btn_ts  = now;
        } else if ((now - s_btn_ts) >= BTN_LONG_HOLD_MS) {
            event     = BTN_EVENT_LONG_HOLD;
            s_btn_fsm = BTN_FSM_AFTER_LONG_HOLD;
        } else if ((now - s_btn_ts) >= BTN_SHORT_HOLD_MS) {
            event     = BTN_EVENT_SHORT_HOLD;
            s_btn_fsm = BTN_FSM_AFTER_SHORT_HOLD;
        }
        break;

    case BTN_FSM_RELEASED_WAIT:
        if (raw_pressed) {
            s_btn_fsm = BTN_FSM_SECOND_PRESS;
            s_btn_ts  = now;
        } else if ((now - s_btn_ts) >= BTN_DOUBLE_CLICK_MS) {
            event     = BTN_EVENT_SINGLE_CLICK;
            s_btn_fsm = BTN_FSM_IDLE;
        }
        break;

    case BTN_FSM_SECOND_PRESS:
        if (!raw_pressed) {
            s_btn_fsm = BTN_FSM_RELEASED_WAIT_2;
            s_btn_ts  = now;
        }
        break;

    case BTN_FSM_RELEASED_WAIT_2:
        if (raw_pressed) {
            s_btn_fsm = BTN_FSM_THIRD_PRESS;
        } else if ((now - s_btn_ts) >= BTN_DOUBLE_CLICK_MS) {
            event     = BTN_EVENT_DOUBLE_CLICK;
            s_btn_fsm = BTN_FSM_IDLE;
        }
        break;

    case BTN_FSM_THIRD_PRESS:
        if (!raw_pressed) {
            event     = BTN_EVENT_TRIPLE_CLICK;
            s_btn_fsm = BTN_FSM_IDLE;
        }
        break;

    case BTN_FSM_AFTER_SHORT_HOLD:
        if ((now - s_btn_ts) >= BTN_LONG_HOLD_MS) {
            event     = BTN_EVENT_LONG_HOLD;
            s_btn_fsm = BTN_FSM_AFTER_LONG_HOLD;
        } else if (!raw_pressed) {
            s_btn_fsm = BTN_FSM_IDLE;
        }
        break;

    case BTN_FSM_AFTER_LONG_HOLD:
        if (!raw_pressed) s_btn_fsm = BTN_FSM_IDLE;
        break;
    }
    return event;
}
