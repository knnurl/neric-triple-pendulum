/**
 ******************************************************************************
 * @file    control_loop.c
 * @brief   M7 bare-metal control loop (Step 2 scaffold).
 ******************************************************************************
 */

#include "control_loop.h"
#include "control_input.h"
#include "main.h"
#include "shared_state.h"
#include "odrive.h"
#include "as5047p.h"
#include "rail_limits.h"
#include "state_est.h"
#include "balance_ctrl.h"
#include "swingup_ctrl.h"
#include "sysid.h"

extern TIM_HandleTypeDef htim7;

/* Cycles per loop period at 400 MHz CPU clock. 5 kHz -> 80000 cycles.
 * Overrun threshold: 1.5x expected, allows ~50% jitter without false alarms. */
#define LOOP_PERIOD_CYC      (uint32_t)(400000000u / CONTROL_LOOP_HZ)
#define LOOP_OVERRUN_CYC     (LOOP_PERIOD_CYC + (LOOP_PERIOD_CYC / 2u))

#if CONTROL_LOOP_OVERRUN_CHECK
static uint32_t s_last_entry_cyc = 0u;
static uint8_t  s_first_tick     = 1u;
#endif

/* Feed ODrive watchdog every WDOG_FEED_INTERVAL ticks.
 * At 5 kHz, 10 ticks = 2 ms interval — well within any 500 ms timeout. */
#define WDOG_FEED_INTERVAL  10u
static uint32_t s_wdog_tick = 0u;

/* Encoder reads decimated to 1 kHz (every 5th tick). Bounds the worst-case
 * ISR cost if the SPI HAL hits its 1 ms timeout: 3 encoders x 1 ms at 1 kHz
 * is recoverable; at the full 5 kHz it starves the M7 heartbeat and M4's
 * watchdog resets the chip (HANDOFF §5.6). */
#define ENC_READ_DECIMATION 5u
static uint32_t s_enc_tick = 0u;

/* Push one (tick, theta, theta_dot, x, x_dot, u) sample into the shared
 * SPSC idlog ring (M4 drains -> UDP 5007). Runs on the 1 kHz estimator tick
 * while mode is SYSID or BALANCE. Ring full -> drop + count: for system ID
 * a visible gap beats silently stale data. */
static void idlog_push(float u)
{
    uint32_t head = g_shared.idlog_head;
    if ((head - g_shared.idlog_tail) >= IDLOG_RING_LEN) {
        g_shared.idlog_drops++;
        return;
    }
    uint32_t i = head % IDLOG_RING_LEN;

    state_est_t e;
    StateEst_Get(&e);   /* same-ISR snapshot */

    g_shared.idlog[i].tick      = g_shared.loop_count;
    g_shared.idlog[i].theta     = e.theta;
    g_shared.idlog[i].theta_dot = e.theta_dot;
    g_shared.idlog[i].x         = e.x;
    g_shared.idlog[i].x_dot     = e.x_dot;
    g_shared.idlog[i].u         = u;

    __DMB();                          /* sample visible before publish */
    g_shared.idlog_head = head + 1u;
}

#if CTRL_INPUT_ACCEL
/* Accel-input path (control_input.h): velocity command integrated from the
 * controllers' acceleration output. Reset rules live in ControlLoop_Tick. */
static float s_vel_cmd = 0.0f;
#endif

/* Send one control-input frame for the external controllers. u is a torque
 * [Nm] on the default path; on the accel path it is a cart acceleration
 * [rev/s^2], integrated here into the streamed velocity command (ZOH accel
 * between 1 kHz controller updates -> velocity ramps smoothly at 5 kHz).
 * The clamp is a hard safety bound independent of the controllers. */
static inline int ctrl_send_u(float u)
{
#if CTRL_INPUT_ACCEL
    s_vel_cmd += u * (1.0f / (float)CONTROL_LOOP_HZ);
    if (s_vel_cmd >  CTRL_VEL_CMD_MAX_REV_S) s_vel_cmd =  CTRL_VEL_CMD_MAX_REV_S;
    if (s_vel_cmd < -CTRL_VEL_CMD_MAX_REV_S) s_vel_cmd = -CTRL_VEL_CMD_MAX_REV_S;
    return Odrive_SetInputVel(s_vel_cmd, 0.0f);
#else
    return Odrive_SetInputTorque(u);
#endif
}

void ControlLoop_Init(void)
{
    /* Enable DWT cycle counter. Used for overrun timing and (later) ISR
     * profile. CoreDebug ensures TRC is enabled before DWT starts. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT  = 0u;
    DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;
}

void ControlLoop_Start(void)
{
    /* HAL_TIM_Base_Start_IT enables update interrupt and starts the counter.
     * From this point the TIM7 ISR fires every 200 us at preempt priority 0. */
    if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK) {
        Error_Handler();
    }
}

void ControlLoop_Tick(void)
{

	GPIOE->BSRR = GPIO_PIN_2;        /* set high — ISR entry */

#if CONTROL_LOOP_OVERRUN_CHECK
    uint32_t now = DWT->CYCCNT;
    if (!s_first_tick) {
        uint32_t delta = now - s_last_entry_cyc;     /* unsigned wrap-safe */
        if (delta > LOOP_OVERRUN_CYC) {
            g_shared.loop_overrun_cnt++;
            g_shared.m7_fault_flags |= M7_FAULT_LOOP_OVERRUN;
        }
    } else {
        s_first_tick = 0u;
    }
    s_last_entry_cyc = now;
#endif

    /* --- Step 3: read all 3 AS5047P encoders (sequential polled SPI) -----
     * ~15 us at 6.25 MHz SCK, decimated to every 5th tick (1 kHz).
     * On timeout, sets M7_FAULT_SPI_TIMEOUT and we keep last-known angles. */
    bool est_tick = false;
#if ENC_READ_NONBLOCKING
    /* IRQ-chained reads EVERY 5 kHz tick: kick this round, consume the
     * previous one (~200 us old data, zero ISR stall). The estimator still
     * runs at its 1 kHz cadence below — it just picks up a fresher angle. */
    AS5047P_TickNonBlocking();
#endif
    if (++s_enc_tick >= ENC_READ_DECIMATION) {
        s_enc_tick = 0u;
        est_tick   = true;
#if ENC_READ_NONBLOCKING
        {
            float ang[3];
            AS5047P_GetAnglesRad(ang);   /* last validated round */
            g_shared.link_angle_rad[0] = ang[0];
            g_shared.link_angle_rad[1] = ang[1];
            g_shared.link_angle_rad[2] = ang[2];
        }
#else
        if (AS5047P_ReadAllBlocking()) {
            float ang[3];
            AS5047P_GetAnglesRad(ang);
            g_shared.link_angle_rad[0] = ang[0];
            g_shared.link_angle_rad[1] = ang[1];
            g_shared.link_angle_rad[2] = ang[2];
        }
#endif

        /* --- Step 4: state estimate [x, x_dot, theta, theta_dot] --------
         * Runs at the same 1 kHz cadence, right after the fresh angles. */
        StateEst_Tick();
    }

    /* --- ODrive link supervision & CAN TX -------------------------------- *
     * Odrive_Tick monitors heartbeat staleness and updates the fault bit.
     * We dispatch the CAN command by controller_mode:
     *   POT_POSITION / POT_VELOCITY -> direct command from g_shared.motor_command
     *   SWINGUP / BALANCE           -> LQR output (Step 4 future)
     *   IDLE / SAFE_STOP / unknown  -> no TX (ODrive idles naturally)
     * If the link is down, force IDLE -- never push commands to a dead bus. */
    uint32_t now_ms  = HAL_GetTick();
    bool     link_ok = Odrive_Tick(now_ms);

    /* Rail hard-limit supervisor. Runs BEFORE the mode dispatch below: when
     * it re-arms after a trip it rewrites motor_command/controller_mode, and
     * this ordering makes the same tick send the corrected target instead of
     * the stale one that caused the trip. */
    RailLimits_Tick(Odrive_GetMotorPos(), now_ms);

    uint8_t mode = g_shared.controller_mode;
    float   cmd  = g_shared.motor_command;
    if (!link_ok) { mode = CTRL_MODE_IDLE; cmd = 0.0f; }

#if CTRL_INPUT_ACCEL
    /* Reset the velocity-command integrator whenever we are NOT streaming
     * an external controller's accel output — covers IDLE, POT modes,
     * SAFE_STOP, link-down forcing, sysid FREE runs, and the tick after
     * any abort. Deliberately KEPT across the swing-up -> balance catch
     * (both in the accel set) so the handoff velocity is continuous. */
    if (!(mode == CTRL_MODE_BALANCE || mode == CTRL_MODE_SWINGUP ||
          (mode == CTRL_MODE_SYSID && SysId_IsDriving()))) {
        s_vel_cmd = 0.0f;
    }
#endif

    int hal_rc = 0;
    bool active = false;
    switch (mode) {
    case CTRL_MODE_POT_POSITION:
        hal_rc = Odrive_SetInputPos(cmd, 0, 0);
        active = true;
        break;
    case CTRL_MODE_POT_VELOCITY:
        hal_rc = Odrive_SetInputVel(cmd, 0.0f);
        active = true;
        break;
    case CTRL_MODE_BALANCE:
        /* LQR loop (balance_ctrl.c). Recomputes on the 1 kHz estimator
         * tick, holds in between; the 5 kHz stream doubles as the ODrive
         * watchdog feed. u is torque or accel per control_input.h —
         * ctrl_send_u handles the difference. On an internal abort the
         * tick returns 0 and has already idled the axis — one trailing
         * zero frame to an IDLE axis is ignored (and still feeds wdog). */
        hal_rc = ctrl_send_u(BalanceCtrl_Tick(est_tick));
        active = true;
        break;
    case CTRL_MODE_SYSID: {
        /* Excitation run (sysid.c). Driving types stream their u; a FREE
         * (unpowered) run sends nothing and takes the explicit-feed path
         * below so the ODrive watchdog stays quiet while the axis idles. */
        float u = SysId_Tick(est_tick);
        if (SysId_IsDriving()) {
            hal_rc = ctrl_send_u(u);
            active = true;
        } else {
            active = false;
        }
        break;
    }
    case CTRL_MODE_SWINGUP:
        /* Energy pump (swingup_ctrl.c). On capture it flips the mode to
         * BALANCE mid-tick and returns a bridging frame, so the stream
         * (= watchdog feed) never pauses across the handoff. On abort it
         * returns 0 with the axis already idled — same contract as the
         * balance tick. */
        hal_rc = ctrl_send_u(SwingupCtrl_Tick(est_tick));
        active = true;
        break;
    case CTRL_MODE_IDLE:
    case CTRL_MODE_SAFE_STOP:
    default:
        /* No TX — intentionally do NOT feed ODrive watchdog here so the
         * ODrive times out and stops the motor on its own. */
        active = false;
        break;
    }

    /* --- High-rate ID/tuning log (1 kHz, SYSID / BALANCE / SWINGUP) ------ */
    if (est_tick &&
        (mode == CTRL_MODE_SYSID || mode == CTRL_MODE_BALANCE ||
         mode == CTRL_MODE_SWINGUP)) {
        idlog_push(g_shared.motor_command);
    }
    if (hal_rc != 0 /* HAL_OK */) {
        g_shared.m7_fault_flags |= M7_FAULT_CAN_TX_FAIL;
    }

    /* Feed the ODrive axis watchdog only when we are NOT streaming commands.
     *
     * In active modes the Set_Input_Pos/Vel stream above already resets the
     * ODrive watchdog every 200 us — an explicit Set_Input_Vel(0) feed there
     * would periodically ZERO the live setpoint (torque ripple). So:
     *   - active modes  → command stream is the feed, no explicit feed
     *   - IDLE          → explicit feed every 2 ms ("STM32 alive, supervising")
     *   - SAFE_STOP     → deliberately withhold feed → ODrive watchdog fires ✓
     *   - Cable pulled / M7 halted → nothing reaches ODrive → fires ✓
     *
     * NOTE (2026-07-11 power-up→balance trace): controller_mode==SAFE_STOP
     * is currently UNREACHABLE — pot_ctrl maps remote SAFE_STOP and IDLE
     * requests alike to exit_to_idle(), which writes IDLE and disarms the
     * axis EXPLICITLY (SetAxisState IDLE — stronger than starvation). The
     * branch stays as belt-and-braces for any future SAFE_STOP writer; the
     * ODrive watchdog's real-world backstop cases are the last line above
     * (cable pulled / M7 halted), which is also how it must be BENCH
     * VERIFIED (STATUS §1c) — no GUI action starves the feed today.
     *
     * We do NOT gate on link_ok: at boot before the first ODrive heartbeat
     * arrives we'd otherwise withhold feeds and the ODrive watchdog would
     * trip almost immediately. Feeds during "no link" cost nothing — they
     * just don't get ACKed.
     */
    s_wdog_tick++;
    if (active || mode == CTRL_MODE_SAFE_STOP) {
        s_wdog_tick = 0u;
    } else if (s_wdog_tick >= WDOG_FEED_INTERVAL) {
        s_wdog_tick = 0u;
        (void)Odrive_FeedWatchdog();
    }

    /* --- Heartbeat to M4 (single 32-bit atomic write) -------------------- *
     * No HSEM required: aligned 32-bit stores are atomic on Cortex-M.       *
     * M4's watchdog task reads this to decide whether to refresh IWDG2.     */
    g_shared.loop_count++;
    g_shared.m7_heartbeat = g_shared.loop_count;

    GPIOE->BSRR = (uint32_t)GPIO_PIN_2 << 16u; // set low - ISR exit //
}
