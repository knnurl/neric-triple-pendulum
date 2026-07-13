/**
 ******************************************************************************
 * @file    watchdog.c
 * @brief   M4 heartbeat-gated watchdog supervisor (Step 7).
 ******************************************************************************
 */

#include "watchdog.h"
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "shared_state.h"

extern IWDG_HandleTypeDef hiwdg2;
extern WWDG_HandleTypeDef hwwdg2;

/* Place the fault log at the fixed D3 SRAM address. Volatile because it
 * survives warm reset; we want explicit writes, not optimised-away stores. */
static volatile watchdog_fault_log_t * const s_fault_log =
        (volatile watchdog_fault_log_t *)WATCHDOG_FAULT_LOG_ADDR;

/* ---- Task body --------------------------------------------------------- */
void StartWatchdogTask(void *argument)
{
    (void)argument;

    /* Initial heartbeat sample. M7 has already initialised the shared region
     * (boot handshake) by the time this task runs. */
    uint32_t last_hb     = g_shared.m7_heartbeat;
    uint32_t stale_ticks = 0u;
    uint8_t  stop_sent   = 0u;   /* SAFE_STOP requested once per stale episode */

    /* Wake on a fixed schedule rather than relative delay, so jitter from
     * higher-priority tasks doesn't shift our cadence. */
    TickType_t next = xTaskGetTickCount();

    for (;;) {
        uint32_t hb = g_shared.m7_heartbeat;     /* atomic 32-bit read */

        if (hb != last_hb) {
            /* M7 is alive — refresh both watchdogs. */
            HAL_WWDG_Refresh(&hwwdg2);
            HAL_IWDG_Refresh(&hiwdg2);
            last_hb     = hb;
            stale_ticks = 0u;
            stop_sent   = 0u;
        } else {
            stale_ticks++;
            if (stale_ticks <= WATCHDOG_STALE_TICK_LIMIT) {
                /* Brief stall — still refresh to absorb transient jitter. */
                HAL_WWDG_Refresh(&hwwdg2);
                HAL_IWDG_Refresh(&hiwdg2);
            } else {
                /* Heartbeat stale past grace. Stop refreshing — WWDG2 will
                 * fire and reset within ~20 ms; IWDG2 within ~2.0 s as
                 * independent backstop (LSI 32 kHz / prescaler 32 = 1 kHz,
                 * reload 2000). Also flag the fault and request a
                 * safe stop in case M7 is still partially alive. Mode
                 * requests are one-shot since 07-11 (F10): payload byte,
                 * then cmd_set_mode=1, then the seq bump — all three are
                 * required for pot_ctrl to see and consume the request. */
                if (!stop_sent) {
                    stop_sent = 1u;
                    taskENTER_CRITICAL();
                    g_shared.m4_fault_flags |= M4_FAULT_WDT_STALE_HB;
                    g_shared.command_mode = CTRL_MODE_SAFE_STOP;
                    g_shared.cmd_set_mode = 1u;
                    g_shared.command_seq++;
                    taskEXIT_CRITICAL();
                }
#ifdef DEBUG
                /* Debug builds: a stale M7 heartbeat is almost always the
                 * debugger holding CM7 (breakpoint, single-step, flash
                 * loader). Resetting the chip ~35 ms into every CM7 halt
                 * makes CM7 undebuggable and aborts flash erases. Keep
                 * refreshing — the fault flag + SAFE_STOP above still fire,
                 * so the failure is visible without the reset. Release
                 * builds keep the hard reset (production safety).
                 *
                 * Corollary (arch review 2026-07-11): this also means a
                 * GENUINE M7 wedge in a DEBUG build never resets — and the
                 * SAFE_STOP requested above is consumed by M7's MAIN LOOP
                 * (PotCtrl_Tick), which a wedged priority-0 ISR starves
                 * too. The only motor backstop is then the ODrive axis
                 * watchdog (U2 config item). Operational rule: DEBUG
                 * builds never run unattended or public-facing. */
                HAL_WWDG_Refresh(&hwwdg2);
                HAL_IWDG_Refresh(&hiwdg2);
#else
                /* no refresh — WWDG2 fires within ~20 ms */
#endif
            }
        }

        vTaskDelayUntil(&next, pdMS_TO_TICKS(WATCHDOG_TASK_PERIOD_MS));
    }
}

/* ---- WWDG2 Early Wakeup Interrupt --------------------------------------- *
 * HAL_WWDG_IRQHandler calls this callback when the EWI fires (~328 us before
 * the reset). Last chance to log fault context to D3 SRAM. Keep it short:
 * the WWDG hardware reset is imminent regardless of what we do here.       */
void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg)
{
    if (hwwdg->Instance != WWDG2) { return; }

    s_fault_log->magic             = WATCHDOG_FAULT_MAGIC;
    s_fault_log->last_m7_heartbeat = g_shared.m7_heartbeat;
    s_fault_log->reset_tick        = HAL_GetTick();
    s_fault_log->m7_fault_flags    = g_shared.m7_fault_flags;
    s_fault_log->m4_fault_flags    = g_shared.m4_fault_flags;
    s_fault_log->loop_overrun_cnt  = g_shared.loop_overrun_cnt;
    s_fault_log->ewi_fired         = 1u;

    /* Optional: a tight breakpoint here lets a connected debugger halt
     * before the reset actually lands. Comment out for unattended use. */
    /* __BKPT(0); */
}
