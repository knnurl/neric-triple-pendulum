/**
 ******************************************************************************
 * @file    watchdog.h
 * @brief   M4 watchdog supervisor.
 *
 *  Two-layer protection (per checklist Step 7):
 *    - WWDG2: window watchdog, ~20 ms from refresh to reset. Primary monitor.
 *    - IWDG2: independent watchdog, ~3 s timeout. Backstop in case any code
 *      path holds off WWDG refreshes (e.g. WWDG ISR loop).
 *
 *  Refresh policy:
 *    The watchdog task wakes every 5 ms and reads g_shared.m7_heartbeat.
 *    If the heartbeat has advanced since the last check, both watchdogs are
 *    refreshed. If the heartbeat has stalled for more than a short grace
 *    window, the task stops refreshing. WWDG2 then fires within ~20 ms and
 *    the system resets. IWDG2 provides a backstop if M4 itself is stuck.
 *
 *  Fault logging:
 *    The WWDG2 Early Wakeup Interrupt fires ~328 us before the reset. We
 *    use that window to write a fault record to D3 SRAM (0x38000000) which
 *    survives the warm reset and can be read by the debugger after reboot.
 *
 *  HSEM:
 *    g_shared.m7_heartbeat is an aligned 32-bit field. Cortex-M loads of
 *    aligned 32-bit values are atomic, so HSEM is NOT required for this
 *    read. Reserved HSEM_ID_HEARTBEAT in shared_state.h is unused.
 ******************************************************************************
 */

#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Watchdog task period (ms). Must be << WWDG2 window (~20 ms). */
#define WATCHDOG_TASK_PERIOD_MS    5u

/* Number of consecutive ticks the heartbeat may be stale before we stop
 * refreshing. 3 ticks * 5 ms = 15 ms grace. WWDG2 then fires within ~20 ms. */
#define WATCHDOG_STALE_TICK_LIMIT  3u

/* D3 SRAM fault log address (per checklist Step 12). D3 SRAM survives warm
 * reset; debugger can read this after the watchdog resets the chip. */
#define WATCHDOG_FAULT_LOG_ADDR    0x38000000u

/* Fault record stored in D3 SRAM. Magic at offset 0 lets us recognise a
 * valid record vs. random SRAM contents after cold boot. */
typedef struct {
    uint32_t magic;              /* WATCHDOG_FAULT_MAGIC if valid */
    uint32_t last_m7_heartbeat;  /* value of g_shared.m7_heartbeat at refusal */
    uint32_t reset_tick;         /* HAL_GetTick() at refusal */
    uint32_t m7_fault_flags;     /* snapshot of g_shared.m7_fault_flags */
    uint32_t m4_fault_flags;     /* snapshot of g_shared.m4_fault_flags */
    uint32_t loop_overrun_cnt;   /* snapshot of g_shared.loop_overrun_cnt */
    uint32_t ewi_fired;          /* set to 1 in WWDG EWI handler */
    uint32_t _reserved[9];
} watchdog_fault_log_t;

#define WATCHDOG_FAULT_MAGIC       0x57444F47u   /* 'WDOG' */

/* Task body — assign to FreeRTOS via osThreadNew(StartWatchdogTask,...) */
void StartWatchdogTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* WATCHDOG_H */
