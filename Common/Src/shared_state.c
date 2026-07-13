/**
 ******************************************************************************
 * @file    shared_state.c
 * @brief   Definition of the single SharedState_t instance placed in D2 SRAM3.
 *
 * Linker places this in section ".shared_d2" — see CM7/STM32H745ZITX_FLASH.ld
 * and CM4/STM32H745ZITX_FLASH.ld. Both cores resolve &g_shared to the same
 * physical SRAM (M7 sees 0x30047000, M4 sees 0x10047000).
 *
 * NOTE: This .c file is COMPILED INTO BOTH the M7 and M4 projects. Each core
 * gets its own extern symbol resolved by its own linker; the addresses are
 * different but point to the same physical memory.
 ******************************************************************************
 */

#include "shared_state.h"
#include "stm32h7xx.h"   /* for HAL types, used by M4 wait helper */

/* Place in the inter-core shared section. NOLOAD in the linker means startup
 * code does not copy initial data from FLASH — it's M7's job to zero this
 * region in SharedState_M7_Init() before releasing M4. */
SharedState_t g_shared __attribute__((section(".shared_d2")));

#ifdef CORE_CM7
/* ---- M7-side init ------------------------------------------------------- *
 * Zero-initialise the 4 KB region, then stamp magic/version, then m7_ready=1.
 * The m7_ready write MUST be the final store so M4 doesn't observe a partly
 * initialised struct. A __DMB() before the flag flush guarantees ordering. */
void SharedState_M7_Init(void)
{
    /* Clear entire instance. Cannot use memset on volatile pointer cleanly
     * without diagnostics; do it as a 32-bit loop. */
    volatile uint32_t *p   = (volatile uint32_t *)&g_shared;
    const  uint32_t   *end = (const uint32_t *)((uint8_t *)&g_shared + sizeof(g_shared));
    while ((const uint32_t *)p < end) { *p++ = 0u; }

    g_shared.magic      = SHARED_STATE_MAGIC;
    g_shared.version    = SHARED_STATE_VERSION;
    g_shared.controller_mode = CTRL_MODE_IDLE;

    /* Ensure all preceding writes drain to memory before the ready flag. */
    __DMB();
    g_shared.m7_ready = 1u;
    __DMB();
}
#endif /* CORE_CM7 */

#ifdef CORE_CM4
/* ---- M4-side wait helper ------------------------------------------------ *
 * Called after HSEM_ID_0 boot release. Polls m7_ready until set or timeout.
 * Uses HAL_GetTick(); safe before scheduler starts. */
int SharedState_M4_WaitReady(uint32_t timeout_ticks)
{
    uint32_t t0 = HAL_GetTick();
    while (g_shared.m7_ready == 0u) {
        if ((HAL_GetTick() - t0) > timeout_ticks) {
            return 0;
        }
    }
    /* Verify magic/version once ready to catch corruption / linker mismatch */
    if (g_shared.magic != SHARED_STATE_MAGIC) { return 0; }
    if (g_shared.version != SHARED_STATE_VERSION) { return 0; }
    return 1;
}
#endif /* CORE_CM4 */
