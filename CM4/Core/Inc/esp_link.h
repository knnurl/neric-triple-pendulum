/**
 ******************************************************************************
 * @file    esp_link.h
 * @brief   Bidirectional UART link to ESP32-S3 display (M4 side).
 *
 *  TX task   : packs telemetry from g_shared, ~50 Hz, DMA out of huart2.
 *  RX task   : drains DMA circular ring on idle-line wake, parses frames,
 *              validates, applies under HSEM_ID_M4_TO_M7 to g_shared.
 *
 *  Wiring (after .ioc applies):
 *      USART2 -> CortexM4
 *      PD5 = USART2_TX
 *      PD6 = USART2_RX
 *      Baud 921600 8N1
 *      DMA1 streams for TX (Normal) and RX (Circular)
 *      USART2 IRQ + DMA IRQs : preempt 5, sub 0
 ******************************************************************************
 */

#ifndef ESP_LINK_H
#define ESP_LINK_H

#include <stdint.h>
#include "esp_link_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TX rate (Hz). 50 Hz keeps display update visually smooth and stays well
 * under bus capacity. Decoupled from the MATLAB UDP telemetry rate. */
#define ESPLINK_TX_RATE_HZ        50u
#define ESPLINK_TX_PERIOD_MS      (1000u / ESPLINK_TX_RATE_HZ)

/* RX ring buffer size. Big enough for two max-length frames back to back.
 * DMA writes circularly; the parser drains. */
#define ESPLINK_RX_RING_SIZE      512u

/* Bounds checks for incoming commands. Sane-value gates before any write
 * to g_shared. Values outside these limits raise ESPLINK_ACK_OUT_OF_RANGE. */
#define ESPLINK_SETPOINT_MAX_ABS  10.0f
#define ESPLINK_GAIN_DELTA_MAX    1.0f
#define ESPLINK_GAIN_INDEX_MAX    5u

/* Task entry points — pass to osThreadNew(...) in main.c. */
void EspLink_TxTask(void *argument);
void EspLink_RxTask(void *argument);

/* Init: starts DMA RX and resets parser state. Call once before the
 * RX/TX tasks are scheduled (or have either task call it on entry — we do
 * it on RX task entry since RX init is more invasive). */
void EspLink_Init(void);

/* Callback hooks from stm32h7xx_it.c / HAL weak overrides. These are not
 * normally called by user code. */
void EspLink_OnRxIdleOrCpltISR(uint16_t bytes_in_dma);  /* HAL_UARTEx_RxEventCallback */
void EspLink_OnTxCpltISR(void);                          /* HAL_UART_TxCpltCallback */

#ifdef __cplusplus
}
#endif

#endif /* ESP_LINK_H */
