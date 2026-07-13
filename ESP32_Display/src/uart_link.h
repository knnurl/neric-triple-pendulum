/**
 * uart_link.h — ESP32-S3 side of the STM32 link.
 *
 *  Provides:
 *    - UartLink_Begin()   : open HardwareSerial1 at 921600, prepare ring buf
 *    - UartLink_Poll()    : drain RX, feed parser, dispatch any complete frame
 *    - UartLink_LastTelemetry() : returns const* to last good telemetry frame
 *                                 (NULL if none yet, or if link is stale)
 *    - UartLink_SendMode/Setpoint/GainDelta/ClearErrors() : TX helpers
 *
 *  Call UartLink_Poll() periodically from the main loop or a FreeRTOS task
 *  (e.g. every 5 ms). It's non-blocking.
 */
#pragma once

#include <stdint.h>
#include "esp_link_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Telemetry-stale threshold. If no valid frame for this long, UartLink_LastTelemetry()
 * returns NULL so the UI can show "signal lost". */
#define UART_LINK_STALE_MS    500u

void UartLink_Begin(int8_t rx_pin, int8_t tx_pin);
void UartLink_Poll(void);

const EspTelemetry_t* UartLink_LastTelemetry(void);

int  UartLink_SendMode(uint8_t mode);
int  UartLink_SendSetpoint(float sp);
int  UartLink_SendGainDelta(uint8_t index, float delta);
int  UartLink_SendClearErrors(void);
int  UartLink_SendHome(void);          /* rail homing / zero reference */
int  UartLink_SendZeroUpright(void);   /* capture upright theta zero */

/* Stats — useful on a debug screen. */
uint32_t UartLink_RxFrames(void);
uint32_t UartLink_CrcErrors(void);
uint32_t UartLink_LastAckSeq(void);
uint8_t  UartLink_LastAckStatus(void);

#ifdef __cplusplus
}
#endif
