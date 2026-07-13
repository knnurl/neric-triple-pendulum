/**
 * ui.h — Professional LVGL UI for the pendulum demo console.
 *
 *  Target: Waveshare ESP32-S3-Touch-LCD-4.3, 800x480, dark kiosk theme.
 *
 *  Three pages, switched by a bottom nav bar:
 *
 *    LIVE       — real-time cart+pendulum visualisation and KPI tiles.
 *    CHALLENGE  — the crowd demo. A physical fader (wired to the STM32 pot
 *                 input, driving POT_POSITION) lets a person try to balance
 *                 the pole by hand while a timer scores how long they keep
 *                 it up. One button hands control to the LQR so the machine
 *                 can show how it's done.
 *    EXPERT     — full control: every mode, setpoint, live LQR gain nudges,
 *                 home / zero-theta / clear-errors, and fault detail.
 *
 *  A persistent top bar (active mode pill, link + rail status) sits above
 *  all three pages.
 *
 *  Touch is handled by LVGL's input device (display_drv.cpp / GT911).
 */
#pragma once

#include <stdint.h>
#include "esp_link_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Build all screens. Call once, after lv_init() + Display_Init(). */
void Ui_Init(void);

/* Refresh every telemetry-bound widget and advance the challenge timer.
 *   t       : latest telemetry, or NULL when the link is stale.
 *   now_ms  : millis(), for the challenge stopwatch.
 * Call at ~30 Hz from the main loop. */
void Ui_UpdateTelemetry(const EspTelemetry_t *t, uint32_t now_ms);

/* Feed the last command acknowledgement (ESPLINK_ACK_*). */
void Ui_SetAckStatus(uint8_t status, uint32_t seq);

#ifdef __cplusplus
}
#endif
