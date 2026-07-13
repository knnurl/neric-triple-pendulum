/**
 * main.cpp — Waveshare ESP32-S3-Touch-LCD-4.3 firmware entry.
 *
 *  Subsystems:
 *      1. UART link to STM32 (uart_link.cpp)
 *      2. LVGL UI (ui.cpp)
 *      3. Waveshare RGB panel + GT911 touch (display_drv.cpp)
 *
 *  Wiring to STM32 (NOT default Arduino RX/TX — those would conflict
 *  with LCD data lines!):
 *      ESP32 GPIO15 (TX) -> STM32 PD6 (USART2_RX)
 *      ESP32 GPIO16 (RX) <- STM32 PD5 (USART2_TX)
 *      GND <-> GND
 *
 *  IMPORTANT: GPIO 17 and GPIO 18 on this board are LCD blue-data pins.
 *  Do NOT change UART pins back to "default" 17/18 — the display will
 *  glitch.
 */

#include <Arduino.h>
#include <lvgl.h>

#include "uart_link.h"
#include "ui.h"
#include "display_drv.h"

/* ============================================================
 *  UART pin assignments to STM32
 * ============================================================ */
static constexpr int8_t UART_RX_PIN = 16;   /* connects to STM32 PD5 (USART2_TX) */
static constexpr int8_t UART_TX_PIN = 15;   /* connects to STM32 PD6 (USART2_RX) */

/* ============================================================
 *  setup / loop
 * ============================================================ */
void setup(void)
{
    Serial.begin(115200);
    delay(200);
    Serial.println("\n--- ESP32 Pendulum Display boot ---");

    /* 1. UART to STM32 first — so we don't lose telemetry frames during
     *    the (slow) display init. The HardwareSerial RX is buffered. */
    UartLink_Begin(UART_RX_PIN, UART_TX_PIN);
    Serial.println("UART link up (GPIO15 TX / GPIO16 RX, 921600).");

    /* 2. LVGL core */
    lv_init();

    /* 3. Display + touch via esp_lcd panel driver */
    if (Display_Init() != 0) {
        Serial.println("FATAL: Display_Init failed — check LCD pin map");
        while (true) { delay(1000); }
    }
    if (Touch_Init() != 0) {
        Serial.println("WARN: Touch_Init failed — continuing without touch");
        /* not fatal — UI still renders, just no input */
    }

    /* 4. Build the UI */
    Ui_Init();
    Serial.println("UI initialised. Awaiting telemetry...");
}

static uint32_t s_last_lv_tick   = 0;
static uint32_t s_last_ui_update = 0;
static uint32_t s_last_ack_seq   = 0xFFFFFFFFu;

void loop(void)
{
    /* Drain UART, dispatch frames into uart_link state */
    UartLink_Poll();

    /* LVGL tick + handler */
    uint32_t now = millis();
    if (now - s_last_lv_tick > 5) {
        lv_tick_inc(now - s_last_lv_tick);
        s_last_lv_tick = now;
    }
    lv_timer_handler();

    /* Refresh telemetry-bound widgets at ~30 Hz */
    if (now - s_last_ui_update > 33) {
        s_last_ui_update = now;
        Ui_UpdateTelemetry(UartLink_LastTelemetry(), now);

        uint32_t aseq = UartLink_LastAckSeq();
        if (aseq != s_last_ack_seq) {
            s_last_ack_seq = aseq;
            Ui_SetAckStatus(UartLink_LastAckStatus(), aseq);
        }
    }
}
