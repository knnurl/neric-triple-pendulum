/**
 * display_drv.h — Waveshare ESP32-S3-Touch-LCD-4.3 driver glue for LVGL.
 *
 *  Uses Espressif's esp_lcd_panel_rgb (built into the Arduino-ESP32 v3
 *  framework via ESP-IDF). Wraps it for LVGL flush + GT911 touch input.
 *
 *  Pin map below is from Waveshare's official schematic for the
 *  ESP32-S3-Touch-LCD-4.3 v1.x. CONFIRM against your board revision —
 *  Waveshare ships at least two variants with different LCD pin maps.
 */
#pragma once

#include <stdint.h>
#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Display resolution */
#define DISPLAY_H_RES  800
#define DISPLAY_V_RES  480

/* ===== RGB LCD pins (verify against your Waveshare schematic) ===== */
#define LCD_PIN_PCLK   7
#define LCD_PIN_HSYNC  46
#define LCD_PIN_VSYNC  3
#define LCD_PIN_DE     5
#define LCD_PIN_BL     2   /* backlight; some revs route this via IO expander */

/* RGB565 data lines — order: B0..B4, G0..G5, R0..R4 (low to high bit) */
#define LCD_PIN_B0     14
#define LCD_PIN_B1     38
#define LCD_PIN_B2     18
#define LCD_PIN_B3     17
#define LCD_PIN_B4     10

#define LCD_PIN_G0     39
#define LCD_PIN_G1     0
#define LCD_PIN_G2     45
#define LCD_PIN_G3     48
#define LCD_PIN_G4     47
#define LCD_PIN_G5     21

#define LCD_PIN_R0     1
#define LCD_PIN_R1     9    /* note: also commonly mapped — verify */
#define LCD_PIN_R2     42
#define LCD_PIN_R3     41
#define LCD_PIN_R4     40

/* ===== GT911 touch I2C ===== */
#define TOUCH_I2C_SDA  8
#define TOUCH_I2C_SCL  9
#define TOUCH_INT_PIN  4
#define TOUCH_RST_PIN  (-1)   /* commonly via IO expander; -1 = unused here */
#define TOUCH_I2C_ADDR 0x5D   /* GT911 default; some boots come up at 0x14 */

/* Init the LCD panel + LVGL display driver. Allocates draw buffers in
 * PSRAM (full-frame double-buffered if memory allows). Returns 0 on
 * success, non-zero on failure. */
int Display_Init(void);

/* Init GT911 touch + LVGL input device. Call after Display_Init. */
int Touch_Init(void);

#ifdef __cplusplus
}
#endif
