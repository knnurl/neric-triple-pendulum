/**
 * lv_conf.h — minimal LVGL 8.3 config for the ESP32-S3 + 800x480 RGB display.
 *
 * Drop into include/ — platformio.ini sets -DLV_CONF_INCLUDE_SIMPLE so LVGL
 * picks this up automatically.
 *
 * If you're using LVGL 9.x, regenerate this from the v9 template instead;
 * the v8 / v9 configs are NOT interchangeable.
 */
#pragma once

#define LV_USE_DEV_VERSION   0
#define LV_COLOR_DEPTH       16
#define LV_COLOR_16_SWAP     0
#define LV_USE_GPU           0

#define LV_MEM_CUSTOM        1     /* use ESP heap caps */
#include <stdlib.h>
#define LV_MEM_CUSTOM_INCLUDE  <stdlib.h>
#define LV_MEM_CUSTOM_ALLOC    malloc
#define LV_MEM_CUSTOM_FREE     free
#define LV_MEM_CUSTOM_REALLOC  realloc

#define LV_DISP_DEF_REFR_PERIOD     16
#define LV_INDEV_DEF_READ_PERIOD    20

#define LV_TICK_CUSTOM       0     /* we call lv_tick_inc manually from loop() */

#define LV_USE_LOG           1
#define LV_LOG_PRINTF        1
#define LV_LOG_LEVEL         LV_LOG_LEVEL_WARN

/* Widgets we use */
#define LV_USE_LABEL         1
#define LV_USE_BTN           1
#define LV_USE_BAR           1
#define LV_USE_SLIDER        1
#define LV_USE_SPINBOX       1
#define LV_USE_LINE          1     /* pendulum-arm visualisation */
#define LV_USE_ARC           1     /* tilt gauge on the challenge screen */

#define LV_LABEL_LONG_TXT_HINT  1

/* Fonts — the professional UI leans on a wide size range (hero timer 48,
 * KPI values 28/40, body 16/20). Each adds a few KB of flash; the N16R8
 * board has 16 MB so this is comfortable. */
#define LV_FONT_MONTSERRAT_12   1
#define LV_FONT_MONTSERRAT_14   1
#define LV_FONT_MONTSERRAT_16   1
#define LV_FONT_MONTSERRAT_20   1
#define LV_FONT_MONTSERRAT_22   1
#define LV_FONT_MONTSERRAT_28   1
#define LV_FONT_MONTSERRAT_40   1
#define LV_FONT_MONTSERRAT_48   1
#define LV_FONT_DEFAULT         &lv_font_montserrat_16

/* Themes — we style everything explicitly (ui_theme in ui.cpp), so the
 * built-in theme only needs to provide sane widget defaults. */
#define LV_USE_THEME_DEFAULT  1
#define LV_THEME_DEFAULT_DARK 1
