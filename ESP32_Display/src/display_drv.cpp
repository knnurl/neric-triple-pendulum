/**
 * display_drv.cpp — Waveshare ESP32-S3-Touch-LCD-4.3 LVGL glue.
 *
 *   - 800x480 RGB565 panel via esp_lcd_panel_rgb
 *   - GT911 capacitive touch via esp_lcd_touch_gt911
 *
 *  Frame buffer strategy:
 *      - Two LVGL draw buffers in PSRAM, 1/10th of full frame each (~75 KB).
 *        Smaller than full-frame, but cheap to allocate and avoids tearing
 *        when the RGB panel hammers its scan-out buffer.
 *      - The RGB panel keeps a full 800x480 RGB565 frame buffer in PSRAM
 *        internally (allocated by esp_lcd_new_rgb_panel). LVGL flush_cb
 *        copies our partial buffer into it via esp_lcd_panel_draw_bitmap.
 */

#include "display_drv.h"
#include <Arduino.h>

#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_gt911.h"
#include "driver/i2c.h"
#include "esp_heap_caps.h"

/* ============================================================
 *  Module state
 * ============================================================ */
static esp_lcd_panel_handle_t s_panel_handle    = nullptr;
static esp_lcd_touch_handle_t s_touch_handle    = nullptr;
static lv_disp_drv_t          s_disp_drv;
static lv_disp_draw_buf_t     s_disp_buf;
static lv_indev_drv_t         s_indev_drv;

/* ============================================================
 *  Display flush (LVGL -> RGB panel)
 * ============================================================ */
static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area,
                          lv_color_t *color_p)
{
    const int xs = area->x1;
    const int ys = area->y1;
    const int xe = area->x2 + 1;
    const int ye = area->y2 + 1;
    esp_lcd_panel_draw_bitmap(s_panel_handle, xs, ys, xe, ye, color_p);
    lv_disp_flush_ready(drv);
}

/* ============================================================
 *  Touch read (GT911 -> LVGL input device)
 * ============================================================ */
static void lvgl_touch_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    if (s_touch_handle == nullptr) {
        data->state = LV_INDEV_STATE_REL;
        return;
    }

    esp_lcd_touch_read_data(s_touch_handle);

    uint16_t tx[1], ty[1], tstrength[1];
    uint8_t  tcnt = 0;
    bool pressed = esp_lcd_touch_get_coordinates(s_touch_handle,
                        tx, ty, tstrength, &tcnt, 1);

    if (pressed && tcnt > 0) {
        data->point.x = tx[0];
        data->point.y = ty[0];
        data->state   = LV_INDEV_STATE_PR;
    } else {
        data->state   = LV_INDEV_STATE_REL;
    }
}

/* ============================================================
 *  Display init
 * ============================================================ */
int Display_Init(void)
{
    /* Backlight pin direct-drive (skip if your rev uses IO expander). */
    if (LCD_PIN_BL >= 0) {
        pinMode(LCD_PIN_BL, OUTPUT);
        digitalWrite(LCD_PIN_BL, HIGH);
    }

    esp_lcd_rgb_panel_config_t panel_config = {};
    panel_config.data_width        = 16;
    panel_config.psram_trans_align = 64;
    panel_config.clk_src           = LCD_CLK_SRC_DEFAULT;
    panel_config.disp_gpio_num     = -1;
    panel_config.pclk_gpio_num     = LCD_PIN_PCLK;
    panel_config.vsync_gpio_num    = LCD_PIN_VSYNC;
    panel_config.hsync_gpio_num    = LCD_PIN_HSYNC;
    panel_config.de_gpio_num       = LCD_PIN_DE;

    panel_config.data_gpio_nums[0]  = LCD_PIN_B0;
    panel_config.data_gpio_nums[1]  = LCD_PIN_B1;
    panel_config.data_gpio_nums[2]  = LCD_PIN_B2;
    panel_config.data_gpio_nums[3]  = LCD_PIN_B3;
    panel_config.data_gpio_nums[4]  = LCD_PIN_B4;
    panel_config.data_gpio_nums[5]  = LCD_PIN_G0;
    panel_config.data_gpio_nums[6]  = LCD_PIN_G1;
    panel_config.data_gpio_nums[7]  = LCD_PIN_G2;
    panel_config.data_gpio_nums[8]  = LCD_PIN_G3;
    panel_config.data_gpio_nums[9]  = LCD_PIN_G4;
    panel_config.data_gpio_nums[10] = LCD_PIN_G5;
    panel_config.data_gpio_nums[11] = LCD_PIN_R0;
    panel_config.data_gpio_nums[12] = LCD_PIN_R1;
    panel_config.data_gpio_nums[13] = LCD_PIN_R2;
    panel_config.data_gpio_nums[14] = LCD_PIN_R3;
    panel_config.data_gpio_nums[15] = LCD_PIN_R4;

    panel_config.timings.pclk_hz           = 16 * 1000 * 1000;
    panel_config.timings.h_res             = DISPLAY_H_RES;
    panel_config.timings.v_res             = DISPLAY_V_RES;
    /* The following timings are typical for the Waveshare 4.3" panel.
     * If you see tearing / shifted image, tune against your panel
     * datasheet — Waveshare's wiki has the exact values. */
    panel_config.timings.hsync_pulse_width = 4;
    panel_config.timings.hsync_back_porch  = 8;
    panel_config.timings.hsync_front_porch = 8;
    panel_config.timings.vsync_pulse_width = 4;
    panel_config.timings.vsync_back_porch  = 8;
    panel_config.timings.vsync_front_porch = 8;
    panel_config.timings.flags.pclk_active_neg = 1;

    panel_config.flags.fb_in_psram = 1;
    panel_config.num_fbs           = 1;       /* one full-frame backbuffer */

    if (esp_lcd_new_rgb_panel(&panel_config, &s_panel_handle) != ESP_OK) {
        Serial.println("ERROR: esp_lcd_new_rgb_panel failed");
        return -1;
    }
    if (esp_lcd_panel_init(s_panel_handle) != ESP_OK) {
        Serial.println("ERROR: esp_lcd_panel_init failed");
        return -2;
    }

    /* LVGL draw buffers in PSRAM. ~10% of full frame is a good default. */
    const size_t draw_px = (size_t)DISPLAY_H_RES * (DISPLAY_V_RES / 10);
    lv_color_t *b1 = (lv_color_t *)heap_caps_aligned_alloc(64,
                        draw_px * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    lv_color_t *b2 = (lv_color_t *)heap_caps_aligned_alloc(64,
                        draw_px * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    if (!b1 || !b2) {
        Serial.println("ERROR: PSRAM alloc for LVGL buffers failed");
        return -3;
    }
    lv_disp_draw_buf_init(&s_disp_buf, b1, b2, draw_px);

    lv_disp_drv_init(&s_disp_drv);
    s_disp_drv.hor_res  = DISPLAY_H_RES;
    s_disp_drv.ver_res  = DISPLAY_V_RES;
    s_disp_drv.flush_cb = lvgl_flush_cb;
    s_disp_drv.draw_buf = &s_disp_buf;
    lv_disp_drv_register(&s_disp_drv);

    return 0;
}

/* ============================================================
 *  Touch init
 * ============================================================ */
int Touch_Init(void)
{
    /* I2C0 for GT911 */
    i2c_config_t cfg = {};
    cfg.mode             = I2C_MODE_MASTER;
    cfg.sda_io_num       = (gpio_num_t)TOUCH_I2C_SDA;
    cfg.scl_io_num       = (gpio_num_t)TOUCH_I2C_SCL;
    cfg.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    cfg.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    cfg.master.clk_speed = 400 * 1000;
    if (i2c_param_config(I2C_NUM_0, &cfg) != ESP_OK) {
        Serial.println("ERROR: i2c_param_config failed");
        return -1;
    }
    if (i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0) != ESP_OK) {
        Serial.println("ERROR: i2c_driver_install failed");
        return -2;
    }

    esp_lcd_panel_io_handle_t io_handle = nullptr;
    esp_lcd_panel_io_i2c_config_t io_cfg = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    io_cfg.dev_addr = TOUCH_I2C_ADDR;
    if (esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_NUM_0,
                                  &io_cfg, &io_handle) != ESP_OK) {
        Serial.println("ERROR: esp_lcd_new_panel_io_i2c failed");
        return -3;
    }

    esp_lcd_touch_config_t tp_cfg = {};
    tp_cfg.x_max        = DISPLAY_H_RES;
    tp_cfg.y_max        = DISPLAY_V_RES;
    tp_cfg.rst_gpio_num = (gpio_num_t)TOUCH_RST_PIN;
    tp_cfg.int_gpio_num = (gpio_num_t)TOUCH_INT_PIN;
    tp_cfg.levels.reset     = 0;
    tp_cfg.levels.interrupt = 0;
    if (esp_lcd_touch_new_i2c_gt911(io_handle, &tp_cfg,
                                     &s_touch_handle) != ESP_OK) {
        Serial.println("ERROR: esp_lcd_touch_new_i2c_gt911 failed");
        return -4;
    }

    lv_indev_drv_init(&s_indev_drv);
    s_indev_drv.type    = LV_INDEV_TYPE_POINTER;
    s_indev_drv.read_cb = lvgl_touch_read_cb;
    lv_indev_drv_register(&s_indev_drv);

    return 0;
}
