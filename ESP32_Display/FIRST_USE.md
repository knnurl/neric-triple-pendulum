# ESP32-S3-Touch-LCD-4.3 — First Use & Programming Guide

Board-specific bring-up notes for the **Waveshare ESP32-S3-Touch-LCD-4.3**
(the pendulum demo console). Distilled from the
[Waveshare wiki](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-4.3).

---

## ⚠️ Two facts that change the bring-up approach

1. **Backlight, LCD-reset and touch-reset are NOT plain GPIOs — they hang off
   an on-board CH422G I²C I/O expander.** Our hand-written
   [src/display_drv.cpp](src/display_drv.cpp) never initialises the CH422G, so
   the backlight would never switch on and touch would never leave reset →
   **black screen.** Waveshare's demo creates an `ESP_IOExpander_CH422G` and
   drives TP_RST / LCD_RST high and LCD_BL on *before* the panel works.
2. **`platformio.ini`'s `espressif32@^6.6.0` pulls Arduino-ESP32 core 2.0.x —
   too old.** The Waveshare display stack needs core **≥ 3.0.6** (the FAQ's
   `esp_memory_utils.h` compile error is exactly this).

**Therefore: for first bring-up use the Arduino IDE + Waveshare's own demo**
(which sets up the CH422G + RGB panel + GT911 correctly), get the screen alive,
then drop our UI files onto that known-good skeleton. Don't fight the
from-scratch PlatformIO driver. The PlatformIO route can be revisited later
once the exact pin map is confirmed from the schematic.

---

## Board facts (confirmed from the wiki)

| Item | Value |
|---|---|
| Module | ESP32-S3-**N16R8** → 16 MB flash, **8 MB OPI PSRAM** |
| Panel | 800×480 RGB, controllerless (driven by `ESP32_Display_Panel` lib) |
| Touch | GT911, I²C address **0x5D** |
| I/O expander | **CH422G** — drives LCD_BL, LCD_RST, TP_RST, SD_CS (EXIO4), USB_SEL |
| Flashing | via the Type-C port silk-screened **"UART"** (onboard CH343 auto-download) |
| Recommended Arduino core | Arduino-ESP32 **v3.0.7** |
| I²C addresses in use | 0x5D (GT911) + 0x20–0x27 / 0x30–0x3f (CH422G) — don't reuse |

---

## Step 1 — First power-on (no programming yet)

Plug USB-C into the port labelled **UART** on the silkscreen. The board ships
with a factory demo — the screen should light and show a UI. If it does, the
panel + backlight + CH422G are all healthy. (Optional: reflash the provided
test firmware from the Demo zip's `\Firmware` folder to re-confirm.)

---

## Step 2 — Toolchain (Arduino IDE)

1. Install **Arduino IDE 2.x**. **Critical:** the install / sketchbook path
   must contain **no spaces or non-English characters** — the FAQ lists this
   as the #1 cause of "missing lv_conf.h".
2. Boards Manager → install **esp32 by Espressif Systems v3.0.7**.
3. Download **ESP32-S3-Touch-LCD-4.3-Demo.zip** from the wiki Resources section.
4. Install these libraries **offline, from the demo zip** (so versions match):
   - **ESP32_Display_Panel** (≥ 0.1.4)
   - **ESP32_IO_Expander** (≥ 0.0.4)
   - **lvgl 8.4.0**
   - the demo's **lv_conf.h**

---

## Step 3 — Tools-menu settings (the #1 black-screen cause — match exactly)

```
Board:            ESP32S3 Dev Module   (or "Waveshare ESP32-S3-Touch-LCD-4.3" if listed)
Flash Size:       16MB (128Mb)
PSRAM:            OPI PSRAM            <- must be OPI, not "Disabled"
Flash Mode:       QIO 80MHz
Partition Scheme: Huge APP (3MB)  or  16M Flash (3MB APP/9.9MB FATFS)  <- our UI needs the room
USB CDC On Boot:  Enabled
Upload Mode:      UART0 / Hardware CDC
Upload Speed:     921600
Port:             the CH343 COM port (Device Manager -> Ports)
```

---

## Step 4 — Flash `09_lvgl_Porting`, confirm screen **and** touch

- In `ESP_Panel_Board_Custom.h` enable touch: `#define ESP_OPEN_TOUCH 1`.
- Upload. **If the COM port isn't recognised:** hold **BOOT**, plug in USB,
  release BOOT (download mode). After upload, press **RESET** to run.
- You should see their LVGL widgets demo with working touch.
- **Stop here until this works** — everything below depends on it.

---

## Step 5 — Graft our UI onto the working demo

Our files need nothing but a live LVGL. Copy into the demo sketch folder:
**`ui.cpp`, `ui.h`, `uart_link.cpp`, `uart_link.h`, `esp_link_proto.h`**.
Then in the demo's `setup()` / `app_main`, replace their example-UI call with
ours (keep their init + LVGL mutex):

```cpp
#include "ui.h"
#include "uart_link.h"

// ... after their IO-expander + panel + touch + lvgl init ...
UartLink_Begin(UART_RX_PIN, UART_TX_PIN);   // pins per Step 6
lvgl_port_lock(-1);                          // their mutex helper
Ui_Init();
lvgl_port_unlock();

// in loop() (or a timer):
UartLink_Poll();
Ui_UpdateTelemetry(UartLink_LastTelemetry(), millis());
```

Note: the demo uses LVGL **8.4**; our UI is written against the 8.3 API, which
is source-compatible within the 8.x line. If any widget call differs, it will
be a trivial signature fix.

---

## Step 6 — UART link to the STM32 (one thing to verify)

Use the board's dedicated **UART PH2.0 header** (the one `05_UART_Test`
exercises) to reach the STM32 — it's independent of the CH343 download/log
port. **The header's exact GPIO numbers are in the
[schematic PDF](https://files.waveshare.com/wiki/ESP32-S3-Touch-LCD-4.3/manual/ESP32-S3-Touch-LCD-4.3-Sch.pdf)**
— find its TX/RX and pass them to `UartLink_Begin(rx, tx)`. (The old README's
GPIO15/16 was a guess; the dedicated header is the correct, RGB-safe choice.)

STM32 side is unchanged:

| ESP32 UART header | STM32 pin | Function |
|---|---|---|
| TX | PD6 (USART2_RX) | ESP → STM32 commands |
| RX | PD5 (USART2_TX) | STM32 → ESP telemetry |
| GND | GND | common (mandatory) |

3.3 V both sides, no level shifter, 921600 8N1.

---

## Gotchas worth pre-empting (from the wiki FAQ)

- **No screen after a successful flash** → wrong Tools settings; re-check Flash
  16MB + **PSRAM OPI**, then press RESET.
- **`esp_memory_utils.h: No such file`** → esp32 core < 3.0.2; use 3.0.7.
- **`missing lv_conf.h`** → a non-English / spaced path in the library location.
- **COM port not found** → BOOT-hold download mode (Step 4).
- **RGB "drift" / torn image** → known RGB-LCD effect; see the ESP RGB-LCD FAQ
  (bounce-buffer / PSRAM bandwidth). The `ESP32_Display_Panel` defaults handle
  it; leave PCLK at the demo's value (~21 MHz) unless you have reason to change.
- **First compile is very slow** → normal.

---

## Frame-rate config (if you later tune performance, ESP-IDF path)

The wiki's menuconfig recommendations for the LVGL benchmark (single core
~26 FPS @ PCLK 21 MHz):

```
CONFIG_FREERTOS_HZ=1000
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y
CONFIG_ESPTOOLPY_FLASHMODE_QIO=y
CONFIG_ESPTOOLPY_FLASHFREQ_120M=y        # consistent with PSRAM
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SPEED_120M=y               # + CONFIG_IDF_EXPERIMENTAL_FEATURES=y
CONFIG_SPIRAM_FETCH_INSTRUCTIONS=y
CONFIG_SPIRAM_RODATA=y
CONFIG_ESP32S3_DATA_CACHE_LINE_64B=y
CONFIG_COMPILER_OPTIMIZATION_PERF=y
# LVGL 8.3:
#   LV_MEM_CUSTOM 1, LV_MEMCPY_MEMSET_STD 1, LV_ATTRIBUTE_FAST_MEM IRAM_ATTR
```

---

## Reference links

- Wiki: https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-4.3
- Demo zip: on the wiki, Resources → Demo
- Schematic: https://files.waveshare.com/wiki/ESP32-S3-Touch-LCD-4.3/manual/ESP32-S3-Touch-LCD-4.3-Sch.pdf
- CH422G datasheet: https://files.waveshare.com/wiki/common/CH422DS1_EN.pdf
- GT911 datasheet: https://files.waveshare.com/wiki/common/GT911_EN_Datasheet.pdf
- LVGL 8.x docs: https://docs.lvgl.io/8.3/
- ESP RGB-LCD driver: https://docs.espressif.com/projects/esp-idf/en/release-v5.3/esp32s3/api-reference/peripherals/lcd/rgb_lcd.html
- GT911 driver (esp-bsp): https://github.com/espressif/esp-bsp/tree/master/components/lcd_touch/esp_lcd_touch_gt911
