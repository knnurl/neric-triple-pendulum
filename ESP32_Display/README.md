# ESP32-S3 Pendulum Console

Companion firmware for the H755 pendulum project. Runs on a Waveshare
**ESP32-S3-Touch-LCD-4.3** (800×480 RGB display, GT911 touch). It is the
public-facing **demo console**: a professional dark-theme touch UI plus a
physical fader for the "try to balance it yourself" crowd demo.

## The UI

A persistent status bar (active mode pill, link + rail status) sits above
three pages, switched by a bottom nav bar:

- **LIVE** — a real-time cart + pendulum visualisation (the cart slides along
  a rail, the pole swings at the live measured angle) beside four KPI tiles
  (tilt from upright, cart position, cart velocity, loop health).
- **CHALLENGE** — the crowd demo. A large stopwatch scores how long the pole
  is kept within 30° of upright, with a best-time readout, a live tilt needle,
  and a second pendulum visualisation. Buttons: **TAKE OVER** (hand control to
  the fader), **LET COMPUTER** (hand control to the LQR — it holds the pole
  effortlessly, which is the whole point), and **STOP**. A veil blocks the page
  until the rail is homed.
- **EXPERT** — every mode (Idle / Manual-Pos / Manual-Vel / Swing-up / Balance
  / E-Stop), a setpoint slider, live LQR gain nudging (pick K0–K5, ±0.05), and
  Home / Zero-θ / Clear-Errors, plus decoded fault flags and loop stats.

All screens are styled explicitly in [src/ui.cpp](src/ui.cpp), so the look is
stable across LVGL point releases. Fonts and the LINE/ARC widgets it needs are
enabled in [include/lv_conf.h](include/lv_conf.h).

## Wiring to STM32

| ESP32 pin | STM32 pin | Function |
|---|---|---|
| **GPIO15** (TX) | **PD6** (USART2_RX) | ESP→STM32 commands |
| **GPIO16** (RX) | **PD5** (USART2_TX) | STM32→ESP telemetry |
| GND | GND | Common (mandatory) |

**Both sides 3.3 V — no level shifter.** 921 600 baud, 8-N-1.

**Do not** use GPIO 17/18 — those are LCD blue-data pins on this board.

## The physical fader (manual-balance demo)

**Recommended wiring: the fader is a slide potentiometer wired to the STM32's
existing pot input (PA3 / ADC1), _not_ to the ESP32.** Rationale:

- It is electrically identical to the rotary pot [`pot_ctrl.c`](../CM7/Core/Src/pot_ctrl.c)
  already reads — wiper to PA3, ends to 3V3/GND. **Zero new firmware:**
  `pot_ctrl.c` already maps it to an absolute rail position through the
  `rail_limits` soft/hard limits, so the fader inherits the entire safety
  envelope for free.
- **Lowest latency** — the fader sits inside the 5 kHz M7 loop, not behind the
  50 Hz UART link. Manual balancing is hard enough without added lag.

With that wiring the **ESP32 does not read the fader at all** — it shows the
resulting cart/pole motion and scores the challenge. The `CHALLENGE` page's
**TAKE OVER** button just sends `POT_POSITION`; the person then slides the
fader and the cart follows (fader-left → cart-left, the intuitive "get under
the falling pole" mapping). **LET COMPUTER** sends `BALANCE`.

> Demo flow: Home the rail → hold the pole up and press **Zero θ** → **TAKE
> OVER** and let a visitor try (centre the fader first so the position-match
> gate arms) → when they give up, **LET COMPUTER** and watch the LQR hold it.

*Alternative (not recommended): read the fader on a spare ESP32 ADC pin and
send `SET_SETPOINT` frames.* This keeps all demo hardware on the console but
adds link latency to the manual path. If you go this way, add an ADC read in
`main.cpp` and call `UartLink_SendSetpoint()`; the UI is unchanged. To make the
manual feel like a velocity joystick instead of position tracking, send
`POT_VELOCITY` on TAKE OVER — a one-line change in `on_action()`.

## Build (PlatformIO)

```
cd ESP32_Display
pio run
pio run -t upload
pio device monitor -b 115200
```

First build installs LVGL 8.3 and `esp_lcd_touch_gt911` via `platformio.ini`.

## Files

```
ESP32_Display/
├── platformio.ini       # build config + library deps
├── include/
│   ├── esp_link_proto.h # MUST stay byte-identical to STM32's Common/Inc/esp_link_proto.h
│   └── lv_conf.h        # LVGL config
└── src/
    ├── main.cpp         # boot, LVGL tick, calls into display_drv + UI
    ├── display_drv.cpp  # esp_lcd_panel_rgb + esp_lcd_touch_gt911 glue
    ├── display_drv.h    # LCD/touch pin map (verify against your board rev!)
    ├── uart_link.cpp    # UART parser + TX helpers (mirror of M4's esp_link.c)
    ├── uart_link.h
    ├── ui.cpp           # professional 3-page LVGL kiosk UI (LIVE/CHALLENGE/EXPERT)
    └── ui.h
```

## Protocol note (this revision)

`esp_link_proto.h` gained a `rail_state` byte in `EspTelemetry_t` (reusing the
old pad byte — still 56 bytes, the static assert holds) and renamed the dead
`INDEX_SEARCH` message (0x24) to `ZERO_UPRIGHT` (the onboard absolute encoder
removed index search). The M4 side (`CM4/Core/Src/esp_link.c`) handles
`ZERO_UPRIGHT` and fills `rail_state`. **Reflash both the STM32 M4 and the
ESP32** after pulling this — the wire layout changed.

## What might need tweaking on your board

The Waveshare board has multiple revisions; LCD pin numbers can shift.
Verify the pins defined at the top of [src/display_drv.h](src/display_drv.h)
against your **specific** board's schematic. If the display shows
scrambled colours or doesn't init, the RGB data pin mapping is wrong.

Timings (porch / pulse-width) are typical defaults. If the image looks
shifted or torn, look up the panel's datasheet timings on Waveshare's
wiki and update `panel_config.timings.*` in `Display_Init()`.

The GT911 touch I2C address can be either **0x14** or **0x5D** depending
on the state of `INT` and `RST` pins at boot. If touch doesn't respond,
flip `TOUCH_I2C_ADDR` in `display_drv.h`.

Backlight pin: on some board revs the BL is direct-drive on a GPIO, on
others it's behind a PCA9554 I/O expander. If the screen is black but
the boot serial says "Display init OK", check `LCD_PIN_BL` vs your rev.

## Versioning

If you change `esp_link_proto.h` here, **also** update
`H7_M4_ETH_ODrive/Common/Inc/esp_link_proto.h` and rebuild M4. The
`_Static_assert` size checks on both sides catch most accidental drift.
