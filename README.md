# NERIC Triple Inverted Pendulum

**A cart on a linear rail that balances a pendulum — starting with one link, built to scale to three — driven by a dual-core STM32H745 and an ODrive S1 motor controller.**

The classic inverted pendulum, taken seriously as an engineering system: real-time control on bare metal, a clean split between the fast control core and the communications core, system identification performed *on the physical rig* rather than from CAD guesses, and a safety envelope designed so the machine fails gracefully instead of fighting itself.

```
        θ                     A pendulum link balances above a cart.
         \                    The cart slides on a belt-driven rail.
          \  (link 1)         A motor pushes the cart to keep the link upright.
           \                  Do it for one link, then two, then three.
     _______O_______
    |    ▓▓▓▓▓▓▓    |  ← cart
 ═══╧═══════════════╧═══  ← linear rail (ODrive S1 motor + belt)
    home            far
```

---

## What makes this interesting

Balancing one pendulum is a textbook exercise. What this project is actually about:

- **A real dual-core architecture.** The Cortex-M7 does nothing but real-time control and safety at 5 kHz. The Cortex-M4 does nothing but talk to the outside world. They share a single 4 KB lock-free region in SRAM and never step on each other.
- **Identify the plant, don't assume it.** Motor torque constants, cart friction, link inertia — all fitted from experiments on the actual hardware. Gains ship as **zero**; the controller physically refuses to arm until real, measured values are pasted in.
- **Safety as a first-class citizen.** Arm gates *refuse* rather than warn. Aborts *coast* (cut torque, let the cart freewheel) rather than fight the mechanics. Every torque command is clamped in firmware regardless of what the host asked for. Layered rail limits — soft clamp, then power cut, then a physical switch and foam stop — each independent of the last.
- **A staged plan, honestly executed.** One link first, balancing before swing-up, with the whole thing instrumented so a bench session finds *physics* bugs, not integration bugs.

---

## How it works

```
   MATLAB (offline brain + live console)        STM32H745              ODrive S1
   ┌────────────────────────────┐   UDP     ┌─────────────────────-───┐   FDCAN
   │ system-ID   →  fit  → LQR   │  ────►   │  CM4  (FreeRTOS)        │   1 Mbps
   │ design      →  paste gains  │  ◄────   │  Ethernet · UART · CAN  │  ┌────────┐
   │ live dashboard + tuning     │          │  supervision only       │  │ motor  │
   └──────────────┬─────────────┘           ├──── g_shared (4 KB) ────┤  │ + belt │
                  │                         │  lock-free, D2 SRAM     │  └───▲────┘
   a human pastes the fitted gains          ├─────-───────────────────┤      │
   into the firmware headers, once  ──────► │  CM7  (bare metal)      │──────┘
                                            │  5 kHz control loop     │  torque or
   ESP32-S3 touch console ── UART ──►       │  estimator · LQR ·      │  velocity
   (public demo + physical fader)           │  swing-up · sys-ID      │  command
                                            └──────-──────────────────┘
```

**The M7 owns everything that must not miss a deadline.** A 5 kHz timer interrupt reads the pendulum encoders, estimates the state `[cart position, cart velocity, angle, angular velocity]`, runs the controller, and streams a command to the motor over CAN. That same command stream doubles as the motor's watchdog feed — if the M7 ever stops, the motor stops.

**The M4 owns everything that talks.** Five FreeRTOS tasks handle UDP telemetry to MATLAB (100 Hz), the command channel, a high-rate logging stream for system identification, the UART link to the demo console, and a heartbeat-gated hardware watchdog. It runs **zero** control math.

**They meet in one place.** A single 4 KB struct (`g_shared`) lives in a shared SRAM bank, mapped into both cores. Every field has exactly one writer; commands cross as one-shot handshake flags; the logging ring is single-producer/single-consumer and lock-free. No mutexes on the real-time side, ever.

---

## The control approach

The strategy is deliberately conservative and staged, anchored in two papers (see [References](#references)):

1. **Balance before swing-up.** Hold an already-upright link steady first. Swing-up (pumping a hanging link to the top) comes only once balance is boringly reliable.
2. **System identification on the rig.** Free-swing, step, and chirp experiments drive a grey-box fit of the plant; an LQR is then designed against the *fitted* model and verified in a nonlinear simulation with the real firmware's torque clamp before a single gain reaches the hardware.
3. **Two input paths, one flag.** The default commands motor **torque**. A compile-time option commands **cart acceleration** through the ODrive's velocity loop instead — which makes cart mass and friction drop out of the model entirely, so free-swing data alone is enough to fit. The firmware supports both; the choice is a one-line switch.

Gains are **pasted by a human, not streamed** — the running machine has no dependency on the host, and every gain change is a deliberate, auditable step.

---

## Hardware

| Part | Detail |
|---|---|
| **Controller** | NUCLEO-H745ZI-Q — STM32H745 dual-core (Cortex-M7 @ 400 MHz + Cortex-M4 @ 200 MHz) |
| **Motor drive** | ODrive S1, torque/velocity over CANSimple @ 1 Mbps, onboard absolute encoder |
| **Pendulum encoders** | AS5047P 14-bit magnetic angle sensors (one per link), SPI |
| **Host link** | Ethernet, static IP, UDP to a MATLAB dashboard |
| **Demo console** | Waveshare ESP32-S3 4.3" touch display over UART, plus a physical fader for the public to try balancing by hand |
| **Rail safety** | End-stop switches (fail-safe wiring) + soft/hard travel limits + foam end stops |

---

## Repository layout

```
CM7/          Cortex-M7 firmware — the real-time control core
  control_loop.c    5 kHz ISR: encoder read, dispatch, motor command, watchdog feed
  state_est.c       [x, ẋ, θ, θ̇] estimator (finite-difference + Butterworth filter)
  balance_ctrl.c    LQR balance controller with arm gate + abort envelope
  swingup_ctrl.c    energy-shaping swing-up with automatic catch → balance handoff
  sysid.c           on-rig excitation generator (chirp / PRBS / step / free swing)
  as5047p.c         encoder driver with live health diagnostics
  odrive.c          ODrive CANSimple protocol
  rail_limits.c     layered rail travel safety
  pot_ctrl.c        button / fader / mode supervision

CM4/          Cortex-M4 firmware — the communications core
  telemetry.c       100 Hz UDP dashboard feed
  matlab_rx.c       command channel (validated, per-field)
  idlog_tx.c        1 kHz system-ID logging stream
  esp_link.c        UART link to the ESP32 console
  watchdog.c        heartbeat-gated hardware watchdog

Common/       Shared inter-core contract (g_shared) and wire protocols
matlab/       System-ID logger, grey-box fitter, LQR + swing-up designers, live GUI
ESP32_Display/  LVGL kiosk UI for the public demo console
docs/         Architecture, status, and a guided tour of the codebase idioms
```

---

## Status

Stage-1 firmware is **complete and continuously compile-checked on both cores**; the project is in **hardware bring-up**. The control chain — estimator, balance, swing-up, system-ID, logging, and the full MATLAB fit/design toolchain — is written and reviewed. First balance is gated on the ordered bench sequence: reflash → encoder wiring → motor configuration → estimator bring-up → identification runs → fitted gains.

> This is active research hardware. Gains ship at zero and the arm gates refuse by design until identified values exist — a fresh checkout will not, and should not, move a motor.

---

## Building

Firmware is developed in **STM32CubeIDE** (dual-core H745 project). Peripheral/pin configuration is managed through the `.ioc` (CubeMX); application code lives in the `USER CODE` regions and in the standalone driver/controller modules above.

The MATLAB toolchain (`matlab/`) runs the identification experiments, fits the plant, designs the gains, and hosts the live dashboard. The ESP32 console (`ESP32_Display/`) is a PlatformIO/Arduino LVGL project — see [`ESP32_Display/FIRST_USE.md`](ESP32_Display/FIRST_USE.md) for its bring-up route.

Each firmware module is self-contained and documented at the top of its source file; the shared inter-core contract is defined in [`Common/Inc/shared_state.h`](Common/Inc/shared_state.h).

---

## References

- **Eltohamy & Kuo (1997)**, *Real-time stabilisation of a triple link inverted pendulum using single control input*, IEE Proc. Control Theory Appl. 144(5) — friction identification, position-heavy weighting, staged bring-up.
- **Kaheman, Fasel, Bramburger, Strom, Kutz & Brunton (2022)**, *The Experimental Multi-Arm Pendulum on a Cart: A Benchmark System for Chaos, Learning, and Control*, [arXiv:2205.06231](https://arxiv.org/abs/2205.06231) — rates, safety layering, on-rig identification, and the acceleration-input approach.

---

*Built as a NERIC project. The architecture is intentionally boring where it counts — the interesting behavior lives in the physics, not in surprises from the firmware.*
