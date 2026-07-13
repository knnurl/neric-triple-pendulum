/**
 ******************************************************************************
 * @file    control_loop.h
 * @brief   M7 bare-metal control loop running from TIM7 update IRQ at 5 kHz.
 *
 * Per the project checklist (Bare-Metal Control Loop section), this is the
 * highest-priority interrupt on M7 (TIM7_IRQn = preempt 0). It must not be
 * preempted by FDCAN, SysTick, or anything else.
 *
 * Per cycle (target 200 us at 5 kHz):
 *   1. read 3x AS5047P encoders (polled SPI, decimated to 1 kHz)
 *   2. update the [x, x_dot, theta, theta_dot] state estimate
 *   3. LQR computation                       (Step 4)
 *   4. clamp + push FDCAN frame to ODrive    (Step 5)
 *   5. increment m7_heartbeat in shared SRAM
 *   6. overrun detection
 *
 * Current scaffold (Step 2): only steps 5 and 6 are wired up. The other
 * steps will be filled in when their checklist sections are addressed.
 ******************************************************************************
 */

#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Enable the cycle-counter-based overrun detector. Disable to remove the
 * branch for absolute-minimum ISR latency in production. */
#define CONTROL_LOOP_OVERRUN_CHECK   1

/* Target loop rate. Must match TIM7 PSC/ARR (199/199 -> 5 kHz). */
#define CONTROL_LOOP_HZ              5000u

/* Initialise DWT cycle counter; safe to call multiple times. Used for
 * overrun detection and (later) ISR self-profiling. */
void ControlLoop_Init(void);

/* Start TIM7 with update interrupt enabled. Call from main() after all
 * peripherals are initialised and shared state is ready. */
void ControlLoop_Start(void);

/* Called from HAL_TIM_PeriodElapsedCallback when htim->Instance == TIM7.
 * Body of the control loop. Lives in the ISR context. */
void ControlLoop_Tick(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_LOOP_H */
