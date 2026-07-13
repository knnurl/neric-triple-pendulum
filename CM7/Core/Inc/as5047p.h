/**
 ******************************************************************************
 * @file    as5047p.h
 * @brief   3x AS5047P 14-bit magnetic angle encoder driver (M7, polled SPI).
 *
 *  The AS5047P shares the AS5048A's SPI framing exactly, so this driver is a
 *  drop-in successor to the previous AS5048A driver (was ma732.[ch]). The
 *  differences that matter here:
 *    - Angle is read from ANGLECOM (0x3FFF), which applies the AS5047P's
 *      Dynamic Angle Error Compensation (DAEC) — near-zero latency at speed,
 *      the reason for the swap (less phase lag into the theta-dot estimate on
 *      the fast upper links). ANGLEUNC (0x3FFE) is the uncompensated angle if
 *      DAEC extrapolation is ever unwanted.
 *    - The error register is ERRFL (0x0001); reading it clears the flags.
 *  Resolution (14-bit, 16384 CPR) and the 2*pi/16384 rad scale are identical
 *  to the AS5048A, so nothing downstream (state_est, filters) changes.
 *
 *  All three encoders share SPI1 (SCK PA5, MISO PA6) with one CS line each.
 *  Reads are sequential: assert CS_n, one polled 16-bit transaction, deassert.
 *  The AS5047P tri-states MISO while its CS is high, so multi-drop works.
 *
 *  SPI requirements: Mode 1 (CPOL=0, CPHA=2nd edge), SCK <= 10 MHz (6.25 MHz
 *  configured), 16-bit frames MSB first.
 *
 *  Frame format (identical to AS5048A):
 *      Command:  [15]=parity (even) [14]=R/W (1=read) [13:0]=address
 *                READ ANGLECOM = 0xFFFF (addr 0x3FFF)
 *      Response: [15]=parity (even) [14]=EF (error flag) [13:0]=data
 *
 *  The AS5047P pipelines responses: the reply to command N arrives during
 *  frame N+1. Since we send READ_ANGLE every frame, each read returns the
 *  angle sampled one read-cycle earlier (1 ms at the 1 kHz decimated rate).
 *  Init primes the pipeline and discards the first (stale) response.
 ******************************************************************************
 */

#ifndef AS5047P_H
#define AS5047P_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Read mechanism ----------------------------------------------------
 * 0 = blocking polled reads at the 1 kHz decimated tick (proven path).
 * 1 = IRQ-chained non-blocking reads EVERY 5 kHz control tick: the control
 *     ISR kicks a 3-frame chain and consumes the PREVIOUS tick's results
 *     (data is one tick = 200 us old, refreshed 5x faster than blocking).
 *     Zero ISR stall; a stuck chain is aborted at the next tick and counted
 *     as SPI_TIMEOUT.
 *
 *     REQUIRES (.ioc, user-applied): SPI1 global interrupt enabled in NVIC,
 *     preempt priority 1 (one below TIM7). Without it every round hits the
 *     stuck-watchdog: angles freeze and M7_FAULT_SPI_TIMEOUT stays set —
 *     safe, and obvious in the GUI.
 *
 *     Flip to 1 only after the blocking path has validated the harness on
 *     the bench (known-good wire before new code). ------------------------ */
#ifndef ENC_READ_NONBLOCKING
#define ENC_READ_NONBLOCKING   0
#endif

/* CS pin mapping — change here if .ioc pins are reassigned.
 * Encoder A (link 1) is wired on PD14 (.ioc label ENC_CS0). Encoders B and C
 * still alias to that pin as placeholders until links 2/3 are wired — set
 * their real CS pins here when you do (MATLAB then shows near-identical
 * L1/L2/L3 only while they alias, a useful "not wired yet" sanity check). */
#define AS5047P_CS0_PORT   GPIOD
#define AS5047P_CS0_PIN    GPIO_PIN_14
#define AS5047P_CS1_PORT   GPIOD          /* placeholder — set when link 2 wired */
#define AS5047P_CS1_PIN    GPIO_PIN_14
#define AS5047P_CS2_PORT   GPIOD          /* placeholder — set when link 3 wired */
#define AS5047P_CS2_PIN    GPIO_PIN_14

/* Initialise: configure CS GPIO (idle high), zero shadow values, then prime
 * the AS5047P command/response pipeline and discard the stale first frame.
 * Call from USER CODE 2 in main, after MX_SPI1_Init. */
void AS5047P_Init(void);

/* Sequential polled read of all 3 encoders (~15 us total at 6.25 MHz).
 * Returns true if all SPI transactions completed; false on HAL error/timeout
 * (sets M7_FAULT_SPI_TIMEOUT, keeps last-known raw values).
 * Frames failing parity, flagged EF, or reading all-zero (dead bus) keep the
 * last-known value and set M7_FAULT_ENC_DATA; EF additionally triggers an
 * ERRFL read so the next frame is clean. */
bool AS5047P_ReadAllBlocking(void);

#if ENC_READ_NONBLOCKING
/* Non-blocking tick: consume the previous round's responses (validate,
 * update angles/diagnostics) and kick this round's IRQ-driven 3-frame
 * chain. Call from the control ISR EVERY 5 kHz tick. Never blocks. */
void AS5047P_TickNonBlocking(void);
#endif

/* Convert the most recent raw 14-bit values into radians [0, 2pi). */
void AS5047P_GetAnglesRad(float out[3]);

/* ---- Angle freshness (arch review 2026-07-11, F1a) ----------------------
 * A rejected frame (parity / EF / dead bus / SPI timeout) keeps the
 * last-known angle — correct for a transient, but a *persistently* dead
 * sensor then freezes theta at a value the theta abort envelope cannot see.
 * The driver counts consecutive read cycles without an ACCEPTED angle per
 * channel; controllers abort their run when link 1 goes stale.
 *
 * AS5047P_STALE_LIMIT is ~10 ms of consecutive misses on either path:
 * blocking reads run at the 1 kHz decimated tick, the NB consume runs every
 * 5 kHz tick. Channels start stale at boot (fresh only after a first
 * accepted frame). */
#if ENC_READ_NONBLOCKING
#define AS5047P_STALE_LIMIT   50u
#else
#define AS5047P_STALE_LIMIT   10u
#endif

/* True if channel idx accepted a validated angle within the last
 * AS5047P_STALE_LIMIT read cycles. ISR-context safe. */
bool AS5047P_AngleFresh(uint8_t idx);

/* Diagnostics: count of frames rejected for bad parity / error flag / dead bus. */
uint32_t AS5047P_GetDataErrCount(void);

#ifdef __cplusplus
}
#endif

#endif /* AS5047P_H */
