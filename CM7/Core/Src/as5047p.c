/**
 ******************************************************************************
 * @file    as5047p.c
 * @brief   3x AS5047P multi-drop SPI driver (M7, polled). See as5047p.h for
 *          the protocol summary. Successor to the AS5048A driver (was
 *          ma732.[ch]); the SPI framing is identical, so the logic is
 *          unchanged — only the register semantics (ANGLECOM / ERRFL) and
 *          naming differ.
 *
 *  Timing budget at 6.25 MHz SCK:
 *      Per encoder: CS setup (~0.7 us) + 16-bit shift (2.56 us)
 *                 + CS hold/high time (~0.7 us) + HAL overhead (~1 us) ≈ 5 us
 *      All three sequential: ~15 us — trivial inside the 1 ms read cadence
 *      (control_loop.c decimates encoder reads to 1 kHz).
 ******************************************************************************
 */

#include "as5047p.h"
#include "main.h"
#include "shared_state.h"

extern SPI_HandleTypeDef hspi1;

/* AS5047P command words (parity pre-computed, see as5047p.h frame format).
 * Identical to the AS5048A: READ_ANGLE reads ANGLECOM (0x3FFF, DAEC), and the
 * "clear" command reads ERRFL (0x0001) whose read clears the error flags. */
#define AS5047P_CMD_READ_ANGLE   0xFFFFu   /* addr 0x3FFF (ANGLECOM), R=1, PAR=1 */
#define AS5047P_CMD_READ_ERRFL   0x4001u   /* addr 0x0001 (ERRFL),    R=1, PAR=0 */
#define AS5047P_CMD_READ_DIAAGC  0xFFFCu   /* addr 0x3FFC (DIAAGC),   R=1, PAR=1 */

#define AS5047P_EF_BIT           0x4000u   /* response bit 14: error flag */
#define AS5047P_DATA_MASK        0x3FFFu   /* response bits 13:0 */

/* 2π / 16384 — scale from 14-bit raw to radians. */
#define AS5047P_LSB_RAD   (6.28318530717958647692f / 16384.0f)

/* Per-transaction timeout (HAL ticks ≈ ms). One 16-bit frame at 6.25 MHz is
 * ~2.6 us; 1 ms only trips if the SPI peripheral or clock is broken.
 *
 * ⚠ KNOWN LIMITATION (arch review F12, fix deliberately deferred): this
 * timeout is ILLUSORY in the caller's actual context. The blocking read
 * runs inside the TIM7 ISR at preempt priority 0, and SysTick shares that
 * priority (TICK_INT_PRIORITY = 0) — so uwTick is FROZEN for the whole ISR
 * and HAL_SPI_TransmitReceive's (HAL_GetTick() - tickstart) comparison can
 * never expire. If the SPI peripheral ever wedges mid-frame (clock/config
 * fault — a dead SENSOR cannot cause this; the master clocks regardless),
 * the ISR spins forever: WWDG2 chip reset in Release, permanent hang in
 * DEBUG (ODrive axis watchdog then stops the motor).
 *
 * The real fix is a register-level 16-bit transfer bounded by the DWT
 * cycle counter (already running for overrun timing): ~400k cycles ≈ 1 ms
 * hard bound independent of uwTick. It is NOT being landed now on purpose:
 * the L1 encoder harness is under active hardware bisection (dead MISO),
 * and replacing the proven transfer path mid-hunt would make "wire or
 * code?" undecidable (prime directive #1). Do it only after the L1 diag
 * tile reads green on THIS code. */
#define AS5047P_HAL_TIMEOUT_MS   1u

/* Shadow buffer — last accepted 14-bit angle from each encoder. */
static uint16_t s_raw_angle[3] = {0u, 0u, 0u};

/* Frames rejected for bad parity / EF / dead bus (diagnostic, monotonic). */
static uint32_t s_data_err_cnt = 0u;

/* Per-channel diagnostics, published into g_shared each read cycle so the
 * GUI can split "dead bus" from "EF (magnet)" from "parity (signal)". */
static uint16_t s_err_parity[3] = {0u, 0u, 0u};
static uint16_t s_err_ef[3]     = {0u, 0u, 0u};
static uint16_t s_err_dead[3]   = {0u, 0u, 0u};
static uint16_t s_diaagc[3]     = {0u, 0u, 0u};

/* Consecutive read cycles without an ACCEPTED angle, per channel (see
 * AS5047P_AngleFresh in the header). Starts saturated so "fresh" is only
 * true after a real validated frame; capped so it never wraps back to
 * looking fresh. Bumped on every cycle (incl. SPI timeouts and DIAAGC/ERRFL
 * substitution rounds — anything that didn't deliver a new angle), zeroed
 * only when an angle is accepted. */
static uint16_t s_stale_streak[3] = {0xFFFFu, 0xFFFFu, 0xFFFFu};

static inline void streak_bump(uint8_t i)
{
    if (s_stale_streak[i] != 0xFFFFu) { s_stale_streak[i]++; }
}

/* DIAAGC slow poll: one encoder every DIAG_POLL_TICKS read cycles (1 kHz),
 * rotating — each encoder's field/AGC diagnostics refresh at ~1 Hz. Costs
 * two extra SPI frames (~10 us) on those ticks; the angle stream is
 * unaffected (the second frame re-primes the READ_ANGLE pipeline). */
#define DIAG_POLL_TICKS  333u
static uint32_t s_diag_tick = 0u;
static uint8_t  s_diag_ch   = 0u;

/* CS port/pin lookup tables for compact loop. */
static GPIO_TypeDef * const s_cs_port[3] = {
    AS5047P_CS0_PORT, AS5047P_CS1_PORT, AS5047P_CS2_PORT
};
static const uint16_t s_cs_pin[3] = {
    AS5047P_CS0_PIN, AS5047P_CS1_PIN, AS5047P_CS2_PIN
};

/* ========================================================================
 *  Helpers
 * ===================================================================== */

static inline void cs_high(uint8_t i)
{
    s_cs_port[i]->BSRR = s_cs_pin[i];                    /* set */
}

static inline void cs_low(uint8_t i)
{
    s_cs_port[i]->BSRR = (uint32_t)s_cs_pin[i] << 16u;   /* reset */
}

/* AS5047P needs ≥350 ns CS↓-to-first-SCK setup AND ≥350 ns CS high time
 * between frames. The volatile loop below runs ~300+ cycles at 400 MHz
 * (≥750 ns) — safe margin for both. */
static inline void cs_delay(void)
{
    for (volatile int i = 0; i < 50; i++) { __NOP(); }
}

/* One 16-bit full-duplex frame on encoder i. Returns HAL status; response
 * word written to *rx. */
static HAL_StatusTypeDef xfer(uint8_t i, uint16_t tx, uint16_t *rx)
{
    cs_low(i);
    cs_delay();
    HAL_StatusTypeDef rc = HAL_SPI_TransmitReceive(
        &hspi1, (uint8_t *)&tx, (uint8_t *)rx, 1, AS5047P_HAL_TIMEOUT_MS);
    cs_high(i);
    cs_delay();          /* enforce CS high time before the next frame */
    return rc;
}

/* Even parity across all 16 bits must be 0 for a valid AS5047P frame
 * (bit 15 is even parity over bits 14:0). */
static inline bool parity_ok(uint16_t v)
{
    return (__builtin_parity((unsigned)v) == 0);
}

/* ========================================================================
 *  Public API
 * ===================================================================== */

void AS5047P_Init(void)
{
    /* Configure CS as output, idle HIGH. The .ioc also sets PD14 up as
     * ENC_CS0 (output, set) — this is belt-and-braces so the driver works
     * even before the stale CubeMX code is regenerated. */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    for (uint8_t i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(s_cs_port[i], s_cs_pin[i], GPIO_PIN_SET);
        g.Pin = s_cs_pin[i];
        HAL_GPIO_Init(s_cs_port[i], &g);
    }

    s_raw_angle[0] = s_raw_angle[1] = s_raw_angle[2] = 0u;
    s_data_err_cnt = 0u;
    /* Stale until the first accepted frame — a dead-from-boot sensor must
     * never read as fresh. */
    s_stale_streak[0] = s_stale_streak[1] = s_stale_streak[2] = 0xFFFFu;

    /* Prime the pipelined command/response chain: the response to the first
     * READ_ANGLE arrives only in the SECOND frame. Two throwaway reads per
     * encoder also clear any EF latched from power-up. */
    uint16_t discard;
    for (uint8_t i = 0; i < 3; i++) {
        (void)xfer(i, AS5047P_CMD_READ_ERRFL, &discard);
        (void)xfer(i, AS5047P_CMD_READ_ANGLE, &discard);
    }
}

bool AS5047P_ReadAllBlocking(void)
{
    bool any_data_err = false;

    for (uint8_t i = 0; i < 3; i++) {
        /* Bump first; only an accepted angle below zeroes it. A timeout
         * early-return then still counts against this channel's streak. */
        streak_bump(i);

        uint16_t rx;
        if (xfer(i, AS5047P_CMD_READ_ANGLE, &rx) != HAL_OK) {
            g_shared.m7_fault_flags |= M7_FAULT_SPI_TIMEOUT;
            return false;
        }

        /* Publish the raw response word before any validation — 0x0000 on
         * the GUI diag tile IS the dead-bus diagnosis. */
        g_shared.enc_raw[i] = rx;

        if (!parity_ok(rx)) {
            /* Corrupted frame — keep last-known angle. */
            any_data_err = true;
            s_data_err_cnt++;
            s_err_parity[i]++;
            continue;
        }

        /* Dead-bus guard. An all-zero response means MISO brought nothing
         * back — disconnected/unpowered sensor, wrong MISO pin, or a CS that
         * never reached the chip. 0x0000 has even parity and no EF, so it
         * would otherwise be ACCEPTED as a valid 0 deg angle — masking a dead
         * encoder as "perfectly upright", which the balance arm-gate would
         * trust. Reject it. (0xFFFF is already caught by the EF check below.
         * A genuine raw==0 reading is 1/16384 and transient; holding the last
         * angle for those frames is harmless.) */
        if (rx == 0x0000u) {
            any_data_err = true;
            s_data_err_cnt++;
            s_err_dead[i]++;
            continue;
        }

        if ((rx & AS5047P_EF_BIT) != 0u) {
            /* Sensor flagged an error (framing/parity on our command, or
             * field strength out of range). Read ERRFL to clear it, then
             * re-prime with a READ_ANGLE so the next cycle's response is an
             * angle again (the response to the ERRFL read is the error
             * register, not the angle). Keep last-known angle this cycle. */
            any_data_err = true;
            s_data_err_cnt++;
            s_err_ef[i]++;
            uint16_t discard;
            (void)xfer(i, AS5047P_CMD_READ_ERRFL, &discard);
            (void)xfer(i, AS5047P_CMD_READ_ANGLE, &discard);
            continue;
        }

        s_raw_angle[i]    = rx & AS5047P_DATA_MASK;
        s_stale_streak[i] = 0u;   /* accepted angle — channel is fresh */
    }

    /* Slow DIAAGC poll, one encoder per period, rotating. Frame 1 sends the
     * DIAAGC read (its response is the pipelined angle — discarded); frame 2
     * sends READ_ANGLE and its response IS the DIAAGC content, while also
     * re-priming the pipeline so the next cycle's read returns an angle. */
    if (++s_diag_tick >= DIAG_POLL_TICKS) {
        s_diag_tick = 0u;
        uint8_t i = s_diag_ch;
        s_diag_ch = (uint8_t)((s_diag_ch + 1u) % 3u);
        uint16_t rx;
        if (xfer(i, AS5047P_CMD_READ_DIAAGC, &rx) == HAL_OK &&
            xfer(i, AS5047P_CMD_READ_ANGLE,  &rx) == HAL_OK &&
            parity_ok(rx) && (rx & AS5047P_EF_BIT) == 0u && rx != 0x0000u) {
            s_diaagc[i] = rx & AS5047P_DATA_MASK;
        }
    }

    /* Publish per-channel diagnostics (M7 is the single writer). */
    for (uint8_t i = 0; i < 3; i++) {
        g_shared.enc_diaagc[i]     = s_diaagc[i];
        g_shared.enc_err_parity[i] = s_err_parity[i];
        g_shared.enc_err_ef[i]     = s_err_ef[i];
        g_shared.enc_err_dead[i]   = s_err_dead[i];
    }

    /* Level-triggered, not latched: reflects whether THIS cycle had a bad
     * frame, mirroring M7_FAULT_ODRIVE_HB_LOST's clear-on-recovery pattern.
     * A permanent latch would make one transient power-up blip paint the
     * fault indicator red forever. Historical count is s_data_err_cnt via
     * AS5047P_GetDataErrCount(). Only touched on a full, timeout-free cycle
     * (the early SPI_TIMEOUT return above leaves this flag as-is). */
    if (any_data_err) {
        g_shared.m7_fault_flags |= M7_FAULT_ENC_DATA;
    } else {
        g_shared.m7_fault_flags &= ~(uint32_t)M7_FAULT_ENC_DATA;
    }
    return true;
}

#if ENC_READ_NONBLOCKING
/* ========================================================================
 *  Non-blocking (IRQ-chained) read path — see as5047p.h for the contract.
 *
 *  Round structure: one 16-bit frame per encoder per 5 kHz tick, chained in
 *  the SPI TxRxCplt interrupt (~20 us total, finishes long before the next
 *  tick). The AS5047P pipeline means the response captured in round N is
 *  the reply to the command transmitted in round N-1, so each frame's
 *  meaning is tracked per encoder:
 *    - normally we transmit READ_ANGLE -> next round's rx is an angle;
 *    - after an EF we substitute one READ_ERRFL (clears the error);
 *    - the DIAAGC poll substitutes one READ_DIAAGC per rotation slot.
 *  A substituted command costs that encoder one angle sample the following
 *  round — 200 us of hold, invisible at these rates.
 * ===================================================================== */

typedef enum {
    NB_MEAN_ANGLE  = 0,   /* response is ANGLECOM */
    NB_MEAN_ERRFL,        /* response is ERRFL content — discard */
    NB_MEAN_DIAAGC,       /* response is DIAAGC content */
    NB_MEAN_UNKNOWN       /* pipeline desynced by an abort — discard.
                             (nb_consume's default branch handles it.) */
} nb_meaning_t;

static volatile uint8_t  s_nb_busy = 0u;     /* chain in flight */
static volatile uint8_t  s_nb_done = 0u;     /* round complete, rx valid */
static volatile uint8_t  s_nb_idx  = 0u;     /* frame being shifted */
static uint16_t          s_nb_tx[3];
static volatile uint16_t s_nb_rx[3];
static uint8_t s_nb_rx_meaning[3];       /* meaning of the round in flight */
static uint8_t s_nb_pending_meaning[3] = /* meaning created by last tx */
    { NB_MEAN_ANGLE, NB_MEAN_ANGLE, NB_MEAN_ANGLE };
static uint8_t  s_nb_errfl_pending[3] = {0u, 0u, 0u};
static uint32_t s_nb_stuck_cnt = 0u;

/* DIAAGC rotation at the 5 kHz tick rate: ~1 Hz per encoder. */
#define NB_DIAG_POLL_TICKS  1665u
static uint32_t s_nb_diag_tick = 0u;
static uint8_t  s_nb_diag_ch   = 0u;

/* Validate + apply one completed round. Mirrors the blocking path's
 * validation exactly (raw publish, parity, dead-bus, EF), differing only
 * in EF recovery: instead of two inline extra frames we substitute the
 * next round's command for that encoder. */
static void nb_consume(void)
{
    bool any_data_err = false;

    for (uint8_t i = 0; i < 3; i++) {
        uint16_t rx = s_nb_rx[i];

        /* Streak semantics match the blocking path: any round that does not
         * deliver an accepted angle (rejects, and the deliberate ERRFL /
         * DIAAGC substitution rounds) bumps; only acceptance zeroes. */
        streak_bump(i);

        switch (s_nb_rx_meaning[i]) {
        case NB_MEAN_ANGLE:
            g_shared.enc_raw[i] = rx;
            if (!parity_ok(rx)) {
                any_data_err = true; s_data_err_cnt++; s_err_parity[i]++;
            } else if (rx == 0x0000u) {
                any_data_err = true; s_data_err_cnt++; s_err_dead[i]++;
            } else if ((rx & AS5047P_EF_BIT) != 0u) {
                any_data_err = true; s_data_err_cnt++; s_err_ef[i]++;
                s_nb_errfl_pending[i] = 1u;     /* clear EF next round */
            } else {
                s_raw_angle[i]    = rx & AS5047P_DATA_MASK;
                s_stale_streak[i] = 0u;
            }
            break;

        case NB_MEAN_DIAAGC:
            if (parity_ok(rx) && (rx & AS5047P_EF_BIT) == 0u && rx != 0x0000u) {
                s_diaagc[i] = rx & AS5047P_DATA_MASK;
            }
            break;

        case NB_MEAN_ERRFL:
        default:
            /* ERRFL content — the read itself cleared the flag. */
            break;
        }
    }

    if (any_data_err) {
        g_shared.m7_fault_flags |= M7_FAULT_ENC_DATA;
    } else {
        g_shared.m7_fault_flags &= ~(uint32_t)M7_FAULT_ENC_DATA;
    }

    for (uint8_t i = 0; i < 3; i++) {
        g_shared.enc_diaagc[i]     = s_diaagc[i];
        g_shared.enc_err_parity[i] = s_err_parity[i];
        g_shared.enc_err_ef[i]     = s_err_ef[i];
        g_shared.enc_err_dead[i]   = s_err_dead[i];
    }
}

void AS5047P_TickNonBlocking(void)
{
    /* Stuck-transfer watchdog: the chain takes ~20 us; still busy a full
     * tick (200 us) later means the SPI wedged or the SPI1 NVIC interrupt
     * was never enabled (.ioc). Abort, count, flag, start fresh. */
    if (s_nb_busy) {
        (void)HAL_SPI_Abort(&hspi1);
        /* A completion IRQ may already be pended for the aborted transfer;
         * clear it so the chain callback can't run against reset state and
         * kick a stray frame (arch review F11). */
        HAL_NVIC_ClearPendingIRQ(SPI1_IRQn);
        for (uint8_t i = 0; i < 3; i++) { cs_high(i); }
        s_nb_busy = 0u;
        s_nb_done = 0u;
        /* The abort may have cut the chain mid-flight: a chip that never
         * received this round's frame will answer an OLDER command next
         * round, so the pending meanings no longer match reality — and a
         * DIAAGC reply mis-tagged as an angle passes parity and lands a
         * theta glitch straight into the FD velocity. Discard the next
         * round wholesale; its transmit frames re-sync every chip's
         * one-deep reply pipeline. */
        for (uint8_t i = 0; i < 3; i++) {
            s_nb_pending_meaning[i] = NB_MEAN_UNKNOWN;
        }
        s_nb_stuck_cnt++;
        g_shared.m7_fault_flags |= M7_FAULT_SPI_TIMEOUT;
    }

    /* Consume the completed round (responses to last round's commands). */
    if (s_nb_done) {
        s_nb_done = 0u;
        nb_consume();
    }

    /* Build this round: the rx we are about to clock in answers the
     * PREVIOUS round's tx, so latch those meanings first. */
    bool diag_due = (++s_nb_diag_tick >= NB_DIAG_POLL_TICKS);
    if (diag_due) { s_nb_diag_tick = 0u; }

    for (uint8_t i = 0; i < 3; i++) {
        s_nb_rx_meaning[i] = s_nb_pending_meaning[i];

        if (s_nb_errfl_pending[i]) {
            s_nb_errfl_pending[i] = 0u;
            s_nb_tx[i] = AS5047P_CMD_READ_ERRFL;
            s_nb_pending_meaning[i] = NB_MEAN_ERRFL;
        } else if (diag_due && i == s_nb_diag_ch) {
            s_nb_tx[i] = AS5047P_CMD_READ_DIAAGC;
            s_nb_pending_meaning[i] = NB_MEAN_DIAAGC;
        } else {
            s_nb_tx[i] = AS5047P_CMD_READ_ANGLE;
            s_nb_pending_meaning[i] = NB_MEAN_ANGLE;
        }
    }
    if (diag_due) { s_nb_diag_ch = (uint8_t)((s_nb_diag_ch + 1u) % 3u); }

    /* Kick frame 0; the SPI IRQ chains frames 1 and 2. */
    s_nb_idx  = 0u;
    s_nb_busy = 1u;
    cs_low(0);
    cs_delay();
    if (HAL_SPI_TransmitReceive_IT(&hspi1,
            (uint8_t *)&s_nb_tx[0], (uint8_t *)(uint16_t *)&s_nb_rx[0], 1)
            != HAL_OK) {
        cs_high(0);
        s_nb_busy = 0u;
        /* Nothing was transmitted, but the meaning builder above already
         * rotated pending — the chips' pipelines no longer match it.
         * Discard the next round (same re-sync as the abort path). */
        for (uint8_t i = 0; i < 3; i++) {
            s_nb_pending_meaning[i] = NB_MEAN_UNKNOWN;
        }
        s_nb_stuck_cnt++;
        g_shared.m7_fault_flags |= M7_FAULT_SPI_TIMEOUT;
    }
}

/* SPI1 transfer-complete: step the chain. Runs at NVIC priority 1 — never
 * preempts the TIM7 control ISR, always completes between ticks. */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1) { return; }

    cs_high(s_nb_idx);
    cs_delay();

    uint8_t next = (uint8_t)(s_nb_idx + 1u);
    if (next < 3u) {
        s_nb_idx = next;
        cs_low(next);
        cs_delay();
        if (HAL_SPI_TransmitReceive_IT(&hspi1,
                (uint8_t *)&s_nb_tx[next], (uint8_t *)(uint16_t *)&s_nb_rx[next], 1)
                != HAL_OK) {
            cs_high(next);
            s_nb_busy = 0u;
            /* Chips from `next` onward never got this round's frame —
             * pipeline desynced; discard the next round (see abort path). */
            for (uint8_t i = 0; i < 3; i++) {
                s_nb_pending_meaning[i] = NB_MEAN_UNKNOWN;
            }
            s_nb_stuck_cnt++;
            g_shared.m7_fault_flags |= M7_FAULT_SPI_TIMEOUT;
        }
    } else {
        /* Round complete — publish. The pair must be atomic w.r.t. the
         * TIM7 tick, which outranks this IRQ (arch review F11): with
         * busy=0 visible before done=1, a tick landing between the two
         * stores sees "idle, nothing to consume", overwrites the round's
         * meanings and kicks a new chain — and the resumed callback then
         * flags done against clobbered buffers (a DIAAGC word tagged as an
         * angle passes parity = theta glitch). Masked, the tick sees
         * either busy (harmless abort of an already-complete transfer,
         * counted as one stuck event) or done (normal consume). */
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        s_nb_busy = 0u;
        s_nb_done = 1u;
        __set_PRIMASK(primask);
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1) { return; }
    for (uint8_t i = 0; i < 3; i++) { cs_high(i); }
    s_nb_busy = 0u;
    s_nb_done = 0u;
    /* Unknown how much of the round reached the chips — treat as desynced
     * and discard the next round (see the abort path in TickNonBlocking). */
    for (uint8_t i = 0; i < 3; i++) {
        s_nb_pending_meaning[i] = NB_MEAN_UNKNOWN;
    }
    s_nb_stuck_cnt++;
    g_shared.m7_fault_flags |= M7_FAULT_SPI_TIMEOUT;
}
#endif /* ENC_READ_NONBLOCKING */

void AS5047P_GetAnglesRad(float out[3])
{
    out[0] = (float)s_raw_angle[0] * AS5047P_LSB_RAD;
    out[1] = (float)s_raw_angle[1] * AS5047P_LSB_RAD;
    out[2] = (float)s_raw_angle[2] * AS5047P_LSB_RAD;
}

bool AS5047P_AngleFresh(uint8_t idx)
{
    return (idx < 3u) && (s_stale_streak[idx] < AS5047P_STALE_LIMIT);
}

uint32_t AS5047P_GetDataErrCount(void)
{
    return s_data_err_cnt;
}
