/**
 ******************************************************************************
 * @file    idlog_tx.h
 * @brief   High-rate ID/balance log streamer: shared ring -> UDP 5007 (M4).
 *
 *  M7 pushes one 24 B sample per 1 kHz estimator tick into g_shared.idlog[]
 *  while controller_mode is SYSID or BALANCE. This task drains the ring
 *  every IDLOG_TX_PERIOD_MS and ships the samples to the MATLAB host
 *  (same IP as telemetry) on UDP port 5007, batched:
 *
 *      ┌────────┬────────┬───────┬───────┬──────────────────────────┐
 *      │ magic  │ seq    │ n     │ drops │ n x sample (24 B each)   │
 *      │ u32    │ u32    │ u16   │ u16   │ tick u32, theta f32,     │
 *      │ 'IDLG' │        │       │       │ theta_dot f32, x f32,    │
 *      │        │        │       │       │ x_dot f32, u f32         │
 *      └────────┴────────┴───────┴───────┴──────────────────────────┘
 *
 *  n <= IDLOG_TX_MAX_SAMPLES (packet stays under one MTU). `drops` is the
 *  low 16 bits of g_shared.idlog_drops so MATLAB can see ring overruns.
 *  Little-endian throughout; captured by matlab/id_logger.m.
 *
 *  When the ring is idle (no SYSID/BALANCE run) nothing is sent — the wire
 *  is silent outside runs.
 ******************************************************************************
 */

#ifndef IDLOG_TX_H
#define IDLOG_TX_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IDLOG_TX_PORT           5007u
#define IDLOG_TX_PERIOD_MS      10u     /* drain cadence: 10 samples/period at 1 kHz */
#define IDLOG_TX_MAX_SAMPLES    40u     /* per packet: 12 + 40*24 = 972 B < MTU */

#define IDLOG_TX_MAGIC          0x474C4449u   /* 'IDLG' little-endian */

#pragma pack(push, 1)
typedef struct {
    uint32_t magic;         /* IDLOG_TX_MAGIC */
    uint32_t seq;           /* packet sequence, monotonic */
    uint16_t n_samples;     /* samples in this packet */
    uint16_t drops;         /* low 16 bits of g_shared.idlog_drops */
} IdLogPktHeader_t;

typedef struct {
    uint32_t tick;          /* M7 loop_count (5 kHz ticks) at sample time */
    float    theta;         /* rad, upright-zeroed */
    float    theta_dot;     /* rad/s */
    float    x;             /* rev (rail frame if homed) */
    float    x_dot;         /* rev/s */
    float    u;             /* Nm */
} IdLogWireSample_t;
#pragma pack(pop)

#if defined(__GNUC__)
_Static_assert(sizeof(IdLogPktHeader_t)   == 12, "idlog header size");
_Static_assert(sizeof(IdLogWireSample_t)  == 24, "idlog sample size");
#endif

/* Task entry (osThreadNew from CM4 main.c). Waits for LWIP, then drains. */
void StartIdLogTxTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* IDLOG_TX_H */
