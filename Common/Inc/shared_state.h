/**
 ******************************************************************************
 * @file    shared_state.h
 * @brief   Inter-core shared state placed in D2 SRAM3.
 *
 *   Physical:  D2 SRAM3, 4 KB region
 *   M7 view:   0x30047000 .. 0x30047FFF
 *   M4 view:   0x10047000 .. 0x10047FFF
 *
 * Single instance: g_shared (defined in Common/Src/shared_state.c) placed by
 * the linker in section ".shared_d2" present in both CM7 and CM4 linker scripts.
 *
 * Ownership rules (no field is written by both cores):
 *   - M7 writes:  m7_heartbeat, m7_ready, link_angle_rad[], cart_position,
 *                 cart_velocity, motor_command, m7_fault_flags, controller_mode,
 *                 rail_state, loop_count, idlog_head, idlog_drops, idlog[]
 *   - M4 writes:  setpoint, lqr_gain_delta[], command_mode, command_seq,
 *                 ctrl_src, cmd_set_mode, cmd_clear_errors, cmd_home,
 *                 cmd_zero_upright, cmd_sysid, sysid_* params, idlog_tail,
 *                 m4_fault_flags, hardfault_log fields (D3, separate region)
 *   (cmd_* request flags are the one sanctioned exception: M4 sets to 1,
 *    M7 clears to 0 on consumption — a handshake, not a shared counter.)
 *
 * Synchronisation:
 *   - m7_heartbeat is a single 32-bit write; Cortex-M loads/stores of aligned
 *     32-bit values are atomic. No HSEM needed.
 *   - m7_ready is a one-shot 32-bit flag set by M7 after init. M4 polls it.
 *   - Command/parameter writes on M4 are serialized between M4 writers
 *     (tcpip_thread, ESP32 RX task, watchdog task) with FreeRTOS critical
 *     sections. Cross-core consistency comes from write ordering: payload
 *     fields are stored BEFORE command_seq is bumped, and M7 only consumes
 *     them after observing a command_seq change. M7 never takes a lock.
 ******************************************************************************
 */

#ifndef SHARED_STATE_H
#define SHARED_STATE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- HSEM ID allocation (shared between M7 and M4) ---------------------- */
#define HSEM_ID_BOOT_SYNC    0u   /* M7 takes/releases to free M4 from boot spin */
#define HSEM_ID_M4_TO_M7     1u   /* reserved; was used for M4 command writes.
                                     M7 never took it, so it only guarded M4
                                     against itself — replaced by FreeRTOS
                                     critical sections (spin-take between tasks
                                     on one core risks livelock). */
#define HSEM_ID_HEARTBEAT    2u   /* reserved; heartbeat is atomic so HSEM optional */

/* ---- Magic / version ---------------------------------------------------- */
#define SHARED_STATE_MAGIC   0x4E455249u   /* 'NERI' */
#define SHARED_STATE_VERSION 1u

/* ---- High-rate ID/balance log ring size (see idlog[] below) ------------- *
 * 128 samples at 1 kHz = 128 ms of buffering; M4 drains every 10 ms.        */
#define IDLOG_RING_LEN       128u

/* ---- Controller mode ----------------------------------------------------
 *  Existing LQR modes (set by M4 or LQR application in Step 4):
 *    IDLE        — no motor output; control_loop.c sends nothing.
 *    SWINGUP     — LQR swing-up (Step 4).
 *    BALANCE     — LQR balance (Step 4).
 *    SAFE_STOP   — emergency stop (no TX, lets ODrive idle).
 *
 *  Pot-test modes (set by M7's pot_ctrl, Step "button/pot"):
 *    POT_POSITION — control_loop.c sends Set_Input_Pos(motor_command, 0, 0)
 *                   motor_command interpreted as target position (rev).
 *    POT_VELOCITY — control_loop.c sends Set_Input_Vel(motor_command, 0)
 *                   motor_command interpreted as target velocity (rev/s).
 *
 *  System-ID mode (Step "LQR stage 1"):
 *    SYSID       — sysid.c generates a bounded torque excitation (chirp /
 *                  PRBS / step) or logs a free swing; control_loop.c streams
 *                  Set_Input_Torque and pushes 1 kHz samples into idlog[].
 * ----------------------------------------------------------------------- */
#define CTRL_MODE_IDLE          0u
#define CTRL_MODE_SWINGUP       1u
#define CTRL_MODE_BALANCE       2u
#define CTRL_MODE_SAFE_STOP     3u
#define CTRL_MODE_POT_POSITION  4u
#define CTRL_MODE_POT_VELOCITY  5u
#define CTRL_MODE_SYSID         6u

/* ---- M7 fault flag bits ------------------------------------------------- */
#define M7_FAULT_LOOP_OVERRUN   (1u << 0)
#define M7_FAULT_SPI_TIMEOUT    (1u << 1)
#define M7_FAULT_CAN_TX_FAIL    (1u << 2)
#define M7_FAULT_ODRIVE_HB_LOST (1u << 3)
#define M7_FAULT_LIMIT_HIT      (1u << 4)
#define M7_FAULT_ENC_DATA       (1u << 5)  /* AS5047P parity fail / EF / dead bus */
#define M7_FAULT_ODRIVE_AXIS    (1u << 6)  /* axis fault / left CLOSED_LOOP, or the
                                              encoder-estimate stream went stale,
                                              during a torque run (run auto-aborted;
                                              arch review 2026-07-11, batch B) */
#define M7_FAULT_INIT_FAIL      (1u << 7)  /* M7 hit Error_Handler (init/HAL failure)
                                              and is dead in its blink loop; stamped
                                              by Error_Handler itself, which also
                                              releases the CM4 boot spin so telemetry
                                              can report the corpse (arch review F5) */

/* ---- M4 fault flag bits ------------------------------------------------- */
#define M4_FAULT_WDT_STALE_HB   (1u << 0)
#define M4_FAULT_UDP_RX_BADLEN  (1u << 1)
#define M4_FAULT_UART_BAD_CRC   (1u << 2)
#define M4_FAULT_HSEM_TIMEOUT   (1u << 3)
#define M4_FAULT_UDP_TX_FAIL    (1u << 4)   /* pbuf_alloc or udp_sendto failed */

/* ---- Shared structure --------------------------------------------------- *
 * All fields volatile: both cores access this RAM concurrently, so every
 * access must hit memory, never a register copy.
 *
 * Cache/coherency status (updated 2026-07-11, batch D): CM7's MPU_Config()
 * (CM7 main.c, runs before HAL_Init) now marks this 4 KB region Normal
 * NON-CACHEABLE + execute-never, so g_shared stays coherent even if the
 * CM7 caches are enabled later. Both caches are currently still DISABLED:
 * enabling the I-cache is data-safe whenever wanted (own change + bench
 * pass); enabling the D-cache additionally needs a DMA/buffer audit for
 * everything OUTSIDE this region first. M4 has no D-cache — nothing needed
 * on that side.
 */
typedef struct {

    /* --- Identity / version (M7 writes once at boot) --------------------- */
    volatile uint32_t magic;            /* SHARED_STATE_MAGIC */
    volatile uint32_t version;          /* SHARED_STATE_VERSION */

    /* --- Boot handshake (M7 writes; M4 polls) ---------------------------- */
    volatile uint32_t m7_ready;         /* 0 -> 1 when M7 shared state init complete */

    /* --- M7 -> M4 telemetry (M7 writes from TIM7 ISR) -------------------- */
    volatile uint32_t m7_heartbeat;     /* increments every control loop cycle */
    volatile uint32_t loop_count;       /* monotonic loop counter */
    volatile uint32_t loop_overrun_cnt; /* total loop-overrun events */
    volatile uint32_t m7_fault_flags;   /* M7_FAULT_* bitmask */
    volatile uint8_t  controller_mode;  /* active mode (M7 writes after accepting M4 cmd) */
    volatile uint8_t  rail_state;       /* RAIL_STATE_* (rail_limits.h): unhomed/ok/
                                           soft-clamp/hard-trip/latched */
    volatile uint8_t  _pad_a[2];

    volatile float    link_angle_rad[3];/* 3x AS5047P pendulum link angles */
    volatile float    cart_position;    /* metres (or rev, application defined) */
    volatile float    cart_velocity;    /* m/s or rev/s */
    volatile float    motor_command;    /* last control output: torque Nm (default),
                                           cart accel rev/s^2 (CTRL_INPUT_ACCEL=1 —
                                           M7 build option, see control_input.h),
                                           or pos/vel target in POT modes */
    volatile float    odrive_motor_pos; /* shadow of ODrive encoder estimate */

    /* --- M4 -> M7 commands (M4 writes under HSEM_ID_M4_TO_M7) ------------ */
    volatile uint8_t  command_mode;       /* requested CTRL_MODE_* — sticky PAYLOAD byte.
                                             The request EVENT is cmd_set_mode below (write
                                             mode first, then cmd_set_mode=1, then
                                             command_seq++). M7 must never act on this byte
                                             without consuming cmd_set_mode (arch review F10:
                                             acting on the sticky byte replayed the last mode
                                             request on every unrelated command). */
    volatile uint8_t  cmd_zero_upright;   /* M4 sets to 1 to capture the pendulum upright zero
                                             (operator holds link 1 vertical); M7 clears on entry.
                                             (Byte was cmd_index_search — removed with the switch
                                             to the ODrive S1 onboard absolute encoder.) */
    volatile uint8_t  ctrl_src;           /* 0 = POT owns motor_command in POT_POSITION/POT_VELOCITY
                                              runtime control, 1 = REMOTE (MATLAB GUI switch) owns it.
                                              Does not affect mode/command acceptance, only which
                                              source's continuous setpoint pot_ctrl.c applies per tick. */
    volatile uint8_t  cmd_clear_errors;   /* M4 sets to 1 to request M7 send Odrive_ClearErrors() over
                                              CAN and clear its own m7_fault_flags; M7 clears on entry.
                                              M4 cannot do either directly: the ODrive only talks to M7
                                              (FDCAN), and m7_fault_flags is M7-owned. */
    volatile uint8_t  cmd_home;           /* M4 sets to 1 to request rail homing; M7 clears on entry.
                                              Wired: creeps onto the home switch. Unwired: zeroes at
                                              the current cart position (see rail_limits.h). */
    volatile uint8_t  cmd_sysid;          /* M4 sets to 1 to request a system-ID run using the
                                              sysid_* params below (write params FIRST, then this,
                                              then bump command_seq); M7 clears on entry. */
    volatile uint8_t  cmd_set_mode;       /* M4 sets to 1 when command_mode above carries a NEW
                                              mode request; M7 (pot_ctrl) clears on consumption.
                                              Added 2026-07-11 (arch review F10) in an old _pad_b
                                              byte — no offsets move. */
    volatile uint8_t  _pad_b[1];
    volatile uint32_t command_seq;        /* bumps on each new command set */
    volatile float    setpoint;           /* target angle/position for active mode */
    volatile float    lqr_gain_delta[6];  /* runtime gain offsets (bounds-checked by M4) */

    /* --- System-ID run parameters (M4 writes before setting cmd_sysid) --- */
    volatile uint8_t  sysid_type;         /* 0=free-swing 1=chirp 2=PRBS 3=step (sysid.h) */
    volatile uint8_t  _pad_c[3];
    volatile float    sysid_amp_nm;       /* torque amplitude, Nm (M7 re-clamps) */
    volatile float    sysid_dur_s;        /* run duration, s */
    volatile float    sysid_f0_hz;        /* chirp start frequency */
    volatile float    sysid_f1_hz;        /* chirp end frequency / PRBS bandwidth */

    /* --- M4 status (M4 writes) ------------------------------------------- */
    volatile uint32_t m4_heartbeat;     /* M4 telemetry heartbeat (for ESP32/MATLAB) */
    volatile uint32_t m4_fault_flags;   /* M4_FAULT_* bitmask */

    /* --- High-rate ID/balance log ring (M7 produces, M4 drains) ---------- *
     * Single-producer/single-consumer lock-free ring. M7 pushes one sample
     * per 1 kHz estimator tick while controller_mode is SYSID or BALANCE:
     * write sample at idlog[head % LEN], __DMB(), then head++ (publish).
     * M4 consumes from tail while tail != head, batches into UDP packets on
     * port 5007 (idlog_tx.c). Ring full -> M7 drops the sample and counts it
     * in idlog_drops (M4-visible; stale data is worse than a gap).          */
    volatile uint32_t idlog_head;       /* M7-owned write index (free-running) */
    volatile uint32_t idlog_tail;       /* M4-owned read index (free-running) */
    volatile uint32_t idlog_drops;      /* M7: samples dropped because ring full */
    struct {
        volatile uint32_t tick;         /* M7 loop_count at sample time (5 kHz ticks) */
        volatile float    theta;        /* rad, upright-zeroed link-1 angle */
        volatile float    theta_dot;    /* rad/s, filtered (state_est.c) */
        volatile float    x;            /* cart position, rev (rail frame if homed) */
        volatile float    x_dot;        /* cart velocity, rev/s */
        volatile float    u;            /* commanded torque this tick, Nm */
    } idlog[IDLOG_RING_LEN];            /* 24 B x 128 = 3072 B */

    /* --- Encoder diagnostics (M7 writes from as5047p.c, M4 mirrors into
     *     the MATLAB telemetry packet). Occupies exactly the 32 bytes of
     *     the old _reserved[8] block, so no other offset moves. ---------- */
    volatile uint16_t enc_raw[3];        /* last raw 16-bit SPI response word
                                            (0x0000 = dead bus / no MISO) */
    volatile uint16_t enc_diaagc[3];     /* last AS5047P DIAAGC read:
                                            [11]=MAGL (field too low)
                                            [10]=MAGH (field too high)
                                            [9]=COF  [8]=LF  [7:0]=AGC value */
    volatile uint16_t enc_err_parity[3]; /* frames rejected: bad parity  (wraps) */
    volatile uint16_t enc_err_ef[3];     /* frames rejected: EF flag set (wraps) */
    volatile uint16_t enc_err_dead[3];   /* frames rejected: all-zero    (wraps) */
    volatile uint16_t _pad_d;

} SharedState_t;

/* The whole struct must fit the 4 KB D2 SRAM3 window mapped by both linker
 * scripts. If this fires, shrink IDLOG_RING_LEN. */
#if defined(__GNUC__)
_Static_assert(sizeof(SharedState_t) <= 4096, "SharedState_t exceeds 4 KB shared region");
#endif

/* Single instance lives in section .shared_d2 (see Common/Src/shared_state.c).
 * Accessed by both M7 and M4; the address resolves to the same physical SRAM
 * because the linker places the section at the same physical bank from each
 * core's view (M7 sees 0x30047000, M4 sees 0x10047000). */
extern SharedState_t g_shared;

/* Helper: M7 must call this once at boot before HSEM_ID_BOOT_SYNC release.
 * Zero-initialises the entire region, then stamps magic/version, then sets
 * m7_ready = 1 as the last write. */
void SharedState_M7_Init(void);

/* Helper: M4 calls this after the HSEM_ID_0 release wait, to spin on the
 * m7_ready flag with a sane timeout. Returns 1 if ready, 0 on timeout. */
int  SharedState_M4_WaitReady(uint32_t timeout_ticks);

#ifdef __cplusplus
}
#endif

#endif /* SHARED_STATE_H */
