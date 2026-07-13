/**
 ******************************************************************************
 * @file    telemetry.h
 * @brief   UDP telemetry M4 -> MATLAB.
 *
 *  M4 sends a fixed-layout, packed, little-endian UDP packet at 100 Hz from
 *  its FreeRTOS telemetry task. Payload is built from g_shared (shared D2
 *  SRAM written by M7's control loop). Three "dummy" test variables are
 *  included so the MATLAB side can verify the link without needing M7 to
 *  produce real state yet.
 *
 *  Networking (matches the existing CubeMX/.ioc settings — DO NOT change):
 *    STM32  IP : 192.168.1.10  (fixed; DHCP off)
 *    Mask      : 255.255.255.0
 *    MATLAB IP : 192.168.1.5   (change MATLAB_REMOTE_IP_* below if different)
 *    UDP port  : 5006          (MATLAB listens here)
 *
 *  Endianness: little-endian. STM32H7 (Cortex-M4/M7) is little-endian and we
 *  do not byte-swap. MATLAB's `typecast` on Windows/x86_64 is also LE, so
 *  raw struct reinterpretation works.
 *
 *  Sequence number: monotonic from 0. MATLAB can detect dropouts by gap >1.
 *
 *  Future Step 8 work will add the reverse path (MATLAB -> M4 parameter
 *  injection on a separate listen port).
 ******************************************************************************
 *  MATLAB receive snippet (copy-paste for verification):
 *
 *    u = udpport("datagram","LocalPort",5006,"EnablePortSharing",true);
 *    while true
 *        if u.NumDatagramsAvailable > 0
 *            dg = read(u, u.NumDatagramsAvailable, "uint8");
 *            for k = 1:numel(dg)
 *                b = dg(k).Data;
 *                magic = typecast(b(1:4),'uint32');
 *                if magic ~= hex2dec('54454C45'); continue; end       % 'TELE'
 *                seq          = typecast(b(5:8),  'uint32');
 *                ts_ms        = typecast(b(9:12), 'uint32');
 *                link_angle   = typecast(b(13:24),'single');           % 3 floats
 *                cart_pos     = typecast(b(25:28),'single');
 *                cart_vel     = typecast(b(29:32),'single');
 *                motor_cmd    = typecast(b(33:36),'single');
 *                odrv_pos     = typecast(b(37:40),'single');
 *                m7_hb        = typecast(b(41:44),'uint32');
 *                loop_cnt     = typecast(b(45:48),'uint32');
 *                overrun_cnt  = typecast(b(49:52),'uint32');
 *                m7_faults    = typecast(b(53:56),'uint32');
 *                m4_faults    = typecast(b(57:60),'uint32');
 *                ctrl_mode    = b(61);
 *                cmd_mode     = b(62);
 *                ctrl_src     = b(63);        % 0=POT, 1=REMOTE
 *                rail_state   = b(64);        % 0=unhomed 1=ok 2=soft 3=hard 4=latched
 *                dummy_sine   = typecast(b(65:68),'single');
 *                dummy_ramp   = typecast(b(69:72),'single');
 *                dummy_count  = typecast(b(73:76),'uint32');
 *                fprintf("seq=%u  sine=%+.3f  ramp=%+.2f  count=%u\n", ...
 *                        seq, dummy_sine, dummy_ramp, dummy_count);
 *            end
 *        end
 *        pause(0.005);
 *    end
 ******************************************************************************
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Network endpoint (M4 -> MATLAB) ----------------------------------- */
#define MATLAB_REMOTE_IP_0      192
#define MATLAB_REMOTE_IP_1      168
#define MATLAB_REMOTE_IP_2      1
#define MATLAB_REMOTE_IP_3      5

#define MATLAB_TELEMETRY_PORT   5006u

/* ---- Cadence ----------------------------------------------------------- */
#define TELEMETRY_RATE_HZ       100u
#define TELEMETRY_PERIOD_MS     (1000u / TELEMETRY_RATE_HZ)

/* ---- Packet magic / version ------------------------------------------- */
#define TELEMETRY_MAGIC         0x54454C45u   /* 'TELE' (little-endian on wire) */

/* ---- Fixed-layout payload --------------------------------------------- *
 * Packed so MATLAB byte offsets match exactly. Size = 76 bytes.            */
#pragma pack(push, 1)
typedef struct {
    uint32_t magic;              /* TELEMETRY_MAGIC */
    uint32_t seq;                /* monotonic */
    uint32_t timestamp_ms;       /* M4 HAL_GetTick() at TX */

    /* State mirrored from shared D2 SRAM (M7 writes) */
    float    link_angle_rad[3];  /* 3x pendulum link angles */
    float    cart_position;
    float    cart_velocity;
    float    motor_command;
    float    odrive_motor_pos;

    /* Status counters */
    uint32_t m7_heartbeat;
    uint32_t loop_count;
    uint32_t loop_overrun_cnt;
    uint32_t m7_fault_flags;
    uint32_t m4_fault_flags;

    uint8_t  controller_mode;    /* CTRL_MODE_* (active on M7) */
    uint8_t  command_mode;       /* CTRL_MODE_* (requested by M4) */
    uint8_t  ctrl_src;           /* mirrors g_shared.ctrl_src: 0=POT, 1=REMOTE */
    uint8_t  rail_state;         /* mirrors g_shared.rail_state: 0=unhomed 1=ok
                                    2=soft-clamp 3=hard-trip 4=latched */

    /* Diagnostics */
    float    cmd_rx_count;       /* datagrams seen on UDP 5005, valid or not
                                    (was dummy_sine — same offset/size). Lets
                                    the GUI split "commands not arriving" from
                                    "commands rejected". */
    float    dummy_ramp;         /* sawtooth -10..+10 over 10 s — link health check */
    uint32_t command_seq;        /* mirrors g_shared.command_seq — increments on every accepted command */

    /* --- Encoder diagnostics (appended 2026-07-07; offsets above are
     *     unchanged — old parsers still read bytes 1..76 correctly). ---- */
    uint16_t enc_raw[3];         /* last raw SPI response word (0x0000 = dead bus) */
    uint16_t enc_diaagc[3];      /* AS5047P DIAAGC: [11]MAGL [10]MAGH [9]COF [8]LF [7:0]AGC */
    uint16_t enc_err_parity[3];  /* rejected frames per cause, wrapping u16 */
    uint16_t enc_err_ef[3];
    uint16_t enc_err_dead[3];
} TelemetryPacket_t;
#pragma pack(pop)

/* Static assertion: keep the wire-format size locked. */
#if defined(__GNUC__)
_Static_assert(sizeof(TelemetryPacket_t) == 106,
               "TelemetryPacket_t size changed — update MATLAB parser");
#endif

/* ---- Task entry (use with osThreadNew) -------------------------------- */
void StartTelemetryTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_H */
