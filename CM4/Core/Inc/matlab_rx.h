/**
 ******************************************************************************
 * @file    matlab_rx.h
 * @brief   M4 UDP receiver for MATLAB-issued commands (§6 inbound path).
 *
 *  Listens on UDP port 5005 for command frames from the MATLAB host.
 *  Mirrors the ESP32 UART command set so the same control surface works
 *  from either end. The RX callback runs in LWIP's tcpip_thread context;
 *  writes to g_shared are HSEM-protected (HSEM_ID_M4_TO_M7).
 *
 *  Packet format (little-endian on the wire):
 *
 *      ┌────────┬────────┬─────────┬────────────────┐
 *      │ magic  │ cmd_id │ _pad[3] │    payload     │
 *      │ u32    │  u8    │  3 B    │ variable bytes │
 *      └────────┴────────┴─────────┴────────────────┘
 *
 *      magic = MATLAB_CMD_MAGIC ('PCMD' little-endian)
 *      cmd_id = MATLAB_CMD_*
 *      _pad   = zeros (reserved for future use / 4-byte alignment of payload)
 *      payload = command-specific, see structs below
 *
 *  UDP carries its own checksum, so no application-level CRC. The magic
 *  word filters random traffic. Out-of-range payloads are rejected.
 ******************************************************************************
 *  MATLAB send snippet:
 *
 *    u = udpport("byte");
 *    target_ip = "192.168.1.10"; target_port = 5005;
 *
 *    % SET_MODE: send mode = 4 (POT_POSITION)
 *    pkt = [typecast(uint32(hex2dec('44434D50')), 'uint8'), ...
 *           uint8(1), uint8(0), uint8(0), uint8(0), ...
 *           uint8(4)];
 *    write(u, pkt, target_ip, target_port);
 *
 *    % SET_SETPOINT: send 1.5 rev
 *    pkt = [typecast(uint32(hex2dec('44434D50')), 'uint8'), ...
 *           uint8(2), uint8(0), uint8(0), uint8(0), ...
 *           typecast(single(1.5), 'uint8')];
 *    write(u, pkt, target_ip, target_port);
 *
 *    % LQR_GAIN_DELTA: bump gain[2] by +0.05
 *    pkt = [typecast(uint32(hex2dec('44434D50')), 'uint8'), ...
 *           uint8(3), uint8(0), uint8(0), uint8(0), ...
 *           uint8(2), uint8(0), uint8(0), uint8(0), ...
 *           typecast(single(0.05), 'uint8')];
 *    write(u, pkt, target_ip, target_port);
 *
 *    % CLEAR_ERRORS
 *    pkt = [typecast(uint32(hex2dec('44434D50')), 'uint8'), ...
 *           uint8(4), uint8(0), uint8(0), uint8(0)];
 *    write(u, pkt, target_ip, target_port);
 *
 *    % SET_CTRL_SRC: 0 = POT, 1 = REMOTE
 *    pkt = [typecast(uint32(hex2dec('44434D50')), 'uint8'), ...
 *           uint8(6), uint8(0), uint8(0), uint8(0), ...
 *           uint8(1)];
 *    write(u, pkt, target_ip, target_port);
 ******************************************************************************
 */

#ifndef MATLAB_RX_H
#define MATLAB_RX_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MATLAB_RX_PORT          5005u
#define MATLAB_CMD_MAGIC        0x44434D50u   /* 'PCMD' little-endian */

/* Command IDs (mirror the ESP_link set) */
#define MATLAB_CMD_SET_MODE       0x01u
#define MATLAB_CMD_SET_SETPOINT   0x02u
#define MATLAB_CMD_LQR_DELTA      0x03u
#define MATLAB_CMD_CLEAR_ERRORS   0x04u
/* 0x05 reserved (was MATLAB_CMD_INDEX_SEARCH — onboard absolute encoder needs no index search) */
#define MATLAB_CMD_SET_CTRL_SRC   0x06u   /* payload: u8, 0=POT 1=REMOTE */
#define MATLAB_CMD_HOME           0x07u   /* no payload: home the rail (zero ref) */
#define MATLAB_CMD_ZERO_UPRIGHT   0x08u   /* no payload: capture pendulum upright zero */
#define MATLAB_CMD_SYSID_RUN      0x09u   /* payload: u8 type, pad[3], f32 amp,
                                             f32 dur, f32 f0, f32 f1 (sysid.h) */

/* Sane-value bounds — same limits as the ESP_link path.
 * SET_SETPOINT carries BOTH the velocity setpoint (|v| <= 10 rev/s) and the
 * position setpoint (0..RAIL_TRAVEL_REV rev), so this ceiling must cover the
 * larger of the two — the rail travel — or "Far end" (= RAIL_TRAVEL_REV) is
 * range-rejected as UDP_RX_BADLEN. pot_ctrl re-clamps velocity to VEL_MAX and
 * position through the rail soft limits, so the wider bound is safe here. */
#define MATLAB_SETPOINT_MAX_ABS   12.0f   /* = RAIL_TRAVEL_REV (rail_limits.h) */
#define MATLAB_GAIN_DELTA_MAX     1.0f    /* per-message nudge */
#define MATLAB_GAIN_TOTAL_MAX     50.0f   /* accumulated |gain delta| ceiling */
#define MATLAB_GAIN_INDEX_MAX     5u
#define MATLAB_SYSID_AMP_MAX      2.0f    /* Nm (M7 re-clamps tighter, sysid.h) */
#define MATLAB_SYSID_DUR_MAX      60.0f   /* s */
#define MATLAB_SYSID_F_MAX        50.0f   /* Hz */

#pragma pack(push, 1)

typedef struct {
    uint32_t magic;
    uint8_t  cmd_id;
    uint8_t  _pad[3];
} MatlabCmdHeader_t;

typedef struct {
    MatlabCmdHeader_t hdr;
    uint8_t  mode;
} MatlabCmdSetMode_t;

typedef struct {
    MatlabCmdHeader_t hdr;
    float    setpoint;
} MatlabCmdSetSetpoint_t;

typedef struct {
    MatlabCmdHeader_t hdr;
    uint8_t  index;
    uint8_t  _pad[3];
    float    delta;
} MatlabCmdGainDelta_t;

typedef struct {
    MatlabCmdHeader_t hdr;
} MatlabCmdClearErrors_t;

typedef struct {
    MatlabCmdHeader_t hdr;
    uint8_t  ctrl_src;   /* 0 = POT, 1 = REMOTE */
} MatlabCmdSetCtrlSrc_t;

typedef struct {
    MatlabCmdHeader_t hdr;
    uint8_t  type;       /* 0=free 1=chirp 2=prbs 3=step */
    uint8_t  _pad[3];
    float    amp_nm;
    float    dur_s;
    float    f0_hz;
    float    f1_hz;
} MatlabCmdSysidRun_t;

#pragma pack(pop)

/* Static size assertions catch protocol drift on the MATLAB side. */
#if defined(__GNUC__)
_Static_assert(sizeof(MatlabCmdHeader_t)      == 8,  "header size");
_Static_assert(sizeof(MatlabCmdSetMode_t)     == 9,  "set_mode size");
_Static_assert(sizeof(MatlabCmdSetSetpoint_t) == 12, "set_setpoint size");
_Static_assert(sizeof(MatlabCmdGainDelta_t)   == 16, "gain_delta size");
_Static_assert(sizeof(MatlabCmdClearErrors_t) == 8,  "clear_errors size");
_Static_assert(sizeof(MatlabCmdSetCtrlSrc_t)  == 9,  "set_ctrl_src size");
_Static_assert(sizeof(MatlabCmdSysidRun_t)    == 28, "sysid_run size");
#endif

/* Init: open UDP PCB on port 5005 and register RX callback.
 * Call AFTER LWIP is up (after MX_LWIP_Init has run and netif is up).
 * Returns 0 on success. */
int MatlabRx_Init(void);

/* Total datagrams seen on port 5005, valid or not. Mirrored into telemetry
 * (cmd_rx_count) so the host can tell "not arriving" from "rejected". */
uint32_t MatlabRx_DatagramCount(void);

#ifdef __cplusplus
}
#endif

#endif /* MATLAB_RX_H */
