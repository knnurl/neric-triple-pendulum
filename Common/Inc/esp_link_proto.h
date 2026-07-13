/**
 ******************************************************************************
 * @file    esp_link_proto.h
 * @brief   UART frame protocol shared by H755 M4 and ESP32-S3 display.
 *
 *  THIS HEADER IS BIT-IDENTICAL ON BOTH SIDES. Copy it into the ESP32
 *  project under ESP32_Display/main/include/ (or symlink). Any change here
 *  requires re-flashing both sides.
 *
 *  Frame layout (host-agnostic, little-endian on the wire):
 *
 *      ┌──────┬──────┬──────┬──────┬─────────────────┬──────┐
 *      │ 0xAA │ 0x55 │ TYPE │ LEN  │     PAYLOAD     │ CRC8 │
 *      │  u8  │  u8  │  u8  │  u8  │   0..200 bytes  │  u8  │
 *      └──────┴──────┴──────┴──────┴─────────────────┴──────┘
 *
 *      LEN  : payload-only length (excluding header + CRC)
 *      CRC8 : Maxim/Dallas (poly 0x31 reflected = 0x8C, init 0x00),
 *             computed over { TYPE, LEN, PAYLOAD[..] }
 *
 *  Identical format both directions; asymmetric TYPE IDs distinguish.
 ******************************************************************************
 */

#ifndef ESP_LINK_PROTO_H
#define ESP_LINK_PROTO_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Frame constants -------------------------------------------------- */
#define ESPLINK_SYNC0           0xAAu
#define ESPLINK_SYNC1           0x55u
#define ESPLINK_HEADER_BYTES    4u           /* SYNC0 SYNC1 TYPE LEN */
#define ESPLINK_TRAILER_BYTES   1u           /* CRC8 */
#define ESPLINK_OVERHEAD        (ESPLINK_HEADER_BYTES + ESPLINK_TRAILER_BYTES)
#define ESPLINK_MAX_PAYLOAD     200u
#define ESPLINK_MAX_FRAME       (ESPLINK_OVERHEAD + ESPLINK_MAX_PAYLOAD)

/* ---- Baud rate (informational; set in HAL init) ----------------------- */
#define ESPLINK_BAUD            921600u

/* ---- Message types ---------------------------------------------------- *
 * M4 -> ESP32  : 0x10..0x1F
 * ESP32 -> M4  : 0x20..0x2F                                                */
#define ESPLINK_TYPE_TELEMETRY      0x10u   /* M4 -> ESP32 : EspTelemetry_t */
#define ESPLINK_TYPE_ACK            0x11u   /* M4 -> ESP32 : EspAck_t       */

#define ESPLINK_TYPE_SET_MODE       0x20u   /* ESP32 -> M4 : EspCmdMode_t   */
#define ESPLINK_TYPE_SET_SETPOINT   0x21u   /* ESP32 -> M4 : EspCmdSetpoint_t */
#define ESPLINK_TYPE_LQR_DELTA      0x22u   /* ESP32 -> M4 : EspCmdGainDelta_t */
#define ESPLINK_TYPE_CLEAR_ERRORS   0x23u   /* ESP32 -> M4 : (empty)        */
#define ESPLINK_TYPE_ZERO_UPRIGHT   0x24u   /* ESP32 -> M4 : (empty) capture upright
                                               theta zero (was INDEX_SEARCH — removed
                                               with the onboard absolute encoder) */
#define ESPLINK_TYPE_HOME           0x25u   /* ESP32 -> M4 : (empty) rail homing */

/* ---- ACK status codes (in EspAck_t.status) ---------------------------- */
#define ESPLINK_ACK_OK              0x00u
#define ESPLINK_ACK_BAD_CRC         0x01u
#define ESPLINK_ACK_BAD_TYPE        0x02u
#define ESPLINK_ACK_BAD_LEN         0x03u
#define ESPLINK_ACK_OUT_OF_RANGE    0x04u

/* ---- Payload structs (all packed, little-endian) ---------------------- */

/* M4 -> ESP32: display telemetry. Subset of g_shared. */
typedef struct __attribute__((packed)) {
    uint32_t seq;                /* monotonic */
    uint32_t timestamp_ms;       /* M4 HAL_GetTick() at TX */

    float    link_angle_rad[3];  /* 3x pendulum link angles */
    float    cart_position;
    float    cart_velocity;
    float    motor_command;      /* setpoint sent to ODrive */
    float    odrive_motor_pos;   /* ODrive encoder estimate */

    uint32_t loop_count;         /* M7 control-loop iteration count */
    uint32_t loop_overrun_cnt;
    uint32_t m7_fault_flags;
    uint32_t m4_fault_flags;

    uint8_t  controller_mode;    /* active CTRL_MODE_* on M7 */
    uint8_t  command_mode;       /* requested CTRL_MODE_* from M4/ESP/MATLAB */
    uint8_t  rail_state;         /* RAIL_STATE_* (rail_limits.h): unhomed/ok/
                                    soft-clamp/hard-trip/latched */
    uint8_t  _pad;
} EspTelemetry_t;

/* M4 -> ESP32: acknowledgement of last ESP32-issued command. */
typedef struct __attribute__((packed)) {
    uint8_t  status;             /* ESPLINK_ACK_* */
    uint8_t  echoed_type;        /* type that was acked / rejected */
    uint16_t _pad;
    uint32_t command_seq;        /* monotonic seq mirroring g_shared.command_seq */
} EspAck_t;

/* ESP32 -> M4: request a controller-mode change. */
typedef struct __attribute__((packed)) {
    uint8_t mode;                /* CTRL_MODE_* */
} EspCmdMode_t;

/* ESP32 -> M4: setpoint update. */
typedef struct __attribute__((packed)) {
    float setpoint;              /* meaning depends on active mode */
} EspCmdSetpoint_t;

/* ESP32 -> M4: LQR gain delta. */
typedef struct __attribute__((packed)) {
    uint8_t index;               /* 0..5 (six-state gain vector) */
    uint8_t _pad[3];
    float   delta;
} EspCmdGainDelta_t;

/* Static size assertions catch accidental layout drift on either side. */
#if defined(__GNUC__)
_Static_assert(sizeof(EspTelemetry_t)     == 56,
               "EspTelemetry_t size drift -- both ends must match");
_Static_assert(sizeof(EspAck_t)           == 8,
               "EspAck_t size drift");
_Static_assert(sizeof(EspCmdMode_t)       == 1, "EspCmdMode_t size");
_Static_assert(sizeof(EspCmdSetpoint_t)   == 4, "EspCmdSetpoint_t size");
_Static_assert(sizeof(EspCmdGainDelta_t)  == 8, "EspCmdGainDelta_t size");
#endif

/* ---- CRC8 (Maxim/Dallas, poly 0x31 reflected = 0x8C, init 0x00) ------ *
 * Inline so both sides get the same implementation without linking
 * anything. Bit-by-bit (no table) — ~80 bytes code, fast enough for
 * <=205-byte frames at 50 Hz. */
static inline uint8_t esplink_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00u;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x01u) ? (uint8_t)((crc >> 1) ^ 0x8Cu)
                                : (uint8_t)(crc >> 1);
        }
    }
    return crc;
}

#ifdef __cplusplus
}
#endif

#endif /* ESP_LINK_PROTO_H */
