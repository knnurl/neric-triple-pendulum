/* COPY OF Common/Inc/esp_link_proto.h from the STM32 project.
 * Keep these two files BYTE-IDENTICAL. Any change requires reflashing both
 * STM32 and ESP32. If you have a build system that can symlink across
 * projects, prefer that. For now: copy-paste.
 *
 * This file is auto-shipped with the ESP32_Display project so the ESP32
 * build is self-contained.
 */

#ifndef ESP_LINK_PROTO_H
#define ESP_LINK_PROTO_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ESPLINK_SYNC0           0xAAu
#define ESPLINK_SYNC1           0x55u
#define ESPLINK_HEADER_BYTES    4u
#define ESPLINK_TRAILER_BYTES   1u
#define ESPLINK_OVERHEAD        (ESPLINK_HEADER_BYTES + ESPLINK_TRAILER_BYTES)
#define ESPLINK_MAX_PAYLOAD     200u
#define ESPLINK_MAX_FRAME       (ESPLINK_OVERHEAD + ESPLINK_MAX_PAYLOAD)

#define ESPLINK_BAUD            921600u

#define ESPLINK_TYPE_TELEMETRY      0x10u
#define ESPLINK_TYPE_ACK            0x11u
#define ESPLINK_TYPE_SET_MODE       0x20u
#define ESPLINK_TYPE_SET_SETPOINT   0x21u
#define ESPLINK_TYPE_LQR_DELTA      0x22u
#define ESPLINK_TYPE_CLEAR_ERRORS   0x23u
#define ESPLINK_TYPE_ZERO_UPRIGHT   0x24u   /* capture upright theta zero
                                               (was INDEX_SEARCH — removed) */
#define ESPLINK_TYPE_HOME           0x25u   /* rail homing (zero ref) */

#define ESPLINK_ACK_OK              0x00u
#define ESPLINK_ACK_BAD_CRC         0x01u
#define ESPLINK_ACK_BAD_TYPE        0x02u
#define ESPLINK_ACK_BAD_LEN         0x03u
#define ESPLINK_ACK_OUT_OF_RANGE    0x04u

typedef struct __attribute__((packed)) {
    uint32_t seq;
    uint32_t timestamp_ms;
    float    link_angle_rad[3];
    float    cart_position;
    float    cart_velocity;
    float    motor_command;
    float    odrive_motor_pos;
    uint32_t loop_count;
    uint32_t loop_overrun_cnt;
    uint32_t m7_fault_flags;
    uint32_t m4_fault_flags;
    uint8_t  controller_mode;
    uint8_t  command_mode;
    uint8_t  rail_state;         /* RAIL_STATE_* mirror below */
    uint8_t  _pad;
} EspTelemetry_t;

typedef struct __attribute__((packed)) {
    uint8_t  status;
    uint8_t  echoed_type;
    uint16_t _pad;
    uint32_t command_seq;
} EspAck_t;

typedef struct __attribute__((packed)) { uint8_t mode;     } EspCmdMode_t;
typedef struct __attribute__((packed)) { float   setpoint; } EspCmdSetpoint_t;
typedef struct __attribute__((packed)) {
    uint8_t index;
    uint8_t _pad[3];
    float   delta;
} EspCmdGainDelta_t;

#if defined(__GNUC__)
_Static_assert(sizeof(EspTelemetry_t)    == 56, "EspTelemetry_t size");
_Static_assert(sizeof(EspAck_t)          == 8,  "EspAck_t size");
_Static_assert(sizeof(EspCmdMode_t)      == 1,  "EspCmdMode_t size");
_Static_assert(sizeof(EspCmdSetpoint_t)  == 4,  "EspCmdSetpoint_t size");
_Static_assert(sizeof(EspCmdGainDelta_t) == 8,  "EspCmdGainDelta_t size");
#endif

/* Maxim CRC8 (poly 0x31 reflected = 0x8C, init 0x00). */
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

/* Controller-mode mirror — must match Common/Inc/shared_state.h on STM32. */
#define CTRL_MODE_IDLE          0u
#define CTRL_MODE_SWINGUP       1u
#define CTRL_MODE_BALANCE       2u
#define CTRL_MODE_SAFE_STOP     3u
#define CTRL_MODE_POT_POSITION  4u
#define CTRL_MODE_POT_VELOCITY  5u
#define CTRL_MODE_SYSID         6u

/* Rail-state mirror — must match CM7/Core/Inc/rail_limits.h on STM32. */
#define RAIL_STATE_UNHOMED      0u
#define RAIL_STATE_OK           1u
#define RAIL_STATE_SOFT_CLAMP   2u
#define RAIL_STATE_HARD_TRIP    3u
#define RAIL_STATE_LATCHED      4u

/* Fault-bit mirrors — must match Common/Inc/shared_state.h on STM32. */
#define M7_FAULT_LOOP_OVERRUN   (1u << 0)
#define M7_FAULT_SPI_TIMEOUT    (1u << 1)
#define M7_FAULT_CAN_TX_FAIL    (1u << 2)
#define M7_FAULT_ODRIVE_HB_LOST (1u << 3)
#define M7_FAULT_LIMIT_HIT      (1u << 4)
#define M7_FAULT_ENC_DATA       (1u << 5)
#define M7_FAULT_ODRIVE_AXIS    (1u << 6)  /* axis fault / left closed-loop or
                                              estimate stream stale during a run
                                              (run auto-aborted; added 07-11) */
#define M7_FAULT_INIT_FAIL      (1u << 7)  /* M7 died in Error_Handler; heartbeat
                                              frozen, red LED blinking (07-11) */

#define M4_FAULT_WDT_STALE_HB   (1u << 0)
#define M4_FAULT_UDP_RX_BADLEN  (1u << 1)
#define M4_FAULT_UART_BAD_CRC   (1u << 2)
#define M4_FAULT_HSEM_TIMEOUT   (1u << 3)
#define M4_FAULT_UDP_TX_FAIL    (1u << 4)

#ifdef __cplusplus
}
#endif

#endif /* ESP_LINK_PROTO_H */
