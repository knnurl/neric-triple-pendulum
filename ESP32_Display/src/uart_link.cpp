/**
 * uart_link.cpp — ESP32-S3 side parser + TX helpers.
 * Mirrors CM4/Core/Src/esp_link.c.
 */

#include "uart_link.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <string.h>

static HardwareSerial s_uart(1);   // UART1 on ESP32-S3

/* ----- Parser state machine (mirror of M4 side) ------------------------ */
enum ParseState { P_SYNC0, P_SYNC1, P_TYPE, P_LEN, P_PAYLOAD, P_CRC };
static ParseState s_pstate     = P_SYNC0;
static uint8_t    s_pmsg_type  = 0;
static uint8_t    s_pmsg_len   = 0;
static uint8_t    s_pbuf[ESPLINK_MAX_PAYLOAD];
static uint16_t   s_pbuf_idx   = 0;

static EspTelemetry_t s_last_telem;
static bool           s_telem_valid    = false;
static uint32_t       s_telem_rx_ms    = 0;

static uint32_t s_rx_frames   = 0;
static uint32_t s_rx_crc_err  = 0;
static uint32_t s_last_ack_seq = 0;
static uint8_t  s_last_ack_status = 0;

/* CRC over TYPE + LEN + payload */
static uint8_t crc_frame(uint8_t type, uint8_t len, const uint8_t *payload)
{
    uint8_t tmp[ESPLINK_MAX_PAYLOAD + 2];
    tmp[0] = type;
    tmp[1] = len;
    if (len) memcpy(&tmp[2], payload, len);
    return esplink_crc8(tmp, (size_t)(2u + len));
}

/* ----- TX --------------------------------------------------------------- */
static int send_frame(uint8_t type, const void *payload, uint8_t len)
{
    if (len > ESPLINK_MAX_PAYLOAD) return -1;
    uint8_t buf[ESPLINK_MAX_FRAME];
    buf[0] = ESPLINK_SYNC0;
    buf[1] = ESPLINK_SYNC1;
    buf[2] = type;
    buf[3] = len;
    if (len) memcpy(&buf[4], payload, len);
    buf[4 + len] = crc_frame(type, len, (const uint8_t *)payload);
    size_t total = (size_t)(ESPLINK_OVERHEAD + len);
    return (s_uart.write(buf, total) == total) ? 0 : -2;
}

int UartLink_SendMode(uint8_t mode)
{
    EspCmdMode_t c; c.mode = mode;
    return send_frame(ESPLINK_TYPE_SET_MODE, &c, sizeof(c));
}

int UartLink_SendSetpoint(float sp)
{
    EspCmdSetpoint_t c; c.setpoint = sp;
    return send_frame(ESPLINK_TYPE_SET_SETPOINT, &c, sizeof(c));
}

int UartLink_SendGainDelta(uint8_t index, float delta)
{
    EspCmdGainDelta_t c = {0};
    c.index = index;
    c.delta = delta;
    return send_frame(ESPLINK_TYPE_LQR_DELTA, &c, sizeof(c));
}

int UartLink_SendClearErrors(void)
{
    return send_frame(ESPLINK_TYPE_CLEAR_ERRORS, nullptr, 0);
}

int UartLink_SendHome(void)
{
    return send_frame(ESPLINK_TYPE_HOME, nullptr, 0);
}

int UartLink_SendZeroUpright(void)
{
    return send_frame(ESPLINK_TYPE_ZERO_UPRIGHT, nullptr, 0);
}

/* ----- RX dispatch ----------------------------------------------------- */
static void dispatch_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    s_rx_frames++;
    switch (type) {
    case ESPLINK_TYPE_TELEMETRY:
        if (len == sizeof(EspTelemetry_t)) {
            memcpy(&s_last_telem, payload, sizeof(s_last_telem));
            s_telem_valid = true;
            s_telem_rx_ms = millis();
        }
        break;
    case ESPLINK_TYPE_ACK:
        if (len == sizeof(EspAck_t)) {
            EspAck_t ack; memcpy(&ack, payload, sizeof(ack));
            s_last_ack_seq    = ack.command_seq;
            s_last_ack_status = ack.status;
        }
        break;
    default:
        /* Unknown type from M4 — ignore. */
        break;
    }
}

/* ----- Parser ---------------------------------------------------------- */
static void parser_reset(void)
{
    s_pstate = P_SYNC0; s_pmsg_type = 0; s_pmsg_len = 0; s_pbuf_idx = 0;
}

static void parser_feed(uint8_t b)
{
    switch (s_pstate) {
    case P_SYNC0:
        if (b == ESPLINK_SYNC0) s_pstate = P_SYNC1;
        break;
    case P_SYNC1:
        if      (b == ESPLINK_SYNC1) s_pstate = P_TYPE;
        else if (b == ESPLINK_SYNC0) /* stay */ ;
        else    s_pstate = P_SYNC0;
        break;
    case P_TYPE:
        s_pmsg_type = b; s_pstate = P_LEN;
        break;
    case P_LEN:
        s_pmsg_len = b;
        if (s_pmsg_len > ESPLINK_MAX_PAYLOAD) { parser_reset(); return; }
        s_pbuf_idx = 0;
        s_pstate   = (s_pmsg_len == 0) ? P_CRC : P_PAYLOAD;
        break;
    case P_PAYLOAD:
        s_pbuf[s_pbuf_idx++] = b;
        if (s_pbuf_idx >= s_pmsg_len) s_pstate = P_CRC;
        break;
    case P_CRC:
        if (crc_frame(s_pmsg_type, s_pmsg_len, s_pbuf) == b) {
            dispatch_frame(s_pmsg_type, s_pbuf, s_pmsg_len);
        } else {
            s_rx_crc_err++;
        }
        parser_reset();
        break;
    }
}

/* ----- Public API ----------------------------------------------------- */
void UartLink_Begin(int8_t rx_pin, int8_t tx_pin)
{
    /* setRxBufferSize MUST precede begin() on the ESP32 Arduino core —
     * called after, it is a no-op (the driver's ring buffer is allocated
     * inside begin) and RX silently stays at the 256-byte default, which
     * is marginal during long LVGL redraws at 921600 baud. (Bug found in
     * the 2026-07-11 review: the calls were in the wrong order.) */
    s_uart.setRxBufferSize(2048);
    s_uart.begin(ESPLINK_BAUD, SERIAL_8N1, rx_pin, tx_pin);
    parser_reset();
    s_telem_valid = false;
    s_telem_rx_ms = 0;
}

void UartLink_Poll(void)
{
    while (s_uart.available()) {
        parser_feed((uint8_t)s_uart.read());
    }
    /* Mark stale */
    if (s_telem_valid && (millis() - s_telem_rx_ms) > UART_LINK_STALE_MS) {
        s_telem_valid = false;
    }
}

const EspTelemetry_t* UartLink_LastTelemetry(void)
{
    return s_telem_valid ? &s_last_telem : nullptr;
}

uint32_t UartLink_RxFrames(void)      { return s_rx_frames; }
uint32_t UartLink_CrcErrors(void)     { return s_rx_crc_err; }
uint32_t UartLink_LastAckSeq(void)    { return s_last_ack_seq; }
uint8_t  UartLink_LastAckStatus(void) { return s_last_ack_status; }
