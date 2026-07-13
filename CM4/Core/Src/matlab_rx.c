/**
 ******************************************************************************
 * @file    matlab_rx.c
 * @brief   M4 UDP receiver for MATLAB parameter injection (§6 inbound).
 ******************************************************************************
 */

#include "matlab_rx.h"
#include "main.h"
#include "shared_state.h"

#include "FreeRTOS.h"
#include "task.h"

#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/tcpip.h"
#include "lwip/netif.h"

#include <string.h>

/* Listening UDP PCB. */
static struct udp_pcb *s_rx_pcb = NULL;

/* Fault counters (also folded into g_shared.m4_fault_flags on errors). */
static uint32_t s_bad_magic_cnt = 0u;
static uint32_t s_bad_len_cnt   = 0u;
static uint32_t s_bad_cmd_cnt   = 0u;
static uint32_t s_range_cnt     = 0u;
static uint32_t s_apply_cnt     = 0u;

/* Every datagram that reaches the 5005 callback, valid or not. Telemetry
 * mirrors this so the GUI can split "commands not arriving" (count frozen)
 * from "commands rejected" (count climbs, command_seq doesn't). */
static uint32_t s_datagram_cnt  = 0u;

uint32_t MatlabRx_DatagramCount(void) { return s_datagram_cnt; }

/* ========================================================================
 *  Command applicators — writes to g_shared under a FreeRTOS critical
 *  section, which serializes against the other M4 writers (ESP32 RX task,
 *  watchdog task). Run in tcpip_thread context (LWIP callback). M7 reads
 *  lock-free: payload fields are stored before command_seq is bumped, and
 *  M7 only consumes them after seeing command_seq change.
 * ===================================================================== */

static void apply_set_mode(const uint8_t *payload, size_t payload_len)
{
    if (payload_len < 1u) { s_bad_len_cnt++; return; }
    uint8_t mode = payload[0];

    if (mode > CTRL_MODE_POT_VELOCITY) {
        s_range_cnt++;
        taskENTER_CRITICAL();
        g_shared.m4_fault_flags |= M4_FAULT_UDP_RX_BADLEN;   /* RMW: serialize vs other M4 writers */
        taskEXIT_CRITICAL();
        return;
    }

    taskENTER_CRITICAL();
    g_shared.command_mode = mode;      /* payload first ... */
    g_shared.cmd_set_mode = 1u;        /* ... then the one-shot request flag (F10) */
    g_shared.command_seq++;
    taskEXIT_CRITICAL();

    s_apply_cnt++;
}

static void apply_set_setpoint(const uint8_t *payload, size_t payload_len)
{
    if (payload_len < sizeof(float)) { s_bad_len_cnt++; return; }
    float sp;
    memcpy(&sp, payload, sizeof(float));

    if (!(sp >= -MATLAB_SETPOINT_MAX_ABS && sp <= MATLAB_SETPOINT_MAX_ABS)) {
        s_range_cnt++;
        taskENTER_CRITICAL();
        g_shared.m4_fault_flags |= M4_FAULT_UDP_RX_BADLEN;   /* RMW: serialize vs other M4 writers */
        taskEXIT_CRITICAL();
        return;
    }

    taskENTER_CRITICAL();
    g_shared.setpoint = sp;
    g_shared.command_seq++;
    taskEXIT_CRITICAL();

    s_apply_cnt++;
}

static void apply_lqr_delta(const uint8_t *payload, size_t payload_len)
{
    /* Layout: u8 idx, u8[3] pad, f32 delta.
     * ACCUMULATES into lqr_gain_delta[idx] (the GUI's +K/-K buttons are
     * incremental nudges), saturating at +/-MATLAB_GAIN_TOTAL_MAX. The
     * previous assignment semantics pinned the gain at the last delta,
     * which made live LQR tuning impossible. */
    if (payload_len < 8u) { s_bad_len_cnt++; return; }
    uint8_t idx = payload[0];
    float   d;
    memcpy(&d, &payload[4], sizeof(float));

    if (idx > MATLAB_GAIN_INDEX_MAX) {
        s_range_cnt++; return;
    }
    if (!(d >= -MATLAB_GAIN_DELTA_MAX && d <= MATLAB_GAIN_DELTA_MAX)) {
        s_range_cnt++; return;
    }

    taskENTER_CRITICAL();
    float acc = g_shared.lqr_gain_delta[idx] + d;
    if (acc >  MATLAB_GAIN_TOTAL_MAX) acc =  MATLAB_GAIN_TOTAL_MAX;
    if (acc < -MATLAB_GAIN_TOTAL_MAX) acc = -MATLAB_GAIN_TOTAL_MAX;
    g_shared.lqr_gain_delta[idx] = acc;
    g_shared.command_seq++;
    taskEXIT_CRITICAL();

    s_apply_cnt++;
}

static void apply_clear_errors(void)
{
    g_shared.m4_fault_flags = 0u;
    s_bad_magic_cnt = s_bad_len_cnt = s_bad_cmd_cnt = s_range_cnt = 0u;

    taskENTER_CRITICAL();
    g_shared.cmd_clear_errors = 1u;   /* M7 picks this up: sends Odrive_ClearErrors() over CAN */
    g_shared.command_seq++;
    taskEXIT_CRITICAL();

    s_apply_cnt++;
}

static void apply_set_ctrl_src(const uint8_t *payload, size_t payload_len)
{
    if (payload_len < 1u) { s_bad_len_cnt++; return; }
    uint8_t src = payload[0];

    if (src > 1u) {
        s_range_cnt++;
        taskENTER_CRITICAL();
        g_shared.m4_fault_flags |= M4_FAULT_UDP_RX_BADLEN;   /* RMW: serialize vs other M4 writers */
        taskEXIT_CRITICAL();
        return;
    }

    taskENTER_CRITICAL();
    g_shared.ctrl_src = src;
    g_shared.command_seq++;
    taskEXIT_CRITICAL();

    s_apply_cnt++;
}

static void apply_home(void)
{
    /* Request rail homing (rail zero reference). Same request-flag pattern
     * as index search; M7 clears the flag on entry. */
    taskENTER_CRITICAL();
    g_shared.cmd_home = 1u;
    g_shared.command_seq++;
    taskEXIT_CRITICAL();
    s_apply_cnt++;
}

static void apply_zero_upright(void)
{
    /* Capture the pendulum upright reference (state_est.c). Request-flag
     * pattern; M7 clears on entry. */
    taskENTER_CRITICAL();
    g_shared.cmd_zero_upright = 1u;
    g_shared.command_seq++;
    taskEXIT_CRITICAL();
    s_apply_cnt++;
}

static void apply_sysid_run(const uint8_t *payload, size_t payload_len)
{
    /* Layout: u8 type, u8[3] pad, f32 amp, f32 dur, f32 f0, f32 f1.
     * Params are stored BEFORE cmd_sysid is set and command_seq bumps, so
     * M7 always sees a coherent set. M7 re-clamps everything (sysid.c);
     * the bounds here just reject garbage early. */
    if (payload_len < 20u) { s_bad_len_cnt++; return; }
    uint8_t type = payload[0];
    float amp, dur, f0, f1;
    memcpy(&amp, &payload[4],  sizeof(float));
    memcpy(&dur, &payload[8],  sizeof(float));
    memcpy(&f0,  &payload[12], sizeof(float));
    memcpy(&f1,  &payload[16], sizeof(float));

    if (type > 3u ||
        !(amp >= 0.0f && amp <= MATLAB_SYSID_AMP_MAX) ||
        !(dur >  0.0f && dur <= MATLAB_SYSID_DUR_MAX) ||
        !(f0  >  0.0f && f0  <= MATLAB_SYSID_F_MAX)   ||
        !(f1  >  0.0f && f1  <= MATLAB_SYSID_F_MAX)) {
        s_range_cnt++;
        taskENTER_CRITICAL();
        g_shared.m4_fault_flags |= M4_FAULT_UDP_RX_BADLEN;   /* RMW: serialize vs other M4 writers */
        taskEXIT_CRITICAL();
        return;
    }

    taskENTER_CRITICAL();
    g_shared.sysid_type   = type;
    g_shared.sysid_amp_nm = amp;
    g_shared.sysid_dur_s  = dur;
    g_shared.sysid_f0_hz  = f0;
    g_shared.sysid_f1_hz  = f1;
    g_shared.cmd_sysid    = 1u;
    g_shared.command_seq++;
    taskEXIT_CRITICAL();

    s_apply_cnt++;
}

/* ========================================================================
 *  UDP RX callback (tcpip_thread context)
 * ===================================================================== */

static void udp_rx_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                       const ip_addr_t *addr, u16_t port)
{
    (void)arg; (void)pcb; (void)addr; (void)port;
    if (p == NULL) return;
    s_datagram_cnt++;

    if (p->tot_len < sizeof(MatlabCmdHeader_t)) {
        s_bad_len_cnt++;
        pbuf_free(p);
        return;
    }

    /* Flatten the (likely single-segment) pbuf into a local buffer so the
     * struct mapping is contiguous. Max useful payload is small. */
    uint8_t buf[64];
    uint16_t len = p->tot_len;
    if (len > sizeof(buf)) len = sizeof(buf);
    pbuf_copy_partial(p, buf, len, 0);
    pbuf_free(p);

    MatlabCmdHeader_t hdr;
    memcpy(&hdr, buf, sizeof(hdr));

    if (hdr.magic != MATLAB_CMD_MAGIC) {
        s_bad_magic_cnt++;
        return;
    }

    const uint8_t *payload     = buf + sizeof(MatlabCmdHeader_t);
    size_t         payload_len = (size_t)len - sizeof(MatlabCmdHeader_t);

    switch (hdr.cmd_id) {
    case MATLAB_CMD_SET_MODE:     apply_set_mode    (payload, payload_len); break;
    case MATLAB_CMD_SET_SETPOINT: apply_set_setpoint(payload, payload_len); break;
    case MATLAB_CMD_LQR_DELTA:    apply_lqr_delta   (payload, payload_len); break;
    case MATLAB_CMD_CLEAR_ERRORS: apply_clear_errors();                      break;
    case MATLAB_CMD_SET_CTRL_SRC: apply_set_ctrl_src(payload, payload_len);  break;
    case MATLAB_CMD_HOME:         apply_home();                              break;
    case MATLAB_CMD_ZERO_UPRIGHT: apply_zero_upright();                      break;
    case MATLAB_CMD_SYSID_RUN:    apply_sysid_run(payload, payload_len);     break;
    default:
        s_bad_cmd_cnt++;
        break;
    }
}

/* ========================================================================
 *  Public init
 * ===================================================================== */

int MatlabRx_Init(void)
{
    /* All LWIP raw-API calls go via the tcpip lock since we're outside
     * tcpip_thread. */
    LOCK_TCPIP_CORE();
    s_rx_pcb = udp_new();
    if (s_rx_pcb == NULL) {
        UNLOCK_TCPIP_CORE();
        return -1;
    }
    if (udp_bind(s_rx_pcb, IP_ADDR_ANY, MATLAB_RX_PORT) != ERR_OK) {
        udp_remove(s_rx_pcb);
        s_rx_pcb = NULL;
        UNLOCK_TCPIP_CORE();
        return -2;
    }
    udp_recv(s_rx_pcb, udp_rx_cb, NULL);
    UNLOCK_TCPIP_CORE();
    return 0;
}
