/**
 ******************************************************************************
 * @file    telemetry.c
 * @brief   UDP telemetry M4 -> MATLAB (Step 6).
 ******************************************************************************
 */

#include "telemetry.h"
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "shared_state.h"

#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/tcpip.h"
#include "lwip/netif.h"
#include "matlab_rx.h"
#include <math.h>
#include <string.h>

#ifndef M_TWOPI
#define M_TWOPI  6.28318530717958647692f
#endif

/* ---- Module state ------------------------------------------------------ */
static struct udp_pcb *s_pcb       = NULL;
static ip_addr_t       s_remote_ip = {0};

/* ---- Dummy waveform state (separate from g_shared, lives on M4 stack) -- */
static float    s_ramp    = -10.0f;   /* sawtooth -10..+10 */

/* Per-cycle increment derived from TELEMETRY_RATE_HZ. */
#define RAMP_DV       ((20.0f) / (10.0f * (float)TELEMETRY_RATE_HZ))      /* +20 over 10 s */

/* ---- Wait for netif default to be up. LWIP runs in tcpip_thread which
 *      is started by MX_LWIP_Init() (called from defaultTask). Until then
 *      netif_default is NULL and DHCP/link callbacks haven't run. ------- */
static void wait_for_lwip_ready(void)
{
    while (netif_default == NULL || !netif_is_up(netif_default)) {
        osDelay(50);
    }
}

/* ---- Build packet from current g_shared snapshot + dummy waveforms ---- */
static void pack_payload(TelemetryPacket_t *pkt, uint32_t seq)
{
    pkt->magic           = TELEMETRY_MAGIC;
    pkt->seq             = seq;
    pkt->timestamp_ms    = HAL_GetTick();

    /* Snapshot shared state. Each field is read independently — accept
     * that two adjacent fields may come from one cycle apart on M7. For
     * 100 Hz vs 5 kHz this is invisible. If you need a strict snapshot,
     * take HSEM_ID_M4_TO_M7 here (we're not, because we only READ). */
    pkt->link_angle_rad[0] = g_shared.link_angle_rad[0];
    pkt->link_angle_rad[1] = g_shared.link_angle_rad[1];
    pkt->link_angle_rad[2] = g_shared.link_angle_rad[2];
    pkt->cart_position     = g_shared.cart_position;
    pkt->cart_velocity     = g_shared.cart_velocity;
    pkt->motor_command     = g_shared.motor_command;
    pkt->odrive_motor_pos  = g_shared.odrive_motor_pos;

    pkt->m7_heartbeat      = g_shared.m7_heartbeat;
    pkt->loop_count        = g_shared.loop_count;
    pkt->loop_overrun_cnt  = g_shared.loop_overrun_cnt;
    pkt->m7_fault_flags    = g_shared.m7_fault_flags;
    pkt->m4_fault_flags    = g_shared.m4_fault_flags;

    pkt->controller_mode   = g_shared.controller_mode;
    pkt->command_mode      = g_shared.command_mode;
    pkt->ctrl_src          = g_shared.ctrl_src;
    pkt->rail_state        = g_shared.rail_state;

    /* Diagnostics */
    s_ramp  += RAMP_DV;
    if (s_ramp > 10.0f) { s_ramp = -10.0f; }

    pkt->cmd_rx_count      = (float)MatlabRx_DatagramCount();
    pkt->dummy_ramp        = s_ramp;
    pkt->command_seq       = g_shared.command_seq;

    for (int i = 0; i < 3; i++) {
        pkt->enc_raw[i]        = g_shared.enc_raw[i];
        pkt->enc_diaagc[i]     = g_shared.enc_diaagc[i];
        pkt->enc_err_parity[i] = g_shared.enc_err_parity[i];
        pkt->enc_err_ef[i]     = g_shared.enc_err_ef[i];
        pkt->enc_err_dead[i]   = g_shared.enc_err_dead[i];
    }
}

/* ---- Send one packet ---------------------------------------------------- *
 * pbuf is freed by udp_sendto on success, but we still call pbuf_free to
 * cover early-return failure paths cleanly. The lock is required because we
 * are not running on the tcpip_thread. */
static void send_packet(const TelemetryPacket_t *pkt)
{
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(*pkt), PBUF_RAM);
    if (p == NULL) {
        taskENTER_CRITICAL();
        g_shared.m4_fault_flags |= M4_FAULT_UDP_TX_FAIL;   /* RMW: serialize vs other M4 writers */
        taskEXIT_CRITICAL();
        return;
    }
    memcpy(p->payload, pkt, sizeof(*pkt));

    LOCK_TCPIP_CORE();
    (void)udp_sendto(s_pcb, p, &s_remote_ip, MATLAB_TELEMETRY_PORT);
    UNLOCK_TCPIP_CORE();

    pbuf_free(p);
}

/* ---- Task entry -------------------------------------------------------- */
void StartTelemetryTask(void *argument)
{
    (void)argument;

    wait_for_lwip_ready();

    LOCK_TCPIP_CORE();
    s_pcb = udp_new();
    UNLOCK_TCPIP_CORE();
    if (s_pcb == NULL) {
        /* Out of UDP PCBs — flag and idle. */
        taskENTER_CRITICAL();
        g_shared.m4_fault_flags |= M4_FAULT_UDP_TX_FAIL;   /* RMW: serialize vs other M4 writers */
        taskEXIT_CRITICAL();
        for (;;) { osDelay(1000); }
    }

    IP4_ADDR(&s_remote_ip,
             MATLAB_REMOTE_IP_0, MATLAB_REMOTE_IP_1,
             MATLAB_REMOTE_IP_2, MATLAB_REMOTE_IP_3);

    /* Open the MATLAB→M4 parameter-injection listener on port 5005 (§6 inbound).
     * Runs in tcpip_thread context once frames start arriving; failure to bind
     * is logged but not fatal — telemetry TX still works. */
    if (MatlabRx_Init() != 0) {
        taskENTER_CRITICAL();
        g_shared.m4_fault_flags |= M4_FAULT_UDP_TX_FAIL;   /* RMW: serialize vs other M4 writers */
        taskEXIT_CRITICAL();
    }

    uint32_t   seq  = 0u;
    TickType_t next = xTaskGetTickCount();

    TelemetryPacket_t pkt;
    for (;;) {
        pack_payload(&pkt, seq++);
        send_packet(&pkt);
        g_shared.m4_heartbeat++;
        vTaskDelayUntil(&next, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}
