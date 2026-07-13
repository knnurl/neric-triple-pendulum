/**
 ******************************************************************************
 * @file    idlog_tx.c
 * @brief   High-rate ID/balance log streamer: shared ring -> UDP 5007 (M4).
 ******************************************************************************
 */

#include "idlog_tx.h"
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "shared_state.h"
#include "telemetry.h"          /* MATLAB_REMOTE_IP_* */

#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/tcpip.h"
#include "lwip/netif.h"
#include <string.h>

static struct udp_pcb *s_pcb       = NULL;
static ip_addr_t       s_remote_ip = {0};
static uint32_t        s_pkt_seq   = 0u;

/* Same LWIP-readiness gate as telemetry.c. */
static void wait_for_lwip_ready(void)
{
    while (netif_default == NULL || !netif_is_up(netif_default)) {
        osDelay(50);
    }
}

/* Drain up to IDLOG_TX_MAX_SAMPLES from the shared SPSC ring into one UDP
 * packet. Returns the number of samples shipped (0 = ring empty, no TX).
 *
 * Ring protocol: M7 publishes with head (free-running), we consume with
 * tail. head is read ONCE per drain — samples landing mid-drain go in the
 * next packet. */
static uint32_t drain_once(void)
{
    uint32_t head = g_shared.idlog_head;   /* single volatile read */
    uint32_t tail = g_shared.idlog_tail;
    uint32_t avail = head - tail;
    if (avail == 0u) return 0u;
    if (avail > IDLOG_TX_MAX_SAMPLES) avail = IDLOG_TX_MAX_SAMPLES;

    uint8_t buf[sizeof(IdLogPktHeader_t)
                + IDLOG_TX_MAX_SAMPLES * sizeof(IdLogWireSample_t)];

    IdLogPktHeader_t hdr;
    hdr.magic     = IDLOG_TX_MAGIC;
    hdr.seq       = s_pkt_seq++;
    hdr.n_samples = (uint16_t)avail;
    hdr.drops     = (uint16_t)(g_shared.idlog_drops & 0xFFFFu);
    memcpy(&buf[0], &hdr, sizeof(hdr));

    for (uint32_t k = 0; k < avail; k++) {
        uint32_t i = (tail + k) % IDLOG_RING_LEN;
        IdLogWireSample_t s;
        s.tick      = g_shared.idlog[i].tick;
        s.theta     = g_shared.idlog[i].theta;
        s.theta_dot = g_shared.idlog[i].theta_dot;
        s.x         = g_shared.idlog[i].x;
        s.x_dot     = g_shared.idlog[i].x_dot;
        s.u         = g_shared.idlog[i].u;
        memcpy(&buf[sizeof(hdr) + k * sizeof(s)], &s, sizeof(s));
    }

    /* Consume only after copying — M7 treats [tail, head) as in-use. */
    g_shared.idlog_tail = tail + avail;

    uint16_t len = (uint16_t)(sizeof(hdr) + avail * sizeof(IdLogWireSample_t));
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    if (p == NULL) {
        taskENTER_CRITICAL();
        g_shared.m4_fault_flags |= M4_FAULT_UDP_TX_FAIL;   /* RMW: serialize vs other M4 writers */
        taskEXIT_CRITICAL();
        return avail;   /* samples consumed but lost; drops visible via seq gap */
    }
    memcpy(p->payload, buf, len);

    LOCK_TCPIP_CORE();
    (void)udp_sendto(s_pcb, p, &s_remote_ip, IDLOG_TX_PORT);
    UNLOCK_TCPIP_CORE();

    pbuf_free(p);
    return avail;
}

void StartIdLogTxTask(void *argument)
{
    (void)argument;

    wait_for_lwip_ready();

    LOCK_TCPIP_CORE();
    s_pcb = udp_new();
    UNLOCK_TCPIP_CORE();
    if (s_pcb == NULL) {
        taskENTER_CRITICAL();
        g_shared.m4_fault_flags |= M4_FAULT_UDP_TX_FAIL;   /* RMW: serialize vs other M4 writers */
        taskEXIT_CRITICAL();
        for (;;) { osDelay(1000); }
    }

    IP4_ADDR(&s_remote_ip,
             MATLAB_REMOTE_IP_0, MATLAB_REMOTE_IP_1,
             MATLAB_REMOTE_IP_2, MATLAB_REMOTE_IP_3);

    TickType_t next = xTaskGetTickCount();
    for (;;) {
        /* Keep draining until the ring is empty this period — after a burst
         * (or a missed slot) one drain per period would never catch up. */
        while (drain_once() == IDLOG_TX_MAX_SAMPLES) { /* full packet -> more */ }
        vTaskDelayUntil(&next, pdMS_TO_TICKS(IDLOG_TX_PERIOD_MS));
    }
}
