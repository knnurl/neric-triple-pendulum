/**
 ******************************************************************************
 * @file    odrive.c
 * @brief   ODrive S1 CAN-Simple comms over FDCAN1 (M7).
 ******************************************************************************
 */

#include "odrive.h"
#include "main.h"
#include "shared_state.h"
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;

/* ---- CANSimple command IDs (ODrive 0.6.x — authoritative per PDF) ------ *
 * Reference: ODrive Documentation 0.6.12, CAN Protocol section, Messages
 * table. Empirically verified against the working reference project. */
#define CMD_HEARTBEAT                    0x001u
#define CMD_ESTOP                        0x002u
#define CMD_GET_ERROR                    0x003u
#define CMD_RX_SDO                       0x004u
#define CMD_TX_SDO                       0x005u
#define CMD_ADDRESS                      0x006u
#define CMD_SET_AXIS_STATE               0x007u
#define CMD_GET_ENCODER_ESTIMATES        0x009u
#define CMD_SET_CONTROLLER_MODE          0x00Bu
#define CMD_SET_INPUT_POS                0x00Cu
#define CMD_SET_INPUT_VEL                0x00Du
#define CMD_SET_INPUT_TORQUE             0x00Eu
#define CMD_CLEAR_ERRORS                 0x018u

/* ---- Internal state (volatile because RX writes are from ISR) ---------- */
static volatile odrive_status_t s_status        = {0};
static volatile float           s_motor_pos     = 0.0f;
static volatile float           s_motor_vel     = 0.0f;
static volatile uint32_t        s_motor_pos_seq = 0u;
static volatile uint32_t        s_last_heartbeat_ms = 0u;
static volatile bool            s_link_alive    = false;

/* Any heartbeat received since boot? Without this latch the boot-time
 * (now_ms - 0) subtraction reads "alive" for the first
 * ODRIVE_HEARTBEAT_TIMEOUT_MS after reset — briefly clearing the HB_LOST
 * fault Odrive_Init just set, even with no ODrive on the bus. */
static volatile bool            s_hb_ever       = false;

/* Encoder-estimate stream freshness (see Odrive_EstimatesFresh). Written
 * only from Odrive_Tick (TIM7 ISR context). */
static volatile bool            s_est_fresh     = false;
static uint32_t                 s_est_seq_prev      = 0u;
static uint32_t                 s_est_seq_change_ms = 0u;

/* ---- Helpers ----------------------------------------------------------- */
static inline uint32_t make_id(uint32_t cmd_id)
{
    return ((uint32_t)ODRIVE_NODE_ID << 5) | (cmd_id & 0x1Fu);
}

/* Queue one frame. Callers span two preemption levels: the TIM7 ISR streams
 * Set_Input_* at 5 kHz while pot_ctrl / rail homing send mode & axis-state
 * changes from the main loop. HAL_FDCAN_AddMessageToTxFifoQ is NOT reentrant
 * (put-index read, message-RAM copy, TXBAR commit), so a TIM7 preemption
 * mid-add could double-book a TX slot — the worst frame to lose that way is
 * exit_to_idle's SetAxisState(IDLE), which would leave the axis energized
 * while the firmware believes it coasts. Mask IRQs around the add (~1-2 us
 * at 400 MHz); save/restore PRIMASK rather than blindly re-enabling so
 * ISR-context callers are unaffected. (Arch review 2026-07-11, F2.) */
static int fdcan_queue(const FDCAN_TxHeaderTypeDef *tx, const uint8_t *payload)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    int rc = (int)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,
                    (FDCAN_TxHeaderTypeDef *)tx, (uint8_t *)payload);
    __set_PRIMASK(primask);
    return rc;
}

static int fdcan_send(uint32_t id, const uint8_t *payload)
{
    FDCAN_TxHeaderTypeDef tx = {0};
    tx.Identifier          = id;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;
    return fdcan_queue(&tx, payload);
}

static int fdcan_send_rtr(uint32_t id)
{
    FDCAN_TxHeaderTypeDef tx = {0};
    tx.Identifier          = id;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_REMOTE_FRAME;
    tx.DataLength          = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;
    return fdcan_queue(&tx, NULL);
}

/* ---- Init -------------------------------------------------------------- *
 * Filter accepts all cmd_ids from ODRIVE_NODE_ID:
 *   FilterID1 = NODE_ID << 5     bits 10:5 = node id, 4:0 = 0
 *   FilterID2 = 0x7E0            mask: check upper 6 bits only
 * Anything else is rejected by the configured reject-default policy. */
bool Odrive_Init(void)
{
    FDCAN_FilterTypeDef f = {0};
    f.IdType       = FDCAN_STANDARD_ID;
    f.FilterType   = FDCAN_FILTER_MASK;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterIndex  = 0;
    f.FilterID1    = (uint32_t)ODRIVE_NODE_ID << 5;
    f.FilterID2    = 0x7E0u;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &f) != HAL_OK) { return false; }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
            FDCAN_REJECT, FDCAN_REJECT,
            FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
        return false;
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) { return false; }
    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        return false;
    }

    /* Non-blocking: do NOT busy-wait for the first heartbeat. If we wait
     * here, M7 stops incrementing g_shared.m7_heartbeat for up to 1.5 s
     * (no ODrive on bus -> full timeout), and M4's watchdog task will
     * declare M7 dead and trigger a WWDG2 reset within ~35 ms. That kills
     * the PHY negotiation and the ETH link LEDs go dark.
     *
     * Instead: declare the link dead at boot; Odrive_Tick (called from the
     * control loop every cycle) will clear the fault bit the moment a real
     * heartbeat arrives. */
    s_link_alive = false;
    g_shared.m7_fault_flags |= M7_FAULT_ODRIVE_HB_LOST;
    return true;
}

/* ---- TX helpers -------------------------------------------------------- */
int Odrive_SetAxisState(uint32_t state)
{
    uint8_t p[8] = {0};
    p[0] = (uint8_t)(state      );
    p[1] = (uint8_t)(state >>  8);
    p[2] = (uint8_t)(state >> 16);
    p[3] = (uint8_t)(state >> 24);
    return fdcan_send(make_id(CMD_SET_AXIS_STATE), p);
}

int Odrive_SetControllerMode(uint32_t ctrl_mode, uint32_t input_mode)
{
    uint8_t p[8] = {0};
    p[0] = (uint8_t)(ctrl_mode       ); p[1] = (uint8_t)(ctrl_mode  >>  8);
    p[2] = (uint8_t)(ctrl_mode  >> 16); p[3] = (uint8_t)(ctrl_mode  >> 24);
    p[4] = (uint8_t)(input_mode      ); p[5] = (uint8_t)(input_mode >>  8);
    p[6] = (uint8_t)(input_mode >> 16); p[7] = (uint8_t)(input_mode >> 24);
    return fdcan_send(make_id(CMD_SET_CONTROLLER_MODE), p);
}

int Odrive_SetInputPos(float pos, int16_t vel_ff, int16_t torque_ff)
{
    uint8_t p[8] = {0};
    memcpy(&p[0], &pos, sizeof(float));
    p[4] = (uint8_t)(vel_ff       ); p[5] = (uint8_t)(vel_ff    >> 8);
    p[6] = (uint8_t)(torque_ff    ); p[7] = (uint8_t)(torque_ff >> 8);
    return fdcan_send(make_id(CMD_SET_INPUT_POS), p);
}

int Odrive_SetInputVel(float vel, float torque_ff)
{
    uint8_t p[8] = {0};
    memcpy(&p[0], &vel,       sizeof(float));
    memcpy(&p[4], &torque_ff, sizeof(float));
    return fdcan_send(make_id(CMD_SET_INPUT_VEL), p);
}

int Odrive_SetInputTorque(float torque_nm)
{
    /* CANSimple Set_Input_Torque (0x00E): f32 torque in Nm at bytes 0-3.
     * Positive = positive motor direction (same sign convention as pos/vel).
     * Requires controller in TORQUE_CONTROL / PASSTHROUGH. Like the other
     * Set_Input_* messages it also resets the ODrive axis watchdog. */
    uint8_t p[8] = {0};
    memcpy(&p[0], &torque_nm, sizeof(float));
    return fdcan_send(make_id(CMD_SET_INPUT_TORQUE), p);
}

int Odrive_ClearErrors(void)
{
    uint8_t p[8] = {0};
    return fdcan_send(make_id(CMD_CLEAR_ERRORS), p);
}

int Odrive_Estop(void)
{
    /* CANSimple Estop (0x002): immediate disarm, latches ESTOP_REQUESTED on
     * the ODrive — Clear_Errors required before the next closed-loop entry.
     * Used by the rail hard-stop switches. */
    uint8_t p[8] = {0};
    return fdcan_send(make_id(CMD_ESTOP), p);
}

int Odrive_FeedWatchdog(void)
{
    /* ODrive S1 v0.6.x has NO standalone Feed_Watchdog CAN command.
     * The axis watchdog is reset automatically by Set_Input_Pos / Set_Input_Vel
     * / Set_Input_Torque. To keep the watchdog fed in non-active states, send
     * Set_Input_Vel(0, 0) — when the axis is in IDLE or any non-CLOSED_LOOP
     * state, the input is ignored by the control logic but the message still
     * resets the watchdog. */
    return Odrive_SetInputVel(0.0f, 0.0f);
}

int Odrive_RequestEncoderEstimates(void)
{
    return fdcan_send_rtr(make_id(CMD_GET_ENCODER_ESTIMATES));
}

/* ---- Status accessors -------------------------------------------------- */
bool Odrive_TakeStatus(odrive_status_t *out)
{
    bool was_fresh;
    __disable_irq();
    was_fresh = s_status.fresh;
    if (was_fresh) {
        out->axis_error       = s_status.axis_error;
        out->axis_state       = s_status.axis_state;
        out->procedure_result = s_status.procedure_result;
        out->trajectory_done  = s_status.trajectory_done;
        out->fresh            = true;
        s_status.fresh        = false;
    }
    __enable_irq();
    return was_fresh;
}

float    Odrive_GetMotorPos(void)    { return s_motor_pos; }
float    Odrive_GetMotorVel(void)    { return s_motor_vel; }
uint32_t Odrive_GetMotorPosSeq(void) { return s_motor_pos_seq; }

uint32_t Odrive_GetAxisError(void)   { return s_status.axis_error; }
uint8_t  Odrive_GetAxisState(void)   { return s_status.axis_state; }
bool     Odrive_EstimatesFresh(void) { return s_est_fresh; }

/* ---- Per-cycle tick (called from control-loop ISR) --------------------- *
 * Updates link-alive flag and fault bit based on heartbeat freshness. */
bool Odrive_Tick(uint32_t now_ms)
{
    uint32_t last = s_last_heartbeat_ms;
    /* s_hb_ever gates the boot window: with s_last_heartbeat_ms still 0,
     * (now_ms - 0) would read "alive" for the first TIMEOUT ms after reset. */
    bool alive = s_hb_ever && ((now_ms - last) <= ODRIVE_HEARTBEAT_TIMEOUT_MS);

    if (alive && !s_link_alive) {
        /* Recovered — clear the fault bit. */
        g_shared.m7_fault_flags &= ~(uint32_t)M7_FAULT_ODRIVE_HB_LOST;
    } else if (!alive && s_link_alive) {
        /* Just lost the link. */
        g_shared.m7_fault_flags |= M7_FAULT_ODRIVE_HB_LOST;
    }
    s_link_alive = alive;

    /* Encoder-estimate stream freshness (arch review F1c). The RX ISR bumps
     * s_motor_pos_seq on every Get_Encoder_Estimates frame; silence for
     * ODRIVE_EST_STALE_MS means x/x_dot are frozen at their last values —
     * the torque-mode controllers abort on this rather than balance against
     * a stale cart. Starts false and stays false until the first frame, so
     * a never-configured broadcast (U2 not done) refuses runs instead of
     * silently running blind. */
    uint32_t seq = s_motor_pos_seq;
    if (seq != s_est_seq_prev) {
        s_est_seq_prev      = seq;
        s_est_seq_change_ms = now_ms;
        s_est_fresh         = true;
    } else if ((now_ms - s_est_seq_change_ms) > ODRIVE_EST_STALE_MS) {
        s_est_fresh = false;
    }

    /* Mirror ODrive motor-pos shadow into shared state for M4 telemetry. */
    g_shared.odrive_motor_pos = s_motor_pos;

    return alive;
}

/* ---- RX callback (HAL weak override; runs at FDCAN1_IT0 preempt 1) ----- */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u) { return; }

    FDCAN_RxHeaderTypeDef rx;
    uint8_t data[8];
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, data) != HAL_OK) {
        return;
    }

    uint8_t node = (uint8_t)((rx.Identifier >> 5)  & 0x3Fu);
    uint8_t cmd  = (uint8_t)( rx.Identifier        & 0x1Fu);

    if (node != ODRIVE_NODE_ID) { return; }

    if (cmd == CMD_HEARTBEAT) {
        s_status.axis_error =  (uint32_t)data[0]
                            | ((uint32_t)data[1] <<  8)
                            | ((uint32_t)data[2] << 16)
                            | ((uint32_t)data[3] << 24);
        s_status.axis_state       = data[4];
        s_status.procedure_result = data[5];
        s_status.trajectory_done  = data[6];
        s_status.fresh            = true;
        s_last_heartbeat_ms       = HAL_GetTick();
        s_hb_ever                 = true;
    }
    else if (cmd == CMD_GET_ENCODER_ESTIMATES) {
        float pos, vel;
        memcpy(&pos, &data[0], sizeof(float));
        memcpy(&vel, &data[4], sizeof(float));
        s_motor_pos = pos;
        s_motor_vel = vel;
        s_motor_pos_seq++;
    }
}
