/**
 ******************************************************************************
 * @file    odrive.h
 * @brief   ODrive S1 CAN-Simple comms over FDCAN1 (M7-only).
 *
 *  CAN ID layout (11-bit standard):
 *      bits [10:5] = node_id (6 bits)
 *      bits  [4:0] = cmd_id  (5 bits)
 *
 *  RX (this MCU as Host):
 *      0x001  Heartbeat           — axis_error[4] axis_state[1] proc_result[1]
 *                                    trajectory_done[1]
 *      0x009  Get_Encoder_Est.    — pos[4] vel[4] (cyclic; ODrive must be
 *                                    configured to broadcast in ODriveTool)
 *
 *  TX (Host -> ODrive):
 *      0x007  Set_Axis_State      — state[4]
 *      0x00B  Set_Controller_Mode — ctrl_mode[4] input_mode[4]
 *      0x00C  Set_Input_Pos       — pos(f32) vel_ff(i16) torque_ff(i16)
 *      0x00D  Set_Input_Vel       — vel(f32) torque_ff(f32)
 *      0x018  Clear_Errors        — identify_flag[1]
 *
 *  Threading:
 *      - TX helpers are called from the TIM7 control-loop ISR (preempt 0).
 *        HAL_FDCAN_AddMessageToTxFifoQ is non-blocking; if the TX FIFO is
 *        full it returns HAL_ERROR and we count it as a CAN TX fail.
 *      - RX callback runs at FDCAN1_IT0 (preempt 1, lower priority than the
 *        control loop). Writes only volatile shadow variables.
 ******************************************************************************
 */

#ifndef ODRIVE_H
#define ODRIVE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Build-time configuration ------------------------------------------ */
#define ODRIVE_NODE_ID                   0u

/* Heartbeat staleness threshold. ODrive heartbeat is ~100 Hz; 200 ms means
 * we tolerate ~20 consecutive missed frames before declaring the link dead. */
#define ODRIVE_HEARTBEAT_TIMEOUT_MS      200u

/* Boot-time wait for first heartbeat. If no heartbeat in this window after
 * starting the FDCAN bus, init reports failure. */
#define ODRIVE_BOOT_HEARTBEAT_WAIT_MS    1500u

/* Encoder-estimate staleness threshold. The cyclic Get_Encoder_Estimates
 * broadcast is configured >= 250 Hz (U2), so 20 ms of silence means 5+
 * consecutive missed frames — or a broadcast that was never configured.
 * Torque-mode controllers abort on this: x/x_dot frozen at their last
 * values is invisible to the state abort envelope. */
#define ODRIVE_EST_STALE_MS              20u

/* Grace window after a controller Start() before axis_error / axis_state
 * are enforced. Arming (ClearErrors + SetAxisState) is asynchronous over
 * CAN, and the heartbeat that reports the new state can lag by up to a
 * heartbeat period (~100 ms) plus processing. After this window, a run
 * whose axis is not in CLOSED_LOOP (self-disarm, rejected arm, latched
 * error) aborts instead of streaming torque to a dead axis. Controllers
 * tick at 1 kHz, so ticks == ms. */
#define ODRIVE_AXIS_SETTLE_MS            300u

/* ---- ODrive AxisState values (cmd 0x007) ------------------------------- */
#define ODRIVE_AXIS_STATE_UNDEFINED             0u
#define ODRIVE_AXIS_STATE_IDLE                  1u
#define ODRIVE_AXIS_STATE_FULL_CALIBRATION      3u
#define ODRIVE_AXIS_STATE_MOTOR_CALIBRATION     4u
#define ODRIVE_AXIS_STATE_ENCODER_INDEX_SEARCH  6u
#define ODRIVE_AXIS_STATE_ENCODER_OFFSET_CALIB  7u
#define ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL   8u

/* ---- Controller / Input modes (cmd 0x00B) ------------------------------ */
#define ODRIVE_CTRL_MODE_VOLTAGE         0u
#define ODRIVE_CTRL_MODE_TORQUE          1u
#define ODRIVE_CTRL_MODE_VELOCITY        2u
#define ODRIVE_CTRL_MODE_POSITION        3u

#define ODRIVE_INPUT_MODE_INACTIVE       0u
#define ODRIVE_INPUT_MODE_PASSTHROUGH    1u
#define ODRIVE_INPUT_MODE_VEL_RAMP       2u

/* ---- Public API -------------------------------------------------------- */

/* Snapshot of ODrive heartbeat, written by the RX ISR. Pulled by foreground
 * via Odrive_TakeStatus(). */
typedef struct {
    uint32_t axis_error;
    uint8_t  axis_state;
    uint8_t  procedure_result;
    uint8_t  trajectory_done;
    bool     fresh;     /* set true on each new heartbeat; consumer clears */
} odrive_status_t;

/* Configure FDCAN1 filter for ODrive node, start the bus, enable RX FIFO0
 * notification. Blocks up to ODRIVE_BOOT_HEARTBEAT_WAIT_MS waiting for the
 * first heartbeat. Returns true on success; false sets the
 * M7_FAULT_ODRIVE_HB_LOST bit in g_shared.m7_fault_flags. */
bool Odrive_Init(void);

/* TX helpers. All non-blocking; return HAL_StatusTypeDef from
 * HAL_FDCAN_AddMessageToTxFifoQ. */
int Odrive_SetAxisState(uint32_t state);
int Odrive_SetControllerMode(uint32_t ctrl_mode, uint32_t input_mode);
int Odrive_SetInputPos(float pos, int16_t vel_ff, int16_t torque_ff);
int Odrive_SetInputVel(float vel, float torque_ff);
int Odrive_SetInputTorque(float torque_nm);   /* TORQUE_CONTROL mode, Nm */
int Odrive_ClearErrors(void);
int Odrive_Estop(void);          /* immediate disarm; latches error on ODrive */
int Odrive_FeedWatchdog(void);   /* must be called periodically in active modes */
int Odrive_RequestEncoderEstimates(void);

/* Take a snapshot of the latest heartbeat (atomic w.r.t. RX ISR). Returns
 * true if a new heartbeat has arrived since the last call. */
bool Odrive_TakeStatus(odrive_status_t *out);

/* Latest motor pos/vel estimates from cyclic Get_Encoder_Estimates frames.
 * Atomic 32-bit reads on Cortex-M. */
float    Odrive_GetMotorPos(void);
float    Odrive_GetMotorVel(void);
uint32_t Odrive_GetMotorPosSeq(void);

/* Latest heartbeat-reported axis error / state (single-word atomic reads;
 * written by the RX ISR). Run-time supervision inputs for the controllers —
 * enforce only after ODRIVE_AXIS_SETTLE_MS from run start. */
uint32_t Odrive_GetAxisError(void);
uint8_t  Odrive_GetAxisState(void);

/* True while cyclic Get_Encoder_Estimates frames are arriving (false after
 * ODRIVE_EST_STALE_MS of silence, and false from boot until the first frame).
 * Maintained by Odrive_Tick; torque-mode controllers abort on false. */
bool     Odrive_EstimatesFresh(void);

/* Tick: call from the control loop ISR each cycle. Updates the heartbeat
 * staleness check and sets M7_FAULT_ODRIVE_HB_LOST if no heartbeat has
 * arrived for ODRIVE_HEARTBEAT_TIMEOUT_MS. Returns true if link is alive. */
bool Odrive_Tick(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* ODRIVE_H */
