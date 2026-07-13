/**
 ******************************************************************************
 * @file    pot_ctrl.h
 * @brief   Button + potentiometer manual control for ODrive testing (M7).
 *
 *  Ported from ODrive_Pot_SW reference. Simplified: dropped the dual-switch
 *  input-source selector (manual / software-sine / MATLAB) — this version
 *  is always "manual pot". MATLAB injection will land in a later Step 6
 *  extension via the UDP RX path.
 *
 *  Wiring (Nucleo-H745ZI-Q):
 *      PC13   user button (active HIGH when pressed on Nucleo)
 *      PA3    potentiometer wiper, ADC1_INP15, 16-bit
 *      PB0    LED green
 *      PE1    LED yellow
 *      PB14   LED red
 *
 *  Loop pacing: call PotCtrl_Tick() every ~10 ms from M7 main()'s while(1).
 *
 *  Bus discipline:
 *      pot_ctrl emits Set_Axis_State / Set_Controller_Mode / Clear_Errors
 *      directly (one-shots on state transitions). The cyclic Set_Input_Pos /
 *      Set_Input_Vel come from control_loop.c at 5 kHz, gated on
 *      g_shared.controller_mode = CTRL_MODE_POT_POSITION / CTRL_MODE_POT_VELOCITY.
 *
 *  The ODrive S1 onboard (absolute) encoder needs no index search: motor +
 *  encoder-offset calibration are done once in odrivetool and saved, so the
 *  axis arms straight into CLOSED_LOOP (errors are cleared just before arming).
 *
 *  Button gestures:
 *      single   : IDLE -> enter POSITION_CONTROL (requires Home first)
 *      double   : POSITION_CONTROL <-> VELOCITY_CONTROL
 *      short hold (~1.2 s): any control state -> IDLE
 *      long hold (~4 s):  any state -> IDLE + Clear_Errors
 ******************************************************************************
 */

#ifndef POT_CTRL_H
#define POT_CTRL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Externally-visible sys state for debug/telemetry. */
typedef enum {
    POT_STATE_IDLE = 0,
    POT_STATE_POSITION_CONTROL,
    POT_STATE_VELOCITY_CONTROL,
    POT_STATE_EXTERNAL          /* balance_ctrl / sysid owns the motor; pot
                                   inert. Falls back to IDLE when the module
                                   hands controller_mode back, or on a button
                                   short/long hold (operator abort). */
} pot_sys_state_t;

/* Init: configure LED + button GPIO. Call once after Odrive_Init(). */
void PotCtrl_Init(void);

/* Tick: poll button + ADC, advance state machine, update LEDs, emit any
 * one-shot ODrive commands. Call every ~10 ms from M7 main loop. */
void PotCtrl_Tick(void);

/* Current state — useful for telemetry / debug. */
pot_sys_state_t PotCtrl_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* POT_CTRL_H */
