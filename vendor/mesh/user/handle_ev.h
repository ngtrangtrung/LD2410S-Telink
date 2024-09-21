/*
 * handle_ev.h
 *
 *  Created on: Oct 5, 2020
 *      Author: DungTran BK
 */

#ifndef HANDLE_EV_H_
#define HANDLE_EV_H_
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "../../../proj/tl_common.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

typedef enum {
	LED_POWER_ON,
	LED_OTA_FAIL,
	LED_OTA_SUCESS,
	LED_PROVISION_FAIL,
	LED_PROVISION_SUCCESS,
	LED_SUC_ADD_APPKEY,
	LED_FAIL_ADD_APPKEY,
	LED_CMD_SET_SUBSCRIPTION,
	LED_CMD_DEL_SUBSCRIPTION,
	LED_CMD_SET_SCENE,
	LED_CMD_DEL_SCENE,
	LED_CMD_DEL_NODE,
	LED_CMD_CHANGE_STATE,
	LED_CMD_BINDING_ENABLE,
	LED_CMD_BINDING_DISABLE,
	LED_CMD_BINDING_FAIL,
	END_LED_EVT
}led_evt_t;

/******************************************************************************/
/*                             EXPORT FUNCTIONS                               */
/******************************************************************************/

void blink_led_power_on(void);
void ev_handle_refresh_config_led(u16 led_mask);
void ev_handle_button_event(u8 st);
void push_led_event_to_fifo( u8 led_evt, u16 mask );
void ev_handle_input_state_change(u8 channel, u8 st);

void ev_handle_loop_task(void);

#endif /* HANDLE_EV_H_ */
