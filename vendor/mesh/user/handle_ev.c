/*
 * handle_ev.c
 *
 *  Created on: Oct 5, 2020
 *      Author: DungTran BK
 */

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "../../common/system_time.h"
#include "../../common/app_provison.h"
#include "../../common/light.h"
#include "../../common/generic_model.h"
#include "../../common/lighting_model.h"
#include "../../common/vendor_model.h"

#include "utilities.h"
#include "radar/radar.h"
#include "execution_scene.h"
#include "led.h"
#include "factory_reset.h"
#include "button.h"
#include "key_report.h"
#include "in_out.h"
#include "../user/led.h"
#include "handle_ev.h"
#include "../user/relay.h"
#include "binding_ctl.h"

#include "debug.h"
#ifdef  HANDLE_EV_DBG_EN
	#define DBG_HANDLE_EV_SEND_STR(x)   Dbg_sendString((s8*)x)
	#define DBG_HANDLE_EV_SEND_INT(x)   Dbg_sendInt(x)
	#define DBG_HANDLE_EV_SEND_HEX(x)   Dbg_sendHex(x)
	#define DBG_HANDLE_EV_SEND_BYTE(x)  Dbg_sendOneByteHex(x)
#else
	#define DBG_HANDLE_EV_SEND_STR(x)
	#define DBG_HANDLE_EV_SEND_INT(x)
	#define DBG_HANDLE_EV_SEND_HEX(x)
	#define DBG_HANDLE_EV_SEND_BYTE(x)
#endif

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

static u8 input_btn_last_state[NUMBER_INPUT] = {
				NO_PRESS,
		#if NUMBER_INPUT > 1
				NO_PRESS
		#endif
		#if NUMBER_INPUT > 2
				NO_PRESS
		#endif
		#if NUMBER_INPUT > 3
				NO_PRESS
		#endif
};

#define MIN_CONFIRM_TIME     2000
#define MAX_CONFIRM_TIME     7000

typedef struct {
	BOOL en_confirm_rst;
	u32  confirm_start_t_ms;
}input_rst_node_t;


static input_rst_node_t  input_rst_node_st = {
		.en_confirm_rst     = FALSE,
		.confirm_start_t_ms = 0
};


enum {
	SETUP_INPUT,
	SETUP_POWER_ON_OFF,
	END_SETUP
};

typedef struct {
	u8 flag;
	u8 mode;
	u8 input_mode[NUMBER_INPUT];
	u8 power_onoff[LIGHT_CNT];
	u32 start_t_ms;
}config_params_t;

static config_params_t config_params_st = {
		.flag = FALSE,
};

#define CONFIG_TIMEOUT_DEFAULT     TIMER_15S

/******************************************************************************/
/*                        PRIVATE FUNCTIONS DECLERATION                       */
/******************************************************************************/

static void ev_push_refresh_led_cmd_to_fifo(void);

/******************************************************************************/
/*                        EXPORT FUNCTIONS DECLERATION                        */
/******************************************************************************/

/**
 * @func   ev_get_config_mode_flag
 * @brief
 * @param  None
 * @retval None
 */
BOOL ev_get_config_mode_flag(void)
{
	return config_params_st.flag;
}

/**
 * @func   blink_led_power_on
 * @brief
 * @param  None
 * @retval None
 */
void blink_led_power_on(void)
{
	switch(get_provision_state()){
		case STATE_DEV_PROVED:
			DBG_HANDLE_EV_SEND_STR("\n STATE_DEV_PROVED");
			LED_pushNormalBlinkLedCmdToFifo(6, LED_COLOR_PINK);
			break;
		case STATE_DEV_PROVING:
		case STATE_DEV_UNPROV:
			DBG_HANDLE_EV_SEND_STR("\n STATE_DEV_UNPROV");
			LED_pushNormalBlinkLedCmdToFifo(6, LED_COLOR_RED);
			break;
	}
}

/**
 * @func   ev_handle_refresh_config_led
 * @brief
 * @param
 * @retval None
 */
void ev_handle_refresh_config_led(u16 led_mask)
{
	if(config_params_st.flag == FALSE){
		switch(get_provision_state())
		{
			case STATE_DEV_PROVED:
				LED_setColor(0, LED_COLOR_BLUE);
				break;
			case STATE_DEV_PROVING:
				LED_setColor(0, LED_COLOR_RED);
				break;
			case STATE_DEV_UNPROV:
				LED_setColor(0, LED_COLOR_RED);
				break;
		}
	}
	else {
		LED_setColor(0, LED_COLOR_PINK);
	}
}

/**
 * @func   ev_check_evt_input_rst_node
 * @brief
 * @param  None
 * @retval None
 */
static void ev_modify_evt_input_rst_node(void)
{
	if(input_rst_node_st.en_confirm_rst == TRUE){
		if(clock_time_exceed_ms(input_rst_node_st.confirm_start_t_ms, MAX_CONFIRM_TIME)){
			DBG_HANDLE_EV_SEND_STR("\nTIME_OUT RST");
			input_rst_node_st.en_confirm_rst = FALSE;
    	}
	}
}

/**
 * @func   ev_handle_refresh_config_led
 * @brief
 * @param
 * @retval None
 */
void ev_handle_input_state_change(u8 channel, u8 st)
{
	if(channel == 0){
	    ev_modify_evt_input_rst_node();
	}
	if(config_params_st.flag == TRUE){
		input_rst_node_st.en_confirm_rst = FALSE;
	}

	if(channel < NUMBER_INPUT){
		switch(st){
			case START_PRESS:
			{
				if(io_handle_input_btn_st   \
						(channel, st, config_params_st.flag) == SUCCESS){
					mesh_tx_cmd_lightness_st(
								channel,
								ele_adr_primary + channel,
								GATEWAY_UNICAST_ADDR,
								LIGHTNESS_STATUS,
								0,
								0
							);
				}
				break;
			}
			case SHORT_HOLD:
			{
				DBG_HANDLE_EV_SEND_STR("\nSHORT_HOLD: ");
				DBG_HANDLE_EV_SEND_INT(channel);
				break;
			}
			case PRESS_ONE_TIME:
			{



//				if(config_params_st.flag == TRUE){
//					if(config_params_st.mode == SETUP_INPUT){
//						config_params_st.input_mode[channel] = IN_TOGGLE_SW;
//					}
//					else if(config_params_st.mode == SETUP_POWER_ON_OFF){
//						if(channel < LIGHT_CNT){
//							config_params_st.power_onoff[channel] = G_ON_POWERUP_OFF;
//						}
//					}
//					// RST Config Time
//					config_params_st.start_t_ms = clock_time_ms();
//					LED_PushFastBlinkLedCmdToFifo(2, LED_COLOR_PINK);
//				}
//				else{
//					if(channel == 0){
//						if(input_rst_node_st.en_confirm_rst == TRUE){
//							u32 temp = clock_time_get_elapsed_time(input_rst_node_st.confirm_start_t_ms);
//							if((temp >= MIN_CONFIRM_TIME)     \
//											&& (temp <= MAX_CONFIRM_TIME)){
//								input_rst_node_st.en_confirm_rst = FALSE;
//								setup_factory_reset_with_delay(TRUE, TIMER_1S5);
//							}
//						}
//					}
//				DBG_HANDLE_EV_SEND_STR("\nBTN_KEY_PRESS_1_TIME");
					key_report_send(channel, BTN_KEY_PRESS_1_TIME);
//				}
				break;
			}
			case PRESS_TWO_TIME:
			{
				if(config_params_st.flag == TRUE){
					if(config_params_st.mode == SETUP_INPUT){
						config_params_st.input_mode[channel] = IN_ANY_CHANGE_SW;
					}
					else if(config_params_st.mode == SETUP_POWER_ON_OFF){
						if(channel < LIGHT_CNT){
							config_params_st.power_onoff[channel] = G_ON_POWERUP_SAVE;
						}
					}
					// RST Config Time
					config_params_st.start_t_ms = clock_time_ms();
					LED_PushFastBlinkLedCmdToFifo(4, LED_COLOR_PINK);
				}
				else{
					key_report_send(channel, BTN_KEY_PRESS_2_TIMES);
				}
				break;
			}
			case PRESS_THREE_TIME:
			{
				if(config_params_st.flag == TRUE){
					if(config_params_st.mode == SETUP_INPUT){
						config_params_st.input_mode[channel] = IN_MOMENTARY_SW;
					}
					else if(config_params_st.mode == SETUP_POWER_ON_OFF){
						if(channel < LIGHT_CNT){
							config_params_st.power_onoff[channel] = G_ON_POWERUP_ON;
						}
					}
					// RST Config Time
					config_params_st.start_t_ms = clock_time_ms();
					LED_PushFastBlinkLedCmdToFifo(6, LED_COLOR_PINK);
				}
				else {
					key_report_send(channel, BTN_KEY_PRESS_3_TIMES);
				}
				break;
			}
			case PRESS_FOUR_TIME:
			{
				DBG_HANDLE_EV_SEND_STR("\nPRESS FOUR: ");
				DBG_HANDLE_EV_SEND_INT(channel);
				break;
			}
			case PRESS_FIVE_TIME:
			{
				if(config_params_st.flag == FALSE){
					key_report_send(channel, BTN_KEY_PRESS_5_TIMES);
					if(channel == 0){
						input_rst_node_st.en_confirm_rst = TRUE;
						input_rst_node_st.confirm_start_t_ms = clock_time_ms();
						// LED
						LedCommand_str cmd_led = COMMAND_LED_DEFAULT;
						cmd_led.blinkInterval = 250;
						cmd_led.ledColor = LED_COLOR_RED;
						cmd_led.blinkTime = 8;
						LED_pushLedCommandToFifo(cmd_led);
					}
				}
				break;
			}
			case PRESS_TEN_TIME:
			{
				break;
			}
			case HOLD_2S:
			{
				LedCommand_str cmd_led = COMMAND_LED_DEFAULT;
				cmd_led.blinkInterval = 100;
				cmd_led.ledColor =    \
						((STATE_DEV_UNPROV == get_provision_state())?(LED_COLOR_RED):(LED_COLOR_BLUE));
				cmd_led.blinkTime = 4;
				LED_pushLedCommandToFifo(cmd_led);
				break;
			}
			case HOLD_5S:
			{
				DBG_HANDLE_EV_SEND_STR("\nPRESS HOLD 5S: ");
				DBG_HANDLE_EV_SEND_INT(channel);
				break;
			}
			case HOLD_10S:
			{
				DBG_HANDLE_EV_SEND_STR("\nPRESS HOLD 10S: ");
				DBG_HANDLE_EV_SEND_INT(channel);
				break;
			}
			case RELEASE:
			{
				DBG_HANDLE_EV_SEND_STR("\nRELEASE: ");
				DBG_HANDLE_EV_SEND_INT(channel);
				if(input_btn_last_state[channel] == HOLD_2S){
					key_report_send(channel, BTN_KEY_HOLD_2_SECONDS);
				}
				if(io_handle_input_btn_st  \
						(channel, st, config_params_st.flag) == SUCCESS){
					mesh_tx_cmd_lightness_st(
								channel,
								ele_adr_primary + channel,
								GATEWAY_UNICAST_ADDR,
								LIGHTNESS_STATUS,
								0,
								0
							);
				}
				break;
			}
			default: break;
		}
		input_btn_last_state[channel] = st;
	}
}

/**
 * @func   ev_setup_config_params
 * @brief
 * @param
 * @retval None
 */
void ev_setup_config_params(u8 mode_setup)
{
	config_params_st.flag = TRUE;
	config_params_st.start_t_ms = clock_time_ms();
	config_params_st.mode = mode_setup;

	io_params_t temp_io_params[NUMBER_INPUT];
	io_get_config_params((u8*)&temp_io_params);
	foreach(i, NUMBER_INPUT){
		config_params_st.input_mode[i] = temp_io_params[i].in_mode;
	}

	foreach(i, LIGHT_CNT){
		config_params_st.power_onoff[i] = model_sig_g_power_onoff.on_powerup[i];
	}
}
/**
 * @func   ev_push_refresh_led_cmd_to_fifo
 * @brief
 * @param
 * @retval None
 */
static void ev_push_refresh_led_cmd_to_fifo(void)
{
	LedCommand_str cmd_led;
	cmd_led.ledMask = MAX_U16;
	cmd_led.ledMode  = LED_MODE_REFRESH;
	LED_pushLedCommandToFifo(cmd_led);
}

/**
 * @func   ev_handle_config_time_expire
 * @brief
 * @param
 * @retval None
 */
void ev_handle_loop_task(void)
{
	if(config_params_st.flag == TRUE){
		if(clock_time_exceed_ms    \
				(config_params_st.start_t_ms, CONFIG_TIMEOUT_DEFAULT)){
			config_params_st.flag = FALSE;
			config_params_st.mode = END_SETUP;
			//ev_push_refresh_led_cmd_to_fifo();
			LED_pushNormalBlinkLedCmdToFifo(4, LED_COLOR_RED);
		}
	}
}

static BOOL en_force_send_all_config = TRUE;

/**
 * @func   ev_handle_button_event
 * @brief
 * @param
 * @retval None
 */
void ev_handle_button_event(u8 st)
{
	static u8 opt_button_last_st = ST_UNKNOWN;

	switch(st)
	{
		case START_PRESS:{


//			static uint8_t status = 0;
//			status = !status;
//			if(status){
//				LED_setColorRed(0);
//				RL_changeRlState(0, Level_High);
//				binding_control(0, G_ON);
////				LED_setColorRed(1);
////				LED_setColorRed(2);
//			}else{
//				LED_setColorBlue(0);
//				RL_changeRlState(0, Level_Low);
//				binding_control(0, G_OFF);
////				LED_setColorBlue(1);
////				LED_setColorBlue(2);

//					__delay_ms(100);
//			}
//			if(config_params_st.flag == TRUE){
//				BOOL change_flag = FALSE;
//				// Save all config
//				if(config_params_st.mode == SETUP_POWER_ON_OFF){
//					foreach(i, LIGHT_CNT){
//						if(config_params_st.power_onoff[i] != model_sig_g_power_onoff.on_powerup[i]){
//							u8 *par = (u8*)&config_params_st.power_onoff[i];
//							u8 par_len = 1;
//							mesh_cb_fun_par_t cb_par;
//							cb_par.adr_src = GATEWAY_UNICAST_ADDR;
//							cb_par.adr_dst = ele_adr_primary + i;
//							u8 idx_out = 0;
//							cb_par.model = mesh_find_ele_resource_in_model(
//														ele_adr_primary + i,
//														SIG_MD_G_POWER_ONOFF_SETUP_S,
//														TRUE,
//														(u8*)&idx_out,
//														TRUE
//							                       );
//							cb_par.op_rsp = G_ON_POWER_UP_STATUS;
//							cb_par.op = G_ON_POWER_UP_SET;
//							cb_par.model_idx = i;
//							// Save New Config
//							mesh_cmd_sig_g_on_powerup_set(par, par_len, (mesh_cb_fun_par_t*)&cb_par);
//							change_flag = TRUE;
//						}
//					}
//				}
//				else if(config_params_st.mode == SETUP_INPUT){
//					foreach(i, NUMBER_INPUT){
//					    if(io_set_input_mode(i, config_params_st.input_mode[i]) == TRUE){
//					    	change_flag = TRUE;
//					    }
//					}
//				}
//				config_params_st.flag = FALSE;
//				if(change_flag == TRUE){
//					LED_pushNormalBlinkLedCmdToFifo(4, LED_COLOR_BLUE);
//				}
//				else {
//					LED_PushFastBlinkLedCmdToFifo(8, LED_COLOR_RED);
//				}
//				en_force_send_all_config = FALSE;
//			}
//			else{
//				ev_push_refresh_led_cmd_to_fifo();
//				en_force_send_all_config = TRUE;
//			}
			break;
		}
		case RELEASE:
			break;
	}
	opt_button_last_st = st;
}
/**
 * @func   ev_handle_button_event
 * @brief
 * @param
 * @retval None
 */
void push_led_event_to_fifo( u8 led_evt, u16 mask )
{
	if(led_evt >= END_LED_EVT){
		DBG_HANDLE_EV_SEND_STR("\n LED event invalid");
		return;
	}
	LedCommand_str cmd_led = COMMAND_LED_DEFAULT;

    switch( led_evt )
    {
		case LED_POWER_ON:
		{
			cmd_led.ledColor =
					((STATE_DEV_UNPROV == get_provision_state())?(LED_COLOR_RED):(LED_COLOR_PINK));
			cmd_led.blinkInterval = 300;
			cmd_led.blinkTime = 6;
			DBG_HANDLE_EV_SEND_STR("\n LED_POWER_ON");
			break;
		}
		case LED_OTA_FAIL:
		{
			cmd_led.ledColor = LED_COLOR_RED;
			cmd_led.blinkInterval = 1000;
			cmd_led.blinkTime = 6;
			DBG_HANDLE_EV_SEND_STR("\n LED_OTA_FAIL");
			break;
		}
		case LED_OTA_SUCESS:
		{
			cmd_led.ledColor = LED_COLOR_BLUE;
			cmd_led.blinkInterval = 1000;
			cmd_led.blinkTime = 6;
			DBG_HANDLE_EV_SEND_STR("\n LED_OTA_SUCESS");
			break;
		}
		case LED_FAIL_ADD_APPKEY:
		case LED_PROVISION_FAIL:
		{
			cmd_led.ledColor = LED_COLOR_RED;
			cmd_led.blinkInterval = 300;
			cmd_led.blinkTime = 6;
			DBG_HANDLE_EV_SEND_STR("\n LED_PROVISION_FAIL");
			break;
		}
		case LED_SUC_ADD_APPKEY:
		case LED_PROVISION_SUCCESS:
		{
			cmd_led.ledColor = LED_COLOR_PINK;
			cmd_led.blinkInterval = 300;
			cmd_led.blinkTime = 6;
			DBG_HANDLE_EV_SEND_STR("\n LED_PROVISION_SUCCESS");
			break;
		}
		case LED_CMD_SET_SUBSCRIPTION:
		case LED_CMD_DEL_SUBSCRIPTION:
		{
			cmd_led.ledColor = LED_COLOR_BLUE;
			cmd_led.blinkInterval = 300;
			cmd_led.blinkTime = 4;
			DBG_HANDLE_EV_SEND_STR("\n LED_CMD_SET_SUBSCRIPTION");
			break;
		}

		case LED_CMD_SET_SCENE:
		case LED_CMD_DEL_SCENE:
		{
			cmd_led.ledColor = LED_COLOR_BLUE;
			cmd_led.blinkInterval = 300;
			cmd_led.blinkTime = 4;
			DBG_HANDLE_EV_SEND_STR("\n LED_CMD_SET_SCENE");
			break;
		}
		case LED_CMD_BINDING_ENABLE:
			cmd_led.ledColor = LED_COLOR_BLUE;
			cmd_led.blinkInterval = 300;
			cmd_led.blinkTime = 4;
			DBG_HANDLE_EV_SEND_STR("\n LED_CMD_BINDING_ENABLE");
			break;

		case LED_CMD_BINDING_DISABLE:
			cmd_led.ledColor = LED_COLOR_RED;
			cmd_led.blinkInterval = 300;
			cmd_led.blinkTime = 4;
			DBG_HANDLE_EV_SEND_STR("\n LED_CMD_BINDING_DISABLE");
			break;

		case LED_CMD_BINDING_FAIL:
			cmd_led.ledColor = LED_COLOR_RED;
			cmd_led.blinkInterval = 300;
			cmd_led.blinkTime = 4;
			DBG_HANDLE_EV_SEND_STR("\n LED_CMD_BINDING_FAIL");
			break;

		case LED_CMD_DEL_NODE:{
			cmd_led.ledColor = LED_COLOR_PINK;
			cmd_led.blinkInterval = 200;
			cmd_led.blinkTime = 4;
			cmd_led.lastState = LAST_STATE_COLOR_NONE;
			DBG_HANDLE_EV_SEND_STR("\n LED_CMD_DEL_NODE");
			break;
		}
		case LED_CMD_CHANGE_STATE:
			cmd_led.blinkInterval = 100;
			cmd_led.blinkTime = 1;
			break;

		default:{
			DBG_HANDLE_EV_SEND_STR("\n UNKNOWN LED EVENT");
			break;
		}
    }
	LED_pushLedCommandToFifo(cmd_led);
}

// End File
