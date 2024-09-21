/*
 * input.c
 *
 *  Created on: Oct 14, 2020
 *      Author: DungTran BK
 */

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "../../../proj/tl_common.h"
#include "../../common/system_time.h"
#include "../../common/app_provison.h"
#include "utilities.h"
#include "relay.h"
#include "led.h"
#include "input.h"

#include "debug.h"
#ifdef  INPUT_DBG_EN
	#define DBG_INPUT_SEND_STR(x)   Dbg_sendString((s8*)x)
	#define DBG_INPUT_SEND_INT(x)   Dbg_sendInt(x)
	#define DBG_INPUT_SEND_HEX(x)   Dbg_sendHex(x)
	#define DBG_INPUT_SEND_BYTE(x)  Dbg_sendOneByteHex(x)
#else
	#define DBG_INPUT_SEND_STR(x)
	#define DBG_INPUT_SEND_INT(x)
	#define DBG_INPUT_SEND_HEX(x)
	#define DBG_INPUT_SEND_BYTE(x)
#endif

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

typeInput_HandleStateCallbackFunc pvInput_HandleStateCallback = NULL;

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/


static const u16 input_pin_arr[NUMBER_INPUT][2] = {
		{
			INPUT0_IO_PIN, INPUT0_IO_BIT
		},
	#if NUMBER_INPUT > 1
		{
			INPUT1_IO_PIN, INPUT1_IO_BIT
		},
	#endif
	#if NUMBER_INPUT > 2
		{
			INPUT2_IO_PIN, INPUT2_IO_BIT
		},
	#endif
	#if NUMBER_INPUT > 2
		{
			INPUT3_IO_PIN, INPUT3_IO_BIT
		},
	#endif
};

static input_params_t  input_params_st[NUMBER_INPUT];

static u32 input_init_start_t_ms = 0;

/******************************************************************************/
/*                        PRIVATE FUNCTIONS DECLERATION                       */
/******************************************************************************/

static void input_filter_scan(void);
static void input_call_handle_function(u8 channel, u8 event);
static void input_handle_press_many_time(u8 channel);
static void input_caculator_press_many_time(u8 channel);
static void input_scan_channel(u8 channel);


static BOOL enable_handle_input_st_flag[NUMBER_INPUT] = {

		FALSE,
	#if NUMBER_INPUT > 1
		FALSE,
	#endif
	#if NUMBER_INPUT > 2
		FALSE,
	#endif
	#if NUMBER_INPUT > 3
		FALSE,
	#endif
};

/******************************************************************************/
/*                        EXPORT FUNCTIONS DECLERATION                        */
/******************************************************************************/

/**
 * @func   input_filter_scan
 * @brief
 * @param  None
 * @retval None
 */
static void input_filter_scan(void)
{
    foreach(i, NUMBER_INPUT){
    	if(INPUT_STATE(i) == INPUT_ST_ACTIVE){
    		input_params_st[i].present_st = INPUT_ST_ACTIVE;
    		input_params_st[i].signal_active_last_t = clock_time_ms();
    	}
    	else{
			if(clock_time_get_elapsed_time    \
					(input_params_st[i].signal_active_last_t) > PULSE_DETECT_TIME_OUT){
				input_params_st[i].present_st = INPUT_ST_INACTIVE;
				/*
				 * Enable handle input state when first time it's up
				 */
				if((enable_handle_input_st_flag[i] == FALSE)     \
							&& (clock_time_exceed_ms(input_init_start_t_ms, TIMER_3S))){
					enable_handle_input_st_flag[i] = TRUE;
					DBG_INPUT_SEND_STR("\n ---------- enable handle");
				}
			}
    	}
    }
}

/**
 * @func   input_call_handle_function
 * @brief
 * @param  None
 * @retval None
 */
static void input_call_handle_function(u8 channel, u8 event)
{
    if(pvInput_HandleStateCallback != NULL){
    	if(enable_handle_input_st_flag[channel] == TRUE){
			pvInput_HandleStateCallback(channel, event);
			DBG_INPUT_SEND_STR("\n call handle function");
    	}
    }
}

/**
 * @func   input_handle_press_many_time
 * @brief
 * @param  input channel
 * @retval None
 */
static void input_handle_press_many_time(u8 channel)
{
	if(channel < NUMBER_INPUT){
		if(input_params_st[channel].press_many_time_flag == FLAG_ACTIVE){
			if(clock_time_get_elapsed_time  \
					(input_params_st[channel].press_last_t) > INPUT_DOWN_TIMER_EXPIRE){
				if(clock_time_get_elapsed_time(input_params_st[channel].press_last_t)   \
						> (INPUT_DOWN_TIMER_EXPIRE + TIMER_100MS)){
					input_params_st[channel].press_many_time_flag = FLAG_INACTIVE;
					return;
				}

				switch(input_params_st[channel].press_cnt){
					case 1:{
						input_call_handle_function(channel, PRESS_ONE_TIME);
						break;
					}
					case 2:{
						input_call_handle_function(channel, PRESS_TWO_TIME);
						break;
					}
					case 3:{
						input_call_handle_function(channel, PRESS_THREE_TIME);
						break;
					}
					case 4:{
						input_call_handle_function(channel, PRESS_FOUR_TIME);
						break;
					}
					case 5:{
						input_call_handle_function(channel, PRESS_FIVE_TIME);
						break;
					}
					case 10:{
						input_call_handle_function(channel, PRESS_TEN_TIME);
						break;
					}
					default:
						break;
				}
				input_params_st[channel].press_many_time_flag = FLAG_INACTIVE;
				input_params_st[channel].press_cnt = 0;
			}
		}
	}
}

/**
 * @func   input_caculator_press_many_time
 * @brief
 * @param  input channel
 * @retval None
 */
static void input_caculator_press_many_time(u8 channel)
{
	if(channel < NUMBER_INPUT){
		if(clock_time_get_elapsed_time    \
				(input_params_st[channel].press_last_t) < INPUT_DOWN_TIMER_EXPIRE){
			input_params_st[channel].press_cnt++;
		}
		else{
			input_params_st[channel].press_cnt = 1;
		}
		if(input_params_st[channel].press_cnt > 0){
			input_params_st[channel].press_many_time_flag = FLAG_ACTIVE;
		}
		input_params_st[channel].press_last_t = clock_time_ms();
	}
}

/**
 * @func   input_blink_led_start_press
 * @brief
 * @param  Input channel
 * @retval None
 */
static void input_blink_led_start_press(void)
{
	if(ev_get_config_mode_flag() == FALSE){
		LedCommand_str cmd_led = COMMAND_LED_DEFAULT;
		cmd_led.blinkInterval = TIMER_100MS;
		cmd_led.ledColor =    \
					((STATE_DEV_UNPROV == get_provision_state())?(LED_COLOR_RED):(LED_COLOR_BLUE));
		cmd_led.blinkTime = 2;
		LED_pushLedCommandToFifo(cmd_led);
	}
}

/**
 * @func   InputButton_ScanAllChannel
 * @brief
 * @param  Input channel
 * @retval None
 */
static void input_scan_channel(u8 channel)
{
	if((input_params_st[channel].before_st == INPUT_ST_INACTIVE)            \
			&& (input_params_st[channel].present_st == INPUT_ST_ACTIVE)){
		input_params_st[channel].signal_high_last_t = clock_time_ms();
		input_params_st[channel].hold_step = 1;
	}
	else if((input_params_st[channel].before_st == INPUT_ST_ACTIVE)         \
			&& (input_params_st[channel].present_st == INPUT_ST_ACTIVE)){

		if(input_params_st[channel].hold_step == 2){
			if(input_params_st[channel].press_flag == TRUE){
				if(clock_time_get_elapsed_time(input_params_st[channel].press_last_t) > INPUT_DOWN_TIMER_EXPIRE){
					input_call_handle_function(channel, SHORT_HOLD);
					input_params_st[channel].press_many_time_flag = FLAG_INACTIVE;
					input_params_st[channel].press_flag = FALSE;
					DBG_INPUT_SEND_STR("\n _________SHORT HOLD_________");
				}
			}
		}

		switch(input_params_st[channel].hold_step){
			case 1:
				if(clock_time_get_elapsed_time(input_params_st[channel].signal_high_last_t) > DETECT_PRESS_TIME){
					input_caculator_press_many_time(channel);
					input_call_handle_function(channel, START_PRESS);
					input_params_st[channel].press_flag = TRUE;
					input_params_st[channel].hold_step = 2;
					input_blink_led_start_press();
					DBG_INPUT_SEND_STR("\n _________START PRESS_________");
				}
				break;

			case 2:
				if(clock_time_get_elapsed_time(input_params_st[channel].signal_high_last_t) > TIMER_1S2){   // Confirm later
					input_call_handle_function(channel, HOLD_2S);
					input_params_st[channel].hold_step = 3;
				}
				break;

			case 3:
				if(clock_time_get_elapsed_time(input_params_st[channel].signal_high_last_t) > TIMER_5S){
					input_call_handle_function(channel, HOLD_5S);
					input_params_st[channel].hold_step = 4;
				}
				break;

			case 4:
				if(clock_time_get_elapsed_time(input_params_st[channel].signal_high_last_t) > TIMER_10S){
					input_call_handle_function(channel, HOLD_10S);
					input_params_st[channel].hold_step = 5;
				}
				break;

			default: break;
		}
	}
	else if((input_params_st[channel].before_st == INPUT_ST_ACTIVE)         \
			&& (input_params_st[channel].present_st == INPUT_ST_INACTIVE)){
		input_call_handle_function(channel, RELEASE);
	}
	else{
		input_handle_press_many_time(channel);
		input_params_st[channel].hold_step = 0;
	}
	input_params_st[channel].before_st = input_params_st[channel].present_st;
}

/**
 * @func   input_scan_all_channel
 * @brief
 * @param  None
 * @retval None
 */

void input_scan_all_channel(void)
{
	u32 elapsed_t_detect_zr_point
					= clock_time_get_elapsed_time(RL_getLastTimeDetectZeroPoint());
	if((elapsed_t_detect_zr_point <= (PULSE_DETECT_TIME_OUT * 3) >> 2)
													|| (elapsed_t_detect_zr_point >= TIMER_3S)){
		input_filter_scan();
		foreach(i, NUMBER_INPUT){
			input_scan_channel(i);
		}
	}
	else {
		foreach(i, NUMBER_INPUT){
		    enable_handle_input_st_flag[i] = FALSE;
		}
	}
}
/**
 * @func   input_init
 * @brief
 * @param  None
 * @retval None
 */
void input_init(void)
{
	foreach(i, NUMBER_INPUT){
		gpio_set_func(
				input_pin_arr[i][INPUT_PIN_POS], AS_GPIO
			);
		gpio_set_input_en(
				input_pin_arr[i][INPUT_PIN_POS], TRUE
			);
		gpio_setup_up_down_resistor(
				input_pin_arr[i][INPUT_PIN_POS], PM_PIN_PULLDOWN_100K
			);
		input_params_st[i].before_st = input_params_st[i].present_st = INPUT_ST_INACTIVE;
		input_params_st[i].press_cnt = 0;
	}
	input_init_start_t_ms = clock_time_ms();
}

/**
 * @func   input_callback_init
 * @brief
 * @param  Function pointer
 * @retval None
 */
void input_callback_init(typeInput_HandleStateCallbackFunc callbackFunc)
{
	if(callbackFunc != NULL){
		pvInput_HandleStateCallback = callbackFunc;
	}
}
// End File
