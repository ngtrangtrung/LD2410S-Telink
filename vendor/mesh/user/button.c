/*
 * button.c
 *
 *  Created on: Sep 12, 2020
 *      Author: DungTran BK
 */

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "../../common/system_time.h"
#include "utilities.h"
#include "radar/radar.h"
#include "button.h"

#include "debug.h"
#ifdef  BUTTON_DBG_EN
	#define DBG_BUTTON_SEND_STR(x)   Dbg_sendString((s8*)x)
	#define DBG_BUTTON_SEND_INT(x)   Dbg_sendInt(x)
	#define DBG_BUTTON_SEND_HEX(x)   Dbg_sendHex(x)
	#define DBG_BUTTON_SEND_BYTE(x)  Dbg_sendOneByteHex(x)
#else
	#define DBG_BUTTON_SEND_STR(x)
	#define DBG_BUTTON_SEND_INT(x)
	#define DBG_BUTTON_SEND_HEX(x)
	#define DBG_BUTTON_SEND_BYTE(x)
#endif

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

typeButton_HandleStateCallbackFunc pvButton_HandleStateCallback = NULL;

typedef struct
{
	u32  press_last_t;
	u8   press_cnt;
	BOOL press_many_time_f;

	u32  start_hold_t;
	u8   prev_state;
	u8   hold_step;
	u8   poll_cnt;

}opt_button_params_t;


opt_button_params_t  opt_button_params =
{
	.press_last_t = 0,
	.press_cnt = 0,
	.press_many_time_f = FALSE,

	.start_hold_t = 0,
	.prev_state = OPTION_BUTTON_RELEASE,
	.hold_step = 0,
	.poll_cnt = 0,
};

#define  OPTION_BUTTON_PRESS_TIME_OUT 	    TIMER_1S
#define  OPTION_BUTTON_PRESS_ELAPSED_TIME 	(clock_time_ms() - opt_button_params.press_last_t)


/******************************************************************************/
/*                       PRIVATE FUNCTION DECLERATION                         */
/******************************************************************************/

static void button_call_handle_function(u8 state);

/******************************************************************************/
/*                            EXPORT FUNCTION                                 */
/******************************************************************************/

/**
 * @func    option_button_init
 * @brief
 * @param   None
 * @retval  None
 */
void option_button_init(void)
{
	gpio_set_func(OPT_BUTTON_IO_PIN, AS_GPIO);
	gpio_set_input_en(OPT_BUTTON_IO_PIN, TRUE);
	gpio_setup_up_down_resistor(OPT_BUTTON_IO_PIN, PM_PIN_PULLUP_10K);
}

/**
 * @func   option_button_callback_init
 * @brief  Init function to handle when option button press/hold/release
 * @param  Function pointer
 * @retval None
 */
void option_button_callback_init(
		typeButton_HandleStateCallbackFunc handleOptionButtonStateCallbackInit)
{
	if(handleOptionButtonStateCallbackInit != NULL){
		pvButton_HandleStateCallback = handleOptionButtonStateCallbackInit;
	}
}

/**
 * @func    option_button_press
 * @brief
 * @param
 * @retval  None
 */
static void option_button_press(void)
{
	if(pvButton_HandleStateCallback != NULL){
		pvButton_HandleStateCallback(START_PRESS);
	}
	// Scan for press many time button
	if(OPTION_BUTTON_PRESS_ELAPSED_TIME < OPTION_BUTTON_PRESS_TIME_OUT){
		opt_button_params.press_cnt++;
	}
	else{
		opt_button_params.press_cnt = 1;
	}
	opt_button_params.press_last_t = clock_time_ms();
	opt_button_params.press_many_time_f = TRUE;
}

/**
 * @func   button_call_handle_function
 * @brief
 * @param  Byte: Option Button State
 * @retval None
 */
static void button_call_handle_function(u8 state)
{
	if(pvButton_HandleStateCallback != NULL){
		pvButton_HandleStateCallback(state);
	}
}

/**
 * @func   option_button_press_times
 * @brief  Count option button press time
 * @param  None
 * @retval None
 */
static void option_button_press_times(void)
{
	if(opt_button_params.press_many_time_f != FALSE){
		if(OPTION_BUTTON_PRESS_ELAPSED_TIME > OPTION_BUTTON_PRESS_TIME_OUT){
			if(OPTION_BUTTON_PRESS_ELAPSED_TIME > TIMER_1S2){
				opt_button_params.press_many_time_f = FALSE;
				return;
			}
			switch (opt_button_params.press_cnt){
			case 1:
				button_call_handle_function(PRESS_ONE_TIME);
				startConfigCommand();
				break;
			case 2:
				button_call_handle_function(PRESS_TWO_TIME);
				break;
			case 3:
				button_call_handle_function(PRESS_THREE_TIME);
				break;
			case 4:
				button_call_handle_function(PRESS_FOUR_TIME);
				break;
			case 5:
				button_call_handle_function(PRESS_FIVE_TIME);
				break;
			case 6:
				button_call_handle_function(PRESS_SIX_TIME);
				break;
			case 8:
				button_call_handle_function(PRESS_EIGHT_TIME);
				break;
			case 10:
				button_call_handle_function(PRESS_TEN_TIME);
				break;
			case 12:
				button_call_handle_function(PRESS_TWELVE_TIME);
				break;
			default:
				break;
			}
			opt_button_params.press_many_time_f = FALSE;
		}
	}
}

/**
 * @func   check_option_button_press
 * @brief  Return status of option button press
 * @param  None
 * @retval Byte: Status
 */
static uint8_t check_option_button_press(void)
{
	if (OPTION_BUTTON_STATE == OPTION_BUTTON_PRESS){
		uint8_t i, boundCnt = 0;
		for (i = 0; i < 4; i++){
			__delay_ms(2);
			if (OPTION_BUTTON_STATE == OPTION_BUTTON_PRESS)
				boundCnt++;
		}
		if (boundCnt == 4){
			return TRUE;
		}
	}
	return FALSE;
}

/**
 * @func   option_button_scan
 * @brief  Option button scan function
 * @param  None
 * @retval None
 */
void option_button_scan(void)
{
	static uint32_t opt_button_scan_timer = 0;

	if(clock_time_get_elapsed_time(opt_button_scan_timer) > TIMER_20MS)
	{
		if((opt_button_params.prev_state == OPTION_BUTTON_RELEASE) && (OPTION_BUTTON_STATE == OPTION_BUTTON_PRESS))
		{
			if(check_option_button_press() == TRUE)
			{
				option_button_press();
				opt_button_params.poll_cnt = 0;
				DBG_BUTTON_SEND_STR("\r\nOption Button Press");
			}
		}else if((opt_button_params.prev_state == OPTION_BUTTON_PRESS) && (OPTION_BUTTON_STATE == OPTION_BUTTON_PRESS))
		{
			//DBG_BUTTON_SEND_STR("\r\nOption Button Hold");
			opt_button_params.poll_cnt++;

			if(opt_button_params.hold_step == 0xFF)
			{
				opt_button_params.start_hold_t = clock_time_ms();
				opt_button_params.hold_step = 0;
			}
			if(OPTION_BUTTON_HOLD_TIME > OPTION_BUTTON_ERROR_HOLD_TIME){
				opt_button_params.hold_step = 0xFE;
			}

			switch(opt_button_params.hold_step)
			{
			case 0:
				if(opt_button_params.poll_cnt >= MIN_POLL_COUNTER_TO_CHANGE_HOLD_STEP)
				{
					opt_button_params.hold_step = 1;
				}
				break;
			case 1:
				if(OPTION_BUTTON_HOLD_TIME > TIMER_2S)
				{
					opt_button_params.hold_step = 2;
					button_call_handle_function(HOLD_2S);
				}
				break;
			case 2:
				if(OPTION_BUTTON_HOLD_TIME > TIMER_5S)
				{
					opt_button_params.hold_step = 3;
					button_call_handle_function(HOLD_5S);
				}
				break;
			case 3:
				if(OPTION_BUTTON_HOLD_TIME > TIMER_10S)
				{
					opt_button_params.hold_step = 4;
					button_call_handle_function(HOLD_10S);
				}
				break;

			case 4:
				if(OPTION_BUTTON_HOLD_TIME > TIMER_15S)
				{
					opt_button_params.hold_step = 5;
					button_call_handle_function(HOLD_15S);
				}
				break;
			}
		}else if((opt_button_params.prev_state == OPTION_BUTTON_RELEASE) &&(OPTION_BUTTON_STATE == OPTION_BUTTON_RELEASE))
		{
			option_button_press_times();
			if((opt_button_params.hold_step >= 2) && (opt_button_params.hold_step < 0xFF)){
				button_call_handle_function(RELEASE);
			}
			button_call_handle_function(NO_PRESS);
			opt_button_params.hold_step = 0xFF;
		}
		opt_button_params.prev_state = OPTION_BUTTON_STATE;
		opt_button_scan_timer = clock_time_ms();
	}
}
// End button.c file

