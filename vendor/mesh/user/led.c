/*
 * Copyright (c) 2019
 * Lumi, JSC.
 * All Rights Reserved
 *
 *
 * Description:
 *
 * Author: DungTran
 *
 * Last Changed By:  $Author: DungTran $
 * Revision:         $Revision: 1.0 $
 * Last Changed:     $Date: 12/17/19 $
 */
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "../../../proj/tl_common.h"
#include "../../common/system_time.h"
#include "fifo.h"
#include "led.h"

#define NUMBER_LED       1

#include "debug.h"
#ifdef  LED_DBG_EN
	#define DBG_LED_SEND_STR(x)   Dbg_sendString((s8*)x)
	#define DBG_LED_SEND_INT(x)   Dbg_sendInt(x)
	#define DBG_LED_SEND_HEX(x)   Dbg_sendHex(x)
	#define DBG_LED_SEND_BYTE(x)  Dbg_sendOneByteHex(x)
#else
	#define DBG_LED_SEND_STR(x)
	#define DBG_LED_SEND_INT(x)
	#define DBG_LED_SEND_HEX(x)
	#define DBG_LED_SEND_BYTE(x)
#endif

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

typeLed_HandleRefreshLedCallbackFunc pvLed_HandleRefreshLed = NULL;

const uint16_t ledAddr[NUMBER_LED][2] ={
		{
			CFG_LED_RED_IO_PIN, CFG_LED_BLUE_IO_PIN
		},
#if NUMBER_LED > 1
		{
			CFG_LED_RED_IO_PIN1, CFG_LED_BLUE_IO_PIN1
		},
#endif
};

enum {
	LED_L = 1,
	LED_H = 0
};

static LedBlinkInit_str toggleLedArr[NUMBER_LED];
static LedCommand_str cmdIsRunning = COMMAND_LED_DEFAULT;
static Fifo_t fifoLedCommands;
static LedCommand_str bufferLedCommands[BUF_LED_CMD_SIZE];

static refresh_led_delay_t refresh_led_delay_st = { 0, 0, 0xFFFF };

/******************************************************************************/
/*                        PRIVATE FUNCTIONS DECLERATION                       */
/******************************************************************************/


static void LED_setColorPink(uint8_t ledNbr);

static void LED_handleRefreshDelay(void);

#define CFG_LED_RED_STATE    ( gpio_read(ledAddr[ledNbr][LED_RED_IDX]) >> CFG_LED_RED_IO_BIT )
#define CFG_LED_BLUE_STATE   ( gpio_read(ledAddr[ledNbr][LED_BLUE_IDX]) >> CFG_LED_BLUE_IO_BIT )

/******************************************************************************/
/*                        EXPORT FUNCTIONS DECLERATION                        */
/******************************************************************************/

/**
 * @func   LED_blinkLedRed
 * @brief
 * @param  BYTE: Led Number
 * @retval None
 */
void LED_blinkLedRed(uint8_t ledNbr)
{
	gpio_toggle(ledAddr[ledNbr][LED_RED_IDX]);
}

/**
 * @func   LED_blinkLedBlue
 * @brief
 * @param  BYTE: Led Number
 * @retval None
 */
void LED_blinkLedBlue(uint8_t ledNbr)
{
	gpio_toggle(ledAddr[ledNbr][LED_BLUE_IDX]);
}

/**
 * @func   LED_blinkPink
 * @brief
 * @param  BYTE: Led Number
 * @retval None
 */
void LED_blinkLedPink(uint8_t ledNbr)
{
	if(CFG_LED_RED_STATE == CFG_LED_BLUE_STATE){
		gpio_toggle(ledAddr[ledNbr][LED_RED_IDX]);
		gpio_toggle(ledAddr[ledNbr][LED_BLUE_IDX]);
	}
	else{
		gpio_toggle(ledAddr[ledNbr][LED_RED_IDX]);
	}
}

/**
 * @func   LED_setColor
 * @brief
 * @param  BYTE: Led Number
 *         BYTE: Led Color
 * @retval None
 */
void LED_setColor(uint8_t ledNbr, LedColor_enum color)
{
	if(ledNbr < NUMBER_LED){
		switch(color){
		case LED_COLOR_RED:
			LED_setColorRed(ledNbr);
		    break;

		case LED_COLOR_BLUE:
			LED_setColorBlue(ledNbr);
			break;

		case LED_COLOR_PINK:
			LED_setColorPink(ledNbr);
			break;

		case LED_COLOR_NONE:
			LED_off(ledNbr);
			break;
		}
	}
}
/**
 * @func   LED_setColorLastState
 * @brief
 * @param  BYTE: Led Number
 *         BYTE: Led Color
 * @retval None
 */
static void LED_setColorLastState(uint8_t ledNbr, LedColor_enum color)
{
	if(ledNbr < NUMBER_LED){
		switch(color){
		case LAST_STATE_ON_RED:
			LED_setColorRed(ledNbr);
		    break;

		case LAST_STATE_ON_BLUE:
			LED_setColorBlue(ledNbr);
			break;

		case LAST_STATE_ON_PINK:
			LED_setColorPink(ledNbr);
			break;

		case LAST_STATE_COLOR_NONE:
			LED_off(ledNbr);
			break;
		}
	}
}

/**
 * @func   LED_setColorRed
 * @brief
 * @param  BYTE: Led Number
 * @retval None
 */
void LED_setColorRed(u8 ledNbr)
{
	if(ledNbr < NUMBER_LED){
		foreach(i, 2){
			if(i == LED_RED_IDX){
				gpio_write(ledAddr[ledNbr][i], LED_H);
			}else{
				gpio_write(ledAddr[ledNbr][i], LED_L);
			}
		}
	}
}

/**
 * @func   LED_setColorGreen
 * @brief
 * @param  BYTE: Led Number
 * @retval None
 */
static void LED_setColorPink(uint8_t ledNbr)
{
	if(ledNbr < NUMBER_LED){
		gpio_write(ledAddr[ledNbr][LED_RED_IDX], LED_H);
		gpio_write(ledAddr[ledNbr][LED_BLUE_IDX], LED_H);
	}
}

/**
 * @func   LED_setColorBlue
 * @brief
 * @param  BYTE: Led Number
 * @retval None
 */
void LED_setColorBlue(uint8_t ledNbr)
{
	if(ledNbr < NUMBER_LED){
		foreach(i, 2){
			if(i == LED_BLUE_IDX){
				gpio_write(ledAddr[ledNbr][i], LED_H);
			}else{
				gpio_write(ledAddr[ledNbr][i], LED_L);
			}
		}
	}
}

/**
 * @func   LED_off
 * @brief
 * @param  BYTE: Led Number
 * @retval None
 */
void LED_off(uint8_t ledNbr)
{
	gpio_write(ledAddr[ledNbr][LED_RED_IDX], LED_L);
	gpio_write(ledAddr[ledNbr][LED_BLUE_IDX], LED_L);
}
/**
 * @func   LED_offAll
 * @brief
 * @param  BYTE: Led Number
 * @retval None
 */
void LED_offAll(void)
{
	foreach(i, NUMBER_LED){
		LED_off(i);
	}
}
/**
 * @func   LED_pushLedCommandToFifo
 * @brief  Push led command to led fifo buffer
 * @param  LED_CmdTypeDef
 * @retval None
 */
uint8_t LED_pushLedCommandToFifo(LedCommand_str ledCmd)
{
	if(FifoPush(&fifoLedCommands, &ledCmd))
		return TRUE;
	return FALSE;
}

/**
 * @func   LED_PushFastBlinkLedCmdToFifo
 * @brief  None
 * @param
 * @retval None
 */
void LED_PushFastBlinkLedCmdToFifo(uint8_t blinkTime, LedColor_enum color)
{
	LedCommand_str ledCmd = COMMAND_LED_DEFAULT;
	ledCmd.ledColor = color;
	ledCmd.blinkTime = blinkTime;
	ledCmd.blinkInterval = 180;
	(void)LED_pushLedCommandToFifo(ledCmd);
}
/**
 * @func   LED_PushNormalBlinkLedCmdToFifo
 * @brief  None
 * @param
 * @retval None
 */
void LED_pushNormalBlinkLedCmdToFifo(uint8_t blinkTime, LedColor_enum color)
{
	LedCommand_str ledCmd = COMMAND_LED_DEFAULT;
	ledCmd.ledColor = color;
	ledCmd.blinkTime = blinkTime;
	(void)LED_pushLedCommandToFifo(ledCmd);
}
/**
 * @func   LED_init
 * @brief  Initialization for led module
 * @param  None
 * @retval None
 */
void LED_init(void)
{
	for(uint8_t i = 0; i < NUMBER_LED; i++){
	    for(u8 j = 0; j < 3; j++){
	    	gpio_set_func(ledAddr[i][j], AS_GPIO);
	        gpio_set_output_en(ledAddr[i][j], 1);
	    }
		toggleLedArr[i].toggleLedInterval = DEFAULT_BLINK_INTERVAL;
		toggleLedArr[i].toggleLedFlag = BLINK_IDLE;
	}
	LED_offAll();
	FifoInit(&fifoLedCommands, &bufferLedCommands, sizeof(LedCommand_str), BUF_LED_CMD_SIZE);
}
/**
 * @func   LED_refreshCallbackInit
 * @brief  Initialization handle function to refresh led
 * @param  Function pointer
 * @retval None
 */
void LED_refreshCallbackInit(typeLed_HandleRefreshLedCallbackFunc refreshLedCallbackInit)
{
	if(refreshLedCallbackInit != NULL){
		pvLed_HandleRefreshLed = refreshLedCallbackInit;
	}
}

/**
 * @func   LED_Refresh
 * @brief  Handle function to refresh led
 * @param  BYTE: ledMask
 * @retval None
 */
void LED_refresh(uint16_t ledMask)
{
	if(pvLed_HandleRefreshLed != NULL){
		pvLed_HandleRefreshLed(ledMask);
	}
}
/**
 * @func   LED_toggle
 * @brief  Inverting the led state
 * @param  BYTE: Led Number
 * @retval None
 */
void LED_toggle(uint8_t ledNbr)
{
	if(ledNbr <= NUMBER_LED){
		switch(toggleLedArr[ledNbr].toggleLedState){
			case LED_COLOR_RED:{
				LED_blinkLedRed(ledNbr);
				break;
			}
			case LED_COLOR_BLUE:{
				LED_blinkLedBlue(ledNbr);
				break;
			}
			case LED_COLOR_PINK:{
				LED_blinkLedPink(ledNbr);
				break;
			}
			default: // Unknown Led Color
				break;
		}
	}
}
/**
 * @func   LED_toggleHandle
 * @brief  Handle Blink Led
 * @param  None
 * @retval None
 */
void LED_toggleHandle(void)
{
	uint8_t i;
	static uint32_t ledBlinkScanTimer = 0;
	uint8_t enableRefreshLed = false;

	if(clock_time_exceed_ms(ledBlinkScanTimer, 10) > 0){
		for(i=0; i<= NUMBER_LED; i++){
			if(toggleLedArr[i].toggleLedFlag == BLINK_ACTIVE){
				if(toggleLedArr[i].toggleLedTimes > 0){
					if(clock_time_exceed_ms(toggleLedArr[i].toggleledLastTime,toggleLedArr[i].toggleLedInterval) > 0){
						toggleLedArr[i].toggleledLastTime = clock_time_ms();
						toggleLedArr[i].toggleLedTimes--;
						if(toggleLedArr[i].toggleLedTimes == 0){
							toggleLedArr[i].toggleLedFlag = BLINK_IDLE;
							if(FifoIsEmpty(&fifoLedCommands) == TRUE){
								switch(toggleLedArr[i].toggleLedLastState){
								case LAST_STATE_ON_RED:
								case LAST_STATE_ON_BLUE:
								case LAST_STATE_COLOR_NONE:
									LED_setColorLastState(i, toggleLedArr[i].toggleLedLastState);
									break;
								case LAST_STATE_REFRESH_LED:
								default:
									enableRefreshLed = true;
									break;
								}
							}
						}else{
							LED_toggle(i);
						}
					}
				}else{
					toggleLedArr[i].toggleLedFlag = BLINK_IDLE;
				}
			}
		}
		ledBlinkScanTimer = clock_time_ms();
		if(enableRefreshLed == TRUE){
			LED_refresh(0xFFFF);
		}
	}
}

/**
 * @func   LED_Blink
 * @brief  Change all led blink parameters
 * @param  BYTE: ledMask, blinkTimes, lastState, ledColor
 * @retval None
 */
void LED_blink(uint16_t ledMask, uint8_t blinkTimes,LedLastState_enum lastState,       \
		             LedColor_enum ledColor, uint16_t blinkInterval)
{
	for(uint8_t i = 0; i < NUMBER_LED; i++){
		if(((ledMask>>i)&0x01) == 1){
			LED_off(i);
			toggleLedArr[i].toggleLedState     = ledColor;
			toggleLedArr[i].toggleledLastTime  = clock_time_ms();
			toggleLedArr[i].toggleLedFlag      = BLINK_ACTIVE;
			toggleLedArr[i].toggleLedTimes     = blinkTimes;
			toggleLedArr[i].toggleLedLastState = lastState;
			toggleLedArr[i].toggleLedInterval  = blinkInterval;
		}
	}
}

/**
 * @func   LED_getBlinkLedFlag
 * @brief
 * @param  None
 * @retval None
 */
uint8_t LED_getBlinkLedFlag(void)
{
	LedBlinkFlag_enum blinkFlag = BLINK_IDLE;
	for(u8 i = 0; i < NUMBER_LED; i++){
		if(toggleLedArr[i].toggleLedFlag == 1){
			blinkFlag = BLINK_ACTIVE;
			break;
		}
	}
	return blinkFlag;
}

/**
 * @func   LED_handleEventFunction
 * @brief  Handle led Command if blink flag = BLINK_IDLE (No Command is running)
 * @param  None
 * @retval None
 */
void LED_handLeEventFunction(void)
{
	uint8_t i;
	LedBlinkFlag_enum blinkFlag = BLINK_IDLE;

	for(i = 0; i < NUMBER_LED; i++){
		if(toggleLedArr[i].toggleLedFlag == 1){
			blinkFlag = BLINK_ACTIVE;
			break;
		}
	}

	if(blinkFlag == BLINK_IDLE){
		// Refresh delay
		LED_handleRefreshDelay();
		// Check FIFO Led
		if (FifoIsEmpty(&fifoLedCommands) == FALSE){
			if(FifoPop(&fifoLedCommands, &cmdIsRunning)){
				if(cmdIsRunning.ledMode == LED_MODE_REFRESH){
					if(pvLed_HandleRefreshLed != NULL){
						pvLed_HandleRefreshLed(cmdIsRunning.ledMask);
					}
				}else if(cmdIsRunning.ledMode == LED_MODE_BLINK){
					LED_blink(cmdIsRunning.ledMask, cmdIsRunning.blinkTime, \
			        		cmdIsRunning.lastState, cmdIsRunning.ledColor, cmdIsRunning.blinkInterval);
				}else if(cmdIsRunning.ledMode == LED_MODE_ON){
					switch(cmdIsRunning.ledColor){
						case LED_COLOR_RED:
						case LED_COLOR_BLUE:
						case LED_COLOR_PINK:
							if(cmdIsRunning.ledMask != 0){
								for(i=0; i < NUMBER_LED; i++){
									if(((cmdIsRunning.ledMask >> i)&0x01) == 1){
										LED_setColor(i, cmdIsRunning.ledColor);
									}
								}
							}
							break;
						default:
							//Unknown led color, so do not change the led color
							break;
						}
				}else if(cmdIsRunning.ledMode == LED_MODE_OFF){
					if(cmdIsRunning.ledMask != 0){
						for(i=0; i < NUMBER_LED; i++){
							if(((cmdIsRunning.ledMask >> i)&0x01) == 1){
								LED_off(i);
							}
						}
					}
				}
			}
		}
	}
	LED_toggleHandle();
}

/**
 * @func   LED_handleRefreshDelay
 * @brief
 * @param  None
 * @retval None
 */
static void LED_handleRefreshDelay(void)
{
	if(refresh_led_delay_st.delay_t != 0){
	    if(clock_time_get_elapsed_time(refresh_led_delay_st.delay_start_t) > refresh_led_delay_st.delay_t){
	    	LED_refresh(refresh_led_delay_st.led_mask);
	    	refresh_led_delay_st.delay_t = 0;
	    }
	}
}

/**
 * @func   LED_setupRefreshLedDelay
 * @brief
 * @param  None
 * @retval None
 */
void LED_setupRefreshLedDelay(uint16_t delay_t, u16 led_mask)
{
	refresh_led_delay_st.delay_t = delay_t;
	refresh_led_delay_st.led_mask = led_mask;
	refresh_led_delay_st.delay_start_t = clock_time_ms();
}

// End File
