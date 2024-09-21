/*
 * Copyright (c) 2019
 * Lumi, JSC.
 * All Rights Reserved
 *
 *
 * Description:led.h
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
#pragma once
#include "config_board.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

typedef void (*typeLed_HandleRefreshLedCallbackFunc)(uint16_t);

enum LedColor_enum{
	LED_COLOR_RED    = 0x00,
	LED_COLOR_BLUE   = 0x01,
	LED_COLOR_PINK   = 0x02,
	LED_COLOR_NONE   = 0x03
};

enum LedColorIdx{
	LED_RED_IDX    = 0x00,
	LED_BLUE_IDX   = 0x01,
	END_LED_IDX    = 0x02
};

typedef uint8_t LedColor_enum;

enum LedLastState_enum{
	LAST_STATE_REFRESH_LED   = 0,
	LAST_STATE_ON_RED        = 1,
	LAST_STATE_ON_BLUE       = 2,
	LAST_STATE_ON_PINK       = 3,
	LAST_STATE_COLOR_NONE    = 4,
	LAST_STATE_REFRESH_LED_DELAY = 5
};
typedef uint8_t LedLastState_enum;

#define COMMAND_LED_DEFAULT  \
{                            \
    LED_MODE_BLINK,          \
    0xFFFF,                  \
    LED_COLOR_RED,           \
    4,                       \
    LAST_STATE_REFRESH_LED,  \
    300,                     \
	0                        \
}

enum BlinkLedStatus_enum{
	NOT_BLINK = 0,
	BLINKING  = 1,
};
typedef uint8_t BlinkLedStatus_enum;

enum LedBlinkFlag_enum{
	BLINK_IDLE,
	BLINK_ACTIVE,
};
typedef uint8_t LedBlinkFlag_enum;

enum LedMode_enum{
	LED_MODE_OFF = 0,
	LED_MODE_ON = 1,
	LED_MODE_BLINK = 2,
	LED_MODE_BLINK_FOREVER = 3,
	LED_MODE_REFRESH = 4,
	LED_MODE_NONE = 0xFF
};
typedef uint8_t LedMode_enum;


typedef struct {
	LedMode_enum      ledMode;
	uint16_t          ledMask;
	LedColor_enum     ledColor;
	uint8_t           blinkTime;
	LedLastState_enum lastState;
	uint16_t          blinkInterval;
	uint16_t          delayTimeRefresh;
}LedCommand_str;

typedef struct {
	LedColor_enum     toggleLedState;
	uint32_t          toggleledLastTime;
	LedBlinkFlag_enum toggleLedFlag;
	uint8_t           toggleLedTimes;
	LedLastState_enum toggleLedLastState;
	uint32_t          toggleLedInterval;
}LedBlinkInit_str;

typedef struct {
	u32  delay_t;
	u32  delay_start_t;
	u16  led_mask;
}refresh_led_delay_t;

#define BUF_LED_CMD_SIZE        10

#define DEFAULT_BLINK_INTERVAL	300
#define MIN_BLINK_INTERVAL	    100
#define MAX_BLINK_INTERVAL      500

/******************************************************************************/
/*                             EXPORT FUNCTIONS                               */
/******************************************************************************/

void LED_setColor(uint8_t ledNbr, LedColor_enum color);
void LED_init(void);
void LED_refreshCallbackInit(typeLed_HandleRefreshLedCallbackFunc refreshLedCallbackInit);
void LED_refresh(uint16_t ledMask);
void LED_handLeEventFunction(void);
void LED_pushNormalBlinkLedCmdToFifo(uint8_t blinkTime, LedColor_enum color);
uint8_t LED_getBlinkLedFlag(void);
uint8_t LED_pushLedCommandToFifo(LedCommand_str ledCmd);
void LED_offAll(void);
void LED_off(uint8_t ledNbr);
void LED_setupRefreshLedDelay(uint16_t delay_t, u16 led_mask);
void LED_PushFastBlinkLedCmdToFifo(uint8_t blinkTime, LedColor_enum color);
void LED_setColorRed(u8 ledNbr);
void LED_setColorBlue(uint8_t ledNbr);
// End File
