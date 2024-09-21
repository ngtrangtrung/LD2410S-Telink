/*
 * utilities.h
 *
 *  Created on: Feb 18, 2020
 *      Author: DungTran BK
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_

#include "../../../proj/tl_common.h"

#define MAX_U32   0xFFFFFFFF
#define MAX_U16   0xFFFF
#define MAX_U8    0xFF

#ifndef BOOL
#define BOOL  unsigned char
#endif

#ifndef FLAG_ACTIVE
#define FLAG_INACTIVE  0
#define FLAG_ACTIVE    1
#endif

#ifndef IRQ_EN
#define IRQ_DISABLE    0
#define IRQ_ENABLE     1
#endif

#ifndef OFF
#define OFF     0
#define ON      1
#endif

enum ButtonState_enum{
	RELEASE           = 0,
	HOLD_2S           = 1,
	HOLD_5S           = 2,
	HOLD_10S          = 3,

	PRESS_TWO_TIME    = 4,
	PRESS_THREE_TIME  = 5,
	PRESS_FOUR_TIME   = 6,
	PRESS_FIVE_TIME   = 7,
	PRESS_TEN_TIME    = 8,

	HOLD_15S          = 9,
	HOLD_7S           = 10,
	HOLD_9S           = 11,
	HOLD_12S          = 12,

	PRESS_SIX_TIME    = 13,
	PRESS_EIGHT_TIME  = 14,

	HOLD_50MS         = 15,
	HOLD_500MS        = 16,
	PRESS_TWELVE_TIME = 17,

	PRESS_ONE_TIME    = 18,    // new
    ST_UNKNOWN        = 0xFC,
	SHORT_HOLD        = 0xFD,
	START_PRESS       = 0xFE,
	NO_PRESS          = 0xFF
};
typedef uint8_t ButtonState_enum;

#define TIMER_5MS      5
#define TIMER_10MS     10
#define TIMER_20MS     20
#define TIMER_40MS     40
#define TIMER_50MS     50
#define TIMER_60MS     60
#define TIMER_80MS     80
#define TIMER_100MS    100
#define TIMER_200MS    200
#define TIMER_300MS    300
#define TIMER_500MS    500

#define TIMER_700MS    700

#define TIMER_1S       1000
#define TIMER_1S2      1200
#define TIMER_1S5      1500
#define TIMER_2S       2000
#define TIMER_3S       3000
#define TIMER_4S       4000
#define TIMER_5S       5000
#define TIMER_9S       9000
#define TIMER_10S      10000
#define TIMER_15S      15000
#define TIMER_20S      20000

#define VD_CONFIG_SW_SET_MAIN_PARAMS         0x10
#define VD_CONFIG_SW_SET_MAP_UNMAP_INPUT     0x11
#define VD_CONFIG_NODE_CNT                   0x03
#define VD_CONFIG_NODE_SET_START_CMD         VD_CONFIG_SW_SET_MAIN_PARAMS

#define VD_CONFIG_GROUP_ASSOCIATION          0x30
#define VD_EN_ON_OFF_GROUP_DEFAULT           0x31

#define CONFIG_NODE_SET    0x00
#define CONFIG_NODE_GET    0x01


enum {
	SLOW_REPORT,
	FAST_REPORT,
	VERY_FAST_REPORT
};

enum {
	SUB_OFF     = 0,
	SUB_ON      = 1,
	SUB_UNKNOWN = 2
}last_state_send_to_sub_t;

BOOL IsMatchVal(uint8_t *buff, uint8_t size, uint8_t val);
void __delay_us(uint32_t us);
void __delay_ms(uint32_t ms);
BOOL ev_get_config_mode_flag(void);

#endif /* UTILITIES_H_ */
