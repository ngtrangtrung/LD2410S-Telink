/*
 * input.h
 *
 *  Created on: Oct 14, 2020
 *      Author: DungTran BK
 */

#ifndef INPUT_H_
#define INPUT_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "utilities.h"
#include "config_board.h"

typedef void (*typeInput_HandleStateCallbackFunc)(u8, u8);

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

typedef struct {
	u8   present_st;
	u8   before_st;
	u32  signal_active_last_t;
	u32  signal_high_last_t;
	u8   hold_step;
	BOOL press_flag;
	u32  press_last_t;
	u8   press_cnt;
	BOOL press_many_time_flag;
}input_params_t;

#define INPUT_INACTIVE      (0)
#define INPUT_ACTIVE        (1)

#define INPUT_ST_INACTIVE   (0)
#define INPUT_ST_ACTIVE     (1)

#define INPUT_PIN_POS       (0)
#define INPUT_BIT_POS       (1)

#define PULSE_DETECT_TIME_OUT    TIMER_50MS
#define NUMBER_INPUT_BUTTON      (2)

#define DETECT_PRESS_TIME        TIMER_100MS
#define INPUT_DOWN_TIMER_EXPIRE  TIMER_1S     //  TIME_DETECT_PRESS < x < TIMER_2s

#define INPUT_STATE(x) ((gpio_read(input_pin_arr[x][INPUT_PIN_POS]) >> input_pin_arr[x][INPUT_BIT_POS])&0x01)

/******************************************************************************/
/*                             EXPORT FUNCTIONS                               */
/******************************************************************************/

void input_scan_all_channel(void);
void input_init(void);
void input_callback_init(typeInput_HandleStateCallbackFunc callbackFunc);

#endif /* INPUT_H_ */
