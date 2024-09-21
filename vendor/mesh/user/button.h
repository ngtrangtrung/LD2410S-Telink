/*
 * button.h
 *
 *  Created on: Sep 12, 2020
 *      Author: DungTran BK
 */

#ifndef BUTTON_H_
#define BUTTON_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "../../../proj/tl_common.h"
#include "config_board.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

typedef void (*typeButton_HandleStateCallbackFunc)(u8);

#define OPTION_BUTTON_HOLD_TIME     clock_time_get_elapsed_time(opt_button_params.start_hold_t)

#define OPTION_BUTTON_PRESS         0x00
#define OPTION_BUTTON_RELEASE       0x01

#define OPTION_BUTTON_STATE         ((gpio_read(OPT_BUTTON_IO_PIN) >> OPT_BUTTON_IO_BIT)&0x01)

#define MIN_POLL_COUNTER_TO_CHANGE_HOLD_STEP  10
#define OPTION_BUTTON_ERROR_HOLD_TIME         TIMER_20S

/******************************************************************************/
/*                              EXPORT FUNCTION                               */
/******************************************************************************/

void option_button_init(void);
void option_button_callback_init(typeButton_HandleStateCallbackFunc handleOptionButtonStateCallbackInit);
void option_button_scan(void);

#endif /* BUTTON_H_ */
