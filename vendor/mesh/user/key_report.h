/*
 * key_report.h
 *
 *  Created on: Feb 27, 2020
 *      Author: DungTran BK
 */

#ifndef KEY_REPORT_H_
#define KEY_REPORT_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "utilities.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
enum ButtonKey_enum{
	BTN_KEY_PRESS_1_TIME,
	BTN_KEY_PRESS_2_TIMES,
	BTN_KEY_PRESS_3_TIMES,
	BTN_KEY_PRESS_5_TIMES,
	BTN_KEY_HOLD_2_SECONDS,
	END_BTN_KEY,
};
typedef uint8_t ButtonKey_enum;

/******************************************************************************/
/*                             EXPORT FUNCTIONS                               */
/******************************************************************************/

void key_report_send(u8 idx, u8 key_code);

#endif /* KEY_REPORT_H_ */
