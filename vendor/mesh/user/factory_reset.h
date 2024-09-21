/*
 * factory_reset.h
 *
 *  Created on: Dec 25, 2019
 *      Author: DungTran BK
 */

#ifndef FACTORY_RESET_H_
#define FACTORY_RESET_H_

#include "../../../proj/tl_common.h"


void start_factory_reset(void);
void setup_factory_reset_with_delay(u8 enable, u16 delay_ms);
BOOL get_factory_reset_flag(void);
void handle_factory_reset_with_delay(void);

#endif /* FACTORY_RESET_H_ */
