/*
 * test_auto.h
 *
 *  Created on: May 29, 2020
 *      Author: DungTran BK
 */

#ifndef TEST_AUTO_H_
#define TEST_AUTO_H_

enum{
	INACTIVE_TEST_PROGRAM_SECOND = 0,
	ACTIVE_TEST_PROGRAM_SECOND   = 1
};

int handle_test_led_program(void);
uint8_t get_enable_test_led_flag(void);
void handle_msg_test_led_proram_second(u8 code);

#endif /* TEST_AUTO_H_ */
