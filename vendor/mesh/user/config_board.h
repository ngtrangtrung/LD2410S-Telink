/*
 * config_board.h
 *
 *  Created on: Oct 15, 2020
 *      Author: DungTran BK
 */

#ifndef CONFIG_BOARD_H_
#define CONFIG_BOARD_H_

// AC input
#define NUMBER_INPUT         _RL_NUMBER_

#define INPUT0_IO_PIN        GPIO_PB5
#define INPUT0_IO_BIT        0x5
#define INPUT1_IO_PIN        GPIO_PB4
#define INPUT1_IO_BIT        0x4

// OPTION button
#define OPT_BUTTON_IO_PIN    GPIO_PD6
#define OPT_BUTTON_IO_BIT    0x6

// LED
#define CFG_LED_RED_IO_PIN   GPIO_PC5
#define CFG_LED_RED_IO_BIT   0x5

#define CFG_LED_BLUE_IO_PIN  GPIO_PC4
#define CFG_LED_BLUE_IO_BIT  0x4

// RELAY
#ifdef _RL_NUMBER_
#define RL_NUMBER            _RL_NUMBER_
#endif

#define PIN_CONTROL_RL0_ON      GPIO_PA6
#define BIT_CONTROL_RL0_ON      0x6

#define PIN_CONTROL_RL0_OFF     GPIO_PD5
#define BIT_CONTROL_RL0_OFF    	0x5

#define PIN_CONTROL_RL1_ON      GPIO_PA5
#define BIT_CONTROL_RL1_ON      0x5

#define PIN_CONTROL_RL1_OFF     GPIO_PD6
#define BIT_CONTROL_RL1_OFF    	0x6

#define PIN_CONTROL_RL2_ON      GPIO_PA4
#define BIT_CONTROL_RL2_ON      0x4

#define PIN_CONTROL_RL2_OFF     GPIO_PA1
#define BIT_CONTROL_RL2_OFF    	0x1
// ZERO
#define PIN_ZERO_DETECT      GPIO_PA3
#define BIT_ZERO_DETECT      0x3


// Define RL Type
#define NORMAL_RELAY         0x0
#define HP_RELAY             0x1
#define LATCHING_RELAY       0x2

#if _HP_RELAY_
#define RELAY_TYPE           HP_RELAY
#else
#define RELAY_TYPE           NORMAL_RELAY
#endif

#if RL_NUMBER == 1
#define BACKUP_MASK      0x01
#elif RL_NUMBER == 2
#define BACKUP_MASK      0x03
#endif

#endif /* CONFIG_BOARD_H_ */
