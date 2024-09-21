/*
 * relay.h
 *
 *  Created on: Oct 14, 2020
 *      Author: DungTran BK
 */

#ifndef RELAY_H_
#define RELAY_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "config_board.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

enum PwStatus_enum{
	NoPW = 0,
	PW = 1,
};
typedef uint8_t PwStatus_enum;

//LATCHING_RELAY
#if RELAY_TYPE == LATCHING_RELAY
	#define	DEFAULT_RL_ON_DELAY	    8800
	#define	DEFAULT_RL_OFF_DELAY	8500
	#define SPECIAL_RL_ON_DELAY     8800
	#define SPECIAL_RL_OFF_DELAY    8500
#endif
// NORMAL_RELAY
#if RELAY_TYPE == NORMAL_RELAY
  #if (RL_NUMBER == 6) || (RL_NUMBER == 10)
	#define	DEFAULT_RL_ON_DELAY		3400
	#define	DEFAULT_RL_OFF_DELAY    4500
	#define SPECIAL_RL_ON_DELAY     3400
	#define SPECIAL_RL_OFF_DELAY    4500
  #elif (RL_NUMBER == 2) || (RL_NUMBER == 4)
	#define	DEFAULT_RL_ON_DELAY		2700
	#define	DEFAULT_RL_OFF_DELAY    1000
	#define SPECIAL_RL_ON_DELAY     2700
	#define SPECIAL_RL_OFF_DELAY    1000
  #endif
#endif
// HP_RELAY
#if RELAY_TYPE == HP_RELAY
	#define	DEFAULT_RL_ON_DELAY	    2700
	#define	DEFAULT_RL_OFF_DELAY	1000
	#define SPECIAL_RL_ON_DELAY     4500
	#define SPECIAL_RL_OFF_DELAY    6000
#endif

#if RELAY_TYPE == LATCHING_RELAY
    #define NUMBER_PIN_CONTROL_RL       4
    #define DEFAULT_CONTROL_RL_PINOUT   {PIN0_CONTROL_RL, PIN1_CONTROL_RL, PIN2_CONTROL_RL, PIN3_CONTROL_RL}
#else
  #if (RL_NUMBER == 1)
	#define  DEFAULT_RL_PINOUT    {PIN_CONTROL_RL0}
  #elif (RL_NUMBER == 2)
	#define  DEFAULT_RL_PINOUT    {PIN_CONTROL_RL0, PIN_CONTROL_RL1}
  #elif (RL_NUMBER == 3)
	#define  DEFAULT_RL_PINOUT    {PIN_CONTROL_RL0, PIN_CONTROL_RL1, PIN_CONTROL_RL2}
  #elif (RL_NUMBER == 4)
	  #define  DEFAULT_RL_PINOUT  {PIN_CONTROL_RL0, PIN_CONTROL_RL1, PIN_CONTROL_RL2, PIN_CONTROL_RL3}
  #elif (RL_NUMBER == 6)
	  #define  DEFAULT_RL_PINOUT  {PIN_CONTROL_RL0, PIN_CONTROL_RL1, PIN_CONTROL_RL2, PIN_CONTROL_RL3,  \
	                               PIN_CONTROL_RL4, PIN_CONTROL_RL5                                     \
	                              }
  #elif (RL_NUMBER == 8)
	  #define  DEFAULT_RL_PINOUT  {PIN_CONTROL_RL0, PIN_CONTROL_RL1, PIN_CONTROL_RL2, PIN_CONTROL_RL3,  \
	                               PIN_CONTROL_RL4, PIN_CONTROL_RL5, PIN_CONTROL_RL6, PIN_CONTROL_RL7   \
	                              }
  #elif (RL_NUMBER == 10)
	  #define  DEFAULT_RL_PINOUT  {PIN_CONTROL_RL0, PIN_CONTROL_RL1, PIN_CONTROL_RL2, PIN_CONTROL_RL3,  \
	                               PIN_CONTROL_RL4, PIN_CONTROL_RL5, PIN_CONTROL_RL6, PIN_CONTROL_RL7,  \
	                               PIN_CONTROL_RL8, PIN_CONTROL_RL9                                     \
	                              }
  #endif
#endif


#define NO_RL_NUMBER_ACTIVE 0xFF

enum PowerParameter_enum{
	POWER_0_50Hz  = 0x32,   // 50 Hz, no power
	POWER_0_60Hz  = 0x3C,   // 60 Hz, no power
};
typedef uint8_t PowerParameter_enum;

typedef struct {
	uint8_t channel;
	uint8_t state;
	uint8_t step;
} relay_active_t;

enum {
	HALF_PERIOD_50HZ  = 10000,
	HALF_PERIOD_60HZ  = 8000,
};

enum relay_module_status_enum{
	RELAY_MODULE_IDLE = 0,
	RELAY_MODULE_BUSY = 1,
};
typedef uint8_t RelayModuleStatus_enum;

enum RelayControlSuccess_enum{
	CONTROL_IN_PROCESSING = 0,
	CONTROL_SUCCESS = 1,
};
typedef uint8_t RelayControlSuccess_enum;

#define ONE_WIRE_SHORT_PULL  9000    //(6ms*1000)*3/2
#define ONE_WIRE_LONG_PULL   60000   //(40ms*1000)*3/2

#if RELAY_TYPE == LATCHING_RELAY
	#define PIN0_CONTROL_RL pin_control_relay_arr[0]
	#define PIN1_CONTROL_RL pin_control_relay_arr[1]
	#define PIN2_CONTROL_RL pin_control_relay_arr[2]
	#define PIN3_CONTROL_RL pin_control_relay_arr[3]
#else
    #define RL0 	pin_control_relay_arr[0]
	#define RL1 	pin_control_relay_arr[1]
	#define RL2 	pin_control_relay_arr[2]
	#define RL3 	pin_control_relay_arr[3]
	#define RL4 	pin_control_relay_arr[4]
	#define RL5 	pin_control_relay_arr[5]
	#define RL6 	pin_control_relay_arr[6]
	#define RL7 	pin_control_relay_arr[7]
	#define RL8 	pin_control_relay_arr[8]
	#define RL9 	pin_control_relay_arr[9]
#endif

#if CACULATOR_FREQ_EN
typedef struct {
	u32 last_irq_t_ms;
	u32 start_t_ms;
	u8  freq;
	u8  cnt;
}caculator_grid_freq_t;
#endif

/******************************************************************************/
/*                             EXPORT FUNCTIONS                               */
/******************************************************************************/

void RL_handleZeroSync(void);
void RL_init(void);
void RL_setUpTimerToChangeRlState(u8 st, u8 idx);
void RL_toggleState(u8 idx);
u8 RL_controlRelayHandle(void);
void RL_changeRlState(u8 idx, u8 st);
void RL_changeAllRlState(u16 st);
void RL_changeAllRlCurrentState(u16 st);
u16 RL_getRelayState(void);
u16 RL_getCurrentRelayState(void);
void RL_changeRelayStateWhenTimerOV(void);

u32 RL_getLastTimeDetectZeroPoint(void);

#endif /* RELAY_H_ */
