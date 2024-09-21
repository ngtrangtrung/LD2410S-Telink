/*
 * relay.c
 *
 *  Created on: Oct 14, 2020
 *      Author: DungTran BK
 */

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "../../../proj/tl_common.h"
#include "../../common/system_time.h"
#include "utilities.h"
#include "user_irq.h"
#include "relay.h"

#include "debug.h"
#ifdef  RL_DBG_EN
	#define DBG_RL_SEND_STR(x)   Dbg_sendString((s8*)x)
	#define DBG_RL_SEND_INT(x)   Dbg_sendInt(x)
	#define DBG_RL_SEND_HEX(x)   Dbg_sendHex(x)
	#define DBG_RL_SEND_BYTE(x)  Dbg_sendOneByteHex(x)
#else
	#define DBG_RL_SEND_STR(x)
	#define DBG_RL_SEND_INT(x)
	#define DBG_RL_SEND_HEX(x)
	#define DBG_RL_SEND_BYTE(x)
#endif

#define relay_num				4


#define ON_RL(x)  				gpio_write(pin_control_relay_arr[x][RL_PIN_POS], ON);
#define ON_RL_PULL_DOWN(x)  	gpio_write(pin_control_relay_arr[x][RL_PIN_POS], OFF);

#define OFF_RL(x) 				gpio_write(pin_control_relay_arr[x][RL_PIN_OFF_POS], ON);
#define OFF_RL_PULL_DOWN(x)  	gpio_write(pin_control_relay_arr[x][RL_PIN_OFF_POS], OFF);
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

static uint16_t hold_time_on[relay_num]  = {DEFAULT_RL_ON_DELAY};
static uint16_t hold_time_off[relay_num] = {DEFAULT_RL_OFF_DELAY};
static uint16_t RLS = 0;
static uint16_t RLS_Current = 0;

#if CACULATOR_FREQ_EN
static caculator_grid_freq_t caculator_grid_freq_st = {
	.last_irq_t_ms = 0,
	.start_t_ms = 0,
	.freq = 50,
	.cnt = 0,
};
#endif

#define RL_PIN_POS       	 (0)
#define RL_BIT_POS       	 (1)
#define RL_PIN_OFF_POS       (2)
#define RL_BIT_OFF_POS       (3)

static u16 pin_control_relay_arr[relay_num][4] ={
		{
			PIN_CONTROL_RL0_ON, BIT_CONTROL_RL0_ON, PIN_CONTROL_RL0_OFF, BIT_CONTROL_RL0_OFF
		},
		#if relay_num > 1
		{
				PIN_CONTROL_RL1_ON, BIT_CONTROL_RL1_ON, PIN_CONTROL_RL1_OFF, BIT_CONTROL_RL1_OFF
		},
		#endif

		#if relay_num > 2
		{
				PIN_CONTROL_RL2_ON, BIT_CONTROL_RL2_ON, PIN_CONTROL_RL2_OFF, BIT_CONTROL_RL2_OFF
		},
		#endif

////		#if relay_num > 3
////		{
////			PIN_CONTROL_RL3, BIT_CONTROL_RL3
////		},
//		#endif
	};

static relay_active_t active_channel_st = {
		.channel = NO_RL_NUMBER_ACTIVE,
		.state = 0xFF,
		.step = 0
	};
static uint8_t relay_module_status = RELAY_MODULE_IDLE;
static uint8_t done_on_off_control_flag = CONTROL_IN_PROCESSING;
static uint32_t start_time_wait_zero_point = 0;
static uint8_t cotrol_when_timeout = 0;
static uint16_t half_period_of_grid = 10000;

/******************************************************************************/
/*                            PRIVATE FUNCTION                                */
/******************************************************************************/

static void RL_turnOn(u8 idx);
static void RL_turnOff(u8 idx);

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/**
 * @func   RL_ChangeRelayStateWhenTimerOV (For RTC Timer)
 * @brief  Interrupt server of RTC module, used to turn on/off
 *         the relay at zero point
 * @param  None
 * @retval None
 */
void RL_changeRelayStateWhenTimerOV(void)
{
	timer1_disable();
	if(RELAY_MODULE_IDLE != relay_module_status){
        if(active_channel_st.channel < relay_num){
			(active_channel_st.state == ON)
					?(RL_turnOn(active_channel_st.channel))
							:(RL_turnOff(active_channel_st.channel));
		}
		done_on_off_control_flag = CONTROL_SUCCESS;
		relay_module_status = RELAY_MODULE_IDLE;
	}
}

/**
 * @func   RL_handleZeroSync
 * @brief
 * @param  None
 * @retval None
 */
u32 RL_getLastTimeDetectZeroPoint(void)
{
	return caculator_grid_freq_st.last_irq_t_ms;
}

/**
 * @func   RL_handleZeroSync
 * @brief
 * @param  None
 * @retval None
 */
void RL_handleZeroSync(void)
{
	if(cotrol_when_timeout == 0){
		cotrol_when_timeout = 1;
        timer1_enable();
		#if !CACULATOR_FREQ_EN
		gpio_en_interrupt(PIN_ZERO_DETECT, IRQ_DISABLE);
		#endif
	}
#if CACULATOR_FREQ_EN
	if(clock_time_exceed_ms(caculator_grid_freq_st.last_irq_t_ms, TIMER_5MS)){
	    caculator_grid_freq_st.cnt++;
	    caculator_grid_freq_st.last_irq_t_ms = clock_time_ms();
	}
	if((clock_time_get_elapsed_time   \
			(caculator_grid_freq_st.start_t_ms) >= TIMER_1S) && (caculator_grid_freq_st.cnt != 0)){
		caculator_grid_freq_st.start_t_ms = clock_time_ms();
		caculator_grid_freq_st.freq = caculator_grid_freq_st.cnt >> 1;
		// Update PERIOD
		if(clock_time_exceed_s(0, 5)){
			if(caculator_grid_freq_st.freq < 55){
				half_period_of_grid = 10000;
			}
			else{
				half_period_of_grid = 8000;
			}
			/*
			DBG_RL_SEND_STR("\n FREQ: ");
			DBG_RL_SEND_INT(caculator_grid_freq_st.freq);
			*/
		}
		caculator_grid_freq_st.cnt = 0;
	}
#else
	DBG_RL_SEND_STR("\n Zero Point Detected");
#endif
}

/**
 * @func   RL_init
 * @brief  Init RL pinout, RTC, ADC...
 * @param  None
 * @retval None
 */
void RL_init(void)
{


	DBG_RL_SEND_STR("\r\n RL_init");
	foreach(i, relay_num){
		gpio_set_func(pin_control_relay_arr[i][RL_PIN_POS], AS_GPIO);
		gpio_set_input_en(pin_control_relay_arr[i][RL_PIN_POS], FALSE);
		gpio_set_output_en(pin_control_relay_arr[i][RL_PIN_POS], TRUE);

		gpio_set_func(pin_control_relay_arr[i][RL_PIN_OFF_POS], AS_GPIO);
		gpio_set_input_en(pin_control_relay_arr[i][RL_PIN_OFF_POS], FALSE);
		gpio_set_output_en(pin_control_relay_arr[i][RL_PIN_OFF_POS], TRUE);

	    if (i == 0) {
	        hold_time_on[i] = SPECIAL_RL_ON_DELAY;
	     	hold_time_off[i] = SPECIAL_RL_OFF_DELAY;
	    }
	    else {
	     	hold_time_on[i] = DEFAULT_RL_ON_DELAY;
	     	hold_time_off[i] = DEFAULT_RL_OFF_DELAY;
	    }
	}
	// ZERO
#if IRQ_GPIO_ENABLE
	gpio_set_interrupt_init(
				PIN_ZERO_DETECT, PM_PIN_PULLUP_1M, 0, FLD_IRQ_GPIO_EN
			);
	gpio_en_interrupt(PIN_ZERO_DETECT, IRQ_DISABLE);
	gpio_irq_callback_init(RL_handleZeroSync);
#endif
	// TIMER
	timer1_hw_config();
	timer1_irq_callback_init(RL_changeRelayStateWhenTimerOV);
}

/**
 * @func   RL_setUpTimerToChangeRlState
 * @brief  Set up compare value for TIMER1
 * @param  Byte: State to change of the active relay channel ,Byte: Active relay channel
 * @retval None
 */
 void RL_setUpTimerToChangeRlState(u8 st, u8 idx)
 {
	uint16_t compare_us;
	compare_us = (st == ON)	\
	     ?(half_period_of_grid - hold_time_on[active_channel_st.channel])	\
	           :(half_period_of_grid - hold_time_off[active_channel_st.channel]);

	timer1_update_cmp_value(compare_us);
}

/**
 * @func   RL_turnOn
 * @brief  Turn on the relay.
 * @param  Byte: relay number
 * @retval None
 */
static void RL_turnOn(u8 idx)
{
	if(idx < relay_num){
		ON_RL(idx);
		__delay_ms(8);
		ON_RL_PULL_DOWN(idx);
	}
}
/**
 * @func   RL_turnOff
 * @brief  Turn off the relay.
 * @param  Byte: Relay Number
 * @retval None
 */
static void RL_turnOff(u8 idx)
{
	if(idx < relay_num){
		OFF_RL(idx);
		__delay_ms(8);
		OFF_RL_PULL_DOWN(idx);
	}
}

/**
 * @func   RL_toggleState
 * @brief  Invert relay status.
 * @param  None
 * @retval None
 */
void RL_toggleState(u8 idx)
{
	RL_changeRlState(idx, ((((RLS >> idx) & 0x01) == ON)?OFF:ON));
}

/**
 * @func   RL_controlRelayHandle
 * @brief  Handling to turn on/off the relay, set the necessary parameters
 *         to turn on/off the relay at the correct zero point.
 * @param  None
 * @retval BYTE
 */
u8 RL_controlRelayHandle(void)
{
    if(relay_module_status == RELAY_MODULE_IDLE){
		if(done_on_off_control_flag == CONTROL_SUCCESS){
			if(active_channel_st.channel != NO_RL_NUMBER_ACTIVE){
				(active_channel_st.state == ON)
						?(RLS_Current |= (1 << active_channel_st.channel))
								:(RLS_Current &= ~(1 << active_channel_st.channel));
				active_channel_st.channel = NO_RL_NUMBER_ACTIVE;
			}
			done_on_off_control_flag = CONTROL_IN_PROCESSING;
		}
		if(RLS != RLS_Current){
			foreach(i, relay_num)
			{
				if(((RLS >> i) & 0x01) != ((RLS_Current >> i) & 0x01)){
					active_channel_st.channel = i;
					active_channel_st.step = 0;
					active_channel_st.state = ((RLS >> i) & 0x01);
					relay_module_status = RELAY_MODULE_BUSY;
					// Set up compare value for TIMER1
					RL_setUpTimerToChangeRlState(
								active_channel_st.state, active_channel_st.channel
							);
					gpio_en_interrupt(
								PIN_ZERO_DETECT, IRQ_ENABLE
							);
					start_time_wait_zero_point = clock_time_ms();
					cotrol_when_timeout = 0;
					break;
				}
			}
		}
	}
    else{
	    if(active_channel_st.channel != NO_RL_NUMBER_ACTIVE){
            if(cotrol_when_timeout == 0){
				if(clock_time_get_elapsed_time(start_time_wait_zero_point) > TIMER_50MS){
//					DBG_RL_SEND_STR("\r\n Wait Zero Point Time out");
					cotrol_when_timeout = 1;
					timer1_enable();
#if !CACULATOR_FREQ_EN
					gpio_en_interrupt(
								PIN_ZERO_DETECT, IRQ_DISABLE
							);
#endif
				}
			}
		}
	}
	return relay_module_status;
}

/**
 * @func   RL_changeRlState
 * @brief  Change state of the relay.
 * @param  Byte: Relay Number, Byte: Relay State
 * @retval None
 */
void RL_changeRlState(u8 idx, u8 st)
{
	if(idx < relay_num){
		// Control the same state of the relay, still shooting status
		if(st == ON){
			RLS |= (1 << idx);
			RLS_Current &= ~(1 << idx);
		}
		else{
			RLS &= ~(1 << idx);
			RLS_Current |= (1 << idx);
		}
	}
}

/**
 * @func   RL_changeAllRlState
 * @brief  Change all Relay status, just use when power on
 * @param  Word: State to change.
 * @retval None
 */
void RL_changeAllRlState(u16 st)
{
	RLS = st & BACKUP_MASK;
}

/**
 * @func   RL_changeAllRlCurrentState
 * @brief  Change all Relay status, just use when power on
 * @param  Word: State to change.
 * @retval None
 */
void RL_changeAllRlCurrentState(u16 st)
{
	RLS_Current = st & BACKUP_MASK;
}

/**
 * @func   RL_getRelayState
 * @brief  Return status of all relay channel
 * @param  None
 * @retval Word: Relay State
 */
u16 RL_getRelayState(void)
{
	return RLS;
}

/**
 * @func   RL_getCurrentRelayState
 * @brief  Get Current state of all relay channel
 * @param  None
 * @retval Word: Current state of all relay channel
 */
u16 RL_getCurrentRelayState(void)
{
	return RLS_Current;
}

// End File
