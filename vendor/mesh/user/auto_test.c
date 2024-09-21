/*
 * test_auto.c
 *
 *  Created on: May 29, 2020
 *      Author: DungTran BK
 */

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "../../../proj_lib/ble/blt_config.h"
#include "../../../stack/ble/ll/ll_adv.h"
#include "../../common/system_time.h"
#include "utilities.h"
#include "../../common/mesh_node.h"
#include "../../common/app_provison.h"
#include "../../common/light.h"

#include "auto_test.h"

#include "debug.h"
#ifdef TEST_AUTO_DBG_EN
#define DBG_TEST_AUTO_SEND_STR(x)   Dbg_sendString((s8*)x)
#define DBG_TEST_AUTO_SEND_INT(x)   Dbg_sendInt(x)
#define DBG_TEST_AUTO_SEND_HEX(x)   Dbg_sendHex(x)
#else
#define DBG_TEST_AUTO_SEND_STR(x)
#define DBG_TEST_AUTO_SEND_INT(x)
#define DBG_TEST_AUTO_SEND_HEX(x)
#endif

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

static uint8_t enable_test_led = DISABLE;

#define AUTO_TEST_STEP_TIME    1000
static uint32_t change_step_auto_test_start_time = 0;

/******************************************************************************/
/*                           EXPORT FUNCTIONS DECLERATION                     */
/******************************************************************************/

/**
 * @func    get_enable_test_led_flag
 * @brief
 * @param   None
 * @retval  None
 */
uint8_t get_enable_test_led_flag(void){
	return enable_test_led;
}

/**
 * @func    handle_test_led_program
 * @brief
 * @param   None
 * @retval  Success: Disable or Enable Test Led and Valid test parameters
 *          Fail:    Enable Test Led but invalid test parameters
 */
int handle_test_led_program(void)
{
	int err = 0;
	if(enable_test_led == ENABLE){
		static u8 present_state[LIGHT_CNT];
		if(STATE_DEV_PROVED == get_provision_state()){
			handle_msg_test_led_proram_second(DISABLE);
			return 0;
		}
		if(clock_time_exceed_ms(change_step_auto_test_start_time, AUTO_TEST_STEP_TIME)){
			foreach(i, LIGHT_CNT){
				if(present_state[i] == G_OFF){
					light_on_off_all_led_manual(i, Level_High);
					present_state[i] = G_ON;
				}else{
				    light_on_off_all_led_manual(i, Level_Low);
				    present_state[i] = G_OFF;
				}
			}
			change_step_auto_test_start_time = clock_time_ms();
		}
	}
	return err;
}

/**
 * @func    handle_msg_test_led_proram_second
 * @brief
 * @param   None
 * @retval  None
 */
void handle_msg_test_led_proram_second(u8 code)
{
	DBG_TEST_AUTO_SEND_STR("\n handle_msg_test_led_proram_second");
	if(enable_test_led == DISABLE){
		if(code == ACTIVE_TEST_PROGRAM_SECOND){
			enable_test_led = ENABLE;
			bls_ll_setAdvEnable(0);  //adv disable
		}
	}else{
		if(code == INACTIVE_TEST_PROGRAM_SECOND){
			// Exit Program Test 2
			enable_test_led = DISABLE;
			bls_ll_setAdvEnable(1);  //adv enable
		}
	}
}
// End File
