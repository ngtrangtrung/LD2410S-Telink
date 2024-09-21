/*
 * Copyright (c) 2019
 * Lumi, JSC.
 * All Rights Reserved
 *
 *
 * Description:
 *
 * Author: DungTran
 *
 * Last Changed By:  $Author: DungTran $
 * Revision:         $Revision: 1.0 $
 * Last Changed:     $Date: 12/17/19 $
 */

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "../../../proj/tl_common.h"
#include "../../../proj_lib/ble/blt_config.h"
#include "../../../proj_lib/ble/service/ble_ll_ota.h"
#include "../../../proj_lib/ble/ll//ll.h"
#include "../../common/app_beacon.h"
#include "../../common/generic_model.h"
#include "handle_ev.h"
#include "utilities.h"
#include "factory_reset.h"

#include "debug.h"
#ifdef  FACTORY_RST_DBG_EN
#define DBG_FACTORY_RST_SEND_STR(x)   Dbg_sendString((s8*)x)
#define DBG_FACTORY_RST_SEND_INT(x)   Dbg_sendInt(x)
#define DBG_FACTORY_RST_SEND_HEX(x)   Dbg_sendHex(x)
#else
#define DBG_FACTORY_RST_SEND_STR(x)
#define DBG_FACTORY_RST_SEND_INT(x)
#define DBG_FACTORY_RST_SEND_HEX(x)
#endif

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

extern u8 manual_factory_reset;

typedef struct{
	u8  flag;
	u32 startTimeDelay;
	u32 delayTime;
}factory_reset_t;

factory_reset_t factoryReset = {FALSE, 0 , 0};

#define MAX_FACTORY_RESET_DELAY_MS    10000

/******************************************************************************/
/*                        PRIVATE FUNCTIONS DECLERATION                       */
/******************************************************************************/

static void show_config_led_reset(void);

/******************************************************************************/
/*                        EXPORT FUNCTIONS DECLERATION                        */
/******************************************************************************/
/**
 * @func   start_factory_reset
 * @brief
 * @param  None
 * @retval None
 */
void start_factory_reset(void)
{
	// add terminate cmd
	if(bls_ll_isConnectState()){
		bls_ll_terminateConnection (0x13);
	}
	sleep_us(500000);   // wait tx buffer send completed.
	#if MANUAL_FACTORY_RESET_TX_STATUS_EN
	manual_factory_reset = 1;
	#else
	irq_disable();
	factory_reset();
    #if DUAL_MODE_WITH_TLK_MESH_EN
	UI_resotre_TLK_4K_with_check();
	#endif
	//show_ota_result(OTA_SUCCESS);
	show_factory_reset();
	start_reboot();
	#endif
}

/**
 * @func   get_factory_reset_flag
 * @brief
 * @param  None
 * @retval None
 */
BOOL get_factory_reset_flag(void)
{
	return factoryReset.flag;
}

/**
 * @func   handle_factory_reset_with_delay
 * @brief
 * @param  None
 * @retval None
 */
void handle_factory_reset_with_delay(void)
{
	if(factoryReset.flag == TRUE){
		if(clock_time_exceed_ms(factoryReset.startTimeDelay, factoryReset.delayTime)){
			if((clock_time_ms() - factoryReset.startTimeDelay) > MAX_FACTORY_RESET_DELAY_MS){
				factoryReset.flag = FALSE;
			}else{
			    start_factory_reset();
			}
		}
	}
}

/**
 * @func   setup_factory_reset_with_delay
 * @brief
 * @param  None
 * @retval None
 */
void setup_factory_reset_with_delay(u8 enable, u16 delay_ms)
{
	if(enable == TRUE){
		factoryReset.flag = TRUE;
		factoryReset.startTimeDelay = clock_time_ms();
		factoryReset.delayTime = delay_ms;
		send_config_node_reset_status_manual();
		show_config_led_reset();
	}else{
		factoryReset.flag = FALSE;
	}
}

/**
 * @func   show_config_led_reset
 * @brief
 * @param
 * @retval None
 */
static void show_config_led_reset(void)
{
	push_led_event_to_fifo(LED_CMD_DEL_NODE, 0xFFFF);
}


/**
 * @func   factory_reset
 * @brief
 * @param
 * @retval None
 */
int factory_reset()
{
	u8 r = irq_disable ();
	for(int i = 0; i < (FLASH_ADR_AREA_1_END - FLASH_ADR_AREA_1_START) / 4096; ++i){
	    u32 adr = FLASH_ADR_AREA_1_START + i*0x1000;
	    if(adr != FLASH_ADR_RESET_CNT){
		    flash_erase_sector(adr);
		}
	}
	for(int i = 0; i < (FLASH_ADR_AREA_2_END - FLASH_ADR_AREA_2_START) / 4096; ++i){
	    u32 adr = FLASH_ADR_AREA_2_START + i*0x1000;
	    if(adr != FLASH_ADR_RESET_CNT){
		    flash_erase_sector(adr);
		}
	}
    flash_erase_sector(FLASH_ADR_RESET_CNT);    // at last should be better, when power off during factory reset erase.

#if ERASE_ALL_CONFIG_WHEN_REST
    flash_erase_sector(FLASH_ADR_EN_GROUP_DEFAULT);
	flash_erase_sector(FLASH_ADR_EXECUTION_SCENE);
	flash_erase_sector(FLASH_ADR_IO_PARAMS);
	flash_erase_sector(FLASH_ADR_BINDING_PARAMS);
#endif

    irq_restore(r);
	return 0;
}

/**
 * @func   kick_out
 * @brief
 * @param
 * @retval None
 */
void kick_out()
{
	factoryReset.flag = TRUE;
	factoryReset.startTimeDelay = clock_time_ms();
	factoryReset.delayTime = TIMER_1S5;
	show_config_led_reset();
}

// End if
