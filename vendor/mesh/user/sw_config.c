/*
 * sw_config.c
 *
 *  Created on: Mar 28, 2022
 *      Author: DungTranBK
 */
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "../../../proj/tl_common.h"
#include "../../common/mesh_node.h"
#include "../../common/vendor_model.h"
#include "utilities.h"
#include "led.h"
#include "config_board.h"
#include "sw_config.h"

#include "debug.h"
#ifdef  SW_CFG_DBG_EN
	#define DBG_SW_CFG_SEND_STR(x)   Dbg_sendString((s8*)x)
	#define DBG_SW_CFG_SEND_INT(x)   Dbg_sendInt(x)
	#define DBG_SW_CFG_SEND_HEX(x)   Dbg_sendHex(x)
	#define DBG_SW_CFG_SEND_BYTE(x)  Dbg_sendOneByteHex(x)
#else
	#define DBG_SW_CFG_SEND_STR(x)
	#define DBG_SW_CFG_SEND_INT(x)
	#define DBG_SW_CFG_SEND_HEX(x)
	#define DBG_SW_CFG_SEND_BYTE(x)
#endif

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

/******************************************************************************/
/*                          PRIVATE FUNCTIONS DECLERATION                     */
/******************************************************************************/

uint8_t en_group_default[NUMBER_INPUT];

static int flash_en_group_def_idx = 0;
#define FLASH_SIZE_EN_GROUP_DEF  4000
#define BLOCK_SIZE_EN_GROUP_DEF  sizeof(en_group_default)

/******************************************************************************/
/*                           EXPORT FUNCTIONS DECLERATION                     */
/******************************************************************************/

/**
 * @func    sw_get_en_disable_group_default
 * @brief
 * @param
 * @retval  None
 */
BOOL sw_get_en_disable_group_default(int idx)
{
	if(idx < ELE_CNT) {
		return en_group_default[idx];
	}
	return FALSE;
}

/**
 * @func    sw_set_enable_disable_group_default
 * @brief
 * @param
 * @retval  None
 */
void sw_response_enable_disable_group_default(int idx)
{
	if(idx < ELE_CNT) {
		en_dis_group_rsp_t en_dis_group_rsp_st;
		en_dis_group_rsp_st.op = VD_EN_ON_OFF_GROUP_DEFAULT;
		en_dis_group_rsp_st.en = en_group_default[idx];
		mesh_tx_cmd_rsp(
				 VD_CONFIG_NODE_STATUS,
				 (u8 *)&en_dis_group_rsp_st,
				 sizeof(en_dis_group_rsp_t),
				 ele_adr_primary + idx,
				 GATEWAY_UNICAST_ADDR,
				 0,
				 0
			 );
	}
}

/**
 * @func    io_store_parameters
 * @brief
 * @param   None
 * @retval  None
 */
static void sw_store_enable_gr_default(void)
{
	flash_en_group_def_idx += BLOCK_SIZE_EN_GROUP_DEF;

    if(flash_en_group_def_idx >= FLASH_SIZE_EN_GROUP_DEF) {
    	DBG_SW_CFG_SEND_STR("\n Erase Sector Enable Group default");
    	flash_en_group_def_idx = 0;
    	flash_erase_sector(FLASH_ADR_EN_GROUP_DEFAULT);
    }
	flash_write_page(
			FLASH_ADR_EN_GROUP_DEFAULT + flash_en_group_def_idx,
			BLOCK_SIZE_EN_GROUP_DEF,
			en_group_default
		);
}

/**
 * @func    io_restore_enable_gr_default_idx
 * @brief
 * @param   None
 * @retval  None
 */
static void io_restore_enable_gr_default_idx(void)
{
	u16 i = 0;
	u8 temp[BLOCK_SIZE_EN_GROUP_DEF];

	memset(temp, 0, BLOCK_SIZE_EN_GROUP_DEF);

    while(i < FLASH_SIZE_EN_GROUP_DEF){
    	flash_read_page(
    			FLASH_ADR_EN_GROUP_DEFAULT + i, BLOCK_SIZE_EN_GROUP_DEF, temp
    		);
    	if(IsMatchVal(&temp[0], BLOCK_SIZE_EN_GROUP_DEF, MAX_U8))
    		break;
    	i += BLOCK_SIZE_EN_GROUP_DEF;
    }
    if(i == 0){
    	flash_en_group_def_idx = FLASH_ADR_EN_GROUP_DEFAULT - BLOCK_SIZE_EN_GROUP_DEF;
    } else{
    	flash_en_group_def_idx = i - BLOCK_SIZE_EN_GROUP_DEF;
    }
}
/**
 * @func    sw_set_enable_disable_group_default
 * @brief
 * @param
 * @retval  None
 */
void sw_set_enable_disable_group_default(int idx, BOOL en)
{
    if(idx < ELE_CNT)
    {
    	LedCommand_str ledCmd = COMMAND_LED_DEFAULT;
    	ledCmd.ledMask = 1 << idx;
    	ledCmd.ledColor = LED_COLOR_BLUE;
    	ledCmd.blinkInterval = 200;
    	ledCmd.blinkTime = 4;
    	LED_pushLedCommandToFifo(ledCmd);

    	if(en == en_group_default[idx]) {
			return;
		}
		BOOL value_is_valid = TRUE;
		if(en == TRUE) {
			en_group_default[idx]= TRUE;
		}else if(en == FALSE) {
			en_group_default[idx] = FALSE;
		}else {
			value_is_valid = FALSE;
		}
		if(value_is_valid == TRUE) {
			// Save to flash
			sw_store_enable_gr_default();
		}
    }
}

/**
 * @func    sw_set_enable_disable_group_default
 * @brief
 * @param
 * @retval  None
 */
void sw_config_init(void)
{
	io_restore_enable_gr_default_idx();
	flash_read_page(
			FLASH_ADR_EN_GROUP_DEFAULT + flash_en_group_def_idx, BLOCK_SIZE_EN_GROUP_DEF, en_group_default
		);
	foreach(i, NUMBER_INPUT) {
		if(en_group_default[i] == MAX_U8) {
			foreach(i, NUMBER_INPUT) {
				en_group_default[i] = DISABLE;
			}
			sw_store_enable_gr_default();
			break;
		}
		en_group_default[i] &= 0x01;
	}
}

// End file
