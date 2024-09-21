/*
 * binding.c
 *
 *  Created on: Mar 18, 2022
 *      Author: DungTranBK
 */

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "../../../proj/tl_common.h"
#include "../../common/mesh_node.h"
#include "../../../proj_lib/sig_mesh/app_mesh.h"
#include "utilities.h"
#include "handle_ev.h"
#include "binding.h"

#include "debug.h"
#ifdef  BIND_CONFIG_DBG_EN
	#define DBG_BIND_SEND_STR(x)   Dbg_sendString((s8*)x)
	#define DBG_BIND_SEND_INT(x)   Dbg_sendInt(x)
	#define DBG_BIND_SEND_HEX(x)   Dbg_sendHex(x)
	#define DBG_BIND_SEND_BYTE(x)  Dbg_sendOneByteHex(x)
#else
	#define DBG_BIND_SEND_STR(x)
	#define DBG_BIND_SEND_INT(x)
	#define DBG_BIND_SEND_HEX(x)
	#define DBG_BIND_SEND_BYTE(x)
#endif

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

binding_para_t binding_para_st[ELE_CNT];
static int adr_binding_params_idx = 0;
control_binding_t control_binding_st[ELE_CNT];

/******************************************************************************/
/*                             PRIVATE FUNCS                                  */
/******************************************************************************/

static void store_binding_para(void);
static void restore_binding_para(void);
static void get_adr_binding_para(void);
static void binding_handle_setup_message(cfg_binding_format_t* cfg);
static void binding_handle_get_message(get_binding_format_t* msg);
static int  binding_enable(int model_idx, u16 group_adr);
static int  binding_disable(int model_idx) ;
static void control_binding_init(void);

/******************************************************************************/
/*                        EXPORT FUNCTIONS DECLERATION                        */
/******************************************************************************/

/**
 * @func    control_binding_init
 * @brief
 * @param
 * @retval  None
 */
static void control_binding_init(void)
{
	foreach_arr(i, control_binding_st) {
		control_binding_st[i].st = G_ONOFF_RSV;
		control_binding_st[i].cnt_same = 0;
		control_binding_st[i].op = G_ONOFF_SET;
	}
}
/**
 * @func    get_group_binding_adr
 * @brief
 * @param   Model Index
 * @retval  Binding Group Address
 */
uint16_t get_group_binding_adr(int idx)
{
	if(binding_para_st[idx].en == TRUE) {
		if((binding_para_st[idx].group_dst >= ADR_GROUP_START_POINT)
							|| (binding_para_st[idx].group_dst < ADR_GROUP_START_POINT)) {
			model_common_t *p_com_md = &model_sig_g_onoff_level.onoff_srv[idx].com;
			foreach(j, SUB_LIST_MAX) {
				if(( p_com_md->sub_list[j] != ADR_UNASSIGNED)
							&& (p_com_md->sub_list[j] != ADR_ALL_NODES)) {
					if(binding_para_st[idx].group_dst == p_com_md->sub_list[j]) {

						DBG_BIND_SEND_STR("\n BGr: ");
						DBG_BIND_SEND_HEX(binding_para_st[idx].group_dst);

						return binding_para_st[idx].group_dst;
					}
				}
			}
		}
	}
	DBG_BIND_SEND_STR("\n BGr: ADR_UNASSIGNED");
	return ADR_UNASSIGNED;
}

/**
 * @func    store_binding_para
 * @brief
 * @param   None
 * @retval  None
 */
static void store_binding_para(void)
{
	adr_binding_params_idx += BLOCK_SIZE_BINDING_PARAMS;

	if(adr_binding_params_idx >= FLASH_SIZE_BINDING_PARAMS) {
		adr_binding_params_idx = 0;
		flash_erase_sector(FLASH_ADR_BINDING_PARAMS);
	}
	flash_write_page (
			FLASH_ADR_BINDING_PARAMS + adr_binding_params_idx,
			BLOCK_SIZE_BINDING_PARAMS,
			(u8*)(&binding_para_st)
		);
}

/**
 * @func    store_default_binding_params
 * @brief
 * @param   None
 * @retval  None
 */
static void store_default_binding_params(void)
{
	foreach_arr(i, binding_para_st) {
		binding_para_st[i].en = FALSE;
		binding_para_st[i].group_dst = ADR_UNASSIGNED;
	}
	store_binding_para();
}
/**
 * @func    restore_binding_para
 * @brief
 * @param   None
 * @retval  None
 */
static void restore_binding_para(void)
{
	flash_read_page(
			FLASH_ADR_BINDING_PARAMS + adr_binding_params_idx,
			BLOCK_SIZE_BINDING_PARAMS,
			(u8*)(&binding_para_st)
		);
	foreach_arr(i, binding_para_st) {
		if(binding_para_st[i].en == MAX_U8) {
			store_default_binding_params();
			break;
		}
		else {
			if((binding_para_st[i].group_dst >= ADR_FIXED_GROUP_START)
					|| ((binding_para_st[i].group_dst < ADR_GROUP_START_POINT)
							&&(binding_para_st[i].group_dst != ADR_UNASSIGNED))
							) {
				store_default_binding_params();
				break;
			}
		}
	}
}

/**
 * @func    get_adr_binding_para
 * @brief
 * @param   None
 * @retval  None
 */
static void get_adr_binding_para(void)
{
	uint16_t i = 0;
	u8 tmp[BLOCK_SIZE_BINDING_PARAMS];

	while(i < FLASH_SIZE_BINDING_PARAMS) {
		flash_read_page(
				FLASH_ADR_BINDING_PARAMS + i,
				BLOCK_SIZE_BINDING_PARAMS,
				(u8*)(&tmp)
			);
		if(IsMatchVal(&tmp[0], BLOCK_SIZE_BINDING_PARAMS, MAX_U8)) {
			break;
		}
		i += BLOCK_SIZE_BINDING_PARAMS;
	}
	if(i == 0) {
		adr_binding_params_idx = FLASH_SIZE_BINDING_PARAMS - BLOCK_SIZE_BINDING_PARAMS;
	}
	else {
		adr_binding_params_idx = i - BLOCK_SIZE_BINDING_PARAMS;
	}
}

/**
 * @func    binding_init
 * @brief
 * @param   None
 * @retval  None
 */
void binding_init(void)
{
	get_adr_binding_para();
	restore_binding_para();
	control_binding_init();
}
/**
 * @func    binding_enable
 * @brief
 * @param
 * @retval  Status
 */
static int binding_enable(int model_idx, u16 group_adr)
{
	BOOL valid_group = FALSE;
	int st = MODEL_IDX_INVALID;

	if(model_idx < NUMBER_INPUT) {
		model_common_t *p_com_md = &model_sig_g_onoff_level.onoff_srv[model_idx].com;
		foreach(j, SUB_LIST_MAX) {
			if(( p_com_md->sub_list[j] != ADR_UNASSIGNED)
						&& (p_com_md->sub_list[j] != ADR_ALL_NODES)) {
				if(group_adr == p_com_md->sub_list[j]) {
					valid_group = TRUE;
					break;
				}
			}
		}
		if(valid_group == TRUE) {
			st = BINDING_SUCCESS;
			binding_para_st[model_idx].group_dst = group_adr;
			binding_para_st[model_idx].en = TRUE;
			// SAVE FLASH
			store_binding_para();
		}
		else {
			st = GROUP_NOT_SET;
		}
	}
	return st;
}
/**
 * @func    binding_disable
 * @brief
 * @param   None
 * @retval  None
 */
static int binding_disable(int model_idx)
{
	binding_para_st[model_idx].en = FALSE;
	store_binding_para();
	return 0;
}
/**
 * @func    binding_handle_setup_message
 * @brief
 * @param   None
 * @retval  None
 */
static void binding_handle_setup_message(cfg_binding_format_t* cfg)
{
	DBG_BIND_SEND_STR("\n binding_handle_setup_message: ");

	int model_idx = cfg->ele_adr - ele_adr_primary;
	int st = BINDING_ERR_UNKNOWN;

	if(model_idx < NUMBER_INPUT) {
		if(cfg->en == ENABLE) {
			st = binding_enable(
						model_idx, cfg->group_adr
					);
			if(st == BINDING_SUCCESS) {
				push_led_event_to_fifo(
							LED_CMD_BINDING_ENABLE, 1 << model_idx
						);
			}
			else {
				push_led_event_to_fifo(
							LED_CMD_BINDING_DISABLE, 1 << model_idx
						);
			}
		}
		else if(cfg->en == DISABLE) {
			binding_disable(model_idx);
			st = BINDING_DISABLE;
			push_led_event_to_fifo(
						LED_CMD_BINDING_DISABLE, 1 << model_idx
					);
		}
		binding_msg_response_t binding_msg_response;

		binding_msg_response.op = VD_CONFIG_GROUP_ASSOCIATION;
		binding_msg_response.st = st;
		binding_msg_response.ele_adr = cfg->ele_adr;
		binding_msg_response.en = binding_para_st[model_idx].en;
		binding_msg_response.group_adr = binding_para_st[model_idx].group_dst;
		mesh_tx_cmd_rsp(
				VD_CONFIG_NODE_STATUS,
				(u8 *)&binding_msg_response,
				sizeof(binding_msg_response),
				cfg->ele_adr,
				GATEWAY_UNICAST_ADDR,
				0,
				0
			);
	}
	else {
		st = MODEL_IDX_INVALID;
	}
	DBG_BIND_SEND_INT(st);
}
/**
 * @func    binding_handle_get_message
 * @brief
 * @param   None
 * @retval  None
 */
static void binding_handle_get_message(get_binding_format_t* msg)
{
	DBG_BIND_SEND_STR("\n binding_handle_get_message");

	int model_idx = \
			msg->ele_adr - ele_adr_primary;
	if(model_idx < NUMBER_INPUT) {
		binding_msg_response_t response_st;
		response_st.op = VD_CONFIG_GROUP_ASSOCIATION;
		response_st.st = BINDING_SUCCESS;
		response_st.ele_adr = msg->ele_adr;
		if(binding_para_st[model_idx].en == TRUE) {
		    if(get_group_binding_adr(model_idx) == ADR_UNASSIGNED) {
		    	response_st.en = FALSE;
		    }
		    else {
		    	response_st.en = TRUE;
		    }
		}
		else {
			response_st.en = FALSE;
		}
		response_st.group_adr = binding_para_st[model_idx].group_dst;
		mesh_tx_cmd_rsp(
				VD_CONFIG_NODE_STATUS,
				(u8 *)&response_st,
				sizeof(response_st),
				msg->ele_adr,
				GATEWAY_UNICAST_ADDR,
				0,
				0
			);
	}
}

/**
 * @func    binding_handle_nw_message
 * @brief
 * @param   None
 * @retval  None
 */
void binding_handle_nw_message(u8 type, u8* par)
{
	if(type == CONFIG_NODE_GET) {
		get_binding_format_t* get_binding_format = (get_binding_format_t*)par;
		binding_handle_get_message(get_binding_format);
	}
	else if(type == CONFIG_NODE_SET) {
		cfg_binding_format_t* cfg_binding_set = (cfg_binding_format_t*)par;
		binding_handle_setup_message(cfg_binding_set);
	}
}
// End file
