/*
 * binding_ctl.c
 *
 *  Created on: Mar 18, 2022
 *      Author: DungTranBK
 */

#include "../../../proj/tl_common.h"
#include "../../common/mesh_node.h"
#include "../../../proj_lib/sig_mesh/app_mesh.h"
#include "../../common/lighting_model.h"
#include "../../common/lighting_model_HSL.h"
#include "utilities.h"
#include "handle_ev.h"
#include "in_out.h"
#include "sw_config.h"
#include "binding.h"
#include "binding_ctl.h"

#include "debug.h"
#ifdef  BIND_CONTROL_DBG_EN
	#define DBG_BIND_CTL_SEND_STR(x)   Dbg_sendString((s8*)x)
	#define DBG_BIND_CTL_SEND_INT(x)   Dbg_sendInt(x)
	#define DBG_BIND_CTL_SEND_HEX(x)   Dbg_sendHex(x)
	#define DBG_BIND_CTL_SEND_BYTE(x)  Dbg_sendOneByteHex(x)
#else
	#define DBG_BIND_CTL_SEND_STR(x)
	#define DBG_BIND_CTL_SEND_INT(x)
	#define DBG_BIND_CTL_SEND_HEX(x)
	#define DBG_BIND_CTL_SEND_BYTE(x)
#endif

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

#define P_ST_TRANS(idx, type)	(&light_res_sw[idx].trans[type])

u8 state_of_group_binding[LIGHT_CNT] = {
		SUB_UNKNOWN,
		#if ELE_CNT > 1
		SUB_UNKNOWN,
		#endif
};

u8 incomming_st[ELE_CNT] = {
		G_ONOFF_RSV,
		#if ELE_CNT > 1
		G_ONOFF_RSV,
		#endif
};
u8 scene_active_arr[ELE_CNT] = {
		FALSE,
	#if ELE_CNT > 1
		FALSE,
	#endif
};
u8 force_control_binding[ELE_CNT] =
{
		FALSE,
	#if ELE_CNT > 1
		FALSE,
	#endif
};
/******************************************************************************/
/*                             PRIVATE FUNCS                                  */
/******************************************************************************/

static BOOL check_en_control(u16 model_idx, u16 adr_dst);

/******************************************************************************/
/*                        EXPORT FUNCTIONS DECLERATION                        */
/******************************************************************************/

/**
 * @func    handle_mesh_cmd_sig_g_on_off_set
 * @brief
 * @param
 * @retval  None
 */
int handle_mesh_cmd_sig_g_on_off_set(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	if((check_en_control(cb_par->model_idx, cb_par->adr_dst) == FALSE)) {
		return -1;
	}
	if((sw_get_en_disable_group_default
				(cb_par->model_idx) == FALSE)
 						&& (cb_par->adr_dst > ADR_ALL_PROXY)) {
		return -1;
	}
    #if (SWITCH_ENABLE_BINDING  \
			|| EN_EVT_ON_OFF_CONTROL_EXECUTION)
	foreach(i, NUMBER_INPUT) {
		if(cb_par->adr_src == ele_adr_primary + i){
			DBG_BIND_CTL_SEND_STR("\n___SRC_INVALID");
			return -1;
		}
	}
    #endif
	if(cb_par->model_idx >= NUMBER_INPUT) {
		return -1;
	}
	mesh_cmd_g_onoff_set_t *p_set = (mesh_cmd_g_onoff_set_t *)par;
    #if (SWITCH_ENABLE_BINDING    \
			|| EN_EVT_ON_OFF_CONTROL_EXECUTION)
	if(par_len == sizeof(mesh_cmd_g_onoff_set_t)+1) {
	    if(par[par_len - 1] == 0x0) {
			st_transition_t *p_trans =   \
					P_ST_TRANS(cb_par->model_idx, ST_TRANS_LIGHTNESS);
			if(get_onoff_from_level(p_trans->present) == p_set->onoff) {
				__delay_ms(50);
				force_control_binding[cb_par->model_idx] = FALSE;
				return -1;
			}
	    }
	}
    #endif
	update_control_message_params(cb_par->model_idx, cb_par->adr_src, cb_par->adr_dst, G_ONOFF_SET);
	binding_control_onoff(cb_par->model_idx, p_set->onoff, TRUE);
	return 0;
}

/**
 * @func    binding_control_onoff
 * @brief
 * @param   None
 * @retval  None
 */
void binding_control_onoff(u16 model_idx, BOOL state, BOOL update_en)
{
	if((check_en_control           \
			(model_idx, nwk_control_msg_para[model_idx].dst) == TRUE)    \
						|| (scene_active_arr[model_idx] == TRUE)) {
		scene_active_arr[model_idx] = FALSE;

		mesh_cmd_g_onoff_set_t g_onoff_set;
		g_onoff_set.onoff = state;
		g_onoff_set.tid = 0;
		g_onoff_set.transit_t  = g_onoff_set.delay = 0;
		io_handle_cmd_sig_on_off(&g_onoff_set, sizeof(mesh_cmd_g_onoff_set_t), 0, model_idx, 0, (void*)0);

		if(update_en == TRUE) {
			incomming_st[model_idx] = state;
		}
	}
}

/**
 * @func    check_en_control
 * @brief
 * @param   None
 * @retval  None
 */
static BOOL check_en_control(u16 model_idx, u16 adr_dst)
{
#if 0
	if(sw_config_get_sw_mode(model_idx) == TOGGLE_SWITCH_TYPE) {
		return TRUE;
	}
	else {
		if((adr_dst <= ADR_FIXED_GROUP_START)) {
			return TRUE;
		}
	}
	return FALSE;
#else
	return TRUE;
#endif
}
/**
 * @func    func_handle_control_message
 * @brief
 * @param
 * @retval  None
 */
int func_handle_control_message(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par, u16 op)
{
	DBG_BIND_CTL_SEND_STR("\n func_handle_control_message");

	u16 binding_adr =  \
			get_group_binding_adr(cb_par->model_idx);
	if(binding_adr != ADR_UNASSIGNED) {
		if(cb_par->adr_dst == binding_adr) {
			u8 st_control = G_ONOFF_RSV;
			if(op == LIGHTNESS_SET) {
				mesh_cmd_lightness_set_t* p_set = (mesh_cmd_lightness_set_t*)par;
				st_control = (p_set->lightness == LUM_OFF)?G_OFF:G_ON;
			}
			else if(op == LIGHT_CTL_SET) {
				mesh_cmd_light_ctl_set_t *p_set = (mesh_cmd_light_ctl_set_t *)par;
				st_control = (p_set->lightness == LUM_OFF)?G_OFF:G_ON;
			}
			else if(op == LIGHT_HSL_SET) {
				mesh_cmd_light_hsl_set_t *p_set = (mesh_cmd_light_hsl_set_t *)par;
				st_control = (p_set->lightness == LUM_OFF)?G_OFF:G_ON;
			}
			if(st_control < G_ONOFF_RSV)
			{
				BOOL en_send = TRUE;
				st_transition_t *p_trans = P_ST_TRANS(cb_par->model_idx, ST_TRANS_LIGHTNESS);
				if(p_trans->target == LEVEL_OFF) {
					if(st_control == G_OFF) {
						control_binding_st[cb_par->model_idx].cnt_same++;
						if(control_binding_st[cb_par->model_idx].cnt_same >= CONTROL_BDG_SAME_ST_CNT_MAX) {
							en_send = FALSE;
						}
					}
					else {
						control_binding_st[cb_par->model_idx].cnt_same = 0;
					}
				}
				else {
					if(st_control == G_ON) {
						control_binding_st[cb_par->model_idx].cnt_same++;
						if(control_binding_st[cb_par->model_idx].cnt_same >= CONTROL_BDG_SAME_ST_CNT_MAX) {
							en_send = FALSE;
						}
					}
					else {
						control_binding_st[cb_par->model_idx].cnt_same = 0;
					}
				}
				if(en_send == TRUE) {
					if(check_en_control(cb_par->model_idx, cb_par->adr_dst) == TRUE) {
						update_control_message_params(
								cb_par->model_idx,cb_par->adr_src,cb_par->adr_dst,G_ONOFF_SET
							);
						binding_control_onoff(
								cb_par->model_idx, st_control, TRUE
							);
						DBG_BIND_CTL_SEND_STR("\n Update Control Msg Params");
					}
				}
			}
			return 0;
		}
	}
	return -1;
}
/**
 * @func    binding_control
 * @brief
 * @param
 * @retval  None
 */
void binding_control(u8 model_idx, BOOL status)
{
	DBG_BIND_CTL_SEND_STR("\n binding_control: ");

	if(model_idx < ELE_CNT) {
		u16 binding_addr = get_group_binding_adr(model_idx);

		DBG_BIND_CTL_SEND_STR("\n PASS_ELE");

		if(binding_addr != ADR_UNASSIGNED) {

			DBG_BIND_CTL_SEND_STR("\n PASS_ADDR");

			BOOL enable_binding = FALSE;
			// CLEAN force control
			if(nwk_control_msg_para[model_idx].dst == binding_addr) {
				force_control_binding[model_idx] = FALSE;
			}
			// POWER ON
			if(state_of_group_binding[model_idx] == SUB_UNKNOWN) {
				state_of_group_binding[model_idx] = status;
				return;
			}

			DBG_BIND_CTL_SEND_STR("\n G_ST0: ");
			DBG_BIND_CTL_SEND_INT(state_of_group_binding[model_idx]);

			// Check Binding
			if(((state_of_group_binding[model_idx] != status)   \
						&& (nwk_control_msg_para[model_idx].dst != binding_addr)   \
								&& (nwk_control_msg_para[model_idx].dst < ADR_ALL_PROXY))
										|| (force_control_binding[model_idx] == TRUE)) {

				DBG_BIND_CTL_SEND_STR("\n ENABLE Binding");

				enable_binding = TRUE;
				state_of_group_binding[model_idx] = status;
				// INTERNAL
				foreach(i, ELE_CNT) {
					if(i != model_idx) {
						u16 temp_binding = get_group_binding_adr(i);
						if((temp_binding != ADR_UNASSIGNED) && (temp_binding == binding_addr)) {
							// MANUAL CONTROL
							update_control_message_params(i, ele_adr_primary + model_idx, binding_addr, G_ONOFF_SET);
							binding_control_onoff(i, status, TRUE);
							state_of_group_binding[i] = status;
						}
					}
				}
			}
			else if((nwk_control_msg_para[model_idx].dst == binding_addr)    \
						|| (nwk_control_msg_para[model_idx].dst >= ADR_ALL_PROXY)) {
				state_of_group_binding[model_idx] = status;
			}

			DBG_BIND_CTL_SEND_STR("\n - P1, DST: ");
			DBG_BIND_CTL_SEND_HEX(nwk_control_msg_para[model_idx].dst);

			DBG_BIND_CTL_SEND_STR(", STATUS: ");
			DBG_BIND_CTL_SEND_INT(status);

			DBG_BIND_CTL_SEND_STR(", G_ST: ");
			DBG_BIND_CTL_SEND_INT(state_of_group_binding[model_idx]);

			DBG_BIND_CTL_SEND_STR(", FORCE_CONTROL: ");
			DBG_BIND_CTL_SEND_INT(force_control_binding[model_idx]);

			DBG_BIND_CTL_SEND_STR(", en_binding: ");
			DBG_BIND_CTL_SEND_INT(enable_binding);

			// Control
			if(enable_binding == TRUE) {
				mesh_cmd_g_onoff_set_t onoff_set;
				onoff_set.onoff = status;
				onoff_set.tid = 0;
				onoff_set.transit_t = 0x0A;
				onoff_set.delay = 1;
				u8 len = sizeof(mesh_cmd_g_onoff_set_t), par[len+1];
				memcpy((u8*)&par, (u8*)&onoff_set, len);
				if(force_control_binding[model_idx] == TRUE) {
					par[len] = 0x0;
					len++;
					force_control_binding[model_idx] = FALSE;
				}
				mesh_tx_cmd_rsp(
						G_ONOFF_SET_NOACK,
						(u8 *)par,
						len,
						ele_adr_primary + model_idx,
						binding_addr,
						0,
						0
					);

				DBG_BIND_CTL_SEND_STR("\n - P2, IDX: ");
				DBG_BIND_CTL_SEND_INT(model_idx);
				DBG_BIND_CTL_SEND_STR(", ST: ");
				DBG_BIND_CTL_SEND_INT(status);

			}
		}
	}
}

// End file
