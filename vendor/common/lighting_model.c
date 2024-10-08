/********************************************************************************************************
 * @file     lighting_model.c 
 *
 * @brief    for TLSR chips
 *
 * @author	 telink
 * @date     Sep. 30, 2010
 *
 * @par      Copyright (c) 2010, Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *           
 *			 The information contained herein is confidential and proprietary property of Telink 
 * 		     Semiconductor (Shanghai) Co., Ltd. and is available under the terms 
 *			 of Commercial License Agreement between Telink Semiconductor (Shanghai) 
 *			 Co., Ltd. and the licensee in separate contract or the terms described here-in. 
 *           This heading MUST NOT be removed from this file.
 *
 * 			 Licensees are granted free, non-transferable use of the information in this 
 *			 file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided. 
 *           
 *******************************************************************************************************/
#include "../../proj/tl_common.h"
#ifndef WIN32
#include "../../proj/mcu/watchdog_i.h"
#endif 
#include "../../proj_lib/ble/ll/ll.h"
#include "../../proj_lib/ble/blt_config.h"
#include "../../vendor/common/user_config.h"
#include "app_health.h"
#include "../../proj_lib/sig_mesh/app_mesh.h"
#include "scene.h"
#include "lighting_model_LC.h"
#include "../mesh/user/binding_ctl.h"
#include "lighting_model.h"

#include "../mesh/user/debug.h"
#ifdef LIGHTING_MODEL_DBG_EN
#define DBG_LIGHTING_MODEL_SEND_STR(x)   Dbg_sendString((s8*)x)
#define DBG_LIGHTING_MODEL_SEND_INT(x)   Dbg_sendInt(x)
#define DBG_LIGHTING_MODEL_SEND_HEX(x)   Dbg_sendHex(x)
#else
#define DBG_LIGHTING_MODEL_SEND_STR(x)
#define DBG_LIGHTING_MODEL_SEND_INT(x)
#define DBG_LIGHTING_MODEL_SEND_HEX(x)
#endif

model_lightness_t       model_sig_lightness;
#if (LIGHT_TYPE_CT_EN)
model_light_ctl_t       model_sig_light_ctl;
#endif

/*
	lighting model command callback function ----------------
*/
u16 mySqrt(u32 x) 
{
   if(x == 0 || x== 1)
        return x;
   //special dispatch for the 65535 val 
   u32 magic_num=0;
   magic_num = (u32)65535*65535;
   if(x >= magic_num){
		return 65535;
   }
   u32 start = 0;
   u32 end = 65535;
   while(start < end)
   {
       u32 mid = start + (end-start)/2;
       u32 res = mid*mid;
       if(res == x)
         return mid;
       else if(res > x)
         end = mid;
       else 
         start = mid;
       if(end -start == 1)
       break;    
    }
    return start;  
}

u16 lightness_to_linear(u16 val)
{
	u32 linear =0;
	u32 linear_temp = 0;
	linear_temp = val*val;
	linear = (linear_temp)/65535;
	if((linear_temp)%65535){
		linear++;
	}
	return linear;
}

u16 linear_to_lightness(u16 val)
{
	u16 lightness =0;
	u32 sqrt_val = 0;
	sqrt_val = val*65535;
	lightness = mySqrt(sqrt_val);
	return lightness;	
}

int is_level_delta_set_op(u16 op)
{
	return ((op == G_DELTA_SET) || (op == G_DELTA_SET_NOACK));
}

int is_level_move_set_op(u16 op)
{
	return ((op == G_MOVE_SET) || (op == G_MOVE_SET_NOACK));
}

int is_lightness_set_op(u16 op)
{
	return ((op == LIGHTNESS_SET) || (op == LIGHTNESS_SET_NOACK));
}

int is_light_ctl_set_op(u16 op)
{
	return ((op == LIGHT_CTL_SET) || (op == LIGHT_CTL_SET_NOACK));
}

#if MD_SERVER_EN

s16 get_target_level_for_move_set(s16 move_delta)
{
	return (move_delta >= 0) ? 32767 : -32768;
}

s16 get_target_level_by_op(s16 target_level, s32 level, u16 op, int light_idx, int st_trans_type)
{
	light_range_s16_t range_s16 = {0};
	light_g_level_range_get(&range_s16, light_idx, st_trans_type);
    if(is_level_delta_set_op(op)){
        return get_val_with_check_range(target_level + level, range_s16.min, range_s16.max, st_trans_type);
    }else if(is_level_move_set_op(op)){
        return get_target_level_for_move_set(level);
    }else{  // ((G_LEVEL_SET == op) || (G_LEVEL_SET_NOACK == op))
        return level;
    }
}

void model_pub_check_set_bind_all(st_pub_list_t *pub_list, mesh_cb_fun_par_t *cb_par, int linear)
{
    mesh_op_resource_t *p_res = (mesh_op_resource_t *)cb_par->res;
    int light_idx = 0;
    if(SIG_MD_G_LEVEL_S == p_res->id){
        light_idx = get_light_idx_from_level_md_idx(cb_par->model_idx);
    }else{
        light_idx = cb_par->model_idx;
    }
    
    if((!PTS_TEST_EN) && pub_list->st[ST_TRANS_PUB_ONOFF]){
	    model_pub_check_set(pub_list->st[ST_TRANS_PUB_ONOFF], (u8 *)(&(model_sig_g_onoff_level.onoff_srv[light_idx])), SIG_MD_G_ONOFF_S == p_res->id);
    }
    
    if(pub_list->st[ST_TRANS_LIGHTNESS]){
        int level_set_st = pub_list->st[ST_TRANS_LIGHTNESS];
        if(PTS_TEST_EN){    // PTS LLN-BV09C: request publish onoff status, even though onoff not change.
            model_pub_check_set(level_set_st, (u8 *)(&(model_sig_g_onoff_level.onoff_srv[light_idx])), SIG_MD_G_ONOFF_S == p_res->id);
        }
#if MD_LIGHTNESS_EN
        model_sig_lightness.srv[light_idx].com.pub_2nd_state = linear;
        model_pub_check_set(level_set_st, (u8 *)(&(model_sig_lightness.srv[light_idx])), SIG_MD_LIGHTNESS_S == p_res->id);
#endif
#if MD_LIGHT_CONTROL_EN
        model_pub_check_set(level_set_st, (u8 *)(&(model_sig_light_lc.srv[light_idx])), SIG_MD_LIGHT_LC_S == p_res->id);
        if(is_light_lc_op(cb_par->op)){
            model_pub_st_cb_re_init_lc_srv(&mesh_lc_onoff_st_publish);    // re-init
        }
#endif
    }
    
#if MD_LEVEL_EN
    foreach(m,LIGHT_CNT){
        if(m == light_idx){
            foreach(i,ELE_CNT_EVERY_LIGHT){
                if(pub_list->st[i]){
                    int level_md_idx = m * ELE_CNT_EVERY_LIGHT + i;
            	    model_pub_check_set(pub_list->st[i], (u8 *)(&(model_sig_g_onoff_level.level_srv[level_md_idx])), SIG_MD_G_LEVEL_S == p_res->id);
                }
            }
            break;
        }
    }
#endif

#if LIGHT_TYPE_CT_EN
    if(pub_list->st[ST_TRANS_LIGHTNESS] || pub_list->st[ST_TRANS_CTL_TEMP]){
        int level_set_st = get_level_set_st(pub_list->st[ST_TRANS_LIGHTNESS], pub_list->st[ST_TRANS_CTL_TEMP]);
	    model_pub_check_set(level_set_st, (u8 *)(&(model_sig_light_ctl.srv[light_idx])), SIG_MD_LIGHT_CTL_S == p_res->id);    
    }
    if(pub_list->st[ST_TRANS_CTL_TEMP] || pub_list->st[ST_TRANS_CTL_D_UV]){
        int level_set_st = get_level_set_st(pub_list->st[ST_TRANS_CTL_TEMP], pub_list->st[ST_TRANS_CTL_D_UV]);
	    model_pub_check_set(level_set_st, (u8 *)(&(model_sig_light_ctl.temp[light_idx])), SIG_MD_LIGHT_CTL_TEMP_S == p_res->id);    
    }
#endif

#if LIGHT_TYPE_HSL_EN
    if(pub_list->st[ST_TRANS_LIGHTNESS] || pub_list->st[ST_TRANS_HSL_HUE] || pub_list->st[ST_TRANS_HSL_SAT]){
        int level_set_st = get_level_set_st(pub_list->st[ST_TRANS_LIGHTNESS], pub_list->st[ST_TRANS_HSL_HUE]);
        level_set_st = get_level_set_st(level_set_st, pub_list->st[ST_TRANS_HSL_SAT]);
	    model_pub_check_set(level_set_st, (u8 *)(&(model_sig_light_hsl.srv[light_idx])), SIG_MD_LIGHT_HSL_S == p_res->id);    
    }
    if(pub_list->st[ST_TRANS_HSL_HUE]){
	    model_pub_check_set(pub_list->st[ST_TRANS_HSL_HUE], (u8 *)(&(model_sig_light_hsl.hue[light_idx])), SIG_MD_LIGHT_HSL_HUE_S == p_res->id);    
    }
    if(pub_list->st[ST_TRANS_HSL_SAT]){
	    model_pub_check_set(pub_list->st[ST_TRANS_HSL_SAT], (u8 *)(&(model_sig_light_hsl.sat[light_idx])), SIG_MD_LIGHT_HSL_SAT_S == p_res->id);    
    }
#endif

#if (LIGHT_TYPE_SEL == LIGHT_TYPE_XYL)
    if(pub_list->st[ST_TRANS_LIGHTNESS] || pub_list->st[ST_TRANS_XYL_X] || pub_list->st[ST_TRANS_XYL_Y]){
        int level_set_st = get_level_set_st(pub_list->st[ST_TRANS_LIGHTNESS], pub_list->st[ST_TRANS_XYL_X]);
        level_set_st = get_level_set_st(level_set_st, pub_list->st[ST_TRANS_XYL_Y]);
	    model_pub_check_set(level_set_st, (u8 *)(&(model_sig_light_xyl.srv[light_idx])), SIG_MD_LIGHT_XYL_S == p_res->id);    
    }
#endif
}

int g_level_set(u8 *par, int par_len, u16 op, int idx, u8 retransaction, int st_trans_type, int force, st_pub_list_t *pub_list)
{
    //model_g_light_s_t *p_model = (model_g_light_s_t *)cb_par->model;
	mesh_set_trans_t set_trans = {0};
	mesh_set_trans_t *p_set_trans = 0;

	s32 delta_offset = 0;
	set_trans.op = op;
	int is_move_flag = (is_level_move_set_op(op));
	
	s32 level_adjust;
	if(is_level_delta_set_op(set_trans.op)){
		mesh_cmd_g_level_delta_t *p_set = (mesh_cmd_g_level_delta_t *)par;
		static s32 delta_last = 0;
		if(retransaction){
			delta_offset = p_set->level - delta_last;   // delta may be different from last,so update too.
			level_adjust = delta_offset;
		}else{
			level_adjust = p_set->level;
		}
		delta_last = p_set->level;
		
		if(par_len >= sizeof(mesh_cmd_g_level_delta_t)){	// existed transit_t and delay.
			if(!is_valid_transition_step(p_set->transit_t)){
				return -1;
			}
			set_trans.transit_t = p_set->transit_t;
			set_trans.delay = p_set->delay;
			p_set_trans = &set_trans;
		}
	}else{
		mesh_cmd_g_level_set_t *p_set = (mesh_cmd_g_level_set_t *)par;
		if(is_move_flag && (0 == p_set->level)){
		    return -1;
		}
		level_adjust = p_set->level;
        set_trans.transit_t = p_set->transit_t;
		
		if(par_len >= sizeof(mesh_cmd_g_level_set_t)){	// existed transit_t and delay.
			if(!is_valid_transition_step(p_set->transit_t)){
				return -1;
			}
			set_trans.delay = p_set->delay;
			p_set_trans = &set_trans;
			#if MD_LIGHT_CONTROL_EN    // for PTS LC server : MMDL / SR / LLC / BV04
            extern u8 lc_onoff_flag;
            if(lc_onoff_flag){          // comfirm later
			    model_sig_g_power_onoff.trans_time[0].val = set_trans.transit_t; // need save ?
			}
			#endif
		}
	}

	if(p_set_trans){
		if(!is_transition_need(p_set_trans->transit_t, p_set_trans->delay)){
			p_set_trans = 0;
		}
	}else{
		u8 def_transit_t = g_def_trans_time_val(idx);
		if(GET_TRANSITION_STEP(def_transit_t)){
			set_trans.transit_t = def_transit_t;
			set_trans.delay = 0;
			par_len = sizeof(mesh_cmd_g_level_delta_t);
			p_set_trans = &set_trans;
		}
	}

	s16 present_level = light_g_level_present_get(idx, st_trans_type);
	s16 target_level = light_g_level_target_get(idx, st_trans_type);
	set_trans.target_val = get_target_level_by_op(target_level, level_adjust, set_trans.op, idx, st_trans_type);
    set_trans.present_val = present_level;
    set_trans.level_move = level_adjust;
    if(!retransaction || delta_offset){
		if(p_set_trans && ((present_level != set_trans.target_val) || force)){ // force only for G_LEVEL_SET_NOACK
			light_g_level_set_idx_with_trans((u8 *)p_set_trans, idx, st_trans_type);
			// no need publish at start time now.
            get_light_pub_list(st_trans_type, present_level, set_trans.target_val, 1, pub_list);
		}else{
		    #if MD_SCENE_EN
		    if(!tansition_forced_by_recall_flag){
			    scene_target_complete_check(idx);
			}
			#endif
			
			if(is_move_flag){
			    // no action if no transition time.
			}else{
        		light_g_level_set_idx(idx, set_trans.target_val, 1, st_trans_type, pub_list);
        	}
        }
    }

	return 0;
}

u16 get_u16_val_and_remain_time(u8 idx, int st_trans_type, u8 *p_remain_t, int target_flag)
{
	mesh_cmd_g_level_st_t level_st; 
	light_g_level_get((u8 *)&level_st, idx, st_trans_type);
    if(0 == *p_remain_t){
	    *p_remain_t = level_st.remain_t;
	}
	return get_lightness_from_level(target_flag ? level_st.target_level : level_st.present_level);
}

s16 get_val_with_check_range(s32 level_target, s16 min, s16 max, int st_trans_type)
{
#if (LIGHT_TYPE_CT_EN)
    if(ST_TRANS_CTL_TEMP == st_trans_type){
        // level value is always valid for CT. 
        min = -32768;
        max = 32767;
    }
#endif
    if(level_target < min){
        // lightness would be set to 0
        if(!((ST_TRANS_LIGHTNESS == st_trans_type) && (LEVEL_OFF == level_target))){
            level_target = min;
        }
    }else if(level_target > max){
        level_target = max;
    }
    return (s16)level_target;
}
#define MMDL_SR_MLTEL_BV_02_C 0
int g_level_set_and_update_last(u8 *par, int par_len, u16 op, int idx, u8 retransaction, int st_trans_type, int force, st_pub_list_t *pub_list)
{
	mesh_cmd_g_level_delta_t set = {0};
	memcpy(&set, par, sizeof(set));
	light_range_s16_t range;
	light_g_level_range_get(&range, idx, st_trans_type);
	if(!is_level_delta_set_op(op)){
		mesh_cmd_g_level_set_t *p_level_set = (mesh_cmd_g_level_set_t *)&set;
		#if MMDL_SR_MLTEL_BV_02_C
		if(p_level_set->level == 0){
			return -1;
		}
		#endif
		p_level_set->level = get_val_with_check_range(p_level_set->level, range.min, range.max, st_trans_type);
	}
	int err = g_level_set((u8 *)&set, par_len, op, idx, retransaction, st_trans_type, force, pub_list);
	return err;
}

// ----------- lightness-------------
static inline int is_valid_lightness(u16 lightness)
{
	return 1;//(lightness != 0);
}

void mesh_level_u16_st_rsp_par_fill(mesh_cmd_lightness_st_t *rsp, u8 idx, int st_trans_type)
{
	mesh_cmd_g_level_st_t level_st; 
	light_g_level_get((u8 *)&level_st, idx, st_trans_type);

	rsp->present = get_lightness_from_level(level_st.present_level);
	rsp->target = get_lightness_from_level(level_st.target_level);
	rsp->remain_t = level_st.remain_t;
}

int mesh_tx_cmd_lightness_st(u8 idx, u16 ele_adr, u16 dst_adr, u16 op_rsp, u8 *uuid, model_common_t *pub_md)
{
	mesh_cmd_lightness_st_t rsp = {0};
	mesh_level_u16_st_rsp_par_fill(&rsp, idx, ST_TRANS_LIGHTNESS);
	 				 
	if(LIGHTNESS_LINEAR_STATUS == op_rsp){
		if(is_linear_flag(idx)){
			u16 linear_present = lightness_to_linear(rsp.present);
			rsp.target = get_light_linear_val(idx);
			if(linear_present <= rsp.target){
				rsp.present = linear_present;
			}else{
				rsp.present = rsp.target;
			}
		}else{
			rsp.present = lightness_to_linear(rsp.present);
	 		rsp.target = lightness_to_linear(rsp.target);
		}
	}
	
	u32 len = sizeof(rsp);
	if(0 == rsp.remain_t){
		len -= 3;
	}

	return mesh_tx_cmd_rsp(op_rsp, (u8 *)&rsp, len, ele_adr, dst_adr, uuid, pub_md);
}

#if MD_LIGHTNESS_EN
int mesh_lightness_st_publish_ll(u8 idx, u16 op_rsp)
{
	model_common_t *p_com_md = &model_sig_lightness.srv[idx].com;
	u16 ele_adr = p_com_md->ele_adr;
	u16 pub_adr = p_com_md->pub_adr;
	if(!pub_adr){
		return -1;
	}
	u8 *uuid = get_virtual_adr_uuid(pub_adr, p_com_md);
	#if 0 // PTS_TEST_EN,  fix SR/LLN/BV-06 in pts7_3_0.exe
	mesh_tx_cmd_lightness_st(idx, ele_adr, pub_adr, op_rsp, uuid, p_com_md);
	p_com_md->pub_2nd_state = 0;     // init
	op_rsp = LIGHTNESS_LINEAR_STATUS;
	return mesh_tx_cmd_lightness_st(idx, ele_adr, pub_adr, op_rsp, uuid, p_com_md);
	#else
	if(p_com_md->pub_2nd_state){
        p_com_md->pub_2nd_state = 0;     // init
        #if 0 // always publish lightness is OK in pts7_3_1.exe
	    op_rsp = LIGHTNESS_STATUS;
	    #endif
	}
	return mesh_tx_cmd_lightness_st(idx, ele_adr, pub_adr, op_rsp, uuid, p_com_md);
	#endif
}

int mesh_lightness_st_publish(u8 idx)
{
	return mesh_lightness_st_publish_ll(idx, LIGHTNESS_STATUS);
}
#endif

int mesh_lightness_st_rsp(mesh_cb_fun_par_t *cb_par)
{
	model_g_light_s_t *p_model = (model_g_light_s_t *)cb_par->model;
	return mesh_tx_cmd_lightness_st(cb_par->model_idx, p_model->com.ele_adr, cb_par->adr_src, cb_par->op_rsp, 0, 0);
}

int mesh_cmd_sig_lightness_get(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	return mesh_lightness_st_rsp(cb_par);
}

int level_s16_set(mesh_cmd_g_level_set_t *p_set, int par_len, u16 op, int idx, u8 retransaction, int st_trans_type, st_pub_list_t *pub_list)
{
	return g_level_set_and_update_last((u8 *)&p_set, par_len, G_LEVEL_SET_NOACK, idx, retransaction, st_trans_type, 0, pub_list);
}

int level_u16_set(mesh_cmd_lightness_set_t *p_set, int par_len, u16 op, int idx, u8 retransaction, int st_trans_type, st_pub_list_t *pub_list)
{
	mesh_cmd_g_level_set_t level_set_tmp;
	memcpy(&level_set_tmp.tid, &p_set->tid, sizeof(mesh_cmd_g_level_set_t) - OFFSETOF(mesh_cmd_g_level_set_t,tid));
	level_set_tmp.level = get_level_from_lightness(p_set->lightness);
	int len_tmp = GET_LEVEL_PAR_LEN(par_len >= sizeof(mesh_cmd_lightness_set_t));
	return g_level_set_and_update_last((u8 *)&level_set_tmp, len_tmp, G_LEVEL_SET_NOACK, idx, retransaction, st_trans_type, 0, pub_list);
}

int mesh_cmd_sig_lightness_set(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	func_handle_control_message(
				par, par_len, cb_par, LIGHTNESS_SET
			);
	return 0;
}

// ----------- lightness default-------------
int mesh_level_def_u16_st_rsp(mesh_cb_fun_par_t *cb_par, int st_trans_type)
{
    model_g_light_s_t *p_model = (model_g_light_s_t *)cb_par->model;
    mesh_cmd_lightness_def_st_t rsp = {0};
    rsp.lightness = light_g_level_def_get_u16(cb_par->model_idx, st_trans_type);
  
    return mesh_tx_cmd_rsp(cb_par->op_rsp, (u8 *)&rsp, sizeof(rsp), p_model->com.ele_adr, cb_par->adr_src, 0, 0);
}

// ----------- lightness range -------------
int mesh_range_st_rsp(u8 st, mesh_cb_fun_par_t *cb_par, int st_trans_type)
{
    model_g_light_s_t *p_model = (model_g_light_s_t *)cb_par->model;
    mesh_cmd_lightness_range_st_t rsp = {0};
    rsp.status = st;
    light_g_level_range_get_u16((light_range_u16_t *)&rsp.min, cb_par->model_idx, st_trans_type);
    return mesh_tx_cmd_rsp(cb_par->op_rsp, (u8 *)&rsp, sizeof(rsp), p_model->com.ele_adr, cb_par->adr_src, 0, 0);
}

// ----------- lightness last-------------
u16 lightness_last_get(int idx)
{
	return get_lightness_from_level(light_g_level_last_get(idx, ST_TRANS_LIGHTNESS));
}

int mesh_cmd_sig_lightness_last_get(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	model_g_light_s_t *p_model = (model_g_light_s_t *)cb_par->model;
	mesh_cmd_lightness_last_st_t rsp = {0};
	rsp.lightness = lightness_last_get(cb_par->model_idx);

	return mesh_tx_cmd_rsp(cb_par->op_rsp, (u8 *)&rsp, sizeof(mesh_cmd_lightness_last_st_t), p_model->com.ele_adr, cb_par->adr_src, 0, 0);
}

// ----------- light CTL model-------------
#if (LIGHT_TYPE_CT_EN)

int mesh_cmd_sig_light_ctl_set(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	func_handle_control_message(
				par, par_len, cb_par, LIGHT_CTL_SET
			);
	return 0;
}
#endif
#endif

//--lighting model command interface-------------------
//-----------access command--------
int is_valid_lum(u8 lum)
{
	return (lum <= 100);	// set lightness to 0 means Generic OFF,
}

int is_valid_temp_100(u8 temp100)
{
	return (temp100 <= 100);
}

int access_cmd_get_lightness(u16 adr,u32 rsp_max)
{
	u8 par[1];
	return SendOpParaDebug(adr, rsp_max, LIGHTNESS_GET, par, 0);
}

int access_cmd_set_lightness_ll(u16 op, u16 adr, u8 rsp_max, u16 lightness, transition_par_t *trs_par)
{
	mesh_cmd_lightness_set_t par = {0};
	u32 par_len = OFFSETOF(mesh_cmd_lightness_set_t,transit_t);
	par.lightness = lightness;
	if(trs_par){
		par_len = sizeof(mesh_cmd_lightness_set_t);
		memcpy(&par.transit_t, trs_par, 2);
	}
	
	return SendOpParaDebug(adr, rsp_max, op, 
						   (u8 *)&par, par_len);
}

int access_cmd_set_lightness(u16 adr, u8 rsp_max, u16 lightness, int ack, transition_par_t *trs_par)
{
	u16 op = ack ? LIGHTNESS_SET : LIGHTNESS_SET_NOACK;
	return access_cmd_set_lightness_ll(op, adr, rsp_max, lightness, trs_par);
}

int access_set_lum(u16 adr, u8 rsp_max, u8 lum, int ack)
{
	if(!is_valid_lum(lum)){
		return -2;
	}

	#if 1
	return access_cmd_set_lightness(adr, rsp_max, lum2_lightness(lum), ack, 0);
	#else
	return access_cmd_set_level(adr, rsp_max, lum2level(lum), ack, 0);
	#endif
}

int access_get_lum(u16 adr, u8 rsp_max)
{
	return access_cmd_get_lightness(adr, rsp_max);
}

int access_cmd_get_lightness_linear(u16 adr,u32 rsp_max)
{
	u8 par[1];
	return SendOpParaDebug(adr, rsp_max, LIGHTNESS_LINEAR_GET, par, 0);
}

int access_cmd_set_lightness_linear(u16 adr, u8 rsp_max, u16 linear, int ack, transition_par_t *trs_par)
{
	u16 op = ack ? LIGHTNESS_LINEAR_SET : LIGHTNESS_LINEAR_SET_NOACK;
	return access_cmd_set_lightness_ll(op, adr, rsp_max, linear, trs_par);
}

int access_cmd_get_lightness_last(u16 adr,u32 rsp_max)
{
	u8 par[1];
	return SendOpParaDebug(adr, rsp_max, LIGHTNESS_LAST_GET, par, 0);
}

int access_cmd_set_lightness_def(u16 adr, u8 rsp_max, u16 lightness, int ack)
{
	mesh_cmd_lightness_def_set_t par = {0};
	u32 par_len = sizeof(mesh_cmd_lightness_def_set_t);
	par.lightness = lightness;
	
	return SendOpParaDebug(adr, rsp_max, ack ? LIGHTNESS_DEFULT_SET : LIGHTNESS_DEFULT_SET_NOACK, 
						   (u8 *)&par, par_len);
}

int access_cmd_get_lightness_def(u16 adr,u32 rsp_max)
{
	u8 par[1];
	return SendOpParaDebug(adr, rsp_max, LIGHTNESS_DEFULT_GET, par, 0);
}

int access_cmd_set_lightness_range(u16 adr, u8 rsp_max, u16 min, u16 max, int ack)
{
	mesh_cmd_lightness_range_set_t par = {0};
	u32 par_len = sizeof(mesh_cmd_lightness_range_set_t);
	par.min = min;
	par.max = max;
	
	return SendOpParaDebug(adr, rsp_max, ack ? LIGHTNESS_RANGE_SET : LIGHTNESS_RANGE_SET_NOACK, 
						   (u8 *)&par, par_len);
}

int access_cmd_get_lightness_range(u16 adr,u32 rsp_max)
{
	u8 par[1];
	return SendOpParaDebug(adr, rsp_max, LIGHTNESS_RANGE_GET, par, 0);
}

// light ctl
#if (LIGHT_TYPE_CT_EN)
int access_cmd_get_light_ctl(u16 adr,u32 rsp_max)
{
	u8 par[1];
	return SendOpParaDebug(adr, rsp_max, LIGHT_CTL_GET, par, 0);
}

int access_cmd_set_light_ctl(u16 adr, u8 rsp_max, u16 lightness, u16 temp, int ack, transition_par_t *trs_par)
{
	mesh_cmd_light_ctl_set_t par = {0};
	u32 par_len = OFFSETOF(mesh_cmd_light_ctl_set_t,transit_t);
	par.lightness = lightness;
	par.temp = temp;
	par.delta_uv = 0;	// comfirm later
	if(trs_par){
		par_len = sizeof(mesh_cmd_light_ctl_set_t);
		memcpy(&par.transit_t, trs_par, 2);
	}
	
	return SendOpParaDebug(adr, rsp_max, ack ? LIGHT_CTL_SET : LIGHT_CTL_SET_NOACK, 
						   (u8 *)&par, par_len);
}

int access_cmd_set_light_ctl_100(u16 adr, u8 rsp_max, u8 lum, u8 temp100, int ack)
{
	if((!is_valid_lum(lum)) || (!is_valid_temp_100(temp100))){
		return -2;
	}

	u16 lightness = lum2_lightness(lum);
	u16 temp = temp100_to_temp(temp100);
	return access_cmd_set_light_ctl(adr, rsp_max, lightness, temp, ack, 0);
}

int access_cmd_get_light_ctl_temp(u16 adr,u32 rsp_max)
{
	u8 par[1];
	return SendOpParaDebug(adr, rsp_max, LIGHT_CTL_TEMP_GET, par, 0);
}

int access_cmd_set_light_ctl_temp(u16 adr, u8 rsp_max, u16 temp, int ack, transition_par_t *trs_par)
{
	mesh_cmd_light_ctl_temp_set_t par = {0};
	u32 par_len = OFFSETOF(mesh_cmd_light_ctl_temp_set_t,transit_t);
	par.temp = temp;
	par.delta_uv = 0;	// comfirm later
	if(trs_par){
		par_len = sizeof(mesh_cmd_light_ctl_temp_set_t);
		memcpy(&par.transit_t, trs_par, 2);
	}
	
	return SendOpParaDebug(adr, rsp_max, ack ? LIGHT_CTL_TEMP_SET : LIGHT_CTL_TEMP_SET_NOACK, 
						   (u8 *)&par, par_len);
}

int access_cmd_set_light_ctl_temp_100(u16 adr, u8 rsp_max, u8 temp100, int ack)
{
	if(!is_valid_temp_100(temp100)){
		return -2;
	}

	u16 temp = temp100_to_temp(temp100);
	return access_cmd_set_light_ctl_temp(adr, rsp_max, temp, ack, 0);
}
#endif
//--lighting model command interface end----------------
