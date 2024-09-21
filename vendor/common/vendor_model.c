/********************************************************************************************************
 * @file     vendor_model.c 
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
#if !WIN32
#include "../../proj/mcu/watchdog_i.h"
#endif 
#include "../../proj_lib/ble/ll/ll.h"
#include "../../proj_lib/ble/blt_config.h"
#include "../../vendor/common/user_config.h"
#include "app_health.h"
#include "../../proj_lib/sig_mesh/app_mesh.h"
#include "vendor_model.h"
#include "fast_provision_model.h"
#include "../mesh/user/factory_reset.h"
#include "../mesh/user/utilities.h"
#include "../mesh/user/config_board.h"
#include "../mesh/user/in_out.h"
#include "../mesh/user/binding.h"
#include "../mesh/user/sw_config.h"
#include "../mesh/user/binding_ctl.h"
#include "lighting_model.h"
#include "generic_model.h"
#include "scene.h"

#include "../mesh/user/debug.h"
#ifdef  VENDOR_MODEL_DBG_EN
#define DBG_VENDOR_MODEL_SEND_STR(x)     Dbg_sendString((s8*)x)
#define DBG_VENDOR_MODEL_SEND_INT(x)     Dbg_sendInt(x)
#define DBG_VENDOR_MODEL_SEND_HEX(x)     Dbg_sendHex(x)
#define DBG_VENDOR_MODEL_SEND_HEX_ONE(x) Dbg_sendHexOneByte(x);
#else
#define DBG_VENDOR_MODEL_SEND_STR(x)
#define DBG_VENDOR_MODEL_SEND_INT(x)
#define DBG_VENDOR_MODEL_SEND_HEX(x)
#define DBG_VENDOR_MODEL_SEND_HEX_ONE(x)
#endif

typeVendor_HandleSetupExecutionSceneCallbackFunc pvVendor_HandleSetupExecutionScene = NULL;

#if ALI_MD_TIME_EN
#include "user_ali_time.h"
#endif

#if (VENDOR_MD_NORMAL_EN)
model_vd_light_t       	model_vd_light;

#if (DUAL_VENDOR_EN)
STATIC_ASSERT((VENDOR_MD_LIGHT_S && 0xffff) != VENDOR_ID_MI);
#if 1   // if not equal, please modify id value in traversal_cps_reset_vendor_id();
STATIC_ASSERT((VENDOR_MD_LIGHT_S && 0xffff0000) == (MIOT_SEPC_VENDOR_MODEL_SER && 0xffff0000));
STATIC_ASSERT((VENDOR_MD_LIGHT_C && 0xffff0000) == (MIOT_SEPC_VENDOR_MODEL_CLI && 0xffff0000));
STATIC_ASSERT((VENDOR_MD_LIGHT_S2 && 0xffff0000) == (MIOT_VENDOR_MD_SER && 0xffff0000));
#endif
#endif
/*
	vendor command callback function ----------------
*/
#if MD_SERVER_EN
#if DEBUG_VENDOR_CMD_EN
u8 vd_onoff_state[ELE_CNT] = {0};

int vd_light_onoff_idx(int idx, int on)
{
    light_onoff_all(on);     // not must, just for indication
    if(vd_onoff_state[idx] != on){
        vd_onoff_state[idx] = on;
        return 1;
    }else{
        return 0;
    }
}

void vd_light_onoff_st_rsp_par_fill(vd_light_onoff_st_t *rsp, u8 idx)
{
    rsp->present_onoff = vd_onoff_state[idx];
}

int vd_light_tx_cmd_onoff_st(u8 idx, u16 ele_adr, u16 dst_adr, u8 *uuid, model_common_t *pub_md)
{
    vd_light_onoff_st_t rsp;
    vd_light_onoff_st_rsp_par_fill(&rsp, idx);
    return mesh_tx_cmd_rsp(VD_LIGHT_ONOFF_STATUS, (u8 *)&rsp, sizeof(rsp), ele_adr, dst_adr, uuid, pub_md);
}

int vd_light_onoff_st_rsp(mesh_cb_fun_par_t *cb_par)
{
    model_g_light_s_t *p_model = (model_g_light_s_t *)cb_par->model;
    return vd_light_tx_cmd_onoff_st(cb_par->model_idx, p_model->com.ele_adr, cb_par->adr_src, 0, 0);
}

int vd_light_onoff_st_publish(u8 idx)
{
	model_common_t *p_com_md = &model_vd_light.srv[idx].com;
	u16 ele_adr = p_com_md->ele_adr;
	u16 pub_adr = p_com_md->pub_adr;
	if(!pub_adr){
		return -1;
	}
	u8 *uuid = get_virtual_adr_uuid(pub_adr, p_com_md);
    return vd_light_tx_cmd_onoff_st(idx, ele_adr, pub_adr, uuid, p_com_md);
}

int cb_vd_light_onoff_get(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
    return vd_light_onoff_st_rsp(cb_par);
}

int cb_vd_light_onoff_set(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	int err = -1;
	int pub_flag = 0;
    //model_g_light_s_t *p_model = (model_g_light_s_t *)cb_par->model;
    vd_light_onoff_set_t *p_set = (vd_light_onoff_set_t *)par;

    if(!cb_par->retransaction){
        if(p_set->onoff < G_ONOFF_RSV){
            pub_flag = vd_light_onoff_idx(cb_par->model_idx, p_set->onoff);
        }
    }
    
    if(VD_LIGHT_ONOFF_SET_NOACK != cb_par->op){
        err = vd_light_onoff_st_rsp(cb_par);
    }else{
        err = 0;
    }

    if(pub_flag){
        model_pub_check_set(ST_G_LEVEL_SET_PUB_NOW, cb_par->model, 1);
    }
    
    return err;
}
#endif

#if SPIRIT_VENDOR_EN
vd_msg_attr_t vd_msg_attr[ATTR_TYPE_MAX_NUM]={
	{ATTR_TARGET_TEMP},
	{ATTR_CURRENT_TEMP},
};

#define SLEEP_INTERVAL 		1000*1000
#define LISTEN_INTERVAL		60*1000  	
#define INDICATE_RETRY_CNT 	6

mesh_tx_indication_t mesh_indication_retry;
mesh_sleep_pre_t		mesh_sleep_time={0, SLEEP_INTERVAL+LISTEN_INTERVAL,SLEEP_INTERVAL};

void vd_msg_attr_indica_retry_start(u16 interval_ms)
{
	mesh_indication_retry.retry_cnt = INDICATE_RETRY_CNT;
	mesh_indication_retry.busy = 1;
    mesh_indication_retry.tick = clock_time();
	mesh_indication_retry.interval_tick = interval_ms*1000*sys_tick_per_us;
}

void vd_msg_attr_indica_retry_stop()
{
	mesh_indication_retry.busy = 0;
}

int mesh_tx_cmd_indica_retry(u16 op, u8 *par, u32 par_len, u16 adr_src, u16 adr_dst, int rsp_max)
{
	int ret = 0;
	u16 retry_interval = 0;
	
	material_tx_cmd_t mat;
	mesh_match_type_t match_type;
	u8 nk_array_idx = get_nk_arr_idx_first_valid();
	u8 ak_array_idx = get_ak_arr_idx_first_valid(nk_array_idx);
	set_material_tx_cmd(&mat, op, par, par_len, adr_src, adr_dst, g_reliable_retry_cnt_def, rsp_max, 0, nk_array_idx, ak_array_idx, 0);
    mesh_match_group_mac(&match_type, mat.adr_dst, mat.op, 1, mat.adr_src);
	
	memcpy(&mesh_indication_retry.mat, &mat, sizeof(material_tx_cmd_t));
	memcpy(&mesh_indication_retry.match_type, &match_type,sizeof(mesh_indication_retry.match_type));
	memset(mesh_indication_retry.ac_par, 0, sizeof(mesh_indication_retry.ac_par));
    memcpy(mesh_indication_retry.ac_par, mat.par, mat.par_len);
    mesh_indication_retry.mat.par = mesh_indication_retry.ac_par;

	ret = mesh_tx_cmd(&mat);
	retry_interval = (par_len+7)/8 * CMD_INTERVAL_MS;
	vd_msg_attr_indica_retry_start(retry_interval);
	u8 tid_pos=0;
	if(is_cmd_with_tid_vendor(&tid_pos, op, 0)){
		if(mesh_indication_retry.ac_par[tid_pos] == 0){ //if tid par is 0,use internal tid value
			u32 ele_idx = get_ele_idx(adr_src);
			mesh_indication_retry.tid = mesh_tid.tx[ele_idx];
			mesh_indication_retry.ac_par[tid_pos] = mesh_indication_retry.tid;
		}
	}
	
	return ret;
}

int access_cmd_attr_indication(u16 op, u16 adr_dst, u16 attr_type, u8 *attr_par, u8 par_len)
{
	u8 ret = -1;
	vd_msg_attr_set_t par;
	if(mesh_indication_retry.busy){
		return ret;
	}

	if(!is_unicast_adr(adr_dst)){
		mesh_tx_segment_finished();
	}
	
	par.attr_type = attr_type;
	par_len = par_len>ATTR_PAR_MAX_LEN?ATTR_PAR_MAX_LEN:par_len;
	par.tid = 0;// use internal tid
	memcpy(&par.attr_par, attr_par, par_len);
	par_len += OFFSETOF(vd_msg_attr_set_t,attr_par);
	
	u8 network_transmit = model_sig_cfg_s.nw_transmit.val;
	model_sig_cfg_s.nw_transmit.val = 0x07;
	ret = mesh_tx_cmd_indica_retry(op, (u8 *)&par, par_len, ele_adr_primary, adr_dst, 1);
	model_sig_cfg_s.nw_transmit.val = network_transmit;
	return ret;

}

void mesh_tx_indication_tick_refresh()
{
	mesh_indication_retry.tick = clock_time();
}

void mesh_tx_indication_proc()
{
 	if(mesh_indication_retry.busy){
	    if(clock_time_exceed(mesh_indication_retry.tick, mesh_indication_retry.interval_tick/sys_tick_per_us)){
			mesh_tx_indication_tick_refresh();
	        if(mesh_indication_retry.retry_cnt){
	            mesh_indication_retry.retry_cnt--;
				mesh_indication_retry.tick = clock_time();
				if(!is_unicast_adr(mesh_indication_retry.mat.adr_dst)){
					mesh_tx_segment_finished();
				}
	            mesh_tx_cmd2_access(&mesh_indication_retry.mat, 0, &mesh_indication_retry.match_type);
	        }else{            			
	        	mesh_indication_retry.busy = 0;
	        }
	    }
    }	
}

u32 get_attr_type_index(u16 attr_type)
{
	for(u8 i=0; i<ATTR_TYPE_MAX_NUM; i++){
		if(attr_type == vd_msg_attr[i].attr_type){
			return i;
		}
	}
	
	return ATTR_TYPE_NOT_EXIST;
}

u8 get_attr_para_len(u16 attr_type)
{
	u8 len = 0;
	switch(attr_type){
		case ATTR_TARGET_TEMP:
			len = 2;
			break;
		default:
			break;
	}

	return len;
}

u8 is_rsp_cmd_tid = 0;
int vd_msg_tx_attr_st(u8 idx, u16 ele_adr, u16 dst_adr, u8 *uuid, model_common_t *pub_md, u8 *par)
{
	u8 ret = -1;
	vd_msg_attr_sts_t rsp;
	vd_msg_attr_sts_t *p_attr = (vd_msg_attr_sts_t *) par;	
   	u8 attr_index = get_attr_type_index(p_attr->attr_type);
	u8 attr_len = get_attr_para_len(p_attr->attr_type);
	
	rsp.attr_type = p_attr->attr_type;
	rsp.tid = p_attr->tid;
	if(ATTR_TYPE_NOT_EXIST != attr_index){
		memcpy(rsp.attr_par, vd_msg_attr[attr_index].attr_par, attr_len);
	}
	else{
		attr_len = 1;
		rsp.attr_type = ATTR_ERR_CODE;
		rsp.err_code = ATTR_NOT_SUPPORT;
	}
	is_rsp_cmd_tid = 1;
    ret = mesh_tx_cmd_rsp(VD_MSG_ATTR_STS, (u8 *)&rsp, OFFSETOF(vd_msg_attr_sts_t, attr_par)+attr_len, ele_adr, dst_adr, uuid, pub_md);
	is_rsp_cmd_tid = 0;
	return ret;
}

int vd_msg_attr_st_rsp(mesh_cb_fun_par_t *cb_par, u8 *par)
{
    model_g_light_s_t *p_model = (model_g_light_s_t *)cb_par->model;
	
    return vd_msg_tx_attr_st(cb_par->model_idx, p_model->com.ele_adr, cb_par->adr_src, 0, 0, par);
}

int cb_vd_msg_attr_get(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	vd_msg_attr_get_t *p_get = (vd_msg_attr_get_t *)par;
	switch (p_get->attr_type) {
#if ALI_MD_TIME_EN
	case ALI_VD_MD_TIME_ATTR_TYPE_TIME:
		return ali_mesh_unix_time_get(par, par_len, cb_par);
	case ALI_VD_MD_TIME_ATTR_TYPE_TIME_ZONE:
		return ali_mesh_timezone_get(par, par_len, cb_par);
	case ALI_VD_MD_TIME_ATTR_TYPE_SNTP:
		return ali_mesh_sntp_get(par, par_len, cb_par);
	case ALI_VD_MD_TIME_ATTR_TYPE_TIMING:
		return ali_mesh_timing_get(par, par_len, cb_par);
#endif
	}
    return vd_msg_attr_st_rsp(cb_par, par);
}

int cb_vd_msg_attr_set(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	int err = -1;
    vd_msg_attr_set_t *p_set = (vd_msg_attr_set_t *)par;
#if 0
    u16 attr_type = p_set->attr_type;
		
	if(par_len > OFFSETOF(vd_msg_attr_set_t, attr_par) + ATTR_PAR_MAX_LEN){
		par_len = ATTR_PAR_MAX_LEN;
	}

	u8 attr_index = get_attr_type_index(attr_type);
	u8 attr_len = get_attr_para_len(attr_type);
	
	if(!cb_par->retransaction){
		if(ATTR_TYPE_NOT_EXIST != attr_index){
			LOG_MSG_INFO(TL_LOG_NODE_SDK,0, 0,"attr_type exist",0);
			if(attr_type) // for err_code test
			memcpy(vd_msg_attr[attr_index].attr_par, p_set->attr_par,attr_len);
		}
	}
	   
    if(VD_MSG_ATTR_SET_NACK != cb_par->op){
        err = vd_msg_attr_st_rsp(cb_par, par);
    }else{
        err = 0;
    }
#else
    switch(p_set->attr_type) {
#if ALI_MD_TIME_EN
    case ALI_VD_MD_TIME_ATTR_TYPE_TIME:
    	return ali_mesh_unix_time_set(par, par_len, cb_par);
    case ALI_VD_MD_TIME_ATTR_TYPE_TIME_ZONE:
    	return ali_mesh_timezone_set(par, par_len, cb_par);
    case ALI_VD_MD_TIME_ATTR_TYPE_SNTP:
    	return ali_mesh_sntp_set(par, par_len, cb_par);
    case ALI_VD_MD_TIME_ATTR_TYPE_TIMING:
    	return ali_mesh_timing_set(par, par_len, cb_par);
    case ALI_VD_MD_TIME_ATTR_TYPE_CYCLE_TIMING:
    	return ali_mesh_cycle_timing_set(par, par_len, cb_par);
    case ALI_VD_MD_TIME_ATTR_TYPE_CYCLE_TIMING_DEL:
    	return ali_mesh_timing_del(par, par_len, cb_par);
#endif
    }
#endif
    return err;
}

int cb_vd_msg_attr_confirm(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
    int err = 0;
	
	if(!is_own_ele(cb_par->adr_src) && (mesh_indication_retry.tid == par[0])){
		vd_msg_attr_indica_retry_stop();
	}
	
    if(cb_par->model){  // model may be Null for status message
        //model_client_common_t *p_model = (model_client_common_t *)(cb_par->model);
    }
    return err;
}

#if ALI_MD_TIME_EN

int cb_vd_msg_attr_upd_time_req(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	return 0;
}


int cb_vd_msg_attr_upd_time_rsp(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
    vd_msg_attr_set_t *p_set = (vd_msg_attr_set_t *)par;
	switch(p_set->attr_type) {
	case ALI_VD_MD_TIME_ATTR_TYPE_TIME:
		return ali_mesh_unix_time_set(par, par_len, cb_par);
	}
	return 0;
}

#endif

#endif
#else
#define cb_vd_light_onoff_get       (0)
#define cb_vd_light_onoff_set       (0)
#define cb_vd_msg_attr_get          (0)
#define cb_vd_msg_attr_set          (0)
#define cb_vd_msg_attr_confirm      (0)
#if ALI_MD_TIME_EN
#define cb_vd_msg_attr_upd_time_req	(0)
#define cb_vd_msg_attr_upd_time_rsp	(0)
#endif
#endif

#if MD_CLIENT_EN
int cb_vd_light_onoff_status(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
    int err = 0;
    if(cb_par->model){  // model may be Null for status message
        //model_client_common_t *p_model = (model_client_common_t *)(cb_par->model);
    }
    return err;
}

int cb_vd_msg_attr_status(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
    int err = 0;
    if(cb_par->model){  // model may be Null for status message
        //model_client_common_t *p_model = (model_client_common_t *)(cb_par->model);
    }
    return err;
}
#else
#define cb_vd_light_onoff_status        (0)
#define cb_vd_msg_attr_status           (0)
#endif

// ------ both server or GW / APP support --------
#if SPIRIT_VENDOR_EN
int vd_msg_tx_attr_confirm(u8 idx, u16 ele_adr, u16 dst_adr, u8 *uuid, model_common_t *pub_md, u16 attr_type)
{
	vd_msg_attr_confirm_t rsp;
	
	rsp.attr_type = attr_type;
    return mesh_tx_cmd_rsp(VD_MSG_ATTR_CONFIRM, (u8 *)&rsp, sizeof(vd_msg_attr_confirm_t), ele_adr, dst_adr, uuid, pub_md);
}

int vd_msg_attr_confirm_rsp(mesh_cb_fun_par_t *cb_par, u16 attr_type)
{
    model_g_light_s_t *p_model = (model_g_light_s_t *)cb_par->model;
	
    return vd_msg_tx_attr_confirm(cb_par->model_idx, p_model->com.ele_adr, cb_par->adr_src, 0, 0, attr_type);
}

int cb_vd_msg_attr_indication(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
    int err = -1;
	vd_msg_attr_set_t *p_set = (vd_msg_attr_set_t *)par;
	if(!is_own_ele(cb_par->adr_src)){
  		err = vd_msg_attr_confirm_rsp(cb_par, p_set->attr_type);
	}
    return err;
}
#endif

//int cb_vd_key_report(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
//{
//    //model_g_light_s_t *p_model = (model_g_light_s_t *)cb_par->model;
//    vd_rc_key_report_t *p_key = (vd_rc_key_report_t *)par;
//    p_key = p_key;
//	LOG_MSG_INFO(TL_LOG_MESH,0,0,"vendor model key report: %d", p_key->code);
//
//#if (! (__PROJECT_MESH_SWITCH__))
//	static u16 key_report_1_ON,  key_report_2_ON,  key_report_3_ON,  key_report_4_ON;
//	static u16 key_report_1_OFF, key_report_2_OFF, key_report_3_OFF, key_report_4_OFF;
//	static u16 key_report_UP,	 key_report_DN,    key_report_L,	 key_report_R;
//
//	u8 lum_level = light_lum_get(0, 0);
//
//	switch(p_key->code){
//		case RC_KEY_1_ON:
//			key_report_1_ON++;
//			break;
//		case RC_KEY_1_OFF:
//			key_report_1_OFF++;
//			break;
//		case RC_KEY_2_ON:
//			key_report_2_ON++;
//			break;
//		case RC_KEY_2_OFF:
//			key_report_2_OFF++;
//			break;
//		case RC_KEY_3_ON:
//			key_report_3_ON++;
//			break;
//		case RC_KEY_3_OFF:
//			key_report_3_OFF++;
//			break;
//		case RC_KEY_4_ON:
//			key_report_4_ON++;
//			break;
//		case RC_KEY_4_OFF:
//			key_report_4_OFF++;
//			break;
//		case RC_KEY_UP:
//			key_report_UP++;
//			if(lum_level >= 100){
//				lum_level = 20;
//			}else{
//				lum_level += 20;
//			}
//
//			access_set_lum(ADR_ALL_NODES, 0, lum_level, 0);
//			break;
//		case RC_KEY_DN:
//			key_report_DN++;
//			if(lum_level <= 20){
//				lum_level = 100;
//			}else{
//				lum_level -= 20;
//			}
//
//			access_set_lum(ADR_ALL_NODES, 0, lum_level, 0);
//			break;
//		case RC_KEY_L:
//			key_report_L++;
//			break;
//		case RC_KEY_R:
//			key_report_R++;
//			break;
//		default:
//			break;
//	}
//#endif
//
//    return 0;
//}
// ------ end --------

#if (LPN_VENDOR_SENSOR_EN && MD_SERVER_EN)
typedef struct{
    u16 hunmidity;
    u16 temp; 
}hx300t_sensor_t;
#define HX300T_MSG_RSP_MODE 0
#define HX300T_MSG_PUB_MODE 1
typedef struct{
    u8 hx300t_tick ;
    u8 msg_mode;
    u16 hx300t_src ;
    u8 pub_idx;
}hx300t_mag_st_t;

hx300t_mag_st_t hx300t_mag;
hx300t_sensor_t hx300t_sensor;

#define HX300_SENSOR_ID     (0x44<<1)
#define HX300_INTER_TIME    (35*1000)

#if ((MCU_CORE_TYPE == MCU_CORE_8258) || (MCU_CORE_TYPE == MCU_CORE_8278))
#if(MCU_CORE_TYPE == MCU_CORE_8258)
#include "../../drivers/8258/i2c.h"
#elif(MCU_CORE_TYPE == MCU_CORE_8278)
#include "../../drivers/8278/i2c.h"
#endif
void i2c_io_init()
{
    i2c_master_init(HX300_SENSOR_ID,8);//set clk to 500k
    i2c_gpio_set(I2C_GPIO_GROUP_C0C1);
}
#elif (MCU_CORE_TYPE == MCU_CORE_8269)
#include "../../proj/drivers/i2c.h"
void i2c_io_init()
{
      // i2c init
    gpio_set_func(PIN_I2C_SDA,AS_GPIO);
    gpio_set_func(PIN_I2C_SCL,AS_GPIO);
    gpio_setup_up_down_resistor(PIN_I2C_SDA, 2);//GPIO_PULL_UP_10K
    gpio_setup_up_down_resistor(PIN_I2C_SCL, 2);//GPIO_PULL_UP_10K    
}

#else
#endif


void sensor_read_start()
{
    u8 r = irq_disable();
    #if (MCU_CORE_TYPE == MCU_CORE_8269)
    i2c_write_start(HX300_SENSOR_ID);
    #else
    i2c_write_byte(HX300_SENSOR_ID,1,0);
    #endif
    irq_restore(r);
}

void sensor_read_fun(u16 *p_humi,u16 *p_temp)
{
    u8 val_buffer[4];
    u8 r = irq_disable();
    #if (MCU_CORE_TYPE == MCU_CORE_8269)
    i2c_read_start_buf(HX300_SENSOR_ID,val_buffer,4);
    #else
    i2c_read_series(0,0,val_buffer,4);
    #endif
    irq_restore(r);
    u16 tempreture,humidity;
    humidity=((val_buffer[0]<<8) | val_buffer[1])&0x3fff;
    tempreture=((val_buffer[2]<<8) | val_buffer[3])>>2;
    humidity    = humidity * 100/ 0x3fff;
    tempreture = tempreture  * 165/ 0x3fff - 40;
    *p_humi = humidity;
    *p_temp = tempreture;
}

void sensor_proc_loop()
{
    if(hx300t_mag.hx300t_tick && clock_time_exceed(hx300t_mag.hx300t_tick,HX300_INTER_TIME)){
        hx300t_mag.hx300t_tick =0;
        sensor_read_fun(&(hx300t_sensor.hunmidity),&(hx300t_sensor.temp));
        if(hx300t_mag.msg_mode == HX300T_MSG_RSP_MODE){
            mesh_tx_cmd_rsp(VD_LPN_SENSOR_STATUS, (u8 *)&hx300t_sensor, 
                               sizeof(hx300t_sensor), ele_adr_primary, hx300t_mag.hx300t_src, 0, 0);
        }else if (hx300t_mag.msg_mode == HX300T_MSG_PUB_MODE){
            vd_lp_sensor_st_send(hx300t_mag.pub_idx);
        }
     }
}

int cb_vd_lpn_sensor_get(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
    // trigger start sensor data send 
    sensor_read_start();
    hx300t_mag.hx300t_tick = clock_time()|1;
    hx300t_mag.msg_mode = HX300T_MSG_RSP_MODE;
    hx300t_mag.hx300t_src = cb_par->adr_src;
    return 0;
}
int cb_vd_lpn_sensor_sts(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
    int err = 0;
    if(cb_par->model){  // model may be Null for status message
        //model_client_common_t *p_model = (model_client_common_t *)(cb_par->model);
    }
    return err;
}
// same as the vd_light_onoff_st_publish
int vd_lp_sensor_st_send(u8 idx)
{
    model_common_t *p_com_md = &model_vd_light.srv[idx].com;
	u16 ele_adr = p_com_md->ele_adr;
	u16 pub_adr = p_com_md->pub_adr;
	if(!pub_adr){
		return -1;
	}
	u8 *uuid = get_virtual_adr_uuid(pub_adr, p_com_md);
	return mesh_tx_cmd_rsp(VD_LPN_SENSOR_STATUS, (u8 *)&hx300t_sensor, sizeof(hx300t_sensor), ele_adr, pub_adr, uuid, p_com_md);
}
int cb_vd_lpn_sensor_st_publish(u8 idx)
{
    #if 0 // not stable 
    if(idx == 0){
        sensor_read_start();
        hx300t_mag.hx300t_tick = clock_time()|1;
        hx300t_mag.msg_mode = HX300T_MSG_PUB_MODE;
	    hx300t_mag.pub_idx = idx;
    }
	#else
	sensor_read_start();
	sleep_us(HX300_INTER_TIME);
	sensor_read_fun(&(hx300t_sensor.hunmidity),&(hx300t_sensor.temp));
	vd_lp_sensor_st_send(idx);
	#endif
	return 0;
}
void mesh_vd_lpn_pub_set()
{
    mesh_cfg_model_pub_set_t pub_set = {0};
    pub_set.ele_adr = ele_adr_primary;
    pub_set.pub_adr = 0xffff;  // publish all
    pub_set.pub_par.appkey_idx = 0;
	pub_set.pub_par.credential_flag = 0;
	pub_set.pub_par.rfu = 0;
	pub_set.pub_par.ttl = 0x3F;
	pub_set.pub_par.period.res= 1;
	pub_set.pub_par.period.steps = 0x1e;//30s interval
	pub_set.pub_par.transmit.count= 0;
	pub_set.pub_par.transmit.invl_steps = 0;
	pub_set.model_id = 0x00000211;    
	mesh_tx_cmd2normal_primary(CFG_MODEL_PUB_SET, (u8 *)&pub_set, sizeof(mesh_cfg_model_pub_set_t), ele_adr_primary, 0);
}

#else

#define cb_vd_lpn_sensor_get (0)
#define cb_vd_lpn_sensor_sts (0)
#endif

/**********************************/
/**
 * @func   cb_vd_dev_opt_set
 * @brief  None
 * @param
 * @retval
 */
int cb_vd_dev_opt_set(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par){

	int err = 0;
	if(cb_par->model){  // model may be Null for status message
	     //model_client_common_t *p_model = (model_client_common_t *)(cb_par->model);
		DBG_VENDOR_MODEL_SEND_STR("\n cb_vd_dev_opt_set");
	}
	return err;
}
/**
 * @func   cb_vd_dev_opt_get
 * @brief  None
 * @param
 * @retval
 */
typedef struct{
	u8 device_uuid[16];
	u8 mac[6];
	u16 product_id;
}light_opt_t;

int cb_vd_dev_opt_get(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par){

	int err = 0;
	 // model may be Null for status message
	if(cb_par->model){
		DBG_VENDOR_MODEL_SEND_STR("\n cb_vd_dev_opt_get");
		light_opt_t rsp;
		memcpy(&rsp.device_uuid, &prov_para.device_uuid, 16);
		memcpy(&rsp.mac, &tbl_mac, 6);
		rsp.product_id = MESH_PID_SEL;
	    err = mesh_tx_cmd_rsp(
	    			VD_LIGHT_OPT_STATUS, (u8 *)&rsp, sizeof(rsp), ele_adr_primary, cb_par->adr_src, 0, 0
	    		);
	}
	return err;
}
#define cb_vd_dev_opt_status   (0)

/**
 * @func   vd_light_state_st_rsp
 * @brief  None
 * @param
 * @retval
 */

typedef struct{
	u16 pid;
	u8  f_ctt;
	u8  state;
	u16 lightness;
	u16 temp;
	u16 hue;
	u16 sat;
	u8  action;
}light_state_st_t;

int vd_light_state_st_rsp(mesh_cb_fun_par_t *cb_par)
{
	DBG_VENDOR_MODEL_SEND_STR("\n vd_light_state_st_rsp");

	if(cb_par->model_idx < LIGHT_CNT){

		model_client_common_t *p_model = (model_client_common_t *)(cb_par->model);

		if(cb_par->adr_dst < ADR_GROUP_START_POINT){

			light_state_st_t rsp;
			rsp.pid = MESH_PID_SEL;
			mesh_cmd_g_level_st_t level_st;
			light_g_level_get((u8 *)&level_st, cb_par->model_idx, ST_TRANS_LIGHTNESS);
			rsp.f_ctt = 0;
			rsp.state = get_onoff_from_level(level_st.present_level);
			rsp.lightness = s16_to_u16(light_res_sw[cb_par->model_idx].trans[ST_TRANS_LIGHTNESS].present);
			rsp.temp = 0;
			rsp.hue = 0;
			rsp.sat = 0;
			rsp.action = 0;
			return mesh_tx_cmd_rsp(
						VD_LIGHT_STATE_STATUS, (u8 *)&rsp, sizeof(rsp), p_model->com.ele_adr, cb_par->adr_src, 0, 0
					);
		}else{
			// Response Random 5 - 35s
			light_publish_status_delay(
						cb_par->model_idx, 5000 + rand() % 30000
					);
		}
	}
	return -1;
}
/**
 * @func   cb_vd_light_state_get
 * @brief  None
 * @param
 * @retval
 */
int cb_vd_light_state_get(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par){
	int err = 0;
	DBG_VENDOR_MODEL_SEND_STR("\n cb_vd_light_state_get");

	if(cb_par->model_idx < LIGHT_CNT){
		if(cb_par->model){
			err = vd_light_state_st_rsp(cb_par);
		}
	}
	return err;
}

/**
 * @func   cb_vd_light_state_status
 * @brief  None
 * @param
 * @retval
 */
#define cb_vd_light_state_status     (0)

// TEST LED ASSEMBLY

/**
 * @func   cb_vd_test_led_set
 * @brief  None
 * @param
 * @retval
 */
int cb_vd_test_led_get(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par){
	int err = 0;
	return err;
}

/**
 * @func   cb_vd_test_led_status
 * @brief  None
 * @param
 * @retval
 */
#define cb_vd_test_led_status        (0)

/**
 * @func   cb_vd_config_node_set
 * @brief  None
 * @param
 * @retval Status code
 */

typedef struct{
	u8 code;
	u8 data;
}vd_config_node_t;

typedef struct{
	u8 ttl;
}vendor_config_ttl_set_t;


#define VD_CONFIG_NODE_RST   0x00
#define VD_CONFIG_SET_TTL    0x01
#define VD_CONFIG_GET_TTL    0x02

int cb_vd_config_node_set(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par){
	int err = 0;
	if(cb_par->adr_src == GATEWAY_UNICAST_ADDR){
		if(cb_par->model_idx == 0){
			if(cb_par->model){
				vd_config_node_t* vd_config_node = (vd_config_node_t*)par;
				if(vd_config_node->code == VD_CONFIG_NODE_RST){
					DBG_VENDOR_MODEL_SEND_STR("\n cb_vd_factory_reset_set ");
					setup_factory_reset_with_delay(TRUE, TIMER_1S);
				}else if(vd_config_node->code == VD_CONFIG_SET_TTL){
					DBG_VENDOR_MODEL_SEND_STR("\n mesh_cmd_sig_cfg_def_ttl_set ");
					mesh_cb_fun_par_t* vd_cb_par = cb_par;
					vd_cb_par->op = CFG_DEFAULT_TTL_SET;
					vendor_config_ttl_set_t *ttl_set = (vendor_config_ttl_set_t*)(par+1);
					mesh_cmd_sig_cfg_def_ttl_set((u8*)ttl_set, sizeof(vendor_config_ttl_set_t), vd_cb_par);
				}else if(vd_config_node->code == VD_CONFIG_GET_TTL){
					mesh_cb_fun_par_t* vd_cb_par = cb_par;
					vd_cb_par->op = CFG_DEFAULT_TTL_GET;
					mesh_cmd_sig_cfg_def_ttl_get((void*)0, 0, vd_cb_par);
				}else{
					// Reserved for the feature use
					DBG_VENDOR_MODEL_SEND_STR("\n Config Node Reserved");
				}
			}
		}else{
			err = -1;
		}
	}
	return err;
}

/**
 * @func   cb_vd_config_model_sub_set
 * @brief  None
 * @param
 * @retval Status code
 */

BOOL vd_config_model_sub_set_flag = 0;

typedef struct{
	uint8_t  sub_type;
	uint16_t ele_adr;
	uint16_t sub_adr;
	union{
		u8 sig_model[2];
		u8 vd_model[4];
		u8 *model;
	};
}vd_config_model_sub_set_t;

typedef struct{
	u16 ele_adr;
	u16 sub_adr;
	union{
		u8 vd_model[4];
		u8 sig_model[2];
		u8 *model;
	};
}vd_config_model_sub_add_del_and_over_write_t;

typedef struct{
	u16 ele_adr;
	union{
		u8 sig_model[2];
		u8 vd_model[4];
		u8 *model;
	};
}vd_config_model_sub_del_all_t;


typedef struct{
	u16 ele_adr;
	u8 sig_model[2];
}vd_config_model_sub_get_t;

typedef struct{
	u8 status;
	u16 sub_adr;
}vd_config_model_sub_status;

enum vd_config_model_sub_type{
	VD_CFG_MODEL_SUB_ADD        = 0,
	VD_CFG_MODEL_SUB_DEL        = 1,
	VD_CFG_MODEL_SUB_OVERWRITE = 2,
	VD_CFG_MODEL_SUB_DEL_ALL    = 3,
	VD_CFG_MODEL_SUB_GET        = 4,
	VD_CFG_MODEL_SUB_STATUS     = 5,
	VD_CFG_MODEL_SUB_UNKNOWN
};
typedef u8 vd_config_model_sub_type;

const u16 sig_model_share_sub_adr[] = {
			SIG_MD_G_ONOFF_S,
	#if LIGHT_TYPE_HSL_EN
			SIG_MD_LIGHT_HSL_S,
	#endif
	#if LIGHT_TYPE_CT_EN
			SIG_MD_LIGHT_CTL_S,
			SIG_MD_LIGHT_CTL_TEMP_S,
	#endif
    #if MD_LIGHTNESS_EN
			SIG_MD_LIGHTNESS_S
    #endif
};


int cb_vd_config_model_sub_set(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	int err = 0;
	DBG_VENDOR_MODEL_SEND_STR("\n cb_vd_config_model_sub_set");

	if(cb_par->adr_src == GATEWAY_UNICAST_ADDR){
		if(cb_par->model_idx < LIGHT_CNT){
			if(cb_par->model){
				model_client_common_t *p_model = (model_client_common_t *)(cb_par->model);
				vd_config_model_sub_set_t* vd_config_model_sub_set = (vd_config_model_sub_set_t *)par;
				vd_config_model_sub_status rsp;
				rsp.sub_adr = vd_config_model_sub_set->sub_adr;
				// Check Invalid opcode
				rsp.status = (vd_config_model_sub_set->sub_type  \
						< VD_CFG_MODEL_SUB_STATUS)?ST_SUCCESS:ST_UNSPEC_ERR;
				if(rsp.status == ST_UNSPEC_ERR) {
					return -1;
				}
				// Check Invalid sub address
				if(rsp.status == ST_SUCCESS){
					if(vd_config_model_sub_set->sub_type <= VD_CFG_MODEL_SUB_OVERWRITE){
						if((vd_config_model_sub_set->sub_adr < ADR_GROUP_START_POINT)
								|| (vd_config_model_sub_set->sub_adr >= ADR_FIXED_GROUP_START)){
							rsp.status = ST_INVALID_ADR;
						}
					}
				}
				// Check Valid sub address before or not
				if(rsp.status == ST_SUCCESS){
					if(vd_config_model_sub_set->sub_type == VD_CFG_MODEL_SUB_ADD)
					{
						for(u8 i = 0; i < SUB_LIST_MAX; i++) {
							u16 temp = model_sig_g_onoff_level.onoff_srv[cb_par->model_idx ].com.sub_list[i];
							if((temp == vd_config_model_sub_set->sub_adr)||(temp == 0xFFFF)||(temp == 0x0000))
							{
								rsp.status = ST_SUCCESS;
								break;
							}
							rsp.status = ST_CAN_NOT_SET;
						}
					}
				}
				u8 len_rsp = (vd_config_model_sub_set->sub_type <= VD_CFG_MODEL_SUB_OVERWRITE) ? 3 : 1;
				mesh_tx_cmd_rsp(
						VD_CONFIG_MODEL_SUB_STATUS, (u8 *)&rsp, len_rsp, p_model->com.ele_adr, cb_par->adr_src, 0, 0
						);
				if(rsp.status != ST_SUCCESS) return -1;

				if((vd_config_model_sub_set->ele_adr < (ele_adr_primary + LIGHT_CNT))  \
							&& (cb_par->adr_dst < (ele_adr_primary + LIGHT_CNT))
										&& (vd_config_model_sub_set->ele_adr >= ele_adr_primary)
													&& (cb_par->adr_dst == vd_config_model_sub_set->ele_adr)){
					mesh_cb_fun_par_t* vd_cb_par = cb_par;
					vd_cb_par->op_rsp = STATUS_NONE;
					int status = 0;
					if(vd_config_model_sub_set->sub_type == VD_CFG_MODEL_SUB_GET){
						vd_config_model_sub_get_t* vd_config_model_sub_get = (vd_config_model_sub_get_t*)(par+1);
						vd_config_model_sub_get->sig_model[0] = (u8)SIG_MD_G_ONOFF_S;
						vd_config_model_sub_get->sig_model[1] = (u8)(SIG_MD_G_ONOFF_S >> 8);
						vd_cb_par->op = CFG_SIG_MODEL_SUB_GET;
						status = mesh_cmd_sig_sig_model_sub_get(
								(u8*)&vd_config_model_sub_get->ele_adr, sizeof(vd_config_model_sub_get_t), vd_cb_par);
					}else{
						vd_config_model_sub_set_flag = 1;
						switch(vd_config_model_sub_set->sub_type){
							case VD_CFG_MODEL_SUB_ADD:
								vd_cb_par->op = CFG_MODEL_SUB_ADD;
								break;
							case VD_CFG_MODEL_SUB_DEL:
								vd_cb_par->op = CFG_MODEL_SUB_DEL;
								break;
							case VD_CFG_MODEL_SUB_OVERWRITE:
								vd_cb_par->op = CFG_MODEL_SUB_OVER_WRITE;
								break;
							case VD_CFG_MODEL_SUB_DEL_ALL:
								vd_cb_par->op = CFG_MODEL_SUB_DEL_ALL;
								break;
						}
						vd_config_model_sub_del_all_t* vd_config_model_sub_del_all = (vd_config_model_sub_del_all_t *)(par+1);
						// Sig Model Share Subcription
						foreach_arr(i, sig_model_share_sub_adr){
							vd_config_model_sub_set->sig_model[0] = (u8)sig_model_share_sub_adr[i];
							vd_config_model_sub_set->sig_model[1] = (u8)(sig_model_share_sub_adr[i] >> 8);
							if(vd_config_model_sub_set->sub_type != VD_CFG_MODEL_SUB_DEL_ALL){
								status = mesh_cmd_sig_cfg_model_sub_set(
										(u8*)&vd_config_model_sub_set->ele_adr, par_len + 1, vd_cb_par);
							}else{
								memcpy(&vd_config_model_sub_del_all->sig_model, &vd_config_model_sub_set->sig_model, sizeof(vd_config_model_sub_del_all->sig_model));
								status = mesh_cmd_sig_cfg_model_sub_set(
										(u8*)&vd_config_model_sub_del_all->ele_adr, 4, vd_cb_par);
							}
						}
						// Vendor Model Share Subcription
						vd_config_model_sub_set->vd_model[0] = (u8) VENDOR_MD_LIGHT_S;
						vd_config_model_sub_set->vd_model[1] = (u8)(VENDOR_MD_LIGHT_S >> 8);
						vd_config_model_sub_set->vd_model[2] = (u8)(VENDOR_MD_LIGHT_S >> 16);
						vd_config_model_sub_set->vd_model[3] = (u8)(VENDOR_MD_LIGHT_S >> 24);
						if(vd_config_model_sub_set->sub_type != VD_CFG_MODEL_SUB_DEL_ALL){
							status = mesh_cmd_sig_cfg_model_sub_set(
									(u8*)&vd_config_model_sub_set->ele_adr,par_len + 3, vd_cb_par);
						}else{
							memcpy(&vd_config_model_sub_del_all->vd_model, &vd_config_model_sub_set->vd_model, sizeof(vd_config_model_sub_del_all->vd_model));
							status = mesh_cmd_sig_cfg_model_sub_set(
									(u8*)&vd_config_model_sub_del_all->ele_adr, 6, vd_cb_par);
						}
						vd_config_model_sub_set_flag = 0;
						rf_link_light_event_callback(LGT_CMD_SET_SUBSCRIPTION);

						#if ENABLE_BLINK_SINGLE_LED
						update_blink_mask(1 << cb_par->model_idx);
						#endif

					}
				}
			}
		}
	}
	return err;
}

typedef struct {
	u32  start_time;
	u16  time_len;
	BOOL active;
	u8   st;
	u16  dst_adr;
}scene_reg_rsp_delay_t;

static scene_reg_rsp_delay_t scene_reg_rsp_delay[LIGHT_CNT];

/**
 * @func   scene_reg_response_delay_init
 * @brief  None
 * @param
 * @retval Status code
 */
static void scene_reg_response_delay_init(u8 idx, u16 dst_adr, u8 st)
{
	if(idx < LIGHT_CNT) {
		scene_reg_rsp_delay[idx].active = TRUE;
		scene_reg_rsp_delay[idx].st = st;
		scene_reg_rsp_delay[idx].dst_adr = dst_adr;
		scene_reg_rsp_delay[idx].start_time = clock_time_ms();
		scene_reg_rsp_delay[idx].time_len = TIMER_1S + rand()%TIMER_9S;
	}
}
/**
 * @func   vendor_model_init
 * @brief  None
 * @param
 * @retval Status code
 */
void vendor_model_init(void)
{
	foreach(i, LIGHT_CNT) {
		scene_reg_rsp_delay[i].active = FALSE;
	}
	scene_callback_init(scene_reg_response_delay_init);
}

/**
 * @func   scene_reg_response_delay_init
 * @brief  None
 * @param
 * @retval Status code
 */
void scene_reg_response_loop(void)
{
	foreach(i, LIGHT_CNT) {
		if(scene_reg_rsp_delay[i].active == TRUE) {
			if(clock_time_exceed_ms(  \
					scene_reg_rsp_delay[i].start_time, scene_reg_rsp_delay[i].time_len)) {
				vd_scene_request_flag = 1;
				mesh_tx_cmd_scene_reg_st(
							i,
							ele_adr_primary + i,
							scene_reg_rsp_delay[i].dst_adr,
							scene_reg_rsp_delay[i].st
						);
				vd_scene_request_flag = 0;
				// Inactive
				scene_reg_rsp_delay[i].active = FALSE;

			}
		}
	}
}

/**
 * @func   cb_vd_config_scene
 * @brief  None
 * @param
 * @retval Status code
 */

BOOL vd_scene_request_flag = 0;

int cb_vd_scene_request(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	DBG_VENDOR_MODEL_SEND_STR("\n cb_vd_scene_request");
	int err = 0;

	if(cb_par->model_idx < LIGHT_CNT){
		if(cb_par->model){

	    	vd_scene_request_flag = 1;

	    	switch(par[0]){
				case VD_SCENE_STORE:
					if(cb_par->adr_src == GATEWAY_UNICAST_ADDR){
						cb_par->op_rsp = SCENE_REG_STATUS;
						cb_par->op = SCENE_STORE;
						mesh_cmd_sig_scene_set(par+1, par_len - 1, cb_par);
						rf_link_light_event_callback(LGT_CMD_CONFIG_SCENE);

						#if ENABLE_BLINK_SINGLE_LED
						update_blink_mask(1 << cb_par->model_idx);
						#endif
					}
					break;

				case VD_SCENE_RECALL:
					cb_par->op_rsp = SCENE_STATUS;
					cb_par->op = SCENE_RECALL;
					mesh_cmd_sig_scene_recall(par+1, par_len - 1, cb_par);
					break;

				case VD_SCENE_DEL:
					if(cb_par->adr_src == GATEWAY_UNICAST_ADDR){
						cb_par->op_rsp = SCENE_REG_STATUS;
						cb_par->op = SCENE_DEL;
						mesh_cmd_sig_scene_set(par+1, par_len - 1, cb_par);
						rf_link_light_event_callback(LGT_CMD_CONFIG_SCENE);

						#if ENABLE_BLINK_SINGLE_LED
						update_blink_mask(1 << cb_par->model_idx);
						#endif
					}
					break;

				case VD_SCENE_GET:
					cb_par->op_rsp = SCENE_STATUS;
					cb_par->op = SCENE_GET;
					mesh_cmd_sig_scene_get(par+1, par_len - 1, cb_par);
					break;

				case VD_SCENE_REG_GET:
					if(cb_par->adr_src == GATEWAY_UNICAST_ADDR){
						cb_par->op_rsp = SCENE_REG_STATUS;
						cb_par->op = SCENE_GET;
						mesh_cmd_sig_scene_reg_get(par+1, par_len - 1, cb_par);
					}
					break;

				default:
					break;
	    	}
	    	vd_scene_request_flag = 0;
		}
	}
	return err;
}

/**
 * @func   cb_vd_scene_request_noack
 * @brief  None
 * @param
 * @retval Status
 */
int cb_vd_scene_request_noack(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	int err = 0;

	if(cb_par->model_idx < LIGHT_CNT){
		if(cb_par->model){
			DBG_VENDOR_MODEL_SEND_STR("\n VD_CONFIG_SCENE_NOACK: ");
			DBG_VENDOR_MODEL_SEND_HEX_ONE(par[0]);
			cb_par->op_rsp = STATUS_NONE;

			switch(par[0]){
				case VD_SCENE_STORE_NOACK:
					if(cb_par->adr_src == GATEWAY_UNICAST_ADDR){
						cb_par->op = SCENE_STORE_NOACK;
						mesh_cmd_sig_scene_set(par+1, par_len - 1, cb_par);
						rf_link_light_event_callback(LGT_CMD_CONFIG_SCENE);

						#if ENABLE_BLINK_SINGLE_LED
						update_blink_mask(1 << cb_par->model_idx);
						#endif
					}
					break;
				case VD_SCENE_RECALL_NOACK:
					cb_par->op = SCENE_RECALL_NOACK;
					mesh_cmd_sig_scene_recall(par+1, par_len - 1, cb_par);
					break;

				case VD_SCENE_DEL_NOACK:
					if(cb_par->adr_src == GATEWAY_UNICAST_ADDR){
						cb_par->op = SCENE_DEL_NOACK;
						mesh_cmd_sig_scene_set(par+1, par_len - 1, cb_par);
						rf_link_light_event_callback(LGT_CMD_CONFIG_SCENE);

						#if ENABLE_BLINK_SINGLE_LED
						update_blink_mask(1 << cb_par->model_idx);
						#endif
					}
					break;

				default:
					break;
			}
		}
	}
	return err;
}

/**
 * @func   cb_vd_scene_response
 * @brief  None
 * @param
 * @retval Status code
 */
#define cb_vd_scene_response              (0)

/**
 * @func   cb_vd_config_model_sub_status
 * @brief  None
 * @param
 * @retval Status code
 */
#define cb_vd_config_model_sub_status     (0)
#define cb_vd_test_production_status      (0)

/**
 * @func   cb_vd_test_production_set
 * @brief  None
 * @param
 * @retval Status code
 */
int cb_vd_test_production_set(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	/*
	if(cb_par->model_idx == 0)
	{
		if(cb_par->model)
		{
			test_production_set_t* test_production_set_st = (test_production_set_t*)par;
			u8 cf_success = TRUE;
			foreach_arr(i, const_confirm_code_arr){
				if(const_confirm_code_arr[i] != test_production_set_st->confirm[i]){
					cf_success = FALSE;
				}
			}
			if(cf_success != FALSE){
				u8 my_mac[4];
				memcpy(&my_mac, &tbl_mac, 4);
				foreach_arr(i, my_mac){
					if(my_mac[i] != test_production_set_st->mac[i]){
						return -1;
					}
				}
				if(test_production_set_st->pro_code.dev_type != DEVICE_SCENES_AC){
					return -1;
				}
				switch(test_production_set_st->pro_code.key){
				    case KEY_PRODUCTION_RESET_TEST:{
						if(pv_handle_reset_test_production != NULL){
							pv_handle_reset_test_production();
						}
				    	break;
				    }
				    default: break;
				}
			}
		}
	}
	*/
	return 0;
}

/**
 * @func   vendor_handle_func_callback_init
 * @brief  None
 * @param
 * @retval Status code
 */
void vendor_handle_func_callback_init(
		typeVendor_HandleSetupExecutionSceneCallbackFunc callbackFunc){

	if(callbackFunc != NULL){
		pvVendor_HandleSetupExecutionScene = callbackFunc;
	}
}

/**
 * @func   cb_vd_setup_execution_scene_set
 * @brief  None
 * @param
 * @retval Status code
 */
int cb_vd_setup_execution_scene_set(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par){

	int err = 0;

	if(cb_par->model_idx < NUMBER_INPUT){
		if(cb_par->model){
			if(cb_par->adr_src == GATEWAY_UNICAST_ADDR){
				if(pvVendor_HandleSetupExecutionScene != NULL){
					pvVendor_HandleSetupExecutionScene(par, par_len, cb_par);
				}
			}else{
				err = -1;
			}
		}
	}
	return err;
}

#define cb_vd_setup_execution_scene_status   (0)

#define cb_vd_test_production_status         (0)



// CONFIGURATION NODE SET

typedef struct{
	u8 msg_type;
	u8 code;
	u8 data[10];
}vd_config_node_ack_t;

typedef struct {
	BOOL flag;
	u32  delay_start_t_ms;
	u32  delay_t_ms;
	u16  adr_dst;
}vd_send_tx_msg_delay_t;

vd_send_tx_msg_delay_t vd_send_tx_msg_delay_st[NUMBER_INPUT][VD_CONFIG_NODE_CNT];

typedef struct {
	BOOL flag;
	u32 delay_start_t_ms;
	u32 delay_t_ms;
}vd_send_on_power_up_delay_t;

vd_send_on_power_up_delay_t  vd_send_on_power_up_delay_st[LIGHT_CNT];


/**
 * @func   vd_config_node_delay_init
 * @brief  None
 * @param  None
 * @retval None
 */
void vd_config_node_rsp_delay_init(void)
{
    foreach(i, NUMBER_INPUT){
    	foreach(j, VD_CONFIG_NODE_CNT){
    		vd_send_tx_msg_delay_st[i][j].flag = FALSE;
    	}
    }
    foreach(i, LIGHT_CNT){
    	vd_send_on_power_up_delay_st[i].flag = FALSE;
    }
}


/**
 * @func   vd_send_config_node_response
 * @brief  None
 * @param  ...
 * @retval Error code
 */
static int vd_send_config_node_response(u8 input_chanel, u8 code, u16 adr_dst)
{
	int err = 0;

	if(input_chanel < NUMBER_INPUT){
		switch(code)
		{
			case VD_CONFIG_SW_SET_MAIN_PARAMS:{

				u8 rsp[sizeof(vd_sw_config_t) + 1];
				rsp[0] = VD_CONFIG_SW_SET_MAIN_PARAMS;
				vd_sw_config_t* vd_sw_config_st = (vd_sw_config_t*)io_get_main_params(input_chanel);
				memcpy(&rsp[1], vd_sw_config_st, sizeof(vd_sw_config_t));

				if(rsp != NULL){
					err = mesh_tx_cmd_rsp(
								VD_CONFIG_NODE_STATUS,
								rsp,
								sizeof(vd_sw_config_t) + 1,
								ele_adr_primary + input_chanel,
								adr_dst,
								0,
								0
						     );
				}
				break;
			}
			case VD_CONFIG_SW_SET_MAP_UNMAP_INPUT:{

				io_params_t *io_params_st = io_get_main_params(input_chanel);
				u8 rsp[2];
				rsp[0] = VD_CONFIG_SW_SET_MAP_UNMAP_INPUT;
				rsp[1] = io_params_st->out_map_input;

				if(io_params_st != NULL){
					err = mesh_tx_cmd_rsp(
								VD_CONFIG_NODE_STATUS,
								rsp,
								sizeof(rsp),
								ele_adr_primary + input_chanel,
								adr_dst,
								0,
								0
						     );
				}
				break;
			}

			case VD_EN_ON_OFF_GROUP_DEFAULT: {
				sw_response_enable_disable_group_default(input_chanel);
				break;
			}

			default:{
				err = -1;
			}
		}
	}
	else{
		err = -1;
	}
	return err;
}

/**
 * @func   vd_handle_send_vd_config_node_with_delay
 * @brief  None
 * @param
 * @retval
 */
void handle_send_vd_config_node_with_delay(void)
{
	foreach(i, NUMBER_INPUT){
		foreach(j, VD_CONFIG_NODE_CNT){
			if(vd_send_tx_msg_delay_st[i][j].flag == TRUE){
				if(clock_time_exceed_ms   \
						(vd_send_tx_msg_delay_st[i][j].delay_start_t_ms, vd_send_tx_msg_delay_st[i][j].delay_t_ms)){
					// Send Configuration parameter
					if(j < (VD_CONFIG_NODE_CNT - 1)) {
						vd_send_config_node_response(i, j|0x10, vd_send_tx_msg_delay_st[i][j].adr_dst); // 0x10, 0x11
					}
					else {
						vd_send_config_node_response(i, VD_EN_ON_OFF_GROUP_DEFAULT, vd_send_tx_msg_delay_st[i][j].adr_dst);  // enable/disable group default
					}
					vd_send_tx_msg_delay_st[i][j].flag = FALSE;
					return;
				}
			}
		}
	}
	foreach(i, LIGHT_CNT){
        if(vd_send_on_power_up_delay_st[i].flag == TRUE){
        	if(clock_time_exceed_ms   \
        			(vd_send_on_power_up_delay_st[i].delay_start_t_ms, vd_send_on_power_up_delay_st[i].delay_t_ms)){
				u8 power_onoff = model_sig_g_power_onoff.on_powerup[i];
				mesh_tx_cmd_rsp(
						G_ON_POWER_UP_STATUS,
						(u8 *)&power_onoff,
						1,
						ele_adr_primary + i,
						GATEWAY_UNICAST_ADDR,
						0,
						0
					);
        		vd_send_on_power_up_delay_st[i].flag = FALSE;
        	}
        }
	}
}

#define GET_VD_CONFIG_NODE_INDEX(x)    (x - 0x10)

/**
 * @func   vd_setup_send_config_node_params_delay
 * @brief  None
 * @param  ...
 * @retval Error code
 */
int vd_setup_send_config_node_params_delay(u8 channel, u8 code, u16 adr_dst, u8 msg_type, u8 fast_report)
{
	DBG_VENDOR_MODEL_SEND_STR("\nvd_setup_send_config_node_params_delay");
	u8 index = GET_VD_CONFIG_NODE_INDEX(code);
	if(channel < NUMBER_INPUT)
	{
		if(index < VD_CONFIG_NODE_CNT)
		{
			vd_send_tx_msg_delay_st[channel][index].flag = TRUE;
			vd_send_tx_msg_delay_st[channel][index].delay_start_t_ms = clock_time_ms();
			if(fast_report == VERY_FAST_REPORT){
				vd_send_tx_msg_delay_st[channel][index].delay_t_ms = (rand() % 5000);
			}
			else if(fast_report == FAST_REPORT){
				vd_send_tx_msg_delay_st[channel][index].delay_t_ms = 500 + (rand() % 10000);    // 0.5s - 10.5s
			}
			else{
			    vd_send_tx_msg_delay_st[channel][index].delay_t_ms = 5000 + (rand() % 10000);  // 5s - 15s
			}
			vd_send_tx_msg_delay_st[channel][index].adr_dst = adr_dst;
			return 0;
		}
	}
	return -1;
}

/**
 * @func   cb_vd_config_node_get
 * @brief  None
 * @param
 * @retval
 */
int cb_vd_config_node_get(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	int err = 0;
	if(cb_par->model_idx < NUMBER_INPUT){
		if(cb_par->model){
			vd_config_node_ack_t *config_node = (vd_config_node_ack_t*)par;
			if(config_node->code == VD_CONFIG_SW_SET_MAIN_PARAMS){
				vd_send_config_node_response(
						cb_par->model_idx, VD_CONFIG_SW_SET_MAIN_PARAMS, cb_par->adr_src
					);
			}
			else if(config_node->code == VD_CONFIG_SW_SET_MAP_UNMAP_INPUT){
				vd_send_config_node_response(
						cb_par->model_idx, VD_CONFIG_SW_SET_MAP_UNMAP_INPUT, cb_par->adr_src
					);
			}
			else if(config_node->code == VD_CONFIG_GROUP_ASSOCIATION) {
				binding_handle_nw_message(
							CONFIG_NODE_GET, config_node->data
						);
			}
			else if(config_node->code == VD_EN_ON_OFF_GROUP_DEFAULT) {
				sw_response_enable_disable_group_default(cb_par->model_idx);
			}
			else {
				err = -1;
			}
		}
	}
	return err;
}

/**
 * @func   cb_vd_config_node_set_ack
 * @brief  None
 * @param
 * @retval ST
 */
int cb_vd_config_node_set_ack(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	int err = 0;

	DBG_VENDOR_MODEL_SEND_STR("\nb_vd_config_node_set_ack");

	if(cb_par->model_idx < NUMBER_INPUT){
		if(cb_par->model){
			vd_config_node_ack_t *config_node = (vd_config_node_ack_t*)par;
			if(config_node->msg_type == CONFIG_NODE_SET){
				DBG_VENDOR_MODEL_SEND_STR("\nmsg SET");
				if(config_node->code == VD_CONFIG_SW_SET_MAIN_PARAMS)
				{
					err = io_config_sw_main_params(
										cb_par->model_idx, config_node->data, par_len - 2
									);
					DBG_VENDOR_MODEL_SEND_STR("\nio_config_sw_main_params");
				}
				else if(config_node->code == VD_CONFIG_SW_SET_MAP_UNMAP_INPUT)
				{
					err = io_config_sw_set_map_unmap_input(
										cb_par->model_idx, config_node->data
									);
				}
				else if(config_node->code == VD_CONFIG_GROUP_ASSOCIATION) {
					binding_handle_nw_message(
								CONFIG_NODE_SET, config_node->data
							);
					return err;
				}
				else if(config_node->code == VD_EN_ON_OFF_GROUP_DEFAULT) {
					sw_set_enable_disable_group_default(
								cb_par->model_idx, config_node->data[0]
							);
				}
				else {
					err = -1;
					return err;
				}
				// Send Response
				if(cb_par->adr_dst < ADR_GROUP_START_POINT){
					cb_vd_config_node_get(par, par_len, cb_par);
				}
				else{
					vd_setup_send_config_node_params_delay(
									cb_par->model_idx, config_node->code,
									cb_par->adr_src, CONFIG_NODE_SET, FAST_REPORT
								);
				}
			}
			else if(config_node->msg_type == CONFIG_NODE_GET){
				// Send Response
				if(cb_par->adr_dst < ADR_GROUP_START_POINT){
					cb_vd_config_node_get(
								par, par_len, cb_par
							);
				}
				else{
					vd_setup_send_config_node_params_delay(
								cb_par->model_idx, config_node->code,
								cb_par->adr_src, CONFIG_NODE_GET, FAST_REPORT
							);
				}
			}
		}
	}else{
		err = -1;
	}
	return err;
}

/**
 * @func   vd_setup_send_all_config_node_params_delay
 * @brief
 * @param  None
 * @retval None
 */
void vd_setup_send_all_config_node_params_delay(u8 fast_report)
{
	foreach(i, NUMBER_INPUT){
		foreach(j, VD_CONFIG_NODE_CNT){
			vd_setup_send_config_node_params_delay(     \
							i,
							VD_CONFIG_NODE_SET_START_CMD + j,
							GATEWAY_UNICAST_ADDR,
							CONFIG_NODE_GET,
							fast_report
						);
		}
	}
	foreach(i, LIGHT_CNT){
		vd_send_on_power_up_delay_st[i].flag = TRUE;
		if(fast_report == VERY_FAST_REPORT){
			vd_send_on_power_up_delay_st[i].delay_t_ms = rand() % 5000;
		}
		else{
		    vd_send_on_power_up_delay_st[i].delay_t_ms = 500 + (rand() % 10000);
		}
		vd_send_on_power_up_delay_st[i].delay_start_t_ms = clock_time_ms();
	}
}

/**
 * @func   cb_vd_control_locally
 * @brief  None
 * @param
 * @retval Status code
 */
int cb_vd_control_locally(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	int err = 0;
	if(par[0] == TYPE_CTL_ON_OFF) {
		mesh_cmd_sig_g_onoff_set(
				(u8*)&par[1], (par_len - 1), cb_par
			);
	}
	else if(par[0] == TYPE_CTL_LIGHTNESS) {
		func_handle_control_message(
					(u8*)&par[1], (par_len - 1), cb_par, LIGHTNESS_SET
				);
	}
	else {
		err = -1;
	}
	return err;
}

/**
 * @func   cb_vd_config_node_status
 * @brief
 * @param  None
 * @retval None
 */
#define cb_vd_config_node_status    (0)

/***********************************/

#if VC_SUPPORT_ANY_VENDOR_CMD_EN
int cb_app_vendor_all_cmd(mesh_cmd_ac_vd_t *ac, int ac_len, mesh_cb_fun_par_vendor_t *cb_par)
{
    return 0;
}
#endif

// use unsegment pkt for vendor command should be better.
// = sizeof(mesh_cmd_lt_unseg_t.data) - sizeof(MIC) - sizeof(op) = 15-4-3 = 8
STATIC_ASSERT(sizeof(vd_rc_key_report_t) <= 8);

#if !WIN32
const 
#endif
mesh_cmd_sig_func_t mesh_cmd_vd_func[] = {
#if (VENDOR_OP_MODE_SEL == VENDOR_OP_MODE_SPIRIT)
    #if 0 // DEBUG_VENDOR_CMD_EN // just for sample, default disable, 
    {VD_LIGHT_ONOFF_SET, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_light_onoff_set, VD_LIGHT_ONOFF_STATUS},
	{VD_LIGHT_ONOFF_GET, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_light_onoff_get, VD_LIGHT_ONOFF_STATUS},
	{VD_LIGHT_ONOFF_SET_NOACK, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_light_onoff_set, STATUS_NONE},
    {VD_LIGHT_ONOFF_STATUS, 1, VENDOR_MD_LIGHT_S, VENDOR_MD_LIGHT_C, cb_vd_light_onoff_status, STATUS_NONE},
    #endif
    
    #if SPIRIT_VENDOR_EN
	{VD_MSG_ATTR_GET, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_msg_attr_get, VD_MSG_ATTR_STS},
	{VD_MSG_ATTR_SET, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_msg_attr_set, VD_MSG_ATTR_STS},
	{VD_MSG_ATTR_SET_NACK, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_msg_attr_set, STATUS_NONE},
    {VD_MSG_ATTR_STS, 1, VENDOR_MD_LIGHT_S, VENDOR_MD_LIGHT_C, cb_vd_msg_attr_status, STATUS_NONE},
    {VD_MSG_ATTR_INDICA, 0, VENDOR_MD_LIGHT_S, VENDOR_MD_LIGHT_C, cb_vd_msg_attr_indication, VD_MSG_ATTR_CONFIRM},
    {VD_MSG_ATTR_CONFIRM, 1, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_msg_attr_confirm, STATUS_NONE},
		#if ALI_MD_TIME_EN
    {VD_MSG_ATTR_UPD_TIME_REQ, 1, VENDOR_MD_LIGHT_S, VENDOR_MD_LIGHT_C, cb_vd_msg_attr_upd_time_req, STATUS_NONE},
    {VD_MSG_ATTR_UPD_TIME_RSP, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_msg_attr_upd_time_rsp, STATUS_NONE},
		#endif
    #endif
    
#elif(VENDOR_OP_MODE_SEL == VENDOR_OP_MODE_DEFAULT)
	
    // {VD_RC_KEY_REPORT, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_key_report, STATUS_NONE},

    #if (DEBUG_VENDOR_CMD_EN)
    {VD_LIGHT_ONOFF_SET, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_light_onoff_set, VD_LIGHT_ONOFF_STATUS},
	{VD_LIGHT_ONOFF_GET, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_light_onoff_get, VD_LIGHT_ONOFF_STATUS},
	{VD_LIGHT_ONOFF_SET_NOACK, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_light_onoff_set, STATUS_NONE},
    {VD_LIGHT_ONOFF_STATUS, 1, VENDOR_MD_LIGHT_S, VENDOR_MD_LIGHT_C, cb_vd_light_onoff_status, STATUS_NONE},
    #endif
        
    #if (FAST_PROVISION_ENABLE)
	{VD_MESH_RESET_NETWORK, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_mesh_reset_network, STATUS_NONE},
	{VD_MESH_ADDR_GET, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_mesh_get_addr, VD_MESH_ADDR_GET_STS},
	{VD_MESH_ADDR_GET_STS, 1, VENDOR_MD_LIGHT_S, VENDOR_MD_LIGHT_C, cb_vd_mesh_addr_sts, STATUS_NONE},
	{VD_MESH_ADDR_SET, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_mesh_set_addr, VD_MESH_ADDR_SET_STS},
	{VD_MESH_ADDR_SET_STS, 1, VENDOR_MD_LIGHT_S, VENDOR_MD_LIGHT_C, cb_vd_mesh_addr_sts, STATUS_NONE},
    {VD_MESH_PROV_DATA_SET, 1, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_mesh_set_provision_data, STATUS_NONE},
    {VD_MESH_PROV_CONFIRM, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_mesh_provision_confirm, VD_MESH_PROV_CONFIRM_STS},
    {VD_MESH_PROV_CONFIRM_STS, 1, VENDOR_MD_LIGHT_S, VENDOR_MD_LIGHT_C, cb_vd_mesh_provison_data_sts, STATUS_NONE},
    {VD_MESH_PROV_COMPLETE, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_mesh_provision_complete, STATUS_NONE},
    #endif
    
    #if (LPN_VENDOR_SENSOR_EN)
    {VD_LPN_SENSOR_GET,0,VENDOR_MD_LIGHT_C,VENDOR_MD_LIGHT_S,cb_vd_lpn_sensor_get,VD_LPN_SENSOR_STATUS},
    {VD_LPN_SENSOR_STATUS, 1, VENDOR_MD_LIGHT_S, VENDOR_MD_LIGHT_C, cb_vd_lpn_sensor_sts, STATUS_NONE},
    #endif

#if LIGHT_OPT_VENDOR_ENABLE
    {VD_LIGHT_OPT_SET, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_dev_opt_set, VD_LIGHT_OPT_STATUS},
	{VD_LIGHT_OPT_GET, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_dev_opt_get, VD_LIGHT_OPT_STATUS},
	{VD_LIGHT_OPT_SET_NOACK, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_dev_opt_set, STATUS_NONE},
    {VD_LIGHT_OPT_STATUS, 1, VENDOR_MD_LIGHT_S, VENDOR_MD_LIGHT_C, cb_vd_dev_opt_status, STATUS_NONE},
#endif
    // Light Status
    {VD_LIGHT_STATE_GET, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_light_state_get, VD_LIGHT_STATE_STATUS},
    {VD_LIGHT_STATE_STATUS, 1, VENDOR_MD_LIGHT_S, VENDOR_MD_LIGHT_C, cb_vd_light_state_status, STATUS_NONE},

    // Binding locally
    {VD_BINDING_CONTROL_LOCALLY, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_control_locally, STATUS_NONE},

    // Factory Reset
    {VD_CONFIG_NODE_SET_NOACK, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_config_node_set, STATUS_NONE},

    // Add Group
    {VD_CONFIG_MODEL_SUB_SET, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_config_model_sub_set, VD_CONFIG_MODEL_SUB_STATUS},
    {VD_CONFIG_MODEL_SUB_STATUS, 1, VENDOR_MD_LIGHT_S, VENDOR_MD_LIGHT_C, cb_vd_config_model_sub_status, STATUS_NONE},

    // Vendor Scene
    {VD_SCENE_REQUEST, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_scene_request, VD_SCENE_RESPONSE},
    {VD_SCENE_REQUEST_NOACK, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_scene_request_noack, STATUS_NONE},
    {VD_SCENE_RESPONSE, 1, VENDOR_MD_LIGHT_S, VENDOR_MD_LIGHT_C, cb_vd_scene_response, STATUS_NONE},

    // Execution Scene
	{VD_SETUP_EXECUTION_SCENE_SET, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_setup_execution_scene_set, VD_SETUP_EXECUTION_SCENE_STATUS},
	{VD_SETUP_EXECUTION_SCENE_STATUS, 1, VENDOR_MD_LIGHT_S, VENDOR_MD_LIGHT_C, cb_vd_setup_execution_scene_status, STATUS_NONE},

	// Test Production
	{VD_TEST_PRODUCTION_STATUS, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_test_production_status, STATUS_NONE},

	// Test Led
	{VD_AUTO_TEST_SET_NOACK, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_test_production_set, STATUS_NONE},

	// CONFIG_NODE_SET_ACK
	{VD_CONFIG_NODE_SET_ACK, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, cb_vd_config_node_set_ack, STATUS_NONE},
	{VD_CONFIG_NODE_STATUS, 1, VENDOR_MD_LIGHT_S, VENDOR_MD_LIGHT_C,  cb_vd_config_node_status, STATUS_NONE},

#endif
};

#if WIN32
void APP_set_vd_id_mesh_cmd_vd_func(u16 vd_id)
{
    foreach_arr(i,mesh_cmd_vd_func){
        //SET_FLD_V(mesh_cmd_vd_func[i].model_id_tx, BIT_RNG(0,15), vd_id);
        //SET_FLD_V(mesh_cmd_vd_func[i].model_id_rx, BIT_RNG(0,15), vd_id);
        mesh_cmd_vd_func[i].model_id_tx = (mesh_cmd_vd_func[i].model_id_tx & 0xFFFF0000) | vd_id;
        mesh_cmd_vd_func[i].model_id_rx = (mesh_cmd_vd_func[i].model_id_rx & 0xFFFF0000) | vd_id;
    }
}
#endif


// don't modify mesh_search_model_id_by_op_vendor()
int mesh_search_model_id_by_op_vendor(mesh_op_resource_t *op_res, u16 op, u8 tx_flag)
{
    foreach_arr(i,mesh_cmd_vd_func){
        if(op == mesh_cmd_vd_func[i].op){
            op_res->cb = mesh_cmd_vd_func[i].cb;
            op_res->op_rsp = mesh_cmd_vd_func[i].op_rsp;
            op_res->sig = 0;
            op_res->status_cmd = mesh_cmd_vd_func[i].status_cmd ? 1 : 0;
            if(tx_flag){
                op_res->id = mesh_cmd_vd_func[i].model_id_tx;
            }else{
                op_res->id = mesh_cmd_vd_func[i].model_id_rx;
            }
            return 0;
        }
    }
    return -1;
}

//--vendor command interface-------------------
#if (VENDOR_OP_MODE_SEL == VENDOR_OP_MODE_DEFAULT)
int vd_cmd_key_report(u16 adr_dst, u8 key_code)
{
	vd_rc_key_report_t key_report = {0};
	key_report.code = key_code;

	return SendOpParaDebug(adr_dst, 0, VD_RC_KEY_REPORT, (u8 *)&key_report, sizeof(key_report));
}

    #if DEBUG_VENDOR_CMD_EN
int vd_cmd_onoff(u16 adr_dst, u8 rsp_max, u8 onoff, int ack)
{
	vd_light_onoff_set_t par = {0};
	par.onoff = onoff;
	par.tid = 0;

	return SendOpParaDebug(adr_dst, rsp_max, ack ? VD_LIGHT_ONOFF_SET : VD_LIGHT_ONOFF_SET_NOACK, 
						   (u8 *)&par, sizeof(vd_light_onoff_set_t));
}
    #endif
#endif

//--vendor command interface end----------------


int is_cmd_with_tid_vendor(u8 *tid_pos_out, u16 op, u8 tid_pos_vendor_app)
{
    int cmd_with_tid = 0;
    switch(op){
        default:
            break;
    #if (VENDOR_OP_MODE_SEL == VENDOR_OP_MODE_DEFAULT)        
		case VD_RC_KEY_REPORT:
			break;
	#endif
            
	#if DEBUG_VENDOR_CMD_EN
        case VD_LIGHT_ONOFF_SET:
        case VD_LIGHT_ONOFF_SET_NOACK:
            cmd_with_tid = 1;
            *tid_pos_out = OFFSETOF(vd_light_onoff_set_t, tid);
            break;
    #endif

	#if SPIRIT_VENDOR_EN
		case VD_MSG_ATTR_SET:
		case VD_MSG_ATTR_SET_NACK:
		case VD_MSG_ATTR_INDICA:
		case VD_MSG_ATTR_CONFIRM: // just RX for server model
		#if ALI_MD_TIME_EN
		case VD_MSG_ATTR_UPD_TIME_REQ:
		case VD_MSG_ATTR_UPD_TIME_RSP:
		#endif
			cmd_with_tid = 1;
            *tid_pos_out = 0;
			break;
		case VD_MSG_ATTR_STS:
			if(!is_rsp_cmd_tid){
				cmd_with_tid = 1;
	            *tid_pos_out = 0;
			}
			break;
	#endif
    }

    #if VC_SUPPORT_ANY_VENDOR_CMD_EN
    if((!cmd_with_tid) && (tid_pos_vendor_app != TID_NONE_VENDOR_OP_VC)){
        if(tid_pos_vendor_app != 0){
            cmd_with_tid = 1;
            *tid_pos_out = tid_pos_vendor_app - 1;
        }
    }
    #endif

    return cmd_with_tid;
}
#endif

