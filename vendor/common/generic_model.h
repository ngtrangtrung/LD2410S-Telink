/********************************************************************************************************
 * @file     generic_model.h 
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

#pragma once

#include "../../proj/tl_common.h"
#include "../../proj_lib/sig_mesh/app_mesh.h"


typedef int (*typeGen_handleCommandSigOnOffSet)(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);

//----------------------------------- op code
// op cmd 0xxxxxxx (SIG)

// op cmd 10xxxxxx xxxxxxxx (SIG)

//------------------vendor op end-------------------

extern u16 size_page0;


int g_onoff_set(mesh_cmd_g_onoff_set_t *p_set, int par_len, int force_last, int idx, u8 retransaction, st_pub_list_t *pub_list);

#if (!MD_SERVER_EN)
//----generic onoff
#define mesh_cmd_sig_g_onoff_get            (0)
#define mesh_cmd_sig_g_onoff_set            (0)
//----generic level
#define mesh_cmd_sig_g_level_get            (0)
//generic default transition time
#define mesh_cmd_sig_def_trans_time_get     (0)
#define mesh_cmd_sig_def_trans_time_set     (0)
//generic power onoff
#define mesh_cmd_sig_g_on_powerup_get       (0)
#define mesh_cmd_sig_g_on_powerup_set       (0)
//----generic power
#define mesh_cmd_sig_g_power_get            (0)
#define mesh_cmd_sig_g_power_set            (0)
//----generic power last
#define mesh_cmd_sig_g_power_last_get       (0)
//----generic power default
#define mesh_cmd_sig_g_power_def_get        (0)
#define mesh_cmd_sig_g_power_def_set        (0)
//----generic power range
#define mesh_cmd_sig_g_power_range_get      (0)
#define mesh_cmd_sig_g_power_range_set      (0)
//battery model
#define mesh_cmd_sig_battery_get            (0)
#define mesh_cmd_sig_g_battery_get          (0)
//location model
#define mesh_cmd_sig_g_location_global_get  (0)
#define mesh_cmd_sig_g_location_local_get   (0)
#define mesh_cmd_sig_g_location_global_set  (0)
#define mesh_cmd_sig_g_global_local_set     (0)
#endif

#if (!MD_CLIENT_EN)
#define mesh_cmd_sig_g_level_status         (0)
#define mesh_cmd_sig_def_trans_time_status  (0)
#define mesh_cmd_sig_g_on_powerup_status    (0)
#define mesh_cmd_sig_g_power_status         (0)
#define mesh_cmd_sig_g_power_last_status    (0)
#define mesh_cmd_sig_g_power_def_status     (0)
#define mesh_cmd_sig_g_power_range_status   (0)
#define mesh_cmd_sig_g_battery_status       (0)
#define mesh_cmd_sig_g_location_global_status   (0)
#define mesh_cmd_sig_g_location_local_status    (0)
#endif
// -----------

extern u8 enable_blink_led_add_sub_flag;

int send_config_node_reset_status_manual(void);
void mesh_set_start_wait_add_appkey_msg_after_set_node_info(u8 en);
void mesh_handle_reset_node_if_cannot_receive_add_ak_msg();
int mesh_tx_cmd_g_onoff_st(u8 idx, u16 ele_adr, u16 dst_adr, u8 *uuid, model_common_t *pub_md, u16 op_rsp);
int mesh_cmd_sig_g_on_powerup_set(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);
// New Func
void gen_callback_init(typeGen_handleCommandSigOnOffSet callbackFunc);
int send_device_status_with_check_dst_adr(mesh_cb_fun_par_t *cb_par);
int mesh_cmd_sig_g_onoff_set(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);

