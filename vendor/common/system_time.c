/********************************************************************************************************
 * @file     system_time.c 
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
#include "../../vendor/common/time_model.h"
#include "time_model.h"
#include "lighting_model_LC.h"
#include "system_time.h"
#include "fast_provision_model.h"
#include "app_heartbeat.h"
#include "../../vendor/common/generic_model.h"

#include "../mesh/user/auto_test.h"
#include "../mesh/user/in_out.h"
#include "../mesh/user/execution_scene.h"
#include "../mesh/user/handle_ev.h"
#include "../mesh/user/bind.h"

#if ALI_MD_TIME_EN
#include "user_ali_time.h"
#endif

#ifndef __PROJECT_MESH_SWITCH__
#define __PROJECT_MESH_SWITCH__     0
#endif

u32 system_time_ms = 0;
u32 system_time_100ms = 0;
u32 system_time_s = 0;
u32 system_time_tick;
#define CHECK_INTERVAL      (1 * CLOCK_SYS_CLOCK_1MS)

void system_timer_handle_ms(){
	if(get_enable_test_led_flag() == ENABLE){
		handle_test_led_program();
	}else{
		#if MD_SERVER_EN
		light_transition_proc();
		mesh_handle_reset_node_if_cannot_receive_add_ak_msg();
		#endif
	}
	// vendor model
	scene_reg_response_loop();
}

void system_timer_handle_100ms()
{
	if(!is_lpn_support_and_en){
        #if (!__PROJECT_MESH_SWITCH__)
        mesh_beacon_poll_100ms();
        #endif
	}
	mesh_heartbeat_poll_100ms();
#if ALI_MD_TIME_EN
	user_ali_time_proc();
#endif
	io_loop_task();
	execution_loop_task();
	ev_handle_loop_task();
	handle_send_vd_config_node_with_delay();
#if BIND_APPKEY_PERIODIC
	bind_periodic_handle();
#endif
}

#if FEATURE_LOWPOWER_EN
_attribute_ram_code_
#endif
void system_time_run(){
    mesh_iv_update_start_poll();
    
    u32 clock_tmp = clock_time();
    u32 t_delta = (u32)(clock_tmp - system_time_tick);

    if(t_delta >= CHECK_INTERVAL){
        u32 interval_cnt = t_delta/CHECK_INTERVAL;
        if(interval_cnt){
            u32 inc_100ms = (system_time_ms % 100 + interval_cnt) / 100;
			system_time_ms += interval_cnt;
						
			if(!(FEATURE_LOWPOWER_EN || SPIRIT_PRIVATE_LPN_EN || __PROJECT_MESH_SWITCH__)){ // (!is_lpn_support_and_en){ // it will cost several tens ms from wake up
			    foreach(i,interval_cnt){
			        system_timer_handle_ms();
			    }
			}
			if(inc_100ms){
                u32 inc_s = (system_time_100ms % 10 + inc_100ms) / 10;
    			system_time_100ms += inc_100ms;
                if(is_lpn_support_and_en || SPIRIT_PRIVATE_LPN_EN || __PROJECT_MESH_SWITCH__){ // it will cost several ms from wake up
                    system_timer_handle_100ms();	// only run once to save time
				}else{
					foreach(i,inc_100ms){
				        system_timer_handle_100ms();
				    }
				}
    			if(inc_s){
    			    system_time_s += inc_s;
                    foreach(i,inc_s){
                        mesh_iv_update_st_poll_s();
						user_system_time_proc();
						#if VC_APP_ENABLE
						void sys_timer_refresh_time_ui();
						sys_timer_refresh_time_ui();
						#endif
				    }
    			}
            }
        }
        system_time_tick += interval_cnt * CHECK_INTERVAL;
        // proc handle 1ms or greater
        light_par_save_proc();

	#if FAST_PROVISION_ENABLE
		mesh_fast_prov_proc();
	#endif
    #if MD_SERVER_EN
        #if MD_TIME_EN
        mesh_time_proc();
        #endif
	    #if (MD_LIGHT_CONTROL_EN)
        LC_property_proc();
	    #endif
	    #if (MD_SCENE_EN)
        scene_status_change_check_all();
	    #endif
        #if ONLINE_STATUS_EN
        online_st_proc();
        #endif

    	if(get_provision_state() == STATE_DEV_PROVED){
			static u32 tmp_current_time_to_publish = 0;
			if(clock_time_exceed_ms(  \
					tmp_current_time_to_publish, publish_status_periodically_ms)){
				set_publish_status_periodically_interval();
				tmp_current_time_to_publish = system_time_ms;
				publish_status_periodically();
			}
    	}

    #endif
    }

#if MD_SERVER_EN
    #if(MD_SENSOR_EN)
	sensor_measure_proc();
	#endif
#endif
}

u32 clock_time_exceed_ms(u32 ref, u32 span_ms){
	return ((u32)(clock_time_ms() - ref) > span_ms);
}

u32 clock_time_exceed_100ms(u32 ref, u32 span_100ms){
	return ((u32)(clock_time_100ms() - ref) > span_100ms);
}

u32 clock_time_exceed_s(u32 ref, u32 span_s){
	return ((u32)(clock_time_s() - ref) > span_s);
}

