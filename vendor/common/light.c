/********************************************************************************************************
 * @file     light.c 
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

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "../../proj_lib/ble/ll/ll.h"
#include "../../proj_lib/ble/blt_config.h"
#include "../../proj_lib/sig_mesh/app_mesh.h"
#include "../../proj_lib/ble/service/ble_ll_ota.h"
#include "../../vendor/common/lighting_model.h"
#include "../../vendor/common/lighting_model_HSL.h"
#include "../../vendor/common/lighting_model_xyL.h"
#include "../../vendor/common/lighting_model_LC.h"
#include "../../vendor/common/generic_model.h"
#include "../../vendor/common/scene.h"
#include "../mesh/user/factory_reset.h"
#include "../mesh/user/led.h"
#include "../mesh/user/utilities.h"
#include "../mesh/user/handle_ev.h"
#include "../mesh/user/relay.h"
#include "../mesh/user/binding_ctl.h"
#include "light.h"
#if HOMEKIT_EN
#include "../../vendor/common/led_cfg.h"
#endif
#if WIN32
#include <stdlib.h>
#else
#include "../../proj/mcu/watchdog_i.h"
#include "../../proj_lib/pm.h"
#endif

#include "../mesh/user/debug.h"
#ifdef LIGHT_DBG_EN
#define DBG_LIGHT_SEND_STR(x)   Dbg_sendString((s8*)x)
#define DBG_LIGHT_SEND_INT(x)   Dbg_sendInt(x)
#define DBG_LIGHT_SEND_HEX(x)   Dbg_sendHex(x)
#else
#define DBG_LIGHT_SEND_STR(x)
#define DBG_LIGHT_SEND_INT(x)
#define DBG_LIGHT_SEND_HEX(x)
#endif
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

const u16 output_control_io_pin[LIGHT_CNT] = {
		RL1_IO_PIN,
#if	LIGHT_CNT > 1
		RL2_IO_PIN,
#endif
#if	LIGHT_CNT > 2
		RL3_IO_PIN,
#endif
#if	LIGHT_CNT > 3
		RL4_IO_PIN
#endif
};

#define LIGHT_ADJUST_INTERVAL       (20)   // unit :ms;     min:20ms; max 100ms

light_res_sw_save_t light_res_sw_save[LIGHT_CNT] = {{{{0}}}};
light_res_sw_trans_t light_res_sw[LIGHT_CNT] ;

#define P_SW_LEVEL_SAVE(idx, type)	(&light_res_sw_save[idx].level[type])
#define P_ST_TRANS(idx, type)		(&light_res_sw[idx].trans[type])

u8 light_pub_trans_step = ST_PUB_TRANS_IDLE;    // 0
u8 *light_pub_model_priority = 0;

STATIC_ASSERT(LIGHTNESS_DEFAULT != 0);	// if want to set 0, please set ONOFF_DEFAULT to 0,
STATIC_ASSERT(LIGHTNESS_MIN != 0);
//STATIC_ASSERT(sizeof(light_res_sw_t) % 4 == 0); // for align
STATIC_ASSERT(LIGHT_ADJUST_INTERVAL <= 100);

static u32 tick_light_save;
publish_status_delay_t publish_status_delay[LIGHT_CNT];

u8 ct_flag = 0;

#if (LIGHT_CNT > 1) && ENABLE_BLINK_SINGLE_LED
static u16 blink_mask = 0xFFFF;
#endif

u32 control_output_start_t_ms[LIGHT_CNT] = {
			0,
	#if LIGHT_CNT > 1
			0,
	#endif
	#if LIGHT_CNT > 2
			0,
	#endif
	#if LIGHT_CNT > 3
			0,
	#endif
};

/******************************************************************************/
/*                          PRIVATE FUNCTIONS DECLERATION                     */
/******************************************************************************/

/******************************************************************************/
/*                           EXPORT FUNCTIONS                                 */
/******************************************************************************/

/**
 * @func    set_on_power_up_onoff
 * @brief   Set power up value
 * @param   INT: idx - Light Index
 *          INT: Transition Type (0: ST_TRANS_CTL_TEMP, 1: ST_TRANS_HSL_HUE...)
 * @retval  None
 */
void set_on_power_up_onoff(int idx, int st_trans_type, u8 onoff)
{
	sw_level_save_t *p_save = P_SW_LEVEL_SAVE(idx, st_trans_type);
	p_save->onoff = onoff;
	light_par_save(0);
}

/**
 * @func    set_on_power_up_last
 * @brief   Set power up last to backup light state
 * @param
 * @retval  None
 */
void set_on_power_up_last(sw_level_save_t *p_save, s16 last)
{
	if(LEVEL_OFF == last){
		p_save->onoff = 0;  // active for all level. include CT, Hue, Sat...
	}else{
		p_save->onoff = 1;
		p_save->last = last;
	}
}

/**
 * @func    get_on_power_up_last
 * @brief   None
 * @param
 * @retval
 */
s16 get_on_power_up_last(sw_level_save_t *p_save)
{
	return (p_save->onoff ? p_save->last : LEVEL_OFF);
}
/**
 * @func    mesh_global_var_init_light_sw
 * @brief   None
 * @param
 * @retval
 */
void mesh_global_var_init_light_sw()
{
	foreach_arr(i,light_res_sw){
		foreach_arr(k,light_res_sw[i].trans){
			sw_level_save_t *p_save = &light_res_sw_save[i].level[k];
			st_transition_t *p_trans = &light_res_sw[i].trans[k];
			s16 val_init = 0;
			if(ST_TRANS_LIGHTNESS == k){
				val_init = u16_to_s16(LIGHTNESS_DEFAULT);
				p_save->min = u16_to_s16(LIGHTNESS_MIN);
				p_save->max = u16_to_s16(LIGHTNESS_MAX);
			    p_save->def = u16_to_s16(0);	// use last, spec page172 requested.
			}
			
            p_save->last = val_init;    // must init "last", even though it's -32768.
			p_save->onoff = (ST_TRANS_LIGHTNESS == k) ? ONOFF_DEFAULT : 1;
            
			p_trans->present = p_trans->target = get_on_power_up_last(p_save);
		}
		ONPOWER_UP_VAL(i) = ONPOWER_UP_STORE; //ONPOWER_UP_STORE; // ONPOWER_UP_DEFAULT; //
		g_def_trans_time_val(i) = PTS_TEST_EN ? 0 : TRANSITION_TIME_DEFAULT_VAL;
	}
}

/**
 * @func    light_res_sw_load
 * @brief   None
 * @param
 * @retval
 */
void light_res_sw_load()
{
	foreach_arr(i,light_res_sw){
		foreach_arr(k,light_res_sw[i].trans){
			sw_level_save_t *p_save = &light_res_sw_save[i].level[k];
			st_transition_t *p_trans = &light_res_sw[i].trans[k];
			s16 level_poweron = 0;
			if((ONPOWER_UP_STORE == ONPOWER_UP_VAL(i))
					|| ((ST_TRANS_LIGHTNESS == k)
							&& (analog_read(DEEP_ANA_REG0)&BIT(OTA_REBOOT_FLAG)))){
				level_poweron = get_on_power_up_last(p_save);
			}
			else if((ONPOWER_UP_OFF == ONPOWER_UP_VAL(i))&&(ST_TRANS_LIGHTNESS == k)){
			    // ONPOWER_UP_OFF only for lightness, others is same to default. please refer to LCTL/BV-06-C 
				level_poweron = LEVEL_OFF;
				set_on_power_up_last(p_save, level_poweron);
			}
			else{	// (ONPOWER_UP_DEFAULT == ONPOWER_UP_VAL(i))
                s16 def = light_g_level_def_get(i, k);
                if(ST_TRANS_LIGHTNESS == k){
                    if(LEVEL_OFF == def){
                        def = p_save->last;
                    }
                }
                #if (0 == PTS_TEST_EN)  // PTS MMDL/SR/LCTL/BV-06 will failed because spec page 174, 6.1.3.2
			    if (ONPOWER_UP_DEFAULT == ONPOWER_UP_VAL(i) && (ST_TRANS_LIGHTNESS != k)){
			        level_poweron = get_on_power_up_last(p_save);
			    }else
			    #endif
			    {
                    level_poweron = def;
                    set_on_power_up_last(p_save, level_poweron);
                }
			} 
			p_trans->present = p_trans->target = level_poweron;
		}
	}
}
/**
 * @func    light_transition_onoff_manual
 * @brief   None
 * @param
 * @retval
 */
void light_transition_onoff_manual(u8 onoff, u8 transit_t, u8 light_idx)
{
	g_addr_dest_arr[light_idx] = ADR_UNASSIGNED;
	mesh_cmd_g_onoff_set_t cmd_onoff = {0, 0, 0, 0};
    cmd_onoff.onoff = !!onoff;
    cmd_onoff.transit_t = transit_t;
    st_pub_list_t pub_list = {{0}};
    g_onoff_set(&cmd_onoff, sizeof(cmd_onoff),1,light_idx,0, &pub_list);
}
/**
 * @func    edch_is_exist
 * @brief   None
 * @param
 * @retval
 */
u8 edch_is_exist()
{
	u32 *p_edch = (u32 *) FLASH_ADR_EDCH_PARA;
	if(*p_edch == 0xffffffff){
		return 0;
	}	
	return 1;
}
/**
 * @func    light_init_st
 * @brief   None
 * @param   None
 * @retval  None
 */
void light_init_st()
{
	foreach(i, LIGHT_CNT){
        // Init Value
        int onoff_present = light_g_onoff_present_get(i);
		light_transition_onoff_manual(G_OFF, 0, i);
		if(onoff_present){
			light_transition_onoff_manual(G_ON, (analog_read(DEEP_ANA_REG0)&BIT(OTA_REBOOT_FLAG))?0:edch_is_exist()?g_def_trans_time_val(i):0, i);
		}
    }
}

/**
 * @func    light_par_save
 * @brief   None
 * @param
 * @retval
 */
void light_par_save(int quick)
{
	tick_light_save = (quick ? (clock_time() - BIT(31)) : clock_time()) | 1;
}
/**
 * @func    light_par_save_proc
 * @brief   None
 * @param
 * @retval
 */
void light_par_save_proc()
{
	// save proc
	if(tick_light_save && clock_time_exceed(tick_light_save, US_DELAY_TIME_TO_SAVE_LIGHT_STATE)){
		tick_light_save = 0;
		/*
		if(!is_actived_factory_test_mode()){
		    mesh_common_store(FLASH_ADR_SW_LEVEL);
		}
		*/
		mesh_common_store(FLASH_ADR_SW_LEVEL);
	}
}

#if (MD_SCENE_EN)
/**
 * @func    scene_status_change_check_all
 * @brief   None
 * @param
 * @retval
 */
void scene_status_change_check_all()
{
	foreach_arr(i,light_res_sw){
		foreach_arr(trans_type,light_res_sw[i].trans){
			st_transition_t *p_trans = P_ST_TRANS(i, trans_type);
        	scene_status_change_check(i, p_trans->present, trans_type);
	    }
	}
}
#endif


#if MD_SERVER_EN
/**
 * @func    light_res_sw_g_level_last_set
 * @brief   None
 * @param
 * @retval  None
 */
void light_res_sw_g_level_last_set(int idx, int st_trans_type)
{
	sw_level_save_t *p_save = P_SW_LEVEL_SAVE(idx, st_trans_type);
	st_transition_t *p_trans = P_ST_TRANS(idx, st_trans_type);
	set_on_power_up_last(p_save, p_trans->target);
	light_par_save(0);
}
#endif

/**
 * @func    light_res_sw_g_level_set
 * @brief   None
 * @param
 * @retval  None
 */
void light_res_sw_g_level_set(int idx, s16 level, int init_time_flag, int st_trans_type)
{
	set_level_current_type(idx, st_trans_type);
	st_transition_t *p_trans = P_ST_TRANS(idx, st_trans_type);
	p_trans->present = level;
	if(init_time_flag){
		p_trans->target = level;
		p_trans->remain_t_ms = 0;
		p_trans->delay_ms = 0;
	}
}
/**
 * @func    light_res_sw_g_level_target_set
 * @brief   None
 * @param
 * @retval  None
 */
void light_res_sw_g_level_target_set(int idx, s16 level, int st_trans_type)	// only for move set command
{
	//set_level_current_type(idx, st_trans_type);
	st_transition_t *p_trans = P_ST_TRANS(idx, st_trans_type);
	p_trans->target = level;
	p_trans->remain_t_ms = 0;
	p_trans->delay_ms = 0;
}
/**
 * @func    light_state_refresh
 * @brief   None
 * @param
 * @retval  None
 */
void light_set_hardware(u8 idx, u8 val)
{
	if(idx >= LIGHT_CNT){
		return;
	}

	DBG_LIGHT_SEND_STR("\n ---- DST: ");
	DBG_LIGHT_SEND_HEX(nwk_control_msg_para[idx].dst);

	if(val > 0){
    	RL_changeRlState(idx, Level_High);
    	binding_control(idx, G_ON);
    }
	else{
    	RL_changeRlState(idx, Level_Low);
    	binding_control(idx, G_OFF);
    }


	// RST DST
#if 0
	if(nwk_control_msg_para[idx].dst >= ADR_GROUP_START_POINT) {
		if(app_control_st[idx] != val) {
			nwk_control_msg_para[idx].dst = ele_adr_primary + model_idx;
		}
	}
#endif

	// POWER ON
	if(state_of_group_binding[idx] == SUB_UNKNOWN) {
		state_of_group_binding[idx] = val&1;
	}
	if(nwk_control_msg_para[idx].dst >= ADR_GROUP_START_POINT) {
		nwk_control_msg_para[idx].dst = ele_adr_primary + idx;
		DBG_LIGHT_SEND_STR("\n Reset destination");
	}

	DBG_LIGHT_SEND_STR("\n ---- R_DST: ");
	DBG_LIGHT_SEND_HEX(nwk_control_msg_para[idx].dst);

	static u32 led_busy_last_t = 0;
	if((is_led_busy() == 0)    \
				&& (clock_time_get_elapsed_time(led_busy_last_t) > TIMER_1S) && (g_addr_dest_arr[idx] != ADR_UNASSIGNED)){
	    push_led_event_to_fifo(LED_CMD_CHANGE_STATE, 0xFFFF);
	}
	else {
		led_busy_last_t = clock_time_ms();
	}
	control_output_start_t_ms[idx] = clock_time_ms();
}

/**
 * @func    light_state_refresh
 * @brief   None
 * @param
 * @retval  None
 */
void light_state_refresh(int idx) // idx: index of LIGHT_CNT.
{
	st_transition_t *p_trans = P_ST_TRANS(idx, ST_TRANS_LIGHTNESS);
	u8 lum_100 = level2lum(p_trans->present);
	CB_NL_PAR_NUM_3(p_nl_level_state_changed,idx * ELE_CNT_EVERY_LIGHT + ST_TRANS_LIGHTNESS, p_trans->present, p_trans->target);
	light_set_hardware(idx, lum_100);
}

/**
 * @func    get_light_pub_list
 * @brief   None
 * @param
 * @retval  None
 */
void get_light_pub_list(int st_trans_type, s16 present_level, s16 target_level, int pub_trans_flag, st_pub_list_t *pub_list)
{
    if(pub_trans_flag){
        pub_list->st[st_trans_type] = ST_G_LEVEL_SET_PUB_TRANS;
    }else{
        pub_list->st[st_trans_type] = (target_level != present_level) ? ST_G_LEVEL_SET_PUB_NOW : ST_G_LEVEL_SET_PUB_NONE;
    }
    if(ST_TRANS_LIGHTNESS == st_trans_type){
        if(pub_list->st[ST_TRANS_LIGHTNESS] && ((LEVEL_OFF == present_level) || (LEVEL_OFF == target_level))){
            pub_list->st[ST_TRANS_PUB_ONOFF] = pub_list->st[ST_TRANS_LIGHTNESS];
        }
    }
}

/**
 * @func    light_g_level_set
 * @brief   None
 * @param
 * @retval
 */
int light_g_level_set(int idx, s16 level, int init_time_flag, int st_trans_type, st_pub_list_t *pub_list)
{
	st_transition_t *p_trans = P_ST_TRANS(idx, st_trans_type);

	if(pub_list){
		get_light_pub_list(st_trans_type, p_trans->present, level, 0, pub_list);
	}
	light_res_sw_g_level_set(idx, level, init_time_flag, st_trans_type);
	light_state_refresh(idx);
    return 0;
}

//---------------------------------- GET FUNCTIONS

/**
 * @func    light_remain_time_get
 * @brief   None
 * @param
 * @retval
 */
u8 light_remain_time_get(st_transition_t *p_trans)
{
	u32 remain_ms = p_trans->remain_t_ms;
	u32 delay_ms = p_trans->delay_ms;

	u8 remain_t = get_transition_step_res(remain_ms/100);
	if(0 == remain_t){
		remain_t = get_transition_step_res((delay_ms+99)/100);
	}
	return remain_t;
}
/**
 * @func    light_g_level_get
 * @brief   None
 * @param
 * @retval
 */
int light_g_level_get(u8 *rsp, int idx, int st_trans_type)
{
	if(idx < LIGHT_CNT){
		set_level_current_type(idx, st_trans_type);
		mesh_cmd_g_level_st_t *p_st = (mesh_cmd_g_level_st_t *)rsp;
		st_transition_t *p_trans = P_ST_TRANS(idx, st_trans_type);
		p_st->present_level = p_trans->present;
		p_st->target_level = p_trans->target;
		p_st->remain_t = light_remain_time_get(p_trans);
		return 0;
	}
	return -1;
}

/**
 * @func    light_g_level_def_get
 * @brief   None
 * @param
 * @retval
 */
s16 light_g_level_def_get(int idx, int st_trans_type)
{
	if(idx < LIGHT_CNT){
		sw_level_save_t *p_save = P_SW_LEVEL_SAVE(idx, st_trans_type);
		return p_save->def;
	}
	return 0;
}

/**
 * @func    light_g_level_def_get_u16
 * @brief   None
 * @param
 * @retval
 */
u16 light_g_level_def_get_u16(int idx, int st_trans_type)
{
	return s16_to_u16(light_g_level_def_get(idx, st_trans_type));
}

/**
 * @func    light_g_level_def_set
 * @brief   None
 * @param
 * @retval
 */
int light_g_level_def_set(s16 val, int idx, int st_trans_type)
{
	if(idx < LIGHT_CNT){
		//set_level_current_type(idx, st_trans_type);
		sw_level_save_t *p_save = P_SW_LEVEL_SAVE(idx, st_trans_type);
		p_save->def = val;
		light_par_save(1);
		return 0;
	}
	return -1;
}

/**
 * @func    light_g_level_def_set_u16
 * @brief   None
 * @param
 * @retval
 */
int light_g_level_def_set_u16(u16 val, int idx, int st_trans_type)
{
	return light_g_level_def_set(u16_to_s16(val), idx, st_trans_type);
}

/**
 * @func    light_g_level_range_get
 * @brief   None
 * @param
 * @retval
 */
int light_g_level_range_get(light_range_s16_t *p_range, int idx, int st_trans_type)
{
	if(idx < LIGHT_CNT){
		//set_level_current_type(idx, st_trans_type);
		sw_level_save_t *p_save = P_SW_LEVEL_SAVE(idx, st_trans_type);
		p_range->min = p_save->min;
		p_range->max = p_save->max;
		return 0;
	}else{
		memset(p_range, 0, sizeof(light_range_s16_t));
		return -1;
	}
}

/**
 * @func    light_g_level_range_get_u16
 * @brief   None
 * @param
 * @retval
 */
int light_g_level_range_get_u16(light_range_u16_t *p_range, int idx, int st_trans_type)
{
	light_range_s16_t range_s16 = {0};
	int err = light_g_level_range_get(&range_s16, idx, st_trans_type);
	p_range->min = get_lightness_from_level(range_s16.min);
	p_range->max = get_lightness_from_level(range_s16.max);
	return err;
}

/**
 * @func    light_g_level_range_set
 * @brief   None
 * @param
 * @retval
 */
int light_g_level_range_set(u16 min, u16 max, int idx, int st_trans_type)
{
	if(idx < LIGHT_CNT){
		//set_level_current_type(idx, st_trans_type);
		sw_level_save_t *p_save = P_SW_LEVEL_SAVE(idx, st_trans_type);
		p_save->min = get_level_from_lightness(min);
		p_save->max = get_level_from_lightness(max);
		light_par_save(1);
		return 0;
	}
	return -1;
}

/**
 * @func    light_g_level_target_get
 * @brief   None
 * @param
 * @retval
 */
s16 light_g_level_target_get(int idx, int st_trans_type)
{
	if(idx < LIGHT_CNT){
		set_level_current_type(idx, st_trans_type);
		st_transition_t *p_trans = P_ST_TRANS(idx, st_trans_type);
		return p_trans->target;
	}
	return 0;
}

/**
 * @func    light_g_level_present_get
 * @brief   None
 * @param
 * @retval
 */
s16 light_g_level_present_get(int idx, int st_trans_type)
{
	if(idx < LIGHT_CNT){
		set_level_current_type(idx, st_trans_type);
		st_transition_t *p_trans = P_ST_TRANS(idx, st_trans_type);
		return p_trans->present;
	}
	return 0;
}

/**
 * @func    light_g_level_present_get_u16
 * @brief   None
 * @param
 * @retval
 */
u16 light_g_level_present_get_u16(int idx, int st_trans_type)
{
	return s16_to_u16(light_g_level_present_get(idx, st_trans_type));
}

/**
 * @func    light_g_onoff_present_get
 * @brief   None
 * @param
 * @retval
 */
u8 light_g_onoff_present_get(int idx)
{
    return (light_g_level_present_get(idx, ST_TRANS_LIGHTNESS) != LEVEL_OFF);
}
/**
 * @func    light_g_level_last_get
 * @brief   None
 * @param
 * @retval
 */
s16 light_g_level_last_get(int idx, int st_trans_type)
{
	if(idx < LIGHT_CNT){
		//set_level_current_type(idx, st_trans_type);
		sw_level_save_t *p_save = P_SW_LEVEL_SAVE(idx, st_trans_type);
		return p_save->last;
	}
	return 0;
}

/**
 * @func    get_light_g_level_by_onoff
 * @brief   None
 * @param
 * @retval
 */
s16 get_light_g_level_by_onoff(int idx, int on, int st_trans_type, int force_last)
{
	if(on){
	    s16 last = light_g_level_last_get(idx, st_trans_type);
		if(ST_TRANS_LIGHTNESS == st_trans_type){
            s16 def = light_g_level_def_get(idx, st_trans_type);
		    return ((force_last || (LEVEL_OFF == def)) ? last : def); // refer to Lightness LLN-BV11-C
		}else{
			return last;
		}
	}else{
		return LEVEL_OFF;
	}
}

/**
 * @func    lum2level
 * @brief   None
 * @param
 * @retval
 */
s16 lum2level(u8 lum)
{
	if(lum > 100){
		lum  = 100;
	}
	return (-32768 + division_round(65535*lum,100));
}

/**
 * @func    level2lum
 * @brief   None
 * @param
 * @retval
 */
u8 level2lum(s16 level)
{
	u16 lightness = level + 32768;
	u32 fix_1p2 = 0;
	if(lightness){	// fix decimals
		#define LEVEL_UNIT_1P2	(65535/100/2)
		if(lightness < LEVEL_UNIT_1P2 + 2){     // +2 to fix accuracy missing
			lightness = LEVEL_UNIT_1P2 * 2;		// make sure lum is not zero when light on.
		}
		fix_1p2 = LEVEL_UNIT_1P2;
	}
	return (((lightness + fix_1p2)*100)/65535);
}

/**
 * @func    lum2_lightness
 * @brief   None
 * @param
 * @retval
 */
u16 lum2_lightness(u8 lum)
{
	return (get_lightness_from_level(lum2level(lum)));
}

/**
 * @func    lightness2_lum
 * @brief   None
 * @param
 * @retval
 */
u8 lightness2_lum(u16 lightness)
{
	return (level2lum(get_level_from_lightness(lightness)));
}

/**
 * @func    temp100_to_temp
 * @brief   None
 * @param
 * @retval
 */
u16 temp100_to_temp(u8 temp100)
{
	if(temp100 > 100){
		temp100  = 100;
	}
	return (CTL_TEMP_MIN + ((CTL_TEMP_MAX - CTL_TEMP_MIN)*temp100)/100);
}

/**
 * @func    temp_to_temp100_hw
 * @brief   None
 * @param
 * @retval
 */
u8 temp_to_temp100_hw(u16 temp) // use for driver pwm, 0--100 is absolute value, not related to temp range
{
	if(temp < CTL_TEMP_MIN){
		temp = CTL_TEMP_MIN;
	}
	
	if(temp > CTL_TEMP_MAX){
		temp = CTL_TEMP_MAX;
	}
	u32 fix_1p2 = (CTL_TEMP_MAX - CTL_TEMP_MIN)/100/2;	// fix decimals
	return (((temp - CTL_TEMP_MIN + fix_1p2)*100)/(CTL_TEMP_MAX - CTL_TEMP_MIN));   // temp100 can be zero.
}

/**
 * @func    temp_to_temp100
 * @brief   None
 * @param
 * @retval
 */
u8 temp_to_temp100(u16 temp)
{
	return temp_to_temp100_hw(temp);   // comfirm later, related with temp range
}

/**
 * @func    light_lum_get
 * @brief   None
 * @param
 * @retval
 */
u8 light_lum_get(int idx, int target_flag)
{
	st_transition_t *p_trans = P_ST_TRANS(idx, ST_TRANS_LIGHTNESS);
    return level2lum(target_flag? p_trans->target : p_trans->present);
}

//---------------------------------- SET FUNCTIONS

/**
 * @func    light_onoff_idx
 * @brief   None
 * @param
 * @retval
 */
int light_onoff_idx(int idx, int on, int init_time_flag){
    if(idx < LIGHT_CNT){
    	int st_trans_type = ST_TRANS_LIGHTNESS;
    	st_pub_list_t pub_list = {{0}};
    	light_g_level_set(idx, get_light_g_level_by_onoff(idx, on, st_trans_type, 1), init_time_flag, st_trans_type, &pub_list);
    }
    return 0;
}

/**
 * @func    light_g_level_set_idx
 * @brief   None
 * @param
 * @retval
 */
int light_g_level_set_idx(int idx, s16 level, int init_time_flag, int st_trans_type, st_pub_list_t *pub_list)
{
    if(idx < LIGHT_CNT){
        light_g_level_set(idx, level, init_time_flag, st_trans_type, pub_list);
    }
    return 0;
}

#if MD_SERVER_EN
/**
 * @func    light_g_level_set_idx_with_trans
 * @brief   None
 * @param
 * @retval
 */
void light_g_level_set_idx_with_trans(u8 *set_trans, int idx, int st_trans_type)
{
    if(idx < LIGHT_CNT){
		set_level_current_type(idx, st_trans_type);
		st_transition_t *p_trans = P_ST_TRANS(idx, st_trans_type);
		mesh_set_trans_t *p_set = (mesh_set_trans_t *)set_trans;
		p_trans->target = p_set->target_val;
		if(0x3F == (p_set->transit_t & 0x3F)){
			p_trans->remain_t_ms = -1;
		}else{
			p_trans->remain_t_ms = 100 * get_transition_100ms((trans_time_t *)&p_set->transit_t);
			if(p_trans->remain_t_ms){
                if(is_level_move_set_op(p_set->op)){
                    // have been make sure (target_val != present_val) and (level_move != 0) before.
				    p_trans->step_1p32768 = ((p_set->level_move * 32768) /(s32)(p_trans->remain_t_ms)) * LIGHT_ADJUST_INTERVAL;
                    u32 abs_step = abs(p_set->level_move);
                    u32 abs_delta = (p_set->target_val - p_set->present_val);
                    u32 mod = abs_delta % abs_step;
                    u32 remain_t_ms_org = p_trans->remain_t_ms;
                    u32 val;
                    if(remain_t_ms_org >= 65536){
                        // remain_t_ms_org is less than 37800000, so (remain_t_ms_org * 100) is less than 0xffffffff
                        val = (((remain_t_ms_org * 100)/abs_step)*(mod))/100;
                    }else{
                        // make sure not overflow
                        val = (remain_t_ms_org * mod) / abs_step;
                    }
                    p_trans->remain_t_ms = remain_t_ms_org * (abs_delta / abs_step) + val;
                }else{
				    p_trans->step_1p32768 = (((p_trans->target - p_trans->present) * 32768) /(s32)(p_trans->remain_t_ms)) * LIGHT_ADJUST_INTERVAL;
				}
			}else{
				p_trans->step_1p32768 = 0;
			}
			p_trans->present_1p32768 = 0; // init
		}
		p_trans->delay_ms = p_set->delay * 5;
    }
}
#endif

/**
 * @func    light_onoff_all
 * @brief   None
 * @param
 * @retval
 */
void light_onoff_all(u8 on){
    foreach(i, LIGHT_CNT){
        light_onoff_idx(i, on, 1);
    }
}

/**
 * @func    set_light_linear_flag
 * @brief   None
 * @param
 * @retval
 */
int set_light_linear_flag(int idx,u16 linear)
{
	if(idx < LIGHT_CNT){
	    light_res_sw[idx].linear_set_flag = 1;
	    light_res_sw[idx].linear = linear;
	}
	return 0;
}

/**
 * @func    clear_light_linear_flag
 * @brief   None
 * @param
 * @retval
 */
int clear_light_linear_flag(int idx)
{
	if(idx < LIGHT_CNT){
	    light_res_sw[idx].linear_set_flag = 0;
	    light_res_sw[idx].linear = 0;
	}
	return 0;
}

/**
 * @func    get_light_linear_val
 * @brief   None
 * @param
 * @retval
 */
u16 get_light_linear_val(int idx)
{
    return light_res_sw[idx].linear;
}

/**
 * @func    is_linear_flag
 * @brief   None
 * @param
 * @retval
 */
int is_linear_flag(int idx)
{
	if(idx < LIGHT_CNT){
	    return light_res_sw[idx].linear_set_flag;
	}
	return 0;
}

/**
 * @func    app_led_en
 * @brief   None
 * @param
 * @retval
 */
void app_led_en (int id, int en)
{
    if(id < LIGHT_CNT){
        light_onoff_idx(id, en, 1);
    }
}

#if MD_SERVER_EN

//---------------------------------- TRANSITION AND DELAY PROCESS
/**
 * @func    light_get_next_level
 * @brief   None
 * @param
 * @retval
 */
#if MD_LIGHTNESS_EN
s16 light_get_next_level(int idx, int st_trans_type)
{
    st_transition_t *p_trans = P_ST_TRANS(idx, st_trans_type);
    sw_level_save_t *p_save = P_SW_LEVEL_SAVE(idx, st_trans_type);
	s32 adjust_1p32768 = p_trans->step_1p32768+ p_trans->present_1p32768;
	s32 result = p_trans->present + (adjust_1p32768 / 32768);
	p_trans->present_1p32768 = adjust_1p32768 % 32768;
    result = get_val_with_check_range(result, p_save->min, p_save->max, st_trans_type);
	return (s16)result;
}
#endif
/**
 * @func    light_transition_log
 * @brief   None
 * @param
 * @retval
 */
void light_transition_log(int st_trans_type, s16 present_level)
{
	if(ST_TRANS_LIGHTNESS == st_trans_type){
		LOG_MSG_INFO(TL_LOG_MESH,0,0,"present_level %d", present_level);
	}else{
		LOG_MSG_INFO(TL_LOG_MESH,0,0,"xxxx 0x%04x", s16_to_u16(present_level));
	}
}

/**
 * @func   light_handle_publish_status_delay
 * @brief  None
 * @param  None
 * @retval None
 */
void light_handle_publish_status_delay(void){

	for(u8 i = 0; i < LIGHT_CNT; i++){
		if(publish_status_delay[i].flag == TRUE){
			if(clock_time_exceed_ms(publish_status_delay[i].delay_start_time,
							  publish_status_delay[i].delay_time)){
				publish_status_delay[i].flag = FALSE;
				// mesh_tx_cmd_lightness_st(i, ele_adr_primary + i, GATEWAY_UNICAST_ADDR, LIGHTNESS_STATUS, 0, 0);
				mesh_tx_cmd_g_onoff_st(
							i, ele_adr_primary + i, GATEWAY_UNICAST_ADDR, 0, 0, G_ONOFF_STATUS
						);
			}
		}
    }
}


static BOOL power_up_flag[LIGHT_CNT] =
		{
				TRUE,
			#if LIGHT_CNT > 1
				TRUE,
			#endif
			#if LIGHT_CNT > 2
				TRUE,
			#endif
			#if LIGHT_CNT > 3
				TRUE,
			#endif
			#if LIGHT_CNT > 4
				TRUE,
			#endif
		};

/**
 * @func    light_publish_status_delay
 * @brief   None
 * @param
 * @retval
 */
void light_publish_status_delay(u8 idx, u32 delay_time)
{
	if(idx < LIGHT_CNT){
		publish_status_delay[idx].flag = TRUE;
		if(power_up_flag[idx] == TRUE){
			publish_status_delay[idx].delay_time = 5000 + ((rand()% 20000));
			power_up_flag[idx] = FALSE;
		}
		else{
		    publish_status_delay[idx].delay_time = delay_time + PUBLISH_DELAY_OFFSET;
		}
		publish_status_delay[idx].delay_start_time = clock_time_ms();
	}
}

/**
 * @func    light_publish_st_power_on
 * @brief   None
 * @param   None
 * @retval  None
 */
void light_publish_st_power_on()
{
	foreach(i, LIGHT_CNT){
		publish_status_delay[i].flag = TRUE;
		publish_status_delay[i].delay_time = 5000 + ((rand()% 30000));
		publish_status_delay[i].delay_start_time = clock_time_ms();
	}
}

/**
 * @func    light_transition_proc
 * @brief   None
 * @param
 * @retval
 */
void light_transition_proc()
{
	light_handle_publish_status_delay();
	static u8 transition_delay_flag[LIGHT_CNT] = {FALSE};

    int all_trans_ok = 1;   // include no transition
	foreach_arr(i,light_res_sw){
		foreach_arr(trans_type,light_res_sw[i].trans){
			st_transition_t *p_trans = P_ST_TRANS(i, trans_type);
			if(transition_delay_flag[i] == FALSE){
				if((p_trans->delay_ms != 0)||(p_trans->remain_t_ms)!= 0){
					transition_delay_flag[i] = TRUE;
				}
			}
			int complete_level = 0;
			if(p_trans->delay_ms){
				p_trans->delay_ms--;
				if((0 == p_trans->delay_ms) && (0 == p_trans->remain_t_ms)){
					complete_level = 1;
				}else{
				    all_trans_ok = 0;
				}
			}else{
				if(p_trans->remain_t_ms){
#if MD_LIGHTNESS_EN
					if(p_trans->present != p_trans->target){
						u32 adjust_interval = LIGHT_ADJUST_INTERVAL;
						if(0 == (p_trans->remain_t_ms % adjust_interval)){
							s16 next_val = light_get_next_level(i, trans_type);
							st_pub_list_t pub_list = {{0}};
							light_g_level_set_idx(i, next_val, 0, trans_type, &pub_list);
							light_transition_log(trans_type, p_trans->present);
						}
					}
#endif
#if ENABLE_TRANS_TIME
					p_trans->remain_t_ms--;
#else
					p_trans->remain_t_ms = 0;
#endif
					if(0 == p_trans->remain_t_ms){
						complete_level = 1;	// make sure the last status is same with target
					}else{
				        all_trans_ok = 0;
				    }
				}
			}
			if(complete_level){

				DBG_LIGHT_SEND_STR("\n ####\n LIGHT complete_level");

				st_pub_list_t pub_list = {{0}};
				light_g_level_set_idx(i, p_trans->target, 0, trans_type, &pub_list);
				light_transition_log(trans_type, p_trans->present);

                #if MD_SCENE_EN
				scene_target_complete_check(i);
				#endif

				if(g_addr_dest_arr[i] < ADR_GROUP_START_POINT){
					if(transition_delay_flag[i] == TRUE){
						light_publish_status_delay(i, 0);
					}else{
						light_publish_status_delay(i, DEFAULT_PUBLISH_DELAY_TIME);
					}
				}else{
					light_publish_status_delay(i, 5000 + (rand() % 30000));
				}
				transition_delay_flag[i] = FALSE;
			}
		}
	}
	if(all_trans_ok){
	    if(light_pub_trans_step){
	        light_pub_trans_step = ST_PUB_TRANS_ALL_OK;
	    }
	}
}
#endif

//---------------------------------- LED FUNCTIONS

/**
 * @func    light_state_refresh_all
 * @brief   None
 * @param
 * @retval
 */
void light_state_refresh_all()
{
    foreach(i, LIGHT_CNT){
        light_state_refresh(i);
    }
}

typedef void (*fp_proc_led)(void);
fp_proc_led 				p_vendor_proc_led 				= 0;


static u32 led_event_pending;
static int led_count = 0;

/**
 * @func    cfg_led_event
 * @brief   None
 * @param
 * @retval
 */
void cfg_led_event (u32 e)
{
	led_event_pending = e;
}

/**
 * @func    is_led_busy
 * @brief   None
 * @param
 * @retval
 */
int is_led_busy()
{
    return (!(!led_count && !led_event_pending));
}

/**
 * @func    proc_led
 * @brief   None
 * @param
 * @retval
 */
void proc_led(void)
{
	if(p_vendor_proc_led){
		p_vendor_proc_led();
		return;
	}
	static	u32 led_ton;
	static	u32 led_toff;
	static	int led_sel;						//
	static	u32 led_tick;
	static	int led_no;
	static	int led_is_on;

	if(!led_count && !led_event_pending) {
		return;  //led flash finished
	}

	if (led_event_pending)
	{
		// new event
		led_ton = (led_event_pending & 0xff) * 64000 * CLOCK_SYS_CLOCK_1US;
		led_toff = ((led_event_pending>>8) & 0xff) * 64000 * CLOCK_SYS_CLOCK_1US;
		led_count = (led_event_pending>>16) & 0xff;
		led_sel = led_event_pending>>24;

		led_event_pending = 0;
		led_tick = clock_time () + 30000000 * CLOCK_SYS_CLOCK_1US;
		led_no = 0;
		led_is_on = 0;
	}

	if( 1 ){
		if( (u32)(clock_time() - led_tick) >= (led_is_on ? led_ton : led_toff) ){
			led_tick = clock_time ();
			int led_off = (led_is_on || !led_ton) && led_toff;
			int led_on = !led_is_on && led_ton;

			led_is_on = !led_is_on;
			if (led_is_on)
			{
				led_no++;
				//led_dbg++;
				if (led_no - 1 == led_count)
				{
					led_count = led_no = 0;
					light_state_refresh_all(); // should not report online status again
					return ;
				}
			}
			
			if( led_off || led_on  ){
                if (led_sel & BIT(0))
                {
#if (LIGHT_CNT > 1) && ENABLE_BLINK_SINGLE_LED
                	for(u8 i = 0; i < LIGHT_CNT; i++)
                	{
                		if(((blink_mask >> i)&0x01) == 0x01)
                		{
                		    light_set_hardware(i, led_on);
                		}
                	}
#else
                	light_set_hardware(0, led_on);
#endif
                }
            }
        }
	}
}

/**
 * @func    update_blink_mask
 * @brief   None
 * @param
 * @retval  None
 */
void update_blink_mask(u16 led_mask)
{
	#if (LIGHT_CNT > 1) && ENABLE_BLINK_SINGLE_LED
	blink_mask = led_mask;
	#endif
}

/**
 * @func    rf_link_light_event_callback
 * @brief   None
 * @param
 * @retval
 */
void rf_link_light_event_callback (u8 status)
{
	if(status == LGT_CMD_SET_MESH_INFO){
        // RSV
	}
	else if(status == LGT_CMD_CONFIG_SCENE){
		cfg_led_event(LED_EVENT_FLASH_1HZ_2S);
		push_led_event_to_fifo(LED_CMD_SET_SCENE, 0xFFFF);
	}
	else if(status == LGT_CMD_WAIT_POWER_OFF_ON_RESET){
		cfg_led_event(LED_EVENT_FLASH_1HZ_3S);
	}
	else if(status == LGT_CMD_SUC_ADD_APPKEY){
		cfg_led_event(LED_EVENT_FLASH_1HZ_2S);
		push_led_event_to_fifo(LED_SUC_ADD_APPKEY, 0xFFFF);
    }
	else if(status == LGT_CMD_SET_SUBSCRIPTION){
    	if(vd_config_model_sub_set_flag == 0){
    		if(enable_blink_led_add_sub_flag == TRUE){
    	        cfg_led_event(LED_EVENT_FLASH_1HZ_2S);
    		}
	        push_led_event_to_fifo(LED_CMD_SET_SUBSCRIPTION, 0xFFFF);
    	}
	}
	else if(status == LGT_CMD_WAIT_CONFIRM_RST){
		cfg_led_event(LED_EVENT_FLASH_1HZ_3S);
	}
	// Default blink mask
#if (LIGHT_CNT > 1) && ENABLE_BLINK_SINGLE_LED
	blink_mask = 0xFFFF;
#endif
}

#ifndef WIN32
/**
 * @func    light_on_off_all_led_manual
 * @brief   None
 * @param
 * @retval
 */
void light_on_off_all_led_manual(u8 idx, GPIO_LevelTypeDef level)
{
	if(idx < LIGHT_CNT){
		for(u8 i = 0; i < LIGHT_CNT; i++){
			gpio_write(output_control_io_pin[i], level);
		}
	}
}

/**
 * @func    light_ev_with_sleep
 * @brief   None
 * @param
 * @retval
 */
void light_ev_with_sleep(u32 count, u32 half_cycle_us)
{
	// Blink
	for(u32 i = 0; i < count; i++){
		wd_clear();
		light_on_off_all_led_manual(0, Level_Low);
		sleep_us(half_cycle_us);
        wd_clear();
        light_on_off_all_led_manual(0, Level_High);
		sleep_us(half_cycle_us);
	}
	wd_clear();
	light_on_off_all_led_manual(0, Level_Low);
    sleep_us(half_cycle_us);
}

/**
 * @func    show_ota_result
 * @brief   None
 * @param
 * @retval
 */
void show_ota_result(int result)
{
	if(result == OTA_SUCCESS){
		push_led_event_to_fifo(LED_OTA_SUCESS, 0xFFFF);
		#if ENABLE_BLINK_LED_OTA
		light_ev_with_sleep(3, 1000*1000);	//0.5Hz shine for  6 second
		#endif
	}
	else{
		push_led_event_to_fifo(LED_OTA_FAIL, 0xFFFF);
		#if ENABLE_BLINK_LED_OTA
		light_ev_with_sleep(6, 500*1000);	//5Hz shine for  6 second
		#endif
	}
}

/**
 * @func    show_factory_reset
 * @brief   None
 * @param
 * @retval
 */
void show_factory_reset(){
	light_ev_with_sleep(0, 500*1000);
}
#endif

