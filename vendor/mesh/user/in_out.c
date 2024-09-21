/*
 * in_out.c
 *
 *  Created on: Oct 22, 2020
 *      Author: DungTran BK
 */
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "../../../proj/tl_common.h"
#include "../../common/system_time.h"
#include "../../../proj_lib/ble/blt_config.h"
#include "../../common/generic_model.h"
#include "../../common/light.h"
#include "../../common/lighting_model.h"
#include "led.h"
#include "utilities.h"
#include "config_board.h"
#include "binding.h"
#include "in_out.h"

#include "debug.h"
#ifdef  IN_OUT_DBG_EN
	#define DBG_IN_OUT_SEND_STR(x)   Dbg_sendString((s8*)x)
	#define DBG_IN_OUT_SEND_INT(x)   Dbg_sendInt(x)
	#define DBG_IN_OUT_SEND_HEX(x)   Dbg_sendHex(x)
	#define DBG_IN_OUT_SEND_BYTE(x)  Dbg_sendOneByteHex(x)
#else
	#define DBG_IN_OUT_SEND_STR(x)
	#define DBG_IN_OUT_SEND_INT(x)
	#define DBG_IN_OUT_SEND_HEX(x)
	#define DBG_IN_OUT_SEND_BYTE(x)
#endif

typeIo_initSendConfigParamsCallbackFunc pvIo_handleSendConfigParams = NULL;

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

static io_params_t  io_params_st[NUMBER_INPUT];
static int adr_io_params_idx = 0;

#define FLASH_SIZE_IO_PARAMS       3921
#define BLOCK_SIZE_IO_PARAMS       sizeof(io_params_st)


typedef struct {
     BOOL delay_change_flag;
     u32  delay_change_start_t;
}out_delay_params_t;

static out_delay_params_t  out_delay_params_st[LIGHT_CNT];

static in_out_init_t in_out_init_st = { .start_t_ms = 0,  .expired_ts = FALSE };

/******************************************************************************/
/*                            PRIVATE FUNCTION                                */
/******************************************************************************/

static void io_store_parameters(void);
static BOOL io_mode_is_delay_trans(u8 idx);

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
/**
 * @func   io_callback_init
 * @brief
 * @param  None
 * @retval None
 */
void io_callback_init(typeIo_initSendConfigParamsCallbackFunc callback_Func)
{
	if(callback_Func != NULL){
		pvIo_handleSendConfigParams = callback_Func;
	}
}

/**
 * @func    io_get_config_params
 * @brief
 * @param   Pointer: dst_par
 * @retval  None
 */
void io_get_config_params(u8 *dst_par)
{
   memcpy(dst_par, (u8*)&io_params_st, sizeof(io_params_st));
}

/**
 * @func    io_set_config_params
 * @brief
 * @param   dst
 * @retval  None
 */
BOOL io_set_input_mode(u8 idx, u8 new_val)
{
   if((idx < NUMBER_INPUT)
		   && (new_val < SW_UNKNOWN)){
	   if(io_params_st[idx].in_mode != new_val){
		   io_params_st[idx].in_mode = new_val;
		   // Handle send to GW
		   if(pvIo_handleSendConfigParams != NULL){
			   pvIo_handleSendConfigParams(
						   idx, VD_CONFIG_SW_SET_MAIN_PARAMS, GATEWAY_UNICAST_ADDR, CONFIG_NODE_SET, FAST_REPORT
					   );
		   }
		   io_store_parameters();
		   return TRUE;
	   }
   }
   return FALSE;
}
/**
 * @func    io_blink_config_led
 * @brief
 * @param   None
 * @retval  None
 */
void io_blink_config_led(u16 mask, u8 st)
{
	LedCommand_str ledCmd = COMMAND_LED_DEFAULT;
	ledCmd.blinkInterval = TIMER_200MS;
	ledCmd.blinkTime = 4;
	ledCmd.ledColor = ((st == SUCCESS) ? LED_COLOR_BLUE : LED_COLOR_RED);
	LED_pushLedCommandToFifo(ledCmd);
}

/**
 * @func    io_get_main_params
 * @brief
 * @param   None
 * @retval  None
 */
io_params_t* io_get_main_params(u8 input_channel)
{
	if(input_channel < NUMBER_INPUT){
	    return (io_params_t*)&io_params_st[input_channel];
	}
	return NULL;
}

/**
 * @func    io_store_parameters
 * @brief
 * @param   None
 * @retval  None
 */
static void io_store_parameters(void)
{
	adr_io_params_idx += BLOCK_SIZE_IO_PARAMS;
    if(adr_io_params_idx >= FLASH_SIZE_IO_PARAMS)
    {
    	DBG_IN_OUT_SEND_STR("\n Erase Sector IO parameter");
    	adr_io_params_idx = 0;
    	flash_erase_sector(FLASH_ADR_IO_PARAMS);
    }
	flash_write_page (
			FLASH_ADR_IO_PARAMS + adr_io_params_idx, BLOCK_SIZE_IO_PARAMS, (u8*)&io_params_st
		);
}

/**
 * @func    io_set_def_parameters
 * @brief
 * @param   None
 * @retval  None
 */
static void io_set_def_parameters(BOOL save)
{
	foreach(i, NUMBER_INPUT){
		io_params_st[i].in_mode = IN_ANY_CHANGE_SW;
		io_params_st[i].out_auto_trans_t_s = 1;
		io_params_st[i].out_mode = OUT_NORMAL;
		io_params_st[i].out_map_input = TRUE;
		// Handle send to GW
		if(pvIo_handleSendConfigParams != NULL){
			pvIo_handleSendConfigParams(
					   i, VD_CONFIG_SW_SET_MAIN_PARAMS, GATEWAY_UNICAST_ADDR, CONFIG_NODE_SET, FAST_REPORT
				   );
		}
	}
	if(save == TRUE){
		io_store_parameters();
	}
}

/**
 * @func    io_restore_parameter
 * @brief
 * @param   None
 * @retval  None
 */
static void io_restore_parameters(void)
{
	flash_read_page(
			FLASH_ADR_IO_PARAMS + adr_io_params_idx, BLOCK_SIZE_IO_PARAMS, (u8*)&io_params_st
		);
	if(io_params_st[0].in_mode == MAX_U8){
		DBG_IN_OUT_SEND_STR("\n Write default IO parameter");
		io_set_def_parameters(TRUE);
	}
}

/**
 * @func    io_restore_io_params_idx
 * @brief
 * @param   None
 * @retval  None
 */
static void io_restore_io_params_idx(void)
{
	u16 i = 0;
	u8 temp[BLOCK_SIZE_IO_PARAMS];

	memset(
		temp, MAX_U8, sizeof(io_params_st)
		);
    while(i < FLASH_SIZE_IO_PARAMS){
    	flash_read_page(
    			FLASH_ADR_IO_PARAMS + i, BLOCK_SIZE_IO_PARAMS, (u8*)&temp
    		);
    	if(IsMatchVal(&temp[0], BLOCK_SIZE_IO_PARAMS, MAX_U8))
    		break;
    	i += BLOCK_SIZE_IO_PARAMS;
    }
    if(i == 0){
    	adr_io_params_idx = FLASH_ADR_IO_PARAMS - BLOCK_SIZE_IO_PARAMS;
    }
    else{
    	adr_io_params_idx = i - BLOCK_SIZE_IO_PARAMS;
    }
}


/**
 * @func   io_setup_delay_trans_params
 * @brief
 * @param  None
 * @retval None
 */
void io_setup_delay_trans_params(u8 idx, BOOL en)
{
	if(idx < LIGHT_CNT){
		if(en == TRUE){
			out_delay_params_st[idx].delay_change_flag = TRUE;
			out_delay_params_st[idx].delay_change_start_t = clock_time_ms();
		}
		else {
			out_delay_params_st[idx].delay_change_flag = FALSE;
			DBG_IN_OUT_SEND_STR("\n TURN OFF Delay");
		}
	}
}

/**
 * @func   io_config_sw_main_params
 * @brief
 * @param  None
 * @retval None
 */
int io_config_sw_main_params(u8 idx, u8* par, u8 par_len)
{
    if(par_len != sizeof(vd_sw_config_t)){
    	goto CFG_ERR;
    }
	vd_sw_config_t *cfg = (vd_sw_config_t*)par;
	if((cfg->input >= SW_UNKNOWN)
			|| (cfg->output >= OUT_UNKNOWN)){
		goto CFG_ERR;
	}
	if((cfg->delay_t_s == 0) || (cfg->delay_t_s > MAX_DELAY_TIME_S)){
		goto CFG_ERR;
	}
	io_params_st[idx].in_mode  = cfg->input;
	io_params_st[idx].out_mode = cfg->output;
	io_params_st[idx].out_auto_trans_t_s = cfg->delay_t_s;
	io_blink_config_led(0xFFFF, SUCCESS);
	io_store_parameters();
	return 0;

CFG_ERR:
	io_blink_config_led(0xFFFF, FAILURE);
	return -1;
}
/**
 * @func   io_config_sw_set_map_unmap_input
 * @brief
 * @param  None
 * @retval None
 */
int io_config_sw_set_map_unmap_input(u8 idx, u8* par)
{
	BOOL en = par[0];

	if(en == TRUE){
		io_params_st[idx].out_map_input = TRUE;
	}
	else if(en == FALSE){
		io_params_st[idx].out_map_input = FALSE;
	}
	else {
		io_blink_config_led(0xFFFF, FAILURE);
		return -1;
	}
	io_blink_config_led(0xFFFF, SUCCESS);
	io_store_parameters();
    return 0;
}

/**
 * @func   io_init
 * @brief
 * @param  None
 * @retval None
 */
void io_init(void)
{
	io_restore_io_params_idx();
	io_restore_parameters();
    foreach(i, LIGHT_CNT){
    	out_delay_params_st[i].delay_change_flag = FALSE;
    }
    in_out_init_st.start_t_ms = clock_time_ms();
}

/**
 * @func   io_handle_cmd_sig_on_off
 * @brief
 * @param  None
 * @retval None
 */
int io_handle_cmd_sig_on_off(mesh_cmd_g_onoff_set_t *p_set, int par_len, int force_last, int idx, u8 retransaction, st_pub_list_t *pub_list)
{
	if(idx < LIGHT_CNT)
	{
		if(io_mode_is_delay_trans(idx) == FALSE){
			g_onoff_set(
					p_set, par_len, force_last, idx, retransaction, pub_list
				);
		}
		else {
			if(io_params_st[idx].out_mode == OUT_DELAY_ON){
				if(p_set->onoff == G_ON){
					io_setup_delay_trans_params(idx, TRUE);
				}
				else{
					g_onoff_set(
							p_set, par_len, force_last, idx, retransaction, pub_list
						);
					io_setup_delay_trans_params(idx, FALSE);
				}
			}
			else if(io_params_st[idx].out_mode == OUT_DELAY_OFF){
				if(p_set->onoff == G_OFF){
					io_setup_delay_trans_params(idx, TRUE);
				}
				else{
					g_onoff_set(
							p_set, par_len, force_last, idx, retransaction, pub_list
						);
					io_setup_delay_trans_params(idx, FALSE);
				}
			}
		}
		return 0;
	}
	else {
		DBG_IN_OUT_SEND_STR("\n Can't handle, not support");
		return -1;
	}
}

/**
 * @func   io_loop_task
 * @brief
 * @param  None
 * @retval None
 */
void io_loop_task(void)
{
	foreach(i, LIGHT_CNT){
		if(io_params_st[i].out_mode != OUT_NORMAL){
			mesh_cmd_g_level_st_t level_st;
			light_g_level_get(
					(u8 *)&level_st, i, ST_TRANS_LIGHTNESS
				 );
			BOOL st = get_onoff_from_level(level_st.present_level);
            BOOL change_flag = FALSE;

			if(st == G_ON){
				if(io_params_st[i].out_mode == OUT_AUTO_OFF){
                    if(clock_time_exceed_ms  \
                    		(control_output_start_t_ms[i], io_params_st[i].out_auto_trans_t_s * 1000)){
                    	light_transition_onoff_manual(G_OFF, 0, i);
                    	change_flag = TRUE;
                    }
				}
				else if (io_params_st[i].out_mode == OUT_DELAY_OFF){
                    if(out_delay_params_st[i].delay_change_flag == TRUE){
                        if(clock_time_exceed_ms  \
                        		(out_delay_params_st[i].delay_change_start_t, io_params_st[i].out_auto_trans_t_s * 1000)){
                        	io_setup_delay_trans_params(i, FALSE);    // Stop delay
                        	light_transition_onoff_manual(G_OFF, 0, i);
                        	change_flag = TRUE;
                        }
                    }
				}
			}
			else {
				if(io_params_st[i].out_mode == OUT_AUTO_ON){
                    if(clock_time_exceed_ms  \
                    		(control_output_start_t_ms[i], io_params_st[i].out_auto_trans_t_s * 1000)){
                    	light_transition_onoff_manual(G_ON, 0, i);
                    	change_flag = TRUE;
                    }
				}
				else if (io_params_st[i].out_mode == OUT_DELAY_ON){
                    if(out_delay_params_st[i].delay_change_flag == TRUE){
                        if(clock_time_exceed_ms  \
                        		(out_delay_params_st[i].delay_change_start_t, io_params_st[i].out_auto_trans_t_s * 1000)){
                        	io_setup_delay_trans_params(i, FALSE);    // Stop delay
                        	light_transition_onoff_manual(G_ON, 0, i);
                        	change_flag = TRUE;
                        }
                    }
				}
			}
			if(change_flag == TRUE){

				DBG_IN_OUT_SEND_STR("\n *************\n g_addr_dest_arr: ");
				DBG_IN_OUT_SEND_HEX(g_addr_dest_arr[i]);


				if(g_addr_dest_arr[i] < ADR_GROUP_START_POINT){
					mesh_tx_cmd_lightness_st(
							i,
							ele_adr_primary + i,
							GATEWAY_UNICAST_ADDR,
							LIGHTNESS_STATUS,
							0,
							0
						);
				}
				else {
					u16 binding_addr = get_group_binding_adr(i);
					if((binding_addr != ADR_UNASSIGNED)
							&& (binding_addr == g_addr_dest_arr[i])) {
						light_publish_status_delay(i, TIMER_1S + (rand()%TIMER_3S));
					}
					else {
						light_publish_status_delay(i, 5000 + (rand() % 30000));
					}
				}
			}
		}
	}
}

/**
 * @func   io_mode_is_delay_trans
 * @brief
 * @param  None
 * @retval
 */
static BOOL io_mode_is_delay_trans(u8 idx)
{
	if((io_params_st[idx].out_mode == OUT_DELAY_ON)
						||(io_params_st[idx].out_mode == OUT_DELAY_OFF)){
		return TRUE;
	}
	return FALSE;
}

static u8 io_handle_last_st[NUMBER_INPUT] = {
			ST_UNKNOWN,
	#if NUMBER_INPUT > 1
			ST_UNKNOWN,
	#endif
	#if NUMBER_INPUT > 2
			ST_UNKNOWN,
	#endif
	#if NUMBER_INPUT > 3
			ST_UNKNOWN,
	#endif
};

/**
 * @func   io_handle_input_btn_st
 * @brief
 * @param  None
 * @retval None
 */
BOOL io_handle_input_btn_st(u8 idx, u8 evt, BOOL no_change)
{
	if(io_params_st[idx].out_map_input == TRUE)
	{
		// This block can be omitted because the input was checked in the input.c file
		if(in_out_init_st.expired_ts == FALSE){
            if(clock_time_exceed_ms(in_out_init_st.start_t_ms, TIMER_2S)){
            	in_out_init_st.expired_ts = TRUE;
            }
            else{
            	return SUCCESS;
            }
		}
		// Check last st
		if(io_handle_last_st[idx] == evt){
			return FAILURE;
		}
		io_handle_last_st[idx] = evt;

		if(no_change == TRUE){
			return SUCCESS;
		}
		mesh_cmd_g_level_st_t level_st;
		light_g_level_get(
				(u8 *)&level_st, idx, ST_TRANS_LIGHTNESS
			 );
		BOOL st = get_onoff_from_level(level_st.present_level);
		u8 st_control = st;

    	if(evt == START_PRESS){
    		switch(io_params_st[idx].in_mode){
				case IN_TOGGLE_SW:{
					if(io_params_st[idx].out_mode == OUT_DELAY_ON){
						io_setup_delay_trans_params(idx, TRUE);
					}
					else if(io_params_st[idx].out_mode == OUT_DELAY_OFF){
						st_control = G_ON;
						io_setup_delay_trans_params(idx, FALSE);
					}
					else{
					    st_control = G_ON;
                    }
					break;
				}
				case IN_MOMENTARY_SW:{
                    if(io_params_st[idx].out_mode == OUT_DELAY_ON){
                        if(st == G_OFF){
                        	io_setup_delay_trans_params(idx, TRUE);
                        }
                        else{
                        	 st_control = G_OFF;
                        	 io_setup_delay_trans_params(idx, FALSE);
                        }
                    } else if(io_params_st[idx].out_mode == OUT_DELAY_OFF){
                        if(st == G_ON){
                        	io_setup_delay_trans_params(idx, TRUE);
                        }
                        else{
                        	st_control = G_ON;
                        	io_setup_delay_trans_params(idx, FALSE);
                        }
                    } else{
					    st_control = (st == G_ON) ? G_OFF : G_ON;
                    }
					break;
				}
				case IN_ANY_CHANGE_SW:{
					if(io_params_st[idx].out_mode == OUT_DELAY_ON){
						if(st == G_OFF){
							io_setup_delay_trans_params(idx, TRUE);
						}
						else{
							 st_control = G_OFF;
							 io_setup_delay_trans_params(idx, FALSE);
						}
					} else if(io_params_st[idx].out_mode == OUT_DELAY_OFF){
						if(st == G_ON){
							io_setup_delay_trans_params(idx, TRUE);
						}
						else{
							st_control = G_ON;
							io_setup_delay_trans_params(idx, FALSE);
						}
					} else {
					    st_control = (st == G_ON) ? G_OFF : G_ON;
                    }
					break;
				}
			}
    		if(st_control != st){
    			light_transition_onoff_manual(st_control, 0, idx);
    			DBG_IN_OUT_SEND_STR("\n Control");
    			return SUCCESS;
    		}
		}
		else if(evt == RELEASE){
			switch(io_params_st[idx].in_mode){
				case IN_TOGGLE_SW:{
					if(io_params_st[idx].out_mode == OUT_DELAY_OFF){
						io_setup_delay_trans_params(idx, TRUE);
					}
					else if(io_params_st[idx].out_mode == OUT_DELAY_ON){
						st_control = G_OFF;
						io_setup_delay_trans_params(idx, FALSE);
					}
					else{
						st_control = G_OFF;
					}
					break;
				}
				case IN_MOMENTARY_SW:{

					break;
				}
				case IN_ANY_CHANGE_SW:{
					if(io_params_st[idx].out_mode == OUT_DELAY_ON){
						if(st == G_OFF){
							io_setup_delay_trans_params(idx, TRUE);
						}
						else{
							 st_control = G_OFF;
							 io_setup_delay_trans_params(idx, FALSE);
						}
					} else if(io_params_st[idx].out_mode == OUT_DELAY_OFF){
						if(st == G_ON){
							io_setup_delay_trans_params(idx, TRUE);
						}
						else{
							st_control = G_ON;
							io_setup_delay_trans_params(idx, FALSE);
						}
					} else{
					    st_control = (st == G_ON) ? G_OFF : G_ON;
					}
					break;
				}
			}
			if(st_control != st){
			    light_transition_onoff_manual(st_control, 0, idx);
			    return SUCCESS;
			}
		}
    }
    return FAILURE;
}

// End File
