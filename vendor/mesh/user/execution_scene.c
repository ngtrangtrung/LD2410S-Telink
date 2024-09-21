/*
 * execution_scene.c
 *
 *  Created on: Apr 15, 2020
 *      Author: DungTran BK
 */

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "../../common/vendor_model.h"
#include "./debug.h"
#include "key_report.h"
#include "../../common/lighting_model_HSL.h"
#include "../../common/lighting_model.h"
#include "../../common/system_time.h"
#include "../../mesh/user/led.h"
#include "utilities.h"
#include "execution_scene.h"

#ifdef EXECUTION_SCENE_DBG_EN
	#define DBG_EXECUTION_SCENE_SEND_STR(x)          Dbg_sendString((s8*)x)
	#define DBG_EXECUTION_SCENE_SEND_INT(x)          Dbg_sendInt(x)
	#define DBG_EXECUTION_SCENE_SEND_HEX(x)          Dbg_sendHex(x)
    #define DBG_EXECUTION_SCENE_SEND_BYTE(x) Dbg_sendHexOneByte(x)
#else
	#define DBG_EXECUTION_SCENE_SEND_STR(x)
	#define DBG_EXECUTION_SCENE_SEND_INT(x)
	#define DBG_EXECUTION_SCENE_SEND_HEX(x)
    #define DBG_EXECUTION_SCENE_SEND_BYTE(x)
#endif

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

static par_execution_scene_t par_execution_scene[NUMBER_INPUT][EV_BTN_MAX];
static setup_execution_scene_set_t setup_exe_scene_set;

const u16 op_sig_support_scene[] = {
	// HSL
	LIGHT_HSL_SET,
	LIGHT_HSL_SET_NOACK,
	// CTL
	LIGHT_CTL_SET,
	LIGHT_CTL_SET_NOACK,
	LIGHT_CTL_TEMP_SET,
	LIGHT_CTL_TEMP_SET_NOACK,
	LIGHTNESS_SET,
	LIGHTNESS_SET_NOACK,
	// Generic ON/OFF
	G_ONOFF_SET,
	G_ONOFF_SET_NOACK,
	// Generic Level
	G_LEVEL_SET,
	G_LEVEL_SET_NOACK
};

const u16 op_vd_support_scene[] = {
	// Auto Transition RGB
	VD_AUTO_TRANS_SET,
	VD_AUTO_TRANS_SET_NOACK
};

#define PAR_SCENE		 par_execution_scene[key_number][key_event]

/******************************************************************************/
/*                        PRIVATE FUNCTIONS DECLERATION                       */
/******************************************************************************/


/******************************************************************************/
/*                        EXPORT FUNCTIONS DECLERATION                        */
/******************************************************************************/
/**
 * @func    restore_execution_scene
 * @brief
 * @param   None
 * @retval  None
 */
static void restore_execution_scene(void){

	flash_read_page(
			FLASH_ADR_EXECUTION_SCENE, sizeof(par_execution_scene), (u8 *)(&par_execution_scene)
			);
}

/**
 * @func    save_execution_scene_to_flash
 * @brief
 * @param   None
 * @retval  None
 */
static void save_execution_scene_to_flash(void){

	flash_erase_sector(FLASH_ADR_EXECUTION_SCENE);
	flash_write_page (
			FLASH_ADR_EXECUTION_SCENE, sizeof(par_execution_scene), (u8 *)(&par_execution_scene)
		);
}

/**
 * @func    delete_all_execution_scene
 * @brief
 * @param   None
 * @retval  None
 */
static void delete_all_execution_scene(void){

	memset(
		&par_execution_scene, 0xFF, sizeof(par_execution_scene)
	    );
	// Save To Flash
	save_execution_scene_to_flash();
}

/**
 * @func    delete_execution_scene
 * @brief
 * @param   None
 * @retval  None
 */
static void delete_execution_scene(u8 key_number, u8 key_ev){

	memset(
		&par_execution_scene[key_number][key_ev], 0xFF, sizeof(par_execution_scene_t)
		);
	save_execution_scene_to_flash();
}

/**
 * @func    check_valid_event_and_key_number
 * @brief
 * @param   None
 * @retval  None
 */
static int check_valid_event_and_key_number(u8 key_number, u8 key_ev){

	if((key_number >= NUMBER_INPUT)|| (key_ev >= EV_BTN_MAX)){
		DBG_EXECUTION_SCENE_SEND_STR("\n key number or key_ev invalid");
		return -1;
	}
	DBG_EXECUTION_SCENE_SEND_STR("\n key number or key_ev valid");
	return 0;
}

/**
 * @func    execution_scene_blink_led_status
 * @brief
 * @param   None
 * @retval  None
 */
static void execution_scene_blink_led_status(u16 mask, LedColor_enum color)
{
    LedCommand_str ledCmd = COMMAND_LED_DEFAULT;
	ledCmd.blinkInterval = TIMER_200MS;
	ledCmd.ledMask = mask;
	ledCmd.ledColor = color;
	LED_pushLedCommandToFifo(ledCmd);
}
/**
 * @func   execution_scene_handle_vendor_setup_execution_scene_set
 * @brief  None
 * @param
 * @retval Status code
 */
static int handle_vendor_setup_execution_scene_set(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par){

    int err = 0;
    u8 index = 0;

    // Copy from Message Type And Message Id To Number Of Destination Address
    memcpy(
    	&setup_exe_scene_set.msg_type_and_id,
    	&par[index],
    	5);
    index += 4;

    u8 key_number = setup_exe_scene_set.setup_event_st.key_nbr;
	u8 key_ev  = setup_exe_scene_set.setup_event_st.key_event;

	switch(setup_exe_scene_set.msg_type_and_id_st.msg_type){
		case MSG_ADD:
		{
			DBG_EXECUTION_SCENE_SEND_STR("\n MSG_ADD");

			if(check_valid_event_and_key_number(key_number, key_ev) == 0){
				if(setup_exe_scene_set.scene_id == SCENE_ID_INVALID){
					DBG_EXECUTION_SCENE_SEND_STR("\n SCENE_ID_INVALID");
					if(setup_exe_scene_set.nums_dest > MAX_SCENE_DEST_ADR){
						// Message Invalid
						err = -1;
					}else{
						// Delete, Not Save To Flash
						memset(
							&par_execution_scene[key_number][key_ev], 0xFF, sizeof(par_execution_scene_t)
							);
						par_execution_scene[key_number][key_ev].btn_event = key_ev;
						par_execution_scene[key_number][key_ev].scene_id = setup_exe_scene_set.scene_id;

						u8 payload_len;

						for(u8 i = 0; i < setup_exe_scene_set.nums_dest; i++){
							par_execution_scene[key_number][key_ev].control_infor[i].dest_addr = par[index+1] | (u16)(par[index+2] << 8);
							index += 2;

							DBG_EXECUTION_SCENE_SEND_STR("\n --- Dest Addr [ ");
							DBG_EXECUTION_SCENE_SEND_INT(i);
							DBG_EXECUTION_SCENE_SEND_STR("] = ");
							DBG_EXECUTION_SCENE_SEND_HEX(par_execution_scene[key_number][key_ev].control_infor[i].dest_addr);

							payload_len = par[++index];
							if(payload_len > MAX_PAYLOAD_LEN)  payload_len = MAX_PAYLOAD_LEN;

							memcpy(
								&par_execution_scene[key_number][key_ev].control_infor[i].payload,
								&par[++index],
								payload_len
								);
							par_execution_scene[key_number][key_ev].control_infor[i].payload_len = payload_len;
							index += (payload_len - 1);

							DBG_EXECUTION_SCENE_SEND_STR("\n Payload Len: ");
							DBG_EXECUTION_SCENE_SEND_INT(payload_len);
	#ifdef EXECUTION_SCENE_DBG_EN
							DBG_EXECUTION_SCENE_SEND_STR("\n Payload: ");
							for(u8 j = 0; j < payload_len; j++){
								DBG_EXECUTION_SCENE_SEND_BYTE(
										par_execution_scene[key_number][key_ev].control_infor[i].payload[j]
										);
								DBG_EXECUTION_SCENE_SEND_STR(" ");
							}
	#endif
						}
					}
					par_execution_scene[key_number][key_ev].trans_time = par[++index];
					DBG_EXECUTION_SCENE_SEND_STR("\n trans time: ");
					DBG_EXECUTION_SCENE_SEND_BYTE(par_execution_scene[key_number][key_ev].trans_time);
				}
				else{
					// SINGLE SCENE
					if(setup_exe_scene_set.scene_id != 0xFFFF){
						DBG_EXECUTION_SCENE_SEND_STR("\n SCENE_ID_VALID");
						// Delete
						delete_execution_scene(key_number, key_ev);
						par_execution_scene[key_number][key_ev].btn_event = key_ev;
						par_execution_scene[key_number][key_ev].scene_id = setup_exe_scene_set.scene_id;
						par_execution_scene[key_number][key_ev].trans_time = par[5];
					}
					 // MULTIPLE SCENE
					else {
						if((setup_exe_scene_set.nums_scene != 0)
								&& (setup_exe_scene_set.nums_scene <= MAX_SCENE_DEST_ADR)){
							DBG_EXECUTION_SCENE_SEND_STR("\n MULTI SCENE");
							delete_execution_scene(key_number, key_ev);

							par_execution_scene[key_number][key_ev].btn_event = key_ev;
							par_execution_scene[key_number][key_ev].scene_id = setup_exe_scene_set.scene_id;

							for(u8 i = 0; i < setup_exe_scene_set.nums_scene; i++){
								par_execution_scene[key_number][key_ev].scene_control[i].in_scene_id = par[index + 1] | (u16)(par[index + 2] << 8);
								index += 2;
								par_execution_scene[key_number][key_ev].scene_control[i].in_trans_t = par[++index];
								par_execution_scene[key_number][key_ev].scene_control[i].in_delay_t = par[++index];
							}
						}
						par_execution_scene[key_number][key_ev].trans_time = par[++index];
					}
				}
				// Flash
				save_execution_scene_to_flash();
			}else{
				err = -1;
			}
			// Response Add
			execution_scene_add_rsp_str exe_scene_add_rsp;
			exe_scene_add_rsp.msg_type_and_id_st.msg_id = setup_exe_scene_set.msg_type_and_id_st.msg_id;
			exe_scene_add_rsp.msg_type_and_id_st.msg_type = MSG_ADD_RSP;
			exe_scene_add_rsp.status_code = ((err == 0)?(EX_STATUS_SUCCESS):(EX_STATUS_FAIL));
			exe_scene_add_rsp.event = setup_exe_scene_set.event;
			exe_scene_add_rsp.scene_id = setup_exe_scene_set.scene_id;

			err = mesh_tx_cmd_rsp(
					 VD_SETUP_EXECUTION_SCENE_STATUS,
					 (u8 *)&exe_scene_add_rsp,
					 sizeof(execution_scene_add_rsp_str),
					 ele_adr_primary,
					 GATEWAY_UNICAST_ADDR,
					 0,
					 0);

			u8 color_st = LED_COLOR_RED;
			if(exe_scene_add_rsp.status_code == EX_STATUS_SUCCESS){
				color_st = LED_COLOR_BLUE;
			}
			execution_scene_blink_led_status(0xFFFF, color_st);
			break;
		}

		case MSG_DELETE:
		{
			DBG_EXECUTION_SCENE_SEND_STR("\n MSG_DELETE");

			execution_scene_del_rsp_str exe_scene_del_rsp;
			exe_scene_del_rsp.event = setup_exe_scene_set.event;

			if(check_valid_event_and_key_number(key_number, key_ev) == 0){
				delete_execution_scene(key_number, key_ev);
			}else{
				// Message Invalid
				err = -1;
			}
			// Response Delete
			exe_scene_del_rsp.msg_type_and_id_st.msg_id = setup_exe_scene_set.msg_type_and_id_st.msg_id;
			exe_scene_del_rsp.msg_type_and_id_st.msg_type = MSG_DELETE_RSP;
			exe_scene_del_rsp.status_code = ((err == 0)?(EX_STATUS_SUCCESS):(EX_STATUS_FAIL));

			err = mesh_tx_cmd_rsp(
					 VD_SETUP_EXECUTION_SCENE_STATUS,
					 (u8 *)&exe_scene_del_rsp,
					 sizeof(execution_scene_del_rsp_str),
					 ele_adr_primary,
					 GATEWAY_UNICAST_ADDR,
					 0,
					 0);
			u8 color_st = LED_COLOR_RED;
			if(exe_scene_del_rsp.status_code == EX_STATUS_SUCCESS){
				color_st = LED_COLOR_BLUE;
			}
			execution_scene_blink_led_status(0xFFFF, color_st);
			break;
		}

		case MSG_GET:
		{
			DBG_EXECUTION_SCENE_SEND_STR("\n MSG_GET");

			u8 rsp[84];
			u8 len = 0;
			u8 idx = 0;
			header_execution_scene_get_rsp_t  header_get_rsp_st;

			header_get_rsp_st.msg_type_and_id_st.msg_id = setup_exe_scene_set.msg_type_and_id_st.msg_id;
			header_get_rsp_st.msg_type_and_id_st.msg_type = MSG_GET_RSP;
			header_get_rsp_st.event = setup_exe_scene_set.event;

			if(check_valid_event_and_key_number(key_number, key_ev) != 0){
				// Event Not Configuration Before, return fail
				DBG_EXECUTION_SCENE_SEND_STR("\n EX_STATUS_FAIL, Event Invalid ");
				header_get_rsp_st.status_code = EX_STATUS_FAIL;
				len = 3;
				memcpy(&rsp, &header_get_rsp_st, len);
			}else{
				if(par_execution_scene[key_number][key_ev].btn_event == 0xFF){
					// Event Not Configuration Before, return fail
					DBG_EXECUTION_SCENE_SEND_STR("\n EX_STATUS_NOT_SET_BEFORE ");
					header_get_rsp_st.status_code = EX_STATUS_NOT_SET_BEFORE;
					len = 3;
					memcpy(&rsp, &header_get_rsp_st,len);
				}
				else{
					/*
					 * Response payload:
					 * - Message type and message id: 1 byte
					 * - Status Code: 1 byte
					 * - Event: 1 byte
					 * - Scene ID: 2 byte
					 * - Number of destination address: 1 byte
					 * - Control Information: variable
					 * - Transition Time: 1 byte
					 * - Additional Condition: variable
					 */
					DBG_EXECUTION_SCENE_SEND_STR("\n EX_STATUS_SUCCESS ");
					header_get_rsp_st.status_code = EX_STATUS_SUCCESS;
					header_get_rsp_st.scene_id = par_execution_scene[key_number][key_ev].scene_id;

					// SINGLE SCENE OR CONTROL DIRECTLY

					if(header_get_rsp_st.scene_id != 0xFFFF){
						header_get_rsp_st.nbr_of_dest = 0;

						foreach(i, MAX_SCENE_DEST_ADR){
							if(par_execution_scene[key_number][key_ev].control_infor[i].payload_len > MAX_PAYLOAD_LEN){
								continue;
							}
							header_get_rsp_st.nbr_of_dest++;
						}
						idx = sizeof(header_execution_scene_get_rsp_t);
						len = idx;
						memcpy(&rsp, &header_get_rsp_st, idx);

						foreach(i, header_get_rsp_st.nbr_of_dest){
							// Destination Address
							rsp[idx++] = (u8)par_execution_scene[key_number][key_ev].control_infor[i].dest_addr;
							rsp[idx++] = (u8)(par_execution_scene[key_number][key_ev].control_infor[i].dest_addr >> 8);
							// payload len
							rsp[idx++] = par_execution_scene[key_number][key_ev].control_infor[i].payload_len;
							len += 3;
							// payload
							foreach(j, par_execution_scene[key_number][key_ev].control_infor[i].payload_len){
								u8 *tmp = (u8*)&par_execution_scene[key_number][key_ev].control_infor[i].payload;
								rsp[idx++] = *(tmp + j);
								len++;
							}
						}
					}

					// MULTI SCENE

					else {
						header_get_rsp_st.nbr_of_scene = 0;
						foreach(i, MAX_SCENE_DEST_ADR){
							if(par_execution_scene[key_number][key_ev].scene_control[i].in_scene_id == LM_SCENE_IN_INVALID){
								continue;
							}
							header_get_rsp_st.nbr_of_scene++;
						}
						idx = sizeof(header_execution_scene_get_rsp_t);
						len = idx;
						memcpy(&rsp, &header_get_rsp_st, idx);

						foreach(i, header_get_rsp_st.nbr_of_scene){
							// Scene ID
							rsp[idx++] = (u8)par_execution_scene[key_number][key_ev].scene_control[i].in_scene_id;
							rsp[idx++] = (u8)(par_execution_scene[key_number][key_ev].scene_control[i].in_scene_id >> 8);
							rsp[idx++] = par_execution_scene[key_number][key_ev].scene_control[i].in_trans_t;
							rsp[idx++] = par_execution_scene[key_number][key_ev].scene_control[i].in_delay_t;
							len += 4;
						}
					}
					rsp[len++] = par_execution_scene[key_number][key_ev].trans_time;
				}
			}

			err = mesh_tx_cmd_rsp(
					 VD_SETUP_EXECUTION_SCENE_STATUS,
					 (u8 *)&rsp,
					 len,
					 ele_adr_primary,
					 GATEWAY_UNICAST_ADDR,
					 0,
					 0);
			break;
		}

		case MSG_DELETE_ALL:
		{
			DBG_EXECUTION_SCENE_SEND_STR("\n MSG_DELETE_ALL");

			delete_all_execution_scene();
			// Response Delete All
			execution_scene_del_all_rsp_str exe_scene_del_all_rsp;
			exe_scene_del_all_rsp.msg_type_and_id_st.msg_id = setup_exe_scene_set.msg_type_and_id_st.msg_id;
			exe_scene_del_all_rsp.msg_type_and_id_st.msg_type = MSG_DELETE_ALL_RSP;
			exe_scene_del_all_rsp.status_code = EX_STATUS_SUCCESS;

			err = mesh_tx_cmd_rsp(
					 VD_SETUP_EXECUTION_SCENE_STATUS,
					 (u8 *)&exe_scene_del_all_rsp,
					 sizeof(execution_scene_del_all_rsp_str),
					 ele_adr_primary,
					 GATEWAY_UNICAST_ADDR,
					 0,
					 0);
			execution_scene_blink_led_status(0xFFFF, LED_COLOR_BLUE);
			break;
		}
	}
    return err;
}

/**
 * @func   execution_scene_init
 * @brief  None
 * @param
 * @retval Status code
 */
void execution_scene_init(void){

    vendor_handle_func_callback_init(
			handle_vendor_setup_execution_scene_set);
    restore_execution_scene();

	DBG_EXECUTION_SCENE_SEND_STR("\n execution_scene_init");
}


/**
 * @func   execution_scene_active
 * @brief  None
 * @param
 * @retval Status code
 */
void execution_scene_active(u8 key_number, u8 key_ev){

	static uint8_t tid, vd_tid;

	if((key_number < NUMBER_INPUT) && (key_ev < EV_BTN_MAX)){
		if(par_execution_scene[key_number][key_ev].btn_event == key_ev){
			if(par_execution_scene[key_number][key_ev].scene_id != SCENE_ID_INVALID){
				if(par_execution_scene[key_number][key_ev].scene_id != 0xFFFF){
					// Control Directly By Scene ID
					vendor_scene_recall_t vendor_scene_recall;
					vendor_scene_recall.msg_type = VENDOR_SCENE_RECALL_NOACK;
					vendor_scene_recall.scene_id = par_execution_scene[key_number][key_ev].scene_id;
					vendor_scene_recall.tid = ++vd_tid;
					vendor_scene_recall.trans_time = par_execution_scene[key_number][key_ev].trans_time;
#if CTL_WITH_FLAG_DEFAULT_POSITION_EN
					vendor_scene_recall.delay_time = 1;
#else
					vendor_scene_recall.delay_time = 0;
					#endif
					DBG_EXECUTION_SCENE_SEND_STR("\n >> Send Scene Recall");

					mesh_tx_cmd_rsp(
							VD_SCENE_REQUEST_NOACK,(u8 *)&vendor_scene_recall, sizeof(vendor_scene_recall_t), ele_adr_primary, 0xFFFF, 0, 0
							);
				}
				else {
					foreach(j, MAX_SCENE_DEST_ADR)
					{
						DBG_EXECUTION_SCENE_SEND_STR("\n RETRY");

						if(par_execution_scene[key_number][key_ev].scene_control[j].in_scene_id != LM_SCENE_IN_INVALID){
							// Control Directly By Scene ID
							vendor_scene_recall_t vendor_scene_recall;
							vendor_scene_recall.msg_type = VENDOR_SCENE_RECALL_NOACK;
							vendor_scene_recall.scene_id = par_execution_scene[key_number][key_ev].scene_control[j].in_scene_id;
							vendor_scene_recall.tid = ++vd_tid;
							vendor_scene_recall.trans_time = par_execution_scene[key_number][key_ev].scene_control[j].in_trans_t;
#if CTL_WITH_FLAG_DEFAULT_POSITION_EN
							vendor_scene_recall.delay_time = 1;
#else
							vendor_scene_recall.delay_time = par_execution_scene[key_number][key_ev].scene_control[j].in_delay_t;
#endif

							DBG_EXECUTION_SCENE_SEND_STR("\n >> Recall \n");
							DBG_EXECUTION_SCENE_SEND_HEX(par_execution_scene[key_number][key_ev].scene_control[j].in_scene_id);

#ifdef EXECUTION_SCENE_DBG_EN
							DBG_EXECUTION_SCENE_SEND_STR("\n");
							u8 *temp = (u8*)&vendor_scene_recall.msg_type;
							foreach(i, sizeof(vendor_scene_recall_t)){
								DBG_EXECUTION_SCENE_SEND_BYTE(*(temp + i));
								DBG_EXECUTION_SCENE_SEND_STR(" ");
							}
#endif
							// -->
							mesh_tx_cmd_rsp(
										VD_SCENE_REQUEST_NOACK,
										(u8 *)&vendor_scene_recall,
										sizeof(vendor_scene_recall_t),
										ele_adr_primary,
										0xFFFF,
										0,
										0
									);
						}
					}
				}
			}else{
				// Control Directly By Opcode
				for(u8 i = 0; i < MAX_SCENE_DEST_ADR; i++){
					if(par_execution_scene[key_number][key_ev].control_infor[i].dest_addr != ADR_UNASSIGNED){
						// Check Payload Len Valid Or Invalid
						if(par_execution_scene[key_number][key_ev].control_infor[i].payload_len > MAX_PAYLOAD_LEN){
							DBG_EXECUTION_SCENE_SEND_STR("\n PAR LEN INVALID");
							continue;
						}
						// Destination Address
						DBG_EXECUTION_SCENE_SEND_STR("\n Dest Adr: ");
						DBG_EXECUTION_SCENE_SEND_HEX(par_execution_scene[key_number][key_ev].control_infor[i].dest_addr);

						foreach_arr(j, op_sig_support_scene){
							u16 op_sig = (par_execution_scene[key_number][key_ev].control_infor[i].payload[1] << 8) |     \
									par_execution_scene[key_number][key_ev].control_infor[i].payload[0];
							if(op_sig == op_sig_support_scene[j]){
								// Sig Opcode
								DBG_EXECUTION_SCENE_SEND_STR("\n Sig Opcode: ");
								DBG_EXECUTION_SCENE_SEND_HEX(op_sig);

								u8 par_control[MAX_PAYLOAD_LEN];
								u8 par_len = par_execution_scene[key_number][key_ev].control_infor[i].payload_len - 2;

								// Main Par
								memcpy(
									&par_control,
									&par_execution_scene[key_number][key_ev].control_infor[i].payload[2],
									par_len
								   );
								// Tid + Transition Time + Delay Time
								tid++;
								par_control[par_len++] = tid;  // Transition Identify
								par_control[par_len++] = par_execution_scene[key_number][key_ev].trans_time; // Transition Time (step 100 ms)
#if CTL_WITH_FLAG_DEFAULT_POSITION_EN
								par_control[par_len++] = 1;
#else
								par_control[par_len++] = 0;   // Delay Time
#endif
								// Control
								mesh_tx_cmd_rsp(
										op_sig,
										(u8 *)&par_control,
										par_len,
										ele_adr_primary,
										par_execution_scene[key_number][key_ev].control_infor[i].dest_addr,
										0,
										0
									);
							}
						}
						// Vendor Opcode
						u8 op_vd = par_execution_scene[key_number][key_ev].control_infor[i].payload[0];

						if(op_vd >= START_VENDOR_OPCODE){
							DBG_EXECUTION_SCENE_SEND_STR("\n Vendor Opcode: ");
							DBG_EXECUTION_SCENE_SEND_HEX(op_vd);
							// Control
							mesh_tx_cmd_rsp(
									op_vd,
									(u8 *)&par_execution_scene[key_number][key_ev].control_infor[i].payload[3],
									par_execution_scene[key_number][key_ev].control_infor[i].payload_len - 3,
									ele_adr_primary,
									par_execution_scene[key_number][key_ev].control_infor[i].dest_addr,
									0,
									0
								);
						}
					}
				}
			}
		}
	}
}


typedef struct {
	u32     send_last_t_ms;
	BOOL    en_send;
	u8      btn_key;
	u8      send_ev_idx;
}send_exe_scene_delay_t;

const u8 auto_send_arr[] =   \
		{ EV_BTN_PRESS_1, EV_BTN_PRESS_2, EV_BTN_HOLD_2_SECONDS };

static send_exe_scene_delay_t  send_exe_scene_delay_st = {
		.send_last_t_ms = 0,
		.en_send 	    = FALSE,
		.btn_key	    = NUMBER_INPUT,
		.send_ev_idx  	= 0xFF
};

typedef struct {
	union {
		u8 msg_type_and_id;
		msg_type_and_id_t msg_type_and_id_st;
	 };
	u8  event;

	union {
		u8  event;
		setup_event_t evt_st;
    };

}msg_get_execution_scene_t;

/**
 * @func   execution_set_up_auto_send
 * @brief  None
 * @param
 * @retval None
 */
void execution_set_up_auto_send(void)
{
	send_exe_scene_delay_st.send_last_t_ms = clock_time_ms() - TIMER_2S;
	send_exe_scene_delay_st.en_send = TRUE;
	send_exe_scene_delay_st.btn_key = 0;
	send_exe_scene_delay_st.send_ev_idx = 0;
}

/**
 * @func   execution_loop_task
 * @brief  None
 * @param
 * @retval None
 */
void execution_loop_task(void)
{
	if(send_exe_scene_delay_st.en_send == TRUE){
        if(clock_time_exceed_ms(send_exe_scene_delay_st.send_last_t_ms, TIMER_2S)){
        	send_exe_scene_delay_st.send_last_t_ms = clock_time_ms();
        	if((send_exe_scene_delay_st.btn_key < NUMBER_INPUT)
        			&& (send_exe_scene_delay_st.send_ev_idx < sizeof(auto_send_arr))){

        		msg_get_execution_scene_t  exe_scene_get_st;
        		exe_scene_get_st.msg_type_and_id_st.msg_type = MSG_GET;
        		exe_scene_get_st.msg_type_and_id_st.msg_id = 0;
        		exe_scene_get_st.event =
        				((send_exe_scene_delay_st.btn_key << 4)&0xF0)|(auto_send_arr[send_exe_scene_delay_st.send_ev_idx]&0x0F);

        		handle_vendor_setup_execution_scene_set(
									(u8*)&exe_scene_get_st,
									sizeof(msg_get_execution_scene_t),
									NULL
        						);
        		send_exe_scene_delay_st.send_ev_idx++;
        		if(send_exe_scene_delay_st.send_ev_idx >= sizeof(auto_send_arr)){
        			send_exe_scene_delay_st.send_ev_idx = 0;
        			send_exe_scene_delay_st.btn_key++;
        		}
        	}
        	else{
        		send_exe_scene_delay_st.en_send = FALSE;
        		DBG_EXECUTION_SCENE_SEND_STR("\nExit auto send");
        	}
        }
	}
}

// End File
