/********************************************************************************************************
 * @file     app.c 
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
#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"
#include "../../proj_lib/ble/ll/ll.h"
#include "../../proj_lib/ble/blt_config.h"
#include "../../proj_lib/ble/ll/ll_whitelist.h"
#include "../../proj_lib/ble/trace.h"
#include "../../proj_lib/ble/ble_common.h"
#include "../../proj/mcu/pwm.h"
#include "../../proj_lib/ble/service/ble_ll_ota.h"
#include "../../proj/drivers/adc.h"
#include "../../proj_lib/ble/blt_config.h"
#include "../../proj_lib/ble/ble_smp.h"
#include "../../proj_lib/mesh_crypto/mesh_crypto.h"
#include "../../proj_lib/mesh_crypto/mesh_md5.h"
#include "../../proj_lib/mesh_crypto/sha256_telink.h"
#include "../../proj_lib/sig_mesh/app_mesh.h"
#include "../common/app_provison.h"
#include "../common/app_beacon.h"
#include "../common/app_proxy.h"
#include "../common/app_health.h"
#include "../common/vendor_model.h"
#include "../../proj/drivers/keyboard.h"
#include "../../stack/ble/gap/gap.h"
#include "../../proj_lib/ble/l2cap.h"
#include "vendor/common/blt_soft_timer.h"
#include "../common/generic_model.h"
#include "vendor/mesh/user/radar/radar.h"

#include "user/factory_reset.h"
#include "user/utilities.h"
#include "user/led.h"
#include "user/button.h"
#include "user/input.h"
#include "user/handle_ev.h"
#include "user/execution_scene.h"
#include "user/relay.h"
#include "user/in_out.h"
#include "user/default_network.h"
#include "user/binding.h"
#include "user/binding_ctl.h"
#include "user/sw_config.h"
#include "user/bind.h"
#include "app.h"

#include "user/debug.h"
#ifdef APP_DBG_EN
#define DBG_APP_SEND_STR(x)   Dbg_sendString((s8*)x)
#define DBG_APP_SEND_INT(x)   Dbg_sendInt(x)
#define DBG_APP_SEND_HEX(x)   Dbg_sendHex(x)
#define DBG_APP_SEND_BYTE(x)  Dbg_sendOneByteHex(x)
#else
#define DBG_APP_SEND_STR(x)
#define DBG_APP_SEND_INT(x)
#define DBG_APP_SEND_HEX(x)
#define DBG_APP_SEND_BYTE(x)
#endif

#if MI_API_ENABLE
#include "../../vendor/common/mi_api/telink_sdk_mible_api.h"
#include "../../vendor/common/mi_api/certi/mijia_profiles/mi_service_server.h"
#endif 
#if (HCI_ACCESS==HCI_USE_UART)
#include "../../proj/drivers/uart.h"
#endif

MYFIFO_INIT(blt_rxfifo, 64, 16);
MYFIFO_INIT(blt_txfifo, 40, 32);


u8		peer_type;
u8		peer_mac[12];

static BOOL app_start_up_flag = TRUE;

//////////////////////////////////////////////////////////////////////////////
//	Initialization: MAC address, Adv Packet, Response Packet
//////////////////////////////////////////////////////////////////////////////

//----------------------- UI ---------------------------------------------
#if (BLT_SOFTWARE_TIMER_ENABLE)
/**
 * @brief   This function is soft timer callback function.
 * @return  <0:delete the timer; ==0:timer use the same interval as prior; >0:timer use the return value as new interval. 
 */
int soft_timer_test0(void)
{
	//gpio 0 toggle to see the effect
	DBG_CHN4_TOGGLE;
	static u32 soft_timer_cnt0 = 0;
	soft_timer_cnt0++;
	return 0;
}
#endif
#if MI_SWITCH_LPN_EN
void mi_mesh_switch_sys_mode(u32 clk)
{
	// ignore hard ware uart function ,and it need to reset all the param part 
	if(clk == 16000000){
		clock_init(SYS_CLK_16M_Crystal);

	}else if (clk == 48000000){
		clock_init(SYS_CLK_48M_Crystal);
	}else{}
}

#endif
//----------------------- handle BLE event ---------------------------------------------
int app_event_handler (u32 h, u8 *p, int n)
{
	static u32 event_cb_num;
	event_cb_num++;
	int send_to_hci = 1;

	if (h == (HCI_FLAG_EVENT_BT_STD | HCI_EVT_LE_META))		//LE event
	{
		u8 subcode = p[0];
		#if MI_API_ENABLE
		telink_ble_mi_app_event(subcode,p,n);
		#endif 
	//------------ ADV packet --------------------------------------------
		if (subcode == HCI_SUB_EVT_LE_ADVERTISING_REPORT)	// ADV packet
		{
			event_adv_report_t *pa = (event_adv_report_t *)p;
			if(LL_TYPE_ADV_NONCONN_IND != (pa->event_type & 0x0F)){
				return 0;
			}

			#if 0 // TESTCASE_FLAG_ENABLE
			u8 mac_pts[] = {0xDA,0xE2,0x08,0xDC,0x1B,0x00};	// 0x001BDC08E2DA
			u8 mac_pts2[] = {0xDB,0xE2,0x08,0xDC,0x1B,0x00};	// 0x001BDC08E2DA
			if(memcmp(pa->mac, mac_pts,6) && memcmp(pa->mac, mac_pts2,6)){
				return 0;
			}
			#endif
			
			#if DEBUG_MESH_DONGLE_IN_VC_EN
			send_to_hci = mesh_dongle_adv_report2vc(pa->data, MESH_ADV_PAYLOAD);
			#else
			send_to_hci = app_event_handler_adv(pa->data, ADV_FROM_MESH, 1);
			#endif
		}

	//------------ connection complete -------------------------------------
		else if (subcode == HCI_SUB_EVT_LE_CONNECTION_COMPLETE)	// connection complete
		{
			#if MI_SWITCH_LPN_EN
			mi_mesh_switch_sys_mode(48000000);
			bls_ll_setAdvParam( ADV_INTERVAL_MIN, ADV_INTERVAL_MAX, \
			 	 	 	 	 	     ADV_TYPE_CONNECTABLE_UNDIRECTED, OWN_ADDRESS_PUBLIC, \
			 	 	 	 	 	     0,  NULL,  BLT_ENABLE_ADV_ALL, ADV_FP_NONE);
			#endif
			
			event_connection_complete_t *pc = (event_connection_complete_t *)p;
			if (!pc->status)							// status OK
			{
				app_led_en (pc->handle, 1);

				peer_type = pc->peer_adr_type;
				memcpy (peer_mac, pc->mac, 6);
			}
			#if DEBUG_BLE_EVENT_ENABLE
			rf_link_light_event_callback(LGT_CMD_BLE_CONN);
			#endif

			#if DEBUG_MESH_DONGLE_IN_VC_EN
			debug_mesh_report_BLE_st2usb(1);
			#endif
			proxy_cfg_list_init_upon_connection();
			#if FEATURE_FRIEND_EN
			fn_update_RecWin(get_RecWin_connected());
			#endif
			#if !MI_API_ENABLE
			mesh_service_change_report();
			#endif
		}

	//------------ connection update complete -------------------------------
		else if (subcode == HCI_SUB_EVT_LE_CONNECTION_UPDATE_COMPLETE)	// connection update
		{
			#if FEATURE_FRIEND_EN
			fn_update_RecWin(get_RecWin_connected());
			#endif
		}
	}

	//------------ disconnect -------------------------------------
	else if (h == (HCI_FLAG_EVENT_BT_STD | HCI_EVT_DISCONNECTION_COMPLETE))		//disconnect
	{
		#if MI_SWITCH_LPN_EN
		mi_mesh_switch_sys_mode(16000000);
		#endif
		event_disconnection_t	*pd = (event_disconnection_t *)p;
		app_led_en (pd->handle, 0);
		#if MI_API_ENABLE
		telink_ble_mi_app_event(HCI_EVT_DISCONNECTION_COMPLETE,p,n);
		#endif 
		//terminate reason
		if(pd->reason == HCI_ERR_CONN_TIMEOUT){

		}
		else if(pd->reason == HCI_ERR_REMOTE_USER_TERM_CONN){  //0x13

		}
		else if(pd->reason == SLAVE_TERMINATE_CONN_ACKED || pd->reason == SLAVE_TERMINATE_CONN_TIMEOUT){

		}
		#if DEBUG_BLE_EVENT_ENABLE
		rf_link_light_event_callback(LGT_CMD_BLE_ADV);
		#endif 

		#if DEBUG_MESH_DONGLE_IN_VC_EN
		debug_mesh_report_BLE_st2usb(0);
		#endif

		mesh_ble_disconnect_cb();
		#if FEATURE_FRIEND_EN
        fn_update_RecWin(FRI_REC_WIN_MS);   // restore
        #endif
	}

	if (send_to_hci)
	{
		//blc_hci_send_data (h, p, n);
	}

	return 0;
}

void proc_ui()
{
	static u32 tick, scan_io_interval_us = 40000;
	if (!clock_time_exceed (tick, scan_io_interval_us))
	{
		return;
	}
	tick = clock_time();

	#if 0
	static u8 st_sw1_last,st_sw2_last;	
	u8 st_sw1 = !gpio_read(SW1_GPIO);
	u8 st_sw2 = !gpio_read(SW2_GPIO);
	
	if(!(st_sw1_last)&&st_sw1){
	    scan_io_interval_us = 100*1000; // fix dithering
	    access_cmd_onoff(0xffff, 0, G_ON, CMD_NO_ACK, 0);
		foreach(i,NET_KEY_MAX){
					mesh_key.net_key[i][0].node_identity =1;
		}
	}
	st_sw1_last = st_sw1;
	
	if(!(st_sw2_last)&&st_sw2){
	    scan_io_interval_us = 100*1000; // fix dithering
	    access_cmd_onoff(0xffff, 0, G_OFF, CMD_NO_ACK, 0);
	}
	st_sw2_last = st_sw2;

	
	#endif

	#if 0
	static u8 st_sw2_last;	
	u8 st_sw2 = !gpio_read(SW2_GPIO);
	
	if(!(st_sw2_last)&&st_sw2){ // dispatch just when you press the button 
		//trigger the unprivison data packet 
		static u8 beacon_data_num;
		beacon_data_num =1;
		mesh_provision_para_reset();
		while(beacon_data_num--){
			unprov_beacon_send(MESH_UNPROVISION_BEACON_WITH_URI,0);
		}
		prov_para.initial_pro_roles = MESH_INI_ROLE_NODE;
	    scan_io_interval_us = 100*1000; // fix dithering
	}
	st_sw2_last = st_sw2;
	#endif
}

/////////////////////////////////////////////////////////////////////
// main loop flow
/////////////////////////////////////////////////////////////////////
#if 0
u8 notify_test_flag =0;
u8 notify_test_buf[19];
void test_sig_mesh_cmd_fun()
{
	if(notify_test_flag){
		static u32 tick_notify_test_tick =0;
		static u16 A_debug_sts_level =0;
		int ret_tmp =-1;
		if(!clock_time_exceed(tick_notify_test_tick,10*1000)){
			return;
		}	
		tick_notify_test_tick = clock_time();
		ret_tmp = mesh_tx_cmd_rsp(G_LEVEL_STATUS, (u8 *)&A_debug_sts_level, sizeof(A_debug_sts_level), ele_adr_primary, 
						ele_adr_primary, 0, 0);
		if(A_debug_notify_pkt_sts == BLE_SUCCESS && ret_tmp == 0){
			A_debug_notify_pkt_sts = HCI_ERR_MAC_CONN_FAILED;
			A_debug_sts_level++;
		}
	}
}
#endif
// simu uart io printf demo 
#if 0
void test_simu_io_user_define_proc()
{
    static u32 A_debug_print_tick =0;
    u8 data_test[4]={1,2,3,4};
    u8 data_val = 0xff;
    if(clock_time_exceed(A_debug_print_tick,100*1000)){
        A_debug_print_tick = clock_time();
        LOG_USER_MSG_INFO(data_test,sizeof(data_test),"user data val is %d",data_val);
    }
}
#endif


// START User Functions

#ifdef APP_DBG_EN
static void send_network_infor(void)
{
	static u32 tmp_time = 0;
	if(clock_time_exceed_ms(tmp_time, 2000)) {
		DBG_APP_SEND_STR("\n\n NETKEY1: ");
		foreach(i,16){
			DBG_APP_SEND_BYTE((u16) mesh_key.net_key[0][0].key[i]);
			DBG_APP_SEND_STR(" ");
		}
		DBG_APP_SEND_STR(" - ");
		DBG_APP_SEND_BYTE((u16) mesh_key.net_key[0][0].index);

		DBG_APP_SEND_STR("\n APPKEY1: ");
		foreach(i,16){
			DBG_APP_SEND_BYTE((u16) mesh_key.net_key[0][0].app_key[0].key[i]);
			DBG_APP_SEND_STR(" ");
		}
		DBG_APP_SEND_STR(" - ");
		DBG_APP_SEND_BYTE((u16) mesh_key.net_key[0][0].app_key[0].index);

		DBG_APP_SEND_STR("\n NETKEY2: ");
		foreach(i,16){
			DBG_APP_SEND_BYTE((u16) mesh_key.net_key[1][0].key[i]);
			DBG_APP_SEND_STR(" ");
		}
		DBG_APP_SEND_STR(" - ");
		DBG_APP_SEND_BYTE((u16) mesh_key.net_key[1][0].index);

		DBG_APP_SEND_STR("\n NETKEY2: ");
		foreach(i,16){
			DBG_APP_SEND_BYTE((u16) mesh_key.net_key[1][0].app_key[0].key[i]);
			DBG_APP_SEND_STR(" ");
		}
		DBG_APP_SEND_STR(" - ");
		DBG_APP_SEND_BYTE((u16) mesh_key.net_key[1][0].app_key[0].index);

		DBG_APP_SEND_STR("\n DEV_KEY: ");
		foreach(i,16){
			DBG_APP_SEND_BYTE((u16) mesh_key.dev_key[i]);
			DBG_APP_SEND_STR(" ");
		}
		DBG_APP_SEND_STR("\n IVI: ");
		foreach(i,4){
			DBG_APP_SEND_BYTE((u16) iv_idx_st.cur[i]);
			DBG_APP_SEND_STR(" ");
		}
		DBG_APP_SEND_STR("\n ELE_ADD: ");
		DBG_APP_SEND_HEX(ele_adr_primary);
		tmp_time = clock_time_ms();
	}
}
#endif

static void app_lm_init()
{
	// Default network
	check_and_del_default_power_on();
#if BIND_APPKEY_PERIODIC
	bind_periodic_init();
#endif
	// Button
	option_button_init();
	option_button_callback_init(ev_handle_button_event);
	// Input
	input_init();
	input_callback_init(ev_handle_input_state_change);
	// Input and Output
	io_init();
	io_callback_init(vd_setup_send_config_node_params_delay);
	// Execution Scene
	execution_scene_init();
	// Led
	LED_init();
	LED_refreshCallbackInit(ev_handle_refresh_config_led);
	LED_refresh(0xFFFF);
	// Relay
	RL_init();
	// CMD Sig ON/OFF callback
	gen_callback_init(handle_mesh_cmd_sig_g_on_off_set);
	// Vendor
	vd_config_node_rsp_delay_init();
	// Binding
    #if BINDING_ENABLE
	binding_init();
    #endif
	sw_config_init();
	// Led Startup
	push_led_event_to_fifo(LED_POWER_ON, 0xFFFF);
}

static void io_taskmanager(void)
{
	if(app_start_up_flag == TRUE){
		if(clock_time_exceed_ms(0, TIMER_1S)){
			app_start_up_flag = FALSE;
		}
		else{
			return;
		}
	}
	option_button_scan();
	input_scan_all_channel();
    LED_handLeEventFunction();
    (void)RL_controlRelayHandle();

	#ifdef APP_DBG_EN
	send_network_infor();
	#endif
}

void debug_destination(void)
{
	static u32 temp = 0;
	if(clock_time_exceed_ms(temp, 1000)) {
		DBG_APP_SEND_STR("\n ---- APP_DST: ");
		DBG_APP_SEND_HEX(nwk_control_msg_para[0].dst);
		temp = clock_time_ms();
	}
}

// END User Functions

void main_loop ()
{
	static u32 tick_loop;
	tick_loop ++;
	mesh_loop_proc_prior(); // priority loop, especially for 8269
	blt_sdk_main_loop ();
	//  add spp UI task:
	proc_ui();
	proc_led();
	handle_factory_reset_with_delay();
	mesh_loop_process();
	io_taskmanager();
	proc_default_network();
	debug_destination();
	serial_proc();
}

void user_init()
{
	#if DEBUG_EVB_EN
	    set_sha256_init_para_mode(1);	// must 1
	#else
		user_sha256_data_proc();
	#endif

	mesh_global_var_init();

    #if (DUAL_MODE_WITH_TLK_MESH_EN)
	dual_mode_en_init();    // must before proc_telink_mesh_to_sig_mesh_, because "dual_mode_state" is used in it.
    #endif
	proc_telink_mesh_to_sig_mesh();		// must at first
	set_blc_hci_flag_fun(0);// disable the hci part of for the lib .
	#if (DUAL_MODE_ADAPT_EN)
	dual_mode_en_init();    // must before factory_reset_handle, because "dual_mode_state" is used in it.
	#endif
	blc_app_loadCustomizedParameters();  //load customized freq_offset cap value and tp value

	usb_id_init();
	usb_log_init();
	usb_dp_pullup_en (1);  //open USB enum

	////////////////// BLE stack initialization ////////////////////////////////////
#if (DUAL_VENDOR_EN)
	mesh_common_retrieve(FLASH_ADR_PROVISION_CFG_S);
	if(DUAL_VENDOR_ST_ALI != provision_mag.dual_vendor_st)
#endif
	{
		ble_mac_init();
	}

	//link layer initialization
	//bls_ll_init (tbl_mac);
#if(MCU_CORE_TYPE == MCU_CORE_8269)
	blc_ll_initBasicMCU(tbl_mac);   //mandatory
#elif((MCU_CORE_TYPE == MCU_CORE_8258) || (MCU_CORE_TYPE == MCU_CORE_8278))
	blc_ll_initBasicMCU();                      //mandatory
	blc_ll_initStandby_module(tbl_mac);				//mandatory
#endif
	blc_ll_initAdvertising_module(tbl_mac); 	//adv module: 		 mandatory for BLE slave,
	blc_ll_initSlaveRole_module();				//slave module: 	 mandatory for BLE slave,
	blc_ll_initPowerManagement_module();        //pm module:      	 optional
	bls_pm_setSuspendMask (SUSPEND_DISABLE);//(SUSPEND_ADV | SUSPEND_CONN)
	
	//l2cap initialization
	//blc_l2cap_register_handler (blc_l2cap_packet_receive);
	blc_l2cap_register_handler (app_l2cap_packet_receive); // define the l2cap part 
	///////////////////// USER application initialization ///////////////////

	mesh_scan_rsp_init();

	u8 status = bls_ll_setAdvParam( ADV_INTERVAL_MIN, ADV_INTERVAL_MAX, \
			 	 	 	 	 	     ADV_TYPE_CONNECTABLE_UNDIRECTED, OWN_ADDRESS_PUBLIC, \
			 	 	 	 	 	     0,  NULL,  BLT_ENABLE_ADV_ALL, ADV_FP_NONE);

	if(status != BLE_SUCCESS){  //adv setting err
		write_reg8(0x8000, 0x11);  //debug
		while(1);
	}
	
	// normally use this settings 
	blc_ll_setAdvCustomedChannel (37, 38, 39);
	bls_ll_setAdvEnable(1);  //adv enable

	rf_set_power_level_index (MY_RF_POWER_INDEX);	
	
    blc_hci_le_setEventMask_cmd(HCI_LE_EVT_MASK_ADVERTISING_REPORT|
								HCI_LE_EVT_MASK_CONNECTION_COMPLETE|
								HCI_LE_EVT_MASK_CONNECTION_UPDATE_COMPLETE);

	////////////////// SPP initialization ///////////////////////////////////
#if (HCI_ACCESS != HCI_NONE)
	#if (HCI_ACCESS==HCI_USE_USB)
	//blt_set_bluetooth_version (BLUETOOTH_VER_4_2);
	//bls_ll_setAdvChannelMap (BLT_ENABLE_ADV_ALL);
	usb_bulk_drv_init (0);
	blc_register_hci_handler (app_hci_cmd_from_usb, blc_hci_tx_to_usb);
	#else	//uart
	sensorInit();
	#endif
#endif

	#if ADC_ENABLE
	adc_drv_init();
	#endif
	rf_pa_init();
	bls_app_registerEventCallback (BLT_EV_FLAG_CONNECT, (blt_event_callback_t)&mesh_ble_connect_cb);
	blc_hci_registerControllerEventHandler(app_event_handler);		//register event callback
	//bls_hci_mod_setEventMask_cmd(0xffff);			//enable all 15 events,event list see ble_ll.h
	bls_set_advertise_prepare (app_advertise_prepare_handler);
	//bls_set_update_chn_cb(chn_conn_update_dispatch);

	bls_ota_registerStartCmdCb(entry_ota_mode);
	bls_ota_registerResultIndicateCb(show_ota_result);

	app_enable_scan_all_device ();	// enable scan adv packet

	// mesh_mode and layer init
	mesh_init_all();
	// OTA init
	#if (DUAL_MODE_ADAPT_EN || DUAL_MODE_WITH_TLK_MESH_EN)
	if(DUAL_MODE_NOT_SUPPORT == dual_mode_state)
	#endif
	{
		bls_ota_clearNewFwDataArea();	 //must
	}
	//blc_ll_initScanning_module(tbl_mac);
	//gatt initialization
	#if((MCU_CORE_TYPE == MCU_CORE_8258) || (MCU_CORE_TYPE == MCU_CORE_8278))
	blc_gap_peripheral_init();    //gap initialization
	#endif
	my_att_init (provision_mag.gatt_mode);
	blc_att_setServerDataPendingTime_upon_ClientCmd(10);

	extern u32 system_time_tick;
	system_time_tick = clock_time();

#if (BLT_SOFTWARE_TIMER_ENABLE)
	blt_soft_timer_init();
	//blt_soft_timer_add(&soft_timer_test0, 200*1000);
#endif

	DBG_APP_SEND_STR("\n POWER ON");
	app_lm_init();

	// Vendor model
	vendor_model_init();
}

// End File
