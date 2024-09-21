/********************************************************************************************************
 * @file     app_config_8258.h 
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

/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

#include "../../vendor/common/version.h"    // include mesh_config.h inside.

//////////////////board sel/////////////////////////////////////

#define PCBA_8258_DONGLE_48PIN          1
#define PCBA_8258_C1T139A30_V1_0        2
#define PCBA_8258_C1T139A30_V1_2        3
#define PCBA_8258_LUMI_RGBW_V1_0        4
#define PCBA_8258_RANGDONG_RGBWW_V1_0   5

#define PCBA_8258_SEL                   PCBA_8258_RANGDONG_RGBWW_V1_0

// Customer Config

#define LIGHT_OPT_VENDOR_ENABLE              1
#define LIGHT_AUTO_TRANS_VENDOR_ENABLE       1
#define GATEWAY_UNICAST_ADDR                 0x0001
#define ENABLE_AUTO_BIND_BEFORE_ADD_APPKEY   1
#define ENABLE_BLINK_LED_OTA                 0
#define ERASE_ALL_CONFIG_WHEN_REST           1
#define ENABLE_TRANS_TIME                    0
#define ENABLE_BLINK_SINGLE_LED              1

#define BINDING_ENABLE                       1
#define SWITCH_ENABLE_BINDING                1
#define EN_EVT_ON_OFF_CONTROL_EXECUTION      1
#define EN_CHECK_SOURCE_CONTROL              1
#define CACULATOR_FREQ_EN                    1
#define BIND_APPKEY_PERIODIC                 1
#define CTL_WITH_FLAG_DEFAULT_POSITION_EN    1

#define _USER_CONFIG_DEFINED_	1	// must define this macro to make others known
#define	__LOG_RT_ENABLE__		0

#if DUAL_VENDOR_EN
#define FLASH_1M_ENABLE         1   // must
#else
#define FLASH_1M_ENABLE         0
#endif

#if FLASH_1M_ENABLE
#define PINGPONG_OTA_DISABLE    0 // it can disable only when 1M flash.
#if	PINGPONG_OTA_DISABLE
#define SWITCH_FW_ENABLE		0 // set to 0, just for particular customer 
#endif
#endif

//////////// product  Information  //////////////////////////////
#define ID_VENDOR				0x248a			// for report
#define ID_PRODUCT_BASE			0x880C
#define STRING_VENDOR			L"Telink"
#define STRING_PRODUCT			L"BLE Mesh"
#define STRING_SERIAL			L"TLSR825X"

#define DEV_NAME                "SigMesh"

#define APPLICATION_DONGLE		0					// or else APPLICATION_DEVICE
#define	USB_PRINTER				1
#define	FLOW_NO_OS				1

/////////////////////HCI ACCESS OPTIONS///////////////////////////////////////
#define HCI_USE_NONE	0
#define HCI_USE_UART	1
#define HCI_USE_USB		2

#if WIN32
#define HCI_ACCESS		HCI_USE_USB
#else
#define HCI_ACCESS		HCI_USE_UART
#endif 

#if (HCI_ACCESS == HCI_USE_UART)
#define UART_TX_PIN		UART_TX_PD7
#define UART_RX_PIN		UART_RX_PA0
#endif

#define HCI_LOG_FW_EN      1
#if HCI_LOG_FW_EN
#define DEBUG_INFO_TX_PIN  GPIO_PD7
#define PRINT_DEBUG_INFO   1
#endif

#define ADC_ENABLE		    0
#if ADC_ENABLE
#define ADC_CHNM_ANA_INPUT  AVSS
#define ADC_CHNM_REF_SRC 	RV_1P428
#endif

#define ONLINE_STATUS_EN            0

#define DUAL_MODE_ADAPT_EN 			0   // dual mode as master with Zigbee
#if (0 == DUAL_MODE_ADAPT_EN)
#define DUAL_MODE_WITH_TLK_MESH_EN  0   // dual mode as slave with Telink mesh
#endif

/////////////////// mesh project config /////////////////////////////////
#if (MESH_RX_TEST || (!MD_DEF_TRANSIT_TIME_EN))
#define TRANSITION_TIME_DEFAULT_VAL (0)
#else
	#if MI_API_ENABLE
#define TRANSITION_TIME_DEFAULT_VAL	 0
	#else
#define TRANSITION_TIME_DEFAULT_VAL (GET_TRANSITION_TIME_WITH_STEP(1, TRANSITION_STEP_RES_1S)) // (0x41)  // 0x41: 1 second // 0x00: means no default transition time
	#endif
#endif

/////////////////// MODULE /////////////////////////////////
#if MI_SWITCH_LPN_EN
#define BLE_REMOTE_PM_ENABLE			1
#else
#define BLE_REMOTE_PM_ENABLE			0
#endif
#define BLE_REMOTE_SECURITY_ENABLE      0
#define BLE_IR_ENABLE					0
#define BLE_SIG_MESH_CERTIFY_ENABLE 	0

#ifndef BLT_SOFTWARE_TIMER_ENABLE
#define BLT_SOFTWARE_TIMER_ENABLE		0
#endif

#if MI_SWITCH_LPN_EN
#define PM_DEEPSLEEP_RETENTION_ENABLE   1
#else
#define PM_DEEPSLEEP_RETENTION_ENABLE   0
#endif
//////////////////////////// KEYSCAN/MIC  GPIO //////////////////////////////////
#define	MATRIX_ROW_PULL					PM_PIN_PULLDOWN_100K
#define	MATRIX_COL_PULL					PM_PIN_PULLUP_10K

#define	KB_LINE_HIGH_VALID				0   //dirve pin output 0 when keyscan, scanpin read 0 is valid
#define DEEPBACK_FAST_KEYSCAN_ENABLE	1   //proc fast scan when deepsleep back trigged by key press, in case key loss
#define KEYSCAN_IRQ_TRIGGER_MODE		0
#define LONG_PRESS_KEY_POWER_OPTIMIZE	1   //lower power when pressing key without release

//stuck key
#define STUCK_KEY_PROCESS_ENABLE		0
#define STUCK_KEY_ENTERDEEP_TIME		60  //in s

//repeat key
#define KB_REPEAT_KEY_ENABLE			0
#define	KB_REPEAT_KEY_INTERVAL_MS		200
#define KB_REPEAT_KEY_NUM				1
//

//----------------------- GPIO for UI --------------------------------
//---------------  Button 

#if (PCBA_8258_SEL == PCBA_8258_DONGLE_48PIN)||(PCBA_8258_SEL == PCBA_8258_LUMI_RGBW_V1_0) || (PCBA_8258_SEL == PCBA_8258_RANGDONG_RGBWW_V1_0)
#define PULL_WAKEUP_SRC_PD6     PM_PIN_PULLUP_1M	//btn
#define PULL_WAKEUP_SRC_PD5     PM_PIN_PULLUP_1M	//btn
#define PD6_INPUT_ENABLE		1
#define PD5_INPUT_ENABLE		1
#define	SW1_GPIO				GPIO_PD6
#define	SW2_GPIO				GPIO_PD5
#elif(PCBA_8258_SEL == PCBA_8258_C1T139A30_V1_2)
#define PULL_WAKEUP_SRC_PB2     PM_PIN_PULLUP_1M	//btn
#define PULL_WAKEUP_SRC_PB3     PM_PIN_PULLUP_1M	//btn
#define PB2_INPUT_ENABLE		1
#define PB3_INPUT_ENABLE		1
#define	SW1_GPIO				GPIO_PB2            // SW2 in board
#define	SW2_GPIO				GPIO_PB3            // SW4 in board

#if 1 // must output 0, because it is keyboard array. pull down is not enough to output low level.
#define PB4_FUNC                AS_GPIO
#define PB4_OUTPUT_ENABLE       1
#define PB4_DATA_OUT            0
#endif
#else   // PCBA_8258_C1T139A30_V1_0
#define PULL_WAKEUP_SRC_PD2     PM_PIN_PULLUP_1M	//btn
#define PULL_WAKEUP_SRC_PD1     PM_PIN_PULLUP_1M	//btn
#define PD2_INPUT_ENABLE		1
#define PD1_INPUT_ENABLE		1
#define	SW1_GPIO				GPIO_PD2
#define	SW2_GPIO				GPIO_PD1
#endif


#define XIAOMI_MODULE_ENABLE	  MI_API_ENABLE
#define XIAOMI_TEST_CODE_ENABLE   0

//---------------  LED / PWM
#if(PCBA_8258_SEL == PCBA_8258_DONGLE_48PIN)

    #define RL1_IO_PIN   GPIO_PA3
    #define RL2_IO_PIN   GPIO_PA2
    #define RL3_IO_PIN   GPIO_PB0
    #define RL4_IO_PIN   GPIO_PA4

#elif(PCBA_8258_SEL == PCBA_8258_C1T139A30_V1_0)   // PCBA_8258_DEVELOPMENT_BOARD

	#define RL1_IO_PIN   GPIO_PD3
	#define RL2_IO_PIN   GPIO_PD4
	#define RL3_IO_PIN   GPIO_PD5
	#define RL4_IO_PIN   GPIO_PD2

#elif (PCBA_8258_SEL == PCBA_8258_LUMI_RGBW_V1_0)

	#define RL1_IO_PIN   GPIO_PB4
	#define RL2_IO_PIN   GPIO_PB5
	#define RL3_IO_PIN   GPIO_PC2
	#define RL4_IO_PIN   GPIO_PC3

#elif (PCBA_8258_SEL == PCBA_8258_RANGDONG_RGBWW_V1_0)

	#define RL1_IO_PIN   GPIO_PC3
	#define RL2_IO_PIN   GPIO_PC4
	#define RL3_IO_PIN   GPIO_PB4
	#define RL4_IO_PIN   GPIO_PB5

#endif

#define GPIO_LED	RL1_IO_PIN

/////////////open SWS digital pullup to prevent MCU err, this is must ////////////
#define PA7_DATA_OUT			1

//save suspend current
//#define PA5_FUNC 	AS_GPIO     // USB DM
//#define PA6_FUNC 	AS_GPIO     // USB DP

/////////////////// Clock  /////////////////////////////////
#define	USE_SYS_TICK_PER_US
#define CLOCK_SYS_TYPE  		CLOCK_TYPE_PLL	//  one of the following:  CLOCK_TYPE_PLL, CLOCK_TYPE_OSC, CLOCK_TYPE_PAD, CLOCK_TYPE_ADC

#if (MI_API_ENABLE)
#define CLOCK_SYS_CLOCK_HZ  	48000000

#else
#define CLOCK_SYS_CLOCK_HZ  	24000000
#define CLOCK_SYS_CLOCK_MHZ     24
#endif
//////////////////Extern Crystal Type///////////////////////
#define CRYSTAL_TYPE			XTAL_12M		//  extern 12M crystal

/////////////////// watchdog  //////////////////////////////
#define MODULE_WATCHDOG_ENABLE		1
#if (MI_API_ENABLE)
#define WATCHDOG_INIT_TIMEOUT		20000  //in mi mode the watchdog timeout is 20s
#else
#define WATCHDOG_INIT_TIMEOUT		13000   //default is 2000 ms
#endif

/////////////////// set default   ////////////////

#include "../common/default_config.h"

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif
