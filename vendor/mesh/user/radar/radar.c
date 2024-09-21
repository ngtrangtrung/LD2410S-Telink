/*
 * radar.c
 *
 *  Created on: Aug 30, 2024
 *      Author: TrungNT
 */

/*******************************************************************************
 *				 _ _                                             _ _
				|   |                                           (_ _)
				|   |        _ _     _ _   _ _ _ _ _ _ _ _ _ _   _ _
				|   |       |   |   |   | |    _ _     _ _    | |   |
				|   |       |   |   |   | |   |   |   |   |   | |   |
				|   |       |   |   |   | |   |   |   |   |   | |   |
				|   |_ _ _  |   |_ _|   | |   |   |   |   |   | |   |
				|_ _ _ _ _| |_ _ _ _ _ _| |_ _|   |_ _|   |_ _| |_ _|
								(C)2022 Lumi
 * Copyright (c) 2022
 * Lumi, JSC.
 * All Rights Reserved
 *
 * File name: Example.h
 *
 * Description:
 *
 *
 * Last Changed By:  $Author: trungnt $
 * Revision:         $Revision: $
 * Last Changed:     $Date: $April 15, 2022
 *
 * Code sample:
 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "radar.h"
#include "../../user/debug.h"
#include "../../user/radar/queue.h"
#include "../../user/radar/uart1_cmd.h"
#include "proj/common/tstring.h"
#include "vendor/common/mesh_common.h"
#include "vendor/common/mesh_node.h"
/******************************************************************************/
/*                     PRIVATE TYPES and DEFINITIONS                         */
/******************************************************************************/

#ifdef  RADAR_DBG_EN
	#define DBG_RADAR_SEND_STR(x)   Dbg_sendString((s8*)x)
	#define DBG_RADAR_SEND_INT(x)   Dbg_sendInt(x)
	#define DBG_RADAR_SEND_HEX(x)   Dbg_sendHex(x)
	#define DBG_RADAR_SEND_BYTE(x)  Dbg_sendOneByteHex(x)
#else
	#define DBG_RADAR_SEND_STR(x)
	#define DBG_RADAR_SEND_INT(x)
	#define DBG_RADAR_SEND_HEX(x)
	#define DBG_RADAR_SEND_BYTE(x)
#endif



enum{
	STEP1 = 0,
	STEP2,
	STEP3,
};

static radar_params_t  radar_pars =
	{
		0,            	// signal_high_t
		0,            	// start_detect_t
		R_SIGNAL_LOW,   // current_signal
		R_SIGNAL_LOW,   // before_signal
		RADAR_INACTIVE, // real_state
		0,            	// pulse_len
		0xFFFF,       	// RADAR_ACTIVE_last_t
	};


static ss_radar_t ss_pars = {
					FALSE,   // active
					0,       // active_last_t
				};


typedef struct{
    u16 cmdid;
} PARAMETER_CONFIG;



typedef struct{
    u16 cmdid;
    u32 equipmentType;
    u16 versionType;
    u16 majorVersion;
    u16 minorVersion;
    u16 patchVersion;
} PARAMETER_FW_VERSION;

typedef struct{
    u16 cmdid;
    u16 enable; // 0 Success,  1 Fail
    u16 protocolVersionNumber;
    u16 bufferSize;
} PARAMETER_SRART_CONFIG;


typedef struct{
    u16 cmdid;
    u16 ackEnd; // 0 Success,  1 Fail
} PARAMETER_END_CONFIG;


typedef struct{
    u16 cmdid;
    u16 ackGeneric; // 0 Success,  1 Fail
} PARAMETER_GENERIC_CONFIG;


typedef struct{
    u16 cmdid;
    u16 ackGenericRead; // 0 Success,  1 Fail
	u32 detectNearestGate;
    u32 detectFarthestGate;
    u32 respondSpeed;
    u32 triggerRefreshRate;
    u32 maintainRefreshRate;
    u32 unattendedDelayTime;
} PARAMETER_GENERIC_READ_CONFIG;



typedef struct{
    u16 cmdid;
    u16 ackTheshold;
} PARAMETER_THRESHOLD_CONFIG;


typedef struct{
    u16 cmdid;
    u16 ackThesholdRead; // 0 Success,  1 Fail
    u32 motionGate0;
    u32 motionGate1;
    u32 motionGate2;
    u32 motionGate3;
    u32 motionGate4;
    u32 motionGate5;
    u32 motionGate6;
    u32 motionGate7;
    u32 holdingGate0;
    u32 holdingGate1;
    u32 holdingGate2;
    u32 holdingGate3;
    u32 holdingGate4;
    u32 holdingGate5;
    u32 holdingGate6;
    u32 holdingGate7;
} PARAMETER_THRESHOLD_READ_CONFIG;



typedef struct{
    u16 cmdid;
    u16 ackSNR;
} PARAMETER_SNR_CONFIG;


typedef struct{
    u16 cmdid;
    u16 ackSNRRead; // 0 Success,  1 Fail
    u32 motionGate0;
    u32 motionGate1;
    u32 motionGate2;
    u32 motionGate3;
    u32 motionGate4;
    u32 motionGate5;
    u32 motionGate6;
    u32 motionGate7;
    u32 holdingGate0;
    u32 holdingGate1;
    u32 holdingGate2;
    u32 holdingGate3;
    u32 holdingGate4;
    u32 holdingGate5;
    u32 holdingGate6;
    u32 holdingGate7;
} PARAMETER_SNR_READ_CONFIG;



typedef struct{
    u16 cmdid;
    u16 ackAutoConfig; // 0 Success,  1 Fail
} PARAMETER_AUTO_CONFIG;


typedef struct{
    u16 cmdid;
    u8 motion; // 0 unmotion 1/2/3 motion
    u16 timeFinish;  // finish 0-100%

} PARAMETER_TIME_FINISH_AUTO_CONFIG;

typedef struct{
    u16 cmdid;
    u8 motion; // 0 unmotion 1/2/3 motion
    u16 distanceTarget;  // khoang cach den doi tuong chuyen dong
}PARAMETER_MOTION_REPORT;

typedef union _DEVREPORT_BUFFER_ {
  PARAMETER_CONFIG						parameterConfig;
  PARAMETER_FW_VERSION        			fwVersion;
  PARAMETER_SRART_CONFIG				startConfig;
  PARAMETER_END_CONFIG					endConfig;
  PARAMETER_GENERIC_CONFIG				genericConfig;
  PARAMETER_GENERIC_READ_CONFIG			genericReadConfig;
  PARAMETER_THRESHOLD_CONFIG			thresholdConfig;
  PARAMETER_THRESHOLD_READ_CONFIG		thresholdReadConfig;
  PARAMETER_SNR_CONFIG					snrConfig;
  PARAMETER_SNR_READ_CONFIG				snrReadConfig;
  PARAMETER_AUTO_CONFIG					autoConfig;
  PARAMETER_TIME_FINISH_AUTO_CONFIG		timeFinishAutoConfig;
  PARAMETER_MOTION_REPORT				motionReport;
}*DEVREPORT_BUFFER_P;


enum{
	CONFIG_SUCCES,
	CONFIG_FAIL,
};
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/


/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
u8 radarCountTest = 0;
bool statusSignalRadarTest = UNMOTION;


bool radar_update_time_flag = FALSE;
bool statusSwitch = FALSE;
bool g_boModeTest = FALSE;


static u8 byStepEventOnlyRadar = STEP1;



Uart1Frame_t uart1RxStr;

static queue_t srQueueReceiver;
static u8 pBuffReceiver[SERIAL_DATA_MAX_LENGHT];

typeRadar_handleChangeState pvRADAR_handleChangeState = NULL;
byte_pUart1CallbackFunc2Args pvUart1RxHandleCallback = NULL;
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void tagretDirectionGPIOConfig(void);
static u8 radar_signal_check(void);
u8 RADAR_getSinal(void);
static u8 RxBufPoll(void);
static void radarEnableInit(void);
void serial_handle_rx_message_received( u8* buff, int len );
void handleDataRx(DEVREPORT_BUFFER_P pCmd, uint8_t length);
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/******************************************************************************/

/**
 * @func   sensorInit
 * @brief  Init Param for Radar Sensor
 * @param  [type] : RADAR, ALL
 * @retval None
 */
void sensorInit(void)
{
	uartRadarInit();
	radarInit(Sensor_HandleRadarState);
	DBG_RADAR_SEND_STR("\nsensorInit");
}




/**
 * @func   radarInit
 * @brief  Init Param for Radar Sensor
 * @param  [pUserHandler] : function call to handle when there is a change in motion
 * @retval None
 */
void radarInit(typeRadar_handleChangeState pUserHandler)
{
	radarEnableInit();
	tagretDirectionGPIOConfig();
	pvRADAR_handleChangeState  = pUserHandler;
//	DBG_RADAR_SEND_STR("\nRADAR Init");
}


static void radarEnableInit(void){
	gpio_set_func(RADAR_EN, AS_GPIO);
	gpio_set_output_en(RADAR_EN, 1);
	gpio_write(RADAR_EN, 0);
}
static void tagretDirectionGPIOConfig(void)
{
	gpio_set_func(TARGET_DIRECTION_GPIO, AS_GPIO);
	gpio_set_input_en(TARGET_DIRECTION_GPIO, TRUE);
	gpio_setup_up_down_resistor(TARGET_DIRECTION_GPIO, PM_PIN_PULLDOWN_100K);
}


/************************************************RADAR_HANDLER*******************************************/
/**
 * @brief  Check GPIO signal for radar
 *
 * @param  None
 *
 * @return [status]  : receiver status result
 */
static u8 radar_signal_check(void)
{
    if((gpio_read(TARGET_DIRECTION_GPIO)) == 0)
    {
        return 0;
    }

    return 1;
}

/**
 * @brief  Scan Motion for Radar
 *
 * @param  None
 *
 * @return [NONE]
 */
void RADAR_filterScanProc(void)
{

	if(radar_signal_check() == RADAR_ACTIVE)
	{
		sleep_us(RADAR_OUT_FILTER_SCAN_INTERVAL);
		if(radar_signal_check() == RADAR_ACTIVE)
		{
			radar_pars.current_signal = R_SIGNAL_HIGH;
			radar_pars.signal_high_t = clock_time_ms();
		}
	}
	else
	{
		if(clock_time_exceed_ms(
				radar_pars.signal_high_t, RADAR_PULL_DETECT_TIMEOUT))
		{
			radar_pars.current_signal = R_SIGNAL_LOW;
		}
	}
}


/**
 * @brief  Hold Event Radar Handler
 *
 * @param  None
 *
 * @return [NONE]
 */
static void RADAR_handleSignalHold(void)
{
	if(radar_pars.real_state == RADAR_INACTIVE)
	{
		if(clock_time_exceed_ms(
				radar_pars.start_detect_t, MIN_TIME_TO_DETECT_RADAR_ACTIVE))
		{
			radar_pars.real_state = RADAR_ACTIVE;

			if(pvRADAR_handleChangeState != NULL)
			{
				pvRADAR_handleChangeState(RADAR_ACTIVE);
			}

//			DBG_RADAR_SEND_STR("RADAR: 1 ");
		}
	}
}



/**
 * @brief  Scan state for Radar
 *
 * @param  None
 *
 * @return [None]
 */
static void RADAR_scanProc(void)
{
    if((radar_pars.before_signal == R_SIGNAL_LOW)
    			&& (radar_pars.current_signal == R_SIGNAL_HIGH))
    {
        // START
    	radar_pars.start_detect_t = clock_time_ms();
    }
    else if((radar_pars.before_signal == R_SIGNAL_HIGH)
    			&& (radar_pars.current_signal == R_SIGNAL_HIGH))
    {
        // HOLD
    	RADAR_handleSignalHold();
    }
    else if((radar_pars.before_signal == R_SIGNAL_HIGH)
    			&& (radar_pars.current_signal == R_SIGNAL_LOW))
    {
        // NO DETECT
    	radar_pars.real_state = RADAR_INACTIVE;
//    	DBG_RADAR_SEND_STR("RADAR: 0");
    }
    else if((radar_pars.before_signal == R_SIGNAL_LOW)
    			&& (radar_pars.current_signal == R_SIGNAL_LOW))
    {
    	// NO DETECT
    	radar_pars.real_state = RADAR_INACTIVE;
    }
    radar_pars.before_signal = radar_pars.current_signal;
}


/**
 * @brief  Radar task hander
 *
 * @param  None
 *
 * @return [None]
 */
void RADAR_task(void)
{
	// RADAR
	if(radar_update_time_flag == FALSE) {
		if(!clock_time_exceed_ms(0, RADAR_POWER_ON_TIME_MS)){
			return;
		}
	}


	RADAR_filterScanProc();
	RADAR_scanProc();

	if(radar_pars.real_state == RADAR_ACTIVE) {
		radar_pars.radar_active_last_t = clock_time_ms();
		if(radar_update_time_flag == FALSE) {
			radar_update_time_flag = TRUE;
		}
	}
}


/**
 * @func    resetStepStartDetect
 * @brief	reset again step detect
 * @param   STEP1/STEP2
 * @retval  None
 */
void resetStepStartDetect(u8 byStep){
	byStepEventOnlyRadar = byStep;
}
/**
 * @func    PIR_handlePirOutInterrupt
 * @brief
 * @param   None
 * @retval  None
 */
RadarState_enum RADAR_getState(void)
{
	return radar_pars.real_state;
}


/**
 * @func    RADAR_setActiveLastTime
 * @brief
 * @param   None
 * @retval  None
 */
void RADAR_setActiveLastTime(void)
{
	radar_pars.radar_active_last_t = clock_time_ms();
}


/**
 * @func    PIR_getActiveLastTime
 * @brief
 * @param   None
 * @retval  None
 */
u32 RADAR_getActiveLastTime(void)
{
	return radar_pars.radar_active_last_t;
}

/**
 * @func   Sensor_HandlePirInactive
 * @brief
 * @param  None
 * @retval None
 */
void Sensor_HandleRadarInactive(void)
{
	if(ss_pars.actived == TRUE) {
		// OFF timeout
			if((RADAR_getState() == RADAR_INACTIVE)   \
						&& (clock_time_exceed_ms(
								RADAR_getActiveLastTime(),ONE_SECOND))){
				// Radar active timeout
/**************************UNMOTION************************************/


/**********************************************************************/
				ss_pars.actived = FALSE;
			}
	}
}


/**
 * @func   Sensor_HandleRadarState
 * @brief
 * @param  state
 * @retval None
 */
void Sensor_HandleRadarState(RadarState_enum state)
{
		if(state == RADAR_ACTIVE) {
			// Control and Update
			if(ss_pars.actived == FALSE) {
				ss_pars.actived = TRUE;
				ss_pars.active_last_t = clock_time_ms();
				radar_pars.stateCurrent = MOTION;
/**************************MOTION************************************/


/********************************************************************/
			}
		}
}




/**
 * @brief  This function to process Radar Event
 *
 * @param  None
 *
 * @return [None]
 */
void statusRadarEventProcess(void)
{
	RADAR_task();
	Sensor_HandleRadarInactive();
}



/***********************************UART-RADAR-HANDLER*******************************************/


void uartRadarInit(void){
	uart_drv_init();
	uart1RxStr.DataReceiverStep = RX_STATE_START_1;



	blc_register_hci_handler (blc_rx_from_uart, blc_hci_tx_to_uart);		//default handler
	handle_tx_message_callback_init(serial_handle_rx_message_received);


	// Initialize queue for receive data
	queues_init(&srQueueReceiver, (u8*)pBuffReceiver, SERIAL_DATA_MAX_LENGHT, sizeof(u8));
}



/**
 * @func    serial_handle_rx_message_received
 * @brief
 * @param
 * @retval  None
 */
void serial_handle_rx_message_received( u8* buff, int len )
{

	for (u8 i = 0; i < len; i++) {
		queues_push(&srQueueReceiver, buff+i);
	}
//	DBG_RADAR_SEND_STR("\nLength: ");
//	DBG_RADAR_SEND_INT(len);
//	DBG_RADAR_SEND_STR("\n");
//	for(uint8_t i = 0; i < len; i++){
//		DBG_RADAR_SEND_HEX(buff[i]);
//		DBG_RADAR_SEND_STR(" ");
//	}
//	DBG_RADAR_SEND_STR("\n");
}



/************************ HANDLER SERIAL RECEIVER *****************************/

/**
 * @func   serial_proc
 * @brief  None
 * @param  None
 * @retval None
 */
void serial_proc(void)
{
	u8 stateRx = RxBufPoll();

	if (stateRx != UART_STATE_IDLE) {
		switch (stateRx) {
			case UART_STATE_ACK_RECEIVED:
				break;

			case UART_STATE_NACK_RECEIVED:
				break;

			case UART_STATE_DATA_RECEIVED:
				handleDataRx((DEVREPORT_BUFFER_P)&uart1RxStr.Data[0], uart1RxStr.DataPacketLength);
				break;

			case UART_STATE_ERROR:
			case UART_STATE_RX_TIMEOUT:
				break;

			default:
				break;
		}
	}
}

/**
 * @func   RxBufPoll
 * @brief  None
 * @param  None
 * @retval None
 */
static u8 RxBufPoll(void)
{
	u8 rcv_data;
	    u8 byUart1State = UART_STATE_IDLE;

	    while (queue_count(&srQueueReceiver) && (byUart1State == UART_STATE_IDLE))
	    {
	    	queues_get(&srQueueReceiver, &rcv_data);

	    	/* DEBUG DIR MCU -> ZB */
	//    	printf("%x  |   ", rcv_data);

//	    	DBG_RADAR_SEND_HEX(rcv_data);
//	    	DBG_RADAR_SEND_STR("\n");



			switch (uart1RxStr.DataReceiverStep)
			{
				case RX_STATE_START_1:
					if(rcv_data == FRAME_HEADER_REPORT_DATA){
						uart1RxStr.DataPacketLength = FRAME_LENGTH_REPORT_DATA;
						uart1RxStr.Data[0] = CMD_REPORT_HEADER;
						uart1RxStr.Data[1] = 0x00;
						uart1RxStr.ReceiverByteCount = 2;
						uart1RxStr.DataReceiverStep = RX_STATE_DATA;
					}else if(rcv_data == FRAME_HEADER_REPORT_START_BYTE1){
						uart1RxStr.Data[0] = CMD_REPORT_AUTO_CONFIG_HEADER;
						uart1RxStr.Data[1] = 0x00;
						uart1RxStr.ReceiverByteCount = 2;
						uart1RxStr.DataReceiverStep = RX_STATE_START_2;
					}else if(rcv_data == FRAME_HEADER_START_BYTE1){
						uart1RxStr.DataReceiverStep = RX_STATE_START_2;
					}else{
						byUart1State = UART_STATE_ERROR;
					}
				break;

				case RX_STATE_START_2:
					if((rcv_data == FRAME_HEADER_START_BYTE2) || (rcv_data == FRAME_HEADER_REPORT_START_BYTE2)){
						uart1RxStr.DataReceiverStep = RX_STATE_START_3;
					}else{
						uart1RxStr.DataReceiverStep = RX_STATE_START_1;
						byUart1State = UART_STATE_ERROR;
					}
				break;

				case RX_STATE_START_3:
					if((rcv_data == FRAME_HEADER_START_BYTE3) || (rcv_data == FRAME_HEADER_REPORT_START_BYTE3)){
						uart1RxStr.DataReceiverStep = RX_STATE_START_4;
					}else{
						uart1RxStr.DataReceiverStep = RX_STATE_START_1;
						byUart1State = UART_STATE_ERROR;
					}
				break;

				case RX_STATE_START_4:
					if((rcv_data == FRAME_HEADER_START_BYTE4) || (rcv_data == FRAME_HEADER_REPORT_START_BYTE4)){

						uart1RxStr.DataReceiverStep = RX_STATE_LENGTH_1;
					}else{
						uart1RxStr.DataReceiverStep = RX_STATE_START_1;
						byUart1State = UART_STATE_ERROR;
					}

				break;

				case RX_STATE_LENGTH_1:
					if(rcv_data > SERIAL_DATA_MAX_LENGHT){
						uart1RxStr.DataReceiverStep = RX_STATE_START_1;
						byUart1State = UART_STATE_ERROR;
					}
					else{
						uart1RxStr.DataPacketLength = rcv_data;
	//					printf("length: %x \n ", uart1RxStr.DataPacketLength);
						uart1RxStr.DataReceiverStep = RX_STATE_LENGTH_2;
					}


				break;
				case RX_STATE_LENGTH_2:
					if(rcv_data > SERIAL_DATA_MAX_LENGHT){
						uart1RxStr.DataReceiverStep = RX_STATE_START_1;
						byUart1State = UART_STATE_ERROR;
					}
					else{
						if((rcv_data == 0) && (uart1RxStr.Data[0] == CMD_REPORT_AUTO_CONFIG_HEADER))
						{
							uart1RxStr.DataPacketLength = uart1RxStr.DataPacketLength + 2;
							uart1RxStr.DataReceiverStep = RX_STATE_DATA;

						}else{
							uart1RxStr.ReceiverByteCount = 0;
							uart1RxStr.DataReceiverStep = RX_STATE_DATA;
						}
					}
				break;

				case RX_STATE_DATA:
					if(uart1RxStr.ReceiverByteCount < uart1RxStr.DataPacketLength){
						uart1RxStr.Data[uart1RxStr.ReceiverByteCount]	= rcv_data;
//						DBG_RADAR_SEND_HEX(uart1RxStr.Data[uart1RxStr.ReceiverByteCount]);
						uart1RxStr.ReceiverByteCount++;



					}else{ // RX_STATE_END_1
							if((rcv_data == FRAME_END_BYTE1) || (rcv_data == FRAME_REPORT_END_BYTE1) || (rcv_data == FRAME_END_REPORT_DATA)){
								if(rcv_data == FRAME_END_REPORT_DATA){
									byUart1State = UART_STATE_DATA_RECEIVED;
									uart1RxStr.DataReceiverStep = RX_STATE_START_1;
								}else{
									uart1RxStr.DataReceiverStep = RX_STATE_END_2;
								}

							}else{
								uart1RxStr.DataReceiverStep = RX_STATE_START_1;
								byUart1State = UART_STATE_ERROR;
							}
					}

				break;
				case RX_STATE_END_2:
					if((rcv_data == FRAME_END_BYTE2) || (rcv_data == FRAME_REPORT_END_BYTE2)){
	//					printf("\nEnd: %x ", rcv_data);
						uart1RxStr.DataReceiverStep = RX_STATE_END_3;
					}else{

						uart1RxStr.DataReceiverStep = RX_STATE_START_1;
						byUart1State = UART_STATE_ERROR;
					}
				break;

				case RX_STATE_END_3:
					if((rcv_data == FRAME_END_BYTE3) || (rcv_data == FRAME_REPORT_END_BYTE3)){
	//					printf("\nEnd: %x ", rcv_data);
						uart1RxStr.DataReceiverStep = RX_STATE_END_4;
					}else{
						uart1RxStr.DataReceiverStep = RX_STATE_START_1;
						byUart1State = UART_STATE_ERROR;
					}
				break;

				case RX_STATE_END_4:
					if((rcv_data == FRAME_END_BYTE4) || (rcv_data == FRAME_REPORT_END_BYTE4)){
	//					printf("\nEnd: %x ", rcv_data);
						byUart1State = UART_STATE_DATA_RECEIVED;
						uart1RxStr.DataReceiverStep = RX_STATE_START_1;
					}else{
						uart1RxStr.DataReceiverStep = RX_STATE_START_1;
						byUart1State = UART_STATE_ERROR;
					}
				break;

				default:
					uart1RxStr.DataReceiverStep = RX_STATE_START_1;
					break;
			}
	    }
	    return byUart1State;
}


/*******************************************************************************/


/*********************************SEND UART FUNC********************************/
/*******************************************************************************/



/**
 * @func    serial_common_cmd_response
 * @brief
 * @param
 * @retval
 */
static int serialSendCommadPackage(u8 *payload, u16 length) {
    if (length > MAX_SERIAL_PAYLOAD_LEN) {
        return -1;
    }

    u8 header[6] = {
        FRAME_HEADER_START_BYTE1,
        FRAME_HEADER_START_BYTE2,
        FRAME_HEADER_START_BYTE3,
        FRAME_HEADER_START_BYTE4,
        LO_UINT16(length),
        HI_UINT16(length)
    };

    u8 footer[4] = {
        FRAME_END_BYTE1,
        FRAME_END_BYTE2,
        FRAME_END_BYTE3,
        FRAME_END_BYTE4
    };

    u8 temp_buffer[6 + length + 4]; // 6 bytes header + length + 4 bytes footer

    memcpy(temp_buffer, header, 6);

    memcpy(temp_buffer + 6, payload, length);

    memcpy(temp_buffer + 6 + length, footer, 4);

    return my_fifo_push(&hci_tx_fifo, temp_buffer, sizeof(temp_buffer), NULL, 0);
}




/**
 * @func    startConfigCommand
 * @brief   call it when start config radar
 * @param   None
 */

void startConfigCommand(void){
	u8 cmdValue[4];
	cmdValue[0] = LO_UINT16(CMD_ENABLE_CONFIG);
	cmdValue[1] = HI_UINT16(CMD_ENABLE_CONFIG);
	cmdValue[2] = LO_UINT16(CMD_ENABLE_VALUE);
	cmdValue[3] = HI_UINT16(CMD_ENABLE_VALUE);

	serialSendCommadPackage(cmdValue, LENGTH_ENABLE_CONFIG);

}

/**
 * @func    endConfigCommand
 * @brief   call it when end config radar
 * @param   None
 */

void endConfigCommand(void){
	u8 cmdValue[2];
	cmdValue[0] = LO_UINT16(CMD_END_CONFIG);
	cmdValue[1] = HI_UINT16(CMD_END_CONFIG);
	serialSendCommadPackage(cmdValue, LENGTH_END_CONFIG);
}



/**
 * @func    writeSerialNumberCommand
 * @brief   Sends a command to write a serial number with a payload
 * 			containing the length (2 bytes) and the serial number.
 * @param   serialNumber: pointer to the buffer containing the serial number
 *          lengthSerialNumber: length of the serial number
 */
void writeSerialNumberCommand(u8* serialNumber, u16 lengthSerialNumber) {
    u8 cmdValue[4 + lengthSerialNumber];
    cmdValue[0] = LO_UINT16(CMD_WRITE_SERIAL_NUMBER);
    cmdValue[1] = HI_UINT16(CMD_WRITE_SERIAL_NUMBER);
    cmdValue[2] = LO_UINT16(lengthSerialNumber);
    cmdValue[3] = HI_UINT16(lengthSerialNumber);
    memcpy(&cmdValue[4], serialNumber, lengthSerialNumber);
    serialSendCommadPackage(cmdValue, lengthSerialNumber + 4);
}


/**
 * @func    readSerialNumberCommand
 * @brief   read serial Number of Radar
 * @param   None
 */
void readSerialNumberCommand(void){
	u8 cmdValue[2];
	cmdValue[0] = LO_UINT16(CMD_READ_SERIAL_NUMBER);
	cmdValue[1] = HI_UINT16(CMD_READ_SERIAL_NUMBER);
	serialSendCommadPackage(cmdValue, LENGTH_READ_SERIAL_NUMBER);
}



/**
 * @func    readFwRadarCommand
 * @brief   read Firmware of Radar
 * @param   None
 */
void readFwRadarCommand(void){
	u8 cmdValue[2];
	cmdValue[0] = LO_UINT16(CMD_READ_FW_RADAR);
	cmdValue[1] = HI_UINT16(CMD_READ_FW_RADAR);
	serialSendCommadPackage(cmdValue, LENGTH_READ_FW_RADAR);
}


// Define structure for parameter entry
typedef struct {
    u16 command;
    u16 value;
} ParameterEntry;

/**
 * @func    writeGenericParameterCommand
 * @brief   Sends a command with payload containing detect and response parameters.
 * @param   detectNearestGate: 2-byte value for additional data of the nearest distance gate
 *           (Gate: 1->11)
 * @param   detectFarthestGate: 2-byte value for additional data of the farthest distance gate
 *           (Range: 1->12)
 * @param   respondSpeed: response speed
 *           (5 (Normal)/ 10 (Fast))	default: fast
 * @param   triggerRefreshRate: 2-byte value for trigger refresh rate in Hertz
 *           (Range: 0.5~8 Hz, 0.5 Hz step) default 4.0 Hz
 * @param   maintainRefreshRate: 2-byte value for maintain refresh rate in Hertz
 *           (Range: 0.5~8 Hz, 0.5 Hz step)
 * @param   unattendedDelayTime: 2-byte value for unattended delay time in seconds
 *           (Range: 10~120 seconds) default: 10s
 */
void writeGenericParameterCommand(
	u16 detectNearestGate,
    u16 detectFarthestGate,
    u16 respondSpeed,
    u16 triggerRefreshRate,
    u16 maintainRefreshRate,
    u16 unattendedDelayTime
) {
    // Array of parameter entries
    ParameterEntry parameters[] = {
        {DETECT_NEAREST_DISTANCE_GATE, detectNearestGate},
        {DETECT_FARTHEST_DISTANCE_GATE, detectFarthestGate},
        {RESPONSE_SPEED, respondSpeed},
        {TRIGGER_REFRESH_RATE, triggerRefreshRate},
        {MAINTAIN_REFRESH_RATE, maintainRefreshRate},
        {UNMANNED_DELAY_TIME, unattendedDelayTime}
    };

    // Calculate the total length of the payload
    u16 payloadLength = 2 + sizeof(parameters) * 6; // 2 bytes for command word, 6 parameters of 6 bytes each

    // Create payload buffer
    u8 payload[payloadLength];

    // Initialize payload with 0
    memset(payload, 0, sizeof(payload));

    payload[0] = LO_UINT16(CMD_WRITE_GENERIC_PARAM_COMMAND);
    payload[1] = HI_UINT16(CMD_WRITE_GENERIC_PARAM_COMMAND);

    // Fill payload buffer
    u8 *ptr = payload + 2;
    for (int i = 0; i < sizeof(parameters) / sizeof(parameters[0]); i++) {
        *ptr++ = LO_UINT16(parameters[i].command);
        *ptr++ = HI_UINT16(parameters[i].command);
        *ptr++ = LO_UINT16(parameters[i].value);
        *ptr++ = HI_UINT16(parameters[i].value);
        *ptr++ = 0x00; // 00 byte
        *ptr++ = 0x00; // 00 byte
    }

    // Send the command package
    serialSendCommadPackage(payload, sizeof(payload));
}



/**
 * @func    readCommonParameterCommand
 * @brief   read Distance Motion and Holding, delay time, response Speed
 * @param   None
 */

void readCommonParameterCommand(void){
	u8 cmdValue[LENGTH_READ_GENERIC_PARAM_COMMAND];
	cmdValue[0] = LO_UINT16(CMD_READ_GENERIC_PARAM_COMMAND);
	cmdValue[1] = HI_UINT16(CMD_READ_GENERIC_PARAM_COMMAND);
	cmdValue[2] = LO_UINT16(DETECT_FARTHEST_DISTANCE_GATE);
	cmdValue[3] = HI_UINT16(DETECT_FARTHEST_DISTANCE_GATE);
	cmdValue[4] = LO_UINT16(DETECT_NEAREST_DISTANCE_GATE);
	cmdValue[5] = HI_UINT16(DETECT_NEAREST_DISTANCE_GATE);
	cmdValue[6] = LO_UINT16(UNMANNED_DELAY_TIME);
	cmdValue[7] = HI_UINT16(UNMANNED_DELAY_TIME);
	cmdValue[8] = LO_UINT16(TRIGGER_REFRESH_RATE);
	cmdValue[9] = HI_UINT16(TRIGGER_REFRESH_RATE);
	cmdValue[10] = LO_UINT16(MAINTAIN_REFRESH_RATE);
	cmdValue[11] = HI_UINT16(MAINTAIN_REFRESH_RATE);
	cmdValue[12] = LO_UINT16(RESPONSE_SPEED);
	cmdValue[13] = HI_UINT16(RESPONSE_SPEED);
	serialSendCommadPackage(cmdValue, LENGTH_READ_GENERIC_PARAM_COMMAND);
}


/**
 * @func    writeSnrParameterCommand
 * @brief   write SNR parameter of Radar
 * @param   16 Gate of Radar sensor, every gate 0.7m
 */
void writeThresholdParameterCommand(
    u16 motionGate0,
    u16 motionGate1,
    u16 motionGate2,
    u16 motionGate3,
    u16 motionGate4,
    u16 motionGate5,
    u16 motionGate6,
    u16 motionGate7,
    u16 holdingGate0,
    u16 holdingGate1,
    u16 holdingGate2,
    u16 holdingGate3,
    u16 holdingGate4,
    u16 holdingGate5,
    u16 holdingGate6,
    u16 holdingGate7
) {
    // Payload buffer size
    const u16 payloadLength = 2 + (16 * 6); // 2 bytes for command word, 16 gates * 6 bytes per gate

    // Create payload buffer
    u8 payload[payloadLength];

    // Initialize payload with 0
    memset(payload, 0, sizeof(payload));

    payload[0] = LO_UINT16(CMD_WRITE_THRESHOLD_COMMAND);
    payload[1] = HI_UINT16(CMD_WRITE_THRESHOLD_COMMAND);

    // Fill payload buffer
    u8 *ptr = payload + 2;

    // Motion Gates
    u16 motionGates[8] = {motionGate0, motionGate1, motionGate2, motionGate3, motionGate4, motionGate5, motionGate6, motionGate7};
    for (u8 i = 0; i < 8; i++) {
        *ptr++ = i; // Motion Gate ID (0-7)
        *ptr++ = 0x00;
        *ptr++ = LO_UINT16(motionGates[i]);
        *ptr++ = HI_UINT16(motionGates[i]);
        *ptr++ = 0x00; // 00 byte
        *ptr++ = 0x00; // 00 byte
    }

    // Holding Gates
    u16 holdingGates[8] = {holdingGate0, holdingGate1, holdingGate2, holdingGate3, holdingGate4, holdingGate5, holdingGate6, holdingGate7};
    for (u8 i = 0; i < 8; i++) {
        *ptr++ = i + 8; // Holding Gate ID (8-15)
        *ptr++ = 00;
        *ptr++ = LO_UINT16(holdingGates[i]);
        *ptr++ = HI_UINT16(holdingGates[i]);
        *ptr++ = 0x00; // 00 byte
        *ptr++ = 0x00; // 00 byte
    }

    // Send the command package
    serialSendCommadPackage(payload, sizeof(payload));
}



/**
 * @func    readThresholdParameterCommand
 * @brief   read Threshold parameter of Radar
 * @param   None
 */
void readThresholdParameterCommand(void) {
    u8 payload[LENGTH_READ_THRESHOLD_COMMAND];

    payload[0] = LO_UINT16(CMD_READ_THRESHOLD_COMMAND);
    payload[1] = HI_UINT16(CMD_READ_THRESHOLD_COMMAND);

    for (GateID gateID = MotionGate0; gateID <= HoldingGate7; gateID++) {
        payload[2 + (gateID * 2)] = LO_UINT16(gateID);
        payload[3 + (gateID * 2)] = HI_UINT16(gateID);
    }

    serialSendCommadPackage(payload, LENGTH_READ_THRESHOLD_COMMAND);
}



/**
 * @func    writeSnrParameterCommand
 * @brief   write SNR parameter of Radar
 * @param   16 Gate of Radar sensor
 */

void writeSnrParameterCommand(
    u16 motionGate0,
    u16 motionGate1,
    u16 motionGate2,
    u16 motionGate3,
    u16 motionGate4,
    u16 motionGate5,
    u16 motionGate6,
    u16 motionGate7,
    u16 holdingGate0,
    u16 holdingGate1,
    u16 holdingGate2,
    u16 holdingGate3,
    u16 holdingGate4,
    u16 holdingGate5,
    u16 holdingGate6,
    u16 holdingGate7
) {
    // Payload buffer size
    const u16 payloadLength = 2 + (16 * 6); // 2 bytes for command word, 16 gates * 6 bytes per gate

    // Create payload buffer
    u8 payload[payloadLength];

    // Initialize payload with 0
    memset(payload, 0, sizeof(payload));

    payload[0] = LO_UINT16(CMD_WRITE_SNR_PARAMETER_COMMAND);
    payload[1] = HI_UINT16(CMD_WRITE_SNR_PARAMETER_COMMAND);

    // Fill payload buffer
    u8 *ptr = payload + 2;


    // Motion Gates
    u16 motionGates[8] = {motionGate0, motionGate1, motionGate2, motionGate3, motionGate4, motionGate5, motionGate6, motionGate7};
    // Motion Gates
    for (u8 i = 0; i < 8; i++) {
        *ptr++ = i; // Motion Gate ID (0-7)
        *ptr++ = 0x00;
        *ptr++ = LO_UINT16(motionGates[i]);
        *ptr++ = HI_UINT16(motionGates[i]);
        *ptr++ = 0x00; // 00 byte
        *ptr++ = 0x00; // 00 byte
    }

    // Holding Gates
       u16 holdingGates[8] = {holdingGate0, holdingGate1, holdingGate2, holdingGate3, holdingGate4, holdingGate5, holdingGate6, holdingGate7};
    for (u8 i = 0; i < 8; i++) {
        *ptr++ = i + 8; // Holding Gate ID (8-15)
        *ptr++ = 00;
        *ptr++ = LO_UINT16(holdingGates[i]);
        *ptr++ = HI_UINT16(holdingGates[i]);
        *ptr++ = 0x00; // 00 byte
        *ptr++ = 0x00; // 00 byte
    }

    // Send the command package
    serialSendCommadPackage(payload, sizeof(payload));
}


/**
 * @func    readSnrParameterCommand
 * @brief   read SNR parameter of Radar
 * @param   None
 */

void readSnrParameterCommand(void) {
    u8 payload[LENGTH_READ_SNR_PARAMETER_COMMAND];

    payload[0] = LO_UINT16(CMD_READ_SNR_PARAMETER_COMMAND);
    payload[1] = HI_UINT16(CMD_READ_SNR_PARAMETER_COMMAND);

    for (GateID gateID = MotionGate0; gateID <= HoldingGate7; gateID++) {
        payload[2 + (gateID * 2)] = LO_UINT16(gateID);
        payload[3 + (gateID * 2)] = HI_UINT16(gateID);
    }

    serialSendCommadPackage(payload, LENGTH_READ_SNR_PARAMETER_COMMAND);
}


/**
 * @func    autoConfigCommand
 * @brief   Auto config sensitivity for Radar
 * @param   Trigger Threshold Coefficient(dB)
 *           (default 2dB)
 * @param   Hold Threshold Coefficient (dB)
 *           (default 1dB)
 * @param   Scan Time(Second): Time choose scan environment
 *           (min: 120 seconds)
 */
void autoConfigCommand(uint16_t triggerTheshold, uint16_t holdThreshold, uint16_t scanTime){
	u8 payload[LENGTH_AUTO_CONFIG_COMMAND];
	payload[0] = LO_UINT16(CMD_AUTO_CONFIG_COMMAND);
	payload[1] = HI_UINT16(CMD_AUTO_CONFIG_COMMAND);
	payload[2] = LO_UINT16(triggerTheshold);
	payload[3] = HI_UINT16(triggerTheshold);
	payload[4] = LO_UINT16(holdThreshold);
	payload[5] = HI_UINT16(holdThreshold);
	payload[6] = LO_UINT16(scanTime);
	payload[7] = HI_UINT16(scanTime);
	serialSendCommadPackage(payload, LENGTH_AUTO_CONFIG_COMMAND);
}








/***************************************************************************************/





/************************************RADAR CONFIG**************************************
*	If you want to configure for radar, let's follow the below step
*
*	1. startConfigCommand
*	2. Use some Function Command
*	3. endConfigCommand
*
* example:
* 	void autoConfigRadar(void){
* 		startConfigCommand();
* 		autoConfigCommand(2,1,120);
* 		endConfigCommand();
* 	}
*
**************************************************************************************/

/**
 * @func    readFirmwareVersionRadar
 * @brief   read version firmware of Radar
 * @param   None
 */
void readFirmwareVersionRadar(void){
	startConfigCommand();
	readFwRadarCommand();
	endConfigCommand();
}

/**
 * @func    readAllParameterforRadar
 * @brief   read all param of Radar
 * @param   None
 */
void readAllParameterforRadar(void)
{
	startConfigCommand();
	readCommonParameterCommand();
	readThresholdParameterCommand();
	readSnrParameterCommand();
	endConfigCommand();

}




/**
 * @func    resetDefaultParammeterForRadar
 * @brief   reset Default Radar Param
 * @param   None
 */
void resetDefaultParammeterForRadar(void)
{
	startConfigCommand();
	writeGenericParameterCommand(DISTANCE_3M5,0,REPORT_FAST,40,5,10);
	writeThresholdParameterCommand(48, 42, 36, 34, 32, 31, 31, 31,
								   31, 31, 31, 31, 31, 31, 31, 31);

	writeSnrParameterCommand(45, 42, 33, 32, 28, 28, 28, 28,
							 28, 28, 28, 28, 28, 28, 28, 28);
	endConfigCommand();
}




/**
 * @func    distanceConfigForRadar
 * @brief   configure distance for Radar
 * @param   distance 0,7 -> 8,4m
 */
void distanceConfigForRadar(DistanceForRadar distance){
	startConfigCommand();
	writeGenericParameterCommand(distance,0,REPORT_FAST,40,5,10);
	endConfigCommand();
}


/**
 * @func    autoConfigForRadar
 * @brief  	run auto configure
 * @param   time min is 120second
 */
void autoConfigForRadar(uint16_t setTime){
	startConfigCommand();
	autoConfigCommand(2,1,setTime);
	endConfigCommand();
}



/*************************************************************************************/


/*******************************RADAR HANDLER RECIEVE UART MSG************************/
/**
 * @function     : handleDataRx
 *
 * @brief        : Handle data received
 *
 * @parameter    : data
 *
 * @return value : None
 */
void handleDataRx(DEVREPORT_BUFFER_P pCmd, uint8_t length){

	u16 cmdID = pCmd->parameterConfig.cmdid;

	DBG_RADAR_SEND_STR("\nCmdID: ");
	DBG_RADAR_SEND_HEX(cmdID);



	switch(cmdID){
	case CMD_READ_FW_RESPONSE:
	{
		u32 equipmentType = pCmd->fwVersion.equipmentType;
		u16 versionType = pCmd->fwVersion.versionType;
		u16 majorVersion = pCmd->fwVersion.majorVersion;
		u16 minorVersion = pCmd->fwVersion.minorVersion;
		u16 patchVersion = pCmd->fwVersion.patchVersion;

	}
		break;
	case CMD_WRITE_GENERIC_PARAM_RESPONSE:
	{
		u16 ackStatus = pCmd->genericConfig.ackGeneric;

		if(ackStatus == CONFIG_SUCCES){
			DBG_RADAR_SEND_STR("\nackGeneric: CONFIG_SUCCES ");
		}




	}
		break;
	case CMD_READ_GENERIC_PARAM_RESPONSE:
	{
		u16 acStatusRead = pCmd->genericReadConfig.ackGenericRead;
		u32 distanceMin = pCmd->genericReadConfig.detectNearestGate;
		u32 distanceMax = pCmd->genericReadConfig.detectFarthestGate;
		u32 respondSpeed = pCmd->genericReadConfig.respondSpeed;
		u32 triggerRefreshRate = pCmd->genericReadConfig.triggerRefreshRate;
		u32 maintainRefreshRate = pCmd->genericReadConfig.maintainRefreshRate;
		u32 unattendedDelayTime = pCmd->genericReadConfig.unattendedDelayTime;

		DBG_RADAR_SEND_STR("\nacStatusRead: ");
		DBG_RADAR_SEND_HEX(acStatusRead);
		DBG_RADAR_SEND_STR("\ndistanceMin: ");
		DBG_RADAR_SEND_HEX(distanceMin);
		DBG_RADAR_SEND_STR("\ndistanceMax: ");
		DBG_RADAR_SEND_HEX(distanceMax);
		DBG_RADAR_SEND_STR("\nrespondSpeed: ");
		DBG_RADAR_SEND_HEX(respondSpeed);
		DBG_RADAR_SEND_STR("\ntriggerRefreshRate: ");
		DBG_RADAR_SEND_HEX(triggerRefreshRate);
		DBG_RADAR_SEND_STR("\nmaintainRefreshRate: ");
		DBG_RADAR_SEND_HEX(maintainRefreshRate);
		DBG_RADAR_SEND_STR("\nunattendedDelayTime: ");
		DBG_RADAR_SEND_HEX(unattendedDelayTime);

	}
		break;

	case CMD_WRITE_THRESHOLD_RESPONSE:
	{
		u16 ackTheshold = pCmd->thresholdConfig.ackTheshold;
		if(ackTheshold == CONFIG_SUCCES){
			// Write sensitivity success
			DBG_RADAR_SEND_STR("\nackTheshold: CONFIG_SUCCES ");
		}

	}
		break;

	case CMD_READ_THRESHOLD_RESPONSE:
	{
		u16 ackThresholdRead = pCmd->thresholdReadConfig.ackThesholdRead;

		if(ackThresholdRead == CONFIG_SUCCES){
			// Data of all gate
			DBG_RADAR_SEND_STR("\nackThresholdRead: READ_SUCCES ");
			u32 motionGate0 = pCmd->thresholdReadConfig.motionGate0;
			u32 motionGate1 = pCmd->thresholdReadConfig.motionGate1;
			u32 motionGate2 = pCmd->thresholdReadConfig.motionGate2;
			u32 motionGate3 = pCmd->thresholdReadConfig.motionGate3;
			u32 motionGate4 = pCmd->thresholdReadConfig.motionGate4;
			u32 motionGate5 = pCmd->thresholdReadConfig.motionGate5;
			u32 motionGate6 = pCmd->thresholdReadConfig.motionGate6;
			u32 motionGate7 = pCmd->thresholdReadConfig.motionGate7;
			u32 holdingGate0 = pCmd->thresholdReadConfig.holdingGate0;
			u32 holdingGate1 = pCmd->thresholdReadConfig.holdingGate1;
			u32 holdingGate2 = pCmd->thresholdReadConfig.holdingGate2;
			u32 holdingGate3 = pCmd->thresholdReadConfig.holdingGate3;
			u32 holdingGate4 = pCmd->thresholdReadConfig.holdingGate4;
			u32 holdingGate5 = pCmd->thresholdReadConfig.holdingGate5;
			u32 holdingGate6 = pCmd->thresholdReadConfig.holdingGate6;
			u32 holdingGate7 = pCmd->thresholdReadConfig.holdingGate7;
		}
	}
		break;
	case CMD_WRITE_SNR_PARAMETER_RESPONSE:
	{
		u16 ackSNR = pCmd->snrConfig.ackSNR;
		if(ackSNR == CONFIG_SUCCES){
			// Write SNR success
			DBG_RADAR_SEND_STR("\nackSNR: CONFIG_SUCCES ");
		}
	}
		break;
	case CMD_READ_SNR_PARAMETER_RESPONSE:
	{
		u16 ackSnrRead = pCmd->snrReadConfig.ackSNRRead;
		if(ackSnrRead == CONFIG_SUCCES){
			DBG_RADAR_SEND_STR("\nackSnrRead: READ_SUCCES ");
			u32 motionSnrGate0 = pCmd->snrReadConfig.motionGate0;
			u32 motionSnrGate1 = pCmd->snrReadConfig.motionGate1;
			u32 motionSnrGate2 = pCmd->snrReadConfig.motionGate2;
			u32 motionSnrGate3 = pCmd->snrReadConfig.motionGate3;
			u32 motionSnrGate4 = pCmd->snrReadConfig.motionGate4;
			u32 motionSnrGate5 = pCmd->snrReadConfig.motionGate5;
			u32 motionSnrGate6 = pCmd->snrReadConfig.motionGate6;
			u32 motionSnrGate7 = pCmd->snrReadConfig.motionGate7;
			u32 holdingSnrGate0 = pCmd->snrReadConfig.holdingGate0;
			u32 holdingSnrGate1 = pCmd->snrReadConfig.holdingGate1;
			u32 holdingSnrGate2 = pCmd->snrReadConfig.holdingGate2;
			u32 holdingSnrGate3 = pCmd->snrReadConfig.holdingGate3;
			u32 holdingSnrGate4 = pCmd->snrReadConfig.holdingGate4;
			u32 holdingSnrGate5 = pCmd->snrReadConfig.holdingGate5;
			u32 holdingSnrGate6 = pCmd->snrReadConfig.holdingGate6;
			u32 holdingSnrGate7 = pCmd->snrReadConfig.holdingGate7;
		}


	}
		break;
	case CMD_AUTO_CONFIG_RESPONSE:
	{
		u16 ackAutoStatus = pCmd->autoConfig.ackAutoConfig;
		if(ackAutoStatus == CONFIG_SUCCES){
			// Run auto configure success
			DBG_RADAR_SEND_STR("\nackAutoStatus: CONFIG_SUCCES ");
		}
	}
		break;

	case CMD_REPORT_HEADER:
	{
		uint8_t motionReport = pCmd->motionReport.motion;
		uint16_t distanceTarget = pCmd->motionReport.distanceTarget;
		switch(motionReport){
		case NO_TARGET: // Radar unmotion
//			DBG_RADAR_SEND_STR("\nNO_TARGET ");
			break;

		case MOTION_TARGET:	// Radar Motion
		case STATIC_TARGET:
		case MOTION_AND_STATIC_TARGET:

//			DBG_RADAR_SEND_STR("\nMOTION_TARGET ");
			break;

		default:
			break;
		}

	}

		break;

	case CMD_REPORT_AUTO_CONFIG_HEADER:
	{
		uint8_t statusMotion = pCmd->timeFinishAutoConfig.motion;
		uint16_t timeFinish	= pCmd->timeFinishAutoConfig.timeFinish;
		DBG_RADAR_SEND_STR(" ");
		DBG_RADAR_SEND_INT(timeFinish);
	}
		break;
	default:
		break;
	}

}


/* END FILE */
