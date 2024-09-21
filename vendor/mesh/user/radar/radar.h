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

#ifndef RADAR_H_
#define RADAR_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
//#include "config.h"
#include "vendor/common/system_time.h"
#include "uart1_cmd.h"
#include "../utilities.h"

/******************************************************************************/
/*                     PRIVATE TYPES and DEFINITIONS                         */
/******************************************************************************/

#define RADAR_OUT_FILTER_SCAN_INTERVAL         		20     	// us
#define RADAR_PULL_DETECT_TIMEOUT              		10		// HOLD 10ms
#define MIN_TIME_TO_DETECT_RADAR_ACTIVE        		300  	// 300ms
#define RADAR_POWER_ON_TIME_MS                 		5000  	// default value 10s
#define	ONE_SECOND									1000



#define TARGET_DIRECTION_GPIO		            	GPIO_PB3

#define RADAR_EN		    			        	GPIO_PB2
#define TARGET_DIRECTION_GPIO		            	GPIO_PB3
#define TARGET_DIRECTION_BIT		            	3

#define SERIAL_DATA_MAX_LENGHT              		128
#define MAX_SERIAL_PAYLOAD_LEN      128

typedef struct _UART1_RX_STR_{
	u8 DataReceiverStep;
	u8 DataPacketLength;
	u8 Data[SERIAL_DATA_MAX_LENGHT];
	u8 ReceiverByteCount;
}Uart1Frame_t;

typedef enum _RX_STATE_ {
	RX_STATE_START_1,
	RX_STATE_START_2,
	RX_STATE_START_3,
	RX_STATE_START_4,
	RX_STATE_LENGTH_1,
	RX_STATE_LENGTH_2,
	RX_STATE_DATA,
	RX_STATE_END_1,
	RX_STATE_END_2,
	RX_STATE_END_3,
	RX_STATE_END_4,
} RX1_STATE;

typedef enum _UART_STATE_ {
    UART_STATE_IDLE,
    UART_STATE_DATA_RECEIVED,
    UART_STATE_ACK_RECEIVED,
    UART_STATE_NACK_RECEIVED,
    UART_STATE_RX_TIMEOUT,
	UART_STATE_ERROR
} UART1_STATE;

enum{
	UNMOTION,
	MOTION
};

typedef u8 RadarHoldStep_enum;

enum radar_input_signal_t {
	R_SIGNAL_LOW,
	R_SIGNAL_HIGH,
};
typedef u8 radar_input_signal_t;


enum RadarState_enum{
	RADAR_INACTIVE = 0,
	RADAR_ACTIVE   = 1
};
typedef u8 RadarState_enum;


typedef struct{
	u32                		signal_high_t;
	u32                		start_detect_t;
	radar_input_signal_t 	current_signal;
	radar_input_signal_t 	before_signal;
	RadarState_enum      	real_state;
	u32                		pulse_len;
	u32						check_noise;
	u32						current_time;
	u32                		radar_active_last_t;
	u8						stateCurrent;
}radar_params_t;

typedef struct {
	bool actived;
	u32  active_last_t;
}ss_radar_t;


typedef void (*typeRadar_handleChangeState)(RadarState_enum);
typedef void (*typeRadar_testHandleChangeState)(RadarState_enum);
typedef void (*byte_pUart1CallbackFunc2Args) (u8 *, u8);
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/


/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/**
 * @func   sensorInit
 * @brief  Init Param for Radar Sensor
 * @param  [type] : RADAR, ALL
 * @retval None
 */
void sensorInit(void);

/**
 * @func   radarInit
 * @brief  Init Param for Radar Sensor
 * @param  [pUserHandler] : function call to handle when there is a change in motion
 * @retval None
 */
void radarInit(typeRadar_handleChangeState pUserHandler);



/**
 * @func   RADAR_task
 * @brief  Task event handler radar
 * @retval None
 */
void RADAR_task(void);
/**
 * @func    RADAR_getState
 * @brief
 * @param   None
 * @retval  None
 */
RadarState_enum RADAR_getState(void);


/**
 * @brief  This function to process Radar Event
 *
 * @param  None
 *
 * @return [None]
 */
void statusRadarEventProcess(void);

/**
 * @func   Sensor_HandlePirState
 * @brief
 * @param  None
 * @retval None
 */
void Sensor_HandleRadarState(RadarState_enum state);


/**
 * @func    RADAR_setActiveLastTime
 * @brief
 * @param   None
 * @retval  None
 */
void RADAR_setActiveLastTime(void);


void uartRadarInit(void);

void serial_proc(void);






/**
 * @func    startConfigCommand
 * @brief   call it when start config radar
 * @param   None
 */

void startConfigCommand(void);

/**
 * @func    endConfigCommand
 * @brief   call it when end config radar
 * @param   None
 */

void endConfigCommand(void);




/**
 * @func    writeSerialNumberCommand
 * @brief   Sends a command to write a serial number with a payload
 * 			containing the length (2 bytes) and the serial number.
 * @param   serialNumber: pointer to the buffer containing the serial number
 *          lengthSerialNumber: length of the serial number
 */
void writeSerialNumberCommand(u8* serialNumber, u16 lengthSerialNumber);



/**
 * @func    readSerialNumberCommand
 * @brief   read serial Number of Radar
 * @param   None
 */
void readSerialNumberCommand(void);

/**
 * @func    readFwRadarCommand
 * @brief   read Firmware of Radar
 * @param   None
 */
void readFwRadarCommand(void);

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
);

/**
 * @func    readCommonParameterCommand
 * @brief   read Distance Motion and Holding, delay time, response Speed
 * @param   None
 */

void readCommonParameterCommand(void);

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
);

/**
 * @func    readThresholdParameterCommand
 * @brief   read Threshold parameter of Radar
 * @param   None
 */
void readThresholdParameterCommand(void);


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
);


/**
 * @func    readSnrParameterCommand
 * @brief   read SNR parameter of Radar
 * @param   None
 */

void readSnrParameterCommand(void);



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
void autoConfigCommand(uint16_t triggerTheshold, uint16_t holdThreshold, uint16_t scanTime);

/**
 * @func    readFirmwareVersionRadar
 * @brief   read version firmware of Radar
 * @param   None
 */
void readFirmwareVersionRadar(void);

/**
 * @func    readAllParameterforRadar
 * @brief   read all param of Radar
 * @param   None
 */
void readAllParameterforRadar(void);

#endif /* RADAR_H_ */
