/*******************************************************************************
 *
 * Copyright (c) 2019
 * Lumi, JSC.
 * All Rights Reserved
 *
 *
 * Description: Include file for application
 *
 * Author: HoangNH
 *
 * Last Changed By:  $Author: HoangNH $
 * Revision:         $Revision: 2.0 $
 * Last Changed:     $Date: 8/8/2019 $
 *
 ******************************************************************************/
#ifndef _SERIAL_API_H_
#define _SERIAL_API_H_
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "proj/common/types.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/


/*! @brief Start of frame Header */
#define FRAME_HEADER_START_BYTE1			0xFD
#define FRAME_HEADER_START_BYTE2			0xFC
#define FRAME_HEADER_START_BYTE3			0xFB
#define FRAME_HEADER_START_BYTE4			0xFA



/*! @brief End of frame */
#define FRAME_END_BYTE1						0x04
#define FRAME_END_BYTE2						0x03
#define FRAME_END_BYTE3						0x02
#define FRAME_END_BYTE4						0x01


/*! @brief Start of frame Header for REPORT DATA*/
#define FRAME_HEADER_REPORT_START_BYTE1		0xF4
#define FRAME_HEADER_REPORT_START_BYTE2		0xF3
#define FRAME_HEADER_REPORT_START_BYTE3		0xF2
#define FRAME_HEADER_REPORT_START_BYTE4		0xF1


#define FRAME_HEADER_REPORT_DATA			0x6E
#define FRAME_LENGTH_REPORT_DATA			0x05
#define FRAME_END_REPORT_DATA				0x62

/*! @brief End of frame */
#define FRAME_REPORT_END_BYTE1				0xF8
#define FRAME_REPORT_END_BYTE2				0xF7
#define FRAME_REPORT_END_BYTE3				0xF6
#define FRAME_REPORT_END_BYTE4				0xF5

/*! @brief Enable config cmd */
#define LENGTH_ENABLE_CONFIG				0x0004
#define CMD_ENABLE_CONFIG					0x00FF
#define CMD_ENABLE_VALUE					0x0002

/*! @brief End config cmd */
#define LENGTH_END_CONFIG					0x0002
#define CMD_END_CONFIG						0x00FE






/*! @brief Write Serial number command */
#define LENGTH_WRITE_SERIAL_NUMBER			0x000C
#define CMD_WRITE_SERIAL_NUMBER				0x0010
#define CMD_WRITE_SERIAL_RESPONSE			0x0110



/*! @brief Read Serial number command */
#define LENGTH_READ_SERIAL_NUMBER			0x0002
#define CMD_READ_SERIAL_NUMBER				0x0011
#define CMD_READ_SERIAL_RESPONSE			0x0111


/*! @brief Read FW Radar command */
#define LENGTH_READ_FW_RADAR				0x0002
#define CMD_READ_FW_RADAR					0x0000
#define CMD_READ_FW_RESPONSE				0x0100


/*! @brief Write generic parameter commands */
#define LENGTH_WRITE_GENERIC_PARAM_COMMAND	0x0026
#define CMD_WRITE_GENERIC_PARAM_COMMAND		0x0070
#define CMD_WRITE_GENERIC_PARAM_RESPONSE	0x0170



/*! @brief Read common parameter commands */
#define LENGTH_READ_GENERIC_PARAM_COMMAND	0x000E
#define CMD_READ_GENERIC_PARAM_COMMAND		0x0071
#define CMD_READ_GENERIC_PARAM_RESPONSE		0x0171

/*! @brief write threshold parameter commands */
#define LENGTH_WRITE_THRESHOLD_COMMAND		0x0062
#define CMD_WRITE_THRESHOLD_COMMAND			0x0072
#define CMD_WRITE_THRESHOLD_RESPONSE		0x0172

/*! @brief Read threshold parameter commands */
#define LENGTH_READ_THRESHOLD_COMMAND		0x0022
#define CMD_READ_THRESHOLD_COMMAND			0x0073
#define CMD_READ_THRESHOLD_RESPONSE			0x0173



/*! @brief Write SNR parameter commands */
#define LENGTH_WRITE_SNR_PARAMETER_COMMAND	0x0062
#define CMD_WRITE_SNR_PARAMETER_COMMAND		0x0076
#define CMD_WRITE_SNR_PARAMETER_RESPONSE	0x0176

/*! @brief Read SNR parameter commands */
#define LENGTH_READ_SNR_PARAMETER_COMMAND	0x0022
#define CMD_READ_SNR_PARAMETER_COMMAND		0x0077
#define CMD_READ_SNR_PARAMETER_RESPONSE		0x0177

/*! @brief Auto configure commands */
#define LENGTH_AUTO_CONFIG_COMMAND			0x0008
#define CMD_AUTO_CONFIG_COMMAND				0x0009
#define CMD_AUTO_CONFIG_RESPONSE			0x0109


#define	CMD_REPORT_HEADER					0xAA
#define	CMD_REPORT_AUTO_CONFIG_HEADER		0xAB


/*! @brief Finish Auto configure commands */
#define LENGTH_AUTO_CONFIG_FINISH_COMMAND	0x0022
#define AUTO_CONFIG_FINISH_VALUE			0x0000




#define HI_UINT16(a) 						(((a) >> 8) & 0xFF)
#define LO_UINT16(a) 						((a) & 0xFF)
#define BUILD_U16(lo, hi)			( (unsigned short)((((hi) & 0x00FF) << 8) + ((lo) & 0x00FF)) )



/*! @brief Max distance gate and hold time radar */
typedef enum {
	DETECT_FARTHEST_DISTANCE_GATE 			= 0x05, // 1~16
	DETECT_NEAREST_DISTANCE_GATE  			= 0x0A, // 0~16
	RESPONSE_SPEED                			= 0x0B,  // 5 Normal/ 10 (Fast)
	TRIGGER_REFRESH_RATE          			= 0x02, // 0.5~8 Hz (0.5 step)
	MAINTAIN_REFRESH_RATE         			= 0x0C, // 0.5~8 Hz (0.5 step)
	UNMANNED_DELAY_TIME           			= 0x06, // 10~120 s
} WriteGenericParam;




typedef enum {
    MotionGate0 = 0,
    MotionGate1,
    MotionGate2,
    MotionGate3,
    MotionGate4,
    MotionGate5,
    MotionGate6,
    MotionGate7,
    HoldingGate0,
    HoldingGate1,
    HoldingGate2,
    HoldingGate3,
    HoldingGate4,
    HoldingGate5,
    HoldingGate6,
    HoldingGate7
} GateID;




typedef enum{
	DISTACE_70_CM 	= 1,
	DISTANCE_1M4,
	DISTANCE_2M1,
	DISTANCE_2M8,
	DISTANCE_3M5,
	DISTANCE_4M2,
	DISTANCE_4M9,
	DISTANCE_5M6,
	DISTANCE_6M3,
	DISTANCE_7M,
	DISTANCE_7M8,
	DISTANCE_8M4,
}DistanceForRadar;



typedef enum{
	REPORT_NORMAL 	= 5,
	REPORT_FAST		= 10,
}SpeedReport;


/*! @brief Set Serial port baud rate */
enum{
	SERIAL_BAUD_9600		= 1,
	SERIAL_BAUD_19200,
	SERIAL_BAUD_38400,
	SERIAL_BAUD_57600,
	SERIAL_BAUD_115200,
	SERIAL_BAUD_230400,
	SERIAL_BAUD_256000,					// default: 256000
	SERIAL_BAUD_460800,
}BaurdRate;


enum{
	NO_TARGET								= 0x00,
	MOTION_TARGET							= 0x01,
	STATIC_TARGET							= 0x02,
	MOTION_AND_STATIC_TARGET				= 0x03,
};

/*! @brief Start of frame Header */
typedef struct{
	u8 length[2];
	u8 cmdId[2];
	u8 cmdValue[2];
}SerialStartFrame_t;

/*! @brief End config cmd */
typedef struct{
	u8 length[2];
	u8 cmdId[2];
}SerialEndFrame_t;

/*! @brief Max distance gate and hold time radar */
typedef struct{
	u8 length[2];
	u8 cmdId[2];
	u8 maxMoveGateId[2];
	u8 maxMoveGatePar[4];
	u8 maxStaticGateId[2];
	u8 maxStaticGatePar[4];
	u8 holdTimeId[2];
	u8 holdTimePar[4];
}SerialConfigMaxDisAndHoldTime_t;


/*! @brief Range Gate Sensitivity Configuration cmd */
typedef struct{
	u8 length[2];
	u8 cmdId[2];
	u8 distanceGateId[2];
	u8 distanceGatePar[4];
	u8 maxMoveGateId[2];
	u8 maxMoveGatePar[4];
	u8 maxStaticGateId[2];
	u8 maxStaticGatePar[4];
}SerialConfigSensitivity_t;

/*! @brief Set Serial port baud rate */

typedef struct{
	u8 length[2];
	u8 cmdId[2];
	u8 baudRateIdx[2];
}SerialConfigBaudRate_t;

/*! @brief Radar restore all config default */
typedef struct{
	u8 length[2];
	u8 cmdId[2];
}SerialRestore_t;

/*! @brief Radar Reset module */
typedef struct{
	u8 length[2];
	u8 cmdId[2];
}SerialRestart_t;

/*! @brief Bluetooth Setting */
typedef struct{
	u8 length[2];
	u8 cmdId[2];
	u8 cmdValue[2];
}SerialBLTSetting_t;

typedef union {
  SerialStartFrame_t 					StartFrame_u;
  SerialEndFrame_t 						EndFrame_u;
  SerialConfigMaxDisAndHoldTime_t		ConfigDisGate_u;
  SerialConfigSensitivity_t				ConfigSensitivity_u;
  SerialConfigBaudRate_t				ConfigBaudRate_u;
  SerialRestore_t						RadarRestorePar_u;
  SerialRestart_t						RadarRestart_u;
  SerialBLTSetting_t					RadarBLTSet_u;
} serial_receive_buffer_t, *serial_receive_buffer_p;

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

#endif /* _SERIAL_API_H_ */
