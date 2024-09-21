/*
 * in_out.h
 *
 *  Created on: Oct 22, 2020
 *      Author: DungTran BK
 */

#ifndef OUTPUT_H_
#define OUTPUT_H_

typedef int (*typeIo_initSendConfigParamsCallbackFunc)(u8, u8, u16, u8, u8);

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
enum {
    IN_TOGGLE_SW,       // contact closed – ON, contact opened – OFF
	IN_MOMENTARY_SW,    // contact is press and release
    IN_ANY_CHANGE_SW,   // device changes status when switch changes status
    SW_UNKNOWN
};

enum {
	OUT_NORMAL,         // Input ON -> Output ON, Input OFF -> Output OFF
	OUT_DELAY_ON,
	OUT_DELAY_OFF,
	OUT_AUTO_ON,
	OUT_AUTO_OFF,
	OUT_UNKNOWN
};

#define MAX_DELAY_TIME_S     10800

typedef struct {
	u8   in_mode;
	u8   out_mode;
	u16  out_auto_trans_t_s;
	BOOL out_map_input;
}io_params_t;

typedef struct {
	u32  start_t_ms;
	BOOL expired_ts;
}in_out_init_t;


typedef struct {
	u8  input;
	u8  output;
	u16 delay_t_s;
}vd_sw_config_t;

/******************************************************************************/
/*                             EXPORT FUNCTIONS                               */
/******************************************************************************/

void io_init(void);
void io_loop_task(void);
BOOL io_handle_input_btn_st(u8 idx, u8 evt, BOOL no_change);
int io_handle_cmd_sig_on_off(mesh_cmd_g_onoff_set_t *p_set, int par_len, int force_last, int idx, u8 retransaction, st_pub_list_t *pub_list);
void io_setup_delay_trans_params(u8 idx, BOOL en);

int io_config_sw_main_params(u8 idx, u8* par, u8 par_len);
int io_config_sw_set_map_unmap_input(u8 idx, u8* par);

io_params_t* io_get_main_params(u8 input_channel);

BOOL io_set_input_mode(u8 idx, u8 new_val);
void io_get_config_params(u8 *dst_par);

void io_callback_init(typeIo_initSendConfigParamsCallbackFunc callback_Func);

#endif /* IN_OUT_H_ */
