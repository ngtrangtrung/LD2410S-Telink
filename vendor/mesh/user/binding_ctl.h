/*
 * binding_ctl.h
 *
 *  Created on: Mar 18, 2022
 *      Author: DungTranBK
 */

#ifndef BINDING_CTL_H_
#define BINDING_CTL_H_
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

/******************************************************************************/
/*                       EXPORT TYPE AND DEFINITION                           */
/******************************************************************************/
#define CONTROL_BDG_SAME_ST_CNT_MAX       0x3

extern u8 scene_active_arr[ELE_CNT];
extern u8 force_control_binding[ELE_CNT];
extern u8 state_of_group_binding[LIGHT_CNT];

/******************************************************************************/
/*                              EXPORT FUNCTION                               */
/******************************************************************************/
int  handle_mesh_cmd_sig_g_on_off_set(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);
void binding_control_onoff(u16 model_idx, BOOL state, BOOL update_en);
int  func_handle_control_message(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par, u16 op);
void binding_control(u8 model_idx, BOOL status);

#endif /* BINDING_CTL_H_ */
