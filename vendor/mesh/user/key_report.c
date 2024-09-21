/*
 * key_report.c
 *
 *  Created on: Feb 27, 2020
 *      Author: DungTran BK
 */

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "../../../proj/tl_common.h"
#include "../../common/mesh_node.h"
#include "../../../proj_lib/sig_mesh/app_mesh.h"
#include "../../common/vendor_model.h"
#include "execution_scene.h"
#include "config_board.h"
#include "led.h"
#include "key_report.h"

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/


/******************************************************************************/
/*                        EXPORT FUNCTIONS DECLERATION                        */
/******************************************************************************/
/**
 * @func   get_ele_addr_base_btn_idx
 * @brief
 * @param  Button Index
 * @retval Element Address
 */
static uint16_t get_ele_addr_base_btn_idx(uint8_t idx)
{
	if(idx < NUMBER_INPUT){
		return (ele_adr_primary + idx);
	}
	return ADR_UNASSIGNED;
}
/**
 * @func   key_report_send
 * @brief
 * @param  None
 * @retval None
 */
void key_report_send(u8 idx, u8 key_code)
{
	uint16_t ele_addr;
	vd_rc_key_report_t key_report = {0};

	key_report.code = key_code;
	ele_addr = get_ele_addr_base_btn_idx(idx);
	if(ele_addr != ADR_UNASSIGNED){
		execution_scene_active(idx, key_code);
		// Change Time Response
		mesh_tx_cmd_rsp(
				VD_RC_KEY_REPORT,(u8 *)&key_report, 1, ele_addr, GATEWAY_UNICAST_ADDR, 0, 0
			);
#if EXECUTION_SCENE_RETRY_TIME > 1
		execution_scene_active(idx, key_code);
#endif
	}
}

//End File
