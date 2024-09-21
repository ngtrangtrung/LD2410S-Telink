/*
 * sw_config.h
 *
 *  Created on: Mar 28, 2022
 *      Author: DungTranBK
 */

#ifndef SW_CONFIG_H_
#define SW_CONFIG_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
typedef struct {
	u8 op;
	u8 en;
}en_dis_group_rsp_t;

/******************************************************************************/
/*                             EXPORT FUNCTIONS                               */
/******************************************************************************/

BOOL sw_get_en_disable_group_default(int model_idx);
void sw_response_enable_disable_group_default(int idx);
void sw_set_enable_disable_group_default(int idx, BOOL en);
void sw_config_init(void);

#endif /* SW_CONFIG_H_ */
