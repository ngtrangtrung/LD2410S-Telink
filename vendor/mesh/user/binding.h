/*
 * binding.h
 *
 *  Created on: Mar 18, 2022
 *      Author: DungTranBK
 */

#ifndef BINDING_H_
#define BINDING_H_

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include "../../../proj/tl_common.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

typedef struct {
	u8  st;
	u8  cnt_same;
	u16 op;
}control_binding_t;

typedef struct{
	u16 ele_adr;
	u8  en;
	u16 group_adr;
}cfg_binding_format_t;

typedef struct{
	u16 ele_adr;
}get_binding_format_t;

typedef struct {
	u8  en;
	u16 group_dst;
}binding_para_t;

typedef struct {
	u8  op;
	u8  st;
	u16 ele_adr;
	u8  en;
	u16 group_adr;
}binding_msg_response_t;

enum {
	BINDING_SUCCESS      = 0,
	GROUP_NOT_SET        = 1,
	GROUP_INVALID        = 2,
	MODEL_IDX_INVALID    = 3,
	BINDING_ERR_UNKNOWN  = 4,
	BINDING_DISABLE      = 5
};

#define FLASH_SIZE_BINDING_PARAMS     4050
#define BLOCK_SIZE_BINDING_PARAMS     sizeof(binding_para_st)

extern binding_para_t    binding_para_st[ELE_CNT];
extern control_binding_t control_binding_st[ELE_CNT];


/******************************************************************************/
/*                             EXPORT FUNCTIONS                               */
/******************************************************************************/

uint16_t get_group_binding_adr(int idx);
void binding_init(void);
void binding_handle_nw_message(u8 type, u8* par);

#endif /* BINDING_H_ */
