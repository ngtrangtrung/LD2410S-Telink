/*
 * utilities.c
 *
 *  Created on: Feb 18, 2020
 *      Author: DungTran BK
 */

#include "utilities.h"

/**
 * @func   IsMatchVal
 * @brief  None
 * @param
 * @retval val is exist in *buff or not
 */
inline BOOL IsMatchVal(uint8_t *buff, uint8_t size, uint8_t val)
{
	for(u8 i = 0; i < size; i++) {
        if (buff[i] != val) return FALSE;
    }
    return TRUE;
}

/**
 * @func    __delay_us
 * @brief
 * @param   None
 * @retval  None
 */
inline void __delay_us(uint32_t us)
{
	uint32_t temp_clock_time = 0;
	temp_clock_time = clock_time();
	while((clock_time() - temp_clock_time) < (CLOCK_SYS_CLOCK_MHZ*us));
}

/**
 * @func    __delay_us
 * @brief
 * @param   None
 * @retval  None
 */
void __delay_ms(uint32_t ms)
{
	__delay_us(ms*1000);
}
