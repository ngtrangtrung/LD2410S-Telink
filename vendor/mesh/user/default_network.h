/*
 * default_network.h
 *
 *  Created on: 2022
 *      Author: Dung Tran
 */

typedef struct {
	BOOL en;
	u32  time_len_s;
	u32  st_time_s;
}m_default_nw_t;

#define OPEN_DF_NW_TIMEOUT         1800 // 30 minutes
#define OPEN_DF_NW_DEFAULT_TIME    60   // s

u8 default_network_is_present(void);
void set_tmp_keys(u8 tmp);
void del_tmp_keys();
void proc_default_network();
void set_default_network_manual(BOOL en, u16 time_s);
void check_and_del_default_power_on(void);
