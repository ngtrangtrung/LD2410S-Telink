/********************************************************************************************************
 * @file     main.c 
 *
 * @brief    for TLSR chips
 *
 * @author	 telink
 * @date     Sep. 30, 2010
 *
 * @par      Copyright (c) 2010, Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *           
 *			 The information contained herein is confidential and proprietary property of Telink 
 * 		     Semiconductor (Shanghai) Co., Ltd. and is available under the terms 
 *			 of Commercial License Agreement between Telink Semiconductor (Shanghai) 
 *			 Co., Ltd. and the licensee in separate contract or the terms described here-in. 
 *           This heading MUST NOT be removed from this file.
 *
 * 			 Licensees are granted free, non-transferable use of the information in this 
 *			 file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided. 
 *           
 *******************************************************************************************************/
#include "proj/tl_common.h"
#include "proj/mcu/watchdog_i.h"
#include "vendor/common/user_config.h"
#include "proj_lib/rf_drv.h"
#include "proj_lib/pm.h"
#include "proj_lib/ble/blt_config.h"
#include "proj_lib/ble/ll/ll.h"
#include "user/user_irq.h"


#include "user/debug.h"
#ifdef MAIN_DBG_EN
#define DBG_MAIN_SEND_STR(x)   Dbg_sendString((s8*)x)
#define DBG_MAIN_SEND_INT(x)   Dbg_sendInt(x)
#define DBG_MAIN_SEND_HEX(x)   Dbg_sendHex(x)
#else
#define DBG_MAIN_SEND_STR(x)
#define DBG_MAIN_SEND_INT(x)
#define DBG_MAIN_SEND_HEX(x)
#endif


extern void user_init();
extern void main_loop ();

#if (HCI_ACCESS==HCI_USE_UART)
#include "../../proj/drivers/uart.h"
extern my_fifo_t hci_rx_fifo;

u16 uart_tx_irq=0, uart_rx_irq=0;

_attribute_ram_code_ void irq_uart_handle()
{
	unsigned char irqS = reg_dma_rx_rdy0;
	if(irqS & FLD_DMA_CHN_UART_RX)	//rx
	{
		uart_rx_irq++;
		reg_dma_rx_rdy0 = FLD_DMA_CHN_UART_RX;
		u8* w = hci_rx_fifo.p + (hci_rx_fifo.wptr & (hci_rx_fifo.num-1)) * hci_rx_fifo.size;
		if(w[0]!=0)
		{
			my_fifo_next(&hci_rx_fifo);
			u8* p = hci_rx_fifo.p + (hci_rx_fifo.wptr & (hci_rx_fifo.num-1)) * hci_rx_fifo.size;
			reg_dma0_addr = (u16)((u32)p);
		}
	}

	if(irqS & FLD_DMA_CHN_UART_TX)	//tx
	{
		uart_tx_irq++;
		reg_dma_rx_rdy0 = FLD_DMA_CHN_UART_TX;
	}
}
#endif


_attribute_ram_code_ void irq_handler(void)
{
	#if DUAL_MODE_ADAPT_EN
	if(rf_mode == RF_MODE_ZIGBEE){
		irq_zigbee_sdk_handler();
	}else
	#endif
	{
		irq_blt_sdk_handler ();  //ble irq proc
	}

#if IRQ_TIMER1_ENABLE
	irq_timer_handle();
#endif

#if (HCI_ACCESS==HCI_USE_UART)
	irq_uart_handle();
#endif

#if	IRQ_GPIO_ENABLE
	irq_gpio_handle();
#endif
}

FLASH_ADDRESS_DEFINE;
#if(MCU_CORE_TYPE == MCU_CORE_8269)
int main (void) {
	FLASH_ADDRESS_CONFIG;
	cpu_wakeup_init();

	clock_init();
	set_tick_per_us(CLOCK_SYS_CLOCK_HZ/1000000);

	gpio_init();

	rf_drv_init(CRYSTAL_TYPE);

	user_init ();

    irq_enable();

	while (1) {
#if (MODULE_WATCHDOG_ENABLE)
		wd_clear(); //clear watch dog
#endif
		main_loop ();
	}
}
#elif((MCU_CORE_TYPE == MCU_CORE_8258) || (MCU_CORE_TYPE == MCU_CORE_8278))
_attribute_ram_code_ int main (void)    //must run in ramcode
{
	FLASH_ADDRESS_CONFIG;
#if PINGPONG_OTA_DISABLE
    ota_fw_check_over_write();  // must at first for main_
#endif
	blc_pm_select_internal_32k_crystal();
	cpu_wakeup_init();

	int deepRetWakeUp = pm_is_MCU_deepRetentionWakeup();  //MCU deep retention wakeUp

	rf_drv_init(RF_MODE_BLE_1M);

	gpio_init( !deepRetWakeUp );  //analog resistance will keep available in deepSleep mode, so no need initialize again

#if (CLOCK_SYS_CLOCK_HZ == 16000000)
	clock_init(SYS_CLK_16M_Crystal);
#elif (CLOCK_SYS_CLOCK_HZ == 24000000)
	clock_init(SYS_CLK_24M_Crystal);
#elif (CLOCK_SYS_CLOCK_HZ == 32000000)
	clock_init(SYS_CLK_32M_Crystal);
#elif (CLOCK_SYS_CLOCK_HZ == 48000000)
	clock_init(SYS_CLK_48M_Crystal);
#endif


#if	(PM_DEEPSLEEP_RETENTION_ENABLE)
		if( pm_is_MCU_deepRetentionWakeup() ){
			user_init_deepRetn ();
		}
		else
#endif
	{
		user_init();
	}

    irq_enable();

	while (1) {
#if (MODULE_WATCHDOG_ENABLE)
		wd_clear(); //clear watch dog
#endif
		main_loop ();
	}
}
#endif
