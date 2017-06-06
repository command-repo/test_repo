
#include <stdint.h>

#include "common/macros.h"

#include "hal/hal_rcc.h"

#include "hal/CMSIS/device-support/stm32f10x.h"

// Converting CFGR[7:4] HPRE value to prescaller
static uint32_t const ahb_presc_list[16] =
{
	[0b0000] = 0,   // AHB prescaller 1
	[0b1000] = 2,   // AHB prescaller 2
	[0b1001] = 4,   // AHB prescaller 4
	[0b1010] = 8,   // AHB prescaller 8
	[0b1011] = 16,  // AHB prescaller 16
	[0b1100] = 64,  // AHB prescaller 64
	[0b1101] = 128, // AHB prescaller 128
	[0b1110] = 256, // AHB prescaller 256
	[0b1111] = 512  // AHB prescaller 512
};

// Converting CFGR[12:10] PPRE1 value to prescaller
static uint32_t const apb1_presc_list[8] =
{
	[0b000] = 0,   // APB1 prescaller 1
	[0b100] = 2,   // APB1 prescaller 2
	[0b101] = 4,   // APB1 prescaller 4
	[0b110] = 8,   // APB1 prescaller 8
	[0b111] = 16,  // APB1 prescaller 16
};

// Converting CFGR[15:13] PPRE2 value to prescaller
static uint32_t const apb2_presc_list[8] =
{
	[0b000] = 0,   // APB2 prescaller 1
	[0b100] = 2,   // APB2 prescaller 2
	[0b101] = 4,   // APB2 prescaller 4
	[0b110] = 8,   // APB2 prescaller 8
	[0b111] = 16,  // APB2 prescaller 16
};

//bool hal_rcc_init(void)
//{
//	bool res = true;
//	
//	return true;
//}

uint32_t hal_rcc_get_freq(hal_rcc_src_t src)
{
	ASSERT(src < HAL_RCC_SRC_END);
	
	uint32_t res = 0;
	uint32_t tmp = 0;
	uint32_t ahb_presc = 0;
	uint32_t apb1_presc = 0;
	uint32_t apb2_presc = 0;
	
	tmp = (RCC->CFGR & RCC_CFGR_HPRE) >> 4;
	ahb_presc = ahb_presc_list[tmp];
	
	tmp = (RCC->CFGR & RCC_CFGR_PPRE1) >> 10;
	apb1_presc = apb1_presc_list[tmp];
	
	tmp = (RCC->CFGR & RCC_CFGR_PPRE2) >> 13;
	apb2_presc = apb2_presc_list[tmp];
	
	SystemCoreClockUpdate();
	res = SystemCoreClock;
	switch(src)
	{
		case HAL_RCC_SRC_SYSCLK:
			break;
		
		case HAL_RCC_SRC_AHB:
			if(ahb_presc > 0)
			{
				res /= ahb_presc;
			}
			break;
		
		case HAL_RCC_SRC_APB1:
			if(ahb_presc > 0)
			{
				res /= ahb_presc;
			}
			if(apb1_presc > 0)
			{
				res /= apb1_presc;
			}
			break;
		
		case HAL_RCC_SRC_APB2:
			if(ahb_presc > 0)
			{
				res /= ahb_presc;
			}
			if(apb2_presc > 0)
			{
				res /= apb2_presc;
			}
			break;
	}
	
	return res;
}


void hal_rcc_reset(void)
{
	NVIC_SystemReset();
}
