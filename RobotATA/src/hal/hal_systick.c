
#include "common/macros.h"

#include "hal/hal_systick.h"
#include "hal/hal_rcc.h"

#include "hal/CMSIS/device-support/stm32f10x.h"

static volatile uint32_t systick_cnt = 0;

void hal_systick_init(void)
{
	uint32_t systick_freq = hal_rcc_get_freq(HAL_RCC_SRC_AHB);
	SysTick_Config(systick_freq / 1000);
}

uint32_t hal_systick_get_ms(void)
{
	return systick_cnt;
}

uint32_t hal_systick_get_past(uint32_t start)
{
	uint32_t res = 0;
	res = systick_cnt - start;
	return res;
}

void hal_systick_delay(uint32_t ms)
{
	uint32_t last_time = hal_systick_get_ms();
	while(hal_systick_get_past(last_time) <= ms)
	{
		asm("nop");
	}
}

void SysTick_Handler(void)
{
	systick_cnt++;
}
