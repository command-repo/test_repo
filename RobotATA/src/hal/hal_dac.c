
#include <stddef.h>

#include "common/macros.h"

#include "hal/hal_dac.h"
#include "hal/hal_gpio.h"

#include "hal/CMSIS/device-support/stm32f10x.h"

#define V_REF (float)3.3

void hal_dac_init(hal_dac_t *obj, hal_dac_list_t dac, hal_dac_align_t align,
                    uint16_t code_val, hal_gpio_t *gpio)
{
	ASSERT(&obj != NULL);
	ASSERT(dac < HAL_DAC_END);
	ASSERT(align < HAL_DAC_ALIGN_END);
	ASSERT(code_val < 4096);
	ASSERT(code_val < 256 || align != HAL_DAC_ALIGN_8_R);
	ASSERT(&gpio != NULL);
	ASSERT(gpio->mode == HAL_GPIO_MODE_AN);
	
	uint16_t tmp = 0;
	
	obj->dac = dac;
	obj->align = align;
	obj->val = code_val;
	obj->gpio = gpio;
	
	// enable clock
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	
	if(dac == HAL_DAC_1)
	{
		tmp = DAC->CR;
	}
	else
	{
		tmp = DAC->CR >> 16;
	}
	
	// enable DAC
	tmp |= DAC_CR_EN1;
	
	// software trigger
	tmp |= DAC_CR_TSEL1;
	
	// disable noise / triangle generators
	tmp &= ~DAC_CR_WAVE1;
	
	// disable DMA
	tmp &= ~DAC_CR_DMAEN1;
	
	// enable output buffer
	tmp &= ~DAC_CR_BOFF1;
	
	// disable DAC channel
	if(dac == HAL_DAC_1)
	{
		// clear DAC reg
		DAC->CR &= ~(DAC_CR_EN1 | DAC_CR_TSEL1 | DAC_CR_WAVE1 | DAC_CR_DMAEN1 |
						DAC_CR_BOFF1);
		
		DAC->CR |= tmp;
	}
	else
	{
		// clear DAC reg
		DAC->CR &= ~((DAC_CR_EN1 | DAC_CR_TSEL1 | DAC_CR_WAVE1 | DAC_CR_DMAEN1 |
						DAC_CR_BOFF1) << 16);
		
		DAC->CR |= tmp << 16;
	}
	hal_dac_set_code(obj, obj->val);
}

void hal_dac_set_code(hal_dac_t *obj, uint16_t code_val)
{
	ASSERT(&obj != NULL);
	ASSERT(code_val < 4096);
	ASSERT(code_val < 256 || obj->align != HAL_DAC_ALIGN_8_R);
	
	obj->val = code_val;
	
	if(obj->dac == HAL_DAC_1)
	{
		if(obj->align == HAL_DAC_ALIGN_8_R)
		{
			DAC->DHR8R1 = obj->val;
		}
		else if(obj->align == HAL_DAC_ALIGN_12_R)
		{
			DAC->DHR12R1 = obj->val;
		}
		else
		{
			DAC->DHR12L1 = obj->val;
		}
		DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
	}
	else
	{
		if(obj->align == HAL_DAC_ALIGN_8_R)
		{
			DAC->DHR8R2 = obj->val;
		}
		else if(obj->align == HAL_DAC_ALIGN_12_R)
		{
			DAC->DHR12R2 = obj->val;
		}
		else
		{
			DAC->DHR12L2 = obj->val;
		}
		DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG2;
	}
}

void hal_dac_set_vltg(hal_dac_t *obj, float voltage)
{
	ASSERT(&obj != NULL);
	ASSERT(voltage <= V_REF);
	
	uint16_t code = 0;
	
	if(obj->align == HAL_DAC_ALIGN_8_R)
	{
		code = (uint16_t)((voltage / V_REF) * 255);
	}
	else
	{
		code = (uint16_t)((voltage / V_REF) * 4095);
	}
	
	hal_dac_set_code(obj, code);
}

uint16_t hal_dac_get_code(hal_dac_t *obj)
{
	ASSERT(&obj != NULL);
	
	uint16_t res = 0;
	
	if(obj->dac == HAL_DAC_1)
	{
		res = DAC->DOR1;
	}
	else
	{
		res = DAC->DOR2;
	}
	
	return res;
}
