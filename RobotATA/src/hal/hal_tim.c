
#include <stddef.h>

#include "common/macros.h"

#include "hal/hal_tim.h"
#include "hal/hal_rcc.h"

#include "hal/CMSIS/device-support/stm32f10x.h"

#define TIM_IRQ_PRIORITY    4
#define MAX_TIM_RESOL   0xFFFF

static IRQn_Type const tim_irq_list[HAL_TIM_END] =
{
	TIM1_CC_IRQn,
	TIM2_IRQn,
	TIM3_IRQn,
#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL)
	TIM4_IRQn,
#else
	0,
#endif
#if defined (STM32F10X_HD) || defined (STM32F10X_CL) || \
	defined (STM32F10X_HD_VL) || defined (STM32F10X_XL)
	TIM5_IRQn,
#else
	0,
#endif
#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || \
	defined (STM32F10X_HD_VL)
	TIM6_DAC_IRQn,
	TIM7_IRQn,
#elif defined (STM32F10X_HD) || defined (STM32F10X_XL) || \
		defined (STM32F10X_CL)
	TIM6_IRQn,
	TIM7_IRQn,
#else
	0,
	0,
#endif
#if defined (STM32F10X_HD) || defined (STM32F10X_XL)
	TIM8_CC_IRQn,
#else
	0,
#endif
#if defined (STM32F10X_XL)
	TIM1_BRK_TIM9_IRQn,
	TIM1_UP_TIM10_IRQn,
	TIM1_TRG_COM_TIM11_IRQn
#else
	0,
	0,
	0
#endif
};

static TIM_TypeDef *const tim_list[HAL_TIM_END] =
{
	TIM1,
	TIM2,
	TIM3,
	TIM4,
	TIM5,
	TIM6,
	TIM7,
	TIM8,
	TIM9,
	TIM10,
	TIM11,
	TIM12,
	TIM13,
	TIM14
};

static uint32_t const rcc_list[HAL_TIM_END] =
{
	RCC_APB2ENR_TIM1EN,
	RCC_APB1ENR_TIM2EN,
	RCC_APB1ENR_TIM3EN,
#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL)
	RCC_APB1ENR_TIM4EN,
#else
	0,
#endif
#if defined (STM32F10X_HD) || defined (STM32F10X_CL) || defined (STM32F10X_HD_VL)
	RCC_APB1ENR_TIM5EN,
#else
	0,
#endif
#if defined (STM32F10X_HD) || defined  (STM32F10X_CL) || \
	defined (STM32F10X_LD_VL) || defined  (STM32F10X_MD_VL) || \
	defined  (STM32F10X_HD_VL)
	RCC_APB1ENR_TIM6EN,
	RCC_APB1ENR_TIM7EN,
#else
	0,
	0,
#endif
#if defined (STM32F10X_HD) || defined (STM32F10X_XL)
	RCC_APB2ENR_TIM8EN,
#else
	0,
#endif
#if defined (STM32F10X_XL)
	RCC_APB2ENR_TIM9EN,
	RCC_APB2ENR_TIM10EN,
	RCC_APB2ENR_TIM11EN,
#else
	0,
	0,
	0,
#endif
#if defined (STM32F10X_HD_VL) || defined (STM32F10X_XL)
	RCC_APB1ENR_TIM12EN,
	RCC_APB1ENR_TIM13EN,
	RCC_APB1ENR_TIM14EN,
#else
	0,
	0,
	0
#endif
};

static volatile uint32_t *const rcc_bus_list[HAL_TIM_END] =
{
	&RCC->APB2ENR,
	&RCC->APB1ENR,
	&RCC->APB1ENR,
	&RCC->APB1ENR,
	&RCC->APB1ENR,
	&RCC->APB1ENR,
	&RCC->APB1ENR,
	&RCC->APB2ENR,
	&RCC->APB2ENR,
	&RCC->APB2ENR,
	&RCC->APB2ENR,
	&RCC->APB1ENR,
	&RCC->APB1ENR,
	&RCC->APB1ENR,
};

static hal_rcc_src_t const rcc_src_list[HAL_TIM_END] =
{
	HAL_RCC_SRC_APB2,
	HAL_RCC_SRC_APB1,
	HAL_RCC_SRC_APB1,
	HAL_RCC_SRC_APB1,
	HAL_RCC_SRC_APB1,
	HAL_RCC_SRC_APB1,
	HAL_RCC_SRC_APB1,
	HAL_RCC_SRC_APB2,
	HAL_RCC_SRC_APB2,
	HAL_RCC_SRC_APB2,
	HAL_RCC_SRC_APB2,
	HAL_RCC_SRC_APB1,
	HAL_RCC_SRC_APB1,
	HAL_RCC_SRC_APB1,
};

static hal_tim_t *obj_list[HAL_TIM_END];

static void hal_tim_calc(hal_tim_list_t tim, uint32_t us, uint16_t *presc,
                            uint16_t *reload);

static void hal_tim_handler(hal_tim_list_t tim);

void hal_tim_init(hal_tim_t *obj, hal_tim_list_t tim)
{
	ASSERT(&obj != NULL);
	ASSERT(tim < HAL_TIM_END);
	
	obj->tim = tim;
	obj->us = 0;
	obj->clbk = 0;
	
	*rcc_bus_list[obj->tim] |= rcc_list[obj->tim];
	
	// enable interrupt
	tim_list[obj->tim]->DIER |= TIM_DIER_UIE;
	
	obj_list[obj->tim] = obj;
	
	NVIC_SetPriority(tim_irq_list[obj->tim], TIM_IRQ_PRIORITY);
	NVIC_EnableIRQ(tim_irq_list[obj->tim]);
}

void hal_tim_set_clbk(hal_tim_t *obj, void (*clbk)(hal_tim_t *obj))
{
	ASSERT(&obj != NULL);
	
	NVIC_DisableIRQ(tim_irq_list[obj->tim]);
	obj->clbk = clbk;
	NVIC_EnableIRQ(tim_irq_list[obj->tim]);
}

void hal_tim_set_us(hal_tim_t *obj, uint32_t us)
{
	ASSERT(&obj != NULL);
	ASSERT(us > 0);
	
	uint16_t presc = 0;
	uint16_t reload = 0;
	obj->us = us;
	hal_tim_calc(obj->tim, obj->us, &presc, &reload);
	
	tim_list[obj->tim]->PSC = presc;
	tim_list[obj->tim]->ARR = reload;
}

void hal_tim_start_once(hal_tim_t *obj)
{
	ASSERT(&obj != NULL);
	ASSERT(obj->us > 0);
	
	tim_list[obj->tim]->CR1 |= TIM_CR1_OPM;
	tim_list[obj->tim]->CR1 |= TIM_CR1_CEN;
}

void hal_tim_start_cyclic(hal_tim_t *obj)
{
	ASSERT(&obj != NULL);
	ASSERT(obj->us > 0);
	
	tim_list[obj->tim]->CR1 &= ~TIM_CR1_OPM;
	tim_list[obj->tim]->CR1 |= TIM_CR1_CEN;
}

void hal_tim_stop(hal_tim_t *obj)
{
	ASSERT(&obj != NULL);
	
	tim_list[obj->tim]->CR1 &= ~TIM_CR1_CEN;
}

static void hal_tim_calc(hal_tim_list_t tim, uint32_t us, uint16_t *presc,
                            uint16_t *reload)
{
	uint32_t tmp_presc = 0;
	uint32_t tmp_reload = 0;
	uint32_t clk_freq = hal_rcc_get_freq(rcc_src_list[tim]);
	if(clk_freq != hal_rcc_get_freq(HAL_RCC_SRC_AHB))
	{
		// if APBx prescaller no equal to 1, TIMx prescaller multiplies by 2
		clk_freq *= 2;
	}
	
	tmp_reload = us * (clk_freq / 1000000);
	if(tmp_reload <= MAX_TIM_RESOL)
	{
		tmp_presc = 1;
	}
	else
	{
		tmp_presc = ((tmp_reload + (MAX_TIM_RESOL / 2)) / MAX_TIM_RESOL) + 1;
		tmp_reload /= tmp_presc;
	}
	
	ASSERT(tmp_presc <= MAX_TIM_RESOL);
	ASSERT(tmp_reload <= MAX_TIM_RESOL);
	
	*presc = (uint16_t)(tmp_presc - 1);
	*reload = (uint16_t)(tmp_reload - 1);
}

static void hal_tim_handler(hal_tim_list_t tim)
{
	if((tim_list[tim]->DIER & TIM_DIER_UIE) &&
		(tim_list[tim]->SR & TIM_SR_UIF))
	{
		tim_list[tim]->SR &= ~TIM_SR_UIF;
		
		if(obj_list[tim]->clbk != NULL)
		{
			obj_list[tim]->clbk(obj_list[tim]);
		}
	}
	else if((tim_list[tim]->DIER & TIM_DIER_CC1IE) &&
			(tim_list[tim]->SR & TIM_SR_CC1IF))
	{
		tim_list[tim]->SR &= ~TIM_SR_CC1IF;
		
		if(obj_list[tim]->clbk != NULL)
		{
			obj_list[tim]->clbk(obj_list[tim]);
		}
	}
	else if((tim_list[tim]->DIER & TIM_DIER_CC2IE) &&
			(tim_list[tim]->SR & TIM_SR_CC2IF))
	{
		tim_list[tim]->SR &= ~TIM_SR_CC2IF;
		
		if(obj_list[tim]->clbk != NULL)
		{
			obj_list[tim]->clbk(obj_list[tim]);
		}
	}
	else if((tim_list[tim]->DIER & TIM_DIER_CC3IE) &&
			(tim_list[tim]->SR & TIM_SR_CC3IF))
	{
		tim_list[tim]->SR &= ~TIM_SR_CC3IF;
		
		if(obj_list[tim]->clbk != NULL)
		{
			obj_list[tim]->clbk(obj_list[tim]);
		}
	}
	else if((tim_list[tim]->DIER & TIM_DIER_CC4IE) &&
			(tim_list[tim]->SR & TIM_SR_CC4IF))
	{
		tim_list[tim]->SR &= ~TIM_SR_CC4IF;
		
		if(obj_list[tim]->clbk != NULL)
		{
			obj_list[tim]->clbk(obj_list[tim]);
		}
	}
}

void TIM1_CC_IRQHandler(void)
{
	hal_tim_handler(HAL_TIM_1);
}

void TIM2_IRQHandler(void)
{
	hal_tim_handler(HAL_TIM_2);
}

void TIM3_IRQHandler(void)
{
	hal_tim_handler(HAL_TIM_3);
}

void TIM4_IRQHandler(void)
{
	hal_tim_handler(HAL_TIM_4);
}

//void TIM5_IRQHandler(void)
//{
//	hal_tim_handler(HAL_TIM_5);
//}

void TIM6_DAC_IRQHandler(void)
{
	hal_tim_handler(HAL_TIM_6);
}

void TIM7_IRQHandler(void)
{
	hal_tim_handler(HAL_TIM_7);
}
