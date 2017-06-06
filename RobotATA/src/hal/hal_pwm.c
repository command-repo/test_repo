
#include <stddef.h>

#include "common/macros.h"

#include "hal/hal_pwm.h"
#include "hal/hal_tim.h"
#include "hal/hal_rcc.h"
#include "hal/hal_gpio.h"

#include "hal/CMSIS/device-support/stm32f10x.h"

#define MAX_PWM_RESOL   0xFFFF

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

static volatile uint32_t *rcc_bus_list[HAL_TIM_END] =
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

static hal_pwm_ch_t const pwm_max_ch_list[HAL_TIM_END] =
{
	4,
	4,
	4,
	4,
	4,
	0,
	0,
	4,
	2,
	1,
	1,
	2,
	1,
	1
	
};

static uint32_t const ccmr_reg_list[HAL_PWM_CH_END][HAL_PWM_MODE_END] = 
{
	{TIM_CCMR1_OC1M, TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1},
	{TIM_CCMR1_OC2M, TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1},
	{TIM_CCMR2_OC3M, TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1},
	{TIM_CCMR2_OC4M, TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1}
};

static GPIO_TypeDef *const gpio_list[HAL_GPIO_PORT_END] =
{
	GPIOA,
	GPIOB,
	GPIOC,
	GPIOD,
	GPIOE,
	GPIOF,
	GPIOG
};

static void hal_pwm_gpio_init(hal_tim_list_t tim, hal_gpio_t *gpio);

static void hal_pwm_calc_freq(hal_tim_list_t tim, uint32_t freq,
                                uint16_t *presc, uint16_t *reload);

static void hal_pwm_calc_duty_cycle(hal_tim_list_t tim, uint8_t duty_cycle,
                                    uint16_t *ccr);

void hal_pwm_init(hal_pwm_t *obj, hal_tim_list_t tim, hal_pwm_ch_t ch,
                    hal_pwm_mode_t mode, hal_gpio_t *gpio)
{
	ASSERT(&obj != NULL);
	ASSERT(tim < HAL_TIM_END);
	ASSERT(ch < pwm_max_ch_list[tim]);
	ASSERT(mode < HAL_PWM_MODE_END);
	ASSERT(&gpio != NULL);
	ASSERT(gpio->mode == HAL_GPIO_MODE_AF);
	
	obj->tim = tim;
	obj->ch = ch;
	obj->mode = mode;
	obj->freq = 0;
	obj->duty_cycle = 0;
	obj->gpio = gpio;
	
	*rcc_bus_list[obj->tim] |= rcc_list[obj->tim];
	
	hal_pwm_gpio_init(obj->tim, obj->gpio);
	
	// enable PWM output
	tim_list[obj->tim]->CCER |= TIM_CCER_CC1E << (obj->ch * 4);
	
	switch(obj->ch)
	{
		case HAL_PWM_CH_1:
			tim_list[obj->tim]->CCMR1 &= ~TIM_CCMR1_OC1M;
			tim_list[obj->tim]->CCMR1 |= ccmr_reg_list[obj->ch][obj->mode];
			break;
		
		case HAL_PWM_CH_2:
			tim_list[obj->tim]->CCMR1 &= ~TIM_CCMR1_OC2M;
			tim_list[obj->tim]->CCMR1 |= ccmr_reg_list[obj->ch][obj->mode];
			break;
		
		case HAL_PWM_CH_3:
			tim_list[obj->tim]->CCMR2 &= ~TIM_CCMR2_OC3M;
			tim_list[obj->tim]->CCMR2 |= ccmr_reg_list[obj->ch][obj->mode];
			break;
		
		case HAL_PWM_CH_4:
			tim_list[obj->tim]->CCMR2 &= ~TIM_CCMR2_OC4M;
			tim_list[obj->tim]->CCMR2 |= ccmr_reg_list[obj->ch][obj->mode];
			break;
	}
	
	// Enable output for advanced timers
	// RM0008 chapter 14.4.18 (page 358)
	if(obj->tim == HAL_TIM_1 || obj->tim == HAL_TIM_8)
	{
		tim_list[obj->tim]->BDTR |= TIM_BDTR_MOE;
	}
}

void hal_pwm_set_freq(hal_pwm_t *obj, uint32_t freq)
{
	ASSERT(&obj != NULL);
	ASSERT(freq > 0);
	
	uint16_t presc = 0;
	uint16_t reload = 0;
	obj->freq = freq;
	hal_pwm_calc_freq(obj->tim, obj->freq, &presc, &reload);
	
	tim_list[obj->tim]->PSC = presc;
	tim_list[obj->tim]->ARR = reload;
}

void hal_pwm_set_dc(hal_pwm_t *obj, uint8_t duty_cycle)
{
	ASSERT(&obj != NULL);
	ASSERT(duty_cycle <= 100);
	
	uint16_t ccr = 0;
	obj->duty_cycle = duty_cycle;
	
	hal_pwm_calc_duty_cycle(obj->tim, obj->duty_cycle, &ccr);
	
	switch(obj->ch)
	{
		case HAL_PWM_CH_1:
			tim_list[obj->tim]->CCR1 = ccr;
			break;
		
		case HAL_PWM_CH_2:
			tim_list[obj->tim]->CCR2 = ccr;
			break;
		
		case HAL_PWM_CH_3:
			tim_list[obj->tim]->CCR3 = ccr;
			break;
		
		case HAL_PWM_CH_4:
			tim_list[obj->tim]->CCR4 = ccr;
			break;
	}
}

void hal_pwm_start(hal_pwm_t *obj)
{
	ASSERT(&obj != NULL);
	ASSERT(obj->freq > 0);
	
	tim_list[obj->tim]->CR1 &= ~TIM_CR1_OPM;
	tim_list[obj->tim]->CR1 |= TIM_CR1_CEN;
}

void hal_pwm_stop(hal_pwm_t *obj)
{
	ASSERT(&obj != NULL);
	
	tim_list[obj->tim]->CR1 &= ~TIM_CR1_CEN;
}

static void hal_pwm_gpio_init(hal_tim_list_t tim, hal_gpio_t *gpio)
{
	// Push-pull type
	if(gpio->pin < HAL_GPIO_PIN_8)
	{
		gpio_list[gpio->port]->CRL |= GPIO_CRL_CNF0_1 << (gpio->pin * 4);
	}
	else
	{
		gpio_list[gpio->port]->CRH |=
						GPIO_CRL_CNF0_1 << ((gpio->pin - HAL_GPIO_PIN_8) * 4);
	}
}

static void hal_pwm_calc_freq(hal_tim_list_t tim, uint32_t freq,
                                uint16_t *presc, uint16_t *reload)
{
	uint32_t tmp_presc = 0;
	uint32_t tmp_reload = 0;
	uint32_t clk_freq = hal_rcc_get_freq(rcc_src_list[tim]);
	if(clk_freq != hal_rcc_get_freq(HAL_RCC_SRC_AHB))
	{
		// if APBx prescaller more then 1, TIMx prescaller multiplies by 2
		clk_freq *= 2;
	}
	
	// increase timer clock frequency or use timer with higher clock frequency
	// to pass this assert
	ASSERT(freq < clk_freq);
	
	tmp_reload = (clk_freq + (freq / 2)) / freq;
	if(tmp_reload <= MAX_PWM_RESOL)
	{
		tmp_presc = 1;
	}
	else
	{
		tmp_presc = ((tmp_reload + (MAX_PWM_RESOL / 2)) / MAX_PWM_RESOL) + 1;
		tmp_reload /= tmp_presc;
	}
	
	// minimum value for correct duty cycle setup (in percent)
	// increase timer clock frequency or use timer with higher clock frequency
	// to pass this assert
	ASSERT(tmp_reload > 100);
	
	ASSERT(tmp_presc <= MAX_PWM_RESOL);
	ASSERT(tmp_reload <= MAX_PWM_RESOL);
	
	*presc = (uint16_t)(tmp_presc - 1);
	*reload = (uint16_t)(tmp_reload - 1);
}

static void hal_pwm_calc_duty_cycle(hal_tim_list_t tim, uint8_t duty_cycle,
                                    uint16_t *ccr)
{
	*ccr = 0;
	if(duty_cycle > 0)
	{
		*ccr = (tim_list[tim]->ARR * duty_cycle) / 100;
		*ccr += 1;
	}
}
