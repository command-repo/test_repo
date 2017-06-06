
#include <stddef.h>

#include "common/macros.h"

#include "hal/hal_exti.h"
#include "hal/hal_gpio.h"

#include "hal/CMSIS/device-support/stm32f10x.h"

#define EXTI_IRQ_PRIORITY 3
#define GPIO_AF_15_EVENTOUT         0x0F

static IRQn_Type const exti_irq_list[HAL_GPIO_PIN_END] =
{
	EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn,
	EXTI3_IRQn, EXTI4_IRQn, EXTI9_5_IRQn,
	EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn,
	EXTI9_5_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn,
	EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn,
	EXTI15_10_IRQn
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

static uint32_t const exti_port_list[HAL_GPIO_PORT_END] =
{
	AFIO_EXTICR1_EXTI0_PA, AFIO_EXTICR1_EXTI0_PB, AFIO_EXTICR1_EXTI0_PC,
	AFIO_EXTICR1_EXTI0_PD, AFIO_EXTICR1_EXTI0_PE, AFIO_EXTICR1_EXTI0_PF,
	AFIO_EXTICR1_EXTI0_PG
};

static uint8_t const exti_src_offset_list[HAL_GPIO_PIN_END] =
{
	0, 4, 8,
	0, 4, 8,
	0, 4, 8,
	0, 4, 8,
};

static hal_exti_t *obj_list[HAL_GPIO_PIN_END];

static void hal_exti_handler(hal_gpio_pin_t pin);

void hal_exti_init(hal_exti_t *obj, hal_gpio_t *gpio,
                    hal_exti_trigger_t trigger)
{
	ASSERT(&obj != NULL);
	ASSERT(&gpio != NULL);
	ASSERT(trigger < HAL_EXTI_TRIGGER_END);
	ASSERT(gpio->mode == HAL_GPIO_MODE_DI);
	
	obj->gpio = gpio;
	obj->trigger = trigger;
	obj->clbk = 0;
	
	uint32_t line_bit = (1 << obj->gpio->pin);
	uint8_t exti_src = 0;
	
	// Enable clock
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	
	// Setup EXTI line source
	exti_src = exti_src_offset_list[obj->gpio->pin];
	AFIO->EXTICR[obj->gpio->pin / 2] &= ~(0x0F << exti_src);
	AFIO->EXTICR[obj->gpio->pin / 2] |= exti_port_list[obj->gpio->port] << exti_src;
	
	// Setup EXTI mask regs
	EXTI->IMR |= line_bit;
	EXTI->EMR &= ~line_bit;
	
	// Clear rising and falling triggers
	EXTI->RTSR &= ~line_bit;
	EXTI->FTSR &= ~line_bit;
	
	// Setup EXTI trigger
	if(obj->trigger == HAL_EXTI_TRIGGER_RISING)
	{
		EXTI->RTSR |= line_bit;
	}
	else if(obj->trigger == HAL_EXTI_TRIGGER_FALLING)
	{
		EXTI->FTSR |= line_bit;
	}
	else
	{
		EXTI->RTSR |= line_bit;
		EXTI->FTSR |= line_bit;
	}
	
	obj_list[obj->gpio->pin] = obj;
	
	NVIC_ClearPendingIRQ(exti_irq_list[obj->gpio->pin]);
	NVIC_SetPriority(exti_irq_list[obj->gpio->pin], EXTI_IRQ_PRIORITY);
	NVIC_EnableIRQ(exti_irq_list[obj->gpio->pin]);
}

void hal_exti_set_clbk(hal_exti_t *obj, void (*clbk)(hal_exti_t *obj))
{
	ASSERT(&obj != NULL);
	
	NVIC_DisableIRQ(exti_irq_list[obj->gpio->pin]);
	obj->clbk = clbk;
	NVIC_EnableIRQ(exti_irq_list[obj->gpio->pin]);
}

void hal_exti_on(hal_exti_t *obj)
{
	ASSERT(&obj != NULL);
	
	uint32_t line_bit = (1 << obj->gpio->pin);
	
	// Clear EXTI line pending bits
	EXTI->PR |= line_bit;
	
	// Setup EXTI line configuration
	EXTI->IMR |= line_bit;
	
	NVIC_ClearPendingIRQ(exti_irq_list[obj->gpio->pin]);
}

void hal_exti_off(hal_exti_t *obj)
{
	ASSERT(&obj != NULL);
	
	uint32_t line_bit = (1 << obj->gpio->pin);
	
	// Clear EXTI line configuration
	EXTI->IMR &= ~line_bit;
}

static void hal_exti_handler(hal_gpio_pin_t pin)
{
	uint32_t line_bit = (1 << pin);
	
	// Clear EXTI line pending bits
	EXTI->PR |= line_bit;
	
	if(obj_list[pin]->clbk != NULL)
	{
		obj_list[pin]->clbk(obj_list[pin]);
	}
}

void EXTI0_IRQHandler(void)
{
	NVIC_ClearPendingIRQ(exti_irq_list[HAL_GPIO_PIN_0]);
	hal_exti_handler(HAL_GPIO_PIN_0);
}

void EXTI1_IRQHandler(void)
{
	NVIC_ClearPendingIRQ(exti_irq_list[HAL_GPIO_PIN_1]);
	hal_exti_handler(HAL_GPIO_PIN_1);
}

void EXTI2_IRQHandler(void)
{
	NVIC_ClearPendingIRQ(exti_irq_list[HAL_GPIO_PIN_2]);
	hal_exti_handler(HAL_GPIO_PIN_2);
}

void EXTI3_IRQHandler(void)
{
	NVIC_ClearPendingIRQ(exti_irq_list[HAL_GPIO_PIN_3]);
	hal_exti_handler(HAL_GPIO_PIN_3);
}

void EXTI4_IRQHandler(void)
{
	NVIC_ClearPendingIRQ(exti_irq_list[HAL_GPIO_PIN_4]);
	hal_exti_handler(HAL_GPIO_PIN_4);
}

void EXTI9_5_IRQHandler(void)
{
	NVIC_ClearPendingIRQ(exti_irq_list[HAL_GPIO_PIN_5]);
	uint32_t pending_bit = EXTI->PR;
	
	uint32_t line_bit = (1 << HAL_GPIO_PIN_5);
	if(pending_bit & line_bit)
	{
		hal_exti_handler(HAL_GPIO_PIN_5);
		return;
	}
	
	line_bit = (1 << HAL_GPIO_PIN_6);
	if(pending_bit & line_bit)
	{
		hal_exti_handler(HAL_GPIO_PIN_6);
		return;
	}
	
	line_bit = (1 << HAL_GPIO_PIN_7);
	if(pending_bit & line_bit)
	{
		hal_exti_handler(HAL_GPIO_PIN_7);
		return;
	}
	
	line_bit = (1 << HAL_GPIO_PIN_8);
	if(pending_bit & line_bit)
	{
		hal_exti_handler(HAL_GPIO_PIN_8);
		return;
	}
	
	line_bit = (1 << HAL_GPIO_PIN_9);
	if(pending_bit & line_bit)
	{
		hal_exti_handler(HAL_GPIO_PIN_9);
		return;
	}
}

void EXTI15_10_IRQHandler(void)
{
	NVIC_ClearPendingIRQ(exti_irq_list[HAL_GPIO_PIN_10]);
	uint32_t pending_bit = EXTI->PR;
	
	uint32_t line_bit = (1 << HAL_GPIO_PIN_10);
	if(pending_bit & line_bit)
	{
		hal_exti_handler(HAL_GPIO_PIN_10);
		return;
	}
	
	line_bit = (1 << HAL_GPIO_PIN_11);
	if(pending_bit & line_bit)
	{
		hal_exti_handler(HAL_GPIO_PIN_11);
		return;
	}
	
	line_bit = (1 << HAL_GPIO_PIN_12);
	if(pending_bit & line_bit)
	{
		hal_exti_handler(HAL_GPIO_PIN_12);
		return;
	}
	
	line_bit = (1 << HAL_GPIO_PIN_13);
	if(pending_bit & line_bit)
	{
		hal_exti_handler(HAL_GPIO_PIN_13);
		return;
	}
	
	line_bit = (1 << HAL_GPIO_PIN_14);
	if(pending_bit & line_bit)
	{
		hal_exti_handler(HAL_GPIO_PIN_14);
		return;
	}
	
	line_bit = (1 << HAL_GPIO_PIN_15);
	if(pending_bit & line_bit)
	{
		hal_exti_handler(HAL_GPIO_PIN_15);
		return;
	}
}
