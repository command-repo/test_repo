
#include <stdint.h>
#include <stddef.h>

#include "common/macros.h"

#include "hal/hal_gpio.h"

#include "hal/CMSIS/device-support/stm32f10x.h"

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

static uint32_t const rcc_list[HAL_GPIO_PORT_END] =
{
	RCC_APB2ENR_IOPAEN,
	RCC_APB2ENR_IOPBEN,
	RCC_APB2ENR_IOPCEN,
	RCC_APB2ENR_IOPDEN,
	RCC_APB2ENR_IOPEEN,
#if defined (STM32F10X_HD) || defined (STM32F10X_XL)
	RCC_APB2ENR_IOPFEN,
	RCC_APB2ENR_IOPGEN
#else
	0,
	0
#endif
};

void hal_gpio_init(hal_gpio_t *obj, hal_gpio_port_t port, hal_gpio_pin_t pin,
					hal_gpio_mode_t mode, bool state)
{
	ASSERT(&obj != NULL);
	ASSERT(port < HAL_GPIO_PORT_END);
	ASSERT(pin < HAL_GPIO_PIN_END);
	ASSERT(mode < HAL_GPIO_MODE_END);
	
	obj->port = port;
	obj->pin = pin;
	obj->mode = mode;
	
	GPIO_TypeDef *gpio = gpio_list[obj->port];
	
	// Enable clock
	RCC->APB2ENR |= rcc_list[obj->port];
	
	if(obj->pin < HAL_GPIO_PIN_8)
	{
		// Switch pin to input
		gpio->CRL &= ~(GPIO_CRL_MODE0 << (obj->pin * 4));
		
		// Switch pin to analog mode
		gpio->CRL &= ~(GPIO_CRL_CNF0 << (obj->pin * 4));
	}
	else
	{
		// Switch pin to input
		gpio->CRH &= ~(GPIO_CRL_MODE0 << ((obj->pin - HAL_GPIO_PIN_8) * 4));
		
		// Switch pin to analog mode
		gpio->CRH &= ~(GPIO_CRL_CNF0 << ((obj->pin - HAL_GPIO_PIN_8) * 4));
	}
	
	switch(obj->mode)
	{
		case HAL_GPIO_MODE_DO:
			if(obj->pin < HAL_GPIO_PIN_8)
			{
				gpio->CRL |= GPIO_CRL_MODE0 << (obj->pin * 4);
			}
			else
			{
				gpio->CRH |= GPIO_CRL_MODE0 << ((obj->pin - HAL_GPIO_PIN_8) * 4);
			}
			break;
		
		case HAL_GPIO_MODE_OD:
			if(obj->pin < HAL_GPIO_PIN_8)
			{
				gpio->CRL |= GPIO_CRL_CNF0_0 << (obj->pin * 4);
				gpio->CRL |= GPIO_CRL_MODE0 << (obj->pin * 4);
			}
			else
			{
				gpio->CRH |= GPIO_CRL_CNF0_0 << ((obj->pin - HAL_GPIO_PIN_8) * 4);
				gpio->CRH |= GPIO_CRL_MODE0 << ((obj->pin - HAL_GPIO_PIN_8) * 4);
			}
			break;
		
		case HAL_GPIO_MODE_DI:
			if(obj->pin < HAL_GPIO_PIN_8)
			{
				gpio->CRL |= GPIO_CRL_CNF0_1 << (obj->pin * 4);
			}
			else
			{
				gpio->CRH |= GPIO_CRL_CNF0_1 << ((obj->pin - HAL_GPIO_PIN_8) * 4);
			}
			
			if(state)
			{
				// Pull-up
				gpio->ODR |= GPIO_ODR_ODR0 << obj->pin;
			}
			else
			{
				// Pull-down
				gpio->ODR &= ~(GPIO_ODR_ODR0 << obj->pin);
			}
			break;
		
		case HAL_GPIO_MODE_AN:
			// Analog mode has already enabled
			break;
		
		case HAL_GPIO_MODE_AF:
			if(obj->pin < HAL_GPIO_PIN_8)
			{
				gpio->CRL |= GPIO_CRL_CNF0_1 << (obj->pin * 4);
				gpio->CRL |= GPIO_CRL_MODE0 << (obj->pin * 4);
			}
			else
			{
				gpio->CRH |= GPIO_CRL_CNF0_1 << ((obj->pin - HAL_GPIO_PIN_8) * 4);
				gpio->CRH |= GPIO_CRL_MODE0 << ((obj->pin - HAL_GPIO_PIN_8) * 4);
			}
			break;
	}
	
	// Setup default state
	if(obj->mode == HAL_GPIO_MODE_DO || obj->mode == HAL_GPIO_MODE_OD)
	{
		if(state)
		{
			gpio->BSRR |= (1 << obj->pin);
		}
		else
		{
			gpio->BRR |= (1 << obj->pin);
		}
	}
}

void hal_gpio_set(hal_gpio_t *obj, bool state)
{
	ASSERT(&obj != NULL);
	ASSERT(obj->mode == HAL_GPIO_MODE_DO || obj->mode == HAL_GPIO_MODE_OD);
	
	if(state)
	{
		gpio_list[obj->port]->BSRR |= (1 << obj->pin);
	}
	else
	{
		gpio_list[obj->port]->BRR |= (1 << obj->pin);
	}
}

void hal_gpio_toggle(hal_gpio_t *obj)
{
	ASSERT(&obj != NULL);
	
	gpio_list[obj->port]->ODR ^= (1 << obj->pin);
}

bool hal_gpio_get(hal_gpio_t *obj)
{
	ASSERT(&obj != NULL);
	ASSERT(obj->mode != HAL_GPIO_MODE_AN && obj->mode != HAL_GPIO_MODE_AF);
	
	bool res;
	
	if(gpio_list[obj->port]->IDR & (1 << obj->pin))
	{
		res = true;
	}
	else
	{
		res = false;
	}
	
	return res;
}
