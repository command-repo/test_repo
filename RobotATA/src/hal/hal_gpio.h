
#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include <stdbool.h>

typedef enum
{
	HAL_GPIO_PORT_0,    /*!< GPIOA */
	HAL_GPIO_PORT_1,    /*!< GPIOB */
	HAL_GPIO_PORT_2,    /*!< GPIOC */
	HAL_GPIO_PORT_3,    /*!< GPIOD */
	HAL_GPIO_PORT_4,    /*!< GPIOE */
	HAL_GPIO_PORT_5,    /*!< GPIOF */
	HAL_GPIO_PORT_6,    /*!< GPIOG */
	HAL_GPIO_PORT_END
} hal_gpio_port_t;

typedef enum
{
	HAL_GPIO_PIN_0,
	HAL_GPIO_PIN_1,
	HAL_GPIO_PIN_2,
	HAL_GPIO_PIN_3,
	HAL_GPIO_PIN_4,
	HAL_GPIO_PIN_5,
	HAL_GPIO_PIN_6,
	HAL_GPIO_PIN_7,
	HAL_GPIO_PIN_8,
	HAL_GPIO_PIN_9,
	HAL_GPIO_PIN_10,
	HAL_GPIO_PIN_11,
	HAL_GPIO_PIN_12,
	HAL_GPIO_PIN_13,
	HAL_GPIO_PIN_14,
	HAL_GPIO_PIN_15,
	HAL_GPIO_PIN_END
} hal_gpio_pin_t;

typedef enum
{
	HAL_GPIO_MODE_DO,
	HAL_GPIO_MODE_OD,
	HAL_GPIO_MODE_DI,
	HAL_GPIO_MODE_AN,
	HAL_GPIO_MODE_AF,
	HAL_GPIO_MODE_END
} hal_gpio_mode_t;

typedef struct
{
	hal_gpio_port_t port;
	hal_gpio_pin_t  pin;
	hal_gpio_mode_t mode;
} hal_gpio_t;

void hal_gpio_init(hal_gpio_t *obj, hal_gpio_port_t port, hal_gpio_pin_t pin,
					hal_gpio_mode_t mode, bool state);

void hal_gpio_set(hal_gpio_t *obj, bool state);

void hal_gpio_toggle(hal_gpio_t *obj);

bool hal_gpio_get(hal_gpio_t *obj);

#endif
