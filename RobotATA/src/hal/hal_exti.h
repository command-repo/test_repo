
#ifndef HAL_EXTI_H
#define HAL_EXTI_H

#include <stdint.h>

#include "hal/hal_gpio.h"

typedef enum
{
	HAL_EXTI_TRIGGER_RISING,
	HAL_EXTI_TRIGGER_FALLING,
	HAL_EXTI_TRIGGER_BOTH,
	HAL_EXTI_TRIGGER_END
} hal_exti_trigger_t;

typedef struct hal_exti_t
{
	hal_gpio_t          *gpio;
	hal_exti_trigger_t  trigger;
	void                (*clbk)(struct hal_exti_t *obj);
} hal_exti_t;

void hal_exti_init(hal_exti_t *obj, hal_gpio_t *gpio,
                    hal_exti_trigger_t trigger);

void hal_exti_set_clbk(hal_exti_t *obj, void (*clbk)(hal_exti_t *obj));

void hal_exti_on(hal_exti_t *obj);

void hal_exti_off(hal_exti_t *obj);

#endif
