
#ifndef DRV_DI_H
#define DRV_DI_H

#include <stdbool.h>
#include <stdint.h>

#include "hal/hal_gpio.h"

typedef struct drv_di_t
{
	hal_gpio_t  *gpio;
	bool        state;
	uint16_t    threshold;
	uint16_t    cnt;
	uint32_t    time;
	void        (*clbk)(struct drv_di_t *obj);
} drv_di_t;

void drv_di_init(drv_di_t *obj, hal_gpio_t *gpio, uint16_t threshold,
                 bool default_state);

void drv_di_set_clbk(drv_di_t *obj, void (*clbk)(drv_di_t *obj));

void drv_di_run(drv_di_t *obj);

bool drv_di_get_state(drv_di_t *obj);

uint16_t drv_di_get_threshold(drv_di_t *obj);

#endif
