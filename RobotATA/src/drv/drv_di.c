
#include <stddef.h>

#include "common/macros.h"
#include "drv/drv_di.h"
#include "hal/hal_gpio.h"
#include "hal/hal_systick.h"

static void drv_di_jitter_filter(drv_di_t *obj);

void drv_di_init(drv_di_t *obj, hal_gpio_t *gpio, uint16_t threshold, bool default_state)
{
	ASSERT(&obj != NULL);
	ASSERT(&gpio != NULL && gpio->mode == HAL_GPIO_MODE_DI);
	
	obj->gpio = gpio;
	obj->threshold = threshold;
	obj->state = default_state;
	obj->cnt = 0;
	obj->time = 0;
	obj->clbk = NULL;
}

void drv_di_set_clbk(drv_di_t *obj, void (*clbk)(drv_di_t *obj))
{
	ASSERT(&obj != NULL);
	
	obj->clbk = clbk;
}

void drv_di_run(drv_di_t *obj)
{
	ASSERT(&obj != NULL);
	
	if(hal_systick_get_past(obj->time) >= 1)
	{
		bool tmp_state = obj->state;
		
		drv_di_jitter_filter(obj);
		if((tmp_state != obj->state) && (obj->clbk != NULL))
		{
			obj->clbk(obj);
		}
		
		obj->time = hal_systick_get_ms();
	}
}

bool drv_di_get_state(drv_di_t *obj)
{
	ASSERT(&obj != NULL);
	
	return obj->state;
}

uint16_t drv_di_get_threshold(drv_di_t *obj)
{
	ASSERT(&obj != NULL);
	
	return obj->threshold;
}

static void drv_di_jitter_filter(drv_di_t *obj)
{
	ASSERT(&obj != NULL);
	
	if(hal_gpio_get(obj->gpio))
	{
		if(obj->cnt < obj->threshold)
		{
			obj->cnt++;
		}
		else
		{
			obj->state = true;
		}
	}
	else
	{
		if(obj->cnt > 0)
		{
			obj->cnt--;
		}
		else
		{
			obj->state = false;
		}
	}
}
