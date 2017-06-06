
#ifndef HAL_DAC_H
#define HAL_DAC_H

#include <stdint.h>

#include "hal/hal_gpio.h"

typedef enum
{
	HAL_DAC_1,
	HAL_DAC_2,
	HAL_DAC_END
} hal_dac_list_t;

typedef enum
{
	HAL_DAC_ALIGN_8_R,
	HAL_DAC_ALIGN_12_R,
	HAL_DAC_ALIGN_12_L,
	HAL_DAC_ALIGN_END
} hal_dac_align_t;

typedef struct
{
	hal_dac_list_t  dac;
	hal_dac_align_t align;
	uint16_t        val;
	hal_gpio_t      *gpio;
} hal_dac_t;

void hal_dac_init(hal_dac_t *obj, hal_dac_list_t dac, hal_dac_align_t align,
                    uint16_t code_val, hal_gpio_t *gpio);

void hal_dac_set_code(hal_dac_t *obj, uint16_t code_val);

void hal_dac_set_vltg(hal_dac_t *obj, float voltage);

uint16_t hal_dac_get_code(hal_dac_t *obj);

#endif
