
#ifndef HAL_PWM_H
#define HAL_PWM_H

#include <stdint.h>
#include <stdbool.h>

#include "hal/hal_tim.h"
#include "hal/hal_gpio.h"

typedef enum
{
	HAL_PWM_CH_1,
	HAL_PWM_CH_2,
	HAL_PWM_CH_3,
	HAL_PWM_CH_4,
	HAL_PWM_CH_END
} hal_pwm_ch_t;

typedef enum
{
	HAL_PWM_MODE_INVERTED,
	HAL_PWM_MODE_NONINVERTED,
	HAL_PWM_MODE_END
} hal_pwm_mode_t;

typedef struct
{
	hal_tim_list_t  tim;
	hal_pwm_ch_t    ch;
	hal_pwm_mode_t  mode;
	uint32_t        freq;
	uint8_t         duty_cycle;
	hal_gpio_t      *gpio;
} hal_pwm_t;

void hal_pwm_init(hal_pwm_t *obj, hal_tim_list_t tim, hal_pwm_ch_t ch,
                    hal_pwm_mode_t mode, hal_gpio_t *gpio);

void hal_pwm_set_freq(hal_pwm_t *obj, uint32_t freq);

void hal_pwm_set_dc(hal_pwm_t *obj, uint8_t duty_cycle);

void hal_pwm_start(hal_pwm_t *obj);

void hal_pwm_stop(hal_pwm_t *obj);

#endif
