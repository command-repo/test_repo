
#include <stddef.h>
#include <string.h>

#include "drv/drv_di.h"

#include "hal/hal_gpio.h"
#include "hal/hal_dac.h"
#include "hal/hal_dma.h"
#include "hal/hal_systick.h"
#include "hal/hal_tim.h"
#include "hal/hal_pwm.h"
#include "hal/hal_wdt.h"

static hal_gpio_t yr_led;
static hal_gpio_t yl_led;
static hal_gpio_t rr_led;
static hal_gpio_t rl_led;

static hal_gpio_t buzzer;

static hal_gpio_t motor1_pos_gpio;
static hal_gpio_t motor1_neg_gpio;
static hal_gpio_t motor2_pos_gpio;
static hal_gpio_t motor2_neg_gpio;
static hal_gpio_t m_sleep;

static hal_pwm_t motor1_pos_pwm;
static hal_pwm_t motor1_neg_pwm;
static hal_pwm_t motor2_pos_pwm;
static hal_pwm_t motor2_neg_pwm;

static void hal_init(void);

int main(void)
{
	hal_init();
	
	hal_pwm_set_freq(&motor1_pos_pwm, 50000);
	hal_pwm_set_freq(&motor1_neg_pwm, 50000);
	hal_pwm_set_freq(&motor2_pos_pwm, 50000);
	hal_pwm_set_freq(&motor2_neg_pwm, 50000);
	hal_pwm_set_dc(&motor1_pos_pwm, 20);
	hal_pwm_set_dc(&motor1_neg_pwm, 0);
	hal_pwm_set_dc(&motor2_pos_pwm, 35);
	hal_pwm_set_dc(&motor2_neg_pwm, 0);
	
	hal_gpio_set(&m_sleep, 0);
	hal_pwm_start(&motor1_pos_pwm);
	hal_pwm_start(&motor1_neg_pwm);
	hal_pwm_start(&motor2_pos_pwm);
	hal_pwm_start(&motor2_neg_pwm);
	hal_gpio_set(&m_sleep, 1);
	
	static uint32_t safety_bit_time = 0;
	while(1)
	{
		//hal_wdt_reload();
		
		if(hal_systick_get_past(safety_bit_time) >= 500)
		{
			safety_bit_time = hal_systick_get_ms();
			hal_gpio_toggle(&rl_led);
			hal_gpio_toggle(&rr_led);
			hal_gpio_toggle(&yl_led);
			hal_gpio_toggle(&yr_led);
		}
	}
}

static void hal_init(void)
{
	hal_systick_init();
	
	hal_gpio_init(&yr_led, HAL_GPIO_PORT_6, HAL_GPIO_PIN_9, HAL_GPIO_MODE_DO, 0);
	hal_gpio_init(&yl_led, HAL_GPIO_PORT_6, HAL_GPIO_PIN_10, HAL_GPIO_MODE_DO, 0);
	hal_gpio_init(&rr_led, HAL_GPIO_PORT_6, HAL_GPIO_PIN_11, HAL_GPIO_MODE_DO, 0);
	hal_gpio_init(&rl_led, HAL_GPIO_PORT_6, HAL_GPIO_PIN_12, HAL_GPIO_MODE_DO, 0);
	
	hal_gpio_init(&buzzer, HAL_GPIO_PORT_5, HAL_GPIO_PIN_6, HAL_GPIO_MODE_DO, 0);
	
	hal_gpio_init(&m_sleep, HAL_GPIO_PORT_4, HAL_GPIO_PIN_15, HAL_GPIO_MODE_DO, 1);
	hal_gpio_init(&motor1_pos_gpio, HAL_GPIO_PORT_4, HAL_GPIO_PIN_9, HAL_GPIO_MODE_AF, 0);
	hal_gpio_init(&motor1_neg_gpio, HAL_GPIO_PORT_4, HAL_GPIO_PIN_11, HAL_GPIO_MODE_AF, 0);
	hal_gpio_init(&motor2_pos_gpio, HAL_GPIO_PORT_4, HAL_GPIO_PIN_13, HAL_GPIO_MODE_AF, 0);
	hal_gpio_init(&motor2_neg_gpio, HAL_GPIO_PORT_4, HAL_GPIO_PIN_14, HAL_GPIO_MODE_AF, 0);
	
	hal_pwm_init(&motor1_pos_pwm, HAL_TIM_1, HAL_PWM_CH_1, HAL_PWM_MODE_NONINVERTED,
					&motor1_pos_gpio);
	hal_pwm_init(&motor1_neg_pwm, HAL_TIM_1, HAL_PWM_CH_2, HAL_PWM_MODE_NONINVERTED,
					&motor1_neg_gpio);
	hal_pwm_init(&motor2_pos_pwm, HAL_TIM_1, HAL_PWM_CH_3, HAL_PWM_MODE_NONINVERTED,
					&motor2_pos_gpio);
	hal_pwm_init(&motor2_neg_pwm, HAL_TIM_1, HAL_PWM_CH_4, HAL_PWM_MODE_NONINVERTED,
					&motor2_neg_gpio);
	
	hal_wdt_init(500);
	//hal_wdt_on();
}
