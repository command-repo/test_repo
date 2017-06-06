
#ifndef HAL_RCC_H
#define HAL_RCC_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
	HAL_RCC_SRC_SYSCLK,
	HAL_RCC_SRC_AHB,
	HAL_RCC_SRC_APB1,
	HAL_RCC_SRC_APB2,
	HAL_RCC_SRC_END
} hal_rcc_src_t;

//bool hal_rcc_init(void);

uint32_t hal_rcc_get_freq(hal_rcc_src_t src);

void hal_rcc_reset(void);

#endif
