
#ifndef HAL_SYSTICK_H
#define HAL_SYSTICK_H

#include <stdint.h>

void hal_systick_init(void);

uint32_t hal_systick_get_ms(void);

uint32_t hal_systick_get_past(uint32_t start);

void hal_systick_delay(uint32_t ms);

#endif
