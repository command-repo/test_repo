
#ifndef HAL_WDT_H
#define HAL_WDT_H

#include <stdint.h>

void hal_wdt_init(uint16_t ms);

void hal_wdt_on(void);

void hal_wdt_reload(void);

#endif
