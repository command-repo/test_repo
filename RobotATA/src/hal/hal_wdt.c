
#include "common/macros.h"

#include "hal/hal_wdt.h"
#include "hal/hal_rcc.h"

#include "hal/CMSIS/device-support/stm32f10x.h"

#define WDT_CLK_PERIOD      32   /*!< WDT clock period in ms without prescaller */
#define WDT_MAX_PRESCALLER  256
#define WDT_MAX_RELOAD      4095

static void hal_wdt_calc(uint16_t ms, uint16_t *presc, uint16_t *reload);

void hal_wdt_init(uint16_t ms)
{
	// Check input parameter value in case of max reload with max prescaller
	ASSERT(((ms * WDT_CLK_PERIOD) / WDT_MAX_PRESCALLER) <= WDT_MAX_RELOAD);
	
	uint16_t presc = 0;
	uint16_t reload = 0;
	
	hal_wdt_calc(ms, &presc, &reload);
	
	// Enables write access to IWDG_PR and IWDG_RLR
	IWDG->KR = 0x5555;
	
	// Set WDT prescaler
	IWDG->PR = presc;
	
	// Set WDT reload value
	IWDG->RLR = reload;
	
	// Refresh regs values
	IWDG->KR = 0xAAAA;
	
	// Disable write access to IWDG_PR and IWDG_RLR
	IWDG->KR = 0x0000;
}

void hal_wdt_on(void)
{
	IWDG->KR = 0xCCCC;
}

void hal_wdt_reload(void)
{
	IWDG->KR = 0xAAAA;
}

static void hal_wdt_calc(uint16_t ms, uint16_t *presc, uint16_t *reload)
{
	uint16_t tmp_presc = 4;
	uint32_t tmp_reload = 0;
	
	do
	{
		tmp_reload = (ms * WDT_CLK_PERIOD) / tmp_presc;
		if(tmp_reload <= WDT_MAX_RELOAD)
		{
			break;
		}
		
		tmp_presc *= 2;
	}
	while(tmp_presc <= WDT_MAX_PRESCALLER);
	
	*reload = (uint16_t)tmp_reload;
	
	switch(tmp_presc)
	{
		case 4:
			*presc = 0;
			break;
		
		case 8:
			*presc = IWDG_PR_PR_0;
			break;
		
		case 16:
			*presc = IWDG_PR_PR_1;
			break;
		
		case 32:
			*presc = (IWDG_PR_PR_1 | IWDG_PR_PR_0);
			break;
		
		case 64:
			*presc = IWDG_PR_PR_2;
			break;
		
		case 128:
			*presc = (IWDG_PR_PR_2 | IWDG_PR_PR_0);
			break;
		
		case 256:
			*presc = (IWDG_PR_PR_2 | IWDG_PR_PR_1);
			break;
	}
}
