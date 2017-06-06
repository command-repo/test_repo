
#include <stddef.h>

#include "common/macros.h"

#include "hal/hal_dma.h"

#include "hal/CMSIS/device-support/stm32f10x.h"

#define DMA_IRQ_PRIORITY 2

static IRQn_Type const dma_irq_list[HAL_DMA_END][HAL_DMA_CH_END] =
{
	{
		DMA1_Channel1_IRQn, DMA1_Channel2_IRQn, DMA1_Channel3_IRQn,
		DMA1_Channel4_IRQn, DMA1_Channel5_IRQn, DMA1_Channel6_IRQn,
		DMA1_Channel7_IRQn
	},
	{
#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || \
	defined (STM32F10X_XL)
		DMA2_Channel1_IRQn, DMA2_Channel2_IRQn, DMA2_Channel3_IRQn,
		DMA2_Channel4_5_IRQn, DMA2_Channel4_5_IRQn
#elif defined (STM32F10X_CL)
		DMA2_Channel1_IRQn, DMA2_Channel2_IRQn, DMA2_Channel3_IRQn,
		DMA2_Channel4_IRQn, DMA2_Channel5_IRQn,
#else
		0,   // DMA2 available only on HD, XL and CL STM32F10x MCUs
		0,   // DMA2 available only on HD, XL and CL STM32F10x MCUs
		0,   // DMA2 available only on HD, XL and CL STM32F10x MCUs
		0,   // DMA2 available only on HD, XL and CL STM32F10x MCUs
		0,   // DMA2 available only on HD, XL and CL STM32F10x MCUs
#endif
	}
};

static DMA_Channel_TypeDef *const dma_ch_list[HAL_DMA_END][HAL_DMA_CH_END] =
{
	{
		DMA1_Channel1, DMA1_Channel2, DMA1_Channel3,
		DMA1_Channel4, DMA1_Channel5, DMA1_Channel6,
		DMA1_Channel7
	},
	{
		DMA2_Channel1, DMA2_Channel2, DMA2_Channel3,
		DMA2_Channel4, DMA2_Channel5
	}
};

static volatile uint32_t *const dma_isr_list[HAL_DMA_END][HAL_DMA_CH_END] =
{
	{
		&DMA1->ISR, &DMA1->ISR, &DMA1->ISR,
		&DMA1->ISR, &DMA1->ISR, &DMA1->ISR,
		&DMA1->ISR
	},
	{
		&DMA2->ISR, &DMA2->ISR, &DMA2->ISR,
		&DMA2->ISR, &DMA2->ISR, &DMA2->ISR,
		&DMA2->ISR
	}
};

static volatile uint32_t *const dma_isr_clr_mask_list[HAL_DMA_END][HAL_DMA_CH_END] =
{
	{
		&DMA1->IFCR, &DMA1->IFCR, &DMA1->IFCR,
		&DMA1->IFCR, &DMA1->IFCR, &DMA1->IFCR,
		&DMA1->IFCR
	},
	{
		&DMA2->IFCR, &DMA2->IFCR, &DMA2->IFCR,
		&DMA2->IFCR, &DMA2->IFCR, &DMA2->IFCR,
		&DMA2->IFCR
	}
};

static hal_dma_t *obj_list[HAL_DMA_END][HAL_DMA_CH_END];

static void hal_dma_hndlr(hal_dma_list_t dma, hal_dma_ch_t ch);

void hal_dma_init(hal_dma_t *obj, hal_dma_list_t dma, hal_dma_ch_t ch,
                    hal_dma_dir_t dir, hal_dma_inc_size_t inc_size)
{
	ASSERT(&obj != NULL);
	ASSERT(dma < HAL_DMA_END);
	ASSERT(ch < HAL_DMA_CH_END);
	
	// DMA2 doesn't have 6th and 7th channels
	ASSERT(dma != HAL_DMA_2 || ch <= HAL_DMA_CH_5);
	
	ASSERT(dir < HAL_DMA_DIR_END);
	ASSERT(inc_size < HAL_DMA_INC_SIZE_END);
	
	obj->dma = dma;
	obj->ch = ch;
	obj->dir = dir;
	obj->inc_size = inc_size;
	obj->src = 0;
	obj->dst = 0;
	obj->len = 0;
	obj->clbk = 0;
	
	obj_list[obj->dma][obj->ch] = obj;
	
	if(obj->dma == HAL_DMA_1)
	{
		RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	}
	else
	{
#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || \
	defined (STM32F10X_XL) || defined (STM32F10X_CL)
		RCC->AHBENR |= RCC_AHBENR_DMA2EN;
#else
	ASSERT(0);  // DMA2 available only on HD, XL and CL STM32F10x MCUs
#endif
	}
	
	// disable DMA
	DMA_Channel_TypeDef *dma_ch = dma_ch_list[obj->dma][obj->ch];
	dma_ch->CCR &= ~DMA_CCR1_EN;
	
	// clear isr reg flags
	volatile uint32_t *isr_clr_reg = dma_isr_clr_mask_list[dma][ch];
	*isr_clr_reg = (DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 |
					DMA_IFCR_CTEIF1) << (ch * 4);
	
	// setup data direction. Default is peripheral to memory
	dma_ch->CCR &= ~(DMA_CCR1_DIR | DMA_CCR1_MEM2MEM);
	if(obj->dir == HAL_DMA_DIR_MEM_TO_PERIPH)
	{
		dma_ch->CCR |= DMA_CCR1_DIR;
	}
	else if(obj->dir == HAL_DMA_DIR_MEM_TO_MEM)
	{
		dma_ch->CCR |= DMA_CCR1_MEM2MEM;
	}
	
	// setup data size. Default is 8
	dma_ch->CCR &= ~DMA_CCR1_MSIZE;
	dma_ch->CCR &= ~DMA_CCR1_PSIZE;
	if(obj->inc_size == HAL_DMA_INC_SIZE_16)
	{
		dma_ch->CCR |= DMA_CCR1_MSIZE_0;
		dma_ch->CCR |= DMA_CCR1_PSIZE_0;
	}
	else if(obj->inc_size == HAL_DMA_INC_SIZE_32)
	{
		dma_ch->CCR |= DMA_CCR1_MSIZE_1;
		dma_ch->CCR |= DMA_CCR1_PSIZE_1;
	}
	
	// setup incremental mode
	dma_ch->CCR |= DMA_CCR1_MINC;
	if(obj->dir == HAL_DMA_DIR_MEM_TO_MEM)
	{
		dma_ch->CCR |= DMA_CCR1_PINC;
	}
	else
	{
		dma_ch->CCR &= ~DMA_CCR1_PINC;
	}
	
	dma_ch->CCR |= DMA_CCR1_TCIE | DMA_CCR1_HTIE | DMA_CCR1_TEIE;
	
	NVIC_SetPriority(dma_irq_list[obj->dma][obj->ch], DMA_IRQ_PRIORITY);
	NVIC_EnableIRQ(dma_irq_list[obj->dma][obj->ch]);
}

void hal_dma_set_clbk(hal_dma_t *obj, void (*clbk)(struct hal_dma_t *obj,
                                                   hal_dma_event_t event))
{
	ASSERT(&obj != NULL);
	
	if(obj->dma == HAL_DMA_1)
	{
		NVIC_DisableIRQ(dma_irq_list[obj->dma][obj->ch]);
		obj->clbk = clbk;
		NVIC_EnableIRQ(dma_irq_list[obj->dma][obj->ch]);
	}
	else
	{
#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || \
	defined (STM32F10X_XL) || defined (STM32F10X_CL)
		NVIC_DisableIRQ(dma_irq_list[obj->dma][obj->ch]);
		obj->clbk = clbk;
		NVIC_EnableIRQ(dma_irq_list[obj->dma][obj->ch]);
#else
		ASSERT(0);  // DMA2 available only on HD, XL and CL STM32F10x MCUs
#endif
	}
}

void hal_dma_set_src(hal_dma_t *obj, void *src)
{
	ASSERT(&obj != NULL);
	ASSERT(src != 0);
	
	DMA_Channel_TypeDef *dma_ch = dma_ch_list[obj->dma][obj->ch];
	
	// this action allowed only when DMA is disabled
	ASSERT((dma_ch->CCR & DMA_CCR1_EN) == 0);
	
	obj->src = (uint32_t)src;
	
	if(obj->dir == HAL_DMA_DIR_MEM_TO_PERIPH)
	{
		dma_ch->CMAR = obj->src;
	}
	else
	{
		dma_ch->CPAR = obj->src;
	}
}

void hal_dma_set_dst(hal_dma_t *obj, void *dst)
{
	ASSERT(&obj != NULL);
	ASSERT(dst != 0);
	
	DMA_Channel_TypeDef *dma_ch = dma_ch_list[obj->dma][obj->ch];
	
	// this action allowed only when DMA is disabled
	ASSERT((dma_ch->CCR & DMA_CCR1_EN) == 0);
	
	obj->dst = (uint32_t)dst;
	
	if(obj->dir == HAL_DMA_DIR_MEM_TO_PERIPH)
	{
		dma_ch->CPAR = obj->dst;
	}
	else
	{
		dma_ch->CMAR = obj->dst;
	}
}

void hal_dma_set_len(hal_dma_t *obj, uint16_t len)
{
	ASSERT(&obj != NULL);
	ASSERT(len > 0);
	
	DMA_Channel_TypeDef *dma_ch = dma_ch_list[obj->dma][obj->ch];
	
	// this action allowed only when DMA is disabled
	ASSERT((dma_ch->CCR & DMA_CCR1_EN) == 0);
	
	obj->len = len;
	
	// setup number of data
	dma_ch->CNDTR = obj->len;
}

void hal_dma_start_once(hal_dma_t *obj)
{
	ASSERT(&obj != NULL);
	ASSERT(obj->len > 0);
	ASSERT(obj->src != 0);
	ASSERT(obj->dst != 0);
	
	DMA_Channel_TypeDef *dma_ch = dma_ch_list[obj->dma][obj->ch];
	
	// this action allowed only when DMA is disabled
	ASSERT((dma_ch->CCR & DMA_CCR1_EN) == 0);
	
	// disable circular mode
	dma_ch->CCR &= ~DMA_CCR1_CIRC;
	
	// Clear interrupt flag to prevent transfer complete interrupt
	volatile uint32_t *isr_clr_reg = dma_isr_clr_mask_list[obj->dma][obj->ch];
	*isr_clr_reg = DMA_IFCR_CTCIF1 << (obj->ch * 4);
	NVIC_EnableIRQ(dma_irq_list[obj->dma][obj->ch]);
	
	// enable DMA
	dma_ch->CCR |= DMA_CCR1_EN;
}

void hal_dma_start_cyclic(hal_dma_t *obj)
{
	ASSERT(&obj != NULL);
	ASSERT(obj->len > 0);
	ASSERT(obj->src != 0);
	ASSERT(obj->dst != 0);
	
	DMA_Channel_TypeDef *dma_ch = dma_ch_list[obj->dma][obj->ch];
	
	// this action allowed only when DMA is disabled
	ASSERT((dma_ch->CCR & DMA_CCR1_EN) == 0);
	
	// enable circular mode
	dma_ch->CCR |= DMA_CCR1_CIRC;
	
	// Clear interrupt flag to prevent transfer complete interrupt
	volatile uint32_t *isr_clr_reg = dma_isr_clr_mask_list[obj->dma][obj->ch];
	*isr_clr_reg = DMA_IFCR_CTCIF1 << (obj->ch * 4);
	NVIC_EnableIRQ(dma_irq_list[obj->dma][obj->ch]);

	// enable DMA
	dma_ch->CCR |= DMA_CCR1_EN;
}

void hal_dma_stop(hal_dma_t *obj)
{
	ASSERT(&obj != NULL);
	
	DMA_Channel_TypeDef *dma_ch = dma_ch_list[obj->dma][obj->ch];

	NVIC_DisableIRQ(dma_irq_list[obj->dma][obj->ch]);
	dma_ch->CCR &= ~DMA_CCR1_EN;

	// Waiting for end of DMA transmission
	while(dma_ch->CCR & DMA_CCR1_EN)
	{
		asm("nop");
	}
}

void hal_dma_set_dir(hal_dma_t *obj, hal_dma_dir_t dir)
{
	ASSERT(&obj != NULL);
	ASSERT(dir < HAL_DMA_DIR_END);
	
	DMA_Channel_TypeDef *dma_ch = dma_ch_list[obj->dma][obj->ch];
	
	// this action allowed only when DMA is disabled
	ASSERT((dma_ch->CCR & DMA_CCR1_EN) == 0);
	
	obj->dir = dir;
	
	// setup data direction. Default is peripheral to memory
	dma_ch->CCR &= ~(DMA_CCR1_DIR | DMA_CCR1_MEM2MEM);
	if(obj->dir == HAL_DMA_DIR_MEM_TO_PERIPH)
	{
		dma_ch->CCR |= DMA_CCR1_DIR;
	}
	else if(obj->dir == HAL_DMA_DIR_MEM_TO_MEM)
	{
		dma_ch->CCR |= DMA_CCR1_MEM2MEM;
	}
	
	// setup incremental mode
	dma_ch->CCR |= DMA_CCR1_MINC;
	if(obj->dir == HAL_DMA_DIR_MEM_TO_MEM)
	{
		dma_ch->CCR |= DMA_CCR1_PINC;
	}
	else
	{
		dma_ch->CCR &= ~DMA_CCR1_PINC;
	}
}

void hal_dma_set_inc_size(hal_dma_t *obj, hal_dma_inc_size_t inc_size)
{
	ASSERT(&obj != NULL);
	ASSERT(inc_size < HAL_DMA_INC_SIZE_END);
	
	DMA_Channel_TypeDef *dma_ch = dma_ch_list[obj->dma][obj->ch];
	
	// this action allowed only when DMA is disabled
	ASSERT((dma_ch->CCR & DMA_CCR1_EN) == 0);
	
	obj->inc_size = inc_size;
	
	// setup data size. Default is 8
	dma_ch->CCR &= ~DMA_CCR1_MSIZE;
	dma_ch->CCR &= ~DMA_CCR1_PSIZE;
	if(obj->inc_size == HAL_DMA_INC_SIZE_16)
	{
		dma_ch->CCR |= DMA_CCR1_MSIZE_0;
		dma_ch->CCR |= DMA_CCR1_PSIZE_0;
	}
	else if(obj->inc_size == HAL_DMA_INC_SIZE_32)
	{
		dma_ch->CCR |= DMA_CCR1_MSIZE_1;
		dma_ch->CCR |= DMA_CCR1_PSIZE_1;
	}
}

bool hal_dma_is_busy(hal_dma_t *obj)
{
	ASSERT(&obj != NULL);
	
	bool res = false;
	DMA_Channel_TypeDef *dma_ch = dma_ch_list[obj->dma][obj->ch];
	
	if(dma_ch->CCR & DMA_CCR1_EN)
	{
		res = true;
	}
	
	return res;
}

uint16_t hal_dma_get_transfered(hal_dma_t *obj)
{
	ASSERT(&obj != NULL);
	
	DMA_Channel_TypeDef *dma_ch = dma_ch_list[obj->dma][obj->ch];
	uint16_t res = obj->len - dma_ch->CNDTR;
	
	return res;
}

uint16_t hal_dma_get_remain(hal_dma_t *obj)
{
	ASSERT(&obj != NULL);
	
	DMA_Channel_TypeDef *dma_ch = dma_ch_list[obj->dma][obj->ch];
	uint16_t res = dma_ch->CNDTR;
	
	return res;
}

static void hal_dma_hndlr(hal_dma_list_t dma, hal_dma_ch_t ch)
{
	DMA_Channel_TypeDef *dma_ch = dma_ch_list[dma][ch];
	uint32_t isr = *dma_isr_list[dma][ch];
	volatile uint32_t *isr_clr_reg = dma_isr_clr_mask_list[dma][ch];
	uint8_t shift = ch * 4;
	
	if((dma_ch->CCR & DMA_CCR1_TCIE) && (isr & (DMA_ISR_TCIF1 << shift)))
	{
		// clear interrupt flag
		*isr_clr_reg = DMA_IFCR_CTCIF1 << shift;
		
		// disable DMA
		dma_ch->CCR &= ~DMA_CCR1_EN;
		
		// generate event
		if(obj_list[dma][ch]->clbk != NULL)
		{
			obj_list[dma][ch]->clbk(obj_list[dma][ch], HAL_DMA_EVENT_CMPLT);
		}
	}
	else if((dma_ch->CCR & DMA_CCR1_HTIE) && (isr & (DMA_ISR_HTIF1 << shift)))
	{
		// clear interrupt flag
		*isr_clr_reg = DMA_IFCR_CHTIF1 << shift;
		
		// generate event
		if(obj_list[dma][ch]->clbk != NULL)
		{
			obj_list[dma][ch]->clbk(obj_list[dma][ch], HAL_DMA_EVENT_HALF);
		}
	}
	else if((dma_ch->CCR & DMA_CCR1_TEIE) && (isr & (DMA_ISR_TEIF1 << shift)))
	{
		// clear interrupt flag
		*isr_clr_reg = DMA_IFCR_CTEIF1 << shift;
		
		// disable DMA
		dma_ch->CCR &= ~DMA_CCR1_EN;
		
		// generate event
		if(obj_list[dma][ch]->clbk != NULL)
		{
			obj_list[dma][ch]->clbk(obj_list[dma][ch], HAL_DMA_EVENT_ERROR);
		}
	}
}

void DMA1_Channel1_IRQHandler(void)
{
	hal_dma_hndlr(HAL_DMA_1, HAL_DMA_CH_1);
}

void DMA1_Channel2_IRQHandler(void)
{
	hal_dma_hndlr(HAL_DMA_1, HAL_DMA_CH_2);
}

void DMA1_Channel3_IRQHandler(void)
{
	hal_dma_hndlr(HAL_DMA_1, HAL_DMA_CH_3);
}

void DMA1_Channel4_IRQHandler(void)
{
	hal_dma_hndlr(HAL_DMA_1, HAL_DMA_CH_4);
}

void DMA1_Channel5_IRQHandler(void)
{
	hal_dma_hndlr(HAL_DMA_1, HAL_DMA_CH_5);
}

void DMA1_Channel6_IRQHandler(void)
{
	hal_dma_hndlr(HAL_DMA_1, HAL_DMA_CH_6);
}

void DMA1_Channel7_IRQHandler(void)
{
	hal_dma_hndlr(HAL_DMA_1, HAL_DMA_CH_7);
}

// DMA2 available only on HD, XL and CL STM32F10x MCUs
#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || \
	defined (STM32F10X_XL)
void DMA2_Channel1_IRQHandler(void)
{
	hal_dma_hndlr(HAL_DMA_2, HAL_DMA_CH_1);
}

void DMA2_Channel2_IRQHandler(void)
{
	hal_dma_hndlr(HAL_DMA_2, HAL_DMA_CH_2);
}

void DMA2_Channel3_IRQHandler(void)
{
	hal_dma_hndlr(HAL_DMA_2, HAL_DMA_CH_3);
}

void DMA2_Channel4_5_IRQHandler(void)
{
	uint32_t isr = *dma_isr_list[HAL_DMA_2][HAL_DMA_CH_4];
	if(isr & DMA_ISR_GIF4)
	{
		hal_dma_hndlr(HAL_DMA_2, HAL_DMA_CH_4);
	}
	else if(isr & DMA_ISR_GIF5)
	{
		hal_dma_hndlr(HAL_DMA_2, HAL_DMA_CH_5);
	}
}

#elif defined (STM32F10X_CL)

void DMA2_Channel1_IRQHandler(void)
{
	hal_dma_hndlr(HAL_DMA_2, HAL_DMA_CH_1);
}

void DMA2_Channel2_IRQHandler(void)
{
	hal_dma_hndlr(HAL_DMA_2, HAL_DMA_CH_2);
}

void DMA2_Channel3_IRQHandler(void)
{
	hal_dma_hndlr(HAL_DMA_2, HAL_DMA_CH_3);
}

void DMA2_Channel4_IRQHandler(void)
{
	hal_dma_hndlr(HAL_DMA_2, HAL_DMA_CH_4);
}

void DMA2_Channel5_IRQHandler(void)
{
	hal_dma_hndlr(HAL_DMA_2, HAL_DMA_CH_5);
}
#endif
