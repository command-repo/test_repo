
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "common/macros.h"

#include "hal/hal_uart.h"
#include "hal/hal_rcc.h"
#include "hal/hal_dma.h"
#include "hal/hal_gpio.h"

#include "hal/CMSIS/device-support/stm32f10x.h"
#include "hal/CMSIS/core-support/cmsis_gcc.h"

#define UART_IRQ_PRIORITY   6
#define MAX_BRR_VAL         0xFFFF

static USART_TypeDef *const uart_list[HAL_UART_LIST_END] =
{
	USART1,
	USART2,
	USART3,
	UART4,
	UART5,
};

static IRQn_Type const uart_irq_list[HAL_UART_LIST_END] =
{
	USART1_IRQn,
	USART2_IRQn,
	USART3_IRQn,
#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || \
	defined (STM32F10X_XL) || (STM32F10X_CL)
	UART4_IRQn,
	UART5_IRQn,
#else
	0,     // UART4 available only on HD, XL and CL STM32F10x MCUs
	0      // UART5 available only on HD, XL and CL STM32F10x MCUs
#endif
};

static uint32_t const rcc_list[HAL_UART_LIST_END] =
{
	RCC_APB2ENR_USART1EN,
	RCC_APB1ENR_USART2EN,
#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL)
	RCC_APB1ENR_USART3EN,
#else
	0
#endif
#if defined (STM32F10X_HD) || defined (STM32F10X_CL) || \
	defined (STM32F10X_HD_VL)
	RCC_APB1ENR_UART4EN,
	RCC_APB1ENR_UART5EN
#else
	0,
	0,
#endif
};

static volatile uint32_t *rcc_bus_list[HAL_UART_LIST_END] =
{
	&RCC->APB2ENR,
	&RCC->APB1ENR,
	&RCC->APB1ENR,
	&RCC->APB1ENR,
	&RCC->APB1ENR
};

static hal_rcc_src_t const rcc_src_list[HAL_UART_LIST_END] =
{
	HAL_RCC_SRC_APB2,
	HAL_RCC_SRC_APB1,
	HAL_RCC_SRC_APB1,
	HAL_RCC_SRC_APB1,
	HAL_RCC_SRC_APB1
};

static GPIO_TypeDef *const gpio_list[HAL_GPIO_PORT_END] =
{
	GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG
};

static void hal_uart_tx_hndlr(hal_uart_list_t uart);
static void hal_uart_rx_hndlr(hal_uart_list_t uart);
static void hal_uart_err_hndlr(hal_uart_list_t uart, hal_uart_event_t event);
static void hal_uart_irq_hndlr(hal_uart_list_t uart);

static void hal_uart1_tx_dma_clbk(hal_dma_t *obj, hal_dma_event_t event);
static void hal_uart1_rx_dma_clbk(hal_dma_t *obj, hal_dma_event_t event);
static void hal_uart2_tx_dma_clbk(hal_dma_t *obj, hal_dma_event_t event);
static void hal_uart2_rx_dma_clbk(hal_dma_t *obj, hal_dma_event_t event);
static void hal_uart3_tx_dma_clbk(hal_dma_t *obj, hal_dma_event_t event);
static void hal_uart3_rx_dma_clbk(hal_dma_t *obj, hal_dma_event_t event);
static void hal_uart4_tx_dma_clbk(hal_dma_t *obj, hal_dma_event_t event);
static void hal_uart4_rx_dma_clbk(hal_dma_t *obj, hal_dma_event_t event);
static void hal_uart5_tx_dma_clbk(hal_dma_t *obj, hal_dma_event_t event);
static void hal_uart5_rx_dma_clbk(hal_dma_t *obj, hal_dma_event_t event);
static void (*dma_clbk_list[HAL_UART_LIST_END * 2])(hal_dma_t *obj,
													hal_dma_event_t event) =
{
	hal_uart1_tx_dma_clbk,
	hal_uart1_rx_dma_clbk,
	hal_uart2_tx_dma_clbk,
	hal_uart2_rx_dma_clbk,
	hal_uart3_tx_dma_clbk,
	hal_uart3_rx_dma_clbk,
	hal_uart4_tx_dma_clbk,
	hal_uart4_rx_dma_clbk,
	hal_uart5_tx_dma_clbk,
	hal_uart5_rx_dma_clbk
};

static hal_uart_t *obj_list[HAL_UART_LIST_END];

void hal_uart_init(hal_uart_t *obj, hal_uart_list_t uart, uint32_t baud,
                    hal_uart_stopbit_t stopbit, hal_uart_prty_t prty,
                    hal_dma_t *dma_tx, hal_dma_t *dma_rx, hal_gpio_t *gpio_tx,
                    hal_gpio_t *gpio_rx)
{
	ASSERT(&obj != NULL);
	ASSERT(uart < HAL_UART_LIST_END);
	ASSERT(baud > 0);
	ASSERT(stopbit < HAL_UART_STOPBIT_LIST_END);
	ASSERT(prty < HAL_UART_PRTY_LIST_END);
	ASSERT(&dma_tx != NULL && dma_tx->dir == HAL_DMA_DIR_MEM_TO_PERIPH &&
			dma_tx->inc_size == HAL_DMA_INC_SIZE_8);
	ASSERT(&dma_rx != NULL && dma_rx->dir == HAL_DMA_DIR_PERIPH_TO_MEM &&
			dma_rx->inc_size == HAL_DMA_INC_SIZE_8);
	ASSERT(&gpio_tx != NULL && gpio_tx->mode == HAL_GPIO_MODE_AF);
	ASSERT(&gpio_rx != NULL && gpio_rx->mode == HAL_GPIO_MODE_AF);
	
	obj->uart = uart;
	obj->baud = baud;
	obj->stopbit = stopbit;
	obj->prty = prty;
	obj->tx.dma = dma_tx;
	obj->rx.dma = dma_rx;
	obj->tx.gpio = gpio_tx;
	obj->rx.gpio = gpio_rx;
	
	obj->tx.buff = NULL;
	obj->tx.len = 0;
	obj->tx.cnt = 0;
	obj->rx.buff = NULL;
	obj->rx.len = 0;
	obj->rx.cnt = NULL;
	
	obj->clbk = NULL;
	
	*rcc_bus_list[obj->uart] |= rcc_list[obj->uart];
	
	// Disable uart
	uart_list[obj->uart]->CR1 &= ~USART_CR1_UE;
	
	// Setup nuber of stop bits
	uart_list[obj->uart]->CR2 &= ~USART_CR2_STOP;
	switch(obj->stopbit)
	{
		case HAL_UART_STOPBIT_0_5:
			uart_list[obj->uart]->CR2 |= USART_CR2_STOP_0;
			break;
		
		case HAL_UART_STOPBIT_1:
			uart_list[obj->uart]->CR2 &= ~USART_CR2_STOP;
			break;
		
		case HAL_UART_STOPBIT_1_5:
			uart_list[obj->uart]->CR2 |= USART_CR2_STOP;
			break;
		
		case HAL_UART_STOPBIT_2:
			uart_list[obj->uart]->CR2 |= USART_CR2_STOP_1;
			break;
	}
	
	// Setup parity type
	uart_list[obj->uart]->CR1 &= ~(USART_CR1_PCE | USART_CR1_PS);
	switch(obj->prty)
	{
		case HAL_UART_PRTY_NONE:
			uart_list[obj->uart]->CR1 &= ~(USART_CR1_PCE | USART_CR1_PS);
			break;
		
		case HAL_UART_PRTY_EVEN:
			uart_list[obj->uart]->CR1 |= USART_CR1_PCE;
			break;
		
		case HAL_UART_PRTY_ODD:
			uart_list[obj->uart]->CR1 |= (USART_CR1_PCE | USART_CR1_PS);
			break;
	}
	
	// Enable Tx and Rx
	uart_list[obj->uart]->CR1 |= (USART_CR1_TE | USART_CR1_RE);
	
	// Disable IDLE interrupt
	uart_list[obj->uart]->CR1 &= ~USART_CR1_IDLEIE;
	
	// Allows error interrupt
	uart_list[obj->uart]->CR3 |= USART_CR3_EIE;
	
	// Enable DMA support
	uart_list[obj->uart]->CR3 |= (USART_CR3_DMAR | USART_CR3_DMAT);
	
	// Calculate UART prescaller
	uint32_t div = hal_rcc_get_freq(rcc_src_list[obj->uart]) / obj->baud;
	// Baud rate is too low or too high
	ASSERT(div > 0 && div <= MAX_BRR_VAL);
	uart_list[obj->uart]->BRR = (uint16_t)div;
	
	obj_list[obj->uart] = obj;
	
	hal_dma_stop(obj->tx.dma);
	hal_dma_stop(obj->rx.dma);
	
	hal_dma_set_clbk(obj->tx.dma, dma_clbk_list[obj->uart * 2]);
	hal_dma_set_dst(obj->tx.dma, (uint8_t*)&uart_list[obj->uart]->DR);
	
	hal_dma_set_clbk(obj->rx.dma, dma_clbk_list[(obj->uart * 2) + 1]);
	hal_dma_set_src(obj->rx.dma, (uint8_t*)&uart_list[obj->uart]->DR);
	
	uart_list[obj->uart]->CR1 |= USART_CR1_UE;
	
	NVIC_ClearPendingIRQ(uart_irq_list[obj->uart]);
	NVIC_SetPriority(uart_irq_list[obj->uart], UART_IRQ_PRIORITY);
	NVIC_EnableIRQ(uart_irq_list[obj->uart]);
}

void hal_uart_set_clbk(hal_uart_t *obj, void (*clbk)(struct hal_uart_t *obj,
                                                     hal_uart_event_t event,
                                                     uint16_t len))
{
	ASSERT(&obj != NULL);
	
	obj->clbk = clbk;
}

bool hal_uart_write(hal_uart_t *obj, const uint8_t *buff, uint16_t len)
{
	ASSERT(&obj != NULL);
	ASSERT(buff != NULL);
	ASSERT(len > 0);
	
	bool res = false;
	
	// If tx isn't busy
	if(obj->tx.buff == NULL)
	{
		if(hal_dma_is_busy(obj->tx.dma) != true)
		{
			obj->tx.buff = (uint8_t*)buff;
			obj->tx.len = len;
			obj->tx.cnt = 0;
			
			hal_dma_set_src(obj->tx.dma, obj->tx.buff);
			hal_dma_set_len(obj->tx.dma, obj->tx.len);
			hal_dma_start_once(obj->tx.dma);
			
			res = true;
		}
	}
	
	return res;
}

bool hal_uart_is_tx_busy(hal_uart_t *obj)
{
	ASSERT(&obj != NULL);
	
	bool res = true;
	
	if(obj->tx.buff == NULL)
	{
		res = false;
	}
	
	return res;
}

void hal_uart_listen(hal_uart_t *obj, uint8_t *buff, uint16_t *len_cnt)
{
	ASSERT(&obj != NULL);
	ASSERT(buff != NULL);
	ASSERT(&len_cnt != NULL);
	ASSERT(*len_cnt > 0);
	
	uart_list[obj->uart]->CR1 &= ~USART_CR1_IDLEIE;
	hal_dma_stop(obj->rx.dma);
	uart_list[obj->uart]->DR;
	
	obj->rx.buff = buff;
	obj->rx.len = *len_cnt;
	*len_cnt = 0;
	obj->rx.cnt = len_cnt;
	
	hal_dma_set_dst(obj->rx.dma, obj->rx.buff);
	hal_dma_set_len(obj->rx.dma, obj->rx.len);
	hal_dma_start_once(obj->rx.dma);
	
	uart_list[obj->uart]->CR1 |= USART_CR1_IDLEIE;
}

bool hal_uart_is_rx_done(hal_uart_t *obj)
{
	ASSERT(&obj != NULL);
	
	bool res = false;
	
	if(obj->rx.buff == NULL)
	{
		res = true;
	}
	
	return res;
}

static void hal_uart_tx_hndlr(hal_uart_list_t uart)
{
	hal_uart_t *obj = obj_list[uart];
	
	obj->tx.cnt = hal_dma_get_transfered(obj->tx.dma);
	obj->tx.buff = NULL;
	obj->tx.len = 0;
	
	if(obj->clbk != NULL)
	{
		obj->clbk(obj, HAL_UART_EVENT_TX_DONE, obj->tx.cnt);
	}
	obj->tx.cnt = 0;
}

static void hal_uart_rx_hndlr(hal_uart_list_t uart)
{
	// Rx buffer has partly filled (package has received)
	// or Rx buffer has already filled
	
	hal_uart_t *obj = obj_list[uart];
	
	*obj->rx.cnt = hal_dma_get_transfered(obj->rx.dma);
	obj->rx.buff = NULL;
	obj->rx.len = 0;
	if(obj->clbk != NULL)
	{
		obj->clbk(obj, HAL_UART_EVENT_RX_DONE, *obj->rx.cnt);
	}
}

static void hal_uart_err_hndlr(hal_uart_list_t uart, hal_uart_event_t event)
{
	// Tx or Rx fail
	if(obj_list[uart]->clbk != NULL)
	{
		obj_list[uart]->clbk(obj_list[uart], event, 0);
	}
}

static void hal_uart1_tx_dma_clbk(hal_dma_t *obj, hal_dma_event_t event)
{
	if(event == HAL_DMA_EVENT_CMPLT)
	{
		hal_uart_tx_hndlr(HAL_UART_1);
	}
	else if(event == HAL_DMA_EVENT_ERROR)
	{
		hal_uart_err_hndlr(HAL_UART_1, HAL_UART_EVENT_TX_FAIL);
	}
}

static void hal_uart1_rx_dma_clbk(hal_dma_t *obj, hal_dma_event_t event)
{
	if(event == HAL_DMA_EVENT_CMPLT)
	{
		hal_uart_rx_hndlr(HAL_UART_1);
	}
	else if(event == HAL_DMA_EVENT_ERROR)
	{
		hal_uart_err_hndlr(HAL_UART_1, HAL_UART_EVENT_RX_FAIL);
	}
}

static void hal_uart2_tx_dma_clbk(hal_dma_t *obj, hal_dma_event_t event)
{
	if(event == HAL_DMA_EVENT_CMPLT)
	{
		hal_uart_tx_hndlr(HAL_UART_2);
	}
	else if(event == HAL_DMA_EVENT_ERROR)
	{
		hal_uart_err_hndlr(HAL_UART_2, HAL_UART_EVENT_TX_FAIL);
	}
}

static void hal_uart2_rx_dma_clbk(hal_dma_t *obj, hal_dma_event_t event)
{
	if(event == HAL_DMA_EVENT_CMPLT)
	{
		hal_uart_rx_hndlr(HAL_UART_2);
	}
	else if(event == HAL_DMA_EVENT_ERROR)
	{
		hal_uart_err_hndlr(HAL_UART_2, HAL_UART_EVENT_RX_FAIL);
	}
}

static void hal_uart3_tx_dma_clbk(hal_dma_t *obj, hal_dma_event_t event)
{
	if(event == HAL_DMA_EVENT_CMPLT)
	{
		hal_uart_tx_hndlr(HAL_UART_3);
	}
	else if(event == HAL_DMA_EVENT_ERROR)
	{
		hal_uart_err_hndlr(HAL_UART_3, HAL_UART_EVENT_TX_FAIL);
	}
}

static void hal_uart3_rx_dma_clbk(hal_dma_t *obj, hal_dma_event_t event)
{
	if(event == HAL_DMA_EVENT_CMPLT)
	{
		hal_uart_rx_hndlr(HAL_UART_3);
	}
	else if(event == HAL_DMA_EVENT_ERROR)
	{
		hal_uart_err_hndlr(HAL_UART_3, HAL_UART_EVENT_RX_FAIL);
	}
}

static void hal_uart4_tx_dma_clbk(hal_dma_t *obj, hal_dma_event_t event)
{
	if(event == HAL_DMA_EVENT_CMPLT)
	{
		hal_uart_tx_hndlr(HAL_UART_4);
	}
	else if(event == HAL_DMA_EVENT_ERROR)
	{
		hal_uart_err_hndlr(HAL_UART_4, HAL_UART_EVENT_TX_FAIL);
	}
}

static void hal_uart4_rx_dma_clbk(hal_dma_t *obj, hal_dma_event_t event)
{
	if(event == HAL_DMA_EVENT_CMPLT)
	{
		hal_uart_rx_hndlr(HAL_UART_4);
	}
	else if(event == HAL_DMA_EVENT_ERROR)
	{
		hal_uart_err_hndlr(HAL_UART_4, HAL_UART_EVENT_RX_FAIL);
	}
}

static void hal_uart5_tx_dma_clbk(hal_dma_t *obj, hal_dma_event_t event)
{
	if(event == HAL_DMA_EVENT_CMPLT)
	{
		hal_uart_tx_hndlr(HAL_UART_5);
	}
	else if(event == HAL_DMA_EVENT_ERROR)
	{
		hal_uart_err_hndlr(HAL_UART_5, HAL_UART_EVENT_TX_FAIL);
	}
}

static void hal_uart5_rx_dma_clbk(hal_dma_t *obj, hal_dma_event_t event)
{
	if(event == HAL_DMA_EVENT_CMPLT)
	{
		hal_uart_rx_hndlr(HAL_UART_5);
	}
	else if(event == HAL_DMA_EVENT_ERROR)
	{
		hal_uart_err_hndlr(HAL_UART_5, HAL_UART_EVENT_RX_FAIL);
	}
}

static void hal_uart_irq_hndlr(hal_uart_list_t uart)
{
	uint32_t status_reg = uart_list[uart]->SR;
	
	if((uart_list[uart]->CR1 & USART_CR1_IDLEIE) &&
		(status_reg & USART_SR_IDLE))
	{
		hal_dma_stop(obj_list[uart]->rx.dma);
		uart_list[uart]->DR;
		// IDLE event has happened (package has received)
		hal_uart_rx_hndlr(uart);
	}
	else if(status_reg &
			(USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE))
	{
		uart_list[uart]->DR;
		// Error event has happened
		hal_uart_err_hndlr(uart, HAL_UART_EVENT_RX_FAIL);
	}
}

void USART1_IRQHandler(void)
{
	NVIC_ClearPendingIRQ(uart_irq_list[HAL_UART_1]);
	hal_uart_irq_hndlr(HAL_UART_1);
}

void USART2_IRQHandler(void)
{
	NVIC_ClearPendingIRQ(uart_irq_list[HAL_UART_2]);
	hal_uart_irq_hndlr(HAL_UART_2);
}

void USART3_IRQHandler(void)
{
	NVIC_ClearPendingIRQ(uart_irq_list[HAL_UART_3]);
	hal_uart_irq_hndlr(HAL_UART_3);
}

#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || \
	defined (STM32F10X_XL) || (STM32F10X_CL)
void UART4_IRQHandler(void)
{
	NVIC_ClearPendingIRQ(uart_irq_list[HAL_UART_4]);
	hal_uart_irq_hndlr(HAL_UART_4);
}

void UART5_IRQHandler(void)
{
	NVIC_ClearPendingIRQ(uart_irq_list[HAL_UART_5]);
	hal_uart_irq_hndlr(HAL_UART_5);
}
#endif
