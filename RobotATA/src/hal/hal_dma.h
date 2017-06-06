
#ifndef HAL_DMA_H
#define HAL_DMA_H

#include <stdint.h>
#include <stdbool.h>

/*
DMA 1 requests for each channel:
| dma ch # |  1   |     2     |     3     |      4       |      5       |     6     |     7     |
| -------- | ---- | --------- | --------- | ------------ | ------------ | --------- | --------- |
| ADC1     | ADC1 | -         | -         | -            | -            | -         | -         |
| SPI/I2S  | -    | SPI1_RX   | SPI1_TX   | SPI2/I2S2_RX | SPI2/I2S2_TX | -         | -         |
| USART    | -    | USART3_TX | USART3_RX | USART1_TX    | USART1_RX    | USART2_RX | USART2_TX |
| I2C      | -    | -         | -         | I2C2_TX      | I2C2_RX      | I2C1_TX   | I2C1_RX   |
| TIM1     | -    | CH1       | -         | CH4/TRIG/COM | UP           | CH3       | -         |
| TIM2     | CH3  | UP        | -         | -            | CH1          | -         | CH2/CH4   |
| TIM3     | -    | CH3       | CH4/UP    | -            | -            | CH1/TRIG  | -         |
| TIM4     | CH1  | -         | -         | CH2          | CH3          | -         | UP        |
| -------- | ---- | --------- | --------- | ------------ | ------------ | --------- | --------- |

DMA 2 requests for each channel:
|   dma ch #   |      1      |       2        |        3        |        4        |    5     |
| ------------ | ----------- | -------------- | --------------- | --------------- | -------- |
| ADC3         | -           | -              | -               | -               | ADC3     |
| SPI/I2S3     | SPI/I2S3_RX | SPI/I2S3_TX    | -               | -               | -        |
| UART4        | -           | -              | UART4_RX        | -               | UART4_TX |
| SDIO         | -           | -              | -               | SDIO            | -        |
| TIM5         | CH4, TRIG   | CH3, UP        | -               | CH2             | CH1      |
| TIM6/DAC_CH1 | -           | -              | TIM6_UP/DAC_CH1 | -               | -        |
| TIM7/DAC_CH2 | -           | -              | -               | TIM7_UP/DAC_CH2 | -        |
| TIM8         | CH3, UP     | CH4, TRIG, COM | CH1             | -               | CH2      |
| ------------ | ----------- | -------------- | --------------- | --------------- | -------- |
ADC3, SDIO and TIM8 DMA requests are available only in HD and XL-density devices
*/

typedef enum
{
	HAL_DMA_1,
	HAL_DMA_2,
	HAL_DMA_END
} hal_dma_list_t;

typedef enum
{
	HAL_DMA_CH_1,
	HAL_DMA_CH_2,
	HAL_DMA_CH_3,
	HAL_DMA_CH_4,
	HAL_DMA_CH_5,
	HAL_DMA_CH_6,
	HAL_DMA_CH_7,
	HAL_DMA_CH_END
} hal_dma_ch_t;

typedef enum
{
	HAL_DMA_DIR_PERIPH_TO_MEM,
	HAL_DMA_DIR_MEM_TO_PERIPH,
	HAL_DMA_DIR_MEM_TO_MEM,
	HAL_DMA_DIR_END
} hal_dma_dir_t;

typedef enum
{
	HAL_DMA_INC_SIZE_8,
	HAL_DMA_INC_SIZE_16,
	HAL_DMA_INC_SIZE_32,
	HAL_DMA_INC_SIZE_END
} hal_dma_inc_size_t;

typedef enum
{
	HAL_DMA_EVENT_CMPLT,
	HAL_DMA_EVENT_HALF,
	HAL_DMA_EVENT_ERROR,
	HAL_DMA_EVENT_END
} hal_dma_event_t;

typedef struct hal_dma_t
{
	hal_dma_list_t      dma;
	hal_dma_ch_t        ch;
	hal_dma_dir_t       dir;
	hal_dma_inc_size_t  inc_size;
	uint32_t            src;
	uint32_t            dst;
	uint16_t            len;
	void                (*clbk)(struct hal_dma_t *obj, hal_dma_event_t event);
} hal_dma_t;

void hal_dma_init(hal_dma_t *obj, hal_dma_list_t dma, hal_dma_ch_t ch,
                    hal_dma_dir_t dir, hal_dma_inc_size_t inc_size);

void hal_dma_set_clbk(hal_dma_t *obj, void (*clbk)(struct hal_dma_t *obj,
                                                   hal_dma_event_t event));

void hal_dma_set_src(hal_dma_t *obj, void *src);

void hal_dma_set_dst(hal_dma_t *obj, void *dst);

void hal_dma_set_len(hal_dma_t *obj, uint16_t len);

void hal_dma_start_once(hal_dma_t *obj);

void hal_dma_start_cyclic(hal_dma_t *obj);

void hal_dma_stop(hal_dma_t *obj);

void hal_dma_set_dir(hal_dma_t *obj, hal_dma_dir_t dir);

void hal_dma_set_inc_size(hal_dma_t *obj, hal_dma_inc_size_t inc_size);

bool hal_dma_is_busy(hal_dma_t *obj);

uint16_t hal_dma_get_transfered(hal_dma_t *obj);

uint16_t hal_dma_get_remain(hal_dma_t *obj);

#endif
