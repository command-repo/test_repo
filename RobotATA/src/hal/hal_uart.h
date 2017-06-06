
#ifndef HAL_UART_H
#define HAL_UART_H

#include <stdint.h>
#include <stdbool.h>

#include "hal/hal_gpio.h"
#include "hal/hal_dma.h"

typedef enum
{
	HAL_UART_1,
	HAL_UART_2,
	HAL_UART_3,
	HAL_UART_4,
	HAL_UART_5,
	HAL_UART_LIST_END
} hal_uart_list_t;

typedef enum
{
	HAL_UART_STOPBIT_0_5,
	HAL_UART_STOPBIT_1,
	HAL_UART_STOPBIT_1_5,
	HAL_UART_STOPBIT_2,
	HAL_UART_STOPBIT_LIST_END
} hal_uart_stopbit_t;

typedef enum
{
	HAL_UART_PRTY_NONE,
	HAL_UART_PRTY_EVEN,
	HAL_UART_PRTY_ODD,
	HAL_UART_PRTY_LIST_END
} hal_uart_prty_t;

typedef enum
{
	HAL_UART_EVENT_TX_DONE,
	HAL_UART_EVENT_TX_FAIL,
	HAL_UART_EVENT_RX_DONE,
	HAL_UART_EVENT_RX_FULL,
	HAL_UART_EVENT_RX_FAIL,
	HAL_UART_EVENT_LIST_END
} hal_uart_event_t;

typedef struct hal_uart_t
{
	hal_uart_list_t     uart;       /// Interface number
	uint32_t            baud;       /// Baud rate
	hal_uart_stopbit_t  stopbit;    /// Number of stop bits
	hal_uart_prty_t     prty;       /// Type of parity
	struct
	{
		hal_dma_t       *dma;       /// Pointer to DMA obj for tx
		hal_gpio_t      *gpio;      /// Pointer to GPIO obj for tx
		uint8_t         *buff;      /// Pointer to tx buffer
		uint16_t        len;        /// Length of tx buffer
		uint16_t        cnt;        /// Number of transmitted bytes
	} tx;
	struct
	{
		hal_dma_t       *dma;       /// Pointer to DMA obj for rx
		hal_gpio_t      *gpio;      /// Pointer to GPIO obj for rx
		uint8_t         *buff;      /// Pointer to rx buffer
		uint16_t        len;        /// Length of rx buffer
		uint16_t        *cnt;       /// Pointer to the number of received bytes
	} rx;
	void                (*clbk)(struct hal_uart_t *obj, hal_uart_event_t event,
                                uint16_t len);
} hal_uart_t;

/**
 * @brief         Initialization of the UART interface and it's object
 *
 * @param[in,out] obj      Pointer to the UART object
 * @param[in]     uart     Name of UART interface
 * @param[in]     baud     Desired baud rate
 * @param[in]     stopbit  Number of stop bits
 * @param[in]     prty     Type of parity
 * @param[in]     dma_tx   Pointer to DMA obj for tx
 * @param[in]     dma_rx   Pointer to DMA obj for rx
 * @param[in]     gpio_tx  Pointer to GPIO obj for tx
 * @param[in]     gpio_rx  Pointer to GPIO obj for rx
 */
void hal_uart_init(hal_uart_t *obj, hal_uart_list_t uart, uint32_t baud,
                    hal_uart_stopbit_t stopbit, hal_uart_prty_t prty,
                    hal_dma_t *dma_tx, hal_dma_t *dma_rx, hal_gpio_t *gpio_tx,
                    hal_gpio_t *gpio_rx);

/**
 * @brief      Set callback for UART events
 *
 * @param[in]  obj   Pointer to the UART object
 * @param[in]  clbk  Pointer to the callback function
 */
void hal_uart_set_clbk(hal_uart_t *obj, void (*clbk)(struct hal_uart_t *obj,
                                                     hal_uart_event_t event,
                                                     uint16_t len));

/**
 * @brief      Write buffer to UART port
 *
 * @param[in]  obj   Pointer to the UART object
 * @param[in]  buff  Pointer to tx buffer. It shouldn't be on stack!
 * @param[in]  len   Length of data, which should be transmitted
 *
 * @return     True in case when data has been written into the tx buffer,
 *             otherwise - false
 */
bool hal_uart_write(hal_uart_t *obj, const uint8_t *buff, uint16_t len);

/**
 * @brief      Check tx state
 *
 * @param[in]  obj   Pointer to the UART object
 *
 * @return     True in case of busy state of tx, otherwise - false
 */
bool hal_uart_is_tx_busy(hal_uart_t *obj);

/**
 * @brief         Set rx buffer
 *
 * @param[in]     obj      Pointer to the UART object
 * @param[out]    buff     Pointer to rx buffer
 * @param[in,out] len_cnt  Pointer to the length of data, which will be received
 *
 * @return        True in case when previous buffer has been readed,
 *                otherwise - false
 */
void hal_uart_listen(hal_uart_t *obj, uint8_t *buff, uint16_t *len_cnt);

/**
 * @brief      Check rx state
 *
 * @param[in]  obj   Pointer to the UART object
 *
 * @return     True in case of rx buff has filled or delay has occurred,
 *             otherwise - false
 */
bool hal_uart_is_rx_done(hal_uart_t *obj);

#endif
