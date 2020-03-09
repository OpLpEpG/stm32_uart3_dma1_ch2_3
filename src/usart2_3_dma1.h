/*
 * uart_dma.h
 *
 *  Created on: 20 Mar 2019
 *      Author: Stefan Jaritz
 */

#ifndef DRV_uart_DMA_H_
#define DRV_uart_DMA_H_

#include <device.h>
#include <stdbool.h>

#include <stm32f1xx_ll_usart.h>

//if CONFIG_UART3_DMA
#define UART2_DMA_NAME "UART2_DMA"
#define UART3_DMA_NAME "UART3_DMA"

typedef enum uart_dma_errors {
	uart_dma_error_success = 0,
	uart_dma_error_devNotFound = 1,
	uart_dma_error_configFailed = 2,
	uart_dma_error_timeout = 3
} uart_dma_error_t;

typedef void (* uart_dma_fkt_rx_t) (u8_t * pD, size_t len);

typedef struct uart_dma_api_s {
	uart_dma_error_t (* init) (bool start, uart_dma_fkt_rx_t rxcb);
	uart_dma_error_t (* readBuffer)(void);
	uart_dma_error_t (* writeBuffer)(u8_t * pB, size_t len, u32_t timeout);
} uart_dma_api_t;

#endif /* DRV_uart_DMA_H_ */