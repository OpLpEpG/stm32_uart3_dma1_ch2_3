#include "usart2_3_dma1.h"
#include <zephyr.h>
#include <device.h>
#include <stm32f103xb.h>
#include <stm32f1xx_ll_bus.h>
#include <stm32f1xx_ll_gpio.h>
#include <stm32f1xx_ll_dma.h>
#include <stm32f1xx_ll_usart.h>

#include <logging/log.h>

LOG_MODULE_REGISTER(UART2DMA, LOG_LEVEL_DBG);

#define UART2_DMA_RX DMA1
#define UART2_DMA_RX_CHANNEL LL_DMA_CHANNEL_6

#define UART2_DMA_TX DMA1
#define UART2_DMA_TX_CHANNEL LL_DMA_CHANNEL_7

#define UART2_INTR_PRIO 2

#define UART2_DMA_RX_BUFF_SIZE (512)

typedef struct uart2_dma {
	struct {
		u8_t buffer[UART2_DMA_RX_BUFF_SIZE];
		uart_dma_fkt_rx_t cb;
	} rx;

	struct {
		struct k_sem txDone;
		struct k_mutex guardM;
	} tx;
} uart2_dma_t;

static uart2_dma_t uart2dma;

// tx dma isr handler
ISR_DIRECT_DECLARE(DMA1_Channel7_IRQHandler) {
	ISR_DIRECT_HEADER();
	// transfer complete
	if (LL_DMA_IsActiveFlag_TC2(DMA1)) 
    {
		LL_DMA_ClearFlag_TC2(DMA1);
		LL_DMA_DisableChannel(DMA1, UART2_DMA_TX_CHANNEL);
    LL_USART_EnableIT_TC(USART2); // need wait for last byte transmit
    //LOG_ERR("DMA TC");
	}
	ISR_DIRECT_FOOTER(1);
	ISR_DIRECT_PM(); // power management
	return 1; // We should check if scheduling decision should be made
}

ISR_DIRECT_DECLARE(UART2_IRQHandler) {
	ISR_DIRECT_HEADER();
	/* Check for IDLE line interrupt */
	if (LL_USART_IsEnabledIT_IDLE(USART2) && LL_USART_IsActiveFlag_IDLE(USART2)) {
		LL_USART_ClearFlag_IDLE(USART2);        /* Clear IDLE line flag */
        
		LL_DMA_DisableChannel(DMA1, UART2_DMA_RX_CHANNEL);

    uart2dma.rx.cb(&uart2dma.rx.buffer[0], sizeof(uart2dma.rx.buffer) - LL_DMA_GetDataLength(UART2_DMA_RX, UART2_DMA_RX_CHANNEL));
	}
	/* Check for TC interrupt */
  if (LL_USART_IsEnabledIT_TC(USART2) && LL_USART_IsActiveFlag_TC(USART2)) {
		LL_USART_ClearFlag_TC(USART2);        /* Clear TC flag */
    LL_USART_DisableIT_TC(USART2);
		k_sem_give(&uart2dma.tx.txDone);
  }
	ISR_DIRECT_FOOTER(1);
	ISR_DIRECT_PM(); // power management
	return 1; // We should check if scheduling decision should be made
}

static uart_dma_error_t uart2_dma_init (bool start, uart_dma_fkt_rx_t rxcb)
{
	if (true == start) {
		LOG_DBG("UART2 DMA on");
		uart2dma.rx.cb = rxcb;
		LL_DMA_EnableChannel(UART2_DMA_RX, UART2_DMA_RX_CHANNEL);
		LL_USART_Enable(USART2);
	} else {
		LOG_DBG("UART2 DMA off");
		uart2dma.rx.cb = NULL;
		LL_USART_Disable(USART2);
		LL_DMA_DisableChannel(UART2_DMA_RX, UART2_DMA_RX_CHANNEL);
		LL_DMA_DisableChannel(UART2_DMA_TX, UART2_DMA_TX_CHANNEL);
	}
	return uart_dma_error_success;
}

static uart_dma_error_t uart2_dma_readBuffer(void) {
  LL_DMA_SetMemoryAddress(DMA1, UART2_DMA_RX_CHANNEL, (u32_t)uart2dma.rx.buffer);
  LL_DMA_SetDataLength(DMA1, UART2_DMA_RX_CHANNEL, sizeof(uart2dma.rx.buffer));
	LL_DMA_EnableChannel(UART2_DMA_RX, UART2_DMA_RX_CHANNEL);
	return uart_dma_error_success;
}

static uart_dma_error_t uart2_dma_writeBuffer(u8_t * pB, size_t len, u32_t timeout) {
	uart_dma_error_t r = uart_dma_error_success;
  if (len == 0) return r;
	k_mutex_lock(&uart2dma.tx.guardM, K_FOREVER);
	LL_DMA_SetMemoryAddress(UART2_DMA_TX, UART2_DMA_TX_CHANNEL, (uint32_t)pB);
	LL_DMA_SetDataLength(UART2_DMA_TX, UART2_DMA_TX_CHANNEL, len);
	LL_DMA_EnableChannel(UART2_DMA_TX, UART2_DMA_TX_CHANNEL);
	if (k_sem_take(&uart2dma.tx.txDone, Z_TIMEOUT_MS(timeout))) {
		r = uart_dma_error_timeout;
	}
	k_mutex_unlock(&uart2dma.tx.guardM);
	return r;
}

static const uart_dma_api_t uart2dmaAPI = {
	.init = uart2_dma_init,
	.readBuffer = uart2_dma_readBuffer,
	.writeBuffer = uart2_dma_writeBuffer,
};



/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static int uart2_dma_initilize(struct device *dev) {

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

   memset(&uart2dma, 0, sizeof(uart2dma));

  k_sem_init(&uart2dma.tx.txDone, 0, 1);
  k_mutex_init(&uart2dma.tx.guardM);
  
  /* Init with LL driver */
  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);  
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /**USART2 GPIO Configuration  
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 DMA Init */
  /* USART2_RX Init */
  LL_DMA_SetDataTransferDirection(DMA1, UART2_DMA_RX_CHANNEL, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetChannelPriorityLevel(DMA1, UART2_DMA_RX_CHANNEL, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA1, UART2_DMA_RX_CHANNEL, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA1, UART2_DMA_RX_CHANNEL, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, UART2_DMA_RX_CHANNEL, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, UART2_DMA_RX_CHANNEL, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA1, UART2_DMA_RX_CHANNEL, LL_DMA_MDATAALIGN_BYTE);
  
  LL_DMA_SetPeriphAddress(DMA1, UART2_DMA_RX_CHANNEL, (u32_t)&USART2->DR);
  LL_DMA_SetMemoryAddress(DMA1, UART2_DMA_RX_CHANNEL, (u32_t)uart2dma.rx.buffer);
  LL_DMA_SetDataLength(DMA1, UART2_DMA_RX_CHANNEL, sizeof(uart2dma.rx.buffer));

  /* USART2_TX Init */
  LL_DMA_SetDataTransferDirection(DMA1, UART2_DMA_TX_CHANNEL, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetChannelPriorityLevel(DMA1, UART2_DMA_TX_CHANNEL, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA1, UART2_DMA_TX_CHANNEL, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA1, UART2_DMA_TX_CHANNEL, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, UART2_DMA_TX_CHANNEL, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, UART2_DMA_TX_CHANNEL, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA1, UART2_DMA_TX_CHANNEL, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_SetPeriphAddress(DMA1, UART2_DMA_TX_CHANNEL, (u32_t)&USART2->DR);
  LL_DMA_SetMemoryAddress(DMA1, UART2_DMA_TX_CHANNEL, (u32_t)NULL);
  LL_DMA_SetDataLength(DMA1, UART2_DMA_TX_CHANNEL, 0);
  // enable transfer complete interrupt
  LL_DMA_EnableIT_TC(DMA1, UART2_DMA_TX_CHANNEL);
  IRQ_DIRECT_CONNECT(DMA1_Channel7_IRQn, UART2_INTR_PRIO, DMA1_Channel7_IRQHandler, 0);
  irq_enable(DMA1_Channel7_IRQn);
  
  LL_USART_InitTypeDef USART_InitStruct = {0};
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
	LL_USART_EnableDMAReq_RX(USART2);
	LL_USART_EnableDMAReq_TX(USART2);
	LL_USART_EnableIT_IDLE(USART2);
	IRQ_DIRECT_CONNECT(USART2_IRQn, UART2_INTR_PRIO, UART2_IRQHandler, 0);
	irq_enable(USART2_IRQn);


  LOG_INF("UART2 DMA initialised");
  return uart_dma_error_success;
}

DEVICE_AND_API_INIT(uart2dma,UART2_DMA_NAME,&uart2_dma_initilize,NULL, NULL,POST_KERNEL,CONFIG_KERNEL_INIT_PRIORITY_DEVICE,&uart2dmaAPI);
