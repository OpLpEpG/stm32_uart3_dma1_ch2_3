
#include "usart3_dma1_ch2_3.h"
#include <zephyr.h>
#include <device.h>
#include <stm32f103xb.h>
#include <stm32f1xx_ll_bus.h>
#include <stm32f1xx_ll_gpio.h>
#include <stm32f1xx_ll_dma.h>
#include <stm32f1xx_ll_usart.h>

#include <logging/log.h>

LOG_MODULE_REGISTER(UART3DMA, LOG_LEVEL_DBG);

#define UART3_DMA_RX DMA1
#define UART3_DMA_RX_CHANNEL LL_DMA_CHANNEL_3

#define UART3_DMA_TX DMA1
#define UART3_DMA_TX_CHANNEL LL_DMA_CHANNEL_2

#define UART3_INTR_PRIO 2

#define UART3_DMA_RX_BUFF_SIZE (512)

typedef struct uart3_dma {
	struct {
		u8_t buffer[UART3_DMA_RX_BUFF_SIZE];
		uart3_dma_fkt_rx_t cb;
	} rx;

	struct {
		struct k_sem txDone;
		struct k_mutex guardM;
	} tx;
} uart3_dma_t;

static uart3_dma_t uart3dma;

// tx dma isr handler
ISR_DIRECT_DECLARE(DMA1_Channel2_IRQHandler) {
	ISR_DIRECT_HEADER();
	// transfer complete
	if (LL_DMA_IsActiveFlag_TC2(DMA1)) 
    {
		LL_DMA_ClearFlag_TC2(DMA1);
		LL_DMA_DisableChannel(DMA1, UART3_DMA_TX_CHANNEL);
    LL_USART_EnableIT_TC(USART3); // need wait for last byte transmit
    //LOG_ERR("DMA TC");
	}
	ISR_DIRECT_FOOTER(1);
	ISR_DIRECT_PM(); // power management
	return 1; // We should check if scheduling decision should be made
}

ISR_DIRECT_DECLARE(UART3_IRQHandler) {
	ISR_DIRECT_HEADER();
	/* Check for IDLE line interrupt */
	if (LL_USART_IsEnabledIT_IDLE(USART3) && LL_USART_IsActiveFlag_IDLE(USART3)) {
		LL_USART_ClearFlag_IDLE(USART3);        /* Clear IDLE line flag */
        
		LL_DMA_DisableChannel(DMA1, UART3_DMA_RX_CHANNEL);

    uart3dma.rx.cb(&uart3dma.rx.buffer[0], sizeof(uart3dma.rx.buffer) - LL_DMA_GetDataLength(UART3_DMA_RX, UART3_DMA_RX_CHANNEL));
	}
	/* Check for TC interrupt */
  if (LL_USART_IsEnabledIT_TC(USART3) && LL_USART_IsActiveFlag_TC(USART3)) {
		LL_USART_ClearFlag_TC(USART3);        /* Clear TC flag */
    LL_USART_DisableIT_TC(USART3);
		k_sem_give(&uart3dma.tx.txDone);
  }
	ISR_DIRECT_FOOTER(1);
	ISR_DIRECT_PM(); // power management
	return 1; // We should check if scheduling decision should be made
}

static uart3_dma_error_t uart3_dma_init (bool start, uart3_dma_fkt_rx_t rxcb)
{
	if (true == start) {
		LOG_DBG("UART3 DMA on");
		uart3dma.rx.cb = rxcb;
		LL_DMA_EnableChannel(UART3_DMA_RX, UART3_DMA_RX_CHANNEL);
		LL_USART_Enable(USART3);
	} else {
		LOG_DBG("UART3 DMA off");
		uart3dma.rx.cb = NULL;
		LL_USART_Disable(USART3);
		LL_DMA_DisableChannel(UART3_DMA_RX, UART3_DMA_RX_CHANNEL);
		LL_DMA_DisableChannel(UART3_DMA_TX, UART3_DMA_TX_CHANNEL);
	}
	return uart3_dma_error_success;
}

static uart3_dma_error_t uart3_dma_readBuffer(void) {
  LL_DMA_SetMemoryAddress(DMA1, UART3_DMA_RX_CHANNEL, (u32_t)uart3dma.rx.buffer);
  LL_DMA_SetDataLength(DMA1, UART3_DMA_RX_CHANNEL, sizeof(uart3dma.rx.buffer));
	LL_DMA_EnableChannel(UART3_DMA_RX, UART3_DMA_RX_CHANNEL);
	return uart3_dma_error_success;
}

static uart3_dma_error_t uart3_dma_writeBuffer(u8_t * pB, size_t len, u32_t timeout) {
	uart3_dma_error_t r = uart3_dma_error_success;
  if (len == 0) return r;
	k_mutex_lock(&uart3dma.tx.guardM, K_FOREVER);
	LL_DMA_SetMemoryAddress(UART3_DMA_TX, UART3_DMA_TX_CHANNEL, (uint32_t)pB);
	LL_DMA_SetDataLength(UART3_DMA_TX, UART3_DMA_TX_CHANNEL, len);
	LL_DMA_EnableChannel(UART3_DMA_TX, UART3_DMA_TX_CHANNEL);
	if (k_sem_take(&uart3dma.tx.txDone, K_MSEC(timeout))) {
		r = uart3_dma_error_timeout;
	}
	k_mutex_unlock(&uart3dma.tx.guardM);
	return r;
}

static const uart3_dma_api_t uart3dmaAPI = {
	.init = uart3_dma_init,
	.readBuffer = uart3_dma_readBuffer,
	.writeBuffer = uart3_dma_writeBuffer,
};



/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static int uart3_dma_initilize(struct device *dev) {

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

   memset(&uart3dma, 0, sizeof(uart3dma));

  k_sem_init(&uart3dma.tx.txDone, 0, 1);
  k_mutex_init(&uart3dma.tx.guardM);
  
  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /**USART3 GPIO Configuration  
  PB10   ------> USART3_TX
  PB11   ------> USART3_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART3 DMA Init */
  /* USART3_RX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);
  
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_3, (u32_t)&USART3->DR);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (u32_t)uart3dma.rx.buffer);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, sizeof(uart3dma.rx.buffer));

  /* USART3_TX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, (u32_t)&USART3->DR);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (u32_t)NULL);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, 0);
  // enable transfer complete interrupt
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
  IRQ_DIRECT_CONNECT(DMA1_Channel2_IRQn, UART3_INTR_PRIO, DMA1_Channel2_IRQHandler, 0);
  irq_enable(DMA1_Channel2_IRQn);
  
  LL_USART_InitTypeDef USART_InitStruct = {0};
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART3);
	LL_USART_EnableDMAReq_RX(USART3);
	LL_USART_EnableDMAReq_TX(USART3);
	LL_USART_EnableIT_IDLE(USART3);
	IRQ_DIRECT_CONNECT(USART3_IRQn, UART3_INTR_PRIO, UART3_IRQHandler, 0);
	irq_enable(USART3_IRQn);


  LOG_INF("UART3 DMA initialised");
  return uart3_dma_error_success;
}

DEVICE_AND_API_INIT(uart3dma,UART3_DMA_NAME,&uart3_dma_initilize,NULL, NULL,POST_KERNEL,CONFIG_KERNEL_INIT_PRIORITY_DEVICE,&uart3dmaAPI);
