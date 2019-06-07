#include <stdio.h>
#include "GPIO_STM32F10x.h"
#include "uart.h"
//#include "misc.h"
//#include "stm32f10x_gpio.h"
/*
* USART DMA setting on STM32
 * USART1 Tx --> DMA Channel 4
 * USART1 Rx --> DMA Channel 5
 * USART2 Tx --> DMA Channel 7
 * USART2 Rx --> DMA Channel 6
 * USART3 Tx --> DMA Channel 2
 * USART3 Rx --> DMA Channel 3
 
*/

uint8_t USART2_index=0,rx2_data_buff[RX2_BUFFER_SIZE];

#ifdef USE_USART2_DMA
	uint8_t rx2_dma_[RX2_DMA_BUF_SIZE];
#endif

void USART1_Init(void)
{
	USART_InitTypeDef USARTx_Init;
	
	USARTx_Init.USART_WordLength = USART_WordLength_8b;
	USARTx_Init.USART_StopBits = USART_StopBits_1;
	USARTx_Init.USART_Parity = USART_Parity_No;
	USARTx_Init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USARTx_Init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USARTx_Init.USART_BaudRate = 115200;
	
	/* Enable UART clock, if using USART2 or USART3 ... we must use RCC_APB1PeriphClockCmd(RCC_APB1Periph_USARTx, ENABLE) */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);
	USART_DeInit(USART1);
	GPIO_PortClock   (GPIOA, true);
	GPIO_PinWrite    (GPIOA, 9, 0);
	GPIO_PinConfigure(GPIOA, 9,
										GPIO_AF_PUSHPULL, 
										GPIO_MODE_OUT50MHZ);
	
	/* Configure PA9 for USART Tx as alternate function push-pull */
	GPIO_PinConfigure(GPIOA, 10,
										GPIO_IN_FLOATING,
										GPIO_MODE_INPUT);
	/* Configure PA10 for USART Rx as input floating */	
	
		/* USART configuration */
	USART_Init(USART1, &USARTx_Init);
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	//USART_ITConfig(USART1,USART_IT_TXE,ENABLE);
	/* Enable USART */
	USART_Cmd(USART1, ENABLE);
	//Enable USART_IT_RXNE
	USER_UART_NVIC();
}

void USART1_SendChar_(char ch) {
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); // wait for "Transmission Complete" flag cleared
	USART_SendData(USART1,ch);
}

void USART1_SendStr_(char *str) {
	while (*str) USART1_SendChar_(*str++);
}

void USART2_Init(void)
{
	USART_InitTypeDef USARTx_Init;
	NVIC_InitTypeDef  NVIC_InitStructure;
	
	USARTx_Init.USART_WordLength = USART_WordLength_8b;
	USARTx_Init.USART_StopBits = USART_StopBits_1;
	USARTx_Init.USART_Parity = USART_Parity_No;
	USARTx_Init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USARTx_Init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USARTx_Init.USART_BaudRate = 115200;
	
	/* Enable UART clock, if using USART2 or USART3 ... we must use RCC_APB1PeriphClockCmd(RCC_APB1Periph_USARTx, ENABLE) */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	GPIO_PortClock   (GPIOA, true);
	GPIO_PinWrite    (GPIOA, 2, 0);
	GPIO_PinConfigure(GPIOA, 2,
										GPIO_AF_PUSHPULL, 
										GPIO_MODE_OUT50MHZ);
	
	/* Configure PA2 for USART Tx as alternate function push-pull */
	GPIO_PinConfigure(GPIOA, 3,
										GPIO_IN_FLOATING,
										GPIO_MODE_INPUT);
	/* Configure PA3 for USART Rx as input floating */
		/* USART configuration */
	USART_Init(USART2, &USARTx_Init);
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	/* Enable USART */
	USART_Cmd(USART2, ENABLE);
	
	/* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

#ifdef USE_USART2_DMA
 /*******************************************************************************
Noi Dung    :   Su dung DMA de xu ly nhan va xu ly du lieu tu USART2
Tham Bien   :   Ko.
Tra Ve      :   Khong.
********************************************************************************/
void USART2_DMAInit(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	USART_InitTypeDef USARTx_Init;
	NVIC_InitTypeDef  NVIC_InitStructure;
	
	/* Enable clock DMA1 */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	/* Enable UART clock, if using USART2 or USART3 ... we must use RCC_APB1PeriphClockCmd(RCC_APB1Periph_USARTx, ENABLE) */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	GPIO_PortClock   (GPIOA, true);
	GPIO_PinWrite    (GPIOA, 2, 0);
	GPIO_PinConfigure(GPIOA, 2,
										GPIO_AF_PUSHPULL, 
										GPIO_MODE_OUT50MHZ);
	
	/* Configure PA2 for USART Tx as alternate function push-pull */
	GPIO_PinConfigure(GPIOA, 3,
										GPIO_IN_FLOATING,
										GPIO_MODE_INPUT);
	/* Configure PA3 for USART Rx as input floating */
	
	USARTx_Init.USART_WordLength = USART_WordLength_8b;
	USARTx_Init.USART_StopBits = USART_StopBits_1;
	USARTx_Init.USART_Parity = USART_Parity_No;
	USARTx_Init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USARTx_Init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USARTx_Init.USART_BaudRate = 115200;
	
		/* USART configuration */
	USART_Init(USART2, &USARTx_Init);
	
	
	/* DMA1 channel1 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel6);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);//UART2_RX_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)rx2_dma_;
	DMA_InitStructure.DMA_BufferSize = RX2_DMA_BUF_SIZE;
	
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel6, &DMA_InitStructure);
		/* Enable DMA1 channel1 */
	DMA_Cmd(DMA1_Channel6, ENABLE);
	/* Enable USART with DMA RX*/
	USART_Cmd(USART2, ENABLE);
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);
	/* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
#endif
void USER_UART_NVIC(void)
{
  NVIC_InitTypeDef  NVIC_InitStructure;
  
	/* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
}	
void USART2_SendChar_(char ch) {
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET); // wait for "Transmission Complete" flag cleared
	USART_SendData(USART2,ch);
}
void u2Transmit(char *str,char length)
{
  while(length)
	{
	  USART2_SendChar_(*str++);
		length--;
	}
}
void USART2_SendStr_(char *str) {
	while (*str) USART2_SendChar_(*str++);
}




