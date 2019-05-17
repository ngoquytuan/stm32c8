/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_H
#define __UART_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
// USART2 Receiver buffer
#define RX2_BUFFER_SIZE 10
extern uint8_t USART2_index,rx2_data_buff[RX2_BUFFER_SIZE];
void USART1_Init(void);
void USART1_SendStr_(char *str);
void USART2_Init(void);
void USART2_SendStr_(char *str);
void USER_UART_NVIC(void);
void USART1_SendChar_(char ch);
void USART2_SendChar_(char ch);
void u2Transmit(char *str,char length);
#define USE_USART2_DMA
#ifdef USE_USART2_DMA
	#define RX2_DMA_BUF_SIZE 10
	extern uint8_t rx2_dma_[RX2_DMA_BUF_SIZE];
	void USART2_DMAInit(void);
#endif // of USE_USART2_DMA
/*
#ifdef USE_USART2_DMA
#endif
*/
#endif /* __UART_H */
