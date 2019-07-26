/*******************************************************
This program was created by tuannq

Project : Test on China C8T6 kit
Version : 1
Date    : 08/May/2018
Author  : tuannq
Company : None
Comments: 
6/June/2019 : Tich hop voi thu vien W5500 cua hang ok
=> Viet ban tin xu ly thoi gian 
15/May/2019 : Kiem tra các thu vien co ban : UART, SPI ok.
=> Lam chu thu vien W5500 co ban.

Chip type               : STM32F103C8T6
Program type            : Examples
External Clock frequency: 8.000000 MHz

The primary differences between MicroLib and the standard C library are:

MicroLib is designed for deeply embedded applications.
MicroLib is optimized to use less code and data memory than using the ARM standard library.
MicroLib has been designed to work without an operating system, however this does not prevent it being used together with any OS or RTOS such as Keil RTX.
MicroLib contains no file I/O or wide character support.
As MicroLib has been optimized to minimize code size, some functions will execute more slowly than the standard C library routines available in the ARM compilation tools.
Both MicroLib and the ARM Standard Library are included in the Keil MDK-ARM.
See Differences from the default C library for more detailed information
http://www.keil.com/arm/microlib.asp
*******************************************************/

#include <stdio.h>
#include <stdlib.h>
#include "GPIO_STM32F10x.h"
//Kiem tra cac ham include trong main.h
#include "main.h"
//Add the following code to support the printf function without having to choose use MicroLIB	  

//Kiem tra dinh nghia fputc
//Redefining low-level library functions to enable direct use of high-level library functions in the C library
//http://www.keil.com/support/man/docs/armlib/armlib_chr1358938931411.htm
int fputc(int ch, FILE *f)
{      
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET)
		; 
    USART_SendData(USART1,(char)ch);   
	return ch;
}

//#define LED(state) GPIO_PinWrite(GPIOA, 11, state);


///* Private typedef -----------------------------------------------------------*/
///* Private define ------------------------------------------------------------*/
///* Private macro -------------------------------------------------------------*/
///* Private variables ---------------------------------------------------------*/

//GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_9)));
int main(void)
{
	
	hardware_init();

	while(1)
	{
		/* Update WWDG counter */
		//WWDG_SetCounter(127);
		IWDG_ReloadCounter(); // Feed IWDG
    tasks();		
		//modbus_slave_exe();
		
	}

		
}






