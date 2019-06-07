



#include "spi.h"

//The following is the initialization code of the SPI module, configured into host mode, access SD Card/W25Q64/NRF24L01						  
//SPI Port initialization
//Here the needle is the initialization of SPI2



void SPI1_Init(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
  	SPI_InitTypeDef  SPI_InitStructure;

//	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE );//PORTB  
//	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_SPI1,  ENABLE );//SPI2  	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO, ENABLE);
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //PB13/14/15 Multiplexed push-pull output 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

 	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);  //PB13/14/15иою╜

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //Set SPI one-way or two-way data mode: SPI is set to two-wire bidirectional full duplex
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;   //
//	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		    //Clock floating low
//	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	    //Data capture on the first clock edge
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//The idle state of the serial synchronous clock is high
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	    //The second transition edge (rising or falling) of the serial synchronous clock is sampled
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		    //NSS signal is managed by hardware (NSS pin) or software (using SSI bit): internal NSS signal has SSI bit control
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;//Baud rate prescaler value: baud rate prescaler value is 256. Initial speed is the lowest speed mode.
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//Specify whether the data transfer starts from the MSB bit or the LSB bit: data transfer starts from the MSB bit
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC Polynomial of value calculation
	SPI_Init(SPI1, &SPI_InitStructure); 
 
	SPI_Cmd(SPI1, ENABLE); //Enable SPI peripherals
	
//	SPI1_ReadWriteByte(0xff);//Start transfer		 
 

}   
//SPI Speed setting function
//SpeedSet:
//SPI_BaudRatePrescaler_2   2Frequency division   
//SPI_BaudRatePrescaler_8   8Frequency division  
//SPI_BaudRatePrescaler_16  16Frequency division  
//SPI_BaudRatePrescaler_256 256Frequency division 
  
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)
{
  	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI1->CR1&=0XFFC7;
	SPI1->CR1|=SPI_BaudRatePrescaler;	//Set SPI speed
	SPI_Cmd(SPI1,ENABLE); 

} 

//SPIx Read and write a byte
//TxData:The byte to be written
//Return value: the byte read
u8 SPI1_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //Check if the specified SPI flag is set or not: send buffer empty flag
		{
		retry++;
		if(retry>200)return 0;
		}			  
	SPI_I2S_SendData(SPI1, TxData); //Send a data via peripheral SPIx
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //Check if the specified SPI flag is set or not: accept the cache non-empty flag
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	return SPI_I2S_ReceiveData(SPI1); //Return data recently received via SPIx					    
}








































