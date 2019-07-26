#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"
#include "w5500lib_init.h"
#include <stdio.h>
unsigned char W5500RecInt = 0;
static const int8_t WZN_ERR = -1;
/****************************************************************************************************************/
//////////
// TODO //
//////////////////////////////////////////////////////////////////////////////////////////////
// Call back function for W5500 SPI - Theses used as parameter of reg_wizchip_xxx_cbfunc()  //
// Should be implemented by WIZCHIP users because host is dependent                         //
//////////////////////////////////////////////////////////////////////////////////////////////
void  wizchip_select(void);
void  wizchip_deselect(void);
void  wizchip_write(uint8_t wb);
uint8_t wizchip_read();

//Send a byte via SPI1
void spi1_wb(uint8_t wb)
{
	while (  SPI_I2S_GetFlagStatus(SPI1 , SPI_I2S_FLAG_TXE) == RESET );  // sending data Wait
	SPI_I2S_SendData(SPI1 , wb);
	while (  SPI_I2S_GetFlagStatus(SPI1 , SPI_I2S_FLAG_RXNE) == RESET ); // Wait for data
	wb = SPI_I2S_ReceiveData(SPI1);        // Dummy read to generate clock
}
/****************************************************************************************************************/
uint8_t spi1_rb(void)
{
	uint8_t rb;
	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
		;
  SPI_I2S_SendData(SPI1, 0xFF);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET) ;
  rb = SPI_I2S_ReceiveData(SPI1);
	return rb;
}
/****************************************************************************************************************/
/****************************************************************************************************************/
/***************************************************************************************************************/
	void spi1burst_wb(uint8_t *pBuf, uint16_t len)
{
	u16 i;
	for(i=0;i<len;i++)//The loop writes the size bytes of the buffer to the W5500.
	{
		spi1_wb(*pBuf++);//Write a byte of data
	}

}
/***************************************************************************************************************/
	void spi1burst_rb(uint8_t *pBuf, uint16_t len)
{
	u16 i;
	//i=SPI_I2S_ReceiveData(SPI1);
	SPI_I2S_SendData(SPI1,0);//Write 1 byte of data
	i=SPI_I2S_ReceiveData(SPI1);
	for(i=0;i<len;i++)//The loop writes the size bytes of the buffer to the W5500.
	{
		SPI_I2S_SendData(SPI1,0);//Write 1 byte of data
		*pBuf++ = SPI_I2S_ReceiveData(SPI1);
	}

}
/***************************************************************************************************************/

//define of CS pin PA.0
/***************************************************************************************************************/
//W5500 Port initialization and configuration interrupt mode
//The corresponding configuration instructions:
//sck PA5  miso PA6  mosi PA7  rst PA2  int PA3  cs PA0
void W5500_GPIO_Init2(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;	

 	EXTI_InitTypeDef EXTI_InitStructure;			//Interrupt configuration, which interrupt line  EXTI_Line0-15
													//mode EXTI_Mode_Interrupt Interrupt EXTI_Mode_Event event
													//Trigger mode  EXTI_Trigger_Falling Falling edge trigger
													//			EXTI_Trigger_Rising	 Rising edge trigger
													//			EXTI_Trigger_Rising_Falling	  Arbitrary level trigger

 	NVIC_InitTypeDef NVIC_InitStructure;			//Interrupt parameter interrupt priority


  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//Enable the alternate function clock



	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE );//PORTA Clock enable


	// W5500_RST Pin initialization configuration (PA2) 
	GPIO_InitStructure.GPIO_Pin  = W5500_RST;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(W5500_RST_GPIO, &GPIO_InitStructure);
	GPIO_ResetBits(W5500_RST_GPIO, W5500_RST);
	
	// W5500_INT Pin initialization configuration(PA3) 	
	GPIO_InitStructure.GPIO_Pin = W5500_INT;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(W5500_INT_GPIO, &GPIO_InitStructure);


	// Initialize the network module SPI-CS pin (PA0)
	GPIO_InitStructure.GPIO_Pin = W5500_SCS;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(W5500_SCS_GPIO, &GPIO_InitStructure);
	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);



 	// Enable the EXTI3 Interrupt 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;				//W5500_INT External interrupt channel
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//Preemption priority 2 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;			//Subpriority 2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//Enable external interrupt channel
	NVIC_Init(&NVIC_InitStructure);


  	// Connect EXTI Line3 to PA3 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource3);


	// PA3 as W5500 interrupt input 
	EXTI_InitStructure.EXTI_Line = EXTI_Line3;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	
  
}
/***************************************************************************************************************/
// Config SPI1 for communication with w5500
void SPI1_W5500_Init(void)
{
 	  GPIO_InitTypeDef GPIO_InitStructure;
  	SPI_InitTypeDef  SPI_InitStructure;


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1 , ENABLE);
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

 	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);  

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //Set SPI one-way or two-way data mode: SPI is set to two-wire bidirectional full duplex
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;   //
//	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		    //Clock floating low
//	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	    //Data capture on the first clock edge
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//The idle state of the serial synchronous clock is high
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	    //The second transition edge (rising or falling) of the serial synchronous clock is sampled
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		    //NSS signal is managed by hardware (NSS pin) or software (using SSI bit): internal NSS signal has SSI bit control
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;//Configure SPI1 speed to be the highest
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//Specify whether the data transfer starts from the MSB bit or the LSB bit: data transfer starts from the MSB bit
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC Polynomial of value calculation
	SPI_Init(SPI1, &SPI_InitStructure); 
	
	SPI_Cmd(SPI1, ENABLE); //Enable SPI peripherals
	
	/* enable DMA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//	SPI1_ReadWriteByte(0xff);//Start transfer		 
 

}

/***************************************************************************************************************/
	void spiburst_wb(uint8_t *pBuf, uint16_t len)
{
	u16 i;
	wizchip_select();//CS chip selection W5500
	for(i=0;i<len;i++)//The loop writes the size bytes of the buffer to the W5500.
	{
		spi1_wb(*pBuf++);//Write a byte of data
	}
	wizchip_deselect();//Pull high CS to cancel the chip selection

}
/***************************************************************************************************************/
	void spiburst_rb(uint8_t *pBuf, uint16_t len)
{
	u16 i;
	wizchip_select();//CS chip selection W5500
	for(i=0;i<len;i++)//The loop writes the size bytes of the buffer to the W5500.
	{
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET) ;
		*pBuf++ = SPI_I2S_ReceiveData(SPI1);
	}
	wizchip_deselect();//Pull high CS to cancel the chip selection

}
//////////
// TODO //
/////////////////////////////////////////////////////////////////
// SPI Callback function for accessing WIZCHIP                 //
// WIZCHIP user should implement with your host spi peripheral //
/////////////////////////////////////////////////////////////////
void  wizchip_select(void)
{
   GPIO_ResetBits(GPIOA, GPIO_Pin_0);
}

void  wizchip_deselect(void)
{
   GPIO_SetBits(GPIOA, GPIO_Pin_0); 
}

void  wizchip_write(uint8_t wb)
{
	while (  SPI_I2S_GetFlagStatus(SPI1 , SPI_I2S_FLAG_TXE) == RESET );  // sending data Wait
	SPI_I2S_SendData(SPI1 , wb);
	while (  SPI_I2S_GetFlagStatus(SPI1 , SPI_I2S_FLAG_RXNE) == RESET ); // Wait for data
	wb = SPI_I2S_ReceiveData(SPI1);        // Dummy read to generate clock
}

uint8_t wizchip_read()
{
	while (  SPI_I2S_GetFlagStatus(SPI1 , SPI_I2S_FLAG_TXE) == RESET );
	SPI_I2S_SendData(SPI1 , 0xff);     // Dummy write to generate clock
	while (  SPI_I2S_GetFlagStatus(SPI1 , SPI_I2S_FLAG_RXNE) == RESET );
	return (unsigned char)SPI_I2S_ReceiveData(SPI1);
}
//////////////////////////////////////////////////////////////////////////
void delay_cnt(volatile unsigned int nCount)
{
	for(; nCount!= 0;nCount--);
}
/***************************************************************************************************************/
void w5500_lib_init(void){

		uint8_t tmp;
		intr_kind temp;
		uint8_t memsize[2][8] = {{2,2,2,2,2,2,2,2},{2,2,2,2,2,2,2,2}};	
		
		//GPIO, SPI, INT
			W5500_GPIO_Init2();
		GPIO_ResetBits(W5500_RST_GPIO, W5500_RST);
		delay_cnt(5000);
		//////////
   // TODO //
   ////////////////////////////////////////////////////////////////////////////////////////////////////
   // First of all, Should register SPI callback functions implemented by user for accessing WIZCHIP //
   ////////////////////////////////////////////////////////////////////////////////////////////////////
   
		/* Critical section callback - No use in this example */
		//reg_wizchip_cris_cbfunc(0, 0);
			
    /* Chip selection call back */
		reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
		/* SPI Read & Write callback function */
    reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);
		GPIO_SetBits(W5500_RST_GPIO, W5500_RST);//RST High to run
		delay_cnt(10000);
    ////////////////////////////////////////////////////////////////////////
		/* WIZCHIP SOCKET Buffer initialize */
		//Initializes to WIZCHIP with SOCKET buffer size 2 or 1 dimension array typed uint8_t
    if(ctlwizchip(CW_INIT_WIZCHIP,(void*)memsize) == -1)
    {
       printf("WIZCHIP Initialized fail.\r\n");
       while(1);
    }
		
		
		/* PHY link status check */
    
		do
    {
       if(ctlwizchip(CW_GET_PHYLINK, (void*)&tmp) == WZN_ERR)
          printf("Unknown PHY Link stauts.\r\n");
    }while(tmp == PHY_LINK_OFF);
		
		//Cau hinh ngat tren S0
		temp = IK_SOCK_0;
		if(ctlwizchip(CW_SET_INTRMASK, &temp) == WZN_ERR)
		{
			printf("Cannot set imr...\r\n");
		}
		
		
		//reg_wizchip_cs_cbfunc(cs_sel,cs_desel);
	  //reg_wizchip_spi_cbfunc(spi1_rb, spi1_wb);
	  //reg_wizchip_spiburst_cbfunc(spi1burst_rb, spi1burst_wb);

}

void Display_Net_Conf()
{
	uint8_t tmpstr[6] = {0,};
	wiz_NetInfo gWIZNETINFO;

	ctlnetwork(CN_GET_NETINFO, (void*) &gWIZNETINFO);
	ctlwizchip(CW_GET_ID,(void*)tmpstr);

	// Display Network Information
	if(gWIZNETINFO.dhcp == NETINFO_DHCP) printf("\r\n===== %s NET CONF : DHCP =====\r\n",(char*)tmpstr);
		else printf("\r\n===== %s NET CONF : Static =====\r\n",(char*)tmpstr);

	printf("\r\nMAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", gWIZNETINFO.mac[0], gWIZNETINFO.mac[1], gWIZNETINFO.mac[2], gWIZNETINFO.mac[3], gWIZNETINFO.mac[4], gWIZNETINFO.mac[5]);
	printf("IP: %d.%d.%d.%d\r\n", gWIZNETINFO.ip[0], gWIZNETINFO.ip[1], gWIZNETINFO.ip[2], gWIZNETINFO.ip[3]);
	printf("GW: %d.%d.%d.%d\r\n", gWIZNETINFO.gw[0], gWIZNETINFO.gw[1], gWIZNETINFO.gw[2], gWIZNETINFO.gw[3]);
	printf("SN: %d.%d.%d.%d\r\n", gWIZNETINFO.sn[0], gWIZNETINFO.sn[1], gWIZNETINFO.sn[2], gWIZNETINFO.sn[3]);
	printf("DNS: %d.%d.%d.%d\r\n", gWIZNETINFO.dns[0], gWIZNETINFO.dns[1], gWIZNETINFO.dns[2], gWIZNETINFO.dns[3]);
}

void Net_Conf(wiz_NetInfo netinfo)
{
	/*
	wiz_NetInfo gWIZNETINFO = {
		{ 0x00, 0x08, 0xDC, 0x44, 0x55, 0x66 },				// Mac address
		{ 192, 168, 1, 91 },								// IP address
		{ 255, 255, 255, 0},								// Subnet mask
		{ 192, 168, 1, 1},									// Gateway
		{ 8, 8, 8, 8},										// DNS Server
	};
	*/
	ctlnetwork(CN_SET_NETINFO, (void*) &netinfo);

	Display_Net_Conf();
}


/***************************************************************************************************************/