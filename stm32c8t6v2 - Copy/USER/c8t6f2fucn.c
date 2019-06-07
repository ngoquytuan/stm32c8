#include "main.h"
#include "GPIO_STM32F10x.h"
#include "time.h"
#include "socket.h"
#include "w5500lib_init.h"

/*
Description
Returns the number of microseconds since the Arduino board began running the current program. 
This number will overflow (go back to zero), after approximately 70 minutes. 
On 16 MHz Arduino boards (e.g. Duemilanove and Nano), this function has a resolution of four microseconds 
(i.e. the value returned is always a multiple of four). On 8 MHz Arduino boards (e.g. the LilyPad),
this function has a resolution of eight microseconds.
*/
uint32_t micros()
{
return 1;
}
uint32_t getns(void);
void resetns(void);

// Time Server Port
#define NTP_PORT 123
static const int NTP_PACKET_SIZE = 48;
/* SNTP Packet array */
uint8_t serverPacket[NTP_PACKET_SIZE] = {0};
uint8_t clientPacket[100] = {0};
uint32_t micros_recv = 0;
uint32_t micros_offset;
uint32_t micros_transmit = 0;
/* Shifts usecs in unixToNtpTime */
//??? ko hieu nhung dung! 
#ifndef USECSHIFT
#define USECSHIFT (1LL << 32) * 1.0e-6
#endif
#define STARTOFTIME 2208988800UL

#ifndef UTIL_H
#define UTIL_H

#define htons(x) ( ((x)<< 8 & 0xFF00) | \
                   ((x)>> 8 & 0x00FF) )
#define ntohs(x) htons(x)

#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)
#endif
/////////////////////////////////////////
// SOCKET NUMBER DEFINION for Examples //
/////////////////////////////////////////
#define SOCK_TCPS        1
#define SOCK_UDPS        0
////////////////////////////////////////////////
// Shared Buffer Definition for LOOPBACK TEST //
////////////////////////////////////////////////
#define DATA_BUF_SIZE   100
uint8_t gDATABUF[DATA_BUF_SIZE];
/* Ban tin chuan server time.nist.gov 48 bytes (https://www.cisco.com/c/en/us/about/press/internet-protocol-journal/back-issues/table-contents-58/154-ntp.html)
00.01.02.03: 1C.01.0D.E3 [LI][VN][Mode].[Stratum].[Poll].[Precision]
04.05.06.07: 00.00.00.10 [Root delay]:The total round-trip delay from the server to the primary reference sourced. The value is a 32-bit signed fixed-point number in units of seconds, with the fraction point between bits 15 and 16. This field is significant only in server messages.
08.09.10.11: 00.00.00.20 [Root Dispersion]: The maximum error due to clock frequency tolerance. The value is a 32-bit signed fixed-point number in units of seconds, with the fraction point between bits 15 and 16. This field is significant only in server messages.
12.13.14.15: 4E.49.53.54 [Reference Identifier]: Name of sender: NIST. For stratum 1 servers this value is a four-character ASCII code that describes the external reference source (refer to Figure 2). For secondary servers this value is the 32-bit IPv4 address of the synchronization source, or the first 32 bits of the Message Digest Algorithm 5 (MD5) hash of the IPv6 address of the synchronization source.
16.17.18.19: E0.28.BA.7F [Reference Timestamp]: unsigned 32-bit seconds value
20.21.22.23: 00.00.00.00       						32-bit fractional
24.25.26.27: 00.00.00.00 [Originate Timestamp, T1]: unsigned 32-bit seconds value
28.29.30.31: 00.00.00.00         					32-bit fractional
32.33.34.35: E0.28.C5.79 [Receive Timestamp, T2]: unsigned 32-bit seconds value
36.37.38.39: E5.7A.55.F6      						32-bit fractional
40.41.42.43: E0.28.C5.79 [Transmit Timestamp, T3]: unsigned 32-bit seconds value
44.45.46.47: E5.7A.5F.93      						32-bit fractional


The next four fields use a 64-bit time-stamp value. This value is an unsigned 32-bit seconds value, and a 32-bit fractional part. In this notation the value 2.5 would be represented by the 64-bit string:

0000|0000|0000|0000|0000|0000|0000|0010.|1000|0000|0000|0000|0000|0000|0000|0000

The unit of time is in seconds, and the epoch is 1 January 1900, meaning that the NTP time will cycle in the year 2036 (two years before the 32-bit Unix time cycle event in 2038).

The smallest time fraction that can be represented in this format is 232 picoseconds.

Reference Timestamp   	This field is the time the system clock was last set or corrected, in 64-bit time-stamp format.

Originate Timestamp   	This value is the time at which the request departed the client for the server, in 64-bit time-stamp format. Th?i di?m b?n tin di t? th?ng h?i

Receive Timestamp	This value is the time at which the client request arrived at the server in 64-bit time-stamp format. Th?i di?m th?ng server nh?n du?c

Transmit Timestamp	This value is the time at which the server reply departed the server, in 64-bit time-stamp format. th?i di?m ph?n h?i t? server
*/
/**
 * @ingroup DATA_TYPE
 *  Network Information for WIZCHIP
 */
typedef struct ntp_Info_t
{
   //uint8_t LI;  			///[LI][VN][Mode]
   //uint8_t Vers;   		///< 
   //uint8_t Mode;   		///<  Mask 
	 uint8_t LiVeMod;  			///[LI][VN][Mode]
   uint8_t stratum;   ///[Stratum] 
   uint8_t polling;  	///[Poll]
   uint8_t precision; ///[Precision]
	 int32_t rootDelay; //[Root delay]:The total round-trip delay from the server to the primary reference sourced.
	 int32_t rootDisper;//[Root Dispersion]: The maximum error due to clock frequency tolerance. The value is a 32-bit signed fixed-point number in units of seconds, with the fraction point between bits 15 and 16. This field is significant only in server messages.
	 uint8_t refID[4];  //[Reference Identifier]: Name of sender: NIST. For stratum 1 servers this value is a four-character ASCII code that describes the external reference source (refer to Figure 2). For secondary servers this value is the 32-bit IPv4 address of the synchronization source, or the first 32 bits of the Message Digest Algorithm 5 (MD5) hash of the IPv6 address of the synchronization source.
	 uint32_t refTime ; //[Reference Timestamp]: unsigned 32-bit seconds value
	 uint32_t refTimeFrac; //[Reference Timestamp]: unsigned 32-bit 32-bit fractional
	 uint32_t OrigTime; //[Originate Timestamp, T1]: unsigned 32-bit seconds value
	 uint32_t OrigTimeFrac; //[Originate Timestamp, T1]: unsigned 32-bit fractional
	 uint32_t RecTime; //[Receive Timestamp, T2]: unsigned 32-bit seconds value
	 uint32_t RecTimeFrac; //[Receive Timestamp, T2]: unsigned 32-bit fractional
	 uint32_t TransTime; //[Transmit Timestamp, T3]: unsigned 32-bit seconds value
	 uint32_t TransTimeFrac; //[Transmit Timestamp, T3]: unsigned 32-bit fractional
}ntp_server;
ntp_server myNtpServer = {
													.LiVeMod = 0b11100011,    // LI, Version, Mode
													.stratum = 0,							// Stratum, or type of clock
													.polling = 6,// Polling Interval
													.precision =0xEC,  // Peer Clock Precision
													.rootDelay =1,
													.rootDisper =1,
													.refID ="GPS ",
													.refTime =1,
													.refTimeFrac =1,
													.OrigTime =1,
													.OrigTimeFrac =1,
													.RecTime =1, 
													.RecTimeFrac =1,
													.TransTime =1,
													.TransTimeFrac =1
};
///////////////////////////////////
// Default Network Configuration //
///////////////////////////////////
wiz_NetInfo gWIZNETINFO = { .mac = {0x0c, 0x29, 0x34,0x7c, 0xab, 0xcd},
                            .ip = {192, 168, 1, 3},
                            .sn = {255,255,255,0},
                            .gw = {192, 168, 1, 1},
                            .dns = {8,8,8,8},
                            .dhcp = NETINFO_STATIC };

wiz_NetInfo netinfo;
void loadNetParas(void);
//////////////////////////////////////////////////////////////////////////////////////////////
														
//////////////////////////////////
// For example of ioLibrary_BSD //
//////////////////////////////////
void network_init(void);								// Initialize Network information and display it
int32_t loopback_tcps(uint8_t, uint8_t*, uint16_t);		// Loopback TCP server
int32_t loopback_udps(uint8_t, uint8_t*, uint16_t);		// Loopback UDP server
int32_t NTPUDP(uint8_t sn, uint8_t* buf);//UDP NTP server
void wzn_event_handle(void);
/*******************************************************************************/
/**
  * @brief  Watch D0G
  * @param  None
  * @retval  Dong nay o cuoi cung ko thi ko chay : RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
  */
  void WWDG_Init(void)
  {
	  /* On Value line devices, WWDG clock counter = (PCLK1 (24MHz)/4096)/8 = 732 Hz (~1366 us)  */
	/* On other devices, WWDG clock counter = (PCLK1(36MHz)/4096)/8 = 1099 Hz (~910 us)  */
	  WWDG_SetPrescaler(WWDG_Prescaler_8);

	  /* Set Window value to 80; WWDG counter should be refreshed only when the counter
		is below 80 (and greater than 64) otherwise a reset will be generated */
	  WWDG_SetWindowValue(80);

	  /* - On Value line devices,
		Enable WWDG and set counter value to 127, WWDG timeout = ~1366 us * 64 = 87.42 ms 
		In this case the refresh window is: ~1366us * (127-80) = 64.20 ms < refresh window < ~1366us * 64 = 87.42ms
		 - On other devices
		Enable WWDG and set counter value to 127, WWDG timeout = ~910 us * 64 = 58.25 ms 
		In this case the refresh window is: ~910 us * (127-80) = 42.77 ms < refresh window < ~910 us * 64 = 58.25ms     
	  */
		/* Enable WWDG clock */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
	  WWDG_Enable(127);
  }
  

/**
  * @brief  CRC8 caculator swat 25.03.2017
  * @param  None
  * @retval  Dong nay o cuoi cung ko thi ko chay : RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
  */
uint8_t crc8(uint8_t *datainput,uint8_t datalength)
{
  uint16_t value,iscan;
  value = datainput[0];
  for(iscan=1;iscan<datalength;iscan++)
    {
      value += datainput[iscan];
      if(value > 0xFF) value -= 256;
      }
      return value;
  }
/**
  * @brief  HEXInStringToDec
  * @param  None
  * @retval  
  */						
unsigned char HEXInStringToDec(unsigned char data)
      {
            if((data>47)&&(data<58)) return (data-48);// 0...9
            else if((data>63)&&(data<71)) return (data-55);//A..F
						else return 0;
      }		

#ifdef USE_EMU_EEPROM // Dung flash cua stm32 lam eeprom
/**
  * @brief  test_eeprom
  * @param  Call this fuction for test store data to eeprom
  * @retval  
  */	
void test_eeprom(void)
{
	uint16_t a,b,c,d,e,f; 
	EE_ReadVariable(0,&a);
	EE_ReadVariable(1,&b);
	EE_ReadVariable(2,&c);
	EE_ReadVariable(3,&d);
	EE_ReadVariable(4,&e);
	EE_ReadVariable(5,&f);
	if(a != 'E')
	{
	EE_WriteVariable(0,'E');
	EE_WriteVariable(1,'E');
	EE_WriteVariable(2,'P');
	EE_WriteVariable(3,'R');
	EE_WriteVariable(4,'0');
	EE_WriteVariable(5,'m');
	EE_WriteVariable(6,178);
	printf("EEPROM Error, save agian!\r\n");
	}
	else printf("EEPROM OK:%c%c%c%c%c%c",a,b,c,d,e,f);

}	
/**
  * @brief  sw_eeprom_stm32
  * @param  make some flash blocks come eeprom for store data
  * @retval  just call this fuction
  */	
void sw_eeprom_stm32(void)
{
		/* Unlock the Flash Program Erase controller */
  FLASH_Unlock();

  /* EEPROM Init */
  if(EE_Init() == FLASH_COMPLETE) printf("EEPROM STM32 ready !\r\n");
	
}		
#endif

/**
* @brief  GPIO_config cho kit C8T6 china : LED PA1
  * @param  None
  * @retval  
  */
void GPIO_config(void)
{
  //LED PA1 : cho kit C8T6 china : LED PA1
	GPIO_PortClock   (GPIOA, true);
	GPIO_PinConfigure(GPIOA, 1, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT10MHZ);
	GPIO_PinWrite(GPIOA, 1, 0);
}
/**
  * @brief  TEST_24C32
  * @param  Kiem tra giao tiep voi 24C32
  * @retval  
  */
#ifdef TEST_24C32 // Kiem tra giao tiep voi 24C32
void E24C32Test (void)
{
	uint8_t i;
	uint8_t data[11]={10,110,12,150,20,4,2,50,30,6,4};
	//EEPROM_WriteReg(0,1);
	//EEPROM_W_Regs(1,10,data);
	//printf("Write:%d\r\n",EEPROM_W_Regs(1,12,"Ngo Quy TUan"));
//	for(i=0;i<30;i++)
//	{
//		EEPROM_WriteReg(i,4);
//		delay_ms(3);
//	}
	EEPROM_W_Regs(0,14,".STM32........");
	for(i=0;i<15;i++)
	{
		printf("MEM %d: %c\r\n",i,EEPROM_ReadReg(i));
	}
//	printf("EEPROM_WriteReg(0x01,11):%d\r\n",EEPROM_WriteReg(0x01,0));
//	printf("EEPROM_WriteReg(0x01,11):%d\r\n",EEPROM_WriteReg(0x02,0));
//	printf("EEPROM_WriteReg(0x01,11):%d\r\n",EEPROM_WriteReg(0x03,0));
//	printf("EEPROM_WriteReg(0x01,11):%d\r\n",EEPROM_WriteReg(0x04,0));
//	printf("EEPROM_WriteReg(0x01,11):%d\r\n",EEPROM_WriteReg(0x05,0));
//	printf("EEPROM_WriteReg(0x01,11):%d\r\n",EEPROM_WriteReg(0x06,0));
//	printf("Write:%d\r\n",EEPROM_W_Regs(1,12,"Ngo Quy TUan"));
//	printf("EEPROM 0x00: %d\r\n",EEPROM_ReadReg(0));
//	printf("EEPROM 0x01: %d\r\n",EEPROM_ReadReg(1));
//	printf("EEPROM 0x02: %d\r\n",EEPROM_ReadReg(2));
//	printf("EEPROM 0x03: %d\r\n",EEPROM_ReadReg(3));
//	printf("EEPROM 0x04: %d\r\n",EEPROM_ReadReg(4));
//	printf("EEPROM 0x05: %d\r\n",EEPROM_ReadReg(5));
//	printf("EEPROM 0x06: %d\r\n",EEPROM_ReadReg(6));
//	printf("EEPROM 0x07: %d\r\n",EEPROM_ReadReg(7));
}	
#endif

/**
  * @brief  tasks
  * @param  Tat ca cac chuong trinh con se duoc goi tu day!
  * @retval  
  */
void tasks(void)
{
	int16_t adc0, adc1, adc2, adc3;  	
	int32_t ret = 0;
	
		
			#ifdef USE_INTERNAL_RTC
				/* If 1s has been elapsed */
				if (TimeDisplay == 1)
				{
					TimeDisplay = 0;
					//printf("ADC!\r\n");
					//adc0 = readADC_SingleEnded(0);
					//adc1 = readADC_SingleEnded(1);
					//adc2 = readADC_SingleEnded(2);
					//adc3 = readADC_SingleEnded(3);
					//printf("ADS:%d,%d,%d,%d\r\n",(adc0),adc1,adc2,adc3);
					#ifdef TEST_INTERNAL_RTC
					/* Display current time */
					//Time_Display(RTC_GetCounter());
					#endif
				}
			#endif
		if(task100ms == ONTIME)
		{
			task100ms = 100;
			//printf("Kiem tra 100ms\r\n");
			#ifdef TEST_ADC
				u = ADCConvertedValue;
				u_kalman = updateEstimate(u);
				printf("%2.1f,%2.1f\r\n",u,u_kalman);
			#endif
			
		}
		#ifdef USE_UART1 // Xu ly U1 buffer
			//UART1 RX process
			if(u1out == ONTIME)
			{
				u1out = STOP;// Da nhan du ban tin UART => Xy ly
				printf("UART1:%s\r\n",USART1_rx_data_buff);
				for(USART1_index=0;USART1_index<RX_BUFFER_SIZE0;USART1_index++)
															{
															USART1_rx_data_buff[USART1_index]=0;
															}  
															USART1_index=0;
			}
		#endif

		#ifdef USE_UART2
			//UART2 RX process
			#ifdef USE_USART2_DMA
				if(u2out == ONTIME)
						{
							u2out = STOP;// Da nhan du ban tin UART => Xy ly
							cach nay ko dc vi hoi ti no lai vao idle -_-
							printf("UART2:%s\r\n",rx2_dma_);
							
							for(USART2_index=0;USART2_index<(RX2_BUFFER_SIZE);USART2_index++)
																		{
																		 rx2_dma_[USART2_index]=0;
																		}  
							USART2_index=0;
							
							/* Reset DMA1 Channel6 remaining bytes register */
							DMA1_Channel6->CNDTR = 10;											
						}
			#else
							if(u2out == ONTIME)
						{
							u2out = STOP;// Da nhan du ban tin UART => Xy ly
							printf("UART2:%s\r\n",rx2_data_buff);
							
							//for modbus slave
							//modbus_slave_exe(rx2_data_buff, USART2_index);
							//for modbus master
							//modbus_master_exe(rx2_data_buff, USART2_index);
							
							for(USART2_index=0;USART2_index<RX2_BUFFER_SIZE;USART2_index++)
																		{
																		 rx2_data_buff[USART2_index]=0;
																		}  
																		USART2_index=0;
						}
			#endif

		#endif

  
		
/* Loopback Test */
    	// TCP server loopback test
    	if( (ret = loopback_tcps(SOCK_TCPS, gDATABUF, 5000)) < 0) {
			printf("SOCKET ERROR : %d\r\n", ret);
		}
			
		/*
    	// UDP server loopback test
		if( (ret = loopback_udps(SOCK_UDPS, clientPacket, 1234)) < 0) {
			printf("SOCKET ERROR : %d\r\n", ret);
		}*/
		// NTP UDP server test
		
		/*
		if( (ret = NTPUDP(SOCK_UDPS, clientPacket)) < 0) {
			printf("SOCKET ERROR : %d\r\n", ret);
		}*/
/*
		// UDP ngat
		if(W5500RecInt == 1)
		{
			W5500RecInt = 0;
			//printf("Co' nga't : %d\r\n", ret);
			wzn_event_handle();
		}
		*/

		if(TimingDelay>5000) 
		{
			TimingDelay = 0;
			//printf("Get ns :%d\r\n",getns());
			//resetns();
			//count1ms = 0;
			//TIM_SetCounter(TIM3,0);
			//micros_transmit = 123;//0.01s
			
			//printf("micros_transmit :%d-",micros_transmit);
			//micros_transmit = (micros_transmit + 1) * USECSHIFT;
			//printf("htonl(micros_transmit) :%u\r\n",micros_transmit);
		}
		
}

/**
  * @brief  tra ve ns bang sysstick vs TIM3
  * @param  None
  * @retval  
  */
uint32_t getns(void)
{
 return 1000*count1ms+TIM_GetCounter(TIM3);
}
void resetns(void)
{
	count1ms = 0;
	TIM_SetCounter(TIM3,0);
}	

/**
  * @brief  hardware_init
  * @param  None
  * @retval  
  */
// Check mcu clock
RCC_ClocksTypeDef mcu_clk;

void hardware_init(void)
{
	
	SystemInit();
	SystemCoreClockUpdate();
	/* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(SystemCoreClock / 1000))
  { 
    /* Capture error */ 
    while (1);
  }
	
	//GPIO config
	GPIO_config();
	
	/* Check if the system has resumed from WWDG reset */
  if (RCC_GetFlagStatus(RCC_FLAG_WWDGRST) != RESET)
  { 
    /* WWDGRST flag set */
    /* Turn on LED1 */
    GPIO_PinWrite(GPIOA, 1, 0);
    /* Clear reset flags */
    RCC_ClearFlag();
  }
  else
  {
    /* WWDGRST flag is not set */
    /* Turn off LED1 */
    GPIO_PinWrite(GPIOA, 1, 1);
  }
	
  
	#ifdef USE_UART1
		USART1_Init();
	#endif
	#ifdef USE_UART2
		#ifdef USE_USART2_DMA
			USART2_DMAInit();
			//printf("USART2_DMAInit\r\n");
		#else
			USART2_Init();
			//printf("USART2_Init\r\n");
		#endif
	#endif
	
	delay_ms(1);
	
	
	#ifdef TEST_UART
	//check clock CPU
	USART1_SendStr_("Uart1 chay nay, C8T6f2!!!\r\n");
	RCC_GetClocksFreq(&mcu_clk);
	printf(">Thach anh: \r\nADCCLK:%d\r\nHCLK:%d\r\nPCLK1:%d\r\nPCLK2:%d\r\nSYSCLK:%d",
																mcu_clk.ADCCLK_Frequency,mcu_clk.HCLK_Frequency,
																mcu_clk.PCLK1_Frequency,mcu_clk.PCLK2_Frequency,mcu_clk.SYSCLK_Frequency);
	#endif

	
	#ifdef USE_EMU_EEPROM
		//using stm32's flash to store data
		//EEPROM STM32 init
		sw_eeprom_stm32();
		#ifdef TEST_EMU_EEPROM
		test_eeprom();
		#endif
	#endif
	
	#ifdef USE_LCD16x2
		LCD_Init();
		LCD_Clear();
		#ifdef TEST_LCD16x2
			LCD_Puts("Market is OPEN");
			LCD_Gotoxy(1,0);
			LCD_Puts(" in love? NO!");
		#endif
	#endif
	
	#ifdef USE_INTERNAL_RTC
	RTC_Init();
	#endif
  
	#ifdef USE_ADC_DMA
		//ADC on PA1 using DMA init
		adcPA1_DMA_init();
		#ifdef USE_KALMAN
			SimpleKalmanFilter(2.0,2.0,0.001);
		#endif
	#endif
	
	/* WWDG configuration */
  //WWDG_Init();
	
	#ifdef TEST_TIM
		//Run some timer examples
		tim_ex();
	#endif


	/* Initialize the I2C EEPROM driver ----------------------------------------*/ 
	#ifdef USE_I2C
		if(I2C_Config()==0) printf("I2C init failed!\r\n");
		
		#ifdef USE_24C32
		if(EEPROM_init()==0) printf("EEPROM init failed!\r\n");
			#ifdef TEST_24C32
				E24C32Test();
			#endif
		
		#endif
		
		//if(ADS1115_init() ==1) printf("ADS1115 init done!\r\n");;
		
	#endif
	
	  TIM_init();
		
		//W5500_GPIO_Init2();
		w5500_lib_init();
    printf("\r\nLoad W5500 config!\r\n");

    /* Network initialization */
    network_init();
	
		//micros_offset = micros()%1000000;
}


			



/////////////////////////////////////////////////////////////
// Intialize the network information to be used in WIZCHIP //
/////////////////////////////////////////////////////////////
void network_init(void)
{
   uint8_t tmpstr[6];
	//loadNetParas();
	// Set Network information from netinfo structure
	ctlnetwork(CN_SET_NETINFO, (void*)&gWIZNETINFO);
	printf("\r\nSet Network information\r\n");
	
		// Get Network information
	ctlnetwork(CN_GET_NETINFO, (void*)&netinfo);

	// Display Network Information
	ctlwizchip(CW_GET_ID,(void*)tmpstr);

	if(netinfo.dhcp == NETINFO_DHCP) printf("\r\n=== %s NET CONF : DHCP ===\r\n",(char*)tmpstr);
	else printf("\r\n=== %s NET CONF : Static ===\r\n",(char*)tmpstr);

	printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",netinfo.mac[0],netinfo.mac[1],netinfo.mac[2],netinfo.mac[3],netinfo.mac[4],netinfo.mac[5]);
	printf("SIP: %d.%d.%d.%d\r\n", netinfo.ip[0],netinfo.ip[1],netinfo.ip[2],netinfo.ip[3]);
	printf("GAR: %d.%d.%d.%d\r\n", netinfo.gw[0],netinfo.gw[1],netinfo.gw[2],netinfo.gw[3]);
	printf("SUB: %d.%d.%d.%d\r\n", netinfo.sn[0],netinfo.sn[1],netinfo.sn[2],netinfo.sn[3]);
	printf("DNS: %d.%d.%d.%d\r\n", netinfo.dns[0],netinfo.dns[1],netinfo.dns[2],netinfo.dns[3]);
	printf("===============END=============\r\n");
}
/////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////
// Loopback Test Example Code using ioLibrary_BSD			       //
///////////////////////////////////////////////////////////////
int32_t loopback_tcps(uint8_t sn, uint8_t* buf, uint16_t port)
{
   int32_t ret;
   uint16_t size = 0, sentsize=0;
   switch(getSn_SR(sn))
   {
      case SOCK_ESTABLISHED :
         if(getSn_IR(sn) & Sn_IR_CON)
         {
            printf("%d:Connected\r\n",sn);
            setSn_IR(sn,Sn_IR_CON);
         }
         if((size = getSn_RX_RSR(sn)) > 0)
         {
            if(size > DATA_BUF_SIZE) size = DATA_BUF_SIZE;
            ret = recv(sn,buf,size);
            if(ret <= 0) return ret;
            sentsize = 0;
            while(size != sentsize)
            {
               ret = send(sn,buf+sentsize,size-sentsize);
               if(ret < 0)
               {
                  close(sn);
                  return ret;
               }
               sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
            }
         }
         break;
      case SOCK_CLOSE_WAIT :
         printf("%d:CloseWait\r\n",sn);
         if((ret=disconnect(sn)) != SOCK_OK) return ret;
         printf("%d:Closed\r\n",sn);
         break;
      case SOCK_INIT :
    	  printf("%d:Listen, port [%d]\r\n",sn, port);
         if( (ret = listen(sn)) != SOCK_OK) return ret;
         break;
      case SOCK_CLOSED:
         printf("%d:LBTStart\r\n",sn);
         if((ret=socket(sn,Sn_MR_TCP,port,0x00)) != sn)
            return ret;
         printf("%d:Opened\r\n",sn);
         break;
      default:
         break;
   }
   return 1;
}

int32_t loopback_udps(uint8_t sn, uint8_t* buf, uint16_t port)
{
   int32_t  ret;
   uint16_t size, sentsize;
   uint8_t  destip[4];
   uint16_t destport;
   //uint8_t  packinfo = 0;
   switch(getSn_SR(sn))
   {
      case SOCK_UDP :
         if((size = getSn_RX_RSR(sn)) > 0)
         {
					    //printf("size:%d\r\n",size);
					  if(size > DATA_BUF_SIZE) size = DATA_BUF_SIZE;
            ret = recvfrom(sn,buf,size,destip,(uint16_t*)&destport);
            if(ret <= 0)
            {
               printf("%d: recvfrom error. %d\r\n",sn,ret);
               return ret;
            }
            size = (uint16_t) ret;
            sentsize = 0;
            while(sentsize != size)
            {
               ret = sendto(sn,buf+sentsize,size-sentsize,destip,destport);
               if(ret < 0)
               {
                  printf("%d: sendto error. %d\r\n",sn,ret);
                  return ret;
               }
               sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
            }
         }
         break;
      case SOCK_CLOSED:
         printf("%d:LBUStart\r\n",sn);
         if((ret=socket(sn,Sn_MR_UDP,port,0x00)) != sn)
            return ret;
         printf("%d:Opened, port [%d]\r\n",sn, port);
         break;
      default :
         break;
   }
   return 1;
}
//////////////////////////////////////////////////////////////////////////////

void loadNetParas(void)
{
  //Load physical address
	gWIZNETINFO.mac[0] = 0x0c;
	gWIZNETINFO.mac[1] = 0x29;
	gWIZNETINFO.mac[2] = 0x34;
	gWIZNETINFO.mac[3] = 0x7c;
	gWIZNETINFO.mac[4] = 0x01;
	gWIZNETINFO.mac[5] = 0x64;
	//Load local IP address
	gWIZNETINFO.ip[0] = 192;
	gWIZNETINFO.ip[1] = 168;
	gWIZNETINFO.ip[2] = 1;
	gWIZNETINFO.ip[3] = 247;
	//Load gateway parameters
	gWIZNETINFO.gw[0] = 192;
	gWIZNETINFO.gw[1] = 168;
	gWIZNETINFO.gw[2] = 1;
	gWIZNETINFO.gw[3] = 1;
	//Load subnet mask
	gWIZNETINFO.sn[0] = 255;
	gWIZNETINFO.sn[1] = 255;
	gWIZNETINFO.sn[2] = 255;
	gWIZNETINFO.sn[3] = 0;
	//Load DNS
	gWIZNETINFO.dns[0] = 8;
	gWIZNETINFO.dns[1] = 8;
	gWIZNETINFO.dns[2] = 8;
	gWIZNETINFO.dns[3] = 8;
	//Static IP configuration by manually.
	gWIZNETINFO.dhcp = NETINFO_STATIC;
}
//Interrupt line 3 PA3 responds to data from W5500 and concatenates a flag.
void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line3) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line3);	//Clear interrupt line
		W5500RecInt=1;
	}
}
void wzn_event_handle(void)
{
	uint16_t ir = 0;
	uint8_t sir = 0;
	uint16_t len = 0;
	static const int8_t WZN_ERR = -1;
	static uint8_t wzn_rx_buf[256];
	
	if (ctlwizchip(CW_GET_INTERRUPT, &ir) == WZN_ERR) {
		printf("Cannot get ir...");
	}
	
	if (ir & IK_SOCK_1) {
		sir = getSn_IR(SOCK_UDPS);
		
	
		//printf("IK_SOCK_1");
		
		if ((sir & Sn_IR_SENDOK) > 0) {
			/* Clear Sn_IR_SENDOK flag. */
			setSn_IR(SOCK_UDPS, Sn_IR_SENDOK);
			printf("app_sent();\r\n");
			//app_sent();
		}
		
		if ((sir & Sn_IR_RECV) > 0) {
			//len = getSn_RX_RSR(SOCK_UDPS);
			//recv(SOCK_UDPS, wzn_rx_buf, len);
			printf("UDP packet received!\r\n");
			/* Clear Sn_IR_RECV flag. */
			setSn_IR(SOCK_UDPS, Sn_IR_RECV);
			
			//app_received(wzn_rx_buf, len);
		}
	}
}


/*
int32_t NTPUDP(uint8_t sn, uint8_t* buf)
{
   int32_t  ret;
   uint16_t size, sentsize;
   uint8_t  destip[4];
   uint16_t destport;
	 uint8_t i;
	 uint32_t unixTime_last_sync = 1559640302;
	 uint32_t transmitTime = 0;
	 uint32_t recvTime = htonl(1559640303 + STARTOFTIME);//gio luc nhan dc ban tin
   //uint8_t  packinfo = 0;
	// Ban tin NTP co size = 56 ( ca header : IP[4],port[2],length[2], tru di header chi con 48
   switch(getSn_SR(sn))
   {
      case SOCK_UDP :
         if((size = getSn_RX_RSR(sn)) > 0)
         {
					  
					  if(size > DATA_BUF_SIZE) size = DATA_BUF_SIZE;
            ret = recvfrom(sn,buf,size,destip,(uint16_t*)&destport);
					  printf("\r\nsize:%d, ret:%d, NTP: ",size,ret);
            if(ret <= 0)
            {
               printf("%d: recvfrom error. %d\r\n",sn,ret);
               return ret;
            }
            size = (uint16_t) ret;
            sentsize = 0;
						
						//in ra ban tin
						for(i=0;i<48;i++)
						{
						   printf("%x ",*(buf+i));
						}
						
						//Tao ban tin NTP
						serverPacket[0] = 0x24;   // LI, Version, Mode // Set version number and mode
						serverPacket[1] = 1; // Stratum, or type of clock
						serverPacket[2] = 0;     // Polling Interval
						serverPacket[3] = -12;  // Peer Clock Precision
						serverPacket[12] = 'G';
						serverPacket[13] = 'P';
						serverPacket[14] = 'S';
						//[Reference Timestamp]: unsigned 32-bit seconds value : Lan lay chuan gan nhat la bao gio
						memcpy(&serverPacket[16], &unixTime_last_sync, 4);
						micros_transmit = (((micros() - micros_offset)%1000000) + 1) * USECSHIFT;
						micros_transmit = htonl(micros_transmit);
						micros_recv = htonl(micros_recv);
						transmitTime = htonl( 1559640308 + STARTOFTIME);// gio luc truyen
						memcpy(&serverPacket[40], &transmitTime, 4);
						memcpy(&serverPacket[24], &buf[40], 4);
						memcpy(&serverPacket[28], &buf[44], 4);
						memcpy(&serverPacket[32], &recvTime, 4);
						memcpy(&serverPacket[36], &micros_recv, 4);
						memcpy(&serverPacket[44], &micros_transmit, 4);
						//Gui tra ban tin NTP
						while(sentsize != NTP_PACKET_SIZE)
            {
               ret = sendto(sn,serverPacket,NTP_PACKET_SIZE,destip,destport);
               if(ret < 0)
               {
                  printf("%d: sendto error. %d\r\n",sn,ret);
                  return ret;
               }
               sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
            }
						//printf("\r\nNTPsent \r\n");

         }
         break;
      case SOCK_CLOSED:
         printf("%d:NTP server start\r\n",sn);
         if((ret=socket(sn,Sn_MR_UDP,NTP_PORT,0x00)) != sn)
            return ret;
         printf("%d:Opened, port [%d]\r\n",sn, NTP_PORT);
         break;
      default :
         break;
   }
   return 1;
}
*/

