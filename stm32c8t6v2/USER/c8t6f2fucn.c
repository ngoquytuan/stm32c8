
#include "main.h"
#include "GPIO_STM32F10x.h"

#include "loopback.h"
#include "socket.h"
#include "w5500lib_init.h"
#include "w5500cn.h"
#include "loopback.h"
//#include "spi.h"
/////////////////////////////////////////
// SOCKET NUMBER DEFINION for Examples //
/////////////////////////////////////////
#define SOCK_TCPS        1
#define SOCK_UDPS        0
////////////////////////////////////////////////
// Shared Buffer Definition for LOOPBACK TEST //
////////////////////////////////////////////////

uint8_t gDATABUF[DATA_BUF_SIZE];
int32_t ret = 0;
wiz_NetInfo MyNetInfo ;

void loadNetParas(void)
{
  //Load physical address
	MyNetInfo.mac[0] = 0x0c;
	MyNetInfo.mac[1] = 0x29;
	MyNetInfo.mac[2] = 0x34;
	MyNetInfo.mac[3] = 0x7c;
	MyNetInfo.mac[4] = 0x01;
	MyNetInfo.mac[5] = 0x64;
	//Load local IP address
	MyNetInfo.ip[0] = 192;
	MyNetInfo.ip[1] = 168;
	MyNetInfo.ip[2] = 1;
	MyNetInfo.ip[3] = 3;
	//Load gateway parameters
	MyNetInfo.gw[0] = 192;
	MyNetInfo.gw[1] = 168;
	MyNetInfo.gw[2] = 1;
	MyNetInfo.gw[3] = 1;
	//Load subnet mask
	MyNetInfo.sn[0] = 255;
	MyNetInfo.sn[1] = 255;
	MyNetInfo.sn[2] = 255;
	MyNetInfo.sn[3] = 0;
	//Load DNS
	MyNetInfo.dns[0] = 8;
	MyNetInfo.dns[1] = 8;
	MyNetInfo.dns[2] = 8;
	MyNetInfo.dns[3] = 8;
	//Static IP configuration by manually.
	MyNetInfo.dhcp = NETINFO_STATIC;
}
uint8_t tmpstr[6] = {0,};
wiz_NetInfo netinfo;
//#define	USE_TQ_LIB
#ifdef USE_TQ_LIB

u32 W5500_Send_Delay=0; //W5500 Send delay count variable(ms)

//Load network parameters
//Description: Gateway, subnet mask, physical address, local IP address, local port number, destination IP address, destination port number, port working mode
void Load_Net_Parameters(void)
{
	Gateway_IP[0] = 192;//Load gateway parameters
	Gateway_IP[1] = 168;
	Gateway_IP[2] = 1;
	Gateway_IP[3] = 1;

	Sub_Mask[0]=255;//Load subnet mask
	Sub_Mask[1]=255;
	Sub_Mask[2]=255;
	Sub_Mask[3]=0;

	Phy_Addr[0]=0x0c;//Load physical address
	Phy_Addr[1]=0x29;
	Phy_Addr[2]=0xab;
	Phy_Addr[3]=0x7c;
	Phy_Addr[4]=0x00;
	Phy_Addr[5]=0x02;

	IP_Addr[0]=192;//Load local IP address
	IP_Addr[1]=168;
	IP_Addr[2]=1;
	IP_Addr[3]=246;

	//S0_Port[0] = 0x13;//Load the port number of port 0 5000 
	//S0_Port[1] = 0x88;
	
	S0_Port[0] = 0x00;//Load the port number of port 0 123 
	S0_Port[1] = 0x7B;

  S0_Mode = UDP_MODE;//Load port 0 working mode, UDP mode
}




//W5500 Initial configuration
void W5500_Initialization(void)
{
	//printf("W5500 read reg :%d %d %d %d\r\n",Read_W5500_1Byte(0),Read_W5500_1Byte(1),Read_W5500_1Byte(2),Read_W5500_1Byte(3));
	//printf("W5500 read reg :%d\r\n",Read_W5500_1Byte(VERR));
	W5500_Init();		//Initialize the W5500 register
	if(Detect_Gateway()) printf("TRUE");	//Check the gateway server 
	Socket_Init(0);		//Specify Socket (0~7) initialization, initialize port 0
	
}




//Description : W5500 port initialization configuration
//NOTE: Set 4 ports respectively, and put the port in the TCP server, TCP client or UDP mode according to the port working mode.
//		 The port status byte Socket_State can be used to determine the working status of the port.
void W5500_Socket_Set(void)
{
	if(S0_State==0)//Port 0 initial configuration
	{
		if(S0_Mode==TCP_SERVER)//TCP Server mode 
		{
			if(Socket_Listen(0)==TRUE)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else if(S0_Mode==TCP_CLIENT)//TCP Client mode
		{
			if(Socket_Connect(0)==TRUE)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else//UDP mode 
		{
			if(Socket_UDP(0)==TRUE)
				S0_State=S_INIT|S_CONN;
			else
				S0_State=0;
		}
	}
}




////W5500 Receive and send received data
////s:The port number
////Description: This procedure first calls S_rx_process() to receive data from the W5500 port receiving data buffer.
////		 The read data is then copied from the Rx_Buffer to the Temp_Buffer buffer for processing.
////		 After processing, copy the data from Temp_Buffer to the Tx_Buffer buffer. Call S_tx_process()
////		 send data.
//void Process_Socket_Data(SOCKET s)
//{
//	u16 size;
//	size=Read_SOCK_Data_Buffer(s, Rx_Buffer);
//	memcpy(Tx_Buffer, Rx_Buffer, size);			
//	Write_SOCK_Data_Buffer(s, Tx_Buffer, size);
//}




//W5500 Receive and send received data
//UDP In the mode, the first 4 bytes of the received data are the IP address of the other port, and the 5th and 6th bytes are the port number. 
//Here, you need to collect the IP and port number of the other port, and then you can perform data return and other functions.
void Process_Socket_Data(SOCKET s)
{
	u16 size;
	if((size = getSn_RX_RSR(s)) > 0) printf("size1 : %d--",size);
	
	size = Read_SOCK_Data_Buffer(s, Rx_Buffer);
	printf("size2 : %d; ",size);
	UDP_DIPR[0] = Rx_Buffer[0];
	UDP_DIPR[1] = Rx_Buffer[1];
	UDP_DIPR[2] = Rx_Buffer[2];
	UDP_DIPR[3] = Rx_Buffer[3];

	UDP_DPORT[0] = Rx_Buffer[4];
	UDP_DPORT[1] = Rx_Buffer[5];
	memcpy(Tx_Buffer, Rx_Buffer+8, size-8);	
	Write_SOCK_Data_Buffer(s, Tx_Buffer, size);
}

/**
  * @brief  w5500_run
  * @param  None
  * @retval  
  */
void w5500_run(void)
{
	  W5500_Socket_Set();//W5500 port initialization configuration

		if(W5500_Interrupt)//Handling W5500 interrupts		
		{
			W5500_Interrupt_Process();//W5500 interrupt handler framework
		}
		if((S0_Data & S_RECEIVE) == S_RECEIVE)//If Socket0 receives data
		{
			S0_Data&=~S_RECEIVE;
			Process_Socket_Data(0);//The W5500 receives and transmits the received data.
		}		
		else if(W5500_Send_Delay>=720000)//Send string periodically
		{
			if(S0_State == (S_INIT|S_CONN))
			{
				//S0_Data&=~S_TRANSMITOK;
				//memcpy(Tx_Buffer, "\r\nW5500 UDP IP:192.168.1.246:5000 \r\n", 35);	
				//Write_SOCK_Data_Buffer(0, Tx_Buffer, 35);//Specify Socket (0~7) to send data processing, port 0 to send 30 bytes of data
			}
			W5500_Send_Delay=0;
		}	

		W5500_Send_Delay++;
}
#endif
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
					Time_Display(RTC_GetCounter());
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

  
    #ifdef USE_TQ_LIB
		w5500_run();
		#endif
		// TCP server loopback test
    	if( (ret = loopback_tcps(SOCK_TCPS, gDATABUF, 5000)) < 0) {
			printf("SOCKET ERROR : %d\r\n", ret);
		}
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
#ifdef USE_TQ_LIB
/* Initialize the SPI w5500 driver ----------------------------------------*/
  //SPI1_W5500_Init();
	//SPI1_Init();	 //Initialize SPI1  PA5 PA6 PA7 IO is SPI mode
	//SPI1_SetSpeed(SPI_BaudRatePrescaler_2);	 //Configure SPI1 speed to be the highest  
	//W5500_GPIO_Init();//Initialize W5500  RST INT SCS corresponds to GPIO status And configure INT interrupt mode
	W5500_GPIO_Init2();
	//W5500_GPIO_Init2();
	
	Load_Net_Parameters();		//Load network parameters	
	printf("\r\nwaiting for Ethernet connection to complete...nho cam day mang\r\n");
	W5500_Hardware_Reset();		//Hardware reset W5500
	W5500_Initialization();		//W5500 initial configuration
	printf("\r\nLoad W5500 config!\r\n");
	#endif
	
	
	W5500_GPIO_Init2();
	w5500_lib_init();
	//W5500_Init();
	
	//printf("\r\nW5500\r\n");
	loadNetParas();
	
	
	
	//printf("\r\nw5500_lib_init\r\n");
	// Set Network information from netinfo structure
	ctlnetwork(CN_SET_NETINFO, (void*)&MyNetInfo);
	printf("\r\nSet Network information\r\n");
	
		// Get Network information
	ctlnetwork(CN_GET_NETINFO, (void*)&netinfo);

	// Display Network Information
	ctlwizchip(CW_GET_ID,(void*)tmpstr);

	printf("Ko co dong nay k chay???W5500 MR :%d\r\n",Read_W5500_1Byte(0));

	//printf("W5500 getway :%d %d %d %d\r\n",Read_W5500_1Byte(0xF),Read_W5500_1Byte(0x10),Read_W5500_1Byte(0x11),Read_W5500_1Byte(0x12));

	if(netinfo.dhcp == NETINFO_DHCP) printf("\r\n=== %s NET CONF : DHCP ===\r\n",(char*)tmpstr);
	else printf("\r\n=== %s NET CONF : Static ===\r\n",(char*)tmpstr);

	printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",netinfo.mac[0],netinfo.mac[1],netinfo.mac[2],netinfo.mac[3],netinfo.mac[4],netinfo.mac[5]);
	printf("SIP: %d.%d.%d.%d\r\n", netinfo.ip[0],netinfo.ip[1],netinfo.ip[2],netinfo.ip[3]);
	printf("GAR: %d.%d.%d.%d\r\n", netinfo.gw[0],netinfo.gw[1],netinfo.gw[2],netinfo.gw[3]);
	printf("SUB: %d.%d.%d.%d\r\n", netinfo.sn[0],netinfo.sn[1],netinfo.sn[2],netinfo.sn[3]);
	printf("DNS: %d.%d.%d.%d\r\n", netinfo.dns[0],netinfo.dns[1],netinfo.dns[2],netinfo.dns[3]);
	printf("===============END=============\r\n");
}


			


