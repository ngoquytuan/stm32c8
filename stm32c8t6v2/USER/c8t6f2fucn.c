
#include "main.h"
#include "GPIO_STM32F10x.h"

#include "loopback.h"
#include "socket.h"
#include "wizchip_conf.h"

//===========================================================================//
void chip_on(void){
  GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS); 
}
//===========================================================================//
void chip_off(void){
  GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); 
}
//===========================================================================//
uint8_t SPI_SendReceiveByte(void)
{
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI1, 0xFF);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET) ;
  return SPI_I2S_ReceiveData(SPI1);
}
//===========================================================================//
void senddata(uint8_t data){	
while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
SPI_I2S_SendData(SPI1, data);
while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET) ;
SPI_I2S_ReceiveData(SPI1);	
	}
//===========================================================================//
void w5500_ini(void){
  uint8_t temp;
  uint8_t W5500FifoSize[2][8] = {{2, 2, 2, 2, 2, 2, 2, 2, }, {2, 2, 2, 2, 2, 2, 2, 2}};

  chip_off();	
 /*pass the read function to the driver*/
  reg_wizchip_spi_cbfunc(SPI_SendReceiveByte, senddata);

  /* CS function register */
   reg_wizchip_cs_cbfunc(chip_on,chip_off);

  if (ctlwizchip(CW_INIT_WIZCHIP, (void*)W5500FifoSize) == -1){
    printf("W5500 initialized fail.\r\n");
      while(1);
  }
//    check phy status
  do
  {
		//GPIOC->ODR ^= GPIO_Pin_13;
		
    if (ctlwizchip(CW_GET_PHYLINK, (void*)&temp) == -1)
    {
       // GPIOC->ODR ^= GPIO_Pin_13;    
			printf("Unknown PHY link status.\r\n");
    }
  } while (temp == PHY_LINK_OFF);
	printf("W5500 ok?\r\n");
}
/*
int32_t loopback_udps(uint8_t sn, uint8_t* buf, uint16_t port)
{
   int32_t  ret;
   uint16_t size, sentsize;
   uint8_t  destip[4];
   uint16_t destport;

   switch(getSn_SR(sn))
   {
      case SOCK_UDP :
         if((size = getSn_RX_RSR(sn)) > 0)
         {
            if(size > DATA_BUF_SIZE) size = DATA_BUF_SIZE;
            ret = recvfrom(sn, buf, size, destip, (uint16_t*)&destport);
            if(ret <= 0)
            {
#ifdef _LOOPBACK_DEBUG_
               printf("%d: recvfrom error. %d\r\n",sn,ret);
#endif
               return ret;
            }
            size = (uint16_t) ret;
            sentsize = 0;
            while(sentsize != size)
            {
               ret = sendto(sn, buf+sentsize, size-sentsize, destip, destport);
               if(ret < 0)
               {
#ifdef _LOOPBACK_DEBUG_
                  printf("%d: sendto error. %d\r\n",sn,ret);
#endif
                  return ret;
               }
               sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
            }
         }
         break;
      case SOCK_CLOSED:
#ifdef _LOOPBACK_DEBUG_
         //printf("%d:UDP loopback start\r\n",sn);
#endif
         if((ret = socket(sn, Sn_MR_UDP, port, 0x00)) != sn)
            return ret;
#ifdef _LOOPBACK_DEBUG_
         printf("%d:Opened, UDP loopback, port [%d]\r\n", sn, port);
#endif
         break;
      default :
         break;
   }
   return 1;
}
*/
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
	size=Read_SOCK_Data_Buffer(s, Rx_Buffer);
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
			printf("Ngat\r\n");
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
				S0_Data&=~S_TRANSMITOK;
				memcpy(Tx_Buffer, "\r\nW5500 UDP IP:192.168.1.246:5000 \r\n", 45);	
				Write_SOCK_Data_Buffer(0, Tx_Buffer, 45);//Specify Socket (0~7) to send data processing, port 0 to send 30 bytes of data
			}
			W5500_Send_Delay=0;
		}	

		W5500_Send_Delay++;
}

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

  
    //w5500_run();
		
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
/* Initialize the SPI w5500 driver ----------------------------------------*/
	SPI1_Init();	 //Initialize SPI1  PA5 PA6 PA7 IO is SPI mode
	SPI1_SetSpeed(SPI_BaudRatePrescaler_2);	 //Configure SPI1 speed to be the highest
  /*
	W5500_GPIO_Init();//Initialize W5500  RST INT SCS corresponds to GPIO status And configure INT interrupt mode
	Load_Net_Parameters();		//Load network parameters	
	printf("\r\nwaiting for Ethernet connection to complete...nho cam day mang\r\n");
	W5500_Hardware_Reset();		//Hardware reset W5500
	W5500_Initialization();		//W5500 initial configuration
	printf("\r\nLoad W5500 config!\r\n");
	*/
	w5500_ini();
}
			


