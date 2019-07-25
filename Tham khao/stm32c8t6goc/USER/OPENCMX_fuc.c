
#include "main.h"
#include "GPIO_STM32F10x.h"
//#include "eeprom_stm.h"
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

/**
  * @brief  tasks
  * @param  None
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
		#ifdef USE_UART1
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
	USART1_SendStr_("Uart1 chay nay!!!\r\n");
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


}
			

