#include "main.h"
#include "GPIO_STM32F10x.h"
#include "time.h"
#include "socket.h"
#include "w5500lib_init.h"
#include "sntp.h"
#include "ntp.h"
#include "snmp.h"

/***********************************************************************************************************************************/
/////////////////////////////////////////
// SOCKET NUMBER DEFINION for Examples //
/////////////////////////////////////////
#define SOCK_TCPS        1
#define SOCK_UDPS        0
#define SOCK_SNTP        2
#define SOCK_agent			 3
#define SOCK_trap				 4
/////////////////////////////////////////
time_t timerun;
time_t unixTime_last_sync = 1559640302;// lan chuan gio gan nhat 
/***********************************************************************************************************************************/
uint8_t managerIP[4] ={192, 168, 1, 13};
uint8_t agentIP[4]   ={192, 168, 1, 246};

uint32_t getns(void);
void resetns(void);


/* SNTP Packet array */
uint8_t serverPacket[NTP_PACKET_SIZE] = {0};
uint8_t clientPacket[NTP_PACKET_RAWSIZE] = {0};
time_t micros_recv = 0;
time_t micros_offset;
time_t transmitTime;
time_t micros_transmit;
time_t recvTime;


////////////////////////////////////////////////
// Shared Buffer Definition for LOOPBACK TEST //
////////////////////////////////////////////////
#define DATA_BUF_SIZE   100
uint8_t gDATABUF[DATA_BUF_SIZE];

/**
 * @ingroup DATA_TYPE
 *  Network Information for WIZCHIP
 */
 //uint8_t sntp_ip[4] ={192, 168, 1, 14};
 uint8_t sntp_ip[4] ={202, 108, 6, 95};
 uint8_t sntp_buf[56];
 datetime sntp;

///////////////////////////////////
// Default Network Configuration //
///////////////////////////////////
wiz_NetInfo gWIZNETINFO = { .mac = {0x0c, 0x29, 0x34,0x7c, 0xab, 0xcd},
                            .ip = {192, 168, 1, 246},
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
int32_t NTPUDP(uint8_t sn);//UDP NTP server
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



/**
* @brief  GPIO_config cho kit C8T6 china : LED PA1
  * @param  None
  * @retval  
  */
void GPIO_config(void)
{
  //LED PA1 : cho kit C8T6 china : LED PA1
	//GPIO_PortClock   (GPIOA, true);
	//GPIO_PinConfigure(GPIOA, 1, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT10MHZ);
	//GPIO_PinWrite(GPIOA, 1, 0);
}

/**
  * @brief  tasks
  * @param  Tat ca cac chuong trinh con se duoc goi tu day!
  * @retval  
  */
void tasks(void)
{
	 	
	int32_t ret = 0;
	
		
	// NTP UDP server chay dau tien cho nhanh
		
		if( (ret = NTPUDP(SOCK_UDPS)) < 0) {
			printf("SOCKET ERROR : %d\r\n", ret);
		}
		//SNMPv1 example
		//Run SNMP Agent Fucntion
		snmpd_run();	
		
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
		if( (ret = loopback_udps(SOCK_UDPS, clientPacket, 123)) < 0) {
			printf("SOCKET ERROR : %d\r\n", ret);
		}
		*/
		

		// UDP ngat
		
		if(W5500RecInt == 1)
		{
			W5500RecInt = 0;
			wzn_event_handle();
		}
		

		if(TimingDelay>999) 
		{
			TimingDelay = 0;
			timerun++;
			//printf("Get ns :%d\r\n",getns());
			//resetns();
			//count1ms = 0;
			//TIM_SetCounter(TIM3,0);
			//micros_transmit = 123;//0.01s
			
			//printf("micros_transmit :%d-",micros_transmit);
			//micros_transmit = (micros_transmit + 1) * USECSHIFT;
			//printf("htonl(micros_transmit) :%u\r\n",micros_transmit);
			//Time_Display(RTC_GetCounter());
			//printf("\r\nmakeTime :%u\r\n",timerun);
			
			//if( (ret = SNTP_run(&sntp)) == 0) {
			//printf("SNTP ERR : %d\r\n", ret);
		//}
			//printf("getSNMPTimeTick : %u ms\r\n", getSNMPTimeTick());
			
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

	  //RTC_Configuration();
	  
		
		//W5500_GPIO_Init2();
		w5500_lib_init();
    printf("\r\nLoad W5500 config!\r\n");
    
    /* Network initialization */
    network_init();
		//micros_offset = micros()%1000000;
		//timerun = makeTime(tmm);
		timerun = 1560066942;
		printf("\r\nmakeTime :%u\r\n",timerun);
		resetns();
		/****************************************************************************************/
		TIM_init();
		//Phan co dinh cua ban tin NTP
		serverPacket[0] = 0x24;   // LI, Version, Mode // Set version number and mode
		serverPacket[1] = 1; // Stratum, or type of clock
		serverPacket[2] = 0;     // Polling Interval
		serverPacket[3] = -12;  // Peer Clock Precision
		serverPacket[12] = 'G';
		serverPacket[13] = 'P';
		serverPacket[14] = 'S';
		//[Reference Timestamp]: unsigned 32-bit seconds value : Lan lay chuan gan nhat la bao gio
		memcpy(&serverPacket[16], &unixTime_last_sync, 4);
		/****************************************************************************************/				
		//SNTT
		//SNTP_init(SOCK_SNTP,sntp_ip,11,sntp_buf);
		
		snmpd_init(managerIP,agentIP,SOCK_agent,SOCK_trap);				
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
		micros_recv = getns();
		recvTime = (timerun + STARTOFTIME);//gio luc nhan dc ban tin
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
	
	if (ir & IK_SOCK_0) {
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
			//printf("UDP packet received!\r\n");
			//printf("recvTime: %u, micros_recv:%u\r\n",recvTime,micros_recv);
			//transmitTime = (timerun + STARTOFTIME);//gio luc tryen ban tin
			//micros_transmit = getns();
			//printf("transmitTime: %u, micros_transmit:%u\r\n",transmitTime,micros_transmit);
			/* Clear Sn_IR_RECV flag. */
			setSn_IR(SOCK_UDPS, Sn_IR_RECV);
			
			//app_received(wzn_rx_buf, len);
		}
	}
}



int32_t NTPUDP(uint8_t sn)
{
   int32_t  ret;
   uint16_t size, sentsize;
   uint8_t  destip[4];
   uint16_t destport;
	 uint8_t i;
	 
	// Ban tin NTP co size = 56 ( ca header : IP[4],port[2],length[2], tru di header chi con 48
   switch(getSn_SR(sn))
   {
      case SOCK_UDP :
         if((size = getSn_RX_RSR(sn)) > 0)
         {
					  ret = recvfrom(sn,clientPacket,size,destip,(uint16_t*)&destport);
					//if(size > DATA_BUF_SIZE) size = DATA_BUF_SIZE;//56 là max voi ban tin NTP
					 if(size != NTP_PACKET_RAWSIZE) return 0;// ban tin NTP raw size phai la 56
            
					  //printf("\r\nsize:%d, ret:%d, NTP: ",size,ret);
            if(ret <= 0)
            {
               printf("%d: recvfrom error. %d\r\n",sn,ret);
               return ret;
            }
            //size = (uint16_t) ret;
						//size = NTP_PACKET_SIZE;//fix luon
            sentsize = 0;
						
						//in ra ban tin
						/*
						for(i=0;i<48;i++)
						{
						   printf("%x ",*(buf+i));
						}
						*/
						//Tao ban tin NTP
						/*
						serverPacket[0] = 0x24;   // LI, Version, Mode // Set version number and mode
						serverPacket[1] = 1; // Stratum, or type of clock
						serverPacket[2] = 0;     // Polling Interval
						serverPacket[3] = -12;  // Peer Clock Precision
						serverPacket[12] = 'G';
						serverPacket[13] = 'P';
						serverPacket[14] = 'S';
						//[Reference Timestamp]: unsigned 32-bit seconds value : Lan lay chuan gan nhat la bao gio
						memcpy(&serverPacket[16], &unixTime_last_sync, 4);
						*/
						//Transmit Timestamp, T3 from client, copy and return! 
						memcpy(&serverPacket[24], &clientPacket[40], 4);
						memcpy(&serverPacket[28], &clientPacket[44], 4);
						
						//Thoi gian nhan dc ban tin
						//printf("recvTime: %u, micros_recv:%u\r\n",recvTime,micros_recv);
						recvTime = htonl(recvTime);
						memcpy(&serverPacket[32], &recvTime, 4);
						//phan thap phan
						micros_recv = (micros_recv + 1) * USECSHIFT;
						micros_recv = htonl(micros_recv);
						memcpy(&serverPacket[36], &micros_recv, 4);
						
						transmitTime = (timerun + STARTOFTIME);//gio luc tryen ban tin
						micros_transmit = getns();
						//printf("tranTime: %u, micros_tran:%u\r\n",transmitTime,micros_transmit);
						
						transmitTime = htonl(transmitTime);// gio luc truyen
						memcpy(&serverPacket[40], &transmitTime, 4);
						
						
						//Tinh toan phan thap phan cua thoi diem truyen tin
						micros_transmit = getns();
						micros_transmit = (micros_transmit + 1) * USECSHIFT;
						micros_transmit = htonl(micros_transmit);//Ko hieu lam gi nhi, nhung dung!
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


