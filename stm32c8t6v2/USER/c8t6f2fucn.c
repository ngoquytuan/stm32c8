#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "GPIO_STM32F10x.h"
#include "time.h"
#include "socket.h"
#include "w5500lib_init.h"
#include "sntp.h"
#include "ntp.h"
#include "snmp.h"
#include "httpServer.h"
#include "webpage.h"

#define _USE_SDCARD_
#ifdef _USE_SDCARD_
#include "ff.h"
#include "mmcHandler.h"
#include "ffconf.h"
#endif
int g_mkfs_done = 0;
int g_sdcard_done = 0;
//////////////////////////////////////////////////////////////////////
// Default Network Configuration /////////////////////////////////////
//////////////////////////////////////////////////////////////////////

wiz_NetInfo gWIZNETINFO = { .mac = {0x0c, 0x29, 0x34,0x7c, 0xab, 0xcd},
                            .ip = {192, 168, 1, 245},
                            .sn = {254,254,254,1},
                            .gw = {193, 168, 0, 4},
                            .dns = {8,8,8,8},
                            .dhcp = NETINFO_STATIC };
//////////////////////////////////////////////////////////////////////
//for snmp
uint8_t managerIP[4] ={192, 168, 1, 6};
uint8_t agentIP[4]   ={192, 168, 1, 246};
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// SOCKET NUMBER DEFINION  ///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
#define SOCK_TCPS        1
#define SOCK_UDPS        0
#define SOCK_SNTP        2
#define SOCK_agent			 3
#define SOCK_trap				 4
#define SOCK_WEBSERVER   5
#define PORT_WEBSERVER  80
#define MAX_HTTPSOCK		 3

//////////////////////////////////////////////////////////////////////
// Shared Buffer Definition WEB server////////////////////////////////
//////////////////////////////////////////////////////////////////////
#define DATA_BUF_SIZEHTTP   2048
uint8_t RX_BUF[DATA_BUF_SIZEHTTP];
uint8_t TX_BUF[DATA_BUF_SIZEHTTP];
uint8_t socknumlist[] = {5, 6, 7};



///////////////////////////////////////////////////////////////////////
// Shared Buffer Definition for LOOPBACK TEST /////////////////////////
///////////////////////////////////////////////////////////////////////
#define DATA_BUF_SIZE   1000
uint8_t gDATABUF[DATA_BUF_SIZE];
///////////////////////////////////////////////////////////////////////


/////////////////////
// PHYStatus check //
/////////////////////
#define SEC_PHYSTATUS_CHECK 		1000		// msec
bool PHYStatus_check_enable = false;


/***********************************************************************************************************************************/



//for NTP server
time_t timerun;
time_t unixTime_last_sync = 1559640302;// lan chuan gio gan nhat 
/* SNTP Packet array */
uint8_t serverPacket[NTP_PACKET_SIZE] = {0};
uint8_t clientPacket[NTP_PACKET_RAWSIZE] = {0};
time_t micros_recv = 0;
time_t micros_offset;
time_t transmitTime;
time_t micros_transmit;
time_t recvTime;

//for NTP client
 //uint8_t sntp_ip[4] ={192, 168, 1, 14};
 uint8_t sntp_ip[4] ={202, 108, 6, 95};// NTP time server
 uint8_t sntp_buf[56];
 datetime sntp;

//////////////////////////////////////////////////////////////////////////////////////////////
														


/*******************************************************************************/


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
		{	//SNMPv1 run
			//Run SNMP Agent Fucntion
			/* SNMP Agent Handler */
			//SMI Network Management Private Enterprise Codes: : moi cong ty phai dang ky 1 so rieng, 
			//tham khao : https://www.iana.org/assignments/enterprise-numbers/enterprise-numbers
			// Vi du Arduino : 36582
    	// SNMP Agent daemon process : User can add the OID and OID mapped functions to snmpData[] array in snmprun.c/.h
			// [net-snmp version 5.7 package for windows] is used for this demo.
			// * Command example
    	// [Command] Get:			  snmpget -v 1 -c public 192.168.1.246 .1.3.6.1.2.1.1.1.0 			// (sysDescr)
    	// [Command] Get: 			snmpget -v 1 -c public 192.168.1.246 .1.3.6.1.4.1.6.1.0 			// (Custom, get LED status)
    	// [Command] Get-Next: 	snmpwalk -v 1 -c public 192.168.1.246 .1.3.6.1
			// [Command] Set: 			snmpset -v 1 -c public 192.168.1.246 .1.3.6.1.4.1.6.2.0 i 1			// (Custom, LED 'On')
    	// [Command] Set: 			snmpset -v 1 -c public 192.168.1.246 .1.3.6.1.4.1.6.2.0 i 0			// (Custom, LED 'Off')
			snmpd_run();	
		}
		
    {// TCP server loopback test
    	if( (ret = loopback_tcps(SOCK_TCPS, gDATABUF, 5000)) < 0) {
			printf("SOCKET ERROR : %d\r\n", ret);
		}
		}
		
		{	// web server 	
			httpServer_run(0);
			httpServer_run(1);
			httpServer_run(2);
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

  
		

			
		
		

		// UDP ngat
		if(W5500RecInt == 1)
		{
			W5500RecInt = 0;
			wzn_event_handle();
		}
		
		//1s
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
		{// PHY status check counter
			if(PHYStatus_check_enable)
			{
				if (phystatus_check_cnt > (SEC_PHYSTATUS_CHECK))
				{
					PHYStatus_Check();
					phystatus_check_cnt = 0;
					//printf("PHYStatus_Check\r\n");
				}
			}
			else
			{
				phystatus_check_cnt = 0;
			}
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
	
	USART1_Init();
	
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

	
	
  
	
	
	//using stm32's flash to store data
	//EEPROM STM32 init
	sw_eeprom_stm32();
	test_eeprom();
	delay_ms(1);
	 
	//SPI
	SPI1_W5500_Init();	
    
    /* Network initialization */
    w5500_lib_init();
		Net_Conf(gWIZNETINFO);
		/****************************************************************************************/
		timerun = 1562919653;
		printf("\r\nmakeTime :%u\r\n",timerun);
		resetns();
		ntpserverdefaultconfig();
		/****************************************************************************************/
		
						
		//SNTT
		//SNTP_init(SOCK_SNTP,sntp_ip,11,sntp_buf);
		/* SNMP(Simple Network Management Protocol) Agent Initialize */
		// NMS (SNMP manager) IP address
		snmpd_init(managerIP,agentIP,SOCK_agent,SOCK_trap);	
		PHYStatus_check_enable = true;		
		
		loadwebpages();
		/* WWDG configuration */
		// IWDG Initialization: STM32 Independent WatchDog
		IWDG_configuration();
}

void loadwebpages()
{
		//Lien quan den webserver
		//reg_httpServer_cbfunc(NVIC_SystemReset, NULL); 
		reg_httpServer_cbfunc(NVIC_SystemReset, IWDG_ReloadCounter); // Callback: STM32 MCU Reset / WDT Reset (IWDG)
		/* HTTP Server Initialization  */
		httpServer_init(TX_BUF, RX_BUF, MAX_HTTPSOCK, socknumlist);
		reg_httpServer_webContent((uint8_t *)"index.html", (uint8_t *)index_page);				// index.html 		: Main page example
		reg_httpServer_webContent((uint8_t *)"netinfo.html", (uint8_t *)netinfo_page);			// netinfo.html 	: Network information example page
		reg_httpServer_webContent((uint8_t *)"netinfo.js", (uint8_t *)wiz550web_netinfo_js);	// netinfo.js 		: JavaScript for Read Network configuration 	(+ ajax.js)
		reg_httpServer_webContent((uint8_t *)"img.html", (uint8_t *)img_page);					// img.html 		: Base64 Image data example page

		// Example #1
		reg_httpServer_webContent((uint8_t *)"dio.html", (uint8_t *)dio_page);					// dio.html 		: Digital I/O control example page
		reg_httpServer_webContent((uint8_t *)"dio.js", (uint8_t *)wiz550web_dio_js);			// dio.js 			: JavaScript for digital I/O control 	(+ ajax.js)

		// Example #2
		reg_httpServer_webContent((uint8_t *)"ain.html", (uint8_t *)ain_page);					// ain.html 		: Analog input monitor example page
		reg_httpServer_webContent((uint8_t *)"ain.js", (uint8_t *)wiz550web_ain_js);			// ain.js 			: JavaScript for Analog input monitor	(+ ajax.js)

		// Example #3
		reg_httpServer_webContent((uint8_t *)"ain_gauge.html", (uint8_t *)ain_gauge_page);		// ain_gauge.html 	: Analog input monitor example page; using Google Gauge chart
		reg_httpServer_webContent((uint8_t *)"ain_gauge.js", (uint8_t *)ain_gauge_js);			// ain_gauge.js 	: JavaScript for Google Gauge chart		(+ ajax.js)

		// AJAX JavaScript functions
		reg_httpServer_webContent((uint8_t *)"ajax.js", (uint8_t *)wiz550web_ajax_js);			// ajax.js			: JavaScript for AJAX request transfer
		
		//favicon.ico
		reg_httpServer_webContent((uint8_t *)"favicon.ico", (uint8_t *)pageico);			// favicon.ico
		//config page
		reg_httpServer_webContent((uint8_t *)"config.html", (uint8_t *)configpage);			// config.html
		display_reg_webContent_list();
}




/////////////////////////////////////////////////////////////
// Intialize the network information to be used in WIZCHIP //
/////////////////////////////////////////////////////////////

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
						//ntpserverdefaultconfig();
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

void wzn_event_handle(void)
{
	uint16_t ir = 0;
	uint8_t sir = 0;
	uint16_t len = 0;
	static const int8_t WZN_ERR = -1;
	
	
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
			//printf("UDP packet received!\r\n");
			//printf("recvTime: %u, micros_recv:%u\r\n",recvTime,micros_recv);
			//transmitTime = (timerun + STARTOFTIME);//gio luc tryen ban tin
			//micros_transmit = getns();
			//printf("transmitTime: %u, micros_transmit:%u\r\n",transmitTime,micros_transmit);
			/* Clear Sn_IR_RECV flag. */
			setSn_IR(SOCK_UDPS, Sn_IR_RECV);
			
		}
	}
}
/********************************************************************************************************************/
void ntpserverdefaultconfig(void)
{
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
  * @brief  sw_eeprom_stm32
  * @param  make some flash blocks come eeprom for store data
  * @retval  just call this fuction
  */	
void sw_eeprom_stm32(void)
{
		/* Unlock the Flash Program Erase controller */
  FLASH_Unlock();
  /* EEPROM Init */
  if(EE_Init() == FLASH_COMPLETE) printf("Emu EEPROM STM32 ready !\r\n");
}

/**
  * @brief  test_eeprom
  * @param  Call this fuction for test store data to eeprom
  * @retval  
  */	
void test_eeprom(void)
{
	uint16_t a,b,c,d,e,f; 
	uint16_t confirm[3];
	
	//Kiem tra confirm xem dung ko, neu sai thi du lieu bi sai => reset factory setting
	// Neu dung thi load config
	
	EE_ReadVariable(0,&confirm[0]);
	EE_ReadVariable(1,&confirm[1]);
	EE_ReadVariable(2,&confirm[2]);	
	
	if( (confirm[0] == 123) && (confirm[1] == 456) && (confirm[2] == 789))
	{
		printf("Right eeprom data, load configs now\r\n");
		//Load IP
		EE_ReadVariable(4,&a);
		EE_ReadVariable(5,&b);
		EE_ReadVariable(6,&c);
		EE_ReadVariable(7,&d);
		gWIZNETINFO.ip[0] = a;
		gWIZNETINFO.ip[1] = b;
		gWIZNETINFO.ip[2] = c;
		gWIZNETINFO.ip[3] = d;
		//printf("Load ip: %d.%d.%d.%d",gWIZNETINFO.ip[0],gWIZNETINFO.ip[1],gWIZNETINFO.ip[2],gWIZNETINFO.ip[3]);
		//Load GW
		EE_ReadVariable(8,&a);
		EE_ReadVariable(9,&b);
		EE_ReadVariable(10,&c);
		EE_ReadVariable(11,&d);
		gWIZNETINFO.gw[0] = a;
		gWIZNETINFO.gw[1] = b;
		gWIZNETINFO.gw[2] = c;
		gWIZNETINFO.gw[3] = d;
		//Load SN
		EE_ReadVariable(12,&a);
		EE_ReadVariable(13,&b);
		EE_ReadVariable(14,&c);
		EE_ReadVariable(15,&d);
		gWIZNETINFO.sn[0] = a;
		gWIZNETINFO.sn[1] = b;
		gWIZNETINFO.sn[2] = c;
		gWIZNETINFO.sn[3] = d;
	}
	else
	{
		printf("Wrong eeprom data\r\n");
		EE_WriteVariable(0,123);
		EE_WriteVariable(1,456);
		EE_WriteVariable(2,789);
		//IP 192.168.1.246
		EE_WriteVariable(4,192);
		EE_WriteVariable(5,168);
		EE_WriteVariable(6,1);
		EE_WriteVariable(7,246);
		//GW: 192.168.1.1
		EE_WriteVariable(8,192);
		EE_WriteVariable(9,168);
		EE_WriteVariable(10,1);
		EE_WriteVariable(11,1);
		//SN 255.255.255.0
		EE_WriteVariable(12,255);
		EE_WriteVariable(13,255);
		EE_WriteVariable(14,255);
		EE_WriteVariable(15,0);
	}
		

}
//Kiem tra xem day mang co cam hay k?
void PHYStatus_Check(void)
{
	uint8_t tmp;
	//static bool LED_status_backup;

	////LED_status_backup = RGBLED_enable;
		ctlwizchip(CW_GET_PHYLINK, (void*) &tmp);

		// Error indicator: LED Green ON when no PHY link detected
		if(tmp == PHY_LINK_OFF)
		{
			/* Turn on LED1 */
			GPIO_PinWrite(GPIOA, 1, 0);
		}
		else{
		/* Turn off LED1 */
    GPIO_PinWrite(GPIOA, 1, 1);
		}

	//RGBLED_enable = LED_status_backup;
}

// IWDG: STM32 Independent Watchdog Initialization
void IWDG_configuration(void)
{
	//RCC_LSICmd(ENABLE); //open LSI
	//while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
	if(RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
	{
		RCC_ClearFlag();
	}

	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_128); // 40Khz / 128 = 0.31KHz; 1 / 0.31KHz = 3.22ms
	//IWDG_SetReload(1250); // 1s, max 0xfff
	IWDG_SetReload(0xfff); // 4095 * 3.22ms = 13185.9ms = 13 seconds; it means if IWDG was not reloaded, MCU will reset!

	//IWDG_ReloadCounter();
	IWDG_Enable();
}

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
	
	//LED PA4 : cho kit C8T6 china : CS SD
	GPIO_PortClock   (GPIOA, true);
	GPIO_PinConfigure(GPIOA, 4, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT10MHZ);
	GPIO_PinWrite(GPIOA, 4, 1);
}


/********************************************************************************************************************/

/********************************************************************************************************************/

/********************************************************************************************************************/
/*
																						END OF FILE
*/
