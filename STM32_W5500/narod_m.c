//===============================================================================//
// This is Open source software. You can place this code on your site, but don't
// forget a link to my YouTube-channel: http://www.youtube.com/c/ElectroHobby1
// Это программное обеспечение распространяется свободно. Вы можете размещать
// его на вашем сайте, но не забудьте указать ссылку на мой YouTube-канал 
// "ElectroHobby" http://www.youtube.com/c/ElectroHobby1
// Автор: ИЛЬКОВЕЦ БОРИС / ILKAVETS BARYS
//=================================================================================//
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include <string.h>
#include <stdio.h> 
#include "misc.h"
#include "socket.h"	
#include "wizchip_conf.h"
#include "Internet/DHCP/dhcp.h"
#include "w5500_ini.h"
#include "timer_ini.h"
#include "string.h"
#define _MAIN_DEBUG_
////////////////////////////////////////////////
#define MY_MAX_DHCP_RETRY	3
#define SOCK_DHCP			0
/////////////////////////////////////////
// SOCKET NUMBER DEFINION for Examples //
/////////////////////////////////////////
#define SOCK_TCPS 1
#define SOCK_TCPS_MY_PORT 9090 //
#define SOCK_TCPS_SERVER_PORT 8283 //9095 narod_m 8283
////////////////////////////////////////////////
// Shared Buffer Definition for LOOPBACK TEST //
////////////////////////////////////////////////
#define DATA_BUF_SIZE   2048
uint8_t gDATABUF[DATA_BUF_SIZE];
uint8_t* send_buf = "Hello";
uint8_t serverIP[4] = {94, 142, 140,101};  //192, 168, 100,8//94, 142, 140,101 narod_mon
uint8_t flag =0;
uint32_t counter = 0;
uint8_t send_tsp_flag = 0;
uint8_t gpio1 =0;
char data_to_send[26];//26
wiz_NetInfo gWIZNETINFO ;
//=============================SWO_ini=====================================//
#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA           0x01000000
struct __FILE { int handle;};
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) {
   if (DEMCR & TRCENA) {

while (ITM_Port32(0) == 0){};
    ITM_Port8(0) = ch;
  }
  return(ch);
}
char *crcOK;
//============================END_ini=====================================//
uint8_t send_tsp(uint16_t ,uint8_t*,uint32_t,uint8_t*);
void my_ip_assign(void);
void my_ip_conflict(void);
void dhcp_rutine (void);
void network_init(void);
void check_answear(uint8_t buf[]);
void make_data_to_send(void);
void TIM4_IRQHandler(void)
{
        if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
        {
					  GPIOC->ODR ^= GPIO_Pin_13;
            TIM_ClearITPendingBit(TIM4, TIM_IT_Update);// сбрасываем флаг
					  DHCP_time_handler();
					 counter++;
					if(counter>(360)&&send_tsp_flag==0){
					 counter = 0;
					 if(flag)	send_tsp(SOCK_TCPS_MY_PORT,serverIP,SOCK_TCPS_SERVER_PORT,(uint8_t*)data_to_send);	
					} 
        }
}
//========================================================================//
void board_led_ini(void){	
  GPIO_InitTypeDef  GPIO_InitStructure;
  /* Initialize LED which connected to PC13, Enable the Clock*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC, GPIO_Pin_13); 
}
//========================================================================//

int main(void)
{
uint32_t i;	
//===================заполняем структурку с параметрами сети==============================//
gWIZNETINFO.mac[0] = 0x00;gWIZNETINFO.mac[1] = 0x08;gWIZNETINFO.mac[2] = 0xdc;gWIZNETINFO.mac[3] = 0x00;gWIZNETINFO.mac[4] = 0xab;gWIZNETINFO.mac[5] = 0xcd;	
gWIZNETINFO.ip[0] = 192;gWIZNETINFO.ip[1] = 168;gWIZNETINFO.ip[2] = 100;gWIZNETINFO.ip[3] = 50;
gWIZNETINFO.sn[0] = 255;gWIZNETINFO.sn[1] = 255;gWIZNETINFO.sn[2] = 255;gWIZNETINFO.sn[3] = 0;
gWIZNETINFO.gw[0] = 192;gWIZNETINFO.gw[1] = 168;gWIZNETINFO.gw[2] = 100;gWIZNETINFO.gw[3] = 1;
gWIZNETINFO.dns[0] = 0;gWIZNETINFO.dns[1] = 0;gWIZNETINFO.dns[2] = 0;gWIZNETINFO.dns[3] = 0;	
gWIZNETINFO.dhcp = NETINFO_DHCP;		
	
//==========================================================================================//	
board_led_ini();
w5500_ini();
timer4_ini();		

//wizchip_setnetinfo(&gWIZNETINFO);	//настраиваем параметры сети для стат.ip
	
//==================================DHCP-====================================================//
 // must be set the default mac before DHCP started.
	setSHAR(gWIZNETINFO.mac);//настройка 
	DHCP_init(SOCK_DHCP, gDATABUF);//передаем номер сокета 
	reg_dhcp_cbfunc(my_ip_assign, my_ip_assign, my_ip_conflict);//передаем функции
	//	которые будут вызваны при разных событияx DHCP(назначение,смена ip,конфликт ip)
//=================================END_DHCP===============================//
  while (1)
  {
		dhcp_rutine();
  }

}
//============================================================================//
uint8_t send_tsp(uint16_t MyPort_N,uint8_t* serverIP,uint32_t serverPortN,uint8_t * buf){
int8_t debag;	
send_tsp_flag=1;
//передаем в функцию номер используемого сокета 0-7,TSP/UDP,порт отправителя	
debag = socket(SOCK_TCPS, Sn_MR_TCP, MyPort_N, 0);	
printf("%d:socet\r\n",debag);	
if (debag == SOCK_TCPS){//создаем сокет
       //передаем в функцию номер исп. сокета,IP адрес получателя и порт 
       debag = connect(SOCK_TCPS,serverIP, serverPortN);
       printf("%d:connect\r\n",debag);
   if (debag == SOCK_OK){//если соединение установлено
		   make_data_to_send();
		   //передаем в функцию номер исп. сокета,данные для передачи и их размер
		   debag = send(SOCK_TCPS,buf,strlen((char*)buf));//sizeof(send_buf)
			 printf("%d:send\r\n",debag); 
      if(debag <1 ){//отправляем данные получателю
	       printf("send_error\r\n");
      }else{
				    unsigned long int i=0;
            printf("send_OK\r\n");
				    while (getSn_RX_RSR(SOCK_TCPS)==0){//||i<9000000
						i++;			
						}
				    if(getSn_RX_RSR(SOCK_TCPS)>0){//если что то пришло
						recv(SOCK_TCPS, gDATABUF, DATA_BUF_SIZE);
						printf("recive %s\r\n",gDATABUF);
						check_answear(gDATABUF);
						}
        }  	   
     }
  }
debag = disconnect(SOCK_TCPS);
printf("%d:disconnect\r\n",debag);
debag = close(SOCK_TCPS);
printf("%d:close\r\n",debag);
send_tsp_flag=0;
return 1;
	}
//=============================================================================//	
void my_ip_assign()//будет вызвана при первом назначении IP от DHCP сервера
{  
   getIPfromDHCP(gWIZNETINFO.ip);//получаем ip от DHCP
	 getGWfromDHCP(gWIZNETINFO.gw);//адрес шлюза
   getSNfromDHCP(gWIZNETINFO.sn);//маска под сети 
   getDNSfromDHCP(gWIZNETINFO.dns);//адрес DNS 
   gWIZNETINFO.dhcp = NETINFO_DHCP;
   /* Network initialization */
    network_init();      // apply from dhcp
	  flag =1;
#ifdef _MAIN_DEBUG_
   printf("DHCP LEASED TIME : %ld Sec.\r\n", getDHCPLeasetime());//получить время аренды на сервере DHCP
#endif
}
//============================================================================//
void my_ip_conflict()
{
#ifdef _MAIN_DEBUG_
	printf("CONFLICT IP from DHCP\r\n");
#endif
   //halt or reset or any...
   while(1); // this example is halt.
}	
//============================================================================//
void network_init()
{
	uint8_t tmpstr[6] = {0,};
	wiz_NetInfo netinfo;
	// Set Network information from netinfo structure
	ctlnetwork(CN_SET_NETINFO, (void*)&gWIZNETINFO);

#ifdef _MAIN_DEBUG_
	// Get Network information
	ctlnetwork(CN_GET_NETINFO, (void*)&netinfo);

	// Display Network Information
	ctlwizchip(CW_GET_ID,(void*)tmpstr);

	if(netinfo.dhcp == NETINFO_DHCP) printf("\r\n=== %s NET CONF : DHCP ===\r\n",(char*)tmpstr);
	else printf("\r\n=== %s NET CONF : Static ===\r\n",(char*)tmpstr);

	printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",netinfo.mac[0],netinfo.mac[1],netinfo.mac[2],
	netinfo.mac[3],netinfo.mac[4],netinfo.mac[5]);
	printf("SIP: %d.%d.%d.%d\r\n", netinfo.ip[0],netinfo.ip[1],netinfo.ip[2],netinfo.ip[3]);
	printf("GAR: %d.%d.%d.%d\r\n", netinfo.gw[0],netinfo.gw[1],netinfo.gw[2],netinfo.gw[3]);
	printf("SUB: %d.%d.%d.%d\r\n", netinfo.sn[0],netinfo.sn[1],netinfo.sn[2],netinfo.sn[3]);
	printf("DNS: %d.%d.%d.%d\r\n", netinfo.dns[0],netinfo.dns[1],netinfo.dns[2],netinfo.dns[3]);
	printf("===========================\r\n");
#endif
}
//============================================================================//
//============================================================================//

void dhcp_rutine(){
uint8_t my_dhcp_retry = 0;		
switch(DHCP_run())
		{
	    case DHCP_IP_ASSIGN://можем сделать что-то сдесь когда мы получили ip
			case DHCP_IP_CHANGED://можем сделать что-то сдесь при изменениее ip
			break;
			case DHCP_IP_LEASED:
			break;
			case DHCP_FAILED://ошибка получения ip адреса от DHCP
				my_dhcp_retry++;//кол-во попыток
				if(my_dhcp_retry > MY_MAX_DHCP_RETRY)
				{
					#ifdef _MAIN_DEBUG_
					printf(">> DHCP %d Failed\r\n", my_dhcp_retry);
					#endif
					my_dhcp_retry = 0;
					DHCP_stop();      // if restart, recall DHCP_init()
					network_init();   // назначаем статический ip
				}
				break;
			default:
				break;
		}
	}
void make_data_to_send(void){
sprintf(data_to_send,"%s%d%s%.2f%s","#4515B775036F\n#gpio1#",gpio1,"\n#T1#",22.2,"\n##\r\n");
}
void check_answear(uint8_t buf[]){
printf("check_answear\r\n");
if(strstr((char*) buf,"#gpio1=0")){//strcmp ==0
gpio1=0;
printf("gpio1=0\r\n");
make_data_to_send();			
}
else if(strstr((char*) buf,"#gpio1=1")){
gpio1=1;
printf("gpio1=1\r\n");
make_data_to_send();	
}	
}
