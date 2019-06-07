

/*
 http://zq281598585.taobao.com/  
启光电子	  ELH    enlighten  sunny

w5500 Network module reference program

Running test MCU STM32F103RBT6


UDP mode

Interrupt judgment method
The test uses the W5500 port socket0 (other ports can be set by themselves)
In UDP mode, port communication can receive data from other UDP mode ports without establishing a connection.
The first 4 bytes of data are the IP address of the other port, and the 5th and 6th bytes are the port number.
W5500 port 0 will send return data after obtaining the IP and port number of the other party.

So this mode just needs to determine the module's own IP and port.


W5500 All pins correspond to the following： 
sck PA5  miso PA6  mosi PA7  rst PA2  int PA3  cs PA0



Test routing data values

Gateway: 192.168.0.1		   （It tests the route for us, and the route used by the average family is 192.168.1.1）
子网掩码：255.255.255.0	   （Basically the same here）
物理地址MAC：0C.29.AB.7C.00.02         (Must ensure that the first byte is even, other byte data values are casual)
本机IP（W5500模块IP）：192.168.0.246   （只要和目标IP不冲突即可）
本机端口：5000  (General default)

Do not do the following configuration in UDP mode
target IP：192.168.0.149	    (and the module should be under the same gateway)
Target port：6000  (General default)

*/



#include  "delay.h"
#include  "led.h"
#include  "usart.h"
#include  "spi.h"
#include  "w5500.h"
#include  "string.h"

u32 W5500_Send_Delay=0; //W5500 Send delay count variable(ms)


uint8_t fixstring[10] ="Tuan123456";

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

	S0_Port[0] = 0;//Load the port number of socket0 is 123 
	S0_Port[1] = 0x7B;

  S0_Mode=UDP_MODE;//Load port 0 working mode, UDP mode
}




//W5500 Initial configuration
void W5500_Initialization(void)
{
	Load_Net_Parameters();
	W5500_Init();		//Initialize the W5500 register
	//Detect_Gateway();	//Check the gateway server 
	//Set the specified Socket 0 to UDP mode
	//why i received the message???
	//Ban tin se den thang S0 tai port = 123.
	//Phai viet lai ham nay
	if(Socket2UDP(S0) == TRUE) S0_State=S_INIT|S_CONN;
	else S0_State=S_Unknown;
}




//Description : W5500 port initialization configuration
//NOTE: Set 4 ports respectively, and put the port in the TCP server, TCP client or UDP mode according to the port working mode.
//		 The port status byte Socket_State can be used to determine the working status of the port.
void W5500_Socket_Set(void)
{
	if(S0_State== S_Unknown)//Port 0 initial configuration
	{
		if(S0_Mode==TCP_SERVER)//TCP Server mode 
		{
			if(Socket_Listen(S0)==TRUE)
				S0_State=S_INIT;
			else
				S0_State=S_Unknown;
		}
		else if(S0_Mode==TCP_CLIENT)//TCP Client mode
		{
			if(Socket_Connect(S0)==TRUE)
				S0_State=S_INIT;
			else
				S0_State=S_Unknown;
		}
		else//UDP mode 
		{
			if(Socket2UDP(S0) == TRUE)
				S0_State=S_INIT|S_CONN;
			else
				S0_State=S_Unknown;
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
/*UDP Message Format
Source Port: The 16-bit port number of the process that originated the UDP message on the source device. 
Destination Port: The 16-bit port number of the process that is the ultimate intended recipient of the message on the destination device. 
Length: The length of the entire UDP datagram, including both header and Data fields.
Checksum: An optional 16-bit checksum ( option)
Data: The encapsulated higher-layer message to be sent.
*/
void Process_Socket_Data(SOCKET s)
{
	u8 i;
	u16 size;
	u16 mlength,startAddr;// do dai thuc su cua ban tin
	size=Read_SOCK_Data_Buffer(s, Rx_Buffer);
	mlength = (Rx_Buffer[6]<<8) + Rx_Buffer[7];
	printf("size : %d,length : %d\r\n",size,mlength);
	/*
	printf("RX buffer : ");
	for(i=0;i<size;i++) 
	{
		printf("[%d]%d-",i,*(Rx_Buffer+i));
	}
	*/
	UDP_DIPR[0] = Rx_Buffer[0];
	UDP_DIPR[1] = Rx_Buffer[1];
	UDP_DIPR[2] = Rx_Buffer[2];
	UDP_DIPR[3] = Rx_Buffer[3];

	UDP_DPORT[0] = Rx_Buffer[4];
	UDP_DPORT[1] = Rx_Buffer[5];
	memcpy(Tx_Buffer, Rx_Buffer+8, mlength);
	/*
	//Co ve TX buffer chua sach se khi gui?
	printf("TX buffer : ");
	for(i=0;i<size;i++) 
	{
		printf("[%d]%d-",i,*(Tx_Buffer+i));
	}
	*/
		//send over sender
	//Write_SOCK_Data_Buffer(s, Tx_Buffer, size);
	// Cai nay IP, Port of des, chon truoc khi nhap TX
	Write_W5500_SOCK_4Byte(s, Sn_DIPR, UDP_DIPR);//Set the destination IP  		
	Write_W5500_SOCK_2Byte(s, Sn_DPORTR, (UDP_DPORT[0]<<8)|+UDP_DPORT[1]);//Set the destination port
	
	startAddr=Read_W5500_SOCK_2Byte(s,Sn_TX_RD);
	printf("Sn_TX_RD 1 : %d ",startAddr);
	
	sendSocketNbuff(s, Tx_Buffer, mlength);
	
	startAddr=Read_W5500_SOCK_2Byte(s,Sn_TX_RD);
	printf("Sn_TX_RD 2 : %d\r\n",startAddr);
}

/*
- Ban tin den phai tuan tu
- If all socket with the same port?
*/

/*
Hercules UDP
IP: module's IP
port : module's port
local port : computer's port
*/


int main(void)
   {


		 delay_init();	    	 //Delay function initialization
		 uart_init(115200);	 //Serial port initialization is 9600  
		 printf("Start...\r\n");	
		 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		//Set NVIC interrupt packet 2: 2 bit preemption priority, 2 bit response priority

		 W5500_GPIO_Init();//Initialize W5500  RST INT SCS corresponds to GPIO status And configure INT interrupt mode	
		 SPI1_Init();	 //Initialize SPI1  PA5 PA6 PA7 IO is SPI mode
		 W5500_Hardware_Reset();		//Hardware reset W5500
		 W5500_Initialization();		//W5500 initial configuration
		 printf("Run...\r\n");
		while(1)
		{

			//W5500_Socket_Set();//W5500 port initialization configuration
      
			if(W5500_Interrupt)//Handling W5500 interrupts		
			{
				W5500_Interrupt_Process();//W5500 interrupt handler framework
			}
			if((S0_Data & S_RECEIVE) == S_RECEIVE)//If Socket0 receives data
			{
				S0_Data&=~S_RECEIVE;
				Process_Socket_Data(S0);//The W5500 receives and transmits the received data.
			}		
			
			else if(W5500_Send_Delay>=720000)//Send string periodically
			{
				if(S0_State == (S_INIT|S_CONN))
				{
					S0_Data&=~S_TRANSMITOK;
					//memcpy(Tx_Buffer, "\r\nW5500 UDP TEST\r\n", 18);	
					sendSocketNbuff(S0, fixstring, 10);//Specify Socket (0~7) to send data processing, port 0 to send 30 bytes of data
					
				}
				W5500_Send_Delay=0;
			}	
			
			W5500_Send_Delay++;

		}

   }

