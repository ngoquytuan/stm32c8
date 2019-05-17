

/*
 http://zq281598585.taobao.com/  
Kaiguang Electronics	  ELH    enlighten  sunny

w5500 Network module reference program

Running test MCU STM32F103RBT6


TCP Client mode 

Interrupt judgment method
The TCP client sets the module itself as a client and then communicates with the server.
Therefore, this mode needs to determine the module's own IP, port and target server's IP, port.


W5500 All pins correspond to the following： 
sck PA5  miso PA6  mosi PA7  rst PA2  int PA3  cs PA0



Test routing data values

Gateway: 192.168.0.1		   （It tests the route for us, and the route used by the average family is 192.168.1.1）
Subnet mask：255.255.255.0	   （Basically the same here）
Physical address MAC：0C.29.AB.7C.00.02         (Must ensure that the first byte is even, other byte data values are casual)
本机IP（W5500 Module IP）：192.168.0.246   （As long as it does not conflict with the target IP）
Native port：5000  （General default）
目标IP：192.168.0.149	    （And the module should be under the same gateway）
Target port: 6000 (generally default)

*/



#include  "delay.h"
#include  "led.h"
#include  "usart.h"
#include  "spi.h"
#include  "w5500.h"
#include  "string.h"

u32 W5500_Send_Delay=0; //W5500 Send delay count variable(ms)


//Load network parameters
//Description: Gateway, subnet mask, physical address, local IP address, local port number, destination IP address, destination port number, port working mode
void Load_Net_Parameters(void)
{
	Gateway_IP[0] = 192;//Load gateway parameters
	Gateway_IP[1] = 168;
	Gateway_IP[2] = 0;
	Gateway_IP[3] = 1;

	Sub_Mask[0]=255;//Load subnet mask
	Sub_Mask[1]=255;
	Sub_Mask[2]=255;
	Sub_Mask[3]=0;

	Phy_Addr[0]=0x0c;//加载物理地址
	Phy_Addr[1]=0x29;
	Phy_Addr[2]=0xab;
	Phy_Addr[3]=0x7c;
	Phy_Addr[4]=0x00;
	Phy_Addr[5]=0x02;

	IP_Addr[0]=192;//加载本机IP地址
	IP_Addr[1]=168;
	IP_Addr[2]=0;
	IP_Addr[3]=246;

	S0_Port[0] = 0x13;//加载端口0的端口号5000 
	S0_Port[1] = 0x88;

	S0_DIP[0]=192;//加载端口0的目的IP地址
	S0_DIP[1]=168;
	S0_DIP[2]=0;
	S0_DIP[3]=149;
	
	S0_DPort[0] = 0x17;//加载端口0的目的端口号6000
	S0_DPort[1] = 0x70;

	S0_Mode=TCP_CLIENT;//Load port 0 working mode, TCP client mode
}




//W5500 Initial configuration
void W5500_Initialization(void)
{
	W5500_Init();		//初始化W5500寄存器
	Detect_Gateway();	//检查网关服务器 
	Socket_Init(0);		//指定Socket(0~7)初始化,初始化端口0
}




//Description : W5500 port initialization configuration
//说明 : 分别设置4个端口,根据端口工作模式,将端口置于TCP服务器、TCP客户端或UDP模式.
//		 从端口状态字节Socket_State可以判断端口的工作情况
void W5500_Socket_Set(void)
{
	if(S0_State==0)//端口0初始化配置
	{
		if(S0_Mode==TCP_SERVER)//TCP服务器模式 
		{
			if(Socket_Listen(0)==TRUE)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else if(S0_Mode==TCP_CLIENT)//TCP客户端模式 
		{
			if(Socket_Connect(0)==TRUE)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else//UDP模式 
		{
			if(Socket_UDP(0)==TRUE)
				S0_State=S_INIT|S_CONN;
			else
				S0_State=0;
		}
	}
}




//W5500 Receive and send received data
//s:The port number
//Description : This procedure is called first S_rx_process()From W5500 The port receives the data buffer to read the data,
//		 The read data is then copied from the Rx_Buffer to the Temp_Buffer buffer for processing.
//		 After processing, copy the data from Temp_Buffer to the Tx_Buffer buffer. Call S_tx_process()
//		 send data.
void Process_Socket_Data(SOCKET s)
{
	u16 size;
	size=Read_SOCK_Data_Buffer(s, Rx_Buffer);
	memcpy(Tx_Buffer, Rx_Buffer, size);			
	Write_SOCK_Data_Buffer(s, Tx_Buffer, size);
}





int main(void)
   {

   	delay_init();	    	 //Delay function initialization

	//uart_init(9600);	 //The serial port is initialized to 9600  
	LED_Init();		  	//Initialize the hardware interface to the LED connection

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		//Set NVIC interrupt packet 2: 2 bit preemption priority, 2 bit response priority

	SPI1_Init();	 //初始化SPI1  PA5 PA6 PA7 IO为SPI模式
	SPI1_SetSpeed(SPI_BaudRatePrescaler_2);	 //配置SPI1速度为最高

	W5500_GPIO_Init();//初始化W5500  RST INT SCS对应GPIO状态 并配置INT中断模式

	Load_Net_Parameters();		//装载网络参数	
	W5500_Hardware_Reset();		//硬件复位W5500
	W5500_Initialization();		//W5500初始化配置

 
 	while(1)
	{

		W5500_Socket_Set();//W5500端口初始化配置

		if(W5500_Interrupt)//处理W5500中断		
		{
			W5500_Interrupt_Process();//W5500中断处理程序框架
		}
		if((S0_Data & S_RECEIVE) == S_RECEIVE)//如果Socket0接收到数据
		{
			S0_Data&=~S_RECEIVE;
			Process_Socket_Data(0);//W5500接收并发送接收到的数据
		}		
		else if(W5500_Send_Delay>=720000)//Send string periodically
		{
			if(S0_State == (S_INIT|S_CONN))
			{
				S0_Data&=~S_TRANSMITOK;
				memcpy(Tx_Buffer, "\r\n启光科技 W5500客户端TEST\r\n", 30);	
				Write_SOCK_Data_Buffer(0, Tx_Buffer, 30);//Specify Socket (0~7) to send data processing, port 0 to send 30 bytes of data
			}
			W5500_Send_Delay=0;
		}	

		W5500_Send_Delay++;

	}

   }

