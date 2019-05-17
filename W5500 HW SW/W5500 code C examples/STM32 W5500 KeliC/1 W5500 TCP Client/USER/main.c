

/*
 http://zq281598585.taobao.com/  
Kaiguang Electronics	  ELH    enlighten  sunny

w5500 Network module reference program

Running test MCU STM32F103RBT6


TCP Client mode 

Interrupt judgment method
The TCP client sets the module itself as a client and then communicates with the server.
Therefore, this mode needs to determine the module's own IP, port and target server's IP, port.


W5500 All pins correspond to the following�� 
sck PA5  miso PA6  mosi PA7  rst PA2  int PA3  cs PA0



Test routing data values

Gateway: 192.168.0.1		   ��It tests the route for us, and the route used by the average family is 192.168.1.1��
Subnet mask��255.255.255.0	   ��Basically the same here��
Physical address MAC��0C.29.AB.7C.00.02         (Must ensure that the first byte is even, other byte data values are casual)
����IP��W5500 Module IP����192.168.0.246   ��As long as it does not conflict with the target IP��
Native port��5000  ��General default��
Ŀ��IP��192.168.0.149	    ��And the module should be under the same gateway��
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

	Phy_Addr[0]=0x0c;//���������ַ
	Phy_Addr[1]=0x29;
	Phy_Addr[2]=0xab;
	Phy_Addr[3]=0x7c;
	Phy_Addr[4]=0x00;
	Phy_Addr[5]=0x02;

	IP_Addr[0]=192;//���ر���IP��ַ
	IP_Addr[1]=168;
	IP_Addr[2]=0;
	IP_Addr[3]=246;

	S0_Port[0] = 0x13;//���ض˿�0�Ķ˿ں�5000 
	S0_Port[1] = 0x88;

	S0_DIP[0]=192;//���ض˿�0��Ŀ��IP��ַ
	S0_DIP[1]=168;
	S0_DIP[2]=0;
	S0_DIP[3]=149;
	
	S0_DPort[0] = 0x17;//���ض˿�0��Ŀ�Ķ˿ں�6000
	S0_DPort[1] = 0x70;

	S0_Mode=TCP_CLIENT;//Load port 0 working mode, TCP client mode
}




//W5500 Initial configuration
void W5500_Initialization(void)
{
	W5500_Init();		//��ʼ��W5500�Ĵ���
	Detect_Gateway();	//������ط����� 
	Socket_Init(0);		//ָ��Socket(0~7)��ʼ��,��ʼ���˿�0
}




//Description : W5500 port initialization configuration
//˵�� : �ֱ�����4���˿�,���ݶ˿ڹ���ģʽ,���˿�����TCP��������TCP�ͻ��˻�UDPģʽ.
//		 �Ӷ˿�״̬�ֽ�Socket_State�����ж϶˿ڵĹ������
void W5500_Socket_Set(void)
{
	if(S0_State==0)//�˿�0��ʼ������
	{
		if(S0_Mode==TCP_SERVER)//TCP������ģʽ 
		{
			if(Socket_Listen(0)==TRUE)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else if(S0_Mode==TCP_CLIENT)//TCP�ͻ���ģʽ 
		{
			if(Socket_Connect(0)==TRUE)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else//UDPģʽ 
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

	SPI1_Init();	 //��ʼ��SPI1  PA5 PA6 PA7 IOΪSPIģʽ
	SPI1_SetSpeed(SPI_BaudRatePrescaler_2);	 //����SPI1�ٶ�Ϊ���

	W5500_GPIO_Init();//��ʼ��W5500  RST INT SCS��ӦGPIO״̬ ������INT�ж�ģʽ

	Load_Net_Parameters();		//װ���������	
	W5500_Hardware_Reset();		//Ӳ����λW5500
	W5500_Initialization();		//W5500��ʼ������

 
 	while(1)
	{

		W5500_Socket_Set();//W5500�˿ڳ�ʼ������

		if(W5500_Interrupt)//����W5500�ж�		
		{
			W5500_Interrupt_Process();//W5500�жϴ��������
		}
		if((S0_Data & S_RECEIVE) == S_RECEIVE)//���Socket0���յ�����
		{
			S0_Data&=~S_RECEIVE;
			Process_Socket_Data(0);//W5500���ղ����ͽ��յ�������
		}		
		else if(W5500_Send_Delay>=720000)//Send string periodically
		{
			if(S0_State == (S_INIT|S_CONN))
			{
				S0_Data&=~S_TRANSMITOK;
				memcpy(Tx_Buffer, "\r\n����Ƽ� W5500�ͻ���TEST\r\n", 30);	
				Write_SOCK_Data_Buffer(0, Tx_Buffer, 30);//Specify Socket (0~7) to send data processing, port 0 to send 30 bytes of data
			}
			W5500_Send_Delay=0;
		}	

		W5500_Send_Delay++;

	}

   }

