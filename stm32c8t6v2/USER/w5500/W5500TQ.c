/**********************************************************************************
W5500 Network module main code

**********************************************************************************/

#include "stm32f10x.h"
#include "spi.h"				
#include "W5500cn.h"
#include "delay.h"	


//Network parameter variable definition 
u8 Gateway_IP[4];//Gateway IP address 
u8 Sub_Mask[4];	//Subnet mask 
u8 Phy_Addr[6];	//Physical address (MAC) 
u8 IP_Addr[4];	//Local IP address 

u8 S0_Port[2];	//Port number of port 0(5000) 
u8 S0_DIP[4];	//Port 0 destination IP address 
u8 S0_DPort[2];	//Port 0 destination port number(6000) 

u8 UDP_DIPR[4];	    //UDP(Broadcast mode, destination host IP address
u8 UDP_DPORT[2];	//UDP(Broadcast) mode, destination host port number


//Port operating mode
u8 S0_Mode =3;	            //Port 0 operating mode, 0: TCP server mode, 1: TCP client mode, 2: UDP (broadcast) mode
#define TCP_SERVER	0x00	//TCP Server mode
#define TCP_CLIENT	0x01	//TCP Client mode
#define UDP_MODE	0x02	//UDP (broadcast) mode 

//Port running status
u8 S0_State =0;	            //Port 0 status record, 1: port complete initialization, 2 ports complete connection (can transfer data normally)
#define S_INIT		0x01	//Port completion initialization 
#define S_CONN		0x02	//Port complete connection,Can transfer data normally 

//Status of the port to send and receive data
u8 S0_Data;	               	//Port 0 receives and sends data status, 1: port receives data, 2: port sends data completed
#define S_RECEIVE	 0x01	//The port received a packet 
#define S_TRANSMITOK 0x02	//The port sends a packet to complete

//Port data buffer 
u8 Rx_Buffer[2048];	//Port receive data buffer 
u8 Tx_Buffer[2048];	//Port send data buffer 

u8 W5500_Interrupt;	//W5500 interrupt flag (0: no interrupt, 1: interrupt)







//W5500 Port initialization and configuration interrupt mode
//The corresponding configuration instructions:
//sck PA5  miso PA6  mosi PA7  rst PA2  int PA3  cs PA0
void W5500_GPIO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;	

 	EXTI_InitTypeDef EXTI_InitStructure;			//Interrupt configuration, which interrupt line  EXTI_Line0-15
													//mode EXTI_Mode_Interrupt Interrupt EXTI_Mode_Event event
													//Trigger mode  EXTI_Trigger_Falling Falling edge trigger
													//			EXTI_Trigger_Rising	 Rising edge trigger
													//			EXTI_Trigger_Rising_Falling	  Arbitrary level trigger

 	NVIC_InitTypeDef NVIC_InitStructure;			//Interrupt parameter interrupt priority


  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//Enable the alternate function clock



	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE );//PORTA Clock enable


	// W5500_RST Pin initialization configuration (PA2) 
	GPIO_InitStructure.GPIO_Pin  = W5500_RST;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(W5500_RST_GPIO, &GPIO_InitStructure);
	GPIO_ResetBits(W5500_RST_GPIO, W5500_RST);
	
	// W5500_INT Pin initialization configuration(PA3) 	
	GPIO_InitStructure.GPIO_Pin = W5500_INT;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(W5500_INT_GPIO, &GPIO_InitStructure);


	// Initialize the network module SPI-CS pin (PA0)
	GPIO_InitStructure.GPIO_Pin = W5500_SCS;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(W5500_SCS_GPIO, &GPIO_InitStructure);
	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);



 	// Enable the EXTI3 Interrupt 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;				//W5500_INT External interrupt channel
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//Preemption priority 2 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;			//Subpriority 2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//Enable external interrupt channel
	NVIC_Init(&NVIC_InitStructure);


  	// Connect EXTI Line3 to PA3 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource3);


	// PA3 as W5500 interrupt input 
	EXTI_InitStructure.EXTI_Line = EXTI_Line3;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
  
}


//Interrupt line 3 PA3 responds to data from W5500 and concatenates a flag.
void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line3) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line3);	//Clear interrupt line
		W5500_Interrupt=1;
	}
}



//Send a byte via SPI1
//dat Byte sent
//No return
void SPI1_Send_Byte(u8 dat)
{
	SPI_I2S_SendData(SPI1,dat);//Write 1 byte of data
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);//Infinite loop waiting for data register empty
}



//PI1 Send 2 bytes of data (16 bits)
//dat:16-bit data to be sent
void SPI1_Send_Short(u16 dat)
{
	SPI1_Send_Byte(dat/256);//Write data high dat>>8
	SPI1_Send_Byte(dat);	//Write data low
}


void spiburst_wb2(uint8_t *pBuf, uint16_t len)
{
	u16 i;
	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//CS chip selection W5500
	for(i=0;i<len;i++)//The loop writes the size bytes of the buffer to the W5500.
	{
		SPI1_Send_Byte(*pBuf++);//Write a byte of data
	}
	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //Pull high CS to cancel the chip selection

}

//Write 1 byte data to the W5500 specified address register via SPI1
// reg:16 Bit register address, dat:Data to be written
void Write_W5500_1Byte(u16 reg, u8 dat)
{
	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//CS chip selection W5500

	SPI1_Send_Short(reg);   //Write 16-bit register address via SPI1
	SPI1_Send_Byte(FDM1|RWB_WRITE|COMMON_R); //Write control byte via SPI1,1 byte data length,Write data,Select general purpose register
	SPI1_Send_Byte(dat);  //Write 1 byte of data

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //Pull high CS to cancel the chip selection
}




//Write 2 bytes of data to the specified address register via SPI1
//Reg: 16-bit register address, dat: 16-bit data to be written (2 bytes)
void Write_W5500_2Byte(u16 reg, u16 dat)
{
	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//CS chip selection W5500
		
	SPI1_Send_Short(reg);//Write 16-bit register address via SPI1
	SPI1_Send_Byte(FDM2|RWB_WRITE|COMMON_R);//Write control byte via SPI1,2 bytes of data length,Write data,Select general purpose register
	SPI1_Send_Short(dat);//Write 16-bit data

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //Pull high CS to cancel the chip selection
}




//Write n bytes of data to the specified address register via SPI1
//Reg: 16-bit register address, *dat_ptr: pointer to be written to the data buffer, size: length of data to be written
void Write_W5500_nByte(u16 reg, u8 *dat_ptr, u16 size)
{
	u16 i;

	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//CS chip selection W5500	
		
	SPI1_Send_Short(reg);//Write 16-bit register address via SPI1
	SPI1_Send_Byte(VDM|RWB_WRITE|COMMON_R);//Write control byte through SPI1, N bytes of data length, write data, select general purpose register

	for(i=0;i<size;i++)//The loop writes the size bytes of the buffer to the W5500.
	{
		SPI1_Send_Byte(*dat_ptr++);//Write a byte of data
	}

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //Pull high CS to cancel the chip selection
}




//Write 1 byte of data to the specified port register via SPI1
//s: port number, reg: 16-bit register address, dat: data to be written
void Write_W5500_SOCK_1Byte(SOCKET s, u16 reg, u8 dat)
{
	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//CS chip selection W5500	
		
	SPI1_Send_Short(reg);//Write 16-bit register address via SPI1
	SPI1_Send_Byte(FDM1|RWB_WRITE|(s*0x20+0x08));//Write control byte through SPI1, 1 byte data length, write data, select port s register
	SPI1_Send_Byte(dat);//Write 1 byte of data

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //Pull high CS to cancel the chip selection
}



//Write 2 bytes of data to the specified port register via SPI1
//s: port number, reg: 16-bit register address, dat: 16-bit data to be written (2 bytes)
void Write_W5500_SOCK_2Byte(SOCKET s, u16 reg, u16 dat)
{
	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//CS chip selection W5500
			
	SPI1_Send_Short(reg);//Write 16-bit register address via SPI1
	SPI1_Send_Byte(FDM2|RWB_WRITE|(s*0x20+0x08));//Write control byte via SPI1,2Byte data length,Write data,Select port s register
	SPI1_Send_Short(dat);//Write 16-bit data

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //Pull high CS to cancel the chip selection
}




//通过SPI1向指定端口寄存器写4个字节数据
//s:端口号,reg:16位寄存器地址,*dat_ptr:待写入的4个字节缓冲区指针
void Write_W5500_SOCK_4Byte(SOCKET s, u16 reg, u8 *dat_ptr)
{
	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//拉低CS CS chip selection W5500
			
	SPI1_Send_Short(reg);//Write 16-bit register address via SPI1
	SPI1_Send_Byte(FDM4|RWB_WRITE|(s*0x20+0x08));//Write control byte via SPI1,4Byte data length,Write data,Select port s register

	SPI1_Send_Byte(*dat_ptr++);//写第1个字节数据
	SPI1_Send_Byte(*dat_ptr++);//写第2个字节数据
	SPI1_Send_Byte(*dat_ptr++);//写第3个字节数据
	SPI1_Send_Byte(*dat_ptr++);//写第4个字节数据

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);//Pull high CS to cancel the chip selection
}



// Read the 1 byte data of the W5500 specified address register
// reg: 16-bit register address
// Return: read 1 byte of data into the register
u8 Read_W5500_1Byte(u16 reg)
{
	u8 i;

	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//Pull down CS, CS chip selection W5500
			
	SPI1_Send_Short(reg);  //Write 16-bit register address via SPI1
	SPI1_Send_Byte(FDM1|RWB_READ|COMMON_R);//Write control byte via SPI1,1Byte data length,Reading data,Select general purpose register

	i=SPI_I2S_ReceiveData(SPI1);
	SPI1_Send_Byte(0x00);//Send empty data, wait for return data
	i=SPI_I2S_ReceiveData(SPI1);//Read 1 byte of data

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);//Pull high CS to cancel the chip selection
	return i;//Return the read register data
}



//读W5500指定端口寄存器的1个字节数据
//s:端口号,reg:16位寄存器地址
//返回值:读取到寄存器的1个字节数据
u8 Read_W5500_SOCK_1Byte(SOCKET s, u16 reg)
{
	u8 i;

	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//拉低CS CS chip selection W5500
			
	SPI1_Send_Short(reg);   //Write 16-bit register address via SPI1
	SPI1_Send_Byte(FDM1|RWB_READ|(s*0x20+0x08));//Write control byte via SPI1,1Byte data length,Reading data,Select port s register

	i=SPI_I2S_ReceiveData(SPI1);
	SPI1_Send_Byte(0x00);   //Send empty data, wait for return data
	i=SPI_I2S_ReceiveData(SPI1);//读取1个字节数据

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);  //Pull high CS to cancel the chip selection
	return i;//Return the read register data
}



//读W5500指定端口寄存器的2个字节数据
//s:端口号,reg:16位寄存器地址
//返回值:读取到寄存器的2个字节数据(16位)
u16 Read_W5500_SOCK_2Byte(SOCKET s, u16 reg)
{
	u16 i;

	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//拉低CS CS chip selection W5500
			
	SPI1_Send_Short(reg);//Write 16-bit register address via SPI1
	SPI1_Send_Byte(FDM2|RWB_READ|(s*0x20+0x08));//Write control byte via SPI1,2Byte data length,Reading data,Select port s register

	i=SPI_I2S_ReceiveData(SPI1);
	SPI1_Send_Byte(0x00);//Send empty data, wait for return data
	i=SPI_I2S_ReceiveData(SPI1);//Read high data
	SPI1_Send_Byte(0x00);//Send empty data, wait for return data
	i*=256;
	i+=SPI_I2S_ReceiveData(SPI1);//Read low order data

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);//Pull high CS to cancel the chip selection
	return i;//Return the read register data
}




//Read data from the W5500 receive data buffer
//s: port number, *dat_ptr: data save buffer pointer
//Return value: the length of the read data, rx_size bytes
u16 Read_SOCK_Data_Buffer(SOCKET s, u8 *dat_ptr)
{
	u16 rx_size;
	u16 offset, offset1;
	u16 i;
	u8 j;

	rx_size=Read_W5500_SOCK_2Byte(s,Sn_RX_RSR);
	if(rx_size==0) return 0;  //Return if no data is received
	if(rx_size>1460) rx_size=1460;

	offset=Read_W5500_SOCK_2Byte(s,Sn_RX_RD);
	offset1=offset;
	offset&=(S_RX_SIZE-1);    //Calculate the actual physical address

	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//CS chip selection W5500

	SPI1_Send_Short(offset);//Write 16-bit address
	SPI1_Send_Byte(VDM|RWB_READ|(s*0x20+0x18));//Write control byte,NByte data length,Reading data,Select port s register
	j=SPI_I2S_ReceiveData(SPI1);
	
	if((offset+rx_size)<S_RX_SIZE)//If the maximum address does not exceed the maximum address of the W5500 receive buffer register
	{
		for(i=0;i<rx_size;i++)//Loop reading rx_size bytes of data
		{
			SPI1_Send_Byte(0x00);//Send a dummy data
			j=SPI_I2S_ReceiveData(SPI1);//Read 1 byte of data
			*dat_ptr=j;//Save the read data to the data save buffer
			dat_ptr++;//Data save buffer pointer address is incremented by 1
		}
	}
	else//If the maximum address exceeds the maximum address of the W5500 receive buffer register
	{
		offset=S_RX_SIZE-offset;
		for(i=0;i<offset;i++)//循环读取出前offset个字节数据
		{
			SPI1_Send_Byte(0x00);//Send a dummy data
			j=SPI_I2S_ReceiveData(SPI1);//Read 1 byte of data
			*dat_ptr=j;//Save the read data to the data save buffer
			dat_ptr++;//Data save buffer pointer address is incremented by 1
		}
		GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);  //Pull high CS to cancel the chip selection

		GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//拉低CS CS chip selection W5500

		SPI1_Send_Short(0x00);//写16位地址
		SPI1_Send_Byte(VDM|RWB_READ|(s*0x20+0x18));//Write control byte,NByte data length,Reading data,Select port s register
		j=SPI_I2S_ReceiveData(SPI1);

		for(;i<rx_size;i++)//Rx_size-offset bytes of data after cyclic reading
		{
			SPI1_Send_Byte(0x00);//发送一个哑数据
			j=SPI_I2S_ReceiveData(SPI1);//读取1个字节数据
			*dat_ptr=j;//将读取到的数据保存到数据保存缓冲区
			dat_ptr++;//数据保存缓冲区指针地址自增1
		}
	}
	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //Pull high CS to cancel the chip selection

	offset1+=rx_size;//Update the actual physical address, that is, the starting address of the received data next time.
	Write_W5500_SOCK_2Byte(s, Sn_RX_RD, offset1);
	Write_W5500_SOCK_1Byte(s, Sn_CR, RECV);//Send start receive command
	return rx_size;//Returns the length of the received data
}



// Write data to the W5500 data send buffer
// s: port number, *dat_ptr: data save buffer pointer, size: length of data to be written
void Write_SOCK_Data_Buffer(SOCKET s, u8 *dat_ptr, u16 size)
{
	u16 offset,offset1;
	u16 i;

	//If it is UDP mode, you can set the IP and port number of the destination host here.
	if((Read_W5500_SOCK_1Byte(s,Sn_MR)&0x0f) == MR_UDP)//If it is UDP mode, update the received host address and port to the module port.
	{													
		Write_W5500_SOCK_4Byte(s, Sn_DIPR, UDP_DIPR);//Set the destination host IP  		
		Write_W5500_SOCK_2Byte(s, Sn_DPORTR, UDP_DPORT[0]*256+UDP_DPORT[1]);//Set the destination host port number				
	}

	offset=Read_W5500_SOCK_2Byte(s,Sn_TX_WR);
	offset1=offset;
	offset&=(S_TX_SIZE-1);//Calculate the actual physical address

	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//Chip selection

	SPI1_Send_Short(offset);//Write 16-bit address
	SPI1_Send_Byte(VDM|RWB_WRITE|(s*0x20+0x10));//Write control byte,NByte data length,Write data,Select port s register

	if((offset+size)<S_TX_SIZE)//If the maximum address does not exceed the maximum address of the W5500 Transmit Buffer Register
	{
		for(i=0;i<size;i++)//Cyclicly write size bytes of data
		{
			SPI1_Send_Byte(*dat_ptr++);//Write one byte of data		
		}
	}
	else//If the maximum address exceeds the maximum address of the W5500 Transmit Buffer Register
	{
		offset=S_TX_SIZE-offset;
		for(i=0;i<offset;i++)//Offset byte data before loop write
		{
			SPI1_Send_Byte(*dat_ptr++);//Write one byte of data
		}
		GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //Set the SCS of the W5500 to a high level

		GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//Set the SCS of the W5500 to a low level

		SPI1_Send_Short(0x00);//Write 16-bit address
		SPI1_Send_Byte(VDM|RWB_WRITE|(s*0x20+0x10));//Write control byte,NByte data length,Write data,Select port s register

		for(;i<size;i++)//Loop write size-offset bytes of data
		{
			SPI1_Send_Byte(*dat_ptr++);//write one byte of data
		}
	}
	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //Set the SCS of the W5500 to a high level

	offset1+=size;//Update the actual physical address, that is, the starting address of the next time the data to be sent is sent to the transmit data buffer.
	Write_W5500_SOCK_2Byte(s, Sn_TX_WR, offset1);
	Write_W5500_SOCK_1Byte(s, Sn_CR, SEND);//Send start send command				
}



//W5500 Hardware reset
//Note: The reset pin of W5500 keeps low level for at least 500us to make W5500 enter reset state.
void W5500_Hardware_Reset(void)
{
	GPIO_ResetBits(W5500_RST_GPIO, W5500_RST);  //Reset pin pulled low
	delay_ms(50);
	GPIO_SetBits(W5500_RST_GPIO, W5500_RST);    //Reset pin pull high
	delay_ms(200);
	while((Read_W5500_1Byte(PHYCFGR)&LINK)==0); //Infinite loop waiting for Ethernet connection to complete
//Here you have to connect the network cable to the route and module, otherwise you will not be able to pass
}




//Initialize the corresponding register of W5500
//Description: After the hardware is initialized, it must be initialized accordingly.
void W5500_Init(void)
{
	u8 i=0;

	Write_W5500_1Byte(MR, RST);  //Software reset W5500, set to 1, automatically cleared after reset
	delay_ms(10);                //Delay 10ms

	//Set the IP address of the gateway (Gateway) to 4 bytes.
	//Using a gateway allows communication to break through the limitations of subnets, 
	//allowing access to other subnets or access through the gateway Internet
	Write_W5500_nByte(GAR, Gateway_IP, 4);
			
	//Set the subnet mask (MASK) value, SUB_MASK is 4 bytes
	//Subnet mask for subnet operations
	Write_W5500_nByte(SUBR,Sub_Mask,4);		
	
	//Set physical address,PHY_ADDR 6 bytes,Used to uniquely identify the physical address value of a network device
	//The address value needs to be applied to the IEEE，According to the OUI regulations, the first 3 bytes are the vendor code, and the last three bytes are the product serial number.
	//If you define your own physical address, note that the first byte must be even
	Write_W5500_nByte(SHAR,Phy_Addr,6);		

	//Set the IP address of this machine,IP_ADDR is 4 bytes
	//Note that the gateway IP must belong to the same subnet as the local IP, otherwise the local machine will not be able to find the gateway.
	Write_W5500_nByte(SIPR,IP_Addr,4);		
	
	//Set the size of the send buffer and receive buffer，Refer to the W5500 data sheet. Initialization is configured to 2k for a total of 16k.
	for(i=0;i<8;i++)
	{
		Write_W5500_SOCK_1Byte(i,Sn_RXBUF_SIZE, 0x02);//Socket Rx memory size=2k
		Write_W5500_SOCK_1Byte(i,Sn_TXBUF_SIZE, 0x02);//Socket Tx mempry size=2k
	}

	//Set the retry time, the default is 2000(200ms) 
	//The value of each unit is 100 microseconds, and the value at initialization is set to 2000 (0x07D0), equal to 200 milliseconds.
	Write_W5500_2Byte(RTR, 0x07d0);

	//Set the number of retries, the default is 8 times 
	//If the number of retransmissions exceeds the set value, a timeout interrupt is generated (the Sn_IR timeout bit (TIMEOUT) in the associated port interrupt register is set to "1")
	Write_W5500_1Byte(RCR,8);

	//Start the interrupt, refer to the W5500 data sheet to determine the type of interrupt you need.
	//IMR_CONFLICT is an abnormal interruption of IP address conflict, IMR_UNREACH is an abnormal interruption of address unreachable when UDP communication
	//Others are Socket event interrupts, add as needed
	Write_W5500_1Byte(IMR,IM_IR7 | IM_IR6);	//Interrupt enable register
	Write_W5500_1Byte(SIMR,S0_IMR);			//socket Interrupt Status 0-7 Registers are configured on S0
	Write_W5500_SOCK_1Byte(0, Sn_IMR, IMR_SENDOK | IMR_TIMEOUT | IMR_RECV | IMR_DISCON | IMR_CON);	//S0 Interrupt function is all open
}




//Check the gateway server
//It belongs to the corresponding operation of the analog one port, and judges whether the W5500 mode enters the communicable state.
//Return value: TRUE (0xFF) is returned successfully, FALSE (0x00) is returned.
u8 Detect_Gateway(void)
{
	u8 ip_adde[4];
	ip_adde[0]=IP_Addr[0]+1;
	ip_adde[1]=IP_Addr[1]+1;
	ip_adde[2]=IP_Addr[2]+1;
	ip_adde[3]=IP_Addr[3]+1;

	//Check the gateway and get the physical address of the gateway
	Write_W5500_SOCK_4Byte(0,Sn_DIPR,ip_adde);//Write a different IP value to the destination address register than the local IP address
	Write_W5500_SOCK_1Byte(0,Sn_MR,MR_TCP);   //Set the socket to TCP mode.
	Write_W5500_SOCK_1Byte(0,Sn_CR,OPEN);     //Open Socket	
	delay_ms(5);//Delay 5ms 	
	
	if(Read_W5500_SOCK_1Byte(0,Sn_SR) != SOCK_INIT)//If the socket fails to open, the TCP mode fails to start.
	{
		Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);//Open unsuccessful, close Socket
		return FALSE;//Return FALSE(0x00)
	}

	Write_W5500_SOCK_1Byte(0,Sn_CR,CONNECT);//Set Socket to Connect mode TCP as client					

	do
	{
		u8 j=0;
		j=Read_W5500_SOCK_1Byte(0,Sn_IR);//Read Socket0 interrupt flag register
		if(j!=0)						 //If there is an interrupt prompt, write 1 clear corresponding interrupt
		Write_W5500_SOCK_1Byte(0,Sn_IR,j);
		delay_ms(5);
		if((j&IR_TIMEOUT) == IR_TIMEOUT) //time out
		{
			return FALSE;	
		}
		else if(Read_W5500_SOCK_1Byte(0,Sn_DHAR) != 0xff)	 //Only judge whether the last MAC is FF. If it is successfully loaded, it should not be FF.
		{
			Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);//Close Socket Description Test successful
			return TRUE;							
		}
	}while(1);
}




//Specify Socket (0~7) initialization
//s:Port to be initialized
void Socket_Init(SOCKET s)
{
	//Set the fragment length, refer to the W5500 data sheet, the value can be modified.	
	Write_W5500_SOCK_2Byte(0, Sn_MSSR, 1460);//Maximum number of fragmented bytes =1460(0x5b4)
	//Set the specified port
	switch(s)
	{
		case 0:
			//Set the port number of port 0
			Write_W5500_SOCK_2Byte(0, Sn_PORT, S0_Port[0]*256+S0_Port[1]);
			//Set port 0 destination (remote) port number
			Write_W5500_SOCK_2Byte(0, Sn_DPORTR, S0_DPort[0]*256+S0_DPort[1]);
			//Set port 0 destination (remote) IP address
			Write_W5500_SOCK_4Byte(0, Sn_DIPR, S0_DIP);			
			
			break;

		case 1:
			break;

		case 2:
			break;

		case 3:
			break;

		case 4:
			break;

		case 5:
			break;

		case 6:
			break;

		case 7:
			break;

		default:
			break;
	}
}



//设置指定Socket(0~7)为客户端与远程服务器连接
//s:待设定的端口
//返回值:成功返回TRUE(0xFF),失败返回FALSE(0x00)
//说明  :当本机Socket工作在客户端模式时,引用该程序,与远程服务器建立连接
//	     如果启动连接后出现超时中断，则与服务器连接失败,需要重新调用该程序连接
//	     该程序每调用一次,就与服务器产生一次连接
u8 Socket_Connect(SOCKET s)
{
	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_TCP);//设置socket为TCP模式
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//打开Socket
	delay_ms(5);//延时5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_INIT)//如果socket打开失败
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//打开不成功,关闭Socket
		return FALSE;//返回FALSE(0x00)
	}
	Write_W5500_SOCK_1Byte(s,Sn_CR,CONNECT);//设置Socket为Connect模式
	return TRUE;//返回TRUE,设置成功
}




//设置指定Socket(0~7)作为服务器等待远程主机的连接
//s:待设定的端口
//返回值:成功返回TRUE(0xFF),失败返回FALSE(0x00)
//说明  :当本机Socket工作在服务器模式时,引用该程序,等等远程主机的连接
//		 该程序只调用一次,就使W5500设置为服务器模式
u8 Socket_Listen(SOCKET s)
{
	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_TCP);//设置socket为TCP模式 
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//打开Socket	
	delay_ms(5);//延时5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_INIT)//如果socket打开失败
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//打开不成功,关闭Socket
		return FALSE;//返回FALSE(0x00)
	}	
	Write_W5500_SOCK_1Byte(s,Sn_CR,LISTEN);//设置Socket为侦听模式	
	delay_ms(5);//延时5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_LISTEN)//如果socket设置失败
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//设置不成功,关闭Socket
		return FALSE;//返回FALSE(0x00)
	}

	return TRUE;

	//至此完成了Socket的打开和设置侦听工作,至于远程客户端是否与它建立连接,则需要等待Socket中断，
	//以判断Socket的连接是否成功。参考W5500数据手册的Socket中断状态
	//在服务器侦听模式不需要设置目的IP和目的端口号
}





// Set the specified Socket (0 ~ 7) to UDP mode
//s: the port to be set
// Return value: success returns TRUE (0xFF), failure returns FALSE (0x00)
// Description: If the Socket works in UDP mode, refer to the program, in UDP mode, Socket communication does not need to establish a connection
// The program is called only once, setting W5500 to UDP mode.
u8 Socket_UDP(SOCKET s)
{
	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_UDP);//Set Socket to UDP mode*/
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//Open Socket*/
	delay_ms(5);
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_UDP)//If Socket fails to open
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//Open unsuccessful, close Socket
		return FALSE;//Return FALSE (0x00)
	}
	else
		return TRUE;

	//This completes the Socket open and UDP mode settings, in which it does not need to establish a connection with the remote host.
	//Because Socket does not need to establish a connection, you can set the port number of the destination host IP and destination Socket before sending data.
	//If the port number of the destination host IP and destination Socket is fixed and has not changed during the running process, it can also be set here.
}





/**
  * @brief  W5500_Interrupt_Process
  * @param  Xu ly ngat W5500
  * @retval  
  */
void W5500_Interrupt_Process(void)
{
	u8 i,j;

Here:
	W5500_Interrupt=0;//Clear interrupt flag
	i = Read_W5500_1Byte(IR);//Read interrupt flag register
	Write_W5500_1Byte(IR, (i&0xf0));//Write back clear interrupt flag
	
//	if((i & CONFLICT) == CONFLICT)//IP Address conflict exception handling
//	{
//		 //Add code yourself
//	}
//
//	if((i & UNREACH) == UNREACH)//UDP Address cannot reach exception handling in mode
//	{
//		//Add code yourself
//	}

	i=Read_W5500_1Byte(SIR);//Read port interrupt flag register
  	
	if((i & S0_INT) == S0_INT)//Socket0 Event processing
	{
		
		j=Read_W5500_SOCK_1Byte(0,Sn_IR); //Read Socket0 interrupt flag register
		//printf("Sn_IR truoc:%d\r\n",j);
		Write_W5500_SOCK_1Byte(0,Sn_IR,j);//Write back clear interrupt flag
		
		
		if(j&IR_CON)//In TCP mode, Socket0 is successfully connected. 
		{
			S0_State|=S_CONN;//Network connection status 0x02, the port is connected, and the data can be transmitted normally.
			printf("TCP S0 is connected\r\n");
		}
		if(j&IR_DISCON)//Socket disconnection processing in TCP mode
		{
			Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);//Close the port and wait for the connection to be reopened
			Socket_Init(0);		//Specify Socket (0~7) initialization, initialize port 0
			S0_State=0;//Network connection status 0x00, port connection failed
			printf("S0 tcp disconnect\r\n");
		}
		if(j&IR_SEND_OK)//Socket0 data transmission is completed, you can start the S_tx_process() function to send data again. 
		{
			S0_Data|=S_TRANSMITOK;//The port sends a packet to complete 
			printf("S0 TRANSMIT OK\r\n");
		}
		if(j&IR_RECV)//Socket receives the data, you can start the S_rx_process () function 
		{
			S0_Data|=S_RECEIVE;//The port received a packet
			printf("S0 0 received\r\n");
		}
		if(j&IR_TIMEOUT)//Socket connection or data transfer timeout processing 
		{
			Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);// Close the port and wait for the connection to be reopened
			S0_State=0;//Network connection status 0x00, port connection failed
			printf("Socket timeout\r\n");
		}
	}
	else printf("else\r\n");

	if(Read_W5500_1Byte(SIR) != 0) 
		goto Here;
}

