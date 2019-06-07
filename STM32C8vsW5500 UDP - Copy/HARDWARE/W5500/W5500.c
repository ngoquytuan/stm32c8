/**********************************************************************************
W5500网络模块 主代码

**********************************************************************************/

#include "stm32f10x.h"
#include "spi.h"				
#include "W5500.h"
#include "delay.h"	

//Network parameter variable definition 
u8 Gateway_IP[4];//Gateway IP address 
u8 Sub_Mask[4];	//Subnet mask 
u8 Phy_Addr[6];	//Physical address (MAC) 
u8 IP_Addr[4];	//Local IP address 

u8 S0_Port[2];	//Port number of port 0 
u8 S0_DIP[4];	//Port 0 destination IP address 
u8 S0_DPort[2];	//Port 0 destination port number 

u8 UDP_DIPR[4];	    //UDP(Broadcast mode, destination host IP address
u8 UDP_DPORT[2];	//UDP(Broadcast) mode, destination host port number


//Port operating mode
u8 S0_Mode =3;	            //Port 0 operating mode, 0: TCP server mode, 1: TCP client mode, 2: UDP (broadcast) mode
#define TCP_SERVER	0x00	//TCP Server mode
#define TCP_CLIENT	0x01	//TCP Client mode
#define UDP_MODE	0x02	//UDP (broadcast) mode 

//Socket status ?
//#define S_Unknown 0x00  //Khong ro trang thai 
//#define S_INIT		0x01	//Port completion initialization 
//#define S_CONN		0x02	//Port complete connection,Can transfer data normally 
u8 S0_State = S_Unknown;	            //Port 0 status record, 1: port complete initialization, 2 ports complete connection (can transfer data normally)

//Status of the port to send and receive data
u8 S0_Data;	               	//Port 0 receives and sends data status, 1: port receives data, 2: port sends data completed
#define S_RECEIVE	 0x01	//The port received a packet 
#define S_TRANSMITOK 0x02	//The port sends a packet to complete

//Port data buffer 
u8 Rx_Buffer[2048];	//Port receive data buffer 
u8 Tx_Buffer[2048];	//Port send data buffer 

u8 W5500_Interrupt;	//W5500 interrupt flag (0: no interrupt, 1: interrupt)







//W5500各端口初始化及配置中断模式
//相应配置说明：
//sck PA5  miso PA6  mosi PA7  rst PA2  int PA3  cs PA0
void W5500_GPIO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;	

 	EXTI_InitTypeDef EXTI_InitStructure;			//中断配置	  即哪个中断线  EXTI_Line0-15
													//模式 EXTI_Mode_Interrupt中断  EXTI_Mode_Event 事件
													//触发方式  EXTI_Trigger_Falling 下降沿触发
													//			EXTI_Trigger_Rising	 上升沿触发
													//			EXTI_Trigger_Rising_Falling	  任意电平触发

 	NVIC_InitTypeDef NVIC_InitStructure;			//中断参数 中断优先级


  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//使能复用功能时钟



	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE );//PORTB时钟使能 


	// W5500_RST引脚初始化配置(PA2) 
	GPIO_InitStructure.GPIO_Pin  = W5500_RST;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(W5500_RST_GPIO, &GPIO_InitStructure);
	GPIO_ResetBits(W5500_RST_GPIO, W5500_RST);
	
	// W5500_INT引脚初始化配置(PA3) 	
	GPIO_InitStructure.GPIO_Pin = W5500_INT;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(W5500_INT_GPIO, &GPIO_InitStructure);


	// 初始化网络模块SPI-CS引脚 (PA0)
	GPIO_InitStructure.GPIO_Pin = W5500_SCS;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(W5500_SCS_GPIO, &GPIO_InitStructure);
	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);



 	// Enable the EXTI3 Interrupt 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;				//W5500_INT所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;			//子优先级2
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//使能外部中断通道
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


//中断线3  PA3响应 W5500来的数据 并置一标志位
void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line3) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line3);	//清中断线
		W5500_Interrupt=1;
	}
}



//通过SPI1发送一个字节
//dat 发送的字节
//无返回
void SPI1_Send_Byte(u8 dat)
{
	SPI_I2S_SendData(SPI1,dat);//写1个字节数据
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);//死循环等待数据寄存器空
}



//PI1发送2个字节数据(16位)
//dat:待发送的16位数据
void SPI1_Send_Short(u16 dat)
{
	SPI1_Send_Byte(dat>>8);//写数据高位   相当于dat>>8
	SPI1_Send_Byte(dat);	//写数据低位
}




//通过SPI1向W5500指定地址寄存器写1个字节数据
// reg:16位寄存器地址,dat:待写入的数据
void Write_W5500_1Byte(u16 reg, u8 dat)
{
	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//CS片选W5500

	SPI1_Send_Short(reg);   //通过SPI1写16位寄存器地址
	SPI1_Send_Byte(FDM1|RWB_WRITE|COMMON_R); //通过SPI1写控制字节,1个字节数据长度,写数据,选择通用寄存器
	SPI1_Send_Byte(dat);  //写1个字节数据

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //拉高CS  取消片选
}




//通过SPI1向指定地址寄存器写2个字节数据
//reg:16位寄存器地址,dat:16位待写入的数据(2个字节)
void Write_W5500_2Byte(u16 reg, u16 dat)
{
	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//CS片选W5500
		
	SPI1_Send_Short(reg);//通过SPI1写16位寄存器地址
	SPI1_Send_Byte(FDM2|RWB_WRITE|COMMON_R);//通过SPI1写控制字节,2个字节数据长度,写数据,选择通用寄存器
	SPI1_Send_Short(dat);//写16位数据

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //拉高CS  取消片选
}




//通过SPI1向指定地址寄存器写n个字节数据
//reg:16位寄存器地址,*dat_ptr:待写入数据缓冲区指针,size:待写入的数据长度
void Write_W5500_nByte(u16 reg, u8 *dat_ptr, u16 size)
{
	u16 i;

	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//CS片选W5500	
		
	SPI1_Send_Short(reg);//通过SPI1写16位寄存器地址
	SPI1_Send_Byte(VDM|RWB_WRITE|COMMON_R);//通过SPI1写控制字节,N个字节数据长度,写数据,选择通用寄存器

	for(i=0;i<size;i++)//循环将缓冲区的size个字节数据写入W5500
	{
		SPI1_Send_Byte(*dat_ptr++);//写一个字节数据
	}

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //拉高CS  取消片选
}




//通过SPI1向指定端口寄存器写1个字节数据
//s:端口号,reg:16位寄存器地址,dat:待写入的数据
void Write_W5500_SOCK_1Byte(SOCKET s, u16 reg, u8 dat)
{
	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//CS片选W5500	
		
	SPI1_Send_Short(reg);//通过SPI1写16位寄存器地址
	SPI1_Send_Byte(FDM1|RWB_WRITE|(s*0x20+0x08));//通过SPI1写控制字节,1个字节数据长度,写数据,选择端口s的寄存器
	SPI1_Send_Byte(dat);//写1个字节数据

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //拉高CS  取消片选
}



//通过SPI1向指定端口寄存器写2个字节数据
//s:端口号,reg:16位寄存器地址,dat:16位待写入的数据(2个字节)
void Write_W5500_SOCK_2Byte(SOCKET s, u16 reg, u16 dat)
{
	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//CS片选W5500
			
	SPI1_Send_Short(reg);//通过SPI1写16位寄存器地址
	SPI1_Send_Byte(FDM2|RWB_WRITE|(s*0x20+0x08));//通过SPI1写控制字节,2个字节数据长度,写数据,选择端口s的寄存器
	SPI1_Send_Short(dat);//写16位数据

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //拉高CS  取消片选
}




//通过SPI1向指定端口寄存器写4个字节数据
//s:端口号,reg:16位寄存器地址,*dat_ptr:待写入的4个字节缓冲区指针
void Write_W5500_SOCK_4Byte(SOCKET s, u16 reg, u8 *dat_ptr)
{
	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//拉低CS CS片选W5500
			
	SPI1_Send_Short(reg);//通过SPI1写16位寄存器地址
	SPI1_Send_Byte(FDM4|RWB_WRITE|(s*0x20+0x08));//通过SPI1写控制字节,4个字节数据长度,写数据,选择端口s的寄存器

	SPI1_Send_Byte(*dat_ptr++);//写第1个字节数据
	SPI1_Send_Byte(*dat_ptr++);//写第2个字节数据
	SPI1_Send_Byte(*dat_ptr++);//写第3个字节数据
	SPI1_Send_Byte(*dat_ptr++);//写第4个字节数据

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);//拉高CS  取消片选
}



//读取W5500指定地址寄存器的1个字节数据
//reg:16位寄存器地址
//返回:读取到寄存器的1个字节数据
u8 Read_W5500_1Byte(u16 reg)
{
	u8 i;

	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//拉低CS CS片选W5500
			
	SPI1_Send_Short(reg);  //通过SPI1写16位寄存器地址
	SPI1_Send_Byte(FDM1|RWB_READ|COMMON_R);//通过SPI1写控制字节,1个字节数据长度,读数据,选择通用寄存器

	i=SPI_I2S_ReceiveData(SPI1);
	SPI1_Send_Byte(0x00);//发送空数据 等待返回数据
	i=SPI_I2S_ReceiveData(SPI1);//读取1个字节数据

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);//拉高CS  取消片选
	return i;//返回读取到的寄存器数据
}



//读W5500指定端口寄存器的1个字节数据
//s:端口号,reg:16位寄存器地址
//返回值:读取到寄存器的1个字节数据
u8 Read_W5500_SOCK_1Byte(SOCKET s, u16 reg)
{
	u8 i;

	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//拉低CS CS片选W5500
			
	SPI1_Send_Short(reg);   //通过SPI1写16位寄存器地址
	SPI1_Send_Byte(FDM1|RWB_READ|(s*0x20+0x08));//通过SPI1写控制字节,1个字节数据长度,读数据,选择端口s的寄存器

	i=SPI_I2S_ReceiveData(SPI1);
	SPI1_Send_Byte(0x00);   //发送空数据 等待返回数据
	i=SPI_I2S_ReceiveData(SPI1);//读取1个字节数据

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);  //拉高CS  取消片选
	return i;//返回读取到的寄存器数据
}



//读W5500指定端口寄存器的2个字节数据
//s:端口号,reg:16位寄存器地址
//返回值:读取到寄存器的2个字节数据(16位)
u16 Read_W5500_SOCK_2Byte(SOCKET s, u16 reg)
{
	u16 i;

	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//拉低CS CS片选W5500
			
	SPI1_Send_Short(reg);//通过SPI1写16位寄存器地址
	SPI1_Send_Byte(FDM2|RWB_READ|(s*0x20+0x08));//通过SPI1写控制字节,2个字节数据长度,读数据,选择端口s的寄存器

	i=SPI_I2S_ReceiveData(SPI1);
	SPI1_Send_Byte(0x00);//发送空数据 等待返回数据
	i=SPI_I2S_ReceiveData(SPI1);//读取高位数据
	SPI1_Send_Byte(0x00);//发送空数据 等待返回数据
	i*=256;
	i+=SPI_I2S_ReceiveData(SPI1);//读取低位数据

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);//拉高CS  取消片选
	return i;//返回读取到的寄存器数据
}




//从W5500接收数据缓冲区中读取数据
//s:端口号,*dat_ptr:数据保存缓冲区指针
//返回值:读取到的数据长度,rx_size个字节
u16 Read_SOCK_Data_Buffer(SOCKET s, u8 *dat_ptr)
{
	u16 rx_size;
	u16 offset, offset1;
	u16 i;
	u8 j;

	rx_size=Read_W5500_SOCK_2Byte(s,Sn_RX_RSR);
	if(rx_size==0) return 0;  //没接收到数据则返回
	if(rx_size>1460) rx_size=1460;

	offset=Read_W5500_SOCK_2Byte(s,Sn_RX_RD);
	offset1=offset;
	offset&=(S_RX_SIZE-1);    //计算实际的物理地址

	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//拉低CS CS片选W5500

	SPI1_Send_Short(offset);//写16位地址
	SPI1_Send_Byte(VDM|RWB_READ|(s*0x20+0x18));//写控制字节,N个字节数据长度,读数据,选择端口s的寄存器
	j=SPI_I2S_ReceiveData(SPI1);
	
	if((offset+rx_size)<S_RX_SIZE)//如果最大地址未超过W5500接收缓冲区寄存器的最大地址
	{
		for(i=0;i<rx_size;i++)//循环读取rx_size个字节数据
		{
			SPI1_Send_Byte(0x00);//发送一个哑数据
			j=SPI_I2S_ReceiveData(SPI1);//读取1个字节数据
			*dat_ptr=j;//将读取到的数据保存到数据保存缓冲区
			dat_ptr++;//数据保存缓冲区指针地址自增1
		}
	}
	else//如果最大地址超过W5500接收缓冲区寄存器的最大地址
	{
		offset=S_RX_SIZE-offset;
		for(i=0;i<offset;i++)//循环读取出前offset个字节数据
		{
			SPI1_Send_Byte(0x00);//发送一个哑数据
			j=SPI_I2S_ReceiveData(SPI1);//读取1个字节数据
			*dat_ptr=j;//将读取到的数据保存到数据保存缓冲区
			dat_ptr++;//数据保存缓冲区指针地址自增1
		}
		GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);  //拉高CS  取消片选

		GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//拉低CS CS片选W5500

		SPI1_Send_Short(0x00);//写16位地址
		SPI1_Send_Byte(VDM|RWB_READ|(s*0x20+0x18));//写控制字节,N个字节数据长度,读数据,选择端口s的寄存器
		j=SPI_I2S_ReceiveData(SPI1);

		for(;i<rx_size;i++)//循环读取后rx_size-offset个字节数据
		{
			SPI1_Send_Byte(0x00);//发送一个哑数据
			j=SPI_I2S_ReceiveData(SPI1);//读取1个字节数据
			*dat_ptr=j;//将读取到的数据保存到数据保存缓冲区
			dat_ptr++;//数据保存缓冲区指针地址自增1
		}
	}
	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //拉高CS  取消片选

	offset1+=rx_size;//更新实际物理地址,即下次读取接收到的数据的起始地址
	Write_W5500_SOCK_2Byte(s, Sn_RX_RD, offset1);
	Write_W5500_SOCK_1Byte(s, Sn_CR, RECV);//发送启动接收命令
	return rx_size;//返回接收到数据的长度
}
void readSnTX_FSR(SOCKET s)
{
	
}

uint16_t getTXFreeSize(SOCKET s)
{
    uint16_t val=0, val1=0;
    do {
        val1 = Read_W5500_SOCK_2Byte(s,Sn_TX_FSR);
        if (val1 != 0)
            val = Read_W5500_SOCK_2Byte(s,Sn_TX_FSR);
    } 
    while (val != val1);
    return val;
}

/* Viet lai ham nay
- 	Tach phan des Ip, port cua UDP ra, viet truoc ham nay va ben ngoai ham nay
- 	Sn_TX_FSR indicates the free size of Socket n TX Buffer Block.
	=> Kiem tra thanh ghi nay xem du lieu con trong ko truoc khi gui ban tin?
	=> Neu da cau hinh thi gia tri thanh ghi nay ko doi ( vi du 2048) nen ko co y nghia gi khi doc no => ignore
*/
void sendSocketNbuff(SOCKET s, u8 *dat_ptr, u16 length)
{
	u16 startAddr,endAddr;
	u16 i;
	u16 freeSizeRegs,snRD;
	//__disable_irq();
	//Socket2UDP(s);// lenh nay reset tat ca cac thanh ghi!!!
	//Nhung mo lai lien tuc rat hay loi ben may tinh???Sao lai the nhi??
	/*
	if((startAddr+length)>=S_TX_SIZE)//If the maximum address does not exceed the maximum address of the W5500 Transmit Buffer Register
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//Socket n is initialized and opened => Reset all Sn_TX_RD, Sn_TX_WR : hay loi hon???
	}
	*/
	//freeSizeRegs=Read_W5500_SOCK_2Byte(s,Sn_TX_FSR);// Cai nay tang dan den max r ve 0???
	freeSizeRegs = getTXFreeSize(s);
	snRD=Read_W5500_SOCK_2Byte(s,Sn_TX_RD);// Cai nay tang dan den max r ve 0???
	
	printf("freeSizeRegs:%d \r\n",freeSizeRegs);
	printf("snRD:%d \r\n",snRD);
	
	//Sn_TX_WR :Socket n TX Write Pointer Register [R/W] [0x0024-0x0025] [0x0000]
	//1.Read the starting address for saving the transmitting data.
	startAddr=Read_W5500_SOCK_2Byte(s,Sn_TX_WR);// Cai nay tang dan den max r ve 0???
	startAddr&=(S_TX_SIZE-1);//startAddr = startAddr & 0x7ff?
	endAddr=startAddr;
	printf("startAddr:%d \r\n",startAddr);
    //2.Save the transmitting data from the starting address of Socket n TX buffer.
	W5500_NCS;//Chip selection
	/* example:
	When the Host writes 5 Bytes Data (0x11, 0x22, 0x33, 0x44, 0x55) to Socket 1抯 TX
    Buffer Block 0x0040 Address by using VDM mode, 5 bytes data are written with the SPI Frame below.*/
    //Write nBytes Data at Socketn抯 TX Buffer Block 0xXXXX in VDM mode
	SPI1_Send_Short(startAddr);//
	//Doan nay ko hieu???
	SPI1_Send_Byte(VDM|RWB_WRITE|(s*0x20+0x10));//Write control byte,NByte data length,Write data,Select port s register?????

	if((startAddr+length)<S_TX_SIZE)//If the maximum address does not exceed the maximum address of the W5500 Transmit Buffer Register
	{
		for(i=0;i<length;i++)//Cyclicly write length bytes of data
		{
			SPI1_Send_Byte(*dat_ptr++);//Write one byte of data		
		}
		endAddr += length;
	}// Doan nay ko hieu luon????
	else//If the maximum address exceeds the maximum address of the W5500 Transmit Buffer Register
	{
		startAddr=S_TX_SIZE-startAddr;
		printf("RE startAddr:%d \r\n",startAddr);
		for(i=0;i<startAddr;i++)//startAddr byte data before loop write
		{
			SPI1_Send_Byte(*dat_ptr++);//Write one byte of data
		}
		/*
		W5500_CS; //Chip deselect

		W5500_NCS;//Chip select

		SPI1_Send_Short(0x00);//Write 16-bit address
		SPI1_Send_Byte(VDM|RWB_WRITE|(s*0x20+0x10));//Write control byte,NByte data length,Write data,Select port s register

		for(;i<length;i++)//Loop write length-startAddr bytes of data
		{
			SPI1_Send_Byte(*dat_ptr++);//write one byte of data
		}
		endAddr = length-startAddr;
		*/
		endAddr=0;
	}
	W5500_CS; //Chip deselect
	/*3. After saving the transmitting data, update Sn_TX_WR to the
	increased value as many as transmitting data size. If the increment value
	exceeds the maximum value 0xFFFF(greater than 0x10000 and the carry
	bit occurs), then the carry bit is ignored and will automatically update
	with the lower 16bits value.*/
	
	//endAddr += length;//Update the actual physical address, that is, the starting address of the next time the data to be sent is sent to the transmit data buffer.
	printf("endAddr:%d ",endAddr);
	Write_W5500_SOCK_2Byte(s, Sn_TX_WR, endAddr);
	
	//4. Transmit the saved data in Socket n TX Buffer by using SEND/SEND command*/
	Write_W5500_SOCK_1Byte(s, Sn_CR, SEND);//Send start send command
}

// Write data to the W5500 data send buffer
//s: port number, *dat_ptr: data save buffer pointer, size: length of data to be written
void Write_SOCK_Data_Buffer(SOCKET s, u8 *dat_ptr, u16 size)
{
	u16 offset,offset1;
	u16 i;
    //KO hi?u trong TCP mode thì th? nào?
	//If it is UDP mode, you can set the IP and port number of the destination host here.
	if((Read_W5500_SOCK_1Byte(s,Sn_MR)&0x0f) == MR_UDP)//If it is UDP mode, update the received host address and port to the module port.
	{													
		Write_W5500_SOCK_4Byte(s, Sn_DIPR, UDP_DIPR);//Set the destination IP  		
		Write_W5500_SOCK_2Byte(s, Sn_DPORTR, (UDP_DPORT[0]<<8)|+UDP_DPORT[1]);//Set the destination port				
	}
    
	//Sn_TX_WR :Socket n TX Write Pointer Register [R/W] [0x0024-0x0025] [0x0000]
	offset=Read_W5500_SOCK_2Byte(s,Sn_TX_WR);//offset có ??a ch? th?ng s0 Tx
	offset1=offset;// L?u ??a ch? này sang th?ng offset1???
	offset&=(S_TX_SIZE-1);//Calculate the actual physical address???? offset = offset & 0x7ff => ??m b?o ??a ch? ko bay ra ngoài khu 16k

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



//W5500硬件复位
//说明：W5500的复位引脚保持低电平至少500us以上,才能使W5500进入复位状态
void W5500_Hardware_Reset(void)
{
	GPIO_ResetBits(W5500_RST_GPIO, W5500_RST);  //复位引脚拉低
	delay_ms(50);
	GPIO_SetBits(W5500_RST_GPIO, W5500_RST);    //复位引脚拉高
	delay_ms(200);
	while((Read_W5500_1Byte(PHYCFGR)&LINK)==0); //死循环等待以太网连接完成	这里要将网线连接到路由和模块上 否则将无法通过
}




//Initialize the corresponding register of W5500
//Description: After the hardware is initialized, it must be initialized accordingly.
void W5500_Init(void)
{
	u8 i=0;
  //MR (Mode Register) [R/W] [0x0000] [0x00]
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
	//The address value needs to be applied to the IEEE,According to the OUI regulations, the first 3 bytes are the vendor code, and the last three bytes are the product serial number.
	//If you define your own physical address, note that the first byte must be even
	Write_W5500_nByte(SHAR,Phy_Addr,6);		

	//Set the IP address of this machine,IP_ADDR is 4 bytes
	//Note that the gateway IP must belong to the same subnet as the local IP, otherwise the local machine will not be able to find the gateway.
	Write_W5500_nByte(SIPR,IP_Addr,4);		
	
	//Set the size of the send buffer and receive buffer,Refer to the W5500 data sheet. Initialization is configured to 2k for a total of 16k.
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
	Write_W5500_1Byte(IMR,IM_IR7 | IM_IR6);	//IP Conflict Interrupt Mask | Destination unreachable Interrupt Mask 
	Write_W5500_1Byte(SIMR,S0_IMR);			//socket Interrupt on S0
	Write_W5500_SOCK_1Byte(S0, Sn_IMR, IMR_SENDOK | IMR_RECV);	//S0 Interrupt SEND command is completed, whenever data is received from a peer.
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
	//Socket n Destination Port Register
	Write_W5500_SOCK_4Byte(S0,Sn_DIPR,ip_adde);//Write a different IP value to the destination address register than the local IP address
	Write_W5500_SOCK_1Byte(S0,Sn_MR,MR_TCP);   //Set the socket to TCP mode.
	Write_W5500_SOCK_1Byte(S0,Sn_CR,OPEN);     //Open Socket	
	delay_ms(5);//Delay 5ms 	
	
	if(Read_W5500_SOCK_1Byte(S0,Sn_SR) != SOCK_INIT)//If the socket fails to open, the TCP mode fails to start.
	{
		Write_W5500_SOCK_1Byte(S0,Sn_CR,CLOSE);//Open unsuccessful, close Socket
		return FALSE;//Return FALSE(0x00)
	}

	Write_W5500_SOCK_1Byte(S0,Sn_CR,CONNECT);//Set Socket to Connect mode TCP as client					

	do
	{
		u8 j=0;
		j=Read_W5500_SOCK_1Byte(S0,Sn_IR);//Read Socket0 interrupt flag register
		//If there is an interrupt prompt, write 1 clear corresponding interrupt
		if(j!=0) Write_W5500_SOCK_1Byte(S0,Sn_IR,j);
		delay_ms(5);
		if((j&IR_TIMEOUT) == IR_TIMEOUT) //time out
		{
			return FALSE;	
		}
		else if(Read_W5500_SOCK_1Byte(S0,Sn_DHAR) != 0xff)	 //Only judge whether the last MAC is FF. If it is successfully loaded, it should not be FF.
		{
			Write_W5500_SOCK_1Byte(S0,Sn_CR,CLOSE);//Close Socket Description Test successful
			return TRUE;							
		}
	}while(1);
}




//Specify Socket (0~7) initialization
//s:Port to be initialized
void Socket_Init(SOCKET s)
{
	//Set the fragment length, refer to the W5500 data sheet, the value can be modified.	
	//(Socket n Maximum Segment Size Register) [R/W] [0x0012-0x0013] [0x0000]
	/*This register is used for MSS (Maximum Segment Size) of TCP, and the register displays MSS set by the other party when TCP is activated in Passive Mode.
	*/
	//Write_W5500_SOCK_2Byte(S0, Sn_MSSR, 1460);//Maximum number of fragmented bytes =1460(0x5b4)
	//Set the specified port
	switch(s)
	{
		case 0:
			//Sn_PORT configures the source port number of Socket n.
			Write_W5500_SOCK_2Byte(0, Sn_PORT, (S0_Port[0]<<8)|S0_Port[1]);
		
			//Set port 0 destination (remote) port number
			//Write_W5500_SOCK_2Byte(0, Sn_DPORTR, S0_DPort[0]*256+S0_DPort[1]);
			//Set port 0 destination (remote) IP address
			//Write_W5500_SOCK_4Byte(0, Sn_DIPR, S0_DIP);			
			
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
// s: the port to be set
// Return value: success returns TRUE (0xFF), failure returns FALSE (0x00)
// Description: If the Socket works in UDP mode, refer to the program, in UDP mode, Socket communication does not need to establish a connection
// The program is called only once, setting W5500 to UDP mode.
// Should be add Sn's port to argument of this function
u8 Socket2UDP(SOCKET s)
{//phai sua lai cho nay nhe >>>>>>>>>>>day
	Write_W5500_SOCK_2Byte(s, Sn_PORT, (S0_Port[0]<<8)|S0_Port[1]); //It should be set before OPEN command is ordered.
	//Sn_MR configures the option or protocol type of Socket n.
	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_UDP); //Set Socket to UDP mode
	//(Socket n Command Register) [R/W] [0x0001] [0x00]
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//Socket n is initialized and opened
	delay_ms(5);
	//Check Socket n Status Register [R] [0x0003] [0x00]
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_UDP)//This indicates Socket n is opened in UDP mode
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//Open unsuccessful, close Socket
		return FALSE;//FALSE(0x00)
	}
	else
		return TRUE;

	// This completes the Socket open and UDP mode settings, in this mode it does not need to establish a connection with the remote host
	// Because Socket does not need to establish a connection, so you can set the destination host IP and destination Socket port number before sending data
	// If the destination host IP and the destination Socket port number is fixed, there is no change during the running, then you can also set here
}





void W5500_Interrupt_Process(void)
{
	u8 i,j;


	W5500_Interrupt=0;//Clear flag interrupt
	/*IR indicates the interrupt status. Each bit of IR can be cleared when the host writes
??value to each bit. If IR is not equal to ?x00? INTn PIN is asserted low until it is ?x00?*/
	i = Read_W5500_1Byte(IR);//read interrupt
	Write_W5500_1Byte(IR, (i&0xf0));//clear interrupt

	if((i & CONFLICT) == CONFLICT)
	{
		printf("INT: IP conflict\r\n");
	}

	if((i & UNREACH) == UNREACH)
	{
		printf("INT: IP UNREACH\r\n");
	}
ProcessSocket0Int:
	i=Read_W5500_1Byte(SIR);//read interrupt, (Interrupt Mask Register) [R/W][0x0016][0x00]	
	if((i & S0_INT) == S0_INT)//Socket0 Interrupt
	{
		j=Read_W5500_SOCK_1Byte(0,Sn_IR); //(Socket n Interrupt Register) [RCW1] [0x0002] [0x00]
		Write_W5500_SOCK_1Byte(0,Sn_IR,j);//clear interrupt

		if(j&IR_CON)////when the connection with peer is successful and then Sn_SR is changed to SOCK_ESTABLISHED.
		{
			S0_State|=S_CONN;
			printf("INT: SOCK_ESTABLISHED\r\n");
		}
		if(j&IR_DISCON)//when FIN or FIN/ACK packet is received from a peer.
		{
			Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);
			Socket_Init(0);	
			S0_State = S_Unknown;
			printf("INT: IR_DISCON\r\n");
		}
		if(j&IR_SEND_OK)//Socket0 SEND command is completed.
		{
			S0_Data|=S_TRANSMITOK;
			printf("INT: SEND OK\r\n");
		}
		if(j&IR_RECV)//whenever data is received from a peer.
		{
			S0_Data|=S_RECEIVE;
			printf("INT: data is received\r\n");
		}
		if(j&IR_TIMEOUT)//when ARP or TCP TIMEOUT occurs.
		{
			Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE); 
			S0_State = S_Unknown;
			printf("INT: TIMEOUT\r\n");
		}
	}

	if(Read_W5500_1Byte(SIR) != 0) 
		goto ProcessSocket0Int;
}

