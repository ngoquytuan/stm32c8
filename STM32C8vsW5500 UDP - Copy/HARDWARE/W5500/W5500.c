/**********************************************************************************
W5500����ģ�� ������

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







//W5500���˿ڳ�ʼ���������ж�ģʽ
//��Ӧ����˵����
//sck PA5  miso PA6  mosi PA7  rst PA2  int PA3  cs PA0
void W5500_GPIO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;	

 	EXTI_InitTypeDef EXTI_InitStructure;			//�ж�����	  ���ĸ��ж���  EXTI_Line0-15
													//ģʽ EXTI_Mode_Interrupt�ж�  EXTI_Mode_Event �¼�
													//������ʽ  EXTI_Trigger_Falling �½��ش���
													//			EXTI_Trigger_Rising	 �����ش���
													//			EXTI_Trigger_Rising_Falling	  �����ƽ����

 	NVIC_InitTypeDef NVIC_InitStructure;			//�жϲ��� �ж����ȼ�


  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//ʹ�ܸ��ù���ʱ��



	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE );//PORTBʱ��ʹ�� 


	// W5500_RST���ų�ʼ������(PA2) 
	GPIO_InitStructure.GPIO_Pin  = W5500_RST;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(W5500_RST_GPIO, &GPIO_InitStructure);
	GPIO_ResetBits(W5500_RST_GPIO, W5500_RST);
	
	// W5500_INT���ų�ʼ������(PA3) 	
	GPIO_InitStructure.GPIO_Pin = W5500_INT;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(W5500_INT_GPIO, &GPIO_InitStructure);


	// ��ʼ������ģ��SPI-CS���� (PA0)
	GPIO_InitStructure.GPIO_Pin = W5500_SCS;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(W5500_SCS_GPIO, &GPIO_InitStructure);
	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);



 	// Enable the EXTI3 Interrupt 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;				//W5500_INT���ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2�� 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;			//�����ȼ�2
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//ʹ���ⲿ�ж�ͨ��
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


//�ж���3  PA3��Ӧ W5500�������� ����һ��־λ
void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line3) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line3);	//���ж���
		W5500_Interrupt=1;
	}
}



//ͨ��SPI1����һ���ֽ�
//dat ���͵��ֽ�
//�޷���
void SPI1_Send_Byte(u8 dat)
{
	SPI_I2S_SendData(SPI1,dat);//д1���ֽ�����
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);//��ѭ���ȴ����ݼĴ�����
}



//PI1����2���ֽ�����(16λ)
//dat:�����͵�16λ����
void SPI1_Send_Short(u16 dat)
{
	SPI1_Send_Byte(dat>>8);//д���ݸ�λ   �൱��dat>>8
	SPI1_Send_Byte(dat);	//д���ݵ�λ
}




//ͨ��SPI1��W5500ָ����ַ�Ĵ���д1���ֽ�����
// reg:16λ�Ĵ�����ַ,dat:��д�������
void Write_W5500_1Byte(u16 reg, u8 dat)
{
	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//CSƬѡW5500

	SPI1_Send_Short(reg);   //ͨ��SPI1д16λ�Ĵ�����ַ
	SPI1_Send_Byte(FDM1|RWB_WRITE|COMMON_R); //ͨ��SPI1д�����ֽ�,1���ֽ����ݳ���,д����,ѡ��ͨ�üĴ���
	SPI1_Send_Byte(dat);  //д1���ֽ�����

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //����CS  ȡ��Ƭѡ
}




//ͨ��SPI1��ָ����ַ�Ĵ���д2���ֽ�����
//reg:16λ�Ĵ�����ַ,dat:16λ��д�������(2���ֽ�)
void Write_W5500_2Byte(u16 reg, u16 dat)
{
	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//CSƬѡW5500
		
	SPI1_Send_Short(reg);//ͨ��SPI1д16λ�Ĵ�����ַ
	SPI1_Send_Byte(FDM2|RWB_WRITE|COMMON_R);//ͨ��SPI1д�����ֽ�,2���ֽ����ݳ���,д����,ѡ��ͨ�üĴ���
	SPI1_Send_Short(dat);//д16λ����

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //����CS  ȡ��Ƭѡ
}




//ͨ��SPI1��ָ����ַ�Ĵ���дn���ֽ�����
//reg:16λ�Ĵ�����ַ,*dat_ptr:��д�����ݻ�����ָ��,size:��д������ݳ���
void Write_W5500_nByte(u16 reg, u8 *dat_ptr, u16 size)
{
	u16 i;

	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//CSƬѡW5500	
		
	SPI1_Send_Short(reg);//ͨ��SPI1д16λ�Ĵ�����ַ
	SPI1_Send_Byte(VDM|RWB_WRITE|COMMON_R);//ͨ��SPI1д�����ֽ�,N���ֽ����ݳ���,д����,ѡ��ͨ�üĴ���

	for(i=0;i<size;i++)//ѭ������������size���ֽ�����д��W5500
	{
		SPI1_Send_Byte(*dat_ptr++);//дһ���ֽ�����
	}

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //����CS  ȡ��Ƭѡ
}




//ͨ��SPI1��ָ���˿ڼĴ���д1���ֽ�����
//s:�˿ں�,reg:16λ�Ĵ�����ַ,dat:��д�������
void Write_W5500_SOCK_1Byte(SOCKET s, u16 reg, u8 dat)
{
	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//CSƬѡW5500	
		
	SPI1_Send_Short(reg);//ͨ��SPI1д16λ�Ĵ�����ַ
	SPI1_Send_Byte(FDM1|RWB_WRITE|(s*0x20+0x08));//ͨ��SPI1д�����ֽ�,1���ֽ����ݳ���,д����,ѡ��˿�s�ļĴ���
	SPI1_Send_Byte(dat);//д1���ֽ�����

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //����CS  ȡ��Ƭѡ
}



//ͨ��SPI1��ָ���˿ڼĴ���д2���ֽ�����
//s:�˿ں�,reg:16λ�Ĵ�����ַ,dat:16λ��д�������(2���ֽ�)
void Write_W5500_SOCK_2Byte(SOCKET s, u16 reg, u16 dat)
{
	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//CSƬѡW5500
			
	SPI1_Send_Short(reg);//ͨ��SPI1д16λ�Ĵ�����ַ
	SPI1_Send_Byte(FDM2|RWB_WRITE|(s*0x20+0x08));//ͨ��SPI1д�����ֽ�,2���ֽ����ݳ���,д����,ѡ��˿�s�ļĴ���
	SPI1_Send_Short(dat);//д16λ����

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //����CS  ȡ��Ƭѡ
}




//ͨ��SPI1��ָ���˿ڼĴ���д4���ֽ�����
//s:�˿ں�,reg:16λ�Ĵ�����ַ,*dat_ptr:��д���4���ֽڻ�����ָ��
void Write_W5500_SOCK_4Byte(SOCKET s, u16 reg, u8 *dat_ptr)
{
	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//����CS CSƬѡW5500
			
	SPI1_Send_Short(reg);//ͨ��SPI1д16λ�Ĵ�����ַ
	SPI1_Send_Byte(FDM4|RWB_WRITE|(s*0x20+0x08));//ͨ��SPI1д�����ֽ�,4���ֽ����ݳ���,д����,ѡ��˿�s�ļĴ���

	SPI1_Send_Byte(*dat_ptr++);//д��1���ֽ�����
	SPI1_Send_Byte(*dat_ptr++);//д��2���ֽ�����
	SPI1_Send_Byte(*dat_ptr++);//д��3���ֽ�����
	SPI1_Send_Byte(*dat_ptr++);//д��4���ֽ�����

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);//����CS  ȡ��Ƭѡ
}



//��ȡW5500ָ����ַ�Ĵ�����1���ֽ�����
//reg:16λ�Ĵ�����ַ
//����:��ȡ���Ĵ�����1���ֽ�����
u8 Read_W5500_1Byte(u16 reg)
{
	u8 i;

	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//����CS CSƬѡW5500
			
	SPI1_Send_Short(reg);  //ͨ��SPI1д16λ�Ĵ�����ַ
	SPI1_Send_Byte(FDM1|RWB_READ|COMMON_R);//ͨ��SPI1д�����ֽ�,1���ֽ����ݳ���,������,ѡ��ͨ�üĴ���

	i=SPI_I2S_ReceiveData(SPI1);
	SPI1_Send_Byte(0x00);//���Ϳ����� �ȴ���������
	i=SPI_I2S_ReceiveData(SPI1);//��ȡ1���ֽ�����

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);//����CS  ȡ��Ƭѡ
	return i;//���ض�ȡ���ļĴ�������
}



//��W5500ָ���˿ڼĴ�����1���ֽ�����
//s:�˿ں�,reg:16λ�Ĵ�����ַ
//����ֵ:��ȡ���Ĵ�����1���ֽ�����
u8 Read_W5500_SOCK_1Byte(SOCKET s, u16 reg)
{
	u8 i;

	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//����CS CSƬѡW5500
			
	SPI1_Send_Short(reg);   //ͨ��SPI1д16λ�Ĵ�����ַ
	SPI1_Send_Byte(FDM1|RWB_READ|(s*0x20+0x08));//ͨ��SPI1д�����ֽ�,1���ֽ����ݳ���,������,ѡ��˿�s�ļĴ���

	i=SPI_I2S_ReceiveData(SPI1);
	SPI1_Send_Byte(0x00);   //���Ϳ����� �ȴ���������
	i=SPI_I2S_ReceiveData(SPI1);//��ȡ1���ֽ�����

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);  //����CS  ȡ��Ƭѡ
	return i;//���ض�ȡ���ļĴ�������
}



//��W5500ָ���˿ڼĴ�����2���ֽ�����
//s:�˿ں�,reg:16λ�Ĵ�����ַ
//����ֵ:��ȡ���Ĵ�����2���ֽ�����(16λ)
u16 Read_W5500_SOCK_2Byte(SOCKET s, u16 reg)
{
	u16 i;

	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//����CS CSƬѡW5500
			
	SPI1_Send_Short(reg);//ͨ��SPI1д16λ�Ĵ�����ַ
	SPI1_Send_Byte(FDM2|RWB_READ|(s*0x20+0x08));//ͨ��SPI1д�����ֽ�,2���ֽ����ݳ���,������,ѡ��˿�s�ļĴ���

	i=SPI_I2S_ReceiveData(SPI1);
	SPI1_Send_Byte(0x00);//���Ϳ����� �ȴ���������
	i=SPI_I2S_ReceiveData(SPI1);//��ȡ��λ����
	SPI1_Send_Byte(0x00);//���Ϳ����� �ȴ���������
	i*=256;
	i+=SPI_I2S_ReceiveData(SPI1);//��ȡ��λ����

	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);//����CS  ȡ��Ƭѡ
	return i;//���ض�ȡ���ļĴ�������
}




//��W5500�������ݻ������ж�ȡ����
//s:�˿ں�,*dat_ptr:���ݱ��滺����ָ��
//����ֵ:��ȡ�������ݳ���,rx_size���ֽ�
u16 Read_SOCK_Data_Buffer(SOCKET s, u8 *dat_ptr)
{
	u16 rx_size;
	u16 offset, offset1;
	u16 i;
	u8 j;

	rx_size=Read_W5500_SOCK_2Byte(s,Sn_RX_RSR);
	if(rx_size==0) return 0;  //û���յ������򷵻�
	if(rx_size>1460) rx_size=1460;

	offset=Read_W5500_SOCK_2Byte(s,Sn_RX_RD);
	offset1=offset;
	offset&=(S_RX_SIZE-1);    //����ʵ�ʵ������ַ

	GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//����CS CSƬѡW5500

	SPI1_Send_Short(offset);//д16λ��ַ
	SPI1_Send_Byte(VDM|RWB_READ|(s*0x20+0x18));//д�����ֽ�,N���ֽ����ݳ���,������,ѡ��˿�s�ļĴ���
	j=SPI_I2S_ReceiveData(SPI1);
	
	if((offset+rx_size)<S_RX_SIZE)//�������ַδ����W5500���ջ������Ĵ���������ַ
	{
		for(i=0;i<rx_size;i++)//ѭ����ȡrx_size���ֽ�����
		{
			SPI1_Send_Byte(0x00);//����һ��������
			j=SPI_I2S_ReceiveData(SPI1);//��ȡ1���ֽ�����
			*dat_ptr=j;//����ȡ�������ݱ��浽���ݱ��滺����
			dat_ptr++;//���ݱ��滺����ָ���ַ����1
		}
	}
	else//�������ַ����W5500���ջ������Ĵ���������ַ
	{
		offset=S_RX_SIZE-offset;
		for(i=0;i<offset;i++)//ѭ����ȡ��ǰoffset���ֽ�����
		{
			SPI1_Send_Byte(0x00);//����һ��������
			j=SPI_I2S_ReceiveData(SPI1);//��ȡ1���ֽ�����
			*dat_ptr=j;//����ȡ�������ݱ��浽���ݱ��滺����
			dat_ptr++;//���ݱ��滺����ָ���ַ����1
		}
		GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS);  //����CS  ȡ��Ƭѡ

		GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS);//����CS CSƬѡW5500

		SPI1_Send_Short(0x00);//д16λ��ַ
		SPI1_Send_Byte(VDM|RWB_READ|(s*0x20+0x18));//д�����ֽ�,N���ֽ����ݳ���,������,ѡ��˿�s�ļĴ���
		j=SPI_I2S_ReceiveData(SPI1);

		for(;i<rx_size;i++)//ѭ����ȡ��rx_size-offset���ֽ�����
		{
			SPI1_Send_Byte(0x00);//����һ��������
			j=SPI_I2S_ReceiveData(SPI1);//��ȡ1���ֽ�����
			*dat_ptr=j;//����ȡ�������ݱ��浽���ݱ��滺����
			dat_ptr++;//���ݱ��滺����ָ���ַ����1
		}
	}
	GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS); //����CS  ȡ��Ƭѡ

	offset1+=rx_size;//����ʵ�������ַ,���´ζ�ȡ���յ������ݵ���ʼ��ַ
	Write_W5500_SOCK_2Byte(s, Sn_RX_RD, offset1);
	Write_W5500_SOCK_1Byte(s, Sn_CR, RECV);//����������������
	return rx_size;//���ؽ��յ����ݵĳ���
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
	When the Host writes 5 Bytes Data (0x11, 0x22, 0x33, 0x44, 0x55) to Socket 1�s TX
    Buffer Block 0x0040 Address by using VDM mode, 5 bytes data are written with the SPI Frame below.*/
    //Write nBytes Data at Socketn�s TX Buffer Block 0xXXXX in VDM mode
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
    //KO hi?u trong TCP mode th�� th? n��o?
	//If it is UDP mode, you can set the IP and port number of the destination host here.
	if((Read_W5500_SOCK_1Byte(s,Sn_MR)&0x0f) == MR_UDP)//If it is UDP mode, update the received host address and port to the module port.
	{													
		Write_W5500_SOCK_4Byte(s, Sn_DIPR, UDP_DIPR);//Set the destination IP  		
		Write_W5500_SOCK_2Byte(s, Sn_DPORTR, (UDP_DPORT[0]<<8)|+UDP_DPORT[1]);//Set the destination port				
	}
    
	//Sn_TX_WR :Socket n TX Write Pointer Register [R/W] [0x0024-0x0025] [0x0000]
	offset=Read_W5500_SOCK_2Byte(s,Sn_TX_WR);//offset c�� ??a ch? th?ng s0 Tx
	offset1=offset;// L?u ??a ch? n��y sang th?ng offset1???
	offset&=(S_TX_SIZE-1);//Calculate the actual physical address???? offset = offset & 0x7ff => ??m b?o ??a ch? ko bay ra ngo��i khu 16k

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



//W5500Ӳ����λ
//˵����W5500�ĸ�λ���ű��ֵ͵�ƽ����500us����,����ʹW5500���븴λ״̬
void W5500_Hardware_Reset(void)
{
	GPIO_ResetBits(W5500_RST_GPIO, W5500_RST);  //��λ��������
	delay_ms(50);
	GPIO_SetBits(W5500_RST_GPIO, W5500_RST);    //��λ��������
	delay_ms(200);
	while((Read_W5500_1Byte(PHYCFGR)&LINK)==0); //��ѭ���ȴ���̫���������	����Ҫ���������ӵ�·�ɺ�ģ���� �����޷�ͨ��
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



//����ָ��Socket(0~7)Ϊ�ͻ�����Զ�̷���������
//s:���趨�Ķ˿�
//����ֵ:�ɹ�����TRUE(0xFF),ʧ�ܷ���FALSE(0x00)
//˵��  :������Socket�����ڿͻ���ģʽʱ,���øó���,��Զ�̷�������������
//	     ����������Ӻ���ֳ�ʱ�жϣ��������������ʧ��,��Ҫ���µ��øó�������
//	     �ó���ÿ����һ��,�������������һ������
u8 Socket_Connect(SOCKET s)
{
	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_TCP);//����socketΪTCPģʽ
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//��Socket
	delay_ms(5);//��ʱ5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_INIT)//���socket��ʧ��
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//�򿪲��ɹ�,�ر�Socket
		return FALSE;//����FALSE(0x00)
	}
	Write_W5500_SOCK_1Byte(s,Sn_CR,CONNECT);//����SocketΪConnectģʽ
	return TRUE;//����TRUE,���óɹ�
}




//����ָ��Socket(0~7)��Ϊ�������ȴ�Զ������������
//s:���趨�Ķ˿�
//����ֵ:�ɹ�����TRUE(0xFF),ʧ�ܷ���FALSE(0x00)
//˵��  :������Socket�����ڷ�����ģʽʱ,���øó���,�ȵ�Զ������������
//		 �ó���ֻ����һ��,��ʹW5500����Ϊ������ģʽ
u8 Socket_Listen(SOCKET s)
{
	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_TCP);//����socketΪTCPģʽ 
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//��Socket	
	delay_ms(5);//��ʱ5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_INIT)//���socket��ʧ��
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//�򿪲��ɹ�,�ر�Socket
		return FALSE;//����FALSE(0x00)
	}	
	Write_W5500_SOCK_1Byte(s,Sn_CR,LISTEN);//����SocketΪ����ģʽ	
	delay_ms(5);//��ʱ5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_LISTEN)//���socket����ʧ��
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//���ò��ɹ�,�ر�Socket
		return FALSE;//����FALSE(0x00)
	}

	return TRUE;

	//���������Socket�Ĵ򿪺�������������,����Զ�̿ͻ����Ƿ�������������,����Ҫ�ȴ�Socket�жϣ�
	//���ж�Socket�������Ƿ�ɹ����ο�W5500�����ֲ��Socket�ж�״̬
	//�ڷ���������ģʽ����Ҫ����Ŀ��IP��Ŀ�Ķ˿ں�
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

