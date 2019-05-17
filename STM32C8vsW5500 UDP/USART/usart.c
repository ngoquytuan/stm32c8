

/**********************************************************************************
 * Serial port 1 configuration
 * Serial port 1 PA9 PA10 configuration and interrupt function
**********************************************************************************/

#include "usart.h"


//////////////////////////////////////////////////////////////////
//Add the following code to support the printf function without having to choose use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//Support functions required by the standard library                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//define _sys_exit() to avoid using semi-host mode  
_sys_exit(int x) 
{ 
	x = x; 
} 
//Redefine the fputc function
int fputc(int ch, FILE *f)
{      
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); 
    USART_SendData(USART1,(uint8_t)ch);   
	return ch;
}
#endif 

/*使用microLib的方法*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/



#if EN_USART1_RX   //If receiving is enabled
//Serial port 1 interrupt service routine
//Note that reading USARTx->SR can avoid inexplicable errors.   	
u8 USART_RX_BUF[USART_REC_LEN];     //Receive buffer, maximum USART_REC_LEN bytes.
//Receiving status
//bit15，	Receive completion flag
//bit14，	received 0x0d
//bit13~0，	Number of valid bytes received 
u16 USART_RX_STA=0;       //Receive status tag	  

//initialization IO Serial port 1 
//bound:Baud rate
void uart_init(u32 bound){
    //GPIO Port setting
    GPIO_InitTypeDef GPIO_InitStructure;				   //IO Port configuration structure
	USART_InitTypeDef USART_InitStructure;				   //Serial port configuration structure
	NVIC_InitTypeDef NVIC_InitStructure;				   //Interrupt configuration structure
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//Enable USART1, GPIOA clock
 	USART_DeInit(USART1);          //Resetting Serial Port 1 It is best to reset the peripherals before turning on the peripherals.
	 //USART1_TX   PA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//Multiplexed push-pull output PA.9 selects the multiplexing state to enter the serial port mode.
    GPIO_Init(GPIOA, &GPIO_InitStructure); //Initialize PA9
   
    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA10


   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;                //波特率设置 一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;     //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;        //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	   //收发模式   这里可以配置仅发 或仅收

    USART_Init(USART1, &USART_InitStructure); //Initialize the serial port

#if EN_USART1_RX		  //如果使能了接收  
   //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
   
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断	 接收到数据进入中断
#endif

    USART_Cmd(USART1, ENABLE);                    //使能串口 

}


//这里 定义了一个16位数据USART_RX_STA 来表示 采集的数据长度  数据状态等 相当于一个寄存器
//USART_RX_STA     15		    14	         13-0
//				 接收完成	接收到0x0d	  接收的数据长度  没接收加1 表示多了一个字节
//USART_RX_STA=0 则为接收数据做准备

//串口进入中断的前提是 数据的最后以回车为准  即  0x0d 0x0a  

void USART1_IRQHandler(void)                	//串口1中断响应程序		 其名字不能随便定义
	{
	u8 Res;													//当串口接收到数据  RXNE将被置1 
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
			{
			if(USART_RX_STA&0x4000)//接收到了0x0d
				{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 			  //接收到回车的后字节  置位状态寄存器 
				}
			else //还没收到0X0D
				{	
				if(Res==0x0d)USART_RX_STA|=0x4000;					 //接收到回车的前一字节  置位状态寄存器
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;			//将接收的数据 存入数组中
					USART_RX_STA++;									//长度+1 为下一次做准备
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
					}		 
				}
			}   		 
     } 

} 
#endif	















































