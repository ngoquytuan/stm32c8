

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

/*ʹ��microLib�ķ���*/
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
//bit15��	Receive completion flag
//bit14��	received 0x0d
//bit13~0��	Number of valid bytes received 
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
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);  //��ʼ��PA10


   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;                //���������� һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;     //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;        //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	   //�շ�ģʽ   ����������ý��� �����

    USART_Init(USART1, &USART_InitStructure); //Initialize the serial port

#if EN_USART1_RX		  //���ʹ���˽���  
   //Usart1 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
   
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�����ж�	 ���յ����ݽ����ж�
#endif

    USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ��� 

}


//���� ������һ��16λ����USART_RX_STA ����ʾ �ɼ������ݳ���  ����״̬�� �൱��һ���Ĵ���
//USART_RX_STA     15		    14	         13-0
//				 �������	���յ�0x0d	  ���յ����ݳ���  û���ռ�1 ��ʾ����һ���ֽ�
//USART_RX_STA=0 ��Ϊ����������׼��

//���ڽ����жϵ�ǰ���� ���ݵ�����Իس�Ϊ׼  ��  0x0d 0x0a  

void USART1_IRQHandler(void)                	//����1�ж���Ӧ����		 �����ֲ�����㶨��
	{
	u8 Res;													//�����ڽ��յ�����  RXNE������1 
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
		{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//��ȡ���յ�������
		
		if((USART_RX_STA&0x8000)==0)//����δ���
			{
			if(USART_RX_STA&0x4000)//���յ���0x0d
				{
				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else USART_RX_STA|=0x8000;	//��������� 			  //���յ��س��ĺ��ֽ�  ��λ״̬�Ĵ��� 
				}
			else //��û�յ�0X0D
				{	
				if(Res==0x0d)USART_RX_STA|=0x4000;					 //���յ��س���ǰһ�ֽ�  ��λ״̬�Ĵ���
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;			//�����յ����� ����������
					USART_RX_STA++;									//����+1 Ϊ��һ����׼��
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
					}		 
				}
			}   		 
     } 

} 
#endif	















































