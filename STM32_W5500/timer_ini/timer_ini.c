#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "timer_ini.h"
void timer4_ini(){//APB1 = 1 таймер тактируется от 8000000Гц 
    TIM_TimeBaseInitTypeDef TIMER_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_TimeBaseStructInit(&TIMER_InitStructure);
    TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	  TIMER_InitStructure.TIM_Prescaler = 36000;//36000000/36000=1000 рас в секунду
	  TIMER_InitStructure.TIM_Period = 2000;// рас в секунду будет срабатыать прерыание
    TIM_TimeBaseInit(TIM4, &TIMER_InitStructure);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
    /* NVIC Configuration */
    /* Enable the TIM4_IRQn Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	}
//void TIM4_IRQHandler(void)
//{
//        if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
//        {
//					  GPIOC->ODR ^= GPIO_Pin_13;
//            TIM_ClearITPendingBit(TIM4, TIM_IT_Update);// сбрасываем флаг
//					  DHCP_time_handler();
//					 counter++;
//					if(counter>9){
//					 counter = 0;
//					 if(flag)	send_tsp(SOCK_TCPS_MY_PORT,serverIP,SOCK_TCPS_SERVER_PORT,send_buf);	
//					} 
//        }
//}
