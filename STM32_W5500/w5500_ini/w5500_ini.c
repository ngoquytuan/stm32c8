#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"
#include "wizchip_conf.h"
#include "w5500_ini.h"
//===========================================================================//
void chip_on(void){
  GPIO_ResetBits(GPIOA, GPIO_Pin_4); 
}
//===========================================================================//
void chip_off(void){
  GPIO_SetBits(GPIOA, GPIO_Pin_4); 
}
//===========================================================================//
/* STM32 в 72 MHz. */
void clock_setup(void) {
    ErrorStatus HSEStartUpStatus;
    RCC_HSEConfig( RCC_HSE_ON);//Turn on the external quartz
    if (HSEStartUpStatus == SUCCESS)
    {   RCC_HCLKConfig( RCC_SYSCLK_Div1);
        RCC_PCLK2Config( RCC_HCLK_Div1);
        RCC_PCLK1Config( RCC_HCLK_Div2);//36
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);//PLLCLK = 8MHz * 9 = 72 MHz
        RCC_PLLCmd( ENABLE); //PLL on
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){}
        RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK); //Configure the system clock 
        while (RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }  
}
//============================================================================//
uint8_t SPI_SendReceiveByte(void)
{
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI1, 0xFF);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET) ;
  return SPI_I2S_ReceiveData(SPI1);
}
//===========================================================================//
void senddata(uint8_t data){	
while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
SPI_I2S_SendData(SPI1, data);
while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET) ;
SPI_I2S_ReceiveData(SPI1);	
	}
//===========================================================================//
  void spi_ini(void){
GPIO_InitTypeDef GPIO_InitStructure;
SPI_InitTypeDef SPI_InitStructure;
RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_4);//cs
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOA , &GPIO_InitStructure);
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ; //miso
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
GPIO_Init(GPIOA, &GPIO_InitStructure);
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7; //SPI sck/mosi
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
GPIO_Init(GPIOA, &GPIO_InitStructure);
//Заполняем структуру с параметрами SPI модуля
SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //полный дуплекс
SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; // передаем по 8 бит
SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; // Полярность и
SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; // фаза тактового сигнала
SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; // Управлять состоянием сигнала NSS аппаратно
SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; // Предделитель SCK
SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; // Первым отправляется старший бит
SPI_InitStructure.SPI_CRCPolynomial = 0x7;
SPI_InitStructure.SPI_Mode = SPI_Mode_Master; // Режим - 
SPI_Init(SPI1, &SPI_InitStructure); //Настраиваем SPI1
//SPI_I2S_ITConfig(SPI1,SPI_I2S_IT_RXNE,ENABLE); //Включаем прерывание по приему байта
SPI_Cmd(SPI1, ENABLE); // Включаем модуль SPI1....
SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);
//NVIC_EnableIRQ(SPI1_IRQn); //Разрешаем прерывания от SPI1
	}

//===========================================================================//
void w5500_ini(void){
  uint8_t temp;
  uint8_t W5500FifoSize[2][8] = {{2, 2, 2, 2, 2, 2, 2, 2, }, {2, 2, 2, 2, 2, 2, 2, 2}};
	clock_setup();
  spi_ini();
  chip_off();	
 /*передаем функции чтения записи драйверу */
  reg_wizchip_spi_cbfunc(SPI_SendReceiveByte, senddata);

  /* CS function register */
   reg_wizchip_cs_cbfunc(chip_on,chip_off);

  if (ctlwizchip(CW_INIT_WIZCHIP, (void*)W5500FifoSize) == -1){
//    printf("W5500 initialized fail.\r\n");
      while(1);
  }
//    check phy status
  do
  {
		//GPIOC->ODR ^= GPIO_Pin_13;
		
    if (ctlwizchip(CW_GET_PHYLINK, (void*)&temp) == -1)
    {
       // GPIOC->ODR ^= GPIO_Pin_13;    
//			printf("Unknown PHY link status.\r\n");
    }
  } while (temp == PHY_LINK_OFF);
}
