/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FUC_H
#define __FUC_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void WWDG_Init(void);
void IWDG_configuration(void);
void test_eeprom(void);
void sw_eeprom_stm32(void);
void GPIO_config(void);
void tasks(void);
void hardware_init(void);

void PHYStatus_Check(void);
int32_t loopback_tcps(uint8_t, uint8_t*, uint16_t);		// Loopback TCP server
int32_t loopback_udps(uint8_t, uint8_t*, uint16_t);		// Loopback UDP server

void loadwebpages();

int32_t NTPUDP(uint8_t sn);//UDP NTP server
void ntpserverdefaultconfig(void);
uint32_t getns(void);
void resetns(void);

void wzn_event_handle(void);
#define USE_UART1

#endif /* __FUC_H */

