#ifndef	W5500_INI
#define	W5500_INI
// W5500 GPIO Define its SPI definition part in the SPI file
#define W5500_SCS		GPIO_Pin_0	//W5500 CS	 
#define W5500_SCS_GPIO	GPIOA
	
#define W5500_RST		GPIO_Pin_2	//W5500 RST
#define W5500_RST_GPIO	GPIOA


#define W5500_INT		GPIO_Pin_3	//W5500 INT
#define W5500_INT_GPIO	GPIOA

void SPI1_W5500_Init(void);
void W5500_GPIO_Init2(void);
void w5500_lib_init(void);

#endif
