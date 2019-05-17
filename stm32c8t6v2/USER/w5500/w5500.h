#ifndef	_W5500_H_
#define	_W5500_H_



//The following are the internal register addresses of the W5500.
//The details are detailed in the corresponding manual.
/***************** Common Register *****************/
#define MR		0x0000
	#define RST		0x80
	#define WOL		0x20
	#define PB		0x10
	#define PPP		0x08
	#define FARP	0x02

#define GAR		0x0001
#define SUBR	0x0005
#define SHAR	0x0009
#define SIPR	0x000f

#define INTLEVEL	0x0013
#define IR		0x0015
	#define CONFLICT	0x80
	#define UNREACH		0x40
	#define PPPOE		0x20
	#define MP			0x10

#define IMR		0x0016
	#define IM_IR7		0x80
	#define IM_IR6		0x40
	#define IM_IR5		0x20
	#define IM_IR4		0x10

#define SIR		0x0017
	#define S7_INT		0x80
	#define S6_INT		0x40
	#define S5_INT		0x20
	#define S4_INT		0x10
	#define S3_INT		0x08
	#define S2_INT		0x04
	#define S1_INT		0x02
	#define S0_INT		0x01

#define SIMR	0x0018
	#define S7_IMR		0x80
	#define S6_IMR		0x40
	#define S5_IMR		0x20
	#define S4_IMR		0x10
	#define S3_IMR		0x08
	#define S2_IMR		0x04
	#define S1_IMR		0x02
	#define S0_IMR		0x01

#define RTR		0x0019
#define RCR		0x001b

#define PTIMER	0x001c
#define PMAGIC	0x001d
#define PHA		0x001e
#define PSID	0x0024
#define PMRU	0x0026

#define UIPR	0x0028
#define UPORT	0x002c

#define PHYCFGR	0x002e
	#define RST_PHY		0x80
	#define OPMODE		0x40
	#define DPX			0x04
	#define SPD			0x02
	#define LINK		0x01

#define VERR	0x0039

/********************* Socket Register *******************/
#define Sn_MR		0x0000
	#define MULTI_MFEN		0x80
	#define BCASTB			0x40
	#define	ND_MC_MMB		0x20
	#define UCASTB_MIP6B	0x10
	#define MR_CLOSE		0x00
	#define MR_TCP		0x01
	#define MR_UDP		0x02
	#define MR_MACRAW		0x04

#define Sn_CR		0x0001
	#define OPEN		0x01
	#define LISTEN		0x02
	#define CONNECT		0x04
	#define DISCON		0x08
	#define CLOSE		0x10
	#define SEND		0x20
	#define SEND_MAC	0x21
	#define SEND_KEEP	0x22
	#define RECV		0x40

#define Sn_IR		0x0002
	#define IR_SEND_OK		0x10
	#define IR_TIMEOUT		0x08
	#define IR_RECV			0x04
	#define IR_DISCON		0x02
	#define IR_CON			0x01

#define Sn_SR		0x0003
	#define SOCK_CLOSED		0x00
	#define SOCK_INIT		0x13
	#define SOCK_LISTEN		0x14
	#define SOCK_ESTABLISHED	0x17
	#define SOCK_CLOSE_WAIT		0x1c
	#define SOCK_UDP		0x22
	#define SOCK_MACRAW		0x02

	#define SOCK_SYNSEND	0x15
	#define SOCK_SYNRECV	0x16
	#define SOCK_FIN_WAI	0x18
	#define SOCK_CLOSING	0x1a
	#define SOCK_TIME_WAIT	0x1b
	#define SOCK_LAST_ACK	0x1d

#define Sn_PORT		0x0004
#define Sn_DHAR	   	0x0006
#define Sn_DIPR		0x000c
#define Sn_DPORTR	0x0010

#define Sn_MSSR		0x0012
#define Sn_TOS		0x0015
#define Sn_TTL		0x0016

#define Sn_RXBUF_SIZE	0x001e
#define Sn_TXBUF_SIZE	0x001f
#define Sn_TX_FSR	0x0020
#define Sn_TX_RD	0x0022
#define Sn_TX_WR	0x0024
#define Sn_RX_RSR	0x0026
#define Sn_RX_RD	0x0028
#define Sn_RX_WR	0x002a

#define Sn_IMR		0x002c
	#define IMR_SENDOK	0x10
	#define IMR_TIMEOUT	0x08
	#define IMR_RECV	0x04
	#define IMR_DISCON	0x02
	#define IMR_CON		0x01

#define Sn_FRAG		0x002d
#define Sn_KPALVTR	0x002f

/*******************************************************************/
/************************ SPI Control Byte *************************/
/*******************************************************************/
/* Operation mode bits */
#define VDM		0x00		  //Variable data length mode
#define FDM1	0x01		  //Fixed 1-byte data length mode
#define	FDM2	0x02		  //Fixed 2 bytes
#define FDM4	0x03		  //Fixed 4 bytes

/* Read_Write control bit */
#define RWB_READ	0x00
#define RWB_WRITE	0x04

/* Block select bits */
#define COMMON_R	0x00

/* Socket 0 */
#define S0_REG		0x08
#define S0_TX_BUF	0x10
#define S0_RX_BUF	0x18

/* Socket 1 */
#define S1_REG		0x28
#define S1_TX_BUF	0x30
#define S1_RX_BUF	0x38

/* Socket 2 */
#define S2_REG		0x48
#define S2_TX_BUF	0x50
#define S2_RX_BUF	0x58

/* Socket 3 */
#define S3_REG		0x68
#define S3_TX_BUF	0x70
#define S3_RX_BUF	0x78

/* Socket 4 */
#define S4_REG		0x88
#define S4_TX_BUF	0x90
#define S4_RX_BUF	0x98

/* Socket 5 */
#define S5_REG		0xa8
#define S5_TX_BUF	0xb0
#define S5_RX_BUF	0xb8

/* Socket 6 */
#define S6_REG		0xc8
#define S6_TX_BUF	0xd0
#define S6_RX_BUF	0xd8

/* Socket 7 */
#define S7_REG		0xe8
#define S7_TX_BUF	0xf0
#define S7_RX_BUF	0xf8

#define TRUE	0xff
#define FALSE	0x00

#define S_RX_SIZE	2048	/*Define the size of the Socket receive buffer, which can be modified according to the settings of W5500_RMSR*/
#define S_TX_SIZE	2048  	/*Define the size of the Socket send buffer, which can be modified according to the settings of W5500_TMSR. */




// W5500 GPIO Define its SPI definition part in the SPI file
#define W5500_SCS		GPIO_Pin_0	//W5500 CS	 
#define W5500_SCS_GPIO	GPIOA
	
#define W5500_RST		GPIO_Pin_2	//W5500 RST
#define W5500_RST_GPIO	GPIOA


#define W5500_INT		GPIO_Pin_3	//W5500 INT
#define W5500_INT_GPIO	GPIOA





// Network parameter variable definition
extern unsigned char Gateway_IP[4];//Gateway IP address 
extern unsigned char Sub_Mask[4];	//子网掩码 
extern unsigned char Phy_Addr[6];	//物理地址(MAC) 
extern unsigned char IP_Addr[4];	//本机IP地址 

extern unsigned char S0_Port[2];	//Port number of port 0(5000) 
extern unsigned char S0_DIP[4];	//Port 0 destination IP address 
extern unsigned char S0_DPort[2];	//Port 0 destination port number(6000) 

extern unsigned char UDP_DIPR[4];	//UDP (broadcast) mode, destination host IP address
extern unsigned char UDP_DPORT[2];	//UDP (broadcast) mode, destination host port number


// Port operating mode 
extern unsigned char S0_Mode;	//Port 0 operating mode, 0: TCP server mode, 1: TCP client mode, 2: UDP (broadcast) mode
#define TCP_SERVER		0x00	//TCP server mode
#define TCP_CLIENT		0x01	//TCP client mode 
#define UDP_MODE		0x02	//UDP (broadcast) mode

//Port running status 
extern unsigned char S0_State;	//Port 0 status record, 1: port complete initialization, 2 ports complete connection (can transfer data normally) 
#define S_INIT			0x01	//Port completion initialization 
#define S_CONN			0x02	//Port complete connection,Can transfer data normally 

// Status of the port to send and receive data 
extern u8 S0_Data;		//Port 0 receives and sends data status, 1: port receives data, 2: port sends data completed 
#define S_RECEIVE		0x01		//The port received a packet 
#define S_TRANSMITOK	0x02		//The port sends a packet to complete 

// Port data buffer 
extern u8 Rx_Buffer[2048];	//Port receive data buffer 
extern u8 Tx_Buffer[2048];	//Port send data buffer 

extern u8 W5500_Interrupt;	//W5500 interrupt flag (0: no interrupt, 1: interrupt)
typedef u8 SOCKET;			//Custom port number data type


extern void W5500_GPIO_Init(void);   //W5500 GPIO initialization configuration
extern void W5500_NVIC_Configuration(void);   //W5500 Receive pin interrupt priority setting
extern void SPI_Configuration(void);          //W5500 SPI initialization configuration(STM32 SPI1)
extern void W5500_Hardware_Reset(void);       //Hardware reset W5500
extern void W5500_Init(void);                 //Initialize the W5500 register function
extern u8 Detect_Gateway(void);    //Check the gateway server
extern void Socket_Init(SOCKET s);            //Specify Socket(0~7)initialization
extern u8 Socket_Connect(SOCKET s);//Set the specified Socket(0~7)Connect the client to the remote server
extern u8 Socket_Listen(SOCKET s); //Set the specified Socket(0~7)Waiting for a connection to the remote host as a server
extern u8 Socket_UDP(SOCKET s);    //Set the specified Socket(0~7)UDP mode
extern u16 Read_SOCK_Data_Buffer(SOCKET s, u8 *dat_ptr);          //Specify Socket(0~7)Receive data processing
extern void Write_SOCK_Data_Buffer(SOCKET s, u8 *dat_ptr, u16 size); //Specify Socket(0~7)Send data processing
extern void W5500_Interrupt_Process(void);    //W5500 interrupt handler framework

#endif

