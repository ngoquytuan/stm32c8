#ifndef	_W5500_H_
#define	_W5500_H_

/*Sockets*/
#define S0 0
/***************** Registers *****************/

#define MR		0x0000//MR (Mode Register) [R/W] [0x0000] [0x00]
//MR is used for S/W reset, ping block mode and PPPoE mode.
	#define RST		0x80 //If this bit is ‘1’, All internal registers will be initialized. It will be automatically cleared as ‘0’ after S/W reset
	#define WOL		0x20
	#define PB		0x10
	#define PPP		0x08
	#define FARP	0x02

#define GAR		0x0001//GAR (Gateway IP Address Register) [R/W] [0x0001 – 0x0004] [0x00]
#define SUBR	0x0005//SUBR (Subnet Mask Register) [R/W] [0x0005 – 0x0008] [0x00]
#define SHAR	0x0009//SHAR (Source Hardware Address Register) [R/W] [0x0009 – 0x000E] [0x00]
#define SIPR	0x000f//SIPR (Source IP Address Register) [R/W] [0x000F – 0x0012] [0x00]

#define INTLEVEL	0x0013//Interrupt Low Level Timer Register [R/W] [0x0013 – 0x0014] [0x0000]
#define IR		0x0015 //IR (Interrupt Register) [R/W] [0x0015] [0x00]
/*
IR indicates the interrupt status. Each bit of IR can be cleared when the host writes
‘1’ value to each bit. If IR is not equal to ‘0x00’, INTn PIN is asserted low until it is
‘0x00’.
*/
	#define CONFLICT	0x80 //Bit is set as ‘1’ when own source IP address is same with the sender IP address in the received ARP request.
	#define UNREACH		0x40
	#define PPPOE		0x20
	#define MP			0x10

#define IMR		0x0016 //IMR 
/*IMR is used to mask interrupts. Each bit of IMR corresponds to each bit of IR. When
a bit of IMR is ‘1’ and the corresponding bit of IR is ‘1’, an interrupt will be issued. In
other words, if a bit of IMR is ‘0’, an interrupt will not be issued even if the
corresponding bit of IR is ‘1’. */
	#define IM_IR7		0x80 //IP Conflict Interrupt Mask
	#define IM_IR6		0x40 //Destination unreachable Interrupt Mask
	#define IM_IR5		0x20 //PPPoE Close Interrupt Mask
	#define IM_IR4		0x10 //Magic Packet Interrupt Mask

#define SIR		0x0017 //(Socket Interrupt Register) [R/W] [0x0017] [0x00]
//When the interrupt of Socket n occurs, the n-th bit of SIR becomes ‘1’.
	#define S7_INT		0x80
	#define S6_INT		0x40
	#define S5_INT		0x20
	#define S4_INT		0x10
	#define S3_INT		0x08
	#define S2_INT		0x04
	#define S1_INT		0x02
	#define S0_INT		0x01

#define SIMR	0x0018 //(Socket Interrupt Mask Register) [R/W] [0x0018] [0x00]
/*
Each bit of SIMR corresponds to each bit of SIR. When a bit of SIMR is ‘1’ and the
corresponding bit of SIR is ‘1’, Interrupt will be issued. In other words, if a bit of SIMR
is ‘0’, an interrupt will be not issued even if the corresponding bit of SIR is ‘1’.
*/
	#define S7_IMR		0x80
	#define S6_IMR		0x40
	#define S5_IMR		0x20
	#define S4_IMR		0x10
	#define S3_IMR		0x08
	#define S2_IMR		0x04
	#define S1_IMR		0x02
	#define S0_IMR		0x01

#define RTR		0x0019 //(Retry Time-value Register) [R/W] [0x0019 – 0x001A] [0x07D0]
/*
RTR configures the retransmission timeout period. The unit of timeout period is
100us and the default of RTR is ‘0x07D0’ or ‘2000’. And so the default timeout period
is 200ms(100us X 2000).*/
#define RCR		0x001b //(Retry Count Register) [R/W] [0x001B] [0x08]
/*RCR configures the number of time of retransmission. When retransmission occurs
as many as ‘RCR+1’, Timeout interrupt is issued (Sn_IR[TIMEOUT] = ‘1’).*/

#define PTIMER	0x001c
#define PMAGIC	0x001d
#define PHA		0x001e
#define PSID	0x0024
#define PMRU	0x0026

#define UIPR	0x0028
#define UPORT	0x002c

#define PHYCFGR	0x002E //(W5500 PHY Configuration Register) [R/W] [0x002E] [0b10111XXX]
/*PHYCFGR configures PHY operation mode and resets PHY. In addition, PHYCFGR
indicates the status of PHY such as duplex, Speed, Link.*/
	#define RST_PHY		0x80 //When this bit is ‘0’, internal PHY is reset. After PHY reset, it should be set as ‘1’.
	#define OPMODE		0x40 //Operation Mode Configuration Bit[R/W]
	#define DPX			0x04
	#define SPD			0x02
	#define LINK		0x01 //Link Status [Read Only],1: Link up

#define VERR	0x0039 //(W5500 Chip Version Register) [R] [0x0039] [0x04]

/********************* Socket Registers *******************/
#define Sn_MR		0x0000 //(Socket n Mode Register) [R/W] [0x0000] [0x00]
	#define MULTI_MFEN		0x80
	#define BCASTB			0x40
	#define	ND_MC_MMB		0x20
	#define UCASTB_MIP6B	0x10
	#define MR_CLOSE		0x00
	#define MR_TCP		0x01
	#define MR_UDP		0x02
	#define MR_MACRAW		0x04

#define Sn_CR		0x0001 //(Socket n Command Register) [R/W] [0x0001] [0x00]
/* This is used to set the command for Socket n such as OPEN, CLOSE, CONNECT, LISTEN,
SEND, and RECEIVE. After W5500 accepts the command, the Sn_CR register is
automatically cleared to 0x00. Even though Sn_CR is cleared to 0x00, the command is
still being processed. To check whether the command is completed or not, please
check the Sn_IR or Sn_SR.*/
	#define OPEN		  0x01 //Socket n is initialized and opened
	#define LISTEN		0x02 //This is valid only in TCP mode
	#define CONNECT		0x04 //This is valid only in TCP mode and operates when Socket n acts as ‘TCP client’.
	#define DISCON		0x08 //Valid only in TCP mode.
	#define CLOSE		  0x10 //Close Socket n.
	#define SEND		  0x20 //SEND transmits all the data in the Socket n TX buffer.
	#define SEND_MAC	0x21
	#define SEND_KEEP	0x22 //Valid only in TCP mode.
	#define RECV		  0x40 //RECV completes the processing of the received data in Socket n RX Buffer by using a RX read pointer register (Sn_RX_RD).

#define Sn_IR		0x0002 //(Socket n Interrupt Register) [RCW1] [0x0002] [0x00]
/*Sn_IR indicates the status of Socket Interrupt such as establishment, termination,
receiving data, timeout). When an interrupt occurs and the corresponding bit of
Sn_IMR is ‘1’, the corresponding bit of Sn_IR becomes ‘1’.
In order to clear the Sn_IR bit, the host should write the bit to ‘1’.*/
	#define IR_SEND_OK		0x10 //SEND command is completed.
	#define IR_TIMEOUT		0x08 //when ARP or TCP TIMEOUT occurs.
	#define IR_RECV			  0x04 //whenever data is received from a peer.
	#define IR_DISCON		  0x02 //when FIN or FIN/ACK packet is received from a peer.
	#define IR_CON			  0x01 //when the connection with peer is successful and then Sn_SR is changed to SOCK_ESTABLISHED.

#define Sn_SR		0x0003 //(Socket n Status Register) [R] [0x0003] [0x00]
/*Sn_SR indicates the status of Socket n. The status of Socket n is changed by Sn_CR
or some special control packet as SYN, FIN packet in TCP.*/
	#define SOCK_CLOSED		    0x00 //This indicates that Socket n is released.
	#define SOCK_INIT		      0x13 //This indicates Socket n is opened with TCP mode.
	#define SOCK_LISTEN		    0x14 //This indicates Socket n is operating as ‘TCP server’ mode and waiting for connection-request (SYN packet) from a peer (‘TCP client’).
	#define SOCK_ESTABLISHED	0x17 //This indicates the status of the connection of Socket n.
	#define SOCK_CLOSE_WAIT		0x1c //This indicates Socket n received the disconnect-request (FIN packet) from the connected peer.
	#define SOCK_UDP		      0x22 //This indicates Socket n is opened in UDP mode
	#define SOCK_MACRAW		    0x02 //This indicates Socket 0 is opened in MACRAW mode (S0_MR(P[3:0]) = ‘0100’)and is valid only in Socket 0.

	#define SOCK_SYNSEND	    0x15 //This indicates Socket n sent the connect-request packet (SYN packet) to a peer.
	#define SOCK_SYNRECV	    0x16 //It indicates Socket n successfully received the connectrequest packet (SYN packet) from a peer.
	#define SOCK_FIN_WAI			0x18 // These indicate Socket n is closing.
	#define SOCK_CLOSING			0x1a // These indicate Socket n is closing.
	#define SOCK_TIME_WAIT		0x1b // These indicate Socket n is closing.
	#define SOCK_LAST_ACK			0x1d // This indicates Socket n is waiting for the response (FIN/ACK packet) to the disconnect-request (FIN packet) by passiveclose.

/* Sn_PORT (Socket n Source Port Register) [R/W] [0x0004-0x0005] [0x0000]
Sn_PORT configures the source port number of Socket n. It is valid when Socket n is
used in TCP/UDP mode. It should be set before OPEN command is ordered.*/
#define Sn_PORT		0x0004 
/* Sn_DHAR (Socket n Destination Hardware Address Register) [R/W] [0x0006-0x000B] [0xFFFFFFFFFFFF]
Sn_DHAR configures the destination hardware address of Socket n when using
SEND_MAC command in UDP mode or it indicates that it is acquired in ARP-process by
CONNECT/SEND command.
*/
#define Sn_DHAR	  0x0006
/*Sn_DIPR (Socket n Destination IP Address Register)
[R/W] [0x000C-0x000F] [0x00000000]
Sn_DIPR configures or indicates the destination IP address of Socket n. It is valid when Socket n is used in TCP/UDP mode.
In TCP client mode, it configures an IP address of ‘TCP server’ before CONNECT command.
In TCP server mode, it indicates an IP address of ‘TCP client’ after successfully establishing connection.
In UDP mode, it configures an IP address of peer to be received the UDP packet by SEND or SEND_MAC command.
*/
#define Sn_DIPR		0x000c
/* Sn_DPORT (Socket n Destination Port Register) [R/W] [0x0010-0x0011] [0x00]
Sn_DPORT configures or indicates the destination port number of Socket n. It is valid
when Socket n is used in TCP/UDP mode.
*/
#define Sn_DPORTR	0x0010

#define Sn_MSSR		0x0012 //(Socket n Maximum Segment Size Register) [R/W] [0x0012-0x0013] [0x0000]
#define Sn_TOS		0x0015
#define Sn_TTL		0x0016
/*Sn_RXBUF_SIZE (Socket n RX Buffer Size Register) [R/W] [0x001E] [0x02]
Sn_RXBUF_SIZE configures the RX buffer block size of Socket n. Socket n RX Buffer Block size can be configured with 1,2,4,8, and 16 Kbytes.
*/
#define Sn_RXBUF_SIZE	0x001e
/*Sn_TXBUF_SIZE (Socket n TX Buffer Size Register) [R/W] [0x001F] [0x02]
Sn_TXBUF_SIZE configures the TX buffer block size of Socket n. Socket n TX Buffer Block size can be configured with 1,2,4,8, and 16 Kbytes.
*/
#define Sn_TXBUF_SIZE	0x001f

//Sn_TX_FSR (Socket n TX Free Size Register) [R] [0x0020-0x0021] [0x0800]
#define Sn_TX_FSR	0x0020
//(Socket n TX Read Pointer Register) [R] [0x0022-0x0023] [0x0000]
#define Sn_TX_RD	0x0022

/*Sn_TX_WR : Socket n TX Write Pointer Register [R/W] [0x0024-0x0025] [0x0000]
It should be read or to be updated like as follows.
1. Read the starting address for saving the transmitting data.
2. Save the transmitting data from the starting address of Socket n TX buffer.
3. After saving the transmitting data, update Sn_TX_WR to the
increased value as many as transmitting data size. If the increment value
exceeds the maximum value 0xFFFF(greater than 0x10000 and the carry
bit occurs), then the carry bit is ignored and will automatically update
with the lower 16bits value.
4. Transmit the saved data in Socket n TX Buffer by using SEND/SEND command*/
#define Sn_TX_WR	0x0024

#define Sn_RX_RSR	0x0026
#define Sn_RX_RD	0x0028
#define Sn_RX_WR	0x002a

/*Sn_IMR (Socket n Interrupt Mask Register) [R/W] [0x002C] [0xFF]
Sn_IMR masks the interrupt of Socket n. Each bit corresponds to each bit of Sn_IR.
When a Socket n Interrupt is occurred and the corresponding bit of Sn_IMR is ‘1’, the
corresponding bit of Sn_IR becomes ‘1’.
*/
#define Sn_IMR		0x002c
	#define IMR_SENDOK	0x10
	#define IMR_TIMEOUT	0x08
	#define IMR_RECV	0x04
	#define IMR_DISCON	0x02
	#define IMR_CON		0x01

#define Sn_FRAG		0x002d
//Sn_KPALVTR (Socket n Keep Alive Time Register) [R/W] [0x002F] [0x00]
//Sn_KPALVTR configures the transmitting timer of ‘KEEP ALIVE(KA)’ packet of SOCKETn.
//It is valid only in TCP mode, and ignored in other modes.
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
#define S_TX_SIZE	2048  /*Define the size of the Socket receive buffer, which can be modified according to the settings of W5500_RMSR*/




// W5500 GPIO Define its SPI definition part in the SPI file
#define W5500_SCS		GPIO_Pin_0	//W5500 CS	 
#define W5500_SCS_GPIO	GPIOA

#define	W5500_NCS GPIO_ResetBits(W5500_SCS_GPIO, W5500_SCS)
#define	W5500_CS  GPIO_SetBits(W5500_SCS_GPIO, W5500_SCS)

#define W5500_RST		GPIO_Pin_2	//W5500 RST
#define W5500_RST_GPIO	GPIOA


#define W5500_INT		GPIO_Pin_3	//W5500 INT
#define W5500_INT_GPIO	GPIOA





// Network parameter variable definition
extern unsigned char Gateway_IP[4];//Gateway IP address 
extern unsigned char Sub_Mask[4];	//???? 
extern unsigned char Phy_Addr[6];	//????(MAC) 
extern unsigned char IP_Addr[4];	//??IP?? 

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
#define S_Unknown 0x00  //Khong ro trang thai 
#define S_INIT		0x01	//Port completion initialization 
#define S_CONN		0x02	//Port complete connection,Can transfer data normally  

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
extern u8 Socket2UDP(SOCKET s);    //Set the specified Socket(0~7)UDP mode
extern u16 Read_SOCK_Data_Buffer(SOCKET s, u8 *dat_ptr);          //Specify Socket(0~7)Receive data processing
extern void Write_SOCK_Data_Buffer(SOCKET s, u8 *dat_ptr, u16 size); //Specify Socket(0~7)Send data processing
extern void W5500_Interrupt_Process(void);    //W5500 interrupt handler framework
void sendSocketNbuff(SOCKET s, u8 *dat_ptr, u16 size);
void Write_W5500_SOCK_2Byte(SOCKET s, u16 reg, u16 dat);
void Write_W5500_SOCK_4Byte(SOCKET s, u16 reg, u8 *dat_ptr);
u16 Read_W5500_SOCK_2Byte(SOCKET s, u16 reg);
void SPI1_Send_Short(u16 dat);
void SPI1_Send_Byte(u8 dat);
void Write_W5500_SOCK_1Byte(SOCKET s, u16 reg, u8 dat);
#endif

