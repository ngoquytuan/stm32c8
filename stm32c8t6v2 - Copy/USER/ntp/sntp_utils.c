#include "sntp_utils.h"
#include "socket.h"
/* Shifts usecs in unixToNtpTime */
#ifndef USECSHIFT
#define USECSHIFT (1LL << 32) * 1.0e-6
#endif
#define NTP_BUF_SIZE   56
// Time Server Port
#define NTP_PORT 123
static const int NTP_PACKET_SIZE = 48;

int timezone = 8;      // change to your timezone
//rtclock.setTime(1508417425);

uint32_t unixTime_last_sync = 0 ,Time_last_sync = 0;

/* SNTP Packet array */
char serverPacket[PACKETSIZE] = {0};
char echoPacket[10] = {0xF};

/* port to listen to sntp messages */
static char srcPort = 123;

uint32_t micros_recv = 0;
uint32_t timeStamp;
extern uint32_t micros_offset, uptime_sec_count;
extern int mb_begin_offset;
uint32_t micros_transmit = 0;
int32_t NTPUDP(uint8_t sn, uint8_t* buf)
{
	int32_t  ret;
   uint16_t size, sentsize;
   uint8_t  destip[4];
   uint16_t destport;
	 uint8_t i;
   //uint8_t  packinfo = 0;
	// Ban tin NTP co size = 56 ( ca header : IP[4],port[2],length[2], tru di header chi con 48
   switch(getSn_SR(sn))
   {
      case SOCK_UDP :
         if((size = getSn_RX_RSR(sn)) > 0)
         {
					  
					  if(size > NTP_BUF_SIZE) size = NTP_BUF_SIZE;
            ret = recvfrom(sn,buf,size,destip,(uint16_t*)&destport);
					  printf("\r\nsize:%d, ret:%d, NTP: ",size,ret);
            if(ret <= 0)
            {
               printf("%d: recvfrom error. %d\r\n",sn,ret);
               return ret;
            }
            size = (uint16_t) ret;
            sentsize = 0;
						//in ra ban tin
						for(i=0;i<48;i++)
						{
						   printf("%x ",*(buf+i));
						}
						
            while(sentsize != size)
            {
               ret = sendto(sn,buf+sentsize,size-sentsize,destip,destport);
               if(ret < 0)
               {
                  printf("%d: sendto error. %d\r\n",sn,ret);
                  return ret;
               }
               sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
            }
         }
         break;
      case SOCK_CLOSED:
         printf("%d:NTP server start\r\n",sn);
         if((ret=socket(sn,Sn_MR_UDP,NTP_PORT,0x00)) != sn)
            return ret;
         printf("%d:Opened, port [%d]\r\n",sn, NTP_PORT);
         break;
      default :
         break;
   }
   return 1;
}
void ntp_recv_and_respond(uint16_t dest_port, uint8_t src_ip[4], uint16_t src_port, const char *data, uint16_t len)
{
   if(48==len)
   {
   ether.ntpProcessAnswer(&timeStamp, srcPort);
   Serial.println("\n=====NTP request received=====");
   Serial.println("Packet length: "+ String(len));
   micros_recv = (((micros() - micros_offset)%1000000) + 1) * USECSHIFT;
   uint32_t recvTime = htonl(rtclock.getTime() + STARTOFTIME);
   uint32_t transmitTime = 0;

   Serial.print("Timestamp: ");
   Serial.println(timeStamp);
   Serial.print("UNIX time: ");
   Serial.println(timeStamp-STARTOFTIME);
   Serial.print("src_ip: ");
   Serial.print(src_ip[0]);
   Serial.print('.');
   Serial.print(src_ip[1]);
   Serial.print('.');
   Serial.print(src_ip[2]);
   Serial.print('.');
   Serial.println(src_ip[3]);
   Serial.print("dest_port: ");
   Serial.println(dest_port);
   Serial.print("src_port: ");
   Serial.println(src_port);

   serverPacket[0] = 0x24;   // LI, Version, Mode // Set version number and mode
   serverPacket[1] = 1; // Stratum, or type of clock
   serverPacket[2] = 0;     // Polling Interval
   serverPacket[3] = -12;  // Peer Clock Precision
   serverPacket[12] = 'G';
   serverPacket[13] = 'P';
   serverPacket[14] = 'S';

   Serial.print("NOW IS: ");
   Serial.print(rtclock.getTime());
   Serial.println();

   memcpy(&serverPacket[16], &unixTime_last_sync, 4);// HERE I need to insert the last gps time sync event time
   micros_transmit = (((micros() - micros_offset)%1000000) + 1) * USECSHIFT;
   micros_transmit = htonl(micros_transmit);
   micros_recv = htonl(micros_recv);

   transmitTime = htonl( rtclock.getTime() + STARTOFTIME);

    memcpy(&serverPacket[40], &transmitTime, 4);
    memcpy(&serverPacket[24], &data[40], 4);
    memcpy(&serverPacket[28], &data[44], 4);
    memcpy(&serverPacket[32], &recvTime, 4);
    memcpy(&serverPacket[36], &micros_recv, 4);
    memcpy(&serverPacket[44], &micros_transmit, 4);
    ether.makeUdpReply(serverPacket,sizeof serverPacket, srcPort);
  }
}