#include "GPIO_STM32F10x.h"             // Keil::Device:GPIO

// Time Server Port
#define NTP_PORT 123
static const int NTP_PACKET_SIZE = 48;
static const int NTP_PACKET_RAWSIZE = 56;

/* Shifts usecs in unixToNtpTime */
//??? ko hieu nhung dung! 
#ifndef USECSHIFT
#define USECSHIFT (1LL << 32) * 1.0e-6
#endif
//Number of seconds from 1st January 1900 to start of Unix epoch
//According to the Time protocol in RFC 868 it is 2208988800L.
#define STARTOFTIME 2208988800UL

#ifndef UTIL_H
#define UTIL_H

#define htons(x) ( ((x)<< 8 & 0xFF00) | \
                   ((x)>> 8 & 0x00FF) )
#define ntohs(x) htons(x)

#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)
#endif

#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24UL)
#define DAYS_PER_WEEK (7UL)
#define SECS_PER_WEEK (SECS_PER_DAY * DAYS_PER_WEEK)
#define SECS_PER_YEAR (SECS_PER_WEEK * 52UL)
#define SECS_YR_2000  (946684800UL) // the time at the start of y2k
#define LEAP_YEAR(Y)  ( ((1970+Y)>0) && !((1970+Y)%4) && ( ((1970+Y)%100) || !((1970+Y)%400) ) )

typedef struct tm_t {
	uint8_t  year;    // years since 1970
	uint8_t  month;   // month of a year - [ 1 to 12 ]
	uint8_t  day;     // day of a month - [ 1 to 31 ]
	uint8_t  weekday; // day of a week (first day is Monday) - [ 0 to 6 ]
	uint8_t  pm;      // AM: 0, PM: 1
	uint8_t  hour;    // hour of a day - [ 0 to 23 ]
	uint8_t  minute;  // minute of an hour - [ 0 to 59 ]
	uint8_t  second;  // second of a minute - [ 0 to 59 ]
} tm_t;
static  const unsigned char monthDays[]={31,28,31,30,31,30,31,31,30,31,30,31}; // API starts months from 1, this array starts from 0
//#define LEAP_YEAR(Y)     ( ((1970+Y)>0) && !((1970+Y)%4) && ( ((1970+Y)%100) || !((1970+Y)%400) ) )
/*
tm_t tmm = {
	.year =49,
	.month =6,
	.day =7,
	.weekday=4,
	.pm = 1,
	.hour = 16,
	.minute = 15,
	.second =1
};
*/
//static inline uint8_t bcd2bin(uint8_t b) { return ( (10*(b>>4)) + (b&0x0F) ); }
//static inline uint8_t bin2bcd(uint8_t b) { return ( ((b/10)<<4) + (b%10) ); }

/* Ban tin chuan server time.nist.gov 48 bytes (https://www.cisco.com/c/en/us/about/press/internet-protocol-journal/back-issues/table-contents-58/154-ntp.html)
00.01.02.03: 1C.01.0D.E3 [LI][VN][Mode].[Stratum].[Poll].[Precision]
04.05.06.07: 00.00.00.10 [Root delay]:The total round-trip delay from the server to the primary reference sourced. The value is a 32-bit signed fixed-point number in units of seconds, with the fraction point between bits 15 and 16. This field is significant only in server messages.
08.09.10.11: 00.00.00.20 [Root Dispersion]: The maximum error due to clock frequency tolerance. The value is a 32-bit signed fixed-point number in units of seconds, with the fraction point between bits 15 and 16. This field is significant only in server messages.
12.13.14.15: 4E.49.53.54 [Reference Identifier]: Name of sender: NIST. For stratum 1 servers this value is a four-character ASCII code that describes the external reference source (refer to Figure 2). For secondary servers this value is the 32-bit IPv4 address of the synchronization source, or the first 32 bits of the Message Digest Algorithm 5 (MD5) hash of the IPv6 address of the synchronization source.
16.17.18.19: E0.28.BA.7F [Reference Timestamp]: unsigned 32-bit seconds value
20.21.22.23: 00.00.00.00       						32-bit fractional
24.25.26.27: 00.00.00.00 [Originate Timestamp, T1]: unsigned 32-bit seconds value
28.29.30.31: 00.00.00.00         					32-bit fractional
32.33.34.35: E0.28.C5.79 [Receive Timestamp, T2]: unsigned 32-bit seconds value
36.37.38.39: E5.7A.55.F6      						32-bit fractional
40.41.42.43: E0.28.C5.79 [Transmit Timestamp, T3]: unsigned 32-bit seconds value
44.45.46.47: E5.7A.5F.93      						32-bit fractional


The next four fields use a 64-bit time-stamp value. This value is an unsigned 32-bit seconds value, and a 32-bit fractional part. In this notation the value 2.5 would be represented by the 64-bit string:

0000|0000|0000|0000|0000|0000|0000|0010.|1000|0000|0000|0000|0000|0000|0000|0000

The unit of time is in seconds, and the epoch is 1 January 1900, meaning that the NTP time will cycle in the year 2036 (two years before the 32-bit Unix time cycle event in 2038).

The smallest time fraction that can be represented in this format is 232 picoseconds.

Reference Timestamp   	This field is the time the system clock was last set or corrected, in 64-bit time-stamp format.

Originate Timestamp   	This value is the time at which the request departed the client for the server, in 64-bit time-stamp format. Th?i di?m b?n tin di t? th?ng h?i

Receive Timestamp	This value is the time at which the client request arrived at the server in 64-bit time-stamp format. Th?i di?m th?ng server nh?n du?c

Transmit Timestamp	This value is the time at which the server reply departed the server, in 64-bit time-stamp format. th?i di?m ph?n h?i t? server
*/

/*typedef struct ntp_Info_t
{
   //uint8_t LI;  			///[LI][VN][Mode]
   //uint8_t Vers;   		///< 
   //uint8_t Mode;   		///<  Mask 
	 uint8_t LiVeMod;  			///[LI][VN][Mode]
   uint8_t stratum;   ///[Stratum] 
   uint8_t polling;  	///[Poll]
   uint8_t precision; ///[Precision]
	 int32_t rootDelay; //[Root delay]:The total round-trip delay from the server to the primary reference sourced.
	 int32_t rootDisper;//[Root Dispersion]: The maximum error due to clock frequency tolerance. The value is a 32-bit signed fixed-point number in units of seconds, with the fraction point between bits 15 and 16. This field is significant only in server messages.
	 uint8_t refID[4];  //[Reference Identifier]: Name of sender: NIST. For stratum 1 servers this value is a four-character ASCII code that describes the external reference source (refer to Figure 2). For secondary servers this value is the 32-bit IPv4 address of the synchronization source, or the first 32 bits of the Message Digest Algorithm 5 (MD5) hash of the IPv6 address of the synchronization source.
	 uint32_t refTime ; //[Reference Timestamp]: unsigned 32-bit seconds value
	 uint32_t refTimeFrac; //[Reference Timestamp]: unsigned 32-bit 32-bit fractional
	 uint32_t OrigTime; //[Originate Timestamp, T1]: unsigned 32-bit seconds value
	 uint32_t OrigTimeFrac; //[Originate Timestamp, T1]: unsigned 32-bit fractional
	 uint32_t RecTime; //[Receive Timestamp, T2]: unsigned 32-bit seconds value
	 uint32_t RecTimeFrac; //[Receive Timestamp, T2]: unsigned 32-bit fractional
	 uint32_t TransTime; //[Transmit Timestamp, T3]: unsigned 32-bit seconds value
	 uint32_t TransTimeFrac; //[Transmit Timestamp, T3]: unsigned 32-bit fractional
}ntp_server;
ntp_server myNtpServer = {
													.LiVeMod = 0b11100011,    // LI, Version, Mode
													.stratum = 0,							// Stratum, or type of clock
													.polling = 6,// Polling Interval
													.precision =0xEC,  // Peer Clock Precision
													.rootDelay =1,
													.rootDisper =1,
													.refID ="GPS ",
													.refTime =1,
													.refTimeFrac =1,
													.OrigTime =1,
													.OrigTimeFrac =1,
													.RecTime =1, 
													.RecTimeFrac =1,
													.TransTime =1,
													.TransTimeFrac =1
};
*/


/*
//The Human date to Unix epoch time (timestamp) 
time_t makeTime(tm_t tmm)
{
// assemble time elements into time_t 
// note year argument is offset from 1970 (see macros in time.h to convert to other formats)
// previous version used full four digit year (or digits since 2000),i.e. 2009 was 2009 or 9
  
	int i;
	uint32_t seconds;

	// seconds from 1970 till 1 jan 00:00:00 of the given year
	seconds = tmm.year*(SECS_PER_DAY * 365);
	for (i = 0; i < tmm.year; i++) {
		if (LEAP_YEAR(i)) {
			seconds +=  SECS_PER_DAY;   // add extra days for leap years
		}
	}

	// add days for this year, months start from 1
	for (i = 1; i < tmm.month; i++) {
		if ( (i == 2) && LEAP_YEAR(tmm.year)) { 
			seconds += SECS_PER_DAY * 29;
		} else {
			seconds += SECS_PER_DAY * monthDays[i-1];  //monthDay array starts from 0
		}
	}
	seconds+= (tmm.day-1) * SECS_PER_DAY;
	seconds+= tmm.hour * SECS_PER_HOUR;
	seconds+= tmm.minute * SECS_PER_MIN;
	seconds+= tmm.second;
	return (time_t)seconds; 
}


uint32_t rtc_set_count(time_t time_stamp) {
	uint32_t h, l;

	return (h << 16) | l;
}
void setTime2 (time_t time_stamp) {
		rtc_set_count(time_stamp);
	}
void setTime (tm_t tmm) {
		time_t mktm = makeTime(tmm); // time will be make to mktm
    		setTime2(mktm);
		//rtc_set_count(time_stamp);
	}

void breakTime(time_t timeInput, tm_t tmm)
{
// break the given time_t into time components
// this is a more compact version of the C library localtime function
// note that year is offset from 1970 !!!

	uint8_t year;
	uint8_t month, monthLength;
	uint32_t time;
	uint32_t days;

	time = (uint32_t)timeInput;
	tmm.second = time % 60;
	time /= 60; // now it is minutes
	tmm.minute = time % 60;
	time /= 60; // now it is hours
	tmm.hour = time % 24;
	time /= 24; // now it is days
	tmm.weekday = ((time + 3) % 7); // Monday is day 1

	year = 0;
	days = 0;
	while((unsigned)(days += (LEAP_YEAR(year) ? 366 : 365)) <= time) {
		year++;
	}
	tmm.year = year; // year is offset from 1970 

	days -= LEAP_YEAR(year) ? 366 : 365;
	time -= days; // now it is days in this year, starting at 0

	days = 0;
	month = 0;
	monthLength = 0;
	for (month=0; month<12; month++) {
		if (month==1) { // february
			if (LEAP_YEAR(year)) {
				monthLength=29;
			} else {
				monthLength=28;
			}
		} else {
			monthLength = monthDays[month];
		}

		if (time >= monthLength) {
			time -= monthLength;
		} else {
			break;
		}
	}
	tmm.month = month + 1;  // jan is month 1  
	tmm.day = time + 1;     // day of month
}

uint32_t rtc_get_count() {
return 1;
}

void getTime(tm_t tm_ptr) {
		time_t res = rtc_get_count();
		breakTime(res, tm_ptr);
	}
time_t getTime2() {
		return rtc_get_count();
	}

time_t now() { return getTime2(); }
void now2(tm_t tmm ) { getTime2(); } 
// Usage: localtime = TimeZone(UnixTime, 8); 
time_t TimeZone(time_t t, int TZ) { return ( t + (TZ * SECS_PER_HOUR)); } 
	// Usage:  1.  localtime = TimeZone(UnixTime, 9, 45)  ->   UTC +09:45 TimeZone; 
time_t TimeZone2(time_t t, int HTZ, int MTZ)  { return ( t + (HTZ * SECS_PER_HOUR) + (MTZ * 60)); }    // HTZ = Hour offset, MTZ = Minute offset
*/

