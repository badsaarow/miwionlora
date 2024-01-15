/*
 * ConfigApp.h
 *
 * Created: 2019-08-08 오전 2:05:01
 *  Author: A14633
 */ 


#ifndef CONFIGAPP_H_
#define CONFIGAPP_H_

/*********************************************************************/
// following codes defines the platforms as well as the hardware 
// configuration
/*********************************************************************/

/*********************************************************************/
// ENABLE_CONSOLE will enable the print out on the hyper terminal
// this definition is very helpful in the debugging process
/*********************************************************************/
#define ENABLE_CONSOLE

/*********************************************************************/
// HARDWARE_SPI enables the hardware SPI implementation on MCU
// silicon. If HARDWARE_SPI is not defined, digital I/O pins will
// be used to bit-bang the RF transceiver
/*********************************************************************/
#define HARDWARE_SPI

#define PROTOCOL_MIWI_PRO

    //#define NWK_ROLE_COORDINATOR
    //#define NWK_ROLE_END_DEVICE

//------------------------------------------------------------------------
// Definition of RF Transceiver. ONLY ONE TRANSCEIVER CAN BE CHOSEN
//------------------------------------------------------------------------
    
#define SX1276RF1KAS

#if defined(SX1276RF1KAS)
			
	//#define AES128_ENABLE
	#if defined(AES128_ENABLE)
		#include "aes.h"
	#endif
	
	//#define USE_SX1276_RADIO
    #ifdef USE_SX1276_RADIO
		#define MODULE_SX1276RF1IAS                         0
		#define MODULE_SX1276RF1JAS                         0
		#define MODULE_SX1276RF1KAS                         1
    #endif
#endif

#define IEEE_STANDARD_MODE

/*********************************************************************/
// ENABLE_NETWORK_FREEZER enables the network freezer feature, which
// stores critical network information into non-volatile memory, so
// that the protocol stack can recover from power loss gracefully.
// The network infor can be saved in data EPROM of MCU, external 
// EEPROM or programming space, if enhanced flash is used in MCU.
// Network freezer feature needs definition of NVM kind to be 
// used, which is specified in HardwareProfile.h
/*********************************************************************/
#define ENABLE_NETWORK_FREEZER


/*********************************************************************/
// MY_ADDRESS_LENGTH defines the size of wireless node permanent 
// address in byte. This definition is not valid for IEEE 802.15.4
// compliant RF transceivers.
/*********************************************************************/
#define MY_ADDRESS_LENGTH       8//5

//*********************************************************************
//LBT (Listen-Before-Talk)
//*********************************************************************
#define LBT_EN

#ifdef LBT_EN
	#define LBT_CHK_TIME			(FIVE_MILI_SECOND)	//(DWORD)(((DWORD)5		*1000)/ONE_TICK_16USEC)		//min 5msec
	#define LBT_TRANS_TIME			(ONE_SECOND * 4)	//(DWORD)(((DWORD)4000	*1000)/ONE_TICK_16USEC)		//max 4sec
	#define LBT_DELAY				(FIFTY_MILI_SECOND)	//(DWORD)(((DWORD)50		*1000)/ONE_TICK_16USEC)		//min 50msec

    #define LBT_BLOCK_DBM			-65//(CHAR)(-65)	//-65dBm


	#define LBT_RETRY_TX			10	//Retry count
	#define LBT_RETRY_INTERVAL		(HUNDRED_MILI_SECOND * 5)	//(DWORD)(((DWORD)500		*1000)/ONE_TICK_16USEC)
#endif


#define ROLE_TREE_PAN_COORDINATOR
//#define ROLE_TREE_COORDINATOR
//#define ROLE_TREE_ENDDEVICE

#ifdef ROLE_TREE_ENDDEVICE
    #define NWK_ROLE_END_DEVICE
	/*********************************************************************/
	// ENABLE_SLEEP will enable the device to go to sleep and wake up 
	// from the sleep
	/*********************************************************************/
	//#define ENABLE_SLEEP	
#else
    #define NWK_ROLE_COORDINATOR
#endif

#ifdef ENABLE_SLEEP
	//#define SLEEP_T1RTC			//T1 타이머에 의한 SLEEP ON/OFF
	#ifdef SLEEP_T1RTC
		#define WAKEUP_TIMEOUT		20	//sec OPERATE to SLEEP 진입시간.
		#define SLEEP_TIMEOUT		20	//sec SLEEP to WAKEUP  진입시간.
	#endif

	#define SLEEP_SWITCH		//스위치에 의한 SLEEP ON/OFF
	#ifdef SLEEP_SWITCH
		#define SW_IN			PORTBbits.RB6
		#define SW_TRIS			TRISB6
	#endif
	
	#if defined(SLEEP_T1RTC)&defined(SLEEP_SWITCH)
		!!select "SLEEP_T1RTC" or "SLEEP_SWITCH"
	#endif
#else
	//#define TEST_PING_PONG_DATA			//데이터 수신시 테스트 데이터 자동 리턴
	//#define TEST_BATTERY					//배터리사용시 MOTE EVB의 45K50으로 부터 수신 데이티 무시하고 "Start Active Scan"됨
#endif

#if defined(ENABLE_NETWORK_FREEZER)
    #define ENABLE_NVM
	#define USER_MEMORY_SIZE	10			//byte start_add 0x00, MiWi Stack에서 사용중인 메모리외에 사용자가 별도 사용할 메모리.
#endif

/*********************************************************************/
// EUI_x defines the xth byte of permanent address for the wireless
// node
/*********************************************************************/
//#define ENABLE_EEPROM_MAC_EUI

#ifndef	ENABLE_EEPROM_MAC_EUI
	#define EUI_7 0x11
	#define EUI_6 0x22
	#define EUI_5 0x33
	#define EUI_4 0x44
	#define EUI_3 0x55
	#define EUI_2 0x66
	#define EUI_1 0x77
	
	//#define EUI_0 0x01
	//#define EUI_0 0x02
	#define EUI_0 0x0F

/*
	#ifdef ROLE_TREE_PAN_COORDINATOR
	#define EUI_0 0x01
	#endif

	#ifdef ROLE_TREE_COORDINATOR
	#define EUI_0 0x02
	#endif

	#ifdef ROLE_TREE_ENDDEVICE
	#define EUI_0 0x05
	#endif
*/	
#endif
//kwlee
unsigned char USE_CHANNEL_NUM_START;
unsigned char USE_CHANNEL_NUM_END;
unsigned char INIT_CHANNEL_NUM;
//#define USE_CHANNEL_NUM_START	20//20
//#define USE_CHANNEL_NUM_END		20//32
//
////initial channel num
//#define INIT_CHANNEL_NUM	20//20

// SignalBw [0:  7.8kHz, 1: 10.4kHz, 2: 15.6kHz, 3: 20.8kHz, 4: 31.25kHz,
//			 5: 41.7kHz, 6: 62.5kHz, 7: 125kHz,  8: 250kHz,  9: 500kHz, other: Reserved]
//#define INIT_bandwidth		8

// SpreadingFactor [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
//#define INIT_SpreadingFactor		7
				
#define Lora_Mode		9	//max range, slow data rate : 0~9(1~10) : min range, fast data rate.
#define SET_RF_POWER	20	//20, 17 ~ 02 dBm.

#define SCAN_TIME_LEVEL		9	//4~14, (60 * (2 ^ n + 1))*16 usec
		
/*********************************************************************/
// TX_BUFFER_SIZE defines the maximum size of application payload
// which is to be transmitted
/*********************************************************************/
#define TX_BUFFER_SIZE 220

/*********************************************************************/
// RX_BUFFER_SIZE defines the maximum size of application payload
// which is to be received
/*********************************************************************/
#define RX_BUFFER_SIZE 220

/*********************************************************************/
// MY_PAN_ID defines the PAN identifier. Use 0xFFFF if prefer a 
// random PAN ID.
// PAN-Coordinator만 유효하며, Coordinator,end device는 연결시 PAN-Coordinator ID를 따름.
/*********************************************************************/
//kwlee 200713

//#define MY_PAN_ID                       0xabc1
unsigned int MY_PAN_ID;
#define MY_SHORT_ADDR                   0x1111
//#define MY_PAN_ID                       0xFFFF
//
//#define MY_SHORT_ADDR                   0xFFFF

/*********************************************************************/
// ADDITIONAL_NODE_ID_SIZE defines the size of additional payload
// will be attached to the P2P Connection Request. Additional payload 
// is the information that the devices what to share with their peers
// on the P2P connection. The additional payload will be defined by 
// the application and defined in main.c
/*********************************************************************/
#define ADDITIONAL_NODE_ID_SIZE   2


/*********************************************************************/
// P2P_CONNECTION_SIZE defines the maximum P2P connections that this 
// device allowes at the same time. 
/*********************************************************************/
#ifdef ROLE_TREE_ENDDEVICE
	#define CONNECTION_SIZE			10//1	kwlee 200714	
#else
	#define CONNECTION_SIZE			10//1	kwlee 200714	
#endif


/*********************************************************************/
// TARGET_SMALL will remove the support of inter PAN communication
// and other minor features to save programming space
/*********************************************************************/
//#define TARGET_SMALL


/*********************************************************************/
// ENABLE_PA_LNA enable the external power amplifier and low noise
// amplifier on the RF board to achieve longer radio communication 
// range. To enable PA/LNA on RF board without power amplifier and
// low noise amplifier may be harmful to the transceiver.
/*********************************************************************/
//#define ENABLE_PA_LNA


/*********************************************************************/
// ENABLE_HAND_SHAKE enables the protocol stack to hand-shake before 
// communicating with each other. Without a handshake process, RF
// transceivers can only broadcast, or hardcoded the destination address
// to perform unicast.
/*********************************************************************/
#define ENABLE_HAND_SHAKE

/*********************************************************************/
// ENABLE_ED_SCAN will enable the device to do an energy detection scan
// to find out the channel with least noise and operate on that channel
/*********************************************************************/
#define ENABLE_ED_SCAN


/*********************************************************************/
// ENABLE_ACTIVE_SCAN will enable the device to do an active scan to 
// to detect current existing connection. 
/*********************************************************************/
#define ENABLE_ACTIVE_SCAN


/*********************************************************************/
// ENABLE_SECURITY will enable the device to encrypt and decrypt
// information transferred
/*********************************************************************/
//#define ENABLE_SECURITY

/*********************************************************************/
// ENABLE_INDIRECT_MESSAGE will enable the device to store the packets
// for the sleeping devices temporily until they wake up and ask for
// the messages
/*********************************************************************/
//#define ENABLE_INDIRECT_MESSAGE


/*********************************************************************/
// ENABLE_BROADCAST will enable the device to broadcast messages for
// the sleeping devices until they wake up and ask for the messages
/*********************************************************************/
#define ENABLE_BROADCAST


/*********************************************************************/
// RFD_WAKEUP_INTERVAL defines the wake up interval for RFDs in second.
// This definition is for the FFD devices to calculated various
// timeout. RFD depends on the setting of the watchdog timer to wake 
// up, thus this definition is not used.
/*********************************************************************/
#define RFD_WAKEUP_INTERVAL     8


/*********************************************************************/
// ENABLE_FREQUENCY_AGILITY will enable the device to change operating
// channel to bypass the sudden change of noise
/*********************************************************************/
#define ENABLE_FREQUENCY_AGILITY


// Constants Validation
    
#if !defined(MRF24J40) && !defined(MRF49XA) && !defined(MRF89XA) && !defined(MRF24XA) && !defined(SX1276RF1KAS)
    #error "One transceiver must be defined for the wireless application"
#endif

#if (defined(MRF24J40) && defined(MRF49XA)) || (defined(MRF24J40) && defined(MRF89XA)) || (defined(MRF49XA) && defined(MRF89XA))
    #error "Only one transceiver can be defined for the wireless application"
#endif

#if !defined(PROTOCOL_P2P) && !defined(PROTOCOL_MIWI) && !defined(PROTOCOL_MIWI_PRO)
    #error "One Microchip proprietary protocol must be defined for the wireless application."
#endif

#if defined(ENABLE_FREQUENCY_AGILITY)
    #define ENABLE_ED_SCAN
#endif

#if MY_ADDRESS_LENGTH > 8
    #error "Maximum address length is 8"
#endif

#if MY_ADDRESS_LENGTH < 2
    #error "Minimum address length is 2"
#endif

#if defined(IEEE_STANDARD_MODE)
   // #define IEEE_802_15_4  //kwlee 200714 fail
    #undef MY_ADDRESS_LENGTH
    #define MY_ADDRESS_LENGTH 8
#endif

#if defined(ENABLE_ACTIVE_SCAN) && defined(TARGET_SMALL)
    #error  Target_Small and Enable_Active_Scan cannot be defined together 
#endif

#if defined(ENABLE_INDIRECT_MESSAGE) && !defined(RFD_WAKEUP_INTERVAL)
    #error "RFD Wakeup Interval must be defined if indirect message is enabled"
#endif

#if (RX_BUFFER_SIZE > 220)
    #error RX BUFFER SIZE too large. Must be <= 127.
#endif

#if (TX_BUFFER_SIZE > 220)
    #error TX BUFFER SIZE too large. Must be <= 127.
#endif

#if (RX_BUFFER_SIZE < 10)
    #error RX BUFFER SIZE too small. Must be >= 10.
#endif

#if (TX_BUFFER_SIZE < 10)
    #error TX BUFFER SIZE too small. Must be >= 10.
#endif

#if (CONNECTION_SIZE > 0xFE)
    #error NETWORK TABLE SIZE too large.  Must be < 0xFF.
#endif

#endif /* CONFIGAPP_H_ */