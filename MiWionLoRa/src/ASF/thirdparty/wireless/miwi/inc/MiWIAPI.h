/*
 * MiWIAPI.h
 *
 * Created: 2019-08-08 오전 2:51:08
 *  Author: A14633
 */ 


#ifndef MIWIAPI_H_
#define MIWIAPI_H_

#include "ConfigApp.h"
#include "MiWi.h"
#include "sys.h"

#define INPUT
#define OUTPUT
#define IOPUT

typedef union __CONNECTION_STATUS
{
    uint8_t Val;
    struct _CONNECTION_STAUTS_bits
    {
        uint8_t RXOnWhenIdle       :1;     //1 = transceiver always on, 0 = transceiver sleeps when idle
        uint8_t directConnection   :1;     //1 = can talk to this device directly, 0 = must route to this device
        uint8_t longAddressValid   :1;     //1 = long address valid, 0 = long address unknown
        uint8_t shortAddressValid  :1;     //1 = short address valid, 0 = short address unknown
        uint8_t FinishJoin         :1;     //1 = already finish joining procedure, 0 = in the process of join
        uint8_t isFamily           :1;     //1 = family member (parent/child), 0 = not family
        uint8_t filler             :1;
        uint8_t isValid            :1;     //1 = this entry is valid, 0 = this entry is not valid
    } bits;   
} CONNECTION_STATUS;
 
typedef struct __CONNECTION_ENTRY
{
    #if !defined(PROTOCOL_P2P)
        WORD_VAL    PANID;                      // PAN Identifier of the peer device. May not necessary in P2P protocol
        WORD_VAL    AltAddress;                 // Alternative address of the peer device. Not necessary in P2P protocol
    #endif
    uint8_t        Address[MY_ADDRESS_LENGTH];     // Permanent address of peer device
        
    CONNECTION_STATUS status;
        
    #if ADDITIONAL_NODE_ID_SIZE > 0
        uint8_t        PeerInfo[ADDITIONAL_NODE_ID_SIZE];  // Additional Node ID information, if defined in application layer
    #endif
} CONNECTION_ENTRY;

extern CONNECTION_ENTRY    ConnectionTable[CONNECTION_SIZE];

extern uint8_t            currentChannel;


bool    MiApp_ProtocolInit(bool bNetworkFreezer);
bool    MiApp_SetChannel(uint8_t Channel);
    
#define START_CONN_DIRECT       0x00
#define START_CONN_ENERGY_SCN   0x01
#define START_CONN_CS_SCN       0x02
#define START_FROM_NVM			0x03
    
bool    MiApp_StartConnection( uint8_t Mode, uint8_t ScanDuration, uint32_t ChannelMap);
   
typedef struct
{
    uint8_t		Channel;                        // Operating Channel of the PAN
    uint8_t        Address[MY_ADDRESS_LENGTH];     // Responding device address
    WORD_VAL    PANID;                          // PAN Identifier
    uint8_t        RSSIValue;                      // RSSI value for the response  
    uint8_t        LQIValue;                       // LQI value for the response
    union
    {
        uint8_t        Val;
        struct
        {
            uint8_t    Role:		2;              // Role of the responding device in the PAN
            uint8_t    Sleep:		1;              // Whether the responding device goes to sleep when idle
            uint8_t    SecurityEn:	1;              // Whether the responding device is capable of securing the data
            uint8_t    RepeatEn:	1;              // Whether the responding device allow repeat
            uint8_t    AllowJoin:	1;              // Whether the responding device allows other device to join
            uint8_t    Direct:		1;              // Whether the responding device in radio range or through a repeater
            uint8_t    altSrcAddr: 1;              // Whether the Address is alternative network address or permanent address
        } bits;
    } Capability;
    #if ADDITIONAL_NODE_ID_SIZE > 0
        uint8_t        PeerInfo[ADDITIONAL_NODE_ID_SIZE];  // Additional Node ID information, if defined in application layer
    #endif
    } ACTIVE_SCAN_RESULT; 
    
#ifdef ENABLE_ACTIVE_SCAN
    extern ACTIVE_SCAN_RESULT ActiveScanResults[ACTIVE_SCAN_RESULT_SIZE];
#endif
    
   
uint8_t    MiApp_SearchConnection(uint8_t ScanDuration, uint32_t ChannelMap);
    
#define CONN_MODE_DIRECT        0x00
#define CONN_MODE_INDIRECT      0x01
#define CONN_MODE_NVM			0x02
     
uint8_t    MiApp_EstablishConnection(uint8_t ActiveScanIndex, uint8_t Mode);
  
void    MiApp_RemoveConnection(uint8_t ConnectionIndex);
    
#define ENABLE_ALL_CONN         0x00
#define ENABLE_PREV_CONN        0x01
#define ENABLE_ACTIVE_SCAN_RSP  0x02
#define DISABLE_ALL_CONN        0x03
   
void    MiApp_ConnectionMode(uint8_t Mode);
 
#define MiApp_FlushTx()   {TxData = PAYLOAD_START;}
#define MiApp_WriteData(a) TxBuffer[TxData++] = a

bool MiApp_BroadcastPacket(bool SecEn );
    
uint8_t MiApp_UnicastConnection(uint8_t ConnectionIndex, bool SecEn);
   
bool MiApp_UnicastAddress(uint8_t *DestinationAddress, bool PermanentAddr, bool SecEn);

#define BROADCAST_TO_ALL            0x01
#define MULTICAST_TO_COORDINATORS   0x02
#define MULTICAST_TO_FFDS           0x03
    
typedef struct
{
    union
    {
        uint8_t Val;
        struct
        {
            uint8_t		broadcast:  2;      // 1: broadcast message
            uint8_t		ackReq:     1;      // 1: sender request acknowledgement in MAC.
            uint8_t		secEn:      1;      // 1: application payload has been secured
            uint8_t		repeat:     1;      // 1: message received through a repeater
            uint8_t		command:    1;      // 1: message is a command frame
            uint8_t        srcPrsnt:   1;      // 1: source address present in the packet
            uint8_t        altSrcAddr: 1;      // 1: source address is alternative network address
        } bits;
    }  flags;

    WORD_VAL    SourcePANID;                // PAN Identifier of the sender
    uint8_t		*SourceAddress;             // pointer to the source address
    uint8_t		*Payload;                   // pointer to the application payload
    uint8_t		PayloadSize;                // application payload length
    int16_t		PacketRSSI;                 // RSSI value of the receive message
    uint8_t 		PacketLQI;                  // LQI value of the received message

} RECEIVED_MESSAGE;
   
bool    MiApp_MessageAvailable(void);

void    MiApp_DiscardMessage(void);
    
#define NOISE_DETECT_ENERGY 0x00
#define NOISE_DETECT_CS     0x01
  
uint8_t MiApp_NoiseDetection(uint32_t ChannelMap, uint8_t ScanDuration, uint8_t DetectionMode, int16_t *NoiseLevel);
    
#define POWER_STATE_SLEEP       0x00
#define POWER_STATE_WAKEUP      0x01
#define POWER_STATE_WAKEUP_DR   0x02

uint8_t    MiApp_TransceiverPowerState(uint8_t Mode);
#define SUCCESS                 0x00
#define ERR_TRX_FAIL            0x01
#define ERR_TX_FAIL             0x02
#define ERR_RX_FAIL             0x03
#define ERR_INVALID_INPUT       0xFF

bool MiApp_InitChannelHopping( uint32_t ChannelMap);
  
bool MiApp_ResyncConnection(uint8_t ConnectionIndex, uint32_t ChannelMap);

// Callback functions
#define MiApp_CB_AllowConnection(handleInConnectionTable) true
//bool MiApp_CB_AllowConnection(uint8_t handleInConnectionTable);
#define MiApp_CB_RFDAcknowledgement(SourceShortAddress, AcknowledgementSeqNum)
//void MiApp_CB_RFDAcknowledgement(WORD sourceAddr, uint8_t Seq);
    
extern RECEIVED_MESSAGE rxMessage;
#if defined(ENABLE_TIME_SYNC)
    extern uint16_t CounterValue;
    extern uint16_t WakeupTimes;
#endif



#endif /* MIWIAPI_H_ */