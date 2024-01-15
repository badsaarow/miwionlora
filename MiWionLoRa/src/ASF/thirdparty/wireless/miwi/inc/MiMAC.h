/*
 * MiMAC.h
 *
 * Created: 2019-08-07 오후 3:50:24
 *  Author: A14633
 */ 


#ifndef MIMAC_H_
#define MIMAC_H_

#include "sys.h"
#include "ConfigApp.h"
#include "ConfigMiWi.h"

#define CHANNEL_ASSESSMENT_CARRIER_SENSE    0x00
#define CHANNEL_ASSESSMENT_ENERGY_DETECT    0x01
    
#define POWER_STATE_DEEP_SLEEP              0x00
#define POWER_STATE_OPERATE                 0xFF


#define PACKET_TYPE_MASK        0x03
#define BROADCAST_MASK          0x04
#define SECURITY_MASK           0x08
#define REPEAT_MASK             0x10
#define ACK_MASK                0x20
#define DSTPRSNT_MASK           0x40
#define SRCPRSNT_MASK           0x80
    
#define PACKET_TYPE_DATA        0x00
#define PACKET_TYPE_COMMAND     0x01
#define PACKET_TYPE_ACK         0x02
#define PACKET_TYPE_RESERVE     0x03

typedef struct
{
    union
    {
        uint8_t    Val;   
        struct
        {
            uint8_t        packetType      : 2;        // type of packet. Possible types are
                                                    // * PACKET_TYPE_DATA - Data type
                                                    // * PACKET_TYPE_COMMAND -  Command type
                                                    // * PACKET_TYPE_ACK -  Acknowledgement type
                                                    // * PACKET_TYPE_RESERVE - Reserved type
            uint8_t        broadcast       : 1;        // 1: broadcast, 0: unicast
            uint8_t        secEn           : 1;        // 1: secure the MAC payload, 0: send plain text
            uint8_t        repeat          : 1;        // 1: allow repeaters to forward the message, 0: send message directly
            uint8_t        ackReq          : 1;        // 1: acknowledgement required, 0: no acknowldgement
            uint8_t        destPrsnt       : 1;        // 1: destination address in the packet, 0: destination address not in the packet
            uint8_t        sourcePrsnt     : 1;        // 1: source address in the packet, 0: source address not in the packet
        } bits;
    } flags;
        
    uint8_t        *DestAddress;           // destination address
    #if defined(IEEE_802_15_4)
		bool		altDestAddr;        // use the alternative network address as destination in the packet
        bool		altSrcAddr;         // use the alternative network address as source in the packet
        WORD_VAL    DestPANID;          // PAN identifier of the destination
    #endif

} MAC_TRANS_PARAM;
  
   
typedef struct
{
    union
    {
        uint8_t        Val;  
        struct
        {
            uint8_t    packetType      :2;             // type of packet. Possible types are
                                                    // * PACKET_TYPE_DATA - Data type
                                                    // * PACKET_TYPE_COMMAND -  Command type
                                                    // * PACKET_TYPE_ACK -  Acknowledgement type
                                                    // * PACKET_TYPE_RESERVE - Reserved type
            uint8_t    broadcast       :1;             // 1: broadcast, 0: unicast
            uint8_t    secEn           :1;             // 1: secure the MAC payload, 0: send plain text
            uint8_t    repeat          :1;             // 1: allow repeaters to forward the message, 0: send message directly
            uint8_t    ackReq          :1;             // 1: acknowledgement required, 0: no acknowldgement
            uint8_t    destPrsnt       :1;             // 1: destination address in the packet, 0: destination address not in the packet
            uint8_t    sourcePrsnt     :1;             // 1: source address in the packet, 0: source address not in the packet
        } bits;
    } flags;
            
    uint8_t *      SourceAddress;                      // Address of the Sender
    uint8_t *      Payload;                            // Pointer to the payload
    uint8_t        PayloadLen;                         // Payload size
    int16_t        RSSIValue;                          // RSSI value for the received packet
    uint8_t        LQIValue;                           // LQI value for the received packet
    #if defined(IEEE_802_15_4)
        bool        altSourceAddress;               // Source address is the alternative network address
        WORD_VAL    SourcePANID;                    // PAN ID of the sender
    #endif
} MAC_RECEIVED_PACKET;
        
typedef struct
{
    union
    {
        uint8_t        Val;
        struct
        {
            uint8_t    RepeaterMode    :1;             // 1: Allow to act as repeater, 0: disable repeater function
            uint8_t    CCAEnable       :1;             // 1: Enable Clear Channel Assessment, 0: Disable CCA
            uint8_t    NetworkFreezer  :1;
            uint8_t    PAddrLength     :4;             // Length of the permanent address, range from 1 to 8.
        } bits;   
    } actionFlags;
        
    uint8_t *PAddress;                                 // Permenet transceiver address
        
} MACINIT_PARAM;
    


#define PROTOCOL_HEADER_SIZE 11

#if defined(ENABLE_SECURITY)
	//#define TX_PACKET_SIZE (TX_BUFFER_SIZE+PROTOCOL_HEADER_SIZE+BLOCK_SIZE+MY_ADDRESS_LENGTH+MY_ADDRESS_LENGTH+10)
	//#define RX_PACKET_SIZE (RX_BUFFER_SIZE+PROTOCOL_HEADER_SIZE+MY_ADDRESS_LENGTH+MY_ADDRESS_LENGTH+BLOCK_SIZE+10)
#else
	#define TX_PACKET_SIZE  (TX_BUFFER_SIZE+PROTOCOL_HEADER_SIZE+MY_ADDRESS_LENGTH+MY_ADDRESS_LENGTH+5)
	#define RX_PACKET_SIZE  (RX_BUFFER_SIZE+PROTOCOL_HEADER_SIZE+MY_ADDRESS_LENGTH+MY_ADDRESS_LENGTH+5)
#endif


#if RX_PACKET_SIZE > 255       
	//#warning "RX\_BUFFER\_SIZE RX_BUFFER_SIZE may be defined too big"
	#undef RX_PACKET_SIZE
	#define RX_PACKET_SIZE 255
#endif

#if TX_PACKET_SIZE > 255        
	//#warning "TX\_BUFFER\_SIZE TX_BUFFER_SIZE may be defined too big"
	#undef TX_PACKET_SIZE
	#define TX_PACKET_SIZE 255
#endif

typedef struct
{
	union
	{
		uint8_t    Val;
		struct
		{
			uint8_t    Valid       :1;
			uint8_t    RSSI        :1;
			uint8_t    DQD         :1;
		} bits;
	} flags;
	uint8_t        Payload[RX_PACKET_SIZE];
	uint8_t        PayloadLen;
} RX_PACKET;

typedef struct
{
	bool        Valid;
	uint8_t        Seq;
	uint16_t        CRC;
	MIWI_TICK   startTick;
} ACK_INFO;

typedef union
{
	uint16_t    Val;
	uint8_t    v[2];
	struct
	{
		uint8_t    RSSI_ATS    :1;
		uint8_t    FFEM        :1;
		uint8_t    LBD         :1;
		uint8_t    EXT         :1;
		uint8_t    WKUP        :1;
		uint8_t    RGUR_FFOV   :1;
		uint8_t    POR         :1;
		uint8_t    RG_FF_IT    :1;
		
		uint8_t    filler      :5;
		uint8_t    ATGL        :1;
		uint8_t    CRL         :1;
		uint8_t    DQD         :1;
	} bits;
} TRANSCEIVER_STATUS;


//extern volatile TRANSCEIVER_STATUS   TransceiverStatus;
//extern MAC_RECEIVED_PACKET  MACRxPacket;




#endif /* MIMAC_H_ */