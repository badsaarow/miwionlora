/**
* \file  radio_driver_SX1276.c
*
* \brief This is the Radio drivers SX1276 source file which contains LoRa-specific
*		 Radio drivers functions declarations and defines for SX1276
*
* Copyright (c) 2019 Microchip Technology Inc. and its subsidiaries. 
*
* \asf_license_start
*
* \page License
*
* Subject to your compliance with these terms, you may use Microchip
* software and any derivatives exclusively with Microchip products. 
* It is your responsibility to comply with third party license terms applicable 
* to your use of third party software (including open source software) that 
* may accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, 
* WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, 
* INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, 
* AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE 
* LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL 
* LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE 
* SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE 
* POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT 
* ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY 
* RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, 
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*
* \asf_license_stop
*
*/
/*
* Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
*/

/************************************************************************/
/*  Includes                                                            */
/************************************************************************/
#ifdef UT
#include "lora_test_main.h"
#endif

#include "radio_interface.h"
#include "radio_registers_SX1276.h"
#include "radio_driver_SX1276.h"
#include "radio_driver_hal.h"
#include "radio_get_set.h"
#include "radio_transaction.h"
#include "sw_timer.h"
#include "sys.h"
#include "stdint.h"
#include "string.h"

#include "MiMAC.h"
#include "MiWi.h"
#include "ConfigMiWi.h"
#include "ConfigSX1276.h"

#include "crc.h"

/************************************************************************/
/*  Global variables                                                    */
/************************************************************************/
RadioConfiguration_t radioConfiguration;
#ifdef UT
uint16_t testRssi;
#endif // UT
extern unsigned int MY_PAN_ID;
volatile RX_PACKET          RxPacket[BANK_SIZE];
volatile TRANSCEIVER_STATUS TransceiverStatus;

MACINIT_PARAM               MACInitParams;
uint8_t                        TxMACSeq;
uint8_t                        MACSeq;
uint8_t                        ReceivedBankIndex;

//Temporary
uint8_t rxData[55];

//#if defined(ENABLE_ACK)
	volatile    bool hasAck = false;
	#if defined(ENABLE_RETRANSMISSION)
		ACK_INFO    AckInfo[ACK_INFO_SIZE];
	#endif
//#endif

#if defined(ENABLE_SECURITY)
	uint32_t OutgoingFrameCounter;
	uint8_t key[KEY_SIZE];
#endif

volatile uint8_t	MACTxBuffer[TX_PACKET_SIZE];

int16_t LBT_TR = LBT_BLOCK_DBM;
int32_t LBT_LIMIT = LBT_BLOCK_DBM;

/************************************************************************/
/*  external variables                                                    */
/************************************************************************/
extern volatile RadioCallbackMask_t radioCallbackMask;
extern MAC_RECEIVED_PACKET MACRxPacket;

/************************************************************************/
/*  Static functions                                                    */
/************************************************************************/

/************************************************************************/
/* Implementations                                                      */
/************************************************************************/

/*********************************************************************//**
\brief	This function sets the radioMode and modulation in order to set
		the transceiver to transmit and receive at a set modulation. 

\param newMode			- Sets the transceiver mode.
\param newModulation	- Sets the modulation.
\param blocking			- Sets if its blocking call or not.
\return					- none.
*************************************************************************/
void Radio_WriteMode(RadioMode_t newMode, RadioModulation_t newModulation, uint8_t blocking)
{
#ifndef UT
    uint8_t opMode;
    uint8_t dioMapping;
    RadioModulation_t currentModulation;
    RadioMode_t currentMode;

    if ((MODULATION_FSK == newModulation) &&
        ((MODE_RXSINGLE == newMode) || (MODE_CAD == newMode)))
    {
        // Unavailable modes for FSK. Just return.
        return;
    }

    // Sanity enforcement on parameters
    newMode &= 0x07;
    newModulation &= 0x01;

    opMode = RADIO_RegisterRead(REG_OPMODE);

    if ((opMode & 0x80) != 0)
    {
        currentModulation = MODULATION_LORA;
    }
    else
    {
        currentModulation = MODULATION_FSK;
    }

    currentMode = opMode & 0x07;

    // If we need to change modulation, we need to do this in sleep mode.
    // Otherwise, we can go straight to changing the current mode to newMode.
    if (newModulation != currentModulation)
    {
        // Go to sleep
        if (MODE_SLEEP != currentMode)
        {
            // Clear mode bits, effectively going to sleep
            RADIO_RegisterWrite(REG_OPMODE, opMode & (~0x07));
            currentMode = MODE_SLEEP;
        }
        // Change modulation
        if (MODULATION_FSK == newModulation)
        {
            // Clear MSB and sleep bits to make it stay in sleep
            opMode = opMode & (~0x87);
        }
        else
        {
            // LoRa mode. Set MSB and clear sleep bits to make it stay in sleep
            opMode = 0x80 | (opMode & (~0x87));
        }
        RADIO_RegisterWrite(REG_OPMODE, opMode);
    }

    // From here on currentModulation is no longer current, we will use
    // newModulation instead as it reflects the chip configuration.
    // opMode reflects the actual configuration of the chip.

    if (newMode != currentMode)
    {
        // If we need to block until the mode switch is ready, configure the
        // DIO5 pin to relay this information.
        if ((MODE_SLEEP != newMode) && (1 == blocking))
        {
            dioMapping = RADIO_RegisterRead(REG_DIOMAPPING2);
            if (MODULATION_FSK == newModulation)
            {
                // FSK mode
                dioMapping |= 0x30;     // DIO5 = 11 means ModeReady in FSK mode
            }
            else
            {
                // LoRa mode
                dioMapping &= ~0x30;    // DIO5 = 00 means ModeReady in LoRa mode
            }
            RADIO_RegisterWrite(REG_DIOMAPPING2, dioMapping);
        }

        // Do the actual mode switch.
        opMode &= ~0x07;                // Clear old mode bits
        opMode |= newMode;              // Set new mode bits
        RADIO_RegisterWrite(REG_OPMODE, opMode);

        // If required and possible, wait for switch to complete
        if (1 == blocking)
        {
            if (MODE_SLEEP != newMode)
            {
               // while (HAL_DIO5PinValue() == 0);
			   /* Wait for Mode change to happen */
			   delay_ms(1);
            }
            else
            {
                SystemBlockingWaitMs(1);
            }
        }
    }
#endif
}

/*********************************************************************//**
\brief	This function sets the receive frequency of the transceiver
		while hopping in FHSS.

\param		- none	
\return		- none.
*************************************************************************/
void RADIO_FHSSChangeChannel(void)
{
    uint32_t freq;
    RADIO_RegisterRead(REG_LORA_IRQFLAGS);

    if (radioConfiguration.frequencyHopPeriod)
    {
        if ((radioConfiguration.radioCallback) &&
            (1 == radioCallbackMask.BitMask.radioFhssfreqCallback))
        {
            radioConfiguration.radioCallback(RADIO_FHSS_NEXT_FREQ_CALLBACK, (void *)&freq);
            Radio_WriteFrequency(freq);
        }
    }

    // Clear FHSSChangeChannel interrupt
    RADIO_RegisterWrite(REG_LORA_IRQFLAGS, 1 << SHIFT1);
}

/*********************************************************************//**
\brief	This function generates random number using the transceiver
		and returns it.

\param		- none	
\return		- returns the random number generated.
*************************************************************************/
uint16_t RADIO_ReadRandom(void)
{
    uint8_t i;
    uint16_t retVal;
    retVal = 0;
	
	// Turn on the RF switch.
	Radio_EnableRfControl(RADIO_RFCTRL_RX); 
	// Enabling Radio Clock
	Radio_SetClockInput();
	
    // Mask all interrupts, do many measurements of RSSI
    Radio_WriteMode(MODE_SLEEP, MODULATION_LORA, 1);
    RADIO_RegisterWrite(REG_LORA_IRQFLAGSMASK, 0xFF);
    Radio_WriteMode(MODE_RXCONT, MODULATION_LORA, 1);
    for (i = 0; i < 16; i++)
    {
        SystemBlockingWaitMs(1);
        retVal <<= SHIFT1;
        retVal |= RADIO_RegisterRead(REG_LORA_RSSIWIDEBAND) & 0x01;
    }
	
	// Turning off the RF switch now.
	Radio_DisableRfControl(RADIO_RFCTRL_RX);
	
    // Return radio to sleep
    Radio_WriteMode(MODE_SLEEP, MODULATION_LORA, 1);
    // Clear interrupts in case any have been generated
    RADIO_RegisterWrite(REG_LORA_IRQFLAGS, 0xFF);
    // Unmask all interrupts
    RADIO_RegisterWrite(REG_LORA_IRQFLAGSMASK, 0x00);
	// Disabling Radio Clock save power
	Radio_ResetClockInput();
	
    return retVal;
}

/*********************************************************************//**
\brief This function reads the RSSI value for LoRa.

\param rssi	- The RSSI measured in the channel.
\return		- ERR_NONE. Other types are not used now.
*************************************************************************/
RadioError_t Radio_ReadLoraRssi(int16_t *rssi)
{	
	if (radioConfiguration.frequency >= HF_FREQ_HZ)
	{
#ifdef UT
		*rssi = testRssi;
#else // UT
		*rssi = RSSI_HF_OFFSET + RADIO_RegisterRead(REG_LORA_RSSIVALUE);
#endif // UT		
	}
	else
	{
#ifdef UT
		*rssi = testRssi;
#else // UT
		*rssi = RSSI_LF_OFFSET + RADIO_RegisterRead(REG_LORA_RSSIVALUE);
#endif // UT
	}

	return ERR_NONE;
}

/*********************************************************************//**
\brief This function reads the RSSI value for FSK.

\param rssi	- The RSSI measured in the channel.
\return		- ERR_NONE. Other types are not used now.
*************************************************************************/
RadioError_t Radio_ReadFSKRssi(int16_t *rssi)
{	
#ifdef UT
	*rssi = testRssi;
#else // UT
	*rssi = -(RADIO_RegisterRead(REG_FSK_RSSIVALUE) >> 1);
#endif // UT
	
	return ERR_NONE;
}


RadioError_t Radio_Reset(void)
{
	printf("Radio Reset..........\n\r");

/*	
	NSS_PIN = 1;

	//        FEM_CTX = 0;		// close antenna switch
	//        FEM_CPS = 0;
	
	//        FEM_CTX = 1;    // close tx antenna switch
	//        FEM_CPS = 1;

	PHY_RESETn = 0;
	__delay_ms(20);
	PHY_RESETn = 1;
	//__delay_ms(10);
	__delay_ms(15);
*/
	
	return ERR_NONE;
}

RadioError_t SX1276_Reset(void)
{
	printf("SX1276 RESET...\r\n");
	return ERR_NONE;
}

RadioError_t SX1276LoRaSetNbTrigPeaks( uint8_t value )
{
	uint8_t RegDetectOptimize;
	
	RegDetectOptimize = RADIO_RegisterRead(REG_LORA_DETECTOPTIMIZE);
	RegDetectOptimize = (RegDetectOptimize & 0xF8) | value;
	RADIO_RegisterWrite(REG_LORA_DETECTOPTIMIZE, RegDetectOptimize);
	
	return ERR_NONE;
}

RadioError_t SX1276LoRaSetSpreadingFactor( uint8_t factor )
{
	// SpreadingFactor [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
	unsigned char RegModemConfig2;

	if( factor > 12 ){
		factor = 12;
	}
	else if( factor < 6 ){
		factor = 6;
	}

	if( factor == 6 ){
		SX1276LoRaSetNbTrigPeaks( 5 );
	}
	else{
		SX1276LoRaSetNbTrigPeaks( 3 );
	}

	RegModemConfig2 = RADIO_RegisterRead(REG_LORA_MODEMCONFIG2);
	RegModemConfig2 = (RegModemConfig2 & RFLR_MODEMCONFIG2_SF_MASK ) | ( factor << 4 );
	RADIO_RegisterWrite(REG_LORA_MODEMCONFIG2, RegModemConfig2);

	//printf("Set SpreadingFactor ");printDec(factor);Printf(" Done \r\n");
	
	return ERR_NONE;
}

RadioError_t SX1276LoRaSetSignalBandwidth( uint8_t bw )
{
	unsigned char RegModemConfig1;
	
	RegModemConfig1 = RADIO_RegisterRead(REG_LORA_MODEMCONFIG1);
	RegModemConfig1 = (RegModemConfig1 & RFLR_MODEMCONFIG1_BW_MASK ) | ( bw << 4 );
	RADIO_RegisterWrite( REG_LORA_MODEMCONFIG1, RegModemConfig1);
	
	//Printf("Set Signal Bandwidth ");PrintDec(bw);Printf(" Done \r\n");
	
	return ERR_NONE;
}

uint8_t SX1276LoRaSetRFPower(uint8_t power)
{
	uint8_t paDac;
	uint8_t ocp,rt_val;

	// PA_BOOST pin used for RF output

	// Lower limit
	if (power < 2)
	{
		power = 2;
	}

	// Upper limit
	if (power >= 20)
	{
		power = 20;
	}
	else if (power > 17)
	{
		power = 17;
	}
	
	rt_val = power;

	ocp = RADIO_RegisterRead(REG_OCP);
	paDac = RADIO_RegisterRead(REG_PADAC);
	paDac &= ~(0x07);
	if (power == 20)
	{
		//paDac |= 0x07;
		paDac = 0x87;
		power = 15;//
		
		ocp &= ~(0x20);		//OCP disabled
		
		port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_INACTIVE);
		
		RADIO_RegisterWrite(REG_PADAC, paDac);
		RADIO_RegisterWrite(REG_PACONFIG, 0xF0 | power);
		//RADIO_RegisterWrite(REG_PACONFIG, 0x80 | power);
		RADIO_RegisterWrite(REG_OCP, ocp);
	}
	else
	{
		//paDac |= 0x04;
		paDac = 0x84;
		
		power -= 2;
		ocp |= 0x20;		//OCP enabled, default (Imax = 100mA)
		
		port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_ACTIVE);
		
		RADIO_RegisterWrite(REG_PADAC, paDac);
		//RADIO_RegisterWrite(REG_PACONFIG, 0x00 | power);
		RADIO_RegisterWrite(REG_PACONFIG, 0x70 | power);
		RADIO_RegisterWrite(REG_OCP, ocp);
	}

	
	
	return rt_val;
}

RadioError_t SX1276_Config(void)
{
	uint8_t set_SpreadingFactor = 0;
	uint8_t set_bandwidth = 0;
	
	
	set_bandwidth = 7;
	set_SpreadingFactor = 12;
	
	RADIO_RegisterWrite(REG_OPMODE,0x80);
	
	delay_ms(10);
	
	RADIO_RegisterWrite(REG_TCXO,0x19);                  // external TXCO
	RADIO_RegisterWrite(REG_OPMODE,0x81);
	
	MiMAC_SetChannel(INIT_CHANNEL_NUM);
	
	RADIO_RegisterWrite(REG_LORA_FIFOTXBASEADDR,0x00);
	RADIO_RegisterWrite(REG_LORA_FIFORXBASEADDR,0x80);

	RADIO_RegisterWrite(REG_LNA,0x20);                    // default (LNA gain max, BOOST off)

	RADIO_RegisterWrite(REG_LORA_MODEMCONFIG1,set_bandwidth<<4|0x02);           // BW=set_bandwidth, EC=4/5
	
	if(set_SpreadingFactor == 6 ){
		SX1276LoRaSetNbTrigPeaks( 5 );
	}
	else{
		SX1276LoRaSetNbTrigPeaks( 3 );
	}
	RADIO_RegisterWrite(REG_LORA_MODEMCONFIG2,set_SpreadingFactor<<4 | 0x07);           // SF6~12(set_SpreadingFactor), TxContinuousMode OFF, CRC ON
	
	RADIO_RegisterWrite(REG_LORA_SYMBTIMEOUTLSB,0xFF);         // RX Time-Out LSB 0x64 -> 0xFF
	RADIO_RegisterWrite(REG_LORA_PREAMBLEMSB,0x00);            // Preamble length MSB(PreambleLength + 4.25 Symbols)
	RADIO_RegisterWrite(REG_LORA_PREAMBLELSB,0x08);            // Preamble Length LSB
	RADIO_RegisterWrite(REG_DIOMAPPING1,0x01);           // Mapping of pins DIO0 to DIO5

	RADIO_RegisterWrite(REG_LORA_IRQFLAGSMASK,0x80);

	
	RADIO_RegisterWrite(REG_PARAMP,0x08);                 // 50us PA ramp-up time
	
	RADIO_RegisterWrite(REG_LORA_MODEMCONFIG3,0x0C);
	
	//RADIO_RegisterWrite(REG_PACONFIG,0xFF);				//set max power
	//RADIO_RegisterWrite(REG_OCP,0x3F);					//default (Imax = 100mA)
	//RADIO_RegisterWrite(REG_PADAC,0x87);					//tx power 20dBm
	
	SX1276LoRaSetRFPower(20);
	
	/*------ RegAgcRef: LoRa HIgh Power Setting-------------*/
	RADIO_RegisterWrite(REG_AGCREF,0x19);
	/*------ RegAgcThresh1: Agc thresh 1-------------*/
	RADIO_RegisterWrite(REG_AGCTHRESH1,0x0C);

	/*------ RegAgcThresh1: Agc thresh 2-------------*/
	RADIO_RegisterWrite(REG_AGCTHRESH2,0x4B);

	// SpiRegBuffer[0]=0x85; // LongRangeMode[7] = 0x1, LowFrequencyModeOn[3] = 0x0, Mode[2:0] = 0x0(SLEEP), 0x1(STDBY), 0x3(TX), 0x6(RXSIGLE) //RX Continuous(0x85) //RX Single(0x86)
	// SpiRegBuffer[0]=0x8D; // LongRangeMode[7] = 0x1, LowFrequencyModeOn[3] = 0x1(0x1 might be high frequency), Mode[2:0] = 0x0(SLEEP), 0x1(STDBY), 0x3(TX), 0x6(RXSIGLE)
	RADIO_RegisterWrite(REG_OPMODE,0x85);             // continuous RX mode
	
	HAL_TCXOPowerOn();
	//HAL_EnableRFCtrl(RFO_HF,TX);	
	INTERRUPT_GlobalInterruptEnable();	
	HAL_RegisterDioInterruptHandler(DIO0, SX1276_RX_Handler);
	HAL_EnableDIO0Interrupt();
	
	return ERR_NONE;
}


void intTest1(void){
	uint8_t temp;
	
	temp=RADIO_RegisterRead(REG_LORA_IRQFLAGS);
	printf("INT FLAG 1: %x\n\r",temp);
	RADIO_RegisterWrite(REG_LORA_IRQFLAGS,0xff);
}

void SX1276_RX_Handler(void)
{
	uint8_t temp,pos,i;
	uint8_t PacketLen;
	uint8_t BankIndex;
	uint8_t bAck;
	uint8_t ackPacket[16];
		
	temp=RADIO_RegisterRead(REG_LORA_IRQFLAGS);
	RADIO_RegisterWrite(REG_LORA_IRQFLAGS, 0xff);
	
	if(temp & 0x50){
		PacketLen=RADIO_RegisterRead(REG_LORA_RXNBBYTES);
		pos = RADIO_RegisterRead(REG_LORA_FIFORXCURRENTADDR);	// read RxCurrentaddr
		RADIO_RegisterWrite(REG_LORA_FIFOADDRPTR, pos);		
		BankIndex = 0;
		memset(RxPacket[BankIndex].Payload,0x00,sizeof(RxPacket[BankIndex].Payload));
		RADIO_FrameRead(0x00,(uint8_t *)RxPacket[BankIndex].Payload,PacketLen);

#if 0//#ifdef MOL_DEBUG				
		printf("RXD: ");
		for(i=0;i<PacketLen;i++){
			printf("%.2x",RxPacket[BankIndex].Payload[i]);
		}
		printf("\n\r");
#endif		

/*
		for(BankIndex = 0; BankIndex < BANK_SIZE; BankIndex++)
		{
			if( RxPacket[BankIndex].flags.bits.Valid == false )
			{
				printf("valid Fail.....\n\r");
				break;
			}
		}
*/
		
		// at least PacketLen > 4
		if( PacketLen == 4 )        // may be an acknowledgement
		{
			//printf("ACK!!");
			bAck = true;			
		}
		else
		{
			bAck = false;
		}

		// Check the Packet Length
		if( PacketLen >= RX_PACKET_SIZE || PacketLen == 0 || (BankIndex >= BANK_SIZE && (bAck==false)) )
		{
IGNORE_HERE:
			//nFSEL = 1;	// bad packet len received
			//RFIF = 0;
			
			SX1276_RX_INIT();
			
			goto RETURN_HERE;
		}

		while(1)
		{
			uint16_t received_crc;
			uint16_t calculated_crc;
			uint8_t i;

			if(bAck==false){
				RxPacket[0].PayloadLen = PacketLen;
				//RxPacket[1].PayloadLen = PacketLen;

				// checking CRC
				received_crc = ((uint16_t)(RxPacket[BankIndex].Payload[RxPacket[BankIndex].PayloadLen-1])) + (((uint16_t)(RxPacket[BankIndex].Payload[RxPacket[BankIndex].PayloadLen-2])) << 8);
				
				if((RxPacket[BankIndex].Payload[0] & PACKET_TYPE_MASK) == PACKET_TYPE_RESERVE)
				{
					calculated_crc = CRC16((uint8_t *)RxPacket[BankIndex].Payload, RxPacket[BankIndex].PayloadLen-2, 0);
					//printf("0: %x - %x\n\r",received_crc,calculated_crc);	
				}
				else if( (RxPacket[BankIndex].Payload[0] & BROADCAST_MASK) || (RxPacket[BankIndex].Payload[0] & DSTPRSNT_MASK))
				{
					calculated_crc = CRC16((uint8_t *)RxPacket[BankIndex].Payload, RxPacket[BankIndex].PayloadLen-2, 0);					
					//printf("1: %x - %x\n\r",received_crc,calculated_crc);	
				}
				else
				{
					calculated_crc = CRC16((uint8_t *)RxPacket[BankIndex].Payload, 2, 0);
							
					// try full address first
					calculated_crc = CRC16(MACInitParams.PAddress, MACInitParams.actionFlags.bits.PAddrLength, calculated_crc);
					calculated_crc = CRC16((uint8_t *)&(RxPacket[BankIndex].Payload[2]), RxPacket[BankIndex].PayloadLen-4, calculated_crc);
					//printf("2: %x - %x\n\r",received_crc,calculated_crc);
				}

				//printf("%x - %x\n\r",received_crc,calculated_crc);						
				if( received_crc != calculated_crc )
				{
					//RxPacketPtr = 0;
					RxPacket[BankIndex].PayloadLen = 0;
					//RegisterSet(FIFORSTREG | 0x0002);            // FIFO synchron latch re-enable
					SX1276_RX_INIT();

					//printf("RX CRC Fail\n\r");

					goto IGNORE_HERE;
				}
						
				#if !defined(TARGET_SMALL)
				RxPacket[BankIndex].flags.bits.DQD = 1;
				RxPacket[BankIndex].flags.bits.RSSI = TransceiverStatus.bits.RSSI_ATS;
				#endif
			}
			
			//printf("CRC CHK OK...\n\r");
/*						
			// send ack / check ack
			#if defined(ENABLE_ACK)
			
			if( ( RxPacket[BankIndex].Payload[0] & PACKET_TYPE_MASK ) == PACKET_TYPE_ACK )  // acknowledgement
			{
				if( RxPacket[BankIndex].Payload[1] == TxMACSeq )
				{
					hasAck = true;
				}
							
				//RxPacketPtr = 0;
				RxPacket[BankIndex].PayloadLen = 0;
				//RegisterSet(FIFORSTREG | 0x0002);
				SX1276_RX_INIT();
			}
			else
			#endif
*/			
			//printf("CHK: %x\r\n",RxPacket[BankIndex].Payload[0]);

			if( ( RxPacket[BankIndex].Payload[0] & PACKET_TYPE_MASK ) == PACKET_TYPE_ACK )  // acknowledgement
			{
				//printf("ACK Receviced....\n\r");
				if( RxPacket[BankIndex].Payload[1] == TxMACSeq )
				{
					//printf("ACK Receviced....\n\r");
					hasAck = true;
				}
	
				//RxPacketPtr = 0;
				RxPacket[BankIndex].PayloadLen = 0;
				//RegisterSet(FIFORSTREG | 0x0002);
				SX1276_RX_INIT();
			}
			else
			{
				uint8_t ackInfoIndex = 0xFF;
							
				if( RxPacket[BankIndex].Payload[0] & DSTPRSNT_MASK )
				{
					//printf("DSTPRSNT CHK 1...\n\r");
					for(i = 0; i < MACInitParams.actionFlags.bits.PAddrLength; i++)
					{
						if( RxPacket[BankIndex].Payload[2+i] != MACInitParams.PAddress[i] )
						{
							//printf("DSTPRSNT CHK 2...\n\r");
							//RxPacketPtr = 0;
							RxPacket[BankIndex].PayloadLen = 0;
							//RegisterSet(FIFORSTREG | 0x0002);
							SX1276_RX_INIT();
							goto IGNORE_HERE;
						}
					}
				}
				
				
/*						
				#if defined(ENABLE_ACK)
				if( (RxPacket[BankIndex].Payload[0] & ACK_MASK) )  // acknowledgement required
				{
					//RegisterSet(FIFORSTREG | 0x0002);
								
					for(i = 0; i < 4; i++)
					{
						ackPacket[i] = MACTxBuffer[i];
					}
					MACTxBuffer[0] = PACKET_TYPE_ACK | BROADCAST_MASK;   // frame control, ack type + broadcast
					MACTxBuffer[1] = RxPacket[BankIndex].Payload[1];     // sequenece number
					calculated_crc = CRC16((uint8_t *)MACTxBuffer, 2, 0);
					MACTxBuffer[2] = calculated_crc>>8;
					MACTxBuffer[3] = calculated_crc;
					
					delay_ms(2);
					
					TxPacket(4, false);
				
					//RegisterSet(FIFORSTREG);
					
					for(i = 0; i < 4; i++)
					{
						MACTxBuffer[i] = ackPacket[i];
					}
				}
				#endif
											
				#if defined(ENABLE_ACK) && defined(ENABLE_RETRANSMISSION)
				
				for(i = 0; i < ACK_INFO_SIZE; i++)
				{
					if( AckInfo[i].Valid && (AckInfo[i].Seq == RxPacket[BankIndex].Payload[1]) &&
					AckInfo[i].CRC == received_crc )
					{
						AckInfo[i].startTick = MiWi_TickGet();
						break;
					}
					if( (ackInfoIndex == 0xFF) && (AckInfo[i].Valid == false) )
					{
						ackInfoIndex = i;
					}
				}
				if( i >= ACK_INFO_SIZE )
				{
					if( ackInfoIndex < ACK_INFO_SIZE )
					{
						AckInfo[ackInfoIndex].Valid = true;
						AckInfo[ackInfoIndex].Seq = RxPacket[BankIndex].Payload[1];
						AckInfo[ackInfoIndex].CRC = received_crc;
						AckInfo[ackInfoIndex].startTick = MiWi_TickGet();
					}

					RxPacket[BankIndex].PayloadLen -= 2;            // remove CRC
					RxPacket[BankIndex].flags.bits.Valid = true;
				}
				#else
*/					
				if( (RxPacket[BankIndex].Payload[0] & ACK_MASK) )  // acknowledgement required
				{
					//RegisterSet(FIFORSTREG | 0x0002);
					
					//Check for me
					if((RxPacket[BankIndex].Payload[0] & PACKET_TYPE_MASK) == PACKET_TYPE_RESERVE){							
						
						if (RxPacket[BankIndex].Payload[4] != myPANID.v[0] ||
								RxPacket[BankIndex].Payload[5] != myPANID.v[1] ||
								RxPacket[BankIndex].Payload[6] != myShortAddress.v[0] ||
								RxPacket[BankIndex].Payload[7] != myShortAddress.v[1] )
						{
							//printf("It is not for me\n\r");
							break;		
						}
					}
					
					for(i = 0; i < 4; i++)
					{
						ackPacket[i] = MACTxBuffer[i];
					}
					MACTxBuffer[0] = PACKET_TYPE_ACK | BROADCAST_MASK;   // frame control, ack type + broadcast
					MACTxBuffer[1] = RxPacket[BankIndex].Payload[1];     // sequenece number
					calculated_crc = CRC16((uint8_t *)MACTxBuffer, 2, 0);
					MACTxBuffer[2] = calculated_crc>>8;
					MACTxBuffer[3] = calculated_crc;
					
					delay_ms(2);
					
					//printf("ACK Sending....");
					SX1276_TX((uint8_t *)MACTxBuffer, 4);
					
					//RegisterSet(FIFORSTREG);
					
					for(i = 0; i < 4; i++)
					{
						MACTxBuffer[i] = ackPacket[i];
					}
				}

		
				RxPacket[BankIndex].PayloadLen -= 2;            // remove CRC
				RxPacket[BankIndex].flags.bits.Valid = true;
				#if !defined(TARGET_SMALL)
				RxPacket[BankIndex].flags.bits.RSSI = TransceiverStatus.bits.RSSI_ATS;
				RxPacket[BankIndex].flags.bits.DQD = TransceiverStatus.bits.DQD;
				#endif
				
//				#endif

				//RegisterSet(FIFORSTREG | 0x0002);
				SX1276_RX_INIT();
							
			}
							
			goto RETURN_HERE;
		}					
	
RETURN_HERE:
		
		return;
		//counter = 0;
		//RFIF = 0;
		
	}
}

RadioError_t SX1276_RX_INIT(void)
{
    unsigned char addr;
	
	RADIO_RegisterWrite(REG_OPMODE,0x81);
	delay_ms(1);
	
    RADIO_RegisterWrite(REG_LORA_IRQFLAGS,0xff);                   // clear interrupt
	//RADIO_RegisterWrite(REG_LORA_IRQFLAGSMASK,0x3f);               // enable rx done and rx timeout
	RADIO_RegisterWrite(REG_LORA_IRQFLAGSMASK,0x0f);               // enable rx done and rx timeout
	
	RADIO_RegisterWrite(REG_DIOMAPPING1,0x01);						//DIO0=00, DIO1=00, DIO2=00, DIO3=01  DIO0=00--RXDONE
    RADIO_RegisterWrite(REG_DIOMAPPING1,0b0100001);			//DIO0=00, DIO1=00, DIO2=00, DIO3=01  DIO0=00--RXDONE

    addr = RADIO_RegisterRead(REG_LORA_FIFORXBASEADDR);            // read RxBaseAddr
    RADIO_RegisterWrite(REG_LORA_FIFOADDRPTR,0x00);                // RxBaseAddr->FifoAddrPtr
	
	
    RADIO_RegisterWrite(REG_OPMODE,0x85);                     // enter rx continuous mode
	
	return ERR_NONE;
}

uint8_t SX1276_RX(void)
{
	uint8_t temp,pos;
	port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_ACTIVE);//kwlee rx mode 210215
	temp=RADIO_RegisterRead(REG_LORA_IRQFLAGS);
	if(temp & 0x40){
		RADIO_RegisterWrite(REG_LORA_IRQFLAGS, 0xff);
		temp=RADIO_RegisterRead(REG_LORA_RXNBBYTES);
		//printf("RX Len: %x\n\r",temp);
	
		memset(rxData,0x00,sizeof(rxData));
	
		pos = RADIO_RegisterRead(REG_LORA_FIFORXCURRENTADDR);	// read RxCurrentaddr
		RADIO_RegisterWrite(REG_LORA_FIFOADDRPTR, pos);
	
		RADIO_FrameRead(0x00,rxData,temp);
		
		return temp;
	}
	
	return 0;
}
    
/**************************************************************************/
RadioError_t SX1276_TX (const int8_t *line, uint8_t len)
{
    uint8_t addr,temp;

    RADIO_RegisterWrite(REG_LORA_IRQFLAGS,0xff);                   // clear interrupt
    RADIO_RegisterWrite(REG_LORA_IRQFLAGSMASK,0xf7);               // enable txdone
	
    RADIO_RegisterWrite(REG_DIOMAPPING1,0b00000001); 		// DIO0=TxDone,DIO1=RxTimeout,DIO3=ValidHeader

    RADIO_RegisterWrite(REG_LORA_PAYLOADLENGTH, len);              // packet length
	
    addr = RADIO_RegisterRead(REG_LORA_FIFOTXBASEADDR);           // read TxBaseAddr	
    RADIO_RegisterWrite(REG_LORA_FIFOADDRPTR,addr);                // TxBaseAddr->FifoAddrPtr
		
	delay_ms(1);

    RADIO_RegisterWrite(REG_OPMODE,0x81);                        // LoRa mode and standby mode 0x09 -> 0x81
 	RADIO_FrameWrite(0x00,line,len);
	RADIO_RegisterWrite(REG_OPMODE,0x83);                     // enter tx mode 0x0b -> 0x03

    temp=RADIO_RegisterRead(REG_LORA_IRQFLAGS);                    // read interrupt
	
	while(!(temp&0x08))                                 // wait for tx done
    {
        temp=RADIO_RegisterRead(REG_LORA_IRQFLAGS);
    }
    	    
    RADIO_RegisterWrite(REG_LORA_IRQFLAGS,0xff);			// clear interrupt
	//RADIO_RegisterWrite(REG_OPMODE,0x85);                     // enter rx continuous mode
	//RADIO_RegisterWrite(REG_OPMODE,0x09);  			// enter standby mode
	
	return ERR_NONE;
}


    
    /**************************************************************************/
	
bool TxPacket(uint8_t TxPacketLen, bool CCA)
{
	bool status,tmp;
	MIWI_TICK t1, t2;
	uint8_t retry_count;
		
	#ifdef ENABLE_CCA
	uint8_t CCARetries;
	#endif
	
	#if defined(ENABLE_ACK) && defined(ENABLE_RETRANSMISSION)
	uint8_t reTry = RETRANSMISSION_TIMES;
	#endif
	
	#if defined(ENABLE_ACK) && defined(ENABLE_RETRANSMISSION)
	while( reTry-- )
	#endif
	{
		
		#if defined(ENABLE_ACK)
		hasAck = false;
		#endif
		
		#ifdef ENABLE_CCA
		CCARetries = 0;
		
		if( CCA )
		{
			RegisterSet(PMCREG | 0x0080);
			Start_CCA:
			DQDOK = 0;
			for(i = 0; i < CCA_TIMES; i++)
			{
				StatusRead();
				if( TransceiverStatus.bits.DQD )
				{
					DQDOK++;
				}
				Delay10us(1);
			}
			
			if( DQDOK > CCA_THRESHOLD )
			{
				if(CCARetries++ > CCA_RETRIES )
				{
					return FALSE;
				}
				goto Start_CCA;
			}
		}
		#endif
		
		//RFIE = 0;
		
		
		#ifdef LBT_EN
		
		//================
		//LBT Check
		//================
		retry_count = 0;
		while(!SX1276_TX_LBT())
		{
			//LBT Scan fail
			retry_count++;
			if(retry_count>=LBT_RETRY_TX)
			{
				status = false;
				break;
			}
			
			//Retry delay
			tmp = 0;
			t2 = MiWi_TickGet();
			do
			{
				//if(tmp==0){tmp = 1; printf("LBT ReTry...\r\n");}		//test only
				t1 = MiWi_TickGet();
			}
			while( MiWi_TickGetDiff(t1, t2) < LBT_RETRY_INTERVAL);
		}
		
		if(retry_count < LBT_RETRY_TX)
		{
			//================
			//Transmit
			//================
			SX1276_TX((uint8_t *)MACTxBuffer, TxPacketLen);
			SX1276_RX_INIT();
			/*
			t2 = MiWi_TickGet();			
			t1 = MiWi_TickGet();
			if( MiWi_TickGetDiff(t1, t2) >= LBT_TRANS_TIME )
			{
				//transmit time over
				printf("Idle time (4sec) over.. \r\n");
			}
			*/
			
			//================
			//SET RX & Delay
			//================
			t2 = MiWi_TickGet();			
			do
			{
				t1 = MiWi_TickGet();
			}
			while( MiWi_TickGetDiff(t1, t2) < LBT_DELAY );
			
			status = true;
		}
		
		#else
		
		SX1276_TX((uint8_t *)MACTxBuffer, TxPacketLen);
		SX1276_RX_INIT();
		status = true;
		
		#endif

		//RFIE = 1;
	}
	
	return status;
}	
	
	
#ifdef LBT_EN

bool SX1276_TX_LBT(void)
{
	int16_t RSSI_buf,RSSI_avg;
	uint16_t rssi_check_count;
	int32_t rssi_sum, tmp;
	MIWI_TICK t1, t2;
		
	rssi_sum = 0;
	rssi_check_count = 0;
		
	t2 = MiWi_TickGet();
	while(1)
	{
        RSSI_buf = MiMAC_ChannelAssessment(CHANNEL_ASSESSMENT_ENERGY_DETECT);
		rssi_sum += RSSI_buf;
		rssi_check_count++;
			
		t1 = MiWi_TickGet();
		if( MiWi_TickGetDiff(t1, t2) > LBT_CHK_TIME*5 )
		{
			//scan timeout
			break;
		}
	}
		
	RSSI_avg = rssi_sum/rssi_check_count;
	
	//tmp = LBT_BLOCK_DBM;	
    tmp = LBT_LIMIT;
	    
    //PrintDec(rssi_check_count);Printf("|");PrintDec(RSSI_avg*-1);Printf("|");PrintDec(tmp*-1);Printf("\n\r");
    //printf("CNT: %d | RSSIAVG: %d | TH: %d \n\r",rssi_check_count,RSSI_avg,tmp);    
		
    //if((LONG)RSSI_avg <= (LONG)LBT_TR)           
    if(tmp > RSSI_avg)
	{
		//TX Ok
        //printf("LBT OK..\r\n");
		return true;
	}
	else
	{
		//TX fail
        //printf("LBT Fail..\r\n");
		return false;
	}
}
	
#endif

bool MiMAC_SetChannel(uint8_t channel)
{
	uint32_t freq;

	freq = (uint32_t)((double)(LRFREQ_START+ ((channel-1) * LRFREQ_OFFSET)) / (double)LRFREQ_STEP);

	RADIO_RegisterWrite(REG_FRFMSB,(uint8_t)((freq >> 16) & 0xFF));
	RADIO_RegisterWrite(REG_FRFMID,(uint8_t)((freq >> 8) & 0xFF));
	RADIO_RegisterWrite(REG_FRFLSB,(uint8_t)(freq & 0xFF));

	delay_ms(20);                                // delay 20 ms to stablize the transceiver
	      
    return true;   
}
    
bool MiMAC_SetPower(uint8_t outputPower)
{
    SX1276LoRaSetRFPower(outputPower);
	
    return true;
}
    
bool MiMAC_Init(MACINIT_PARAM initValue)
{
    uint8_t i;
         
    MACInitParams = initValue;
 
//SPI INIT           
//PHY_CS = 1;           // nSEL inactive
//nFSEL = 1;           // nFFS inactive
//SPI_SDO = 0;        
//SPI_SCK = 0;        
    
    MACSeq = 0x00;
	
    ReceivedBankIndex = 0xFF;
        
    for(i = 0; i < BANK_SIZE; i++)
    {
        RxPacket[i].flags.Val = 0;
    }
        
    #if defined(ENABLE_ACK) && defined(RETRANSMISSION)
        for(i = 0; i < ACK_INFO_SIZE; i++)
        {
            AckInfo[i].Valid = false;
        }
    #endif
    
    #if defined(ENABLE_SECURITY)
        #if defined(ENABLE_NETWORK_FREEZER)
            if( initValue.actionFlags.bits.NetworkFreezer )
            {
                nvmGetOutFrameCounter(OutgoingFrameCounter.v);
                OutgoingFrameCounter.Val += FRAME_COUNTER_UPDATE_INTERVAL;
                nvmPutOutFrameCounter(OutgoingFrameCounter.v);
            }
            else
            {
                OutgoingFrameCounter.Val = 0;
                nvmPutOutFrameCounter(OutgoingFrameCounter.v);
                OutgoingFrameCounter.Val = 1;
            }        
        #else
        	OutgoingFrameCounter.Val = 1;
        #endif
        	
        for(i = 0; i < KEY_SIZE; i++)
        {
            key[i] = mySecurityKey[i];
        }
    #endif
    
    return true;
}
    

bool MiMAC_SendPacket(  MAC_TRANS_PARAM transParam, uint8_t *MACPayload, uint8_t MACPayloadLen)
{
#if 1	
    uint8_t i;
    uint8_t TxIndex;
    uint16_t crc;
   
    if( MACPayloadLen > TX_BUFFER_SIZE )
    {
        return false;
    }

    #if defined(INFER_DEST_ADDRESS)
        //transParam.flags.bits.destPrsnt = 0;
    #else
        //transParam.flags.bits.destPrsnt = (transParam.flags.bits.broadcast) ? 0:1;
    #endif
        
    #if !defined(SOURCE_ADDRESS_ABSENT)
        //transParam.flags.bits.sourcePrsnt = 1;
    #endif
    

	    
    if( transParam.flags.bits.packetType == PACKET_TYPE_COMMAND )
    {
        transParam.flags.bits.sourcePrsnt = 1;
    }
      
    MACTxBuffer[0] = transParam.flags.Val;
	
#if 0	
	printf("CHK VAL: %x\n\r",MACTxBuffer[0]);
	
	printf("ACK : %x\n\r",transParam.flags.bits.ackReq);
	printf("BCAST : %x\n\r",transParam.flags.bits.broadcast);
	printf("DST : %x\n\r",transParam.flags.bits.destPrsnt);
	printf("PTYPE : %x\n\r",transParam.flags.bits.packetType);
	printf("RPT : %x\n\r",transParam.flags.bits.repeat);
	printf("SEC : %x\n\r",transParam.flags.bits.secEn);
	printf("SRC : %x\n\r",transParam.flags.bits.sourcePrsnt);
#endif
	
    MACTxBuffer[1] = MACSeq++;
    
	TxMACSeq = MACTxBuffer[1];
	
	crc = CRC16((uint8_t *)MACTxBuffer, 2, 0);
     
    TxIndex = 2;
      
    if( transParam.flags.bits.destPrsnt )
    {   
        for(i = 0; i < MACInitParams.actionFlags.bits.PAddrLength; i++)
        {
            MACTxBuffer[TxIndex++] = transParam.DestAddress[i];
        }
    }
    if( transParam.flags.bits.broadcast == 0 )
    {
		if (transParam.flags.bits.packetType != PACKET_TYPE_RESERVE){
			crc = CRC16(transParam.DestAddress, MACInitParams.actionFlags.bits.PAddrLength, crc);
		}
    }
    
	if( transParam.flags.bits.sourcePrsnt )
    {   
        for(i = 0; i < MACInitParams.actionFlags.bits.PAddrLength; i++)
        {
            MACTxBuffer[TxIndex++] = MACInitParams.PAddress[i];
        }
		crc = CRC16((uint8_t *)&(MACTxBuffer[TxIndex-MACInitParams.actionFlags.bits.PAddrLength]), MACInitParams.actionFlags.bits.PAddrLength, crc);
    }
    
    #if 0//defined(ENABLE_SECURITY)
        if( transParam.flags.bits.secEn )
        {
    
            for(i = 0; i < 4; i++)
            {
                MACTxBuffer[TxIndex++] = OutgoingFrameCounter.v[i];
            }
            OutgoingFrameCounter.Val++;
            #if defined(ENABLE_NETWORK_FREEZER)
                if( (OutgoingFrameCounter.v[0] == 0) && ((OutgoingFrameCounter.v[1] & 0x03) == 0) )
                {
                    nvmPutOutFrameCounter(OutgoingFrameCounter.v);
                }    
            #endif
            MACTxBuffer[TxIndex++] = KEY_SEQUENCE_NUMBER;
    
            {
                uint8_t headerLen;
                    
                headerLen = TxIndex;
                    
                for(i = 0; i < MACPayloadLen; i++)
                {
                    MACTxBuffer[TxIndex++] = MACPayload[i];
                }
                    
                #if SECURITY_LEVEL == SEC_LEVEL_CTR
                    {
                        uint8_t nounce[BLOCK_SIZE];
                            
                        for(i = 0; i < BLOCK_SIZE; i++)
                        {
                            if( i < TxIndex )
                            {
                                nounce[i] = MACTxBuffer[i];
                            }
                            else
                            {
                                nounce[i] = 0;
                            }
                        }
                        CTR(&(MACTxBuffer[headerLen]), MACPayloadLen, key, nounce);
                    }
                #elif (SECURITY_LEVEL == SEC_LEVEL_CCM_64) || (SECURITY_LEVEL == SEC_LEVEL_CCM_32) || (SECURITY_LEVEL == SEC_LEVEL_CCM_16)
                    CCM_Enc((BYTE *)MACTxBuffer, headerLen, MACPayloadLen, key);
                    TxIndex += SEC_MIC_LEN;
                #elif (SECURITY_LEVEL == SEC_LEVEL_CBC_MAC_64) || (SECURITY_LEVEL == SEC_LEVEL_CBC_MAC_32) || (SECURITY_LEVEL == SEC_LEVEL_CBC_MAC_16)
                    CBC_MAC(MACTxBuffer, TxIndex, key, &(MACTxBuffer[TxIndex]));
                    TxIndex += SEC_MIC_LEN;
                #endif
                    
                crc = CRC16((BYTE *)&(MACTxBuffer[headerLen-5]), (MACPayloadLen + SEC_MIC_LEN + 5), crc);
            }
        }
        else
    
    #endif
    
    {
        for(i = 0; i < MACPayloadLen; i++)
        {
            MACTxBuffer[TxIndex++] = MACPayload[i];
        }
            
        crc = CRC16(MACPayload, MACPayloadLen, crc);
    }    
    
	//printf("TX CRC: %.4x",crc);
	
    MACTxBuffer[TxIndex++] = *(((uint8_t *)&crc)+1);	//(uint8_t)(crc>>8); 
    MACTxBuffer[TxIndex++] = (uint8_t)crc;

    return TxPacket(TxIndex, MACInitParams.actionFlags.bits.CCAEnable);
#else
	return 0;	
#endif
}
    

bool MiMAC_ReceivedPacket(void)
{
#if 1	
    uint8_t i,tmp;
    MIWI_TICK currentTick;

/*        
    tmp = SPIReadReg(LR_RegIrqFlags);
    if(RF_INT_PIN || (tmp & 0x40))
    {
        RFIF = 1;
        NOP();
    }
*/
        
    #if defined(ENABLE_ACK) && defined(ENABLE_RETRANSMISSION)
        for(i = 0; i < ACK_INFO_SIZE; i++)
        {
            currentTick = MiWi_TickGet();
            if( AckInfo[i].Valid && (currentTick.Val > AckInfo[i].startTick.Val) && 
                (MiWi_TickGetDiff(currentTick, AckInfo[i].startTick) > ONE_SECOND) )
            {
                AckInfo[i].Valid = false;
            }
        }
    #endif
            
    if( ReceivedBankIndex != 0xFF )
    {
	
        return false;
    }
       
    for(i = 0; i < BANK_SIZE; i++)
    {
        if( RxPacket[i].flags.bits.Valid )
        {
            uint8_t PayloadIndex;
            uint8_t j;

            MACRxPacket.flags.Val = RxPacket[i].Payload[0];
            MACRxPacket.PayloadLen = RxPacket[i].PayloadLen;
            PayloadIndex = 2;
                
            if( MACRxPacket.flags.bits.destPrsnt )
            {
                PayloadIndex += MACInitParams.actionFlags.bits.PAddrLength;
            }
                
            if( MACRxPacket.flags.bits.sourcePrsnt )
            {
                MACRxPacket.SourceAddress = (uint8_t *)&(RxPacket[i].Payload[PayloadIndex]);
                PayloadIndex += MACInitParams.actionFlags.bits.PAddrLength;
            }
            else
            {
                MACRxPacket.SourceAddress = NULL;
            }
  
/*                
            #if 0 //defined(ENABLE_SECURITY)
                if( MACRxPacket.flags.bits.secEn )
                {
                    // check key sequence number first
                    if( KEY_SEQUENCE_NUMBER != RxPacket[i].Payload[PayloadIndex+4] )    
                    {
                        RxPacket[i].flags.Val = 0;
                        return false;
                    }
                        
                    // check frame counter now
                    if( MACRxPacket.flags.bits.sourcePrsnt )
                    {
    
                        for(j = 0; j < CONNECTION_SIZE; j++)
                        {
                            if( (ConnectionTable[j].status.bits.isValid) && 
                                isSameAddress(ConnectionTable[j].Address, MACRxPacket.SourceAddress) )
                            {
                                break;
                            }
                        }
                        if( j < CONNECTION_SIZE )
                        {
                            DWORD_VAL FrameCounter;
                            uint8_t k;
                                
                            for(k = 0; k < 4; k++)
                            {
                                FrameCounter.v[k] = RxPacket[i].Payload[PayloadIndex+k];    
                            }
                                
                            if( IncomingFrameCounter[j].Val > FrameCounter.Val )
                            {
                                RxPacket[i].flags.Val = 0;
                                return false;
                            }
                            else
                            {
                                IncomingFrameCounter[j].Val = FrameCounter.Val;
                            }
                        }
                    }
                        
                    // now decrypt the data
                    PayloadIndex += 5;      // bypass the frame counter and key sequence number
                        
                    #if SECURITY_LEVEL == SEC_LEVEL_CTR
                        {
                            uint8_t nounce[BLOCK_SIZE];
                                
                            for(j = 0; j < BLOCK_SIZE; j++)
                            {
                                if( j < PayloadIndex )
                                {
                                    nounce[j] = RxPacket[i].Payload[j];
                                }
                                else
                                {
                                    nounce[j] = 0;
                                }
                            }
                                
                            CTR(&(RxPacket[i].Payload[PayloadIndex]), (RxPacket[i].PayloadLen - PayloadIndex), key, nounce); 
                        }
                    #elif (SECURITY_LEVEL == SEC_LEVEL_CCM_64) || (SECURITY_LEVEL == SEC_LEVEL_CCM_32) || (SECURITY_LEVEL == SEC_LEVEL_CCM_16)

                        if(CCM_Dec((BYTE *)RxPacket[i].Payload, PayloadIndex, RxPacket[i].PayloadLen-PayloadIndex, key) == FALSE)
                        {
                            RxPacket[i].flags.Val = 0;
                            return FALSE;
                        }

                    #elif (SECURITY_LEVEL == SEC_LEVEL_CBC_MAC_64) || (SECURITY_LEVEL == SEC_LEVEL_CBC_MAC_32) || (SECURITY_LEVEL == SEC_LEVEL_CBC_MAC_16)
                        {
                            BYTE MIC[BLOCK_SIZE];
                                
                            CBC_MAC(RxPacket[i].Payload, (RxPacket[i].PayloadLen - SEC_MIC_LEN), key, MIC);
                            for(j = 0; j < SEC_MIC_LEN; j++)
                            {
                                if( MIC[j] != RxPacket[i].Payload[RxPacket[i].PayloadLen-SEC_MIC_LEN+j] )
                                {
                                    RxPacket[i].flags.Val = 0;
                                    return FALSE;
                                }    
                            }
                        }
                    #endif
                    MACRxPacket.PayloadLen -= (PayloadIndex + SEC_MIC_LEN);
                }   
                else
                {
                    MACRxPacket.PayloadLen -= PayloadIndex;
                }
    
            #else
                
                MACRxPacket.PayloadLen -= PayloadIndex;
                
            #endif
*/                
            MACRxPacket.Payload = (uint8_t *)&(RxPacket[i].Payload[PayloadIndex]);
            #if !defined(TARGET_SMALL)
                if( RxPacket[i].flags.bits.RSSI )
                {
                    #if RSSI_THRESHOLD == RSSI_THRESHOLD_103
                        MACRxPacket.RSSIValue = 1;
                    #elif RSSI_THRESHOLD == RSSI_THRESHOLD_97
                        MACRxPacket.RSSIValue = 7;
                    #elif RSSI_THRESHOLD == RSSI_THRESHOLD_91
                        MACRxPacket.RSSIValue = 13;
                    #elif RSSI_THRESHOLD == RSSI_THRESHOLD_85
                        MACRxPacket.RSSIValue = 19;
                    #elif RSSI_THRESHOLD == RSSI_THRESHOLD_79
                        MACRxPacket.RSSIValue = 25;
                    #elif RSSI_THRESHOLD == RSSI_THRESHOLD_73
                        MACRxPacket.RSSIValue = 31;
                    #endif
                }
                else
                {
                    MACRxPacket.RSSIValue = 0;
                }
                MACRxPacket.LQIValue = RxPacket[i].flags.bits.DQD;
            #endif
            ReceivedBankIndex = i;
             
            return true;
        }
    }
    
    return false;    
#else
	return 0;
#endif	
}
     
void MiMAC_DiscardPacket(void)
{
	if( ReceivedBankIndex < BANK_SIZE )
	{
		RxPacket[ReceivedBankIndex].flags.Val = false;
		ReceivedBankIndex = 0xFF;
	}
}

int16_t MiMAC_ChannelAssessment(uint8_t AssessmentMode)
{
	int16_t tmp;
	if(AssessmentMode == CHANNEL_ASSESSMENT_CARRIER_SENSE)
	{
		tmp = RADIO_RegisterRead(REG_LORA_PKTRSSIVALUE);		
		tmp += RSSI_OFFSET_HF;
		//printf("PKTRSSI: %d\r\n",tmp);
		return tmp;
	}
	else
	{
		tmp = RADIO_RegisterRead(REG_LORA_RSSIVALUE);		
		tmp += RSSI_OFFSET_HF;
		//printf("RSSIVAL: %d\r\n",tmp);
		return tmp;//(double)SX1276LR->RegRssiValue;
	}
}
  
bool MiMAC_SetAltAddress(uint8_t *Address, uint8_t *PANID)
{
	return false;
}

/**
 End of File
 */
