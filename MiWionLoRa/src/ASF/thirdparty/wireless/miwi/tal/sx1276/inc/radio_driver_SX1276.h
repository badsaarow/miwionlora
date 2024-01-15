/**
* \file  radio_driver_SX1276.h
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

/******************************************************************************
  @Company:
    Microchip Technology Inc.

  @File Name:
    radio_driver_SX1276.h

  @Summary:
    This is the Radio Driver SX1276 header file which contains LoRa-specific
    Radio Driver functions declarations and defines for SX1276

  @Description:
    This header file provides LoRa-specific implementations for Radio Driver for SX1276.
    Copyright (c) 2013 - 2016 released Microchip Technology Inc.  All rights reserved.

    Microchip licenses to you the right to use, modify, copy and distribute
    Software only when embedded on a Microchip microcontroller or digital signal
    controller that is integrated into your product or third party product
    (pursuant to the sublicense terms in the accompanying license agreement).

    You should refer to the license agreement accompanying this Software for
    additional information regarding your rights and obligations.

    SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
    EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
    MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
    IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
    CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
    OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
    CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
    SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *************************************************************************
 *                           radio_driver_SX1276.h
 *
 * Radio Driver SX1276 header file
 *
 ******************************************************************************/

#ifndef RADIO_DRIVER_H
#define	RADIO_DRIVER_H

#ifdef	__cplusplus
extern "C" {
#endif
    
/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include "radio_interface.h"
#include <stdint.h>
#include <stddef.h>

#include "MiMAC.h"

/************************************************************************/
/* Defines                                                              */
/************************************************************************/
#define ENABLED						1
#define DISABLED					0
// The Freq greater than 862Mhz is HF frequency 
#define HF_FREQ_HZ					862000000UL
// For freq above 862Mhz the RSSI offset to be used
#ifndef RSSI_HF_OFFSET
#define RSSI_HF_OFFSET				-157
#endif
// For freq below 862Mhz the RSSI offset to be used
#ifndef RSSI_LF_OFFSET
#define RSSI_LF_OFFSET				-164
#endif

/************************************************************************/
/* Types                                                                */
/************************************************************************/

/************************************************************************/
/* External variables                                                   */
/************************************************************************/
extern RadioConfiguration_t radioConfiguration;

/************************************************************************/
/* Prototypes                                                           */
/************************************************************************/
/*********************************************************************//**
\brief This function reads the RSSI value for LoRa.

\param rssi	- The RSSI measured in the channel.
\return		- ERR_NONE. Other types are not used now.
*************************************************************************/
RadioError_t Radio_ReadLoraRssi(int16_t *rssi);

/*********************************************************************//**
\brief This function reads the RSSI value for FSK.

\param rssi	- The RSSI measured in the channel.
\return		- ERR_NONE. Other types are not used now.
*************************************************************************/
RadioError_t Radio_ReadFSKRssi(int16_t *rssi);


//Brett
RadioError_t Radio_Reset(void);
RadioError_t SX1276_Reset(void);
RadioError_t SX1276_Config(void);
RadioError_t SX1276LoRaSetNbTrigPeaks( uint8_t value );
RadioError_t SX1276LoRaSetSpreadingFactor( uint8_t factor );
RadioError_t SX1276LoRaSetSignalBandwidth( uint8_t bw );
uint8_t SX1276LoRaSetRFPower(uint8_t power);
RadioError_t SX1276_RX_INIT(void);
RadioError_t SX1276_TX (const int8_t *line, uint8_t len);
bool SX1276_TX_LBT(void);
bool MiMAC_SetChannel(uint8_t channel);
bool MiMAC_SetPower(uint8_t outputPower);
bool MiMAC_Init(MACINIT_PARAM initValue);
bool MiMAC_SendPacket(  MAC_TRANS_PARAM transParam, uint8_t *MACPayload, uint8_t MACPayloadLen);
bool MiMAC_ReceivedPacket(void);
void MiMAC_DiscardPacket(void);
int16_t MiMAC_ChannelAssessment(uint8_t AssessmentMode);
bool MiMAC_SetAltAddress(uint8_t *Address, uint8_t *PANID);
uint8_t SX1276_RX(void);
void Start_Tx(void);
bool TxPacket(uint8_t TxPacketLen, bool CCA);
void SX1276_RX_Handler(void);
void intTest1(void);


#define         DRVSREG                 0xC623
                
#define         RAW_RF_DEV              ((WORD)19+2*((WORD)CRYSTAL_PPM*915/1000))
#define         RF_DEV                  ( ((RAW_RF_DEV % 15) < 8 ) ? (RAW_RF_DEV - (RAW_RF_DEV % 15)) : (RAW_RF_DEV - (RAW_RF_DEV % 15) + 15) )                 
#define         TXCREG                  (0x9800 | (((WORD)RF_DEV/15 - 1)<<4) | TX_POWER)
#define         FREQ_START              320
#define         FREQ_STEP               105
#define         CHANNEL_NUM             32
#define         FULL_CHANNEL_MAP        0xFFFFFFFF

#define XTAL_FREQ_CW                                   32000000
#define FREQ_STEP_CW                                   61.03515625

#ifdef	__cplusplus
}
#endif

#endif	/* RADIO_DRIVER_H */

/**
 End of File
*/