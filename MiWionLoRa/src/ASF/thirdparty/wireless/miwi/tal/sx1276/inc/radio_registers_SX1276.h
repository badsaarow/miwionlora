/**
* \file  radio_registers_SX1276.h
*
* \brief This is the Radio register SX1276 source file which contains LoRa-specific
*		 Radio register functions declarations and defines for SX1276
*
* Copyright (c) 2018 Microchip Technology Inc. and its subsidiaries. 
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

#ifndef RADIO_REGISTERS_H
#define	RADIO_REGISTERS_H

#ifdef	__cplusplus
extern "C" {
#endif


#define REG_WRITE                                       0x80

// Common registers
#define REG_FIFO                                        0x00
#define REG_OPMODE                                      0x01
#define REG_FRFMSB                                      0x06
#define REG_FRFMID                                      0x07
#define REG_FRFLSB                                      0x08
#define REG_PACONFIG                                    0x09
#define REG_PARAMP                                      0x0A
#define REG_OCP                                         0x0B
#define REG_LNA                                         0x0C
#define REG_DIOMAPPING1                                 0x40
#define REG_DIOMAPPING2                                 0x41
#define REG_VERSION                                     0x42
#define REG_TCXO                                        0x4B
#define REG_PADAC                                       0x4D
#define REG_FORMERTEMP                                  0x5B
#define REG_AGCREF                                      0x61
#define REG_AGCTHRESH1                                  0x62
#define REG_AGCTHRESH2                                  0x63
#define REG_AGCTHRESH3                                  0x64
#define REG_PLL                                         0x70

// FSK only registers
#define REG_FSK_BITRATEMSB                              0x02
#define REG_FSK_BITRATELSB                              0x03
#define REG_FSK_FDEVMSB                                 0x04
#define REG_FSK_FDEVLSB                                 0x05
#define REG_FSK_RXCONFIG                                0x0D
#define REG_FSK_RSSICONFIG                              0x0E
#define REG_FSK_RSSICOLLISION                           0x0F
#define REG_FSK_RSSITHRESH                              0x10
#define REG_FSK_RSSIVALUE                               0x11
#define REG_FSK_RXBW                                    0x12
#define REG_FSK_AFCBW                                   0x13
#define REG_FSK_OOKPEAK                                 0x14
#define REG_FSK_OOKFIX                                  0x15
#define REG_FSK_OOKAVG                                  0x16
#define REG_FSK_RES17                                   0x17
#define REG_FSK_RES18                                   0x18
#define REG_FSK_RES19                                   0x19
#define REG_FSK_AFCFEI                                  0x1A
#define REG_FSK_AFCMSB                                  0x1B
#define REG_FSK_AFCLSB                                  0x1C
#define REG_FSK_FEIMSB                                  0x1D
#define REG_FSK_FEILSB                                  0x1E
#define REG_FSK_PREAMBLEDETECT                          0x1F
#define REG_FSK_RXTIMEOUT1                              0x20
#define REG_FSK_RXTIMEOUT2                              0x21
#define REG_FSK_RXTIMEOUT3                              0x22
#define REG_FSK_RXDELAY                                 0x23
#define REG_FSK_OSC                                     0x24
#define REG_FSK_PREAMBLEMSB                             0x25
#define REG_FSK_PREAMBLELSB                             0x26
#define REG_FSK_SYNCCONFIG                              0x27
#define REG_FSK_SYNCVALUE1                              0x28
#define REG_FSK_SYNCVALUE2                              0x29
#define REG_FSK_SYNCVALUE3                              0x2A
#define REG_FSK_SYNCVALUE4                              0x2B
#define REG_FSK_SYNCVALUE5                              0x2C
#define REG_FSK_SYNCVALUE6                              0x2D
#define REG_FSK_SYNCVALUE7                              0x2E
#define REG_FSK_SYNCVALUE8                              0x2F
#define REG_FSK_PACKETCONFIG1                           0x30
#define REG_FSK_PACKETCONFIG2                           0x31
#define REG_FSK_PAYLOADLENGTH                           0x32
#define REG_FSK_NODEADRS                                0x33
#define REG_FSK_BROADCASTADRS                           0x34
#define REG_FSK_FIFOTHRESH                              0x35
#define REG_FSK_SEQCONFIG1                              0x36
#define REG_FSK_SEQCONFIG2                              0x37
#define REG_FSK_TIMERRESOL                              0x38
#define REG_FSK_TIMER1COEF                              0x39
#define REG_FSK_TIMER2COEF                              0x3A
#define REG_FSK_IMAGECAL                                0x3B
#define REG_FSK_TEMP                                    0x3C
#define REG_FSK_LOWBAT                                  0x3D
#define REG_FSK_IRQFLAGS1                               0x3E
#define REG_FSK_IRQFLAGS2                               0x3F
#define REG_FSK_PLLHOP                                  0x44
#define REG_FSK_BITRATEFRAC                             0x5D

// LoRa only registers
#define REG_LORA_FIFOADDRPTR                            0x0D
#define REG_LORA_FIFOTXBASEADDR                         0x0E
#define REG_LORA_FIFORXBASEADDR                         0x0F
#define REG_LORA_FIFORXCURRENTADDR                      0x10
#define REG_LORA_IRQFLAGSMASK                           0x11
#define REG_LORA_IRQFLAGS                               0x12
#define REG_LORA_RXNBBYTES                              0x13
#define REG_LORA_RXHEADERCNTVALUEMSB                    0x14
#define REG_LORA_RXHEADERCNTVALUELSB                    0x15
#define REG_LORA_RXPACKETCNTVALUEMSB                    0x16
#define REG_LORA_RXPACKETCNTVALUELSB                    0x17
#define REG_LORA_MODEMSTAT                              0x18
#define REG_LORA_PKTSNRVALUE                            0x19
#define REG_LORA_PKTRSSIVALUE                           0x1A
#define REG_LORA_RSSIVALUE                              0x1B
#define REG_LORA_HOPCHANNEL                             0x1C
#define REG_LORA_MODEMCONFIG1                           0x1D
#define REG_LORA_MODEMCONFIG2                           0x1E
#define REG_LORA_SYMBTIMEOUTLSB                         0x1F
#define REG_LORA_PREAMBLEMSB                            0x20
#define REG_LORA_PREAMBLELSB                            0x21
#define REG_LORA_PAYLOADLENGTH                          0x22
#define REG_LORA_PAYLOADMAXLENGTH                       0x23
#define REG_LORA_HOPPERIOD                              0x24
#define REG_LORA_FIFORXBYTEADDR                         0x25
#define REG_LORA_MODEMCONFIG3                           0x26
#define REG_LORA_FEIMSB                                 0x28
#define REG_LORA_FEIMID                                 0x29
#define REG_LORA_FEILSB                                 0x2A
#define REG_LORA_RSSIWIDEBAND                           0x2C
#define REG_LORA_DETECTOPTIMIZE                         0x31
#define REG_LORA_INVERTIQ                               0x33
#define REG_LORA_DETECTIONTHRESHOLD                     0x37


/*!
 * RegPaConfig
 */
#define RFLR_PACONFIG_PASELECT_MASK                 0x7F 
#define RFLR_PACONFIG_PASELECT_PABOOST              0x80 
#define RFLR_PACONFIG_PASELECT_RFO                  0x00 // Default

#define RFLR_PACONFIG_MAX_POWER_MASK                0x8F

#define RFLR_PACONFIG_OUTPUTPOWER_MASK              0xF0 

 /*!
 * RegModemConfig1
 */
#define RFLR_MODEMCONFIG1_BW_MASK                   0x0F 

// SignalBw [0: 7.8kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,
// 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]
#define RFLR_MODEMCONFIG1_BW_7_81_KHZ               0 
#define RFLR_MODEMCONFIG1_BW_10_41_KHZ              1 
#define RFLR_MODEMCONFIG1_BW_15_62_KHZ              2 
#define RFLR_MODEMCONFIG1_BW_20_83_KHZ              3 
#define RFLR_MODEMCONFIG1_BW_31_25_KHZ              4 
#define RFLR_MODEMCONFIG1_BW_41_66_KHZ              5 
#define RFLR_MODEMCONFIG1_BW_62_50_KHZ              6 
#define RFLR_MODEMCONFIG1_BW_125_KHZ                7 // Default
#define RFLR_MODEMCONFIG1_BW_250_KHZ                8 
#define RFLR_MODEMCONFIG1_BW_500_KHZ                9 

 /*!
 * RegModemConfig2
 */
#define RFLR_MODEMCONFIG2_SF_MASK                   0x0F 
#define RFLR_MODEMCONFIG2_SF_6                      0x60 
#define RFLR_MODEMCONFIG2_SF_7                      0x70 // Default
#define RFLR_MODEMCONFIG2_SF_8                      0x80 
#define RFLR_MODEMCONFIG2_SF_9                      0x90 
#define RFLR_MODEMCONFIG2_SF_10                     0xA0 
#define RFLR_MODEMCONFIG2_SF_11                     0xB0 
#define RFLR_MODEMCONFIG2_SF_12                     0xC0 


#define LRFREQ_STEP									61.03515625
#define LRFREQ_OFFSET								200000          // Channel offset 200Khz

#define LRFREQ_START_1								917100000       // Channel 1, 917.1Mhz
#define LRFREQ_START_20								920900000       // Channel 20, 920.9Mhz
#define LRFREQ_START								LRFREQ_START_1
        


//FSK/////////////////////////////////////
#define REG_FIFO                                    0x00
// Common settings
#define REG_OPMODE                                  0x01
#define REG_BITRATEMSB                              0x02
#define REG_BITRATELSB                              0x03
#define REG_FDEVMSB                                 0x04
#define REG_FDEVLSB                                 0x05
#define REG_FRFMSB                                  0x06
#define REG_FRFMID                                  0x07
#define REG_FRFLSB                                  0x08
// Tx settings
#define REG_PACONFIG                                0x09
#define REG_PARAMP                                  0x0A
#define REG_OCP                                     0x0B
// Rx settings
#define REG_LNA                                     0x0C
#define REG_RXCONFIG                                0x0D
#define REG_RSSICONFIG                              0x0E
#define REG_RSSICOLLISION                           0x0F
#define REG_RSSITHRESH                              0x10
#define REG_RSSIVALUE                               0x11
#define REG_RXBW                                    0x12
#define REG_AFCBW                                   0x13
#define REG_OOKPEAK                                 0x14
#define REG_OOKFIX                                  0x15
#define REG_OOKAVG                                  0x16
#define REG_RES17                                   0x17
#define REG_RES18                                   0x18
#define REG_RES19                                   0x19
#define REG_AFCFEI                                  0x1A
#define REG_AFCMSB                                  0x1B
#define REG_AFCLSB                                  0x1C
#define REG_FEIMSB                                  0x1D
#define REG_FEILSB                                  0x1E
#define REG_PREAMBLEDETECT                          0x1F
#define REG_RXTIMEOUT1                              0x20
#define REG_RXTIMEOUT2                              0x21
#define REG_RXTIMEOUT3                              0x22
#define REG_RXDELAY                                 0x23
// Oscillator settings
#define REG_OSC                                     0x24
// Packet handler settings
#define REG_PREAMBLEMSB                             0x25
#define REG_PREAMBLELSB                             0x26
#define REG_SYNCCONFIG                              0x27
#define REG_SYNCVALUE1                              0x28
#define REG_SYNCVALUE2                              0x29
#define REG_SYNCVALUE3                              0x2A
#define REG_SYNCVALUE4                              0x2B
#define REG_SYNCVALUE5                              0x2C
#define REG_SYNCVALUE6                              0x2D
#define REG_SYNCVALUE7                              0x2E
#define REG_SYNCVALUE8                              0x2F
#define REG_PACKETCONFIG1                           0x30
#define REG_PACKETCONFIG2                           0x31
#define REG_PAYLOADLENGTH                           0x32
#define REG_NODEADRS                                0x33
#define REG_BROADCASTADRS                           0x34
#define REG_FIFOTHRESH                              0x35
// SM settings
#define REG_SEQCONFIG1                              0x36
#define REG_SEQCONFIG2                              0x37
#define REG_TIMERRESOL                              0x38
#define REG_TIMER1COEF                              0x39
#define REG_TIMER2COEF                              0x3A
// Service settings
#define REG_IMAGECAL                                0x3B
#define REG_TEMP                                    0x3C
#define REG_LOWBAT                                  0x3D
// Status
#define REG_IRQFLAGS1                               0x3E
#define REG_IRQFLAGS2                               0x3F
// I/O settings
#define REG_DIOMAPPING1                             0x40
#define REG_DIOMAPPING2                             0x41
// Version
#define REG_VERSION                                 0x42
// Additional settings
#define REG_PLLHOP                                  0x44
#define REG_TCXO                                    0x4B
#define REG_PADAC                                   0x4D
#define REG_FORMERTEMP                              0x5B
#define REG_BITRATEFRAC                             0x5D
#define REG_AGCREF                                  0x61
#define REG_AGCTHRESH1                              0x62
#define REG_AGCTHRESH2                              0x63
#define REG_AGCTHRESH3                              0x64
#define REG_PLL                                     0x70

#define RF_OPMODE_LONGRANGEMODE_MASK                0x7F
#define RF_OPMODE_LONGRANGEMODE_OFF                 0x00
#define RF_OPMODE_LONGRANGEMODE_ON                  0x80

#define RF_OPMODE_MODULATIONTYPE_MASK               0x9F
#define RF_OPMODE_MODULATIONTYPE_FSK                0x00  // Default
#define RF_OPMODE_MODULATIONTYPE_OOK                0x20

#define RF_OPMODE_MODULATIONSHAPING_MASK            0xE7
#define RF_OPMODE_MODULATIONSHAPING_00              0x00  // Default
#define RF_OPMODE_MODULATIONSHAPING_01              0x08
#define RF_OPMODE_MODULATIONSHAPING_10              0x10
#define RF_OPMODE_MODULATIONSHAPING_11              0x18

#define RF_OPMODE_MASK                              0xF8
#define RF_OPMODE_SLEEP                             0x00
#define RF_OPMODE_STANDBY                           0x01  // Default
#define RF_OPMODE_SYNTHESIZER_TX                    0x02
#define RF_OPMODE_TRANSMITTER                       0x03
#define RF_OPMODE_SYNTHESIZER_RX                    0x04
#define RF_OPMODE_RECEIVER                          0x05

/*!
 * RegBitRate (bits/sec)
 */
#define RF_BITRATEMSB_1200_BPS                      0x68
#define RF_BITRATELSB_1200_BPS                      0x2B
#define RF_BITRATEMSB_2400_BPS                      0x34
#define RF_BITRATELSB_2400_BPS                      0x15
#define RF_BITRATEMSB_4800_BPS                      0x1A  // Default
#define RF_BITRATELSB_4800_BPS                      0x0B  // Default
#define RF_BITRATEMSB_9600_BPS                      0x0D
#define RF_BITRATELSB_9600_BPS                      0x05
#define RF_BITRATEMSB_15000_BPS                     0x08
#define RF_BITRATELSB_15000_BPS                     0x55
#define RF_BITRATEMSB_19200_BPS                     0x06
#define RF_BITRATELSB_19200_BPS                     0x83
#define RF_BITRATEMSB_38400_BPS                     0x03
#define RF_BITRATELSB_38400_BPS                     0x41
#define RF_BITRATEMSB_76800_BPS                     0x01
#define RF_BITRATELSB_76800_BPS                     0xA1
#define RF_BITRATEMSB_153600_BPS                    0x00
#define RF_BITRATELSB_153600_BPS                    0xD0
#define RF_BITRATEMSB_57600_BPS                     0x02
#define RF_BITRATELSB_57600_BPS                     0x2C
#define RF_BITRATEMSB_115200_BPS                    0x01
#define RF_BITRATELSB_115200_BPS                    0x16
#define RF_BITRATEMSB_12500_BPS                     0x0A
#define RF_BITRATELSB_12500_BPS                     0x00
#define RF_BITRATEMSB_25000_BPS                     0x05
#define RF_BITRATELSB_25000_BPS                     0x00
#define RF_BITRATEMSB_50000_BPS                     0x02
#define RF_BITRATELSB_50000_BPS                     0x80
#define RF_BITRATEMSB_100000_BPS                    0x01
#define RF_BITRATELSB_100000_BPS                    0x40
#define RF_BITRATEMSB_150000_BPS                    0x00
#define RF_BITRATELSB_150000_BPS                    0xD5
#define RF_BITRATEMSB_200000_BPS                    0x00
#define RF_BITRATELSB_200000_BPS                    0xA0
#define RF_BITRATEMSB_250000_BPS                    0x00
#define RF_BITRATELSB_250000_BPS                    0x80
#define RF_BITRATEMSB_32768_BPS                     0x03
#define RF_BITRATELSB_32768_BPS                     0xD1

#define RFLR_OPMODE_LONGRANGEMODE_MASK              0x7F
#define RFLR_OPMODE_LONGRANGEMODE_OFF               0x00 // Default
#define RFLR_OPMODE_LONGRANGEMODE_ON                0x80

#define RFLR_OPMODE_ACCESSSHAREDREG_MASK            0xBF
#define RFLR_OPMODE_ACCESSSHAREDREG_ENABLE          0x40
#define RFLR_OPMODE_ACCESSSHAREDREG_DISABLE         0x00 // Default

#define RFLR_OPMODE_FREQMODE_ACCESS_MASK            0xF7
#define RFLR_OPMODE_FREQMODE_ACCESS_LF              0x08 // Default
#define RFLR_OPMODE_FREQMODE_ACCESS_HF              0x00

#define RFLR_OPMODE_MASK                            0xF8
#define RFLR_OPMODE_SLEEP                           0x00
#define RFLR_OPMODE_STANDBY                         0x01 // Default
#define RFLR_OPMODE_SYNTHESIZER_TX                  0x02
#define RFLR_OPMODE_TRANSMITTER                     0x03
#define RFLR_OPMODE_SYNTHESIZER_RX                  0x04
#define RFLR_OPMODE_RECEIVER                        0x05
// LoRa specific modes
#define RFLR_OPMODE_RECEIVER_SINGLE                 0x06
#define RFLR_OPMODE_CAD                             0x07

#define RF_PACKETCONFIG1_PACKETFORMAT_MASK          0x7F
#define RF_PACKETCONFIG1_PACKETFORMAT_FIXED         0x00
#define RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE      0x80  // Default

#define RF_PACKETCONFIG1_DCFREE_MASK                0x9F
#define RF_PACKETCONFIG1_DCFREE_OFF                 0x00  // Default
#define RF_PACKETCONFIG1_DCFREE_MANCHESTER          0x20
#define RF_PACKETCONFIG1_DCFREE_WHITENING           0x40

#define RF_PACKETCONFIG1_CRC_MASK                   0xEF
#define RF_PACKETCONFIG1_CRC_ON                     0x10  // Default
#define RF_PACKETCONFIG1_CRC_OFF                    0x00

#define RF_PACKETCONFIG1_CRCAUTOCLEAR_MASK          0xF7
#define RF_PACKETCONFIG1_CRCAUTOCLEAR_ON            0x00  // Default
#define RF_PACKETCONFIG1_CRCAUTOCLEAR_OFF           0x08

#define RF_PACKETCONFIG1_ADDRSFILTERING_MASK         0xF9
#define RF_PACKETCONFIG1_ADDRSFILTERING_OFF          0x00  // Default
#define RF_PACKETCONFIG1_ADDRSFILTERING_NODE         0x02
#define RF_PACKETCONFIG1_ADDRSFILTERING_NODEBROADCAST 0x04

#define RF_PACKETCONFIG1_CRCWHITENINGTYPE_MASK      0xFE
#define RF_PACKETCONFIG1_CRCWHITENINGTYPE_CCITT     0x00  // Default
#define RF_PACKETCONFIG1_CRCWHITENINGTYPE_IBM       0x01

#define RF_DIOMAPPING1_DIO0_MASK                    0x3F
#define RF_DIOMAPPING1_DIO0_00                      0x00  // Default
#define RF_DIOMAPPING1_DIO0_01                      0x40
#define RF_DIOMAPPING1_DIO0_10                      0x80
#define RF_DIOMAPPING1_DIO0_11                      0xC0

#define RF_DIOMAPPING1_DIO1_MASK                    0xCF
#define RF_DIOMAPPING1_DIO1_00                      0x00  // Default
#define RF_DIOMAPPING1_DIO1_01                      0x10
#define RF_DIOMAPPING1_DIO1_10                      0x20
#define RF_DIOMAPPING1_DIO1_11                      0x30

#define RF_DIOMAPPING1_DIO2_MASK                    0xF3
#define RF_DIOMAPPING1_DIO2_00                      0x00  // Default
#define RF_DIOMAPPING1_DIO2_01                      0x04
#define RF_DIOMAPPING1_DIO2_10                      0x08
#define RF_DIOMAPPING1_DIO2_11                      0x0C

#define RF_DIOMAPPING1_DIO3_MASK                    0xFC
#define RF_DIOMAPPING1_DIO3_00                      0x00  // Default
#define RF_DIOMAPPING1_DIO3_01                      0x01
#define RF_DIOMAPPING1_DIO3_10                      0x02
#define RF_DIOMAPPING1_DIO3_11                      0x03

/*!
 * RegDioMapping2
 */
#define RF_DIOMAPPING2_DIO4_MASK                    0x3F
#define RF_DIOMAPPING2_DIO4_00                      0x00  // Default
#define RF_DIOMAPPING2_DIO4_01                      0x40
#define RF_DIOMAPPING2_DIO4_10                      0x80
#define RF_DIOMAPPING2_DIO4_11                      0xC0

#define RF_DIOMAPPING2_DIO5_MASK                    0xCF
#define RF_DIOMAPPING2_DIO5_00                      0x00  // Default
#define RF_DIOMAPPING2_DIO5_01                      0x10
#define RF_DIOMAPPING2_DIO5_10                      0x20
#define RF_DIOMAPPING2_DIO5_11                      0x30

#define RF_DIOMAPPING2_MAP_MASK                     0xFE
#define RF_DIOMAPPING2_MAP_PREAMBLEDETECT           0x01
#define RF_DIOMAPPING2_MAP_RSSI                     0x00  // Default

/*!
    * Constant values need to compute the RSSI value
*/
#define RSSI_OFFSET_LF                              -164.0
#define RSSI_OFFSET_HF                              -157.0

#ifdef	__cplusplus
}
#endif

#endif	/* RADIO_REGISTERS_H */

/**
 End of File
*/