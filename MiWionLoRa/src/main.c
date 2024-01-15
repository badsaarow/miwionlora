/**
* \file  main.c
*
* \brief Serial Provisioning of LoRaWAN Demo Application
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


/****************************** INCLUDES **************************************/
#include "system_low_power.h"
#include "radio_driver_hal.h"

//#include "lorawan.h"
//#include "enddevice_demo.h"

#include "sys.h"
#include "system_init.h"
#include "system_assert.h"
#include "aes_engine.h"
#include "sio2host.h"
#include "extint.h"
#include "conf_app.h"
#include "sw_timer.h"
#include "common_hw_timer.h"

#include "radio_driver_SX1276.h"
#include "radio_driver_hal.h"
#include "radio_registers_SX1276.h"
#include "MiWi.h"
#include "nvm.h"
#include "SymbolTime.h"

#include "pmm.h"
#include "conf_pmm.h"
#include "sleep_timer.h"
#include "sleep.h"

#include "conf_sio2host.h"
#if (ENABLE_PDS == 1)
#include "pds_interface.h"
#endif
extern unsigned int MY_PAN_ID;
#define OPCHANNEL		0xffffffff//0x04000000 //400000 23//4000000 27// 80000 20
uint8_t Data_rx;
extern uint8_t serial_rx_buf_tail;
extern uint8_t serial_rx_buf_tail1;
uint16_t TEMP_PAN_ID;
uint8_t RX_TEMP[10];
uint8_t uartRxData[SERIAL_RX_BUF_SIZE_HOST], uartRxDataIndex;
uint64_t uartRecevingGetTime=0,RefT0GetTime;
uint8_t uartAlive=0,uartActive=0,uartCompleted=0;
uint8_t uartAlive1=0,uartActive1=0,uartCompleted1=0;
extern uint8_t serial_rx_buf1[SERIAL_RX_BUF_SIZE_HOST];
extern uint8_t serial_rx_buf[SERIAL_RX_BUF_SIZE_HOST];

extern uint8_t myLongAddress[8];
extern  void edbg_eui_read_eui64(uint8_t *eui);
uint8_t RF_OUTPUT=13,RF_CHANNEL=20,SF_DR=7;
extern uint8_t USE_CHANNEL_NUM_START;
extern uint8_t USE_CHANNEL_NUM_END;
extern uint8_t INIT_CHANNEL_NUM;
/************************** Macro definition ***********************************/
/* Button debounce time in ms */
#define APP_DEBOUNCE_TIME       50
uint8_t rx_Char[4];
uint32_t TX_CNT, RX_CNT;
/************************** Global variables ***********************************/
bool button_pressed = false;
bool factory_reset = false;
bool bandSelected = false;
uint32_t longPress = 0;

bool certMode = false;

extern bool certAppEnabled;
#ifdef CONF_PMM_ENABLE
bool deviceResetsForWakeup = false;
#endif

#if ADDITIONAL_NODE_ID_SIZE > 0
uint8_t AdditionalNodeID[ADDITIONAL_NODE_ID_SIZE] = {0x01};
#endif

uint8_t set_ScanDuration,set_SearchDuration;

/************************** Extern variables ***********************************/

extern uint8_t rxData[];
extern volatile uint8_t	MACTxBuffer[];
extern int16_t LBT_TR;
extern int32_t LBT_LIMIT;

#if SAMD || SAMR21 || SAML21 || SAMR30 || SAMR34 || SAMR35
static struct usart_module host_uart_module1;
#else
static usart_serial_options_t usart_serial_options = {
	.baudrate     = USART_HOST_BAUDRATE,
	.charlength   = USART_HOST_CHAR_LENGTH,
	.paritytype   = USART_HOST_PARITY,
	.stopbits     = USART_HOST_STOP_BITS
};
#endif

#if SAMD || SAMR21 || SAML21 || SAMR30 || SAMR34 || SAMR35
static struct usart_module host_uart_module;
#else
static usart_serial_options_t usart_serial_options = {
	.baudrate     = USART_HOST_BAUDRATE,
	.charlength   = USART_HOST_CHAR_LENGTH,
	.paritytype   = USART_HOST_PARITY,
	.stopbits     = USART_HOST_STOP_BITS
};
#endif
/************************** Function Prototypes ********************************/
void timer0Compare64bitTime(void); 
static void driver_init(void);
void PrintMenu(void);
void ConsoleProc(void);

void PrintMenuC(void);
void ConsoleProcC(void);

void Sleep(void);

void NVM_Write();
void NVM_Read();

#if (_DEBUG_ == 1)
static void assertHandler(SystemAssertLevel_t level, uint16_t code);
#endif /* #if (_DEBUG_ == 1) */

/*********************************************************************//**
 \brief      Uninitializes app resources before going to low power mode
*************************************************************************/
#ifdef CONF_PMM_ENABLE
static void app_resources_uninit(void);
#endif

/****************************** FUNCTIONS **************************************/

static void print_reset_causes(void)
{
    enum system_reset_cause rcause = system_get_reset_cause();
    printf("Last reset cause: ");
    if(rcause & (1 << 6)) {
        printf("System Reset Request\r\n");
    }
    if(rcause & (1 << 5)) {
        printf("Watchdog Reset\r\n");
    }
    if(rcause & (1 << 4)) {
        printf("External Reset\r\n");
    }
    if(rcause & (1 << 2)) {
        printf("Brown Out 33 Detector Reset\r\n");
    }
    if(rcause & (1 << 1)) {
        printf("Brown Out 12 Detector Reset\r\n");
    }
    if(rcause & (1 << 0)) {
        printf("Power-On Reset\r\n");
    }
}

#ifdef CONF_PMM_ENABLE
static void appWakeup(uint32_t sleptDuration)
{
    HAL_Radio_resources_init();
    sio2host_init();
    printf("\r\nsleep_ok %ld ms\r\n", sleptDuration);

}
#endif

#if (_DEBUG_ == 1)
static void assertHandler(SystemAssertLevel_t level, uint16_t code)
{
    printf("\r\n%04x\r\n", code);
    (void)level;
}
#endif /* #if (_DEBUG_ == 1) */

void PrintMenuC(void)
{
	printf("\r\nSAMR34 Certification Firmware\r\n");
	printf("**************************************\r\n");
	printf("1: Send Message\r\n");
	printf("2: CW\r\n");
	printf("3: Send LBT Message\r\n");
	printf("4: RX Mode\r\n");
	printf("c: Channel\r\n");
	printf("b: Bandwidth\r\n");
	printf("s: Spreading Factor\r\n");
	printf("p: Output Power\r\n");
	printf("t: LBT Threshold\r\n");
	printf("x: SLEEP\r\n");
	
	printf("\n\rH: Print Menu\r\n");
	printf("**************************************\r\n");
}
void ConsoleProcC(void)
{
	char rxChar;
	uint8_t i,rlt,tmp;
	uint32_t OperatingChannel = 0x00300000;//0xFFFFFFFF;
	uint64_t timeVal;
	
	int16_t LBT_tmp;
	
	uint32_t fdev = 0;
	uint32_t datarate = 50e3;
	uint16_t preambleLen = 5;
	bool fixLen = 1;
	bool crcOn = 1;
	uint8_t tmp2=0;
	
	port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_ACTIVE);//kwlee 200721
	if((-1) != (rxChar = sio2host_getchar_nowait()))
	{
		switch(rxChar){
			case '1':
				printf("\r\nStart TX...\r\n");
				MiApp_FlushTx();				
				for(i=0;i<16;i++){
					MiApp_WriteData(i);
				}
				MiMAC_SetChannel(RF_CHANNEL);
				delay_ms(100);
				SX1276LoRaSetRFPower(RF_OUTPUT);
				delay_ms(100);
				
				//port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_INACTIVE);//kwlee 200721
				if(RF_OUTPUT==20){
					port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_INACTIVE);//kwlee 200721
					}else{
					port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_ACTIVE);//kwlee 200727
				}
				printf("Press 'q' to stop TX....\r\n");
				while(1){
					
					if((-1) != (rxChar = sio2host_getchar_nowait())){
						if(rxChar == 'q') {
							printf("Stop TX....\r\n");
							
							SX1276_RX_INIT();
							break;
						}
					}
					
					SX1276_TX((uint8_t *)MACTxBuffer, 16);
				}
				
			break;
			case '2':
				printf("\r\nStart CW...\r\n");
				
				MiMAC_SetChannel(RF_CHANNEL);
				delay_ms(100);
				SX1276LoRaSetRFPower(RF_OUTPUT);
				delay_ms(100);
				
				if(RF_OUTPUT==20){
					port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_INACTIVE);//kwlee 200721
				}else{
					port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_ACTIVE);//kwlee 200727
				}
				//
				
				tmp2 = RADIO_RegisterRead( REG_OPMODE );
				RADIO_RegisterWrite( REG_OPMODE, ( tmp2 & RF_OPMODE_MASK ) | 0 ); //MODEM_FSK = 0;
				tmp2 = RADIO_RegisterRead( REG_OPMODE );
				RADIO_RegisterWrite( REG_OPMODE, ( tmp2 & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF );
				RADIO_RegisterWrite( REG_DIOMAPPING1, 0x00 );
				RADIO_RegisterWrite( REG_DIOMAPPING2, 0x30 ); // DIO5=ModeReady
				fdev = ( uint16_t )( ( double )fdev / ( double )FREQ_STEP_CW );
				RADIO_RegisterWrite( REG_FDEVMSB, ( uint8_t )( fdev >> 8 ) );
				RADIO_RegisterWrite( REG_FDEVLSB, ( uint8_t )( fdev & 0xFF ) );

				datarate = ( uint16_t )( ( double )XTAL_FREQ_CW / ( double )datarate );
				RADIO_RegisterWrite( REG_BITRATEMSB, ( uint8_t )( datarate >> 8 ) );
				RADIO_RegisterWrite( REG_BITRATELSB, ( uint8_t )( datarate & 0xFF ) );

				RADIO_RegisterWrite( REG_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
				RADIO_RegisterWrite( REG_PREAMBLELSB, preambleLen & 0xFF );

				tmp2 = RADIO_RegisterRead( REG_PACKETCONFIG1 );
				
				RADIO_RegisterWrite( REG_PACKETCONFIG1, ( tmp2 & RF_PACKETCONFIG1_CRC_MASK & RF_PACKETCONFIG1_PACKETFORMAT_MASK ) | ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) | ( crcOn << 4 ) );
								
				RADIO_RegisterWrite( REG_DIOMAPPING1, ( RADIO_RegisterRead( REG_DIOMAPPING1 ) & RF_DIOMAPPING1_DIO0_MASK & RF_DIOMAPPING1_DIO1_MASK & RF_DIOMAPPING1_DIO2_MASK ) |RF_DIOMAPPING1_DIO1_01 );

				RADIO_RegisterWrite( REG_DIOMAPPING2, ( RADIO_RegisterRead( REG_DIOMAPPING2 ) & RF_DIOMAPPING2_DIO4_MASK & RF_DIOMAPPING2_MAP_MASK ) );
				
				RADIO_RegisterWrite( REG_OPMODE, ( RADIO_RegisterRead( REG_OPMODE ) & RF_OPMODE_MASK ) | RF_OPMODE_TRANSMITTER );
				
				printf("Press 'q' to stop TX....\r\n");
				while(1){
					rxChar = sio2host_getchar();
					if(rxChar == 'q') {
						printf("Stop TX....\r\n");
						SX1276_Reset();
						SX1276_Config();
						SX1276_RX_INIT();
						break;
					}
				
				}				
			break;
			case '3':
				printf("\r\nStart TX with LBT...\r\n");
				MiApp_FlushTx();
				for(i=0;i<16;i++){
					MiApp_WriteData(i);
				}
				MiMAC_SetChannel(RF_CHANNEL);
				delay_ms(100);
				SX1276LoRaSetRFPower(RF_OUTPUT);
				delay_ms(100);
				
				if(RF_OUTPUT==20){
					port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_INACTIVE);//kwlee 200721
				}else{
					port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_ACTIVE);//kwlee 200727
				}
				
				printf("Press 'q' to stop TX....\r\n");
				while(1){
					//port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_INACTIVE);//kwlee 200721
					if((-1) != (rxChar = sio2host_getchar_nowait())){
						if(rxChar == 'q') {
							printf("Stop TX....\r\n");
							
							SX1276_RX_INIT();
							break;
						}
					}					
					if(MiApp_BroadcastPacket(false)){
						printf("*");
					} else {
						printf("LBT TX Fail!!\r\n");
					}
				}
			break;
			case '4':
			//port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_INACTIVE);//kwlee 200721
				printf("\r\nStart RX...\r\n");
				SX1276_RX_INIT();
				printf("RX MODE...\r\n\r\n");

			break;
			case 'c':
				printf("\r\nChannel setting...");
				printf("\r\nChannel: 20 ~ 32 ( 920.9MHz ~ 923.3MHz )");
				
				rxChar = sio2host_getchar();
				tmp = (rxChar-'0')*10;
				rxChar = sio2host_getchar();
				tmp += (rxChar-'0');
				
				if(tmp>19 && tmp<33)
				{
					//20, 26, 32
					MiMAC_SetChannel(tmp);
					RF_CHANNEL=tmp;
					printf("\r\nSet Channel - %d - Done\r\n\r\n",tmp);
				}
				else
				{
					printf("\r\nInvalid value - %d \r\n\r\n",tmp);
				}
				
			break;
			case 'b':
				// SignalBw [0:  7.8kHz, 1: 10.4kHz, 2: 15.6kHz, 3: 20.8kHz, 4: 31.25kHz,
				//			 5: 41.7kHz, 6: 62.5kHz, 7: 125kHz,  8: 250kHz,  9: 500kHz, other: Reserved]
				printf("\r\nBandwidth setting...\r\n");
				printf("0:  7.8Khz,1: 10.4Khz ,2: 15.6Khz ,3: 20.8Khz ,4: 31.25Khz \r\n");
				printf("5: 41.7Khz,6: 62.5Khz ,7: 125Khz , 8: 250Khz , 9: 500Khz");
				rxChar = sio2host_getchar();
				tmp = (rxChar-'0');
				
				if(tmp>=0 && tmp<=9)
				{
					
					SX1276LoRaSetSignalBandwidth(tmp);
					
					printf("\r\nSet SignalBw - %d - Done\r\n\r\n",tmp);
				}
				else
				{
					printf("\r\nInvalid value - %d \r\n\r\n",tmp);
				}
				
			break;
			case 's':
				
				// SpreadingFactor [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
				printf("\r\nSpreading Factor setting...\r\n");
				printf("06: SF6, 07: SF7, 08: SF8, 09: SF9, 10: SF10, 11: SF11, 12: SF12 \r\n");
				rxChar = sio2host_getchar();
				tmp = (rxChar-'0')*10;
				rxChar = sio2host_getchar();
				tmp += (rxChar-'0');
				
				if(tmp>=6 && tmp<=12)
				{
					
					SX1276LoRaSetSpreadingFactor(tmp);
					
					printf("\r\nSet SpreadingFactor - %d - Done\r\n\r\n",tmp);
				}
				else
				{
					printf("\r\nInvalid value - %d \r\n\r\n",tmp);
				}
				
				
			break;
			case 'p':
				printf("\r\nOutput Power setting...\r\n");				
				printf("20, 17 ~ 02 dBm");
				rxChar = sio2host_getchar();
				tmp = (rxChar-'0')*10;
				rxChar = sio2host_getchar();
				tmp += (rxChar-'0');
				
				if((tmp>=2 && tmp<=17) || tmp == 20)
				{
					
					SX1276LoRaSetRFPower(tmp);
					RF_OUTPUT=tmp;
					printf("\r\nSet Output Power - %d - Done\r\n\r\n",tmp);
				}
				else
				{
					printf("\r\nInvalid value - %d \r\n\r\n",tmp);
				}
			break;
			case 't':
				printf("\n\rLBT Threshold setting...\r\n");
				
				printf("Current Threshold Value: %d dBm\r\n",LBT_TR);
				printf("Input threshold Value: (-)10 ~ 99 \r\n");				
				LBT_tmp = 0;
				rxChar = sio2host_getchar();
				tmp = (rxChar-'0')*10;
				rxChar = sio2host_getchar();
				tmp += (rxChar-'0');
				
				LBT_tmp = tmp;
				
				if(LBT_tmp < 10){
					LBT_tmp = 0;
				} else if(LBT_tmp >= 99){
					LBT_tmp = 99;
				}
				LBT_TR = LBT_tmp * -1;
				LBT_LIMIT = LBT_tmp * -1;
				
				printf("Changed Threshold Value: %d dBm\r\n",LBT_TR);
				
			break;
			case 'h':
				PrintMenuC();
			break;
			case 'x':
				
				//sleep;
				Sleep();
				
			break;
		}

	}
}
unsigned char Ascii2Hex(unsigned int AH)
{	unsigned char temp,rv;
	temp = AH >> 8;
	rv = temp - 0x30;	rv <<= 4;
	temp = AH;
	rv += temp - 0x30;
	return(rv);
}
char hex2ascii(char toconv)
{
    if (toconv<0x0A)    toconv += 0x30;
    else        toconv += 0x37; 
    return (toconv);
}
int hex2int(char *hdec) {
    int finalval = 0;
   // while (*hdec) {
        
        int onebyte = *hdec++; 
        
        if (onebyte >= '0' && onebyte <= '9'){onebyte = onebyte - '0';}
        else if (onebyte >= 'a' && onebyte <='f') {onebyte = onebyte - 'a' + 10;}
        else if (onebyte >= 'A' && onebyte <='F') {onebyte = onebyte - 'A' + 10;}  
        else {
			printf("error\r\n");
		}
        finalval = (finalval << 4) | (onebyte & 0xF);
  //  }
    finalval = finalval - 524288;
    return finalval;
}

void PrintMenu(void)
{
	printf("\r\nSAMR34 MoL Firmware\r\n");
	printf("**************************************\r\n");
	printf("S: Start Network\r\n");
	//printf("N: Start Network from NVM\r\n");
	printf("J: Join Network\r\n");
	//printf("K: Join Network from NVM\r\n");
	printf("R: Start UART Communication\r\n");
	printf("P: Output Power \r\n");
	printf("F: Change SF,DR\r\n");
	printf("C: Change Channel\r\n");
	printf("I: Change PANID\r\n");
	printf("9: Device Info\r\n");
	printf("0: Dump Connection\r\n");
	//printf("x: Sleep\r\n");	
	printf("\n\rH: Print Menu\r\n");
	printf("**************************************\r\n");
}

void ConsoleProc(void)
{
	char rxChar;
	uint8_t i,rlt,tmp;
	
	//uint32_t OperatingChannel = 0x00300000;//0xFFFFFFFF;
	uint32_t OperatingChannel = OPCHANNEL;//0xFFFFFFFF;
	
	uint64_t timeVal;
	uint8_t dst[2],Init_Temp=1; //kwlee
	
	port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_ACTIVE);//kwlee 200721
	if((-1) != (rxChar = sio2host_getchar_nowait()))
	{
		switch(rxChar){
			//case 't':
			//timeVal = tmr_read_count();
			//printf("%l\n\r",timeVal);
			//break;
			//case 'd':
			//case 'D':
			//MiApp_ConnectionMode(DISABLE_ALL_CONN);
			break;
			case 'r':
			case 'R':
			
			printf("\n\r======= Start UART Communication =======\n\r");
			do{
				timer0Compare64bitTime(); 
				if(uartCompleted) { // 데이터가 들어오다 5ms 이상 데이터가 없으면 프레임 끝으로 봄 
					uartCompleted=0; 
					if(!Init_Temp){
						MiApp_FlushTx();
						for(i=0;i<uartRxDataIndex;i++){

							MiApp_WriteData(uartRxData[i]);
							printf(" %c",uartRxData[i]);
						}
						printf(" \n\r");
						uartRxDataIndex=0;
						if(RF_OUTPUT==20){
							port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_INACTIVE);//kwlee 200721
							}else{
							port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_ACTIVE);//kwlee 200727
						}
						MiApp_BroadcastPacket(false);
						printf("Broadcast...\r\n");

						port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_ACTIVE);//kwlee 200721
						SX1276_RX_INIT();
					}
					Init_Temp=0;
			}else if( MiApp_MessageAvailable() )
				{
					printf("\n\r[Data Received]\n\r");
					printf("SRC PID: %x\r\n",rxMessage.SourcePANID);
					printf("SRC ADR: %.2x%.2x\r\n",rxMessage.SourceAddress[1],rxMessage.SourceAddress[0]);
					printf("Length: %d\r\n",rxMessage.PayloadSize);
					printf("RSSI: %ddBm\r\n",rxMessage.PacketRSSI);
			
					printf("Payload: ");
					for(i=0;i<rxMessage.PayloadSize;i++){
						printf("%.2x ",*(uint8_t*)(rxMessage.Payload+i));
					}
					printf("\n\r");
					//printf("RX CNT : %6D , TX CNT : %6D ",RX_CNT,TX_CNT);
					//printf("\n\r");
					//RX_CNT++;
					//TX_CNT++;
			
					port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_ACTIVE);//kwlee 200721
					MiApp_DiscardMessage();
				}
		
		  
			}while(1);
			break;
			//case 'e':
			//case 'E':
			//MiApp_ConnectionMode(ENABLE_ALL_CONN);
			break;
			case 's':
			case 'S':
				MiMAC_SetChannel(RF_CHANNEL);
				delay_ms(10);
				SX1276LoRaSetRFPower(RF_OUTPUT);
				delay_ms(10);

				//if(RF_OUTPUT==20){
					//port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_INACTIVE);//kwlee 200721
					//}else{
					//port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_ACTIVE);//kwlee 200727
				//}
				port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_ACTIVE);//kwlee 210223 rx
				MiApp_ConnectionMode(ENABLE_ALL_CONN);
				if(MiApp_StartConnection(START_CONN_ENERGY_SCN, set_ScanDuration, OperatingChannel)){
					printf("Network Created....\n\r");
					DumpConnection(0xff);
					}else{
					printf("Network Start Fail....\n\r");
				}
				SX1276_RX_INIT();//kwlee 210223
				break;
			case 'n':
			case 'N':
				MiMAC_SetChannel(RF_CHANNEL);
				delay_ms(10);
				SX1276LoRaSetRFPower(RF_OUTPUT);
				delay_ms(10);

				if(RF_OUTPUT==20){
					port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_INACTIVE);//kwlee 200721
					}else{
					port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_ACTIVE);//kwlee 200727
				}
				MiApp_ConnectionMode(ENABLE_ALL_CONN);
				if(MiApp_StartConnection(START_FROM_NVM, 0, 0)){
					printf("Network Created....\n\r");
					DumpConnection(0xff);
					}else{
					printf("Network Start Fail....\n\r");
				}
				break;
			case 'j':
			case 'J':
				MiMAC_SetChannel(RF_CHANNEL);
				delay_ms(10);
				SX1276LoRaSetRFPower(RF_OUTPUT);
				delay_ms(10);
			
				//port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_INACTIVE);//kwlee 200721
				if(RF_OUTPUT==20){
					port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_INACTIVE);//kwlee 200721
					}else{
					port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_ACTIVE);//kwlee 200727
				}
				MiApp_ConnectionMode(DISABLE_ALL_CONN);
				//i = MiApp_SearchConnection(set_SearchDuration, (uint32_t)1<<(OperatingChannel-1));
				i = MiApp_SearchConnection(set_SearchDuration, OperatingChannel);
				//printf("scan result: %d \n\r",i);
				if(i>0){
					if( MiApp_EstablishConnection(0, CONN_MODE_DIRECT) == 0xFF ){
						printf("Join Fail....\n\r");
					} else {
						printf("Join Success....\n\r");
						DumpConnection(0xff);
					}
				}
				else
				{
					printf("\r\nNot found network... Rescan Please...... \r\n");
				}
			break;
			case 'k':
			case 'K':
				MiMAC_SetChannel(RF_CHANNEL);
				delay_ms(10);
				SX1276LoRaSetRFPower(RF_OUTPUT);
				delay_ms(10);
			
				//port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_INACTIVE);//kwlee 200721
				if(RF_OUTPUT==20){
					port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_INACTIVE);//kwlee 200721
					}else{
					port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_ACTIVE);//kwlee 200727
				}
					if( MiApp_EstablishConnection(0, CONN_MODE_NVM) == 0xFF ){
						printf("Join Fail....\n\r");
					} else {
						printf("Join Success....\n\r");
						//DumpConnection(0xff);
					}
			
				break;
			case 'p':
			case 'P':
				printf("\r\nOutput Power setting...\r\n");
				printf("20, 17 ~ 02 dBm");
				rxChar = sio2host_getchar();
				tmp = (rxChar-'0')*10;
				rxChar = sio2host_getchar();
				tmp += (rxChar-'0');
				
				if((tmp>=2 && tmp<=17) || tmp == 20)
				{
					//RF_OUTPUT=tmp;
					SX1276LoRaSetRFPower(tmp);
					RF_OUTPUT=tmp;
					printf("\r\nSet Output Power - %d - Done\r\n\r\n",RF_OUTPUT);
				}
				else
				{
					printf("\r\nInvalid value - %d \r\n\r\n",tmp);
				}
				break;
			case 'c':
			case 'C':
				printf("\r\nChannel setting...");
				printf("\r\nChannel: 20 ~ 32 ( 920.9MHz ~ 923.3MHz )");
				
				rxChar = sio2host_getchar();
				tmp = (rxChar-'0')*10;
				rxChar = sio2host_getchar();
				tmp += (rxChar-'0');
				
				if(tmp>19 && tmp<33)
				{
					//20, 26, 32
					MiMAC_SetChannel(tmp);
					RF_CHANNEL=tmp;
					currentChannel=RF_CHANNEL;
					printf("\r\nSet Channel - %d - Done\r\n\r\n",tmp);
					USE_CHANNEL_NUM_START=RF_CHANNEL;
					USE_CHANNEL_NUM_END=RF_CHANNEL;
					INIT_CHANNEL_NUM=RF_CHANNEL;
					
				}
				else
				{
					printf("\r\nInvalid value - %d \r\n\r\n",tmp);
				}
				break;
			case 'f':
			case 'F':
				// SpreadingFactor [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
				printf("\r\nSpreading Factor setting...\r\n");
				printf("06: SF6, 07: SF7, 08: SF8, 09: SF9, 10: SF10, 11: SF11, 12: SF12 \r\n");
				rxChar = sio2host_getchar();
				tmp = (rxChar-'0')*10;
				rxChar = sio2host_getchar();
				tmp += (rxChar-'0');
			
				if(tmp>=6 && tmp<=12)
				{
					SF_DR=tmp;
					SX1276LoRaSetSpreadingFactor(SF_DR);
				
					printf("\r\nSet SpreadingFactor - %d - Done\r\n\r\n",SF_DR);
				}
				else
				{
					printf("\r\nInvalid value - %d \r\n\r\n",tmp);
				}
				break;
			case 'i':
			case 'I': 
			    printf("\r\nCurrent PANID : %4x\r\n",MY_PAN_ID);
				printf("PANID setting...\r\n");
				printf("Input PANID : 0x \r\n");
				TEMP_PAN_ID=0;
				rx_Char[0] = sio2host_getchar();
				
				printf("%c",rx_Char[0]);

				rx_Char[1] = sio2host_getchar();

				printf("%c",rx_Char[1]);
				
				rx_Char[2] = sio2host_getchar();

				printf("%c",rx_Char[2]);
				
				rx_Char[3] = sio2host_getchar();

				printf("%c",rx_Char[3]);
				rxChar=hex2int(&rx_Char[0]);
				TEMP_PAN_ID=(rxChar<<12);
				rxChar=hex2int(&rx_Char[1]);
				TEMP_PAN_ID|=(rxChar<<8);
				rxChar=hex2int(&rx_Char[2]);
				TEMP_PAN_ID|=(rxChar<<4);
				rxChar=hex2int(&rx_Char[3]);
				TEMP_PAN_ID|=(rxChar);

				MY_PAN_ID=TEMP_PAN_ID;
				myPANID.v[1]=MY_PAN_ID>>8;
				myPANID.v[0]=MY_PAN_ID;
				printf("\r\nCurrent PANID : %4x\r\n",MY_PAN_ID);
	
				break;
			 
			case '9':
				printf("Channel: %d \r\n",currentChannel);
				printf("Output Power - %d \r\n",RF_OUTPUT);
				printf("PANID: 0x%.4x \r\n",myPANID.Val);
				printf("ShortAddr: 0x%.4x \r\n", myShortAddress.Val);
				printf("SpreadingFactor - %d \r\n",SF_DR);
				break;
			case '0':
			DumpConnection(0xff);
			printf("\r\nSpreadingFactor - %d \r\n",SF_DR);
			break;
			//case 'x':
			//case 'X':
				//Sleep();
				//break;			
			case 'h':
			case 'H':
			PrintMenu();
			break;
		}
	}
}

static void extint_callback(void)
{

}

static void configure_extint_channel(void)
{

	struct extint_chan_conf config_extint_chan;
	extint_chan_get_config_defaults(&config_extint_chan);
	config_extint_chan.gpio_pin           = BUTTON_0_EIC_PIN;
	config_extint_chan.gpio_pin_mux       = BUTTON_0_EIC_MUX;
	config_extint_chan.gpio_pin_pull      = EXTINT_PULL_UP;
	config_extint_chan.detection_criteria = EXTINT_DETECT_BOTH;
	extint_chan_set_config(BUTTON_0_EIC_LINE, &config_extint_chan);
	extint_register_callback(extint_callback,BUTTON_0_EIC_LINE,EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(BUTTON_0_EIC_LINE,EXTINT_CALLBACK_TYPE_DETECT);
	while (extint_chan_is_detected(BUTTON_0_EIC_LINE)) {
		extint_chan_clear_detected(BUTTON_0_EIC_LINE);
	}
}


static void main_clock_select_osc16m(void)
{
	struct system_gclk_gen_config gclk_conf;
	struct system_clock_source_osc16m_config osc16m_conf;

	/* Switch to new frequency selection and enable OSC16M */
	system_clock_source_osc16m_get_config_defaults(&osc16m_conf);
	osc16m_conf.fsel = CONF_CLOCK_OSC16M_FREQ_SEL;
	osc16m_conf.on_demand = 0;
	osc16m_conf.run_in_standby = CONF_CLOCK_OSC16M_RUN_IN_STANDBY;
	system_clock_source_osc16m_set_config(&osc16m_conf);
	system_clock_source_enable(SYSTEM_CLOCK_SOURCE_OSC16M);
	while(!system_clock_source_is_ready(SYSTEM_CLOCK_SOURCE_OSC16M));

	/* Select OSC16M as mainclock */
	system_gclk_gen_get_config_defaults(&gclk_conf);
	gclk_conf.source_clock = SYSTEM_CLOCK_SOURCE_OSC16M;
	system_gclk_gen_set_config(GCLK_GENERATOR_0, &gclk_conf);
	if (CONF_CLOCK_OSC16M_ON_DEMAND) {
		OSCCTRL->OSC16MCTRL.reg |= OSCCTRL_OSC16MCTRL_ONDEMAND;
	}

}

static void main_clock_select_dfll(void)
{
	struct system_gclk_gen_config gclk_conf;

	/* Select OSCULP32K as new clock source for mainclock temporarily */
	system_gclk_gen_get_config_defaults(&gclk_conf);
	gclk_conf.source_clock = SYSTEM_CLOCK_SOURCE_XOSC32K;
	system_gclk_gen_set_config(GCLK_GENERATOR_0, &gclk_conf);

	/* Select XOSC32K for GCLK1. */
	system_gclk_gen_get_config_defaults(&gclk_conf);
	gclk_conf.source_clock = SYSTEM_CLOCK_SOURCE_XOSC32K;
	system_gclk_gen_set_config(GCLK_GENERATOR_1, &gclk_conf);
	system_gclk_gen_enable(GCLK_GENERATOR_1);

	struct system_gclk_chan_config dfll_gclk_chan_conf;

	system_gclk_chan_get_config_defaults(&dfll_gclk_chan_conf);
	dfll_gclk_chan_conf.source_generator = GCLK_GENERATOR_1;
 
	struct system_clock_source_dfll_config dfll_conf;
	system_clock_source_dfll_get_config_defaults(&dfll_conf);

	dfll_conf.loop_mode      = SYSTEM_CLOCK_DFLL_LOOP_MODE_CLOSED;
	dfll_conf.on_demand      = false;
	dfll_conf.run_in_stanby  = CONF_CLOCK_DFLL_RUN_IN_STANDBY;
	dfll_conf.multiply_factor = CONF_CLOCK_DFLL_MULTIPLY_FACTOR;
	system_clock_source_dfll_set_config(&dfll_conf);
	system_clock_source_enable(SYSTEM_CLOCK_SOURCE_DFLL);
	while(!system_clock_source_is_ready(SYSTEM_CLOCK_SOURCE_DFLL));
	if (CONF_CLOCK_DFLL_ON_DEMAND) {
		OSCCTRL->DFLLCTRL.bit.ONDEMAND = 1;
	}

	/* Select DFLL for mainclock. */
	system_gclk_gen_get_config_defaults(&gclk_conf);
	gclk_conf.source_clock = SYSTEM_CLOCK_SOURCE_DFLL;
	system_gclk_gen_set_config(GCLK_GENERATOR_0, &gclk_conf);

}

static void main_clock_select(const enum system_clock_source clock_source)
{
	if (clock_source == SYSTEM_CLOCK_SOURCE_DFLL) {
		main_clock_select_dfll();
		system_clock_source_disable(SYSTEM_CLOCK_SOURCE_OSC16M);
		} else if (clock_source == SYSTEM_CLOCK_SOURCE_OSC16M) {
		main_clock_select_osc16m();
		system_clock_source_disable(SYSTEM_CLOCK_SOURCE_DFLL);
		system_gclk_chan_disable(OSCCTRL_GCLK_ID_DFLL48);
		system_gclk_gen_disable(GCLK_GENERATOR_1);
		} else {
		return ;
	}
}

static void test_active_mode(const enum system_performance_level performance_level)
{

	enum system_performance_level curr_pl = system_get_performance_level();

	printf("System will switch to PL:%d \r\n",performance_level);
	if (curr_pl == performance_level) {
		return ;
	}

	if (curr_pl < performance_level) {

		/* Scaling up the performance level first and then increase clock frequency */
		system_switch_performance_level(performance_level);
		main_clock_select(SYSTEM_CLOCK_SOURCE_DFLL);

		} else {
		/* Scaling down clock frequency and then Scaling down the performance level */
		main_clock_select(SYSTEM_CLOCK_SOURCE_OSC16M);
		system_switch_performance_level(performance_level);
	}

	/* Toggles LED0 once clock frequency successfully */
	//led_toggle_indication(LED0_TOGGLE_2);
}


static void put_radio_to_sleep(void)
{
	uint8_t op_mode, current_mode;
	uint8_t new_mode = 0x00;
	
	/* Initialize the Radio Hardware */
	//HAL_RadioInit();
	
	/* Power On the TCXO Oscillator */
	//HAL_TCXOPowerOn();
	
	/* Reset the radio */
	RADIO_Reset();
 

	op_mode = RADIO_RegisterRead(0x01);
	current_mode = op_mode & 0x07;
	
	if (new_mode != current_mode)
	{
		// Do the actual mode switch.
		op_mode &= ~0x07;                // Clear old mode bits
		op_mode |= new_mode;             // Set new mode bits
		while (op_mode != RADIO_RegisterRead(0x01))
		{
			RADIO_RegisterWrite(0x01, op_mode);
		}
	}
	
	/* Power off the oscillator after putting radio to sleep */
	HAL_TCXOPowerOff();
	
	/* Disable the SPI interface */
	HAL_RadioDeInit();
}

void Sleep(void)
{
	put_radio_to_sleep();

	system_apb_clock_clear_mask(SYSTEM_CLOCK_APB_APBC, MCLK_APBCMASK_SERCOM0);
	system_gclk_chan_disable(SERCOM0_GCLK_ID_CORE);

	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);

	pin_conf.direction = PORT_PIN_DIR_OUTPUT;
	
	//pin_conf.input_pull = PORT_PIN_PULL_DOWN;
	port_pin_set_config(PIN_PA04D_SERCOM0_PAD0, &pin_conf);
	
	pin_conf.direction = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(PIN_PA05D_SERCOM0_PAD1, &pin_conf);
	port_pin_set_config(PIN_PA28, &pin_conf);

	port_pin_set_output_level(PIN_PA09, false);
	port_pin_set_output_level(PIN_PA13, false);
	system_set_sleepmode(SYSTEM_SLEEPMODE_BACKUP);//SYSTEM_SLEEPMODE_BACKUP
	system_clock_source_disable(SYSTEM_CLOCK_SOURCE_XOSC32K);

	system_sleep();
	system_clock_source_enable(SYSTEM_CLOCK_SOURCE_XOSC32K);
}
void timer0Compare64bitTime(void) 
{ 
	uint64_t temp64; 
	if(uartAlive) { 
		uartRecevingGetTime = SwTimerGetTime(); 
		uartAlive = 0;
		uartActive=1; 
	} 
	if(uartActive) { 
		temp64 = SwTimerGetTime(); 
		// RefT0GetTime = temp64; 
		// 테스트용 
		temp64 -= uartRecevingGetTime; 
		 
		if(temp64 > 100) // > 5msec
		{
			uartRxDataIndex = serial_rx_buf_tail;
			memcpy(&uartRxData[0],&serial_rx_buf[0],uartRxDataIndex);
			memset(&serial_rx_buf[0],0,uartRxDataIndex);
			serial_rx_buf_tail=0;
			uartActive=0;
			uartCompleted=1;
		}
	} 
}
void usart_serial_write_P0(const uint8_t *tx_data,uint16_t length)
{
	uint16_t i;
	for(i=0;i<length;i++){
		sio2host_putchar(tx_data[i]);
	}
}
void usart_serial_write_P1(const uint8_t *tx_data,uint16_t length)
{
	uint16_t i;
	for(i=0;i<length;i++){
		sio2host_putchar1(tx_data[i]);
	}
}
/**
 * \mainpage
 * \section preface Preface
 * This is the reference manual for the LORAWAN Demo Application of EU Band
 */
int main(void)
{
	uint8_t i,j,k,l,m;
	uint32_t OperatingChannel = OPCHANNEL;//0xFFFFFFFF;
    /* System Initialization */
    system_init();
    /* Initialize the delay driver */
    delay_init();
    /* Initialize the board target resources */
    board_init();	
    /* Initialize Hardware and Software Modules */
    driver_init();
    /* Initialize the Serial Interface */
	edbg_eui_read_eui64((uint8_t *)&myLongAddress[0]);
    sio2host_init();
	sio2host_init1();
	 
	/*NVM Init for Network Freezer*/
	NVM_Init();
MY_PAN_ID=0xabcd;
			
    print_reset_causes();
#if (_DEBUG_ == 1)
    SYSTEM_AssertSubscribe(assertHandler);
#endif
    
	/////////////////////////////////////////
	certMode = false;
	/////////////////////////////////////////

	printf("[MiWi On LoRa - AT Command SAMR35J17B V0.1]-0x");
	 for(i = 0; i < 8; i++)
	 {
		 printf("%.2x",myLongAddress[8-1-i]);
	 }
	 printf("\n\r");
	SX1276_Reset();	
	SX1276_Config();
	SX1276_RX_INIT();
	
	// Channel [20 ~ 32 ( 920.9MHz ~ 923.3MHz )]
	MiMAC_SetChannel(RF_CHANNEL);	
	
	// SignalBw [0:  7.8kHz, 1: 10.4kHz, 2: 15.6kHz, 3: 20.8kHz, 4: 31.25kHz,
	//			 5: 41.7kHz, 6: 62.5kHz, 7: 125kHz,  8: 250kHz,  9: 500kHz, other: Reserved]
	SX1276LoRaSetSignalBandwidth(7);
	
	// SpreadingFactor [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
	SX1276LoRaSetSpreadingFactor(SF_DR);//11
	
	// RF Output Power [20, 17 ~ 02 dBm]
	SX1276LoRaSetRFPower(RF_OUTPUT);
			
	MiApp_ProtocolInit(false);
	 
	// Need improvement - auto setting
	set_ScanDuration = 14;
	set_SearchDuration = 14;
 
	if(RF_OUTPUT==20){
		port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_INACTIVE);//kwlee 200721
		}else{
		port_pin_set_output_level(RF_SWITCH_PIN, RF_SWITCH_ACTIVE);//kwlee 200727
	}

	USE_CHANNEL_NUM_START=RF_CHANNEL;
	USE_CHANNEL_NUM_END=RF_CHANNEL;
    INIT_CHANNEL_NUM=RF_CHANNEL;
	currentChannel=RF_CHANNEL;
	
	MY_PAN_ID=0xabc1;
	myPANID.v[1]=MY_PAN_ID>>8;
	myPANID.v[0]=MY_PAN_ID;
	
	j=0;
	k=0;
	l=0;
	m=0;
	if(certMode){
		PrintMenuC();
		
		while(1){
			ConsoleProcC();
		}
		
	} else {
		PrintMenu();
		
		while(1){
			//SYSTEM_RunTasks();
			if( MiApp_MessageAvailable() )
			{
				printf("\n\r[Data Received]\n\r");
				printf("SRC PID: %x\r\n",rxMessage.SourcePANID);
				printf("SRC ADR: %.2x%.2x\r\n",rxMessage.SourceAddress[1],rxMessage.SourceAddress[0]);
				printf("Length: %d\r\n",rxMessage.PayloadSize);
				printf("RSSI: %ddBm\r\n",rxMessage.PacketRSSI);
				
				printf("Payload: ");
				for(i=0;i<rxMessage.PayloadSize;i++){
					printf("%.2x ",*(uint8_t*)(rxMessage.Payload+i));
				}
				printf("\n\r");
				printf("RX CNT : %6D , TX CNT : %6D ",RX_CNT,TX_CNT);
				printf("\n\r");
				RX_CNT++;
				TX_CNT++;
				
				
				MiApp_DiscardMessage();
			}
			
			ConsoleProc();
			 
		}		
	}		
}

/* Initializes all the hardware and software modules used for Stack operation */
static void driver_init(void)
{
    /* Initialize the Radio Hardware */
    HAL_RadioInit();
    /* Initialize the AES Hardware Engine */
    AESInit();
    /* Initialize the Software Timer Module */
    SystemTimerInit();
#ifdef CONF_PMM_ENABLE
    /* Initialize the Sleep Timer Module */
    SleepTimerInit();
#endif
#if (ENABLE_PDS == 1)
    /* PDS Module Init */
    PDS_Init();
#endif
}

#ifdef CONF_PMM_ENABLE
static void app_resources_uninit(void)
{
    /* Disable USART TX and RX Pins */
    struct port_config pin_conf;
    port_get_config_defaults(&pin_conf);
    pin_conf.powersave  = true;
    port_pin_set_config(HOST_SERCOM_PAD0_PIN, &pin_conf);
    port_pin_set_config(HOST_SERCOM_PAD1_PIN, &pin_conf);
    /* Disable UART module */
    sio2host_deinit();
    /* Disable Transceiver SPI Module */
    HAL_RadioDeInit();
}
#endif
/**
 End of File
 */
