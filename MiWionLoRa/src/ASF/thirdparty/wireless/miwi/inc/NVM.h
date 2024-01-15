/*
 * NVM.h
 *
 * Created: 2019-08-08 오후 1:17:09
 *  Author: A14633
 */ 


#ifndef NVM_H_
#define NVM_H_

#include "MiWiAPI.h"

#if defined(ENABLE_NVM)


/******************************************************************************/
// there are three ways to use NVM to store data: External EPROM, Data EEPROM
// and programming space. Definition should be found in HardwareProfile.h
/******************************************************************************/
//#define USE_EXTERNAL_EEPROM		//24AA02E64 128byte. EUI64.
//#define USE_DATA_EEPROM			//1024byte. USE_EXTERNAL_EEPROM 와 함께 사용시 EXTERNAL_EEPROM은 EUI64만 사용.
#define USE_PROGRAMMING_SPACE

#if defined(USE_EXTERNAL_EEPROM)

#define SPI_WRT_STATUS  0x01
#ifndef SPI_WRITE
#define SPI_WRITE       0x02
#endif
#ifndef SPI_READ
#define SPI_READ        0x03
#endif
#define SPI_DIS_WRT     0x04
#define SPI_RD_STATUS   0x05
#define SPI_EN_WRT      0x06

//24AA02E48,24AA02E64(RN2903)
//#define EEPROM_MAC_EUI_48
#define EEPROM_MAC_EUI_64	//(RN2903)
//현재 마이크로칩은 OUI 24bit 0x0004A3, 0X001EC0, 0xD88039, 0x5410EC 를 사용하고 있지만 향후 추가 변경될수 있음.

#if	defined(EEPROM_MAC_EUI_48)
#define EEPROM_MAC_ADDR 0xFA		//EUI-48,0xFA~(24-bit OUI,24-bit Extension ID)~ 0xFF
#elif defined(EEPROM_MAC_EUI_64)
#define EEPROM_MAC_ADDR 0xF8		//EUI-64,0xF8~(24-bit OUI,40-bit Extension ID)~ 0xFF(RN2903)
#endif

#ifndef USE_DATA_EEPROM
#define MCHP_1KBIT              1
#define MCHP_2KBIT              2
#define MCHP_4KBIT              3
#define MCHP_8KBIT              4
#define MCHP_16KBIT             5
#define MCHP_32KBIT             6
#define MCHP_64KBIT             7
#define MCHP_128KBIT            8
#define MCHP_256KBIT            9
#define MCHP_512KBIT            10
#define MCHP_1MBIT              11

#define MCHP_EEPROM             MCHP_8KBIT


#if MCHP_EEPROM == 0
#error MCHP_EEPROM is not defined
#elif MCHP_EEPROM < MCHP_32KBIT
#define NVM_PAGE_SIZE   16
#elif MCHP_EEPROM < MCHP_128KBIT
#define NVM_PAGE_SIZE   32
#elif MCHP_EEPROM < MCHP_512KBIT
#define NVM_PAGE_SIZE   64
#elif MCHP_EEPROM < MCHP_1MBIT
#define NVM_PAGE_SIZE   128
#elif MCHP_EEPROM == MCHP_1MBIT
#error Microchip 1MBit EEPROM is not supported at this time. User need to modify the EEPROM access function to make it work. The address must be 3 bytes.
#else
#error Invalid MCHP EEPROM part
#endif

#if MCHP_EEPROM == MCHP_1KBIT
#define TOTAL_NVM_BYTES 128
#elif MCHP_EEPROM == MCHP_2KBIT
#define TOTAL_NVM_BYTES 256
#elif MCHP_EEPROM == MCHP_4KBIT
#define TOTAL_NVM_BYTES 512
#elif MCHP_EEPROM == MCHP_8KBIT
#define TOTAL_NVM_BYTES 1024
#elif MCHP_EEPROM == MCHP_16KBIT
#define TOTAL_NVM_BYTES 2048
#elif MCHP_EEPROM == MCHP_32KBIT
#define TOTAL_NVM_BYTES 4096
#elif MCHP_EEPROM == MCHP_64KBIT
#define TOTAL_NVM_BYTES 8192
#elif MCHP_EEPROM == MCHP_128KBIT
#define TOTAL_NVM_BYTES 16384
#elif MCHP_EEPROM == MCHP_256KBIT
#define TOTAL_NVM_BYTES 32768
#elif MCHP_EEPROM == MCHP_512KBIT
#define TOTAL_NVM_BYTES 65535
#elif MCHP_EEPROM == MCHP_1MBIT
#error Microchip 1MBit EEPROM is not supported at this time. User need to modify the EEPROM access function to make it work. The address must be 3 bytes.
#else
#error MCHP_EEPROM is not defined
#endif
#endif

#endif

#if defined(USE_PROGRAMMING_SPACE)

#endif

#if defined(USE_DATA_EEPROM)
	#define TOTAL_NVM_BYTES     1024
#endif

extern uint16_t        nvmMyMemory;
extern uint16_t        nvmMyPANID;
extern uint16_t        nvmCurrentChannel;
extern uint16_t        nvmConnMode;
extern uint16_t        nvmConnectionTable;
extern uint16_t        nvmOutFrameCounter;

//MiWi PRO
extern uint16_t        nvmMyShortAddress;
extern uint16_t        nvmMyParent;

extern uint16_t    nvmRoutingTable;
extern uint16_t    nvmFamilyTree;
extern uint16_t    nvmNeighborRoutingTable;
extern uint16_t    nvmRole;


void NVMRead(uint8_t *dest, uint16_t addr, uint16_t count);
void NVMWrite(uint8_t *source, uint16_t addr, uint16_t count);

void i2c_EEPROM_Read(uint8_t *dest, uint16_t address, uint16_t count);
void i2c_EEPROM_Write(uint8_t *source, uint16_t address, uint16_t count);

bool NVMInit(void);

void NVM_Init();
void NVM_Write();
void NVM_Read();
void NVM_Write_CH(uint8_t channel);
uint8_t NVM_Read_CH();

#if defined(USE_DATA_EEPROM) || defined(USE_PROGRAMMING_SPACE)
	#define nvmGetMyMemory( x )                  NVMRead( (uint8_t *)x, nvmMyMemory, USER_MEMORY_SIZE)
	#define nvmPutMyMemory( x )                  NVMWrite((uint8_t *)x, nvmMyMemory, USER_MEMORY_SIZE)

	#define nvmGetMyPANID( x )                  NVMRead( (uint8_t *)x, nvmMyPANID, 2)
	#define nvmPutMyPANID( x )                  NVMWrite((uint8_t *)x, nvmMyPANID, 2)

	#define nvmGetCurrentChannel( x )           NVMRead( (uint8_t *)x, nvmCurrentChannel, 1)
	#define nvmPutCurrentChannel( x )           NVMWrite((uint8_t *)x, nvmCurrentChannel, 1)

	#define nvmGetConnMode( x )                 NVMRead( (uint8_t *)x, nvmConnMode, 1)
	#define nvmPutConnMode( x )                 NVMWrite((uint8_t *)x, nvmConnMode, 1)

	#define nvmGetConnectionTable( x )          NVMRead( (uint8_t *)x, nvmConnectionTable, (uint16_t)CONNECTION_SIZE * sizeof(CONNECTION_ENTRY))
	#define nvmPutConnectionTable( x )          NVMWrite((uint8_t *)x, nvmConnectionTable, (uint16_t)CONNECTION_SIZE * sizeof(CONNECTION_ENTRY))
	#define nvmPutConnectionTableIndex(x, y)    NVMWrite((uint8_t *)x, nvmConnectionTable+((uint16_t)y * sizeof(CONNECTION_ENTRY)), sizeof(CONNECTION_ENTRY))

	#define nvmGetOutFrameCounter( x )          NVMRead( (uint8_t *)x, nvmOutFrameCounter, 4)
	#define nvmPutOutFrameCounter( x )          NVMWrite((uint8_t *)x, nvmOutFrameCounter, 4)
	#else
	#define nvmGetMyPANID( x )                  i2c_EEPROM_Read( (uint8_t *)x, nvmMyPANID, 2)
	#define nvmPutMyPANID( x )                  i2c_EEPROM_Write((uint8_t *)x, nvmMyPANID, 2)

	#define nvmGetCurrentChannel( x )           i2c_EEPROM_Read( (uint8_t *)x, nvmCurrentChannel, 1)
	#define nvmPutCurrentChannel( x )           i2c_EEPROM_Write((uint8_t *)x, nvmCurrentChannel, 1)

	#define nvmGetConnMode( x )                 i2c_EEPROM_Read( (uint8_t *)x, nvmConnMode, 1)
	#define nvmPutConnMode( x )                 i2c_EEPROM_Write((uint8_t *)x, nvmConnMode, 1)

	#define nvmGetConnectionTable( x )          i2c_EEPROM_Read( (uint8_t *)x, nvmConnectionTable, (uint16_t)CONNECTION_SIZE * sizeof(CONNECTION_ENTRY))
	#define nvmPutConnectionTable( x )          i2c_EEPROM_Write((uint8_t *)x, nvmConnectionTable, (uint16_t)CONNECTION_SIZE * sizeof(CONNECTION_ENTRY))
	#define nvmPutConnectionTableIndex(x, y)    i2c_EEPROM_Write((uint8_t *)x, nvmConnectionTable+((uint16_t)y * sizeof(CONNECTION_ENTRY)), sizeof(CONNECTION_ENTRY))

	#define nvmGetOutFrameCounter( x )          i2c_EEPROM_Read( (uint8_t *)x, nvmOutFrameCounter, 4)
	#define nvmPutOutFrameCounter( x )          i2c_EEPROM_Write((uint8_t *)x, nvmOutFrameCounter, 4)
#endif


#if defined(PROTOCOL_MIWI_PRO)
	#if defined(USE_DATA_EEPROM) || defined(USE_PROGRAMMING_SPACE)
		#define nvmGetMyShortAddress( x )       NVMRead( (uint8_t *)x, nvmMyShortAddress, 2)
		#define nvmPutMyShortAddress( x )       NVMWrite((uint8_t *)x, nvmMyShortAddress, 2)

		#define nvmGetMyParent( x )             NVMRead( (uint8_t *)x, nvmMyParent, 1)
		#define nvmPutMyParent( x )             NVMWrite((uint8_t *)x, nvmMyParent, 1)

		#define nvmGetRoutingTable( x )         NVMRead( (uint8_t *)x, nvmRoutingTable, NUM_COORDINATOR/8)
		#define nvmPutRoutingTable( x )         NVMWrite((uint8_t *)x, nvmRoutingTable, NUM_COORDINATOR/8)

		#define nvmGetNeighborRoutingTable( x ) NVMRead( (uint8_t *)x, nvmNeighborRoutingTable, ((uint16_t)NUM_COORDINATOR/8) * ((uint16_t)NUM_COORDINATOR))
		#define nvmPutNeighborRoutingTable( x ) NVMWrite((uint8_t *)x, nvmNeighborRoutingTable, ((uint16_t)NUM_COORDINATOR/8) * ((uint16_t)NUM_COORDINATOR))

		#define nvmGetFamilyTree( x )           NVMRead( (uint8_t *)x, nvmFamilyTree, NUM_COORDINATOR)
		#define nvmPutFamilyTree( x ) NVMWrite((uint8_t *)x, nvmFamilyTree, NUM_COORDINATOR)

		#define nvmGetRole( x )                 NVMRead( (uint8_t *)x, nvmRole, 1)
		#define nvmPutRole( x )                 NVMWrite((uint8_t *)x, nvmRole, 1)
#endif

#if defined(NWK_ROLE_COORDINATOR)
	#if (USER_MEMORY_SIZE + 2+1+1+((7+MY_ADDRESS_LENGTH) * CONNECTION_SIZE)+4+2+1 + (NUM_COORDINATOR/8)+(NUM_COORDINATOR/8*NUM_COORDINATOR)+NUM_COORDINATOR+1 > TOTAL_NVM_BYTES)
		//#error NETWORK_FREEZER Memory Over
	#endif
#else
//END Device
	#if (USER_MEMORY_SIZE + 2+1+1+((7+MY_ADDRESS_LENGTH) * CONNECTION_SIZE)+4+2+1 > TOTAL_NVM_BYTES)
		//#error NETWORK_FREEZER Memory Over
	#endif
#endif


#endif

#if 0
#if defined(PROTOCOL_MIWI_PRO)

#define nvmGetMyShortAddress( x )       NVMRead( (uint8_t *)x, (far ROM void *)&nvmMyShortAddress, sizeof(WORD_VAL))
#define nvmPutMyShortAddress( x )       NVMWrite((uint8_t *)x, (ROM uint8_t *)&nvmMyShortAddress, sizeof(WORD_VAL))

#define nvmGetMyParent( x )             NVMRead( (uint8_t *)x, (far ROM void *)&nvmMyParent, 1)
#define nvmPutMyParent( x )             NVMWrite((uint8_t *)x, (ROM uint8_t *)&nvmMyParent, 1)

	#if defined(NWK_ROLE_COORDINATOR)

		#define nvmGetRoutingTable( x )         NVMRead( (uint8_t *)x, (far ROM void *)&nvmRoutingTable, NUM_COORDINATOR)
		#define nvmPutRoutingTable( x )         NVMWrite((uint8_t *)x, (ROM uint8_t *)&nvmRoutingTable, NUM_COORDINATOR)

		#define nvmGetNeighborRoutingTable( x ) NVMRead((uint8_t *)x, (far ROM void *)&nvmNeighborRoutingTable, ((uint16_t)NUM_COORDINATOR/8 * (uint16_t)NUM_COORDINATOR))
		#define nvmPutNeighborRoutingTable( x ) NVMWrite((uint8_t *)x, (ROM uint8_t *)&nvmNeighborRoutingTable, ((uint16_t)NUM_COORDINATOR/8 * (uint16_t)NUM_COORDINATOR))

		#define nvmGetFamilyTree( x )           NVMRead((uint8_t *)x, (far ROM void *)&nvmFamilyTree, NUM_COORDINATOR)
		#define nvmPutFamilyTree( x )           NVMWrite((uint8_t *)x, (ROM uint8_t *)&nvmFamilyTree, NUM_COORDINATOR)

		#define nvmGetRole( x )                 NVMRead( (uint8_t *)x, (far ROM void *)&nvmRole, 1)
		#define nvmPutRole( x )                 NVMWrite((uint8_t *)x, (ROM uint8_t *)&nvmRole, 1)

	#endif

#endif
#endif

#endif

#endif /* NVM_H_ */