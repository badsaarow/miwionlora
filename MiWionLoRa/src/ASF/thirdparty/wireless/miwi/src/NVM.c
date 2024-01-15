/*
 * NVM.c
 *
 * Created: 2019-08-08 오후 3:20:08
 *  Author: A14633
 */ 

#include "NVM.h"
#include "nvm.h"

#define TEST_PAGE_ADDR     (FLASH_NB_OF_PAGES - 8) * NVMCTRL_PAGE_SIZE


 uint16_t   nvmMyMemory;
 uint16_t   nvmMyPANID;
 uint16_t   nvmCurrentChannel;
 uint16_t   nvmConnMode;
 uint16_t   nvmConnectionTable;
 uint16_t   nvmOutFrameCounter;
 
 uint16_t   nvmMyShortAddress;
 uint16_t   nvmMyParent;
 
 uint16_t   nvmRoutingTable;
 uint16_t   nvmNeighborRoutingTable;
 uint16_t   nvmFamilyTree;
 uint16_t   nvmRole;
 

void NVM_Init()
{
	struct nvm_config config;
	enum status_code status;

	/* Get the default configuration */
	nvm_get_config_defaults(&config);

	/* Set wait state to 1 */
	config.wait_states = 1;

	/* Enable automatic page write mode */
	config.manual_page_write = false;

	/* Set the NVM configuration */
	status = nvm_set_config(&config);
}


void NVM_Write()
{
	uint8_t buffer[NVMCTRL_PAGE_SIZE],i;
	enum status_code status;
	
	for(i=0;i<NVMCTRL_PAGE_SIZE;i++)
	{
		buffer[i] = i;
	}
	
	status = nvm_write_buffer(TEST_PAGE_ADDR, buffer, NVMCTRL_PAGE_SIZE);
	if(status != STATUS_OK){
		printf("NVM Write Fail");
	}
}

void NVM_Read()
{
	uint8_t buffer[NVMCTRL_PAGE_SIZE],i;
	enum status_code status;
	
	status = nvm_read_buffer(TEST_PAGE_ADDR, buffer, NVMCTRL_PAGE_SIZE);
	if(status != STATUS_OK){
		printf("NVM Write Fail");
	}
	
	printf("NVM Data: \n\r");
	for(i=0;i<NVMCTRL_PAGE_SIZE;i++)
	{
		printf("0x%x ",buffer[i]);
		if(i%16 == 15) printf("\n\r");
	}
}

void NVM_Write_CH(uint8_t channel)
{
	enum status_code status;
	uint8_t ch;
	
	ch = channel;
	status = nvm_write_buffer(TEST_PAGE_ADDR, &ch, 1);
	if(status != STATUS_OK){
		printf("NVM Write Fail");
	}else{
		printf("current channel %d \r\n",ch);
	}
}
void NVM_Write_DATA(uint8_t channel,uint16_t panid,uint8_t sf)
{
	enum status_code status;
	uint8_t ch;
	
	ch = channel;
	status = nvm_write_buffer(TEST_PAGE_ADDR, &ch, 1);
	if(status != STATUS_OK){
		printf("NVM Write Fail");
		}else{
		printf("current channel %d \r\n",ch);
	}
}
uint8_t NVM_Read_CH()
{
	enum status_code status;
	uint8_t ch=0;
	
	status = nvm_read_buffer(TEST_PAGE_ADDR, &ch, 1);
	if(status != STATUS_OK){
		printf("NVM Write Fail");
	}else{
		printf("read channel %2x \r\n",ch);
	}
	return ch;
}

bool NVMInit(void)
{
	NVM_Init();
	return true;
}

void NVMRead(uint8_t *dest, uint16_t addr, uint16_t count)
{
	
}

void NVMWrite(uint8_t *source, uint16_t addr, uint16_t count)
{
	
}

void i2c_EEPROM_Read(uint8_t *dest, uint16_t address, uint16_t count)
{
	
}

void i2c_EEPROM_Write(uint8_t *source, uint16_t address, uint16_t count)
{
	
}