/*
 * crc.c
 *
 * Created: 2019-09-04 오후 8:59:10
 *  Author: A14633
 */ 

#include "sys.h"
#include "stdint.h"

uint16_t CRC16(uint8_t *ptr,  uint8_t count, uint16_t initCRC)
{
	uint16_t crc;
	uint8_t i,j;
	
	crc = initCRC;
	//while(--count >= 0 )
	for(j=0;j<count;j++)
	{
		crc = crc ^ ((uint16_t) *ptr++ << 8);
		for(i = 0; i < 8; i++)
		{
			if( crc & 0x8000 )
			{
				crc = (crc << 1) ^ 0x1021;
			}
			else
			{
				crc = crc << 1;
			}
		}
	}
	
	return crc;
}
//uint16_t CRC16(uint8_t *ptr, signed char count, uint16_t initCRC)
//{
	//uint16_t crc;
	//uint8_t i;
	//
	//crc = initCRC;
	//while(--count >= 0 )
	//{
		//crc = crc ^ ((uint16_t) *ptr++ << 8);
		//for(i = 0; i < 8; i++)
		//{
			//if( crc & 0x8000 )
			//{
				//crc = (crc << 1) ^ 0x1021;
			//}
			//else
			//{
				//crc = crc << 1;
			//}
		//}
	//}
	//
	//return crc;
//}