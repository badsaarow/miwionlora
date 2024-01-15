/*
 * crc.h
 *
 * Created: 2019-09-04 오후 8:59:24
 *  Author: A14633
 */ 


#ifndef CRC_H_
#define CRC_H_
#include "sys.h"
#include "stdint.h"

uint8_t CRC16(uint8_t *ptr, signed char count, uint16_t initCRC);


#endif /* CRC_H_ */