/*
 * SymbolTime.h
 *
 * Created: 2019-08-08 오전 3:34:24
 *  Author: A14633
 */ 


#ifndef SYMBOLTIME_H_
#define SYMBOLTIME_H_

#include "sys.h"
#include "tc.h"

#if 0
	#define ONE_SECOND (((DWORD)CLOCK_FREQ/1000 * 62500) / (SYMBOL_TO_TICK_RATE / 1000))
	/* SYMBOLS_TO_TICKS to only be used with input (a) as a constant, otherwise you will blow up the code */
	#define SYMBOLS_TO_TICKS(a) (((DWORD)CLOCK_FREQ/100000) * a / ((DWORD)SYMBOL_TO_TICK_RATE/100000))
	#define TICKS_TO_SYMBOLS(a) (((DWORD)SYMBOL_TO_TICK_RATE/100000) * a / ((DWORD)CLOCK_FREQ/100000))
	#define ONE_TICK_16USEC		16	//usec
#else
	#define ONE_SECOND 4096
	/* SYMBOLS_TO_TICKS to only be used with input (a) as a constant, otherwise you will blow up the code */
	#define SYMBOLS_TO_TICKS(a) (1*a)
	#define TICKS_TO_SYMBOLS(a) (1*a)
	#define ONE_TICK_16USEC		16	//usec
#endif

#define ONE_MILI_SECOND     (ONE_SECOND/1000)
#define HUNDRED_MILI_SECOND (ONE_SECOND/10)
#define FORTY_MILI_SECOND   (ONE_SECOND/25)
#define FIFTY_MILI_SECOND   (ONE_SECOND/20)
#define TWENTY_MILI_SECOND  (ONE_SECOND/50)
#define TEN_MILI_SECOND     (ONE_SECOND/100)
#define FIVE_MILI_SECOND    (ONE_SECOND/200)
#define TWO_MILI_SECOND     (ONE_SECOND/500)
#define ONE_MINUTE          (ONE_SECOND*60)
#define ONE_HOUR            (ONE_MINUTE*60)

#define MiWi_TickGetDiff(a,b) (a.Val - b.Val)

/************************ DATA TYPES *******************************/


/******************************************************************
 // Time unit defined based on IEEE 802.15.4 specification.
 // One tick is equal to one symbol time, or 16us. The Tick structure
 // is four bytes in length and is capable of represent time up to
 // about 19 hours.
 *****************************************************************/
typedef union _MIWI_TICK
{
    uint32_t Val;
    struct _MIWI_TICK_bytes
    {
        uint8_t b0;
        uint8_t b1;
        uint8_t b2;
        uint8_t b3;
    } byte;
    uint8_t v[4];
    struct _MIWI_TICK_words
    {
        uint16_t w0;
        uint16_t w1;
    } word;
} MIWI_TICK;

void InitSymbolTimer(void);
MIWI_TICK MiWi_TickGet(void);

/************************ VARIABLES ********************************/

//extern uint8_t timerExtension1,timerExtension2;



#endif /* SYMBOLTIME_H_ */