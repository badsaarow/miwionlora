/*
 * SymbolTime.c
 *
 * Created: 2019-08-10 오전 8:12:35
 *  Author: A14633
 */ 

#include "SymbolTime.h"

#define CONF_TC_MODULE TC3

uint8_t timerExtension1,timerExtension2;
struct tc_module tc_instance;

void tc_callback_to_toggle_led(struct tc_module *const module_inst)
{
	//port_pin_toggle_output_level(LED0_PIN);
}

void configure_tc_callbacks(void)
{
	tc_register_callback(&tc_instance, tc_callback_to_toggle_led,TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&tc_instance, TC_CALLBACK_OVERFLOW);
	
}

void InitSymbolTimer()
{
    timerExtension1 = 0;
    timerExtension2 = 0;
	
	struct tc_config config_tc;
	
	tc_get_config_defaults(&config_tc);
	config_tc.counter_size = TC_COUNTER_SIZE_16BIT;
	config_tc.clock_source = GCLK_GENERATOR_1;
	config_tc.clock_prescaler = TC_CLOCK_PRESCALER_DIV8;
	config_tc.counter_16_bit.value = 0x0000;
	config_tc.oneshot = false;
	tc_init(&tc_instance, CONF_TC_MODULE, &config_tc);
	tc_enable(&tc_instance);
	
	//configure_tc_callbacks();
	
	printf("Timer Init : %d Hz....\n\r", system_gclk_gen_get_hz(GCLK_GENERATOR_1));
}
	

MIWI_TICK MiWi_TickGet(void)
{
    MIWI_TICK currentTime;
    
    //BYTE failureCounter;
    uint8_t IntFlag1;
    uint8_t IntFlag2;
    
    /* copy the byte extension */
    currentTime.byte.b2 = 0;
    currentTime.byte.b3 = 0;
    
	//printf("TMR");
	
	currentTime.Val = tc_get_count_value(&tc_instance);
	//printf("%d\r\n",currentTime.Val);
	
	
    /* disable the timer to prevent roll over of the lower 16 bits while before/after reading of the extension */
    //TMR_IE = 0;
/*
    do
    {
        IntFlag1 = TMR_IF;
        currentTime.byte.b0 = TMR_L;
        currentTime.byte.b1 = TMR_H;
        IntFlag2 = TMR_IF;
    } while(IntFlag1 != IntFlag2);

    if( IntFlag1 > 0 )
    {
        //TMR_IF = 0;
        timerExtension1++;
        if(timerExtension1 == 0)
        {
            timerExtension2++;
        }
    }
*/
    /* copy the byte extension */
    currentTime.byte.b2 += timerExtension1;
    currentTime.byte.b3 += timerExtension2;
    
    /* enable the timer*/
    //TMR_IE = 1;

    return currentTime;
}