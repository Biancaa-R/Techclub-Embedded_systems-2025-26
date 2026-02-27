#include <xc.h>
#define _XTAL_FREQ 6000000  // 6MHz crystal frequency
void adc_init()
{
    
        //ADC initialization
        ADCON1 =0x8E;
        ADCON0=0x81;
        __delay_ms(10); //acquisition time
}