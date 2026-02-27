#define _XTAL_FREQ 6000000  // 6MHz crystal frequency
#include <xc.h>

void pwm_init()
{
        //PWM configurations
        TRISC=0xFB; //RC2 as output rest of the bits as Input
        CCP1CON =0x0C;
        T2CON=0x06;
        PR2=0x5E;
        
}
