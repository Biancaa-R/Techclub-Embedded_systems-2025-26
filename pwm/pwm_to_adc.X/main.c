/*
 * File:   adc_1.c
 * Author: Biancaa. R

 */

// PIC16F877A Configuration Bit Settings

// CONFIG
#pragma config FOSC = EXTRC     // Oscillator Selection bits (RC oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#include "lcd_drivers.h"
#include "adc_drivers.h"
#include "pwm_drivers.h"
#include <xc.h>
#define _XTAL_FREQ 6000000  // 6MHz crystal frequency

void adc_lcd_init();
void lcd_number_convert(unsigned int i);
void lcd_command(unsigned char);
void lcd_data(unsigned char);

extern unsigned int adc_calibrated_data,adc_volt;
extern unsigned int adc_high,adc_low;

void main()
{
    adc_lcd_init();
    while(1){
        ADCON0 |=0x04;
        while(ADCON0 & 0x04); //Cearing go done bit
        adc_high= ADRESH;
        adc_low=ADRESL;
        adc_volt =(adc_high<<8)+adc_low;
        adc_calibrated_data=((unsigned long)adc_volt*338)/1024;
        CCPR1L =(adc_calibrated_data >> 2);
        CCP1CON =((CCP1CON & 0xCF) | ((adc_calibrated_data)&0x03) <<4);
        lcd_command(0x80);
        lcd_number_convert(adc_calibrated_data);
    }
}
    
void adc_lcd_init()
    {
    lcd_init();
    pwm_init();
    adc_init();
}
