/* 
 * File:   main.c
 * Author: Biancaa. R
 *
 * Created on 24 July, 2025, 2:01 AM
 */
// CONFIG
#pragma config FOSC = EXTRC     // Oscillator Selection bits (RC oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>

#define _XTAL_FREQ 20000000
#define RS RC2
#define EN RC1
void init(void);
void i2c_init(const unsigned long);
void i2c_write(unsigned char);
int i2c_read(int);
void i2c_start();
void i2c_wait();
void i2c_stop();
void i2c_command(unsigned char);
void lcd_data(unsigned char);
int bcd_2_dec(int);
int dec_2_bcd(int);
void settime(void);
void update(void);
char msg1[5]={"TIME:"};
char msg2[5]={"DATE:"};
int i,j,k,l;
int sec=56;
int min=59;
int hour=23;
int date=28;
int month=12;
int year=25;
char sec1,sec2,min1,min2,hour1,hour2,date1,date2,month1,month2,year1,year2;
int main(int argc, char** argv) {
    init();
    i2c_init(100);
    settime();
    while(1){
        update();
        sec1=sec/10;
        sec2=sec%10;
        min1=min/10;
        min2=min%10;
        hour1=hour/10;
        hour2=hour%10;
        date1=date/10;
        date2=date%10;
        month1=month/10;
        month2=month%10;
        year1=year/10;
        year2=year%10;
        //data at line 1
        //data at line 2
        for(int i=0;i<5;i++){
            lcd_data(msg2[i]);
        }
        lcd_data(date1+'0');
        lcd_data(date2+'0');
        lcd_data(0x2D);
        lcd_data(month1+'0');
        lcd_data(month2+'0');
        lcd_data(0x2D);
        lcd_data(year1+'0');
        lcd_data(year2+'0');
        __delay_ms(2000);
    }
    return (EXIT_SUCCESS);
}

void init(void){
    TRISD=0x00;
    TRISC=0x18;
    PORTD=0x00;//setting outputs as 0
    lcd_command(0x38);
    __delay_ms(100);
    lcd_command(0x38);
    __delay_ms(100);
    lcd_command(0x38);
    __delay_ms(100);
    lcd_command(0x38);
    __delay_ms(100);
    lcd_command(0x0C);
    __delay_ms(100);
    lcd_command(0x06);
    __delay_ms(100);
    lcd_command(0x01);
    __delay_ms(100);
}

void lcd_command(unsigned char i){
    
}

void lcd_data(unsigned char i){
    RS=1;
    PORTD=i;
    EN=1;
    EN=0;
    __delay_ms(5);
}

void i2c_init(const unsigned long freq_k){
    SSPCON=0x28;
    SSPSTAT=0x00;
    SSPCON2=0x00;
    SSPADD=(_XTAL_FREQ/(4*freq_k*100))-1;
}

void i2c_wait(){
    while(SSPCON2 &0xIF || SSPSTAT &0x04);
}

void i2c_start(){
    i2c_wait();
    SEN=1;
}