
#include <xc.h>
#define _XTAL_FREQ 6000000  // 6MHz crystal frequency

void lcd_command(unsigned char i){
    PORTB&=~0x08; 
    PORTD=i;
    PORTB |=0x01; //EN AT RB0
    PORTB &= ~0x01; //EN is 0 at RB0
    __delay_ms(100);
}

void lcd_data(unsigned char i){
    PORTB |=0x08;
    PORTD=i;
    PORTB |=0x01; //EN AT RB0
    PORTB &= ~0x01; //EN is 0 at RB0
    __delay_ms(100);
    
}

void lcd_number_convert(unsigned int num) {
    char digits[4];  // 3 characters + null terminator
    digits[3] = '\0';

    // Extract least significant digit
    digits[2] = (num % 10) + '0';
    num /= 10;

    // Extract middle digit or add space
    if (num > 0)
        digits[1] = (num % 10) + '0';
    else
        digits[1] = ' ';
    num /= 10;

    // Extract most significant digit or add space
    if (num > 0)
        digits[0] = (num % 10) + '0';
    else
        digits[0] = ' ';

    // Display all 3 characters
    lcd_data(digits[0]);
    lcd_data(digits[1]);
    lcd_data(digits[2]);
}

void lcd_init(){
    //In port B we have Pwm
    TRISB=0x00;
    TRISD=0x00;
    PORTD=0x00;
    PORTB=0x00;
            //LCD configurations
        lcd_command(0x30);
        __delay_ms(100);
        lcd_command(0x30);
        __delay_ms(100);
        lcd_command(0x30);
        __delay_ms(100);
        lcd_command(0x38);
        __delay_ms(100);
        lcd_command(0x06); //Set cursor shift right
        __delay_ms(100);
        lcd_command(0x0C); //Display on cursor off
        __delay_ms(100);
        lcd_command(0x01); //Clear display
        __delay_ms(100);
        
}