/* 
 * File:   lcd_drivers.h
 * Author: Biancaa. R
 *
 * Created on 9 June, 2025, 12:25 AM
 */

#ifndef LCD_DRIVERS_H
#define	LCD_DRIVERS_H

#ifdef	__cplusplus
extern "C" {
#endif

void lcd_command(unsigned char i);
void lcd_data(unsigned char i);
void lcd_number_convert(unsigned int num);
void lcd_init();
#ifdef	__cplusplus
}
#endif

#endif	/* LCD_DRIVERS_H */

