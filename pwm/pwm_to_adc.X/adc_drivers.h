/* 
 * File:   adc_drivers.h
 * Author: Biancaa. R
 *
 * Created on 9 June, 2025, 12:18 AM
 */

#ifndef ADC_DRIVERS_H
#define	ADC_DRIVERS_H

#ifdef	__cplusplus
extern "C" {
#endif

unsigned int adc_calibrated_data,adc_volt;
unsigned int adc_high,adc_low;
void adc_init();

#ifdef	__cplusplus
}
#endif

#endif	/* ADC_DRIVERS_H */

