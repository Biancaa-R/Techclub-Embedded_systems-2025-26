#include <xc.h>
#include <stdint.h>
#define _XTAL_FREQ 8000000UL

// CONFIG BITS
#pragma config OSC = INTIO67, FCMEN = OFF, IESO = OFF
#pragma config PWRT = OFF, BOREN = OFF, BORV = 3
#pragma config WDT = OFF, WDTPS = 32768
#pragma config CCP2MX = PORTC, PBADEN = ON, LPT1OSC = OFF
#pragma config STVREN = ON, LVP = OFF, XINST = OFF
#pragma config CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF
#pragma config CPB = OFF, CPD = OFF
#pragma config WRT0 = OFF, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF
#pragma config WRTC = OFF, WRTB = OFF, WRTD = OFF
#pragma config EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF
#pragma config EBTRB = OFF
#pragma config MCLRE = OFF  // Use RE3 as digital input, MCLR internally tied


// UART Functions
void UART_Init(void) {
    TRISCbits.TRISC6 = 0;  // TX pin output
    TRISCbits.TRISC7 = 1;  // RX pin input

    SPBRG = 51;            // 9600 baud @ 8MHz
    SPBRGH = 0;

    TXSTAbits.SYNC = 0;    // Async mode
    TXSTAbits.BRGH = 1;    // High speed
    BAUDCONbits.BRG16 = 0; // 8-bit baud

    RCSTAbits.SPEN = 1;    // Enable serial port
    TXSTAbits.TXEN = 1;    // Enable TX
    RCSTAbits.CREN = 1;    // Enable RX
}

void UART_Write(char data) {
    while(!PIR1bits.TXIF); // Wait for TX buffer
    TXREG = data;
}

void UART_Write_Text(const char* text) {
    while(*text) {
        UART_Write(*text++);
    }
}

char UART_Read(void) {
    while(!PIR1bits.RCIF); // Wait for data
    return RCREG;
}

void main(void) {
    OSCCON = 0x76;       // 8 MHz internal oscillator
    TRISB = 0x00;        // LEDs as output
    TRISA =0xFF;

    UART_Init();
    __delay_ms(500);

    UART_Write_Text("Welcome to Newyork\r\n");
    UART_Write_Text("Press 1 for RedLED, 2 for YellowLED, 3 for GreenLED\r\n");

    while(1) {
        char data = UART_Read();
        char c = RCREG;          // Read the data
        UART_Write(data); 
        LATB = (LATB & 0xF8) | (c & 0x07); // Optional: debug LEDs
        if (PORTAbits.RA0 ==0) data = '1';
        else if (PORTAbits.RA1 ==0) data = '2';
        else if (PORTAbits.RA2 ==0) data = '3';
        switch(data) {
            case '1': PORTB = 0b001; break;
            case '2': PORTB = 0b010; break;
            case '3': PORTB = 0b100; break;
            default:  PORTB = 0b000; break;
        }
        __delay_ms(200);
    }
}
