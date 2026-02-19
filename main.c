/*
 * File:   main.c
 * Author: Ashton
 *
 * Created on February 5, 2026, 9:36 AM
 */


#include "xc.h"
#include <xc.h>
#include <stdint.h>
#include <stdio.h>

#pragma config POSCMOD = NONE
#pragma config I2C1SEL = PRI
#pragma config OSCIOFNC = OFF
#pragma config FCKSM = CSECMD
#pragma config FNOSC = FRCPLL
#pragma config FWDTEN = OFF
#pragma config ICS = PGx2
#pragma config JTAGEN = OFF

void uart1_init(void) {
    __builtin_write_OSCCONL(OSCCON & ~(1<<6)); // Unlock PPS
    RPOR4bits.RP8R = 3;           // Use RP8 for U1TX
    RPINR18bits.U1RXR = 9;        // Use RP9 for U1RX
    __builtin_write_OSCCONL(OSCCON | (1<<6));  // Lock PPS

    U1MODE = 0x8000;   // Enable UART 8N1
    U1STA  = 0x0400;   // Enable TX
    U1BRG  = 51;       // 9600 baud @ Fcy=8MHz
}

int __C30_UART = 1;  // redirect printf to UART

void adc_init(void) {
    AD1CON1 = 0x00E0;        // auto-convert after sampling
    AD1CON2 = 0x0000;        // single channel
    AD1CON3 = 0x1F02;        // TAD
    AD1CHS  = 0;             // use AN0 (pin 2)
    AD1PCFG = 0xFFFE;        // all digital except AN0
    AD1CON1bits.ADON = 1;
}

uint16_t adc_read(void) {
    AD1CON1bits.SAMP = 1;
    for (volatile int i = 0; i < 640; i++);  // sampling delay ~80us
    AD1CON1bits.SAMP = 0;
    while (!AD1CON1bits.DONE);
    return ADC1BUF0;
}

void timer1_init(void) {
    T1CON = 0;          // reset Timer1 config
    TMR1 = 0;           // clear counter
    PR1 = 8000;         // 1 ms period @ Fcy=8MHz w/ 1:1 prescaler
    T1CONbits.TCKPS = 0; // 1:1 prescaler
    T1CONbits.TON = 1;   // turn on Timer1
}

void delay_ms(uint16_t ms) {
    while (ms--) {
        TMR1 = 0;              // reset timer
        while (!IFS0bits.T1IF); // wait for overflow
        IFS0bits.T1IF = 0;      // clear flag
    }
}

int main(void) {
    uart1_init();
    adc_init();
    timer1_init();

    while (1) {
        uint16_t val = adc_read();
        printf("%u\n", val);
        delay_ms(7);
    }
}
