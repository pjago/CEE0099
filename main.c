/*
 * File:   main.c
 * Author: pjago, pedro.m4rtins@gmail.com
 * 
 * Created on September 4, 2017, 10:11 AM
 */

// CONFIG
#pragma config FOSC = INTOSCIO  // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA4/OSC2/CLKOUT pin, I/O function on RA5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select bit (MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Detect (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)

#include <conio.h>
#include <stdio.h>
#include <xc.h>

#define _XTAL_FREQ 8000000      // FOSC = 8 Mhz. @delay
#define ONEBIT 104              // BAUD = 19200 bps

#define WBIT(F) { \
asm("RRF " F ", f"); \
asm("MOVF STATUS, w"); \
asm("MOVWF GPIO"); \
}

#define RBIT(F) { \
asm("RRF " F ", f"); \
asm("BTFSC GPIO, 1"); \
asm("BSF " F ", 7"); \
}

unsigned char BUF[17] = "                ", RC = 0;
unsigned int T1ZOH = 0;

signed char PWM @ &CCPR1L;
bit TX @ (((unsigned) &GPIO)*8) + 0;
bit RX @ (((unsigned) &GPIO)*8) + 1;

// UX

unsigned char SENT;
void rsend (unsigned char msg) {
    /* Basic Logic
       TX pin is usually high. A high to low bit is the starting bit and 
       a low to high bit is the ending bit. No parity bit. No flow control.
       BitCount is the number of bits to transmit. Data is transmitted LSB first.
    */
    SENT = msg; 	       // this magically solves the sending 0x00 bug
    asm("BCF GPIO, 0");    _delay(ONEBIT - 1);
    WBIT("rsend@msg");     _delay(ONEBIT - 3);
    WBIT("rsend@msg");     _delay(ONEBIT - 3);
    WBIT("rsend@msg"); 	   _delay(ONEBIT - 3);
    WBIT("rsend@msg");     _delay(ONEBIT - 3);
    WBIT("rsend@msg");     _delay(ONEBIT - 3);
    WBIT("rsend@msg");     _delay(ONEBIT - 3);
    WBIT("rsend@msg");     _delay(ONEBIT - 3);
    WBIT("rsend@msg");     _delay(ONEBIT - 3);
    asm("BSF GPIO, 0");    _delay(ONEBIT - 1);
}

unsigned char rsget () {
    unsigned char msg = 0;
    while (RX==1) continue;
    _delay(ONEBIT - 1);
    _delay(ONEBIT >> 1);   // Take sample value in the mid of bit duration
    RBIT("rsget@msg"); 	   _delay(ONEBIT - 3);
    RBIT("rsget@msg"); 	   _delay(ONEBIT - 3);
    RBIT("rsget@msg"); 	   _delay(ONEBIT - 3);
    RBIT("rsget@msg"); 	   _delay(ONEBIT - 3);
    RBIT("rsget@msg"); 	   _delay(ONEBIT - 3);
    RBIT("rsget@msg"); 	   _delay(ONEBIT - 3);
    RBIT("rsget@msg"); 	   _delay(ONEBIT - 3);
    RBIT("rsget@msg"); 	   _delay(ONEBIT - 3);
    _delay(ONEBIT >> 1);   // This better be the stop bit
    return msg;
}

// INTERFACE

void read_tmr1 () {
    T1ZOH = TMR1;
    rsend(T1ZOH >> 8);
    rsend(T1ZOH);
    TMR1 -= T1ZOH;
    return;
}

void write (signed char duty) {
    if (duty > 100) PWM = 100;
    else if (duty < 0) PWM = 0;
    else PWM = duty;
}

// MAIN CODE
int main (void) {
    OSCCON = 0x77;         // Internal 8 Mhz clock
    OPTION_REG = 0x07;     // GPIO pull ups can be enabled
    T1CON = 0x03;          // TMR1 external clock, TMR1/1
    PR2 = 100;             // PWM period = [(PR2)+1]*4*TOSC*T2CKPS
    T2CON = 0x04;          // TMR2 enabled at maximum frequency
    CCP1CON = 0x0C;        // PWM mode active high
    CMCON0 = 0x07;         // Comparator Disabled
    ANSEL = 0x00;          // No pins as analog pins
    ADCON0 = 0x00;         // No analog conversion
    WPU = 0x22;            // Enable weak pull-up at TACH, RX
    TRISIO = 0x2A;         // TRISIO = TACH, CLKO, MCRL, CCP1, RX, TX
    TMR1 = 0;
    PWM = 0;
    TX = 1;	               // TX pin is high in idle state
    while (1) {
        char x = rsget();
        if (x == '\n') {   //bug when we write pwm = 10 @fmarcolino
            switch (BUF[0]) {
                case 'r': case '7': read_tmr1(); break;
                case 'w': case '5': write(BUF[1]); break;
                case 'x': case '1': write(BUF[1]); read_tmr1(); break;
                case 's': case '2': write(0); break;
            }
            RC = 0;
        }
        else {
            BUF[RC] = x;
            RC++;
        }
    }
    return 0;
}