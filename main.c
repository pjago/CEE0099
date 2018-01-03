/*
 * File:   main.c
 * Author: pjago, pedro.m4rtins@gmail.com
 * 
 * Created on September 4, 2017, 10:11 AM
 */

// CONFIG

#pragma config FOSC = HS        // Oscillator Selection bits (High Speed crystal)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#include <xc.h>

#define _XTAL_FREQ 4000000      // FOSC = 4 Mhz @delay

char BUF[17] = "                ", RC = 0;
volatile unsigned int T1ZOH = 0;

char PWM @ &CCPR2L;
bit  BUZ @ (((unsigned) &PORTA)*8) + 5;

// UX

void beep (char dash) { // Morse code
    if (!dash) for (int j = 0; j < 100; ++j) {
        BUZ = 1; __delay_us(1000);
        BUZ = 0; __delay_us(500);
    }
    else for (int j = 0; j < 500; ++j) {
        BUZ = 1; __delay_us(1000);
        BUZ = 0; __delay_us(500);
    }
}

void rsend (char msg) {
    while (!TXIF) continue;
    TXREG = msg;
}

char rsget () {
    while (!RCIF) continue;
    return RCREG;
}

// INTERFACE

void read_tmr1 () {
    T1ZOH = TMR1;
    rsend(T1ZOH >> 8);   //BKWD: Big Little Endian
    rsend(T1ZOH);
    TMR1 -= T1ZOH;
}

void write (signed char duty) {
    if (duty > 100) PWM = 100;
    else if (duty < 0) PWM = 0;
    else PWM = duty;
}

unsigned char PWMZOH = 0;
unsigned char kT0 = 0;
unsigned char T0PS = 0;
void interrupt sampling () {
    kT0++;
    if (kT0 > T0PS) {
        kT0 = 0;
        T1ZOH = TMR1;
        PWM = PWMZOH;
        rsend(T1ZOH);
        rsend(T1ZOH >> 8);
        TMR1 -= T1ZOH;
    }
    TMR0IF = 0;
}

int main (void) {
    OPTION_REG = 0x83;      // Disable PORTB pull-ups, TMR0 internal clock 1/16
    INTCON = 0x80;          // Enable global interrupts
    T1CON = 0x03;           // TMR1 external clock, TMR1/1
    CCP1CON = 0x0F;         // CCP1 module is on PWM mode
    CCPR1L = 0;             // PWM duty = (CCPR1L:CCP1CON<5:4>)*TOSC*T2CKPS
    CCP2CON = 0x0F;         // CCP2 module is on PWM mode
    CCPR2L = 0;             // PWM duty = (CCPR2L:CCP2CON<5:4>)*TOSC*T2CKPS
    PR2 = 100;              // PWM period = [(PR2)+1]*4*TOSC*T2CKPS
    T2CON = 0x04;           // TMR2 enabled at maximum frequency
    ADCON1 = 0x05;          // PORTA A0, A1, A3 are analog inputs
    TRISC = 0xFD;           // C0 clock TMR1, C1 duty PWM, C6/C7 = TX/RX
    TRISB = 0xFF;           // B1 will be read at startup to decide protocol
    TRISA = 0xDF;           // A5 buzz a buzzer
    RCSTA = 0x90;           // asynchronous 8 bit RS-232
    TXSTA = 0x24;           // asynchronous 8 bit RS-232
    SPBRG = 12;             // BAUD generator = FOSC / (16*BAUD) - 1;
    
    // Two modes of operation. One breaks backwards compatibility.
    // To enter the usual mode, press only SW2 or RESET
    if (PORTB & 0x02) {
        beep(0); __delay_ms(100);
        beep(0); __delay_ms(100);
        beep(0); __delay_ms(100);
        while (1) {
            // buffer_serial
            char x = rsget();
            if (x == '\n') {    // BKWD: won't write PWM = 10
                switch (BUF[0]) {
                    case '7': read_tmr1(); break;
                    case '5': write(BUF[1]); break;
                    case '1': write(BUF[1]); read_tmr1(); break;
                    case '2': write(0); beep(0); TMR1 = 0; break; // BKWD: can't really stop
                }
                RC = 0;
            }
            else {
                BUF[RC] = x;
                RC++;
            }
        }
    }
    // To enter the new mode: hold S2 down and press SW2 or RESET
    else {
        beep(0); __delay_ms(100);
        beep(1); __delay_ms(100);
        while (1) {
            // ping_pong
            char cmd = rsget();
            char msg = rsget();
            if (cmd == 'x') {
                PWMZOH = msg;
            }
            else if (cmd == 'r') {
                INTCON &= 0xDF;  
                T1ZOH = TMR1;
                write(msg);
                rsend(T1ZOH); 
                rsend(T1ZOH >> 8);
                TMR1 -= T1ZOH;
            }
            else if (cmd == 's') {
                PWM = 0;         // stop PWM now
                PWMZOH = 0;      // stop PWM later
                T1CON &= 0xFE;   // stop TMR1
                INTCON &= 0xDF;  // stop T0IF interrupts
                T1ZOH = 0;       // clear TMR1 first-hold
                beep(0);
            }
            else if (cmd == 't') {
                T0PS = msg;
                kT0 = T0PS;
                TMR1 = 0;
                T1CON |= 0x01;         // start TMR1
                TMR0 = 255;            // takes two cycles to count again
                asm("BSF INTCON, 5");  // enable interrupt on TOIF
            }
        }
    }
    return 0;
}