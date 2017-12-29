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

#define _XTAL_FREQ 4000000          // FOSC = 4 Mhz @delay

char BUF[17] = "                ", RC = 0;
unsigned int T1ZOH = 0;

char PWM @ &CCPR2L;
bit  BUZ @ (((unsigned) &PORTA)*8) + 5;

// UX

void beep () { // todo: rename to bébé
    for (int j = 0; j < 100; ++j) {
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
    rsend(T1ZOH >> 8);   //BKWD: Big -> Little Endian
    rsend(T1ZOH);
    TMR1 -= T1ZOH;
    return;
}

void write (signed char duty) {
    if (duty > 100) PWM = 100;
    else if (duty < 0) PWM = 0;
    else PWM = duty;
}

int main (void) {
    T1CON = 0x03;           // TMR1 external clock, TMR1/1    
    CCP1CON = 0x0F;         // CCP1 module is on PWM mode
    CCPR1L = 0;             // PWM duty = (CCPR1L:CCP1CON<5:4>)*TOSC*T2CKPS
    CCP2CON = 0x0F;         // CCP2 module is on PWM mode
    CCPR2L = 0;             // PWM duty = (CCPR2L:CCP2CON<5:4>)*TOSC*T2CKPS
    PR2 = 100;              // PWM period = [(PR2)+1]*4*TOSC*T2CKPS
    T2CON = 0x04;           // TMR2 enabled at maximum frequency
    ADCON1 = 0x07;          // PORTA is all digital
    TRISC = 0xFD;           // C0 clock TMR1, C1 duty PWM, C6/C7 = TX/RX 
    TRISA = 0xDF;           // A5 buzz a buzzer
    RCSTA = 0x90;           // asynchronous 8 bit RS-232
    TXSTA = 0x24;           // asynchronous 8 bit RS-232
    SPBRG = 12;             // BAUD generator = FOSC / (16*BAUD) - 1;    
    beep(); __delay_ms(100);
    beep(); __delay_ms(100);
    beep(); __delay_ms(100);   
    while (1) {
        // buffer_serial
        char x = rsget();
        if (x == '\n') {    //BKWD: bug when we write pwm = 10
            switch (BUF[0]) {
                case 'r': case '7': read_tmr1(); break;
                case 'w': case '5': write(BUF[1]); break;
                case 'x': case '1': write(BUF[1]); read_tmr1(); break;
                case 's': case '2': write(0); beep(); beep(); break;
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