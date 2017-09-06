/*
 * File:   main.c
 * Author: pjago, pedro.m4rtins@gmail.com
 * 
 * Created on September 4, 2017, 10:11 AM
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

const unsigned int T1FOSC = 0x1000;  // RTC = 0x8000 Hz, CKPS = 0b11 @T1CON
const unsigned int RPSMIN = 8;       // RPS minimal pickup value @rps_timer

typedef struct {
    volatile int count;
    unsigned int time;
    unsigned int value;
    unsigned int div;
    unsigned int max;
} rpm;

rpm FAN = {-1, 0, 0, 7, 5000};

void interrupt raycast_intersection_cleared (void) {
    RB7 = !RB7;
    FAN.count++;
    INTF = 0;
    return;
}

rpm rps_timer1 (rpm x, unsigned int t) {
    if (x.time > t) {
        x.count = 0;
        x.time = t;
    }
    else if (x.count > 0) {
        x.value = T1FOSC / (t - x.time) * x.count / x.div;
        if (x.value > x.max) x.max = x.value;
        x.count = 0;
        x.time = t;
    }
    else if ( (t - x.time) > (T1FOSC / RPSMIN) ) {
        x.count = -1;
        x.value = 0;
        x.time = t;
    }
    return x;
}

int main (void) {
    //SETUP

    TMR1CS = 1;             // TMR1 uses external clock of 2^15 Hz
    T1CKPS0 = 1;            // using a pre-scaler of 8 ...
    T1CKPS1 = 1;            // TMR1 increments at 2^12 kHz
    TMR1ON = 1;             // TMR1 enabled
    
    CCP1M3 = 1;             // CCP1 module is on PWM mode ...
    CCP1M2 = 1;             // CCP1 is connected to pin C2
    CCP1CON &= 0x0F;        // CCP1CON<5:4> is the LSB for a reason
    CCPR1L = 0;             // PWM duty = (CCPR1L:CCP1CON<5:4>)*TOSC*T2CKPS
    PR2 = 100;              // PWM period = [(PR2)+1]*4*TOSC*T2CKPS
    T2CKPS0 = 0;            // T2CKPS is set to 1 (00) ...
    T2CKPS1 = 0;            // increasing the PWM frequency
    TMR2ON = 1;             // TMR2 needs to be on for PWM to work
    TRISC2 = 0;             // C2 powers a LED
    
    nRBPU = 0;              // B internal pull-ups enabled
    TRISB0 = 1;             // B0 reads the IR output
    TRISB7 = 0;             // B7 powers a LED
    INTEDG = 0;             // B0 falling edge trigger the interrupt
    INTE = 1;               // enable the external interrupt
    GIE = 1;                // global interrupt enable

    //LOOP   

    while (1) {
        CCPR1L = 100 * FAN.value / FAN.max;
        FAN = rps_timer1(FAN, TMR1);
    }
    return 0;
}