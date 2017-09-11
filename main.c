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
    volatile unsigned int count;
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
    
    T1CON = 0xFB;           // TMR1 external clock, sync, prescale 8    
    CCP1CON = 0x0F;         // CCP1 module is on PWM mode
    CCPR1L = 0;             // PWM duty = (CCPR1L:CCP1CON<5:4>)*TOSC*T2CKPS
    PR2 = 100;              // PWM period = [(PR2)+1]*4*TOSC*T2CKPS
    T2CON = 0x04;           // TMR2 enabled at maximum frequency
    TRISC = 0xFB;           // C2 powers a LED
    OPTION_REG = 0x00;      // B internal pull-ups, B0 INTE falling edge
    TRISB = 0x7F;           // B0 reads IR, B7 powers a LED
    INTCON = 0x90;          // enable external interrupt on B0

    //LOOP   
    
    while (1) {
        CCPR1L = 100 * FAN.value / FAN.max;
        FAN = rps_timer1(FAN, TMR1);
    }
    return 0;
}