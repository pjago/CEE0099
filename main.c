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

#include <stdio.h>
#include <xc.h>

const unsigned long IFOSC = 1000000;  // CLK = 4 Mhz, CYC = 1 Mhz @beep @delay
const unsigned int T1FOSC = 0x1000;   // RTC = 0x8000 Hz, CKPS = 0b11 @T1CON
const unsigned int RPSMIN = 8;        // RPS minimal pickup value @rps_timer

struct {
    unsigned RS : 1;     //Register Select
    unsigned EN : 1;     //ENable
} LCD @ &PORTE;

void beep () {
    _delay(IFOSC/10);
    for (int j = 0; j < 100; ++j) {
        RA5 = 1; _delay(IFOSC/1000);
        RA5 = 0; _delay(IFOSC/2000);
    } 
}

void putch (char msg) {
    LCD.RS = 1;
    PORTD = msg;
    LCD.EN = 1; _delay(IFOSC/1000000);
    LCD.EN = 0; _delay(IFOSC/20000);
    return;
}

void prog_lcd (char msg) {
    LCD.RS = 0;
    PORTD = msg;
    LCD.EN = 1; _delay(IFOSC/1000000);
    LCD.EN = 0; _delay(IFOSC/20000);
    return;
}

void init_lcd () {
    prog_lcd(0x30); _delay(IFOSC/200);
    prog_lcd(0x30); _delay(IFOSC/10000);
    prog_lcd(0x30);
    prog_lcd(0x38);
    prog_lcd(0x01); _delay(IFOSC/500);
    prog_lcd(0x0C);
    prog_lcd(0x06);
    return;
}

void main_lcd (long value, int duty) {
    prog_lcd(0xC0);
    printf("%4d ", value);
    prog_lcd(0xC8);
    printf("PWM %3d%%", duty);
    return;
}

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
    TRISD = 0x00;           // LCD output
    TRISE = 0xFC;           // LCD enable e rs
    TRISA = 0xDF;           // A5 buzz a buzzer
    
    beep();
    beep();
    beep();
    init_lcd();
    prog_lcd(0x80);
    printf("LAB.CONT.DIGITAL");
    prog_lcd(0xC0);
    printf("Seja Bem-Vindo!");
    
    //LOOP   
    
    while (1) {
        CCPR1L = 100 * FAN.value / FAN.max;
        FAN = rps_timer1(FAN, TMR1);
        main_lcd(FAN.value, CCPR1L); //todo: only per sec
    }
    return 0;
}