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

#include <conio.h>
#include <stdio.h>
#include <xc.h>

const unsigned long IFOSC = 1000000;  // CLK = 4 Mhz, CLK/4 = 1 Mhz @delay
const unsigned int T1FOSC = 0x1000;   // RTC = 0x8000 Hz, CKPS = 0b11 @T1CON
const unsigned int RPSMIN = 8;        // RPS minimal pickup value @rps_timer1
const unsigned long BAUD = 19200;     // BAUD rate @DeviceManager @RBPRG

char PWM @ &CCPR1L;
bit  BUZ @ (((unsigned) &PORTA)*8) + 5;
bit  PUT = 0;

struct {
    unsigned CH : 8;
    unsigned RS : 1;     //Register Select
    unsigned RW : 1;     //Read/Write
    unsigned EN : 1;     //ENable
} LCD @ &PORTD;

// UI

void beep () {
    _delay(IFOSC/10);
    for (int j = 0; j < 100; ++j) {
        BUZ = 1; _delay(IFOSC/1000);
        BUZ = 0; _delay(IFOSC/2000);
    } 
}

void putch (char msg) {    
    if (PUT) {
        LCD.RS = 1;
        LCD.CH = msg;
        LCD.EN = 1; _delay(IFOSC/1000000);
        LCD.EN = 0; _delay(IFOSC/20000);
    }
    else {
        while(!TXIF) continue;
        TXREG = msg;
    }
    return;
}

char getch () {
    while(!RCIF) continue;
    return RCREG;
}

void prog_lcd (char msg) {
    LCD.RS = 0;
    LCD.CH = msg;
    LCD.EN = 1; _delay(IFOSC/1000000);
    LCD.EN = 0; _delay(IFOSC/20000);
    return;
}

void init_lcd () {
    PUT = 1;
    LCD.RW = 0;
    prog_lcd(0x30); _delay(IFOSC/200);
    prog_lcd(0x30); _delay(IFOSC/10000);
    prog_lcd(0x30);
    prog_lcd(0x38);
    prog_lcd(0x01); _delay(IFOSC/500);
    prog_lcd(0x0C);
    prog_lcd(0x06);
    prog_lcd(0x80);
    printf("LAB.CONT.DIGITAL");
    prog_lcd(0xC0);
    printf("Seja Bem-Vindo!");
    return;
}

void main_lcd (unsigned int value, char duty) {
    PUT = 1;
    prog_lcd(0xC0);
    printf("bps: %3u", value);
    prog_lcd(0xC8);
    printf("pwm: %3d%%", duty);
}

// MEASUREMENT

typedef struct {
    volatile unsigned int count;
    unsigned int time;
    unsigned int value;
    unsigned int div;
    unsigned int max;
} rpm;

rpm FAN = {-1, 0, 0, 7, 5000};

void interrupt raycast_intersection_cleared (void) {
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

void read_tmr1 () {
    PUT = 0;
    FAN = rps_timer1(FAN, TMR1);
    printf("%u", FAN.value);
    return;
}

void read_tmr0 () {
    PUT = 0;
    printf("%u", TMR0);
    TMR0 = 0;
    return;
}

void write (char duty) {
    PWM = duty;
}

void io_switch (char *x) {
	switch (x[0]) {
		case '7': read_tmr0();               break;
        case '5': write(x[1]);				 break;
		case '1': read_tmr0(); write(x[1]);  break;
        case 'r': read_tmr1();               break;
        case 'w': write(x[1]);				 break;
        case 'x': read_tmr1(); write(x[1]);  break;
        case 's':
        case '2': write(0);	beep(); beep();  break;
	}
}

int buffer_serial (char *buffer, int idx) {
    buffer[idx] = getch();
    if (buffer[idx] == '\n') {
        idx=0;
        io_switch(buffer);
    }
    else {
        idx++;
    }
    return idx;
}

int main (void) {
    //SETUP    
    
    T1CON = 0xFB;           // TMR1 external clock, sync, TMR1/8    
    CCP1CON = 0x0F;         // CCP1 module is on PWM mode
    CCPR1L = 0;             // PWM duty = (CCPR1L:CCP1CON<5:4>)*TOSC*T2CKPS
    PR2 = 100;              // PWM period = [(PR2)+1]*4*TOSC*T2CKPS
    T2CON = 0x04;           // TMR2 enabled at maximum frequency
    TRISC = 0xFB;           // C2 powers a LED
    OPTION_REG = 0x38;      // B pull-ups, B0 falling, T0CKI falling, TMR0/1
    TRISB = 0x7F;           // B0 reads IR, B7 powers a LED
    INTCON = 0x90;          // enable external interrupt on B0
    TRISD = 0x00;           // LCD output
    TRISE = 0xF8;           // LCD register select, read/write, enable
    TRISA = 0xCF;           // A5 buzz a buzzer, A4 counts rotations
    RCSTA = 0x90;           // asynchronous 8 bit RS-232
    TXSTA = 0x24;           // asynchronous 8 bit RS-232
    SPBRG = 12;             // BAUD generator = IFOSC / (4*BAUD) - 1;
    
    init_lcd();
    beep();
    beep();
    beep();
    
    //LOOP     
    
    unsigned int logged;
    char buffer[6];
    int idx = 0;
    while (1) {
        idx = buffer_serial(buffer, idx);
        if (TMR1 < T1FOSC) logged = 0;
        else if (!logged) {
            main_lcd(TMR0, PWM);
            logged = 1;
        }
    }
    return 0;
}