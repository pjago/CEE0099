/*
 * File:   main.c
 * Author: pjago, pedro.m4rtins@gmail.com
 * 
 * Created on September 4, 2017, 10:11 AM
 */

// CONFIG

// CONFIG1L
#pragma config PLLDIV = 5       // PLL Prescaler Selection bits (Divide by 5 (20 MHz oscillator input))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = HSPLL_HS  // Oscillator Selection bits (HS oscillator, PLL enabled (HSPLL))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

#include <conio.h>
#include <stdio.h>
#include <xc.h>

#define _XTAL_FREQ 20000000 // CLK = 20 Mhz

const unsigned long IFOSC = 5000000;  // CLK = 20 Mhz, CLK/4 = 5 Mhz @delay
const unsigned int T1FOSC = 360;      // T1CLK = B*T1FOSC Hz, CKPS = 0b00 @T1CON
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
    __delay_ms(100);
    for (int j = 0; j < 100; ++j) {
        BUZ = 1; __delay_us(1000);
        BUZ = 0; __delay_us(500);
    } 
}

void putch (char msg) {    
    if (PUT) {
        LCD.RS = 1;
        LCD.CH = msg;
        LCD.EN = 1; __delay_us(1);
        LCD.EN = 0; __delay_us(50);
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
    LCD.EN = 1; __delay_us(1);
    LCD.EN = 0; __delay_us(50);
    return;
}

void init_lcd () {
    PUT = 1;
    LCD.RW = 0;
    prog_lcd(0x30); __delay_ms(5);
    prog_lcd(0x30); __delay_us(100);
    prog_lcd(0x30);
    prog_lcd(0x38);
    prog_lcd(0x01); __delay_ms(2);
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
    printf("rps: %3u", value);
    prog_lcd(0xC8);
    printf("pwm: %3d%%", duty);
}

// MEASUREMENT

typedef struct {
    volatile unsigned int count;
    unsigned int value;
} rpm;

rpm FAN = {-1, 0};

void high_priority interrupt raycast_intersection_cleared (void) {
    FAN.count++;
    INT0IF = 0;
    return;
}

rpm rps_timer1 (rpm x, unsigned int dt) {
    if (x.count > 0) {
        x.value = T1FOSC * x.count / dt;
        x.count = 0;
    }
    else {
        x.count = -1;
        x.value = 0;
    }
    return x;
}

void read_tmr1 () {
    PUT = 0;
    FAN = rps_timer1(FAN, TMR1);
    printf("%u", FAN.value);
    TMR1 = 0;
    return;
}

void read_tmr0 () {
    PUT = 0;
    printf("%u", TMR0);
    TMR0 = 0;
    FAN = rps_timer1(FAN, TMR1);
    TMR1 = 0;
    return;
}

void write (char duty) {
    PWM = duty;
}

// PROTOCOL

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
    main_lcd(FAN.value, PWM);
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
    T1CON = 0x0B;           // TMR1 external clock, sync, TMR1/1    
    CCP1CON = 0x0F;         // CCP1 module is on PWM mode
    CCPR1L = 0;             // PWM duty = (CCPR1L:CCP1CON<5:4>)*TOSC*T2CKPS
    PR2 = 100;              // PWM period = [(PR2)+1]*4*TOSC*T2CKPS
    T2CON = 0x04;           // TMR2 enabled at maximum frequency
    TRISC = 0xFB;           // C2 powers a LED
    T0CON = 0xFF;           // T0CKI falling, TMR0/1
    INTCON = 0x90;          // GIEH enabled, INT0IE enabled
    INTCON2 = 0x00;         // B pull-ups, B0 falling
    TRISB = 0x7F;           // B0 reads IR, B7 powers a LED
    INTCON = 0x90;          // enable external interrupt on B0
    TRISD = 0x00;           // LCD output
    TRISE = 0xF8;           // LCD register select, read/write, enable
    TRISA = 0xCF;           // A5 buzz a buzzer, A4 counts rotations
    RCSTA = 0x90;           // asynchronous 8 bit RS-232
    TXSTA = 0x24;           // asynchronous 8 bit RS-232
    SPBRG = 64;             // BAUD generator = IFOSC / (4*BAUD) - 1;    
    init_lcd();
    beep();
    beep();
    beep();    
    char buffer[6];
    int idx = 0;
    while (1) idx = buffer_serial(buffer, idx);
    return 0;
}