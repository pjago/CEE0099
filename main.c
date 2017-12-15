/*
 * File:   main.c
 * Author: pjago, pedro.m4rtins@gmail.com
 * 
 * Created on September 4, 2017, 10:11 AM
 */

// CONFIG

// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator (HS))
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
#pragma config CCP2MX = OFF     // CCP2 MUX bit (CCP2 input/output is multiplexed with RB3)
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

#define _XTAL_FREQ 4000000          // FOSC = 4 Mhz, PLL = 1. @delay
const unsigned long FCY = 1000000;  // FOSC/4 = 1 Mhz. @SPBRG
const unsigned long T1FCY = 125000; // FCY/8, T1CKPS = 0b11. @T1CON @rsget

volatile enum {TMR0OF, NIL} ERR = NIL;
char BUF[17] = "                ", RC = 0;
unsigned char T0ZOH = 0;
char ALIVE = 0;

struct {
    unsigned CH : 4;
    unsigned EN : 1;     //ENable
    unsigned RS : 1;     //Register Select
} LCD @ &LATB = {0, 0, 0};
signed char PWM @ &CCPR2L;
bit BUZ @ (((unsigned) &LATB)*8) + 6;

// UX

void beep () {
    for (int j = 0; j < 100; ++j) {
        BUZ = 1; __delay_us(1000);
        BUZ = 0; __delay_us(500);
    }
}

void putch (char msg) {
    CCP2CON = 0x00; // @CCP2MX B3 as OUT
    LCD.RS = 1;
    LCD.CH = (msg >> 4);
    LCD.EN = 1; __delay_us(1);
    LCD.EN = 0; __delay_us(50);
    LCD.CH = msg;
    LCD.EN = 1; __delay_us(1);
    LCD.EN = 0; __delay_us(50);
    CCP2CON = 0x0F; // @CCP2MX B3 as PWM
    return;
}

void prog_lcd (char msg) {
    CCP2CON = 0x00; // @CCP2MX B3 as OUT
    LCD.RS = 0;
    LCD.CH = (msg >> 4);
    LCD.EN = 1; __delay_us(1);
    LCD.EN = 0; __delay_us(50);
    LCD.CH = msg;
    LCD.EN = 1; __delay_us(1);
    LCD.EN = 0; __delay_us(50);
    CCP2CON = 0x0F; // @CCP2MX B3 as PWM
    return;
}

void init_lcd () {
    __delay_ms(20);
    prog_lcd(0x02); __delay_ms(5);
    prog_lcd(0x28);
    prog_lcd(0x01); __delay_ms(2);
    prog_lcd(0x0C);
    prog_lcd(0x06);
    prog_lcd(0x80);
    printf("LAB.CONT.DIGITAL");
    prog_lcd(0xC0);
    printf("Seja Bem-Vindo!");
    return;
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

void read_tmr0 () {
    T0ZOH = TMR0;
    rsend(0);    //to mantain backwards compatibility @fmarcolino
    rsend(T0ZOH);
    TMR0 -= T0ZOH;
    return;
}

void write (signed char duty) {
    if (duty > 100) PWM = 100;
    else if (duty < 0) PWM = 0;
    else PWM = duty;
}

// MAIN CODE

void low_priority interrupt tmr_overflow () {
    if (ALIVE) {
        BUZ = 1;
        ERR = TMR0OF;
    }
    TMR0IF = 0;
    return;
}

int main (void) {
    T1CON = 0xF1;           // TMR1 internal clock, sync, TMR1/8
    CCP2CON = 0x0F;         // CCP2 module is on PWM mode (B3 also on LCD!!!)
    CCPR2L = 0;             // PWM duty = (CCPR2L:CCP2CON<5:4>)*TOSC*T2CKPS
    PR2 = 100;              // PWM period = [(PR2)+1]*4*TOSC*T2CKPS
    T2CON = 0x04;           // TMR2 enabled at maximum frequency
    T0CON = 0xFF;           // T0CKI falling, TMR0/1
    ADCON0 = 0x07;          // Select analog channel AN1
    ADCON1 = 0x0D;          // Use A0 and A1 as analog pins
    ADCON2 = 0x00;          // Left justified, maximum frequency AD
    TRISA = 0xFF;           // A4 counts rotations, A1, A2, and A3 are switches
    TRISB = 0x00;           // B0-5 controls LCD, B3 drives PWM, B7 a Buzzer
    TRISC = 0xC0;           // C7 and C6 must be initially set as input (USART)
    RCSTA = 0x90;           // asynchronous 8 bit RS-232
    TXSTA = 0x24;           // asynchronous 8 bit RS-232
    SPBRG = 12;             // BAUD generator = FOSC / (16*BAUD) - 1;
    INTCON2 = 0x00;         // Set TMR0 interrupt to low priority
    IPR1 = 0x30;            // Set RX/TX interrupts to high priority
    init_lcd(); __delay_ms(1500);
    beep(); __delay_ms(100);
    beep(); __delay_ms(100);
    beep(); __delay_ms(100);
    TMR0IF = 0;
    TMR1IF = 0;
    TMR0 = 0;
    TMR1 = 0;
    INTCON = 0xC0;          // Enable TMR0 interrupts
    while (1) {
        // main_lcd
        if (TMR1IF) {
            prog_lcd(0x80);
            if (ERR == NIL) printf("%s", BUF);
            else if (ERR == TMR0OF) printf("TMR0 OVERFLOW =P");
            prog_lcd(0xC0);
            printf("tmr %3u ", T0ZOH);
            prog_lcd(0xC8);
            printf("pwm %3d%%", PWM);
            TMR1IF = 0;
        }
        // buffer_serial
        char x = rsget();
        if (x == '\n') {    //bug when we write pwm = 10 @fmarcolino
            ALIVE = 1;
            switch (BUF[0]) {
                case 'r': case '7': read_tmr0(); break;
                case 'w': case '5': write(BUF[1]); break;
                case 'x': case '1': write(BUF[1]); read_tmr0(); break;
                case 's': case '2': ALIVE = 0; write(0); beep(); beep(); break;
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