/*
 * File:   main.c
 * Author: pjago, pedro.m4rtins@gmail.com
 * 
 * Created on September 4, 2017, 10:11 AM
 */

#include <xc.h>

typedef struct {
    unsigned int time;
    int value;
    int count;
    int div;
    int max;
} rpm;

rpm FAN = {0, 0, -1, 7, 5000};

void interrupt raycast_intersection_cleared (void) {
    RB7 = !RB7;
    FAN.count++;
    INTF = 0;
    return;
}

rpm rpm_timer (rpm in, unsigned int t) {
    rpm out; 
    
    if (out.time > t) {
        out.count = 0;
        out.value = in.value;
        out.time = t;
    }
    else if (in.count > 0) {
        out.count = 0;
        out.value = 37500 / (t - in.time) * 1000 * in.count / in.div;
        if (out.value > in.max) out.max = out.value;
        out.time = t;
    }
    else if (t - in.time > 37500) {
        out.count = -1;
        out.value = 0;
        out.time = t;
    }
    else {
        out = in;
    }
    return out;
}

int main (void) {
    //setup
    TMR1CS = 0;             // TMR1 uses internal clock of 5 MHz
    T1CKPS0 = 1;            // using a pre-scaler of 8 ...
    T1CKPS1 = 1;            // TMR1 increments at 625 kHz
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

    //loop
    while (1) {
        FAN = rpm_timer(FAN, TMR1);
        CCPR1L = 10 * FAN.value / FAN.max * 10; //TODO: MATLAB COM
    }
    return 0;
}