/* 
 * File:   main.c
 * Author: Mike Flynn
 * Created on May 3, 2018, 3:32 PM
 * 
 * The purpose of this program is to use the PIC32MX250F128B to output a 
 *  10Hz square wave onto an LED to give a dim lighting effect on the LED
 */


// DEVCFG3
// USERID = No Setting
#pragma config PMDL1WAY = OFF           // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow multiple reconfigurations)
#pragma config FUSBIDIO = ON            // USB USID Selection (Controlled by the USB Module)
#pragma config FVBUSONIO = ON           // USB VBUS ON Selection (Controlled by USB Module)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_24         // PLL Multiplier (24x Multiplier)
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider (2x Divider)
#pragma config UPLLEN = ON              // USB PLL Enable (Enabled)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (PLL Divide by 2)

// DEVCFG1
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = HS             // Primary Oscillator Configuration (HS osc mode)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>


// Helper Function Prototypes


int main(int argc, char** argv) {
    
    
    __builtin_disable_interrupts();
    
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);
    
    BMXCONbits.BMXWSDRM = 0x0;                              // 0 data RAM access wait states
    INTCONbits.MVEC = 0x1;                                  // enable multi vector interrupts
    DDPCONbits.JTAGEN = 0;                                  // disable JTAG to get pins back
    
    TRISAbits.TRISA4 = 0;                                   // A4 is a digital output
    LATAbits.LATA4 = 0;                                     // A4 is off initially

    __builtin_enable_interrupts();

    
    
    while(1) {
        
        while(!PORTBbits.RB4) {
            _CP0_SET_COUNT(0);                                  // Set count to 0                        
            while(_CP0_GET_COUNT() < 12000) {};                 // Wait .5 ms

            LATAbits.LATA4 = 1;                                 // A4 off

            _CP0_SET_COUNT(0); 
            while(_CP0_GET_COUNT() < 12000) {};

            LATAbits.LATA4 = 0;                                 // A4 on

        };
    };     

    return (EXIT_SUCCESS);
}

