/* 
 * File:   main.c
 * Author: mkf434
 *
 * Created on May 7, 2018, 1:39 PM
 */

// Preprocessor Commands


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

void setupSPI(void);

// Main Function

int main(void) {

    char buf[100] = {};         // Init Buffer
    
    while(1) {
        unsigned short master_message = 0;
        SPI1BUF = master_message;
    }
    
    
    return (EXIT_SUCCESS);
}

// Helper Functions

void setupSPI(void) {
    
    RPB14Rbits.RPB14R = 0b0011;     // SCK1 is B14
    RPB8Rbits.RPB8R = 0b0011;       // SDO1 is B8
    RPB7Rbits.RPB7R = 0b0011;       // SS1 is B7
    
    SPI1BUF;                        // Clear the Rx Buffer      
    SPI1BRG = 0x1;                  // Set Baud Rate to 12 MHz, (50MHz/2*12MHz)-1 >= 1
    SPI1STATbits.SPIROV = 0;        // Clear the overflow bit
    SPI1CONbits.MSSEN = 0;          // Disable automatic control of SS1
    SPI1CONbits.MODE32 = 0;         // Disable 
    SPI1CONbits.MODE16 = 1; 
    SPI1CONbits.ON = 1; 
    
    TRISBbits.TRISB7 = 0;           // B7 is a digital output
    LATBbits.LATB7 = 0;             // B7 is off initially
    
}

