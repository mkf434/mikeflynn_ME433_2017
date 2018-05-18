/* 
 * File:   
 * Author:
 * Created on May 15, 2018, 5:09 PM
 * 
 *  The purpose of this program 
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

// Header Files

#include <xc.h>                         // Pretty standard header files
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "ILI9163C.h"
#include "I2C.h"
#include "LCD.h"

#define WHO_AM_I 0x0F

// Helper Function Prototypes
void main(void) {
    
     LCDinit();
     initPololu();
    
    clearScreen();
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<48000000){;}
    
    char string[200];
    sprintf(string, "Loading");
    writeString(string,20,20,WHITE);
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<48000000){;}
    
    clearScreen();
    
    char string4[200];
    sprintf(string4, "Talking to Pololu");
    writeString(string4,20,20,WHITE);
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<48000000){;}
    
    
    
    unsigned char r;
    
    r = getMessage(WHO_AM_I);
    
    clearScreen();
    
    if(r==0x69) {
    
    char string2[200];
    sprintf(string2, "Success");
    writeString(string2,20,20,WHITE);
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<48000000){;}
    
    } else {
    
    char string1[200];
    sprintf(string1, "Keep Trying");
    writeString(string1,20,20,WHITE);
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<48000000){;}
    
    }
    
    

    
}

// Helper Functions



