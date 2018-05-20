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
#define OUT_TEMP_L 0x20

// Helper Function Prototypes
void main(void) {
    
     LCDinit();
     initPololu();
    
    clearScreen();
    //_CP0_SET_COUNT(0);
    //while(_CP0_GET_COUNT()<48000000){;}
    
    char string[200];
    sprintf(string, "Loading Success");
    writeString(string,20,20,WHITE);
    //_CP0_SET_COUNT(0);
    //while(_CP0_GET_COUNT()<48000000){;}
    
    clearScreen();
    
    char string4[200];
    sprintf(string4, "Talking to Pololu");
    writeString(string4,20,20,WHITE);
    //_CP0_SET_COUNT(0);
    //while(_CP0_GET_COUNT()<48000000){;}
    
    unsigned char data[200];
    
    short temp;
    short gyro_X;
    short gyro_Y;
    short gyro_Z;
    short accel_X;
    short accel_Y;
    short accel_Z;
    
    short data2[7];
    char string5[10];
    
    int y = 0;
    
    
    while(y<100){
        
        _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<2400000){;}              // Wait 12000*200 = 1 ms*200 = .2 s = 5 hz
    
    getMultipleMessages(OUT_TEMP_L,data,14);
        
    temp = (data[0]<<4)|data[1];
    data2[0] = temp;
    gyro_X = (data[2]<<4)|data[3];
    data2[1] = gyro_X;
    gyro_Y = (data[4]<<4)|data[5];
    data2[2] = gyro_Y;
    gyro_Z = (data[6]<<4)|data[7];
    data2[3] = gyro_Z;
    accel_X = (data[8]<<4)|data[9];
    data2[4] = accel_X;
    accel_Y = (data[10]<<4)|data[11];
    data2[5] = accel_Y;
    accel_Z = (data[12]<<4)|data[13];
    data2[6] = accel_Z;
    int xxx = 0; 
    
    y = y + 1;
    clearScreen();
    
    while(xxx<7){
        sprintf(string5,"%hu",data2[xxx]);
        
        writeString(string5,10,10 + (xxx*10),WHITE);
        
        xxx++;
    }
    
    }
    
    
    
}
    
    
    
    
    
    
    //unsigned char r;
    
    //r = getMessage(WHO_AM_I);
    
    //clearScreen();
    
   //if(r==0x69) {
    
    //char string2[200];
    //sprintf(string2, "Success");
   // writeString(string2,20,20,WHITE);
   // _CP0_SET_COUNT(0);
   // while(_CP0_GET_COUNT()<48000000){;}
    
  //  } else {
    
    //char string1[200];
    //sprintf(string1, "Keep Trying");
    //writeString(string1,20,20,WHITE);
    //_CP0_SET_COUNT(0);
    //while(_CP0_GET_COUNT()<48000000){;}
    
    //}
    
    

    

// Helper Functions



