/* 
 * File:   main6.c
 * Author: Mike Flynn
 * Created on May 13, 2018, 9:29 PM
 * 
 * The purpose of this program is to create an algorithm that allows the 
 *  user to draw any ASCII character at a specified location, as well as 
 *  to print out "Hello World" to the TNT LCD using the PIC32MX250F128B
 *  and the ILI9163C controller on the LCD with SPI to do so.
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
#include "ILI9163C.h"
#include <math.h>

// Helper Function Prototypes

void putChar(const char *letter, unsigned short x, unsigned short y, unsigned short color);
void loopString(const char *string, unsigned short x, unsigned short y, unsigned short color);
void makeProgressBar(int bar_length, unsigned short x, unsigned y,int status);

// Main Function

void main(void) {

    SPI1_init();
    LCD_init();
    
    TRISAbits.TRISA4 = 0;           // A7 is a digital output
    LATAbits.LATA4 = 1;             // A7 is on initially
    
    LCD_clearScreen(BLACK);
   
    char string[200];
    sprintf(string, "How are ya bob");
    loopString(string, 10, 10, WHITE);
  
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<48000000){;}
    
    LCD_clearScreen(BLACK);
    
    int gg = 0;
    char string1[200];
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<4800000){;}
    
    
    
       
    sprintf(string1, "Hello World!");
    
    loopString(string1, 28, 32, WHITE);
    
    
    
    //makeProgressBar(20,28,62,gg);
    
    int xx = 0; 
    const char bar[5] = {0xFF, 0x00, 0x00, 0x00, 0x00};
    const char bar2[5] = {0x81, 0x00, 0x00, 0x00, 0x00};
    const char black[5]= {0xFF,0xFF, 0xFF,0xFF, 0xFF};
    
    putChar(bar,15,62,WHITE);
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<4800000){;}
    sprintf(string1, "     Percent");
    loopString(string1, 28, 90, WHITE);
    
    while(1){
    
    while(xx<100){
        
        loopString(black,28, 90, BLACK);
        //sprintf(string1, "%d",xx);
        //loopString(string1, 28, 90, WHITE);
        
        putChar(bar,15+xx,62,WHITE);
        xx++;
        _CP0_SET_COUNT(0);
        while(_CP0_GET_COUNT()<240000){;}
        
    }
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<4800000){;}
    
    putChar(bar,15+xx,62,WHITE);
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<4800000){;}
    
    sprintf(string1, "100  Percent");
    loopString(string1, 28, 90, WHITE);
    
    }
    
    
    //}
    
    
}

// Helper Functions

void putChar(const char *letter, unsigned short x, unsigned short y, unsigned short color){
    
    int ii = 0;
    int jj = 0;
    
    char bits[8] = {0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};
    int num_iis = *(&letter + 1) - letter;
    
 
        while(ii<5){  
            while(jj<8){
                        
                if(bits[7-jj]&&(bits[7-jj]&(letter[ii]))){
                    if((x+ii) < 130 && (x-ii) > 0 && (y+jj) < 131 && (y-jj) > 0){
                        LCD_drawPixel(x+ii,y+jj,color);
                    }            
                }            
 
                jj++;
            
            }
            jj = 0;
            ii++;
            
        }
    
    
    
    
}

void loopString(const char *str, unsigned short x, unsigned short y, unsigned short color){
    
    int ii = 0; 
    int letter = 0; 
    int nextline = 0;
    
    int x1 = x;
    int y1 = y;
    
    while(str[ii]){
        
        letter = (int) str[ii];
        letter = letter - 32;
        if(x+(6*ii)>125){ nextline = nextline + 1; x1 = 5; }
        putChar(ASCII[letter],x1+(6*ii),y1+(9*nextline), color);
        ii++;    
        
    }
    
}

void makeProgressBar(int bar_length, unsigned short x, unsigned y,int status){
    
    int xx = 0; 

    putChar(0xFF,x,y,WHITE);
    while(xx<bar_length){  
        putChar(0x81,x+xx,y,WHITE);
        
    }
    
    putChar(0xFF,x+xx,y,WHITE);
}