/* 
 * File:   main.c
 * Author: Mike Flynn
 * Created on May 7, 2018, 1:39 PM
 * 
 * The purpose of the program is to use the PIC32MX250F128B along with 
 *  a MCP4902 DAC and an Nscope to output a 5Hz triangular wave and a 
 *  10 Hz sine wave from the two outputs of the MCP4902
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

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>

#define PI 3.14159265               // Pi constant

// Helper Function Prototypes

void setupSPI(void);            // Initializing everything needed to use SPI1
void writeSPI(int buf);         // Update the SPI buffer
int makeWaves(int x);           // Generating the SPI bytes to send to the mcp

// Main Function

int main(void) {
    
    setupSPI();
    int y = 0;                      // triangle wave update counter and input value
    int yconst = 0;                 // sine wave input value
    int y2 = 0;                     // sine wave update counter
    int x = 0x7000;                 // first byte as 7 sets channel a register
    int x2 = 0xF000;                // first byte as F sets channel b register
    int z = 0x000;                  // MCP4902 only uses 3/4 bytes, need the last byte to properly load the register
    
     
    while(1) {
        _CP0_SET_COUNT(0);                              //Updates are 1 ms long
        
        if(y2==100) { y2 = 0; } else { y2 = y2 + 1;}    //10Hz sine wave so only 100, 1 ms updates
        yconst = makeWave(y2);                          // Determine the SPI bytes to send
    
        if(y==200) { y = 0; } else { y = y + 1;}        // 5Hz triangle wave so only 200, 1 ms updates
        
        while(_CP0_GET_COUNT() < 24000){;}              // Wait to update value until the rest of the 1 ms has passed
        
        writeSPI((x)|(y<<4)|(z));                       // piece together message using bitwise shifting and or comparisons
        writeSPI((x2)|(yconst<<4)|(z));                 // same as above except this is for Vout b, previous line for Vouta
    } 
    
    return (EXIT_SUCCESS);
    
}

// Helper Functions

void setupSPI(void) {
    
    __builtin_disable_interrupts();

    RPB14Rbits.RPB14R = 0b0011;     // SCK1 is B14
    RPB8Rbits.RPB8R = 0b0011;       // SDO1 is B8
    //RPB7Rbits.RPB7R = 0b0011;     // SS1 is B7
    
    SPI1BUF;                        // Clear the Rx Buffer      
    SPI1BRG = 0x1;                  // Set Baud Rate to 12 MHz, (50MHz/2*12MHz)-1 >= 1
    SPI1STATbits.SPIROV = 0;        // Clear the overflow bit
    SPI1CONbits.SSEN = 0;           // disable automatic control of SS1
    SPI1CONbits.MSTEN = 1;          // SPI1 is a master
    SPI1CONbits.MODE32 = 0;         // Disable 32 bit mode
    SPI1CONbits.MODE16 = 1;         // Enable 16 bit mode
    SPI1CONbits.ON = 1;             // SPI is on
    
    TRISBbits.TRISB7 = 0;           // B7 is a digital output
    LATBbits.LATB7 = 1;             // B7 is on initially
    
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);
    
    BMXCONbits.BMXWSDRM = 0x0;                              // 0 data RAM access wait states
    INTCONbits.MVEC = 0x1;                                  // enable multi vector interrupts
    DDPCONbits.JTAGEN = 0;                                  // disable JTAG to get pins back
    
    TRISAbits.TRISA4 = 0;                                   // A4 is a digital output
    LATAbits.LATA4 = 1;                                     // A4 is on initially

    __builtin_enable_interrupts();
    
}

void writeSPI(int buf) {                    // Update the SPI buffer        
    
    while(SPI1STATbits.SPIBUSY) {;}         // Wait until you can access the buffer
    
    LATBbits.LATB7 = 0;                     // Slave/Chip select low to start SPI protocol
    SPI1BUF = buf;                          // Write updated command to buffer
    while(SPI1STATbits.SPIBUSY) {;}         // wait until the buffer is done writing
    
    LATBbits.LATB7 = 1;                     // Slave/Chip select high to latch registers and execute update
    
}

int makeWave(int x) {                       // Generating the SPI bytes to send to the mcp
    
    int sinVALS[51];                        // Int array of sign values
    
    sinVALS[0] = 100;                       // Used calculator to calculate these values 
    sinVALS[1] = 106;                       // for f(x) = ((1.65sin(x)+1.65)/4.2)*255
    sinVALS[2] = 113;                       // where Vmax is 4.2 due to MCP4902 leeching
    sinVALS[3] = 119;                       // operational power from Vref since Vdd is NC
    sinVALS[4] = 125;                       // and times 255 to get into terms of MCP step sizes
    sinVALS[5] = 131;                       // with delta x = 2*pi/100
    sinVALS[6] = 137;
    sinVALS[7] = 143; 
    sinVALS[8] = 148; 
    sinVALS[9] = 154; 
    sinVALS[10] = 159; 
    sinVALS[11] = 164;
    sinVALS[12] = 169; 
    sinVALS[13] = 173; 
    sinVALS[14] = 177; 
    sinVALS[15] = 181; 
    sinVALS[16] = 185;
    sinVALS[17] = 188; 
    sinVALS[18] = 191; 
    sinVALS[19] = 193; 
    sinVALS[20] = 197; 
    sinVALS[21] = 198;
    sinVALS[22] = 199; 
    sinVALS[23] = 200; 
    sinVALS[24] = 200; 
    sinVALS[25] = 200; 
    sinVALS[26] = 200;
    sinVALS[27] = 199; 
    sinVALS[28] = 197; 
    sinVALS[29] = 195; 
    sinVALS[30] = 193; 
    sinVALS[31] = 191;
    sinVALS[32] = 188; 
    sinVALS[33] = 185; 
    sinVALS[34] = 181; 
    sinVALS[35] = 177; 
    sinVALS[36] = 173;
    sinVALS[37] = 169; 
    sinVALS[38] = 164; 
    sinVALS[39] = 159; 
    sinVALS[40] = 154;
    sinVALS[41] = 148;
    sinVALS[42] = 143;
    sinVALS[43] = 137; 
    sinVALS[44] = 131; 
    sinVALS[45] = 125; 
    sinVALS[46] = 119; 
    sinVALS[47] = 112;
    sinVALS[48] = 106; 
    sinVALS[49] = 101;                  // Here return the value if 0 < x < pi
    sinVALS[50] = 100;                  // and return the negative half of the 
                                        // wave if pi < x < 2*pi
    
    if(x<50){ return(sinVALS[x]); } else { return(200-sinVALS[x-50]);}
  
}
