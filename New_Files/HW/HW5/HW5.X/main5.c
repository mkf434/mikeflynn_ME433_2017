/* 
 * File:   main.c
 * Author: Mike Flynn
 * Created on May 12, 2018, 11:47 PM
 * 
 * This file uses the GPIO expander chip MCP23008 and the PIC32MX250F128B
 *  to read the input on an input pin on the expander chip, which is 
 *  connected to an pushbutton, if the button is pushed, the expander chip 
 *  turns on one of the output pins to turn on an led and turns it off if 
 *  the pushbutton is released. All done through I2C communication between
 *  the PIC (master) and the MCP(slave)
 * 
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

#define SLAVE_ADDR_WRITE 0x40           // Slave address with and without the
#define SLAVE_ADDR_READ 0x41            //  read/write bit in hexidecimal


// Helper Function Prototypes

void i2c_master_setup(void);                // Init I2C2, which we use as a master
void i2c_master_start(void);                // Start a transmission on the I2C bus
void i2c_master_restart(void);              // Send a restart 
void i2c_master_send(unsigned char byte);   // Send a byte
unsigned char i2c_master_recv(void);        // Read a byte
void i2c_master_ack(int val);               // send an ack, 1 for stop 0 for "sir, may I have another?"
void i2c_master_stop(void);                 // Send a stop 
void initExpander();                        // Set GP0-3 as outputs and GP4-7 as inputs
void setExpander(char pin, char level);     // Change values on the outputs according to inputs
char getExpander();                         // Figure out if the button is being pressed

// Main Function

void main(void) {
    TRISAbits.TRISA4 = 0;           // A4 is a digital output
    LATAbits.LATA4 = 1;             // A4 is on initially

    i2c_master_setup();             // init I2C2, which we use as a master 
    initExpander();                 // Set GP0-3 as outputs and GP4-7 as inputs   
    char level = 0xFF;              // Initialize argument variables 
    char pin = 0xFF;                //  used later in the code 
    
    
    while(1){
            
        _CP0_SET_COUNT(0);                      // Reset clock
        while(_CP0_GET_COUNT()<48000){;}        // Wait 2 ms
        level = 0x00;                           // Reset level, just to be safe
        level = getExpander();                  // Get the port levels
        setExpander(pin,level);                 // Change values accordingly 
           
   }
    
}

// Helper Functions

// I2C Master utilities, 100 kHz, using polling rather than interrupts
// The functions must be called in the correct order as per the I2C protocol
// Change I2C1 to the I2C channel you are using
// I2C pins need pull-up resistors, 2k-10k

void i2c_master_setup(void) {
  ANSELBbits.ANSB2 = 0;               // Turn of default analog input on B2 and B3
  ANSELBbits.ANSB3 = 0;
  
  I2C2BRG = 233;                    // I2CBRG = [1/(2*Fsck) - PGD]*Pblck - 2 
                                    // PGD = 104 ns, 100hz for now, Pbclk = 48 MHz                          
  I2C2CONbits.ON = 1;               // turn on the I2C2 module
  
}


void i2c_master_start(void) {       // Start a transmission on the I2C bus
    I2C2CONbits.SEN = 1;            // Send the start bit
    while(I2C2CONbits.SEN) { ; }    // Wait for the start bit to be sent
}

void i2c_master_restart(void) {     
    I2C2CONbits.RSEN = 1;           // Send a restart 
    while(I2C2CONbits.RSEN) { ; }   // Wait for the restart to clear
}

void i2c_master_send(unsigned char byte) {      // Send a byte to slave
  I2C2TRN = byte;                               // If an address, bit 0 = 0 for write, 1 for read
  while(I2C2STATbits.TRSTAT) { ; }              // Wait for the transmission to finish
  if(I2C2STATbits.ACKSTAT) {                    // If this is high, slave has not acknowledged
                                                // ("I2C2 Master: failed to receive ACK\r\n");
  }
}

unsigned char i2c_master_recv(void) { // Receive a byte from the slave
    I2C2CONbits.RCEN = 1;             // Start receiving data
    while(!I2C2STATbits.RBF) { ; }    // Wait to receive the data
    return I2C2RCV;                   // Read and return the data
}

void i2c_master_ack(int val) {        // Sends ACK = 0 (slave should send another byte)
                                      //  or NACK = 1 (no more bytes requested from slave)
    I2C2CONbits.ACKDT = val;          // Store ACK/NACK in ACKDT
    I2C2CONbits.ACKEN = 1;            // Send ACKDT
    while(I2C2CONbits.ACKEN) { ; }    // Wait for ACK/NACK to be sent
}

void i2c_master_stop(void) {          // Send a STOP:
  I2C2CONbits.PEN = 1;                // Comm is complete and master relinquishes bus
  while(I2C2CONbits.PEN) { ; }        // Wait for STOP to complete
}

void initExpander(){                    // Set GP0-3 as outputs and GP4-7 as inputs
    
  unsigned char IODIR_REG = 0x00;       // IODIR register address on MCP23008
  unsigned char OLAT_REG = 0x0A;        // OLAT register address on MCP23008
  unsigned char SET_IODIR = 0xF0;       // IODIR register values to be set
  unsigned char SET_OLAT = 0x02;        // OLAT register values to be set
  
  i2c_master_start();                   // Begin the start sequence
  i2c_master_send(SLAVE_ADDR_WRITE);    // Send the slave address
  i2c_master_send(IODIR_REG);           // Send iodir register address     
  i2c_master_send(SET_IODIR);           // Send iodir register bit values
  i2c_master_stop(); 
  
  i2c_master_start();                   // Begin the start sequence
  i2c_master_send(SLAVE_ADDR_WRITE);    // Send the slave address
  i2c_master_send(OLAT_REG);            // Send iodir register address     
  i2c_master_send(SET_OLAT);            // Send iodir register bit values
  i2c_master_stop(); 
  
}

char getExpander(){                    // Figure out if the button is being pressed
    
  i2c_master_start();                  // Begin the start sequence
  i2c_master_send(SLAVE_ADDR_WRITE);   // Send the slave address
  i2c_master_send(0x09);               // Write to GPIO register
  i2c_master_restart();                // Send a RESTART so we can begin reading 
  i2c_master_send(SLAVE_ADDR_READ);    // Send the slave address
  unsigned char r = i2c_master_recv(); // Save the value of GP7
  i2c_master_ack(1);                   // Make the ack so the slave knows we got it
  i2c_master_stop();                   // Make the stop bit
  
  return r;                            // Return what was sent
    
}

void setExpander(char pin, char level){     // Change values on the outputs according to inputs
    
  unsigned char OLAT_REG = 0x0A;            // OLAT register address on MCP23008
  unsigned char LED_ON = 0x03;              // Make GP0 high, turning on the LED
  unsigned char LED_OFF = 0x02;             // Make Gp0 low, turning off the LED
  
  if(level==0x02|level==0x03){
        i2c_master_start();                       // Begin the start sequence
        i2c_master_send(SLAVE_ADDR_WRITE);        // Send the slave address
        i2c_master_send(OLAT_REG);                // Send iodir register address     
        i2c_master_send(0x02);                    // Send iodir register bit values
        i2c_master_stop(); 

        LATAbits.LATA4 = 1;                       // A4 is on 
        _CP0_SET_COUNT(0);                        // Reset clock
        while(_CP0_GET_COUNT()<48000){;}         // Wait 2 ms
        LATAbits.LATA4 = 0;                       // A4 is off

    } else {
            i2c_master_start();                       // Begin the start sequence
            i2c_master_send(SLAVE_ADDR_WRITE);        // Send the slave address
            i2c_master_send(OLAT_REG);                // Send iodir register address     
            i2c_master_send(0x03);                    // Send iodir register bit values
            i2c_master_stop();                        // Send a stop 

            LATAbits.LATA4 = 0;                       // A4 is off
            _CP0_SET_COUNT(0);                        // Reset clock
            while(_CP0_GET_COUNT()<48000){;}          // Wait 2 ms
            LATAbits.LATA4 = 1;                       // A4 is off
            }
    
}



