/* 
 * File:   main5.c
 * Author: mkf434
 *
 * Created on May 12, 2018, 11:47 PM
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

#define SLAVE_ADDR_WRITE 0x40
#define SLAVE_ADDR_READ 0x41


// Helper Function Prototypes

void i2c_master_setup(void);
void i2c_master_start(void);
void i2c_master_restart(void);
void i2c_master_send(unsigned char byte);
unsigned char i2c_master_recv(void);
void i2c_master_ack(int val);
void i2c_master_stop(void);
void initExpander();
void setExpander(char pin, char level);
char getExpander();

// Main Function
void main(void) {
    TRISAbits.TRISA4 = 0;                                   // A4 is a digital output
    LATAbits.LATA4 = 1;                                     // A4 is on initially

    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<48000){;}  // wait 2 ms
    
    i2c_master_setup();               // init I2C2, which we use as a master  
    initExpander();
    char level = 0x0; 
    char pin = 0x1; 
    
    //while(1){
    //     level = getExpander();
    //    setExpander(pin,level);
    //}
    
}

// Helper Function Prototypes

// I2C Master utilities, 100 kHz, using polling rather than interrupts
// The functions must be callled in the correct order as per the I2C protocol
// Change I2C1 to the I2C channel you are using
// I2C pins need pull-up resistors, 2k-10k

void i2c_master_setup(void) {
  ANSELBbits.ANSB2 = 0;               // Turn of default analog input on B2 and B3
  ANSELBbits.ANSB3 = 0;
  
  I2C2BRG = 53;                    // I2CBRG = [1/(2*Fsck) - PGD]*Pblck - 2 
                                    // PGD = 104 ns, 100hz for now, Pbclk = 48 MHz                          
  I2C2CONbits.ON = 1;               // turn on the I2C2 module
  
}

// Start a transmission on the I2C bus
void i2c_master_start(void) {
    I2C2CONbits.SEN = 1;            // send the start bit
    while(I2C2CONbits.SEN) { ; }    // wait for the start bit to be sent
}

void i2c_master_restart(void) {     
    I2C2CONbits.RSEN = 1;           // send a restart 
    while(I2C2CONbits.RSEN) { ; }   // wait for the restart to clear
}

void i2c_master_send(unsigned char byte) {      // send a byte to slave
  I2C2TRN = byte;                               // if an address, bit 0 = 0 for write, 1 for read
  while(I2C2STATbits.TRSTAT) { ; }              // wait for the transmission to finish
  if(I2C2STATbits.ACKSTAT) {                    // if this is high, slave has not acknowledged
                                                // ("I2C2 Master: failed to receive ACK\r\n");
  }
}

unsigned char i2c_master_recv(void) { // receive a byte from the slave
    I2C2CONbits.RCEN = 1;             // start receiving data
    while(!I2C2STATbits.RBF) { ; }    // wait to receive the data
    return I2C2RCV;                   // read and return the data
}

void i2c_master_ack(int val) {        // sends ACK = 0 (slave should send another byte)
                                      // or NACK = 1 (no more bytes requested from slave)
    I2C2CONbits.ACKDT = val;          // store ACK/NACK in ACKDT
    I2C2CONbits.ACKEN = 1;            // send ACKDT
    while(I2C2CONbits.ACKEN) { ; }    // wait for ACK/NACK to be sent
}

void i2c_master_stop(void) {          // send a STOP:
  I2C2CONbits.PEN = 1;                // comm is complete and master relinquishes bus
  while(I2C2CONbits.PEN) { ; }        // wait for STOP to complete
}

void initExpander(){
    
  unsigned char IODIR_REG = 0x00;      // IODIR register address on MCP23008
  unsigned char OLAT_REG = 0x0A;       // OLAT register address on MCP23008
  unsigned char SET_IODIR = 0x00;      // IODIR register values to be set
  unsigned char SET_OLAT = 0xFF;       // OLAT register values to be set
  
  i2c_master_start();                  // Begin the start sequence
  i2c_master_send(SLAVE_ADDR_WRITE);         // send the slave address
  i2c_master_send(IODIR_REG);          // send iodir register address     
  i2c_master_send(SET_IODIR);          // send iodir register bit values
  i2c_master_stop(); 
  
  i2c_master_start();                  // Begin the start sequence
  i2c_master_send(SLAVE_ADDR_WRITE);   // send the slave address
  i2c_master_send(OLAT_REG);           // send iodir register address     
  i2c_master_send(SET_OLAT);           // send iodir register bit values
  i2c_master_stop(); 
  
}

char getExpander(){                    // Get value of inputs from MCP23008
    
  i2c_master_start();                  // Begin the start sequence
  i2c_master_send(SLAVE_ADDR_WRITE);         // send the slave address
  i2c_master_restart();                // send a RESTART so we can begin reading 
  i2c_master_send(SLAVE_ADDR_READ);         // send the slave address
  char r = i2c_master_recv();          // save the value of GP7
  i2c_master_ack(1);                   // make the ack so the slave knows we got it
  i2c_master_stop();                   // make the stop bit
  
  return r;
    
}

void setExpander(char pin, char level){
  
  unsigned char OLAT_REG = 0x0A;       // OLAT register address on MCP23008
  unsigned char LED_ON = 0xFF;         // Make GP0 high, turning on the LED
  unsigned char LED_OFF = 0x00;        // Make Gp0 low, turning off the LED
  
  if(pin && level) 
    {
        i2c_master_start();                  // Begin the start sequence
        i2c_master_send(SLAVE_ADDR_WRITE);         // send the slave address
        i2c_master_send(OLAT_REG);           // send iodir register address     
        i2c_master_send(LED_ON);             // send iodir register bit values
        i2c_master_stop(); 
    } else {
              i2c_master_start();                  // Begin the start sequence
              i2c_master_send(SLAVE_ADDR_WRITE);         // send the slave address
              i2c_master_send(OLAT_REG);           // send iodir register address     
              i2c_master_send(LED_OFF);            // send iodir register bit values
              i2c_master_stop(); 

           }
    
}



