

#include <xc.h>                         // Pretty standard header files
#include <stdio.h>
#include <stdlib.h>

static const char SAD_WRITE = 0xD6;
static const char SAD_READ = 0xD7;

// I2C Master utilities, 100 kHz, using polling rather than interrupts
// The functions must be called in the correct order as per the I2C protocol
// Change I2C1 to the I2C channel you are using
// I2C pins need pull-up resistors, 2k-10k

void i2c_master_setup(void) {
  ANSELBbits.ANSB2 = 0;               // Turn of default analog input on B2 and B3
  ANSELBbits.ANSB3 = 0;
  
  I2C2BRG = 55;                    // I2CBRG = [1/(2*Fsck) - PGD]*Pblck - 2 
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

void initPololu(){                    // Set GP0-3 as outputs and GP4-7 as inputs
    
  TRISAbits.TRISA0 = 0;               // A0 is an output
  LATAbits.LATA0 = 1;                 // A0 is on, CS high to enable I2C
  TRISAbits.TRISA4 = 0;               // A4 is an output
  LATAbits.LATA4 = 1;                 // A4 is on, code got loaded
  i2c_master_setup();                 // Set PGD value and turn on I2C
  
}

void sendMessage(unsigned char registr, unsigned char message){                    // Figure out if the button is being pressed
    
  i2c_master_start();                   // Begin the start sequence
  i2c_master_send(SAD_WRITE);    // Send the slave address
  i2c_master_send(registr);           // Send iodir register address     
  i2c_master_send(message);           // Send iodir register bit values
  i2c_master_stop(); 

}

unsigned char getMessage(unsigned char registr){     // Change values on the outputs according to inputs
    
  i2c_master_start();                  // Begin the start sequence
  i2c_master_send(SAD_WRITE);   // Send the slave address
  i2c_master_send(registr);               // Write to GPIO register
  i2c_master_start();                // Send a START so we can begin reading 
  i2c_master_send(SAD_READ);    // Send the slave address
  unsigned char r = i2c_master_recv(); // Save the value of GP7
  i2c_master_ack(1);                   // Make the nack so the slave knows we got it
  i2c_master_stop();                   // Make the stop bit
  
  return r; 
    
}

void getMultipleMessages( unsigned char registr, unsigned char * data, int length) {
  int x = 0; 

  while(x<length){

    data[x] = getMessage(registr);
    x++;
  }
  
}




