


#ifndef I2C_H__
#define I2C_H__

void i2c_master_setup(void);                // Init I2C2, which we use as a master
void i2c_master_start(void);                // Start a transmission on the I2C bus
void i2c_master_restart(void);              // Send a restart 
void i2c_master_send(unsigned char byte);   // Send a byte
unsigned char i2c_master_recv(void);        // Read a byte
void i2c_master_ack(int val);               // send an ack, 1 for stop 0 for "sir, may I have another?"
void i2c_master_stop(void);                 // Send a stop 
void sendMessage(unsigned char address, unsigned char message);
unsigned char getMessage(unsigned char registr);
void initPololu(void);
void getMultipleMessages(unsigned char registr, unsigned char * data, int length)


#endif