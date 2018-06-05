
/*/
// Date - 4/26/18
// Author - Mike Flynn
// 
//  Hello_World433.c is the first file created to 
//  run on the PIC32 breadboard circuit, it sends a 
//  pwm signal to an LED. The signal is cut and pauses
//  when the user button is pressed. 
//
/*/


#include<xc.h>                                  // processor SFR definitions
#include<sys/attribs.h>                         // __ISR macro

// DEVCFG0
#pragma config DEBUG = OFF                      // no debugging
#pragma config JTAGEN = OFF                     // no jtag
#pragma config ICESEL = ICS_PGx1                // use PGED1 and PGEC1
#pragma config PWP = OFF                        // no write protect
#pragma config BWP = OFF                        // no boot write protect
#pragma config CP = OFF                         // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL                   // use primary oscillator with pll
#pragma config FSOSCEN = OFF                    // turn off secondary oscillator
#pragma config IESO = OFF                       // no switching clocks
#pragma config POSCMOD = HS                     // high speed crystal mode
#pragma config OSCIOFNC = OFF                   // disable secondary osc
#pragma config FPBDIV = DIV_1                   // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD                   // do not enable clock switch
#pragma config WDTPS = PS1048576                // use slowest wdt
#pragma config WINDIS = OFF                     // wdt no window mode
#pragma config FWDTEN = OFF                     // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25             // wdt window at 25%

// DEVCFG2                                      // get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2                 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24                 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2                 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2                 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON                      // USB clock on

// DEVCFG3
#pragma config USERID = 0101010101010101        // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF                   // allow multiple reconfigurations
#pragma config IOL1WAY = OFF                    // allow multiple reconfigurations
#pragma config FUSBIDIO = ON                    // USB pins controlled by USB module
#pragma config FVBUSONIO = ON                   // USB BUSON controlled by USB module

int main() {

    __builtin_disable_interrupts();                                 // disable the interrupts         
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    BMXCONbits.BMXWSDRM = 0x0;                                      // 0 data RAM access wait states
    INTCONbits.MVEC = 0x1;                                          // enable multi vector interrupts
    DDPCONbits.JTAGEN = 0;                                          // disable JTAG to get pins back
    
    TRISAbits.TRISA4 = 0;                                           // A4 is a digital output
    LATAbits.LATA4 = 1;                                             // A4 is on initially

    __builtin_enable_interrupts();

    while(1) {
	    // use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
		  // remember the core timer runs at half the sysclk
    }
}
