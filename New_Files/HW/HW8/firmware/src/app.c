/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include <xc.h>                         // Pretty standard header files
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/attribs.h>

#include "ILI9163C.h"
#include "I2C.h"
#include "LCD.h"

#include "app.h"


#define WHO_AM_I 0x0F
#define OUT_TEMP_L 0x20
#define OUT_TEMP_H 0x21



// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    TRISAbits.TRISA4 = 0;             // A4 is a digital output
    LATAbits.LATA4 = 0;               // A4 is off initially
    TRISBbits.TRISB4 = 1;             // B4 is an input 
    

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
                appData.state = APP_STATE_SERVICE_TASKS;
                
                LCDinit();
                initPololu();

                clearScreen();
                
                char string2[200];
                sprintf(string2, "HW1 - 2kHz LED");
                writeString(string2,25,20,WHITE);
                _CP0_SET_COUNT(0);
                while(_CP0_GET_COUNT()<24000000){;}
                
                
                
                
                
                int count = 0;
                while(count<10000){
                    
                    
                
                    _CP0_SET_COUNT(0);                                  // Set count to 0                        
                    while(_CP0_GET_COUNT() < 12000) {};                 // Wait .5 ms

                    LATAbits.LATA4 = 1;                                 // A4 off

                    _CP0_SET_COUNT(0); 
                    while(_CP0_GET_COUNT() < 12000) {};

                    LATAbits.LATA4 = 0; 
                    
                    if(count==0){sprintf(string2, "LED Now Blinking");
                writeString(string2,20,40,WHITE);}
                    
                    count++;                                            // A4 on
                }
                
                clearScreen();
                
                
                sprintf(string2, "HW7 - IMU");
                writeString(string2,40,10,WHITE);
                _CP0_SET_COUNT(0);
                while(_CP0_GET_COUNT()<24000000){;}
                
                

                unsigned char r;
                unsigned char r2;
                unsigned char r3; 

                r = getMessage(WHO_AM_I);
                r2 = getMessage(OUT_TEMP_L);
                r3 = getMessage(OUT_TEMP_H);

                

               if(r==0x69) {

                
                sprintf(string2, "Successful Contact");
                writeString(string2,15,20,WHITE);
                _CP0_SET_COUNT(0);
                while(_CP0_GET_COUNT()<48000000){;}

                } else {

                char string1[200];
                sprintf(string1, "Keep Trying");
                writeString(string1,30,60,WHITE);
                _CP0_SET_COUNT(0);
                while(_CP0_GET_COUNT()<48000000){;}

                }

                

                char string4[200];
                sprintf(string4, "Talking to Pololu");
                writeString(string4,17,30,WHITE);

                _CP0_SET_COUNT(0);
                while(_CP0_GET_COUNT()<48000000){;}

                unsigned char data[200];

                unsigned short temp;
                unsigned short gyro_X;
                unsigned short gyro_Y;
                unsigned short gyro_Z;
                unsigned short accel_X;
                unsigned short accel_Y;
                unsigned short accel_Z;

                char rr;    
                unsigned short data2[7];
                char string5[10];
                
                sprintf(string2, "IMU Now Live");
                writeString(string2,35,40,WHITE);

                while(1){

                    _CP0_SET_COUNT(0);
                while(_CP0_GET_COUNT()<2400000){;}              // Wait 12000*200 = 1 ms*200 = .2 s = 5 hz

                getMultipleMessages(OUT_TEMP_L,data,14);

                temp = (data[1]<<8)|data[0];
                data2[0] = temp;
                gyro_X = (data[3]<<8)|data[2];
                data2[1] = gyro_X;
                gyro_Y = (data[5]<<8)|data[4];
                data2[2] = gyro_Y;
                gyro_Z = (data[7]<<8)|data[6];
                data2[3] = gyro_Z;
                accel_X = (data[9]<<8)|data[8];
                data2[4] = accel_X;
                accel_Y = (data[11]<<8)|data[10];
                data2[5] = accel_Y;
                accel_Z = (data[13]<<8)|data[12];
                data2[6] = accel_Z;

                int xxx = 4; 

                

                while(xxx<7){
                    sprintf(string5,"%d",data2[xxx]);
                    clearChar(10,(40 + (xxx*10)));
                    clearChar(15,(40 + (xxx*10)));
                    clearChar(20,(40 + (xxx*10)));
                    clearChar(25,(40 + (xxx*10)));
                    clearChar(30,(40 + (xxx*10)));
                    clearChar(35,(40 + (xxx*10)));
                    writeString(string5,10,40 + (xxx*10),WHITE);

                    xxx++;
                }

                }



                
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
        
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
