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

#include "app.h"
#include "I2C.h"
#include "ILI9163C.h"
#include "LCD.h"



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
