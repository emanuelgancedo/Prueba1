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
#define GetSystemClock()       (SYS_CLK_FREQ)

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

/* TODO:  Add any necessary callback funtions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/void MyISR(uintptr_t context, uint32_t alarmCount);


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
int escribir = 0;
   
void MyISR(uintptr_t context, uint32_t alarmCount)
{
    static unsigned int contador = 10;
    
    contador--;
    if(!contador)
    {
        contador = 10;
        PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_0);
    }
    //if(!escribir)
        //escribir =1;
   
}
 
SYS_MODULE_OBJ MyHandleIsr ;
void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    // set the direction of the pins and Open the Timer                 
   MyHandleIsr = DRV_TMR_Open( DRV_TMR_INDEX_0, DRV_IO_INTENT_EXCLUSIVE );
 
   // calculate the divider value and register the ISR
   uint32_t desiredFrequency = 10 ; // 10 hertz
   uint32_t actualFrequency = DRV_TMR_CounterFrequencyGet(MyHandleIsr) ;
   uint32_t divider = actualFrequency/desiredFrequency; // cacluate divider value
   DRV_TMR_AlarmRegister(MyHandleIsr, divider, true, 0 , MyISR);
 
   // Starting the Timer   
   DRV_TMR_Start(MyHandleIsr);
    
   TRISAbits.TRISA0 = 0;
   LATAbits.LATA0 = 0;
   
   /*
    TRISCbits.TRISC2 = 1;
    ANSELCbits.ANSC2 = 1;
    
    
    AD1CAL1 = DEVADC1;                  // Writing Calibration-Data to ADC-Registers
     AD1CAL2 = DEVADC2;
     AD1CAL3 = DEVADC3;
     AD1CAL4 = DEVADC4;
     AD1CAL5 = DEVADC5;
 
    AD1CON1 = 0;                        // First Clear AD1CON1-Register
    AD1CON1bits.STRGSRC = 1;            // Scan Trigger Src = Global Software Trigger (GSWTRG)
    AD1CON1bits.FILTRDLY = 0b11111 ;
    
    AD1CON2 = 0;                        // Clear AD1CON2-Register
     AD1CON2bits.ADCSEL = 1;             // ADC Clock-Src = SYSCLK (200 MHz)
     AD1CON2bits.ADCDIV = 4;             // Clock Divider 4 means (8*TQ) = 25 MHz
     AD1CON2bits.SAMC = 32;              // S&H-Sample Time = 32TAD
 
    AD1CON3 = 0;                        // Clear AD1CON3-Register
 
    AD1CON3bits.VREFSEL = 3;
    AD1IMOD = 0;                        // All SampleAndHold-Units set to Standard-values
                                         // Single-ended Inputs with unsigned integer-Format, Standard-Inputs
     //AD1IMODbits.SH3ALT = 1;             // Alternate Input for Sample And Hold 3 (uses normalla AN3, now AN48 - maybe this has blocked AN3??)
 
    AD1GIRQEN1 = 0;                     // No Interrupts used
     AD1GIRQEN2 = 0;                     // No Interrupts
 
    AD1CSS1 = 0;                        // First Clear all Channel-Scan-Select-Bits (Register 1)
     AD1CSS2 = 0;                        // Clear all CSS-Bits of Register 2
     //AD1CSS1bits.CSS21 = 1;               // Select CSS3 = AN3 for ChannelScan
 
    AD1CMPCON1 = 0;                     // No Comperator-Functions
     AD1CMPCON2 = 0;
     AD1CMPCON3 = 0;
     AD1CMPCON4 = 0;
     AD1CMPCON5 = 0;
     AD1CMPCON6 = 0;
 
    AD1FLTR1 = 0;                       // No Filter / Oversampling
     AD1FLTR2 = 0;
     AD1FLTR3 = 0;
     AD1FLTR4 = 0;
     AD1FLTR5 = 0;
     AD1FLTR6 = 0;

    AD1TRG1 = 0;                        // No Triggers
     AD1TRG2 = 0;                        // No Triggers
     AD1TRG3 = 0;                        // No Triggers

    AD1TRG2bits.TRGSRC7 = 1;            // Trigger for AN3 = STRIG (because AN3 is a Channel1-Input!)

    AD1CON1bits.ADCEN = 1;              // Enable ADC
 
    while (AD1CON2bits.ADCRDY == 0);    // Wait for end of Calibration
    */
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
unsigned int analog;
void APP_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            
              /*              
            //Y+ salida y en 1
            TRISBbits.TRISB12 = 1;
            ANSELBbits.ANSB12 = 1;
                              
            //Y- digital 0
            TRISFbits.TRISF13 = 1;
            //LATFbits.LATF13 = 0;
            
            //X+ en entrada y analogico
            TRISAbits.TRISA1 = 0;
            LATAbits.LATA1 = 1;
            
            //X- alta impedancia
            TRISFbits.TRISF12 = 0;
            LATFbits.LATF12 = 0;
            
                        
            //Leo ADC Y+
            AD1CSS1bits.CSS7 = 1;
            AD1CSS1bits.CSS29 = 0;
            AD1CON3bits.GSWTRG = 1;                         // Manually trigger a ChannelScan-Conversation
            
            while (AD1DSTAT1bits.ARDY7 == 0)                // Wait for AN3-Data Ready
            {        asm("nop");    }

            analog = AD1DATA7;
            
            
            
            //Y+ salida y en 1
            TRISBbits.TRISB12 = 0;
            LATBbits.LATB12 = 1;
                              
            //Y- digital 0
            TRISFbits.TRISF13 = 0;
            LATFbits.LATF13 = 0;
            
            //X+ en entrada y analogico
            TRISAbits.TRISA1 = 1;
            ANSELAbits.ANSA1 = 1;
            
            //X- alta impedancia
            TRISFbits.TRISF12 = 1;
            
                        
            //Leo ADC X+
            AD1CSS1bits.CSS7 = 0;
            AD1CSS1bits.CSS29 = 1;
            AD1CON3bits.GSWTRG = 1;                         // Manually trigger a ChannelScan-Conversation
            
            while (AD1DSTAT1bits.ARDY29 == 0)                // Wait for AN3-Data Ready
            {        asm("nop");    }

            analog = AD1DATA29;
                        
                
                */
                
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
