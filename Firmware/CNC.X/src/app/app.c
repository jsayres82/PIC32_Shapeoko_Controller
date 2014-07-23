/*******************************************************************************
  MPLAB Harmony Application Source File

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
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
uint32_t stepSize[NUM_STEP_SIZES] =
{
    FULL_STEP,
    HALF_STEP,
    QUARTER_STEP,
    SIXTEENTH_STEP
};
// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appObject;



char newLine[LINE_SIZE];
uint32_t instructionCount=0;
char instruction[256];
char rx_Fifo[RX_FIFO_LIMIT];
uint32_t rx_Fifo_Head = 0;
uint32_t rx_Fifo_Tail = 0;
char timer_counter;
uint64_t steps_X = 0;
uint64_t steps_Y = 0;
uint64_t steps_Z = 0;
volatile bool xLimitDetected = FALSE;    // Set if X limit interrupt activ
volatile bool yLimitDetected = FALSE;
volatile bool zLimitDetected = FALSE;
// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO: 090. Implement any local functions necessary to complete this
 * application.
 */


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO: 092. Implement any callback functions necessary to complete this
 * application.
 */


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine
// *****************************************************************************
// *****************************************************************************

/******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    GcodeInit();
    //settings_init(); // Load grbl settings from EEPROM
    //memset(&sys, 0, sizeof(sys));  // Clear all system variables
    sys.abort = true;   // Set abort to complete initialization
    sys.state = STATE_INIT;  // Set alarm state to indicate unknown initial position
    uint16_t tmr2Period, tmr3Period;
    /* TODO: 093. Initialize your application's state machine and other
     * parameters.
     */
    // Set stepper motor Step Sizes
    MotionGoHome();
    
    /* Place the App state machine in it's initial state. */
    appObject.state = APP_STATE_INIT;
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    /* check the application state*/
    switch ( appObject.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:

            // Execute system reset upon a system abort, where the main program will return to this loop.
            // Once here, it is safe to re-initialize the system. At startup, the system will automatically
            // reset to finish the initialization process.
            if (sys.abort)
            {
              // Reset system.
              //serial_reset_read_buffer(); // Clear serial read buffer
              plan_init(); // Clear block buffer and planner variables
              //gc_init(); // Set g-code parser to default state
              //protocol_init(); // Clear incoming line data and execute startup lines
              //spindle_init();
              //coolant_init();
              //limits_init();
              //st_reset(); // Clear stepper subsystem variables.

              // Sync cleared gcode and planner positions to current system position, which is only
              // cleared upon startup, not a reset/abort.
              sys_sync_current_position();

              // Reset system variables.
              sys.abort = false;
              sys.execute = 0;
              if (bit_istrue(settings.flags,BITFLAG_AUTO_START)) { sys.auto_start = true; }

              // Check for power-up and set system alarm if homing is enabled to force homing cycle
              // by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
              // startup scripts, but allows access to settings and internal commands. Only a homing
              // cycle '$H' or kill alarm locks '$X' will disable the alarm.
              // NOTE: The startup script will run after successful completion of the homing cycle, but
              // not after disabling the alarm locks. Prevents motion startup blocks from crashing into
              // things uncontrollably. Very bad.
              #ifdef HOMING_INIT_LOCK
                if (sys.state == STATE_INIT && bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE))
                    sys.state = STATE_ALARM;
              #endif

              // Check for and report alarm state after a reset, error, or an initial power up.
              if (sys.state == STATE_ALARM)
                report_feedback_message(MESSAGE_ALARM_LOCK);
              else
              {
                sys.state = STATE_IDLE;              // All systems go. Set system to ready
                protocol_execute_startup();          // Execute startup script.
              }
            }
            appObject.state = APP_STATE_HANDLE_COMMUNICATION;
            break;
       
        case APP_STATE_PROCESS_COMMANDS:
       
            break;
        /* Application's Handle communication state. */
        case APP_STATE_HANDLE_COMMUNICATION:
            protocol_execute_runtime();
            read_buffer();
            break;
      
        default:
            /* TODO: 098. Handle error in application's state machine. */
           break;
    }
}

void read_buffer(void)
{
    // while(rx_Fifo_Head == rx_Fifo_Tail)     /* Wait for an input which will cause*/
    //     continue;                         /* the head pointer to increase one */
    uint32_t counter;
    static uint32_t lineIndex = 1;
    static uint32_t charCount = 0;
   while(rx_Fifo_Head != rx_Fifo_Tail)
   {
       rx_Fifo_Tail++;                   /* Increase the tail to read*/
                                           /* the next input value*/
       if (rx_Fifo_Tail == RX_FIFO_LIMIT)   /*If we've reached the last */
           rx_Fifo_Tail = 0;               /*of our buffer, then we go back */
                                           /* to the first location*/
      
       /* If we come to a Carriage Return then the */
           /* line has ended so interpret that line */
       if(((rx_Fifo[rx_Fifo_Tail] == '\n') || (rx_Fifo[rx_Fifo_Tail] == '\r')))// End of line reached
       {

           // Runtime command check point before executing line. Prevent any furthur line executions.
           // NOTE: If there is no line, this function should quickly return to the main program when
           // the buffer empties of non-executable data.
           protocol_execute_runtime();
           if (sys.abort) { return; } // Bail to main program upon system abort

//           if (char_counter > 0)// Line is complete. Then execute!
//           {
//               line[char_counter] = 0; // Terminate string
//               report_status_message(protocol_execute_line(line));
//           }
//           else  // Empty or comment line. Skip block.
//           {
//               report_status_message(STATUS_OK); // Send status message for syncing purposes.
//           }
//
//           protocol_reset_line_buffer();
//       }
//       else
//       {
//           if (iscomment) // Throw away all comment character
//               if (c == ')') // End of comment. Resume line.
//                   iscomment = false;
//       }
//    }
//    else
//    {
           for(counter = lineIndex; counter != rx_Fifo_Tail; counter++)
           {
               if(counter == RX_FIFO_LIMIT-1)
               {
                   counter = 0;
               }
               else if(rx_Fifo[counter] == ' ')
               {
                   // Throw away whitepace and control characters
               }
               else if(rx_Fifo[counter] == '/')
               {
                   // Block delete not supported. Ignore character.
               }
               else if((rx_Fifo[counter] >= 'a')&&(rx_Fifo[counter] <= 'z'))    // If the letters are lowercase
               {
                   newLine[charCount++] = (rx_Fifo[counter]-32);                   // Set them to uppercase
               }
               else
               {
                   newLine[charCount++] =  rx_Fifo[counter];
               }

           }
           newLine[charCount] = 0;
           report_status_message(protocol_execute_line(newLine));
           charCount = 0;
           lineIndex = (rx_Fifo_Tail+2);
       }
   }
}



/*******************************************************************************
 End of File
 */



