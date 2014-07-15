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
    uint16_t tmr2Period, tmr3Period;
    /* TODO: 093. Initialize your application's state machine and other
     * parameters.
     */
    // Set stepper motor Step Sizes
    MotionGoHome();
    
    /* Place the App state machine in it's initial state. */
    appObject.state = APP_STATE_HANDLE_COMMUNICATION;
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
            /* TODO: 094. Perform initial tasks needed by your application. */
            break;
        /* Application's initial state. */
        case APP_STATE_PROCESS_COMMANDS:
            /* TODO: 094. Perform initial tasks needed by your application. */

            break;
        /* Application's initial state. */
        case APP_STATE_HANDLE_COMMUNICATION:
            /* TODO: 094. Perform initial tasks needed by your application. */
            read_buffer();
            break;
        /* TODO: 096. Implement your applications state-machine. */

        /* The default state should never be executed. */
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
       if(((rx_Fifo[rx_Fifo_Tail] == '\n') || (rx_Fifo[rx_Fifo_Tail] == '\r')))
       {
           for(counter = lineIndex; counter != rx_Fifo_Tail; counter++)
           {
               if(counter == RX_FIFO_LIMIT-1)
               {
                   counter = 0;
               }
               if(rx_Fifo[counter] == '$')
               {

               }
               else if(rx_Fifo[counter] == ' ')
               {
                   // Throw away whitepace and control characters
               }
               else if(rx_Fifo[counter] == '%')
               {

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
           ExecuteLine(newLine);
           charCount = 0;
           lineIndex = (rx_Fifo_Tail+2);
       }
   }
}

// Executes run-time commands, when required. This is called from various check points in the main
// program, primarily where there may be a while loop waiting for a buffer to clear space or any
// point where the execution time from the last check point may be more than a fraction of a second.
// This is a way to execute runtime commands asynchronously (aka multitasking) with grbl's g-code
// parsing and planning functions. This function also serves as an interface for the interrupts to
// set the system runtime flags, where only the main program handles them, removing the need to
// define more computationally-expensive volatile variables. This also provides a controlled way to
// execute certain tasks without having two or more instances of the same task, such as the planner
// recalculating the buffer upon a feedhold or override.
// NOTE: The sys.execute variable flags are set by any process, step or serial interrupts, pinouts,
// limit switches, or the main program.
void protocol_execute_runtime()
{
  if (sys.execute) { // Enter only if any bit flag is true
    uint8_t rt_exec = sys.execute; // Avoid calling volatile multiple times

    // System alarm. Everything has shutdown by something that has gone severely wrong. Report
    // the source of the error to the user. If critical, Grbl disables by entering an infinite
    // loop until system reset/abort.
    if (rt_exec & (EXEC_ALARM | EXEC_CRIT_EVENT)) {
      sys.state = STATE_ALARM; // Set system alarm state

      // Critical event. Only hard limit qualifies. Update this as new critical events surface.
      if (rt_exec & EXEC_CRIT_EVENT) {
        report_alarm_message(ALARM_HARD_LIMIT);
        report_feedback_message(MESSAGE_CRITICAL_EVENT);
        bit_false(sys.execute,EXEC_RESET); // Disable any existing reset
        do {
          // Nothing. Block EVERYTHING until user issues reset or power cycles. Hard limits
          // typically occur while unattended or not paying attention. Gives the user time
          // to do what is needed before resetting, like killing the incoming stream.
        } while (bit_isfalse(sys.execute,EXEC_RESET));

      // Standard alarm event. Only abort during motion qualifies.
      } else {
        // Runtime abort command issued during a cycle, feed hold, or homing cycle. Message the
        // user that position may have been lost and set alarm state to enable the alarm lockout
        // to indicate the possible severity of the problem.
        report_alarm_message(ALARM_ABORT_CYCLE);
      }
      bit_false(sys.execute,(EXEC_ALARM | EXEC_CRIT_EVENT));
    }

    // Execute system abort.
    if (rt_exec & EXEC_RESET) {
      sys.abort = true;  // Only place this is set true.
      return; // Nothing else to do but exit.
    }

    // Execute and serial print status
    if (rt_exec & EXEC_STATUS_REPORT) {
      //report_realtime_status();
      bit_false(sys.execute,EXEC_STATUS_REPORT);
    }

    // Initiate stepper feed hold
    if (rt_exec & EXEC_FEED_HOLD) {
      //st_feed_hold(); // Initiate feed hold.
      bit_false(sys.execute,EXEC_FEED_HOLD);
    }

    // Reinitializes the stepper module running state and, if a feed hold, re-plans the buffer.
    // NOTE: EXEC_CYCLE_STOP is set by the stepper subsystem when a cycle or feed hold completes.
    if (rt_exec & EXEC_CYCLE_STOP) {
      //st_cycle_reinitialize();
      bit_false(sys.execute,EXEC_CYCLE_STOP);
    }

    if (rt_exec & EXEC_CYCLE_START) {
      //st_cycle_start(); // Issue cycle start command to stepper subsystem
      //if (bit_istrue(settings.flags,BITFLAG_AUTO_START)) {
       // sys.auto_start = true; // Re-enable auto start after feed hold.
     // }
      bit_false(sys.execute,EXEC_CYCLE_START);
    }
  }

  // Overrides flag byte (sys.override) and execution should be installed here, since they
  // are runtime and require a direct and controlled interface to the main stepper program.
}


// Directs and executes one line of formatted input from protocol_process. While mostly
// incoming streaming g-code blocks, this also executes Grbl internal commands, such as
// settings, initiating the homing cycle, and toggling switch states. This differs from
// the runtime command module by being susceptible to when Grbl is ready to execute the
// next line during a cycle, so for switches like block delete, the switch only effects
// the lines that are processed afterward, not necessarily real-time during a cycle,
// since there are motions already stored in the buffer. However, this 'lag' should not
// be an issue, since these commands are not typically used during a cycle.
uint8_t protocol_execute_line(char *line)
{
  // Grbl internal command and parameter lines are of the form '$4=374.3' or '$' for help
  if(line[0] == '$') {

    uint8_t char_counter = 1;
    uint8_t helper_var = 0; // Helper variable
    float parameter, value;
    switch( line[char_counter] ) {
      case 0 : report_grbl_help(); break;
      case '$' : // Prints Grbl settings
        if ( line[++char_counter] != 0 ) { return(STATUS_UNSUPPORTED_STATEMENT); }
        else { report_grbl_settings(); }
        break;
      case '#' : // Print gcode parameters
        if ( line[++char_counter] != 0 ) { return(STATUS_UNSUPPORTED_STATEMENT); }
       // else { report_gcode_parameters(); }
        break;
      case 'G' : // Prints gcode parser state
        if ( line[++char_counter] != 0 ) { return(STATUS_UNSUPPORTED_STATEMENT); }
//        else { report_gcode_modes(); }
        break;
      case 'C' : // Set check g-code mode
        if ( line[++char_counter] != 0 ) { return(STATUS_UNSUPPORTED_STATEMENT); }
        // Perform reset when toggling off. Check g-code mode should only work if Grbl
        // is idle and ready, regardless of alarm locks. This is mainly to keep things
        // simple and consistent.
        if ( sys.state == STATE_CHECK_MODE ) {
          MotionReset();
          report_feedback_message(MESSAGE_DISABLED);
        } else {
          if (sys.state) { return(STATUS_IDLE_ERROR); }
          sys.state = STATE_CHECK_MODE;
          report_feedback_message(MESSAGE_ENABLED);
        }
        break;
      case 'X' : // Disable alarm lock
        if ( line[++char_counter] != 0 ) { return(STATUS_UNSUPPORTED_STATEMENT); }
        if (sys.state == STATE_ALARM) {
          report_feedback_message(MESSAGE_ALARM_UNLOCK);
          sys.state = STATE_IDLE;
          // Don't run startup script. Prevents stored moves in startup from causing accidents.
        }
        break;
      case 'H' : // Perform homing cycle
        if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) {
          // Only perform homing if Grbl is idle or lost.
          if ( sys.state==STATE_IDLE || sys.state==STATE_ALARM ) {
            MotionGoHome();
           // if (!sys.abort) { protocol_execute_startup(); } // Execute startup scripts after successful homing.
          } else { return(STATUS_IDLE_ERROR); }
        } else { return(STATUS_SETTING_DISABLED); }
        break;
//    case 'J' : break;  // Jogging methods
      // TODO: Here jogging can be placed for execution as a seperate subprogram. It does not need to be
      // susceptible to other runtime commands except for e-stop. The jogging function is intended to
      // be a basic toggle on/off with controlled acceleration and deceleration to prevent skipped
      // steps. The user would supply the desired feedrate, axis to move, and direction. Toggle on would
      // start motion and toggle off would initiate a deceleration to stop. One could 'feather' the
      // motion by repeatedly toggling to slow the motion to the desired location. Location data would
      // need to be updated real-time and supplied to the user through status queries.
      //   More controlled exact motions can be taken care of by inputting G0 or G1 commands, which are
      // handled by the planner. It would be possible for the jog subprogram to insert blocks into the
      // block buffer without having the planner plan them. It would need to manage de/ac-celerations
      // on its own carefully. This approach could be effective and possibly size/memory efficient.
      case 'N' : // Startup lines.
        if ( line[++char_counter] == 0 ) { // Print startup lines
          for (helper_var=0; helper_var < N_STARTUP_LINE; helper_var++) {
            if (!(settings_read_startup_line(helper_var, line))) {
              report_status_message(STATUS_SETTING_READ_FAIL);
            } else {
              //report_startup_line(helper_var,line);
            }
          }
          break;
        } else { // Store startup line
          helper_var = true;  // Set helper_var to flag storing method.
          // No break. Continues into default: to read remaining command characters.
        }
      default :  // Storing setting methods
        if(!read_float(line, &char_counter, &parameter)) { return(STATUS_BAD_NUMBER_FORMAT); }
        if(line[char_counter++] != '=') { return(STATUS_UNSUPPORTED_STATEMENT); }
        if (helper_var) { // Store startup line
          // Prepare sending gcode block to gcode parser by shifting all characters
          helper_var = char_counter; // Set helper variable as counter to start of gcode block
          do {
            line[char_counter-helper_var] = line[char_counter];
          } while (line[char_counter++] != 0);
          // Execute gcode block to ensure block is valid.
          helper_var = ExecuteLine(line); // Set helper_var to returned status code.
          if (helper_var) { return(helper_var); }
          else {
            helper_var = (uint8_t)(parameter); // Set helper_var to int value of parameter
            settings_store_startup_line(helper_var,line);
          }
        } else { // Store global setting.
          if(!read_float(line, &char_counter, &value)) { return(STATUS_BAD_NUMBER_FORMAT); }
          if(line[char_counter] != 0) { return(STATUS_UNSUPPORTED_STATEMENT); }
          return(settings_store_global_setting(parameter, value));
        }
    }
    return(STATUS_OK); // If '$' command makes it to here, then everything's ok.

  } else {
    return(ExecuteLine(line));    // Everything else is gcode
  }
}

/*******************************************************************************
 End of File
 */



