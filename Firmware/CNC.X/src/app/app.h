/*
 * File:   app.h
 * Author: Justin
 *
 * Created on May 22, 2014, 11:02 PM
 */

/*******************************************************************************

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
 *******************************************************************************/

#ifndef _APP_HEADER_H
#define _APP_HEADER_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <plib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "bsp_config.h"
#include "g_code.h"
#include "motion.h"
#include "report.h"
#include "settings.h"

extern volatile bool xLimitDetected;
extern volatile bool yLimitDetected;
extern volatile bool zLimitDetected;
extern volatile block_t *current_block;










// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************




// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behaviour of the application at various times.
*/

typedef enum
{
    /* Application's state machine's initial state. */
    APP_STATE_INIT,// Initial power up state.

    /* TODO: 080. Define states used by the application state machine. */
    APP_STATE_IDLE,

    APP_STATE_HANDLE_COMMUNICATION,

    APP_STATE_PROCESS_COMMANDS,



} APP_STATES;





// *****************************************************************************
/* Axis

  Summary:
    Holds stepper motor axis control data

  Description:
     This structure defines the needed control signals used by the BSD-02LH to
    control a stepper motor.
*/

typedef struct
{
    /* The application's current state */
    bool enable;
    uint32_t numSteps;
    bool direction;
    int stepRes;
    
    /* TODO: 082. Define the data used by the application. */

} AxisData_t;



// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{

    /* The application's current state */
    APP_STATES state;

    /* TODO: 082. Define the data used by the application. */
} APP_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers or other modules when certain events
   occur. */

/* TODO: 083. Define protypes for any callback functions provided by the
 * application.
 */

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:


  Description:


  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks ( void );

/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */
void read_buffer(void);

extern APP_DATA appObject;
extern char rx_Fifo[RX_FIFO_LIMIT];
extern uint32_t rx_Fifo_Head, rx_Fifo_Tail;
extern char timer_counter;
extern uint64_t steps_X, steps_Y, steps_Z;

#endif /* _APP_HEADER_H */
/*******************************************************************************
 End of File
 */


