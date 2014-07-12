/* 
 * File:   system_definitions.h
 * Author: Justin
 *
 * Created on May 24, 2014, 4:50 PM
 */
/*
 * File:   system_definitions.h
 * Author: Justin
 *
 * Created on May 22, 2014, 11:18 PM
 */



/*******************************************************************************
  System Definitions

  File Name:
    sys_definitions.h

  Summary:
    MPLAB Harmony project system definitions.

  Description:
    This file contains the system-wide protoypes and definitions for an MPLAB
    Harmony project.
 *******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _SYSTEM_DEFINITIONS_H
#define _SYSTEM_DEFINITIONS_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Macros
// *****************************************************************************
// *****************************************************************************
#define T2_FREQUENCY    1000
#define T3_FREQUENCY    1000
#define T4_FREQUENCY    1000
#define T1_FREQUENCY    1000


#define XS_PWM_INIT  (OC_OFF|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER3_SRC|OC_PWM_FAULT_PIN_DISABLE)

#define YZ_PWM_INIT  (OC_OFF|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER2_SRC|OC_CONTINUE_PULSE)

#define XS_PWM_ENA  (OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER3_SRC|OC_CONTINUE_PULSE)

#define YZ_PWM_ENA  (OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER2_SRC|OC_CONTINUE_PULSE)
//#define YZ_PWM_ENA  (T2_ON | T2_SOURCE_INT | T2_PS_1_256)
//#define XS_PWM_ENA  (T3_ON | T3_SOURCE_INT | T3_PS_1_256)

#define TIMER2_INIT (T2_OFF|T2_IDLE_CON|T2_GATE_OFF|T2_PS_1_1|\
                                T2_32BIT_MODE_OFF|T2_SOURCE_INT)

#define TIMER3_INIT (T3_OFF|T3_IDLE_CON|T3_GATE_OFF|T3_PS_1_1|T3_SOURCE_INT)


#define TIMER2_INT_INIT       (T2_INT_ON|T2_INT_PRIOR_3|T2_INT_SUB_PRIOR_0)
#define TIMER3_INT_INIT       (T3_INT_ON|T3_INT_PRIOR_3|T3_INT_SUB_PRIOR_0)

///#define TIMER2_ON (T2_ON|T2_IDLE_CON|T2_GATE_OFF|T2_PS_1_1|T2_32BIT_MODE_OFF|T2_SOURCE_INT)
//#define T2_TICK     (GetPeripheralClock()/T2_PS_1_256/T1_FREQUENCY)

#define TIMER2_ON (T2_ON|T2_IDLE_CON|T2_GATE_OFF|T2_PS_1_1|T2_32BIT_MODE_OFF|T2_SOURCE_INT)
#define T2_TICK     (GetPeripheralClock()/T2_PS_1_256/T2_FREQUENCY)

#define TIMER3_ON (T3_ON|T3_IDLE_CON|T3_GATE_OFF|T3_PS_1_1|T3_SOURCE_INT)
#define T3_TICK     (GetPeripheralClock()/T3_PS_1_256/T3_FREQUENCY)

#define TIMER4_ON (T4_ON|T4_IDLE_CON|T4_GATE_OFF|T4_PS_1_1|T4_SOURCE_INT)
#define T4_TICK   (GetPeripheralClock()/T4_PS_1_256/T4_FREQUENCY)


// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* OC Enumeration.

  Summary:
 Port and Bit number information

  Description:
 Aids in initializing I/O

  Remarks:
    None.
*/


typedef enum
{
    /* PWM 1 */
     OC1 = 0,

    /* PWM 2 */
     OC2 = 1,

    /*  PWM 3 */
     OC3 = 2,

    /* PWM 4 */
     OC4 = 3,

     /*  PWM 5 */
     OC5 = 4
}ocModule_e;

// *****************************************************************************
/* Pin Configuration.

  Summary:
 Port and Bit number information

  Description:
 Aids in initializing I/O

  Remarks:
    None.
*/

typedef struct pinconfig_t{
    /* Port Channel( A,B,C,D,E,F,G ) */
    IoPortId port;
    /* Port Bit( 0,1,2...15 ) */
    uint32_t pin;
}pinconfig_t;



// *****************************************************************************
/* Axis Driver Board Connections.

  Summary:
   Pins used to interface and control each axis.

  Description:
    Structure holds the ports information for each axis's driver board.

  Remarks:
    None.
*/


typedef struct AxisPins_t{
    /* Driver Board Output Enable */
    pinconfig_t enablePin;
    /* Motor Motion Direction Control Pin*/
    pinconfig_t directionPin;
    /* PWM Output Pin Used to Trigger Stepping */
    ocModule_e stepPWM;
    /* Stepping Resolution Pin 1 */
    pinconfig_t mS1Pin;
    /* Stepping Resolutin Pin 2 */
    pinconfig_t mS2Pin;
    /* Logic indicator of initial state step (INPUT) */
    pinconfig_t homePin;

}AxisPins_t;



#endif /* _SYSTEM_DEFINITIONS_H */
/*******************************************************************************
 End of File
*/

