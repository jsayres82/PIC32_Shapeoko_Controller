/* 
 * File:   bsp_config.h
 * Author: Justin
 *
 * Created on May 24, 2014, 4:56 PM
 */

/**************************************************************************
  Company:
 Justin Sayres
  File Name:
    bsp_config.h
  Summary:
    Board support configuration file.
  Description:
    Board support configuration file.

    This contains all the configuration that is required to be done for the
    application running on PIC32 USB starter kit
  **************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _BSP_CONFIG_H
#define _BSP_CONFIG_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <plib.h>
#include "system_definitions.h"
#include "system_config.h"
#include "nuts_bolts.h"
/******************************************************************************/
extern volatile uint32_t milliSecondCount;
extern volatile AxisPins_t  xAxis, yAxis, zAxis;
extern pinconfig_t xLimitInput;
extern pinconfig_t yLimitInput;
// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Input Constant

  Summary:
    Defines the constant which identifies input

  Description:
    This constant identifies input
*/

#define BSP_INPUT        1

#define X_POSITIVE_DIR         (void mPORTGSetbits(BIT_12))
#define Y_POSITIVE_DIR         (void mPORTESetbits(BIT_0))
#define Z_POSITIVE_DIR         (void mPORTASetbits(BIT_1))

#define X_NEGATIVE_DIR         (void mPORTGClearbits(BIT_12))
#define Y_NEGATIVE_DIR         (void mPORTEClearbits(BIT_0))
#define Z_NEGATIVE_DIR         (void mPORTAClearbits(BIT_1))

#define DISABLE_STEPPER_INT()  (CloseTimer2(),CloseTimer3(),CloseTimer4())
#define ENABLE_STEPPER_INT()   (EnableIntT2,EnableIntT3,EnableIntT4)

// *****************************************************************************
/* Output Constant

  Summary:
    Defines the constant which identifies output

  Description:
    This constant identifies output
*/

#define BSP_OUTPUT                              0


// *****************************************************************************
/* Digital Pin Constant

  Summary:
    Defines the constant which identifies digital pin

  Description:
    This constant identifies digital pin
*/

#define BSP_DIGITAL_PIN                         PORTS_PIN_MODE_DIGITAL

#define POSITIVE    1
#define NEGATIVE    0
// *****************************************************************************
/* analog Pin Constant

  Summary:
    Defines the constant which identifies analog pin

  Description:
    This constant identifies analog pin
*/


#define FULL_STEP       1
#define HALF_STEP       2
#define QUARTER_STEP    4
#define SIXTEENTH_STEP  16


// *****************************************************************************
/* Stepper Motor Step Size

  Summary:
    Step size states enumeration

  Description:
    This enumeration defines the valid step sizes for the steppers.
*/

typedef enum
{
    /* Step Size Settings for BSD-02LH. */
    SIXTEENTH = 0,

    QUARTER,

    HALF,

    FULL,

    NUM_STEP_SIZES

} StepSize_e;




// *****************************************************************************
// *****************************************************************************
// Section: UART Pins
// *****************************************************************************
// *****************************************************************************
/* The section below identifies the pins that are associated with the UART
   connected to RS-232 on the board */

#define MY_UART UART2

// *****************************************************************************
// *****************************************************************************
// Section: Analog Inputs 
// *****************************************************************************
// *****************************************************************************
/* The section below identifies the analog inputs connected to potentiometer */


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************

/**************************************************************************
  Function:
     void BSP_Initialize(void);

  Summary:
    Performs the necessary actions to initialize a board
  Description:
    This routine performs the necessary actions to initialize a board
  Parameters:
    None
  Return:
    None
  Conditions:
    None
*/

void BSP_Initialize(void);

// *****************************************************************************
/* Function: void BSP_Timer2Start(int16_t period)

  Summary:
    Turns ON Timer 2.

  Description:
    This function turns ON Timer 2.
  Parameters:
    period: The period to set timer.
  Return:
    None
  Conditions:
    BSP should be initialized by calling void BSP_Initialize(void) function.
*/

void BSP_Timer2Start(int16_t frequency);

// *****************************************************************************
/* Function: void BSP_Timer3Start(int16_t period)

  Summary:
    Turns ON Timer 3.

  Description:
    This function turns ON Timer 3.
  Parameters:
    period: The period to set timer.
  Return:
    None
  Conditions:
    BSP should be initialized by calling void BSP_Initialize(void) function.
*/

void BSP_Timer3Start(int16_t frequency);


// *****************************************************************************
/* Function: void BSP_Timer2Stop(void)

  Summary:
    Stops the timer.

  Description:
    This function turns OFF Timer 3.
  Parameters:
    None
  Return:
    None
  Conditions:
    BSP should be initialized by calling void BSP_Initialize(void) function.
*/

void BSP_Timer2Stop(void);



// *****************************************************************************
/* Function: void BSP_Timer3Stop(void)

  Summary:
   Stops the timer.

  Description:
    This function turns OFF Timer 3.
  Parameters:
    None.
  Return:
    None
  Conditions:
    BSP should be initialized by calling void BSP_Initialize(void) function.
*/

void BSP_Timer3Stop(void);




// *****************************************************************************
/* Function:
   void BSP_PWM_Enable(ocModule_e module, int16_t period, uint16_t dutyCycle)

  Summary:
    Turns on PWM module.

  Description:
    This function turns on the desired PWM at the specified duty cycle.
  Parameters:
    module: Module ID as specified by ocModule_e enum.
    dutyCycle: Duty cycle for the PWM to be set at.
  Return:
    None
  Conditions:
   OC Modules timer should be set before calling this function.
*/

void BSP_PWM_Enable(ocModule_e module, uint16_t dutyCycle);

// *****************************************************************************
/* Function: void BSP_PWM_Disable(ocModule_e module)

  Summary:
   Turns of the PWM output.

  Description:
    This function stops pulse generation of the desired module
  Parameters:
    module: LED ID as specified by ocModule_e enum.
  Return:
    None
  Conditions:
    BSP should be initialized by calling void BSP_Initialize(void) function.
*/

void BSP_PWM_Disable(ocModule_e module);
// *****************************************************************************
/* Function: void BSP_AxisEnable(axis_e axis, direction_e dir)

  Summary:
    Enables the BSD-02LH board outputs.
 Parameters:
    axis: Board ID as specified by axis_e enum.
    dir:  The motor movement direction as specified by direction_e enum.
  Return values:
    None
  Description:
    This routine enables the outputs of the BSD-02Lh and sets the direction.
*/

void BSP_AxisEnable(uint8_t axis, uint8_t dir);

// *****************************************************************************
/* Function: void BSP_AxisDisable(axis_e axis)

  Summary:
   Stops the desired stepper motor

  Description:
   Disables the stepper motor BSD-02LH
  Parameters:
    axis: Board ID as specified by axis_e enum.
  Return values:
    None
  Conditions:
    None
*/

void BSP_AxisDisable(uint8_t axis);


// *****************************************************************************
/* Function: void BSP_SetStepSize(axis_e axis, int stepSize);

  Summary:
    Sets the step size on the BSD-02LH board

  Description:
    Sets the step size on the BSD-02LH board
  Parameters:
    axis: Board ID as specified by axis_e enum.
    stepSize:  The step setting as defined by StepSize_e enum.
  Return Values:
    None
  Conditions:
    None

*/

void BSP_SetStepSize(uint8_t axis, StepSize_e stepSize);




// *****************************************************************************
/* Function:
    int BSP_GetLimitSwitch()
  Summary:
    Gets the status of the X and Y limit switches.
  Description:
    Returns the status of the X and Y limit switches.
  Parameters:
    None
  Return Values:
    X and Y limit switch state.
  Conditions:
    None
*/

int BSP_GetLimitSwitch(void);


// *****************************************************************************
/* Function:
    char BSP_GetUartChar()
  Summary:
    Returns character in the RX register.
  Description:
   Returns character in the RX register.
  Parameters:
    None
  Return Values:
    RX Recieve Register value.
  Conditions:
    None
*/

char BSP_GetUartChar();
// *****************************************************************************
/* Function:
    void BSP_StartTimer(uint32_t period)
  Summary:
    Sets up the core timer to generate Core Timer Interrupt after number
    of instructions specified in the parameter.
  Description:
    This function resets the Count register (Register 9 of the CP0 registers)
    to Zero. Also sets the Compare register (Register 11 of CP0 registers) with
    value provided in the period parameter. Core Timer Interrupt is generated when
    Count register value reaches the Compare register.
  Parameters:
    period: timer count in number of instructions.
  Return Values:
    None.
  Conditions:
    None
*/
//void BSP_StartTimer(uint32_t period);
void BSP_PutCharacter(const char character);
void BSP_WriteString(const char *string);
void BSP_MoveAxis(uint8_t axis, uint8_t dir, uint16_t period);
#endif //_BSP_CONFIG_H

/*******************************************************************************
 End of File
*/



