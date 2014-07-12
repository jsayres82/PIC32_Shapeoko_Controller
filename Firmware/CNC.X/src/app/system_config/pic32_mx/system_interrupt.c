/*******************************************************************************
 System Interrupts File

  File Name:
    system_int.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It impements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmomy system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" funcitons to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2012 released Microchip Technology Inc.  All rights reserved.

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

#include <xc.h>
#include <sys/attribs.h>
#include "app.h"
#include "bsp_config.h"

char uart_rd = 0;
volatile block_t *current_block = NULL;  // A pointer to the block currently being traced
// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

/* TODO: 030. PIC32MX - Implement interrupt vector functions required by any
 * interrupt-driven modules in the system. For driver tasks that are interrupt
 * driven, the interrupt vector should point to the drivers task function.
 * The drivers task function implements its state machine.  See the Driver
 * Overview section of the MPLABX Harmony Help.
 */



void __ISR(_EXTERNAL_1_VECTOR, ipl1) _Interrupt_Z_Limit(void)
{
   
    BSP_AxisDisable(Z_AXIS);
    zLimitDetected = TRUE;
    /* Clear the interrupt flag */
    mINT1ClearIntFlag();
}



void __ISR(_EXTERNAL_2_VECTOR, ipl1) _Interrupt_XY_Limit(void)
{
    unsigned int temp;

    temp = mPORTCRead();
    
    if(temp & xLimitInput.pin)
    {
        xLimitDetected = TRUE;
        BSP_AxisDisable(X_AXIS);
        //BSP_Timer2Stop();
    }
    if(temp & yLimitInput.pin)
    {
        yLimitDetected = TRUE;
        BSP_AxisDisable(Y_AXIS);
        //BSP_Timer2Stop();
    }

    /* Clear the interrupt flag */
    mINT2ClearIntFlag();


}

void __ISR(_TIMER_4_VECTOR, ipl2) _InterruptHandler_TMR4(void)
{
   // if(current_block->direction_bits[Z_AXIS])
    //   PORTSetBits(zAxis.directionPin.port, zAxis.directionPin.pin);
   // else
     //  PORTClearBits(zAxis.directionPin.port, zAxis.directionPin.pin);
    if(steps_Z)
    {
        if(!(mPORTAReadBits(zAxis.enablePin.pin)))
            steps_Z--;
    }
    if (current_block != Null)
    {
        if (current_block->steps_z)
        {
            if(!(mPORTAReadBits(zAxis.enablePin.pin)))
                current_block->steps_z--;
        }
       else
        {
            //CloseOC2();
            BSP_AxisDisable(Z_AXIS);
            BSP_Timer4Stop();
        }
    }
    else
    {
       BSP_AxisDisable(Z_AXIS);
    }
    // clear the interrupt flag
    mT4ClearIntFlag();
}

void __ISR(_TIMER_3_VECTOR, ipl2) _InterruptHandler_TMR3(void)
{

    // if(current_block->direction_bits[Y_AXIS])
    //   PORTSetBits(xAxis.directionPin.port, xAxis.directionPin.pin);
    // else
    //     PORTClearBits(yAxis.directionPin.port, yAxis.directionPin.pin);
    if(steps_X)
    {
        if(!(mPORTGReadBits(xAxis.enablePin.pin)))
            steps_X--;
    }
    else if (current_block != Null)
    {
        if(current_block->steps_x)
        {
            if(!(mPORTGReadBits(xAxis.enablePin.pin)))
                current_block->steps_x--;
        }
        else
        {
            //CloseOC2();
            BSP_AxisDisable(X_AXIS);
            BSP_Timer3Stop();
        }
    }
    else
    {
        //CloseOC2();
        BSP_AxisDisable(X_AXIS);
        BSP_Timer3Stop();
    }
     
    // clear the interrupt flag
    mT3ClearIntFlag();
}

void __ISR(_TIMER_2_VECTOR, ipl3) _InterruptHandler_TMR2(void)
{
    //if(current_block->direction_bits[X_AXIS])
    //    PORTSetBits(xAxis.directionPin.port, xAxis.directionPin.pin);
    //else
    //   PORTClearBits(xAxis.directionPin.port, xAxis.directionPin.pin);
    if(steps_Y)
    {
        if(!(mPORTEReadBits(yAxis.enablePin.pin)))
            steps_Y--;
    }
    else if (current_block != Null)
    {
        if( (current_block->steps_y))
        {
            if(!(mPORTEReadBits(yAxis.enablePin.pin)))
                current_block->steps_y--;
        }
        else
        {
            //CloseOC1();
            BSP_AxisDisable(Y_AXIS);
            BSP_Timer2Stop();
        }
    }
    else
    {
       //CloseOC1();
       BSP_AxisDisable(Y_AXIS);
       BSP_Timer2Stop();
    }
        
     // clear the interrupt flag
     mT2ClearIntFlag();
}



//Main Timer for total movement time and block handling
void __ISR(_TIMER_1_VECTOR, ipl3) _InterruptHandler_TMR1(void)
{
    // clear the interrupt flag
    mT1ClearIntFlag();

    if(current_block == Null)
    {
        current_block = plan_get_current_block();
        if(current_block != Null)
        {
            if(current_block->activeAxisCount == 3)
            {
                if(current_block->minStepAxis < N_AXIS) // If they are not all equal  // TODO:  If any of the step counts are equal we dont need Timer4
                {
                    switch(current_block->minStepAxis)  // Need to configure OC Module to be Single Pulse Output and other two OC modules to be continuous pulse
                    {
                        case X_AXIS:
                            BSP_Timer4Start((uint16_t)current_block->steppingFreq[X_AXIS]);// Use Timer4 Interrupt to trigger single output pulse
                            OpenOC2((OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER3_SRC|OC_SINGLE_PULSE),  (ReadPeriod3()>>1), ReadPeriod3());   // X_AXIS = Single Pulse

                            BSP_Timer2Start((uint16_t)current_block->steppingFreq[Y_AXIS]);
                            OpenOC1((OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER2_SRC|OC_CONTINUE_PULSE),  (ReadPeriod2()>>1), ReadPeriod2()); // Y_AXIS = Continuous Pulse

                            BSP_Timer3Start((uint16_t)current_block->steppingFreq[Z_AXIS]);
                            OpenOC3((OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER3_SRC|OC_CONTINUE_PULSE),  (ReadPeriod3()>>1), ReadPeriod3()); // Z_AXIS = Continuous Pulse
                            break;

                        case Y_AXIS:
                            BSP_Timer4Start((uint16_t)current_block->steppingFreq[Y_AXIS]);// Use Timer4 Interrupt to trigger single output pulse
                            OpenOC1((OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER3_SRC|OC_SINGLE_PULSE), (ReadPeriod3()>>1), ReadPeriod3());    // Y_AXIS = Single Pulse

                            BSP_Timer2Start((uint16_t)current_block->steppingFreq[X_AXIS]);
                            OpenOC2((OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER2_SRC|OC_CONTINUE_PULSE),  (ReadPeriod2()>>1), ReadPeriod2());   // X_AXIS = Continuous Pulse

                            BSP_Timer3Start((uint16_t)current_block->steppingFreq[Z_AXIS]);
                            OpenOC3((OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER3_SRC|OC_CONTINUE_PULSE),  (ReadPeriod3()>>1), ReadPeriod3()); // Z_AXIS = Continuous Pulse
                            break;

                        case Z_AXIS:
                            BSP_Timer4Start((uint16_t)current_block->steppingFreq[Z_AXIS]);// Use Timer4 Interrupt to trigger single output pulse
                            OpenOC3((OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER3_SRC|OC_SINGLE_PULSE),  (ReadPeriod3()>>1), ReadPeriod3());    // Z_AXIS = Single Pulse

                            BSP_Timer3Start((uint16_t)current_block->steppingFreq[X_AXIS]);
                            OpenOC2((OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER3_SRC|OC_CONTINUE_PULSE),  (ReadPeriod3()>>1), ReadPeriod3());   // X_AXIS = Continuous Pulse

                            BSP_Timer2Start((uint16_t)current_block->steppingFreq[Y_AXIS]);
                            OpenOC1((OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER2_SRC|OC_CONTINUE_PULSE),  (ReadPeriod2()>>1), ReadPeriod2()); // Y_AXIS = Continuous Pulse
                            break;

                        default:
                            // error
                            break;
                    }
                }
                else
                {
                    BSP_Timer2Start((uint16_t)current_block->steppingFreq[X_AXIS]);     // All Steps Counts are equal so just use on timer
                    OpenOC1((OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER2_SRC|OC_CONTINUE_PULSE),  (ReadPeriod2()>>1), ReadPeriod2()); // Y_AXIS = Continuous Pulse
                    OpenOC2((OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER2_SRC|OC_CONTINUE_PULSE),  (ReadPeriod2()>>1), ReadPeriod2());   // X_AXIS = Continuous Pulse
                    OpenOC3((OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER2_SRC|OC_CONTINUE_PULSE),  (ReadPeriod2()>>1), ReadPeriod2()); // Z_AXIS = Continuous Pulse
                }
                BSP_AxisEnable(Y_AXIS, current_block->direction_bits[Y_AXIS]);
                BSP_AxisEnable(X_AXIS, current_block->direction_bits[X_AXIS]);
                BSP_AxisEnable(Z_AXIS, current_block->direction_bits[Z_AXIS]);
            }
            else if (current_block->activeAxisCount == 2)   // 2 Axis Enabled
            {
                if(current_block->steps_y)
                {
                    BSP_Timer2Start((uint16_t)current_block->steppingFreq[Y_AXIS]);
                    OpenOC1((OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER2_SRC|OC_CONTINUE_PULSE),  (ReadPeriod2()>>1), ReadPeriod2()); // Y_AXIS = Continuous Pulse


                    if(current_block->steps_x)
                    {
                        BSP_Timer3Start((uint16_t)current_block->steppingFreq[X_AXIS]);
                        OpenOC2((OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER3_SRC|OC_CONTINUE_PULSE),  (ReadPeriod3()>>1), ReadPeriod3());   // X_AXIS = Continuous Pulse

                        BSP_AxisEnable(X_AXIS, current_block->direction_bits[X_AXIS]);
                        BSP_AxisEnable(Y_AXIS, current_block->direction_bits[Y_AXIS]);
                    }
                    else
                    {
                        BSP_Timer3Start((uint16_t)current_block->steppingFreq[Z_AXIS]);
                        OpenOC3((OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER3_SRC|OC_CONTINUE_PULSE),  (ReadPeriod3()>>1), ReadPeriod3()); // Z_AXIS = Continuous Pulse

                        BSP_AxisEnable(Z_AXIS, current_block->direction_bits[Z_AXIS]);
                        BSP_AxisEnable(Y_AXIS, current_block->direction_bits[Y_AXIS]);
                    }
                }
                else
                {
                    BSP_Timer2Start((uint16_t)current_block->steppingFreq[X_AXIS]);
                    OpenOC2((OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER2_SRC|OC_CONTINUE_PULSE),  (ReadPeriod2()>>1), ReadPeriod2());   // X_AXIS = Continuous Pulse


                    BSP_Timer3Start((uint16_t)current_block->steppingFreq[Z_AXIS]);
                    OpenOC3((OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER3_SRC|OC_CONTINUE_PULSE),  (ReadPeriod3()>>1), ReadPeriod3()); // Z_AXIS = Continuous Pulse

                    BSP_AxisEnable(Z_AXIS, current_block->direction_bits[Z_AXIS]);
                    BSP_AxisEnable(X_AXIS, current_block->direction_bits[X_AXIS]);
                }
            }
            else        // Only One axis Enabled
            {
                if(current_block->steps_y)
                {
                    BSP_Timer2Start((uint16_t)current_block->steppingFreq[Y_AXIS]);
                    OpenOC1((OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER2_SRC|OC_CONTINUE_PULSE),  (ReadPeriod2()>>1), ReadPeriod2()); // Y_AXIS = Continuous Pulse
                    BSP_AxisEnable(Y_AXIS, current_block->direction_bits[Y_AXIS]);
                }
                else if(current_block->steps_x)
                {
                    BSP_Timer2Start((uint16_t)current_block->steppingFreq[X_AXIS]);
                    OpenOC2((OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER2_SRC|OC_CONTINUE_PULSE),  (ReadPeriod2()>>1), ReadPeriod2());   // X_AXIS = Continuous Pulse

                    BSP_AxisEnable(X_AXIS, current_block->direction_bits[X_AXIS]);
                }
                else
                {
                    BSP_Timer2Start((uint16_t)current_block->steppingFreq[Z_AXIS]);
                    OpenOC3((OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER2_SRC|OC_CONTINUE_PULSE),  (ReadPeriod2()>>1), ReadPeriod2()); // Z_AXIS = Continuous Pulse

                    BSP_AxisEnable(Z_AXIS, current_block->direction_bits[Z_AXIS]);
                }
                //BSP_Timer1Start(0x1);
                milliSecondCount = 0;
            }
        }
    }
    else
    {
        if(milliSecondCount*1000 >= current_block->moveTime)
        {
            current_block = Null;
            plan_discard_current_block();
        }
    }
}




// UART 2 interrupt handler
// it is set at priority level 2 with software context saving
void __ISR(_UART2_VECTOR, ipl2) IntUart2Handler(void)
{
      // We don't care about TX interrupt
  if ( INTGetFlag(INT_SOURCE_UART_RX(MY_UART)))
    {
      INTClearFlag(INT_SOURCE_UART_RX(MY_UART));
      
      uart_rd = UARTGetDataByte(MY_UART);     /*uart_rd = data in the Uart Rx buffer */
      BSP_PutCharacter(uart_rd);
      rx_Fifo_Head++;        /* increase position in rx buffer */ 
      if (rx_Fifo_Head == RX_FIFO_LIMIT) /* check check for max position */ 
          rx_Fifo_Head = 0;             /* wrap back to first position in buffer */
      if (rx_Fifo_Head == rx_Fifo_Tail)
      {
          /* check for FIFO overrun*/
      }
      rx_Fifo[rx_Fifo_Head] = uart_rd; /*place Character in buffer*/
    }



  // We don't care about TX interrupt
  if ( INTGetFlag(INT_SOURCE_UART_TX(MY_UART)) )
    {
      INTClearFlag(INT_SOURCE_UART_TX(MY_UART));
    }
}

void __ISR(_CORE_TIMER_VECTOR, IPL2SOFT) CoreTimerHandler(void)
{
    // clear the interrupt flag
    mCTClearIntFlag();

    // .. things to do
    milliSecondCount++;


    // update the period
    UpdateCoreTimer(CORE_TICKS_PER_MILLISECOND);


}

/*******************************************************************************
 End of File
 */


