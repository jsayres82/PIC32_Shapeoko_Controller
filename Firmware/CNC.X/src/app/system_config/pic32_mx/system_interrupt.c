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
volatile block_t *current_block = Null;  // A pointer to the block currently being traced
uint8_t timer2ActiveAxis;
uint8_t timer3ActiveAxis;
uint8_t timer4ActiveAxis;
uint32_t timerSteps[N_AXIS];
bool blockMoveActive = FALSE;
uint8_t timer2AxisCount;
uint8_t timer3AxisCount;
uint8_t timer4AxisCount;
uint8_t axisCompletedCount;
bool zAxisStep = FALSE;
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
   
    if(current_block->steps[Z_AXIS])
    {
        current_block->steps[Z_AXIS]--;
        zAxisStep = TRUE;
        
    }
   else
   {
        BSP_AxisDisable(Z_AXIS);
        BSP_Timer4Stop();
    }
    mT4ClearIntFlag();              // clear the interrupt flag
}

void __ISR(_TIMER_3_VECTOR, ipl2) _InterruptHandler_TMR3(void)
{

    if(current_block->steps[Y_AXIS])
    {
           current_block->steps[Y_AXIS]--;
    }
    else
    {
        BSP_AxisDisable(Y_AXIS);
        BSP_Timer3Stop();
        axisCompletedCount++;

        if(axisCompletedCount >= current_block->activeAxisCount)
        {
            OpenCoreTimer(100);
            //CloseCoreTimer();
            //plan_discard_current_block();
           // current_block = plan_get_current_block();
           // blockMoveActive = FALSE;
           // OpenCoreTimer(10);
        }
    }
    
     
    mT3ClearIntFlag();    // clear the interrupt flag
}

void __ISR(_TIMER_2_VECTOR, ipl2) _InterruptHandler_TMR2(void)
{
    if(zAxisStep)
    {
        zAxisStep = FALSE;
        OpenOC3((OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER2_SRC|OC_SINGLE_PULSE),  (ReadPeriod2()>>1), ReadPeriod2());
    }

    if(current_block->steps[X_AXIS])
    {
        current_block->steps[X_AXIS]--;
    }
    else if(current_block->steps[Z_AXIS])
    {
        if(current_block->axisTimerOrder[0] == X_AXIS)
        {
            BSP_AxisDisable(X_AXIS);
            axisCompletedCount++;
        }
    }
    else
    {
        BSP_AxisDisable(X_AXIS);
        BSP_Timer2Stop();
        axisCompletedCount++;
        if(axisCompletedCount >= current_block->activeAxisCount)
        {
            OpenCoreTimer(100);
            //CloseCoreTimer();
            //plan_discard_current_block();
            //current_block = plan_get_current_block();
            //blockMoveActive = FALSE;
            //OpenCoreTimer(10);
        }
    }
    
         
     mT2ClearIntFlag();     // clear the interrupt flag
}



//Main Timer for total movement time and block handling
void __ISR(_TIMER_1_VECTOR, ipl2) _InterruptHandler_TMR1(void)
{
    //

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
//      switch (data) {
//    case CMD_STATUS_REPORT: sys.execute |= EXEC_STATUS_REPORT; break; // Set as true
//    case CMD_CYCLE_START:   sys.execute |= EXEC_CYCLE_START; break; // Set as true
//    case CMD_FEED_HOLD:     sys.execute |= EXEC_FEED_HOLD; break; // Set as true
//    case CMD_RESET:         mc_reset(); break; // Call motion control reset routine.
//    default: // Write character to buffer
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
  if ( INTGetFlag(INT_SOURCE_UART_TX(MY_UART)))
    {
      INTClearFlag(INT_SOURCE_UART_TX(MY_UART));
    }
}







void __ISR(_CORE_TIMER_VECTOR, ipl2) CoreTimerHandler(void)
{

    uint8_t i;

    if(current_block != Null)       // If we just finished a block
        plan_discard_current_block();
    current_block = plan_get_current_block();

    if(current_block != Null)   // If there is a new movement to do
    {
        for(i = 0; i < current_block->activeAxisCount; i++)
        {
            switch(current_block->axisTimerOrder[i])
            {
                case X_AXIS:
                    if(current_block->direction_bits[X_AXIS])
                         PORTSetBits(xAxis.directionPin.port, xAxis.directionPin.pin);
                    else
                        PORTClearBits(xAxis.directionPin.port, xAxis.directionPin.pin);

                    PORTClearBits(xAxis.enablePin.port, xAxis.enablePin.pin);

                    WritePeriod2(current_block->timerPeriod[X_AXIS]);
                    OpenOC2((OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER2_SRC|OC_CONTINUE_PULSE),  (ReadPeriod2()>>1), ReadPeriod2());   // X_AXIS = Single Pulse

                    OpenTimer2(current_block->timerConfig, current_block->timerPeriod[X_AXIS]);
                    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
                    mT2IntEnable(1);

                    break;

                case Y_AXIS:
                    if(current_block->direction_bits[Y_AXIS])
                         PORTSetBits(yAxis.directionPin.port, yAxis.directionPin.pin);
                    else
                        PORTClearBits(yAxis.directionPin.port, yAxis.directionPin.pin);

                    PORTClearBits(yAxis.enablePin.port, yAxis.enablePin.pin);
                    
                    WritePeriod3(current_block->timerPeriod[Y_AXIS]);
                    OpenOC1((OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                                |OC_TIMER3_SRC|OC_CONTINUE_PULSE),  (ReadPeriod3()>>1), ReadPeriod3()); // Y_AXIS = Continuous Pulse
                    OpenTimer3(current_block->timerConfig, current_block->timerPeriod[Y_AXIS]);
                    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2);
                    mT3IntEnable(1);
                    break;

                case Z_AXIS:
                    if(current_block->direction_bits[Z_AXIS])
                         PORTSetBits(zAxis.directionPin.port, zAxis.directionPin.pin);
                    else
                        PORTClearBits(zAxis.directionPin.port, zAxis.directionPin.pin);

                    PORTClearBits(zAxis.enablePin.port, zAxis.enablePin.pin);

                    if(!current_block->steps[X_AXIS])                   // If x axis is not active
                    {
                        WritePeriod2(current_block->timerPeriod[Z_AXIS]);
                        OpenTimer2(current_block->timerConfig, current_block->timerPeriod[Z_AXIS]);
                        ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
                        mT2IntEnable(1);
                    }
                    WritePeriod4(current_block->timerPeriod[Z_AXIS]);
                    OpenTimer4(current_block->timerConfig, current_block->timerPeriod[Z_AXIS]);
                    //OpenOC3((OC_ON|OC_IDLE_STOP|OC_TIMER_MODE16 \
                    //        |OC_TIMER2_SRC|OC_SINGLE_PULSE),  (ReadPeriod4()>>1), ReadPeriod4());    // Z_AXIS = Single Pulse

                    ConfigIntTimer4(T4_INT_ON | T4_INT_PRIOR_2);
                    mT4IntEnable(1);


                    break;

                default:
                    break;
            }
          }
        axisCompletedCount = 0;

    }
    else
    {
        CloseCoreTimer();
        // Disable the Timer
    }

    // clear the interrupt flag
    mCTClearIntFlag();

}

/*******************************************************************************
 End of File
 */


