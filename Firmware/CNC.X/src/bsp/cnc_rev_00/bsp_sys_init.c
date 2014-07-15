/*******************************************************************************
 Board initialization file for PIC32 USB starter kit

 Company:
    Microchip Technology Inc.

 File Name:
    bsp_sys_init.c

 Summary:
    Board initialization file for CNC control board Rev 00

 Description:
    This file contains the initialization of board specific I/O.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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
#include <xc.h>
#include <plib.h>
#include "bsp_config.h"


unsigned int temp;
volatile uint32_t milliSecondCount;


volatile AxisPins_t  xAxis = {
    .directionPin.port = IOPORT_G,
    .directionPin.pin  = BIT_12,
    .enablePin.port    = IOPORT_G,
    .enablePin.pin     = BIT_14,
    .homePin.port      = IOPORT_G,
    .homePin.pin       = BIT_6,
    .mS1Pin.port       = IOPORT_G,
    .mS1Pin.pin        = BIT_13,
    .mS2Pin.port       = IOPORT_G,
    .mS2Pin.pin        = BIT_15,
    .stepPWM           = OC2
};

volatile AxisPins_t  yAxis = {
    .directionPin.port = IOPORT_E,
    .directionPin.pin  = BIT_0,
    .enablePin.port    = IOPORT_E,
    .enablePin.pin     = BIT_1,
    .homePin.port      = IOPORT_E,
    .homePin.pin       = BIT_5,
    .mS1Pin.port       = IOPORT_E,
    .mS1Pin.pin        = BIT_2,
    .mS2Pin.port       = IOPORT_E,
    .mS2Pin.pin        = BIT_3,
    .stepPWM           = OC1
};

volatile AxisPins_t  zAxis = {
    .directionPin.port = IOPORT_A,
    .directionPin.pin  = BIT_1,
    .enablePin.port    = IOPORT_A,
    .enablePin.pin     = BIT_6,
    .homePin.port      = IOPORT_G,
    .homePin.pin       = BIT_7,
    .mS1Pin.port       = IOPORT_A,
    .mS1Pin.pin        = BIT_7,
    .mS2Pin.port       = IOPORT_A,
    .mS2Pin.pin        = BIT_2,
    .stepPWM           = OC3
};


pinconfig_t zLimitInt = {
    .port               = IOPORT_E,
    .pin                = BIT_8
};
pinconfig_t xyLimitInt = {
    .port               = IOPORT_E,
    .pin                = BIT_9
};
pinconfig_t xLimitInput = {
    .port               = IOPORT_C,
    .pin                = BIT_3
};
pinconfig_t yLimitInput = {
    .port               = IOPORT_C,
    .pin                = BIT_2
};


pinconfig_t spindleEna =
{
    .port              = IOPORT_D,
    .pin               = BIT_13
};

pinconfig_t spindleVcc  =
{
    .port              = IOPORT_D,
    .pin               = BIT_5
};

pinconfig_t spindleTach  =
{
    .port              = IOPORT_D,
    .pin               = BIT_10
};

pinconfig_t levelShifterEnable  =
{
    .port              = IOPORT_E,
    .pin               = BIT_4
};
ocModule_e spindleSpdPWM = OC4;





// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function: void BSP_Initialize(void)

  Summary:
    Performs the neccassary actions to initialize a board

  Description:
    This routine performs the neccassary actions to initialize a board

  Remarks:
    This routine performs direct register accesses, when the PORTS PLIB and
    system service become available, these register accesses will be be replaced
    by the PLIB\system service interfaces.

*/

void BSP_Initialize(void )
{
    //Timer Intitialization, Timers OFF, Period set to 1 milliSecond
    OpenTimer1(T1_OFF | T1_SOURCE_INT | T1_PS_1_1, 1);
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);
     //Timer Intitialization, Timers OFF, Period set to 1 milliSecond
    OpenTimer2(T2_OFF | T2_SOURCE_INT | T2_PS_1_256, T2_TICK);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    
    OpenTimer3(T3_OFF | T3_SOURCE_INT | T3_PS_1_256, T3_TICK);
    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2);

    OpenTimer4(T4_OFF | T4_SOURCE_INT | T4_PS_1_256, T4_TICK);
    ConfigIntTimer4(T4_INT_ON | T4_INT_PRIOR_2);


    // Configure Y Axis Output Pins
    // mPORTEOpenDrainOpen( BIT_0|BIT_1|BIT_2|BIT_3);
    PORTSetPinsDigitalOut(IOPORT_E, BIT_0|BIT_1|BIT_2|BIT_3);
    mPORTESetBits( BIT_0|BIT_1|BIT_2|BIT_3);
    //Configure Y Axis Input Pins
    PORTSetPinsDigitalIn(yAxis.homePin.port, yAxis.homePin.pin);
    //Configure Y Axis PWM
    OpenOC1(YZ_PWM_INIT,0,0);
    
    // Configure X Axis Output Pins
    // mPORTGOpenDrainOpen(BIT_12|BIT_13|BIT_14|BIT_15);
    PORTSetPinsDigitalOut(IOPORT_G,BIT_12|BIT_13|BIT_14|BIT_15);
    mPORTGSetBits(BIT_12|BIT_13|BIT_14|BIT_15);
    //Configure X Axis Input Pins
    PORTSetPinsDigitalIn(xAxis.homePin.port, xAxis.homePin.pin);
    //Configure X Axis and Spindle PWM
    OpenOC2(XS_PWM_INIT,0,0);

    // Configure Z Axis Output Pins
    // mPORTAOpenDrainOpen(BIT_1 |BIT_2|BIT_6|BIT_7);
    PORTSetPinsDigitalOut(IOPORT_A,BIT_1 |BIT_2|BIT_6|BIT_7);
    mPORTASetBits(BIT_1 |BIT_2|BIT_6|BIT_7);
    //Configure Z Axis Input Pins
    PORTSetPinsDigitalIn(zAxis.homePin.port, zAxis.homePin.pin);
    //Configure Z Axis PWM
    OpenOC3(YZ_PWM_INIT,0,0);

    // Configure Spindle Output Pins
    mPORTDOpenDrainOpen(spindleEna.pin|spindleVcc.pin);
    PORTSetPinsDigitalOut(IOPORT_D, spindleEna.pin|spindleVcc.pin);
    mPORTDClearBits(spindleEna.pin|spindleVcc.pin);
    //Configure Spindle PWM
    OpenOC4(XS_PWM_INIT,0,0);

    // Configure X and Y Limit Input Pins
    PORTSetPinsDigitalIn(xLimitInput.port, xLimitInput.pin);
    PORTSetPinsDigitalIn(yLimitInput.port, yLimitInput.pin);

    temp = mPORTCRead();


    // This initialization assumes 36MHz Fpb clock. If it changes,
    UARTConfigure(MY_UART, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(MY_UART, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(MY_UART, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(MY_UART, GetPeripheralClock(), DESIRED_BAUDRATE);
    UARTEnable(MY_UART, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
    // Configure UART2 RX Interrupt
    INTEnable(INT_SOURCE_UART_RX(MY_UART), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(MY_UART), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(MY_UART), INT_SUB_PRIORITY_LEVEL_0);
    

    // Configure XY and Z Limit Input Pins as inputs
    PORTSetPinsDigitalIn(IOPORT_E, xyLimitInt.pin| zLimitInt.pin);
    // Configure interrupts
    ConfigINT1(EXT_INT_PRI_1 | RISING_EDGE_INT | EXT_INT_ENABLE);
    ConfigINT2(EXT_INT_PRI_1 | RISING_EDGE_INT | EXT_INT_ENABLE);

    // The Core timer should halt when we are halted at a debug breakpoint.
    _CP0_BIC_DEBUG(_CP0_DEBUG_COUNTDM_MASK);

    // set up the core timer interrupt with a prioirty of 2 and zero sub-priority
    mConfigIntCoreTimer((CT_INT_ON | CT_INT_PRIOR_2));


}


void BSP_MoveAxis(uint8_t axis, uint8_t dir, uint16_t period)
{
    switch(axis)
    {
        case X_AXIS:

            BSP_Timer3Start(period);
            OpenOC2(XS_PWM_ENA,ReadPeriod3()>>1, ReadPeriod3()>>1);

            if(dir == POSITIVE)
            {
                PORTSetBits(xAxis.directionPin.port, xAxis.directionPin.pin);
            }
            else
            {
                PORTClearBits(xAxis.directionPin.port, xAxis.directionPin.pin);
            }

            PORTClearBits(xAxis.enablePin.port, xAxis.enablePin.pin);
            
            break;

        case Y_AXIS:

            BSP_Timer2Start(period);
            OpenOC1(YZ_PWM_ENA, (ReadPeriod2()>>1), ReadPeriod2()>>1);

            if(dir == POSITIVE)
            {
                PORTSetBits(yAxis.directionPin.port, yAxis.directionPin.pin);
            }
            else
            {
                PORTClearBits(yAxis.directionPin.port, yAxis.directionPin.pin);
            }

            PORTClearBits(yAxis.enablePin.port, yAxis.enablePin.pin);
            
            break;

        case Z_AXIS:

            BSP_Timer2Start(period);
            OpenOC1(YZ_PWM_ENA, period, (period>>1));

            if(dir == POSITIVE)
                PORTSetBits(zAxis.directionPin.port, zAxis.directionPin.pin);
            else
                PORTClearBits(zAxis.directionPin.port, zAxis.directionPin.pin);

            PORTClearBits(zAxis.enablePin.port, zAxis.enablePin.pin);
            break;

        default:
            break;
    }

}
void BSP_Timer1Start(uint16_t frequency)
{
    uint32_t ticks;
    T1CON = 0x00;
    mT1ClearIntFlag();
    ticks = GetPeripheralClock()/frequency;
    if(ticks <= 65536)
    {
        OpenTimer1((T1_ON|T1_PS_1_1),(uint16_t)ticks);
    }
    else if(((ticks >> 3)<= 65536))
    {
        OpenTimer1((T1_ON| T1_SOURCE_INT|T1_PS_1_8),T1_PS_1_8*(uint16_t)ticks);
    }
    else if(((ticks >> 6)<= 65536))
    {
        OpenTimer1((T1_ON| T1_SOURCE_INT|T1_PS_1_64),T1_PS_1_64*(uint16_t)ticks);
    }
    else if(((ticks >> 7)<= 65536))
    {
        OpenTimer1((T1_ON| T1_SOURCE_INT|T1_PS_1_256),T1_PS_1_256*(uint16_t)ticks);
    }
    else
    {
        //error
    }
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);
    mT1IntEnable(1);
}
void BSP_Timer2Start(uint16_t frequency)
{
    uint32_t ticks;
    mT2ClearIntFlag();
    ticks = (GetPeripheralClock()/frequency);
    if(ticks <= 65536)
    {
        OpenTimer2((T2_ON| T2_SOURCE_INT|T2_PS_1_1),(uint16_t)ticks);
    }
    else if((ticks >> 1)<= 65536)
    {
        OpenTimer2((T2_ON| T2_SOURCE_INT|T2_PS_1_2),T2_PS_1_2*(uint16_t)ticks);
    }
    else if(((ticks >> 2)<= 65536))
    {
        OpenTimer2((T2_ON| T2_SOURCE_INT|T2_PS_1_4),T2_PS_1_4*(uint16_t)ticks);
    }
    else if(((ticks >> 3)<= 65536))
    {
        OpenTimer2((T2_ON| T2_SOURCE_INT|T2_PS_1_8),T2_PS_1_8*(uint16_t)ticks);
    }
    else if(((ticks >> 4)<= 65536))
    {
        OpenTimer2((T2_ON| T2_SOURCE_INT|T2_PS_1_16),T2_PS_1_16*(uint16_t)ticks);
    }
    else if(((ticks >> 5)<= 65536))
    {
        OpenTimer2((T2_ON| T2_SOURCE_INT|T2_PS_1_32),T2_PS_1_32*(uint16_t)ticks);
    }
    else if(((ticks >> 6)<= 65536))
    {
        OpenTimer2((T2_ON| T2_SOURCE_INT|T2_PS_1_64),T2_PS_1_64*(uint16_t)ticks);
    }
    else if(((ticks >> 7)<= 65536))
    {
        OpenTimer2((T2_ON| T2_SOURCE_INT|T2_PS_1_256),T2_PS_1_256*(uint16_t)ticks);
    }
    else
    {
        //error
    }
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2IntEnable(1);
}

void BSP_Timer3Start(uint16_t frequency)
{
    uint32_t ticks;
    mT3ClearIntFlag();
    ticks = (GetPeripheralClock()/frequency);
    if(ticks <= 65536)
    {
        OpenTimer3((T3_ON| T3_SOURCE_INT|T3_PS_1_1),(uint16_t)ticks);
    }
    else if((ticks >> 1)<= 65536)
    {
        OpenTimer3((T3_ON| T3_SOURCE_INT|T3_PS_1_2),T3_PS_1_2*(uint16_t)ticks);
    }
    else if(((ticks >> 2)<= 65536))
    {
        OpenTimer3((T3_ON| T3_SOURCE_INT|T3_PS_1_4),T3_PS_1_4*(uint16_t)ticks);
    }
    else if(((ticks >> 3)<= 65536))
    {
        OpenTimer3((T3_ON| T3_SOURCE_INT|T3_PS_1_8),T3_PS_1_8*(uint16_t)ticks);
    }
    else if(((ticks >> 4)<= 65536))
    {
        OpenTimer3((T3_ON| T3_SOURCE_INT|T3_PS_1_16),T3_PS_1_16*(uint16_t)ticks);
    }
    else if(((ticks >> 5)<= 65536))
    {
        OpenTimer3((T3_ON| T3_SOURCE_INT|T3_PS_1_32),T3_PS_1_32*(uint16_t)ticks);
    }
    else if(((ticks >> 6)<= 65536))
    {
        OpenTimer3((T3_ON| T3_SOURCE_INT|T3_PS_1_64),T3_PS_1_64*(uint16_t)ticks);
    }
    else if(((ticks >> 7)<= 65536))
    {
        OpenTimer3((T3_ON| T3_SOURCE_INT|T3_PS_1_256),T3_PS_1_256*(uint16_t)ticks);
    }
    else
    {
        //error
    }
    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2);
    mT3IntEnable(1);
}
void BSP_Timer4Start(int16_t frequency)
{
    uint32_t ticks;
    mT4ClearIntFlag();
    ticks = (GetPeripheralClock()/frequency);
    if(ticks <= 65536)
    {
        OpenTimer4((T4_ON| T4_SOURCE_INT|T4_PS_1_1),(uint16_t)ticks);
    }
    else if((ticks >> 1)<= 65536)
    {
        OpenTimer4((T4_ON| T4_SOURCE_INT|T4_PS_1_2),T4_PS_1_2*(uint16_t)ticks);
    }
    else if(((ticks >> 2)<= 65536))
    {
        OpenTimer4((T4_ON| T4_SOURCE_INT|T4_PS_1_4),T4_PS_1_4*(uint16_t)ticks);
    }
    else if(((ticks >> 3)<= 65536))
    {
        OpenTimer4((T4_ON| T4_SOURCE_INT|T4_PS_1_8),T4_PS_1_8*(uint16_t)ticks);
    }
    else if(((ticks >> 4)<= 65536))
    {
        OpenTimer4((T4_ON| T4_SOURCE_INT|T4_PS_1_16),T4_PS_1_16*(uint16_t)ticks);
    }
    else if(((ticks >> 5)<= 65536))
    {
        OpenTimer4((T4_ON| T4_SOURCE_INT|T4_PS_1_32),T4_PS_1_32*(uint16_t)ticks);
    }
    else if(((ticks >> 6)<= 65536))
    {
        OpenTimer4((T4_ON| T4_SOURCE_INT|T4_PS_1_64),T4_PS_1_64*(uint16_t)ticks);
    }
    else if(((ticks >> 7)<= 65536))
    {
        OpenTimer4((T4_ON| T4_SOURCE_INT|T4_PS_1_256),T4_PS_1_256*(uint16_t)ticks);
    }
    else
    {
        //error
    }
    ConfigIntTimer4(T4_INT_ON | T4_INT_PRIOR_2);
    mT4IntEnable(1);
}


void BSP_Timer1Stop(void)
{
    CloseTimer1();
}

void BSP_Timer2Stop(void)
{
    CloseTimer2();
}
void BSP_Timer3Stop(void)
{
    CloseTimer3();
}
void BSP_Timer4Stop(void)
{
    CloseTimer4();
}

void BSP_PWM_Enable(ocModule_e module, uint16_t dutyCycle)
{
    int16_t period;

    switch(module)
    {
        case OC1:
            period = ReadTimer2();
            OpenOC1(YZ_PWM_ENA, period, dutyCycle);
            break;
        case OC2:
            period = ReadTimer3();
            OpenOC2(XS_PWM_ENA, period, dutyCycle);
            break;
        case OC3:
            period = ReadTimer2();
            OpenOC3(YZ_PWM_ENA, period, dutyCycle);
            break;
        case OC4:
            period = ReadTimer3();
            OpenOC4(XS_PWM_ENA, period, dutyCycle);
            break;
        case OC5:
           // SetPulseOC5
            break;
        default:
            break;
    }

}

void BSP_PWM_Disable(ocModule_e module)
{
    switch(module)
    {
        case OC1:
            CloseOC1();
            break;
        case OC2:
            CloseOC2();
            break;
        case OC3:
            CloseOC3();
            break;
        case OC4:
            CloseOC4();
            break;
        case OC5:
           // SetPulseOC5
            break;
        default:
            break;
    }

}


void BSP_AxisEnable(uint8_t axis, uint8_t dir)
{

    switch(axis)
    {
        case X_AXIS:
            if(dir)
                PORTSetBits(xAxis.directionPin.port, xAxis.directionPin.pin);
            else
                PORTClearBits(xAxis.directionPin.port, xAxis.directionPin.pin);
            PORTClearBits(xAxis.enablePin.port, xAxis.enablePin.pin);
            break;
        case Y_AXIS:
            if(dir)
                PORTSetBits(yAxis.directionPin.port, yAxis.directionPin.pin);
            else
            PORTClearBits(yAxis.directionPin.port, yAxis.directionPin.pin);
            PORTClearBits(yAxis.enablePin.port, yAxis.enablePin.pin);
            break;
        case Z_AXIS:
            if(dir)
                PORTSetBits(zAxis.directionPin.port, zAxis.directionPin.pin);
            else
                PORTClearBits(zAxis.directionPin.port, zAxis.directionPin.pin);
            PORTClearBits(zAxis.enablePin.port, zAxis.enablePin.pin);
            break;
        default:
            break;
    }
}

void BSP_AxisDisable(uint8_t axis)
{

    switch(axis)
    {
        case X_AXIS:
            PORTSetBits(xAxis.enablePin.port, xAxis.enablePin.pin);
            break;
        case Y_AXIS:
            PORTSetBits(yAxis.enablePin.port, yAxis.enablePin.pin);
            break;
        case Z_AXIS:
           PORTSetBits(zAxis.enablePin.port, zAxis.enablePin.pin);
            break;
        default:
            break;
    }
}


void BSP_SetStepSize(uint8_t axis, StepSize_e stepSize)
{
    switch(axis)
    {

        case X_AXIS:
            switch(stepSize)
            {
                case 0:
                    PORTClearBits(xAxis.mS1Pin.port, xAxis.mS1Pin.pin);
                    PORTClearBits(xAxis.mS2Pin.port, xAxis.mS2Pin.pin);
                    break;
                case 1:
                    PORTClearBits(xAxis.mS1Pin.port, xAxis.mS1Pin.pin);
                    PORTSetBits(xAxis.mS2Pin.port, xAxis.mS2Pin.pin);
                    break;
                case 2:
                    PORTSetBits(xAxis.mS1Pin.port, xAxis.mS1Pin.pin);
                    PORTClearBits(xAxis.mS2Pin.port, xAxis.mS2Pin.pin);
                    break;
                case 3:
                    PORTSetBits(xAxis.mS1Pin.port, xAxis.mS1Pin.pin);
                    PORTSetBits(xAxis.mS2Pin.port, xAxis.mS2Pin.pin);
                    break;
                default:
                    break;
            }
            break;
        case Y_AXIS:
            switch(stepSize)
            {
                case 0:
                    PORTClearBits(yAxis.mS1Pin.port, yAxis.mS1Pin.pin);
                    PORTClearBits(yAxis.mS2Pin.port, yAxis.mS2Pin.pin);
                    break;
                case 1:
                    PORTClearBits(yAxis.mS1Pin.port, yAxis.mS1Pin.pin);
                    PORTSetBits(yAxis.mS2Pin.port, yAxis.mS2Pin.pin);
                    break;
                case 2:
                    PORTSetBits(yAxis.mS1Pin.port, yAxis.mS1Pin.pin);
                    PORTClearBits(yAxis.mS2Pin.port, yAxis.mS2Pin.pin);
                    break;
                case 3:
                    PORTSetBits(yAxis.mS1Pin.port, yAxis.mS1Pin.pin);
                    PORTSetBits(yAxis.mS2Pin.port, yAxis.mS2Pin.pin);
                    break;
                default:
                    break;
            }
            break;
        case Z_AXIS:
            switch(stepSize)
            {
                case 0:
                    PORTClearBits(zAxis.mS1Pin.port, zAxis.mS1Pin.pin);
                    PORTClearBits(zAxis.mS2Pin.port, zAxis.mS2Pin.pin);
                    break;
                case 1:
                    PORTClearBits(zAxis.mS1Pin.port, zAxis.mS1Pin.pin);
                    PORTSetBits(zAxis.mS2Pin.port, zAxis.mS2Pin.pin);
                    break;
                case 2:
                    PORTSetBits(zAxis.mS1Pin.port, zAxis.mS1Pin.pin);
                    PORTClearBits(zAxis.mS2Pin.port, zAxis.mS2Pin.pin);
                    break;
                case 3:
                    PORTSetBits(zAxis.mS1Pin.port, zAxis.mS1Pin.pin);
                    PORTSetBits(zAxis.mS2Pin.port, zAxis.mS2Pin.pin);
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

int BSP_GetLimitSwitch(void)
{
    return((PORTReadBits(xLimitInput.port, xLimitInput.pin)<<1) | \
                          (PORTReadBits(yLimitInput.port, yLimitInput.pin)));
}

char BSP_GetUartChar()
{
    UARTGetDataByte(MY_UART);
}



// helper functions
void BSP_WriteString(const char *string)
{
  while (*string != '\0')
    {
      while (!UARTTransmitterIsReady(MY_UART))
        ;

      UARTSendDataByte(MY_UART, *string);

      string++;

      while (!UARTTransmissionHasCompleted(MY_UART))
        ;
    }
}
void BSP_PutCharacter(const char character)
{
  while (!UARTTransmitterIsReady(MY_UART))
    ;

  UARTSendDataByte(MY_UART, character);

  while (!UARTTransmissionHasCompleted(MY_UART))
    ;
}

void delay_ms(uint32_t period)
{
    milliSecondCount = 0;

    while(period - milliSecondCount);
}


//void BSP_SwitchOFFLED(BSP_LED led)
//{

    /* switch OFF the LED */
  //  PLIB_PORTS_PinWrite ( PORTS_ID_0 ,
    //                     PORT_CHANNEL_A ,
    //                     led,
     ///                    0 );
//}

//void BSP_ToggleLED(BSP_LED led)
//{

   // PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_A,led );
//}



///BSP_SWITCH_STATE BSP_ReadSwitch( BSP_SWITCH bspSwitch )
///{
 //   return ( PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_D, bspSwitch) );
//}

//uint32_t BSP_ReadCoreTimer()
//{
//    uint32_t timer;
//
//    // get the count reg
//    asm volatile("mfc0   %0, $9" : "=r"(timer));
//
//    return(timer);
//}
//
//void BSP_StartTimer(uint32_t period)
//{
//    /* Reset the coutner */
//
//    uint32_t loadZero = 0;
//
//    asm volatile("mtc0   %0, $9" : "+r"(loadZero));
//    asm volatile("mtc0   %0, $11" : "+r" (period));
//
//}

/******************************************************************************/
/******************************************************************************/

/*******************************************************************************
 End of File
*/


