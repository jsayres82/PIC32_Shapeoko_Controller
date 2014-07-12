/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #if defined(__PIC24E__)
            #include <p24Exxxx.h>
    #elif defined (__PIC24F__)||defined (__PIC24FK__)
        #include <p24Fxxxx.h>
    #elif defined(__PIC24H__)
        #include <p24Hxxxx.h>
    #endif

#endif

#include <xc.h>
#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */
#include <stdio.h>
#include <stdlib.h>
#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp              */
#include "lcd.h"
#include "pause.h"
#include "maketime.h"
#include "interrupt.h"
#include "uart.h"
#include "steppercontrol.h"
#include "parser.h"
#include "gcode.h"


/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/


void read_buffer(void);




/******************************************************************************/
/* Main Program                                                               */
int x;
int linePtr = 1;
int charCount = 0;
char buffer_line[LINE_SIZE];
int insCount=0;
char instruction[256];
extern char rx_Fifo[RX_FIFO_LIMIT];
extern int rx_Fifo_Head, rx_Fifo_Tail;
extern char timer_counter;
extern unsigned long int steps_X, steps_Y, steps_Z;
int main(void)
{

    
    /* Configure the oscillator for the device */
    ConfigureOscillator();

    /* Initialize IO ports and peripherals */
    InitApp();
    initialize();

    while(1){

        read_buffer();
         

    }
}

void read_buffer(void){
         // while(rx_Fifo_Head == rx_Fifo_Tail)     /* Wait for an input which will cause*/
           //     continue;                         /* the head pointer to increase one */

       if(rx_Fifo_Head != rx_Fifo_Tail){
         rx_Fifo_Tail++;                           /* Increase the tail to read*/
                                                     /* the next input value*/

          if (rx_Fifo_Tail == RX_FIFO_LIMIT)        /*If we've reached the last */
              rx_Fifo_Tail = 0;                     /*of our buffer, then we go back to the*/
                                                    /*first location*/

          if(((rx_Fifo[rx_Fifo_Tail] == '\n') || (rx_Fifo[rx_Fifo_Tail] == '\r'))){ /* If we come to a Carriage Return then the */
               lcd_clear();                                                         /* line has ended so interpret that line */
               for(x=linePtr; x<rx_Fifo_Tail; x++){
                   if(x == RX_FIFO_LIMIT-1){
                       x = 0;
                   }
                     if(rx_Fifo[x] == '$'){

                    }
                    else if(rx_Fifo[x] == ' '){

                    }
                    else if(rx_Fifo[x] == '%'){

                    }
                    else if((rx_Fifo[x] > 0x61)&&(rx_Fifo[x] <= 0x7A)){
                            buffer_line[charCount] = rx_Fifo[x]-32;
                            //lcd_putch(line[charCount]);
                            charCount++;
                    }
                    else {
                     buffer_line[charCount] =  rx_Fifo[x];
                     //lcd_putch(line[charCount]);
                     charCount++;
                             }

              }
               lcd_clear();
               interpret_Line(buffer_line, charCount);

               charCount = 0;
               linePtr = (rx_Fifo_Tail);
          }
       }
}


