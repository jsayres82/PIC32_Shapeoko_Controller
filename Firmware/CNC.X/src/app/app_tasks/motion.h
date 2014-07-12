/* 
 * File:   motion.h
 * Author: Justin
 *
 * Created on May 3, 2013, 5:06 PM
 */

#ifndef _MOTION_H
#define	_MOTION_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "program_planner.h"
#include <xc.h>
#include <float.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdint.h>
#include "g_code.h"
#include "math.h"
#include "bsp_config.h"
#include "app.h"
#include "nuts_bolts.h"





    
#define MICRON_PER_PULSE 12.7



#define TIMER1_MAX 16383.5          //us
#define T1_PRESCALE8 131068         //us
#define T1_PRESCALE64 1048544       //us
#define T1_PRESCALE256 4194176      //us
#define MAX_MOVEMENT_TIME 21194176  //second for longest movement with feedrate at 250mm/min

void MotionLine(float, float, float, float, uint8_t);
void MotionArc(float*, float*, float*, uint8_t, uint8_t, uint8_t, float, uint8_t, float, uint8_t);
void MotionReset(void);
void seek(float , float , float);

#ifdef	__cplusplus
}
#endif

#endif	/* MOTION_H */

