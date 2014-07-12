/* 
 * File:   defaults.h
 * Author: Justin
 *
 * Created on June 23, 2014, 11:38 PM
 */

#ifndef DEFAULTS_H
#define	DEFAULTS_H

#ifdef	__cplusplus
extern "C" {
#endif
#include "g_code.h"
#include "report.h"
//#include "stepper.h"
#include "nuts_bolts.h"
#include "settings.h"
//#include "eeprom.h"
#include "limits.h"
//#include "bsp_config.h"

/* Defaults */
#define FULL_STEP 1
#define HALF_STEP 2
#define QUARTER_STEP 4
#define EIGHT_STEP 8
#define SIXTEENTH_STEP 16


#define Z_MOTOR_RES 400
#define LEAD_SCREW_RES 1.250  //mm

#define X_MOTOR_RES 200 //200 steps per revolution (1.8 degree)
#define Y_MOTOR_RES 200 //200 steps per revolution (1.8 degree)


#define X_BELT_PITCH 2.032  //mm
#define Y_BELT_PITCH 2.032  //mm


#define X_PULLEY_TEETH 20
#define Y_PULLEY_TEETH 20



#define X_STEPS_MM(step_size)  (X_MOTOR_RES*step_size/(X_PULLEY_TEETH*X_BELT_PITCH))
#define Y_STEPS_MM(step_size)  (Y_MOTOR_RES*step_size/(X_PULLEY_TEETH*X_BELT_PITCH))
#define Z_STEPS_MM(step_size)  (Z_MOTOR_RES*step_size/LEAD_SCREW_RES)

#define HOME_X_POSITION 0
#define HOME_Y_POSITION 0
#define HOME_Z_POSITION 0


#define POSITIVE 1
#define NEGATIVE 0

#define MAX_X_TRAVEL 215 // mm
#define MAX_Y_TRAVEL 215 // mm
#define MAX_TRAVEL 300 // mm

#define LINEAR_FEEDRATE 4 //mm per sec
#define SEEK_FEEDRATE 15  // mm per sec

  // Description: Shapeoko CNC mill with three NEMA 17 stepper motors, driven by Synthetos
  // grblShield with a 24V, 4.2A power supply.
  #define MICROSTEPS_XY 4
  #define STEP_REVS_XY 200
  #define MM_PER_REV_XY (20*2.032) // 0.08 in belt pitch, 18 pulley teeth
  #define MICROSTEPS_Z 2
  #define STEP_REVS_Z 400
  #define MM_PER_REV_Z 1.250 // 1.25 mm/rev leadscrew
  #define DEFAULT_X_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Y_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Z_STEPS_PER_MM (MICROSTEPS_Z*STEP_REVS_Z/MM_PER_REV_Z)
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_MM_PER_ARC_SEGMENT 0.1
  #define DEFAULT_RAPID_FEEDRATE 1000.0 // mm/min
  #define DEFAULT_FEEDRATE 250.0
  #define DEFAULT_ACCELERATION (15.0*60*60) // 15*60*60 mm/min^2 = 15 mm/s^2
  #define DEFAULT_JUNCTION_DEVIATION 0.05 // mm
  #define DEFAULT_STEPPING_INVERT_MASK ((1<<0|1<<1))
  #define DEFAULT_REPORT_INCHES     0 // false
  #define DEFAULT_AUTO_START    1 // true
  #define DEFAULT_INVERT_ST_ENABLE  0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE     0  // false
  #define DEFAULT_HOMING_ENABLE         0  // false
  #define DEFAULT_HOMING_DIR_MASK       0 // move negative dir
  #define DEFAULT_HOMING_RAPID_FEEDRATE 250.0 // mm/min
  #define DEFAULT_HOMING_FEEDRATE 25.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 100 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 255 // msec (0-255)
  #define DEFAULT_DECIMAL_PLACES 3
  #define DEFAULT_N_ARC_CORRECTION 25




#ifdef	__cplusplus
}
#endif

#endif	/* DEFAULTS_H */

