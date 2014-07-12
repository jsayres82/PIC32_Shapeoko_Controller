/* 
 * File:   stepper.h
 * Author: Justin
 *
 * Created on June 24, 2014, 11:13 PM
 */

#ifndef STEPPER_H
#define	STEPPER_H

#ifdef	__cplusplus
extern "C" {
#endif



#include <xc.h>
#include <plib.h>

// Initialize and setup the stepper motor subsystem
void st_init();

// Enable steppers, but cycle does not start unless called by motion control or runtime command.
void st_wake_up();

// Immediately disables steppers
void st_go_idle();

// Reset the stepper subsystem variables
void st_reset();

// Notify the stepper subsystem to start executing the g-code program in buffer.
void st_cycle_start();

// Reinitializes the buffer after a feed hold for a resume.
void st_cycle_reinitialize();

// Initiates a feed hold of the running program
void st_feed_hold();



#ifdef	__cplusplus
}
#endif

#endif	/* STEPPER_H */

