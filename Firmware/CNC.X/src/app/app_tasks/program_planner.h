/*
 * File:   program_manager.h
 * Author: Justin
 *
 * Created on May 6, 2013, 7:25 PM
 */

#ifndef _PROGRAM_PLANNER_H
#define	_PROGRAM_PLANNER_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "nuts_bolts.h"

// The number of linear motions that can be in the plan at any give time
#ifndef BLOCK_BUFFER_SIZE
  #define BLOCK_BUFFER_SIZE 18
#endif

// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in
// the source g-code and may never actually be reached if acceleration management is active.
typedef struct {

  // Fields used by the bresenham algorithm for tracing the line
  uint8_t direction_bits[N_AXIS];            // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
  uint32_t steppingFreq[N_AXIS];
  float maxTimerFrequency;
  uint32_t steps[N_AXIS];                    // Step count along each axis
  int32_t  step_event_count;                 // The number of step events required to complete this block
  uint8_t activeAxisCount;                   // The number of axis that are required to move for the block
  uint8_t minStepAxis;                       // Used to determine the which axis have the highest step count and will use Timer2 and Timer3
                                            // to create a continous stream of pulses on their OCx pin.  If three axis are enabled
                                            // the third axis will use Timer4 to create single output pulses on it OCx Pin.
  uint8_t axisTimerOrder[N_AXIS];
  uint16_t timerConfig[N_AXIS];
  uint16_t timerPeriod[N_AXIS];
  uint8_t numberOfTimers;
  // Fields used by the motion planner to manage acceleration
  float nominal_speed;               // The nominal speed for this block in mm/min
  float entry_speed;                 // Entry speed at previous-current block junction in mm/min
  float max_entry_speed;             // Maximum allowable junction entry speed in mm/min
  float millimeters;                 // The total travel of this block in mm
  uint8_t recalculate_flag;           // Planner flag to recalculate trapezoids on entry junction
  uint8_t nominal_length_flag;        // Planner flag for nominal speed always reached

  // Settings for the trapezoid generator
  uint32_t initial_rate;              // The step rate at start of block
  uint32_t final_rate;                // The step rate at end of block
  int32_t rate_delta;                 // The steps/minute to add or subtract when changing speed (must be positive)
  uint32_t accelerate_until;          // The index of the step event on which to stop acceleration
  uint32_t decelerate_after;          // The index of the step event on which to start decelerating
  uint32_t nominal_rate;              // The nominal step rate for this block in step_events/minute
  uint32_t coreTimerTicks;
  float moveTime;
} block_t;

// Initialize the motion plan subsystem
void plan_init();

// Add a new linear movement to the buffer. x, y and z is the signed, absolute target position in
// millimaters. Feed rate specifies the speed of the motion. If feed rate is inverted, the feed
// rate is taken to mean "frequency" and would complete the operation in 1/feed_rate minutes.
void plan_buffer_line(float x, float y, float z, float feed_rate, uint8_t invert_feed_rate);

// Called when the current block is no longer needed. Discards the block and makes the memory
// availible for new blocks.
void plan_discard_current_block();

// Gets the current block. Returns NULL if buffer empty
block_t *plan_get_current_block();

// Reset the planner position vector (in steps)
void plan_set_current_position(int32_t x, int32_t y, int32_t z);

// Reinitialize plan with a partially completed block
void plan_cycle_reinitialize(int32_t step_events_remaining);

// Reset buffer
void plan_reset_buffer();

// Returns the status of the block ring buffer. True, if buffer is full.
uint8_t plan_check_full_buffer();

// Block until all buffered steps are executed
void plan_synchronize();

#ifdef	__cplusplus
}
#endif

#endif	/* PROGRAM_MANAGER_H */

