/*
 * File:   motion.c
 * Author: Justin Sayres
 *
 * Created on 01/08/13
 */

#include "motion.h"
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include "settings.h"
#include "program_planner.h"
#include "program_planner.h"
#include "defaults.h"


 void MotionLine(float X, float Y, float Z, float feed_rate, uint8_t invert_feed_rate)
 {
    plan_buffer_line(X,Y,Z,250, invert_feed_rate);
//    float  target[3];
//    uint8_t xDirection = POSITIVE;
//    uint8_t yDirection = POSITIVE;
//    uint8_t zDirection = POSITIVE;
//
//
//    target[X_AXIS] = lround((X)*settings.steps_per_mm[X_AXIS]);//*50800);  //steps per mm
//    target[Y_AXIS] = lround((Y)*settings.steps_per_mm[Y_AXIS]);//*50800);  //steps per mm
//    target[Z_AXIS] = lround((Z)*settings.steps_per_mm[Z_AXIS]);//*65024);  //steps per mm
//
//    // Number of steps for each axis
//    steps_X = (uint32_t)(fabs(target[X_AXIS]-gcode.position[X_AXIS]));
//    steps_Y = (uint32_t)(fabs(target[Y_AXIS]-gcode.position[Y_AXIS]));
//    steps_Z = (uint32_t)(fabs(target[Z_AXIS]-gcode.position[Z_AXIS]));
//
//    // Compute direction bits
//    if (target[X_AXIS] < gcode.position[X_AXIS])
//        xDirection = NEGATIVE;
//    if (target[Y_AXIS] < gcode.position[Y_AXIS])
//        yDirection = NEGATIVE;
//    if (target[Z_AXIS] < gcode.position[Z_AXIS])
//        zDirection = NEGATIVE;
    sys.state = STATE_QUEUED;
    mEnableIntCoreTimer();
    //Set the core timer to interrupt 25 nanoSeconds
    OpenCoreTimer(1000);          // Trigger the core timer to get a block

    

    
   // BSP_MoveAxis(X_AXIS, xDirection, ReadPeriod3()>>1);
   // BSP_MoveAxis(Y_AXIS, yDirection, ReadPeriod2()>>1);
    //BSP_MoveAxis(Z_AXIS, zDirection, ReadPeriod3());
  // Bail if this is a zero-length task
  //if (step_event_count == 0) { return; };

 }


// Execute an arc in offset mode format. position == current xyz, target == target xyz,
// offset == offset from current xyz, axis_XXX defines circle plane in tool space, axis_linear is
// the direction of helical travel, radius == circle radius, isclockwise boolean. Used
// for vector transformation direction.
// The arc is approximated by generating a huge number of tiny, linear segments. The length of each
// segment is configured in settings.mm_per_arc_segment.
void MotionArc(float *position, float *target, float *offset, uint8_t axis_0, uint8_t axis_1,
  uint8_t axis_linear, float feed_rate, uint8_t invert_feed_rate, float radius, uint8_t isclockwise)
{
  float center_axis0 = position[axis_0] + offset[axis_0];
  float center_axis1 = position[axis_1] + offset[axis_1];
  float linear_travel = target[axis_linear] - position[axis_linear];
  float r_axis0 = -offset[axis_0];  // Radius vector from center to current location
  float r_axis1 = -offset[axis_1];
  float rt_axis0 = target[axis_0] - center_axis0;
  float rt_axis1 = target[axis_1] - center_axis1;

  // CCW angle between position and target from circle center. Only one atan2() trig computation required.
  float angular_travel = atan2(r_axis0*rt_axis1-r_axis1*rt_axis0, r_axis0*rt_axis0+r_axis1*rt_axis1);
  if (isclockwise) { // Correct atan2 output per direction
    if (angular_travel >= 0) { angular_travel -= 2*M_PI; }
  } else {
    if (angular_travel <= 0) { angular_travel += 2*M_PI; }
  }

  float millimeters_of_travel = hypot(angular_travel*radius, fabs(linear_travel));
  if (millimeters_of_travel == 0.0) { return; }
  uint16_t segments = floor(millimeters_of_travel/settings.mm_per_arc_segment);
  // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
  // by a number of discrete segments. The inverse feed_rate should be correct for the sum of
  // all segments.
  if (invert_feed_rate) { feed_rate *= segments; }

  float theta_per_segment = angular_travel/segments;
  float linear_per_segment = linear_travel/segments;

  /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
     and phi is the angle of rotation. Solution approach by Jens Geisler.
         r_T = [cos(phi) -sin(phi);
                sin(phi)  cos(phi] * r ;

     For arc generation, the center of the circle is the axis of rotation and the radius vector is
     defined from the circle center to the initial position. Each line segment is formed by successive
     vector rotations. This requires only two cos() and sin() computations to form the rotation
     matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
     all double numbers are single precision on the Arduino. (True double precision will not have
     round off issues for CNC applications.) Single precision error can accumulate to be greater than
     tool precision in some cases. Therefore, arc path correction is implemented.

     Small angle approximation may be used to reduce computation overhead further. This approximation
     holds for everything, but very small circles and large mm_per_arc_segment values. In other words,
     theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
     to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
     numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
     issue for CNC machines with the single precision Arduino calculations.

     This approximation also allows mc_arc to immediately insert a line segment into the planner
     without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
     a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead.
     This is important when there are successive arc motions.
  */
  // Vector rotation matrix values
  float cos_T = 1-0.5*theta_per_segment*theta_per_segment; // Small angle approximation
  float sin_T = theta_per_segment;

  float arc_target[3];
  float sin_Ti;
  float cos_Ti;
  float r_axisi;
  uint16_t i;
  int8_t count = 0;

  // Initialize the linear axis
  arc_target[axis_linear] = position[axis_linear];

  for (i = 1; i<segments; i++) { // Increment (segments-1)

    if (count < settings.n_arc_correction) {
      // Apply vector rotation matrix
      r_axisi = r_axis0*sin_T + r_axis1*cos_T;
      r_axis0 = r_axis0*cos_T - r_axis1*sin_T;
      r_axis1 = r_axisi;
      count++;
    } else {
      // Arc correction to radius vector. Computed only every n_arc_correction increments.
      // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
      cos_Ti = cos(i*theta_per_segment);
      sin_Ti = sin(i*theta_per_segment);
      r_axis0 = -offset[axis_0]*cos_Ti + offset[axis_1]*sin_Ti;
      r_axis1 = -offset[axis_0]*sin_Ti - offset[axis_1]*cos_Ti;
      count = 0;
    }

    // Update arc_target location
    arc_target[axis_0] = center_axis0 + r_axis0;
    arc_target[axis_1] = center_axis1 + r_axis1;
    arc_target[axis_linear] += linear_per_segment;
    MotionLine(arc_target[X_AXIS], arc_target[Y_AXIS], arc_target[Z_AXIS], feed_rate, invert_feed_rate);

    // Bail mid-circle on system abort. Runtime command check already performed by MotionLine.
    //if (sys.abort) { return; }
  }
  // Ensure last segment arrives at target location.
  MotionLine(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], feed_rate, invert_feed_rate);
}


// Method to ready the system to reset by setting the runtime reset command and killing any
// active processes in the system. This also checks if a system reset is issued while Grbl
// is in a motion state. If so, kills the steppers and sets the system alarm to flag position
// lost, since there was an abrupt uncontrolled deceleration. Called at an interrupt level by
// runtime abort command and hard limits. So, keep to a minimum.
void MotionReset()
{
  // Only this function can set the system reset. Helps prevent multiple kill calls.
  if (bit_isfalse(sys.execute, EXEC_RESET)) {
    sys.execute |= EXEC_RESET;

    // Kill spindle and coolant.
   // spindle_stop();
   // coolant_stop();

    // Kill steppers only if in any motion state, i.e. cycle, feed hold, homing, or jogging
    // NOTE: If steppers are kept enabled via the step idle delay setting, this also keeps
    // the steppers enabled by avoiding the go_idle call altogether, unless the motion state is
    // violated, by which, all bets are off.
    switch (sys.state) {
      case STATE_CYCLE: case STATE_HOLD: case STATE_HOMING: // case STATE_JOG:
        sys.execute |= EXEC_ALARM; // Execute alarm state.
        BSP_AxisDisable(X_AXIS);
        BSP_AxisDisable(Y_AXIS);
        BSP_AxisDisable(Z_AXIS); // Execute alarm force kills steppers. Position likely lost.
    }
  }
}

// Perform homing cycle to locate and set machine zero. Only '$H' executes this command.
// NOTE: There should be no motions in the buffer and Grbl must be in an idle state before
// executing the homing cycle. This prevents incorrect buffered plans after homing.
void MotionGoHome(void)
{
    plan_init();
    uint16_t limitStatus;
    uint16_t stepCount;
  sys.state = STATE_HOMING; // Set system state variable
 // LIMIT_PCMSK &= ~LIMIT_MASK; // Disable hard limits pin change register for cycle duration

    xLimitDetected = FALSE;
    yLimitDetected = FALSE;

    BSP_SetStepSize(X_AXIS, QUARTER);
    BSP_SetStepSize(Y_AXIS, QUARTER);
    BSP_SetStepSize(Z_AXIS, QUARTER);
    settings.steps_per_mm[X_AXIS] = X_STEPS_MM(QUARTER_STEP);
    settings.steps_per_mm[Y_AXIS] = Y_STEPS_MM(QUARTER_STEP);
    settings.steps_per_mm[Z_AXIS] = Z_STEPS_MM(QUARTER_STEP);
    // Find out if X or Y axis are at limit
    limitStatus = mPORTCReadBits(xLimitInput.pin||yLimitInput.pin);

    if(limitStatus)
    {
        if(limitStatus & xLimitInput.pin)     // If Xat Limit
        {
           stepCount = 0;
           BSP_Timer3Start(100);
           OpenOC2(XS_PWM_ENA,  (ReadPeriod3()>>1), ReadPeriod3()>>1);
           BSP_AxisEnable(X_AXIS, NEGATIVE);
           while((mPORTCReadBits(xLimitInput.pin)))
           {
            stepCount++;
            steps_X = 0x1;
            if(stepCount >= 20)
                mPORTGToggleBits(xAxis.directionPin.pin);
           }
           BSP_AxisDisable(X_AXIS);
           if(mPORTGReadBits(xAxis.directionPin.pin) == POSITIVE)
               gcode.position[X_AXIS] = 215.000;
           else
               gcode.position[X_AXIS] =0;
        }
        if(limitStatus & yLimitInput.pin)     // If Xat Limit
        {
           stepCount = 0;
           BSP_Timer2Start(100);
           OpenOC1(XS_PWM_ENA,  (ReadPeriod2()>>1), ReadPeriod2()>>1);
           BSP_AxisEnable(Y_AXIS, NEGATIVE);
           while((mPORTCReadBits(yLimitInput.pin)))
           {
            stepCount++;
            steps_Y = 0x1;
            if(stepCount >= 20)
                mPORTEToggleBits(yAxis.directionPin.pin);
           }
           if(mPORTEReadBits(xAxis.directionPin.pin) == POSITIVE)
               gcode.position[Y_AXIS] = 215.000;
           else
               gcode.position[Y_AXIS] =0;
        }
    }
    else
    {
        steps_Y = 0xFFFFFFFF;
        BSP_Timer2Start(250);
        OpenOC1(YZ_PWM_ENA, (ReadPeriod2()>>1), ReadPeriod2());
        BSP_AxisEnable(Y_AXIS, NEGATIVE);
        while(!(yLimitDetected)){};
        BSP_Timer2Start(10);
        OpenOC1(YZ_PWM_ENA, (ReadPeriod2()>>1), ReadPeriod2());

        while((mPORTCReadBits(yLimitInput.pin)))
        {
            steps_Y = 0x1;
            BSP_AxisEnable(Y_AXIS, POSITIVE);
        }
        BSP_AxisDisable(Y_AXIS);
        gcode.position[Y_AXIS] = 0;



        steps_X = 0xFFFFFFFF;
        BSP_Timer3Start(250);
        OpenOC2(XS_PWM_ENA,  (ReadPeriod3()>>1), ReadPeriod3());
        BSP_AxisEnable(X_AXIS, NEGATIVE);
        while(!(xLimitDetected)){};
        BSP_Timer3Start(10);
        OpenOC2(XS_PWM_ENA,  (ReadPeriod3()>>1), ReadPeriod3());
        while((mPORTCReadBits(xLimitInput.pin)))
        {
            steps_X = 0x1;
            BSP_AxisEnable(X_AXIS, POSITIVE);
        }
        BSP_AxisDisable(X_AXIS);
        gcode.position[X_AXIS] = 0;
        gcode.position[Z_AXIS] = 0;

       
    }
//    steps_X = 0xFFFFFFFF;
//    BSP_Timer3Start(1500);
//    OpenOC2(XS_PWM_ENA,  (ReadPeriod3()), ReadPeriod3());
//    BSP_AxisEnable(X_AXIS, NEGATIVE);
//    while(!(xLimitDetected)){};
//    BSP_Timer3Start(100);
//    OpenOC2(XS_PWM_ENA,  (ReadPeriod3()), ReadPeriod3());
//    gcode.position[X_AXIS] = 0;
//    while((mPORTCReadBits(xLimitInput.pin)))
//    {
//        steps_X = 0x1;
//        BSP_AxisEnable(X_AXIS, POSITIVE);
//    }
//    BSP_AxisDisable(X_AXIS);
//
//
//    steps_Y = 0xFFFFFFFF;
//    BSP_Timer2Start(1500);
//    OpenOC1(YZ_PWM_ENA, (ReadPeriod2()), ReadPeriod2());
//    BSP_AxisEnable(Y_AXIS, NEGATIVE);
//    while(!(yLimitDetected)){};
//    BSP_Timer2Start(100);
//    OpenOC1(YZ_PWM_ENA, (ReadPeriod2()), ReadPeriod2());
//    gcode.position[Y_AXIS] = 0;
//    while((mPORTCReadBits(yLimitInput.pin)))
//    {
//        steps_Y = 0x1;
//        BSP_AxisEnable(Y_AXIS, POSITIVE);
//    }
//    BSP_AxisDisable(Y_AXIS);
//
//    gcode.position[Z_AXIS] = 0;

  protocol_execute_runtime(); // Check for reset and set system abort.
  if (sys.abort) { return; } // Did not complete. Alarm state set by mc_alarm.

  // The machine should now be homed and machine zero has been located. Upon completion,
  // reset system position and sync internal position vectors.
  clear_vector_float(sys.position); // Set machine zero
  sys_sync_current_position();
  sys.state = STATE_IDLE; // Set system state to IDLE to complete motion and indicate homed.

  // Pull-off axes (that have been homed) from limit switches before continuing motion.
  // This provides some initial clearance off the switches and should also help prevent them
  // from falsely tripping when hard limits are enabled.
  int8_t x_dir, y_dir, z_dir;
  x_dir = y_dir = z_dir = 0;
  if (HOMING_LOCATE_CYCLE & (1<<X_AXIS)) {
    if (settings.homing_dir_mask & (1<<xAxis.directionPin.pin)) { x_dir = 1; }
    else { x_dir = -1; }
  }
  if (HOMING_LOCATE_CYCLE & (1<<Y_AXIS)) {
    if (settings.homing_dir_mask & (1<<yAxis.directionPin.pin)) { y_dir = 1; }
    else { y_dir = -1; }
  }
  if (HOMING_LOCATE_CYCLE & (1<<Z_AXIS)) {
    if (settings.homing_dir_mask & (1<<zAxis.directionPin.pin)) { z_dir = 1; }
    else { z_dir = -1; }
  }
  MotionLine(x_dir*settings.homing_pulloff, y_dir*settings.homing_pulloff,
          z_dir*settings.homing_pulloff, settings.homing_seek_rate, false);
  //st_cycle_start(); // Move it. Nothing should be in the buffer except this motion.
  plan_synchronize(); // Make sure the motion completes.

  // The gcode parser position circumvented by the pull-off maneuver, so sync position vectors.
  sys_sync_current_position();

  // If hard limits feature enabled, re-enable hard limits pin change register after homing cycle.
 // if (bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE)) { LIMIT_PCMSK |= LIMIT_MASK; }
  // Finished!
}


 void seek(float , float , float);