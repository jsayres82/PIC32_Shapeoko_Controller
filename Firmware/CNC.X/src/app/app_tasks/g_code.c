/*
 * File:   gcode.c
 * Author: Justin Sayres
 *
 * Created on 01/08/13
 */
#include <xc.h>
#include <float.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include "g_code.h"
#include "program_planner.h"
#include "motion.h"
#include "nuts_bolts.h"
#include "report.h"
#include "settings.h"

void ftoa(float Value, char* Buffer);
char float_value[8];

char command[MAX_COMMANDS];
int value_cmd[MAX_COMMANDS];
int intValue;

int inputType;

 int modalGroupWords = 0;  // Bitflag variable to track and check modal group words in block
 int axisWords = 0;        // Bitflag to track which XYZ(ABC) parameters exist in block

float inverseFeedRate = -1; // negative inverseFeedRate means no inverseFeedRate specified
char absoluteOverride = false; // true(1) = absolute motion for this block only {G53}
char nonModalAction = NON_MODAL_NONE; // Tracks the actions of modal group 0 (non-modal)
float target[32], offset[3];


float decimal_place = 1;
gcode_machine_status gcode;


//static void select_plane(char axis_0, char axis_1, char axis_2)
//{
  //gcode.planeAxis_0 = axis_0;
  //gcode.planeAxis_1 = axis_1;
  //gcode.planeAxis_2 = axis_2;
//}

 //Sets g-code parser position in mm. Input in steps. Called by the system abort and hard
 //limit pull-off routines.
void gc_set_current_position(float x, float y, float z)
{
    gcode.position[X_AXIS] = x/50800;
    gcode.position[Y_AXIS] = y/50800;
    gcode.position[Z_AXIS] = z/65024;
}


static float to_millimeters(float value)
{
  return(gcode.inchesMode ? (value * MM_PER_INCH) : value);
}


// Parses the next statement and leaves the counter on the first character following
// the statement. Returns 1 if there was a statements, 0 if end of string was reached
// or there was an error (check state.status_code).
static int next_statement(char *letter, float *float_ptr, char *line, uint8_t *charCounter)
{
  if (line[*charCounter] == 0) {
    return(0); // No more statements
  }

  *letter = line[*charCounter];
  if((*letter < 'A') || (*letter > 'Z')) {
   FAIL(STATUS_EXPECTED_COMMAND_LETTER);
    return(0);
  }
  (*charCounter)++;
  if (!read_float(line, charCounter, float_ptr)) {
    FAIL(STATUS_BAD_NUMBER_FORMAT);
    return(0);
  };
  return(1);
}



int ExecuteLine(char* line){

  uint8_t charCounter = 0;
  char letter;
  float value;
  int intValue;

  uint16_t modalGroupWords = 0;  // Bitflag variable to track and check modal group words in block
  uint8_t axisWords = 0;          // Bitflag to track which XYZ(ABC) parameters exist in block

  float inverseFeedRate = -1; // negative inverseFeedRate means no inverseFeedRate specified
  uint8_t absoluteOverride = false; // true(1) = absolute motion for this block only {G53}
  uint8_t nonModalAction = NON_MODAL_NONE; // Tracks the actions of modal group 0 (non-modal)

  float target[3], offset[3];
  clear_vector(target); // XYZ(ABC) axes parameters.
  clear_vector(offset); // IJK Arc offsets are incremental. Value of zero indicates no change.

  gcode.lineStatus = STATUS_GOOD;

  /* Pass 1: Commands and set all modes. Check for modal group violations.
     NOTE: Modal group numbers are defined in Table 4 of NIST RS274-Ngcode v3, pg.20 */
  char group_number = MODAL_GROUP_NONE;
  while(next_statement(&letter, &value, line, &charCounter))
  {
    intValue = (int)value;
    switch(letter) {
      case 'G':
        // Set modal group values
        switch(intValue) {
          case 4: case 10: case 28: case 30: case 53: case 92: group_number = MODAL_GROUP_0; break;
          case 0: case 1: case 2: case 3: case 80: group_number = MODAL_GROUP_1; break;
          case 17: case 18: case 19: group_number = MODAL_GROUP_2; break;
          case 90: case 91: group_number = MODAL_GROUP_3; break;
          case 93: case 94: group_number = MODAL_GROUP_5; break;
          case 20: case 21: group_number = MODAL_GROUP_6; break;
          case 54: case 55: case 56: case 57: case 58: case 59: group_number = MODAL_GROUP_12; break;
        }
        // Set 'G' commands
        switch(intValue) {
          case 0: gcode.motionMode = MOTION_MODE_SEEK; break;
          case 1: gcode.motionMode = MOTION_MODE_LINEAR ; break;
          case 2: gcode.motionMode = MOTION_MODE_CW_ARC; break;
          case 3: gcode.motionMode = MOTION_MODE_CCW_ARC; break;
          case 4: nonModalAction = NON_MODAL_DWELL; break;
          case 10: nonModalAction = NON_MODAL_SET_COORDINATE_DATA; break;
          case 17: //select_plane(X_AXIS, Y_AXIS, Z_AXIS);
              break;
          case 18: //select_plane(X_AXIS, Z_AXIS, Y_AXIS);
              break;
          case 19: //select_plane(Y_AXIS, Z_AXIS, X_AXIS);
          break;
          case 20: gcode.inchesMode = true; break;
          case 21: gcode.inchesMode = false; break;
          case 28: case 30:
            intValue = (int)(10*value); // Multiply by 10 to pick up Gxx.1
            switch(intValue) {
              case 280: nonModalAction = NON_MODAL_GO_HOME_0; break;
              case 281: nonModalAction = NON_MODAL_SET_HOME_0; break;
              case 300: nonModalAction = NON_MODAL_GO_HOME_1; break;
              case 301: nonModalAction = NON_MODAL_SET_HOME_1; break;
              default: FAIL(STATUS_UNSUPPORTED_STATEMENT);
            }
          case 53: absoluteOverride = true; break;
          case 54: case 55: case 56: case 57: case 58: case 59:
            gcode.coordSelect = intValue-54;
            break;
          case 80: gcode.motionMode = MOTION_MODE_CANCEL; break;
          case 90: gcode.absoluteMode = true; break;
          case 91: gcode.absoluteMode = false; break;
          case 92:
            intValue = (int)(10*value); // Multiply by 10 to pick up G92.1
            switch(intValue) {
              case 920: nonModalAction = NON_MODAL_SET_COORDINATE_OFFSET; break;
              case 921: nonModalAction = NON_MODAL_RESET_COORDINATE_OFFSET; break;
              default: FAIL(STATUS_UNSUPPORTED_STATEMENT);
            }
            break;
          case 93: gcode.inverseFeedRateMode = true; break;
          case 94: gcode.inverseFeedRateMode = false; break;
          default: FAIL(STATUS_UNSUPPORTED_STATEMENT);
        }
        break;
      case 'M':
        // Set modal group values
        switch(intValue) {
          case 0: case 1: case 2: case 30: group_number = MODAL_GROUP_4; break;
          case 3: case 4: case 5: group_number = MODAL_GROUP_7; break;
        }
        // Set 'M' commands
        switch(intValue) {
          case 0: gcode.programFlow = PROGRAM_FLOW_PAUSED; break; // Program pause
          case 1: break; // Optional stop not supported. Ignore.
          case 2: case 30: gcode.programFlow = PROGRAM_FLOW_COMPLETED; break; // Program end and reset
          case 3: gcode.spindleDirection = 1; break;
          case 4: gcode.spindleDirection = -1; break;
          case 5: gcode.spindleDirection = 0; break;
          #ifdef ENABLE_M7
            case 7: gcode.coolantMode = COOLANT_MIST_ENABLE; break;
          #endif
          case 8: gcode.coolantMode = COOLANT_FLOOD_ENABLE; break;
          case 9: gcode.coolantMode = COOLANT_DISABLE; break;
          default:FAIL(STATUS_UNSUPPORTED_STATEMENT);
        }
        break;

    }
    //Check for modal group multiple command violations in the current block
    if (group_number)
    {
        if ( bit_istrue(modalGroupWords,bit(group_number)) )
        {
           FAIL(STATUS_MODAL_GROUP_VIOLATION);
        }
        else
        {
            bit_true(modalGroupWords,bit(group_number));
        }
        group_number = MODAL_GROUP_NONE; // Reset for next command.
    }/* End Modal Grout check */

  }/* End While (modal_counter > charCouhtert)*/


  // If there were any errors parsing this line, we will return right away with the bad news
  if (gcode.lineStatus) { return(gcode.lineStatus); }

  /* Pass 2: Parameters. All units converted according to current block commands. Position
     parameters are converted and flagged to indicate a change. These can have multiple connotations
     for different commands. Each will be converted to their proper value upon execution. */
  float p = 0, r = 0;
  char l = 0;
  charCounter = 0;

  while(next_statement(&letter, &value, line, &charCounter)){
    switch(letter) {
      case 'G': case 'M': case 'N': break; // Ignore command statements and line numbers
      case 'F':
        if (value <= 0) {  FAIL(STATUS_INVALID_STATEMENT);  } // Must be greater than zero
        if (gcode.inverseFeedRateMode) {
          inverseFeedRate = to_millimeters(value); // seconds per motion for this motion only
        } else {
          gcode.feedRate = to_millimeters(value); // millimeters per minute
        }
        break;
      case 'I': case 'J': case 'K': offset[letter-'I'] = to_millimeters(value); break;
      case 'L': l = value; break;
      case 'P': p = value; break;
      case 'R': r = to_millimeters(value); break;
      case 'S':
        if (value < 0) {  FAIL(STATUS_INVALID_STATEMENT);  } // Cannot be negative
        // TBD: Spindle speed not supported due to PWM issues, but may come back once resolved.
        // gcode.spindle_speed = value;
        break;
      case 'T':
        if (value < 0) { FAIL(STATUS_INVALID_STATEMENT); } // Cannot be negative
        gcode.tool = value;
        break;
      case 'X': target[X_AXIS] = to_millimeters(value); bit_true(axisWords,bit(X_AXIS)); break;
      case 'Y': target[Y_AXIS] = to_millimeters(value); bit_true(axisWords,bit(Y_AXIS)); break;
      case 'Z': target[Z_AXIS] = to_millimeters(value); bit_true(axisWords,bit(Z_AXIS)); break;
      default: FAIL(STATUS_UNSUPPORTED_STATEMENT);
    }
       
  }/* End While loop */

  // If there were any errors parsing this line, we will return right away with the bad news
  if (gcode.lineStatus) { return(gcode.lineStatus); }


  /* Execute Commands: Perform by order of execution defined in NIST RS274-Ngcode.v3, Table 8, pg.41.
     NOTE: Independent non-motion/settings parameters are set out of this order for code efficiency
     and simplicity purposes, but this should not affect proper g-code execution. */

  // ([F]: Set feed and seek rates.)
  // TODO: Seek rates can change depending on the direction and maximum speeds of each axes. When
  // max axis speed is installed, the calculation can be performed here, or maybe in the planner.



  /* I need to do this to control my spindle*/
  //if (sys.state != STATE_CHECK_MODE) {
    //  ([M6]: Tool change should be executed here.)

    // [M3,M4,M5]: Update spindle state
   // spindle_run(gcode.spindleDirection);

    // [*M7,M8,M9]: Update coolant state
    //coolant_run(gcode.coolantMode);
  //}
/* I need to do this to control my spindle*/

  // [G54,G55,...,G59]: Coordinate system selection
  if ( bit_istrue(modalGroupWords,bit(MODAL_GROUP_12)) )
  { // Check if called in block
    //float coord_data[N_AXIS];
    //if (!(settings_read_coord_data(gcode.coord_select,coord_data))) { return(STATUS_SETTING_READ_FAIL); }
    //memcpy(gcode.coord_system,coord_data,sizeof(coord_data));
  }




  // [G4,G10,G28,G30,G92,G92.1]: Perform dwell, set coordinate system data, homing, or set axis offsets.
  // NOTE: These commands are in the same modal group, hence are mutually exclusive. G53 is in this
  // modal group and do not effect these actions.
  switch (nonModalAction)
  {
    case NON_MODAL_DWELL:
      if (p < 0)
      { // Time cannot be negative.
        ERROR(BAD_DATA);
      }
      else
      {
        // Ignore dwell in check gcodeode modes
        //if (sys.state != STATE_CHECK_MODE) {
         // dwell(p);
      }
      break;
    case NON_MODAL_SET_COORDINATE_DATA:

      intValue = floor(p); // Convert p value to int.
      
      if ((l != 2 && l != 20) || (intValue < 0 || intValue > 1))
      { // L2 and L20. P1=G54, P2=G55, ...
        ERROR(UNKNOWN_COMMAND);
      }
      else if (!axisWords && l==2)
      { // No axis words.
        ERROR(BAD_DATA);
      }
      else
      {
        if (intValue > 0) {
            //intValue--;
        } // Adjust P1-P6 index to EEPROM coordinate data indexing.
        else {
            intValue = gcode.coordSelect;
        } // Index P0 as the active coordinate system
        float coord_data[N_AXIS];
        //if (!settings_read_coord_data(intValue,coord_data)) { return(STATUS_SETTING_READ_FAIL); }

        int i;
        // Update axes defined only in block. Always in machine coordinates. Can change non-active system.
        for (i=0; i<N_AXIS; i++)
        { // Axes indices are consistent, so loop may be used.
          if (bit_istrue(axisWords,bit(i)) )
          {
            if (l == 20)
              coord_data[i] = gcode.position[i]-target[i]; // L20: Update axis current position to target
            else
              coord_data[i] = target[i]; // L2: Update coordinate system axis
          }
        }
        /*  THIS IS FOR SAVING COORDINATES TO EEPROM FOR POWER ON POSITIONING */
        //settings_write_coord_data(intValue,coord_data);
        // Update system coordinate system if currently active.

   /*  I NEED TO SET UP PSV MEMORY READS AND WRITES USING DEE EMULATION LIBRARY   */
        //if (gcode.coord_select == intValue) { memcpy(gcode.coord_system,coord_data,sizeof(coord_data)); }

      }
      axisWords = 0; // Axis words used. Lock out from motion modes by clearing flags.
      break;
    case NON_MODAL_GO_HOME_0: case NON_MODAL_GO_HOME_1:
      // Move to intermediate position before going home. Obeys current coordinate system and offsets
      // and absolute and incremental modes.
      if (axisWords) {
        // Apply absolute mode coordinate offsets or incremental mode offsets.
        char i;
        for (i=0; i<N_AXIS; i++) { // Axes indices are consistent, so loop may be used.
          if ( bit_istrue(axisWords,bit(i)) ) {
            if (gcode.absoluteMode) {
             target[i] += gcode.coordSystem[i] + gcode.coordOffset[i];
            } else {
             target[i] += gcode.position[i];
            }
          } else {
            target[i] = gcode.position[i];
          }
        }
        MotionLine(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], settings.default_seek_rate, false);
      }

      // Retreive G28/30 go-home position data (in machine coordinates) from EEPROM
      //float coord_data[N_AXIS];
      //if (nonModalAction == NON_MODAL_GO_HOME_1) {
  /*  I NEED TO SET UP PSV MEMORY READS AND WRITES USING DEE EMULATION LIBRARY   */
       // if (!settings_read_coord_data(SETTING_INDEX_G30 ,coord_data)) { return(STATUS_SETTING_READ_FAIL); }
      //}
      //else {
        //if (!settings_read_coord_data(SETTING_INDEX_G28 ,coord_data)) { return(STATUS_SETTING_READ_FAIL); }
      //}
      //MotionLine(coord_data[X_AXIS], coord_data[Y_AXIS], coord_data[Z_AXIS], settings.default_seek_rate, false);
      //memcpy(gcode.position, coord_data, sizeof(coord_data)); // gcode.position[] = coord_data[];
      //axisWords = 0; // Axis words used. Lock out from motion modes by clearing flags.
      break;

    case NON_MODAL_SET_HOME_0: case NON_MODAL_SET_HOME_1:
      //if (nonModalAction == NON_MODAL_SET_HOME_1) {
        //settings_write_coord_data(SETTING_INDEX_G30,gcode.position);
      //} else {
        //settings_write_coord_data(SETTING_INDEX_G28,gcode.position);
      //}
      break;
    case NON_MODAL_SET_COORDINATE_OFFSET:
        if (!axisWords) { // No axis words
            FAIL(STATUS_INVALID_STATEMENT);
      } else {
        // Update axes defined only in block. Offsets current system to defined value. Does not update when
        // active coordinate system is selected, but is still active unless G92.1 disables it.
        char i;
        for (i=0; i<=2; i++) { // Axes indices are consistent, so loop may be used.
         if (bit_istrue(axisWords,bit(i)) ) {
            gcode.coordOffset[i] = gcode.position[i]-gcode.coordSystem[i]-target[i];
          }
       }
     }
      axisWords = 0; // Axis words used. Lock out from motion modes by clearing flags.
      break;
    case NON_MODAL_RESET_COORDINATE_OFFSET:
      //clear_vector(gcode.coord_offset); // Disable G92 offsets by zeroing offset vector.
      break;
  }





  // [G0,G1,G2,G3,G80]: Perform motion modes.
  // NOTE: Commands G10,G28,G30,G92 lock out and prevent axis words from use in motion modes.
  // Enter motion modes only if there are axis words or a motion mode command word in the block.
  if ( bit_istrue(modalGroupWords,bit(MODAL_GROUP_1)) || axisWords )
  {

    // G1,G2,G3 require F word in inverse time mode.
    if ( gcode.inverseFeedRateMode )
    {
      if (inverseFeedRate < 0 && gcode.motionMode != MOTION_MODE_CANCEL)
        ERROR(BAD_DATA);
    }
    // Absolute override G53 only valid with G0 and G1 active.
    if ( absoluteOverride && !(gcode.motionMode == MOTION_MODE_SEEK || gcode.motionMode == MOTION_MODE_LINEAR))
      ERROR(BAD_DATA);
    // Report any errors.
    if (gcode.lineStatus) { return(gcode.lineStatus); }

    // Convert all target position data to machine coordinates for executing motion. Apply
    // absolute mode coordinate offsets or incremental mode offsets.
    // NOTE: Tool offsets may be appended to these conversions when/if this feature is added.

    unsigned int i;

    for (i=0; i<=2; i++)
    { // Axes indices are consistent, so loop may be used to save flash space.
      if ( bit_istrue(axisWords,bit(i)) )
      {
        if (!absoluteOverride)
        { // Do not update target in absolute override mode
          if (gcode.absoluteMode)
              target[i] += gcode.coordSystem[i] + gcode.coordOffset[i]; // Absolute mode where offset is current position
          else 
              target[i] += gcode.position[i]; // Incremental mode
        }
      }
      else
      {
        target[i] = gcode.position[i]; // No axis word in block. Keep same axis position.
      }
    }

    switch (gcode.motionMode)
    {
      case MOTION_MODE_CANCEL:
        if (axisWords){ FAIL(STATUS_INVALID_STATEMENT); } // No axis words allowed while active.
        break;
      case MOTION_MODE_SEEK:
        if (!axisWords){ FAIL(STATUS_INVALID_STATEMENT); }
        else { MotionLine(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], 10000,0); }
        break;

      case MOTION_MODE_LINEAR:
        // TODO: Inverse time requires F-word with each statement. Need to do a check. Also need
        // to check for initial F-word upon startup. Maybe just set to zero upon initialization
        // and after an inverse time move and then check for non-zero feed rate each time. This
        // should be efficient and effective.
        if (!axisWords){ FAIL(STATUS_INVALID_STATEMENT); }
        else{MotionLine(target[X_AXIS], target[Y_AXIS], target[Z_AXIS],  10000, 0);};
        // Actual code:
        //else { mc_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS],
        //  (gc.inverse_feed_rate_mode) ? inverse_feed_rate : gc.feed_rate, gc.inverse_feed_rate_mode); }
        break;
      case MOTION_MODE_CW_ARC: case MOTION_MODE_CCW_ARC:
        // Check if at least one of the axes of the selected plane has been specified. If in center
        // format arc mode, also check for at least one of the IJK axes of the selected plane was sent.
        if ( !( bit_false(axisWords,bit(gcode.planeAxis_2)) ) ||
             ( !r && !offset[gcode.planeAxis_0] && !offset[gcode.planeAxis_1] ) ) {
          ERROR(BAD_DATA);
        } else {
          if (r != 0) { // Arc Radius Mode
            /*
              We need to calculate the center of the circle that has the designated radius and passes
              through both the current position and the target position. This method calculates the following
              set of equations where [x,y] is the vector from current to target position, d == magnitude of
              that vector, h == hypotenuse of the triangle formed by the radius of the circle, the distance to
              the center of the travel vector. A vector perpendicular to the travel vector [-y,x] is scaled to the
              length of h [-y/d*h, x/d*h] and added to the center of the travel vector [x/2,y/2] to form the new point
              [i,j] at [x/2-y/d*h, y/2+x/d*h] which will be the center of our arc.

              d^2 == x^2 + y^2
              h^2 == r^2 - (d/2)^2
              i == x/2 - y/d*h
              j == y/2 + x/d*h

                                                                   O <- [i,j]
                                                                -  |
                                                      r      -     |
                                                          -        |
                                                       -           | h
                                                    -              |
                                      [0,0] ->  C -----------------+--------------- T  <- [x,y]
                                                | <------ d/2 ---->|

              C - Current position
              T - Target position
              O - center of circle that pass through both C and T
              d - distance from C to T
              r - designated radius
              h - distance from center of CT to O

              Expanding the equations:

              d -> sqrt(x^2 + y^2)
              h -> sqrt(4 * r^2 - x^2 - y^2)/2
              i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
              j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2

              Which can be written:

              i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
              j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2

              Which we for size and speed reasons optimize to:

              h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
              i = (x - (y * h_x2_div_d))/2
              j = (y + (x * h_x2_div_d))/2
          */


            // Calculate the change in position along each selected axis
            float x = target[gcode.planeAxis_0]-gcode.position[gcode.planeAxis_0];
            float y = target[gcode.planeAxis_1]-gcode.position[gcode.planeAxis_1];

            clear_vector(offset);
            // First, use h_x2_div_d to compute 4*h^2 to check if it is negative or r is smaller
            // than d. If so, the sqrt of a negative number is complex and error out.
            float h_x2_div_d = 4 * r*r - x*x - y*y;
            if (h_x2_div_d < 0) { ERROR(STATUS_ARC_RADIUS_ERROR); return(gcode.lineStatus); }
            // Finish computing h_x2_div_d.
            h_x2_div_d = -sqrt(h_x2_div_d)/hypot(x,y); // == -(h * 2 / d)
            // Invert the sign of h_x2_div_d if the circle is counter clockwise (see sketch below)
            if (gcode.motionMode == MOTION_MODE_CCW_ARC) { h_x2_div_d = -h_x2_div_d; }

            /* The counter clockwise circle lies to the left of the target direction. When offset is positive,
               the left hand circle will be generated - when it is negative the right hand circle is generated.


                                                             T  <-- Target position

                                                             ^
                  Clockwise circles with this center         |          Clockwise circles with this center will have
                  will have > 180 deg of angular travel      |          < 180 deg of angular travel, which is a good thing!
                                                   \         |          /
      center of arc when h_x2_div_d is positive ->  x <----- | -----> x <- center of arc when h_x2_div_d is negative
                                                             |
                                                             |

                                                             C  <-- Current position                                */

            // Negative R is g-code-alese for "I want a circle with more than 180 degrees of travel" (go figure!),
            // even though it is advised against ever generating such circles in a single line of g-code. By
            // inverting the sign of h_x2_div_d the center of the circles is placed on the opposite side of the line of
            // travel and thus we get the unadvisably long arcs as prescribed.
            if (r < 0) {
                h_x2_div_d = -h_x2_div_d;
                r = -r; // Finished with r. Set to positive for mc_arc
            }
            // Complete the operation by calculating the actual center of the arc
            offset[gcode.planeAxis_0] = 0.5*(x-(y*h_x2_div_d));
            offset[gcode.planeAxis_1] = 0.5*(y+(x*h_x2_div_d));

          } else { // Arc Center Format Offset Mode
            r = hypot(offset[gcode.planeAxis_0], offset[gcode.planeAxis_1]); // Compute arc radius for mc_arc
          }

          // Set clockwise/counter-clockwise sign for mc_arc computations
          uint8_t isclockwise = false;
          if (gcode.motionMode == MOTION_MODE_CW_ARC) { isclockwise = true; }

          // Trace the arc
          MotionArc(gcode.position, target, offset, gcode.planeAxis_0, gcode.planeAxis_1, gcode.planeAxis_2,
            (gcode.inverseFeedRateMode) ? inverseFeedRate : gcode.feedRate, gcode.inverseFeedRateMode,
            r, isclockwise);
        }
        break;
    }

    // Report any errors.
    if (gcode.lineStatus) { return(gcode.lineStatus); }

    // As far as the parser is concerned, the position is now == target. In reality the
    // motion control system might still be processing the action and the real tool position
    // in any intermediate location.
    memcpy(gcode.position, target, sizeof(target)); // gcode.position[] = target[];
  }

  // M0,M1,M2,M30: Perform non-running program flow actions. During a program pause, the buffer may
  // refill and can only be resumed by the cycle start run-time command.
  if (gcode.programFlow)
  {
    //plan_synchronize(); // Finish all remaining buffered motions. Program paused when complete.
    //sys.auto_start = false; // Disable auto cycle start. Forces pause until cycle start issued.

    // If complete, reset to reload defaults (G92.2,G54,G17,G90,G94,M48,G40,M5,M9). Otherwise,
    // re-enable program flow after pause complete, where cycle start will resume the program.
    if (gcode.programFlow == PROGRAM_FLOW_COMPLETED){ MotionReset(); }
    else { gcode.programFlow = PROGRAM_FLOW_RUNNING; }
  }
  return(gcode.lineStatus);
}


/**************************************************
 *
 *    ftoa - converts float to string
 *
 ***************************************************
 *
 *    This is a simple implemetation with rigid
 *    parameters:
 *            - Buffer must be 8 chars long
 *            - 3 digits precision max
 *            - absolute range is -524,287 to 524,287
 *            - resolution (epsilon) is 0.125 and
 *              always rounds down
 **************************************************/
 void ftoa(float Value, char* Buffer)
 {
     union
     {
         float f;

         struct
         {
             unsigned int    mantissa_lo : 16;
             unsigned int    mantissa_hi : 7;
             unsigned int     exponent : 8;
             unsigned int     sign : 1;
         };
     } helper;

     unsigned long mantissa;
     signed char exponent;
     unsigned int int_part;
     char frac_part[3];
     int i, count = 0;

     helper.f = Value;
     //mantissa is LS 23 bits
     mantissa = helper.mantissa_lo;
     mantissa += ((unsigned long) helper.mantissa_hi << 16);
     //add the 24th bit to get 1.mmmm^eeee format
     mantissa += 0x00800000;
     //exponent is biased by 127
     exponent = (signed char) helper.exponent - 127;

     //too big to shove into 8 chars
     if (exponent > 18)
     {
         Buffer[0] = 'I';
         Buffer[1] = 'n';
         Buffer[2] = 'f';
         Buffer[3] = '\0';
         return;
     }

     //too small to resolve (resolution of 1/8)
     if (exponent < -3)
     {
         Buffer[0] = '0';
         Buffer[1] = '\0';
         return;
     }

     count = 0;

     //add negative sign (if applicable)
     if (helper.sign)
     {
         Buffer[0] = '-';
         count++;
     }

     //get the integer part
     int_part = mantissa >> (23 - exponent);
     //convert to string
     itoa(&Buffer[count],int_part, 10 );

     //find the end of the integer
     for (i = 0; i < 8; i++)
         if (Buffer[i] == '\0')
         {
             count = i;
             break;
         }

     //not enough room in the buffer for the frac part
     if (count > 5)
         return;

     //add the decimal point
     Buffer[count++] = '.';

     //use switch to resolve the fractional part
     switch (0x7 & (mantissa  >> (20 - exponent)))
     {
         case 0:
             frac_part[0] = '0';
             frac_part[1] = '0';
             frac_part[2] = '0';
             break;
         case 1:
             frac_part[0] = '1';
             frac_part[1] = '2';
             frac_part[2] = '5';
             break;
         case 2:
             frac_part[0] = '2';
             frac_part[1] = '5';
             frac_part[2] = '0';
             break;
         case 3:
             frac_part[0] = '3';
             frac_part[1] = '7';
             frac_part[2] = '5';
             break;
         case 4:
             frac_part[0] = '5';
             frac_part[1] = '0';
             frac_part[2] = '0';
             break;
         case 5:
             frac_part[0] = '6';
             frac_part[1] = '2';
             frac_part[2] = '5';
             break;
         case 6:
             frac_part[0] = '7';
             frac_part[1] = '5';
             frac_part[2] = '0';
             break;
         case 7:
             frac_part[0] = '8';
             frac_part[1] = '7';
             frac_part[2] = '5';
             break;
     }

     //add the fractional part to the output string
     for (i = 0; i < 3; i++)
         if (count < 7)
             Buffer[count++] = frac_part[i];

     //make sure the output is terminated
     Buffer[count] = '\0';
 
}

 

