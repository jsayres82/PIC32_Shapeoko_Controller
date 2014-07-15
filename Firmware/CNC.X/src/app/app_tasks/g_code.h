/*
 * File:   gcode.h
 * Author: Justin
 *
 * Created on April 23, 2013, 5:06 PM
 */

#ifndef _G_CODE_H
#define	_G_CODE_H

#ifdef	__cplusplus
extern "C" {
#endif
#define GCODE_H

#define LINE_SIZE 255
#define RX_FIFO_LIMIT 100         /* size of the FIFO array */
    
#define MM_PER_INCH (25.40)
#define INCH_PER_MM (0.0393701)


#define MAX_COMMANDS 8
#define false 0
#define true 1

#define N_AXIS 3 // Number of axes
//#define X_AXIS 0 // Axis indexing value
//#define Y_AXIS 1
//#define Z_AXIS 2

#define FAIL(STATUS) gcode.lineStatus = STATUS;


/* Status settings */
#define ERROR(STATUS) gcode.lineStatus = STATUS;

/* ERROR LIST */
#define STATUS_GOOD 0
#define STATUS_ERROR 1
#define UNKNOWN_COMMAND 2
#define BAD_DATA 3




/* GCODE COMMANDS */
#define COOLANT_MIST_ENABLE 2
#define COOLANT_FLOOD_ENABLE 1
#define COOLANT_DISABLE 0 // Must be zero.


typedef struct {
  char lineStatus;             // Parser status for current block
  char motionMode;             // {G0, G1, G2, G3, G80}
  char inverseFeedRateMode;  // {G93, G94}
  char inchesMode;             // 0 = millimeter mode, 1 = inches mode {G20, G21}
  char absoluteMode;           // 0 = relative motion, 1 = absolute motion {G90, G91}
  char programFlow;            // {M0, M1, M2, M30}
  char spindleDirection;        // 1 = CW, -1 = CCW, 0 = Stop {M3, M4, M5}
  char coolantMode;            // 0 = Disable, 1 = Flood Enable {M8, M9}
  float feedRate;                 // Millimeters/min
//  float seek_rate;                 // Millimeters/min. Will be used in v0.9 when axis independence is installed
  float position[3];               // Where the interpreter considers the tool to be at this point in the code
  char tool;
//  uint16_t spindle_speed;          // RPM/100
  char planeAxis_0,
          planeAxis_1,
          planeAxis_2;            // The axes of the selected plane
  char coordSelect;               // Active work coordinate system number. Default: 0=G54.
  float coordSystem[N_AXIS];      // Current work coordinate system (G54+). Stores offset from absolute machine
                                   // position in mm. Loaded from EEPROM when called.
  float coordOffset[N_AXIS];      // Retains the G92 coordinate offset (work coordinates) relative to
                                   // machine zero in mm. Non-persistent. Cleared upon reset and boot.
} gcode_machine_status;
extern gcode_machine_status gcode;

// Define modal group internal numbers for checking multiple command violations and tracking the
// type of command that is called in the block. A modal group is a group of g-code commands that are
// mutually exclusive, or cannot exist on the same line, because they each toggle a state or execute
// a unique motion. These are defined in the NIST RS274-NGC v3 g-code standard, available online,
// and are similar/identical to other g-code interpreters by manufacturers (Haas,Fanuc,Mazak,etc).
#define MODAL_GROUP_NONE 0
#define MODAL_GROUP_0 1 // [G4,G10,G28,G30,G53,G92,G92.1] Non-modal
#define MODAL_GROUP_1 2 // [G0,G1,G2,G3,G80] Motion
#define MODAL_GROUP_2 3 // [G17,G18,G19] Plane selection
#define MODAL_GROUP_3 4 // [G90,G91] Distance mode
#define MODAL_GROUP_4 5 // [M0,M1,M2,M30] Stopping
#define MODAL_GROUP_5 6 // [G93,G94] Feed rate mode
#define MODAL_GROUP_6 7 // [G20,G21] Units
#define MODAL_GROUP_7 8 // [M3,M4,M5] Spindle turning
#define MODAL_GROUP_12 9 // [G54,G55,G56,G57,G58,G59] Coordinate system selection

// Define command actions for within execution-type modal groups (motion, stopping, non-modal). Used
// internally by the parser to know which command to execute.
#define MOTION_MODE_SEEK 0 // G0
#define MOTION_MODE_LINEAR 1 // G1
#define MOTION_MODE_CW_ARC 2  // G2
#define MOTION_MODE_CCW_ARC 3  // G3
#define MOTION_MODE_CANCEL 4 // G80

#define PROGRAM_FLOW_RUNNING 0
#define PROGRAM_FLOW_PAUSED 1 // M0, M1
#define PROGRAM_FLOW_COMPLETED 2 // M2, M30

#define NON_MODAL_NONE 0
#define NON_MODAL_DWELL 1 // G4
#define NON_MODAL_SET_COORDINATE_DATA 2 // G10
#define NON_MODAL_GO_HOME_0 3 // G28
#define NON_MODAL_SET_HOME_0 4 // G28.1
#define NON_MODAL_GO_HOME_1 5 // G30
#define NON_MODAL_SET_HOME_1 6 // G30.1
#define NON_MODAL_SET_COORDINATE_OFFSET 7 // G92
#define NON_MODAL_RESET_COORDINATE_OFFSET 8 //G92.1

#define COOLANT_MIST_ENABLE 2   //M8
#define COOLANT_FLOOD_ENABLE 1  //M9
#define COOLANT_DISABLE 0 // Must be zero.

// Define system state bit map. The state variable primarily tracks the individual functions
// of Grbl to manage each without overlapping. It is also used as a messaging flag for
// critical events.
#define STATE_IDLE       0 // Must be zero.
#define STATE_INIT       1 // Initial power up state.
#define STATE_QUEUED     2 // Indicates buffered blocks, awaiting cycle start.
#define STATE_CYCLE      3 // Cycle is running
#define STATE_HOLD       4 // Executing feed hold
#define STATE_HOMING     5 // Performing homing cycle
#define STATE_ALARM      6 // In alarm state. Locks out all g-code processes. Allows settings access.
#define STATE_CHECK_MODE 7 // G-code check mode. Locks out planner and motion only.


typedef struct{
    char letter;
    float value;

}commandData;

// Bit field and masking macros
#define bit(n) (1 << n)
#define bit_true(x,mask) (x |= mask)
#define bit_false(x,mask) (x &= ~mask)
#define bit_toggle(x,mask) (x ^= mask)
#define bit_istrue(x,mask) ((x & mask) != 0)
#define bit_isfalse(x,mask) ((x & mask) == 0)
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))


/***************************************************************************
 *
 *  Functions
 *
 * ***************************************************************************/
void GcodeInit(void);
static void select_plane(char, char, char);
int ExecuteLine( char*);

void ftoa(float , char* );

 int faltb(float , float );
#ifdef	__cplusplus
}
#endif

#endif	/* GCODE_H */

