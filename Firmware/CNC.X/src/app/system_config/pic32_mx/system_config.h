/* 
 * File:   system_config.h
 * Author: Justin
 *
 * Created on May 24, 2014, 4:50 PM
 */
///*
// * File:   system_config.h
// * Author: Justin
// *
// * Created on May 22, 2014, 11:18 PM
// */
///*******************************************************************************
//  MPLAB Harmony System Configuration Header
//
//  File Name:
//    system_config.h
//
//  Summary:
//    Build-time configuration header for the system defined by this MPLAB Harmony
//    project.
//
//  Description:
//    An MPLAB Project may have multiple configurations.  This file defines the
//    build-time options for a single configuration.
//
//    TODO: 015. PIC32MX - Describe the purpose and properties of this
//    configuration.
//
//  Remarks:
//    This configuration header must not define any prototypes or data
//    definitions (or include any files that do).  It only provides macro
//    definitions for build-time configuration options that are not instantiated
//    until used by another MPLAB Harmony module or application.
//*******************************************************************************/
//
//// DOM-IGNORE-BEGIN
///*******************************************************************************
//Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.
//
//Microchip licenses to you the right to use, modify, copy and distribute
//Software only when embedded on a Microchip microcontroller or digital signal
//controller that is integrated into your product or third party product
//(pursuant to the sublicense terms in the accompanying license agreement).
//
//You should refer to the license agreement accompanying this Software for
//additional information regarding your rights and obligations.
//
//SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
//EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
//MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
//IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
//CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
//OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
//INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
//CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
//SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
//(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
//*******************************************************************************/
//// DOM-IGNORE-END
//
#ifndef _SYS_CONFIG_H
#define _SYS_CONFIG_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
/*  This section Includes other configuration headers necessary to completely
    define this configuration.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Configuration
// *****************************************************************************
// *****************************************************************************

/* TODO: 020. PIC32MX - Define any required build-time configuration options
 * for the application.
 */
/* CLK System Service Configuration */
// SYSCLK = 80 MHz (8MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
// SYSCLK = 80 MHz (8MHz Crystal/ DIV_12   * MUL_20  / DIV_2)

#define SYS_CLK_CONFIG_PRIMARY_XTAL         (8000000L)    // 8 MHz
#define SYS_CLK_CONFIG_SECONDARY_XTAL            0           // Unused
#define SYS_CLK_CONFIG_SYSPLL_INP_DIVISOR        2
#define SYS_CLK_CONFIGBIT_USBPLL_ENABLE     false
#define SYS_CLK_CONFIGBIT_USBPLL_DIVISOR         2
#define SYS_CLK_CONFIG_FREQ_ERROR_LIMIT         10
#define SYS_CLK_CONFIG_PLL_MULTIPLIER           20
#define SYS_CLK_CONFIG_PLL_INPUT_DIVIDE          2
#define SYS_CLK_CONFIG_PLL_OUTPUT_DIVIDE         1
#define PERIPHERAL_BUS_SYS_CLK_DIVIDE            2
#define FIN_CLK                             (SYS_CLK_CONFIG_SECONDARY_XTAL/SYS_CLK_CONFIG_PLL_INPUT_DIVIDE)
#define SYS_CLK_PRIMARY_CLOCK               ((FIN_CLK*SYS_CLK_CONFIG_PLL_MULTIPLIER)/SYS_CLK_CONFIG_PLL_OUTPUT_DIVIDE)   // 80 MHz
#define PERIPHERAL_BUS_CLOCK                (SYS_CLK_PRIMARY_CLOCK/PERIPHERAL_BUS_SYS_CLK_DIVIDE)

#define SystemClock()                 (SYS_CLK_PRIMARY_CLOCK)
#define	GetSystemClock()              (80000000L)
#define	GetPeripheralClock()          (40000000L)
#define	GetInstructionClock()         (40000000L)
#define CORE_TICKS_PER_SECOND         (40000000L)                  //  40,000,000
#define CORE_TICKS_PER_MILLISECOND    (40,000)               //      40,000
#define CORE_TICKS_PER_MICROSECOND    (40)            //          40
#define PBUS_TICKS_PER_SECOND         (40,000,000)                   //  40,000,000
#define PBUS_TICKS_PER_MILLISECOND    (40,000)                //      40,000
#define PBUS_TICKS_PER_MICROSECOND    (40)             //           40

#define DESIRED_BAUDRATE              (57600)      //The desired BaudRate

// Default settings. Used when resetting EEPROM. Change to desired name in defaults.h
#define DEFAULTS_GENERIC

#define Null    ((void *)0)


// Define runtime command special characters. These characters are 'picked-off' directly from the
// serial read data stream and are not passed to the grbl line execution parser. Select characters
// that do not and must not exist in the streamed g-code program. ASCII control characters may be
// used, if they are available per user setup. Also, extended ASCII codes (>127), which are never in
// g-code programs, maybe selected for interface programs.
// NOTE: If changed, manually update help message in report.c.
#define CMD_STATUS_REPORT '?'
#define CMD_FEED_HOLD '!'
#define CMD_CYCLE_START '~'
#define CMD_RESET 0x18 // ctrl-x

// The temporal resolution of the acceleration management subsystem. Higher number give smoother
// acceleration but may impact performance.
// NOTE: Increasing this parameter will help any resolution related issues, especially with machines
// requiring very high accelerations and/or very fast feedrates. In general, this will reduce the
// error between how the planner plans the motions and how the stepper program actually performs them.
// However, at some point, the resolution can be high enough, where the errors related to numerical
// round-off can be great enough to cause problems and/or it's too fast for the Arduino. The correct
// value for this parameter is machine dependent, so it's advised to set this only as high as needed.
// Approximate successful values can range from 30L to 100L or more.
#define ACCELERATION_TICKS_PER_SECOND 50L

// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end
// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
#define MINIMUM_PLANNER_SPEED 0.0 // (mm/min)

// Minimum stepper rate. Sets the absolute minimum stepper rate in the stepper program and never runs
// slower than this value, except when sleeping. This parameter overrides the minimum planner speed.
// This is primarily used to guarantee that the end of a movement is always reached and not stop to
// never reach its target. This parameter should always be greater than zero.
#define MINIMUM_STEPS_PER_MINUTE 10 // (steps/min) - Integer value only

// Time delay increments performed during a dwell. The default value is set at 50ms, which provides
// a maximum time delay of roughly 55 minutes, more than enough for most any application. Increasing
// this delay will increase the maximum dwell time linearly, but also reduces the responsiveness of
// run-time command executions, like status reports, since these are performed between each dwell
// time step. Also, keep in mind that the Arduino delay timer is not very accurate for long delays.
#define DWELL_TIME_STEP 50 // Integer (1-255) (milliseconds)

// If homing is enabled, homing init lock sets Grbl into an alarm state upon power up. This forces
// the user to perform the homing cycle (or override the locks) before doing anything else. This is
// mainly a safety feature to remind the user to home, since position is unknown to Grbl.
#define HOMING_INIT_LOCK // Comment to disable

// The homing cycle seek and feed rates will adjust so all axes independently move at the homing
// seek and feed rates regardless of how many axes are in motion simultaneously. If disabled, rates
// are point-to-point rates, as done in normal operation. For example in an XY diagonal motion, the
// diagonal motion moves at the intended rate, but the individual axes move at 70% speed. This option
// just moves them all at 100% speed.
#define HOMING_RATE_ADJUST // Comment to disable

// Define the homing cycle search patterns with bitmasks. The homing cycle first performs a search
// to engage the limit switches. HOMING_SEARCH_CYCLE_x are executed in order starting with suffix 0
// and searches the enabled axes in the bitmask. This allows for users with non-standard cartesian
// machines, such as a lathe (x then z), to configure the homing cycle behavior to their needs.
// Search cycle 0 is required, but cycles 1 and 2 are both optional and may be commented to disable.
// After the search cycle, homing then performs a series of locating about the limit switches to hone
// in on machine zero, followed by a pull-off maneuver. HOMING_LOCATE_CYCLE governs these final moves,
// and this mask must contain all axes in the search.
// NOTE: Later versions may have this installed in settings.
#define HOMING_SEARCH_CYCLE_0 (1<<Z_AXIS)                // First move Z to clear workspace.
#define HOMING_SEARCH_CYCLE_1 ((1<<X_AXIS)|(1<<Y_AXIS))  // Then move X,Y at the same time.
// #define HOMING_SEARCH_CYCLE_2                         // Uncomment and add axes mask to enable
#define HOMING_LOCATE_CYCLE   ((1<<X_AXIS)|(1<<Y_AXIS)|(1<<Z_AXIS)) // Must contain ALL search axes

// Number of homing cycles performed after when the machine initially jogs to limit switches.
// This help in preventing overshoot and should improve repeatability. This value should be one or
// greater.
#define N_HOMING_LOCATE_CYCLE 2 // Integer (1-128)

// Number of blocks Grbl executes upon startup. These blocks are stored in EEPROM, where the size
// and addresses are defined in settings.h. With the current settings, up to 5 startup blocks may
// be stored and executed in order. These startup blocks would typically be used to set the g-code
// parser state depending on user preferences.
#define N_STARTUP_LINE 2 // Integer (1-5)

// ---------------------------------------------------------------------------------------
// FOR ADVANCED USERS ONLY:

// The number of linear motions in the planner buffer to be planned at any give time. The vast
// majority of RAM that Grbl uses is based on this buffer size. Only increase if there is extra
// available RAM, like when re-compiling for a Teensy or Sanguino. Or decrease if the Arduino
// begins to crash due to the lack of available RAM or if the CPU is having trouble keeping
// up with planning new incoming motions as they are executed.
// #define BLOCK_BUFFER_SIZE 18  // Uncomment to override default in planner.h.

// Line buffer size from the serial input stream to be executed. Also, governs the size of
// each of the startup blocks, as they are each stored as a string of this size. Make sure
// to account for the available EEPROM at the defined memory address in settings.h and for
// the number of desired startup blocks.
// NOTE: 70 characters is not a problem except for extreme cases, but the line buffer size
// can be too small and g-code blocks can get truncated. Officially, the g-code standards
// support up to 256 characters. In future versions, this default will be increased, when
// we know how much extra memory space we can re-invest into this.
// #define LINE_BUFFER_SIZE 70  // Uncomment to override default in protocol.h

// Serial send and receive buffer size. The receive buffer is often used as another streaming
// buffer to store incoming blocks to be processed by Grbl when its ready. Most streaming
// interfaces will character count and track each block send to each block response. So,
// increase the receive buffer if a deeper receive buffer is needed for streaming and avaiable
// memory allows. The send buffer primarily handles messages in Grbl. Only increase if large
// messages are sent and Grbl begins to stall, waiting to send the rest of the message.
// #define RX_BUFFER_SIZE 128 // Uncomment to override defaults in serial.h
// #define TX_BUFFER_SIZE 64

// Toggles XON/XOFF software flow control for serial communications. Not officially supported
// due to problems involving the Atmega8U2 USB-to-serial chips on current Arduinos. The firmware
// on these chips do not support XON/XOFF flow control characters and the intermediate buffer
// in the chips cause latency and overflow problems with standard terminal programs. However,
// using specifically-programmed UI's to manage this latency problem has been confirmed to work.
// As well as, older FTDI FT232RL-based Arduinos(Duemilanove) are known to work with standard
// terminal programs since their firmware correctly manage these XON/XOFF characters. In any
// case, please report any successes to grbl administrators!
// #define ENABLE_XONXOFF // Default disabled. Uncomment to enable.

// Creates a delay between the direction pin setting and corresponding step pulse by creating
// another interrupt (Timer2 compare) to manage it. The main Grbl interrupt (Timer1 compare)
// sets the direction pins, and does not immediately set the stepper pins, as it would in
// normal operation. The Timer2 compare fires next to set the stepper pins after the step
// pulse delay time, and Timer2 overflow will complete the step pulse, except now delayed
// by the step pulse time plus the step pulse delay. (Thanks langwadt for the idea!)
//   This is an experimental feature that should only be used if your setup requires a longer
// delay between direction and step pin settings (some opto coupler based drivers), as it may
// adversely effect Grbl's high-end performance (>10kHz). Please notify Grbl administrators
// of your successes or difficulties, as we will monitor this and possibly integrate this as a
// standard feature for future releases. However, we suggest to first try our direction delay
// hack/solution posted in the Wiki involving inverting the stepper pin mask.
// NOTE: Uncomment to enable. The recommended delay must be > 3us and the total step pulse
// time, which includes the Grbl settings pulse microseconds, must not exceed 127us. Reported
// successful values for certain setups have ranged from 10 to 20us.
// #define STEP_PULSE_DELAY 10 // Step pulse delay in microseconds. Default disabled.

// Uncomment the following define if you are using hardware that drives high when your limits
// are reached. You will need to ensure that you have appropriate pull-down resistors on the
// limit switch input pins, or that your hardware drives the pins low when they are open (non-
// triggered).
// #define LIMIT_SWITCHES_ACTIVE_HIGH

// ---------------------------------------------------------------------------------------

// TODO: Install compile-time option to send numeric status codes rather than strings.

// *****************************************************************************
// *****************************************************************************
// Section: Middleware Configuration
// *****************************************************************************
// *****************************************************************************




/* TODO: 022. PIC32MX - Define any required build-time configuration options
 * for any middleware libraries used by the system.  Descriptions of the
 * supported options are available in the "Configuring the Library" section in
 * the help document for every MPLAB Harmony library.
 */

// *****************************************************************************
// *****************************************************************************
// Section: Driver Configuration
// *****************************************************************************
// *****************************************************************************

/* TODO: 024. PIC32MX - Define any required build-time configuration options
 * for any driver libraries used by the system.  Descriptions of the supported
 * options are available in the "Configuring the Library" section in the help
 * document for every MPLAB Harmony library.
 */

// *****************************************************************************
// *****************************************************************************
// Section: System Service Configuration
// *****************************************************************************
// *****************************************************************************

/* TODO: 026. PIC32MX - Define any required build-time configuration options for
 * any system service libraries used by the system.  Descriptions of the
 * supported options are available in the "Configuring the Library" section in
 * the help document for every MPLAB Harmony library.
 */

#endif // _SYS_CONFIG_H
/*******************************************************************************
 End of File
*/
