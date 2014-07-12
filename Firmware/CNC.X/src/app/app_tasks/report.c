


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


#include "report.h"
#include "bsp_config.h"
#include "settings.h"
#include "nuts_bolts.h"
#include "g_code.h"
//#include "coolant_control.h"



// Handles the primary confirmation protocol response for streaming interfaces and human-feedback.
// For every incoming line, this method responds with an 'ok' for a successful command or an
// 'error:'  to indicate some error event with the line or some critical system error during
// operation. Errors events can originate from the g-code parser, settings module, or asynchronously
// from a critical error, such as a triggered hard limit. Interface should always monitor for these
// responses.
// NOTE: In silent mode, all error codes are greater than zero.
// TODO: Install silent mode to return only numeric values, primarily for GUIs.
void report_status_message(uint8_t status_code)
{
  if (status_code == 0) { // STATUS_OK
    BSP_WriteString("ok\r\n");
  } else {
    BSP_WriteString("error: ");
    switch(status_code) {
      case STATUS_BAD_NUMBER_FORMAT:
      BSP_WriteString("Bad number format"); break;
      case STATUS_EXPECTED_COMMAND_LETTER:
      BSP_WriteString("Expected command letter"); break;
      case STATUS_UNSUPPORTED_STATEMENT:
      BSP_WriteString("Unsupported statement"); break;
      case STATUS_ARC_RADIUS_ERROR:
      BSP_WriteString("Invalid radius"); break;
      case STATUS_MODAL_GROUP_VIOLATION:
      BSP_WriteString("Modal group violation"); break;
      case STATUS_INVALID_STATEMENT:
      BSP_WriteString("Invalid statement"); break;
      case STATUS_SETTING_DISABLED:
      BSP_WriteString("Setting disabled"); break;
      case STATUS_SETTING_VALUE_NEG:
      BSP_WriteString("Value < 0.0"); break;
      case STATUS_SETTING_STEP_PULSE_MIN:
      BSP_WriteString("Value < 3 usec"); break;
      case STATUS_SETTING_READ_FAIL:
      BSP_WriteString("EEPROM read fail. Using defaults"); break;
      case STATUS_IDLE_ERROR:
      BSP_WriteString("Busy or queued"); break;
      case STATUS_ALARM_LOCK:
      BSP_WriteString("Alarm lock"); break;
      case STATUS_OVERFLOW:
      BSP_WriteString("Line overflow"); break;
    }
    BSP_WriteString("\r\n");
  }
}

// Prints alarm messages.
void report_alarm_message(int8_t alarm_code)
{
  BSP_WriteString("ALARM: ");
  switch (alarm_code) {
    case ALARM_HARD_LIMIT:
    BSP_WriteString("Hard limit"); break;
    case ALARM_ABORT_CYCLE:
    BSP_WriteString("Abort during cycle"); break;
  }
  BSP_WriteString(". MPos?\r\n");
  //delay_ms(500); // Force delay to ensure message clears serial write buffer.
}

// Prints feedback messages. This serves as a centralized method to provide additional
// user feedback for things that are not of the status/alarm message protocol. These are
// messages such as setup warnings, switch toggling, and how to exit alarms.
// NOTE: For interfaces, messages are always placed within brackets. And if silent mode
// is installed, the message number codes are less than zero.
// TODO: Install silence feedback messages option in settings
void report_feedback_message(uint8_t message_code)
{
  BSP_WriteString("[");
  switch(message_code) {
    case MESSAGE_CRITICAL_EVENT:
    BSP_WriteString("Reset to continue"); break;
    case MESSAGE_ALARM_LOCK:
    BSP_WriteString("'$H'|'$X' to unlock"); break;
    case MESSAGE_ALARM_UNLOCK:
    BSP_WriteString("Caution: Unlocked"); break;
    case MESSAGE_ENABLED:
    BSP_WriteString("Enabled"); break;
    case MESSAGE_DISABLED:
    BSP_WriteString("Disabled"); break;
  }
  BSP_WriteString("]\r\n");
}


// Welcome message
void report_init_message()
{
  BSP_WriteString("\r\nGrbl " GRBL_VERSION " ['$' for help]\r\n");
}

// Grbl help message
void report_grbl_help() {
  BSP_WriteString("$$ (view Grbl settings)\r\n"
                      "$# (view # parameters)\r\n"
                      "$G (view parser state)\r\n"
                      "$N (view startup blocks)\r\n"
                      "$x=value (save Grbl setting)\r\n"
                      "$Nx=line (save startup block)\r\n"
                      "$C (check gcode mode)\r\n"
                      "$X (kill alarm lock)\r\n"
                      "$H (run homing cycle)\r\n"
                      "~ (cycle start)\r\n"
                      "! (feed hold)\r\n"
                      "? (current status)\r\n"
                      "ctrl-x (reset Grbl)\r\n");
}

// Grbl global settings print out.
// NOTE: The numbering scheme here must correlate to storing in settings.c
void report_grbl_settings() {
  BSP_WriteString("$0="); printFloat(settings.steps_per_mm[X_AXIS]);
  BSP_WriteString(" (x, step/mm)\r\n$1="); printFloat(settings.steps_per_mm[Y_AXIS]);
  BSP_WriteString(" (y, step/mm)\r\n$2="); printFloat(settings.steps_per_mm[Z_AXIS]);
  BSP_WriteString(" (z, step/mm)\r\n$3="); printInteger(settings.pulse_microseconds);
  BSP_WriteString(" (step pulse, usec)\r\n$4="); printFloat(settings.default_feed_rate);
  BSP_WriteString(" (default feed, mm/min)\r\n$5="); printFloat(settings.default_seek_rate);
  BSP_WriteString(" (default seek, mm/min)\r\n$6="); printInteger(settings.invert_mask);
  BSP_WriteString(" (step port invert mask, int:"); print_uint8_base2(settings.invert_mask);
  BSP_WriteString(")\r\n$7="); printInteger(settings.stepper_idle_lock_time);
  BSP_WriteString(" (step idle delay, msec)\r\n$8="); printFloat(settings.acceleration/(60*60)); // Convert from mm/min^2 for human readability
  BSP_WriteString(" (acceleration, mm/sec^2)\r\n$9="); printFloat(settings.junction_deviation);
  BSP_WriteString(" (junction deviation, mm)\r\n$10="); printFloat(settings.mm_per_arc_segment);
  BSP_WriteString(" (arc, mm/segment)\r\n$11="); printInteger(settings.n_arc_correction);
  BSP_WriteString(" (n-arc correction, int)\r\n$12="); printInteger(settings.decimal_places);
  BSP_WriteString(" (n-decimals, int)\r\n$13="); printInteger(bit_istrue(settings.flags,BITFLAG_REPORT_INCHES));
  BSP_WriteString(" (report inches, bool)\r\n$14="); printInteger(bit_istrue(settings.flags,BITFLAG_AUTO_START));
  BSP_WriteString(" (auto start, bool)\r\n$15="); printInteger(bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE));
  BSP_WriteString(" (invert step enable, bool)\r\n$16="); printInteger(bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE));
  BSP_WriteString(" (hard limits, bool)\r\n$17="); printInteger(bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE));
  BSP_WriteString(" (homing cycle, bool)\r\n$18="); printInteger(settings.homing_dir_mask);
  BSP_WriteString(" (homing dir invert mask, int:"); print_uint8_base2(settings.homing_dir_mask);
  BSP_WriteString(")\r\n$19="); printFloat(settings.homing_feed_rate);
  BSP_WriteString(" (homing feed, mm/min)\r\n$20="); printFloat(settings.homing_seek_rate);
  BSP_WriteString(" (homing seek, mm/min)\r\n$21="); printInteger(settings.homing_debounce_delay);
  BSP_WriteString(" (homing debounce, msec)\r\n$22="); printFloat(settings.homing_pulloff);
  BSP_WriteString(" (homing pull-off, mm)\r\n");
}

void print_uint8_base2(uint8_t n)
{
	unsigned char buf[8];
	uint8_t i = 0;

	for (; i < 8; i++) {
		buf[i] = n & 1;
		n >>= 1;
	}

	for (; i > 0; i--)
		BSP_PutCharacter('0' + buf[i - 1]);
}

static void print_uint32_base10(unsigned long n)
{
  unsigned char buf[10];
  uint8_t i = 0;

  if (n == 0) {
    BSP_PutCharacter('0');
    return;
  }

  while (n > 0) {
    buf[i++] = n % 10 + '0';
    n /= 10;
  }

  for (; i > 0; i--)
    BSP_PutCharacter(buf[i-1]);
}

void printInteger(long n)
{
  if (n < 0) {
    BSP_PutCharacter('-');
    n = -n;
  }
  print_uint32_base10(n);
}

// Convert float to string by immediately converting to a long integer, which contains
// more digits than a float. Number of decimal places, which are tracked by a counter,
// may be set by the user. The integer is then efficiently converted to a string.
// NOTE: AVR '%' and '/' integer operations are very efficient. Bitshifting speed-up
// techniques are actually just slightly slower. Found this out the hard way.
void printFloat(float n)
{
  if (n < 0) {
    BSP_PutCharacter('-');
    n = -n;
  }

  uint8_t decimals = settings.decimal_places;
  while (decimals >= 2) { // Quickly convert values expected to be E0 to E-4.
    n *= 100;
    decimals -= 2;
  }
  if (decimals) { n *= 10; }
  n += 0.5; // Add rounding factor. Ensures carryover through entire value.

  // Generate digits backwards and store in string.
  unsigned char buf[10];
  uint8_t i = 0;
  uint32_t a = (long)n;
  buf[settings.decimal_places] = '.'; // Place decimal point, even if decimal places are zero.
  while(a > 0) {
    if (i == settings.decimal_places) { i++; } // Skip decimal point location
    buf[i++] = (a % 10) + '0'; // Get digit
    a /= 10;
  }
  while (i < settings.decimal_places) {
     buf[i++] = '0'; // Fill in zeros to decimal point for (n < 1)
  }
  if (i == settings.decimal_places) { // Fill in leading zero, if needed.
    i++;
    buf[i++] = '0';
  }

  // Print the generated string.
  for (; i > 0; i--)
    BSP_PutCharacter(buf[i-1]);
}
