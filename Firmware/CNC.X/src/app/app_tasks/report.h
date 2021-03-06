/* 
 * File:   report.h
 * Author: Justin
 *
 * Created on June 23, 2014, 11:13 PM
 */

#ifndef REPORT_H
#define	REPORT_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdint.h>
    
// Define Grbl status codes.
#define STATUS_OK 0
#define STATUS_BAD_NUMBER_FORMAT 1
#define STATUS_EXPECTED_COMMAND_LETTER 2
#define STATUS_UNSUPPORTED_STATEMENT 3
#define STATUS_ARC_RADIUS_ERROR 4
#define STATUS_MODAL_GROUP_VIOLATION 5
#define STATUS_INVALID_STATEMENT 6
#define STATUS_SETTING_DISABLED 7
#define STATUS_SETTING_VALUE_NEG 8
#define STATUS_SETTING_STEP_PULSE_MIN 9
#define STATUS_SETTING_READ_FAIL 10
#define STATUS_IDLE_ERROR 11
#define STATUS_ALARM_LOCK 12
#define STATUS_OVERFLOW 13

// Define Grbl alarm codes. Less than zero to distinguish alarm error from status error.
#define ALARM_HARD_LIMIT -1
#define ALARM_ABORT_CYCLE -2

// Define Grbl feedback message codes.
#define MESSAGE_CRITICAL_EVENT 1
#define MESSAGE_ALARM_LOCK 2
#define MESSAGE_ALARM_UNLOCK 3
#define MESSAGE_ENABLED 4
#define MESSAGE_DISABLED 5

void printString(const char *s);

void printPgmString(const char *s);

void printInteger(long n);

void print_uint8_base2(uint8_t n);

void printFloat(float n);

// Prints system status messages.
void report_status_message(uint8_t status_code);

// Prints system alarm messages.
void report_alarm_message(int8_t alarm_code);

// Prints miscellaneous feedback messages.
void report_feedback_message(uint8_t message_code);

// Prints welcome message
void report_init_message();

// Prints Grbl help and current global settings
void report_grbl_help();

// Prints Grbl global settings
void report_grbl_settings();

// Prints realtime status report
void report_realtime_status();

// Prints Grbl persistent coordinate parameters
void report_gcode_parameters();

// Prints current g-code parser mode state
void report_gcode_modes();

// Prints startup line
void report_startup_line(uint8_t n, char *line);

#ifdef	__cplusplus
}
#endif

#endif	/* REPORT_H */

