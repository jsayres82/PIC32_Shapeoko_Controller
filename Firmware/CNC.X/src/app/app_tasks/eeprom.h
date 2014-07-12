/* 
 * File:   eeprom.h
 * Author: Justin
 *
 * Created on June 24, 2014, 1:07 AM
 */

#ifndef EEPROM_H
#define	EEPROM_H

#ifdef	__cplusplus
extern "C" {
#endif


char eeprom_get_char(unsigned int addr);
void eeprom_put_char( unsigned int addr, unsigned char new_value );
void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size);
int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size);



#ifdef	__cplusplus
}
#endif

#endif	/* EEPROM_H */

