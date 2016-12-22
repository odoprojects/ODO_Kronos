/*
 * Eeprom.h
 *
 * Created: 29.08.2016 17:39:28
 *  Author: pieru
 */ 


#ifndef EEPROM_H_
#define EEPROM_H_


#define EEPROM_OFFSET				0
#define EEPROM_PROTOCOL_VERSION		2
#define EEPROM_MODE					1

#define EEPROM_OFFSET_TENS_MANAGER	100

#define EPR_MAGIC_uint8_t              0
#define EPR_BAUDRATE				1	//4bajty
#define EPR_INTEGRITY_uint8_t			5   //1bajt Here the xored sum over eeprom is stored
#define EPR_VERSION					6   //1bajt Version id for updates in EEPROM storage

//***************************************
#define EEPROM_TENS0_START			0
#define EPR_TENS0_CONSTATNT			EEPROM_TENS0_START+1+EEPROM_OFFSET_TENS_MANAGER //2bajty
#define EPR_TENS0_ZERO_SCALE_MASS	EEPROM_TENS0_START+3+EEPROM_OFFSET_TENS_MANAGER //4bajty
#define EPR_TENS0_ROLL_WEIGHT		EEPROM_TENS0_START+5+EEPROM_OFFSET_TENS_MANAGER //2bajty
#define EEPROM_TENS0_COUNT			8  //suma bajtow zajmowana przez zmienne dotyczace TENS0

//***************************************
//***************************************
#define EEPROM_TENS1_START			EEPROM_TENS0_COUNT+1
#define EPR_TENS1_CONSTATNT			EEPROM_TENS1_START+1+EEPROM_OFFSET_TENS_MANAGER //2bajty
#define EPR_TENS1_ZERO_SCALE_MASS	EEPROM_TENS1_START+3+EEPROM_OFFSET_TENS_MANAGER //4bajty
#define EPR_TENS1_ROLL_WEIGHT		EEPROM_TENS1_START+5+EEPROM_OFFSET_TENS_MANAGER //2bajty
#define EEPROM_TENS1_COUNT			8  //suma bajtow zajmowana przez zmienne dotyczace TENS1
//***************************************

void eprSetuint8_t(unsigned int pos,uint8_t value);
void eprSetInt16(unsigned int pos,int16_t value);
void eprSetInt32(unsigned int pos,int32_t value);
void eprSetFloat(unsigned int pos,float value);
uint8_t eprGetuint8_t(unsigned int pos);
int16_t eprGetInt16(unsigned int pos);
int32_t eprGetInt32(unsigned int pos);
float eprGetFloat(unsigned int pos);

void initBaudrate();

uint8_t get_epr_version();
void readDataFromEEPROM();
void restoreEEPROMSettingsFromConfiguration();
void load_default_variables();
void storeDataIntoEEPROM();

#endif /* EEPROM_H_ */