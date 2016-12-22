/*
 * Eeprom.c
 *
 * Created: 29.08.2016 17:39:07
 *  Author: pieru
 */ 
#include <avr/eeprom.h>
#include "Eeprom.h"
#include "hardware.h"
#include "temperature_control.h"
#include "Configuration.h"
#include "tensometer.h"

//extern struct TemperatureController tempController[ANALOG_INPUTS];


void eprSetuint8_t(unsigned int pos,uint8_t value)
{
	eeprom_write_byte((unsigned char *)(EEPROM_OFFSET + pos), value);
}
void eprSetInt16(unsigned int pos,int16_t value)
{
	eeprom_write_word((unsigned int*)(EEPROM_OFFSET + pos),value);
}
void eprSetInt32(unsigned int pos,int32_t value)
{
	eeprom_write_dword((uint32_t*)(EEPROM_OFFSET + pos),value);
}
void eprSetFloat(unsigned int pos,float value)
{
	eeprom_write_block(&value,(void*)(EEPROM_OFFSET + pos), 4);
}
uint8_t eprGetuint8_t(unsigned int pos)
{
	return eeprom_read_byte ((unsigned char *)(EEPROM_OFFSET + pos));
}
int16_t eprGetInt16(unsigned int pos)
{
	return eeprom_read_word((uint16_t *)(EEPROM_OFFSET + pos));
}
int32_t eprGetInt32(unsigned int pos)
{
	return eeprom_read_dword((uint32_t*)(EEPROM_OFFSET + pos));
}
float eprGetFloat(unsigned int pos)
{
	float v;
	eeprom_read_block(&v,(void *)(EEPROM_OFFSET + pos),4); // newer gcc have eeprom_read_block but not arduino 22
	return v;
}

uint8_t computeChecksum()
{
	unsigned int i;
	uint8_t checksum = 0;
	for(i = 0; i < 2048; i++)
	{
		if(i == EPR_INTEGRITY_uint8_t) continue;
		checksum += eprGetuint8_t(i);
	}
	return checksum;
}

void updateChecksum()
{
	uint8_t newcheck = computeChecksum();
	if(newcheck!=eprGetuint8_t(EPR_INTEGRITY_uint8_t))
	eprSetuint8_t(EPR_INTEGRITY_uint8_t,newcheck);
}

void initBaudrate()
{
	// Invariant - baudrate is intitalized with or without eeprom!
	baudrate = BAUDRATE;
	if(eprGetuint8_t(EPR_MAGIC_uint8_t) == EEPROM_MODE)
	{
		baudrate = eprGetInt32(EPR_BAUDRATE);
	}
}


void storeDataIntoEEPROM(){
	
	eprSetuint8_t(EPR_VERSION, EEPROM_PROTOCOL_VERSION);
	eprSetInt32(EPR_BAUDRATE,baudrate);
	
	eprSetInt16(EPR_TENS0_CONSTATNT, tensometer[TENSOMETER_0].constantCalibrationValue );
	eprSetInt32(EPR_TENS0_ZERO_SCALE_MASS, tensometer[TENSOMETER_0].zeroScaleMass );
	eprSetInt16(EPR_TENS0_ROLL_WEIGHT, tensometer[TENSOMETER_0].rollWeight );
	
	eprSetInt16(EPR_TENS1_CONSTATNT, tensometer[TENSOMETER_1].constantCalibrationValue );
	eprSetInt32(EPR_TENS1_ZERO_SCALE_MASS, tensometer[TENSOMETER_1].zeroScaleMass );
	eprSetInt16(EPR_TENS1_ROLL_WEIGHT, tensometer[TENSOMETER_1].rollWeight );
	
	eprSetuint8_t(EPR_INTEGRITY_uint8_t, computeChecksum());
	
}

void restoreEEPROMSettingsFromConfiguration()
{
	load_default_variables();
	storeDataIntoEEPROM();
}

void readDataFromEEPROM()
{
	uint8_t newcheck = computeChecksum();
	if(newcheck != eprGetuint8_t(EPR_INTEGRITY_uint8_t))
		restoreEEPROMSettingsFromConfiguration();
	newcheck = eprGetuint8_t(EPR_VERSION);
	if(EEPROM_PROTOCOL_VERSION != get_epr_version())
		restoreEEPROMSettingsFromConfiguration();
	
	//baudrate = eprGetInt32(EPR_BAUDRATE);
	
	tensometer[TENSOMETER_0].constantCalibrationValue = eprGetInt16(EPR_TENS0_CONSTATNT);
	tensometer[TENSOMETER_0].zeroScaleMass = eprGetInt32(EPR_TENS0_ZERO_SCALE_MASS);
	tensometer[TENSOMETER_0].rollWeight = eprGetInt16(EPR_TENS0_ROLL_WEIGHT);
	
	tensometer[TENSOMETER_1].constantCalibrationValue = eprGetInt16(EPR_TENS1_CONSTATNT);
	tensometer[TENSOMETER_1].zeroScaleMass = eprGetInt32(EPR_TENS1_ZERO_SCALE_MASS);
	tensometer[TENSOMETER_1].rollWeight = eprGetInt16(EPR_TENS1_ROLL_WEIGHT);
}

uint8_t get_epr_version(){
	return eprGetuint8_t(EPR_VERSION);
}

void load_default_variables(){
	tempController[EXT0].heatManager = EXT0_HEAT_MANAGER;
	tempController[EXT0].sensorPin = EXT0_ANALOG_INPUT;
	tempController[EXT0].pidDriveMax = EXT0_DRIVE_MAX;
	tempController[EXT0].pidPGain = EXT0_PID_PGAIN;
	tempController[EXT0].pidIGain = EXT0_PID_IGAIN;
	tempController[EXT0].pidDGain = EXT0_PID_DGAIN;
	tempController[EXT0].pidMax = EXT0_PID_MAX;
	tempController[EXT0].pidDriveMin = EXT0_DRIVE_MIN;
	tempController[EXT0].sensorType = DEF_SENSOR_TYPE;//SENSOR_TYPE_TT2_231KC6_1;
	fan[EXT0].coolerSpeed = EXT0_EXTRUDER_COOLER_SPEED;
	
	tempController[EXT1].heatManager = EXT1_HEAT_MANAGER;
	tempController[EXT1].sensorPin = EXT1_ANALOG_INPUT;
	tempController[EXT1].pidDriveMax = EXT1_DRIVE_MAX;
	tempController[EXT1].pidPGain = EXT1_PID_PGAIN;
	tempController[EXT1].pidIGain = EXT1_PID_IGAIN;
	tempController[EXT1].pidDGain = EXT1_PID_DGAIN;
	tempController[EXT1].pidMax = EXT1_PID_MAX;
	tempController[EXT1].pidDriveMin = EXT1_DRIVE_MIN;
	tempController[EXT1].sensorType = SENSOR_TYPE_TT2_231KC6_1;
	fan[EXT1].coolerSpeed = EXT1_EXTRUDER_COOLER_SPEED;
	
	tempController[BED0].heatManager = BED0_HEAT_MANAGER;
	tempController[BED0].sensorPin = BED0_ANALOG_INPUT;
	tempController[BED0].pidDriveMax = BED0_DRIVE_MAX;
	tempController[BED0].pidPGain = BED0_PID_PGAIN;
	tempController[BED0].pidIGain = BED0_PID_IGAIN;
	tempController[BED0].pidDGain = BED0_PID_DGAIN;
	tempController[BED0].pidMax = BED0_PID_MAX;
	tempController[BED0].pidDriveMin = BED0_DRIVE_MIN;
	tempController[BED0].sensorType = SENSOR_TYPE_TT2_231KC6_1;
	
	tempController[BED1].heatManager = BED1_HEAT_MANAGER;
	tempController[BED1].sensorPin = BED1_ANALOG_INPUT;
	tempController[BED1].pidDriveMax = BED1_DRIVE_MAX;
	tempController[BED1].pidPGain = BED1_PID_PGAIN;
	tempController[BED1].pidIGain = BED1_PID_IGAIN;
	tempController[BED1].pidDGain = BED1_PID_DGAIN;
	tempController[BED1].pidMax = BED1_PID_MAX;
	tempController[BED1].pidDriveMin = BED1_DRIVE_MIN;
	tempController[BED1].sensorType = SENSOR_TYPE_TT2_231KC6_1;
	
	tempController[BED2].heatManager = BED2_HEAT_MANAGER;
	tempController[BED2].sensorPin = BED2_ANALOG_INPUT;
	tempController[BED2].pidDriveMax = BED2_DRIVE_MAX;
	tempController[BED2].pidPGain = BED2_PID_PGAIN;
	tempController[BED2].pidIGain = BED2_PID_IGAIN;
	tempController[BED2].pidDGain = BED2_PID_DGAIN;
	tempController[BED2].pidMax = BED2_PID_MAX;
	tempController[BED2].pidDriveMin = BED2_DRIVE_MIN;
	tempController[BED2].sensorType = SENSOR_TYPE_TT2_231KC6_1;
	
	tempController[BED3].heatManager = BED3_HEAT_MANAGER;
	tempController[BED3].sensorPin = BED3_ANALOG_INPUT;
	tempController[BED3].pidDriveMax = BED3_DRIVE_MAX;
	tempController[BED3].pidPGain = BED3_PID_PGAIN;
	tempController[BED3].pidIGain = BED3_PID_IGAIN;
	tempController[BED3].pidDGain = BED3_PID_DGAIN;
	tempController[BED3].pidMax = BED3_PID_MAX;
	tempController[BED3].pidDriveMin = BED3_DRIVE_MIN;
	tempController[BED3].sensorType = SENSOR_TYPE_TT2_231KC6_1;
	
	tempController[CHAMB0].heatManager = CHAMB0_HEAT_MANAGER;
	tempController[CHAMB0].sensorPin = CHAMBER0_ANALOG_INPUT;
	tempController[CHAMB0].pidDriveMax = CHAMB0_DRIVE_MAX;
	tempController[CHAMB0].pidPGain = CHAMB0_PID_PGAIN;
	tempController[CHAMB0].pidIGain = CHAMB0_PID_IGAIN;
	tempController[CHAMB0].pidDGain = CHAMB0_PID_DGAIN;
	tempController[CHAMB0].pidMax = CHAMB0_PID_MAX;
	tempController[CHAMB0].pidDriveMin = CHAMB0_DRIVE_MIN;
	tempController[CHAMB0].sensorType = SENSOR_TYPE_TT2_231KC6_1;
}