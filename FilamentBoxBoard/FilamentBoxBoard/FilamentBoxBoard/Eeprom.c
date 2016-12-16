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

//extern struct TemperatureController tempController[ANALOG_INPUTS];


void eprSetuint8_t(unsigned int pos,uint8_t value)
{
	eeprom_write_uint8_t((unsigned char *)(EEPROM_OFFSET + pos), value);
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
	return eeprom_read_uint8_t ((unsigned char *)(EEPROM_OFFSET + pos));
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
	
	eprSetuint8_t(EPR_EXT0_HEAT_MANAGER, tempController[EXT0].heatManager );
	eprSetuint8_t(EPR_EXT0_ANALOG_PIN, tempController[EXT0].sensorPin);
	eprSetuint8_t(EPR_EXT0_DRIVE_MAX, tempController[EXT0].pidDriveMax );
	eprSetFloat(EPR_EXT0_PID_PGAIN, tempController[EXT0].pidPGain );
	eprSetFloat(EPR_EXT0_PID_IGAIN, tempController[EXT0].pidIGain );
	eprSetFloat(EPR_EXT0_PID_DGAIN, tempController[EXT0].pidDGain );
	eprSetuint8_t(EPR_EXT0_PID_MAX, tempController[EXT0].pidMax );
	eprSetuint8_t(EPR_EXT0_DRIVE_MIN, tempController[EXT0].pidDriveMin );
	eprSetuint8_t(EPR_EXT0_TYPE_SENSOR, tempController[EXT0].sensorType );
	eprSetuint8_t(EPR_EXT0_COOLER_SPEED, fan[EXT0].coolerSpeed );
	
	eprSetuint8_t(EPR_EXT1_HEAT_MANAGER, tempController[EXT1].heatManager );
	eprSetuint8_t(EPR_EXT1_ANALOG_PIN, tempController[EXT1].sensorPin);
	eprSetuint8_t(EPR_EXT1_DRIVE_MAX, tempController[EXT1].pidDriveMax );
	eprSetFloat(EPR_EXT1_PID_PGAIN, tempController[EXT1].pidPGain );
	eprSetFloat(EPR_EXT1_PID_IGAIN, tempController[EXT1].pidIGain );
	eprSetFloat(EPR_EXT1_PID_DGAIN, tempController[EXT1].pidDGain );
	eprSetuint8_t(EPR_EXT1_PID_MAX, tempController[EXT1].pidMax );
	eprSetuint8_t(EPR_EXT1_DRIVE_MIN, tempController[EXT1].pidDriveMin );
	eprSetuint8_t(EPR_EXT1_TYPE_SENSOR, tempController[EXT1].sensorType );
	eprSetuint8_t(EPR_EXT1_COOLER_SPEED, fan[EXT1].coolerSpeed );
	
	eprSetuint8_t(EPR_BED0_HEAT_MANAGER, tempController[BED0].heatManager );
	eprSetuint8_t(EPR_BED0_ANALOG_PIN, tempController[BED0].sensorPin);
	eprSetuint8_t(EPR_BED0_DRIVE_MAX, tempController[BED0].pidDriveMax );
	eprSetFloat(EPR_BED0_PID_PGAIN, tempController[BED0].pidPGain );
	eprSetFloat(EPR_BED0_PID_IGAIN, tempController[BED0].pidIGain );
	eprSetFloat(EPR_BED0_PID_DGAIN, tempController[BED0].pidDGain );
	eprSetuint8_t(EPR_BED0_PID_MAX, tempController[BED0].pidMax );
	eprSetuint8_t(EPR_BED0_DRIVE_MIN, tempController[BED0].pidDriveMin );	
	eprSetuint8_t(EPR_BED0_TYPE_SENSOR, tempController[BED0].sensorType );
	
	eprSetuint8_t(EPR_BED1_HEAT_MANAGER, tempController[BED1].heatManager );
	eprSetuint8_t(EPR_BED1_ANALOG_PIN, tempController[BED1].sensorPin);
	eprSetuint8_t(EPR_BED1_DRIVE_MAX, tempController[BED1].pidDriveMax );
	eprSetFloat(EPR_BED1_PID_PGAIN, tempController[BED1].pidPGain );
	eprSetFloat(EPR_BED1_PID_IGAIN, tempController[BED1].pidIGain );
	eprSetFloat(EPR_BED1_PID_DGAIN, tempController[BED1].pidDGain );
	eprSetuint8_t(EPR_BED1_PID_MAX, tempController[BED1].pidMax );
	eprSetuint8_t(EPR_BED1_DRIVE_MIN, tempController[BED1].pidDriveMin );
	eprSetuint8_t(EPR_BED1_TYPE_SENSOR, tempController[BED1].sensorType );
	
	eprSetuint8_t(EPR_BED2_HEAT_MANAGER, tempController[BED2].heatManager );
	eprSetuint8_t(EPR_BED2_ANALOG_PIN, tempController[BED2].sensorPin);
	eprSetuint8_t(EPR_BED2_DRIVE_MAX, tempController[BED2].pidDriveMax );
	eprSetFloat(EPR_BED2_PID_PGAIN, tempController[BED2].pidPGain );
	eprSetFloat(EPR_BED2_PID_IGAIN, tempController[BED2].pidIGain );
	eprSetFloat(EPR_BED2_PID_DGAIN, tempController[BED2].pidDGain );
	eprSetuint8_t(EPR_BED2_PID_MAX, tempController[BED2].pidMax );
	eprSetuint8_t(EPR_BED2_DRIVE_MIN, tempController[BED2].pidDriveMin );
	eprSetuint8_t(EPR_BED2_TYPE_SENSOR, tempController[BED2].sensorType );

	eprSetuint8_t(EPR_BED3_HEAT_MANAGER, tempController[BED3].heatManager );
	eprSetuint8_t(EPR_BED3_ANALOG_PIN, tempController[BED3].sensorPin);
	eprSetuint8_t(EPR_BED3_DRIVE_MAX, tempController[BED3].pidDriveMax );
	eprSetFloat(EPR_BED3_PID_PGAIN, tempController[BED3].pidPGain );
	eprSetFloat(EPR_BED3_PID_IGAIN, tempController[BED3].pidIGain );
	eprSetFloat(EPR_BED3_PID_DGAIN, tempController[BED3].pidDGain );
	eprSetuint8_t(EPR_BED3_PID_MAX, tempController[BED3].pidMax );
	eprSetuint8_t(EPR_BED3_DRIVE_MIN, tempController[BED3].pidDriveMin );
	eprSetuint8_t(EPR_BED3_TYPE_SENSOR, tempController[BED3].sensorType );

	eprSetuint8_t(EPR_CHAMB0_HEAT_MANAGER, tempController[CHAMB0].heatManager );
	eprSetuint8_t(EPR_CHAMB0_ANALOG_PIN, tempController[CHAMB0].sensorPin);
	eprSetuint8_t(EPR_CHAMB0_DRIVE_MAX, tempController[CHAMB0].pidDriveMax );
	eprSetFloat(EPR_CHAMB0_PID_PGAIN, tempController[CHAMB0].pidPGain );
	eprSetFloat(EPR_CHAMB0_PID_IGAIN, tempController[CHAMB0].pidIGain );
	eprSetFloat(EPR_CHAMB0_PID_DGAIN, tempController[CHAMB0].pidDGain );
	eprSetuint8_t(EPR_CHAMB0_PID_MAX, tempController[CHAMB0].pidMax );
	eprSetuint8_t(EPR_CHAMB0_DRIVE_MIN, tempController[CHAMB0].pidDriveMin );	
	eprSetuint8_t(EPR_CHAMB0_TYPE_SENSOR, tempController[CHAMB0].sensorType );
	eprSetuint8_t(EPR_CHAMB0_COOLER_SPEED, fan[CHAMB0_FAN_COUNT].coolerSpeed );
		
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
	
	baudrate = eprGetInt32(EPR_BAUDRATE);
	
	tempController[EXT0].heatManager = eprGetuint8_t(EPR_EXT0_HEAT_MANAGER);
	tempController[EXT0].sensorPin = eprGetuint8_t(EPR_EXT0_ANALOG_PIN);
	tempController[EXT0].pidDriveMax = eprGetuint8_t(EPR_EXT0_DRIVE_MAX);
	tempController[EXT0].pidPGain = eprGetFloat(EPR_EXT0_PID_PGAIN);
	tempController[EXT0].pidIGain = eprGetFloat(EPR_EXT0_PID_IGAIN);
	tempController[EXT0].pidDGain = eprGetFloat(EPR_EXT0_PID_DGAIN);
	tempController[EXT0].pidMax = eprGetuint8_t(EPR_EXT0_PID_MAX);
	tempController[EXT0].pidDriveMin = eprGetuint8_t(EPR_EXT0_DRIVE_MIN);
	tempController[EXT0].sensorType = eprGetuint8_t(EPR_EXT0_TYPE_SENSOR);
	fan[EXT0].coolerSpeed = eprGetuint8_t(EPR_EXT0_COOLER_SPEED);
		
	tempController[EXT1].heatManager = eprGetuint8_t(EPR_EXT1_HEAT_MANAGER);
	tempController[EXT1].sensorPin = eprGetuint8_t(EPR_EXT1_ANALOG_PIN);
	tempController[EXT1].pidDriveMax = eprGetuint8_t(EPR_EXT1_HEAT_MANAGER);
	tempController[EXT1].pidPGain = eprGetFloat(EPR_EXT1_HEAT_MANAGER);
	tempController[EXT1].pidIGain = eprGetFloat(EPR_EXT1_HEAT_MANAGER);
	tempController[EXT1].pidDGain = eprGetFloat(EPR_EXT1_HEAT_MANAGER);
	tempController[EXT1].pidMax = eprGetuint8_t(EPR_EXT1_HEAT_MANAGER);
	tempController[EXT1].pidDriveMin = eprGetuint8_t(EPR_EXT1_HEAT_MANAGER);
	tempController[EXT1].sensorType = eprGetuint8_t(EPR_EXT1_TYPE_SENSOR);
	fan[EXT1].coolerSpeed = eprGetuint8_t(EPR_EXT1_COOLER_SPEED);
	
	tempController[BED0].heatManager = eprGetuint8_t(EPR_BED0_HEAT_MANAGER);
	tempController[BED0].sensorPin = eprGetuint8_t(EPR_BED0_ANALOG_PIN);
	tempController[BED0].pidDriveMax = eprGetuint8_t(EPR_BED0_DRIVE_MAX);
	tempController[BED0].pidPGain = eprGetFloat(EPR_BED0_PID_PGAIN);
	tempController[BED0].pidIGain = eprGetFloat(EPR_BED0_PID_IGAIN);
	tempController[BED0].pidDGain = eprGetFloat(EPR_BED0_PID_DGAIN);
	tempController[BED0].pidMax = eprGetuint8_t(EPR_BED0_PID_MAX);
	tempController[BED0].pidDriveMin = eprGetuint8_t(EPR_BED0_DRIVE_MIN);
	tempController[BED0].sensorType = eprGetuint8_t(EPR_BED0_TYPE_SENSOR);
	
	tempController[BED1].heatManager = eprGetuint8_t(EPR_BED1_HEAT_MANAGER);
	tempController[BED1].sensorPin = eprGetuint8_t(EPR_BED1_ANALOG_PIN);
	tempController[BED1].pidDriveMax = eprGetuint8_t(EPR_BED1_DRIVE_MAX);
	tempController[BED1].pidPGain = eprGetFloat(EPR_BED1_PID_PGAIN);
	tempController[BED1].pidIGain = eprGetFloat(EPR_BED1_PID_IGAIN);
	tempController[BED1].pidDGain = eprGetFloat(EPR_BED1_PID_DGAIN);
	tempController[BED1].pidMax = eprGetuint8_t(EPR_BED1_PID_MAX);
	tempController[BED1].pidDriveMin = eprGetuint8_t(EPR_BED1_DRIVE_MIN);
	tempController[BED1].sensorType = eprGetuint8_t(EPR_BED1_TYPE_SENSOR);
	
	tempController[BED2].heatManager = eprGetuint8_t(EPR_BED2_HEAT_MANAGER);
	tempController[BED2].sensorPin = eprGetuint8_t(EPR_BED2_ANALOG_PIN);
	tempController[BED2].pidDriveMax = eprGetuint8_t(EPR_BED2_DRIVE_MAX);
	tempController[BED2].pidPGain = eprGetFloat(EPR_BED2_PID_PGAIN);
	tempController[BED2].pidIGain = eprGetFloat(EPR_BED2_PID_IGAIN);
	tempController[BED2].pidDGain = eprGetFloat(EPR_BED2_PID_DGAIN);
	tempController[BED2].pidMax = eprGetuint8_t(EPR_BED2_PID_MAX);
	tempController[BED2].pidDriveMin = eprGetuint8_t(EPR_BED2_DRIVE_MIN);
	tempController[BED2].sensorType = eprGetuint8_t(EPR_BED2_TYPE_SENSOR);
	
	tempController[BED3].heatManager = eprGetuint8_t(EPR_BED3_HEAT_MANAGER);
	tempController[BED3].sensorPin = eprGetuint8_t(EPR_BED3_ANALOG_PIN);
	tempController[BED3].pidDriveMax = eprGetuint8_t(EPR_BED3_DRIVE_MAX);
	tempController[BED3].pidPGain = eprGetFloat(EPR_BED3_PID_PGAIN);
	tempController[BED3].pidIGain = eprGetFloat(EPR_BED3_PID_IGAIN);
	tempController[BED3].pidDGain = eprGetFloat(EPR_BED3_PID_DGAIN);
	tempController[BED3].pidMax = eprGetuint8_t(EPR_BED3_PID_MAX);
	tempController[BED3].pidDriveMin = eprGetuint8_t(EPR_BED3_DRIVE_MIN);
	tempController[BED3].sensorType = eprGetuint8_t(EPR_BED3_TYPE_SENSOR);
	
	tempController[CHAMB0].heatManager = eprGetuint8_t(EPR_CHAMB0_HEAT_MANAGER);
	tempController[CHAMB0].sensorPin = eprGetuint8_t(EPR_CHAMB0_ANALOG_PIN);
	tempController[CHAMB0].pidDriveMax = eprGetuint8_t(EPR_CHAMB0_DRIVE_MAX);
	tempController[CHAMB0].pidPGain = eprGetFloat(EPR_CHAMB0_PID_PGAIN);
	tempController[CHAMB0].pidIGain = eprGetFloat(EPR_CHAMB0_PID_IGAIN);
	tempController[CHAMB0].pidDGain = eprGetFloat(EPR_CHAMB0_PID_DGAIN);
	tempController[CHAMB0].pidMax = eprGetuint8_t(EPR_CHAMB0_PID_MAX);
	tempController[CHAMB0].pidDriveMin = eprGetuint8_t(EPR_CHAMB0_DRIVE_MIN);
	tempController[CHAMB0].sensorType = eprGetuint8_t(EPR_CHAMB0_TYPE_SENSOR);
	fan[CHAMB0].coolerSpeed = eprGetuint8_t(EPR_CHAMB0_COOLER_SPEED);
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