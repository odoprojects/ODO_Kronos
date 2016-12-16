/*
 * temperature_control.h
 *
 * Created: 13.08.2016 18:30:59
 *  Author: Madman
 */ 


#ifndef TEMPERATURE_CONTROL_H_
#define TEMPERATURE_CONTROL_H_
#include <avr/io.h>
#include "hardware.h"
#include "Configuration.h"

#define HTR_OFF 0
#define HTR_PID 1
#define HTR_SLOWBANG 2
#define HTR_DEADTIME 3

extern volatile uint16_t PIDTune_status;

#define PIDTUNE_TAPIDFINISHED	0x1
#define PIDTUNE_TAPIDFAILEDTIMEOUT	0x2
#define PIDTUNE_TAPIDFAILEDHIGH		0x4
#define PIDTUNE_AUTOTUNE_START		0x8000

typedef struct TemperatureController
{
	//uint8_t Index - 
	uint8_t pwmIndex; ///< pwm index for output control. 0-2 = Extruder, 3 = Fan, 4 = Heated Bed
	uint8_t analogIndex; 
	uint8_t sensorType; ///< Type of temperature sensor.
	uint8_t sensorPin; ///< Pin to read extruder temperature.
	int16_t currentTemperature; ///< Currenttemperature value read from sensor.
	int16_t targetTemperature; ///< Target temperature value in units of sensor.
	float currentTemperatureC; ///< Current temperature in degC.
	float targetTemperatureC; ///< Target temperature in degC.
	uint32_t lastTemperatureUpdate; ///< Time in millis of the last temperature update
	int8_t heatManager; ///< How is temperature controled. 0 = on/off, 1 = PID-Control, 3 = deat time control
	float tempIState; ///< Temp. var. for PID computation.
	uint8_t pidDriveMax; ///< Used for windup in PID calculation.
	uint8_t pidDriveMin; ///< Used for windup in PID calculation.
	float pidPGain; ///< Pgain (proportional gain) for PID temperature control [0,01 Units].
	float pidIGain; ///< Igain (integral) for PID temperature control [0,01 Units].
	float pidDGain;  ///< Dgain (damping) for PID temperature control [0,01 Units].
	uint8_t pidMax; ///< Maximum PWM value, the heater should be set.
	float tempIStateLimitMax;
	float tempIStateLimitMin;
	uint8_t tempPointer;
	float tempArray[4];
	uint8_t flags;
}TTemperatureController;

typedef struct Fan{
	//struct TemperatureController temperatureCont;
	uint8_t coolerSpeed; ///< Speed to use when enabled
	uint8_t coolerPWM; ///< current PWM setting
}TFan;

typedef struct autotunePIDs
{
	uint8_t controller_ID;
	float autotune_temp;
	uint8_t maxCycles;
	uint8_t save_to_eepr;
	
}TautotunePIDs;

extern TautotunePIDs autotunepidtable;

extern TTemperatureController tempController[ANALOG_INPUTS];
//extern struct TemperatureController tempController[ANALOG_INPUTS];
#define FAN_COUNT	NUM_EXTRUDER+1 //ilosc ekstruderów + dodatkowe wentylatory np. CHAMB0
#define CHAMB0_FAN_COUNT 2 //indeks tablicy odpowiadajacy w strukturze FAN danemu wentylatorowi
//extern struct Fan fan[FAN_COUNT];
extern TFan fan[FAN_COUNT];

uint8_t pwm_pos[ANALOG_INPUTS]; 

void manageTemperatures();
void MANAGE_TEMPETATURE_EVENT();
void register_manage_temperature_event_callback(void (*callback)());
void init_temperature_control();
void setTargetTemperature(struct TemperatureController *temperatureCont, uint16_t tar);
int8_t updateCurrentTemperature(struct TemperatureController *temperatureCont);
//extern struct TemperatureController tempController[ANALOG_INPUTS];
int8_t autotunePID(struct TemperatureController *temperatureCont, float temp, uint8_t controllerId, int maxCycles, uint8_t storeValues);
void disableAllHeater();

#endif /* TEMPERATURE_CONTROL_H_ */