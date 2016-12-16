/*
 * Configuration.h
 *
 * Created: 30.08.2016 10:04:21
 *  Author: pieru
 */ 


#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

//***************************************
#define EXT0_HEAT_MANAGER		1
#define EXT0_DRIVE_MAX			230
#define EXT0_PID_PGAIN			23.29
#define EXT0_PID_IGAIN			0.84
#define EXT0_PID_DGAIN			161.24
#define EXT0_PID_MAX			255
#define EXT0_DRIVE_MIN			40
#define EXT0_EXTRUDER_COOLER_SPEED 255

//***************************************
#define EXT1_HEAT_MANAGER		1
#define EXT1_DRIVE_MAX			230
#define EXT1_PID_PGAIN			23.29
#define EXT1_PID_IGAIN			0.84
#define EXT1_PID_DGAIN			161.24
#define EXT1_PID_MAX			255
#define EXT1_DRIVE_MIN			40
#define EXT1_EXTRUDER_COOLER_SPEED 255

//***************************************
#define BED0_HEAT_MANAGER		1
#define BED0_DRIVE_MAX			255
#define BED0_PID_PGAIN			196
#define BED0_PID_IGAIN			33
#define BED0_PID_DGAIN			290
#define BED0_PID_MAX			255
#define BED0_DRIVE_MIN			80
//***************************************
#define BED1_HEAT_MANAGER		1
#define BED1_DRIVE_MAX			255
#define BED1_PID_PGAIN			196
#define BED1_PID_IGAIN			33
#define BED1_PID_DGAIN			290
#define BED1_PID_MAX			255
#define BED1_DRIVE_MIN			80
//***************************************
#define BED2_HEAT_MANAGER		1
#define BED2_DRIVE_MAX			255
#define BED2_PID_PGAIN			196
#define BED2_PID_IGAIN			33
#define BED2_PID_DGAIN			290
#define BED2_PID_MAX			255
#define BED2_DRIVE_MIN			80
//***************************************
#define BED3_HEAT_MANAGER		1
#define BED3_DRIVE_MAX			255
#define BED3_PID_PGAIN			196
#define BED3_PID_IGAIN			33
#define BED3_PID_DGAIN			290
#define BED3_PID_MAX			255
#define BED3_DRIVE_MIN			80
//***************************************
#define CHAMB0_HEAT_MANAGER		1
#define CHAMB0_DRIVE_MAX		255
#define CHAMB0_PID_PGAIN		196
#define CHAMB0_PID_IGAIN		33
#define CHAMB0_PID_DGAIN		290
#define CHAMB0_PID_MAX			255
#define CHAMB0_DRIVE_MIN		80
//***************************************

#define NUM_PWM 6 //ilosc kanalow pwm jakie beda obslugiwac grzalki - ext0, ext1, bed0, bed1, bed2, bed3,
#define NUM_EXTRUDER	2
#define CELSIUS_EXTRA_BITS 3
#define PID_CONTROL_RANGE 20

#define TEMP_INT_TO_FLOAT(temp) ((float)(temp)/(float)(1<<CELSIUS_EXTRA_BITS))
#define TEMP_FLOAT_TO_INT(temp) ((int)((temp)*(1<<CELSIUS_EXTRA_BITS)))

#define FAN_NUMBER					4
#define SENSOR_TYPE_TT2_231KC6_1	203
#define DEF_SENSOR_TYPE				204
#define MAXTEMP 275
#define NUM_TEMPS_USERTHERMISTOR0		0
#define USER_THERMISTORTABLE0 {}
#define NUM_TEMPS_USERTHERMISTOR1		0
#define USER_THERMISTORTABLE1 {}
#define NUM_TEMPS_USERTHERMISTOR2		0
#define USER_THERMISTORTABLE2 {}

#define USE_GENERIC_THERMISTORTABLE_1	1
#define GENERIC_THERM1_T0				25.0
#define GENERIC_THERM1_R0				231439
#define GENERIC_THERM1_BETA				4537
#define GENERIC_THERM1_MIN_TEMP			-20
#define GENERIC_THERM1_MAX_TEMP			300
#define GENERIC_THERM1_R1				0
#define GENERIC_THERM1_R2				4700

#define USE_GENERIC_THERMISTORTABLE_2	1
#define GENERIC_THERM1_T0				25.0
#define GENERIC_THERM1_R0				231439
#define GENERIC_THERM1_BETA				4537
#define GENERIC_THERM1_MIN_TEMP			-20
#define GENERIC_THERM1_MAX_TEMP			300
#define GENERIC_THERM1_R1				0
#define GENERIC_THERM1_R2				4700

#define GENERIC_THERM_VREF				5
#define GENERIC_THERM_NUM_ENTRIES		33
#define TEMP_PID						1
#define HEATED_BED_SET_INTERVAL			5000
#define MAX_PWM							255

#define EXTRUDER_FAN_COOL_TEMP	50
#define MIN_DEFECT_TEMPERATURE	-15
#define MAX_DEFECT_TEMPERATURE	300
#define SCALE_PID_TO_MAX		1

#if !defined(HEATER_PWM_SPEED)
#define HEATER_PWM_SPEED 0
#endif
#if HEATER_PWM_SPEED < 0
#define HEATER_PWM_SPEED 0
#endif
#if HEATER_PWM_SPEED > 2
#define HEATER_PWM_SPEED 2
#endif

#if HEATER_PWM_SPEED == 0
#define HEATER_PWM_STEP 1
#define HEATER_PWM_MASK 255
#elif HEATER_PWM_SPEED == 1
#define HEATER_PWM_STEP 2
#define HEATER_PWM_MASK 254
#else
#define HEATER_PWM_STEP 4
#define HEATER_PWM_MASK 252
#endif

#if !defined(COOLER_PWM_SPEED)
#define COOLER_PWM_SPEED 0
#endif
#if COOLER_PWM_SPEED < 0
#define COOLER_PWM_SPEED 0
#endif
#if COOLER_PWM_SPEED > 2
#define COOLER_PWM_SPEED 2
#endif

#if COOLER_PWM_SPEED == 0
#define COOLER_PWM_STEP 1
#define COOLER_PWM_MASK 255
#elif COOLER_PWM_SPEED == 1
#define COOLER_PWM_STEP 2
#define COOLER_PWM_MASK 254
#else
#define COOLER_PWM_STEP 4
#define COOLER_PWM_MASK 252
#endif


void init();


#endif /* CONFIGURATION_H_ */