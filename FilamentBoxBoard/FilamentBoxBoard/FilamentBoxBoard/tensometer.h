/*
 * tensometer.h
 *
 * Created: 12/12/2016 4:08:06 PM
 *  Author: Tomek
 */ 


#ifndef TENSOMETER_H_
#define TENSOMETER_H_

#include <avr/io.h>

#define TENSOMETER_NUMBER	2
#define TENSOMETER_0	0
#define TENSOMETER_1	1

#define TENSOMETER_CALIBRATION_NO_LOAD	1
#define TENSOMETER_CALIBRATION_LOAD		2
#define TENSOMETER_CALIBRATION_COUTING	5
#define TENSOMETER_CALIBRATION_INTERVAL_MS	100


#define TENSOMETER_RESOLUTION	65535
#define TENSOMETER_MAX_WEIGHT 5000 //w gramach
#define TENSOMETER_ACCURACY	TENSOMETER_MAX_WEIGHT/TENSOMETER_RESOLUTION

#define TENSOMETER_MEASURE_INTERVAL	1000 // w ms
#define DEFAULT_ROLL_WEIGHT		100

#define AVERAGE_COUNT	4 //z ilu pomiarow usredniamy wynik

typedef struct Tensometer{
	uint16_t RAW_Value;
	uint16_t weight;
	uint16_t rollWeight;
	uint16_t filamentWeight;
	uint16_t setFilamentWeight;
	uint8_t  numberConversion;		//zmienna pomocnicza sluzaca do oblicznia wartosci sredniej
	int32_t tempPreCalculateMeasure;  //zmienna pomocnicza sluzaca do oblicznia wartosci sredniej
	uint16_t constantCalibrationValue;
	int32_t zeroScaleMass;
	uint16_t calibatrionMass;
}Ttensometer;

extern Ttensometer tensometer[TENSOMETER_NUMBER];
extern uint8_t calibration;
extern uint8_t calibrating_tensometer;
void initTensometer();
void MANAGE_TENSOMETER_EVENT();
void register_manage_tensometer_event_callback(void (*callback)());
void manageTensometer();
void make_tensometer_measure();
void get_roll_weight(uint8_t input_mux);
void offset_calibration(void);

#endif /* TENSOMETER_H_ */