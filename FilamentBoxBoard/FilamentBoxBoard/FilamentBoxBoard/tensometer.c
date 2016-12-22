/*
 * tensometer.c
 *
 * Created: 12/12/2016 4:07:24 PM
 *  Author: Tomek
 */ 
#include "tensometer.h"
#include "softSPI.h"
#include "hardware.h"
#include "Pins.h"
#include <avr/io.h>
#include <util/delay.h>

Ttensometer tensometer[TENSOMETER_NUMBER];
uint8_t calibration = 0;
uint8_t calibrating_tensometer = 0;


static void (*tensometer_event_callback)();

void register_manage_tensometer_event_callback(void (*callback)()) {
	tensometer_event_callback = callback;
}

void MANAGE_TENSOMETER_EVENT() {
	if( tensometer_event_callback ) {
		(*tensometer_event_callback)();
	}
}


void initTensometer(){
	PIN_OUT(ADS_A0_PIN);
	ADS_A0_LOW;
	
	PIN_OUT(ADS_SCLK_PIN);
	ADS_SCLK_LOW;
	
	PIN_IN(ADS_DRDY_PIN);
	ADS_DRDY_LOW;
	
	PIN_OUT(ADS_PDWN_PIN);
	ADS_PDWN_LOW;
	
	for (int i =0;i<TENSOMETER_NUMBER;i++)
	{
		get_roll_weight(i);
		tensometer[i].RAW_Value = 0;
		tensometer[i].filamentWeight = 0;
		tensometer[i].weight = 0;
		tensometer[i].numberConversion=0;
		tensometer[i].tempPreCalculateMeasure=0;
		tensometer[i].setFilamentWeight=0;
		//tensometer[i].constantCalibrationValue=0;
		//tensometer[i].zeroScaleMass=0;
	}
	
	ADS_PDWN_LOW;
	_delay_ms(10);
	ADS_PDWN_HIGH; // ADS ON
	
	ADS_A0_LOW;
}

// 	Mass is calculated from ADC code using the formula:
// 	w = m ? c + wzs – wt
// 	where:
// 	• w = mass
// 	• c = the ADC code
// 	• wt = tare weight
// 	• m, wzs, wt = values determined in the calibration process
// 	m is a calibration constant, and is calculated using Equation 1:
// 	(1)
// 	where wfs is the user-specified calibration mass, cfs is the ADC code taken with the calibration mass
// 	applied, and czs is the ADC measurement taken with no load.
// 	wzs , the zero-scale mass, is calculated from m and czs using Equation 2:
// 	wzs = –m ? czs

void calulate_calibration_constant(uint8_t input_mux, uint32_t calibration_mass_gram, int32_t adc_code_mass, int32_t adc_code_noload){
	calibration_mass_gram *=1000;
	tensometer[input_mux].constantCalibrationValue = (uint16_t)(calibration_mass_gram/(adc_code_mass - adc_code_noload));
	tensometer[input_mux].zeroScaleMass = (int32_t)tensometer[input_mux].constantCalibrationValue*adc_code_noload ;
	tensometer[input_mux].zeroScaleMass *= (-1);
	calibration_mass_gram;
}



uint8_t calibration_sequence(){
	uint16_t calibatrionMass = tensometer[calibrating_tensometer].calibatrionMass;
	
	modbus_puts_s(PSTR("kalibracja rozpoczeta!! Zdejmij obciazenie z belki za:"));
	for (int i=TENSOMETER_CALIBRATION_COUTING;i>=0;i--){
		modbus_puts_var_int(PSTR(" Count: "),i);
		_delay_ms(1000);
	}
	for (int i=0;i<TENSOMETER_CALIBRATION_COUTING;i++){
		make_tensometer_measure(calibrating_tensometer);
		_delay_ms(TENSOMETER_CALIBRATION_INTERVAL_MS);
	}
	
	int16_t adc_code_noload = (int16_t)tensometer[calibrating_tensometer].RAW_Value;
	
	modbus_puts_s(PSTR("Zaloz obciazenie testowe za:"));
	for (int i=TENSOMETER_CALIBRATION_COUTING;i>=0;i--){
		modbus_puts_var_int(PSTR(" Count: "),i);
		_delay_ms(1000);
	}
	
	for (int i=0;i<TENSOMETER_CALIBRATION_COUTING;i++){
		make_tensometer_measure(calibrating_tensometer);
		_delay_ms(TENSOMETER_CALIBRATION_INTERVAL_MS);
	}
	
	int16_t adc_code_mass = (int16_t)tensometer[calibrating_tensometer].RAW_Value;
	
	calulate_calibration_constant(calibrating_tensometer,calibatrionMass,adc_code_mass,adc_code_noload);
	storeDataIntoEEPROM();
	modbus_puts_s(PSTR("Kalibracja zakonczona!!!"));
	return 1;
}

void offset_calibration(void)
{
	for(uint8_t i=0; i<26; i++)
	{
		ADS_SCLK_HIGH;
		_delay_us(2);
		ADS_SCLK_LOW;
		_delay_us(2);
	}
	_delay_ms(1500);
}

 
void make_tensometer_measure(){

	static uint8_t input_mux=0;
	

	
	
	int32_t data=0; 		
	while (ADS_DRDY_IF_HI);	// Czekamy na koniec przetwarzania
			
	for(uint8_t i=0; i<24; i++)
	{
		data<<=1;
		ADS_SCLK_HIGH;
		_delay_us(2);
		if(ADS_DRDY_IF_HI) data |= 1;
		ADS_SCLK_LOW;
		_delay_us(2);
	}
			
	ADS_SCLK_HIGH;
	_delay_us(2);
	ADS_SCLK_LOW;
	_delay_us(2);
		
	tensometer[input_mux].tempPreCalculateMeasure = data>>8;  //>>8
	if (tensometer[input_mux].tempPreCalculateMeasure & 0x8000)
		tensometer[input_mux].tempPreCalculateMeasure |= 0xffff0000;
	
	tensometer[input_mux].RAW_Value = tensometer[input_mux].tempPreCalculateMeasure;
	tensometer[input_mux].numberConversion=0;
	tensometer[input_mux].tempPreCalculateMeasure=0;
	
	int32_t weight = tensometer[input_mux].constantCalibrationValue;
	int32_t RAW_Value = tensometer[input_mux].RAW_Value;
	weight *= RAW_Value;
	weight += tensometer[input_mux].zeroScaleMass;
	weight = weight/1000;
	tensometer[input_mux].weight = (uint16_t)weight;
	tensometer[input_mux].filamentWeight = tensometer[input_mux].weight - tensometer[input_mux].rollWeight;	

	
	/*modbus_puts_var_int(PSTR(" imput mux =  "),input_mux);
	modbus_puts_var_int(PSTR(" weight: "),tensometer[input_mux].weight);*/
	
	if (input_mux)
	{
		ADS_A0_HIGH;
		input_mux=0;
	}
	else
	{
		ADS_A0_LOW;
		input_mux=1;
	}
}

// void get_tensometer_measure(uint8_t input_mux)
// {	
// 	uint32_t data=0; 
// 
// 	while (ADS_DRDY_IF_HI);	// Czekaj na koniec przetwarzania
// 	
// 	for(uint8_t i=0; i<24; i++)
// 	{
// 		ADS_SCLK_HIGH;
// 		_delay_us(10);
// 		ADS_SCLK_LOW;
// 		_delay_us(10);
// 		
// 		data<<=1;
// 		data|=((ADS_DRDY_IF_HI)>0) ? 1 : 0;
// 	}
// 	
// 	ADS_SCLK_HIGH;
// 	_delay_us(10);
// 	ADS_SCLK_LOW;
// 	_delay_us(10);
// 	
// 	tensometer[input_mux].tempPreCalculateMeasure += data>>8;
// 	
// 	if (input_mux)
// 	{
// 		ADS_A0_HIGH;
// 	}
// 	else  ADS_A0_LOW;
// 
// }


// void calculate_weight(uint8_t input_mux){
// 	uint16_t preCalculateMeasure = tensometer[input_mux].preCalculateMeasure;	
// 	tensometer[input_mux].weight =	TENSOMETER_ACCURACY*preCalculateMeasure;	
// 	tensometer[input_mux].filamentWeight = tensometer[input_mux].weight - tensometer[input_mux].rollWeight;
// }

void get_roll_weight(uint8_t input_mux){
	tensometer[input_mux].rollWeight = DEFAULT_ROLL_WEIGHT;
}

void manageTensometer(){
	if (!Filament_measure_timer)	// wstawic Filament_measure_timer
	{
		make_tensometer_measure();
		
		Filament_measure_timer = TENSOMETER_MEASURE_INTERVAL;
	}
	if (calibration){
		calibration_sequence();
		calibration = 0;
	}
}