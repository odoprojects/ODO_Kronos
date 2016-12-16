/*
 * tensometer.c
 *
 * Created: 12/12/2016 4:07:24 PM
 *  Author: Tomek
 */ 
#include "tensometer.h"
#include "softSPI.h"
#include "Pins.h"
#include <util/delay.h>

Ttensometer tensometer[TENSOMETER_NUMBER];

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
	}
	
	ADS_PDWN_LOW;
	_delay_ms(10);
	ADS_PDWN_HIGH; // ADS ON
	
	ADS_A0_LOW;
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

 
void make_tensometer_measure(void){

//         uint32_t tmp = 0;
//         
//         for(uint8_t i = 0; i < 24; i++){
// 	        tmp <<= 1;
// 	        SCK_SET;
// 	        _delay(5);
// 	        if(SDO_IN) tmp |= 1;
// 	        SCK_RESET;
// 	        _delay(5);
//         }
//         SCK_SET;
//         _delay(5);
//         SCK_RESET;
//         tmp >>= 5;
//         if(tmp & (1<<18)) tmp |= 0xfff80000;
//         return tmp;


		
		static uint8_t input_mux=0;
		//static uint8_t numberConversion=0;
		//static uint32_t tempPreCalculateMeasure=0;
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
		
		
//		uint32_t tmp = 0xFFFFFF;
//		tmp >>= 5;
//		if(tmp & 0x40000) 
//		tmp |= 0xfff80000;
	

		if (tensometer[input_mux].numberConversion++ < AVERAGE_COUNT)
		{
			tensometer[input_mux].tempPreCalculateMeasure += data>>8;  //>>8		
			if (tensometer[input_mux].tempPreCalculateMeasure & 0x8000)
				tensometer[input_mux].tempPreCalculateMeasure |= 0xffff0000;
		}else
		{
			tensometer[input_mux].RAW_Value = tensometer[input_mux].tempPreCalculateMeasure/AVERAGE_COUNT;
			tensometer[input_mux].numberConversion=0;
			tensometer[input_mux].tempPreCalculateMeasure=0;
		}
		
		if (input_mux)
		{
//			ADS_A0_HIGH;
//			input_mux=0;
		}
		else
		{
//			ADS_A0_LOW;
//			input_mux=1;
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