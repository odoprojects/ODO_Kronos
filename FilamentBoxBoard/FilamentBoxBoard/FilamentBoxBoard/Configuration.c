/*
 * Configuration.c
 *
 * Created: 30.08.2016 10:04:04
 *  Author: pieru
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include "hardware.h"
#include "Configuration.h"
#include "temperature_control.h"
#include "Eeprom.h"
#include "mkuart.h"

//extern struct TemperatureController tempController[ANALOG_INPUTS];
//extern struct Fan fan[NUM_EXTRUDER];



void init(){
	
//***************************************
//	Init hardware
//***************************************
	//restoreEEPROMSettingsFromConfiguration();
	set_default_pins();
	Pin_init();
	Timer0_init();
	Timer1_init();
//	readDataFromEEPROM();
	
	//initBaudrate();
	USARTX_Init(baudrate);
	initTensometer();
	sei();
//	ADC_init();
	
//***************************************
//	Init software
//***************************************
	//	load_default_variables();

 //  init_temperature_control();
	


	
 
	 
}


