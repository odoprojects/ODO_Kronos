/*
 * modbus.c
 *
 * Created: 21.08.2016 15:29:58
 *  Author: Madman
 */ 

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <string.h>

#include "mkuart.h"
#include "hardware.h"
#include "modbus.h"
#include "Modbus_registers.h"
#include "temperature_control.h"
#include "tensometer.h"

//extern struct TemperatureController tempController[ANALOG_INPUTS];

uint8_t autotune = 0;
volatile char bufor[64];

char calculate_LRC(char *frame, char lenght){
	
	uint8_t LRC=0;
	for(uint8_t i = 1; i < lenght; i++){
		LRC+=frame[i];
	}
	return ((uint8_t)(-((int8_t)LRC)));
}

void delay_transmision(uint16_t delay)
{
		long t_time = millis+delay;
		while (t_time>millis);
}

uint8_t check_modbus_blocade(){
	modbus_transmission_blocade_timer = MODBUS_BLOCADE_TRANSMISSION_TIMEOUT_MS;
	
	while(block_modbus_transmission){
		if (modbus_transmission_blocade_timer==0){
			return 0;
		}
	}
	
	return 1;
}

void mod_send_ascii(char c){
	char first = c/16;
	char sec = c - (16 * first);
		
	first = int_to_ascii(first);
	sec = int_to_ascii(sec);
	uart_putc(first);
	uart_putc(sec);
}

void modbus_puts(char *s)		// wysy³a ³añcuch z pamiêci RAM na UART
{
	check_modbus_blocade();
	register char c;
	uart_putc(':');
	for (uint8_t i=0; i<s[0]; i++){
		c = s[i+1];
		mod_send_ascii(c);
	}
	uart_putc(CR);
	uart_putc(LF);
	delay_transmision(2);
}

void modbus_puts_s(char *s)		// wysy³a ³añcuch z pamiêci RAM na UART
{
	check_modbus_blocade();
	register char c;
	uart_putc(':');
	uart_putc(';');
	uart_puts(s);
	uart_putc(CR);
	uart_putc(LF);
	delay_transmision(2);
}

void modbus_puts_var_int(char *s, uint16_t data)		// wysy³a ³añcuch z pamiêci RAM na UART
{
	check_modbus_blocade();
	register char c;
	uart_putc(':');
	uart_putc(';');
	uart_puts(s);
	uart_putint(data, 10);
	uart_putc(CR);
	uart_putc(LF);
	delay_transmision(2);
}

void modbus_puts_var_float(char *s, float data, uint8_t digits)		// wysy³a ³añcuch z pamiêci RAM na UART
{
	check_modbus_blocade();
	register char c;
	uart_putc(':');
	uart_putc(';');
	uart_puts(s);
	uart_putFloat(data, digits);
	uart_putc(CR);
	uart_putc(LF);
	delay_transmision(2);
}

//void mod_send_reset_command(char *data){
	
//}
int8_t mod_write_single_register(char *data){
	const uint16_t write_value = (data[(data[0]-2)]*256+data[(data[0]-1)]); 
	const uint16_t address_register = data[DATA_POS]*256+data[DATA_POS+1];
	switch(address_register){
		case MODBUS_RESET_ADR:
			disableAllHeater();		
		break;
		case MODBUS_ALIVE_ADR:
			modbus_puts_s(PSTR("Filament Box Alive"));
		break;
		case MODBUS_EEPROM_SAVE:
			storeDataIntoEEPROM();
			modbus_puts_s(PSTR("AUX: EEprom saved"));
		break;	
		case MODBUS_EXT0_SET_TEMP_ADR:
			setTargetTemperature(&tempController[EXT0],write_value);
		break;
		case MODBUS_EXT1_SET_TEMP_ADR:
			setTargetTemperature(&tempController[EXT1],write_value);
		break;
		case MODBUS_BED_SET_TEMP_ADR:
			setTargetTemperature(&tempController[BED0],write_value);
			setTargetTemperature(&tempController[BED1],write_value);
			setTargetTemperature(&tempController[BED2],write_value);
			setTargetTemperature(&tempController[BED3],write_value);
		break;
		
		/*case MODBUS_DOOR_STATUS_ADR:
			if (write_value & DOOR_OPEN_MASK)
			{
				door_status &= ~DOOR_CLOSE_OPEN_MASK;
				door_status |= DOOR_OPEN_MASK;
				
			} 
			else if(write_value & DOOR_CLOSE_MASK)
			{
				door_status &= ~DOOR_CLOSE_OPEN_MASK;
				door_status |= DOOR_CLOSE_MASK;
			}
		break;
		//--------------EXT0---------------			
			case MODBUS_EXT0_R_W_SENSOR_TYPE_ADR:
				tempController[EXT0].sensorType = write_value;
			break;
			case MODBUS_EXT0_R_W_HEAT_MANAGER_ADR:
				tempController[EXT0].heatManager = write_value;
			break;
			case MODBUS_EXT0_R_W_PID_P_ADR:
				tempController[EXT0].pidPGain = (float)write_value/100.0;
			break;
			case MODBUS_EXT0_R_W_PID_I_ADR:
				tempController[EXT0].pidIGain = (float)write_value/100.0;
			break;
			case MODBUS_EXT0_R_W_PID_D_ADR:
				tempController[EXT0].pidDGain = (float)write_value/100.0;
			break;
			case MODBUS_EXT0_R_W_DRIVE_MAX_ADR:
				tempController[EXT0].pidDriveMax = write_value;
			break;
			case MODBUS_EXT0_R_W_DRVE_MIN_ADR:
				tempController[EXT0].pidDriveMin = write_value;
			break;
			case MODBUS_EXT0_R_W_PID_MAX_ADR:	
				tempController[EXT0].pidMax = write_value;
			break;	
//--------------EXT1---------------			
			case MODBUS_EXT1_R_W_SENSOR_TYPE_ADR:
				tempController[EXT1].sensorType = write_value;
			break;
			case MODBUS_EXT1_R_W_HEAT_MANAGER_ADR:
				tempController[EXT1].heatManager = write_value;
			break;
			case MODBUS_EXT1_R_W_PID_P_ADR:
				tempController[EXT1].pidPGain = (float)write_value/100.0;
			break;
			case MODBUS_EXT1_R_W_PID_I_ADR:
				tempController[EXT1].pidIGain = (float)write_value/100.0;
			break;
			case MODBUS_EXT1_R_W_PID_D_ADR:
				tempController[EXT1].pidDGain = (float)write_value/100.0;
			break;
			case MODBUS_EXT1_R_W_DRIVE_MAX_ADR:
				tempController[EXT1].pidDriveMax = write_value;
			break;
			case MODBUS_EXT1_R_W_DRVE_MIN_ADR:
				tempController[EXT1].pidDriveMin = write_value;
			break;
			case MODBUS_EXT1_R_W_PID_MAX_ADR:	
				tempController[EXT1].pidMax = write_value;
			break;	
	//--------------BED---------------			
			case MODBUS_BED_R_W_SENSOR_TYPE_ADR:
				tempController[BED0].sensorType = write_value;
				tempController[BED1].sensorType = write_value;
				tempController[BED2].sensorType = write_value;
				tempController[BED3].sensorType = write_value;
			break;
			case MODBUS_BED_R_W_HEAT_MANAGER_ADR:
				tempController[BED0].heatManager = write_value;
				tempController[BED1].heatManager = write_value;
				tempController[BED2].heatManager = write_value;
				tempController[BED3].heatManager = write_value;
			break;
			case MODBUS_BED_R_W_PID_P_ADR:
				tempController[BED0].pidPGain = (float)write_value/100.0;
				tempController[BED1].pidPGain = (float)write_value/100.0;
				tempController[BED2].pidPGain = (float)write_value/100.0;
				tempController[BED3].pidPGain = (float)write_value/100.0;
			break;
			case MODBUS_BED_R_W_PID_I_ADR:
				tempController[BED0].pidIGain = (float)write_value/100.0;
				tempController[BED1].pidIGain = (float)write_value/100.0;
				tempController[BED2].pidIGain = (float)write_value/100.0;
				tempController[BED3].pidIGain = (float)write_value/100.0;
			break;
			case MODBUS_BED_R_W_PID_D_ADR:
				tempController[BED0].pidDGain = (float)write_value/100.0;
				tempController[BED1].pidDGain = (float)write_value/100.0;
				tempController[BED2].pidDGain = (float)write_value/100.0;
				tempController[BED3].pidDGain = (float)write_value/100.0;
			break;
			case MODBUS_BED_R_W_DRIVE_MAX_ADR:
				tempController[BED0].pidDriveMax = write_value;
				tempController[BED1].pidDriveMax = write_value;
				tempController[BED2].pidDriveMax = write_value;
				tempController[BED3].pidDriveMax = write_value;
			break;
			case MODBUS_BED_R_W_DRVE_MIN_ADR:
				tempController[BED0].pidDriveMin = write_value;
				tempController[BED1].pidDriveMin = write_value;
				tempController[BED2].pidDriveMin = write_value;
				tempController[BED3].pidDriveMin = write_value;
			break;
			case MODBUS_BED_R_W_PID_MAX_ADR:	
				tempController[BED0].pidMax = write_value;
				tempController[BED1].pidMax = write_value;
				tempController[BED2].pidMax = write_value;
				tempController[BED3].pidMax = write_value;
			break;	
				//--------------CHAMBER0---------------			
			case MODBUS_CHAMBER0_R_W_SENSOR_TYPE_ADR:
				tempController[CHAMB0].sensorType = write_value;
			break;
			case MODBUS_CHAMBER0_R_W_HEAT_MANAGER_ADR:
				tempController[CHAMB0].heatManager = write_value;
			break;
			case MODBUS_CHAMBER0_R_W_PID_P_ADR:
				tempController[CHAMB0].pidPGain = (float)write_value/100.0;
			break;
			case MODBUS_CHAMBER0_R_W_PID_I_ADR:
				tempController[CHAMB0].pidIGain = (float)write_value/100.0;
			break;
			case MODBUS_CHAMBER0_R_W_PID_D_ADR:
				tempController[CHAMB0].pidDGain = (float)write_value/100.0;
			break;
			case MODBUS_CHAMBER0_R_W_DRIVE_MAX_ADR:
				tempController[CHAMB0].pidDriveMax = write_value;
			break;
			case MODBUS_CHAMBER0_R_W_DRVE_MIN_ADR:
				tempController[CHAMB0].pidDriveMin = write_value;
			break;
			case MODBUS_CHAMBER0_R_W_PID_MAX_ADR:	
				tempController[CHAMB0].pidMax = write_value;
			break;			
	//--------------BED0---------------			
			case MODBUS_BED0_R_W_SENSOR_TYPE_ADR:
				tempController[BED0].sensorType = write_value;
			break;
			case MODBUS_BED0_R_W_HEAT_MANAGER_ADR:
				tempController[BED0].heatManager = write_value;
			break;
			case MODBUS_BED0_R_W_PID_P_ADR:
				tempController[BED0].pidPGain = (float)write_value/100.0;
			break;
			case MODBUS_BED0_R_W_PID_I_ADR:
				tempController[BED0].pidIGain = (float)write_value/100.0;
			break;
			case MODBUS_BED0_R_W_PID_D_ADR:
				tempController[BED0].pidDGain = (float)write_value/100.0;
			break;
			case MODBUS_BED0_R_W_DRIVE_MAX_ADR:
				tempController[BED0].pidDriveMax = write_value;
			break;
			case MODBUS_BED0_R_W_DRVE_MIN_ADR:
				tempController[BED0].pidDriveMin = write_value;
			break;
			case MODBUS_BED0_R_W_PID_MAX_ADR:	
				tempController[BED0].pidMax = write_value;
			break;	
//--------------BED1---------------			
			case MODBUS_BED1_R_W_SENSOR_TYPE_ADR:
				tempController[BED1].sensorType = write_value;
			break;
			case MODBUS_BED1_R_W_HEAT_MANAGER_ADR:
				tempController[BED1].heatManager = write_value;
			break;
			case MODBUS_BED1_R_W_PID_P_ADR:
				tempController[BED1].pidPGain = (float)write_value/100.0;
			break;
			case MODBUS_BED1_R_W_PID_I_ADR:
				tempController[BED1].pidIGain = (float)write_value/100.0;
			break;
			case MODBUS_BED1_R_W_PID_D_ADR:
				tempController[BED1].pidDGain = (float)write_value/100.0;
			break;
			case MODBUS_BED1_R_W_DRIVE_MAX_ADR:
				tempController[BED1].pidDriveMax = write_value;
			break;
			case MODBUS_BED1_R_W_DRVE_MIN_ADR:
				tempController[BED1].pidDriveMin = write_value;
			break;
			case MODBUS_BED1_R_W_PID_MAX_ADR:	
				tempController[BED1].pidMax = write_value;
			break;
//--------------BED2---------------			
			case MODBUS_BED2_R_W_SENSOR_TYPE_ADR:
				tempController[BED2].sensorType = write_value;
			break;
			case MODBUS_BED2_R_W_HEAT_MANAGER_ADR:
				tempController[BED2].heatManager = write_value;
			break;
			case MODBUS_BED2_R_W_PID_P_ADR:
				tempController[BED2].pidPGain = (float)write_value/100.0;
			break;
			case MODBUS_BED2_R_W_PID_I_ADR:
				tempController[BED2].pidIGain = (float)write_value/100.0;
			break;
			case MODBUS_BED2_R_W_PID_D_ADR:
				tempController[BED2].pidDGain = (float)write_value/100.0;
			break;
			case MODBUS_BED2_R_W_DRIVE_MAX_ADR:
				tempController[BED2].pidDriveMax = write_value;
			break;
			case MODBUS_BED2_R_W_DRVE_MIN_ADR:
				tempController[BED2].pidDriveMin = write_value;
			break;
			case MODBUS_BED2_R_W_PID_MAX_ADR:	
				tempController[BED2].pidMax = write_value;
			break;	
			
//--------------BED3---------------			
			case MODBUS_BED3_R_W_SENSOR_TYPE_ADR:
				tempController[BED3].sensorType = write_value;
			break;
			case MODBUS_BED3_R_W_HEAT_MANAGER_ADR:
				tempController[BED3].heatManager = write_value;
			break;
			case MODBUS_BED3_R_W_PID_P_ADR:
				tempController[BED3].pidPGain = (float)write_value/100.0;
			break;
			case MODBUS_BED3_R_W_PID_I_ADR:
				tempController[BED3].pidIGain = (float)write_value/100.0;
			break;
			case MODBUS_BED3_R_W_PID_D_ADR:
				tempController[BED3].pidDGain = (float)write_value/100.0;
			break;
			case MODBUS_BED3_R_W_DRIVE_MAX_ADR:
				tempController[BED3].pidDriveMax = write_value;
			break;
			case MODBUS_BED3_R_W_DRVE_MIN_ADR:
				tempController[BED3].pidDriveMin = write_value;
			break;
			case MODBUS_BED3_R_W_PID_MAX_ADR:
				tempController[BED3].pidMax = write_value;
			break;*/
		///////// TENSOMETER_0 //////////
		case MODBUS_TENS_0_FILAMENT_WEIGHT_R_W_ADR:
			tensometer[TENSOMETER_0].filamentWeight = write_value;
		break;
		case MODBUS_TENS_0_ROLL_WEIGHT_R_W_ADR:
			tensometer[TENSOMETER_0].rollWeight = write_value;
		break;
		case MODBUS_TENS_0_CALIBRATION_CONSTANT_R_W_ADR:
			tensometer[TENSOMETER_0].constantCalibrationValue = write_value;
		break;
		case MODBUS_TENS_0_MASS_NO_LOAD_GRAM_R_W_ADR:
			tensometer[TENSOMETER_0].zeroScaleMass = write_value;
		break;
		case MODBUS_TENS_0_CALIBRATION_MASS_GRAM_R_W_ADR:
			tensometer[TENSOMETER_0].calibatrionMass = write_value;
			calibration = 1;
			calibrating_tensometer = TENSOMETER_0;
		break;
		///////// TENSOMETER_1 //////////
		case MODBUS_TENS_1_FILAMENT_WEIGHT_R_W_ADR:
			tensometer[TENSOMETER_1].filamentWeight = write_value;
		break;
		case MODBUS_TENS_1_ROLL_WEIGHT_R_W_ADR:
			tensometer[TENSOMETER_1].rollWeight = write_value;
		break;
		case MODBUS_TENS_1_CALIBRATION_CONSTANT_R_W_ADR:
			tensometer[TENSOMETER_1].constantCalibrationValue = write_value;
		break;
		case MODBUS_TENS_1_MASS_NO_LOAD_GRAM_R_W_ADR:
			tensometer[TENSOMETER_1].zeroScaleMass = write_value;
		break;
		case MODBUS_TENS_1_CALIBRATION_MASS_GRAM_R_W_ADR:
			tensometer[TENSOMETER_1].calibatrionMass = write_value;
			calibration = 1;
			calibrating_tensometer = TENSOMETER_1;
		break;
	}
	modbus_puts(data);
	return 1;
}

int8_t mod_write_multiple_registers(char *data){
	
	uint16_t number_of_registers = data[5]*256+data[6]; //na przed ostatniej pozycji znajduje siê w jednym bajcie iloœæ rejestrów do odczytu
	uint16_t first_register = data[DATA_POS]*256+data[DATA_POS+1];

	uint16_t controller_ID = 0;
	float autotune_temp = 50;
	uint16_t tmpData=0;
	uint8_t save_to_eepr = 0;
	uint16_t maxCycles = 0;
	uint16_t tmp = 0;
	uint8_t j =0;
	autotunepidtable.controller_ID = 0;
	autotunepidtable.autotune_temp = 0;
	autotunepidtable.maxCycles = 0;
	autotunepidtable.save_to_eepr = 0;
	
	for (uint8_t i=0; i<number_of_registers; i++)
	{		
		switch(first_register+i){
			case MODBUS_SET_PID_AUTOTUNE_CONTROLLER_ID_ADR:
				controller_ID = ((data[8+j])<<8);
				j++;
				controller_ID |= (data[8+j]);
				autotunepidtable.controller_ID = controller_ID;
				
			break;
			case MODBUS_SET_PID_AUTOTUNE_TEMPERATURE_ADR:
				tmp = ((data[8+j])<<8);
				j++;
				tmp |= (data[8+j]);
				autotune_temp = (float)(tmp/100.0);
				autotunepidtable.autotune_temp = autotune_temp;
			break;
			case MODBUS_SET_PID_AUTOTUNE_MAX_CYCLES_AND_SAVE_ADR:
				maxCycles = ((data[8+j])<<8);
				j++;
				maxCycles |= (data[8+j]);
				if (maxCycles & 0x8000)
				{
					save_to_eepr = 1;
					maxCycles = maxCycles & 0x7FFF;
				}				
				autotunepidtable.maxCycles = maxCycles;
				autotunepidtable.save_to_eepr = save_to_eepr;
				autotune = 1;
			break;

		}	
		j++;
	}

	data[0] = 7; 
	data[data[0]] = calculate_LRC(data, data[0]); //obliczamy LRC wpisujac je na koncu struktury danych
	modbus_puts(data);
	return 1;
}

void get_int16_t_data(uint16_t set_data, uint8_t *dataPos, char *data){
	//uint16_t temp = (tempController[tempContrIndex].currentTemperatureC) * 100;
	//uint16_t temp = (tempController[tempContrIndex].currentTemperature);
	uint8_t first = set_data  >> 8;
	uint8_t dp = *dataPos;
	dp++;
	data[dp+3] = first;
	uint8_t second = set_data & 0x00ff;
	dp++;
	data[dp+3] = second;
	*dataPos = dp;
}

int8_t mod_read_input_register(char *data){
	
	const uint16_t number_of_registers = data[data[0]-1]; //na przed ostatniej pozycji znajduje siê w jednym bajcie iloœæ rejestrów do odczytu
	const uint16_t first_register = data[DATA_POS]*256+data[DATA_POS+1]; 
	uint8_t data_counter = 0;
	uint16_t tmp = 0;
	float tmpf = 0;
	for (uint8_t i=0; i<number_of_registers; i++)
	{
		switch(first_register+i){
			/*case MODBUS_EXT0_READ_TEMP_ADR:
				get_int16_t_data(150, &data_counter, data);
				//get_int16_t_data(tempController[EXT0].currentTemperature, &data_counter, data);			
			break;
			case MODBUS_EXT0_READ_TEMPC_ADR:
				get_int16_t_data(1550, &data_counter, data);
				//get_int16_t_data((uint16_t)(tempController[EXT0].currentTemperatureC*100), &data_counter, data);
			break;	
			case MODBUS_EXT1_READ_TEMP_ADR:
				get_int16_t_data(160, &data_counter, data);
				//get_int16_t_data(tempController[EXT1].currentTemperature, &data_counter, data);
			break;
			case MODBUS_EXT1_READ_TEMPC_ADR:
				get_int16_t_data(1650, &data_counter, data);
				//get_int16_t_data((uint16_t)(tempController[EXT1].currentTemperatureC*100), &data_counter, data);
			break;	
			case MODBUS_BED_READ_TEMP_ADR:
				tmp = ((tempController[BED0].currentTemperature+tempController[BED1].currentTemperature+tempController[BED2].currentTemperature+tempController[BED3].currentTemperature)/4);
				get_int16_t_data(170, &data_counter, data);	
				//get_int16_t_data(tmp, &data_counter, data);			
			break;
			case MODBUS_BED_READ_TEMPC_ADR:
				tmpf = (((tempController[BED0].currentTemperatureC+tempController[BED1].currentTemperatureC+tempController[BED2].currentTemperatureC+tempController[BED3].currentTemperatureC)/4)*100);
				get_int16_t_data(1750, &data_counter, data);
				//get_int16_t_data((uint16_t)tmpf, &data_counter, data);			
			break;							
			case MODBUS_BED0_READ_TEMP_ADR:
				get_int16_t_data((uint16_t)(tempController[BED0].currentTemperatureC*100), &data_counter, data);			
			break;
			case MODBUS_BED1_READ_TEMP_ADR:
				get_int16_t_data((uint16_t)(tempController[BED1].currentTemperatureC*100), &data_counter, data);		
			break;
			case MODBUS_BED2_READ_TEMP_ADR:
				get_int16_t_data((uint16_t)(tempController[BED2].currentTemperatureC*100), &data_counter, data);	
			break;
			case MODBUS_BED3_READ_TEMP_ADR:
				get_int16_t_data((uint16_t)(tempController[BED3].currentTemperatureC*100), &data_counter, data);
			break;
			case MODBUS_CHAMBER0_READ_TEMP_ADR:
				get_int16_t_data((uint16_t)(tempController[CHAMB0].currentTemperatureC*100), &data_counter, data);	
			break;	
			case MODBUS_AUTOTUNE_PID_STATUS:
				data_counter++;
				data[data_counter+3] = PIDTune_status >> 8;
				data_counter++;
				data[data_counter+3] = PIDTune_status & 0x00ff;	
			break;
			case MODBUS_EXT0_R_W_SENSOR_TYPE_ADR:
			get_int16_t_data((uint16_t)tempController[EXT0].sensorType, &data_counter, data);
			break;
			case MODBUS_EXT0_R_W_HEAT_MANAGER_ADR:
			get_int16_t_data((uint16_t)tempController[EXT0].heatManager, &data_counter, data);
			break;
			case MODBUS_EXT0_R_W_PID_P_ADR:
			get_int16_t_data((uint16_t)(tempController[EXT0].pidPGain*100.0), &data_counter, data);
			break;
			case MODBUS_EXT0_R_W_PID_I_ADR:
			get_int16_t_data((uint16_t)(tempController[EXT0].pidIGain*100.0), &data_counter, data);
			break;
			case MODBUS_EXT0_R_W_PID_D_ADR:
			get_int16_t_data((uint16_t)(tempController[EXT0].pidDGain*100.0), &data_counter, data);
			break;
			case MODBUS_EXT0_R_W_DRIVE_MAX_ADR:
			get_int16_t_data((uint16_t)tempController[EXT0].pidDriveMax, &data_counter, data);
			break;
			case MODBUS_EXT0_R_W_DRVE_MIN_ADR:
			get_int16_t_data((uint16_t)tempController[EXT0].pidDriveMin, &data_counter, data);
			break;
			case MODBUS_EXT0_R_W_PID_MAX_ADR:
			get_int16_t_data((uint16_t)tempController[EXT0].pidMax, &data_counter, data);
			break;
			//--------------EXT1---------------
			case MODBUS_EXT1_R_W_SENSOR_TYPE_ADR:
			get_int16_t_data((uint16_t)tempController[EXT1].sensorType, &data_counter, data);
			break;
			case MODBUS_EXT1_R_W_HEAT_MANAGER_ADR:
			get_int16_t_data((uint16_t)tempController[EXT1].heatManager, &data_counter, data);
			break;
			case MODBUS_EXT1_R_W_PID_P_ADR:
			get_int16_t_data((uint16_t)(tempController[EXT1].pidPGain*100.0), &data_counter, data);
			break;
			case MODBUS_EXT1_R_W_PID_I_ADR:
			get_int16_t_data((uint16_t)(tempController[EXT1].pidIGain*100.0), &data_counter, data);
			break;
			case MODBUS_EXT1_R_W_PID_D_ADR:
			get_int16_t_data((uint16_t)(tempController[EXT1].pidDGain*100.0), &data_counter, data);
			break;
			case MODBUS_EXT1_R_W_DRIVE_MAX_ADR:
			get_int16_t_data((uint16_t)tempController[EXT1].pidDriveMax, &data_counter, data);
			break;
			case MODBUS_EXT1_R_W_DRVE_MIN_ADR:
			get_int16_t_data((uint16_t)tempController[EXT1].pidDriveMin, &data_counter, data);
			break;
			case MODBUS_EXT1_R_W_PID_MAX_ADR:
			get_int16_t_data((uint16_t)tempController[EXT1].pidMax, &data_counter, data);
			break;
			//--------------BED---------------
			case MODBUS_BED_R_W_SENSOR_TYPE_ADR:
			get_int16_t_data((uint16_t)tempController[BED0].pidMax, &data_counter, data);
			//get_int16_t_data((uint16_t)tempController[BED1].pidMax, &data_counter, data);
			//get_int16_t_data((uint16_t)tempController[BED2].pidMax, &data_counter, data);
			//get_int16_t_data((uint16_t)tempController[BED3].pidMax, &data_counter, data);
			break;
			case MODBUS_BED_R_W_HEAT_MANAGER_ADR:
			get_int16_t_data((uint16_t)tempController[BED0].heatManager, &data_counter, data);
			//get_int16_t_data((uint16_t)tempController[BED1].heatManager, &data_counter, data);
			//get_int16_t_data((uint16_t)tempController[BED2].heatManager, &data_counter, data);
			//get_int16_t_data((uint16_t)tempController[BED3].heatManager, &data_counter, data);
			break;
			case MODBUS_BED_R_W_PID_P_ADR:
			get_int16_t_data((uint16_t)(tempController[BED0].pidPGain*100.0), &data_counter, data);
			//get_int16_t_data((uint16_t)tempController[BED1].pidIGain*100.0, &data_counter, data);
			//get_int16_t_data((uint16_t)tempController[BED2].pidIGain*100.0, &data_counter, data);
			//get_int16_t_data((uint16_t)tempController[BED3].pidIGain*100.0, &data_counter, data);
			break;
			case MODBUS_BED_R_W_PID_I_ADR:
			get_int16_t_data((uint16_t)(tempController[BED0].pidIGain*100.0), &data_counter, data);
			//get_int16_t_data((uint16_t)tempController[BED1].pidIGain*100.0, &data_counter, data);
			//get_int16_t_data((uint16_t)tempController[BED2].pidIGain*100.0, &data_counter, data);
			//get_int16_t_data((uint16_t)tempController[BED3].pidIGain*100.0, &data_counter, data);
			break;
			case MODBUS_BED_R_W_PID_D_ADR:
			get_int16_t_data((uint16_t)(tempController[BED0].pidDGain*100.0), &data_counter, data);
			//get_int16_t_data((uint16_t)tempController[BED1].pidDGain*100.0, &data_counter, data);
			//get_int16_t_data((uint16_t)tempController[BED2].pidDGain*100.0, &data_counter, data);
			//get_int16_t_data((uint16_t)tempController[BED3].pidDGain*100.0, &data_counter, data);
			break;
			case MODBUS_BED_R_W_DRIVE_MAX_ADR:
			get_int16_t_data((uint16_t)tempController[BED0].pidDriveMax, &data_counter, data);
			//get_int16_t_data((uint16_t)tempController[BED1].pidDriveMax, &data_counter, data);
			//get_int16_t_data((uint16_t)tempController[BED2].pidDriveMax, &data_counter, data);
			//get_int16_t_data((uint16_t)tempController[BED3].pidDriveMax, &data_counter, data);
			break;
			case MODBUS_BED_R_W_DRVE_MIN_ADR:
			get_int16_t_data((uint16_t)tempController[BED0].pidDriveMin, &data_counter, data);
			//get_int16_t_data((uint16_t)tempController[BED1].pidDriveMin, &data_counter, data);
			//get_int16_t_data((uint16_t)tempController[BED2].pidDriveMin, &data_counter, data);
			//get_int16_t_data((uint16_t)tempController[BED3].pidDriveMin, &data_counter, data);
			break;
			case MODBUS_BED_R_W_PID_MAX_ADR:
			get_int16_t_data((uint16_t)tempController[BED0].pidMax, &data_counter, data);
			//get_int16_t_data((uint16_t)tempController[BED1].pidMax, &data_counter, data);
			//get_int16_t_data((uint16_t)tempController[BED2].pidMax, &data_counter, data);
			//get_int16_t_data((uint16_t)tempController[BED3].pidMax, &data_counter, data);
			break;
			//--------------CHAMBER0---------------
			case MODBUS_CHAMBER0_R_W_SENSOR_TYPE_ADR:
			get_int16_t_data(tempController[CHAMB0].sensorType, &data_counter, data);
			break;
			case MODBUS_CHAMBER0_R_W_HEAT_MANAGER_ADR:
			get_int16_t_data(tempController[CHAMB0].heatManager, &data_counter, data);
			break;
			case MODBUS_CHAMBER0_R_W_PID_P_ADR:
			get_int16_t_data((uint16_t)(tempController[CHAMB0].pidPGain*100), &data_counter, data);
			break;
			case MODBUS_CHAMBER0_R_W_PID_I_ADR:
			get_int16_t_data((uint16_t)(tempController[CHAMB0].pidIGain*100), &data_counter, data);
			break;
			case MODBUS_CHAMBER0_R_W_PID_D_ADR:
			get_int16_t_data((uint16_t)(tempController[CHAMB0].pidDGain*100), &data_counter, data);
			break;
			case MODBUS_CHAMBER0_R_W_DRIVE_MAX_ADR:
			get_int16_t_data(tempController[CHAMB0].pidDriveMax, &data_counter, data);
			break;
			case MODBUS_CHAMBER0_R_W_DRVE_MIN_ADR:
			get_int16_t_data(tempController[CHAMB0].pidDriveMin, &data_counter, data);
			break;
			case MODBUS_CHAMBER0_R_W_PID_MAX_ADR:
			get_int16_t_data(tempController[CHAMB0].pidMax, &data_counter, data);
			break;

			//--------------BED0---------------
			case MODBUS_BED0_R_W_SENSOR_TYPE_ADR:
			get_int16_t_data((uint16_t)tempController[BED0].sensorType, &data_counter, data);
			break;
			case MODBUS_BED0_R_W_HEAT_MANAGER_ADR:
			get_int16_t_data((uint16_t)tempController[BED0].heatManager, &data_counter, data);
			break;
			case MODBUS_BED0_R_W_PID_P_ADR:
			get_int16_t_data((uint16_t)(tempController[BED0].pidPGain*100.0), &data_counter, data);
			break;
			case MODBUS_BED0_R_W_PID_I_ADR:
			get_int16_t_data((uint16_t)(tempController[BED0].pidIGain*100.0), &data_counter, data);
			break;
			case MODBUS_BED0_R_W_PID_D_ADR:
			get_int16_t_data((uint16_t)(tempController[BED0].pidDGain*100.0), &data_counter, data);
			break;
			case MODBUS_BED0_R_W_DRIVE_MAX_ADR:
			get_int16_t_data((uint16_t)tempController[BED0].pidDriveMax, &data_counter, data);
			break;
			case MODBUS_BED0_R_W_DRVE_MIN_ADR:
			get_int16_t_data((uint16_t)tempController[BED0].pidDriveMin, &data_counter, data);
			break;
			case MODBUS_BED0_R_W_PID_MAX_ADR:
			get_int16_t_data((uint16_t)tempController[BED0].pidMax, &data_counter, data);
			break;
			//--------------BED1---------------
			case MODBUS_BED1_R_W_SENSOR_TYPE_ADR:
			get_int16_t_data((uint16_t)tempController[BED1].sensorType, &data_counter, data);
			break;
			case MODBUS_BED1_R_W_HEAT_MANAGER_ADR:
			get_int16_t_data((uint16_t)tempController[BED1].heatManager, &data_counter, data);
			break;
			case MODBUS_BED1_R_W_PID_P_ADR:
			get_int16_t_data((uint16_t)(tempController[BED1].pidPGain*100.0), &data_counter, data);
			break;
			case MODBUS_BED1_R_W_PID_I_ADR:
			get_int16_t_data((uint16_t)(tempController[BED1].pidIGain*100.0), &data_counter, data);
			break;
			case MODBUS_BED1_R_W_PID_D_ADR:
			get_int16_t_data((uint16_t)(tempController[BED1].pidDGain*100.0), &data_counter, data);
			break;
			case MODBUS_BED1_R_W_DRIVE_MAX_ADR:
			get_int16_t_data((uint16_t)tempController[BED1].pidDriveMax, &data_counter, data);
			break;
			case MODBUS_BED1_R_W_DRVE_MIN_ADR:
			get_int16_t_data((uint16_t)tempController[BED1].pidDriveMin, &data_counter, data);
			break;
			case MODBUS_BED1_R_W_PID_MAX_ADR:
			get_int16_t_data((uint16_t)tempController[BED1].pidMax, &data_counter, data);
			break;
			//--------------BED2---------------
			case MODBUS_BED2_R_W_SENSOR_TYPE_ADR:
			get_int16_t_data((uint16_t)tempController[BED2].sensorType, &data_counter, data);
			break;
			case MODBUS_BED2_R_W_HEAT_MANAGER_ADR:
			get_int16_t_data((uint16_t)tempController[BED2].heatManager, &data_counter, data);
			break;
			case MODBUS_BED2_R_W_PID_P_ADR:
			get_int16_t_data((uint16_t)(tempController[BED2].pidPGain*100.0), &data_counter, data);
			break;
			case MODBUS_BED2_R_W_PID_I_ADR:
			get_int16_t_data((uint16_t)(tempController[BED2].pidIGain*100.0), &data_counter, data);
			break;
			case MODBUS_BED2_R_W_PID_D_ADR:
			get_int16_t_data((uint16_t)(tempController[BED2].pidDGain*100.0), &data_counter, data);
			break;
			case MODBUS_BED2_R_W_DRIVE_MAX_ADR:
			get_int16_t_data((uint16_t)tempController[BED2].pidDriveMax, &data_counter, data);
			break;
			case MODBUS_BED2_R_W_DRVE_MIN_ADR:
			get_int16_t_data((uint16_t)tempController[BED2].pidDriveMin, &data_counter, data);
			break;
			case MODBUS_BED2_R_W_PID_MAX_ADR:
			get_int16_t_data((uint16_t)tempController[BED2].pidMax, &data_counter, data);
			break;
			
			//--------------BED3---------------
			case MODBUS_BED3_R_W_SENSOR_TYPE_ADR:
			get_int16_t_data((uint16_t)tempController[BED3].sensorType, &data_counter, data);
			break;
			case MODBUS_BED3_R_W_HEAT_MANAGER_ADR:
			get_int16_t_data((uint16_t)tempController[BED3].heatManager, &data_counter, data);
			break;
			case MODBUS_BED3_R_W_PID_P_ADR:
			get_int16_t_data((uint16_t)(tempController[BED3].pidPGain*100.0), &data_counter, data);
			break;
			case MODBUS_BED3_R_W_PID_I_ADR:
			get_int16_t_data((uint16_t)(tempController[BED3].pidIGain*100.0), &data_counter, data);
			break;
			case MODBUS_BED3_R_W_PID_D_ADR:
			get_int16_t_data((uint16_t)(tempController[BED3].pidDGain*100.0), &data_counter, data);
			break;
			case MODBUS_BED3_R_W_DRIVE_MAX_ADR:
			get_int16_t_data((uint16_t)tempController[BED3].pidDriveMax, &data_counter, data);
			break;
			case MODBUS_BED3_R_W_DRVE_MIN_ADR:
			get_int16_t_data((uint16_t)tempController[BED3].pidDriveMin, &data_counter, data);
			break;
			case MODBUS_BED3_R_W_PID_MAX_ADR:
			get_int16_t_data((uint16_t)tempController[BED3].pidMax, &data_counter, data);
			break;*/
			///////// TENSOMETER_0 //////////
			case MODBUS_TENS_0_RAW_VALUE_WEIGHT_R_ADR:
				get_int16_t_data((uint16_t)tensometer[TENSOMETER_0].RAW_Value, &data_counter, data);
			break;
			case MODBUS_TENS_0_WEIGHT_GRAM_R_ADR:
				get_int16_t_data((uint16_t)tensometer[TENSOMETER_0].weight, &data_counter, data);
			break;
			case MODBUS_TENS_0_FILAMENT_WEIGHT_R_W_ADR:
				get_int16_t_data((uint16_t)tensometer[TENSOMETER_0].filamentWeight, &data_counter, data);
			break;
			case MODBUS_TENS_0_ROLL_WEIGHT_R_W_ADR:
				get_int16_t_data((uint16_t)tensometer[TENSOMETER_0].rollWeight, &data_counter, data);
			break;
			case MODBUS_TENS_0_CALIBRATION_CONSTANT_R_W_ADR:
				get_int16_t_data((uint16_t)tensometer[TENSOMETER_0].constantCalibrationValue, &data_counter, data);
			break;
			case MODBUS_TENS_0_MASS_NO_LOAD_GRAM_R_W_ADR:
				get_int16_t_data((uint16_t)tensometer[TENSOMETER_0].zeroScaleMass, &data_counter, data);
			break;
			case MODBUS_TENS_0_CALIBRATION_MASS_GRAM_R_W_ADR:
			get_int16_t_data((uint16_t)tensometer[TENSOMETER_0].calibatrionMass, &data_counter, data);
			break;
			///////// TENSOMETER_1 //////////
			case MODBUS_TENS_1_RAW_VALUE_WEIGHT_R_ADR:
				get_int16_t_data((uint16_t)tensometer[TENSOMETER_1].RAW_Value, &data_counter, data);
			break;
			case MODBUS_TENS_1_WEIGHT_GRAM_R_ADR:
				get_int16_t_data((uint16_t)tensometer[TENSOMETER_1].weight, &data_counter, data);
			break;
			case MODBUS_TENS_1_FILAMENT_WEIGHT_R_W_ADR:
				get_int16_t_data((uint16_t)tensometer[TENSOMETER_1].filamentWeight, &data_counter, data);
			break;
			case MODBUS_TENS_1_ROLL_WEIGHT_R_W_ADR:
				get_int16_t_data((uint16_t)tensometer[TENSOMETER_1].rollWeight, &data_counter, data);
			break;
			case MODBUS_TENS_1_CALIBRATION_CONSTANT_R_W_ADR:
				get_int16_t_data((uint16_t)tensometer[TENSOMETER_1].constantCalibrationValue, &data_counter, data);
			break;
			case MODBUS_TENS_1_MASS_NO_LOAD_GRAM_R_W_ADR:
				get_int16_t_data((uint16_t)tensometer[TENSOMETER_1].zeroScaleMass, &data_counter, data);
			break;
			case MODBUS_TENS_1_CALIBRATION_MASS_GRAM_R_W_ADR:
				get_int16_t_data((uint16_t)tensometer[TENSOMETER_1].calibatrionMass, &data_counter, data);
			break;
		}
	}
	data[0] = data_counter+4; //4 - stala liczba - 1 bajt adres slave-a, 1 bajt funkcja, 1 bajt liczba danych oraz 1 bajt LRC
	data[3] = data_counter;
	data[4+data_counter] = calculate_LRC(data, data[0]); //obliczamy LRC wpisujac je na koncu struktury danych
	modbus_puts(data);
	return 1;
}


//int8_t mod_read_register(char *data){
	//
	//const uint16_t number_of_registers = data[data[0]-1]; //na przed ostatniej pozycji znajduje siê w jednym bajcie iloœæ rejestrów do odczytu
	//const uint16_t first_register = data[DATA_POS]*256+data[DATA_POS+1];
	//uint8_t data_counter = 0;
	//uint16_t tmp = 0;
	//float tmpf = 0;
	//for (uint8_t i=0; i<number_of_registers; i++)
	//{
		//switch(first_register+i){
			////--------------EXT0---------------
			//case MODBUS_EXT0_R_W_SENSOR_TYPE_ADR:
			//get_int16_t_data((uint16_t)tempController[EXT0].sensorType, &data_counter, data);
			//break;
			//case MODBUS_EXT0_R_W_HEAT_MANAGER_ADR:
			//get_int16_t_data((uint16_t)tempController[EXT0].heatManager, &data_counter, data);
			//break;
			//case MODBUS_EXT0_R_W_PID_P_ADR:
			//get_int16_t_data((uint16_t)tempController[EXT0].pidPGain*100.0, &data_counter, data);
			//break;
			//case MODBUS_EXT0_R_W_PID_I_ADR:
			//get_int16_t_data((uint16_t)tempController[EXT0].pidIGain*100.0, &data_counter, data);
			//break;
			//case MODBUS_EXT0_R_W_PID_D_ADR:
			//get_int16_t_data((uint16_t)tempController[EXT0].pidDGain*100.0, &data_counter, data);
			//break;
			//case MODBUS_EXT0_R_W_DRIVE_MAX_ADR:
			//get_int16_t_data((uint16_t)tempController[EXT0].pidDriveMax, &data_counter, data);
			//break;
			//case MODBUS_EXT0_R_W_DRVE_MIN_ADR:
			//get_int16_t_data((uint16_t)tempController[EXT0].pidDriveMin, &data_counter, data);
			//break;
			//case MODBUS_EXT0_R_W_PID_MAX_ADR:
			//get_int16_t_data((uint16_t)tempController[EXT0].pidMax, &data_counter, data);
			//break;
			////--------------EXT1---------------
			//case MODBUS_EXT1_R_W_SENSOR_TYPE_ADR:
			//get_int16_t_data((uint16_t)tempController[EXT1].sensorType, &data_counter, data);
			//break;
			//case MODBUS_EXT1_R_W_HEAT_MANAGER_ADR:
			//get_int16_t_data((uint16_t)tempController[EXT1].heatManager, &data_counter, data);
			//break;
			//case MODBUS_EXT1_R_W_PID_P_ADR:
			//get_int16_t_data((uint16_t)tempController[EXT1].pidPGain*100.0, &data_counter, data);
			//break;
			//case MODBUS_EXT1_R_W_PID_I_ADR:
			//get_int16_t_data((uint16_t)tempController[EXT1].pidIGain*100.0, &data_counter, data);
			//break;
			//case MODBUS_EXT1_R_W_PID_D_ADR:
			//get_int16_t_data((uint16_t)tempController[EXT1].pidDGain*100.0, &data_counter, data);
			//break;
			//case MODBUS_EXT1_R_W_DRIVE_MAX_ADR:
			//get_int16_t_data((uint16_t)tempController[EXT1].pidDriveMax, &data_counter, data);
			//break;
			//case MODBUS_EXT1_R_W_DRVE_MIN_ADR:
			//get_int16_t_data((uint16_t)tempController[EXT1].pidDriveMin, &data_counter, data);
			//break;
			//case MODBUS_EXT1_R_W_PID_MAX_ADR:
			//get_int16_t_data((uint16_t)tempController[EXT1].pidMax, &data_counter, data);
			//break;
			////--------------BED---------------
			//case MODBUS_BED_R_W_SENSOR_TYPE_ADR:
				//get_int16_t_data((uint16_t)tempController[BED0].pidMax, &data_counter, data);
				//get_int16_t_data((uint16_t)tempController[BED1].pidMax, &data_counter, data);
				//get_int16_t_data((uint16_t)tempController[BED2].pidMax, &data_counter, data);
				//get_int16_t_data((uint16_t)tempController[BED3].pidMax, &data_counter, data);
			//break;
			//case MODBUS_BED_R_W_HEAT_MANAGER_ADR:
				//get_int16_t_data((uint16_t)tempController[BED0].heatManager, &data_counter, data);
				//get_int16_t_data((uint16_t)tempController[BED1].heatManager, &data_counter, data);
				//get_int16_t_data((uint16_t)tempController[BED2].heatManager, &data_counter, data);
				//get_int16_t_data((uint16_t)tempController[BED3].heatManager, &data_counter, data);
			//break;
			//case MODBUS_BED_R_W_PID_P_ADR:
				//get_int16_t_data((uint16_t)tempController[BED0].pidIGain*100.0, &data_counter, data);
				//get_int16_t_data((uint16_t)tempController[BED1].pidIGain*100.0, &data_counter, data);
				//get_int16_t_data((uint16_t)tempController[BED2].pidIGain*100.0, &data_counter, data);
				//get_int16_t_data((uint16_t)tempController[BED3].pidIGain*100.0, &data_counter, data);
			//break;
			//case MODBUS_BED_R_W_PID_I_ADR:
				//get_int16_t_data((uint16_t)tempController[BED0].pidIGain*100.0, &data_counter, data);
				//get_int16_t_data((uint16_t)tempController[BED1].pidIGain*100.0, &data_counter, data);
				//get_int16_t_data((uint16_t)tempController[BED2].pidIGain*100.0, &data_counter, data);
				//get_int16_t_data((uint16_t)tempController[BED3].pidIGain*100.0, &data_counter, data);
			//break;
			//case MODBUS_BED_R_W_PID_D_ADR:
				//get_int16_t_data((uint16_t)tempController[BED0].pidDGain*100.0, &data_counter, data);
				//get_int16_t_data((uint16_t)tempController[BED1].pidDGain*100.0, &data_counter, data);
				//get_int16_t_data((uint16_t)tempController[BED2].pidDGain*100.0, &data_counter, data);
				//get_int16_t_data((uint16_t)tempController[BED3].pidDGain*100.0, &data_counter, data);
			//break;
			//case MODBUS_BED_R_W_DRIVE_MAX_ADR:
				//get_int16_t_data((uint16_t)tempController[BED0].pidDriveMax, &data_counter, data);
				//get_int16_t_data((uint16_t)tempController[BED1].pidDriveMax, &data_counter, data);
				//get_int16_t_data((uint16_t)tempController[BED2].pidDriveMax, &data_counter, data);
				//get_int16_t_data((uint16_t)tempController[BED3].pidDriveMax, &data_counter, data);
			//break;
			//case MODBUS_BED_R_W_DRVE_MIN_ADR:
				//get_int16_t_data((uint16_t)tempController[BED0].pidDriveMin, &data_counter, data);
				//get_int16_t_data((uint16_t)tempController[BED1].pidDriveMin, &data_counter, data);
				//get_int16_t_data((uint16_t)tempController[BED2].pidDriveMin, &data_counter, data);
				//get_int16_t_data((uint16_t)tempController[BED3].pidDriveMin, &data_counter, data);
			//break;
			//case MODBUS_BED_R_W_PID_MAX_ADR:
				//get_int16_t_data((uint16_t)tempController[BED0].pidMax, &data_counter, data);
				//get_int16_t_data((uint16_t)tempController[BED1].pidMax, &data_counter, data);
				//get_int16_t_data((uint16_t)tempController[BED2].pidMax, &data_counter, data);
				//get_int16_t_data((uint16_t)tempController[BED3].pidMax, &data_counter, data);
			//break;
			////--------------CHAMBER0---------------
			//case MODBUS_CHAMBER0_R_W_SENSOR_TYPE_ADR:
			//get_int16_t_data(tempController[CHAMB0].sensorType, &data_counter, data);
			//break;
			//case MODBUS_CHAMBER0_R_W_HEAT_MANAGER_ADR:
			//get_int16_t_data(tempController[CHAMB0].heatManager, &data_counter, data);
			//break;
			//case MODBUS_CHAMBER0_R_W_PID_P_ADR:
			//get_int16_t_data(tempController[CHAMB0].pidPGain, &data_counter, data);
			//break;
			//case MODBUS_CHAMBER0_R_W_PID_I_ADR:
			//get_int16_t_data(tempController[CHAMB0].pidIGain, &data_counter, data);
			//break;
			//case MODBUS_CHAMBER0_R_W_PID_D_ADR:
			//get_int16_t_data(tempController[CHAMB0].pidDGain, &data_counter, data);
			//break;
			//case MODBUS_CHAMBER0_R_W_DRIVE_MAX_ADR:
			//get_int16_t_data(tempController[CHAMB0].pidDriveMax, &data_counter, data);
			//break;
			//case MODBUS_CHAMBER0_R_W_DRVE_MIN_ADR:
			//get_int16_t_data(tempController[CHAMB0].pidDriveMin, &data_counter, data);
			//break;
			//case MODBUS_CHAMBER0_R_W_PID_MAX_ADR:
			//get_int16_t_data(tempController[CHAMB0].pidMax, &data_counter, data);
			//break;
//
			////--------------BED0---------------
			//case MODBUS_BED0_R_W_SENSOR_TYPE_ADR:
			//get_int16_t_data((uint16_t)tempController[BED0].sensorType, &data_counter, data);
			//break;
			//case MODBUS_BED0_R_W_HEAT_MANAGER_ADR:
			//get_int16_t_data((uint16_t)tempController[BED0].heatManager, &data_counter, data);
			//break;
			//case MODBUS_BED0_R_W_PID_P_ADR:
			//get_int16_t_data((uint16_t)tempController[BED0].pidPGain*100.0, &data_counter, data);
			//break;
			//case MODBUS_BED0_R_W_PID_I_ADR:
			//get_int16_t_data((uint16_t)tempController[BED0].pidIGain*100.0, &data_counter, data);
			//break;
			//case MODBUS_BED0_R_W_PID_D_ADR:
			//get_int16_t_data((uint16_t)tempController[BED0].pidDGain*100.0, &data_counter, data);
			//break;
			//case MODBUS_BED0_R_W_DRIVE_MAX_ADR:
			//get_int16_t_data((uint16_t)tempController[BED0].pidDriveMax, &data_counter, data);
			//break;
			//case MODBUS_BED0_R_W_DRVE_MIN_ADR:
			//get_int16_t_data((uint16_t)tempController[BED0].pidDriveMin, &data_counter, data);
			//break;
			//case MODBUS_BED0_R_W_PID_MAX_ADR:
			//get_int16_t_data((uint16_t)tempController[BED0].pidMax, &data_counter, data);
			//break;
			////--------------BED1---------------
			//case MODBUS_BED1_R_W_SENSOR_TYPE_ADR:
			//get_int16_t_data((uint16_t)tempController[BED1].sensorType, &data_counter, data);
			//break;
			//case MODBUS_BED1_R_W_HEAT_MANAGER_ADR:
			//get_int16_t_data((uint16_t)tempController[BED1].heatManager, &data_counter, data);
			//break;
			//case MODBUS_BED1_R_W_PID_P_ADR:
			//get_int16_t_data((uint16_t)tempController[BED1].pidPGain*100.0, &data_counter, data);
			//break;
			//case MODBUS_BED1_R_W_PID_I_ADR:
			//get_int16_t_data((uint16_t)tempController[BED1].pidIGain*100.0, &data_counter, data);
			//break;
			//case MODBUS_BED1_R_W_PID_D_ADR:
			//get_int16_t_data((uint16_t)tempController[BED1].pidDGain*100.0, &data_counter, data);
			//break;
			//case MODBUS_BED1_R_W_DRIVE_MAX_ADR:
			//get_int16_t_data((uint16_t)tempController[BED1].pidDriveMax, &data_counter, data);
			//break;
			//case MODBUS_BED1_R_W_DRVE_MIN_ADR:
			//get_int16_t_data((uint16_t)tempController[BED1].pidDriveMin, &data_counter, data);
			//break;
			//case MODBUS_BED1_R_W_PID_MAX_ADR:
			//get_int16_t_data((uint16_t)tempController[BED1].pidMax, &data_counter, data);
			//break;
			////--------------BED2---------------
			//case MODBUS_BED2_R_W_SENSOR_TYPE_ADR:
			//get_int16_t_data((uint16_t)tempController[BED2].sensorType, &data_counter, data);
			//break;
			//case MODBUS_BED2_R_W_HEAT_MANAGER_ADR:
			//get_int16_t_data((uint16_t)tempController[BED2].heatManager, &data_counter, data);
			//break;
			//case MODBUS_BED2_R_W_PID_P_ADR:
			//get_int16_t_data((uint16_t)tempController[BED2].pidPGain*100.0, &data_counter, data);
			//break;
			//case MODBUS_BED2_R_W_PID_I_ADR:
			//get_int16_t_data((uint16_t)tempController[BED2].pidIGain*100.0, &data_counter, data);
			//break;
			//case MODBUS_BED2_R_W_PID_D_ADR:
			//get_int16_t_data((uint16_t)tempController[BED2].pidDGain*100.0, &data_counter, data);
			//break;
			//case MODBUS_BED2_R_W_DRIVE_MAX_ADR:
			//get_int16_t_data((uint16_t)tempController[BED2].pidDriveMax, &data_counter, data);
			//break;
			//case MODBUS_BED2_R_W_DRVE_MIN_ADR:
			//get_int16_t_data((uint16_t)tempController[BED2].pidDriveMin, &data_counter, data);
			//break;
			//case MODBUS_BED2_R_W_PID_MAX_ADR:
			//get_int16_t_data((uint16_t)tempController[BED2].pidMax, &data_counter, data);
			//break;
			//
			////--------------BED3---------------
			//case MODBUS_BED3_R_W_SENSOR_TYPE_ADR:
			//get_int16_t_data((uint16_t)tempController[BED3].sensorType, &data_counter, data);
			//break;
			//case MODBUS_BED3_R_W_HEAT_MANAGER_ADR:
			//get_int16_t_data((uint16_t)tempController[BED3].heatManager, &data_counter, data);
			//break;
			//case MODBUS_BED3_R_W_PID_P_ADR:
			//get_int16_t_data((uint16_t)tempController[BED3].pidPGain*100.0, &data_counter, data);
			//break;
			//case MODBUS_BED3_R_W_PID_I_ADR:
			//get_int16_t_data((uint16_t)tempController[BED3].pidIGain*100.0, &data_counter, data);
			//break;
			//case MODBUS_BED3_R_W_PID_D_ADR:
			//get_int16_t_data((uint16_t)tempController[BED3].pidDGain*100.0, &data_counter, data);
			//break;
			//case MODBUS_BED3_R_W_DRIVE_MAX_ADR:
			//get_int16_t_data((uint16_t)tempController[BED3].pidDriveMax, &data_counter, data);
			//break;
			//case MODBUS_BED3_R_W_DRVE_MIN_ADR:
			//get_int16_t_data((uint16_t)tempController[BED3].pidDriveMin, &data_counter, data);
			//break;
			//case MODBUS_BED3_R_W_PID_MAX_ADR:
			//get_int16_t_data((uint16_t)tempController[BED3].pidMax, &data_counter, data);
			//break;
//
//
//
//
		//}
	//}
	//data[0] = data_counter+4; //4 - stala liczba - 1 bajt adres slave-a, 1 bajt funkcja, 1 bajt liczba danych oraz 1 bajt LRC
	//data[3] = data_counter;
	//data[4+data_counter] = calculate_LRC(data, data[0]); //obliczamy LRC wpisujac je na koncu struktury danych
	//modbus_puts(data);
	//return 1;
//}

void parse_uart_data( char *pBuf ) {

	if ((pBuf[ADDRESS_POS] == SLAVE_ADDRESS) || (pBuf[ADDRESS_POS] == ALL_SLAVES_ADR))
	{
		if (pBuf[pBuf[0]] == calculate_LRC(pBuf, pBuf[0]))
		{
			uint16_t tmp = pBuf[FUNCT_POS];
			switch(pBuf[FUNCT_POS]){
				case READ_COILS:
					//mod_read_coils(pBuf[FUNCT_POS], data);
					break;
	
				case READ_DISCRETE_IMPUTS:
					//mod_read_discrete_inputs(pBuf[FUNCT_POS], data);
					break;
				
				case READ_HOLDING_REGISTER:
					//mod_read_holding_register(pBuf[FUNCT_POS],data);
					mod_read_input_register(pBuf);
					break;
				
				case READ_INPUT_REGISTER:
					mod_read_input_register(pBuf);
					break;
				
				case WRITE_SINGLE_COILS:
					break;
				
				case WRITE_SINGLE_REGISTER:
					mod_write_single_register(pBuf);
					break;
				
				case WIRTE_MULTIPLE_COILS:
					break;
				
				case WRITE_MULTIPLE_REGISTERS:
					mod_write_multiple_registers(pBuf);
					break;
				
				case DIAGNOSTIC:
					break;
				
				//default:
		
			}
		}
	}
}
