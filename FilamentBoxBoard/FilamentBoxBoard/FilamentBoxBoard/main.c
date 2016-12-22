/*
 * AUX_Board.c
 *
 * Created: 19.08.2016 11:09:36
 * Author : pieru
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "eeprom.h"
#include "hardware.h"
#include "Pins.h"
#include "mkuart.h"
#include <util/delay.h>
#include "modbus.h"
#include "temperature_control.h"
#include "Configuration.h"
#include "tensometer.h"
#include "softSPI.h"
#include "Drivers.h"
#include "RFID_RC522.h"

//volatile uint8_t x = 0;

volatile uint32_t t1 = 0;
long baudrate = BAUDRATE;         ///< Communication speed rate.
//extern uint16_t osAnalogInputValues[ANALOG_INPUTS];
//extern struct TemperatureController tempController[ANALOG_INPUTS];

// uint8_t SelfTestBuffer[64];
// void handleRFID(uint8_t byte, uint8_t *str);
int main(void)
{
	
	
	uint32_t tempData;
	register_uart_str_rx_event_callback( parse_uart_data );
	register_manage_tensometer_event_callback(manageTensometer);
	register_filament_drivers_event_callback(manageFilamentDrivers);
	init();
	
// 	uint8_t byte;
// 	uint8_t str[MAX_LEN];
// 	_delay_ms(50);
// 	spi_init();
// 	_delay_ms(1000);
// 	mfrc522_init();
// 	
// 	//check version of the reader
// 	byte = mfrc522_read(VersionReg);
// 	byte = mfrc522_read(ComIEnReg);
// 	mfrc522_write(ComIEnReg,byte|0x20);
// 	byte = mfrc522_read(DivIEnReg);
// 	mfrc522_write(DivIEnReg,byte|0x80);
// 	_delay_ms(1500);
/*
	tempController[EXT0].pidPGain = 38.21;
	tempController[EXT0].pidIGain = 8.45;
	tempController[EXT0].pidDGain = 43.18;
	
	#define BED_P	64.36f
	#define BED_I	28.59f
	#define BED_D	36.21f
	
	tempController[BED0].pidPGain = BED_P;
	tempController[BED0].pidIGain = BED_I;
	tempController[BED0].pidDGain = BED_D;

	tempController[BED1].pidPGain = BED_P;
	tempController[BED1].pidIGain = BED_I;
	tempController[BED1].pidDGain = BED_D;

	tempController[BED2].pidPGain = BED_P;
	tempController[BED2].pidIGain = BED_I;
	tempController[BED2].pidDGain = BED_D;

	tempController[BED3].pidPGain = BED_P;
	tempController[BED3].pidIGain = BED_I;
	tempController[BED3].pidDGain = BED_D;
*/
	//modbus_puts_s("Witaj");
	
	//initMFRC522();
	
	initializeDrivers();
	offset_calibration();
    Filament_measure_timer = 1000;
	
	while (1) 
    {		
		UART_RX_STR_EVENT(bufor) ;	// zdarzenie odbiorcze MODBUS UART3
		MANAGE_TENSOMETER_EVENT();
		FILAMENT_DRIVERS_EVENT();
		//MANAGE_RFID_EVENT();
	/*	if (!Filament_measure_timer)	// wstawic Filament_measure_timer
		{		
			make_tensometer_measure();
			Filament_measure_timer = TENSOMETER_MEASURE_INTERVAL;		
			//ADS_SCLK_TOG;
		}*/
/*		
		if (autotune)
		{
			autotunePID(&tempController[autotunepidtable.controller_ID],  autotunepidtable.autotune_temp, autotunepidtable.controller_ID, autotunepidtable.maxCycles, autotunepidtable.save_to_eepr);
			autotune = 0;
		}
		
		if (door_status & DOOR_CLOSE_OPEN_MASK)
		{
			if (!door_setup)
			{
				Door_move_timer = DOOR_TIME;
				Door_delay_timer = DOOR_DELAY_OPEN;
				door_setup = 1;
			}else
			{
				if (Door_move_timer)
				{
					if (door_status & DOOR_OPEN_MASK)
					{
						MOTUP0_PIN_HIGH;
						MOTPWM0_PIN_HIGH;
						if (!Door_delay_timer)
						{
							MOTUP1_PIN_HIGH;
							MOTPWM1_PIN_HIGH;
						}
					}
					if (door_status & DOOR_CLOSE_MASK)
					{
						MOTDWN0_PIN_HIGH;
						MOTPWM0_PIN_HIGH;
						if (!Door_delay_timer)
						{
							MOTDWN1_PIN_HIGH;
							MOTPWM1_PIN_HIGH;
						}
					}
				}else
				{
					door_setup = 0;
					if (door_status & DOOR_OPEN_MASK)
					{
						MOTPWM0_PIN_LOW;
						MOTPWM1_PIN_LOW;
						MOTUP0_PIN_LOW;
						MOTUP1_PIN_LOW;
						door_status &= ~DOOR_OPEN_MASK;
					}
					if (door_status & DOOR_CLOSE_MASK)
					{
						MOTPWM0_PIN_LOW;
						MOTPWM1_PIN_LOW;
						MOTDWN0_PIN_LOW;
						MOTDWN1_PIN_LOW;
						door_status &= ~DOOR_CLOSE_MASK;
					}
				}
				
			}
		}
		*/
    }
}
