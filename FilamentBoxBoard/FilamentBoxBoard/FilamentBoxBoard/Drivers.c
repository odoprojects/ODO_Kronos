#include "Drivers.h"
#include "Pins.h" 
#include <avr/io.h>

uint8_t filament_driver_status = 0;
volatile uint32_t driver_speed = 0;
volatile uint8_t driver0_enabled = 0;
volatile uint8_t driver1_enabled = 0;

static void (*filament_drivers_event_callback)();


void register_filament_drivers_event_callback(void (*callback)()) {
	filament_drivers_event_callback = callback;
}

void FILAMENT_DRIVERS_EVENT() {
	if( filament_drivers_event_callback ) {
		(*filament_drivers_event_callback)();
	}
}

void disable_driver(uint8_t driver){
	switch(driver){
		case DRIVER_0:{
			DRIVER0_ENABLE_PIN_LOW;
			driver0_enabled = 0;
			break;
		}
		case DRIVER_1:{
			DRIVER1_ENABLE_PIN_LOW;
			driver1_enabled = 0;
			break;
		}
	}
}

void enable_driver(uint8_t driver){
	switch(driver){
		case DRIVER_0:{
			DRIVER0_ENABLE_PIN_HIGH;
			driver0_enabled = 1;
			break;
		}
		case DRIVER_1:{
			DRIVER1_ENABLE_PIN_HIGH;
			driver1_enabled = 1;
			break;
		}
	}
}

void set_dir_driver(uint8_t driver ,uint8_t dir){
	switch(driver){
		case DRIVER_0:{
			if(dir == DIR_0){		
				DRIVER0_DIR_PIN_LOW;
			}else if(dir == DIR_1){
				DRIVER0_DIR_PIN_HIGH;
			}
			break;
		}
		case DRIVER_1:{
			if(dir == DIR_0){
				DRIVER1_DIR_PIN_LOW;
			}else if(dir == DIR_1){
				DRIVER1_DIR_PIN_HIGH;
			}
			break;
		}
	}
}

void initializeDrivers() {
	PIN_OUT(DRIVER0_STEP_PIN);
	PIN_OUT(DRIVER0_DIR_PIN);
	PIN_OUT(DRIVER0_ENABLE_PIN);
	
	PIN_OUT(DRIVER1_STEP_PIN);
	PIN_OUT(DRIVER1_DIR_PIN);
	PIN_OUT(DRIVER1_ENABLE_PIN);

	DRIVER0_STEP_PIN_LOW;
	DRIVER0_DIR_PIN_LOW;
	DRIVER0_ENABLE_PIN_LOW;
	
	DRIVER1_STEP_PIN_LOW;
	DRIVER1_DIR_PIN_LOW;
	DRIVER1_ENABLE_PIN_LOW;
	
	driver_speed = DRIVER_COSNTANT_DELAY;
}

void manageFilamentDrivers(){
	
}

