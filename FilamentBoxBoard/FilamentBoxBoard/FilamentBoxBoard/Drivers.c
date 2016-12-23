#include "Drivers.h"
#include "Pins.h" 
#include <avr/io.h>

uint8_t filament_driver_status = 0;
volatile uint32_t driver_speed = 0;




static void (*filament_drivers_event_callback)();


void register_filament_drivers_event_callback(void (*callback)()) {
	filament_drivers_event_callback = callback;
}

void FILAMENT_DRIVERS_EVENT() {
	if( filament_drivers_event_callback ) {
		(*filament_drivers_event_callback)();
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
	DRIVER0_DISABLE;
	
	DRIVER1_STEP_PIN_LOW;
	DRIVER1_DIR_PIN_LOW;
	DRIVER1_DISABLE;
	
	PIN_IN(FILAMENT_ENDSTOP_0_E_PIN);
	PIN_IN(FILAMENT_ENDSTOP_0_P_PIN);
	PIN_IN(FILAMENT_ENDSTOP_1_E_PIN);
	PIN_IN(FILAMENT_ENDSTOP_1_P_PIN);
	
	FILAMENT_ENDSTOP_0_E_PIN_LOW;
	FILAMENT_ENDSTOP_0_P_PIN_LOW;
	FILAMENT_ENDSTOP_1_E_PIN_LOW;
	FILAMENT_ENDSTOP_1_P_PIN_LOW;
	driver_speed = DRIVER_COSNTANT_DELAY;
}


void manageFilamentDrivers(){
	
	if (step0)
	{
		filament_driver_0_timer = DRIVER_TIMEOUT;
		step0 = 0;
	}
	if (step1)
	{
		filament_driver_1_timer = DRIVER_TIMEOUT;
		step1 = 0;
	}
	if (!filament_driver_0_timer)
	{
		filament_driver_0_status &= ~FILAMENT_DRIVER_STATUS;
		filament_driver_0_status |= IDLE_DRIVER_STATUS;
	}
	if (!filament_driver_1_timer)
	{
		filament_driver_1_status &= ~FILAMENT_DRIVER_STATUS;
		filament_driver_1_status |= IDLE_DRIVER_STATUS;
	}
	
	if(filament_driver_0_status && IDLE_DRIVER_STATUS){
		DRIVER0_DISABLE;
	}else if(filament_driver_0_status && PRINTING_DRIVER_STATUS){
		DRIVER0_ENABLE;
		DRIVER0_DIR_PIN_LOW;
	}else if(filament_driver_0_status && PULL_OUT_DRIVER_STATUS){
		DRIVER0_ENABLE;
		DRIVER0_DIR_PIN_HIGH;
	}
	
	if(filament_driver_1_status && IDLE_DRIVER_STATUS){
		DRIVER1_DISABLE;
	}else if(filament_driver_1_status && PRINTING_DRIVER_STATUS){
		DRIVER1_ENABLE;
		DRIVER1_DIR_PIN_LOW;
	}else if(filament_driver_1_status && PULL_OUT_DRIVER_STATUS){
		DRIVER1_ENABLE;
		DRIVER1_DIR_PIN_HIGH;
	}
}

