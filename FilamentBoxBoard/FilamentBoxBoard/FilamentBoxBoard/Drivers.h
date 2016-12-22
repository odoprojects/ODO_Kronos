#ifndef DRIVERS_H_
#define DRIVERS_H_

#include <avr/io.h>

#define DIR_0	0
#define DIR_1	1

#define DRIVER_0	0
#define DRIVER_1	1

#define DRIVER_COSNTANT_DELAY 10


void register_filament_drivers_event_callback(void (*callback)());
void FILAMENT_DRIVERS_EVENT();
void manageFilamentDrivers();
void initializeDrivers();

extern uint8_t filament_driver_status;
extern volatile uint32_t driver_speed;
extern volatile uint8_t driver0_enabled;
extern volatile uint8_t driver1_enabled;
/*
void gotoPosition(float newPos)
{
	enable();
	int32_t target = floor(newPos * stepsPerMM + 0.5f) - position;
	position += target;
	if(target > 0) {
		HAL::digitalWrite(dirPin, !invertDir);
		} else {
		target = -target;
		HAL::digitalWrite(dirPin, invertDir);
	}
	while(target) {
		HAL::digitalWrite(stepPin, HIGH);
		HAL::delayMicroseconds(delayUS);
		HAL::digitalWrite(stepPin, LOW);
		HAL::delayMicroseconds(delayUS);
		target--;
		HAL::pingWatchdog();
		if((target & 127) == 0)
		Commands::checkForPeriodicalActions(false);
	}
}
void enable()
{
	HAL::digitalWrite(enablePin, invertEnable);
}
void disable()
{
	HAL::digitalWrite(enablePin, !invertEnable);
}
*/

#endif // DRIVERS_H_INCLUDED
