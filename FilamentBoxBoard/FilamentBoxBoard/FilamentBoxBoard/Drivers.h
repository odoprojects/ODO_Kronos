#ifndef DRIVERS_H_
#define DRIVERS_H_

#include <avr/io.h>

#define DIR_0	0
#define DIR_1	1

#define DRIVER_0	0
#define DRIVER_1	1

#define DRIVER_COSNTANT_DELAY 10

#define FILAMENT_DRIVER_STATUS				0xE0

#define IDLE_DRIVER_STATUS					0x80
#define PRINTING_DRIVER_STATUS				0x40
#define PULL_OUT_DRIVER_STATUS				0x20



#define STEPS0_NEEDED_MASK		1
#define STEPS1_NEEDED_MASK		2
#define DRIVER_HIGH_DELAY_US	2

#define DRIVER_TIMEOUT			10000

void register_filament_drivers_event_callback(void (*callback)());
void FILAMENT_DRIVERS_EVENT();
void manageFilamentDrivers();
void initializeDrivers();

extern uint8_t filament_driver_0_status;
extern uint8_t filament_driver_1_status;

extern volatile uint8_t step_tick0;
extern volatile uint8_t step_tick1;

extern volatile uint8_t step0;
extern volatile uint8_t step1;

inline void change_driver_speed(uint32_t speed){
	driver_speed = speed;
}


#endif // DRIVERS_H_INCLUDED
