/*
 * hardware.c
 *
 * Created: 17.08.2016 12:34:04
 *  Author: pieru
 */ 

#include <avr/io.h>
//#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "hardware.h"
#include "Pins.h"
#include "Configuration.h"
#include "temperature_control.h"


volatile uint32_t millis = 0;

volatile uint16_t Door_move_timer = 0;
volatile uint16_t Door_delay_timer = 0;
volatile uint16_t Filament_measure_timer = 0;

uint8_t door_status = 0; 
uint8_t door_setup = 0;
//*************************************************************************
//******************  I/O *************************************************
//*************************************************************************


void Pin_init(){
	
/*
	PIN_OUT(EXT0_PWM_PIN);
	EXT0_PWM_PIN_LOW;
	PIN_OUT(EXT1_PWM_PIN);
	EXT1_PWM_PIN_LOW;
	PIN_OUT(BED0_PWM_PIN);
	BED0_PWM_PIN_LOW;
	PIN_OUT(BED1_PWM_PIN);
	BED1_PWM_PIN_LOW;
	PIN_OUT(BED2_PWM_PIN);
	BED2_PWM_PIN_LOW;
	PIN_OUT(BED3_PWM_PIN);
	BED3_PWM_PIN_LOW;
	
	PIN_OUT(MOTUP0_PIN);
	MOTUP0_PIN_LOW;
	
	PIN_OUT(MOTDWN0_PIN);
	MOTDWN0_PIN_LOW;
	
	PIN_OUT(MOTUP1_PIN);
	MOTUP1_PIN_LOW;
	
	PIN_OUT(MOTDWN1_PIN);
	MOTDWN1_PIN_LOW;
	
	PIN_OUT(MOTPWM0_PIN);
	MOTPWM0_PIN_LOW;
	
	PIN_OUT(MOTPWM1_PIN);
	MOTPWM1_PIN_LOW;
	
	PIN_OUT(LED0_PIN);
	LED0_PIN_HIGH;
*/
}

void set_default_pins(){
	DDRA |= 0xFF; //ustawiamy PORTA jako wyjscie
	PORTA &= ~(0xFF); //ustawiamy logiczne 0 na calym porcie
	
	DDRB |= 0xF1;	//ustawiam 1111 0001 ¿eby nie zmieniaæ MISO MOSI SCK
	PORTB &= ~(0xF1);
	
	DDRC |= 0xFF; //ustawiamy PORTC jako wyjscie
	PORTC &= ~(0xFF);//ustawiamy logiczne 0 na calym porcie
	
	DDRD |= 0xFF; //ustawiamy PORTD jako wyjscie
	PORTD &= ~(0xFF);//ustawiamy logiczne 0 na calym porcie
	
	DDRE |= 0xFF; //ustawiamy PORTE jako wyjscie
	PORTE &= ~(0xFF);//ustawiamy logiczne 0 na calym porcie
	
	//Analogów nie ruszamy!!!
	//DDRF |= 0xFF; //ustawiamy PORTF jako wyjscie
	//PORTF &= ~(0xFF);//ustawiamy logiczne 0 na calym porcie
	
	DDRG |= 0xFF; //ustawiamy PORTG jako wyjscie
	PORTG &= ~(0xFF);//ustawiamy logiczne 0 na calym porcie
	
	DDRH |= 0xFF; //ustawiamy PORTH jako wyjscie
	PORTH &= ~(0xFF);//ustawiamy logiczne 0 na calym porcie
	
	DDRJ |= 0xFF; //ustawiamy PORTJ jako wyjscie
	PORTJ &= ~(0xFF);//ustawiamy logiczne 0 na calym porcie
	
	//Analogów nie ruszamy!!!
	//DDRK |= 0xFF; //ustawiamy PORTK jako wyjscie
	//PORTK &= ~(0xFF);//ustawiamy logiczne 0 na calym porcie
	
	DDRL |= 0xFF; //ustawiamy PORTL jako wyjscie
	PORTL &= ~(0xFF);//ustawiamy logiczne 0 na calym porcie
}

//*************************************************************************
//******************  Timer *************************************************
//*************************************************************************

void Timer0_init(){
	//Timer0 comp A to heater pid and pwm management
	//prescaler 64, interrupt on, and interrupt period set to 256us
	//TCCR0A = 0x00;  // Setup PWM interrupt
 	TCCR0A |= (1<<WGM01);
 	TCCR0B |= WART_PRESCALER_TIM0; //prescaler 64
 	OCR0A = 64; // inetrrupt 256us
 	TIMSK0 |= (1<<OCIE0A);
//    EXTRUDER_TCCR = 0; // need Normal not fastPWM set by arduino init
//    EXTRUDER_TIMSK |= (1<<EXTRUDER_OCIE); // Activate compa interrupt on timer 0
    
 //   PWM_TCCR = 0;  // Setup PWM interrupt
  //  PWM_OCR = 64;
//    PWM_TIMSK |= (1<<PWM_OCIE);

}

void Timer1_init(){
	TCCR1B |= (1<<WGM12); //tryb pracy CTC
	TCCR1B |= WART_PRESCALER_TIM1;
	OCR1A = WART_OCR1;
	TIMSK1 = (1<<OCIE1A);
	
}



ISR(TIMER1_COMPA_vect){
	uint16_t n;

	millis++;

	n = Door_move_timer;	
	if (n) Door_move_timer = --n;
	n = Door_delay_timer;		
	if (n) Door_delay_timer = --n;
	n = Filament_measure_timer;
	if (n) Filament_measure_timer = --n;
	
}



ISR(PWM_TIMER_VECTOR){
	static uint8_t pwm_count_cooler = 0;
	static uint8_t pwm_count_heater = 0;
	static uint8_t pwm_pos_set[NUM_PWM];
	
	static uint8_t pwm_cooler_pos_set[NUM_EXTRUDER];


	OCR0A += 64;

	if(pwm_count_heater==0)
	{
		if((pwm_pos_set[0] = (pwm_pos[0] & HEATER_PWM_MASK)) > 0) EXT0_PWM_PIN_HIGH;
		if((pwm_pos_set[1] = (pwm_pos[1] & HEATER_PWM_MASK)) > 0) EXT1_PWM_PIN_HIGH;
		if((pwm_pos_set[2] = (pwm_pos[2] & HEATER_PWM_MASK)) > 0) BED0_PWM_PIN_HIGH;
		if((pwm_pos_set[3] = (pwm_pos[3] & HEATER_PWM_MASK)) > 0) BED1_PWM_PIN_HIGH;
		if((pwm_pos_set[4] = (pwm_pos[4] & HEATER_PWM_MASK)) > 0) BED2_PWM_PIN_HIGH;
		if((pwm_pos_set[5] = (pwm_pos[5] & HEATER_PWM_MASK)) > 0) BED3_PWM_PIN_HIGH;
	}
	if(pwm_count_cooler == 0 )
	{   
		if((pwm_cooler_pos_set[0] = (fan[0].coolerPWM & COOLER_PWM_MASK)) > 0) EXT0_FAN_PWM_PIN_HIGH;
		if((pwm_cooler_pos_set[1] = (fan[1].coolerPWM & COOLER_PWM_MASK)) > 0) EXT1_FAN_PWM_PIN_HIGH;
	}
		
	if(pwm_pos_set[0] == pwm_count_heater && pwm_pos_set[0] != HEATER_PWM_MASK) EXT0_PWM_PIN_LOW;
	if(pwm_cooler_pos_set[0] == pwm_count_cooler && pwm_cooler_pos_set[0] != COOLER_PWM_MASK) EXT0_FAN_PWM_PIN_LOW;
	
	if(pwm_pos_set[1] == pwm_count_heater && pwm_pos_set[1] != HEATER_PWM_MASK) EXT1_PWM_PIN_LOW;
	if(pwm_cooler_pos_set[1] == pwm_count_cooler && pwm_cooler_pos_set[1] != COOLER_PWM_MASK) EXT1_FAN_PWM_PIN_LOW;

	if(pwm_pos_set[2] == pwm_count_heater && pwm_pos_set[2] != HEATER_PWM_MASK) BED0_PWM_PIN_LOW;
	if(pwm_pos_set[3] == pwm_count_heater && pwm_pos_set[3] != HEATER_PWM_MASK) BED1_PWM_PIN_LOW;
	if(pwm_pos_set[4] == pwm_count_heater && pwm_pos_set[4] != HEATER_PWM_MASK) BED2_PWM_PIN_LOW;
	if(pwm_pos_set[5] == pwm_count_heater && pwm_pos_set[5] != HEATER_PWM_MASK) BED3_PWM_PIN_LOW;
	
	pwm_count_cooler += COOLER_PWM_STEP;
	pwm_count_heater += HEATER_PWM_STEP;
	
}




//*************************************************************************
//******************  ADC *************************************************
//*************************************************************************


//const uint8_t osAnalogInputChannels[] PROGMEM = ANALOG_INPUT_CHANNELS;
uint8_t osAnalogInputChannels[] = ANALOG_INPUT_CHANNELS;
uint16_t osAnalogInputValues[ANALOG_INPUTS];
uint8_t osAnalogInputCounter[ANALOG_INPUTS];
uint16_t osAnalogInputBuildup[ANALOG_INPUTS];
uint8_t osAnalogInputPos=0; //= 0; // Current sampling position


void ADC_init() // !!!! must be use after sei() !!!!
{
#if ANALOG_INPUTS > 0	
	ADMUX |= (1<<REFS0); //napiêcie odniesienia jako AVCC
	
	//uint8_t channel = pgm_read_uint8_t(&osAnalogInputChannels[osAnalogInputPos]);
	uint8_t channel = tempController[osAnalogInputPos].sensorPin;
	#if defined(ADCSRB) && defined(MUX5)
		if(channel & 8)  // Reading channel 0-7 or 8-15?
			ADCSRB |= (1<<MUX5);
		else
			ADCSRB &= (1<<MUX5);
	#endif
	ADMUX = (ADMUX & ~(0x1F)) | (channel & 7);

	ADCSRA|= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADIE);  //Wlaczamy ADC i przerwanie ADC oraz ustawiamy prescaler na 128
	#if defined(DISABLE_DIGITAL_IO_FUNCTIONS_0_7)
		DIDR0 |= DISABLE_DIGITAL_IO_FUNCTIONS_0_7; //wylaczamy funkcje cyfrowe pinów wykorzystywanych do pomiarów adc
	#endif
	#if defined(DISABLE_DIGITAL_IO_FUNCTIONS_8_15)
		DIDR1 |= DISABLE_DIGITAL_IO_FUNCTIONS_8_15; //wylaczamy funkcje cyfrowe pinów wykorzystywanych do pomiarów adc
	#endif
	ADCSRA |= (1<<ADSC); //start pomiaru ADC
#endif
}

#if ANALOG_INPUTS > 0
ISR(ADC_vect)
{
	osAnalogInputBuildup[osAnalogInputPos] += ADCW;
	if(++osAnalogInputCounter[osAnalogInputPos] >= (1<<ANALOG_INPUT_SAMPLE))
	{
		#if ANALOG_INPUT_BITS + ANALOG_INPUT_SAMPLE < 12
			osAnalogInputValues[osAnalogInputPos] = osAnalogInputBuildup[osAnalogInputPos] << (12 - ANALOG_INPUT_BITS - ANALOG_INPUT_SAMPLE);
		#endif
		#if ANALOG_INPUT_BITS + ANALOG_INPUT_SAMPLE > 12
			osAnalogInputValues[osAnalogInputPos] = osAnalogInputBuildup[osAnalogInputPos] >> (ANALOG_INPUT_BITS + ANALOG_INPUT_SAMPLE - 12);
		#endif
		#if ANALOG_INPUT_BITS + ANALOG_INPUT_SAMPLE == 12
			osAnalogInputValues[osAnalogInputPos] = osAnalogInputBuildup[osAnalogInputPos];
		#endif
		osAnalogInputBuildup[osAnalogInputPos] = 0;
		osAnalogInputCounter[osAnalogInputPos] = 0;
		// Start next conversion
		if(++osAnalogInputPos >= ANALOG_INPUTS) {
			osAnalogInputPos = 0;
		}
		//uint8_t channel = pgm_read_uint8_t(&osAnalogInputChannels[osAnalogInputPos]);
		uint8_t channel = tempController[osAnalogInputPos].sensorPin;
		#if defined(ADCSRB) && defined(MUX5)
			if(channel & 8)  // Reading channel 0-7 or 8-15?
				ADCSRB |= (1<<MUX5);
			else
				ADCSRB &= ~(1<<MUX5);
		#endif
		uint8_t test = (ADMUX & ~(0x1F)) | (channel & 7);
		//ADMUX = (ADMUX & ~(0x1F)) | (channel & 7);
		ADMUX = test;
	}
	ADCSRA |= (1<<ADSC);  // start next conversion
}
#endif

