/*
 * hardware.h
 *
 * Created: 13.08.2016 18:21:04
 *  Author: Madman
 */ 


#ifndef HARDWARE_H_
#define HARDWARE_H_

#include <avr/io.h>
#include <avr/pgmspace.h>

#define PROGMEM
#define PGM_P const char *
typedef char prog_char;
//#undef PSTR
//#define PSTR(s) s
#undef pgm_read_byte_near
#define pgm_read_byte_near(x) (*(int8_t*)x)
#undef pgm_read_byte
#define pgm_read_byte(x) (*(int8_t*)x)
#undef pgm_read_float
#define pgm_read_float(addr) (*(const float *)(addr))
#undef pgm_read_word
//#define pgm_read_word(addr) (*(const unsigned int *)(addr))
#define pgm_read_word(addr) (*(addr))
#undef pgm_read_word_near
#define pgm_read_word_near(addr) pgm_read_word(addr)
#undef pgm_read_dword
#define pgm_read_dword(addr) (*(addr))
//#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#undef pgm_read_dword_near
#define pgm_read_dword_near(addr) pgm_read_dword(addr)
#define _BV(x) (1 << (x))

//*************************************************************************
//******************  I/O *************************************************
//*************************************************************************

void Pin_init();
void set_default_pins();

//*************************************************************************
//******************  SERIAL **********************************************
//*************************************************************************

#define BAUDRATE 250000
extern long baudrate;

// definuiujemy ktory port szeregowy bedzie uzywany do transmisji MODBUS - 0,1,2
#define  MODBUS_PORT_TRANSMISION 2

#if MODBUS_PORT_TRANSMISION == 0
	#define DEF_UCSRXB	UCSR0B
	#define DEF_UDRIEX	UDRIE0
	#define DEF_UBRRXH	UBRR0H
	#define DEF_UBRRXL	UBRR0L
	#define DEF_UCSRXC	UCSR0C
	#define DEF_RXENX	RXEN0
	#define DEF_TXENX	TXEN0
	#define DEF_RXCIEX	RXCIE0
	#define DEF_UCSZX0	UCSZ00
	#define DEF_UCSZX1	UCSZ01
	#define DEF_USARTX_UDRE_VECT	USART0_UDRE_vect
	#define DEF_USARTX_RX_VECT	USART0_RX_vect
	#define	DEF_UDRX	UDR0
#endif

#if MODBUS_PORT_TRANSMISION == 1
	#define DEF_UCSRXB	UCSR1B
	#define DEF_UDRIEX	UDRIE1
	#define DEF_UBRRXH	UBRR1H
	#define DEF_UBRRXL	UBRR1L
	#define DEF_UCSRXC	UCSR1C
	#define DEF_RXENX	RXEN1
	#define DEF_TXENX	TXEN1
	#define DEF_RXCIEX	RXCIE1
	#define DEF_UCSZX0	UCSZ10
	#define DEF_UCSZX1	UCSZ11
	#define DEF_USARTX_UDRE_VECT	USART1_UDRE_vect
	#define DEF_USARTX_RX_VECT		USART1_RX_vect
	#define	DEF_UDRX	UDR1
#endif

#if MODBUS_PORT_TRANSMISION == 2
	#define DEF_UCSRXB	UCSR2B
	#define DEF_UDRIEX	UDRIE2
	#define DEF_UBRRXH	UBRR2H
	#define DEF_UBRRXL	UBRR2L
	#define DEF_UCSRXC	UCSR2C
	#define DEF_RXENX	RXEN2
	#define DEF_TXENX	TXEN2
	#define DEF_RXCIEX	RXCIE2
	#define DEF_UCSZX0	UCSZ20
	#define DEF_UCSZX1	UCSZ21
	#define DEF_USARTX_UDRE_VECT	USART2_UDRE_vect
	#define DEF_USARTX_RX_VECT		USART2_RX_vect
	#define	DEF_UDRX	UDR2
#endif
//*************************************************************************
//******************  Timer *************************************************
//*************************************************************************

#define WART_PRESCALER_TIM1	(1<<CS11)|(1<<CS10) //p64
#define WART_PRESCALER_TIM0	(1<<CS01)|(1<<CS00) //p64
#define WART_OCR1	0xF9 // interwa³ co 1ms
#define WART_OCR0	0x3F // interwa³ co 256us

void Timer0_init();
void Timer1_init();

extern volatile uint32_t millis;
extern volatile uint16_t Door_move_timer;
extern volatile uint16_t Door_delay_timer;
extern volatile uint16_t modbus_transmission_blocade_timer;
extern volatile uint16_t Filament_measure_timer;

extern uint8_t door_status;
extern uint8_t door_setup;

#define DOOR_TIME	50000
#define DOOR_DELAY_OPEN	3000

#define DOOR_CLOSE_OPEN_MASK	0xC0	//maska na close i na open
#define DOOR_OPEN_MASK 0x80
#define DOOR_CLOSE_MASK 0x40


#define STEPPER_MOTOR_TIMER_VECTOR		TIMER0_COMPA_vect


//*************************************************************************
//******************  ADC *************************************************
//*************************************************************************

#define ANALOG_INPUTS 7 //how many analog inputs do we use

//!!!uwaga przy dodawaniu nowych trzeba te¿ zdefiniowaæ w³aœciwe rejestry w Modbus_registers.h

//definiujemy indeksy struktury dla poszczegolnych czujnikow (musi zaczynaæ siê od 0)
#define EXT0	0
#define EXT1	1
#define BED0	2
#define BED1	3
#define BED2	4
#define BED3	5
#define CHAMB0	6

//definiujemy dla indeksów powy¿ej wejscia analogowe przypisane do struktury 
#define EXT0_ANALOG_INPUT	0 //ADC0
#define EXT1_ANALOG_INPUT	1 //ADC1
#define BED0_ANALOG_INPUT	2 //ADC2
#define BED1_ANALOG_INPUT	3 //ADC3
#define BED2_ANALOG_INPUT	8 //ADC4
#define BED3_ANALOG_INPUT	9 //ADC5
#define CHAMBER0_ANALOG_INPUT	10 //ADC6

#define ANALOG_INPUT_CHANNELS {EXT0_ANALOG_INPUT, EXT1_ANALOG_INPUT, BED0_ANALOG_INPUT, BED1_ANALOG_INPUT, BED2_ANALOG_INPUT, BED3_ANALOG_INPUT, CHAMBER0_ANALOG_INPUT}

//extern const uint8_t osAnalogInputChannels[] PROGMEM;// = {0,1,2,3};  //which analog channel do we use
extern uint8_t osAnalogInputChannels[];// = {0,1,2,3};  //which analog channel do we use
extern uint16_t osAnalogInputValues[ANALOG_INPUTS];

#define DISABLE_DIGITAL_IO_FUNCTIONS_0_7 (1<<ADC0D)|(1<<ADC1D)|(1<<ADC2D)|(1<<ADC3D)|(1<<ADC4D)|(1<<ADC5D)|(1<<ADC6D)|(1<<ADC7D)
//#undef DISABLE_DIGITAL_IO_FUNCTIONS_0_7

#define DISABLE_DIGITAL_IO_FUNCTIONS_8_15 (1<<ADC8D)|(1<<ADC9D)|(1<<ADC10D)|(1<<ADC11D)|(1<<ADC12D)|(1<<ADC13D)|(1<<ADC14D)|(1<<ADC15D)
//#undef DISABLE_DIGITAL_IO_FUNCTIONS_8_15

#define ANALOG_INPUT_SAMPLE 5
// Bits of the ADC converter
#define ANALOG_INPUT_BITS 10
#define ANALOG_REDUCE_BITS 0
#define ANALOG_REDUCE_FACTOR 1




void ADC_init();


#endif /* HARDWARE_H_ */