/*
 * mkuart.c
 *
 *  Created on: 2016-08-15
 *       Autor: Madman
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
//#include <util/delay.h>

#include "mkuart.h"
#include "Pins.h"
#include "hardware.h"
#include "modbus.h"

volatile uint8_t new_line;
uint8_t block_modbus_transmission = 0;
volatile TRamka PrzetworzonaRamka;

// bufor UART_RxBuf
volatile char UART_RxBuf[UART_RX_BUF_SIZE];
// definiujemy indeksy okreœlaj¹ce iloœæ danych w buforze
volatile uint8_t UART_RxHead; // indeks oznaczaj¹cy „g³owê wê¿a”
volatile uint8_t UART_RxTail; // indeks oznaczaj¹cy „ogon wê¿a”

// bufor UART_TxBuf 
volatile char UART_TxBuf[UART_TX_BUF_SIZE];
// definiujemy indeksy okreœlaj¹ce iloœæ danych w buforze
volatile uint8_t UART_TxHead; // indeks oznaczaj¹cy „g³owê wê¿a”
volatile uint8_t UART_TxTail; // indeks oznaczaj¹cy „ogon wê¿a”

// wskaŸnik do funkcji callback dla zdarzenia UART_RX_STR_EVENT()
static void (*uart_rx_str_event_callback)(char *pBuf);


// funkcja do rejestracji funkcji zwrotnej w zdarzeniu UART_RX_STR_EVENT()
void register_uart_str_rx_event_callback(void (*callback)(char *pBuf)) {
	uart_rx_str_event_callback = callback;
}

// Zdarzenie do obliczenia indeksu w buforze cyklicznym z ramk¹ do wykonania
void UART_RX_STR_EVENT(char *rbuf) {
	if( new_line ) {
		if( uart_rx_str_event_callback ) {
			uart_get_str( rbuf );
			(*uart_rx_str_event_callback)( rbuf );
			} else {
			UART_RxHead = UART_RxTail;
		}
	}	
}

uint8_t ascii_to_int(uint8_t ascii){
	uint8_t value = 0;
	
	if ((ascii>=48) && (ascii<=57))
	{
		value = ascii - 48;	
	}else if ((ascii>=65) && (ascii<=70))
	{
		value = ascii - 55;
	}
	return value;
}

uint8_t int_to_ascii(uint8_t value){
	if ((value>=0) && (value<=9))
	{
		  value = value + 48;
	}else if ((value>=10) && (value<=15))
	{
		  value = value + 55;	
	}	
	return value;	
}


// ******************************************************************************************
// ******************** Funkcje inicjalizuj¹ce porty szeregowe ******************************
// ******************************************************************************************
//
void USARTX_Init( uint32_t baud ) {
	uint16_t _ubr = (F_CPU/16/baud-1);
	/* Ustawienie prêdkoœci */
	DEF_UBRRXH = (uint8_t)(_ubr>>8);
	DEF_UBRRXL = (uint8_t)_ubr;

	/* Za³¹czenie nadajnika I odbiornika */
	//UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	DEF_UCSRXB |= (1<<DEF_RXENX)|(1<<DEF_TXENX)|(1<<DEF_RXCIEX); 
	/* Ustawienie format ramki: 8bitów danych, 1 bit stopu */
	DEF_UCSRXC = (1<<DEF_UCSZX1)|(1<<DEF_UCSZX0);
	
}


// ******************************************************************************************
// ******************** Funkcje wysylajace do UART-a ****************************************
// ******************************************************************************************

// definiujemy funkcjê dodaj¹c¹ jeden bajt do bufora cyklicznego dla UART0

void uart_putc( char data ) { 
	uint8_t tmp_head;

    tmp_head  = (UART_TxHead + 1) & UART_TX_BUF_MASK;

    // pêtla oczekuje je¿eli brak miejsca w buforze cyklicznym na kolejne znaki
    while ( tmp_head == UART_TxTail ){}

    UART_TxBuf[tmp_head] = data;
    UART_TxHead = tmp_head;

    // inicjalizujemy przerwanie wystêpuj¹ce, gdy bufor jest pusty, dziêki
    // czemu w dalszej czêœci wysy³aniem danych zajmie siê ju¿ procedura
    // obs³ugi przerwania

    DEF_UCSRXB |= (1<<DEF_UDRIEX);
}

void uart_puts(char *s)		// wysy³a ³añcuch z pamiêci RAM na UART
{
  register char c;
  while ((c = *s++)) uart_putc(c);			// dopóki nie napotkasz 0 wysy³aj znak
}

void uart_putint(int value, int radix)	// wysy³a na port szeregowy tekst
{
	char string[17];			// bufor na wynik funkcji itoa
	itoa(value, string, radix);		// konwersja value na ASCII
	uart_puts(string);			// wyœlij string na port szeregowy
}

void uart_printNumber(uint32_t n) 
{
	char buf[11]; // Assumes 8-bit chars plus zero byte.
	char *str = &buf[10];
	*str = '\0';
	do {
		unsigned long m = n;
		n /= 10;
		*--str = '0'+(m - 10 * n);
	} while(n);

	uart_putc(str);
}

void uart_putFloat(float number, uint8_t digits)
{
	if (isnan(number)) {
		uart_puts(PSTR("NAN"));
		return;
	}
	if (isinf(number)) {
		uart_puts(PSTR("INF"));
		return;
	}
	// Handle negative numbers
	if (number < 0.0)
	{
		uart_putc('-');
		number = -number;
	}
	// Round correctly so that print(1.999, 2) prints as "2.00"
	float rounding = 0.5;
	for (uint8_t i=0; i<digits; ++i)
	rounding /= 10.0;

	number += rounding;

	// Extract the integer part of the number and print it
	unsigned long int_part = (unsigned long)number;
	float remainder = number - (float)int_part;
	uart_printNumber(int_part);

	// Print the decimal point, but only if there are digits beyond
	if (digits > 0)
	uart_putc('.');

	// Extract digits from the remainder one at a time
	while (digits-- > 0)
	{
		remainder *= 10.0;
		int toPrint = (int)remainder;
		uart_putc('0'+toPrint);
		remainder -= toPrint;
	}
}

uint8_t uart_puta(char *s){
	  register char c;
	  char first = 0;
	  char sec = 0;
	  while ((c = *s++)) {
		  
		  if ((c==FRAME_START) || (c==CR) || (c==LF))
		  {
				uart_putc(c);
		  } 
		  else
		  {
				first = c/16;
				sec = c - (16 * first);
				
				first = int_to_ascii(first);
				sec = int_to_ascii(sec);
				
				if ((first == -1)||(sec == -1))
				{
					return 0;
				}
				uart_putc(first);
				uart_putc(sec);
		  }

	  }
	  return 1;
}


// definiujemy procedurê obs³ugi przerwania nadawczego, pobieraj¹c¹ dane z bufora cyklicznego
// i wysylajacego na UART0

ISR( DEF_USARTX_UDRE_VECT)  {
    // sprawdzamy czy indeksy s¹ ró¿ne
    if ( UART_TxHead != UART_TxTail ) {
    	// obliczamy i zapamiêtujemy nowy indeks ogona wê¿a (mo¿e siê zrównaæ z g³ow¹)
    	UART_TxTail = (UART_TxTail + 1) & UART_TX_BUF_MASK;
    	// zwracamy bajt pobrany z bufora  jako rezultat funkcji
    	DEF_UDRX = UART_TxBuf[UART_TxTail];
    } else {
	// zerujemy flagê przerwania wystêpuj¹cego gdy bufor pusty
	DEF_UCSRXB &= ~(1<<DEF_UDRIEX);
    }
}


// ******************************************************************************************
// ************ Funkcje odbieraj¹ca z UART0 do bufora ***************************************
// ******************************************************************************************

// definiujemy funkcjê pobieraj¹c¹ jeden bajt z bufora cyklicznego
int uart_getc(void) {
	// sprawdzamy czy indeksy s¹ równe
	if ( UART_RxHead == UART_RxTail ) return -1;

	// obliczamy i zapamiêtujemy nowy indeks „ogona wê¿a” (mo¿e siê zrównaæ z g³ow¹)
	UART_RxTail = (UART_RxTail + 1) & UART_RX_BUF_MASK;
	// zwracamy bajt pobrany z bufora  jako rezultat funkcji
	return UART_RxBuf[UART_RxTail];
}

char * uart_get_str(char *buf) {
	char c;
	char * wsk = buf;
	uint8_t parse_data = 0;
	char data_pair = 0;
	uint8_t buff_index = 0;
	
	*buf++; //zwiekszamy adres wskaznika na bufor tak aby w pierwszej komorce na koniec wpisac ilosc danych odebranych
	
	if( new_line ) {
		while( (c = uart_getc()) ) {
			if( (13 == c) || (c < 0) ) break;
			if (parse_data)
			{
				if (data_pair)
				{
					buff_index++;
					*buf++ = (ascii_to_int(data_pair) * 16) + ascii_to_int(c);	//sklejenie pierwszego i drugiego znaku oraz na zakonczenie zwiekszenie wskaznika o 1 wskazujacego na bofor wyjsciowy	
					data_pair = 0;				
									
				}else{
					data_pair = c;			//pierwszy znak 
				}			
			}else if (c == FRAME_START) // gdy wykryjemy znak ":" - czyli poczatek ramki rozpoczynamy sklejanie dwoch znakow ascii w jedna wartosc  
			{
				parse_data = 1;
				data_pair = 0;
			}
		}
		//gdy skonczylismy obrobke danych to w pierwszej komorce bufora wyjsciowego wpisujemy ilosc danych odebranych 
		//gdy w buforze kolowym nie bylo zadnych danych to wtedy w pierwszej komorce bedzie wpisane 0
		*wsk = buff_index; 
		//*buf=0;
		new_line--;
	}

	return wsk;
}


// definiujemy procedurê obs³ugi przerwania odbiorczego z UART0

ISR( DEF_USARTX_RX_VECT ) {
	
    register uint8_t tmp_head;
    register char data;
	//static uint8_t buff;
	//static uint8_t bitstart;
	
    data = DEF_UDRX; //pobieramy natychmiast bajt danych z bufora sprzêtowego

    // obliczamy nowy indeks „g³owy wê¿a”
    tmp_head = ( UART_RxHead + 1) & UART_RX_BUF_MASK;

    // sprawdzamy, czy w¹¿ nie zacznie zjadaæ w³asnego ogona
    if ( tmp_head == UART_RxTail ) {
	    // tutaj mo¿emy w jakiœ wygodny dla nas sposób obs³u¿yæ  b³¹d spowodowany
	    // prób¹ nadpisania danych w buforze, mog³oby dojœæ do sytuacji gdzie
	    // nasz w¹¿ zacz¹³by zjadaæ w³asny ogon
	    UART_RxHead = UART_RxTail;
	} else {
   		switch( data ) {
    			case 0:					// ignorujemy bajt = 0
    			case 10: break;			// ignorujemy znak LF
    			case 13: 	// sygnalizujemy obecnoœæ kolejnej linii w buforze
						UART_RxHead = tmp_head;
						UART_RxBuf[tmp_head] = data;
						new_line++;
						block_modbus_transmission = 0;
						break;
				case 58: block_modbus_transmission = 1;	//zmienna blokuj¹ca transmisje
    			default : UART_RxHead = tmp_head; UART_RxBuf[tmp_head] = data;
    		}
    }	
}







