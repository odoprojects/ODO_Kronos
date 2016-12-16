/*
 * mkuart.h
 *
 *  Created on: 2016-08-15
 *       Autor: Madman
 */

#ifndef MKUART_H_
#define MKUART_H_



#define UART0_BAUD 9600		// tu definiujemy prêdkoœæ dla UART0
//#define __UBRR0 F_CPU/16/UART0_BAUD-1  // obliczamy UBRR dla U2X=0
#define __UBRR0 ((F_CPU)/(UART0_BAUD*16UL)-1)
//#define UART1_BAUD 115200		// tu definiujemy prêdkoœæ dla UART1
//#define __UBRR1 F_CPU/16/UART1_BAUD-1  // obliczamy UBRR dla U2X=0

//#define UART2_BAUD 115200		// tu definiujemy prêdkoœæ dla UART2
//#define __UBRR2 F_CPU/16/UART2_BAUD-1  // obliczamy UBRR dla U2X=0


// definicje na potrzeby RS485
//#define UART_DE_PORT PORTD
//#define UART_DE_DIR DDRD
//#define UART_DE_BIT (1<<PD2)

//#define UART_DE_ODBIERANIE  UART_DE_PORT &= ~UART_DE_BIT
//#define UART_DE_NADAWANIE  UART_DE_PORT |= UART_DE_BIT


#define UART_RX_BUF_SIZE 64 // definiujemy bufor o rozmiarze 64 bajtów
// definiujemy maskê dla naszego bufora
#define UART_RX_BUF_MASK ( UART_RX_BUF_SIZE - 1)

#define UART_TX_BUF_SIZE 128 // definiujemy bufor o rozmiarze 512 bajtów
// definiujemy maskê dla naszego bufora
#define UART_TX_BUF_MASK ( UART_TX_BUF_SIZE - 1)


extern volatile uint8_t new_line;

typedef struct ramka{
//	uint8_t adres;
	uint8_t funkcja;
	uint8_t dane[16];
	//uint8_t *dane;
//	uint8_t CRC;
	uint8_t liczba_danych;
}TRamka;


extern volatile TRamka PrzetworzonaRamka;
extern uint8_t autotune;

// deklaracje funkcji publicznych

void USARTX_Init( uint32_t baud );
//void USART1_Init( uint16_t baud );
//void USART2_Init( uint16_t baud );

int uart_getc(void);
void uart_putc( char data );
void uart_puts(char *s);
void uart_putint(int value, int radix);
void uart_putFloat(float number, uint8_t digits);
void uart_printNumber(uint32_t n);

uint8_t ascii_to_int(uint8_t ascii);
uint8_t int_to_ascii(uint8_t value);

//void send_ModBus_frame(uint8_t *data_arr,  uint8_t data_lenght);
char * uart_get_str(char * buf);

void UART_RX_STR_EVENT(char * rbuf);
void register_uart_str_rx_event_callback(void (*callback)(char * pBuf));


#endif /* MKUART_H_ */
