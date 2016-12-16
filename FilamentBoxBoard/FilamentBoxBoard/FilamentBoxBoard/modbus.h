/*
 * modbus.h
 *
 * Created: 21.08.2016 15:30:13
 *  Author: Madman
 */ 


#ifndef MODBUS_H_
#define MODBUS_H_

// Format ramki MODBUS
//   0 1 2 3
// S A F D D D C
//           L
// S - 1 Bajt strtowy - ":" 0x3A
// A - 1 Bajt Adres Slave-a
// D - Bajty danych
// L - Ostatni bajt danych to ew liczba rejestrów do odczytania (max 128 - 16 bitowych)
// C C - 2 Bajty check sumy LRC
// Poza bajtem satrtu wszystkie pozosta³e presy³ane s¹ dwoma znakami ASCII


// definicje dla transmisji modbus
#define MODBUS_FRAME_BASE_NUM	6
#define FRAME_START				0x3A

#define SLAVE_ADDRESS			0x64 //Adres naszego slave-a
#define ALL_SLAVES_ADR			0x00

#define CR						0x0D
#define LF						0x0A



////znane i niezmienne pozycje znaków w ramce MODBUS
//#define ADDRESS_POS		0
//#define CMD_POS			1
//#define DATA_POS		2

//znane i niezmienne pozycje znaków w ramce MODBUS
#define ADDRESS_POS		1
#define FUNCT_POS		2
#define DATA_POS		3

#define SEND_ADDRESS_POS		1
#define SEND_FUNCT_POS			2
#define SEND_DATA_LENGHT_POS	3
#define SEND_DATA_POS			4



#define READ_COILS					0x01
#define READ_DISCRETE_IMPUTS		0x02
#define READ_HOLDING_REGISTER		0x03
#define READ_INPUT_REGISTER			0x04
#define WRITE_SINGLE_COILS			0x05
#define WRITE_SINGLE_REGISTER		0x06
#define WIRTE_MULTIPLE_COILS		0x0F
#define WRITE_MULTIPLE_REGISTERS	0x10
#define DIAGNOSTIC					0x08

void parse_uart_data( char * pBuf );
void modbus_puts_s(char *s);
void modbus_puts_var_int(char *s, uint16_t data);
void modbus_puts_var_float(char *s, float data, uint8_t digits);

int8_t mod_read_coils(uint8_t * data); //modbus function 1 - Odczyt stanów wyjœæ binarnych. Np. wyjœæ PLC
int8_t mod_read_discrete_inputs(uint8_t * data); //modbus function 2 - Odczyt stanów wejœæ binarnych. Np. wejœæ PLC
int8_t mod_read_holding_register(uint8_t * data); //modbus function 3 - Odczyt rejestrów pamiêtaj¹cych
int8_t mod_read_input_register(char * data); //modbus function 4 - Odczyt rejestrów wejœciowych
int8_t mod_write_multiple_registers(char *data);
int8_t mod_read_register(char *data);

//int8_t mod_read_coils(uint8_t funct, uint8_t * data); //modbus function 1 - Odczyt stanów wyjœæ binarnych. Np. wyjœæ PLC
//int8_t mod_read_discrete_inputs(uint8_t funct, uint8_t * data); //modbus function 2 - Odczyt stanów wejœæ binarnych. Np. wejœæ PLC
//int8_t mod_read_holding_register(uint8_t funct, uint8_t * data); //modbus function 3 - Odczyt rejestrów pamiêtaj¹cych
//int8_t mod_read_input_register(uint8_t funct, uint8_t * data); //modbus function 4 - Odczyt rejestrów wejœciowych
extern volatile char bufor[];

#endif /* MODBUS_H_ */