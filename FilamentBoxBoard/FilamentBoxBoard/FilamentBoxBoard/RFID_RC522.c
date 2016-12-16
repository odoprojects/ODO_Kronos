
#include "RFID_RC522.h"
//#include <avr/io.h>
#include "spi.h"
#include "avr/delay.h"

// Firmware data for self-test
// Reference values based on firmware version
// Hint: if needed, you can remove unused self-test data to save flash memory
//
// Version 0.0 (0x90)
// Philips Semiconductors; Preliminary Specification Revision 2.0 - 01 August 2005; 16.1 self-test
const uint8_t MFRC522_firmware_referenceV0_0[] PROGMEM = {
	0x00, 0x87, 0x98, 0x0f, 0x49, 0xFF, 0x07, 0x19,
	0xBF, 0x22, 0x30, 0x49, 0x59, 0x63, 0xAD, 0xCA,
	0x7F, 0xE3, 0x4E, 0x03, 0x5C, 0x4E, 0x49, 0x50,
	0x47, 0x9A, 0x37, 0x61, 0xE7, 0xE2, 0xC6, 0x2E,
	0x75, 0x5A, 0xED, 0x04, 0x3D, 0x02, 0x4B, 0x78,
	0x32, 0xFF, 0x58, 0x3B, 0x7C, 0xE9, 0x00, 0x94,
	0xB4, 0x4A, 0x59, 0x5B, 0xFD, 0xC9, 0x29, 0xDF,
	0x35, 0x96, 0x98, 0x9E, 0x4F, 0x30, 0x32, 0x8D
};
// Version 1.0 (0x91)
// NXP Semiconductors; Rev. 3.8 - 17 September 2014; 16.1.1 self-test
const uint8_t MFRC522_firmware_referenceV1_0[] PROGMEM = {
	0x00, 0xC6, 0x37, 0xD5, 0x32, 0xB7, 0x57, 0x5C,
	0xC2, 0xD8, 0x7C, 0x4D, 0xD9, 0x70, 0xC7, 0x73,
	0x10, 0xE6, 0xD2, 0xAA, 0x5E, 0xA1, 0x3E, 0x5A,
	0x14, 0xAF, 0x30, 0x61, 0xC9, 0x70, 0xDB, 0x2E,
	0x64, 0x22, 0x72, 0xB5, 0xBD, 0x65, 0xF4, 0xEC,
	0x22, 0xBC, 0xD3, 0x72, 0x35, 0xCD, 0xAA, 0x41,
	0x1F, 0xA7, 0xF3, 0x53, 0x14, 0xDE, 0x7E, 0x02,
	0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D, 0x29, 0x79
};
// Version 2.0 (0x92)
// NXP Semiconductors; Rev. 3.8 - 17 September 2014; 16.1.1 self-test
const uint8_t MFRC522_firmware_referenceV2_0[] PROGMEM = {
	0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95,
	0xD0, 0xE3, 0x0D, 0x3D, 0x27, 0x89, 0x5C, 0xDE,
	0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B, 0x89, 0x82,
	0x51, 0x3A, 0xEB, 0x02, 0x0C, 0xA5, 0x00, 0x49,
	0x7C, 0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81,
	0x5D, 0x48, 0x76, 0xD5, 0x71, 0x61, 0x21, 0xA9,
	0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B, 0x6D,
	0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F
};
// Clone
// Fudan Semiconductor FM17522 (0x88)
const uint8_t FM17522_firmware_reference[] PROGMEM = {
	0x00, 0xD6, 0x78, 0x8C, 0xE2, 0xAA, 0x0C, 0x18,
	0x2A, 0xB8, 0x7A, 0x7F, 0xD3, 0x6A, 0xCF, 0x0B,
	0xB1, 0x37, 0x63, 0x4B, 0x69, 0xAE, 0x91, 0xC7,
	0xC3, 0x97, 0xAE, 0x77, 0xF4, 0x37, 0xD7, 0x9B,
	0x7C, 0xF5, 0x3C, 0x11, 0x8F, 0x15, 0xC3, 0xD7,
	0xC1, 0x5B, 0x00, 0x2A, 0xD0, 0x75, 0xDE, 0x9E,
	0x51, 0x64, 0xAB, 0x3E, 0xE9, 0x15, 0xB5, 0xAB,
	0x56, 0x9A, 0x98, 0x82, 0x26, 0xEA, 0x2A, 0x62
};
#define NULL ( (void *) 0)


uint8_t ID;
 //Member variables
Uid uid;								// Used by PICC_ReadCardSerial().
MIFARE_Key key;
 //Size of the MFRC522 FIFO
static const uint8_t FIFO_SIZE = 64;		// The FIFO is 64 uint8_ts.
uint8_t _chipSelectPin;		// Arduino pin connected to MFRC522's SPI slave select input (Pin 24, NSS, active low)
uint8_t _resetPowerDownPin;	// Arduino pin connected to MFRC522's reset and power down input (Pin 6, NRSTPD, active low)

// A struct used for passing the UID of a PICC.


 //A struct used for passing a MIFARE Crypto1 key


void handleMFRC522(){
	if (PICC_IsNewCardPresent()){
		if (PICC_ReadCardSerial()){
			enum PICC_Type piccType = PICC_GetType(uid.sak);
		}
	}
	uint8_t sector         = 1;
	uint8_t blockAddr      = 4;
	uint8_t dataBlock[]    = {
		0x01, 0x02, 0x03, 0x04, //  1,  2,   3,  4,
		0x05, 0x06, 0x07, 0x08, //  5,  6,   7,  8,
		0x08, 0x09, 0xff, 0x0b, //  9, 10, 255, 12,
		0x0c, 0x0d, 0x0e, 0x0f  // 13, 14,  15, 16
	};
	uint8_t trailerBlock   = 7;
	enum StatusCode status;
	uint8_t buffer[18];
	uint8_t size = sizeof(buffer);
	status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, trailerBlock, &key, &(uid));
	if (status != STATUS_OK) {
		
		return;
	}
	PICC_DumpMifareClassicSectorToSerial(&(uid), &key, sector);
	status = MIFARE_Read(blockAddr, buffer, &size);
	if (status == STATUS_OK) {
		status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_B, trailerBlock, &key, &(uid));
		if (status != STATUS_OK) {
			
			return;
		}
		status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_B, trailerBlock, &key, &(uid));
		if (status != STATUS_OK) {

			return;
		}
		status = MIFARE_Read(blockAddr, buffer, &size);
		if (status != STATUS_OK) {
			
		}
	}
}

 void initMFRC522() {
 	spi_init();
	PCD_Init();
	for (uint8_t i = 0; i < 6; i++) {
		key.keyuint8_t[i] = 0xFF;
	}
 } 
 void PCD_WriteRegister(	uint8_t reg,		///< The register to write to. One of the PCD_Register enums.
 									uint8_t value		///< The value to write.
 								) {
 
 	ENABLE_CHIP();
 	spi_transmit(reg&0x7E);
 	spi_transmit(value);
 	DISABLE_CHIP();
 	
 	
 } 
 void PCD_WriteRegisters(uint8_t reg,		///< The register to write to. One of the PCD_Register enums.
 									uint8_t count,		///< The number of uint8_ts to write to the register
 									uint8_t *values	///< The values to write. uint8_t array.
 								) {
 	
 	ENABLE_CHIP();
 	spi_transmit(reg&0x7E);
 	for (uint8_t index = 0; index < count; index++) {
 		spi_transmit(values[index]);
 	}
 	DISABLE_CHIP();
 	
 } 
 uint8_t PCD_ReadRegister(uint8_t reg	///< The register to read from. One of the PCD_Register enums.
 								) {
 	uint8_t value;
 	
 	
 	ENABLE_CHIP();
 	spi_transmit(0x80 | (reg & 0x7E));
 	value = spi_transmit(0x00);
 	DISABLE_CHIP();
 	
 	return value;
 } // End PCD_ReadRegister()
 
 
 void PCD_ReadRegisters(	uint8_t reg,		///< The register to read from. One of the PCD_Register enums.
 								uint8_t count,		///< The number of uint8_ts to read
 								uint8_t *values,	///< uint8_t array to store the values in.
 								uint8_t rxAlign	///< Only bit positions rxAlign..7 in values[0] are updated.
 								) {
 	if (count == 0) {
 		return;
 	}
 	uint8_t address = 0x80 | (reg & 0x7E);		// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
 	uint8_t index = 0;
 	ENABLE_CHIP();
 	count--;
 	spi_transmit(address);
 	while (index < count) {
 		if (index == 0 && rxAlign) {		// Only update bit positions rxAlign..7 in values[0]
 			// Create bit mask for bit positions rxAlign..7
 			uint8_t mask = 0;
 			for (uint8_t i = rxAlign; i <= 7; i++) {
 				mask |= (1 << i);
 			}
 			// Read value and tell that we want to read the same address again.
 			uint8_t value = spi_transmit(address);
 			// Apply mask to both current value of values[0] and the new data in value.
 			values[0] = (values[index] & ~mask) | (value & mask);
 		}
 		else { // Normal case
 			values[index] = spi_transmit(address);	// Read value and tell that we want to read the same address again.
 		}
 		index++;
 	}
 		
 	values[index] = spi_transmit(0);
 	DISABLE_CHIP();	
 } 
 
 
 void PCD_SetRegisterBitMask(	uint8_t reg,	///< The register to update. One of the PCD_Register enums.
 										uint8_t mask	///< The bits to set.
 									) { 
 	uint8_t tmp;
 	tmp = PCD_ReadRegister(reg);
 	PCD_WriteRegister(reg, tmp | mask);			// set bit mask
 } // End PCD_SetRegisterBitMask()
 
 
 void PCD_ClearRegisterBitMask(	uint8_t reg,	///< The register to update. One of the PCD_Register enums.
 										uint8_t mask	///< The bits to clear.
 									  ) {
 	uint8_t tmp;
 	tmp = PCD_ReadRegister(reg);
 	PCD_WriteRegister(reg, tmp & (~mask));		// clear bit mask
 } // End PCD_ClearRegisterBitMask()
 
 
 
 enum StatusCode PCD_CalculateCRC(uint8_t *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
 												uint8_t length,	///< In: The number of uint8_ts to transfer.
 												uint8_t *result	///< Out: Pointer to result buffer. Result is written to result[0..1], low uint8_t first.
 					 ) {
 	PCD_WriteRegister(CommandReg, PCD_Idle);		// Stop any active command.
 	PCD_WriteRegister(DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
 	PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);		// FlushBuffer = 1, FIFO initialization
 	PCD_WriteRegisters(FIFODataReg, length, data);	// Write data to the FIFO
 	PCD_WriteRegister(CommandReg, PCD_CalcCRC);		// Start the calculation
 	
 	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73?s.
 	// TODO check/modify for other architectures than Arduino Uno 16bit
 	uint16_t i = 5000;
 	uint8_t n;
 	while (1) {
 		n = PCD_ReadRegister(DivIrqReg);	// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
 		if (n & 0x04) {						// CRCIRq bit set - calculation done
 			break;
 		}
 		if (--i == 0) {						// The emergency break. We will eventually terminate on this one after 89ms. Communication with the MFRC522 might be down.
 			return STATUS_TIMEOUT;
 		}
 	}
 	PCD_WriteRegister(CommandReg, PCD_Idle);		// Stop calculating CRC for new content in the FIFO.
 	
 	// Transfer the result from the registers to the result buffer
 	result[0] = PCD_ReadRegister(CRCResultRegL);
 	result[1] = PCD_ReadRegister(CRCResultRegH);
 	return STATUS_OK;
 } // End PCD_CalculateCRC()
// 
// 
// /////////////////////////////////////////////////////////////////////////////////////
// // Functions for manipulating the MFRC522
// /////////////////////////////////////////////////////////////////////////////////////
// 
// 
 void PCD_Init() {
 	DISABLE_CHIP();
 	SPI_CS_0_DDR |=(1<<SPI_0_RESET);
 	SPI_CS_0_PORT |=(1<<SPI_0_RESET); 
	PCD_Reset(); 
 	//SPI_CS_0_PORT &= (~(1<<SPI_0_RESET));
 	// When communicating with a PICC we need a timeout if something goes wrong.
 	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
 	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
 	PCD_WriteRegister(TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
 	PCD_WriteRegister(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25?s.
 	PCD_WriteRegister(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
 	PCD_WriteRegister(TReloadRegL, 0xE8);
 	
 	PCD_WriteRegister(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
 	PCD_WriteRegister(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
 	PCD_AntennaOn();						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
 } // End PCD_Init()
// 
// 
  //void PCD_Init(	uint8_t resetPowerDownPin	///< Arduino pin connected to MFRC522's reset and power down input (Pin 6, NRSTPD, active low)
  					//) {
  	//PCD_Init( SS, resetPowerDownPin); // SS is defined in pins_arduino.h
  //} // End PCD_Init()
// 
// 
// // void PCD_Init(	uint8_t chipSelectPin,		///< Arduino pin connected to MFRC522's SPI slave select input (Pin 24, NSS, active low)
// // 						uint8_t resetPowerDownPin	///< Arduino pin connected to MFRC522's reset and power down input (Pin 6, NRSTPD, active low)
// // 					) {
// // 	_chipSelectPin = chipSelectPin;
// // 	_resetPowerDownPin = resetPowerDownPin; 
// // 	// Set the chipSelectPin as digital output, do not select the slave yet
// // 	PCD_Init();
// // } // End PCD_Init()
// 
 
 void PCD_Reset() {
 	PCD_WriteRegister(CommandReg, PCD_SoftReset);	// Issue the SoftReset command.
 	// The datasheet does not mention how long the SoftRest command takes to complete.
 	// But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg) 
 	// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74?s. Let us be generous: 50ms.
 	_delay_ms(50);
 	// Wait for the PowerDown bit in CommandReg to be cleared
 	while (PCD_ReadRegister(CommandReg) & (1<<4)) {
 		// PCD still restarting - unlikely after waiting 50ms, but better safe than sorry.
 	}
 } // End PCD_Reset()
 
 
 void PCD_AntennaOn() {
 	uint8_t value = PCD_ReadRegister(TxControlReg);
 	if ((value & 0x03) != 0x03) {
 		PCD_WriteRegister(TxControlReg, value | 0x03);
 	}
 } // End PCD_AntennaOn()
// 
// 
 void PCD_AntennaOff() {
 	PCD_ClearRegisterBitMask(TxControlReg, 0x03);
 } // End PCD_AntennaOff()
 
 
 uint8_t PCD_GetAntennaGain() {
 	return PCD_ReadRegister(RFCfgReg) & (0x07<<4);
 } 
 void PCD_SetAntennaGain(uint8_t mask) {
 	if (PCD_GetAntennaGain() != mask) {						// only bother if there is a change
 		PCD_ClearRegisterBitMask(RFCfgReg, (0x07<<4));		// clear needed to allow 000 pattern
 		PCD_SetRegisterBitMask(RFCfgReg, mask & (0x07<<4));	// only set RxGain[2:0] bits
 	}
 }
 uint8_t PCD_PerformSelfTest() {
 	// This follows directly the steps outlined in 16.1.1
 	// 1. Perform a soft reset.
 	PCD_Reset();
 	
 	// 2. Clear the internal buffer by writing 25 uint8_ts of 00h
 	uint8_t ZEROES[25] = {0x00};
 	PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);	// flush the FIFO buffer
 	PCD_WriteRegisters(FIFODataReg, 25, ZEROES);	// write 25 uint8_ts of 00h to FIFO
 	PCD_WriteRegister(CommandReg, PCD_Mem);		// transfer to internal buffer
 	
 	// 3. Enable self-test
 	PCD_WriteRegister(AutoTestReg, 0x09);
 	
 	// 4. Write 00h to FIFO buffer
 	PCD_WriteRegister(FIFODataReg, 0x00);
 	
 	// 5. Start self-test by issuing the CalcCRC command
 	PCD_WriteRegister(CommandReg, PCD_CalcCRC);
 	
 	// 6. Wait for self-test to complete
 	uint8_t n;
 	for (uint8_t i = 0; i < 0xFF; i++) {
 		// The datasheet does not specify exact completion condition except
 		// that FIFO buffer should contain 64 uint8_ts.
 		// While selftest is initiated by CalcCRC command
 		// it behaves differently from normal CRC computation,
 		// so one can't reliably use DivIrqReg to check for completion.
 		// It is reported that some devices does not trigger CRCIRq flag
 		// during selftest.
 		n = PCD_ReadRegister(FIFOLevelReg);
 		if (n >= 64) {
 			break;
 		}
 	}
 	PCD_WriteRegister(CommandReg, PCD_Idle);		// Stop calculating CRC for new content in the FIFO.
 	
 	// 7. Read out resulting 64 uint8_ts from the FIFO buffer.
 	uint8_t result[64];
 	PCD_ReadRegisters(FIFODataReg, 64, result, 0);
 	
 	// Auto self-test done
 	// Reset AutoTestReg register to be 0 again. Required for normal operation.
 	PCD_WriteRegister(AutoTestReg, 0x00);
 	
 	// Determine firmware version (see section 9.3.4.8 in spec)
 	uint8_t version = PCD_ReadRegister(VersionReg);
 	
 	// Pick the appropriate reference values
 	/*const uint8_t *reference;
 	switch (version) {
 		case 0x88:	// Fudan Semiconductor FM17522 clone
 			reference = FM17522_firmware_reference;
 			break;
 		case 0x90:	// Version 0.0
 			reference = MFRC522_firmware_referenceV0_0;
 			break;
 		case 0x91:	// Version 1.0
 			reference = MFRC522_firmware_referenceV1_0;
 			break;
 		case 0x92:	// Version 2.0
 			reference = MFRC522_firmware_referenceV2_0;
 			break;
 		default:	// Unknown version
 			return 0; // abort test
 	}
 	
 	// Verify that the results match up to our expectations
 	for (uint8_t i = 0; i < 64; i++) {
 		if (result[i] != pgm_read_uint8_t(&(reference[i]))) {
 			return 0;
 		}
 	}*/
 	
 	// Test passed; all is good.
 	return 1;
 } // End PCD_PerformSelfTest()
 
 /////////////////////////////////////////////////////////////////////////////////////
 // Functions for communicating with PICCs
 /////////////////////////////////////////////////////////////////////////////////////
 
 
 enum StatusCode PCD_TransceiveData(	uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
 													uint8_t sendLen,		///< Number of uint8_ts to transfer to the FIFO.
 													uint8_t *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
 													uint8_t *backLen,		///< In: Max number of uint8_ts to write to *backData. Out: The number of uint8_ts returned.
 													uint8_t *validBits,	///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits. Default NULL.
 													uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
 													uint8_t checkCRC		///< In: 1 => The last two uint8_ts of the response is assumed to be a CRC_A that must be validated.
 								 ) {
 	uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
 	return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
 } // End PCD_TransceiveData()
 
 
 enum StatusCode PCD_CommunicateWithPICC(	uint8_t command,		///< The command to execute. One of the PCD_Command enums.
 														uint8_t waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
 														uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
 														uint8_t sendLen,		///< Number of uint8_ts to transfer to the FIFO.
 														uint8_t *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
 														uint8_t *backLen,		///< In: Max number of uint8_ts to write to *backData. Out: The number of uint8_ts returned.
 														uint8_t *validBits,	///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits.
 														uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
 														uint8_t checkCRC		///< In: 1 => The last two uint8_ts of the response is assumed to be a CRC_A that must be validated.
 									 ) {
 	uint8_t n, _validBits;
 	uint16_t i;
 	
 	// Prepare values for BitFramingReg
 	uint8_t txLastBits = validBits ? *validBits : 0;
 	uint8_t bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
 	
 	PCD_WriteRegister(CommandReg, PCD_Idle);			// Stop any active command.
 	PCD_WriteRegister(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
 	PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
 	PCD_WriteRegisters(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
 	PCD_WriteRegister(BitFramingReg, bitFraming);		// Bit adjustments
 	PCD_WriteRegister(CommandReg, command);				// Execute the command
 	if (command == PCD_Transceive) {
 		PCD_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
 	}
 	
 	// Wait for the command to complete.
 	// In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
 	// Each iteration of the do-while-loop takes 17.86?s.
 	// TODO check/modify for other architectures than Arduino Uno 16bit
 	i = 2000;
 	while (1) {
 		n = PCD_ReadRegister(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
 		if (n & waitIRq) {					// One of the interrupts that signal success has been set.
 			break;
 		}
 		if (n & 0x01) {						// Timer interrupt - nothing received in 25ms
 			return STATUS_TIMEOUT;
 		}
 		if (--i == 0) {						// The emergency break. If all other conditions fail we will eventually terminate on this one after 35.7ms. Communication with the MFRC522 might be down.
 			return STATUS_TIMEOUT;
 		}
 	}
 	
 	// Stop now if any errors except collisions were detected.
 	uint8_t errorRegValue = PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
 	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
 		return STATUS_ERROR;
 	}	
 
 	// If the caller wants data back, get it from the MFRC522.
 	if (backData && backLen) {
 		n = PCD_ReadRegister(FIFOLevelReg);			// Number of uint8_ts in the FIFO
 		if (n > *backLen) {
 			return STATUS_NO_ROOM;
 		}
 		*backLen = n;											// Number of uint8_ts returned
 		PCD_ReadRegisters(FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
 		_validBits = PCD_ReadRegister(ControlReg) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received uint8_t. If this value is 000b, the whole uint8_t is valid.
 		if (validBits) {
 			*validBits = _validBits;
 		}
 	}
 	
 	// Tell about collisions
 	if (errorRegValue & 0x08) {		// CollErr
 		return STATUS_COLLISION;
 	}
 	
 	// Perform CRC_A validation if requested.
 	if (backData && backLen && checkCRC) {
 		// In this case a MIFARE Classic NAK is not OK.
 		if (*backLen == 1 && _validBits == 4) {
 			return STATUS_MIFARE_NACK;
 		}
 		// We need at least the CRC_A value and all 8 bits of the last uint8_t must be received.
 		if (*backLen < 2 || _validBits != 0) {
 			return STATUS_CRC_WRONG;
 		}
 		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
 		uint8_t controlBuffer[2];
 		enum StatusCode status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
 		if (status != STATUS_OK) {
 			return status;
 		}
 		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
 			return STATUS_CRC_WRONG;
 		}
 	}
 	
 	return STATUS_OK;
 } // End PCD_CommunicateWithPICC()
 
 
 enum StatusCode PICC_RequestA(	uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
 											uint8_t *bufferSize	///< Buffer size, at least two uint8_ts. Also number of uint8_ts returned if STATUS_OK.
 										) {
 	return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
 } // End PICC_RequestA()
 
 enum StatusCode PICC_WakeupA(	uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
 											uint8_t *bufferSize	///< Buffer size, at least two uint8_ts. Also number of uint8_ts returned if STATUS_OK.
 										) {
 	return PICC_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
 } // End PICC_WakeupA()
 
 
 enum StatusCode PICC_REQA_or_WUPA(	uint8_t command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
 												uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
 												uint8_t *bufferSize	///< Buffer size, at least two uint8_ts. Also number of uint8_ts returned if STATUS_OK.
 											) {
 	uint8_t validBits;
 	enum StatusCode status;
 	
 	if (bufferATQA == NULL || *bufferSize < 2) {	// The ATQA response is 2 uint8_ts long.
 		return STATUS_NO_ROOM;
 	}
 	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
 	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) uint8_t. TxLastBits = BitFramingReg[2..0]
 	status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits,0,0);
 	if (status != STATUS_OK) {
 		return status;
 	}
 	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
 		return STATUS_ERROR;
 	}
 	return STATUS_OK;
 } // End PICC_REQA_or_WUPA()
 
 
 enum StatusCode PICC_Select(	Uid *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
 											uint8_t validBits		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
 										 ) {
 	uint8_t uidComplete;
 	uint8_t selectDone;
 	uint8_t useCascadeTag;
 	uint8_t cascadeLevel = 1;
 	enum StatusCode result;
 	uint8_t count;
 	uint8_t index;
 	uint8_t uidIndex;					// The first index in uid->uiduint8_t[] that is used in the current Cascade Level.
 	int8_t currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
 	uint8_t buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 uint8_t standard frame + 2 uint8_ts CRC_A
 	uint8_t bufferUsed;				// The number of uint8_ts used in the buffer, ie the number of uint8_ts to transfer to the FIFO.
 	uint8_t rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
 	uint8_t txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted uint8_t. 
 	uint8_t *responseBuffer;
 	uint8_t responseLength;
 	
 	// Description of buffer structure:
 	//		uint8_t 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
 	//		uint8_t 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete uint8_ts, Low nibble: Extra bits. 
 	//		uint8_t 2: UID-data or CT		See explanation below. CT means Cascade Tag.
 	//		uint8_t 3: UID-data
 	//		uint8_t 4: UID-data
 	//		uint8_t 5: UID-data
 	//		uint8_t 6: BCC					Block Check Character - XOR of uint8_ts 2-5
 	//		uint8_t 7: CRC_A
 	//		uint8_t 8: CRC_A
 	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
 	//
 	// Description of uint8_ts 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
 	//		UID size	Cascade level	uint8_t2	uint8_t3	uint8_t4	uint8_t5
 	//		========	=============	=====	=====	=====	=====
 	//		 4 uint8_ts		1			uid0	uid1	uid2	uid3
 	//		 7 uint8_ts		1			CT		uid0	uid1	uid2
 	//						2			uid3	uid4	uid5	uid6
 	//		10 uint8_ts		1			CT		uid0	uid1	uid2
 	//						2			CT		uid3	uid4	uid5
 	//						3			uid6	uid7	uid8	uid9
 	
 	// Sanity checks
 	if (validBits > 80) {
 		return STATUS_INVALID;
 	}
 	
 	// Prepare MFRC522
 	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
 	
 	// Repeat Cascade Level loop until we have a complete UID.
 	uidComplete = 0;
 	while (!uidComplete) {
 		// Set the Cascade Level in the SEL uint8_t, find out if we need to use the Cascade Tag in uint8_t 2.
 		switch (cascadeLevel) {
 			case 1:
 				buffer[0] = PICC_CMD_SEL_CL1;
 				uidIndex = 0;
 				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 uint8_ts
 				break;
 			
 			case 2:
 				buffer[0] = PICC_CMD_SEL_CL2;
 				uidIndex = 3;
 				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 uint8_ts
 				break;
 			
 			case 3:
 				buffer[0] = PICC_CMD_SEL_CL3;
 				uidIndex = 6;
 				useCascadeTag = 0;						// Never used in CL3.
 				break;
 			
 			default:
 				return STATUS_INTERNAL_ERROR;
 				break;
 		}
 		
 		// How many UID bits are known in this Cascade Level?
 		currentLevelKnownBits = validBits - (8 * uidIndex);
 		if (currentLevelKnownBits < 0) {
 			currentLevelKnownBits = 0;
 		}
 		// Copy the known bits from uid->uiduint8_t[] to buffer[]
 		index = 2; // destination index in buffer[]
 		if (useCascadeTag) {
 			buffer[index++] = PICC_CMD_CT;
 		}
 		uint8_t uint8_tsToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of uint8_ts needed to represent the known bits for this level.
 		if (uint8_tsToCopy) {
 			uint8_t maxuint8_ts = useCascadeTag ? 3 : 4; // Max 4 uint8_ts in each Cascade Level. Only 3 left if we use the Cascade Tag
 			if (uint8_tsToCopy > maxuint8_ts) {
 				uint8_tsToCopy = maxuint8_ts;
 			}
 			for (count = 0; count < uint8_tsToCopy; count++) {
 				buffer[index++] = uid->uiduint8_t[uidIndex + count];
 			}
 		}
 		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
 		if (useCascadeTag) {
 			currentLevelKnownBits += 8;
 		}
 		
 		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
 		selectDone = 0;
 		while (!selectDone) {
 			// Find out how many bits and uint8_ts to send and receive.
 			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
 				////Serial.print(F("SELECT: currentLevelKnownBits=")); //Serial.println(currentLevelKnownBits, DEC);
 				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole uint8_ts
 				// Calculate BCC - Block Check Character
 				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
 				// Calculate CRC_A
 				result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
 				if (result != STATUS_OK) {
 					return result;
 				}
 				txLastBits		= 0; // 0 => All 8 bits are valid.
 				bufferUsed		= 9;
 				// Store response in the last 3 uint8_ts of buffer (BCC and CRC_A - not needed after tx)
 				responseBuffer	= &buffer[6];
 				responseLength	= 3;
 			}
 			else { // This is an ANTICOLLISION.
 				////Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); //Serial.println(currentLevelKnownBits, DEC);
 				txLastBits		= currentLevelKnownBits % 8;
 				count			= currentLevelKnownBits / 8;	// Number of whole uint8_ts in the UID part.
 				index			= 2 + count;					// Number of whole uint8_ts: SEL + NVB + UIDs
 				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
 				bufferUsed		= index + (txLastBits ? 1 : 0);
 				// Store response in the unused part of buffer
 				responseBuffer	= &buffer[index];
 				responseLength	= sizeof(buffer) - index;
 			}
 			
 			// Set bit adjustments
 			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
 			PCD_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
 			
 			// Transmit the buffer and receive the response.
 			result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign,0);
 			
 			if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
 				uint8_t valueOfCollReg = PCD_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
 				if (valueOfCollReg & 0x20) { // CollPosNotValid
 					return STATUS_COLLISION; // Without a valid collision position we cannot continue
 				}
 				uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
 				if (collisionPos == 0) {
 					collisionPos = 32;
 				}
 				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen 
 					return STATUS_INTERNAL_ERROR;
 				}
 				// Choose the PICC with the bit set.
 				currentLevelKnownBits = collisionPos;
 				count			= (currentLevelKnownBits - 1) % 8; // The bit to modify
 				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First uint8_t is index 0.
 				buffer[index]	|= (1 << count);
 			}
 			else if (result != STATUS_OK) {
 				return result;
 			}
 			else { // STATUS_OK
 				if (currentLevelKnownBits >= 32) { // This was a SELECT.
 					selectDone = 1; // No more anticollision 
 					// We continue below outside the while.
 				}
 				else { // This was an ANTICOLLISION.
 					// We now have all 32 bits of the UID in this Cascade Level
 					currentLevelKnownBits = 32;
 					// Run loop again to do the SELECT.
 				}
 			}
 		} // End of while (!selectDone)
 		
 		// We do not check the CBB - it was constructed by us above.
 		
 		// Copy the found UID uint8_ts from buffer[] to uid->uiduint8_t[]
 		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
 		uint8_tsToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
 		for (count = 0; count < uint8_tsToCopy; count++) {
 			uid->uiduint8_t[uidIndex + count] = buffer[index++];
 		}
 		
 		// Check response SAK (Select Acknowledge)
 		if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 uint8_t + CRC_A).
 			return STATUS_ERROR;
 		}
 		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those uint8_ts are not needed anymore.
 		result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
 		if (result != STATUS_OK) {
 			return result;
 		}
 		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
 			return STATUS_CRC_WRONG;
 		}
 		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
 			cascadeLevel++;
 		}
 		else {
 			uidComplete = 1;
 			uid->sak = responseBuffer[0];
 		}
 	} // End of while (!uidComplete)
 	
 	// Set correct uid->size
 	uid->size = 3 * cascadeLevel + 1;
 	
 	return STATUS_OK;
 } // End PICC_Select()
 
 
 enum StatusCode PICC_HaltA() {
 	enum StatusCode result;
 	uint8_t buffer[4];
 	
 	// Build command buffer
 	buffer[0] = PICC_CMD_HLTA;
 	buffer[1] = 0;
 	// Calculate CRC_A
 	result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
 	if (result != STATUS_OK) {
 		return result;
 	}
 	
 	// Send the command.
 	// The standard says:
 	//		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
 	//		HLTA command, this response shall be interpreted as 'not acknowledge'.
 	// We interpret that this way: Only STATUS_TIMEOUT is a success.
 	result = PCD_TransceiveData(buffer, sizeof(buffer), NULL, 0,NULL,0,0);
 	if (result == STATUS_TIMEOUT) {
 		return STATUS_OK;
 	}
 	if (result == STATUS_OK) { // That is ironically NOT ok in this case ;-)
 		return STATUS_ERROR;
 	}
 	return result;
 } // End PICC_HaltA()
 
 
 /////////////////////////////////////////////////////////////////////////////////////
 // Functions for communicating with MIFARE PICCs
 /////////////////////////////////////////////////////////////////////////////////////
 
 
 enum StatusCode PCD_Authenticate(uint8_t command,		///< PICC_CMD_MF_AUTH_KEY_A or PICC_CMD_MF_AUTH_KEY_B
 											uint8_t blockAddr, 	///< The block number. See numbering in the comments in the .h file.
 											MIFARE_Key *key,	///< Pointer to the Crypteo1 key to use (6 uint8_ts)
 											Uid *uid			///< Pointer to Uid struct. The first 4 uint8_ts of the UID is used.
 											) {
 	uint8_t waitIRq = 0x10;		// IdleIRq
 	
 	// Build command buffer
 	uint8_t sendData[12];
 	sendData[0] = command;
 	sendData[1] = blockAddr;
 	for (uint8_t i = 0; i < MF_KEY_SIZE; i++) {	// 6 key uint8_ts
 		sendData[2+i] = key->keyuint8_t[i];
 	}
 	// Use the last uid uint8_ts as specified in http://cache.nxp.com/documents/application_note/AN10927.pdf
 	// section 3.2.5 "MIFARE Classic Authentication".
 	// The only missed case is the MF1Sxxxx shortcut activation,
 	// but it requires cascade tag (CT) uint8_t, that is not part of uid.
 	for (uint8_t i = 0; i < 4; i++) {				// The last 4 uint8_ts of the UID
 		sendData[8+i] = uid->uiduint8_t[i+uid->size-4];
 	}
 	
 	// Start the authentication.
 	return PCD_CommunicateWithPICC(PCD_MFAuthent, waitIRq, &sendData[0], sizeof(sendData),NULL,NULL,NULL,0,0);
 } // End PCD_Authenticate()
 
 /**
  * Used to exit the PCD from its authenticated state.
  * Remember to call this function after communicating with an authenticated PICC - otherwise no new communications can start.
  */
 void PCD_StopCrypto1() {
 	// Clear MFCrypto1On bit
 	PCD_ClearRegisterBitMask(Status2Reg, 0x08); // Status2Reg[7..0] bits are: TempSensClear I2CForceHS reserved reserved MFCrypto1On ModemState[2:0]
 } // End PCD_StopCrypto1()
 
 /**
  * Reads 16 uint8_ts (+ 2 uint8_ts CRC_A) from the active PICC.
  * 
  * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
  * 
  * For MIFARE Ultralight only addresses 00h to 0Fh are decoded.
  * The MF0ICU1 returns a NAK for higher addresses.
  * The MF0ICU1 responds to the READ command by sending 16 uint8_ts starting from the page address defined by the command argument.
  * For example; if blockAddr is 03h then pages 03h, 04h, 05h, 06h are returned.
  * A roll-back is implemented: If blockAddr is 0Eh, then the contents of pages 0Eh, 0Fh, 00h and 01h are returned.
  * 
  * The buffer must be at least 18 uint8_ts because a CRC_A is also returned.
  * Checks the CRC_A before returning STATUS_OK.
  * 
  * @return STATUS_OK on success, STATUS_??? otherwise.
  */
 enum StatusCode MIFARE_Read(	uint8_t blockAddr, 	///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
 											uint8_t *buffer,		///< The buffer to store the data in
 											uint8_t *bufferSize	///< Buffer size, at least 18 uint8_ts. Also number of uint8_ts returned if STATUS_OK.
 										) {
 	enum StatusCode result;
 	
 	// Sanity check
 	if (buffer == NULL || *bufferSize < 18) {
 		return STATUS_NO_ROOM;
 	}
 	
 	// Build command buffer
 	buffer[0] = PICC_CMD_MF_READ;
 	buffer[1] = blockAddr;
 	// Calculate CRC_A
 	result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
 	if (result != STATUS_OK) {
 		return result;
 	}
 	
 	// Transmit the buffer and receive the response, validate CRC_A.
 	return PCD_TransceiveData(buffer, 4, buffer, bufferSize, NULL, 0, 1);
 } // End MIFARE_Read()
 
 /**
  * Writes 16 uint8_ts to the active PICC.
  * 
  * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
  * 
  * For MIFARE Ultralight the operation is called "COMPATIBILITY WRITE".
  * Even though 16 uint8_ts are transferred to the Ultralight PICC, only the least significant 4 uint8_ts (uint8_ts 0 to 3)
  * are written to the specified address. It is recommended to set the remaining uint8_ts 04h to 0Fh to all logic 0.
  * * 
  * @return STATUS_OK on success, STATUS_??? otherwise.
  */
 enum StatusCode MIFARE_Write(	uint8_t blockAddr, ///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The page (2-15) to write to.
 											uint8_t *buffer,	///< The 16 uint8_ts to write to the PICC
 											uint8_t bufferSize	///< Buffer size, must be at least 16 uint8_ts. Exactly 16 uint8_ts are written.
 										) {
 	enum StatusCode result;
 	
 	// Sanity check
 	if (buffer == NULL || bufferSize < 16) {
 		return STATUS_INVALID;
 	}
 	
 	// Mifare Classic protocol requires two communications to perform a write.
 	// Step 1: Tell the PICC we want to write to block blockAddr.
 	uint8_t cmdBuffer[2];
 	cmdBuffer[0] = PICC_CMD_MF_WRITE;
 	cmdBuffer[1] = blockAddr;
 	result = PCD_MIFARE_Transceive(cmdBuffer, 2,0); // Adds CRC_A and checks that the response is MF_ACK.
 	if (result != STATUS_OK) {
 		return result;
 	}
 	
 	// Step 2: Transfer the data
 	result = PCD_MIFARE_Transceive(buffer, bufferSize,0); // Adds CRC_A and checks that the response is MF_ACK.
 	if (result != STATUS_OK) {
 		return result;
 	}
 	
 	return STATUS_OK;
 } // End MIFARE_Write()
 
 /**
  * Writes a 4 uint8_t page to the active MIFARE Ultralight PICC.
  * 
  * @return STATUS_OK on success, STATUS_??? otherwise.
  */
 enum StatusCode MIFARE_Ultralight_Write(	uint8_t page, 		///< The page (2-15) to write to.
 														uint8_t *buffer,	///< The 4 uint8_ts to write to the PICC
 														uint8_t bufferSize	///< Buffer size, must be at least 4 uint8_ts. Exactly 4 uint8_ts are written.
 													) {
 	enum StatusCode result;
 	
 	// Sanity check
 	if (buffer == NULL || bufferSize < 4) {
 		return STATUS_INVALID;
 	}
 	
 	// Build commmand buffer
 	uint8_t cmdBuffer[6];
 	cmdBuffer[0] = PICC_CMD_UL_WRITE;
 	cmdBuffer[1] = page;
 	memcpy(&cmdBuffer[2], buffer, 4);
 	
 	// Perform the write
 	result = PCD_MIFARE_Transceive(cmdBuffer, 6,0); // Adds CRC_A and checks that the response is MF_ACK.
 	if (result != STATUS_OK) {
 		return result;
 	}
 	return STATUS_OK;
 } // End MIFARE_Ultralight_Write()
 
 /**
  * MIFARE Decrement subtracts the delta from the value of the addressed block, and stores the result in a volatile memory.
  * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
  * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
  * Use MIFARE_Transfer() to store the result in a block.
  * 
  * @return STATUS_OK on success, STATUS_??? otherwise.
  */
 enum StatusCode MIFARE_Decrement(	uint8_t blockAddr, ///< The block (0-0xff) number.
 												int32_t delta		///< This number is subtracted from the value of block blockAddr.
 											) {
 	return MIFARE_TwoStepHelper(PICC_CMD_MF_DECREMENT, blockAddr, delta);
 } // End MIFARE_Decrement()
 
 /**
  * MIFARE Increment adds the delta to the value of the addressed block, and stores the result in a volatile memory.
  * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
  * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
  * Use MIFARE_Transfer() to store the result in a block.
  * 
  * @return STATUS_OK on success, STATUS_??? otherwise.
  */
 enum StatusCode MIFARE_Increment(	uint8_t blockAddr, ///< The block (0-0xff) number.
 												int32_t delta		///< This number is added to the value of block blockAddr.
 											) {
 	return MIFARE_TwoStepHelper(PICC_CMD_MF_INCREMENT, blockAddr, delta);
 } // End MIFARE_Increment()
 
 /**
  * MIFARE Restore copies the value of the addressed block into a volatile memory.
  * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
  * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
  * Use MIFARE_Transfer() to store the result in a block.
  * 
  * @return STATUS_OK on success, STATUS_??? otherwise.
  */
 enum StatusCode MIFARE_Restore(	uint8_t blockAddr ///< The block (0-0xff) number.
 											) {
 	// The datasheet describes Restore as a two step operation, but does not explain what data to transfer in step 2.
 	// Doing only a single step does not work, so I chose to transfer 0L in step two.
 	return MIFARE_TwoStepHelper(PICC_CMD_MF_RESTORE, blockAddr, 0L);
 } // End MIFARE_Restore()
 
 /**
  * Helper function for the two-step MIFARE Classic protocol operations Decrement, Increment and Restore.
  * 
  * @return STATUS_OK on success, STATUS_??? otherwise.
  */
 enum StatusCode MIFARE_TwoStepHelper(	uint8_t command,	///< The command to use
 													uint8_t blockAddr,	///< The block (0-0xff) number.
 													int32_t data		///< The data to transfer in step 2
 													) {
 	enum StatusCode result;
 	uint8_t cmdBuffer[2]; // We only need room for 2 uint8_ts.
 	
 	// Step 1: Tell the PICC the command and block address
 	cmdBuffer[0] = command;
 	cmdBuffer[1] = blockAddr;
 	result = PCD_MIFARE_Transceive(	cmdBuffer, 2,0); // Adds CRC_A and checks that the response is MF_ACK.
 	if (result != STATUS_OK) {
 		return result;
 	}
 	
 	// Step 2: Transfer the data
 	result = PCD_MIFARE_Transceive(	(uint8_t *)&data, 4, 1); // Adds CRC_A and accept timeout as success.
 	if (result != STATUS_OK) {
 		return result;
 	}
 	
 	return STATUS_OK;
 } // End MIFARE_TwoStepHelper()
 
 /**
  * MIFARE Transfer writes the value stored in the volatile memory into one MIFARE Classic block.
  * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
  * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
  * 
  * @return STATUS_OK on success, STATUS_??? otherwise.
  */
 enum StatusCode MIFARE_Transfer(	uint8_t blockAddr ///< The block (0-0xff) number.
 											) {
 	enum StatusCode result;
 	uint8_t cmdBuffer[2]; // We only need room for 2 uint8_ts.
 	
 	// Tell the PICC we want to transfer the result into block blockAddr.
 	cmdBuffer[0] = PICC_CMD_MF_TRANSFER;
 	cmdBuffer[1] = blockAddr;
 	result = PCD_MIFARE_Transceive(	cmdBuffer, 2,0); // Adds CRC_A and checks that the response is MF_ACK.
 	if (result != STATUS_OK) {
 		return result;
 	}
 	return STATUS_OK;
 } // End MIFARE_Transfer()
 
 /**
  * Helper routine to read the current value from a Value Block.
  * 
  * Only for MIFARE Classic and only for blocks in "value block" mode, that
  * is: with access bits [C1 C2 C3] = [110] or [001]. The sector containing
  * the block must be authenticated before calling this function. 
  * 
  * @param[in]   blockAddr   The block (0x00-0xff) number.
  * @param[out]  value       Current value of the Value Block.
  * @return STATUS_OK on success, STATUS_??? otherwise.
   */
 enum StatusCode MIFARE_GetValue(uint8_t blockAddr, int32_t *value) {
 	enum StatusCode status;
 	uint8_t buffer[18];
 	uint8_t size = sizeof(buffer);
 	
 	// Read the block
 	status = MIFARE_Read(blockAddr, buffer, &size);
 	if (status == STATUS_OK) {
 		// Extract the value
 		int32_t b3 = buffer[3];
 		int32_t b2 = buffer[2];
 		int32_t b1 = buffer[1];
 		int32_t b0 = buffer[0];
 		*value = (b3<<24) | (b2<<16) | (b1<<8) | b0;
 	}
 	return status;
 } // End MIFARE_GetValue()
 
 /**
  * Helper routine to write a specific value into a Value Block.
  * 
  * Only for MIFARE Classic and only for blocks in "value block" mode, that
  * is: with access bits [C1 C2 C3] = [110] or [001]. The sector containing
  * the block must be authenticated before calling this function. 
  * 
  * @param[in]   blockAddr   The block (0x00-0xff) number.
  * @param[in]   value       New value of the Value Block.
  * @return STATUS_OK on success, STATUS_??? otherwise.
  */
 enum StatusCode MIFARE_SetValue(uint8_t blockAddr, int32_t value) {
 	uint8_t buffer[18];
 	
 	// Translate the int32_t into 4 uint8_ts; repeated 2x in value block
 	buffer[0] = buffer[ 8] = (value & 0xFF);
 	buffer[1] = buffer[ 9] = (value & 0xFF00) >> 8;
 	buffer[2] = buffer[10] = (value & 0xFF0000) >> 16;
 	buffer[3] = buffer[11] = (value & 0xFF000000) >> 24;
 	// Inverse 4 uint8_ts also found in value block
 	buffer[4] = ~buffer[0];
 	buffer[5] = ~buffer[1];
 	buffer[6] = ~buffer[2];
 	buffer[7] = ~buffer[3];
 	// Address 2x with inverse address 2x
 	buffer[12] = buffer[14] = blockAddr;
 	buffer[13] = buffer[15] = ~blockAddr;
 	
 	// Write the whole data block
 	return MIFARE_Write(blockAddr, buffer, 16);
 } // End MIFARE_SetValue()
 
 /**
  * Authenticate with a NTAG216.
  * 
  * Only for NTAG216. First implemented by Gargantuanman.
  * 
  * @param[in]   passWord   password.
  * @param[in]   pACK       result success???.
  * @return STATUS_OK on success, STATUS_??? otherwise.
  */
 enum StatusCode PCD_NTAG216_AUTH(uint8_t* passWord, uint8_t pACK[]) //Authenticate with 32bit password
 {
 	enum StatusCode result;
 	uint8_t				cmdBuffer[18]; // We need room for 16 uint8_ts data and 2 uint8_ts CRC_A.
 	
 	cmdBuffer[0] = 0x1B; //Comando de autentificacion
 	
 	for (uint8_t i = 0; i<4; i++)
 		cmdBuffer[i+1] = passWord[i];
 	
 	result = PCD_CalculateCRC(cmdBuffer, 5, &cmdBuffer[5]);
 	
 	if (result!=STATUS_OK) {
 		return result;
 	}
 	
 	// Transceive the data, store the reply in cmdBuffer[]
 	uint8_t waitIRq		= 0x30;	// RxIRq and IdleIRq
 	uint8_t cmdBufferSize	= sizeof(cmdBuffer);
 	uint8_t validBits		= 0;
 	uint8_t rxlength		= 5;
 	result = PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, cmdBuffer, 7, cmdBuffer, &rxlength, &validBits,0,0);
 	
 	pACK[0] = cmdBuffer[0];
 	pACK[1] = cmdBuffer[1];
 	
 	if (result!=STATUS_OK) {
 		return result;
 	}
 	
 	return STATUS_OK;
 } // End PCD_NTAG216_AUTH()
 
 
 /////////////////////////////////////////////////////////////////////////////////////
 // Support functions
 /////////////////////////////////////////////////////////////////////////////////////
 
 /**
  * Wrapper for MIFARE protocol communication.
  * Adds CRC_A, executes the Transceive command and checks that the response is MF_ACK or a timeout.
  * 
  * @return STATUS_OK on success, STATUS_??? otherwise.
  */
 enum StatusCode PCD_MIFARE_Transceive(	uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO. Do NOT include the CRC_A.
 													uint8_t sendLen,		///< Number of uint8_ts in sendData.
 													uint8_t acceptTimeout	///< 1 => A timeout is also success
 												) {
 	enum StatusCode result;
 	uint8_t cmdBuffer[18]; // We need room for 16 uint8_ts data and 2 uint8_ts CRC_A.
 	
 	// Sanity check
 	if (sendData == NULL || sendLen > 16) {
 		return STATUS_INVALID;
 	}
 	
 	// Copy sendData[] to cmdBuffer[] and add CRC_A
 	memcpy(cmdBuffer, sendData, sendLen);
 	result = PCD_CalculateCRC(cmdBuffer, sendLen, &cmdBuffer[sendLen]);
 	if (result != STATUS_OK) { 
 		return result;
 	}
 	sendLen += 2;
 	
 	// Transceive the data, store the reply in cmdBuffer[]
 	uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
 	uint8_t cmdBufferSize = sizeof(cmdBuffer);
 	uint8_t validBits = 0;
 	result = PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, cmdBuffer, sendLen, cmdBuffer, &cmdBufferSize, &validBits,0,0);
 	if (acceptTimeout && result == STATUS_TIMEOUT) {
 		return STATUS_OK;
 	}
 	if (result != STATUS_OK) {
 		return result;
 	}
 	// The PICC must reply with a 4 bit ACK
 	if (cmdBufferSize != 1 || validBits != 4) {
 		return STATUS_ERROR;
 	}
 	if (cmdBuffer[0] != MF_ACK) {
 		return STATUS_MIFARE_NACK;
 	}
 	return STATUS_OK;
 } // End PCD_MIFARE_Transceive()
 
 /**
  * Returns a 
   pointer to a status code name.
  * 
  * @return const __FlashStringHelper *
  */
 const void *GetStatusCodeName(enum StatusCode code	///< One of the enum StatusCode enums.
 										) {
 	switch (code) {
 		case STATUS_OK:				return F("Success.");
 		case STATUS_ERROR:			return F("Error in communication.");
 		case STATUS_COLLISION:		return F("Collission detected.");
 		case STATUS_TIMEOUT:		return F("Timeout in communication.");
 		case STATUS_NO_ROOM:		return F("A buffer is not big enough.");
 		case STATUS_INTERNAL_ERROR:	return F("Internal error in the code. Should not happen.");
 		case STATUS_INVALID:		return F("Invalid argument.");
 		case STATUS_CRC_WRONG:		return F("The CRC_A does not match.");
 		case STATUS_MIFARE_NACK:	return F("A MIFARE PICC responded with NAK.");
 		default:					return F("Unknown error");
 	}
 } // End GetStatusCodeName()
 
 /**
  * Translates the SAK (Select Acknowledge) to a PICC type.
  * 
  * @return PICC_Type
  */
   enum PICC_Type PICC_GetType(uint8_t sak		///< The SAK uint8_t returned from PICC_Select().
   										) {
   	// http://www.nxp.com/documents/application_note/AN10833.pdf 
   	// 3.2 Coding of Select Acknowledge (SAK)
   	// ignore 8-bit (iso14443 starts with LSBit = bit 1)
   	// fixes wrong type for manufacturer Infineon (http://nfc-tools.org/index.php?title=ISO14443A)
   	sak &= 0x7F;
   	switch (sak) {
   		case 0x04:	return PICC_TYPE_NOT_COMPLETE;	// UID not complete
   		case 0x09:	return PICC_TYPE_MIFARE_MINI;
   		case 0x08:	return PICC_TYPE_MIFARE_1K;
   		case 0x18:	return PICC_TYPE_MIFARE_4K;
   		case 0x00:	return PICC_TYPE_MIFARE_UL;
   		case 0x10:
   		case 0x11:	return PICC_TYPE_MIFARE_PLUS;
   		case 0x01:	return PICC_TYPE_TNP3XXX;
   		case 0x20:	return PICC_TYPE_ISO_14443_4;
   		case 0x40:	return PICC_TYPE_ISO_18092;
   		default:	return PICC_TYPE_UNKNOWN;
   	}
   } // End PICC_GetType()
 
 /**
  * Returns a __FlashStringHelper pointer to the PICC type name.
  * 
  * @return const __FlashStringHelper *
  */
 // const void *PICC_GetTypeName(enum PICC_Type piccType	///< One of the enum PICC_Type enums.
 // 													) {
 // 	switch (piccType) {
 // 		case PICC_TYPE_ISO_14443_4:		return F("PICC compliant with ISO/IEC 14443-4");
 // 		case PICC_TYPE_ISO_18092:		return F("PICC compliant with ISO/IEC 18092 (NFC)");
 // 		case PICC_TYPE_MIFARE_MINI:		return F("MIFARE Mini, 320 uint8_ts");
 // 		case PICC_TYPE_MIFARE_1K:		return F("MIFARE 1KB");
 // 		case PICC_TYPE_MIFARE_4K:		return F("MIFARE 4KB");
 // 		case PICC_TYPE_MIFARE_UL:		return F("MIFARE Ultralight or Ultralight C");
 // 		case PICC_TYPE_MIFARE_PLUS:		return F("MIFARE Plus");
 // 		case PICC_TYPE_TNP3XXX:			return F("MIFARE TNP3XXX");
 // 		case PICC_TYPE_NOT_COMPLETE:	return F("SAK indicates UID is not complete.");
 // 		case PICC_TYPE_UNKNOWN:
 // 		default:						return F("Unknown type");
 // 	}
 // } // End PICC_GetTypeName()
 
 /**
  * Dumps debug info about the connected PCD to //Serial.
  * Shows all known firmware versions
  */
 // void PCD_DumpVersionTo//Serial() {
 // 	// Get the MFRC522 firmware version
 // 	uint8_t v = PCD_ReadRegister(VersionReg);
 // 	//Serial.print(F("Firmware Version: 0x"));
 // 	//Serial.print(v, HEX);
 // 	// Lookup which version
 // 	switch(v) {
 // 		case 0x88: //Serial.println(F(" = (clone)"));  break;
 // 		case 0x90: //Serial.println(F(" = v0.0"));     break;
 // 		case 0x91: //Serial.println(F(" = v1.0"));     break;
 // 		case 0x92: //Serial.println(F(" = v2.0"));     break;
 // 		default:   //Serial.println(F(" = (unknown)"));
 // 	}
 // 	// When 0x00 or 0xFF is returned, communication probably failed
 // 	if ((v == 0x00) || (v == 0xFF))
 // 		//Serial.println(F("WARNING: Communication failure, is the MFRC522 properly connected?"));
 // } // End PCD_DumpVersionTo//Serial()
 
 /**
  * Dumps debug info about the selected PICC to //Serial.
  * On success the PICC is halted after dumping the data.
  * For MIFARE Classic the factory default key of 0xFFFFFFFFFFFF is tried. 
  */
 // void PICC_DumpToSerial(Uid *uid	///< Pointer to Uid struct returned from a successful PICC_Select().
 // 								) {
 // 	MIFARE_Key key;
 // 	
 // 	// Dump UID, SAK and Type
 // 	PICC_DumpDetailsToSerial(uid);
 // 	
 // 	// Dump contents
 // 	enum PICC_Type piccType = PICC_GetType(uid->sak);
 // 	switch (piccType) {
 // 		case PICC_TYPE_MIFARE_MINI:
 // 		case PICC_TYPE_MIFARE_1K:
 // 		case PICC_TYPE_MIFARE_4K:
 // 			// All keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
 // 			for (uint8_t i = 0; i < 6; i++) {
 // 				key.keyuint8_t[i] = 0xFF;
 // 			}
 // 			PICC_DumpMifareClassicToSerial(uid, piccType, &key);
 // 			break;
 // 			
 // 		case PICC_TYPE_MIFARE_UL:
 // 			PICC_DumpMifareUltralightToSerial();
 // 			break;
 // 			
 // 		case PICC_TYPE_ISO_14443_4:
 // 		case PICC_TYPE_ISO_18092:
 // 		case PICC_TYPE_MIFARE_PLUS:
 // 		case PICC_TYPE_TNP3XXX:
 // 			////Serial.println(F("Dumping memory contents not implemented for that PICC type."));
 // 			break;
 // 			
 // 		case PICC_TYPE_UNKNOWN:
 // 		case PICC_TYPE_NOT_COMPLETE:
 // 		default:
 // 			break; // No memory dump here
 // 	}
 // 	
 // 	////Serial.println();
 // 	PICC_HaltA(); // Already done if it was a MIFARE Classic PICC.
 // } // End PICC_DumpTo//Serial()
 
 /**
  * Dumps card info (UID,SAK,Type) about the selected PICC to //Serial.
  */
 // void PICC_DumpDetailsTo//Serial(Uid *uid	///< Pointer to Uid struct returned from a successful PICC_Select().
 // 									) {
 // 	// UID
 // 	//Serial.print(F("Card UID:"));
 // 	for (uint8_t i = 0; i < uid->size; i++) {
 // 		if(uid->uiduint8_t[i] < 0x10)
 // 			//Serial.print(F(" 0"));
 // 		else
 // 			//Serial.print(F(" "));
 // 		//Serial.print(uid->uiduint8_t[i], HEX);
 // 	} 
 // 	//Serial.println();
 // 	
 // 	// SAK
 // 	//Serial.print(F("Card SAK: "));
 // 	if(uid->sak < 0x10)
 // 		//Serial.print(F("0"));
 // 	//Serial.println(uid->sak, HEX);
 // 	
 // 	// (suggested) PICC type
 // 	enum PICC_Type piccType = PICC_GetType(uid->sak);
 // 	//Serial.print(F("PICC type: "));
 // 	//Serial.println(PICC_GetTypeName(piccType));
 // } // End PICC_DumpDetailsTo//Serial()
 
 /**
  * Dumps memory contents of a MIFARE Classic PICC.
  * On success the PICC is halted after dumping the data.
  */
 void PICC_DumpMifareClassicToSerial(	Uid *uid,			///< Pointer to Uid struct returned from a successful PICC_Select().
 												enum PICC_Type piccType,	///< One of the enum PICC_Type enums.
 												MIFARE_Key *key		///< Key A used for all sectors.
 											) {
 	uint8_t no_of_sectors = 0;
 	switch (piccType) {
 		case PICC_TYPE_MIFARE_MINI:
 			// Has 5 sectors * 4 blocks/sector * 16 uint8_ts/block = 320 uint8_ts.
 			no_of_sectors = 5;
 			break;
 			
 		case PICC_TYPE_MIFARE_1K:
 			// Has 16 sectors * 4 blocks/sector * 16 uint8_ts/block = 1024 uint8_ts.
 			no_of_sectors = 16;
 			break;
 			
 		case PICC_TYPE_MIFARE_4K:
 			// Has (32 sectors * 4 blocks/sector + 8 sectors * 16 blocks/sector) * 16 uint8_ts/block = 4096 uint8_ts.
 			no_of_sectors = 40;
 			break;
 			
 		default: // Should not happen. Ignore.
 			break;
 	}
 	
 	// Dump sectors, highest address first.
 	if (no_of_sectors) {
 		////Serial.println(F("Sector Block   0  1  2  3   4  5  6  7   8  9 10 11  12 13 14 15  AccessBits"));
 		for (int8_t i = no_of_sectors - 1; i >= 0; i--) {
 			PICC_DumpMifareClassicSectorToSerial(uid, key, i);
 		}
 	}
 	PICC_HaltA(); // Halt the PICC before stopping the encrypted session.
 	PCD_StopCrypto1();
 } // End PICC_DumpMifareClassicTo//Serial()
 
 /**
  * Dumps memory contents of a sector of a MIFARE Classic PICC.
  * Uses PCD_Authenticate(), MIFARE_Read() and PCD_StopCrypto1.
  * Always uses PICC_CMD_MF_AUTH_KEY_A because only Key A can always read the sector trailer access bits.
  */
 void PICC_DumpMifareClassicSectorToSerial(Uid *uid,			///< Pointer to Uid struct returned from a successful PICC_Select().
 													MIFARE_Key *key,	///< Key A for the sector.
 													uint8_t sector			///< The sector to dump, 0..39.
 													) {
 	enum StatusCode status;
 	uint8_t firstBlock;		// Address of lowest address to dump actually last block dumped)
 	uint8_t no_of_blocks;		// Number of blocks in sector
 	uint8_t isSectorTrailer;	// Set to 1 while handling the "last" (ie highest address) in the sector.
 	
 	// The access bits are stored in a peculiar fashion.
 	// There are four groups:
 	//		g[3]	Access bits for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
 	//		g[2]	Access bits for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
 	//		g[1]	Access bits for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
 	//		g[0]	Access bits for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
 	// Each group has access bits [C1 C2 C3]. In this code C1 is MSB and C3 is LSB.
 	// The four CX bits are stored together in a nible cx and an inverted nible cx_.
 	uint8_t c1, c2, c3;		// Nibbles
 	uint8_t c1_, c2_, c3_;		// Inverted nibbles
 	uint8_t invertedError;		// 1 if one of the inverted nibbles did not match
 	uint8_t g[4];				// Access bits for each of the four groups.
 	uint8_t group;				// 0-3 - active group for access bits
 	uint8_t firstInGroup;		// 1 for the first block dumped in the group
 	
 	// Determine position and size of sector.
 	if (sector < 32) { // Sectors 0..31 has 4 blocks each
 		no_of_blocks = 4;
 		firstBlock = sector * no_of_blocks;
 	}
 	else if (sector < 40) { // Sectors 32-39 has 16 blocks each
 		no_of_blocks = 16;
 		firstBlock = 128 + (sector - 32) * no_of_blocks;
 	}
 	else { // Illegal input, no MIFARE Classic PICC has more than 40 sectors.
 		return;
 	}
 		
 	// Dump blocks, highest address first.
 	uint8_t uint8_tCount;
 	uint8_t buffer[18];
 	uint8_t blockAddr;
 	isSectorTrailer = 1;
 	for (int8_t blockOffset = no_of_blocks - 1; blockOffset >= 0; blockOffset--) {
 		blockAddr = firstBlock + blockOffset;
 		// Sector number - only on first line
 		if (isSectorTrailer) {
 // 			if(sector < 10)
 // 				//Serial.print(F("   ")); // Pad with spaces
 // 			else
 // 				//Serial.print(F("  ")); // Pad with spaces
 // 			//Serial.print(sector);
 // 			//Serial.print(F("   "));
 		}
 		else {
 //			//Serial.print(F("       "));
 		}
 		// Block number
 		if(blockAddr < 10){}
 //			//Serial.print(F("   ")); // Pad with spaces
 		else {
 // 			if(blockAddr < 100)
 // 				//Serial.print(F("  ")); // Pad with spaces
 // 			else
 // 				//Serial.print(F(" ")); // Pad with spaces
 		}
 // 		//Serial.print(blockAddr);
 // 		//Serial.print(F("  "));
 		// Establish encrypted communications before reading the first block
 		if (isSectorTrailer) {
 			status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, firstBlock, key, uid);
 			if (status != STATUS_OK) {
 				//Serial.print(F("PCD_Authenticate() failed: "));
 				//Serial.println(GetStatusCodeName(status));
 				return;
 			}
 		}
 		// Read block
 		uint8_tCount = sizeof(buffer);
 		status = MIFARE_Read(blockAddr, buffer, &uint8_tCount);
 		if (status != STATUS_OK) {
 			//Serial.print(F("MIFARE_Read() failed: "));
 			//Serial.println(GetStatusCodeName(status));
 			continue;
 		}
 		// Dump data
 		for (uint8_t index = 0; index < 16; index++) {
 			if(buffer[index] < 0x10){}
 				//Serial.print(F(" 0"));
 			else
 				//Serial.print(F(" "));
 			//Serial.print(buffer[index], HEX);
 			if ((index % 4) == 3) {
 				//Serial.print(F(" "));
 			}
 		}
 		// Parse sector trailer data
 		if (isSectorTrailer) {
 			c1  = buffer[7] >> 4;
 			c2  = buffer[8] & 0xF;
 			c3  = buffer[8] >> 4;
 			c1_ = buffer[6] & 0xF;
 			c2_ = buffer[6] >> 4;
 			c3_ = buffer[7] & 0xF;
 			invertedError = (c1 != (~c1_ & 0xF)) || (c2 != (~c2_ & 0xF)) || (c3 != (~c3_ & 0xF));
 			g[0] = ((c1 & 1) << 2) | ((c2 & 1) << 1) | ((c3 & 1) << 0);
 			g[1] = ((c1 & 2) << 1) | ((c2 & 2) << 0) | ((c3 & 2) >> 1);
 			g[2] = ((c1 & 4) << 0) | ((c2 & 4) >> 1) | ((c3 & 4) >> 2);
 			g[3] = ((c1 & 8) >> 1) | ((c2 & 8) >> 2) | ((c3 & 8) >> 3);
 			isSectorTrailer = 0;
 		}
 		
 		// Which access group is this block in?
 		if (no_of_blocks == 4) {
 			group = blockOffset;
 			firstInGroup = 1;
 		}
 		else {
 			group = blockOffset / 5;
 			firstInGroup = (group == 3) || (group != (blockOffset + 1) / 5);
 		}
 		
 		if (firstInGroup) {
 			// Print access bits
 			//Serial.print(F(" [ "));
 			//Serial.print((g[group] >> 2) & 1, DEC); //Serial.print(F(" "));
 			//Serial.print((g[group] >> 1) & 1, DEC); //Serial.print(F(" "));
 			//Serial.print((g[group] >> 0) & 1, DEC);
 			//Serial.print(F(" ] "));
 			if (invertedError) {
 				//Serial.print(F(" Inverted access bits did not match! "));
 			}
 		}
 		
 		if (group != 3 && (g[group] == 1 || g[group] == 6)) { // Not a sector trailer, a value block
 			int32_t b3 = buffer[3];
 			int32_t b2 = buffer[2];
 			int32_t b1 = buffer[1];
 			int32_t b0 = buffer[0];
 			int32_t value = (b3<<24) | (b2<<16) | (b1<<8) | b0;
 			//Serial.print(F(" Value=0x")); //Serial.print(value, HEX);
 			//Serial.print(F(" Adr=0x")); //Serial.print(buffer[12], HEX);
 		}
 		//Serial.println();
 	}
 	
 	return;
 } // End PICC_DumpMifareClassicSectorTo//Serial()
 
 /**
  * Dumps memory contents of a MIFARE Ultralight PICC.
  */
 void PICC_DumpMifareUltralightToSerial() {
 	enum StatusCode status;
 	uint8_t uint8_tCount;
 	uint8_t buffer[18];
 	uint8_t i;
 	
 	//Serial.println(F("Page  0  1  2  3"));
 	// Try the mpages of the original Ultralight. Ultralight C has more pages.
 	for (uint8_t page = 0; page < 16; page +=4) { // Read returns data for 4 pages at a time.
 		// Read pages
 		uint8_tCount = sizeof(buffer);
 		status = MIFARE_Read(page, buffer, &uint8_tCount);
 		if (status != STATUS_OK) {
 			//Serial.print(F("MIFARE_Read() failed: "));
 			//Serial.println(GetStatusCodeName(status));
 			break;
 		}
 		// Dump data
 		for (uint8_t offset = 0; offset < 4; offset++) {
 			i = page + offset;
 			if(i < 10){}
 				//Serial.print(F("  ")); // Pad with spaces
 			else{}
 				//Serial.print(F(" ")); // Pad with spaces
 			//Serial.print(i);
 			//Serial.print(F("  "));
 			for (uint8_t index = 0; index < 4; index++) {
 				i = 4 * offset + index;
 				if(buffer[i] < 0x10){}
 					//Serial.print(F(" 0"));
 				else{}
 					//Serial.print(F(" "));
 				//Serial.print(buffer[i], HEX);
 			}
 			//Serial.println();
 		}
 	}
 } // End PICC_DumpMifareUltralightTo//Serial()
 
 /**
  * Calculates the bit pattern needed for the specified access bits. In the [C1 C2 C3] tuples C1 is MSB (=4) and C3 is LSB (=1).
  */
 void MIFARE_SetAccessBits(	uint8_t *accessBitBuffer,	///< Pointer to uint8_t 6, 7 and 8 in the sector trailer. uint8_ts [0..2] will be set.
 									uint8_t g0,				///< Access bits [C1 C2 C3] for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
 									uint8_t g1,				///< Access bits C1 C2 C3] for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
 									uint8_t g2,				///< Access bits C1 C2 C3] for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
 									uint8_t g3					///< Access bits C1 C2 C3] for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
 								) {
 	uint8_t c1 = ((g3 & 4) << 1) | ((g2 & 4) << 0) | ((g1 & 4) >> 1) | ((g0 & 4) >> 2);
 	uint8_t c2 = ((g3 & 2) << 2) | ((g2 & 2) << 1) | ((g1 & 2) << 0) | ((g0 & 2) >> 1);
 	uint8_t c3 = ((g3 & 1) << 3) | ((g2 & 1) << 2) | ((g1 & 1) << 1) | ((g0 & 1) << 0);
 	
 	accessBitBuffer[0] = (~c2 & 0xF) << 4 | (~c1 & 0xF);
 	accessBitBuffer[1] =          c1 << 4 | (~c3 & 0xF);
 	accessBitBuffer[2] =          c3 << 4 | c2;
 } // End MIFARE_SetAccessBits()
 
 
 /**
  * Performs the "magic sequence" needed to get Chinese UID changeable
  * Mifare cards to allow writing to sector 0, where the card UID is stored.
  *
  * Note that you do not need to have selected the card through REQA or WUPA,
  * this sequence works immediately when the card is in the reader vicinity.
  * This means you can use this method even on "bricked" cards that your reader does
  * not recognise anymore (see MIFARE_UnbrickUidSector).
  * 
  * Of course with non-bricked devices, you're free to select them before calling this function.
  */
 uint8_t MIFARE_OpenUidBackdoor(uint8_t logErrors) {
 	// Magic sequence:
 	// > 50 00 57 CD (HALT + CRC)
 	// > 40 (7 bits only)
 	// < A (4 bits only)
 	// > 43
 	// < A (4 bits only)
 	// Then you can write to sector 0 without authenticating
 	
 	PICC_HaltA(); // 50 00 57 CD
 	
 	uint8_t cmd = 0x40;
 	uint8_t validBits = 7; /* Our command is only 7 bits. After receiving card response,
 						  this will contain amount of valid response bits. */
 	uint8_t response[32]; // Card's response is written here
 	uint8_t received;
 	enum StatusCode status = PCD_TransceiveData(&cmd, (uint8_t)1, response, &received, &validBits, (uint8_t)0, 0); // 40
 	if(status != STATUS_OK) {
 		if(logErrors) {
 			//Serial.println(F("Card did not respond to 0x40 after HALT command. Are you sure it is a UID changeable one?"));
 			//Serial.print(F("Error name: "));
 			//Serial.println(GetStatusCodeName(status));
 		}
 		return 0;
 	}
 	if (received != 1 || response[0] != 0x0A) {
 		if (logErrors) {
 			//Serial.print(F("Got bad response on backdoor 0x40 command: "));
 			//Serial.print(response[0], HEX);
 			//Serial.print(F(" ("));
 			//Serial.print(validBits);
 			//Serial.print(F(" valid bits)\r\n"));
 		}
 		return 0;
 	}
 	
 	cmd = 0x43;
 	validBits = 8;
 	status = PCD_TransceiveData(&cmd, (uint8_t)1, response, &received, &validBits, (uint8_t)0, 0); // 43
 	if(status != STATUS_OK) {
 		if(logErrors) {
 			//Serial.println(F("Error in communication at command 0x43, after successfully executing 0x40"));
 			//Serial.print(F("Error name: "));
 			//Serial.println(GetStatusCodeName(status));
 		}
 		return 0;
 	}
 	if (received != 1 || response[0] != 0x0A) {
 		if (logErrors) {
 			//Serial.print(F("Got bad response on backdoor 0x43 command: "));
 			//Serial.print(response[0], HEX);
 			//Serial.print(F(" ("));
 			//Serial.print(validBits);
 			//Serial.print(F(" valid bits)\r\n"));
 		}
 		return 0;
 	}
 	
 	// You can now write to sector 0 without authenticating!
 	return 1;
 } // End MIFARE_OpenUidBackdoor()
 
 /**
  * Reads entire block 0, including all manufacturer data, and overwrites
  * that block with the new UID, a freshly calculated BCC, and the original
  * manufacturer data.
  *
  * It assumes a default KEY A of 0xFFFFFFFFFFFF.
  * Make sure to have selected the card before this function is called.
  */
 uint8_t MIFARE_SetUid(uint8_t *newUid, uint8_t uidSize, uint8_t logErrors) {
 	
 	// UID + BCC uint8_t can not be larger than 16 together
 	if (!newUid || !uidSize || uidSize > 15) {
 		if (logErrors) {
 			//Serial.println(F("New UID buffer empty, size 0, or size > 15 given"));
 		}
 		return 0;
 	}
 	
 	// Authenticate for reading
 	MIFARE_Key key = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
 	enum StatusCode status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, (uint8_t)1, &key, &uid);
 	if (status != STATUS_OK) {
 		
 		if (status == STATUS_TIMEOUT) {
 			// We get a read timeout if no card is selected yet, so let's select one
 			
 			// Wake the card up again if sleeping
 //			  uint8_t atqa_answer[2];
 //			  uint8_t atqa_size = 2;
 //			  PICC_WakeupA(atqa_answer, &atqa_size);
 			
 			if (!PICC_IsNewCardPresent() || !PICC_ReadCardSerial()) {
 				//Serial.println(F("No card was previously selected, and none are available. Failed to set UID."));
 				return 0;
 			}
 			
 			status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, (uint8_t)1, &key, &uid);
 			if (status != STATUS_OK) {
 				// We tried, time to give up
 				if (logErrors) {
 					//Serial.println(F("Failed to authenticate to card for reading, could not set UID: "));
 					//Serial.println(GetStatusCodeName(status));
 				}
 				return 0;
 			}
 		}
 		else {
 			if (logErrors) {
 				//Serial.print(F("PCD_Authenticate() failed: "));
 				//Serial.println(GetStatusCodeName(status));
 			}
 			return 0;
 		}
 	}
 	
 	// Read block 0
 	uint8_t block0_buffer[18];
 	uint8_t uint8_tCount = sizeof(block0_buffer);
 	status = MIFARE_Read((uint8_t)0, block0_buffer, &uint8_tCount);
 	if (status != STATUS_OK) {
 		if (logErrors) {
 			//Serial.print(F("MIFARE_Read() failed: "));
 			//Serial.println(GetStatusCodeName(status));
 			//Serial.println(F("Are you sure your KEY A for sector 0 is 0xFFFFFFFFFFFF?"));
 		}
 		return 0;
 	}
 	
 	// Write new UID to the data we just read, and calculate BCC uint8_t
 	uint8_t bcc = 0;
 	for (uint8_t i = 0; i < uidSize; i++) {
 		block0_buffer[i] = newUid[i];
 		bcc ^= newUid[i];
 	}
 	
 	// Write BCC uint8_t to buffer
 	block0_buffer[uidSize] = bcc;
 	
 	// Stop encrypted traffic so we can send raw uint8_ts
 	PCD_StopCrypto1();
 	
 	// Activate UID backdoor
 	if (!MIFARE_OpenUidBackdoor(logErrors)) {
 		if (logErrors) {
 			//Serial.println(F("Activating the UID backdoor failed."));
 		}
 		return 0;
 	}
 	
 	// Write modified block 0 back to card
 	status = MIFARE_Write((uint8_t)0, block0_buffer, (uint8_t)16);
 	if (status != STATUS_OK) {
 		if (logErrors) {
 			//Serial.print(F("MIFARE_Write() failed: "));
 			//Serial.println(GetStatusCodeName(status));
 		}
 		return 0;
 	}
 	
 	// Wake the card up again
 	uint8_t atqa_answer[2];
 	uint8_t atqa_size = 2;
 	PICC_WakeupA(atqa_answer, &atqa_size);
 	
 	return 1;
 }
 
 /**
  * Resets entire sector 0 to zeroes, so the card can be read again by readers.
  */
 uint8_t MIFARE_UnbrickUidSector(uint8_t logErrors) {
 	MIFARE_OpenUidBackdoor(logErrors);
 	
 	uint8_t block0_buffer[] = {0x01, 0x02, 0x03, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
 	
 	// Write modified block 0 back to card
 	enum StatusCode status = MIFARE_Write((uint8_t)0, block0_buffer, (uint8_t)16);
 	if (status != STATUS_OK) {
 		if (logErrors) {
 			//Serial.print(F("MIFARE_Write() failed: "));
 			//Serial.println(GetStatusCodeName(status));
 		}
 		return 0;
 	}
 	return 1;
 }
 
 /////////////////////////////////////////////////////////////////////////////////////
 // Convenience functions - does not add extra functionality
 /////////////////////////////////////////////////////////////////////////////////////
 
 /**
  * Returns 1 if a PICC responds to PICC_CMD_REQA.
  * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
  * 
  * @return uint8_t
  */
 uint8_t PICC_IsNewCardPresent() {
 	uint8_t bufferATQA[2];
 	uint8_t bufferSize = sizeof(bufferATQA);
 	enum StatusCode result = PICC_RequestA(bufferATQA, &bufferSize);
 	return (result == STATUS_OK || result == STATUS_COLLISION);
 } // End PICC_IsNewCardPresent()
 
 /**
  * Simple wrapper around PICC_Select.
  * Returns 1 if a UID could be read.
  * Remember to call PICC_IsNewCardPresent(), PICC_RequestA() or PICC_WakeupA() first.
  * The read UID is available in the class variable uid.
  * 
  * @return uint8_t
  */
 uint8_t PICC_ReadCardSerial() {
 	enum StatusCode result = PICC_Select(&uid,0);
 	return (result == STATUS_OK);
 } // End 
