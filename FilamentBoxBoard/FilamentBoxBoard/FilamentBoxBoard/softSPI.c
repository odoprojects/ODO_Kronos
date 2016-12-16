/*
 * softSPI.c
 *
 * Created: 12/12/2016 4:16:53 PM
 *  Author: Tomek
 */ 


#include "hardware.h"
#include "Pins.h"
#include <util/delay.h>

void ads_start()
{
	// A0=0 AIN1, A0=1 AIN2
	ADS_PDWN_HIGH; // ADS ON	
}

void SPI_RX_Data(uint32_t *wsk)
{
	uint32_t data=0;
	
		for(uint8_t i=0; i<24; i++)
		{
			ADS_SCLK_HIGH;
			_delay_us(10);
			ADS_SCLK_LOW;
			_delay_us(10);
			
			data<<=1;
			data|=((ADS_DRDY_IF_HI)>0) ? 1 : 0;
		}
		ADS_SCLK_HIGH;
		_delay_us(10);
		ADS_SCLK_LOW;
		_delay_us(10);
		
	wsk =  data;
}