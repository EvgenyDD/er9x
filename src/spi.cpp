#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "spi.h"

#include "avr/io.h"
#include <avr/interrupt.h>

uint16_t SPI_Send(uint16_t data)
{
	SPDR = data;

	/* Wait for transfer finished. */
	while(!(SPSR & (1<<SPIF)));

	/* Read the data (8 bits) from DR. */
	return SPDR;
}


void SPI_Init()
{
	DDRB |= /*(1<<0)|*/(1<<1)|(1<<2); //spi pins on port b MOSI SCK,SS outputs
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0)/*(1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<CPOL)|(1<<CPHA)*/;  // SPI enable, Master, f/16
}

void SPI_DeInit()
{
	SPCR  &= ~(1<<SPE);  // SPI disable
}
