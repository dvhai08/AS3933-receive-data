#include "AS3933.h"
#include "main.h"
#include "spi.h"

/*HARDWARE*/
#define SPIx	hspi1
#define CS_AS3933_H() LL_GPIO_SetOutputPin(CS_AS3933_GPIO_Port, CS_AS3933_Pin);
#define CS_AS3933_L() LL_GPIO_ResetOutputPin(CS_AS3933_GPIO_Port, CS_AS3933_Pin);

/**
* params:CLEAR_WAKE, RESET_RSSI, CALIB_RCOSC
* CLEAR_FALSE, PRESET_DEFAULT, CALIB_RCO_LC
*/
static void write_direct(uint8_t b_cmd)
{
	uint8_t b_cmd_tmp = 0;

	b_cmd_tmp = 0xC0 | b_cmd;
	CS_AS3933_H();
//	spi_transmit(SPIx, &cmd, 1);
	HAL_SPI_Transmit(&SPIx, &b_cmd_tmp, 1, 1000);
	CS_AS3933_L();
}

static void write(uint8_t reg, uint8_t data)
{
	uint8_t cmd = reg & 0x3F;
	CS_AS3933_H();
//	spi_transmit(SPIx, &cmd, 1);
//	spi_transmit(SPIx, &data, 1);
	HAL_SPI_Transmit(&SPIx, &cmd, 1, 1000);
	HAL_SPI_Transmit(&SPIx, &data, 1, 1000);
	CS_AS3933_L();
}

static uint8_t read(uint8_t reg)
{
	uint8_t b_tmp = reg | 0x40;

	CS_AS3933_H();
//	spi_transmit(SPIx, &b_tmp, 1);
//	spi_transmit_receive(SPIx, &b_tmp, &b_tmp, 1);
	HAL_SPI_Transmit(&SPIx, &b_tmp, 1, 1000);
	b_tmp = 0;
	HAL_SPI_Receive(&SPIx, &b_tmp, 1, 1000);
	CS_AS3933_L();

	return b_tmp;
}

void as3933_write_Rx(uint8_t reg, uint8_t val)
{
	write(reg, val);
}

uint8_t as3933_read_Rx(uint8_t reg)
{
	return read(reg);
}

void as3933_write_direct(uint8_t cmd)
{
	write_direct(cmd);
}

void as3933_set_pattern_16(uint8_t *pattern)
{
   // PATT2B	
	write(AS3933_R5, pattern[1]);
	// PATT1B
	write(AS3933_R6, pattern[0]);
}

void as3933_get_pattern_16(uint8_t *pattern)
{
	pattern[1] = read(AS3933_R5);
	pattern[0] = read(AS3933_R6);
}

void as3933_set_listening_mode(void)
{
	as3933_write_direct(AS3933_CLEAR_WAKE);
}

void as3933_reset_rssi(void)
{
	as3933_write_direct(AS3933_RESET_RSSI);
}




