
#include "P25Q80.h"
#include "spi_driver.h"

#define spi0_write spi_write0

static uint8_t buffer[0x100];
static uint8_t buf_p=0;

void flash_write_single(uint8_t byte)
{
	buffer[0] = byte;
	spi_write(buffer, 1);
}

void flash_write_en(void)
{
	flash_write_single(0x06);
}

// bool flash_is_busy(void)
// {
// 	uint8_t* read;
// 	read = flash_read_reg();
// 	if(read[1] & 0x01) {
// 		return true;
// 	} else {
// 		return false;
// 	}
// }

// void flash_wait_idle(void)
// {
// 	while(flash_is_busy()){ ; }
// }

void flash_write(uint8_t* content, uint32_t addr, uint16_t length)
{
	flash_write_en();
	memcpy(buffer + 4, content, length);
	buffer[0] = 0x02;
	buffer[1] = (addr >> 16) & 0xFF;
	buffer[2] = (addr >> 8) & 0xFF;
	buffer[3] = addr & 0xFF;
	spi_write(buffer, length + 4);
}

// void flash_read(uint8_t* buf, uint32_t addr, uint8_t length)
// {
// 	uint8_t* read;
// 	buffer[0] = 0x03;
// 	buffer[1] = (addr >> 16) & 0xFF;
// 	buffer[2] = (addr >> 8) & 0xFF;
// 	buffer[3] = addr & 0xFF;
// 	memset(buffer + 4, 0xFF, length);
// 	read = spi0_write(buffer, length + 4);
// 	memcpy(buf, read + 4, length);
// }

uint8_t dummy[0x20] __attribute__((section(".ARM.__at_0x20000000")));
// void flash_direct_read(uint8_t* buf, uint32_t addr, uint16_t length)
// {
// 	buffer[0] = 0x03;
// 	buffer[1] = (addr >> 16) & 0xFF;
// 	buffer[2] = (addr >> 8) & 0xFF;
// 	buffer[3] = addr & 0xFF;
// 	// 备份原ram内容
// 	buffer[4] = *(buf - 1);
// 	buffer[5] = *(buf - 2);
// 	buffer[6] = *(buf - 3);
// 	buffer[7] = *(buf - 4);
// 	spi0_trans_advance(buffer, 4, buf - 4, length + 4);
// 	// 恢复原ram内容
// 	*(buf - 1) = buffer[4];
// 	*(buf - 2) = buffer[5];
// 	*(buf - 3) = buffer[6];
// 	*(buf - 4) = buffer[7];
// }

// uint8_t* flash_read_id(void)
// {
// 	uint8_t* read;
// 	buffer[0] = 0x4B;
// 	buffer[1] = 0xFF;
// 	buffer[2] = 0xFF;
// 	buffer[3] = 0xFF;
// 	buffer[4] = 0xFF;
// 	read = spi0_write(buffer, 5 + 16);
// 	return read;
// }

// uint8_t* flash_read_reg(void)
// {
// 	uint8_t* read;
// 	buffer[0] = 0x05;
// 	buffer[1] = 0xFF;
// 	read = spi0_write(buffer, 2);
// 	return read;
// }

void flash_page_erase(uint32_t addr)
{
	flash_write_en();
	buffer[0] = 0x81;
	buffer[1] = (addr >> 16) & 0xFF;
	buffer[2] = (addr >> 8) & 0xFF;
	buffer[3] = addr & 0xFF;
	spi_write(buffer, 4);
}

void flash_sector_erase(uint32_t addr)
{
	flash_write_en();
	buffer[0] = 0x20;
	buffer[1] = (addr >> 16) & 0xFF;
	buffer[2] = (addr >> 8) & 0xFF;
	buffer[3] = addr & 0xFF;
	spi_write(buffer, 4);
}

void flash_block32k_erase(uint32_t addr)
{
	flash_write_en();
	buffer[0] = 0x52;
	buffer[1] = (addr >> 16) & 0xFF;
	buffer[2] = (addr >> 8) & 0xFF;
	buffer[3] = addr & 0xFF;
	spi_write(buffer, 4);
}

void flash_chip_erase(void)
{
	flash_write_en();
	buffer[0] = 0x60;
	spi_write(buffer, 1);
}

static uint8_t spi_rev0[0x10];


static bool isIdle(uint8_t* read)
{
	if(read[1] & 0x01) {
		return true;
	} else {
		return false;
	}
}
