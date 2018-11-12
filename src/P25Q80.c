
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

void flash_write_single_q(uint8_t byte)
{
	buffer[buf_p] = byte;
	spi_queue_push(buffer+buf_p,NULL,1,NULL);
	buf_p = (buf_p+1)&0xFF;
}

void flash_write_en_q(void)
{
	flash_write_single_q(0x06);
}
void flash_chip_erase_q(void)
{
	flash_write_en_q();
	buffer[buf_p] = 0x60;
	spi_queue_push(buffer+buf_p,NULL,1,NULL);
	buf_p = (buf_p+1)&0xFF;
}

void flash_sector_erase_q(uint32_t addr)
{
	flash_write_en_q();
	buffer[buf_p] = 0x20;
	buffer[(buf_p+1)&0xFF] = (addr >> 16) & 0xFF;
	buffer[(buf_p+2)&0xFF] = (addr >> 8) & 0xFF;
	buffer[(buf_p+3)&0xFF] = addr & 0xFF;
	spi_queue_push(buffer+buf_p,NULL,4,NULL);
	buf_p = (buf_p+4)&0xFF;
}

static bool isIdle(uint8_t* read)
{
	if(read[1] & 0x01) {
		return true;
	} else {
		return false;
	}
}
void flash_wait_idle_q(void)
{
	buffer[buf_p] = 0x05;
	buffer[(buf_p+1)&0xFF] = 0xFF;
	spi_queue_push(buffer+buf_p,spi_rev0,2,isIdle);
	buf_p = (buf_p+2)&0xFF;
}

static uint16_t fbb_ptr=0;
static uint8_t flash_big_buffer[0x2000];
void flash_write_q(uint8_t* content, uint32_t addr, uint16_t length)
{
	flash_write_en_q();
	if(fbb_ptr+length+4>0x2000) fbb_ptr = 0;
	memcpy(flash_big_buffer+fbb_ptr+4, content, length);
	flash_big_buffer[fbb_ptr+0] = 0x02;
	flash_big_buffer[fbb_ptr+1] = (addr >> 16) & 0xFF;
	flash_big_buffer[fbb_ptr+2] = (addr >> 8) & 0xFF;
	flash_big_buffer[fbb_ptr+3] = addr & 0xFF;
	spi_queue_push(flash_big_buffer+fbb_ptr,NULL,length+4,NULL);
	fbb_ptr += length+4;
}

void flash_read_q(uint8_t* buf, uint32_t addr, uint8_t length)
{
	// uint8_t* read;
	// buffer[0] = 0x03;
	// buffer[1] = (addr >> 16) & 0xFF;
	// buffer[2] = (addr >> 8) & 0xFF;
	// buffer[3] = addr & 0xFF;
	// memset(buffer + 4, 0xFF, length);
	// read = spi0_write(buffer, length + 4);
	// memcpy(buf, read + 4, length);

	flash_big_buffer[0] = 0x03;
	flash_big_buffer[1] = (addr >> 16) & 0xFF;
	flash_big_buffer[2] = (addr >> 8) & 0xFF;
	flash_big_buffer[3] = addr & 0xFF;
	spi_queue_push(flash_big_buffer,buf,length+4,NULL);
}
