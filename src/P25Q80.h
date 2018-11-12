
#ifndef _P25Q80_H_
#define _P25Q80_H_
#include <stdint.h>
#include <stdbool.h>

bool flash_is_busy(void);
void flash_write_en(void);
void flash_write_single(uint8_t byte);
void flash_write(uint8_t* content, uint32_t addr, uint16_t length);
void flash_read(uint8_t* buf, uint32_t addr, uint8_t length);
// void flash_direct_read(uint8_t* buf, uint32_t addr, uint16_t length);
void flash_page_erase(uint32_t addr);
void flash_block32k_erase(uint32_t addr);
uint8_t* flash_read_id(void);
uint8_t* flash_read_reg(void);
void flash_wait_idle(void);

void flash_write_single_q(uint8_t byte);
void flash_write_en_q(void);
void flash_chip_erase_q(void);
void flash_wait_idle_q(void);
void flash_write_q(uint8_t* content, uint32_t addr, uint16_t length);
void flash_read_q(uint8_t* buf, uint32_t addr, uint8_t length);
void flash_sector_erase_q(uint32_t addr);

#endif
