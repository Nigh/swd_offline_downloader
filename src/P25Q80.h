
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

#endif
