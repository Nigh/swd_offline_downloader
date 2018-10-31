
#ifndef _SPI_DRIVER_H_
#define _SPI_DRIVER_H_
#include <stdint.h>
#include "Driver_SPI.h"

extern ARM_DRIVER_SPI Driver_SPI1;

extern bool spi_flag;

void spi_init(void);
void spi_write(uint8_t *buffer, uint16_t length);
void spi_wr(uint8_t *buffer, uint8_t *rev, uint16_t length);
uint8_t* spi_write0(uint8_t *buffer, uint16_t length);

typedef struct
{
    uint8_t *w;
    uint8_t *r;
    uint16_t l;
    bool (*callback)(uint8_t *r);
}sSPI_QUEUE;

extern uint8_t sq_w;
extern uint8_t sq_r;
#define SPI_QUEUE_LENGTH 0x40
extern sSPI_QUEUE spi_queue[];
void spi_queue_push(uint8_t *buffer, uint8_t *rev, uint16_t length,bool (*callback)(uint8_t *r));
void spi_queue_dispatch(void);

#endif
