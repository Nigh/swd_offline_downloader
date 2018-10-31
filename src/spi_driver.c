
#include "spi_driver.h"

#include "stm32f10x.h"
#include "SPI_STM32F10x.h"
#include "GPIO_STM32F10x.h"
static uint8_t txBuffer[16];
static uint8_t rxBuffer[0x100];
static uint8_t dataBuffer[16];


sSPI_QUEUE spi_queue[SPI_QUEUE_LENGTH];
uint8_t sq_w = 0;
uint8_t sq_r = 0;
#include "scheduler.h"
void spi_queue_push(uint8_t *buffer, uint8_t *rev, uint16_t length,bool (*callback)(uint8_t *r))
{
    if(sq_w==sq_r){
        platform_simple_evt_put(spi_queue_dispatch);
    }
    spi_queue[sq_w].w=buffer;
    spi_queue[sq_w].r=rev;
    spi_queue[sq_w].l=length;
    spi_queue[sq_w].callback=callback;
    sq_w = (sq_w+1)&(SPI_QUEUE_LENGTH-1);
}
void spi_queue_dispatch(void)
{
    if(sq_w!=sq_r){
        spi_wr(spi_queue[sq_r].w,spi_queue[sq_r].r,spi_queue[sq_r].l);
        sq_r = (sq_r+1)&(SPI_QUEUE_LENGTH-1);
    }
}
// __weak void SPI1_Callback(uint32_t event) { ; }
extern void SPI1_Callback(uint32_t event);
void spi_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    // GPIO_PinConfigure(GPIOA,4,GPIO_OUT_PUSH_PULL,GPIO_MODE_OUT50MHZ);
    // GPIO_PinWrite(GPIOA,4,1);
    Driver_SPI1.Initialize(SPI1_Callback);
    Driver_SPI1.PowerControl(ARM_POWER_FULL);
    Driver_SPI1.Control(ARM_SPI_MODE_MASTER |
                        ARM_SPI_CPOL0_CPHA0 |
                        ARM_SPI_MSB_LSB |
						// ARM_SPI_SS_MASTER_UNUSED |
                        ARM_SPI_SS_MASTER_SW |
                        // ARM_SPI_SS_MASTER_HW_OUTPUT |
                        // ARM_SPI_DATA_BITS(8), 10000000);
                        // ARM_SPI_DATA_BITS(8), 8000000);
                        ARM_SPI_DATA_BITS(8), 2000000);
    Driver_SPI1.Transfer(txBuffer, rxBuffer, sizeof(txBuffer));
}

void spi_write(uint8_t *buffer, uint16_t length)
{
	Driver_SPI1.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
    Driver_SPI1.Send(buffer, length);
}

void spi_wr(uint8_t *buffer, uint8_t *rev, uint16_t length)
{
	Driver_SPI1.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
    if(rev!=NULL){
        Driver_SPI1.Transfer(buffer, rev, length);
    }else{
        Driver_SPI1.Send(buffer, length);
    }
}

uint8_t* spi_write0(uint8_t *buffer, uint16_t length)
{
	Driver_SPI1.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
    Driver_SPI1.Transfer(buffer, rxBuffer, length);
    return rxBuffer;
}
