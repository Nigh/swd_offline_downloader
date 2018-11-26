
#include "spi_driver.h"

#include "stm32f10x.h"
#include "SPI_STM32F10x.h"
#include "GPIO_STM32F10x.h"
uint8_t sq_w = 0;
uint8_t sq_r = 0;
#include "scheduler.h"

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
                        // ARM_SPI_DATA_BITS(8), 8000000);  // 无中断产生？
                        // ARM_SPI_DATA_BITS(8), 4000000);  // 偶尔无中断产生
                        ARM_SPI_DATA_BITS(8), 2000000);     // 有中断产生
    Driver_SPI1.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
    // Driver_SPI1.Transfer(txBuffer, rxBuffer, sizeof(txBuffer));
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
