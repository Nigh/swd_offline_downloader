
#include "spi_driver.h"

#include "stm32f10x.h"
#include "SPI_STM32F10x.h"
#include "GPIO_STM32F10x.h"
uint8_t sq_w = 0;
uint8_t sq_r = 0;
#include "scheduler.h"

void SPI_Init(SPI_TypeDef* SPIx, SPI_InitTypeDef* SPI_InitStruct);
void SPI_StructInit(SPI_InitTypeDef* SPI_InitStruct);
void SPI_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_SSOutputCmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_DataSizeConfig(SPI_TypeDef* SPIx, uint16_t SPI_DataSize);
void SPI_I2S_SendData(SPI_TypeDef* SPIx, uint16_t Data);
uint16_t SPI_I2S_ReceiveData(SPI_TypeDef* SPIx);
// __weak void SPI1_Callback(uint32_t event) { ; }
extern void SPI1_Callback(uint32_t event);
uint8_t spi_inited = 0;
void spi_init(void)
{
    if(spi_inited) return;
    spi_inited = 1;
    // uint8_t fill=0xFF;
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1|RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    Driver_SPI1.Initialize(NULL);
    Driver_SPI1.PowerControl(ARM_POWER_FULL);
    Driver_SPI1.Control(ARM_SPI_MODE_MASTER |
                        ARM_SPI_CPOL0_CPHA0 |
                        ARM_SPI_MSB_LSB |
						ARM_SPI_SS_MASTER_UNUSED |
                        // ARM_SPI_SS_MASTER_SW |
                        // ARM_SPI_SS_MASTER_HW_OUTPUT |
                        // ARM_SPI_DATA_BITS(8), 10000000);
                        // ARM_SPI_DATA_BITS(8), 8000000);  // 无中断产生？
                        // ARM_SPI_DATA_BITS(8), 4000000);  // 偶尔无中断产生
                        ARM_SPI_DATA_BITS(8), 2000000);  // 偶尔产生overflow错误
                        // ARM_SPI_DATA_BITS(8), 1000000);     // 
                        // ARM_SPI_DATA_BITS(8), 500000);     // 
    GPIO_PinConfigure(GPIOA,4,GPIO_OUT_PUSH_PULL,GPIO_MODE_OUT10MHZ);
    GPIO_PinWrite(GPIOA,4,1);
    // Driver_SPI1.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
    // Driver_SPI1.Send(&fill, 1);


// /*定义结构体，下面NSS配置时也要用到GPIO所以这里一起定义*/
// SPI_InitTypeDef  SPI_InitStructure;
// GPIO_InitTypeDef GPIO_InitStructure;

// /* 使能 SPI1 & GPIOA 时钟 */
//          RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1|RCC_APB2Periph_AFIO, ENABLE);

// /* Configure SPI1 pins: NSS, SCK, MISO and MOSI */
//          GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
//         GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//         GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//         GPIO_Init(GPIOA, &GPIO_InitStructure);

// //SPI1 NSS for   W25X16                   
//          GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
//          GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//          GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//          GPIO_Init(GPIOC, &GPIO_InitStructure);

// GPIO_SetBits(GPIOA, GPIO_Pin_4);

// //SPI1 configuration 
//           SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //SPI1设置为两线全双工
// SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                    //设置SPI1为主模式
// SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                  //SPI发送接收8位帧结构
// SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                //串行时钟在不操作时，时钟为高电平
// SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;                //第二个时钟沿开始采样数据
// SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                //NSS信号由软件（使用SSI位）管理
// SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //定义波特率预分频的值:波特率预分频值为8
// SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;    //数据传输从MSB位开始
// // SPI_InitStructure.SPI_CRCPolynomial = 7;    //CRC值计算的多项式
// SPI_Init(SPI1, &SPI_InitStructure);
// /* Enable SPI1  */
// SPI_Cmd(SPI1, ENABLE);   //使能SPI1外设
// SPI_I2S_SendData(SPI1, 0xFF);
}

// void spi_w_byte(uint8_t b)
// {
//     SPI_I2S_SendData(SPI1, b);
// }

// void DMA_Config(void)
// {
//     DMA_InitTypeDef DMA_InitStructure;

//     /*开启时钟*/
//     RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);


//     DMA_DeInit(DMA1_Channel2);
//     DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI1->DR;
//     DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Rx_Buffer;
//     DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
//     DMA_InitStructure.DMA_BufferSize = 4096;
//     DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//     DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//     DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//     DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//     DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//     DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
//     DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//     DMA_Init(DMA1_Channel2, &DMA_InitStructure);

    
//     DMA_DeInit(DMA1_Channel3);
//     DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI1->DR;
//     DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Tx_Buffer;
//     DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
//     DMA_InitStructure.DMA_BufferSize = 4096;
//     DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//     DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//     DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//     DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//     DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//     DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
//     DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//     DMA_Init(DMA1_Channel3, &DMA_InitStructure);

//     DMA_Cmd (DMA1_Channel2,ENABLE);
//     DMA_Cmd (DMA1_Channel3,ENABLE);

// }

// void SPI_DMA_PageWrite(u32 WriteAddr)
// {
//     /* Enable the write access to the FLASH */
//     SPI_FLASH_WriteEnable();
//     /* Select the FLASH: Chip Select low */
//     SPI_FLASH_CS_LOW();
//     /* Send "Write to Memory " instruction */
//     SPI_FLASH_SendByte(W25X_PageProgram);
//     /* Send WriteAddr high nibble address byte to write to */
//     SPI_FLASH_SendByte((WriteAddr & 0xFF0000) >> 16);
//     /* Send WriteAddr medium nibble address byte to write to */
//     SPI_FLASH_SendByte((WriteAddr & 0xFF00) >> 8);
//     /* Send WriteAddr low nibble address byte to write to */
//     SPI_FLASH_SendByte(WriteAddr & 0xFF);


//       SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);

//       while(DMA_GetFlagStatus(DMA1_FLAG_TC3) == RESET);

//     /* Deselect the FLASH: Chip Select high */
//     SPI_FLASH_CS_HIGH();

// }

// void SPI_DMA_BufferRead(u32 ReadAddr)
// {
//     /* Select the FLASH: Chip Select low */
//     SPI_FLASH_CS_LOW();


//     /* Send "Read from Memory " instruction */
//     SPI_FLASH_SendByte(W25X_ReadData);


//     /* Send ReadAddr high nibble address byte to read from */
//     SPI_FLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
//     /* Send ReadAddr medium nibble address byte to read from */
//     SPI_FLASH_SendByte((ReadAddr& 0xFF00) >> 8);
//     /* Send ReadAddr low nibble address byte to read from */
//     SPI_FLASH_SendByte(ReadAddr & 0xFF);


//     SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_RxOnly;
//     SPI_Init(SPI1, &SPI_InitStructure);


//     //SPI_FLASH_SendByte(0xff);
//     SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);

//     while(DMA_GetFlagStatus(DMA1_FLAG_TC2) == RESET);

//     /* Deselect the FLASH: Chip Select high */
//     SPI_FLASH_CS_HIGH();
// }

// void SPI1_DMA_Configuration( void )
// {
 
//     RCC->AHBENR |= 1<<0 ;                     //DMA1时钟使能
 
//     /*------------------配置SPI1_RX_DMA通道Channel2---------------------*/
 
//     DMA1_Channel2->CCR &= ~( 1<<14 ) ;        //非存储器到存储器模式
//     DMA1_Channel2->CCR |=    2<<12   ;        //通道优先级高
//     DMA1_Channel2->CCR &= ~( 3<<10 ) ;        //存储器数据宽度8bit
//     DMA1_Channel2->CCR &= ~( 3<<8  ) ;        //外设数据宽度8bit
//     DMA1_Channel2->CCR |=    1<<7    ;        //存储器地址增量模式
//     DMA1_Channel2->CCR &= ~( 1<<6  ) ;        //不执行外设地址增量模式
//     DMA1_Channel2->CCR &= ~( 1<<5  ) ;        //执行循环操作
//     DMA1_Channel2->CCR &= ~( 1<<4  ) ;        //从外设读
 
//     DMA1_Channel2->CNDTR &= 0x0000   ;        //传输数量寄存器清零
//     DMA1_Channel2->CNDTR = buffersize ;       //传输数量设置为buffersize个
 
//     DMA1_Channel2->CPAR = SPI1_DR_Addr ;      //设置外设地址，注意PSIZE
//     DMA1_Channel2->CMAR = (u32)SPI1_RX_Buff ; //设置DMA存储器地址，注意MSIZE
 
//     /*------------------配置SPI1_TX_DMA通道Channel3---------------------*/
 
//     DMA1_Channel3->CCR &= ~( 1<<14 ) ;        //非存储器到存储器模式
//     DMA1_Channel3->CCR |=    0<<12   ;        //通道优先级最低
//     DMA1_Channel3->CCR &= ~( 3<<10 ) ;        //存储器数据宽度8bit
//     DMA1_Channel3->CCR &= ~( 3<<8 )  ;        //外设数据宽度8bit
//     DMA1_Channel3->CCR |=    1<<7    ;        //存储器地址增量模式
//     DMA1_Channel3->CCR &= ~( 1<<6 )  ;        //不执行外设地址增量模式
//     DMA1_Channel3->CCR &= ~( 1<<5 ) ;         //不执行循环操作
//     DMA1_Channel3->CCR |=    1<<4    ;        //从存储器读
 
//     DMA1_Channel3->CNDTR &= 0x0000   ;        //传输数量寄存器清零
//     DMA1_Channel3->CNDTR = buffersize ;       //传输数量设置为buffersize个
    
//     DMA1_Channel3->CPAR = SPI1_DR_Addr ;      //设置外设地址，注意PSIZE
//     DMA1_Channel3->CMAR = (uint32_t)SPI1_TX_Buff ; //设置DMA存储器地址，注意MSIZE              
// }
// PWR | CS | MI | CLK | MO |
void spi_uninit(void)
{
    spi_inited = 0;
    // GPIO_PinWrite(RED_LED,1);
    // GPIO_PinConfigure(RED_LED,GPIO_OUT_PUSH_PULL,GPIO_MODE_OUT2MHZ);
    Driver_SPI1.PowerControl(ARM_POWER_OFF);
    Driver_SPI1.Control(ARM_SPI_MODE_INACTIVE, 500000);
	Driver_SPI1.Uninitialize();
    GPIO_PinConfigure(GPIOA,4,GPIO_OUT_PUSH_PULL,GPIO_MODE_OUT10MHZ);
    GPIO_PinWrite(GPIOA,4,1);
    // Delayms(1);
}

ARM_SPI_STATUS status;
extern bool volatile program_wait;
void spi_write(uint8_t *buffer, uint16_t length)
{
	// Driver_SPI1.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
    while(1){
        GPIO_PinConfigure(GPIOA,4,GPIO_OUT_PUSH_PULL,GPIO_MODE_OUT10MHZ);
        GPIO_PinWrite(GPIOA,4,0);
    	// Delayms(1);
        Driver_SPI1.Send(buffer, length);
        status = Driver_SPI1.GetStatus();
        while(!status.busy){
            __nop();
            status = Driver_SPI1.GetStatus();
        }
        while(status.busy){
            __nop();
            status = Driver_SPI1.GetStatus();
            if(status.data_lost){
                spi_uninit();
                spi_init();
                continue;
            }
        }
        GPIO_PinWrite(GPIOA,4,1);
        // program_wait = false;
        break;
    }
}

void spi_wr(uint8_t *buffer, uint8_t *rev, uint16_t length)
{
    while(1){
    	// Driver_SPI1.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
        GPIO_PinConfigure(GPIOA,4,GPIO_OUT_PUSH_PULL,GPIO_MODE_OUT10MHZ);
        GPIO_PinWrite(GPIOA,4,0);
    	// Delayms(1);
        if(rev!=NULL){
            Driver_SPI1.Transfer(buffer, rev, length);
            status = Driver_SPI1.GetStatus();
            while(!status.busy){
                __nop();
                status = Driver_SPI1.GetStatus();
            }
            while(status.busy){
                __nop();
                status = Driver_SPI1.GetStatus();
                if(status.data_lost){
                    spi_uninit();
                    spi_init();
                    continue;
                }
            }
            // program_wait = false;
        }else{
            Driver_SPI1.Send(buffer, length);
            status = Driver_SPI1.GetStatus();
            while(!status.busy){
                __nop();
                status = Driver_SPI1.GetStatus();
            }
            while(status.busy){
                __nop();
                status = Driver_SPI1.GetStatus();
                if(status.data_lost){
                    spi_uninit();
                    spi_init();
                    continue;
                }
            }
            // program_wait = false;
        }
        GPIO_PinWrite(GPIOA,4,1);
        break;
    }
}
