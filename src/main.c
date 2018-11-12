#include <stdio.h>
#include "stm32f10x.h"
#include "SWD_host.h"
#include "SWD_flash.h"

#include "debug_cm.h"
#include "DAP_config.h"
// #include "Driver_USART.h"
// #include "Driver_SPI.h"
/*SWD引脚定义在DAP_config.h文件中*/

#define sector_size (4096)

// uint32_t Flash_Page_Size = 1024;
// uint32_t Flash_Start_Addr = 0x08000000;
uint32_t Flash_Page_Size = sector_size;
uint32_t Flash_Start_Addr = 0x00000000;

uint8_t buff[sector_size+0x10] = {0};

#include "spi_driver.h"
#include "usart_driver.h"
#include "scheduler.h"

extern ARM_DRIVER_USART Driver_USART1;
extern ARM_DRIVER_USART Driver_USART2;

static uint8_t rxBuffer[sector_size+0x10] = {0};
static uint8_t txBuffer[0x100] = {0};

bool volatile program_state = false;
bool volatile program_wait = false;
void program_from_flash(void);


#define code_bin beep

void power_manage(void)
{
	// __WFI();
}
void platform_scheduler(void)
{
	app_sched_execute();
	power_manage();
}
void timer_config(void)
{
	// 定时 TIMER 设定
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseInitStrue;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_DeInit(TIM3);
	TIM_TimeBaseInitStrue.TIM_Period = 9999;			        //设置自动重装载值
	TIM_TimeBaseInitStrue.TIM_Prescaler = 71;		            //预分频系数 - 1000k
	TIM_TimeBaseInitStrue.TIM_CounterMode = TIM_CounterMode_Up;	//计数器向上溢出
	TIM_TimeBaseInitStrue.TIM_ClockDivision = TIM_CKD_DIV1;		//时钟的分频
	TIM_TimeBaseInitStrue.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStrue);				//TIM初始化设置
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM3, ENABLE);	//使能TIM
}
void rtc_config(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	RCC_HSEConfig(RCC_HSE_ON);
	while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET) {}
	RCC_RTCCLKConfig(RCC_RTCCLKSource_HSE_Div128);
	RCC_RTCCLKCmd(ENABLE);

	RTC_EnterConfigMode();
	RTC_WaitForSynchro();
	RTC_WaitForLastTask();
	RTC_ITConfig(RTC_IT_SEC, ENABLE);
	RTC_WaitForLastTask();
	RTC_SetPrescaler(32767);
	RTC_WaitForLastTask();
	RTC_ExitConfigMode();
}
void NVIC_config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	/* RTC中断 */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* TIM3 中断 */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
#include "GPIO_STM32F10x.h"
void gpio_config(void)
{
    GPIO_PinConfigure(GPIOB,12,GPIO_IN_PULL_UP,GPIO_MODE_INPUT);
    GPIO_PinConfigure(GPIOB,13,GPIO_OUT_PUSH_PULL,GPIO_MODE_OUT2MHZ);
    GPIO_PinConfigure(GPIOB,14,GPIO_OUT_PUSH_PULL,GPIO_MODE_OUT2MHZ);
    GPIO_PinConfigure(GPIOB,15,GPIO_OUT_PUSH_PULL,GPIO_MODE_OUT2MHZ);
    
    GPIO_PinWrite(GPIOB,13,1);
    GPIO_PinWrite(GPIOB,14,1);
    GPIO_PinWrite(GPIOB,15,1);
}

void rtc_2hz_routine(void){
    static bool tic=false;
    static uint8_t key_d=0;
    if(key_d>0) key_d-=1;
    tic=!tic;
    // if(tic){
    //     GPIO_PinWrite(GPIOB,13,1);
    //     GPIO_PinWrite(GPIOB,14,1);
    // }else{
    //     GPIO_PinWrite(GPIOB,13,0);
    //     GPIO_PinWrite(GPIOB,14,0);
    // }
    // GPIO_PinWrite(GPIOB,15,GPIO_PinRead(GPIOB,12));
    if(GPIO_PinRead(GPIOB,12)==0 && key_d == 0 && program_state==false){
        key_d = 4;
        platform_simple_evt_put(program_from_flash);
    }
}
void RTC_IRQHandler(void)
{
	platform_simple_evt_put(rtc_2hz_routine);
	RTC_ClearFlag(RTC_FLAG_SEC);
}

extern void timer_10ms_routine(void);
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
		platform_simple_evt_put(timer_10ms_routine);
	}
	TIM_ClearITPendingBit(TIM3, TIM_FLAG_Update);
}
void timer_10ms_routine(void){

}
uint16_t up_flag=0;
bool spi_flag=false;
uint16_t spi_length;
int main(void)
{
    uint32_t addr;
    SystemInit();
    APP_SCHED_INIT(16, 32);
    gpio_config();
    timer_config();
    rtc_config();
    NVIC_config();

    spi_init();
    usart_init();
    Driver_USART1.Receive(rxBuffer, sizeof(rxBuffer));
    // Driver_USART2.Receive(rxBuffer, sizeof(rxBuffer));

    // swd_init_debug();

    // flash_is_busy();
    while(1)
    {
        platform_scheduler();
    }
}

typedef struct
{
    uint8_t *data;
} sUART_EVT;

void echo(sUART_EVT* ue, uint16_t size){
    uint16_t length = (ue->data[2]<<8) | ue->data[3];
    Driver_USART1.Send(ue->data+4, length);
}
void flash_init(void){
    static uint8_t _[4]={0x5e,0x7f,0x01,0x01};
    Driver_USART1.Send(_, 4);
    flash_wait_idle_q();
    flash_chip_erase_q();
    flash_wait_idle_q();
}
void flash_write_block(sUART_EVT* ue, uint16_t size){
    uint8_t i;
    uint32_t sn = ue->data[0];
    uint32_t addr = (ue->data[1]<<16) | (ue->data[2]<<8) | ue->data[3];
    flash_wait_idle_q();
    flash_sector_erase_q(sn*0x1000);
    flash_wait_idle_q();
    flash_write_q(ue->data,0x1F0000+sn*4,0x4);
    flash_wait_idle_q();
    for(i=0;i<16;i++){
        flash_write_q(ue->data+4+i*0x100,sn*0x1000+i*0x100,0x100);
        flash_wait_idle_q();
    }
}


struct sbin_head
{
    uint8_t sn;
    uint8_t addr[3];
}bin_head;

void swd_start(void)
{
    GPIO_PinWrite(GPIOB,14,1);
    GPIO_PinWrite(GPIOB,15,1);
    swd_init();
}

void swd_error(void)
{
    GPIO_PinWrite(GPIOB,15,0);

    GPIO_PinWrite(GPIOB,13,0);
    delaymS(40);
    GPIO_PinWrite(GPIOB,13,1);
    delaymS(40);
    GPIO_PinWrite(GPIOB,13,0);
    delaymS(40);
    GPIO_PinWrite(GPIOB,13,1);
    delaymS(40);
    GPIO_PinWrite(GPIOB,13,0);
    delaymS(40);
    GPIO_PinWrite(GPIOB,13,1);
}

void swd_success(void)
{
    GPIO_PinWrite(GPIOB,14,0);
    GPIO_PinWrite(GPIOB,13,0);
    delaymS(300);
    GPIO_PinWrite(GPIOB,13,1);
}

static uint8_t _buf1[0x200] __attribute__((section(".ARM.__at_0x20000020")));
static uint8_t _buf2[0x10];
uint8_t flash_WE = 0x06;

#define sn_block_size 0x10
#define flash_simple_write(a,b,l) program_wait = true;spi_wr(a,b,l);while(program_wait)
#define flash_write_enable() flash_simple_write(&flash_WE,NULL,1)
#define flash_wait_idle_simple() _buf1[0x104] = 0x05;\
    _buf1[0x105] = 0xFF;\
    do{\
        _buf2[1] = 0x00;\
        flash_simple_write(_buf1+0x104,_buf2,2);\
    }while(_buf2[1]&0x1)
void flash_write_block_simple(sUART_EVT* ue, uint16_t size){
    uint8_t i;
    uint16_t j;
    uint32_t sn = ue->data[0];
    uint32_t addr = (ue->data[1]<<16) | (ue->data[2]<<8) | ue->data[3];
    program_state = true;

    flash_wait_idle_simple();
	_buf1[0] = 0x20;
	_buf1[1] = ((sn*0x1000) >> 16) & 0xFF;
	_buf1[2] = ((sn*0x1000) >> 8) & 0xFF;
	_buf1[3] = (sn*0x1000) & 0xFF;
    flash_write_enable();
    flash_simple_write(_buf1,NULL,4);
    flash_wait_idle_simple();

    // 写地址+校验和
	_buf1[0] = 0x02;
	_buf1[1] = ((0x1F0000+sn*sn_block_size) >> 16) & 0xFF;
	_buf1[2] = ((0x1F0000+sn*sn_block_size) >> 8) & 0xFF;
	_buf1[3] = (0x1F0000+sn*sn_block_size) & 0xFF;
    memcpy(_buf1+4, ue->data, 4);
    memcpy(_buf1+8, ue->data+0x1004, 4);
    flash_write_enable();
    flash_simple_write(_buf1,NULL,12);
    flash_wait_idle_simple();

    for(i=0;i<16;i++){
        _buf1[0] = 0x02;
        _buf1[1] = ((sn*0x1000+i*0x100) >> 16) & 0xFF;
        _buf1[2] = ((sn*0x1000+i*0x100) >> 8) & 0xFF;
        _buf1[3] = (sn*0x1000+i*0x100) & 0xFF;
        memcpy(_buf1+4, ue->data+4+i*0x100, 0x100);
        flash_write_enable();
        flash_simple_write(_buf1,NULL,0x104);
        flash_wait_idle_simple();
    }

    // 读取校验和
    _buf1[0] = 0x03;
    _buf1[1] = ((0x1F0000+sn*sn_block_size+4) >> 16) & 0xFF;
    _buf1[2] = ((0x1F0000+sn*sn_block_size+4) >> 8) & 0xFF;
    _buf1[3] = (0x1F0000+sn*sn_block_size+4) & 0xFF;
    flash_simple_write(_buf1,_buf2,8);

    // 读取内容
    _buf1[0] = 0x03;
    _buf1[1] = ((sn*0x1000) >> 16) & 0xFF;
    _buf1[2] = ((sn*0x1000) >> 8) & 0xFF;
    _buf1[3] = (sn*0x1000) & 0xFF;
    flash_simple_write(_buf1,buff,0x1004);
    // 计算校验和
    uint8_t checksum[4]={0x00,0xFF,0xAA,0x55};
    uint8_t offset=0;
    for(j=0;j<0x1000;j++){
        if(buff[j+4]!=0xFF){
            offset+=buff[j+4];
            checksum[offset&0x03]^=buff[j+4];
        }
    }

    if((checksum[0]!=_buf2[4])
    ||(checksum[1]!=_buf2[5])
    ||(checksum[2]!=_buf2[6])
    ||(checksum[3]!=_buf2[7])){
        // swd_error();
        _buf2[0]=0x5e;
        _buf2[1]=0x75;
        _buf2[2]=0x00;
        _buf2[3]=0xFF;
        Driver_USART1.Send(_buf2, 8);
    }else{
        _buf1[0]=0x5e;
        _buf1[1]=0x75;
        _buf1[2]=sn;
        _buf1[3]=0x01;
        Driver_USART1.Send(_buf1, 4);
    }
    program_state = false;
}

void flash_read_back_simple(sUART_EVT* ue, uint8_t l){
    uint32_t addr = (ue->data[4]<<16) | (ue->data[5]<<8) | ue->data[6];
    uint16_t length = (ue->data[2]<<8) | ue->data[3];
    program_state = true;
    
	_buf1[0] = 0x03;
	_buf1[1] = (addr >> 16) & 0xFF;
	_buf1[2] = (addr >> 8) & 0xFF;
	_buf1[3] = (addr) & 0xFF;
    flash_simple_write(_buf1,_buf1+0x4,length+4);
    flash_wait_idle_simple();
    Driver_USART1.Send(_buf1+0x8, length);
    
    program_state = false;
}

void flash_read_id(void){
    static uint8_t _buf[0x20];
    program_state = true;
	_buf1[0] = 0x4b;
	_buf1[1] = 0xFF;
	_buf1[2] = 0xFF;
	_buf1[3] = 0xFF;
    flash_simple_write(_buf1,_buf1+0x4,20);
    Driver_USART1.Send(_buf1+0x4, 16);
    program_state = false;
}

void swd_set_nrf52_reset(void)
{
    uint32_t ap_index_return;

    swd_init_debug();

    swd_read_ap(0x010000FC, &ap_index_return);
    if (ap_index_return == 0x02880000) {
        // Have CTRL-AP
        swd_write_ap(0x01000000, 1);  // CTRL-AP reset hold
    }
    else {
        // No CTRL-AP - Perform a soft reset
        // 0x05FA0000 = VECTKEY, 0x4 = SYSRESETREQ
        uint32_t swd_mem_write_data = 0x05FA0000 | 0x4;
        swd_write_memory(0xE000ED0C, (uint8_t *) &swd_mem_write_data, 4);
    }
    delaymS(1);
    swd_read_ap(0x010000FC, &ap_index_return);
    if (ap_index_return == 0x02880000) {
        // Device has CTRL-AP
        swd_write_ap(0x01000000, 0);  // CTRL-AP reset release
    }
    else {
        // No CTRL-AP - Soft reset has been performed
    }
}
#define swd_exit() program_state = false; swd_off()
void program_from_flash(void){
    // static uint8_t _buf2[0x10];
    uint32_t _addr_head=0x1F0000;
    uint32_t _addr_bin;
    uint32_t _addr_prog;
    program_state = true;
    swd_start();

    if(target_flash_init(Flash_Start_Addr)!=ERROR_SUCCESS){
        swd_exit();
        swd_error();
        return;
    }
    if(target_flash_erase_chip()!=ERROR_SUCCESS){
        swd_exit();
        swd_error();
        return;
    }
    while(1){
        _buf1[0] = 0x03;
        _buf1[1] = (_addr_head >> 16) & 0xFF;
        _buf1[2] = (_addr_head >> 8) & 0xFF;
        _buf1[3] = _addr_head & 0xFF;
        program_wait = true;
        spi_wr(_buf1,_buf2,8);
        while(program_wait);
        if(_buf2[5]!=0xFF){
            Driver_USART1.Send(_buf2+4, 4);
            _addr_bin = _buf2[4]*0x1000;
            _addr_prog = (_buf2[5]<<16) | (_buf2[6]<<8) | _buf2[7];
            _buf1[0] = 0x03;
            _buf1[1] = (_addr_bin >> 16) & 0xFF;
            _buf1[2] = (_addr_bin >> 8) & 0xFF;
            _buf1[3] = _addr_bin & 0xFF;
            program_wait = true;
            spi_wr(_buf1,buff,0x1000+4);
            while(program_wait);
            if(target_flash_program_page(Flash_Start_Addr + _addr_prog, &buff[4], sector_size)!=ERROR_SUCCESS){
                swd_exit();
                swd_error();
                return;
            }
        }else{
            uint8_t sn=0;
            uint32_t addr;
            while(1){
                // 读取校验和
                _buf1[0] = 0x03;
                _buf1[1] = ((0x1F0000+sn*sn_block_size) >> 16) & 0xFF;
                _buf1[2] = ((0x1F0000+sn*sn_block_size) >> 8) & 0xFF;
                _buf1[3] = (0x1F0000+sn*sn_block_size) & 0xFF;
                flash_simple_write(_buf1,_buf2,12);
                addr = (_buf2[5]<<16)|(_buf2[6]<<8)|_buf2[7];
                if(_buf2[4]==0xFF) break;
                // 读取chip内容
                swd_read_memory(Flash_Start_Addr + addr, buff, sector_size);

                // 计算校验和
                uint8_t checksum[4]={0x00,0xFF,0xAA,0x55};
                uint8_t offset=0;
                for(uint16_t j=0;j<0x1000;j++){
                    if(buff[j]!=0xFF){
                        offset+=buff[j];
                        checksum[offset&0x03]^=buff[j];
                    }
                }
                Driver_USART1.Send(checksum, 4);
                if((checksum[0]!=_buf2[8])
                ||(checksum[1]!=_buf2[9])
                ||(checksum[2]!=_buf2[10])
                ||(checksum[3]!=_buf2[11])){
                    swd_error();
                    swd_exit();
                    return;
                }
                sn+=1;
            }
            swd_success();
            // swd_set_target_state_hw(RESET_RUN);
            // swd_set_nrf52_reset();
			// delaymS(1);
            swd_set_target_state_hw(RUN);
            break;
        }
        _addr_head+=sn_block_size;
    }
    swd_exit();
}
// head: 0x5e
// cmd: 0x00-0x7F, 0x8000-0xFFFF
// length: 0x0000-0xFFFF
// payload: xxxx
// CRC: xx

// init:0x5E,0x7F,0x00,0x00
// write datablock: 0x5E,0x75,0x10,0x04, 8bit sn, 24bit addr, 4k data, crc
// test program: 0x5E,0x71,0x00,0x00, crc
void uart_parser(uint16_t max)
{
    sUART_EVT uart_evt;
    uint16_t i=0;
    while(i<max){
        if(rxBuffer[i]==0x5E){
            switch (rxBuffer[i+1])
            {
                case 0x7F:
				flash_init();
                return;
                case 0x76:
                platform_simple_evt_put(flash_read_id);
                break;
                case 0x75:
                uart_evt.data = rxBuffer+4+i;
                // app_sched_event_put(&uart_evt,sizeof(uart_evt),(app_sched_event_handler_t)flash_write_block);
                app_sched_event_put(&uart_evt,sizeof(uart_evt),(app_sched_event_handler_t)flash_write_block_simple);
                return;
                case 0x74:
                uart_evt.data = rxBuffer+i;
                app_sched_event_put(&uart_evt,sizeof(uart_evt),(app_sched_event_handler_t)flash_read_back_simple);
                return;
                case 0x71:
                platform_simple_evt_put(program_from_flash);
                return;
                case 0x77:  // echo
                uart_evt.data = rxBuffer+i;
                app_sched_event_put(&uart_evt,sizeof(uart_evt),(app_sched_event_handler_t)echo);
                return;
                // default:
                // return;
            }
        }
        i+=1;
    }
}

void USART1_Callback(uint32_t event)
{
    if(event & ARM_USART_EVENT_RX_TIMEOUT)
    {
        Driver_USART1.Control(ARM_USART_ABORT_RECEIVE, 1);
        uint32_t length = Driver_USART1.GetRxCount();
        // memcpy(txBuffer, rxBuffer, length);
        // Driver_USART1.Send(txBuffer, length);
        Driver_USART1.Receive(rxBuffer, sizeof(rxBuffer));
        uart_parser(length);
    }
}

void SPI1_Callback(uint32_t event)
{
    // GPIO_PinWrite(GPIOA,4,1);
    if(event & ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        if(!program_state){
            Driver_SPI1.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
            if(spi_queue[(sq_r+SPI_QUEUE_LENGTH-1)&(SPI_QUEUE_LENGTH-1)].callback!=NULL){
                if(spi_queue[(sq_r+SPI_QUEUE_LENGTH-1)&(SPI_QUEUE_LENGTH-1)].callback(spi_queue[(sq_r+SPI_QUEUE_LENGTH-1)&(SPI_QUEUE_LENGTH-1)].r)==true){
                    sq_r = (sq_r+SPI_QUEUE_LENGTH-1)&(SPI_QUEUE_LENGTH-1);
                }
            }
            platform_simple_evt_put(spi_queue_dispatch);
        }else{
            Driver_SPI1.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
            program_wait = false;
        }
    }
}

int fputc(int ch, FILE *f)
{
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, ch);
    return ch;
}


// void USART2_Callback(uint32_t event)
// {
//     if(event & ARM_USART_EVENT_RX_TIMEOUT)
//     {
//         Driver_USART2.Control(ARM_USART_ABORT_RECEIVE, 1);
//         uint32_t length = Driver_USART2.GetRxCount();
//         memcpy(txBuffer, rxBuffer, length);
//         Driver_USART2.Receive(rxBuffer, sizeof(rxBuffer));
//         Driver_USART1.Send(txBuffer, length);
//     }
// }
