
#include "usart_driver.h"
#include "Driver_USART.h"

__weak void USART1_Callback(uint32_t event) { ; }
__weak void USART2_Callback(uint32_t event) { ; }

extern ARM_DRIVER_USART Driver_USART1;
extern ARM_DRIVER_USART Driver_USART2;

void usart_init(void)
{
    Driver_USART1.Initialize(USART1_Callback);
    Driver_USART1.PowerControl(ARM_POWER_FULL);
    Driver_USART1.Control(ARM_USART_MODE_ASYNCHRONOUS |
                          ARM_USART_DATA_BITS_8 |
                          ARM_USART_PARITY_NONE |
                          ARM_USART_STOP_BITS_1 |
                          ARM_USART_FLOW_CONTROL_NONE, 115200);
    Driver_USART1.Control(ARM_USART_CONTROL_TX, 1);
    Driver_USART1.Control(ARM_USART_CONTROL_RX, 1);
    // Driver_USART1.Receive(rxBuffer, sizeof(rxBuffer));
    // Driver_USART1.Send(test, sizeof(test));

    Driver_USART2.Initialize(USART2_Callback);
    Driver_USART2.PowerControl(ARM_POWER_FULL);
    Driver_USART2.Control(ARM_USART_MODE_ASYNCHRONOUS |
                          ARM_USART_DATA_BITS_8 |
                          ARM_USART_PARITY_NONE |
                          ARM_USART_STOP_BITS_1 |
                          ARM_USART_FLOW_CONTROL_NONE, 115200);
    Driver_USART2.Control(ARM_USART_CONTROL_TX, 1);
    Driver_USART2.Control(ARM_USART_CONTROL_RX, 1);
    // Driver_USART2.Receive(rxBuffer, sizeof(rxBuffer));
    // Driver_USART2.Send(test, sizeof(test));
}
