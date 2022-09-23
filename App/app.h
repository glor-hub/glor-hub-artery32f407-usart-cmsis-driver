#ifndef _COMMON_H_
#define _COMMON_H_

#define TRUE 1
#define FALSE 0

#define PASSED 0
#define FAILED 1

#define HARDWARE_TESTING_MODE

typedef enum {
    ENABLE_CMD = 0,
    DISABLE_CMD,
    NUM_PERIPH_COMMANDS
} ePeriphCmd;

typedef enum {
    GPIO_PORT_A = 0,
    GPIO_PORT_B,
    GPIO_PORT_C,
    GPIO_PORT_D,
    GPIO_PORT_E,
    GPIO_PORT_F,
    GPIO_PORT_G,
    GPIO_PORT_H,
    GPIO_PORT_I,
    USART_1,
    USART_2,
    USART_3,
    UART_4,
    UART_5,
    USART_6,
    DMA_1,
    DMA_2
} ePeriphTypes;

#endif //_COMMON_H_
