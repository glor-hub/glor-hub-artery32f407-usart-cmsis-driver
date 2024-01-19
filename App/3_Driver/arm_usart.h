#ifndef _ARM_USART_H_
#define _ARM_USART_H_

#include "at32f403a_407.h"
#include <stdbool.h>
#include "ringbuffer.h"
#include "arm_dma.h"
#include "arm_gpio.h"

//select UARTx (USARTx (x=1,2,3,6) are not supported in this driver version)
//use DMA if necessary

/*******************************************
UART4
********************************************/
#define _TEST_APP_UART4_ENABLE_
// #define _TEST_APP_UART4_TX_USE_DMA_
// #define _TEST_APP_UART4_RX_USE_DMA_

/*******************************************
UART5
********************************************/
// #define _TEST_APP_UART5_ENABLE_
// #define _TEST_APP_UART5_TX_USE_DMA_
// #define _TEST_APP_UART5_RX_USE_DMA_

/*******************************************
UART7
********************************************/
// #define _TEST_APP_UART7_ENABLE_
// #define _TEST_APP_UART7_TX_USE_DMA_
// #define _TEST_APP_UART7_RX_USE_DMA_

/*******************************************
UART8
********************************************/
#define _TEST_APP_UART8_ENABLE_
// #define _TEST_APP_UART8_TX_USE_DMA_
// #define _TEST_APP_UART8_RX_USE_DMA_

/*******************************************

********************************************/

typedef enum {
    TEST_APP_ARM_USART1 = 0,
    TEST_APP_ARM_USART2,
    TEST_APP_ARM_USART3,
    TEST_APP_ARM_UART4,
    TEST_APP_ARM_UART5,
    TEST_APP_ARM_USART6,
    TEST_APP_ARM_UART7,
    TEST_APP_ARM_UART8,
    TEST_APP_ARM_USART_TYPES
} eTEST_APP_ARM_USART_Types_t;

// USART Driver state
#define TEST_APP_ARM_USART_DRIVER_FLAG_INITIALIZED          (uint32_t)(1U << 0)
#define TEST_APP_ARM_USART_DRIVER_FLAG_CONFIGURATED         (uint32_t)(1U << 1)
#define TEST_APP_ARM_USART_DRIVER_FLAG_TX_ENABLED           (uint32_t)(1U << 2)
#define TEST_APP_ARM_USART_DRIVER_FLAG_RX_ENABLED           (uint32_t)(1U << 3)
#define TEST_APP_ARM_USART_DRIVER_FLAG_DMA_TX_ENABLED       (uint32_t)(1U << 4)
#define TEST_APP_ARM_USART_DRIVER_FLAG_DMA_RX_ENABLED       (uint32_t)(1U << 5)

//USART default/remap pin definitions
#define TEST_APP_ARM_USART_GPIO_PIN_DEF_DEFAULT     (uint32_t)0x00
#define TEST_APP_ARM_UART4_GPIO_PIN_DEF_REMAP1      UART4_GMUX_0010
#define TEST_APP_ARM_UART5_GPIO_PIN_DEF_REMAP1      UART5_GMUX_0001
#define TEST_APP_ARM_UART7_GPIO_PIN_DEF_REMAP1      UART7_GMUX
#define TEST_APP_ARM_UART8_GPIO_PIN_DEF_REMAP1      UART8_GMUX

//USART Baudrate
#define TEST_APP_ARM_USART_BAUDRATE_9600    ((uint32_t)9600)
#define TEST_APP_ARM_USART_BAUDRATE_19200   ((uint32_t)19200)
#define TEST_APP_ARM_USART_BAUDRATE_38400   ((uint32_t)38400)
#define TEST_APP_ARM_USART_BAUDRATE_57600   ((uint32_t)57600)
#define TEST_APP_ARM_USART_BAUDRATE_115200  ((uint32_t)115200)

//USART Event

#define TEST_APP_ARM_USART_EVENT_RX_COMPLETE         ((uint32_t)1UL << 0)  //Receive completed
#define TEST_APP_ARM_USART_EVENT_TX_COMPLETE         ((uint32_t)1UL << 1)  //Transmit completed (optional)
#define TEST_APP_ARM_USART_EVENT_TX_UNDERFLOW        ((uint32_t)1UL << 2)  //Transmit data not available (Synchronous Slave)
#define TEST_APP_ARM_USART_EVENT_RX_OVERFLOW         ((uint32_t)1UL << 3)  //Receive data overflow
#define TEST_APP_ARM_USART_EVENT_RX_TIMEOUT          ((uint32_t)1UL << 4)  //Receive character timeout (optional)
#define TEST_APP_ARM_USART_EVENT_RX_BREAK            ((uint32_t)1UL << 5)  //Break detected on receive
#define TEST_APP_ARM_USART_EVENT_RX_FRAMING_ERROR    ((uint32_t)1UL << 6)  //Framing error detected on receive
#define TEST_APP_ARM_USART_EVENT_RX_NOISE_ERROR      ((uint32_t)1UL << 7)  //Noise error detected on receive
#define TEST_APP_ARM_USART_EVENT_RX_PARITY_ERROR     ((uint32_t)1UL << 8)  //Parity error detected on receive

typedef struct {
    uint32_t                    BaudRate;
    usart_data_bit_num_type     DataBit;
    usart_stop_bit_num_type     StopBit;
    usart_parity_selection_type Parity;
} TEST_APP_ARM_USART_Config_t;

typedef struct {
    volatile uint8_t TxBusy;
    volatile uint8_t RxBusy;
    uint8_t TxUnderflow;
    uint8_t RxOverflow;
    uint8_t RxBreak;
    uint8_t RxFramingError;
    uint8_t RxNoiseError;
    uint8_t RxParityError;
} volatile TEST_APP_ARM_USART_XferStatus_t;

typedef struct {
    confirm_state DrvStateOn;
    uint32_t DrvStatus;
    uint32_t DrvFlag;
    TEST_APP_ARM_USART_XferStatus_t XferStatus;
} TEST_APP_ARM_USART_Status_t;

typedef struct {
    void                    *pTxData;
    void                    *pRxData;
    uint32_t                TxNum;
    uint32_t                RxNum;
    volatile uint32_t       TxCnt;         // Number of data received
    volatile uint32_t       RxCnt;         // Number of data sent
} TEST_APP_ARM_USART_Transfer_t;

typedef struct {
    eTEST_APP_ARM_GPIO_Ports_t  TxPort;
    uint32_t                    TxPin;
    eTEST_APP_ARM_GPIO_Ports_t  RxPort;
    uint32_t                    RxPin;
} TEST_APP_ARM_USART_GPIO_t;

typedef struct {
    eTEST_APP_ARM_DMA_Chan_t    TxChan;
    eTEST_APP_ARM_DMA_Chan_t    RxChan;
    confirm_state               TxEnable;
    confirm_state               RxEnable;
    void (*TxEvent_cb)(uint32_t event);
    void (*RxEvent_cb)(uint32_t event);
    confirm_state               TxFlexModeEnable;
    confirm_state               RxFlexModeEnable;
    dma_flexible_request_type   TxFlexPeriphReq;
    dma_flexible_request_type   RxFlexPeriphReq;
} TEST_APP_ARM_USART_DMA_t;

typedef struct {
    eTEST_APP_ARM_USART_Types_t     usart_type;
    IRQn_Type                       IrqNum;     // USART IRQ Number
    uint32_t                        GpioPinDef;
    TEST_APP_RingBuffer_t           Event;
    TEST_APP_ARM_USART_Config_t     Config;
    TEST_APP_ARM_USART_GPIO_t       Gpio;
    TEST_APP_ARM_USART_Transfer_t   Transfer;
    TEST_APP_ARM_USART_Status_t     Status;
    TEST_APP_ARM_USART_DMA_t        DMA;
} TEST_APP_ARM_USART_Resources_t;

typedef struct {
    uint32_t (*Initialize)(uint32_t baudrate,
                           usart_data_bit_num_type dataBit,
                           usart_stop_bit_num_type stopBit,
                           usart_parity_selection_type parity,
                           uint32_t gpio_pin_def);
    uint32_t (*Uninitialize)(void);
    void (*Event_cb)(void);
    uint32_t (*Send)(void *pdata, uint32_t num);
    uint32_t (*Recieve)(void *pdata, uint32_t num);
    TEST_APP_ARM_USART_Transfer_t (*GetTransfer)(void);
    TEST_APP_ARM_USART_Status_t (*GetStatus)(void);
} TEST_APP_ARM_USART_Driver_t;

void TEST_APP_ARM_USART_StartUp(void);
void TEST_APP_ARM_USART_IRQHandler(eTEST_APP_ARM_USART_Types_t usart_type);

#endif //_ARM_USART_H_ 
