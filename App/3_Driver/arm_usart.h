#ifndef _ARM_USART_H_
#define _ARM_USART_H_

#include "at32f403a_407.h"
#include <stdbool.h>
#include "ringbuffer.h"

//select UARTx
//use DMA if necessary
//use DMA circular buffer if necessary (default - linear buffer)

/*******************************************
UART4

For DMA:
DMA2 CHANNEL5 (default config) - Tx
DMA2_CHANNEL3 (default config) - Rx
********************************************/
#define _TEST_APP_UART4_PERIPH_ENABLE_
#define _TEST_APP_UART4_TX_USE_DMA_
// #define _TEST_APP_UART4_TX_DMA_CIRCULAR_MODE_
// #define _TEST_APP_UART4_RX_USE_DMA_
// #define _TEST_APP_UART4_RX_DMA_CIRCULAR_MODE_

/*******************************************
UART5

For DMA:
DMA1 CHANNEL5 (with flexible mapping) - Tx
DMA1_CHANNEL4 (with flexible mapping) - Rx
********************************************/
// #define _TEST_APP_UART5_PERIPH_ENABLE_
// #define _TEST_APP_UART5_TX_USE_DMA_
// #define _TEST_APP_UART5_TX_DMA_CIRCULAR_MODE_
// #define _TEST_APP_UART5_RX_USE_DMA_
// #define _TEST_APP_UART5_RX_DMA_CIRCULAR_MODE_

/*******************************************
UART7

For DMA:
DMA1 CHANNEL3 (with flexible mapping) - Tx
DMA1_CHANNEL2 (with flexible mapping) - Rx
********************************************/
// #define _TEST_APP_UART7_PERIPH_ENABLE_
// #define _TEST_APP_UART7_TX_USE_DMA_
// #define _TEST_APP_UART7_TX_DMA_CIRCULAR_MODE_
// #define _TEST_APP_UART7_RX_USE_DMA_
// #define _TEST_APP_UART7_RX_DMA_CIRCULAR_MODE_

/*******************************************
UART8

For DMA:
DMA2 CHANNEL6 (with flexible mapping) - Tx
DMA2_CHANNEL4 (with flexible mapping) - Rx
********************************************/
// #define _TEST_APP_UART8_PERIPH_ENABLE_
// #define _TEST_APP_UART8_TX_USE_DMA_
// #define _TEST_APP_UART8_TX_DMA_CIRCULAR_MODE_
// #define _TEST_APP_UART8_RX_USE_DMA_
// #define _TEST_APP_UART8_RX_DMA_CIRCULAR_MODE_

/*******************************************

********************************************/

#define TEST_APP_ARM_USART_TX_BUFF_SIZE 256
#define TEST_APP_ARM_USART_RX_BUFF_SIZE 256

#define TEST_APP_ARM_USART_EVENT_BUFF_SIZE 8

//USART Driver Status
#define TEST_APP_ARM_USART_DRIVER_NO_ERROR           ((uint32_t)0UL)
#define TEST_APP_ARM_USART_DRIVER_ERROR              ((uint32_t)1UL << 0)
#define TEST_APP_ARM_USART_DRIVER_ERROR_PARAMETER    ((uint32_t)1UL << 1)
#define TEST_APP_ARM_USART_DRIVER_ERROR_BUSY         ((uint32_t)1UL << 2)
#define TEST_APP_ARM_USART_DRIVER_ERROR_TIMEOUT      ((uint32_t)1UL << 3)
#define TEST_APP_ARM_USART_DRIVER_ERROR_UNSUPPORTED  ((uint32_t)1UL << 4)

// USART Driver flags
#define TEST_APP_ARM_USART_FLAG_INITIALIZED          (uint32_t)(1U << 0)
#define TEST_APP_ARM_USART_FLAG_CONFIGURATED         (uint32_t)(1U << 1)
#define TEST_APP_ARM_USART_FLAG_TX_ENABLED           (uint32_t)(1U << 2)
#define TEST_APP_ARM_USART_FLAG_RX_ENABLED           (uint32_t)(1U << 3)

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
    uint8_t TxBusy;
    uint8_t RxBusy;
    uint8_t TxUnderflow;
    uint8_t RxOverflow;
    uint8_t RxBreak;
    uint8_t RxFramingError;
    uint8_t RxNoiseError;
    uint8_t RxParityError;
} TEST_APP_ARM_USART_XferStatus_t;

typedef struct {
    uint32_t DrvStatus;
    uint32_t DrvFlag;
    TEST_APP_ARM_USART_XferStatus_t XferStatus;
} TEST_APP_ARM_USART_Status_t;

typedef struct {
    void                    *pTxData;
    void                    *pRxData;
    volatile uint32_t       TxNum;
    volatile uint32_t       RxNum;
    volatile uint32_t       TxCnt;         // Number of data received
    volatile uint32_t       RxCnt;         // Number of data sent
} TEST_APP_ARM_USART_Transfer_t;

typedef struct {
    gpio_type               *pTxGpio;
    gpio_type               *pRxGpio;
    uint32_t                 TxPin;
    uint32_t                 RxPin;
} TEST_APP_ARM_USART_GPIO_t;

typedef struct {
    confirm_state               TxEnable;
    confirm_state               RxEnable;
    dma_type                    *pTxDMAx;
    dma_type                    *pRxDMAx;
    dma_channel_type            *pTxDMAxChany;
    dma_channel_type            *pRxDMAxChany;
    confirm_state               TxFlexModeEnable;
    confirm_state               RxFlexModeEnable;
    uint8_t                     TxFlexChannelx;
    uint8_t                     RxFlexChannelx;
    dma_flexible_request_type   TxFlexPeriphReq;
    dma_flexible_request_type   RxFlexPeriphReq;
    IRQn_Type                   TxIrqNum;         // DMA Tx channel IRQ Number
    IRQn_Type                   RxIrqNum;         // DMA Rx channel IRQ Number
    uint32_t                    *pTxEvent;     // ringbuffer
    uint32_t                    *pRxEvent;     // ringbuffer
    dma_init_type               TxCfg;
    dma_init_type               RxCfg;
} TEST_APP_ARM_USART_DMA_t;

typedef struct {
    usart_type                  *pUSARTx;
    IRQn_Type                   IrqNum;         // USART IRQ Number
    TEST_APP_RingBuffer_t                Event;
    TEST_APP_ARM_USART_Config_t          Config;
    TEST_APP_ARM_USART_GPIO_t            Gpio;
    TEST_APP_ARM_USART_Transfer_t        Transfer;
    TEST_APP_ARM_USART_Status_t          Status;
    TEST_APP_ARM_USART_DMA_t             DMA;
} TEST_APP_ARM_USART_Resources_t;

typedef struct {
    uint32_t (*Initialize)(uint32_t baudRate, usart_data_bit_num_type dataBit,
                           usart_stop_bit_num_type stopBit,
                           usart_parity_selection_type parity);
    uint32_t (*Uninitialize)(void);
    void (*Event_cb)(void);
    uint32_t (*Send)(void *pdata, uint32_t num);
    uint32_t (*Recieve)(void *pdata, uint32_t num);
    TEST_APP_ARM_USART_Transfer_t (*GetTransfer)(void);
    TEST_APP_ARM_USART_Status_t (*GetStatus)(void);
} TEST_APP_ARM_USART_Driver_t;

uint32_t TEST_APP_ARM_USART_Init(TEST_APP_ARM_USART_Resources_t *p_res);
uint32_t TEST_APP_ARM_USART_Uninit(TEST_APP_ARM_USART_Resources_t *p_res);
bool TEST_APP_ARM_USART_isReady(uint32_t status);
uint32_t TEST_APP_ARM_USART_SetResources(TEST_APP_ARM_USART_Resources_t *p_res, usart_type *p_usartx,
        void *p_event_buff, void *p_tx_buff,
        void *p_rx_buff, uint32_t BaudRate,
        usart_data_bit_num_type DataBit,
        usart_stop_bit_num_type StopBit,
        usart_parity_selection_type parity);
void TEST_APP_ARM_USART_IRQHandler(TEST_APP_ARM_USART_Driver_t *p_drv, TEST_APP_ARM_USART_Resources_t *p_res);
void TEST_APP_ARM_USART_cb(TEST_APP_ARM_USART_Driver_t *p_drv, TEST_APP_ARM_USART_Resources_t *p_res);
void TEST_APP_ARM_USART_WriteByte(TEST_APP_ARM_USART_Resources_t *p_res, uint8_t *pByte);
uint8_t TEST_APP_ARM_USART_ReadByte(TEST_APP_ARM_USART_Resources_t *p_res);
uint32_t TEST_APP_ARM_USART_Recieve(TEST_APP_ARM_USART_Resources_t *p_res, void *pdata, uint32_t num);
uint32_t TEST_APP_ARM_USART_Send(TEST_APP_ARM_USART_Resources_t *p_res, void *pdata, uint32_t num);
TEST_APP_ARM_USART_Status_t TEST_APP_ARM_USART_GetStatus(TEST_APP_ARM_USART_Resources_t *p_res);
TEST_APP_ARM_USART_Transfer_t TEST_APP_ARM_USART_GetTransfer(TEST_APP_ARM_USART_Resources_t *p_res);

#endif //_ARM_USART_H_ 
