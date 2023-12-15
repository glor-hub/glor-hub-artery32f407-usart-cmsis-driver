#ifndef _ARM_USART_H_
#define _ARM_USART_H_

#include "at32f403a_407.h"
#include <stdbool.h>
#include "ringbuffer.h"
#include "arm_dma.h"

//select UARTx (USARTx are not supported in this driver version)
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
#define _TEST_APP_UART4_TX_DMA_CIRCULAR_MODE_
#define _TEST_APP_UART4_RX_USE_DMA_
// #define _TEST_APP_UART4_RX_DMA_CIRCULAR_MODE_

/*******************************************
UART5

For DMA:
DMA1 CHANNEL5 (with flexible mapping) - Tx
DMA1_CHANNEL4 (with flexible mapping) - Rx
********************************************/
#define _TEST_APP_UART5_PERIPH_ENABLE_
#define _TEST_APP_UART5_TX_USE_DMA_
#define _TEST_APP_UART5_TX_DMA_CIRCULAR_MODE_
#define _TEST_APP_UART5_RX_USE_DMA_
// #define _TEST_APP_UART5_RX_DMA_CIRCULAR_MODE_

/*******************************************
UART7

For DMA:
DMA1 CHANNEL3 (with flexible mapping) - Tx
DMA1_CHANNEL2 (with flexible mapping) - Rx
********************************************/
#define _TEST_APP_UART7_PERIPH_ENABLE_
#define _TEST_APP_UART7_TX_USE_DMA_
#define _TEST_APP_UART7_TX_DMA_CIRCULAR_MODE_
#define _TEST_APP_UART7_RX_USE_DMA_
// #define _TEST_APP_UART7_RX_DMA_CIRCULAR_MODE_

/*******************************************
UART8

For DMA:
DMA2 CHANNEL6 (with flexible mapping) - Tx
DMA2_CHANNEL4 (with flexible mapping) - Rx
********************************************/
#define _TEST_APP_UART8_PERIPH_ENABLE_
#define _TEST_APP_UART8_TX_USE_DMA_
#define _TEST_APP_UART8_TX_DMA_CIRCULAR_MODE_
#define _TEST_APP_UART8_RX_USE_DMA_
// #define _TEST_APP_UART8_RX_DMA_CIRCULAR_MODE_

/*******************************************

********************************************/

typedef enum {
    TEST_APP_ARM_USART1_TX_CHAN = 0,
    TEST_APP_ARM_USART1_RX_CHAN,
    TEST_APP_ARM_USART2_TX_CHAN,
    TEST_APP_ARM_USART2_RX_CHAN,
    TEST_APP_ARM_USART3_TX_CHAN,
    TEST_APP_ARM_USART3_RX_CHAN,
    TEST_APP_ARM_UART4_TX_CHAN,
    TEST_APP_ARM_UART4_RX_CHAN,
    TEST_APP_ARM_UART5_TX_CHAN,
    TEST_APP_ARM_UART5_RX_CHAN,
    TEST_APP_ARM_USART6_TX_CHAN,
    TEST_APP_ARM_USART6_RX_CHAN,
    TEST_APP_ARM_UART7_TX_CHAN,
    TEST_APP_ARM_UART7_RX_CHAN,
    TEST_APP_ARM_UART8_TX_CHAN,
    TEST_APP_ARM_UART8_RX_CHAN,
    TEST_APP_ARM_USART_CHANS
} eTEST_APP_ARM_USART_Chan_t;

#define TEST_APP_ARM_USART_TX_BUFF_SIZE 256
#define TEST_APP_ARM_USART_RX_BUFF_SIZE 256

#define TEST_APP_ARM_USART_EVENT_BUFF_SIZE 8

// USART Driver flags
#define TEST_APP_ARM_USART_FLAG_INITIALIZED          (uint32_t)(1U << 0)
#define TEST_APP_ARM_USART_FLAG_CONFIGURATED         (uint32_t)(1U << 1)
#define TEST_APP_ARM_USART_FLAG_TX_ENABLED           (uint32_t)(1U << 2)
#define TEST_APP_ARM_USART_FLAG_RX_ENABLED           (uint32_t)(1U << 3)

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
    uint32_t                PinDef;
    gpio_type               *pTxGpio;
    gpio_type               *pRxGpio;
    uint32_t                 TxPin;
    uint32_t                 RxPin;
} TEST_APP_ARM_USART_GPIO_t;

typedef struct {
    eTEST_APP_ARM_DMA_Chan_t    TxChan;
    eTEST_APP_ARM_DMA_Chan_t    RxChan;
    confirm_state               TxEnable;
    confirm_state               RxEnable;
    confirm_state               TxFlexModeEnable;
    confirm_state               RxFlexModeEnable;
    dma_flexible_request_type   TxFlexPeriphReq;
    dma_flexible_request_type   RxFlexPeriphReq;
    dma_init_type               TxCfg;
    dma_init_type               RxCfg;
} TEST_APP_ARM_USART_DMA_t;

typedef struct {
    eTEST_APP_ARM_USART_Chan_t          TxChan;
    eTEST_APP_ARM_USART_Chan_t          RxChan;
    usart_type                          *pUSARTx;
    IRQn_Type                           IrqNum;     // USART IRQ Number
    TEST_APP_RingBuffer_t               Event;
    TEST_APP_ARM_USART_Config_t         Config;
    TEST_APP_ARM_USART_GPIO_t           Gpio;
    TEST_APP_ARM_USART_Transfer_t       Transfer;
    TEST_APP_ARM_USART_Status_t         Status;
    TEST_APP_ARM_USART_DMA_t            DMA;
} TEST_APP_ARM_USART_Resources_t;

typedef struct {
    uint32_t (*Initialize)(uint32_t baudRate, usart_data_bit_num_type dataBit,
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

uint32_t TEST_APP_ARM_USART_Init(TEST_APP_ARM_USART_Resources_t *p_res);
uint32_t TEST_APP_ARM_USART_Uninit(TEST_APP_ARM_USART_Resources_t *p_res);
uint32_t TEST_APP_ARM_USART_SetResources(TEST_APP_ARM_USART_Resources_t *p_res,
        usart_type *p_usartx,
        eTEST_APP_ARM_USART_Chan_t tx_chan,
        eTEST_APP_ARM_USART_Chan_t rx_chan,
        void *p_event_buff, void *p_tx_buff,
        void *p_rx_buff, uint32_t BaudRate,
        usart_data_bit_num_type DataBit,
        usart_stop_bit_num_type StopBit,
        usart_parity_selection_type parity,
        uint32_t gpio_pin_def);
void TEST_APP_ARM_USART_IRQHandler(TEST_APP_ARM_USART_Resources_t *p_res);
void TEST_APP_ARM_USART_cb(TEST_APP_ARM_USART_Resources_t *p_res);
void TEST_APP_ARM_USART_WriteByte(TEST_APP_ARM_USART_Resources_t *p_res, uint8_t *pByte);
uint8_t TEST_APP_ARM_USART_ReadByte(TEST_APP_ARM_USART_Resources_t *p_res);
uint32_t TEST_APP_ARM_USART_Recieve(TEST_APP_ARM_USART_Resources_t *p_res, void *pdata, uint32_t num);
uint32_t TEST_APP_ARM_USART_Send(TEST_APP_ARM_USART_Resources_t *p_res, void *pdata, uint32_t num);
TEST_APP_ARM_USART_Status_t TEST_APP_ARM_USART_GetStatus(TEST_APP_ARM_USART_Resources_t *p_res);
TEST_APP_ARM_USART_Transfer_t TEST_APP_ARM_USART_GetTransfer(TEST_APP_ARM_USART_Resources_t *p_res);
void TEST_APP_ARM_USART_SetDefaultTxBuffer(TEST_APP_ARM_USART_Resources_t *p_res, void *pbuff);
void TEST_APP_ARM_USART_SetDefaultRxBuffer(TEST_APP_ARM_USART_Resources_t *p_res, void *pbuff);

#endif //_ARM_USART_H_ 
