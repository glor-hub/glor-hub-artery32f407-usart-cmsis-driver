#ifndef _ARM_USART_H_
#define _ARM_USART_H_

#include "at32f403a_407.h"
#include <stdbool.h>

//USART Driver Status
#define ARM_USART_DRIVER_OK                 ((uint32_t)0UL)
#define ARM_USART_DRIVER_ERROR              ((uint32_t)1UL << 0)
#define ARM_USART_DRIVER_ERROR_UNSUPPORTED  ((uint32_t)1UL << 1)
#define ARM_USART_DRIVER_ERROR_BUSY         ((uint32_t)1UL << 2)
#define ARM_USART_DRIVER_ERROR_TIMEOUT      ((uint32_t)1UL << 3)

//USART Baudrate
#define ARM_USART_BAUDRATE_9600    ((uint32_t)9600)
#define ARM_USART_BAUDRATE_19200   ((uint32_t)19200)
#define ARM_USART_BAUDRATE_38400   ((uint32_t)38400)
#define ARM_USART_BAUDRATE_57600   ((uint32_t)57600)
#define ARM_USART_BAUDRATE_115200  ((uint32_t)115200)

//USART Event
#define ARM_USART_EVENT_SEND_COMPLETE       ((uint32_t)1UL << 0)  //Send completed; however USART may still transmit data
#define ARM_USART_EVENT_RECEIVE_COMPLETE    ((uint32_t)1UL << 1)  //Receive completed
#define ARM_USART_EVENT_TRANSFER_COMPLETE   ((uint32_t)1UL << 2)  //Transfer completed
#define ARM_USART_EVENT_TX_COMPLETE         ((uint32_t)1UL << 3)  //Transmit completed (optional)
#define ARM_USART_EVENT_TX_UNDERFLOW        ((uint32_t)1UL << 4)  //Transmit data not available (Synchronous Slave)
#define ARM_USART_EVENT_RX_OVERFLOW         ((uint32_t)1UL << 5)  //Receive data overflow
#define ARM_USART_EVENT_RX_TIMEOUT          ((uint32_t)1UL << 6)  //Receive character timeout (optional)
#define ARM_USART_EVENT_RX_BREAK            ((uint32_t)1UL << 7)  //Break detected on receive
#define ARM_USART_EVENT_RX_FRAMING_ERROR    ((uint32_t)1UL << 8)  //Framing error detected on receive
#define ARM_USART_EVENT_RX_PARITY_ERROR     ((uint32_t)1UL << 9)  //Parity error detected on receive

typedef struct {
    uint32_t                    baud_rate;
    usart_data_bit_num_type     data_bit;
    usart_stop_bit_num_type     stop_bit;
    usart_parity_selection_type parity;
} ARM_USART_Config_t;

typedef struct {
    uint32_t tx_busy;
    uint32_t rx_busy;
    uint32_t tx_underflow;
    uint32_t rx_overflow;
    uint32_t rx_break;
    uint32_t rx_framing_error;
    uint32_t rx_parity_error;
} ARM_USART_Status_t;

typedef struct {
    void                    *p_tx_buff;
    void                    *p_rx_buff;
    volatile uint32_t       tx_num;
    volatile uint32_t       rx_num;
    volatile uint32_t       tx_cnt;         // Number of data received
    volatile uint32_t       rx_cnt;         // Number of data sent
    ARM_USART_Status_t      status;
} ARM_USART_Transfer_t;

typedef struct {
    gpio_type               *p_tx_gpio;
    gpio_type               *p_rx_gpio;
    uint32_t                 tx_pin;
    uint32_t                 rx_pin;
} ARM_USART_GPIO_t;

typedef struct {
    usart_type                  *pUSART_x;
    IRQn_Type                   irq_num;         // USART IRQ Number
    ARM_USART_Config_t          config;
    ARM_USART_GPIO_t            gpio;
    ARM_USART_Transfer_t        *p_xfer;
} ARM_USART_Resources_t;

typedef struct {
    uint32_t (*Initialize)(uint32_t baud_rate, usart_data_bit_num_type data_bit,
                           usart_stop_bit_num_type stop_bit,
                           usart_parity_selection_type parity);
    uint32_t (*Uninitialize)(void);
    void (*Event_cb)(void);
    uint32_t (*Send)(const void *pdata, uint32_t num);
    uint32_t (*Recieve)(void *pdata, uint32_t num);
    ARM_USART_Transfer_t (*GetTransfer)(void);
    ARM_USART_Status_t (*GetStatus)(void);
} ARM_USART_Driver_t;

uint32_t ARM_USART_Init(ARM_USART_Resources_t *p_res);
bool ARM_USART_isReady(uint32_t status);
uint32_t ARM_USART_SetResources(usart_type *p_usartx, ARM_USART_Resources_t *p_res,
                                ARM_USART_Transfer_t *p_xfer_struct, void *p_tx_buff,
                                void *p_rx_buff, uint32_t baud_rate,
                                usart_data_bit_num_type data_bit,
                                usart_stop_bit_num_type stop_bit,
                                usart_parity_selection_type parity);
uint32_t ARM_USART_InitTransfer(ARM_USART_Resources_t *p_res,
                                void *p_tx_buff, void *p_rx_buff);

#endif //_ARM_USART_H_ 
