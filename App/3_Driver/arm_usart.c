//********************************************************************************
//arm_uart.c
//********************************************************************************
#include <string.h>
#include "arm_usart.h"
#include "arm_clock.h"
#include "arm_gpio.h"
#include "app.h"

#ifdef _APP_DEBUG_
#include "assert.h"
#include "LCD_2004.h"
#endif//_APP_DEBUG_

//********************************************************************************
//Macros
//********************************************************************************

//********************************************************************************
//Enums
//********************************************************************************

//********************************************************************************
//Typedefs
//********************************************************************************

//********************************************************************************
//Variables
//********************************************************************************

//********************************************************************************
//Prototypes
//********************************************************************************

static uint32_t ARM_USART_GPIO_Config(ARM_USART_Resources_t *p_res, confirm_state new_state);

//================================================================================
//Public
//================================================================================

bool ARM_USART_isReady(uint32_t status)
{
    return (status == ARM_USART_DRIVER_OK);
}

uint32_t ARM_USART_Init(ARM_USART_Resources_t *p_res)
{
    uint32_t status_ready = ARM_USART_DRIVER_OK;
    uint32_t status = p_res->sta.status;
    if(!(ARM_CRM_USART_ClockEnable(p_res->pUSART_x, TRUE))) {
        return ARM_USART_DRIVER_ERROR;
    }
//asynchronous mode is default
    usart_reset(p_res->pUSART_x);
    usart_enable(p_res->pUSART_x, TRUE);
    usart_init(p_res->pUSART_x, p_res->config.baud_rate, p_res->config.data_bit, p_res->config.stop_bit);
    usart_parity_selection_config(p_res->pUSART_x, p_res->config.parity);
    status |= ARM_USART_FLAG_CONFIGURATED;
// usart_interrupt_enable(p_res->pUSART_x, USART_RDBF_INT, TRUE);
// usart_interrupt_enable(p_res->pUSART_x, USART_TDBE_INT, TRUE);
    usart_transmitter_enable(p_res->pUSART_x, TRUE);
    status |= ARM_USART_FLAG_TX_ENABLED;
    usart_receiver_enable(p_res->pUSART_x, TRUE);
    status |= ARM_USART_FLAG_RX_ENABLED;
    status_ready |= ARM_USART_GPIO_Config(p_res, TRUE);
    //clear and enable UARTx IRQ
    NVIC_ClearPendingIRQ(p_res->irq_num);
    NVIC_EnableIRQ(p_res->irq_num);
    status |= ARM_USART_FLAG_INITIALIZED;
    p_res->sta.status |= status;
    return status_ready;
}

uint32_t ARM_USART_Uninit(ARM_USART_Resources_t *p_res)
{
    uint32_t status_ready = ARM_USART_DRIVER_OK;
    uint32_t status = p_res->sta.status;
    if(status & ARM_USART_FLAG_INITIALIZED) {
        //disable and clear UARTx IRQ
        NVIC_DisableIRQ(p_res->irq_num);
        NVIC_ClearPendingIRQ(p_res->irq_num);
        usart_transmitter_enable(p_res->pUSART_x, FALSE);
        status &= ~ARM_USART_FLAG_TX_ENABLED;
        usart_receiver_enable(p_res->pUSART_x, FALSE);
        status &= ~ARM_USART_FLAG_RX_ENABLED;
        status_ready |= ARM_USART_GPIO_Config(p_res, FALSE);
        usart_enable(p_res->pUSART_x, FALSE);
        usart_reset(p_res->pUSART_x);
        status &= ~ARM_USART_FLAG_CONFIGURATED;
        if(!(ARM_CRM_USART_ClockEnable(p_res->pUSART_x, FALSE))) {
            return ARM_USART_DRIVER_ERROR;
        }
        status &= ~ARM_USART_FLAG_INITIALIZED;
    } else {
#ifdef _APP_DEBUG_
        LOG("USART not been initialized");
#endif//_APP_DEBUG_
    }
    p_res->sta.status |= status;
    return status_ready;
}

uint32_t ARM_USART_SetResources(ARM_USART_Resources_t *p_res, usart_type *p_usartx,
                                void *p_tx_buff,
                                void *p_rx_buff, uint32_t baud_rate,
                                usart_data_bit_num_type data_bit,
                                usart_stop_bit_num_type stop_bit,
                                usart_parity_selection_type parity)
{
    p_res->pUSART_x = p_usartx;
    p_res->config.baud_rate = baud_rate;
    p_res->config.data_bit = data_bit;
    p_res->config.stop_bit = stop_bit;
    p_res->config.parity = parity;
    p_res->sta.xfer_sta.tx_busy = 0;
    p_res->sta.xfer_sta.rx_busy = 0;
    p_res->sta.xfer_sta.tx_underflow = 0;
    p_res->sta.xfer_sta.rx_overflow = 0;
    p_res->sta.xfer_sta.rx_break = 0;
    p_res->sta.xfer_sta.rx_framing_error = 0;
    p_res->sta.xfer_sta.rx_parity_error = 0;
    p_res->xfer.p_tx_buff = p_tx_buff;
    p_res->xfer.p_rx_buff = p_rx_buff;
    p_res->xfer.tx_num = 0;
    p_res->xfer.rx_num = 0;
    p_res->xfer.tx_cnt = 0;
    p_res->xfer.rx_cnt = 0;
    if(p_res->pUSART_x == UART4) {
        p_res->irq_num = UART4_IRQn;
        p_res->gpio.p_tx_gpio = GPIOC;
        p_res->gpio.tx_pin = GPIO_PINS_10;
        p_res->gpio.p_rx_gpio = GPIOC;
        p_res->gpio.rx_pin = GPIO_PINS_11;
    } else if(p_res->pUSART_x == UART5) {
        p_res->irq_num = UART5_IRQn;
        p_res->gpio.p_tx_gpio = GPIOC;
        p_res->gpio.tx_pin = GPIO_PINS_12;
        p_res->gpio.p_rx_gpio = GPIOD;
        p_res->gpio.rx_pin = GPIO_PINS_2;
    } else if(p_res->pUSART_x == UART7) {
        p_res->irq_num = UART7_IRQn;
        p_res->gpio.p_tx_gpio = GPIOE;
        p_res->gpio.tx_pin = GPIO_PINS_7;
        p_res->gpio.p_rx_gpio = GPIOE;
        p_res->gpio.rx_pin = GPIO_PINS_8;
    } else if(p_res->pUSART_x == UART8) {
        p_res->irq_num = UART8_IRQn;
        p_res->gpio.p_tx_gpio = GPIOE;
        p_res->gpio.tx_pin = GPIO_PINS_0;
        p_res->gpio.p_rx_gpio = GPIOE;
        p_res->gpio.rx_pin = GPIO_PINS_1;
    } else {
#ifdef _APP_DEBUG_
        LOG("USART configuration error");
#endif//_APP_DEBUG_
        return ARM_USART_DRIVER_ERROR_UNSUPPORTED;
    }
    return ARM_USART_DRIVER_OK;
}

uint32_t ARM_USART_ResetResources(ARM_USART_Resources_t *p_res, usart_type *p_usartx,
                                  void *p_tx_buff, void *p_rx_buff)
{
    p_res->pUSART_x = p_usartx;
    p_res->config.baud_rate = 0;
    p_res->config.data_bit = USART_DATA_8BITS;
    p_res->config.stop_bit = USART_STOP_1_BIT;
    p_res->config.parity = USART_PARITY_NONE;
    p_res->sta.xfer_sta.tx_busy = 0;
    p_res->sta.xfer_sta.rx_busy = 0;
    p_res->sta.xfer_sta.tx_underflow = 0;
    p_res->sta.xfer_sta.rx_overflow = 0;
    p_res->sta.xfer_sta.rx_break = 0;
    p_res->sta.xfer_sta.rx_framing_error = 0;
    p_res->sta.xfer_sta.rx_parity_error = 0;
    p_res->xfer.p_tx_buff = p_tx_buff;
    p_res->xfer.p_rx_buff = p_rx_buff;
    p_res->xfer.tx_num = 0;
    p_res->xfer.rx_num = 0;
    p_res->xfer.tx_cnt = 0;
    p_res->xfer.rx_cnt = 0;
    if(p_res->pUSART_x == UART4) {
        p_res->irq_num = UART4_IRQn;
        p_res->gpio.p_tx_gpio = GPIOC;
        p_res->gpio.tx_pin = GPIO_PINS_10;
        p_res->gpio.p_rx_gpio = GPIOC;
        p_res->gpio.rx_pin = GPIO_PINS_11;
    } else if(p_res->pUSART_x == UART5) {
        p_res->irq_num = UART5_IRQn;
        p_res->gpio.p_tx_gpio = GPIOC;
        p_res->gpio.tx_pin = GPIO_PINS_12;
        p_res->gpio.p_rx_gpio = GPIOD;
        p_res->gpio.rx_pin = GPIO_PINS_2;
    } else if(p_res->pUSART_x == UART7) {
        p_res->irq_num = UART7_IRQn;
        p_res->gpio.p_tx_gpio = GPIOE;
        p_res->gpio.tx_pin = GPIO_PINS_7;
        p_res->gpio.p_rx_gpio = GPIOE;
        p_res->gpio.rx_pin = GPIO_PINS_8;
    } else if(p_res->pUSART_x == UART8) {
        p_res->irq_num = UART8_IRQn;
        p_res->gpio.p_tx_gpio = GPIOE;
        p_res->gpio.tx_pin = GPIO_PINS_0;
        p_res->gpio.p_rx_gpio = GPIOE;
        p_res->gpio.rx_pin = GPIO_PINS_1;
    } else {
#ifdef _APP_DEBUG_
        LOG("USART configuration error");
#endif//_APP_DEBUG_
        return ARM_USART_DRIVER_ERROR_UNSUPPORTED;
    }
    return ARM_USART_DRIVER_OK;
}

uint32_t ARM_USART_Recieve(ARM_USART_Resources_t *p_res, void *pdata, uint32_t num)
{
    if(!(p_res->sta.status & ARM_USART_FLAG_RX_ENABLED)) {
        return ARM_USART_DRIVER_ERROR;
    }
    if(num == 0) {
        return ARM_USART_DRIVER_ERROR_PARAMETER;
    }
    if(p_res->sta.xfer_sta.rx_busy) {
        return ARM_USART_DRIVER_ERROR_BUSY;
    }
    p_res->xfer.rx_cnt = 0;
    p_res->xfer.rx_num = num;
    p_res->xfer.p_rx_buff = pdata;
    p_res->sta.xfer_sta.rx_busy = 1;
    usart_interrupt_enable(p_res->pUSART_x, USART_RDBF_INT, TRUE);
    return ARM_USART_DRIVER_OK;
}

void ARM_USART_WriteByte(ARM_USART_Resources_t *p_res, uint8_t *pByte)
{
    p_res->pUSART_x->dt = *pByte;
}

uint32_t ARM_USART_Send(ARM_USART_Resources_t *p_res, const void *pdata, uint32_t num)
{
    if(!(p_res->sta.status & ARM_USART_FLAG_TX_ENABLED)) {
        return ARM_USART_DRIVER_ERROR;
    }
    if(num == 0) {
        return ARM_USART_DRIVER_ERROR_PARAMETER;
    }
    if(p_res->sta.xfer_sta.tx_busy) {
        return ARM_USART_DRIVER_ERROR_BUSY;
    }
    p_res->xfer.tx_cnt = 0;
    p_res->xfer.tx_num = num;
    p_res->xfer.p_tx_buff = pdata;
    p_res->sta.xfer_sta.tx_busy = 1;
    usart_interrupt_enable(p_res->pUSART_x, USART_TDBE_INT, TRUE);
    return ARM_USART_DRIVER_OK;
}

uint8_t ARM_USART_ReadByte(ARM_USART_Resources_t *p_res)
{
    uint8_t byte;
    byte = p_res->pUSART_x->dt;
    return byte;
}

ARM_USART_Status_t ARM_USART_GetStatus(ARM_USART_Resources_t *p_res)
{
    ARM_USART_Status_t status;
    status.status = p_res->sta.status;
    status.xfer_sta.tx_busy = p_res->sta.xfer_sta.tx_busy;
    status.xfer_sta.rx_busy = p_res->sta.xfer_sta.rx_busy;
    status.xfer_sta.tx_underflow = p_res->sta.xfer_sta.tx_underflow;
    status.xfer_sta.rx_overflow = p_res->sta.xfer_sta.rx_overflow;
    status.xfer_sta.rx_break = p_res->sta.xfer_sta.rx_break;
    status.xfer_sta.rx_framing_error = p_res->sta.xfer_sta.rx_framing_error;
    status.xfer_sta.rx_parity_error = p_res->sta.xfer_sta.rx_parity_error;
    return status;
}

ARM_USART_Transfer_t ARM_USART_GetTransfer(ARM_USART_Resources_t *p_res)
{
    ARM_USART_Transfer_t transfer;
    transfer.p_tx_buff = p_res->xfer.p_tx_buff;
    transfer.p_rx_buff = p_res->xfer.p_rx_buff;
    transfer.tx_num = p_res->xfer.tx_num;
    transfer.rx_num = p_res->xfer.rx_num;
    transfer.tx_cnt = p_res->xfer.tx_cnt;
    transfer.rx_cnt = p_res->xfer.rx_cnt;
    return transfer;
}

//================================================================================
//Private
//================================================================================

static uint32_t ARM_USART_GPIO_Config(ARM_USART_Resources_t *p_res, confirm_state new_state)
{
    if(!ARM_CRM_GPIO_ClockEnable(p_res->gpio.p_tx_gpio, TRUE)) {
#ifdef _APP_DEBUG_
        LOG("USART configuration error");
#endif//_APP_DEBUG_        
        return ARM_USART_DRIVER_ERROR;
    }
    if(new_state) {
        ARM_GPIO_Config(p_res->gpio.p_tx_gpio, p_res->gpio.tx_pin, GPIO_MODE_MUX, GPIO_OUTPUT_PUSH_PULL,
                        GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_STRONGER);
    } else {
        ARM_GPIO_Config(p_res->gpio.p_tx_gpio, p_res->gpio.tx_pin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                        GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
    }

    if(!ARM_CRM_GPIO_ClockEnable(p_res->gpio.p_rx_gpio, TRUE)) {
#ifdef _APP_DEBUG_
        LOG("USART configuration error");
#endif//_APP_DEBUG_
        return ARM_USART_DRIVER_ERROR;
    }
    if(new_state) {
        //use any out_type and drive_strength for input i/o:
        ARM_GPIO_Config(p_res->gpio.p_rx_gpio, p_res->gpio.rx_pin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                        GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_MODERATE);
    } else {
        ARM_GPIO_Config(p_res->gpio.p_rx_gpio, p_res->gpio.rx_pin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                        GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
    }
    return ARM_USART_DRIVER_OK;
}

