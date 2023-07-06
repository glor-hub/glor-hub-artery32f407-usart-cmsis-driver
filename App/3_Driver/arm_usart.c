//********************************************************************************
//arm_uart.c
//********************************************************************************
#include "arm_usart.h"
#include "arm_clock.h"
#include "arm_gpio.h"

//********************************************************************************
//Macros
//********************************************************************************

#define ARM_USART_DRIVER_OK         ((uint32_t)0UL)
#define ARM_USART_DRIVER_ERROR      ((uint32_t)1UL << 0)
#define ARM_DRIVER_ERROR_BUSY       ((uint32_t)1UL << 1)
#define ARM_DRIVER_ERROR_TIMEOUT    ((uint32_t)1UL << 2)

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

static uint32_t ARM_USART_GPIO_Config(ARM_USART_Resources_t *p_res);

//================================================================================
//Public
//================================================================================

bool ARM_USART_isReady(uint32_t status)
{
    return (status == ARM_USART_DRIVER_OK);
}

uint32_t ARM_USART_Init(ARM_USART_Resources_t *p_res)
{
    uint32_t drv_status = ARM_USART_DRIVER_OK;
    if(ARM_CRM_USART_ClockEnable(p_res->pUSART_x, TRUE)) {
        drv_status |=  ARM_USART_DRIVER_ERROR;
    }
    //asynchronous mode is default
    usart_reset(p_res->pUSART_x);
    usart_enable(p_res->pUSART_x, TRUE);
    usart_init(p_res->pUSART_x, p_res->config.baud_rate, p_res->config.data_bit, p_res->config.stop_bit);
    usart_parity_selection_config(p_res->pUSART_x, p_res->config.parity);
    usart_interrupt_enable(p_res->pUSART_x, USART_RDBF_INT, TRUE);
    // usart_interrupt_enable(p_res->pUSART_x, USART_TDBE_INT, TRUE);
    usart_transmitter_enable(p_res->pUSART_x, TRUE);
    usart_receiver_enable(p_res->pUSART_x, TRUE);
    drv_status |= ARM_USART_GPIO_Config(p_res);
    return drv_status;
}

uint32_t ARM_USART_SetResources(usart_type *p_usartx, ARM_USART_Resources_t *p_res,
                                ARM_USART_Transfer_t *p_xfer_struct, void *p_tx_buff,
                                void *p_rx_buff, uint32_t baud_rate,
                                usart_data_bit_num_type data_bit,
                                usart_stop_bit_num_type stop_bit,
                                usart_parity_selection_type parity)
{
    uint32_t drv_status = ARM_USART_DRIVER_OK;
    p_res->pUSART_x = p_usartx;
    if(p_usartx == UART4) {
        p_res->irq_num = UART4_IRQn;
        p_res->gpio.p_tx_gpio = GPIOC;
        p_res->gpio.tx_pin = GPIO_PINS_10;
        p_res->gpio.p_rx_gpio = GPIOC;
        p_res->gpio.rx_pin = GPIO_PINS_11;
    } else if(p_usartx == UART5) {
        p_res->irq_num = UART5_IRQn;
        p_res->gpio.p_tx_gpio = GPIOC;
        p_res->gpio.tx_pin = GPIO_PINS_12;
        p_res->gpio.p_rx_gpio = GPIOD;
        p_res->gpio.rx_pin = GPIO_PINS_2;
    } else if(p_usartx == UART7) {
        p_res->irq_num = UART7_IRQn;
        p_res->gpio.p_tx_gpio = GPIOE;
        p_res->gpio.tx_pin = GPIO_PINS_7;
        p_res->gpio.p_rx_gpio = GPIOE;
        p_res->gpio.rx_pin = GPIO_PINS_8;
    } else if(p_usartx == UART8) {
        p_res->irq_num = UART8_IRQn;
        p_res->gpio.p_tx_gpio = GPIOE;
        p_res->gpio.tx_pin = GPIO_PINS_0;
        p_res->gpio.p_rx_gpio = GPIOE;
        p_res->gpio.rx_pin = GPIO_PINS_1;
    } else {
        drv_status |= ARM_USART_DRIVER_ERROR_UNSUPPORTED;
#ifdef _APP_DEBUG_
        LOG("USART configuration error");
#endif//_APP_DEBUG_
    }
    p_res->config.baud_rate = baud_rate;
    p_res->config.data_bit = data_bit;
    p_res->config.stop_bit = stop_bit;
    p_res->config.parity = parity;
    p_res->p_xfer = p_xfer_struct;
    p_res->p_xfer->p_tx_buff = p_tx_buff;
    p_res->p_xfer->p_rx_buff = p_rx_buff;
    p_res->p_xfer->tx_num = 0;
    p_res->p_xfer->rx_num = 0;
    p_res->p_xfer->tx_cnt = 0;
    p_res->p_xfer->rx_cnt = 0;
    p_res->p_xfer->status.tx_busy = 0;
    p_res->p_xfer->status.tx_busy = 0;
    p_res->p_xfer->status.tx_busy = 0;
    p_res->p_xfer->status.rx_busy = 0;
    p_res->p_xfer->status.tx_underflow = 0;
    p_res->p_xfer->status.rx_overflow = 0;
    p_res->p_xfer->status.rx_framing_error = 0;
    p_res->p_xfer->status.rx_parity_error = 0;
    ARM_USART_GPIO_Config(p_res);
    return drv_status;
}

// void ARM_USART_Recieve(ARM_USART_Resources_t *p_res, ARM_USART_Transfer_t *p_tr, void *pdata, uint32_t num)
// {
//     p_tr->rx_num = num;
//     while(p_tr->cnt < p_tr->rx_num) &&
//     }

// void ARM_USART_SendData(usart_type *pUSART_x, uint8_t *pdata, uint8_t len)
// {

// }


//================================================================================
//Private
//================================================================================

static uint32_t ARM_USART_GPIO_Config(ARM_USART_Resources_t *p_res)
{
    uint32_t drv_status = ARM_USART_DRIVER_OK;
    if(ARM_CRM_GPIO_ClockEnable(p_res->gpio.p_tx_gpio, TRUE)) {
        drv_status |=  ARM_USART_DRIVER_ERROR;
#ifdef _APP_DEBUG_
        LOG("USART configuration error");
#endif//_APP_DEBUG_
    }
    ARM_GPIO_Config(p_res->gpio.p_tx_gpio, p_res->gpio.tx_pin, GPIO_MODE_MUX, GPIO_OUTPUT_PUSH_PULL,
                    GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_STRONGER);
    if(ARM_CRM_GPIO_ClockEnable(p_res->gpio.p_rx_gpio, TRUE)) {
        drv_status |=  ARM_USART_DRIVER_ERROR;
#ifdef _APP_DEBUG_
        LOG("USART configuration error");
#endif//_APP_DEBUG_
    }
    //use any out_type and drive_strength for input i/o:
    ARM_GPIO_Config(p_res->gpio.p_rx_gpio, p_res->gpio.rx_pin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                    GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_MODERATE);
    return drv_status;
}

