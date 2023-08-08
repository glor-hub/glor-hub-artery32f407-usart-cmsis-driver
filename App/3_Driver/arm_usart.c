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
    uint32_t status = p_res->Status.status;
    if(!(ARM_CRM_USART_ClockEnable(p_res->pUSART_x, TRUE))) {
        return ARM_USART_DRIVER_ERROR;
    }
    RingBuffer_Init(&(p_res->Event), p_res->Event.p_buff, p_res->Event.buff_size);
//asynchronous mode is default
    usart_reset(p_res->pUSART_x);
    usart_enable(p_res->pUSART_x, TRUE);
    usart_init(p_res->pUSART_x, p_res->Config.baud_rate, p_res->Config.data_bit, p_res->Config.stop_bit);
    usart_parity_selection_config(p_res->pUSART_x, p_res->Config.parity);
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
    p_res->Status.status |= status;
    return status_ready;
}

uint32_t ARM_USART_Uninit(ARM_USART_Resources_t *p_res)
{
    uint32_t status_ready = ARM_USART_DRIVER_OK;
    uint32_t status = p_res->Status.status;
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
    p_res->Status.status |= status;
    return status_ready;
}

uint32_t ARM_USART_SetResources(ARM_USART_Resources_t *p_res, usart_type *p_usartx,
                                void *p_event_buff, void *p_tx_buff,
                                void *p_rx_buff, uint32_t baud_rate,
                                usart_data_bit_num_type data_bit,
                                usart_stop_bit_num_type stop_bit,
                                usart_parity_selection_type parity)
{
    p_res->pUSART_x = p_usartx;
    p_res->Event.p_buff = p_event_buff;
    p_res->Event.buff_size = ARM_USART_EVENT_BUFF_SIZE;
    p_res->Config.baud_rate = baud_rate;
    p_res->Config.data_bit = data_bit;
    p_res->Config.stop_bit = stop_bit;
    p_res->Config.parity = parity;
    p_res->Status.xfer_sta.tx_busy = 0;
    p_res->Status.xfer_sta.rx_busy = 0;
    p_res->Status.xfer_sta.tx_underflow = 0;
    p_res->Status.xfer_sta.rx_overflow = 0;
    p_res->Status.xfer_sta.rx_break = 0;
    p_res->Status.xfer_sta.rx_framing_error = 0;
    p_res->Status.xfer_sta.rx_parity_error = 0;
    p_res->Transfer.p_tx_buff = p_tx_buff;
    p_res->Transfer.p_rx_buff = p_rx_buff;
    p_res->Transfer.tx_num = 0;
    p_res->Transfer.rx_num = 0;
    p_res->Transfer.tx_cnt = 0;
    p_res->Transfer.rx_cnt = 0;
    if(p_res->pUSART_x == UART4) {
        p_res->irq_num = UART4_IRQn;
        p_res->Gpio.p_tx_gpio = GPIOC;
        p_res->Gpio.tx_pin = GPIO_PINS_10;
        p_res->Gpio.p_rx_gpio = GPIOC;
        p_res->Gpio.rx_pin = GPIO_PINS_11;
    } else if(p_res->pUSART_x == UART5) {
        p_res->irq_num = UART5_IRQn;
        p_res->Gpio.p_tx_gpio = GPIOC;
        p_res->Gpio.tx_pin = GPIO_PINS_12;
        p_res->Gpio.p_rx_gpio = GPIOD;
        p_res->Gpio.rx_pin = GPIO_PINS_2;
    } else if(p_res->pUSART_x == UART7) {
        p_res->irq_num = UART7_IRQn;
        p_res->Gpio.p_tx_gpio = GPIOE;
        p_res->Gpio.tx_pin = GPIO_PINS_8;
        p_res->Gpio.p_rx_gpio = GPIOE;
        p_res->Gpio.rx_pin = GPIO_PINS_7;
    } else if(p_res->pUSART_x == UART8) {
        p_res->irq_num = UART8_IRQn;
        p_res->Gpio.p_tx_gpio = GPIOE;
        p_res->Gpio.tx_pin = GPIO_PINS_1;
        p_res->Gpio.p_rx_gpio = GPIOE;
        p_res->Gpio.rx_pin = GPIO_PINS_0;
    } else {
#ifdef _APP_DEBUG_
        LOG("USART configuration error");
#endif//_APP_DEBUG_
        return ARM_USART_DRIVER_ERROR_UNSUPPORTED;
    }
    return ARM_USART_DRIVER_OK;
}

void ARM_USART_IRQHandler(ARM_USART_Driver_t *p_drv, ARM_USART_Resources_t *p_res)
{
    uint32_t event = 0;
    if((usart_flag_get(p_res->pUSART_x, USART_RDBF_FLAG) == SET) &&
       (p_res->Status.xfer_sta.rx_busy == 1)) {
        *((uint8_t *)p_res->Transfer.p_rx_buff + p_res->Transfer.rx_cnt) = ARM_USART_ReadByte(p_res);
        p_res->Transfer.rx_cnt++;
        if(usart_flag_get(p_res->pUSART_x, USART_ROERR_FLAG) == SET) {
            p_res->Status.xfer_sta.rx_overflow = 1;
            event |= ARM_USART_EVENT_RX_OVERFLOW;
        }
        if(usart_flag_get(p_res->pUSART_x, USART_FERR_FLAG) == SET) {
            p_res->Status.xfer_sta.rx_framing_error = 1;
            event |= ARM_USART_EVENT_RX_FRAMING_ERROR;
        }
        if(usart_flag_get(p_res->pUSART_x, USART_PERR_FLAG) == SET) {
            p_res->Status.xfer_sta.rx_parity_error = 1;
            event |= ARM_USART_EVENT_RX_PARITY_ERROR;
        }
        if(p_res->Transfer.rx_cnt == p_res->Transfer.rx_num) {
            event |= ARM_USART_EVENT_RX_COMPLETE;
            p_res->Status.xfer_sta.rx_busy = 0;
            usart_interrupt_enable(p_res->pUSART_x, USART_RDBF_INT, FALSE);
        }
    }
    if((usart_flag_get(p_res->pUSART_x, USART_TDBE_FLAG) == SET) &&
       (p_res->Status.xfer_sta.tx_busy == 1))  {
        ARM_USART_WriteByte(p_res,
                            ((uint8_t *)p_res->Transfer.p_tx_buff + p_res->Transfer.tx_cnt));
        p_res->Transfer.tx_cnt++;
        if(p_res->Transfer.tx_cnt == p_res->Transfer.tx_num) {
            while(usart_flag_get(p_res->pUSART_x, USART_TDC_FLAG != SET));
            event |= ARM_USART_EVENT_TX_COMPLETE;
            p_res->Status.xfer_sta.tx_busy = 0;
            usart_interrupt_enable(p_res->pUSART_x, USART_TDBE_INT, FALSE);
        }
    }
    if(event) {
        RingBuffer_Write(&(p_res->Event), &event);
    }
}

void ARM_USART_Event_cb(ARM_USART_Driver_t *p_drv, ARM_USART_Resources_t *p_res)
{
    uint32_t event;
    RingBuffer_Read(&(p_res->Event), &event);
    if(event & ARM_USART_EVENT_RX_COMPLETE) {
        p_res->Transfer.rx_cnt = 0;
        p_res->Transfer.rx_num = 0;
        p_res->Status.xfer_sta.rx_busy = 0;
#ifdef _APP_DEBUG_
        LCD_Printf(0, 0, SET, "%s", p_res->Transfer.p_rx_buff);
        LOG("USART recieved test data");
        memcpy(p_res->Transfer.p_tx_buff, p_res->Transfer.p_rx_buff, ARM_USART_RX_BUFF_SIZE);
        p_drv->Send(p_res->Transfer.p_tx_buff, 8);
#endif//_APP_DEBUG_          
    }
    if(event & ARM_USART_EVENT_TX_COMPLETE) {
        p_res->Transfer.tx_cnt = 0;
        p_res->Transfer.tx_num = 0;
        p_res->Status.xfer_sta.tx_busy = 0;
#ifdef _APP_DEBUG_
        p_drv->Recieve(p_res->Transfer.p_rx_buff, 8);
#endif//_APP_DEBUG_                 
    }
    if(event & ARM_USART_EVENT_RX_OVERFLOW) {
#ifdef _APP_DEBUG_
        LOG("USART receiver overflow");
#endif//_APP_DEBUG_                
    }
    if(event & ARM_USART_EVENT_RX_FRAMING_ERROR) {
#ifdef _APP_DEBUG_
        LOG("USART receiver framing error");
#endif//_APP_DEBUG_                
    }
    if(event & ARM_USART_EVENT_RX_PARITY_ERROR) {
#ifdef _APP_DEBUG_
        LOG("USART receiver parity error");
#endif//_APP_DEBUG_                
    }

}

uint32_t ARM_USART_Recieve(ARM_USART_Resources_t *p_res, void *pdata, uint32_t num)
{
    if(!(p_res->Status.status & ARM_USART_FLAG_RX_ENABLED)) {
        return ARM_USART_DRIVER_ERROR;
    }
    if(num == 0) {
        return ARM_USART_DRIVER_ERROR_PARAMETER;
    }
    if(p_res->Status.xfer_sta.rx_busy) {
        return ARM_USART_DRIVER_ERROR_BUSY;
    }
    p_res->Transfer.rx_cnt = 0;
    p_res->Transfer.rx_num = num;
    p_res->Transfer.p_rx_buff = pdata;
    p_res->Status.xfer_sta.rx_busy = 1;
    usart_interrupt_enable(p_res->pUSART_x, USART_RDBF_INT, TRUE);
    return ARM_USART_DRIVER_OK;
}

void ARM_USART_WriteByte(ARM_USART_Resources_t *p_res, uint8_t *pByte)
{
    p_res->pUSART_x->dt = *pByte;
}

uint32_t ARM_USART_Send(ARM_USART_Resources_t *p_res, void *pdata, uint32_t num)
{
    if(!(p_res->Status.status & ARM_USART_FLAG_TX_ENABLED)) {
        return ARM_USART_DRIVER_ERROR;
    }
    if(num == 0) {
        return ARM_USART_DRIVER_ERROR_PARAMETER;
    }
    if(p_res->Status.xfer_sta.tx_busy) {
        return ARM_USART_DRIVER_ERROR_BUSY;
    }
    p_res->Transfer.tx_cnt = 0;
    p_res->Transfer.tx_num = num;
    p_res->Transfer.p_tx_buff = pdata;
    p_res->Status.xfer_sta.tx_busy = 1;
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
    status.status = p_res->Status.status;
    status.xfer_sta.tx_busy = p_res->Status.xfer_sta.tx_busy;
    status.xfer_sta.rx_busy = p_res->Status.xfer_sta.rx_busy;
    status.xfer_sta.tx_underflow = p_res->Status.xfer_sta.tx_underflow;
    status.xfer_sta.rx_overflow = p_res->Status.xfer_sta.rx_overflow;
    status.xfer_sta.rx_break = p_res->Status.xfer_sta.rx_break;
    status.xfer_sta.rx_framing_error = p_res->Status.xfer_sta.rx_framing_error;
    status.xfer_sta.rx_parity_error = p_res->Status.xfer_sta.rx_parity_error;
    return status;
}

ARM_USART_Transfer_t ARM_USART_GetTransfer(ARM_USART_Resources_t *p_res)
{
    ARM_USART_Transfer_t transfer;
    transfer.p_tx_buff = p_res->Transfer.p_tx_buff;
    transfer.p_rx_buff = p_res->Transfer.p_rx_buff;
    transfer.tx_num = p_res->Transfer.tx_num;
    transfer.rx_num = p_res->Transfer.rx_num;
    transfer.tx_cnt = p_res->Transfer.tx_cnt;
    transfer.rx_cnt = p_res->Transfer.rx_cnt;
    return transfer;
}

//================================================================================
//Private
//================================================================================

static uint32_t ARM_USART_GPIO_Config(ARM_USART_Resources_t *p_res, confirm_state new_state)
{
    if(!ARM_CRM_GPIO_ClockEnable(p_res->Gpio.p_tx_gpio, TRUE)) {
#ifdef _APP_DEBUG_
        LOG("USART configuration error");
#endif//_APP_DEBUG_        
        return ARM_USART_DRIVER_ERROR;
    }
    if(new_state) {
        ARM_GPIO_Config(p_res->Gpio.p_tx_gpio, p_res->Gpio.tx_pin, GPIO_MODE_MUX, GPIO_OUTPUT_PUSH_PULL,
                        GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_STRONGER);
    } else {
        ARM_GPIO_Config(p_res->Gpio.p_tx_gpio, p_res->Gpio.tx_pin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                        GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
    }

    if(!ARM_CRM_GPIO_ClockEnable(p_res->Gpio.p_rx_gpio, TRUE)) {
#ifdef _APP_DEBUG_
        LOG("USART configuration error");
#endif//_APP_DEBUG_
        return ARM_USART_DRIVER_ERROR;
    }
    if(new_state) {
        //use any out_type and drive_strength for input i/o:
        ARM_GPIO_Config(p_res->Gpio.p_rx_gpio, p_res->Gpio.rx_pin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                        GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_MODERATE);
    } else {
        ARM_GPIO_Config(p_res->Gpio.p_rx_gpio, p_res->Gpio.rx_pin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                        GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
    }
    return ARM_USART_DRIVER_OK;
}

