//********************************************************************************
//usart.c
//********************************************************************************
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "usart.h"
#include "arm_usart.h"
#include "arm_gpio.h"
#include "arm_clock.h"

//********************************************************************************
//Macros
//********************************************************************************

#define USART_TX_BUFF_SIZE 256
#define USART_RX_BUFF_SIZE 256

//********************************************************************************
//Enums
//********************************************************************************

//********************************************************************************
//Typedefs
//********************************************************************************

//********************************************************************************
//Prototypes
//********************************************************************************

static void UART_IRQHandler(ARM_USART_Resources_t *p_res);
static uint32_t USART_Initialize(ARM_USART_Driver_t *p_drv, uint32_t baud_rate, usart_data_bit_num_type data_bit,
                                 usart_stop_bit_num_type stop_bit,
                                 usart_parity_selection_type parity);
#ifdef _UART4_PERIPH_

static uint32_t UART4_Initialize(uint32_t baud_rate, usart_data_bit_num_type data_bit,
                                 usart_stop_bit_num_type stop_bit,
                                 usart_parity_selection_type parity);

#endif//_UART4_PERIPH_

#ifdef _UART5_PERIPH_

static uint32_t UART5_Initialize(uint32_t baud_rate, usart_data_bit_num_type data_bit,
                                 usart_stop_bit_num_type stop_bit,
                                 usart_parity_selection_type parity);

#endif//_UART5_PERIPH_

#ifdef _UART7_PERIPH_

static uint32_t UART7_Initialize(uint32_t baud_rate, usart_data_bit_num_type data_bit,
                                 usart_stop_bit_num_type stop_bit,
                                 usart_parity_selection_type parity);

#endif//_UART7_PERIPH_

#ifdef _UART8_PERIPH_

static uint32_t UART8_Initialize(uint32_t baud_rate, usart_data_bit_num_type data_bit,
                                 usart_stop_bit_num_type stop_bit,
                                 usart_parity_selection_type parity);

#endif//_UART8_PERIPH_

//********************************************************************************
//Variables
//********************************************************************************

#ifdef _UART4_PERIPH_
static ARM_USART_Resources_t        UART4_Resources;
static ARM_USART_Transfer_t         UART4_Transfer;
static ARM_USART_Driver_t UART4_Driver = {
    UART4_Initialize,
    // UART4_Uninitialize,
    // UART4_Event_cb,
    // UART4_Send,
    // UART4_Recieve,
    // UART4_GetTransfer,
    // UART4_GetStatus,
};

static uint8_t UART4_Tx_Buff[USART_TX_BUFF_SIZE] = {0};
static uint8_t UART4_Rx_Buff[USART_RX_BUFF_SIZE] = {0};

#endif//_UART4_PERIPH_

#ifdef _UART5_PERIPH_

static ARM_USART_Resources_t        UART5_Resources;
static ARM_USART_Transfer_t         UART5_Transfer;
static ARM_USART_Driver_t UART5_Driver = {
    UART5_Initialize,
    // UART5_Uninitialize,
    // UART5_Event_cb,
    // UART5_Send,
    // UART5_Recieve,
    // UART5_GetTransfer,
    // UART5_GetStatus,
};
static uint8_t UART5_Tx_Buff[USART_TX_BUFF_SIZE] = {0};
static uint8_t UART5_Rx_Buff[USART_RX_BUFF_SIZE] = {0};

#endif//_UART5_PERIPH_

#ifdef _UART7_PERIPH_

static ARM_USART_Resources_t        UART7_Resources;
static ARM_USART_Transfer_t         UART7_Transfer;
static ARM_USART_Driver_t UART7_Driver = {
    UART7_Initialize,
    // UART7_Uninitialize,
    // UART7_Event_cb,
    // UART7_Send,
    // UART7_Recieve,
    // UART7_GetTransfer,
    // UART7_GetStatus,
};
static uint8_t UART7_Tx_Buff[USART_TX_BUFF_SIZE] = {0};
static uint8_t UART7_Rx_Buff[USART_RX_BUFF_SIZE] = {0};

#endif//_UART7_PERIPH_

#ifdef _UART8_PERIPH_

static ARM_USART_Resources_t        UART8_Resources;
static ARM_USART_Transfer_t         UART8_Transfer;
static ARM_USART_Driver_t UART8_Driver = {
    UART8_Initialize,
    // UART8_Uninitialize,
    // UART8_Event_cb,
    // UART8_Send,
    // UART8_Recieve,
    // UART8_GetTransfer,
    // UART8_GetStatus,
};
static uint8_t UART8_Tx_Buff[USART_TX_BUFF_SIZE] = {0};
static uint8_t UART8_Rx_Buff[USART_RX_BUFF_SIZE] = {0};

#endif//_UART8_PERIPH_




//================================================================================
//Public
//================================================================================

error_status USART_Init(void)
{
    uint32_t drv_status = ARM_USART_DRIVER_OK;

#ifdef _UART4_PERIPH_

    drv_status |= USART_Initialize(&UART4_Driver, ARM_USART_BAUDRATE_57600, USART_DATA_8BITS,
                                   USART_STOP_1_BIT,
                                   USART_PARITY_NONE);

#endif//_UART4_PERIPH_

#ifdef _UART5_PERIPH_

    drv_status |= USART_Initialize(&UART5_Driver, ARM_USART_BAUDRATE_57600, USART_DATA_8BITS,
                                   USART_STOP_1_BIT,
                                   USART_PARITY_NONE);

#endif//_UART5_PERIPH_

#ifdef _UART7_PERIPH_

    drv_status |= USART_Initialize(&UART7_Driver, ARM_USART_BAUDRATE_57600, USART_DATA_8BITS,
                                   USART_STOP_1_BIT,
                                   USART_PARITY_NONE);

#endif//_UART7_PERIPH;7

#ifdef _UART8_PERIPH_

    drv_status |= USART_Initialize(&UART8_Driver, ARM_USART_BAUDRATE_57600, USART_DATA_8BITS,
                                   USART_STOP_1_BIT,
                                   USART_PARITY_NONE);

#endif//_UART8_PERIPH_

    return ARM_USART_isReady(drv_status) ? SUCCESS : ERROR;
}



#ifdef _UART4_PERIPH_

void UART4_IRQHandler()
{
    UART_IRQHandler(&UART4_Resources);
}


#endif//_UART4_PERIPH_

#ifdef _UART5_PERIPH_

#endif//_UART5_PERIPH_

#ifdef _UART7_PERIPH_

#endif//_UART7_PERIPH_

#ifdef _UART8_PERIPH_


#endif//_UART8_PERIPH_


//================================================================================
//Private
//================================================================================

static void UART_IRQHandler(ARM_USART_Resources_t *p_res)
{

}

static uint32_t USART_Initialize(ARM_USART_Driver_t *p_drv, uint32_t baud_rate, usart_data_bit_num_type data_bit,
                                 usart_stop_bit_num_type stop_bit,
                                 usart_parity_selection_type parity)
{
    uint32_t drv_status = ARM_USART_DRIVER_OK;
    drv_status |= p_drv->Initialize(baud_rate, data_bit,
                                    stop_bit, parity);
    return drv_status;
}

#ifdef _UART4_PERIPH_

static uint32_t UART4_Initialize(uint32_t baud_rate, usart_data_bit_num_type data_bit,
                                 usart_stop_bit_num_type stop_bit,
                                 usart_parity_selection_type parity)
{
    uint32_t drv_status = ARM_USART_DRIVER_OK;
    ARM_USART_Resources_t *p_res = &UART4_Resources;
    drv_status |= ARM_USART_SetResources(UART4, p_res, &UART4_Transfer,
                                         &UART4_Tx_Buff[0], &UART4_Rx_Buff[0],
                                         baud_rate, data_bit, stop_bit, parity);
    drv_status |= ARM_USART_Init(p_res);
//clear and enable UARTx IRQ
    NVIC_ClearPendingIRQ(p_res->irq_num);
    NVIC_EnableIRQ(p_res->irq_num);
    return drv_status;
}

#endif//_UART4_PERIPH_

#ifdef _UART5_PERIPH_

static uint32_t UART5_Initialize(uint32_t baud_rate, usart_data_bit_num_type data_bit,
                                 usart_stop_bit_num_type stop_bit,
                                 usart_parity_selection_type parity)
{
    uint32_t drv_status = ARM_USART_DRIVER_OK;
    ARM_USART_Resources_t *p_res = &UART5_Resources;
    drv_status |= ARM_USART_SetResources(UART5, p_res, &UART5_Transfer,
                                         &UART5_Tx_Buff[0], &UART5_Rx_Buff[0],
                                         baud_rate, data_bit, stop_bit, parity);
    drv_status |= ARM_USART_Init(p_res);
//clear and enable UARTx IRQ
    NVIC_ClearPendingIRQ(p_res->irq_num);
    NVIC_EnableIRQ(p_res->irq_num);
    return drv_status;
}

#endif//_UART5_PERIPH_

#ifdef _UART7_PERIPH_

static uint32_t UART7_Initialize(uint32_t baud_rate, usart_data_bit_num_type data_bit,
                                 usart_stop_bit_num_type stop_bit,
                                 usart_parity_selection_type parity)
{
    uint32_t drv_status = ARM_USART_DRIVER_OK;
    ARM_USART_Resources_t *p_res = &UART7_Resources;
    drv_status |= ARM_USART_SetResources(UART7, p_res, &UART7_Transfer,
                                         &UART7_Tx_Buff[0], &UART7_Rx_Buff[0],
                                         baud_rate, data_bit, stop_bit, parity);
    drv_status |= ARM_USART_Init(p_res);
//clear and enable UARTx IRQ
    NVIC_ClearPendingIRQ(p_res->irq_num);
    NVIC_EnableIRQ(p_res->irq_num);
    return drv_status;
}

#endif//_UART7_PERIPH_

#ifdef _UART8_PERIPH_

static uint32_t UART8_Initialize(uint32_t baud_rate, usart_data_bit_num_type data_bit,
                                 usart_stop_bit_num_type stop_bit,
                                 usart_parity_selection_type parity)
{
    uint32_t drv_status = ARM_USART_DRIVER_OK;
    ARM_USART_Resources_t *p_res = &UART8_Resources;
    drv_status |= ARM_USART_SetResources(UART8, p_res, &UART8_Transfer,
                                         &UART8_Tx_Buff[0], &UART8_Rx_Buff[0],
                                         baud_rate, data_bit, stop_bit, parity);
    drv_status |= ARM_USART_Init(p_res);
//clear and enable UARTx IRQ
    NVIC_ClearPendingIRQ(p_res->irq_num);
    NVIC_EnableIRQ(p_res->irq_num);
    return drv_status;
}

#endif//_UART8_PERIPH_


