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

#ifdef _APP_DEBUG_
#include "assert.h"
#include "LCD_2004.h"
#include "timer.h"
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
//Prototypes
//********************************************************************************

static uint32_t USART_Initialize(ARM_USART_Driver_t *p_drv, uint32_t baud_rate, usart_data_bit_num_type data_bit,
                                 usart_stop_bit_num_type stop_bit,
                                 usart_parity_selection_type parity);

#ifdef _UART4_PERIPH_
static uint32_t UART4_Initialize(uint32_t baud_rate, usart_data_bit_num_type data_bit,
                                 usart_stop_bit_num_type stop_bit,
                                 usart_parity_selection_type parity);
static uint32_t UART4_Uninitialize(void);
static void UART4_Event_cb(void);
static uint32_t UART4_Recieve(void *pbuff, uint32_t num);
static uint32_t UART4_Send(void *pbuff, uint32_t num);
static ARM_USART_Status_t UART4_GetStatus(void);
static ARM_USART_Transfer_t UART4_GetTransfer(void);
#endif//_UART4_PERIPH_

#ifdef _UART5_PERIPH_
static uint32_t UART5_Initialize(uint32_t baud_rate, usart_data_bit_num_type data_bit,
                                 usart_stop_bit_num_type stop_bit,
                                 usart_parity_selection_type parity);
static uint32_t UART5_Uninitialize(void);
static void UART5_Event_cb(void);
static uint32_t UART5_Recieve(void *pbuff, uint32_t num);
static uint32_t UART5_Send(void *pbuff, uint32_t num);
static ARM_USART_Status_t UART5_GetStatus(void);
static ARM_USART_Transfer_t UART5_GetTransfer(void);
#endif//_UART5_PERIPH_

#ifdef _UART7_PERIPH_
static uint32_t UART7_Initialize(uint32_t baud_rate, usart_data_bit_num_type data_bit,
                                 usart_stop_bit_num_type stop_bit,
                                 usart_parity_selection_type parity);
static uint32_t UART7_Uninitialize(void);
static void UART7_Event_cb(void);
static uint32_t UART7_Recieve(void *pbuff, uint32_t num);
static uint32_t UART7_Send(void *pbuff, uint32_t num);
static ARM_USART_Status_t UART7_GetStatus(void);
static ARM_USART_Transfer_t UART7_GetTransfer(void);
#endif//_UART7_PERIPH_

#ifdef _UART8_PERIPH_
static uint32_t UART8_Initialize(uint32_t baud_rate, usart_data_bit_num_type data_bit,
                                 usart_stop_bit_num_type stop_bit,
                                 usart_parity_selection_type parity);
static uint32_t UART8_Uninitialize(void);
static void UART8_Event_cb(void);
static uint32_t UART8_Recieve(void *pbuff, uint32_t num);
static uint32_t UART8_Send(void *pbuff, uint32_t num);
static ARM_USART_Status_t UART8_GetStatus(void);
static ARM_USART_Transfer_t UART8_GetTransfer(void);
#endif//_UART8_PERIPH_

//********************************************************************************
//Variables
//********************************************************************************

#ifdef _UART4_PERIPH_
static ARM_USART_Resources_t        UART4_Resources;
static ARM_USART_Driver_t UART4_Driver = {
    0U,
    UART4_Initialize,
    UART4_Uninitialize,
    UART4_Event_cb,
    UART4_Send,
    UART4_Recieve,
    UART4_GetTransfer,
    UART4_GetStatus,
};
static uint8_t UART4_Tx_Buff[ARM_USART_TX_BUFF_SIZE];
static uint8_t UART4_Rx_Buff[ARM_USART_RX_BUFF_SIZE];
#endif//_UART4_PERIPH_

#ifdef _UART5_PERIPH_
static ARM_USART_Resources_t        UART5_Resources;
static ARM_USART_Driver_t UART5_Driver = {
    0U,
    UART5_Initialize,
    UART5_Uninitialize,
    UART5_Event_cb,
    UART5_Send,
    UART5_Recieve,
    UART5_GetTransfer,
    UART5_GetStatus,
};
static uint8_t UART5_Tx_Buff[ARM_USART_TX_BUFF_SIZE];
static uint8_t UART5_Rx_Buff[ARM_USART_RX_BUFF_SIZE];
#endif//_UART5_PERIPH_

#ifdef _UART7_PERIPH_
static ARM_USART_Resources_t        UART7_Resources;
static ARM_USART_Driver_t UART7_Driver = {
    0U,
    UART7_Initialize,
    UART7_Uninitialize,
    UART7_Event_cb,
    UART7_Send,
    UART7_Recieve,
    UART7_GetTransfer,
    UART7_GetStatus,
};

static uint8_t UART7_Tx_Buff[ARM_USART_TX_BUFF_SIZE];
static uint8_t UART7_Rx_Buff[ARM_USART_RX_BUFF_SIZE];
#endif//_UART7_PERIPH_

#ifdef _UART8_PERIPH_
static ARM_USART_Resources_t        UART8_Resources;
static ARM_USART_Driver_t UART8_Driver = {
    0U,
    UART8_Initialize,
    UART8_Uninitialize,
    UART8_Event_cb,
    UART8_Send,
    UART8_Recieve,
    UART8_GetTransfer,
    UART8_GetStatus,
};
static uint8_t UART8_Tx_Buff[ARM_USART_TX_BUFF_SIZE];
static uint8_t UART8_Rx_Buff[ARM_USART_RX_BUFF_SIZE];
#endif//_UART8_PERIPH_

//================================================================================
//Public
//================================================================================

error_status USART_Init(void)
{
    uint32_t status_ready = ARM_USART_DRIVER_OK;

#ifdef _UART4_PERIPH_
    status_ready |= USART_Initialize(&UART4_Driver, ARM_USART_BAUDRATE_57600, USART_DATA_8BITS,
                                     USART_STOP_1_BIT,
                                     USART_PARITY_NONE);
#endif//_UART4_PERIPH_

#ifdef _UART5_PERIPH_
    status_ready |= USART_Initialize(&UART5_Driver, ARM_USART_BAUDRATE_57600, USART_DATA_8BITS,
                                     USART_STOP_1_BIT,
                                     USART_PARITY_NONE);
#endif//_UART5_PERIPH_

#ifdef _UART7_PERIPH_
    status_ready |= USART_Initialize(&UART7_Driver, ARM_USART_BAUDRATE_57600, USART_DATA_8BITS,
                                     USART_STOP_1_BIT,
                                     USART_PARITY_NONE);
#endif//_UART7_PERIPH_

#ifdef _UART8_PERIPH_
    status_ready |= USART_Initialize(&UART8_Driver, ARM_USART_BAUDRATE_57600, USART_DATA_8BITS,
                                     USART_STOP_1_BIT,
                                     USART_PARITY_NONE);
#endif//_UART8_PERIPH_

    return ARM_USART_isReady(status_ready) ? SUCCESS : ERROR;
}

error_status USART_Uninitialize(ARM_USART_Driver_t *p_drv)
{
    uint32_t status_ready;
    status_ready = p_drv->Uninitialize();
    return ARM_USART_isReady(status_ready) ? SUCCESS : ERROR;
}

void USART_Event_cb(void)
{
    uint32_t event;

#ifdef _UART4_PERIPH_
    ARM_USART_Driver_t *p4_drv = &UART4_Driver;
    event = p4_drv->event;
    if(event) {
        p4_drv->Event_cb();
    }
#endif//_UART4_PERIPH_

#ifdef _UART5_PERIPH_
    ARM_USART_Driver_t *p5_drv = &UART5_Driver;
    event = p5_drv->event;
    if(event) {
        p5_drv->Event_cb();
    }
#endif//_UART5_PERIPH_

#ifdef _UART7_PERIPH_
    ARM_USART_Driver_t *p7_drv = &UART7_Driver;
    event = p7_drv->event;
    if(event) {
        p7_drv->Event_cb();
    }
#endif//_UART7_PERIPH_

#ifdef _UART8_PERIPH_
    ARM_USART_Driver_t *p8_drv = &UART8_Driver;
    event = p8_drv->event;
    if(event) {
        p8_drv->Event_cb();
    }
#endif//_UART8_PERIPH_

}

int8_t USART_printf(ARM_USART_Driver_t *p_drv, char *fmt, ...)
{
    uint32_t status_ready;
    int8_t res, len;
    char *strbuff = (char *)p_drv->GetTransfer().p_tx_buff;
    va_list args;
    va_start(args, fmt);
    res = vsprintf(strbuff, fmt, args);
    len = strlen(strbuff);
    status_ready = p_drv->Send(strbuff, len);
    va_end(args);
    if(status_ready == ARM_USART_DRIVER_OK) {
        return res;
    } else {
        return -1;
    }
}

#ifdef _APP_DEBUG_
error_status USART_Test(void)
{
    uint32_t status_ready = ARM_USART_DRIVER_OK;

#ifdef _UART4_PERIPH_
    ARM_USART_Driver_t *p4_drv = &UART4_Driver;
    ARM_USART_Resources_t *p4_res = &UART4_Resources;
    p4_res->xfer.p_tx_buff = UART4_Tx_Buff;
    USART_printf(p4_drv, "%s", "Hello, World, from UART4!\n");
    status_ready |= p4_drv->Recieve(UART4_Rx_Buff, 8);
#endif//_UART4_PERIPH_

#ifdef _UART5_PERIPH_
    ARM_USART_Driver_t *p5_drv = &UART5_Driver;
    ARM_USART_Resources_t *p5_res = &UART5_Resources;
    p5_res->xfer.p_tx_buff = UART5_Tx_Buff;
    USART_printf(p5_drv, "%s", "Hello, World, from UART5!\n");
    status_ready |= p5_drv->Recieve(UART5_Rx_Buff, 8);
#endif//_UART5_PERIPH_

#ifdef _UART7_PERIPH_
    ARM_USART_Driver_t *p7_drv = &UART7_Driver;
    ARM_USART_Resources_t *p7_res = &UART7_Resources;
    p7_res->xfer.p_tx_buff = UART7_Tx_Buff;
    USART_printf(p7_drv, "%s", "Hello, World, from UART7!\n");
    status_ready |= p7_drv->Recieve(UART7_Rx_Buff, 8);
#endif//_UART7_PERIPH_

#ifdef _UART8_PERIPH_
    ARM_USART_Driver_t *p8_drv = &UART8_Driver;
    ARM_USART_Resources_t *p8_res = &UART8_Resources;
    p8_res->xfer.p_tx_buff = UART8_Tx_Buff;
    USART_printf(p8_drv, "%s", "Hello, World, from UART8!\n");
    status_ready |= p8_drv->Recieve(UART8_Rx_Buff, 8);
#endif//_UART8_PERIPH_

    return ARM_USART_isReady(status_ready) ? SUCCESS : ERROR;
}
#endif//_APP_DEBUG_

#ifdef _UART4_PERIPH_
void UART4_IRQHandler()
{
    ARM_USART_IRQHandler(&UART4_Driver, &UART4_Resources);
}
#endif//_UART4_PERIPH_

#ifdef _UART5_PERIPH_
void UART5_IRQHandler()
{
    ARM_USART_IRQHandler(&UART5_Driver, &UART5_Resources);
}
#endif//_UART5_PERIPH_

#ifdef _UART7_PERIPH_
void UART7_IRQHandler()
{
    ARM_USART_IRQHandler(&UART7_Driver, &UART7_Resources);
}
#endif//_UART7_PERIPH_

#ifdef _UART8_PERIPH_
void UART8_IRQHandler()
{
    ARM_USART_IRQHandler(&UART8_Driver, &UART8_Resources);
}
#endif//_UART8_PERIPH_



//================================================================================
//Private
//================================================================================

static uint32_t USART_Initialize(ARM_USART_Driver_t *p_drv, uint32_t baud_rate, usart_data_bit_num_type data_bit,
                                 usart_stop_bit_num_type stop_bit,
                                 usart_parity_selection_type parity)
{
    uint32_t status_ready;
    status_ready = p_drv->Initialize(baud_rate, data_bit,
                                     stop_bit, parity);
    return status_ready;
}

#ifdef _UART4_PERIPH_
static uint32_t UART4_Initialize(uint32_t baud_rate, usart_data_bit_num_type data_bit,
                                 usart_stop_bit_num_type stop_bit,
                                 usart_parity_selection_type parity)
{
    uint32_t status_ready = ARM_USART_DRIVER_OK;
    ARM_USART_Resources_t *p_res = &UART4_Resources;
    p_res->sta.status = 0U;
    status_ready |= ARM_USART_SetResources(p_res, UART4,
                                           UART4_Tx_Buff, UART4_Rx_Buff,
                                           baud_rate, data_bit, stop_bit, parity);
    status_ready |= ARM_USART_Init(p_res);
    return status_ready;
}

static uint32_t UART4_Uninitialize(void)
{
    uint32_t status_ready = ARM_USART_DRIVER_OK;
    ARM_USART_Resources_t *p_res = &UART4_Resources;
    status_ready |= ARM_USART_SetResources(p_res, UART4,
                                           UART4_Tx_Buff, UART4_Rx_Buff,
                                           0, USART_DATA_8BITS,
                                           USART_STOP_1_BIT,
                                           USART_PARITY_NONE);
    status_ready |= ARM_USART_Uninit(p_res);
    return status_ready;
}

static void UART4_Event_cb(void)
{
    ARM_USART_Event_cb(&UART4_Driver, &UART4_Resources);
}

static uint32_t UART4_Send(void *pbuff, uint32_t num)
{
    return ARM_USART_Send(&UART4_Resources, pbuff, num);
}

static uint32_t UART4_Recieve(void *pbuff, uint32_t num)
{
    return ARM_USART_Recieve(&UART4_Resources, pbuff, num);
}

static ARM_USART_Status_t UART4_GetStatus(void)
{
    return ARM_USART_GetStatus(&UART4_Resources);
}

static ARM_USART_Transfer_t UART4_GetTransfer(void)
{
    return ARM_USART_GetTransfer(&UART4_Resources);
}
#endif//_UART4_PERIPH_

#ifdef _UART5_PERIPH_
static uint32_t UART5_Initialize(uint32_t baud_rate, usart_data_bit_num_type data_bit,
                                 usart_stop_bit_num_type stop_bit,
                                 usart_parity_selection_type parity)
{
    uint32_t status_ready = ARM_USART_DRIVER_OK;
    ARM_USART_Resources_t *p_res = &UART5_Resources;
    p_res->sta.status = 0U;
    status_ready |= ARM_USART_SetResources(p_res, UART5,
                                           UART5_Tx_Buff, UART5_Rx_Buff,
                                           baud_rate, data_bit, stop_bit, parity);
    status_ready |= ARM_USART_Init(p_res);
    return status_ready;
}

static uint32_t UART5_Uninitialize(void)
{
    uint32_t status_ready = ARM_USART_DRIVER_OK;
    ARM_USART_Resources_t *p_res = &UART5_Resources;
    status_ready |= ARM_USART_SetResources(p_res, UART5,
                                           UART5_Tx_Buff, UART5_Rx_Buff,
                                           0, USART_DATA_8BITS,
                                           USART_STOP_1_BIT,
                                           USART_PARITY_NONE);
    status_ready |= ARM_USART_Uninit(p_res);
    return status_ready;
}

static void UART5_Event_cb(void)
{
    ARM_USART_Event_cb(&UART5_Driver, &UART5_Resources);
}

static uint32_t UART5_Send(void *pbuff, uint32_t num)
{
    return ARM_USART_Send(&UART5_Resources, pbuff, num);
}

static uint32_t UART5_Recieve(void *pbuff, uint32_t num)
{
    return ARM_USART_Recieve(&UART5_Resources, pbuff, num);
}

static ARM_USART_Status_t UART5_GetStatus(void)
{
    return ARM_USART_GetStatus(&UART5_Resources);
}

static ARM_USART_Transfer_t UART5_GetTransfer(void)
{
    return ARM_USART_GetTransfer(&UART5_Resources);
}
#endif//_UART5_PERIPH_

#ifdef _UART7_PERIPH_
static uint32_t UART7_Initialize(uint32_t baud_rate, usart_data_bit_num_type data_bit,
                                 usart_stop_bit_num_type stop_bit,
                                 usart_parity_selection_type parity)
{
    uint32_t status_ready = ARM_USART_DRIVER_OK;
    ARM_USART_Resources_t *p_res = &UART7_Resources;
    p_res->sta.status = 0U;
    status_ready |= ARM_USART_SetResources(p_res, UART7,
                                           UART7_Tx_Buff, UART7_Rx_Buff,
                                           baud_rate, data_bit, stop_bit, parity);
    status_ready |= ARM_USART_Init(p_res);
    return status_ready;
}

static uint32_t UART7_Uninitialize(void)
{
    uint32_t status_ready = ARM_USART_DRIVER_OK;
    ARM_USART_Resources_t *p_res = &UART7_Resources;
    status_ready |= ARM_USART_SetResources(p_res, UART7,
                                           UART7_Tx_Buff, UART7_Rx_Buff,
                                           0, USART_DATA_8BITS,
                                           USART_STOP_1_BIT,
                                           USART_PARITY_NONE);
    status_ready |= ARM_USART_Uninit(p_res);
    return status_ready;
}

static void UART7_Event_cb(void)
{
    ARM_USART_Event_cb(&UART7_Driver, &UART7_Resources);
}

static uint32_t UART7_Send(void *pbuff, uint32_t num)
{
    return ARM_USART_Send(&UART7_Resources, pbuff, num);
}

static uint32_t UART7_Recieve(void *pbuff, uint32_t num)
{
    return ARM_USART_Recieve(&UART7_Resources, pbuff, num);
}

static ARM_USART_Status_t UART7_GetStatus(void)
{
    return ARM_USART_GetStatus(&UART7_Resources);
}

static ARM_USART_Transfer_t UART7_GetTransfer(void)
{
    return ARM_USART_GetTransfer(&UART7_Resources);
}
#endif//_UART7_PERIPH_

#ifdef _UART8_PERIPH_
static uint32_t UART8_Initialize(uint32_t baud_rate, usart_data_bit_num_type data_bit,
                                 usart_stop_bit_num_type stop_bit,
                                 usart_parity_selection_type parity)
{
    uint32_t status_ready = ARM_USART_DRIVER_OK;
    ARM_USART_Resources_t *p_res = &UART8_Resources;
    p_res->sta.status = 0U;
    status_ready |= ARM_USART_SetResources(p_res, UART8,
                                           UART8_Tx_Buff, UART8_Rx_Buff,
                                           baud_rate, data_bit, stop_bit, parity);
    status_ready |= ARM_USART_Init(p_res);
    return status_ready;
}

static uint32_t UART8_Uninitialize(void)
{
    uint32_t status_ready = ARM_USART_DRIVER_OK;
    ARM_USART_Resources_t *p_res = &UART8_Resources;
    status_ready |= ARM_USART_SetResources(p_res, UART8,
                                           UART8_Tx_Buff, UART8_Rx_Buff,
                                           0, USART_DATA_8BITS,
                                           USART_STOP_1_BIT,
                                           USART_PARITY_NONE);
    status_ready |= ARM_USART_Uninit(p_res);
    return status_ready;
}

static void UART8_Event_cb(void)
{
    ARM_USART_Event_cb(&UART8_Driver, &UART8_Resources);
}

static uint32_t UART8_Send(void *pbuff, uint32_t num)
{
    return ARM_USART_Send(&UART8_Resources, pbuff, num);
}

static uint32_t UART8_Recieve(void *pbuff, uint32_t num)
{
    return ARM_USART_Recieve(&UART8_Resources, pbuff, num);
}

static ARM_USART_Status_t UART8_GetStatus(void)
{
    return ARM_USART_GetStatus(&UART8_Resources);
}

static ARM_USART_Transfer_t UART8_GetTransfer(void)
{
    return ARM_USART_GetTransfer(&UART8_Resources);
}
#endif//_UART8_PERIPH_
