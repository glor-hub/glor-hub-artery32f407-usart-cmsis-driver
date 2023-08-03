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
static uint32_t UART4_Send(const void *pbuff, uint32_t num);
static ARM_USART_Status_t UART4_GetStatus(void);
static ARM_USART_Transfer_t UART4_GetTransfer(void);
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

static uint8_t UART4_Tx_Buff[USART_TX_BUFF_SIZE];
static uint8_t UART4_Rx_Buff[USART_RX_BUFF_SIZE];
#endif//_UART4_PERIPH_

#ifdef _UART5_PERIPH_
static ARM_USART_Resources_t        UART5_Resources;
static ARM_USART_Transfer_t         UART5_Transfer;
static ARM_USART_Driver_t UART5_Driver = {
    UART5_Initialize,
    // UART5_Uninitialize,
    UART5_Event_cb,
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
    UART7_Event_cb,
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
    UART8_Event_cb,
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
    ARM_USART_Driver_t *p_drv = &UART4_Driver;
    event = p_drv->event;
    if(event) {
        p_drv->Event_cb();
    }
#endif//_UART4_PERIPH_

}

#ifdef _UART4_PERIPH_

void UART4_IRQHandler()
{
    uint32_t event = 0;
    ARM_USART_Driver_t *p_drv = &UART4_Driver;
    ARM_USART_Resources_t *p_res = &UART4_Resources;
    if((usart_flag_get(p_res->pUSART_x, USART_RDBF_FLAG) == SET) &&
       (p_res->sta.xfer_sta.rx_busy == 1)) {
        *((uint8_t *)p_res->xfer.p_rx_buff + p_res->xfer.rx_cnt) = ARM_USART_ReadByte(&UART4_Resources);
        p_res->xfer.rx_cnt++;
        if(usart_flag_get(p_res->pUSART_x, USART_ROERR_FLAG) == SET) {
            p_res->sta.xfer_sta.rx_overflow = 1;
            event |= ARM_USART_EVENT_RX_OVERFLOW;
        }
        if(usart_flag_get(p_res->pUSART_x, USART_FERR_FLAG) == SET) {
            p_res->sta.xfer_sta.rx_framing_error = 1;
            event |= ARM_USART_EVENT_RX_FRAMING_ERROR;
        }
        if(usart_flag_get(p_res->pUSART_x, USART_PERR_FLAG) == SET) {
            p_res->sta.xfer_sta.rx_parity_error = 1;
            event |= ARM_USART_EVENT_RX_PARITY_ERROR;
        }
        if(p_res->xfer.rx_cnt == p_res->xfer.rx_num) {
            event |= ARM_USART_EVENT_RX_COMPLETE;
            p_res->sta.xfer_sta.rx_busy = 0;
            usart_interrupt_enable(p_res->pUSART_x, USART_RDBF_INT, FALSE);
        }
    }
    if((usart_flag_get(p_res->pUSART_x, USART_TDBE_FLAG) == SET) &&
       (p_res->sta.xfer_sta.tx_busy == 1))  {
        ARM_USART_WriteByte(&UART4_Resources,
                            ((uint8_t *)p_res->xfer.p_tx_buff + p_res->xfer.tx_cnt));
        p_res->xfer.tx_cnt++;
        if(p_res->xfer.tx_cnt == p_res->xfer.tx_num) {
            while(usart_flag_get(p_res->pUSART_x, USART_TDC_FLAG != SET));
            event |= ARM_USART_EVENT_TX_COMPLETE;
            p_res->sta.xfer_sta.tx_busy = 0;
            usart_interrupt_enable(p_res->pUSART_x, USART_TDBE_INT, FALSE);
        }
    }
    if(event) {
        p_drv->event = event;
    }
}

#endif//_UART4_PERIPH_

#ifdef _UART5_PERIPH_

#endif//_UART5_PERIPH_

#ifdef _UART7_PERIPH_

#endif//_UART7_PERIPH_

#ifdef _UART8_PERIPH_


#endif//_UART8_PERIPH_

#ifdef _APP_DEBUG_
error_status USART_Test(void)
{
    uint32_t status_ready;
#ifdef _UART4_PERIPH_
    ARM_USART_Driver_t *p_drv = &UART4_Driver;
    ARM_USART_Resources_t *p_res = &UART4_Resources;
    p_res->xfer.p_tx_buff = UART4_Tx_Buff;
    USART_printf(p_drv, "%s", "Hello, World, from UART4!\n");
    status_ready = p_drv->Recieve(UART4_Rx_Buff, 8);
#endif//_UART4_PERIPH_
    return ARM_USART_isReady(status_ready) ? SUCCESS : ERROR;
}
#endif//_APP_DEBUG_



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
                                           &UART4_Tx_Buff[0], &UART4_Rx_Buff[0],
                                           baud_rate, data_bit, stop_bit, parity);
    status_ready |= ARM_USART_Init(p_res);
    return status_ready;
}

static uint32_t UART4_Uninitialize(void)
{
    uint32_t status_ready = ARM_USART_DRIVER_OK;
    ARM_USART_Resources_t *p_res = &UART4_Resources;
    status_ready |= ARM_USART_ResetResources(p_res, UART4, &UART4_Tx_Buff[0], &UART4_Rx_Buff[0]);
    status_ready |= ARM_USART_Uninit(p_res);
    return status_ready;
}

static void UART4_Event_cb(void)
{
    uint32_t event;
    ARM_USART_Driver_t *p_drv = &UART4_Driver;
    ARM_USART_Resources_t *p_res = &UART4_Resources;
    event = p_drv->event;
    p_drv->event = 0;
    if(event & ARM_USART_EVENT_RX_COMPLETE) {
        p_res->xfer.rx_cnt = 0;
        p_res->xfer.rx_num = 0;
        p_res->sta.xfer_sta.rx_busy = 0;
#ifdef _APP_DEBUG_
        LCD_Printf(0, 0, SET, "%s", UART4_Rx_Buff);
        LOG("UART4 recieved test data");
        memcpy(UART4_Tx_Buff, UART4_Rx_Buff, sizeof(UART4_Rx_Buff));
        ARM_USART_Driver_t *p_drv = &UART4_Driver;
        p_drv->Send(UART4_Tx_Buff, 8);
#endif//_APP_DEBUG_          
    }
    if(event & ARM_USART_EVENT_TX_COMPLETE) {
        p_res->xfer.tx_cnt = 0;
        p_res->xfer.tx_num = 0;
        p_res->sta.xfer_sta.tx_busy = 0;
#ifdef _APP_DEBUG_
        ARM_USART_Driver_t *p_drv = &UART4_Driver;
        p_drv->Recieve(UART4_Rx_Buff, 8);
#endif//_APP_DEBUG_                 
    }
    if(event & ARM_USART_EVENT_RX_OVERFLOW) {
#ifdef _APP_DEBUG_
        LOG("UART4 receiver overflow");
#endif//_APP_DEBUG_                
    }
    if(event & ARM_USART_EVENT_RX_FRAMING_ERROR) {
#ifdef _APP_DEBUG_
        LOG("UART4 receiver framing error");
#endif//_APP_DEBUG_                
    }
    if(event & ARM_USART_EVENT_RX_PARITY_ERROR) {
#ifdef _APP_DEBUG_
        LOG("UART4 receiver parity error");
#endif//_APP_DEBUG_                
    }
}

static uint32_t UART4_Send(const void *pbuff, uint32_t num)
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
    status_ready |= ARM_USART_SetResources(UART5, p_res, &UART5_Transfer,
                                           &UART5_Tx_Buff[0], &UART5_Rx_Buff[0],
                                           baud_rate, data_bit, stop_bit, parity);
    status_ready |= ARM_USART_Init(p_res);
//clear and enable UARTx IRQ
    NVIC_ClearPendingIRQ(p_res->irq_num);
    NVIC_EnableIRQ(p_res->irq_num);
    return status_ready;
}

#endif//_UART5_PERIPH_

#ifdef _UART7_PERIPH_

static uint32_t UART7_Initialize(uint32_t baud_rate, usart_data_bit_num_type data_bit,
                                 usart_stop_bit_num_type stop_bit,
                                 usart_parity_selection_type parity)
{
    uint32_t status_ready = ARM_USART_DRIVER_OK;
    ARM_USART_Resources_t *p_res = &UART7_Resources;
    status_ready |= ARM_USART_SetResources(UART7, p_res, &UART7_Transfer,
                                           &UART7_Tx_Buff[0], &UART7_Rx_Buff[0],
                                           baud_rate, data_bit, stop_bit, parity);
    status_ready |= ARM_USART_Init(p_res);
//clear and enable UARTx IRQ
    NVIC_ClearPendingIRQ(p_res->irq_num);
    NVIC_EnableIRQ(p_res->irq_num);
    return status_ready;
}

#endif//_UART7_PERIPH_

#ifdef _UART8_PERIPH_

static uint32_t UART8_Initialize(uint32_t baud_rate, usart_data_bit_num_type data_bit,
                                 usart_stop_bit_num_type stop_bit,
                                 usart_parity_selection_type parity)
{
    uint32_t status_ready = ARM_USART_DRIVER_OK;
    ARM_USART_Resources_t *p_res = &UART8_Resources;
    status_ready |= ARM_USART_SetResources(UART8, p_res, &UART8_Transfer,
                                           &UART8_Tx_Buff[0], &UART8_Rx_Buff[0],
                                           baud_rate, data_bit, stop_bit, parity);
    status_ready |= ARM_USART_Init(p_res);
//clear and enable UARTx IRQ
    NVIC_ClearPendingIRQ(p_res->irq_num);
    NVIC_EnableIRQ(p_res->irq_num);
    return status_ready;
}

#endif//_UART8_PERIPH_
