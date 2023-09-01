//********************************************************************************
//arm_uart.c
//********************************************************************************
#include <string.h>
#include "arm_usart.h"
#include "arm_clock.h"
#include "arm_gpio.h"
#include "app.h"
#include "arm_dma.h"

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
    uint32_t status = p_res->Status.Status;
    if(!(ARM_CRM_USART_ClockEnable(p_res->pUSARTx, TRUE))) {
        return ARM_USART_DRIVER_ERROR;
    }
//asynchronous mode is default
    usart_reset(p_res->pUSARTx);
    usart_enable(p_res->pUSARTx, TRUE);
    usart_init(p_res->pUSARTx, p_res->Config.BaudRate, p_res->Config.DataBit, p_res->Config.StopBit);
    usart_parity_selection_config(p_res->pUSARTx, p_res->Config.Parity);
    status |= ARM_USART_FLAG_CONFIGURATED;
    if(p_res->DMA.TxEnable) {
        //clock and reset DMA
        if(!ARM_DMA_Init(p_res->DMA.pTxDMAxChany)) {
#ifdef _APP_DEBUG_
            LOG("USART DMA driver error");
#endif//_APP_DEBUG_
            return ARM_USART_DRIVER_ERROR;
        }
        //clear and enable DMAxChany IRQ
        NVIC_ClearPendingIRQ(p_res->DMA.TxIrqNum);
        NVIC_EnableIRQ(p_res->DMA.TxIrqNum);
        usart_dma_transmitter_enable(p_res->pUSARTx, TRUE);
    }
    if(p_res->DMA.RxEnable) {
        //clock and reset DMA
        if(!ARM_DMA_Init(p_res->DMA.pRxDMAxChany)) {
#ifdef _APP_DEBUG_
            LOG("USART DMA driver error");
#endif//_APP_DEBUG_
            return ARM_USART_DRIVER_ERROR;
        }
        //clear and enable DMAxChany IRQ
        NVIC_ClearPendingIRQ(p_res->DMA.RxIrqNum);
        NVIC_EnableIRQ(p_res->DMA.RxIrqNum);
        usart_dma_receiver_enable(p_res->pUSARTx, TRUE);
    }
// usart_interrupt_enable(p_res->pUSARTx, USART_RDBF_INT, TRUE);
// usart_interrupt_enable(p_res->pUSARTx, USART_TDBE_INT, TRUE);
    usart_transmitter_enable(p_res->pUSARTx, TRUE);
    status |= ARM_USART_FLAG_TX_ENABLED;
    usart_receiver_enable(p_res->pUSARTx, TRUE);
    status |= ARM_USART_FLAG_RX_ENABLED;
    status_ready |= ARM_USART_GPIO_Config(p_res, TRUE);
//clear and enable UARTx IRQ
    NVIC_ClearPendingIRQ(p_res->IrqNum);
    NVIC_EnableIRQ(p_res->IrqNum);
    status |= ARM_USART_FLAG_INITIALIZED;
    p_res->Status.Status |= status;
    return status_ready;
}

uint32_t ARM_USART_Uninit(ARM_USART_Resources_t *p_res)
{
    uint32_t status_ready = ARM_USART_DRIVER_OK;
    uint32_t status = p_res->Status.Status;
    if(status & ARM_USART_FLAG_INITIALIZED) {
        //disable and clear UARTx IRQ
        NVIC_DisableIRQ(p_res->IrqNum);
        NVIC_ClearPendingIRQ(p_res->IrqNum);
        if(p_res->DMA.TxEnable) {
            usart_dma_transmitter_enable(p_res->pUSARTx, FALSE);
            //disable and clear DMAxChany IRQ
            NVIC_DisableIRQ(p_res->DMA.TxIrqNum);
            NVIC_ClearPendingIRQ(p_res->DMA.TxIrqNum);
            dma_channel_enable(p_res->DMA.pTxDMAxChany, FALSE);

        }
        if(p_res->DMA.RxEnable) {
            //disable and clear DMAxChany IRQ
            usart_dma_receiver_enable(p_res->pUSARTx, FALSE);
            NVIC_DisableIRQ(p_res->DMA.RxIrqNum);
            NVIC_ClearPendingIRQ(p_res->DMA.RxIrqNum);
            dma_channel_enable(p_res->DMA.pRxDMAxChany, FALSE);
        }
        usart_transmitter_enable(p_res->pUSARTx, FALSE);
        status &= ~ARM_USART_FLAG_TX_ENABLED;
        usart_receiver_enable(p_res->pUSARTx, FALSE);
        status &= ~ARM_USART_FLAG_RX_ENABLED;
        status_ready |= ARM_USART_GPIO_Config(p_res, FALSE);
        usart_enable(p_res->pUSARTx, FALSE);
        usart_reset(p_res->pUSARTx);
        status &= ~ARM_USART_FLAG_CONFIGURATED;
        if(!(ARM_CRM_USART_ClockEnable(p_res->pUSARTx, FALSE))) {
            return ARM_USART_DRIVER_ERROR;
        }
        status &= ~ARM_USART_FLAG_INITIALIZED;
    } else {
#ifdef _APP_DEBUG_
        LOG("USART not been initialized");
#endif//_APP_DEBUG_
    }
    p_res->Status.Status |= status;
    return status_ready;
}

uint32_t ARM_USART_SetResources(ARM_USART_Resources_t *p_res, usart_type *p_usartx,
                                void *p_event_buff, void *p_tx_buff,
                                void *p_rx_buff, uint32_t BaudRate,
                                usart_data_bit_num_type data_bit,
                                usart_stop_bit_num_type stop_bit,
                                usart_parity_selection_type parity)
{
    p_res->pUSARTx = p_usartx;
    RingBuffer_Init(&(p_res->Event), p_event_buff, ARM_USART_EVENT_BUFF_SIZE);
    p_res->Config.BaudRate = BaudRate;
    p_res->Config.DataBit = data_bit;
    p_res->Config.StopBit = stop_bit;
    p_res->Config.Parity = parity;
    p_res->Status.XferSta.TxBusy = 0;
    p_res->Status.XferSta.RxBusy = 0;
    p_res->Status.XferSta.TxUnderflow = 0;
    p_res->Status.XferSta.RxOverflow = 0;
    p_res->Status.XferSta.RxBreak = 0;
    p_res->Status.XferSta.RxFramingError = 0;
    p_res->Status.XferSta.RxParityError = 0;
    p_res->Transfer.pTxData = p_tx_buff;
    p_res->Transfer.pRxData = p_rx_buff;
    p_res->Transfer.TxNum = 0;
    p_res->Transfer.RxNum = 0;
    p_res->Transfer.TxCnt = 0;
    p_res->Transfer.RxCnt = 0;
    dma_default_para_init(&(p_res->DMA.TxCfg));
    dma_default_para_init(&(p_res->DMA.RxCfg));
    p_res->DMA.TxCfg.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    p_res->DMA.TxCfg.memory_inc_enable = TRUE;
    p_res->DMA.RxCfg.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    p_res->DMA.RxCfg.memory_inc_enable = TRUE;
    if(p_res->pUSARTx == UART4) {
        p_res->IrqNum = UART4_IRQn;
        p_res->Gpio.pTxGpio = GPIOC;
        p_res->Gpio.TxPin = GPIO_PINS_10;
        p_res->Gpio.pRxGpio = GPIOC;
        p_res->Gpio.RxPin = GPIO_PINS_11;
        p_res->DMA.TxIrqNum = DMA2_Channel4_5_IRQn;
        p_res->DMA.RxIrqNum = DMA2_Channel3_IRQn;

#ifdef _UART4_TX_USE_DMA_
        p_res->DMA.TxEnable = TRUE;
#ifdef _UART4_TX_DMA_CIRCULAR_MODE_
        p_res->DMA.TxCfg.loop_mode_enable = TRUE;
#endif //_UART4_TX_DMA_CIRCULAR_MODE_
#else
#ifdef _UART4_TX_DMA_CIRCULAR_MODE_
#error "UART4 DMA configuration error"
#endif //_UART4_TX_DMA_CIRCULAR_MODE_
        p_res->DMA.TxEnable = FALSE;
#endif //_UART4_TX_USE_DMA_

#ifdef _UART4_RX_USE_DMA_
        p_res->DMA.RxEnable = TRUE;
#ifdef _UART4_RX_DMA_CIRCULAR_MODE_
        p_res->DMA.RxCfg.loop_mode_enable = TRUE;
#endif //_UART4_RX_DMA_CIRCULAR_MODE_
#else
#ifdef _UART4_RX_DMA_CIRCULAR_MODE_
#error "UART4 DMA configuration error"
#endif //_UART4_RX_DMA_CIRCULAR_MODE_
        p_res->DMA.RxEnable = FALSE;
#endif //_UART4_RX_USE_DMA_

        p_res->DMA.pTxDMAxChany = DMA2_CHANNEL5;
        p_res->DMA.pRxDMAxChany = DMA2_CHANNEL3;
    } else if(p_res->pUSARTx == UART5) {
        p_res->IrqNum = UART5_IRQn;
        p_res->Gpio.pTxGpio = GPIOC;
        p_res->Gpio.TxPin = GPIO_PINS_12;
        p_res->Gpio.pRxGpio = GPIOD;
        p_res->Gpio.RxPin = GPIO_PINS_2;
        p_res->DMA.TxIrqNum = DMA1_Channel4_IRQn;
        p_res->DMA.RxIrqNum = DMA1_Channel5_IRQn;

#ifdef _UART5_TX_USE_DMA_
        dma_flexible_config(DMA1, FLEX_CHANNEL4, DMA_FLEXIBLE_UART5_TX);
        p_res->DMA.TxEnable = TRUE;
#ifdef _UART5_TX_DMA_CIRCULAR_MODE_
        p_res->DMA.TxCfg.loop_mode_enable = TRUE;
#endif //_UART5_TX_DMA_CIRCULAR_MODE_
#else
#ifdef _UART5_TX_DMA_CIRCULAR_MODE_
#error "UART5 DMA configuration error"
#endif //_UART5_TX_DMA_CIRCULAR_MODE_
        p_res->DMA.TxEnable = FALSE;
#endif //_UART5_TX_USE_DMA_

#ifdef _UART5_RX_USE_DMA_
        dma_flexible_config(DMA1, FLEX_CHANNEL5, DMA_FLEXIBLE_UART5_RX);
        p_res->DMA.RxEnable = TRUE;
#ifdef _UART5_RX_DMA_CIRCULAR_MODE_
        p_res->DMA.RxCfg.loop_mode_enable = TRUE;
#endif //_UART5_RX_DMA_CIRCULAR_MODE_
#else
#ifdef _UART5_RX_DMA_CIRCULAR_MODE_
#error "UART5 DMA configuration error"
#endif //_UART5_RX_DMA_CIRCULAR_MODE_
        p_res->DMA.RxEnable = FALSE;
#endif //_UART5_RX_USE_DMA_

        p_res->DMA.pTxDMAxChany = DMA1_CHANNEL4;
        p_res->DMA.pRxDMAxChany = DMA1_CHANNEL5;
    } else if(p_res->pUSARTx == UART7) {
        p_res->IrqNum = UART7_IRQn;
        p_res->Gpio.pTxGpio = GPIOE;
        p_res->Gpio.TxPin = GPIO_PINS_8;
        p_res->Gpio.pRxGpio = GPIOE;
        p_res->Gpio.RxPin = GPIO_PINS_7;
        p_res->DMA.TxIrqNum = DMA1_Channel2_IRQn;
        p_res->DMA.RxIrqNum = DMA1_Channel3_IRQn;

#ifdef _UART5_TX_USE_DMA_
        dma_flexible_config(DMA1, FLEX_CHANNEL2, DMA_FLEXIBLE_UART7_TX);
        p_res->DMA.TxEnable = TRUE;
#ifdef _UART5_TX_DMA_CIRCULAR_MODE_
        p_res->DMA.TxCfg.loop_mode_enable = TRUE;
#endif //_UART5_TX_DMA_CIRCULAR_MODE_
#else
#ifdef _UART5_TX_DMA_CIRCULAR_MODE_
#error "UART5 DMA configuration error"
#endif //_UART5_TX_DMA_CIRCULAR_MODE_
        p_res->DMA.TxEnable = FALSE;
#endif //_UART5_TX_USE_DMA_

#ifdef _UART7_RX_USE_DMA_
        dma_flexible_config(DMA1, FLEX_CHANNEL3, DMA_FLEXIBLE_UART7_RX);
        p_res->DMA.RxEnable = TRUE;
#ifdef _UART7_RX_DMA_CIRCULAR_MODE_
        p_res->DMA.RxCfg.loop_mode_enable = TRUE;
#endif //_UART7_RX_DMA_CIRCULAR_MODE_
#else
#ifdef _UART7_RX_DMA_CIRCULAR_MODE_
#error "UART7 DMA configuration error"
#endif //_UART7_RX_DMA_CIRCULAR_MODE_
        p_res->DMA.RxEnable = FALSE;
#endif //_UART7_RX_USE_DMA_

        p_res->DMA.pTxDMAxChany = DMA1_CHANNEL2;
        p_res->DMA.pRxDMAxChany = DMA1_CHANNEL3;
    } else if(p_res->pUSARTx == UART8) {
        p_res->IrqNum = UART8_IRQn;
        p_res->Gpio.pTxGpio = GPIOE;
        p_res->Gpio.TxPin = GPIO_PINS_1;
        p_res->Gpio.pRxGpio = GPIOE;
        p_res->Gpio.RxPin = GPIO_PINS_0;
        p_res->DMA.TxIrqNum = DMA2_Channel6_7_IRQn;
        p_res->DMA.RxIrqNum = DMA2_Channel4_5_IRQn;

#ifdef _UART8_TX_USE_DMA_
        dma_flexible_config(DMA2, FLEX_CHANNEL6, DMA_FLEXIBLE_UART8_TX);
        p_res->DMA.TxEnable = TRUE;
#ifdef _UART8_TX_DMA_CIRCULAR_MODE_
        p_res->DMA.TxCfg.loop_mode_enable = TRUE;
#endif //_UART8_TX_DMA_CIRCULAR_MODE_
#else
#ifdef _UART8_TX_DMA_CIRCULAR_MODE_
#error "UART8 DMA configuration error"
#endif //_UART8_TX_DMA_CIRCULAR_MODE_
        p_res->DMA.TxEnable = FALSE;
#endif //_UART8_TX_USE_DMA_

#ifdef _UART8_RX_USE_DMA_
        dma_flexible_config(DMA2, FLEX_CHANNEL4, DMA_FLEXIBLE_UART8_RX);
        p_res->DMA.RxEnable = TRUE;
#ifdef _UART8_RX_DMA_CIRCULAR_MODE_
        p_res->DMA.RxCfg.loop_mode_enable = TRUE;
#endif //_UART8_RX_DMA_CIRCULAR_MODE_
#else
#ifdef _UART8_RX_DMA_CIRCULAR_MODE_
#error "UART8 DMA configuration error"
#endif //_UART8_RX_DMA_CIRCULAR_MODE_
        p_res->DMA.RxEnable = FALSE;
#endif //_UART8_RX_USE_DMA_

        p_res->DMA.pTxDMAxChany = DMA2_CHANNEL6;
        p_res->DMA.pRxDMAxChany = DMA2_CHANNEL4;
    } else {
#ifdef _APP_DEBUG_
        LOG("USART configuration error");
#endif//_APP_DEBUG_
        return ARM_USART_DRIVER_ERROR_UNSUPPORTED;
    }
    RingBuffer_t *pbuff_str_tx = ARM_DMA_GetEventBuffStr(p_res->DMA.pTxDMAxChany);
    p_res->DMA.pTxEvent = pbuff_str_tx->pBuff;
    RingBuffer_t *pbuff_str_rx = ARM_DMA_GetEventBuffStr(p_res->DMA.pRxDMAxChany);
    p_res->DMA.pRxEvent = pbuff_str_rx->pBuff;
    return ARM_USART_DRIVER_OK;
}

void ARM_USART_IRQHandler(ARM_USART_Driver_t *p_drv, ARM_USART_Resources_t *p_res)
{
    uint32_t event = 0;
    if(p_res->Status.XferSta.RxBusy == 1) {
        if(usart_flag_get(p_res->pUSARTx, USART_RDBF_FLAG) == SET) {
            *((uint8_t *)p_res->Transfer.pRxData + p_res->Transfer.RxCnt) = ARM_USART_ReadByte(p_res);
            p_res->Transfer.RxCnt++;
        }
        if(usart_flag_get(p_res->pUSARTx, USART_ROERR_FLAG) == SET) {
            p_res->Status.XferSta.RxOverflow = 1;
            event |= ARM_USART_EVENT_RX_OVERFLOW;
        }
        if(usart_flag_get(p_res->pUSARTx, USART_FERR_FLAG) == SET) {
            p_res->Status.XferSta.RxFramingError = 1;
            event |= ARM_USART_EVENT_RX_FRAMING_ERROR;
        }
        if(usart_flag_get(p_res->pUSARTx, USART_NERR_FLAG) == SET) {
            p_res->Status.XferSta.RxNoiseError = 1;
            event |= ARM_USART_EVENT_RX_NOISE_ERROR;
        }
        if(usart_flag_get(p_res->pUSARTx, USART_PERR_FLAG) == SET) {
            p_res->Status.XferSta.RxParityError = 1;
            event |= ARM_USART_EVENT_RX_PARITY_ERROR;
        }
        if(p_res->Transfer.RxCnt == p_res->Transfer.RxNum) {
            event |= ARM_USART_EVENT_RX_COMPLETE;
            p_res->Status.XferSta.RxBusy = 0;
            usart_interrupt_enable(p_res->pUSARTx, USART_RDBF_INT, FALSE);
            usart_interrupt_enable(p_res->pUSARTx, USART_ERR_INT, FALSE);
            usart_interrupt_enable(p_res->pUSARTx, USART_PERR_INT, FALSE);
        }
    }
    if(p_res->Status.XferSta.TxBusy == 1) {

        if(usart_flag_get(p_res->pUSARTx, USART_TDBE_FLAG) == SET) {
            ARM_USART_WriteByte(p_res,
                                ((uint8_t *)p_res->Transfer.pTxData + p_res->Transfer.TxCnt));
            p_res->Transfer.TxCnt++;
            if(p_res->Transfer.TxCnt == p_res->Transfer.TxNum) {
                while(usart_flag_get(p_res->pUSARTx, USART_TDC_FLAG != SET));
                event |= ARM_USART_EVENT_TX_COMPLETE;
                p_res->Status.XferSta.TxBusy = 0;
                usart_interrupt_enable(p_res->pUSARTx, USART_TDBE_INT, FALSE);
            }
        }
    }
    if(event) {
        RingBuffer_Write(&(p_res->Event), &event);
    }
}

void ARM_USART_cb(ARM_USART_Driver_t *p_drv, ARM_USART_Resources_t *p_res)
{
    uint32_t event = 0;
    uint32_t dma_tx_event = 0;
    uint32_t dma_rx_event = 0;
    RingBuffer_Read(&(p_res->Event), &event);
    if(p_res->DMA.TxEnable) {
        RingBuffer_t *pbuff_str_tx = ARM_DMA_GetEventBuffStr(p_res->DMA.pTxDMAxChany);
        RingBuffer_Read(pbuff_str_tx, &dma_tx_event);
    }
    if(p_res->DMA.RxEnable) {
        RingBuffer_t *pbuff_str_rx = ARM_DMA_GetEventBuffStr(p_res->DMA.pRxDMAxChany);
        RingBuffer_Read(pbuff_str_rx, &dma_rx_event);
    }
    if((event & ARM_USART_EVENT_RX_COMPLETE) || (dma_rx_event & ARM_DMA_EVENT_FULL_DATA)) {
        p_res->Transfer.RxCnt = 0;
        p_res->Transfer.RxNum = 0;
        p_res->Status.XferSta.RxBusy = 0;
        if(p_res->DMA.RxEnable) {
            dma_channel_enable(p_res->DMA.pRxDMAxChany, FALSE);
        }
#ifdef _APP_DEBUG_
        LCD_Printf(0, 0, SET, "%s", p_res->Transfer.pRxData);
        LOG("USART recieved test data");
        memcpy(p_res->Transfer.pTxData, p_res->Transfer.pRxData, ARM_USART_RX_BUFF_SIZE);
        p_drv->Send(p_res->Transfer.pTxData, 8);
#endif//_APP_DEBUG_          
    }
    if((event & ARM_USART_EVENT_TX_COMPLETE) || (dma_tx_event & ARM_DMA_EVENT_FULL_DATA)) {
        p_res->Transfer.TxCnt = 0;
        p_res->Transfer.TxNum = 0;
        p_res->Status.XferSta.TxBusy = 0;
        if(p_res->DMA.TxEnable) {
            dma_channel_enable(p_res->DMA.pTxDMAxChany, FALSE);
        }
#ifdef _APP_DEBUG_
        p_drv->Recieve(p_res->Transfer.pRxData, 8);
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
    if(event & ARM_USART_EVENT_RX_NOISE_ERROR) {
#ifdef _APP_DEBUG_
        LOG("USART receiver noise error");
#endif//_APP_DEBUG_                
    }
    if(event & ARM_USART_EVENT_RX_PARITY_ERROR) {
#ifdef _APP_DEBUG_
        LOG("USART receiver parity error");
#endif//_APP_DEBUG_
    }
    if(dma_tx_event & ARM_DMA_EVENT_DATA_ERROR) {
#ifdef _APP_DEBUG_
        LOG("USART DMA data error");
#endif//_APP_DEBUG_
    }
    if(dma_rx_event & ARM_DMA_EVENT_DATA_ERROR) {
#ifdef _APP_DEBUG_
        LOG("USART DMA data error");
#endif//_APP_DEBUG_
    }
}

uint32_t ARM_USART_Recieve(ARM_USART_Resources_t *p_res, void *pdata, uint32_t num)
{
    if(!(p_res->Status.Status & ARM_USART_FLAG_RX_ENABLED)) {
        return ARM_USART_DRIVER_ERROR;
    }
    if(num == 0) {
        return ARM_USART_DRIVER_ERROR_PARAMETER;
    }
    if(p_res->Status.XferSta.RxBusy) {
        return ARM_USART_DRIVER_ERROR_BUSY;
    }
    p_res->Transfer.RxCnt = 0;
    p_res->Transfer.RxNum = num;
    p_res->Transfer.pRxData = pdata;
    p_res->Status.XferSta.RxBusy = 1;
    if(p_res->DMA.RxEnable) {
        ARM_DMA_Config(p_res->DMA.pRxDMAxChany, &p_res->DMA.RxCfg, p_res->pUSARTx->dt,
                       (uint32_t)pdata, num, DMA_PRIORITY_LOW);
        dma_channel_enable(p_res->DMA.pRxDMAxChany, TRUE);
        dma_interrupt_enable(p_res->DMA.pRxDMAxChany, DMA_FDT_INT, TRUE);
        dma_interrupt_enable(p_res->DMA.pRxDMAxChany, DMA_DTERR_INT, TRUE);
    } else {
        usart_interrupt_enable(p_res->pUSARTx, USART_RDBF_INT, TRUE);
    }
    usart_interrupt_enable(p_res->pUSARTx, USART_ERR_INT, TRUE);
    usart_interrupt_enable(p_res->pUSARTx, USART_PERR_INT, TRUE);

    return ARM_USART_DRIVER_OK;
}

void ARM_USART_WriteByte(ARM_USART_Resources_t *p_res, uint8_t *pByte)
{
    p_res->pUSARTx->dt = *pByte;
}

uint32_t ARM_USART_Send(ARM_USART_Resources_t *p_res, void *pdata, uint32_t num)
{
    if(!(p_res->Status.Status & ARM_USART_FLAG_TX_ENABLED)) {
        return ARM_USART_DRIVER_ERROR;
    }
    if(num == 0) {
        return ARM_USART_DRIVER_ERROR_PARAMETER;
    }
    if(p_res->Status.XferSta.TxBusy) {
        return ARM_USART_DRIVER_ERROR_BUSY;
    }
    p_res->Transfer.TxCnt = 0;
    p_res->Transfer.TxNum = num;
    p_res->Transfer.pTxData = pdata;
    p_res->Status.XferSta.TxBusy = 1;
    if(p_res->DMA.TxEnable) {
        ARM_DMA_Config(p_res->DMA.pTxDMAxChany, &p_res->DMA.TxCfg, p_res->pUSARTx->dt,
                       (uint32_t)pdata, num, DMA_PRIORITY_LOW);
        dma_channel_enable(p_res->DMA.pTxDMAxChany, TRUE);
        dma_interrupt_enable(p_res->DMA.pTxDMAxChany, DMA_FDT_INT, TRUE);
        dma_interrupt_enable(p_res->DMA.pTxDMAxChany, DMA_DTERR_INT, TRUE);
    } else {
        usart_interrupt_enable(p_res->pUSARTx, USART_TDBE_INT, TRUE);
    }

    return ARM_USART_DRIVER_OK;
}

uint8_t ARM_USART_ReadByte(ARM_USART_Resources_t *p_res)
{
    uint8_t byte;
    byte = p_res->pUSARTx->dt;
    return byte;
}

ARM_USART_Status_t ARM_USART_GetStatus(ARM_USART_Resources_t *p_res)
{
    ARM_USART_Status_t status;
    status.Status = p_res->Status.Status;
    status.XferSta.TxBusy = p_res->Status.XferSta.TxBusy;
    status.XferSta.RxBusy = p_res->Status.XferSta.RxBusy;
    status.XferSta.TxUnderflow = p_res->Status.XferSta.TxUnderflow;
    status.XferSta.RxOverflow = p_res->Status.XferSta.RxOverflow;
    status.XferSta.RxBreak = p_res->Status.XferSta.RxBreak;
    status.XferSta.RxFramingError = p_res->Status.XferSta.RxFramingError;
    status.XferSta.RxParityError = p_res->Status.XferSta.RxParityError;
    return status;
}

ARM_USART_Transfer_t ARM_USART_GetTransfer(ARM_USART_Resources_t *p_res)
{
    ARM_USART_Transfer_t transfer;
    transfer.pTxData = p_res->Transfer.pTxData;
    transfer.pRxData = p_res->Transfer.pRxData;
    transfer.TxNum = p_res->Transfer.TxNum;
    transfer.RxNum = p_res->Transfer.RxNum;
    transfer.TxCnt = p_res->Transfer.TxCnt;
    transfer.RxCnt = p_res->Transfer.RxCnt;
    return transfer;
}

//================================================================================
//Private
//================================================================================

static uint32_t ARM_USART_GPIO_Config(ARM_USART_Resources_t *p_res, confirm_state new_state)
{
    if(!ARM_CRM_GPIO_ClockEnable(p_res->Gpio.pTxGpio, TRUE)) {
#ifdef _APP_DEBUG_
        LOG("USART configuration error");
#endif//_APP_DEBUG_        
        return ARM_USART_DRIVER_ERROR;
    }
    if(new_state) {
        ARM_GPIO_Config(p_res->Gpio.pTxGpio, p_res->Gpio.TxPin, GPIO_MODE_MUX, GPIO_OUTPUT_PUSH_PULL,
                        GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_STRONGER);
    } else {
        ARM_GPIO_Config(p_res->Gpio.pTxGpio, p_res->Gpio.TxPin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                        GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
    }

    if(!ARM_CRM_GPIO_ClockEnable(p_res->Gpio.pRxGpio, TRUE)) {
#ifdef _APP_DEBUG_
        LOG("USART configuration error");
#endif//_APP_DEBUG_
        return ARM_USART_DRIVER_ERROR;
    }
    if(new_state) {
        //use any out_type and drive_strength for input i/o:
        ARM_GPIO_Config(p_res->Gpio.pRxGpio, p_res->Gpio.RxPin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                        GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_MODERATE);
    } else {
        ARM_GPIO_Config(p_res->Gpio.pRxGpio, p_res->Gpio.RxPin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                        GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
    }
    return ARM_USART_DRIVER_OK;
}

