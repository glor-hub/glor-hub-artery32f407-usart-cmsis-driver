//********************************************************************************
//arm_uart.c
//********************************************************************************
#include <string.h>
#include "arm_driver.h"
#include "arm_usart.h"
#include "arm_clock.h"
#include "arm_gpio.h"
#include "app.h"
#include "arm_dma.h"
#include "timer.h"

#ifdef _TEST_APP_DEBUG_
#include "assert.h"
#include "LCD_2004.h"
#endif//_TEST_APP_DEBUG_

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

static uint32_t ARM_USART_GPIO_Config(TEST_APP_ARM_USART_Resources_t *p_res, confirm_state new_state);

//================================================================================
//Public
//================================================================================

uint32_t TEST_APP_ARM_USART_Init(TEST_APP_ARM_USART_Resources_t *p_res)
{
    uint32_t drv_status = TEST_APP_ARM_DRIVER_NO_ERROR;
    uint32_t drv_flag = p_res->Status.DrvFlag;
    if(!(TEST_APP_ARM_CRM_USART_ClockEnable(p_res->pUSARTx, TRUE))) {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
        return TEST_APP_ARM_DRIVER_ERROR;
    }
//asynchronous mode is default
    usart_reset(p_res->pUSARTx);
    usart_enable(p_res->pUSARTx, TRUE);
    usart_init(p_res->pUSARTx, p_res->Config.BaudRate, p_res->Config.DataBit, p_res->Config.StopBit);
    usart_parity_selection_config(p_res->pUSARTx, p_res->Config.Parity);
    drv_flag |= TEST_APP_ARM_USART_FLAG_CONFIGURATED;
    if(p_res->DMA.TxEnable) {
        //clock and reset DMA
        if(!TEST_APP_ARM_DMA_Init(p_res->DMA.pTxDMAxChany)) {
#ifdef _TEST_APP_DEBUG_
            LOG("USART DMA driver error");
#endif//_TEST_APP_DEBUG_
            p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
            return TEST_APP_ARM_DRIVER_ERROR;
        }
        if(p_res->DMA.TxFlexModeEnable) {
            dma_flexible_config(p_res->DMA.pTxDMAx, p_res->DMA.TxFlexChannelx,
                                p_res->DMA.TxFlexPeriphReq);
        }
        TEST_APP_RingBuffer_t *pbuff_str_tx = TEST_APP_ARM_DMA_GetEventBuffStr(p_res->DMA.pTxDMAxChany);
        p_res->DMA.pTxEvent = pbuff_str_tx->pBuff;
        //clear and enable DMAxChany IRQ
        NVIC_ClearPendingIRQ(p_res->DMA.TxIrqNum);
        NVIC_EnableIRQ(p_res->DMA.TxIrqNum);
        usart_dma_transmitter_enable(p_res->pUSARTx, TRUE);
    }
    if(p_res->DMA.RxEnable) {
        //clock and reset DMA
        if(!TEST_APP_ARM_DMA_Init(p_res->DMA.pRxDMAxChany)) {
#ifdef _TEST_APP_DEBUG_
            LOG("USART DMA driver error");
#endif//_TEST_APP_DEBUG_
            p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
            return TEST_APP_ARM_DRIVER_ERROR;
        }
        if(p_res->DMA.RxFlexModeEnable) {
            dma_flexible_config(p_res->DMA.pRxDMAx, p_res->DMA.RxFlexChannelx,
                                p_res->DMA.RxFlexPeriphReq);
        }
        TEST_APP_RingBuffer_t *pbuff_str_rx = TEST_APP_ARM_DMA_GetEventBuffStr(p_res->DMA.pRxDMAxChany);
        p_res->DMA.pRxEvent = pbuff_str_rx->pBuff;
        //clear and enable DMAxChany IRQ
        NVIC_ClearPendingIRQ(p_res->DMA.RxIrqNum);
        NVIC_EnableIRQ(p_res->DMA.RxIrqNum);
        usart_dma_receiver_enable(p_res->pUSARTx, TRUE);
    }
    usart_transmitter_enable(p_res->pUSARTx, TRUE);
    drv_flag |= TEST_APP_ARM_USART_FLAG_TX_ENABLED;
    usart_receiver_enable(p_res->pUSARTx, TRUE);
    drv_flag |= TEST_APP_ARM_USART_FLAG_RX_ENABLED;
    drv_status |= ARM_USART_GPIO_Config(p_res, TRUE);
//clear and enable UARTx IRQ
    NVIC_ClearPendingIRQ(p_res->IrqNum);
    NVIC_EnableIRQ(p_res->IrqNum);
    drv_flag |= TEST_APP_ARM_USART_FLAG_INITIALIZED;
    p_res->Status.DrvStatus |= drv_status;
    p_res->Status.DrvFlag |= drv_flag;
    return drv_status;
}


uint32_t TEST_APP_ARM_USART_Uninit(TEST_APP_ARM_USART_Resources_t *p_res)
{
    uint32_t drv_status = TEST_APP_ARM_DRIVER_NO_ERROR;
    uint32_t drv_flag = p_res->Status.DrvFlag;
    if(drv_flag & TEST_APP_ARM_USART_FLAG_INITIALIZED) {
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
        drv_flag &= ~TEST_APP_ARM_USART_FLAG_TX_ENABLED;
        usart_receiver_enable(p_res->pUSARTx, FALSE);
        drv_flag &= ~TEST_APP_ARM_USART_FLAG_RX_ENABLED;
        drv_status |= ARM_USART_GPIO_Config(p_res, FALSE);
        usart_enable(p_res->pUSARTx, FALSE);
        usart_reset(p_res->pUSARTx);
        drv_flag &= ~TEST_APP_ARM_USART_FLAG_CONFIGURATED;
        if(!(TEST_APP_ARM_CRM_USART_ClockEnable(p_res->pUSARTx, FALSE))) {
            p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
            return TEST_APP_ARM_DRIVER_ERROR;
        }
        memset(p_res, 0, sizeof(TEST_APP_ARM_USART_Resources_t));
        drv_flag &= ~TEST_APP_ARM_USART_FLAG_INITIALIZED;
    } else {
#ifdef _TEST_APP_DEBUG_
        LOG("USART not been initialized");
#endif//_TEST_APP_DEBUG_
    }
    p_res->Status.DrvStatus |= drv_status;
    p_res->Status.DrvFlag |= drv_flag;
    return drv_status;
}

uint32_t TEST_APP_ARM_USART_SetResources(TEST_APP_ARM_USART_Resources_t *p_res,
        usart_type *p_usartx,
        void *p_event_buff, void *p_tx_buff,
        void *p_rx_buff, uint32_t BaudRate,
        usart_data_bit_num_type data_bit,
        usart_stop_bit_num_type stop_bit,
        usart_parity_selection_type parity,
        uint32_t gpio_pin_def)
{
    p_res->pUSARTx = p_usartx;
    TEST_APP_RingBuffer_Init(&(p_res->Event), p_event_buff, TEST_APP_ARM_USART_EVENT_BUFF_SIZE);
    p_res->Gpio.PinDef = gpio_pin_def;
    p_res->Config.BaudRate = BaudRate;
    p_res->Config.DataBit = data_bit;
    p_res->Config.StopBit = stop_bit;
    p_res->Config.Parity = parity;
    p_res->Status.DrvStatus = TEST_APP_ARM_DRIVER_NO_ERROR;
    p_res->Status.DrvFlag = 0U;
    p_res->Status.XferStatus.TxBusy = 0;
    p_res->Status.XferStatus.RxBusy = 0;
    p_res->Status.XferStatus.TxUnderflow = 0;
    p_res->Status.XferStatus.RxOverflow = 0;
    p_res->Status.XferStatus.RxBreak = 0;
    p_res->Status.XferStatus.RxFramingError = 0;
    p_res->Status.XferStatus.RxParityError = 0;
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
    /****************************************************************
        UART4
    ***************************************************************/
    if(p_res->pUSARTx == UART4) {
        p_res->IrqNum = UART4_IRQn;
        switch(p_res->Gpio.PinDef) {
            case TEST_APP_ARM_USART_GPIO_PIN_DEF_DEFAULT: {
                p_res->Gpio.pTxGpio = GPIOC;
                p_res->Gpio.TxPin = GPIO_PINS_10;
                p_res->Gpio.pRxGpio = GPIOC;
                p_res->Gpio.RxPin = GPIO_PINS_11;
                break;
            }
            case TEST_APP_ARM_UART4_GPIO_PIN_DEF_REMAP1: {
                p_res->Gpio.pTxGpio = GPIOA;
                p_res->Gpio.TxPin = GPIO_PINS_0;
                p_res->Gpio.pRxGpio = GPIOA;
                p_res->Gpio.RxPin = GPIO_PINS_1;
                break;
            }
            default: {
                break;
            }
        }
        p_res->DMA.TxIrqNum = DMA2_Channel4_5_IRQn;
        p_res->DMA.RxIrqNum = DMA2_Channel3_IRQn;
#ifdef _TEST_APP_UART4_TX_USE_DMA_
        p_res->DMA.TxEnable = TRUE;
#ifdef _TEST_APP_UART4_TX_DMA_CIRCULAR_MODE_
        p_res->DMA.TxCfg.loop_mode_enable = TRUE;
#endif //_TEST_APP_UART4_TX_DMA_CIRCULAR_MODE_
#else
#ifdef _TEST_APP_UART4_TX_DMA_CIRCULAR_MODE_
//dma to be anable
#error "UART4 DMA configuration error"
#endif //_TEST_APP_UART4_TX_DMA_CIRCULAR_MODE_
        p_res->DMA.TxEnable = FALSE;
#endif //_TEST_APP_UART4_TX_USE_DMA_

#ifdef _TEST_APP_UART4_RX_USE_DMA_
        p_res->DMA.RxEnable = TRUE;
#ifdef _TEST_APP_UART4_RX_DMA_CIRCULAR_MODE_
        p_res->DMA.RxCfg.loop_mode_enable = TRUE;
#endif //_TEST_APP_UART4_RX_DMA_CIRCULAR_MODE_
#else
#ifdef _TEST_APP_UART4_RX_DMA_CIRCULAR_MODE_
        //dma to be anable
#error "UART4 DMA configuration error"
#endif //_TEST_APP_UART4_RX_DMA_CIRCULAR_MODE_
        p_res->DMA.RxEnable = FALSE;
#endif //_TEST_APP_UART4_RX_USE_DMA_
        p_res->DMA.pTxDMAx = DMA2;
        p_res->DMA.pRxDMAx = DMA2;
        p_res->DMA.pTxDMAxChany = DMA2_CHANNEL5;
        p_res->DMA.pRxDMAxChany = DMA2_CHANNEL3;
        p_res->DMA.TxFlexModeEnable = TRUE;
        p_res->DMA.RxFlexModeEnable = TRUE;
        p_res->DMA.TxFlexChannelx = FLEX_CHANNEL5;
        p_res->DMA.RxFlexChannelx = FLEX_CHANNEL3;
        p_res->DMA.TxFlexPeriphReq = DMA_FLEXIBLE_UART4_TX;
        p_res->DMA.RxFlexPeriphReq = DMA_FLEXIBLE_UART4_RX;
        /****************************************************************
            UART5
        ***************************************************************/
    } else if(p_res->pUSARTx == UART5) {
        p_res->IrqNum = UART5_IRQn;
        switch(p_res->Gpio.PinDef) {
            case TEST_APP_ARM_USART_GPIO_PIN_DEF_DEFAULT: {
                p_res->Gpio.pTxGpio = GPIOC;
                p_res->Gpio.TxPin = GPIO_PINS_12;
                p_res->Gpio.pRxGpio = GPIOD;
                p_res->Gpio.RxPin = GPIO_PINS_2;
                break;
            }
            case TEST_APP_ARM_UART5_GPIO_PIN_DEF_REMAP1: {
                p_res->Gpio.pTxGpio = GPIOB;
                p_res->Gpio.TxPin = GPIO_PINS_9;
                p_res->Gpio.pRxGpio = GPIOB;
                p_res->Gpio.RxPin = GPIO_PINS_8;
                break;
            }
            default: {
                break;
            }
        }
        p_res->DMA.TxIrqNum = DMA1_Channel5_IRQn;
        p_res->DMA.RxIrqNum = DMA1_Channel4_IRQn;
#ifdef _TEST_APP_UART5_TX_USE_DMA_
        p_res->DMA.TxEnable = TRUE;
#ifdef _TEST_APP_UART5_TX_DMA_CIRCULAR_MODE_
        p_res->DMA.TxCfg.loop_mode_enable = TRUE;
#endif //_TEST_APP_UART5_TX_DMA_CIRCULAR_MODE_
#else
#ifdef _TEST_APP_UART5_TX_DMA_CIRCULAR_MODE_
        //dma to be anable
#error "UART5 DMA configuration error"
#endif //_TEST_APP_UART5_TX_DMA_CIRCULAR_MODE_
        p_res->DMA.TxEnable = FALSE;
#endif //_TEST_APP_UART5_TX_USE_DMA_

#ifdef _TEST_APP_UART5_RX_USE_DMA_
        p_res->DMA.RxEnable = TRUE;
#ifdef _TEST_APP_UART5_RX_DMA_CIRCULAR_MODE_
        p_res->DMA.RxCfg.loop_mode_enable = TRUE;
#endif //_TEST_APP_UART5_RX_DMA_CIRCULAR_MODE_
#else
#ifdef _TEST_APP_UART5_RX_DMA_CIRCULAR_MODE_
        //dma to be anable
#error "UART5 DMA configuration error"
#endif //_TEST_APP_UART5_RX_DMA_CIRCULAR_MODE_
        p_res->DMA.RxEnable = FALSE;
#endif //_TEST_APP_UART5_RX_USE_DMA_
        p_res->DMA.pTxDMAx = DMA1;
        p_res->DMA.pRxDMAx = DMA1;
        p_res->DMA.pTxDMAxChany = DMA1_CHANNEL5;
        p_res->DMA.pRxDMAxChany = DMA1_CHANNEL4;
        p_res->DMA.TxFlexModeEnable = TRUE;
        p_res->DMA.RxFlexModeEnable = TRUE;
        p_res->DMA.TxFlexChannelx = FLEX_CHANNEL5;
        p_res->DMA.RxFlexChannelx = FLEX_CHANNEL4;
        p_res->DMA.TxFlexPeriphReq = DMA_FLEXIBLE_UART5_TX;
        p_res->DMA.RxFlexPeriphReq = DMA_FLEXIBLE_UART5_RX;
        /****************************************************************
            UART7
        ***************************************************************/
    } else if(p_res->pUSARTx == UART7) {
        p_res->IrqNum = UART7_IRQn;
        switch(p_res->Gpio.PinDef) {
            case TEST_APP_ARM_USART_GPIO_PIN_DEF_DEFAULT: {
                p_res->Gpio.pTxGpio = GPIOE;
                p_res->Gpio.TxPin = GPIO_PINS_8;
                p_res->Gpio.pRxGpio = GPIOE;
                p_res->Gpio.RxPin = GPIO_PINS_7;
                break;
            }
            case TEST_APP_ARM_UART7_GPIO_PIN_DEF_REMAP1: {
                p_res->Gpio.pTxGpio = GPIOB;
                p_res->Gpio.TxPin = GPIO_PINS_4;
                p_res->Gpio.pRxGpio = GPIOB;
                p_res->Gpio.RxPin = GPIO_PINS_3;
                break;
            }
            default: {
                break;
            }
        }
        p_res->DMA.TxIrqNum = DMA1_Channel3_IRQn;
        p_res->DMA.RxIrqNum = DMA1_Channel2_IRQn;
#ifdef _TEST_APP_UART7_TX_USE_DMA_
        p_res->DMA.TxEnable = TRUE;
#ifdef _TEST_APP_UART7_TX_DMA_CIRCULAR_MODE_
        p_res->DMA.TxCfg.loop_mode_enable = TRUE;
#endif //_TEST_APP_UART7_TX_DMA_CIRCULAR_MODE_
#else
#ifdef _TEST_APP_UART7_TX_DMA_CIRCULAR_MODE_
        //dma to be anable
#error "UART7 DMA configuration error"
#endif //_TEST_APP_UART7_TX_DMA_CIRCULAR_MODE_
        p_res->DMA.TxEnable = FALSE;
#endif //_TEST_APP_UART7_TX_USE_DMA_
#ifdef _TEST_APP_UART7_RX_USE_DMA_
        p_res->DMA.RxEnable = TRUE;
#ifdef _TEST_APP_UART7_RX_DMA_CIRCULAR_MODE_
        p_res->DMA.RxCfg.loop_mode_enable = TRUE;
#endif //_TEST_APP_UART7_RX_DMA_CIRCULAR_MODE_
#else
#ifdef _TEST_APP_UART7_RX_DMA_CIRCULAR_MODE_
        //dma to be anable
#error "UART7 DMA configuration error"
#endif //_TEST_APP_UART7_RX_DMA_CIRCULAR_MODE_
        p_res->DMA.RxEnable = FALSE;
#endif //_TEST_APP_UART7_RX_USE_DMA_
        p_res->DMA.pTxDMAx = DMA1;
        p_res->DMA.pRxDMAx = DMA1;
        p_res->DMA.pTxDMAxChany = DMA1_CHANNEL3;
        p_res->DMA.pRxDMAxChany = DMA1_CHANNEL2;
        p_res->DMA.TxFlexModeEnable = TRUE;
        p_res->DMA.RxFlexModeEnable = TRUE;
        p_res->DMA.TxFlexChannelx = FLEX_CHANNEL3;
        p_res->DMA.RxFlexChannelx = FLEX_CHANNEL2;
        p_res->DMA.TxFlexPeriphReq = DMA_FLEXIBLE_UART7_TX;
        p_res->DMA.RxFlexPeriphReq = DMA_FLEXIBLE_UART7_RX;
        /****************************************************************
            UART8
        ***************************************************************/
    } else if(p_res->pUSARTx == UART8) {
        p_res->IrqNum = UART8_IRQn;
        switch(p_res->Gpio.PinDef) {
            case TEST_APP_ARM_USART_GPIO_PIN_DEF_DEFAULT: {
                p_res->Gpio.pTxGpio = GPIOE;
                p_res->Gpio.TxPin = GPIO_PINS_1;
                p_res->Gpio.pRxGpio = GPIOE;
                p_res->Gpio.RxPin = GPIO_PINS_0;
                break;
            }
            case TEST_APP_ARM_UART8_GPIO_PIN_DEF_REMAP1: {
                p_res->Gpio.pTxGpio = GPIOC;
                p_res->Gpio.TxPin = GPIO_PINS_2;
                p_res->Gpio.pRxGpio = GPIOC;
                p_res->Gpio.RxPin = GPIO_PINS_3;
                break;
            }
            default: {
                break;
            }
        }
        p_res->DMA.TxIrqNum = DMA2_Channel6_7_IRQn;
        p_res->DMA.RxIrqNum = DMA2_Channel4_5_IRQn;
#ifdef _TEST_APP_UART8_TX_USE_DMA_
        p_res->DMA.TxEnable = TRUE;
#ifdef _TEST_APP_UART8_TX_DMA_CIRCULAR_MODE_
        p_res->DMA.TxCfg.loop_mode_enable = TRUE;
#endif //_TEST_APP_UART8_TX_DMA_CIRCULAR_MODE_
#else
#ifdef _TEST_APP_UART8_TX_DMA_CIRCULAR_MODE_
        //dma to be anable
#error "UART8 DMA configuration error"
#endif //_TEST_APP_UART8_TX_DMA_CIRCULAR_MODE_
        p_res->DMA.TxEnable = FALSE;
#endif //_TEST_APP_UART8_TX_USE_DMA_

#ifdef _TEST_APP_UART8_RX_USE_DMA_
        p_res->DMA.RxEnable = TRUE;
#ifdef _TEST_APP_UART8_RX_DMA_CIRCULAR_MODE_
        p_res->DMA.RxCfg.loop_mode_enable = TRUE;
#endif //_TEST_APP_UART8_RX_DMA_CIRCULAR_MODE_
#else
#ifdef _TEST_APP_UART8_RX_DMA_CIRCULAR_MODE_
        //dma to be anable
#error "UART8 DMA configuration error"
#endif //_TEST_APP_UART8_RX_DMA_CIRCULAR_MODE_
        p_res->DMA.RxEnable = FALSE;
#endif //_TEST_APP_UART8_RX_USE_DMA_
        p_res->DMA.pTxDMAx = DMA2;
        p_res->DMA.pRxDMAx = DMA2;
        p_res->DMA.pTxDMAxChany = DMA2_CHANNEL6;
        p_res->DMA.pRxDMAxChany = DMA2_CHANNEL4;
        p_res->DMA.TxFlexModeEnable = TRUE;
        p_res->DMA.RxFlexModeEnable = TRUE;
        p_res->DMA.TxFlexChannelx = FLEX_CHANNEL6;
        p_res->DMA.RxFlexChannelx = FLEX_CHANNEL4;
        p_res->DMA.TxFlexPeriphReq = DMA_FLEXIBLE_UART8_TX;
        p_res->DMA.RxFlexPeriphReq = DMA_FLEXIBLE_UART8_RX;
    } else {
#ifdef _TEST_APP_DEBUG_
        LOG("USART configuration error");
#endif//_TEST_APP_DEBUG_
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR_UNSUPPORTED;
        return TEST_APP_ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    return TEST_APP_ARM_DRIVER_NO_ERROR;
}

void TEST_APP_ARM_USART_IRQHandler(TEST_APP_ARM_USART_Resources_t *p_res)
{
    uint32_t event = 0;
    if(p_res->Status.XferStatus.RxBusy == 1) {
        if(usart_flag_get(p_res->pUSARTx, USART_RDBF_FLAG) == SET) {
            *((uint8_t *)p_res->Transfer.pRxData + p_res->Transfer.RxCnt) = TEST_APP_ARM_USART_ReadByte(p_res);
            p_res->Transfer.RxCnt++;
        }
        if(usart_flag_get(p_res->pUSARTx, USART_ROERR_FLAG) == SET) {
            p_res->Status.XferStatus.RxOverflow = 1;
            event |= TEST_APP_ARM_USART_EVENT_RX_OVERFLOW;
        }
        if(usart_flag_get(p_res->pUSARTx, USART_FERR_FLAG) == SET) {
            p_res->Status.XferStatus.RxFramingError = 1;
            event |= TEST_APP_ARM_USART_EVENT_RX_FRAMING_ERROR;
        }
        if(usart_flag_get(p_res->pUSARTx, USART_NERR_FLAG) == SET) {
            p_res->Status.XferStatus.RxNoiseError = 1;
            event |= TEST_APP_ARM_USART_EVENT_RX_NOISE_ERROR;
        }
        if(usart_flag_get(p_res->pUSARTx, USART_PERR_FLAG) == SET) {
            p_res->Status.XferStatus.RxParityError = 1;
            event |= TEST_APP_ARM_USART_EVENT_RX_PARITY_ERROR;
        }
        if(p_res->Transfer.RxCnt == p_res->Transfer.RxNum) {
            event |= TEST_APP_ARM_USART_EVENT_RX_COMPLETE;
            usart_interrupt_enable(p_res->pUSARTx, USART_RDBF_INT, FALSE);
            usart_interrupt_enable(p_res->pUSARTx, USART_ERR_INT, FALSE);
            usart_interrupt_enable(p_res->pUSARTx, USART_PERR_INT, FALSE);
            p_res->Status.XferStatus.RxBusy = 0;
        }
    }
    if(p_res->Status.XferStatus.TxBusy == 1) {
        if(usart_flag_get(p_res->pUSARTx, USART_TDBE_FLAG) == SET) {
            TEST_APP_ARM_USART_WriteByte(p_res,
                                         ((uint8_t *)p_res->Transfer.pTxData + p_res->Transfer.TxCnt));
            p_res->Transfer.TxCnt++;
            if(p_res->Transfer.TxCnt == p_res->Transfer.TxNum) {
                while(usart_flag_get(p_res->pUSARTx, USART_TDC_FLAG != SET));
                event |= TEST_APP_ARM_USART_EVENT_TX_COMPLETE;
                usart_interrupt_enable(p_res->pUSARTx, USART_TDBE_INT, FALSE);
                p_res->Status.XferStatus.TxBusy = 0;
            }
        }
    }
    if(event) {
        TEST_APP_RingBuffer_Write(&(p_res->Event), &event);
    }
}

void TEST_APP_ARM_USART_cb(TEST_APP_ARM_USART_Resources_t *p_res)
{
    uint32_t event = 0;
    uint32_t dma_tx_event = 0;
    uint32_t dma_rx_event = 0;
    TEST_APP_RingBuffer_Read(&(p_res->Event), &event);
    if(p_res->DMA.TxEnable) {
        TEST_APP_RingBuffer_t *pbuff_str_tx = TEST_APP_ARM_DMA_GetEventBuffStr(p_res->DMA.pTxDMAxChany);
        TEST_APP_RingBuffer_Read(pbuff_str_tx, &dma_tx_event);
    }
    if(p_res->DMA.RxEnable) {
        TEST_APP_RingBuffer_t *pbuff_str_rx = TEST_APP_ARM_DMA_GetEventBuffStr(p_res->DMA.pRxDMAxChany);
        TEST_APP_RingBuffer_Read(pbuff_str_rx, &dma_rx_event);
    }
    if(event & TEST_APP_ARM_USART_EVENT_RX_COMPLETE) {
        p_res->Transfer.RxCnt = 0;
        p_res->Transfer.RxNum = 0;
        // p_res->Status.XferStatus.RxBusy = 0;
#ifdef _TEST_APP_DEBUG_
        TEST_APP_LCD2004_Printf(0, 0, SET, "%s", p_res->Transfer.pRxData);
        LOG("USART recieved test data");
        memcpy(p_res->Transfer.pTxData, p_res->Transfer.pRxData, TEST_APP_ARM_USART_RX_BUFF_SIZE);
#endif//_TEST_APP_DEBUG_ 
    }
    if((dma_rx_event & TEST_APP_ARM_DMA_EVENT_FULL_DATA) &&
       (p_res->Status.XferStatus.RxBusy == 1)) {
        p_res->Status.XferStatus.RxBusy = 0;
#ifdef _TEST_APP_DEBUG_
        TEST_APP_LCD2004_Printf(0, 0, SET, "%s", p_res->Transfer.pRxData);
        LOG("USART recieved test data via DMA");
        memcpy(p_res->Transfer.pTxData, p_res->Transfer.pRxData, TEST_APP_ARM_USART_RX_BUFF_SIZE);
#endif//_TEST_APP_DEBUG_                
    }
    if(event & TEST_APP_ARM_USART_EVENT_TX_COMPLETE) {
        p_res->Transfer.TxCnt = 0;
        p_res->Transfer.TxNum = 0;
        // p_res->Status.XferStatus.TxBusy = 0;
#ifdef _TEST_APP_DEBUG_
        LOG("USART transmitted test data");
#endif//_TEST_APP_DEBUG_                 
    }
    if((dma_tx_event & TEST_APP_ARM_DMA_EVENT_FULL_DATA) &&
       (p_res->Status.XferStatus.TxBusy == 1)) {
        p_res->Status.XferStatus.TxBusy = 0;
#ifdef _TEST_APP_DEBUG_
        LOG("USART transmitted test data via DMA");
#endif//_TEST_APP_DEBUG_                 
    }
    if(event & TEST_APP_ARM_USART_EVENT_RX_OVERFLOW) {
#ifdef _TEST_APP_DEBUG_
        LOG("USART receiver overflow");
#endif//_TEST_APP_DEBUG_                
    }
    if(event & TEST_APP_ARM_USART_EVENT_RX_FRAMING_ERROR) {
#ifdef _TEST_APP_DEBUG_
        LOG("USART receiver framing error");
#endif//_TEST_APP_DEBUG_                
    }
    if(event & TEST_APP_ARM_USART_EVENT_RX_NOISE_ERROR) {
#ifdef _TEST_APP_DEBUG_
        LOG("USART receiver noise error");
#endif//_TEST_APP_DEBUG_                
    }
    if(event & TEST_APP_ARM_USART_EVENT_RX_PARITY_ERROR) {
#ifdef _TEST_APP_DEBUG_
        LOG("USART receiver parity error");
#endif//_TEST_APP_DEBUG_
    }
    if(dma_tx_event & TEST_APP_ARM_DMA_EVENT_DATA_ERROR) {
#ifdef _TEST_APP_DEBUG_
        LOG("USART DMA data error");
#endif//_TEST_APP_DEBUG_
    }
    if(dma_rx_event & TEST_APP_ARM_DMA_EVENT_DATA_ERROR) {
#ifdef _TEST_APP_DEBUG_
        LOG("USART DMA data error");
#endif//_TEST_APP_DEBUG_
    }
}

uint32_t TEST_APP_ARM_USART_Recieve(TEST_APP_ARM_USART_Resources_t *p_res, void *pdata, uint32_t num)
{
    if(!(p_res->Status.DrvFlag & TEST_APP_ARM_USART_FLAG_RX_ENABLED)) {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
        return TEST_APP_ARM_DRIVER_ERROR;
    }
    if(num == 0) {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR_PARAMETER;
        return TEST_APP_ARM_DRIVER_ERROR_PARAMETER;
    }
    if(p_res->Status.XferStatus.RxBusy)  {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR_BUSY;
        return TEST_APP_ARM_DRIVER_ERROR_BUSY;
    } else {
        p_res->Status.DrvStatus &= ~TEST_APP_ARM_DRIVER_ERROR_BUSY;
    }
    p_res->Status.XferStatus.RxBusy = 1;
    if(p_res->DMA.RxEnable) {
        dma_channel_enable(p_res->DMA.pRxDMAxChany, FALSE);
        TEST_APP_ARM_DMA_Config(p_res->DMA.pRxDMAxChany, &p_res->DMA.RxCfg, (uint32_t)(&p_res->pUSARTx->dt),
                                (uint32_t)(volatile void *)pdata, num, DMA_PRIORITY_LOW);
        dma_channel_enable(p_res->DMA.pRxDMAxChany, TRUE);
        dma_interrupt_enable(p_res->DMA.pRxDMAxChany, DMA_FDT_INT, TRUE);
        dma_interrupt_enable(p_res->DMA.pRxDMAxChany, DMA_DTERR_INT, TRUE);
    } else {
        p_res->Transfer.RxCnt = 0;
        p_res->Transfer.RxNum = num;
        p_res->Transfer.pRxData = pdata;
        usart_interrupt_enable(p_res->pUSARTx, USART_RDBF_INT, TRUE);
    }
    usart_interrupt_enable(p_res->pUSARTx, USART_ERR_INT, TRUE);
    usart_interrupt_enable(p_res->pUSARTx, USART_PERR_INT, TRUE);
    return TEST_APP_ARM_DRIVER_NO_ERROR;
}

void TEST_APP_ARM_USART_WriteByte(TEST_APP_ARM_USART_Resources_t *p_res, uint8_t *pByte)
{
    p_res->pUSARTx->dt = *pByte;
}

uint32_t TEST_APP_ARM_USART_Send(TEST_APP_ARM_USART_Resources_t *p_res, void *pdata, uint32_t num)
{
    if(!(p_res->Status.DrvFlag & TEST_APP_ARM_USART_FLAG_TX_ENABLED)) {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
        return TEST_APP_ARM_DRIVER_ERROR;
    }
    if(num == 0) {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR_PARAMETER;
        return TEST_APP_ARM_DRIVER_ERROR_PARAMETER;
    }

    if(p_res->Status.XferStatus.TxBusy) {
        TimerEnable(TIMER_USART_TIMEOUT, 200);
        while(p_res->Status.XferStatus.TxBusy && (!TimerTestFlag(TIMER_USART_TIMEOUT)));
        if(TimerTestFlag(TIMER_USART_TIMEOUT)) {
            p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR_BUSY;
            return TEST_APP_ARM_DRIVER_ERROR_BUSY;
        } else {
            p_res->Status.DrvStatus &= ~TEST_APP_ARM_DRIVER_ERROR_BUSY;
        }
        TimerDisable(TIMER_USART_TIMEOUT);
    }
    p_res->Status.XferStatus.TxBusy = 1;
    if(p_res->DMA.TxEnable) {
        dma_channel_enable(p_res->DMA.pTxDMAxChany, FALSE);
        TEST_APP_ARM_DMA_Config(p_res->DMA.pTxDMAxChany, &p_res->DMA.TxCfg, (uint32_t)(&p_res->pUSARTx->dt),
                                (uint32_t)(volatile void *)pdata, num, DMA_PRIORITY_LOW);
        dma_channel_enable(p_res->DMA.pTxDMAxChany, TRUE);
        dma_interrupt_enable(p_res->DMA.pTxDMAxChany, DMA_FDT_INT, TRUE);
        dma_interrupt_enable(p_res->DMA.pTxDMAxChany, DMA_DTERR_INT, TRUE);
    } else {
        p_res->Transfer.TxCnt = 0;
        p_res->Transfer.TxNum = num;
        p_res->Transfer.pTxData = pdata;
        usart_interrupt_enable(p_res->pUSARTx, USART_TDBE_INT, TRUE);
    }
    return TEST_APP_ARM_DRIVER_NO_ERROR;
}

uint8_t TEST_APP_ARM_USART_ReadByte(TEST_APP_ARM_USART_Resources_t *p_res)
{
    uint8_t byte;
    byte = p_res->pUSARTx->dt;
    return byte;
}

TEST_APP_ARM_USART_Status_t TEST_APP_ARM_USART_GetStatus(TEST_APP_ARM_USART_Resources_t *p_res)
{
    TEST_APP_ARM_USART_Status_t status;
    status.DrvStatus = p_res->Status.DrvStatus;
    status.DrvFlag = p_res->Status.DrvFlag;
    status.XferStatus.TxBusy = p_res->Status.XferStatus.TxBusy;
    status.XferStatus.RxBusy = p_res->Status.XferStatus.RxBusy;
    status.XferStatus.TxUnderflow = p_res->Status.XferStatus.TxUnderflow;
    status.XferStatus.RxOverflow = p_res->Status.XferStatus.RxOverflow;
    status.XferStatus.RxBreak = p_res->Status.XferStatus.RxBreak;
    status.XferStatus.RxFramingError = p_res->Status.XferStatus.RxFramingError;
    status.XferStatus.RxParityError = p_res->Status.XferStatus.RxParityError;
    return status;
}

void TEST_APP_ARM_USART_SetDefaultTxBuffer(TEST_APP_ARM_USART_Resources_t *p_res, void *pbuff)
{
    p_res->Transfer.pTxData = pbuff;
}

void TEST_APP_ARM_USART_SetDefaultRxBuffer(TEST_APP_ARM_USART_Resources_t *p_res, void *pbuff)
{
    p_res->Transfer.pRxData = pbuff;
}

TEST_APP_ARM_USART_Transfer_t TEST_APP_ARM_USART_GetTransfer(TEST_APP_ARM_USART_Resources_t *p_res)
{
    TEST_APP_ARM_USART_Transfer_t transfer;
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

static uint32_t ARM_USART_GPIO_Config(TEST_APP_ARM_USART_Resources_t *p_res, confirm_state new_state)
{
    if(new_state) {
        if(!TEST_APP_ARM_CRM_GPIO_ClockEnable(p_res->Gpio.pTxGpio, TRUE)) {
#ifdef _TEST_APP_DEBUG_
            LOG("USART configuration error");
#endif//_TEST_APP_DEBUG_        
            p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
            return TEST_APP_ARM_DRIVER_ERROR;
        }
        if(!TEST_APP_ARM_CRM_GPIO_ClockEnable(p_res->Gpio.pRxGpio, TRUE)) {
#ifdef _TEST_APP_DEBUG_
            LOG("USART configuration error");
#endif//_TEST_APP_DEBUG_
            p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
            return TEST_APP_ARM_DRIVER_ERROR;
        }
        //enable GPIO IOMUX clock (for pin remapping)
        if(!(p_res->Gpio.PinDef == TEST_APP_ARM_USART_GPIO_PIN_DEF_DEFAULT)) {
            crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
            gpio_pin_remap_config(p_res->Gpio.PinDef, TRUE);
            //in remap mode configure both Tx and Rx as multiplexed function mode
            TEST_APP_ARM_GPIO_Config(p_res->Gpio.pTxGpio, p_res->Gpio.TxPin, GPIO_MODE_MUX, GPIO_OUTPUT_PUSH_PULL,
                                     GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_STRONGER);
            TEST_APP_ARM_GPIO_Config(p_res->Gpio.pRxGpio, p_res->Gpio.RxPin, GPIO_MODE_MUX, GPIO_OUTPUT_PUSH_PULL,
                                     GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_STRONGER);
        } else {
            //configure Tx as multiplexed function mode
            TEST_APP_ARM_GPIO_Config(p_res->Gpio.pTxGpio, p_res->Gpio.TxPin, GPIO_MODE_MUX, GPIO_OUTPUT_PUSH_PULL,
                                     GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_STRONGER);
            //configure Rx as input, use any out_type and drive_strength for input i/o:
            TEST_APP_ARM_GPIO_Config(p_res->Gpio.pRxGpio, p_res->Gpio.RxPin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                                     GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
        }

    } else {
        //release pins
        TEST_APP_ARM_GPIO_Config(p_res->Gpio.pTxGpio, p_res->Gpio.TxPin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                                 GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
        TEST_APP_ARM_GPIO_Config(p_res->Gpio.pRxGpio, p_res->Gpio.RxPin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                                 GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
        //disable remap (for pins remapping release)
        if(!(p_res->Gpio.PinDef == TEST_APP_ARM_USART_GPIO_PIN_DEF_DEFAULT)) {
            gpio_pin_remap_config(p_res->Gpio.PinDef, FALSE);
        }
    }
    return TEST_APP_ARM_DRIVER_NO_ERROR;
}

