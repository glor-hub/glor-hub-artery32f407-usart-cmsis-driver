//********************************************************************************
//arm_dma.c
//********************************************************************************
#include <stdlib.h>
#include "arm_dma.h"
#include "arm_clock.h"
#include "app.h"

#ifdef _TEST_APP_DEBUG_
#include "assert.h"
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

typedef void (*Func_cb)(uint32_t event);

typedef struct {
    confirm_state circular_mode;
    //DMA FirmWare Library Flexible channels
    uint8_t FlexibleChanDef;
    //DMA Flags FirmWare Library Definitions
    uint32_t GlobalFlagDef;
    uint32_t FullDataFlagDef;
    uint32_t HalfDataFlagDef;
    uint32_t DataErrFlagDef;
    Func_cb Event_cb;
} ARM_DMA_Resources_t;

typedef struct {
    dma_type *pStatusReg;
    dma_channel_type *pChannelReg;
} ARM_DMA_Registers_t;

//********************************************************************************
//Prototypes
//********************************************************************************


//********************************************************************************
//Variables
//********************************************************************************

static ARM_DMA_Resources_t ARM_DMA_Resources[TEST_APP_ARM_DMA_CHANS] = {
    FALSE, FLEX_CHANNEL1, DMA1_GL1_FLAG, DMA1_FDT1_FLAG, DMA1_HDT1_FLAG, DMA1_DTERR1_FLAG, NULL,
    FALSE, FLEX_CHANNEL2, DMA1_GL2_FLAG, DMA1_FDT2_FLAG, DMA1_HDT2_FLAG, DMA1_DTERR2_FLAG, NULL,
    FALSE, FLEX_CHANNEL3, DMA1_GL3_FLAG, DMA1_FDT3_FLAG, DMA1_HDT3_FLAG, DMA1_DTERR3_FLAG, NULL,
    FALSE, FLEX_CHANNEL4, DMA1_GL4_FLAG, DMA1_FDT4_FLAG, DMA1_HDT4_FLAG, DMA1_DTERR4_FLAG, NULL,
    FALSE, FLEX_CHANNEL5, DMA1_GL5_FLAG, DMA1_FDT5_FLAG, DMA1_HDT5_FLAG, DMA1_DTERR5_FLAG, NULL,
    FALSE, FLEX_CHANNEL6, DMA1_GL6_FLAG, DMA1_FDT6_FLAG, DMA1_HDT6_FLAG, DMA1_DTERR6_FLAG, NULL,
    FALSE, FLEX_CHANNEL7, DMA1_GL7_FLAG, DMA1_FDT7_FLAG, DMA1_HDT7_FLAG, DMA1_DTERR7_FLAG, NULL,
    FALSE, FLEX_CHANNEL1, DMA2_GL1_FLAG, DMA2_FDT1_FLAG, DMA2_HDT1_FLAG, DMA2_DTERR1_FLAG, NULL,
    FALSE, FLEX_CHANNEL2, DMA2_GL2_FLAG, DMA2_FDT2_FLAG, DMA2_HDT2_FLAG, DMA2_DTERR2_FLAG, NULL,
    FALSE, FLEX_CHANNEL3, DMA2_GL3_FLAG, DMA2_FDT3_FLAG, DMA2_HDT3_FLAG, DMA2_DTERR3_FLAG, NULL,
    FALSE, FLEX_CHANNEL4, DMA2_GL4_FLAG, DMA2_FDT4_FLAG, DMA2_HDT4_FLAG, DMA2_DTERR4_FLAG, NULL,
    FALSE, FLEX_CHANNEL5, DMA2_GL5_FLAG, DMA2_FDT5_FLAG, DMA2_HDT5_FLAG, DMA2_DTERR5_FLAG, NULL,
    FALSE, FLEX_CHANNEL6, DMA2_GL6_FLAG, DMA2_FDT6_FLAG, DMA2_HDT6_FLAG, DMA2_DTERR6_FLAG, NULL,
    FALSE, FLEX_CHANNEL7, DMA2_GL7_FLAG, DMA2_FDT7_FLAG, DMA2_HDT7_FLAG, DMA2_DTERR7_FLAG, NULL
};

static ARM_DMA_Registers_t ARM_DMA_Registers[TEST_APP_ARM_DMA_CHANS] = {
    DMA1, DMA1_CHANNEL1,
    DMA1, DMA1_CHANNEL2,
    DMA1, DMA1_CHANNEL3,
    DMA1, DMA1_CHANNEL4,
    DMA1, DMA1_CHANNEL5,
    DMA1, DMA1_CHANNEL6,
    DMA1, DMA1_CHANNEL7,
    DMA2, DMA2_CHANNEL1,
    DMA2, DMA2_CHANNEL2,
    DMA2, DMA2_CHANNEL3,
    DMA2, DMA2_CHANNEL4,
    DMA2, DMA2_CHANNEL5,
    DMA2, DMA2_CHANNEL6,
    DMA2, DMA2_CHANNEL7
};

static IRQn_Type ARM_DMA_IrqNumber[TEST_APP_ARM_DMA_CHANS] = {
    DMA1_Channel1_IRQn,
    DMA1_Channel2_IRQn,
    DMA1_Channel3_IRQn,
    DMA1_Channel4_IRQn,
    DMA1_Channel5_IRQn,
    DMA1_Channel6_IRQn,
    DMA1_Channel7_IRQn,
    DMA2_Channel1_IRQn,
    DMA2_Channel2_IRQn,
    DMA2_Channel3_IRQn,
    DMA2_Channel4_5_IRQn,
    DMA2_Channel4_5_IRQn,
    DMA2_Channel6_7_IRQn,
    DMA2_Channel6_7_IRQn
};

//================================================================================
//Public
//================================================================================

bool TEST_APP_ARM_DMA_Init(eTEST_APP_ARM_DMA_Chan_t chan)
{
    dma_channel_type *pDMAxChany = ARM_DMA_Registers[chan].pChannelReg;
    uint32_t drv_status = TEST_APP_ARM_DMA_STA_NO_ERROR;
    if(!TEST_APP_ARM_CRM_DMA_ClockEnable(pDMAxChany, TRUE)) {
        drv_status |= TEST_APP_ARM_DMA_STA_ERROR;
    }
    dma_reset(pDMAxChany);
    return (drv_status == TEST_APP_ARM_DMA_STA_NO_ERROR);
}

void TEST_APP_ARM_DMA_FlexibleConfig(eTEST_APP_ARM_DMA_Chan_t chan,
                                     dma_flexible_request_type flex_periph_req)
{
    dma_type *pDMAx = ARM_DMA_Registers[chan].pStatusReg;
    uint8_t flex_chan = ARM_DMA_Resources[chan].FlexibleChanDef;
    dma_flexible_config(pDMAx, flex_chan, flex_periph_req);
}


void TEST_APP_DMA_Enable(eTEST_APP_ARM_DMA_Chan_t chan, confirm_state state)
{
    dma_channel_type *pDMAxChany = ARM_DMA_Registers[chan].pChannelReg;
    if(state) {
        dma_channel_enable(pDMAxChany, TRUE);
    } else {
        dma_channel_enable(pDMAxChany, FALSE);
    }
}

void TEST_APP_DMA_ClearAndEnableIRQ(eTEST_APP_ARM_DMA_Chan_t chan)
{
    IRQn_Type irq_num = ARM_DMA_IrqNumber[chan];
//clear and enable channel IRQ
    NVIC_ClearPendingIRQ(irq_num);
    NVIC_EnableIRQ(irq_num);
}

void TEST_APP_DMA_DisableAndClearIRQ(eTEST_APP_ARM_DMA_Chan_t chan)
{
    IRQn_Type irq_num = ARM_DMA_IrqNumber[chan];
//disable and clear channel IRQ
    NVIC_DisableIRQ(irq_num);
    NVIC_ClearPendingIRQ(irq_num);
}

void TEST_APP_ARM_DMA_Config(eTEST_APP_ARM_DMA_Chan_t chan, dma_init_type *pDMA_Cfg,
                             uint32_t periph_addr, uint32_t mem_addr,
                             uint16_t buff_size, dma_priority_level_type priority,
                             void (*func_cb)(uint32_t event))
{
    ARM_DMA_Resources_t *p_res = &ARM_DMA_Resources[chan];
    dma_channel_type *pDMAxChany = ARM_DMA_Registers[chan].pChannelReg;
    if(pDMA_Cfg->loop_mode_enable) {
        p_res->circular_mode = TRUE;
    } else {
        p_res->circular_mode = FALSE;
    }
    pDMA_Cfg->peripheral_base_addr = periph_addr;
    pDMA_Cfg->memory_base_addr = mem_addr;
    pDMA_Cfg->buffer_size = buff_size;
    pDMA_Cfg->priority = priority;
    dma_init(pDMAxChany, pDMA_Cfg);
    p_res->Event_cb = func_cb;
}

void TEST_APP_DMA_InterruptEnable(eTEST_APP_ARM_DMA_Chan_t chan,
                                  eTEST_APP_ARM_DMA_InterruptTypes_t interrupt_type,
                                  confirm_state state)
{
    dma_channel_type *pDMAxChany = ARM_DMA_Registers[chan].pChannelReg;
    switch(interrupt_type) {
        case TEST_APP_ARM_DMA_FULL_DATA_TRANSFER_INT: {
            dma_interrupt_enable(pDMAxChany, DMA_FDT_INT, state);
        }
        case TEST_APP_ARM_DMA_HALF_DATA_TRANSFER_INT: {
            dma_interrupt_enable(pDMAxChany, DMA_HDT_INT, state);
        }
        case TEST_APP_ARM_DMA_ERR_DATA_TRANSFER_INT: {
            dma_interrupt_enable(pDMAxChany, DMA_DTERR_INT, state);
        }

    }
}

void TEST_APP_ARM_DMA_IRQHandlerChannel(eTEST_APP_ARM_DMA_Chan_t chan)
{
    ARM_DMA_Resources_t *p_res = &ARM_DMA_Resources[chan];
    dma_channel_type *pDMAxChany = ARM_DMA_Registers[chan].pChannelReg;
    uint32_t event = 0;
    if(dma_flag_get(p_res->FullDataFlagDef)) {
        event |= TEST_APP_ARM_DMA_EVENT_FULL_DATA;
        if(!p_res->circular_mode) {
            dma_interrupt_enable(pDMAxChany, DMA_FDT_INT, FALSE);
            dma_interrupt_enable(pDMAxChany, DMA_DTERR_INT, FALSE);
        }
        dma_flag_clear(p_res->FullDataFlagDef);
    }
    if(dma_flag_get(p_res->HalfDataFlagDef)) {
        event |= TEST_APP_ARM_DMA_EVENT_HALF_DATA;
        if(!p_res->circular_mode) {
            dma_interrupt_enable(pDMAxChany, DMA_HDT_INT, FALSE);
        }
        dma_flag_clear(p_res->HalfDataFlagDef);
    }
    if(dma_flag_get(p_res->DataErrFlagDef)) {
        event |= TEST_APP_ARM_DMA_EVENT_DATA_ERROR;
        dma_flag_clear(p_res->DataErrFlagDef);
    }
    if((event != 0) && (p_res->Event_cb != NULL)) {
        p_res->Event_cb(event);
    }
}

void TEST_APP_ARM_DMA_IRQHandlerChannel_y_z(eTEST_APP_ARM_DMA_Chan_t chany, eTEST_APP_ARM_DMA_Chan_t chanz)
{
    ARM_DMA_Resources_t *p_res_y = &ARM_DMA_Resources[chany];
    ARM_DMA_Resources_t *p_res_z = &ARM_DMA_Resources[chanz];
    dma_channel_type *pDMAxChany = ARM_DMA_Registers[chany].pChannelReg;
    dma_channel_type *pDMAxChanz = ARM_DMA_Registers[chanz].pChannelReg;
    uint32_t event_y = 0;
    if(dma_flag_get(p_res_y->FullDataFlagDef)) {
        event_y |= TEST_APP_ARM_DMA_EVENT_FULL_DATA;
        if(!p_res_y->circular_mode) {
            dma_interrupt_enable(pDMAxChany, DMA_FDT_INT, FALSE);
            dma_interrupt_enable(pDMAxChany, DMA_DTERR_INT, FALSE);
        }

        dma_flag_clear(p_res_y->FullDataFlagDef);
    }
    if(dma_flag_get(p_res_y->HalfDataFlagDef)) {
        event_y |= TEST_APP_ARM_DMA_EVENT_HALF_DATA;
        if(!p_res_y->circular_mode) {
            dma_interrupt_enable(pDMAxChany, DMA_HDT_INT, FALSE);
        }
        dma_flag_clear(p_res_y->HalfDataFlagDef);
    }
    if(dma_flag_get(p_res_y->DataErrFlagDef)) {
        event_y |= TEST_APP_ARM_DMA_EVENT_DATA_ERROR;
        dma_flag_clear(p_res_y->DataErrFlagDef);
    }
    if((event_y != 0) && (p_res_y->Event_cb != NULL)) {
        p_res_y->Event_cb(event_y);
    }
    uint32_t event_z = 0;
    if(dma_flag_get(p_res_z->FullDataFlagDef)) {
        event_z |= TEST_APP_ARM_DMA_EVENT_FULL_DATA;
        if(!p_res_z->circular_mode) {
            dma_interrupt_enable(pDMAxChanz, DMA_FDT_INT, FALSE);
            dma_interrupt_enable(pDMAxChanz, DMA_DTERR_INT, FALSE);
        }
        dma_flag_clear(p_res_z->FullDataFlagDef);
    }
    if(dma_flag_get(p_res_z->HalfDataFlagDef)) {
        event_z |= TEST_APP_ARM_DMA_EVENT_HALF_DATA;
        if(!p_res_z->circular_mode) {
            dma_interrupt_enable(pDMAxChanz, DMA_HDT_INT, FALSE);
        }
        dma_flag_clear(p_res_z->HalfDataFlagDef);
    }
    if(dma_flag_get(p_res_z->DataErrFlagDef)) {
        event_z |= TEST_APP_ARM_DMA_EVENT_DATA_ERROR;
        dma_flag_clear(p_res_z->DataErrFlagDef);
    }
    if((event_z != 0) && (p_res_z->Event_cb != NULL)) {
        p_res_z->Event_cb(event_z);
    }
}

//================================================================================
//Private
//================================================================================

