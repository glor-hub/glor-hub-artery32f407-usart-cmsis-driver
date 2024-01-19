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

typedef enum {
    ARM_DMA_GLOBAL_FLAG,
    ARM_DMA_FULL_DATA_FLAG,
    ARM_DMA_HALF_DATA_FLAG,
    ARM_DMA_DATA_ERROR_FLAG,
    ARM_DMA_FLAGS
} eARM_DMA_Flags_t;

typedef enum {
    ARM_DMA_STATUS_REG,
    ARM_DMA_CHANNEL_REG,
    ARM_DMA_REGS
} eARM_DMA_pRegisters_t;

typedef struct {
    uint32_t GlobalFlag;
    uint32_t FullDataFlag;
    uint32_t HalfDataFlag;
    uint32_t DataErrFlag;
} ARM_DMA_FlagsDef_t;

typedef struct {
    dma_type *pStatusReg;
    dma_channel_type *pChannelReg;
} ARM_DMA_pRegisters_t;

// typedef struct {
//     dma_init_type Config;
// } ARM_DMA_Config_t;

//Resources structure for DMA channel
typedef struct {
    IRQn_Type IrqNum;
    //DMA Flexible channel (Artery library definition)
    uint8_t FlexibleChanDef;
    //DMA Flags (Artery library definitions)
    ARM_DMA_FlagsDef_t FlagDef;
    ARM_DMA_pRegisters_t Registers;
    dma_init_type Config;
    void (*Event_cb)(uint32_t event);
} ARM_DMA_Resources_t;


//********************************************************************************
//Prototypes
//********************************************************************************

static void ARM_DMA_SetChanResources(eTEST_APP_ARM_DMA_Chan_t chan);

//********************************************************************************
//Variables
//********************************************************************************

static ARM_DMA_Resources_t ARM_DMA_Resources[TEST_APP_ARM_DMA_CHANS];

static uint8_t ARM_DMA_FlexChan[TEST_APP_ARM_DMA_CHANS] = {
    FLEX_CHANNEL1,
    FLEX_CHANNEL2,
    FLEX_CHANNEL3,
    FLEX_CHANNEL4,
    FLEX_CHANNEL5,
    FLEX_CHANNEL6,
    FLEX_CHANNEL7,
    FLEX_CHANNEL1,
    FLEX_CHANNEL2,
    FLEX_CHANNEL3,
    FLEX_CHANNEL4,
    FLEX_CHANNEL5,
    FLEX_CHANNEL6,
    FLEX_CHANNEL7
};

static uint32_t ARM_DMA_FlagsDef[TEST_APP_ARM_DMA_CHANS][ARM_DMA_FLAGS] = {
    DMA1_GL1_FLAG, DMA1_FDT1_FLAG, DMA1_HDT1_FLAG, DMA1_DTERR1_FLAG,
    DMA1_GL2_FLAG, DMA1_FDT2_FLAG, DMA1_HDT2_FLAG, DMA1_DTERR2_FLAG,
    DMA1_GL3_FLAG, DMA1_FDT3_FLAG, DMA1_HDT3_FLAG, DMA1_DTERR3_FLAG,
    DMA1_GL4_FLAG, DMA1_FDT4_FLAG, DMA1_HDT4_FLAG, DMA1_DTERR4_FLAG,
    DMA1_GL5_FLAG, DMA1_FDT5_FLAG, DMA1_HDT5_FLAG, DMA1_DTERR5_FLAG,
    DMA1_GL6_FLAG, DMA1_FDT6_FLAG, DMA1_HDT6_FLAG, DMA1_DTERR6_FLAG,
    DMA1_GL7_FLAG, DMA1_FDT7_FLAG, DMA1_HDT7_FLAG, DMA1_DTERR7_FLAG,
    DMA2_GL1_FLAG, DMA2_FDT1_FLAG, DMA2_HDT1_FLAG, DMA2_DTERR1_FLAG,
    DMA2_GL2_FLAG, DMA2_FDT2_FLAG, DMA2_HDT2_FLAG, DMA2_DTERR2_FLAG,
    DMA2_GL3_FLAG, DMA2_FDT3_FLAG, DMA2_HDT3_FLAG, DMA2_DTERR3_FLAG,
    DMA2_GL4_FLAG, DMA2_FDT4_FLAG, DMA2_HDT4_FLAG, DMA2_DTERR4_FLAG,
    DMA2_GL5_FLAG, DMA2_FDT5_FLAG, DMA2_HDT5_FLAG, DMA2_DTERR5_FLAG,
    DMA2_GL6_FLAG, DMA2_FDT6_FLAG, DMA2_HDT6_FLAG, DMA2_DTERR6_FLAG,
    DMA2_GL7_FLAG, DMA2_FDT7_FLAG, DMA2_HDT7_FLAG, DMA2_DTERR7_FLAG
};

static ARM_DMA_pRegisters_t ARM_DMA_pRegisters[TEST_APP_ARM_DMA_CHANS] = {
    {DMA1, DMA1_CHANNEL1},
    {DMA1, DMA1_CHANNEL2},
    {DMA1, DMA1_CHANNEL3},
    {DMA1, DMA1_CHANNEL4},
    {DMA1, DMA1_CHANNEL5},
    {DMA1, DMA1_CHANNEL6},
    {DMA1, DMA1_CHANNEL7},
    {DMA2, DMA2_CHANNEL1},
    {DMA2, DMA2_CHANNEL2},
    {DMA2, DMA2_CHANNEL3},
    {DMA2, DMA2_CHANNEL4},
    {DMA2, DMA2_CHANNEL5},
    {DMA2, DMA2_CHANNEL6},
    {DMA2, DMA2_CHANNEL7}
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

bool TEST_APP_ARM_DMA_Init(eTEST_APP_ARM_DMA_Chan_t chan, void (*pfunc_cb)(uint32_t event))
{
    uint32_t drv_status = TEST_APP_ARM_DMA_STA_NO_ERROR;
    ARM_DMA_Resources_t *p_res = &ARM_DMA_Resources[chan];
    ARM_DMA_SetChanResources(chan);
    if(!TEST_APP_ARM_CRM_PeriphClockEnable(TEST_APP_PERIPH_DMA, chan, TRUE)) {
        drv_status |= TEST_APP_ARM_DMA_STA_ERROR;
    }
    dma_reset(p_res->Registers.pChannelReg);
    p_res->Event_cb = pfunc_cb;
    return (drv_status == TEST_APP_ARM_DMA_STA_NO_ERROR);
}

void TEST_APP_ARM_DMA_FlexibleConfig(eTEST_APP_ARM_DMA_Chan_t chan,
                                     dma_flexible_request_type flex_periph_req)
{
    ARM_DMA_Resources_t *p_res = &ARM_DMA_Resources[chan];
    uint8_t flex_chan = ARM_DMA_Resources[chan].FlexibleChanDef;
    dma_flexible_config(p_res->Registers.pStatusReg, flex_chan, flex_periph_req);
}


void TEST_APP_DMA_Enable(eTEST_APP_ARM_DMA_Chan_t chan, confirm_state state)
{
    ARM_DMA_Resources_t *p_res = &ARM_DMA_Resources[chan];
    dma_channel_enable(p_res->Registers.pChannelReg, state);
}

void TEST_APP_DMA_ClearAndEnableIRQ(eTEST_APP_ARM_DMA_Chan_t chan)
{
    ARM_DMA_Resources_t *p_res = &ARM_DMA_Resources[chan];
    IRQn_Type irq_num = p_res->IrqNum;
//clear and enable channel IRQ
    NVIC_ClearPendingIRQ(irq_num);
    NVIC_EnableIRQ(irq_num);
}

void TEST_APP_DMA_DisableAndClearIRQ(eTEST_APP_ARM_DMA_Chan_t chan)
{
    ARM_DMA_Resources_t *p_res = &ARM_DMA_Resources[chan];
    IRQn_Type irq_num = p_res->IrqNum;
//disable and clear channel IRQ
    NVIC_DisableIRQ(irq_num);
    NVIC_ClearPendingIRQ(irq_num);
}

void TEST_APP_ARM_DMA_Config(eTEST_APP_ARM_DMA_Chan_t chan,
                             uint32_t periph_addr, uint32_t mem_addr,
                             uint16_t buff_size, dma_dir_type dir,
                             confirm_state loop_mode_enable,
                             dma_priority_level_type priority)
{
    ARM_DMA_Resources_t *p_res = &ARM_DMA_Resources[chan];
    p_res->Config.memory_inc_enable = TRUE;
    p_res->Config.peripheral_base_addr = periph_addr;
    p_res->Config.memory_base_addr = mem_addr;
    p_res->Config.buffer_size = buff_size;
    p_res->Config.direction = dir;
    p_res->Config.loop_mode_enable = loop_mode_enable;
    p_res->Config.priority = priority;
    dma_init(p_res->Registers.pChannelReg, &(p_res->Config));
}

void TEST_APP_DMA_InterruptEnable(eTEST_APP_ARM_DMA_Chan_t chan,
                                  eTEST_APP_ARM_DMA_InterruptTypes_t interrupt_type,
                                  confirm_state state)
{
    ARM_DMA_Resources_t *p_res = &ARM_DMA_Resources[chan];
    switch(interrupt_type) {
        case TEST_APP_ARM_DMA_FULL_DATA_TRANSFER_INT: {
            dma_interrupt_enable(p_res->Registers.pChannelReg, DMA_FDT_INT, state);
            break;
        }
        case TEST_APP_ARM_DMA_HALF_DATA_TRANSFER_INT: {
            dma_interrupt_enable(p_res->Registers.pChannelReg, DMA_HDT_INT, state);
            break;
        }
        case TEST_APP_ARM_DMA_ERR_DATA_TRANSFER_INT: {
            dma_interrupt_enable(p_res->Registers.pChannelReg, DMA_DTERR_INT, state);
            break;
        }
        default:
            break;

    }
}

void TEST_APP_ARM_DMA_IRQHandlerChannel(eTEST_APP_ARM_DMA_Chan_t chan)
{

    ARM_DMA_Resources_t *p_res = &ARM_DMA_Resources[chan];
    uint32_t event = 0;
    if(dma_flag_get(p_res->FlagDef.FullDataFlag)) {
        event |= TEST_APP_ARM_DMA_EVENT_FULL_DATA;
        if(!p_res->Config.loop_mode_enable) {
            dma_interrupt_enable(p_res->Registers.pChannelReg, DMA_FDT_INT, FALSE);
            dma_interrupt_enable(p_res->Registers.pChannelReg, DMA_DTERR_INT, FALSE);
        } else {
            event |= TEST_APP_ARM_DMA_EVENT_LOOP_MODE;
        }
        dma_flag_clear(p_res->FlagDef.FullDataFlag);
    }
    if(dma_flag_get(p_res->FlagDef.HalfDataFlag)) {
        event |= TEST_APP_ARM_DMA_EVENT_HALF_DATA;
        if(!p_res->Config.loop_mode_enable) {
            dma_interrupt_enable(p_res->Registers.pChannelReg, DMA_HDT_INT, FALSE);
        } else {
            event |= TEST_APP_ARM_DMA_EVENT_LOOP_MODE;
        }
        dma_flag_clear(p_res->FlagDef.HalfDataFlag);
    }
    if(dma_flag_get(p_res->FlagDef.DataErrFlag)) {
        event |= TEST_APP_ARM_DMA_EVENT_DATA_ERROR;
        dma_flag_clear(p_res->FlagDef.DataErrFlag);
    }
    if((event != 0) && (p_res->Event_cb != NULL)) {
        p_res->Event_cb(event);
    }
}

void TEST_APP_ARM_DMA_IRQHandlerChannel_y_z(eTEST_APP_ARM_DMA_Chan_t chany, eTEST_APP_ARM_DMA_Chan_t chanz)
{
    ARM_DMA_Resources_t *p_res_y = &ARM_DMA_Resources[chany];
    ARM_DMA_Resources_t *p_res_z = &ARM_DMA_Resources[chanz];
    uint32_t event_y = 0;
    if(dma_flag_get(p_res_y->FlagDef.FullDataFlag)) {
        event_y |= TEST_APP_ARM_DMA_EVENT_FULL_DATA;
        if(!p_res_y->Config.loop_mode_enable) {
            dma_interrupt_enable(p_res_y->Registers.pChannelReg, DMA_FDT_INT, FALSE);
            dma_interrupt_enable(p_res_y->Registers.pChannelReg, DMA_DTERR_INT, FALSE);
        }

        dma_flag_clear(p_res_y->FlagDef.FullDataFlag);
    }
    if(dma_flag_get(p_res_y->FlagDef.HalfDataFlag)) {
        event_y |= TEST_APP_ARM_DMA_EVENT_HALF_DATA;
        if(!p_res_y->Config.loop_mode_enable) {
            dma_interrupt_enable(p_res_y->Registers.pChannelReg, DMA_HDT_INT, FALSE);
        }
        dma_flag_clear(p_res_y->FlagDef.HalfDataFlag);
    }
    if(dma_flag_get(p_res_y->FlagDef.DataErrFlag)) {
        event_y |= TEST_APP_ARM_DMA_EVENT_DATA_ERROR;
        dma_flag_clear(p_res_y->FlagDef.DataErrFlag);
    }
    if((event_y != 0) && (p_res_y->Event_cb != NULL)) {
        p_res_y->Event_cb(event_y);
    }
    uint32_t event_z = 0;
    if(dma_flag_get(p_res_z->FlagDef.FullDataFlag)) {
        event_z |= TEST_APP_ARM_DMA_EVENT_FULL_DATA;
        if(!p_res_z->Config.loop_mode_enable) {
            dma_interrupt_enable(p_res_z->Registers.pChannelReg, DMA_FDT_INT, FALSE);
            dma_interrupt_enable(p_res_z->Registers.pChannelReg, DMA_DTERR_INT, FALSE);
        }
        dma_flag_clear(p_res_z->FlagDef.FullDataFlag);
    }
    if(dma_flag_get(p_res_z->FlagDef.HalfDataFlag)) {
        event_z |= TEST_APP_ARM_DMA_EVENT_HALF_DATA;
        if(!p_res_z->Config.loop_mode_enable) {
            dma_interrupt_enable(p_res_z->Registers.pChannelReg, DMA_HDT_INT, FALSE);
        }
        dma_flag_clear(p_res_z->FlagDef.HalfDataFlag);
    }
    if(dma_flag_get(p_res_z->FlagDef.DataErrFlag)) {
        event_z |= TEST_APP_ARM_DMA_EVENT_DATA_ERROR;
        dma_flag_clear(p_res_z->FlagDef.DataErrFlag);
    }
    if((event_z != 0) && (p_res_z->Event_cb != NULL)) {
        p_res_z->Event_cb(event_z);
    }
}

//================================================================================
//Private
//================================================================================

static void ARM_DMA_SetChanResources(eTEST_APP_ARM_DMA_Chan_t chan)
{
    ARM_DMA_Resources_t *p_res = &ARM_DMA_Resources[chan];
    p_res->IrqNum = ARM_DMA_IrqNumber[chan];
    p_res->FlexibleChanDef = ARM_DMA_FlexChan[chan];
    p_res->FlagDef.GlobalFlag = ARM_DMA_FlagsDef[chan][ARM_DMA_GLOBAL_FLAG];
    p_res->FlagDef.FullDataFlag = ARM_DMA_FlagsDef[chan][ARM_DMA_FULL_DATA_FLAG];
    p_res->FlagDef.HalfDataFlag = ARM_DMA_FlagsDef[chan][ARM_DMA_HALF_DATA_FLAG];
    p_res->FlagDef.DataErrFlag = ARM_DMA_FlagsDef[chan][ARM_DMA_DATA_ERROR_FLAG];
    p_res->Registers.pStatusReg = ARM_DMA_pRegisters[chan].pStatusReg;
    p_res->Registers.pChannelReg = ARM_DMA_pRegisters[chan].pChannelReg;
    dma_default_para_init(&(p_res->Config));
}
