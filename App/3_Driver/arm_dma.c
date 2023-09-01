//********************************************************************************
//arm_dma.c
//********************************************************************************
#include "arm_dma.h"
#include "arm_clock.h"
#include "app.h"

#ifdef _APP_DEBUG_
#include "assert.h"
#endif//_APP_DEBUG_

//********************************************************************************
//Macros
//********************************************************************************

#define ARM_DMA_EVENT_BUFF_SIZE 8

//********************************************************************************
//Enums
//********************************************************************************

//********************************************************************************
//Typedefs
//********************************************************************************

typedef struct {
//DMA Flag Definitions
    uint32_t GlobalFlagDef;
    uint32_t FullDataFlagDef;
    uint32_t HalfDataFlagDef;
    uint32_t DataErrFlagDef;
    void *pEventBuff;
    RingBuffer_t *pEventBuffStr;
} ARM_DMA_Resources_t;

//********************************************************************************
//Variables
//********************************************************************************

uint32_t DMA1_EventBuff[DMA1_CHANS][ARM_DMA_EVENT_BUFF_SIZE];
uint32_t DMA2_EventBuff[DMA2_CHANS][ARM_DMA_EVENT_BUFF_SIZE];

RingBuffer_t DMA1_EventBuffStr[DMA1_CHANS];
RingBuffer_t DMA2_EventBuffStr[DMA2_CHANS];

ARM_DMA_Resources_t ARM_DMA1_Resources[DMA1_CHANS];
ARM_DMA_Resources_t ARM_DMA2_Resources[DMA2_CHANS];

//********************************************************************************
//Prototypes
//********************************************************************************

static ARM_DMA_Resources_t *ARM_DMA_GetResourcesStr(dma_channel_type *pDMAxChan_y);
static bool ARM_DMA_SetResources(dma_channel_type *pDMAxChan_y);

//================================================================================
//Public
//================================================================================

bool ARM_DMA_Init(dma_channel_type *pDMAxChan_y)
{
    uint32_t drv_status = ARM_DMA_STA_READY;
    if(!ARM_CRM_DMA_ClockEnable(pDMAxChan_y, TRUE)) {
        drv_status |= ARM_DMA_STA_ERROR;
    }
    dma_reset(pDMAxChan_y);
    if(!ARM_DMA_SetResources(pDMAxChan_y)) {
        drv_status |= ARM_DMA_STA_ERROR;
    }
    ARM_DMA_Resources_t *p_res = ARM_DMA_GetResourcesStr(pDMAxChan_y);
    RingBuffer_Init(p_res->pEventBuffStr, p_res->pEventBuff,
                    ARM_DMA_EVENT_BUFF_SIZE);
    return (drv_status == ARM_DMA_STA_READY);
}

RingBuffer_t *ARM_DMA_GetEventBuffStr(dma_channel_type *pDMAxChan_y)
{
    ARM_DMA_Resources_t *p_res = ARM_DMA_GetResourcesStr(pDMAxChan_y);
    return p_res->pEventBuffStr;
}

void ARM_DMA_Config(dma_channel_type *pDMAxChan_y, dma_init_type *pDMA_Cfg, uint32_t periph_addr,
                    uint32_t mem_addr, uint16_t buff_size, dma_priority_level_type priority)
{
    pDMA_Cfg->peripheral_base_addr = periph_addr;
    pDMA_Cfg->memory_base_addr = mem_addr;
    pDMA_Cfg->buffer_size = buff_size;
    pDMA_Cfg->priority = priority;
    dma_init(pDMAxChan_y, pDMA_Cfg);
}

void ARM_DMA_IRQHandlerChannel(dma_channel_type *pDMAxChan_y)
{
    ARM_DMA_Resources_t *p_res = ARM_DMA_GetResourcesStr(pDMAxChan_y);
    uint32_t event = 0;
    if(dma_flag_get(p_res->FullDataFlagDef)) {
        event |= ARM_DMA_EVENT_FULL_DATA;
        dma_interrupt_enable(pDMAxChan_y, DMA_FDT_INT, FALSE);
        dma_interrupt_enable(pDMAxChan_y, DMA_DTERR_INT, FALSE);
        dma_flag_clear(p_res->FullDataFlagDef);
    }
    if(dma_flag_get(p_res->HalfDataFlagDef)) {
        event |= ARM_DMA_EVENT_HALF_DATA;
        dma_interrupt_enable(pDMAxChan_y, DMA_FDT_INT, FALSE);
        dma_interrupt_enable(pDMAxChan_y, DMA_FDT_INT, FALSE);
        dma_flag_clear(p_res->HalfDataFlagDef);
    }
    if(dma_flag_get(p_res->DataErrFlagDef)) {
        event |= ARM_DMA_EVENT_DATA_ERROR;
        dma_flag_clear(p_res->DataErrFlagDef);
    }
    if(event) {
        RingBuffer_Write(p_res->pEventBuffStr, &event);
    }
}

void ARM_DMA_IRQHandlerChannel_y_z(dma_channel_type *pDMAxChan_y, dma_channel_type *pDMAxChan_z)
{
    ARM_DMA_Resources_t *p_res_y = ARM_DMA_GetResourcesStr(pDMAxChan_y);
    ARM_DMA_Resources_t *p_res_z = ARM_DMA_GetResourcesStr(pDMAxChan_y);
    uint32_t event_y = 0;
    if(dma_flag_get(p_res_y->FullDataFlagDef)) {
        event_y |= ARM_DMA_EVENT_FULL_DATA;
        dma_interrupt_enable(pDMAxChan_y, DMA_FDT_INT, FALSE);
        dma_interrupt_enable(pDMAxChan_y, DMA_DTERR_INT, TRUE);
        dma_flag_clear(p_res_y->FullDataFlagDef);
    }
    if(dma_flag_get(p_res_y->HalfDataFlagDef)) {
        event_y |= ARM_DMA_EVENT_HALF_DATA;
        dma_interrupt_enable(pDMAxChan_y, DMA_FDT_INT, FALSE);
        dma_interrupt_enable(pDMAxChan_y, DMA_FDT_INT, TRUE);
        dma_flag_clear(p_res_y->HalfDataFlagDef);
    }
    if(dma_flag_get(p_res_y->DataErrFlagDef)) {
        event_y |= ARM_DMA_EVENT_DATA_ERROR;
        dma_flag_clear(p_res_y->DataErrFlagDef);
    }
    if(event_y) {
        RingBuffer_Write(p_res_y->pEventBuffStr, &event_y);
    }
    uint32_t event_z = 0;
    if(dma_flag_get(p_res_z->FullDataFlagDef)) {
        event_z |= ARM_DMA_EVENT_FULL_DATA;
        dma_interrupt_enable(pDMAxChan_y, DMA_FDT_INT, FALSE);
        dma_interrupt_enable(pDMAxChan_y, DMA_DTERR_INT, TRUE);
        dma_flag_clear(p_res_z->FullDataFlagDef);
    }
    if(dma_flag_get(p_res_z->HalfDataFlagDef)) {
        event_z |= ARM_DMA_EVENT_HALF_DATA;
        dma_interrupt_enable(pDMAxChan_y, DMA_FDT_INT, FALSE);
        dma_interrupt_enable(pDMAxChan_y, DMA_FDT_INT, TRUE);
        dma_flag_clear(p_res_z->HalfDataFlagDef);
    }
    if(dma_flag_get(p_res_z->DataErrFlagDef)) {
        event_z |= ARM_DMA_EVENT_DATA_ERROR;
        dma_flag_clear(p_res_z->DataErrFlagDef);
    }
    if(event_z) {
        RingBuffer_Write(p_res_z->pEventBuffStr, &event_z);
    }
}

//================================================================================
//Private
//================================================================================

static ARM_DMA_Resources_t *ARM_DMA_GetResourcesStr(dma_channel_type *pDMAxChan_y)
{
    ARM_DMA_Resources_t *p_res;
    if(pDMAxChan_y == DMA1_CHANNEL1) {
        p_res = &ARM_DMA1_Resources[DMA1_CHAN1];
    } else if(pDMAxChan_y == DMA1_CHANNEL2) {
        p_res = &ARM_DMA1_Resources[DMA1_CHAN2];
    } else if(pDMAxChan_y == DMA1_CHANNEL3) {
        p_res = &ARM_DMA1_Resources[DMA1_CHAN3];
    } else if(pDMAxChan_y == DMA1_CHANNEL4) {
        p_res = &ARM_DMA1_Resources[DMA1_CHAN4];
    } else if(pDMAxChan_y == DMA1_CHANNEL5) {
        p_res = &ARM_DMA1_Resources[DMA1_CHAN5];
    } else if(pDMAxChan_y == DMA1_CHANNEL6) {
        p_res = &ARM_DMA1_Resources[DMA1_CHAN6];
    } else if(pDMAxChan_y == DMA1_CHANNEL7) {
        p_res = &ARM_DMA1_Resources[DMA1_CHAN7];
    } else if(pDMAxChan_y == DMA2_CHANNEL1) {
        p_res = &ARM_DMA2_Resources[DMA2_CHAN1];
    } else if(pDMAxChan_y == DMA2_CHANNEL2) {
        p_res = &ARM_DMA2_Resources[DMA2_CHAN2];
    } else if(pDMAxChan_y == DMA2_CHANNEL3) {
        p_res = &ARM_DMA2_Resources[DMA2_CHAN3];
    } else if(pDMAxChan_y == DMA2_CHANNEL4) {
        p_res = &ARM_DMA2_Resources[DMA2_CHAN4];
    } else if(pDMAxChan_y == DMA2_CHANNEL5) {
        p_res = &ARM_DMA2_Resources[DMA2_CHAN5];
    } else if(pDMAxChan_y == DMA2_CHANNEL6) {
        p_res = &ARM_DMA2_Resources[DMA2_CHAN6];
    } else if(pDMAxChan_y == DMA2_CHANNEL7) {
        p_res = &ARM_DMA2_Resources[DMA2_CHAN7];
    } else {
#ifdef _APP_DEBUG_
        LOG("Invalid DMA Configuration");
#endif//_APP_DEBUG_        
    }
    return p_res;
}

static bool ARM_DMA_SetResources(dma_channel_type *pDMAxChan_y)
{
    uint32_t drv_status = ARM_DMA_STA_READY;
    ARM_DMA_Resources_t *p_res = ARM_DMA_GetResourcesStr(pDMAxChan_y);
    if(pDMAxChan_y == DMA1_CHANNEL1) {
        p_res->pEventBuffStr = &DMA1_EventBuffStr[DMA1_CHAN1];
        p_res->pEventBuff = &DMA1_EventBuff[DMA1_CHAN1];
        p_res->GlobalFlagDef = DMA1_GL1_FLAG;
        p_res->FullDataFlagDef = DMA1_FDT1_FLAG;
        p_res->HalfDataFlagDef = DMA1_HDT1_FLAG;
        p_res->DataErrFlagDef = DMA1_DTERR1_FLAG;
    } else if(pDMAxChan_y == DMA1_CHANNEL2) {
        p_res->pEventBuffStr = &DMA1_EventBuffStr[DMA1_CHAN2];
        p_res->pEventBuff = &DMA1_EventBuff[DMA1_CHAN2];
        p_res->GlobalFlagDef = DMA1_GL2_FLAG;
        p_res->FullDataFlagDef = DMA1_FDT2_FLAG;
        p_res->HalfDataFlagDef = DMA1_HDT2_FLAG;
        p_res->DataErrFlagDef = DMA1_DTERR2_FLAG;
    } else if(pDMAxChan_y == DMA1_CHANNEL3) {
        p_res->pEventBuffStr = &DMA1_EventBuffStr[DMA1_CHAN1];
        p_res->pEventBuff = &DMA1_EventBuff[DMA1_CHAN1];
        p_res->GlobalFlagDef = DMA1_GL3_FLAG;
        p_res->FullDataFlagDef = DMA1_FDT3_FLAG;
        p_res->HalfDataFlagDef = DMA1_HDT3_FLAG;
        p_res->DataErrFlagDef = DMA1_DTERR3_FLAG;
    } else if(pDMAxChan_y == DMA1_CHANNEL4) {
        p_res->pEventBuffStr = &DMA1_EventBuffStr[DMA1_CHAN1];
        p_res->pEventBuff = &DMA1_EventBuff[DMA1_CHAN1];
        p_res->GlobalFlagDef = DMA1_GL4_FLAG;
        p_res->FullDataFlagDef = DMA1_FDT4_FLAG;
        p_res->HalfDataFlagDef = DMA1_HDT4_FLAG;
        p_res->DataErrFlagDef = DMA1_DTERR4_FLAG;
    } else if(pDMAxChan_y == DMA1_CHANNEL5) {
        p_res->pEventBuffStr = &DMA1_EventBuffStr[DMA1_CHAN1];
        p_res->pEventBuff = &DMA1_EventBuff[DMA1_CHAN1];
        p_res->GlobalFlagDef = DMA1_GL5_FLAG;
        p_res->FullDataFlagDef = DMA1_FDT5_FLAG;
        p_res->HalfDataFlagDef = DMA1_HDT5_FLAG;
        p_res->DataErrFlagDef = DMA1_DTERR5_FLAG;
    } else if(pDMAxChan_y == DMA1_CHANNEL6) {
        p_res->pEventBuffStr = &DMA1_EventBuffStr[DMA1_CHAN1];
        p_res->pEventBuff = &DMA1_EventBuff[DMA1_CHAN1];
        p_res->GlobalFlagDef = DMA1_GL6_FLAG;
        p_res->FullDataFlagDef = DMA1_FDT6_FLAG;
        p_res->HalfDataFlagDef = DMA1_HDT6_FLAG;
        p_res->DataErrFlagDef = DMA1_DTERR6_FLAG;
    } else if(pDMAxChan_y == DMA1_CHANNEL7) {
        p_res->pEventBuffStr = &DMA1_EventBuffStr[DMA1_CHAN1];
        p_res->pEventBuff = &DMA1_EventBuff[DMA1_CHAN1];
        p_res->GlobalFlagDef = DMA1_GL7_FLAG;
        p_res->FullDataFlagDef = DMA1_FDT7_FLAG;
        p_res->HalfDataFlagDef = DMA1_HDT7_FLAG;
        p_res->DataErrFlagDef = DMA1_DTERR7_FLAG;
    } else if(pDMAxChan_y == DMA2_CHANNEL1) {
        p_res->pEventBuffStr = &DMA2_EventBuffStr[DMA2_CHAN1];
        p_res->pEventBuff = &DMA2_EventBuff[DMA2_CHAN1];
        p_res->GlobalFlagDef = DMA2_GL1_FLAG;
        p_res->FullDataFlagDef = DMA2_FDT1_FLAG;
        p_res->HalfDataFlagDef = DMA2_HDT1_FLAG;
        p_res->DataErrFlagDef = DMA2_DTERR1_FLAG;
    } else if(pDMAxChan_y == DMA2_CHANNEL2) {
        p_res->pEventBuffStr = &DMA2_EventBuffStr[DMA2_CHAN2];
        p_res->pEventBuff = &DMA2_EventBuff[DMA2_CHAN2];
        p_res->GlobalFlagDef = DMA2_GL2_FLAG;
        p_res->FullDataFlagDef = DMA2_FDT2_FLAG;
        p_res->HalfDataFlagDef = DMA2_HDT2_FLAG;
        p_res->DataErrFlagDef = DMA2_DTERR2_FLAG;
    } else if(pDMAxChan_y == DMA2_CHANNEL3) {
        p_res->pEventBuffStr = &DMA2_EventBuffStr[DMA2_CHAN3];
        p_res->pEventBuff = &DMA2_EventBuff[DMA2_CHAN3];
        p_res->GlobalFlagDef = DMA2_GL3_FLAG;
        p_res->FullDataFlagDef = DMA2_FDT3_FLAG;
        p_res->HalfDataFlagDef = DMA2_HDT3_FLAG;
        p_res->DataErrFlagDef = DMA2_DTERR3_FLAG;
    } else if(pDMAxChan_y == DMA2_CHANNEL4) {
        p_res->pEventBuffStr = &DMA2_EventBuffStr[DMA2_CHAN4];
        p_res->pEventBuff = &DMA2_EventBuff[DMA2_CHAN4];
        p_res->GlobalFlagDef = DMA2_GL4_FLAG;
        p_res->FullDataFlagDef = DMA2_FDT4_FLAG;
        p_res->HalfDataFlagDef = DMA2_HDT4_FLAG;
        p_res->DataErrFlagDef = DMA2_DTERR4_FLAG;
    } else if(pDMAxChan_y == DMA2_CHANNEL5) {
        p_res->pEventBuffStr = &DMA2_EventBuffStr[DMA2_CHAN5];
        p_res->pEventBuff = &DMA2_EventBuff[DMA2_CHAN5];
        p_res->GlobalFlagDef = DMA2_GL5_FLAG;
        p_res->FullDataFlagDef = DMA2_FDT5_FLAG;
        p_res->HalfDataFlagDef = DMA2_HDT5_FLAG;
        p_res->DataErrFlagDef = DMA2_DTERR5_FLAG;
    } else if(pDMAxChan_y == DMA2_CHANNEL6) {
        p_res->pEventBuffStr = &DMA2_EventBuffStr[DMA2_CHAN6];
        p_res->pEventBuff = &DMA2_EventBuff[DMA2_CHAN6];
        p_res->GlobalFlagDef = DMA2_GL6_FLAG;
        p_res->FullDataFlagDef = DMA2_FDT6_FLAG;
        p_res->HalfDataFlagDef = DMA2_HDT6_FLAG;
        p_res->DataErrFlagDef = DMA2_DTERR6_FLAG;
    } else if(pDMAxChan_y == DMA2_CHANNEL7) {
        p_res->pEventBuffStr = &DMA2_EventBuffStr[DMA2_CHAN7];
        p_res->pEventBuff = &DMA2_EventBuff[DMA2_CHAN7];
        p_res->GlobalFlagDef = DMA2_GL7_FLAG;
        p_res->FullDataFlagDef = DMA2_FDT7_FLAG;
        p_res->HalfDataFlagDef = DMA2_HDT7_FLAG;
        p_res->DataErrFlagDef = DMA2_DTERR7_FLAG;
    } else {
#ifdef _APP_DEBUG_
        LOG("Invalid DMA Configuration");
#endif//_APP_DEBUG_
    }
    return (drv_status == ARM_DMA_STA_READY);
}

