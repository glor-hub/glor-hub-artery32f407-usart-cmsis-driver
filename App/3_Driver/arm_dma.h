#ifndef _ARM_DMA_H_
#define _ARM_DMA_H_

#include "at32f403a_407.h"
#include <stdbool.h>
#include "ringbuffer.h"

typedef enum {
    TEST_APP_ARM_DMA1_CHAN1 = 0,
    TEST_APP_ARM_DMA1_CHAN2,
    TEST_APP_ARM_DMA1_CHAN3,
    TEST_APP_ARM_DMA1_CHAN4,
    TEST_APP_ARM_DMA1_CHAN5,
    TEST_APP_ARM_DMA1_CHAN6,
    TEST_APP_ARM_DMA1_CHAN7,
    TEST_APP_ARM_DMA2_CHAN1,
    TEST_APP_ARM_DMA2_CHAN2,
    TEST_APP_ARM_DMA2_CHAN3,
    TEST_APP_ARM_DMA2_CHAN4,
    TEST_APP_ARM_DMA2_CHAN5,
    TEST_APP_ARM_DMA2_CHAN6,
    TEST_APP_ARM_DMA2_CHAN7,
    TEST_APP_ARM_DMA_CHANS,
    TEST_APP_ARM_DMA_CHAN_UNDEFINED,
} eTEST_APP_ARM_DMA_Chan_t;


typedef enum {
    TEST_APP_ARM_DMA_FULL_DATA_TRANSFER_INT = 0,
    TEST_APP_ARM_DMA_HALF_DATA_TRANSFER_INT,
    TEST_APP_ARM_DMA_ERR_DATA_TRANSFER_INT
} eTEST_APP_ARM_DMA_InterruptTypes_t;


// DMA Driver Status
#define TEST_APP_ARM_DMA_STA_NO_ERROR       ((uint32_t)0UL)
#define TEST_APP_ARM_DMA_STA_ERROR          ((uint32_t)1UL << 0)
#define TEST_APP_ARM_DMA_STA_UNSUPPORTED    ((uint32_t)1UL << 1)

// DMA Event
#define TEST_APP_ARM_DMA_EVENT_FULL_DATA    ((uint32_t)1UL << 0)
#define TEST_APP_ARM_DMA_EVENT_HALF_DATA    ((uint32_t)1UL << 1)
#define TEST_APP_ARM_DMA_EVENT_DATA_ERROR   ((uint32_t)1UL << 2)
#define TEST_APP_ARM_DMA_EVENT_LOOP_MODE    ((uint32_t)1UL << 3)

#define TEST_APP_ARM_DMA_LOOP_MODE_ENABLE   TRUE
#define TEST_APP_ARM_DMA_LOOP_MODE_DISABLE  FALSE

bool TEST_APP_ARM_DMA_Init(eTEST_APP_ARM_DMA_Chan_t chan,
                           void (*pfunc_cb)(uint32_t event));
void TEST_APP_ARM_DMA_FlexibleConfig(eTEST_APP_ARM_DMA_Chan_t chan,
                                     dma_flexible_request_type flex_periph_req);
void TEST_APP_DMA_Enable(eTEST_APP_ARM_DMA_Chan_t chan, confirm_state state);
void TEST_APP_DMA_ClearAndEnableIRQ(eTEST_APP_ARM_DMA_Chan_t chan);
void TEST_APP_DMA_DisableAndClearIRQ(eTEST_APP_ARM_DMA_Chan_t chan);
void TEST_APP_ARM_DMA_Config(eTEST_APP_ARM_DMA_Chan_t chan,
                             uint32_t periph_addr, uint32_t mem_addr,
                             uint16_t buff_size, dma_dir_type dir,
                             confirm_state loop_mode_enable,
                             dma_priority_level_type priority);
void TEST_APP_DMA_InterruptEnable(eTEST_APP_ARM_DMA_Chan_t chan,
                                  eTEST_APP_ARM_DMA_InterruptTypes_t interrupt_type,
                                  confirm_state state);
void TEST_APP_ARM_DMA_IRQHandlerChannel(eTEST_APP_ARM_DMA_Chan_t chan);
void TEST_APP_ARM_DMA_IRQHandlerChannel_y_z(eTEST_APP_ARM_DMA_Chan_t chany, eTEST_APP_ARM_DMA_Chan_t chanz);

#endif //_ARM_DMA_H_ 
