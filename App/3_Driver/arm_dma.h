#ifndef _ARM_DMA_H_
#define _ARM_DMA_H_

#include "at32f403a_407.h"
#include <stdbool.h>
#include "ringbuffer.h"

// DMA Driver Status
#define ARM_DMA_STA_READY               ((uint32_t)0UL)
#define ARM_DMA_STA_ERROR               ((uint32_t)1UL << 0)
#define ARM_DMA_STA_UNSUPPORTED         ((uint32_t)1UL << 1)

// DMA Event
#define ARM_DMA_EVENT_FULL_DATA     ((uint32_t)1UL << 0)
#define ARM_DMA_EVENT_HALF_DATA     ((uint32_t)1UL << 1)
#define ARM_DMA_EVENT_DATA_ERROR    ((uint32_t)1UL << 2)

typedef enum {
    DMA1_CHAN1 = 0,
    DMA1_CHAN2,
    DMA1_CHAN3,
    DMA1_CHAN4,
    DMA1_CHAN5,
    DMA1_CHAN6,
    DMA1_CHAN7,
    DMA1_CHANS
} DMA1_Chans;

typedef enum {
    DMA2_CHAN1 = 0,
    DMA2_CHAN2,
    DMA2_CHAN3,
    DMA2_CHAN4,
    DMA2_CHAN5,
    DMA2_CHAN6,
    DMA2_CHAN7,
    DMA2_CHANS
} DMA2_Chans;

bool ARM_DMA_Init(dma_channel_type *pDMAxChan_y);
void ARM_DMA_Config(dma_channel_type *pDMAxChan_y, dma_init_type *pDMA_Cfg, uint32_t periph_addr,
                    uint32_t mem_addr, uint16_t buff_size, dma_priority_level_type priority);
RingBuffer_t *ARM_DMA_GetEventBuffStr(dma_channel_type *pDMAxChan_y);
void ARM_DMA_IRQHandlerChannel(dma_channel_type *pDMAxChan_y);
void ARM_DMA_IRQHandlerChannel_y_z(dma_channel_type *pDMAxChan_y, dma_channel_type *pDMAxChan_z);

#endif //_ARM_DMA_H_ 
