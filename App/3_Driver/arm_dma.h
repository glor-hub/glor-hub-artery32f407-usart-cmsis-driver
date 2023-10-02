#ifndef _ARM_DMA_H_
#define _ARM_DMA_H_

#include "at32f403a_407.h"
#include <stdbool.h>
#include "ringbuffer.h"

// DMA Driver Status
#define TEST_APP_ARM_DMA_STA_READY               ((uint32_t)0UL)
#define TEST_APP_ARM_DMA_STA_ERROR               ((uint32_t)1UL << 0)
#define TEST_APP_ARM_DMA_STA_UNSUPPORTED         ((uint32_t)1UL << 1)

// DMA Event
#define TEST_APP_ARM_DMA_EVENT_FULL_DATA     ((uint32_t)1UL << 0)
#define TEST_APP_ARM_DMA_EVENT_HALF_DATA     ((uint32_t)1UL << 1)
#define TEST_APP_ARM_DMA_EVENT_DATA_ERROR    ((uint32_t)1UL << 2)

bool TEST_APP_ARM_DMA_Init(dma_channel_type *pDMAxChan_y);
void TEST_APP_ARM_DMA_Config(dma_channel_type *pDMAxChan_y, dma_init_type *pDMA_Cfg, uint32_t periph_addr,
                             uint32_t mem_addr, uint16_t buff_size, dma_priority_level_type priority);
TEST_APP_RingBuffer_t *TEST_APP_ARM_DMA_GetEventBuffStr(dma_channel_type *pDMAxChan_y);
void TEST_APP_ARM_DMA_IRQHandlerChannel(dma_channel_type *pDMAxChan_y);
void TEST_APP_ARM_DMA_IRQHandlerChannel_y_z(dma_channel_type *pDMAxChan_y, dma_channel_type *pDMAxChan_z);

#endif //_ARM_DMA_H_ 
