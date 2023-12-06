#ifndef _RINGBUFFER_H_
#define _RINGBUFFER_H_

#include "at32f403a_407.h"

typedef enum {
    RING_BUFF_NO_ERROR = 0,
    RING_BUFF_OVERFLOW_ERR,
    RING_BUFF_UNDERFLOW_ERR,
    RING_BUFF_LAST_ERROR_CODE
} eTEST_APP_RingBufferError_t;

typedef struct {
    volatile uint8_t In;
    volatile uint8_t Out;
    volatile uint8_t Count;
    void *pBuff;
    uint8_t BuffSize;
} TEST_APP_RingBuffer_t;

void TEST_APP_RingBuffer_Init(TEST_APP_RingBuffer_t *p_struct,  void *p_buffer, uint8_t size);
eTEST_APP_RingBufferError_t TEST_APP_RingBuffer_Write(TEST_APP_RingBuffer_t *p_struct,  uint32_t *p_data);
eTEST_APP_RingBufferError_t TEST_APP_RingBuffer_Read(TEST_APP_RingBuffer_t *p_struct,  uint32_t *p_data);
uint8_t TEST_APP_RingBuffer_GetCount(TEST_APP_RingBuffer_t *p_struct);
void TEST_APP_RingBuffer_Reset(TEST_APP_RingBuffer_t *p_struct);

#endif //_RINGBUFFER_H_ 
