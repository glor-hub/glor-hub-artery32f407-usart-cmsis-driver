#ifndef _RINGBUFFER_H_
#define _RINGBUFFER_H_

#include "at32f403a_407.h"

#define RING_BUFF_OK             0
#define RING_BUFF_OVERFLOW_ERR  -1
#define RING_BUFF_UNDERFLOW_ERR -2

typedef struct {
    uint8_t in;
    uint8_t out;
    uint8_t count;
    void *p_buff;
    uint8_t buff_size;
} RingBuffer_t;

void RingBuffer_Init(RingBuffer_t *p_struct,  void *p_buffer, uint8_t size);
int8_t RingBuffer_Write(RingBuffer_t *p_struct,  uint32_t *p_data);
int8_t RingBuffer_Read(RingBuffer_t *p_struct,  uint32_t *p_data);
uint8_t RingBuffer_GetCount(RingBuffer_t *p_struct);
void RingBuffer_Reset(RingBuffer_t *p_struct);

#endif //_RINGBUFFER_H_ 
