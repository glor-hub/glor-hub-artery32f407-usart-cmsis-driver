//********************************************************************************
//ringbuffer.c
//********************************************************************************
#include <stdbool.h>
#include "ringbuffer.h"

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


//********************************************************************************
//Variables
//********************************************************************************

//********************************************************************************
//Prototypes
//********************************************************************************

static bool RingBuffer_isEmpty(TEST_APP_RingBuffer_t *p_struct);

static bool RingBuffer_isFull(TEST_APP_RingBuffer_t *p_struct);

//================================================================================
//Public
//================================================================================

void TEST_APP_RingBuffer_Init(TEST_APP_RingBuffer_t *p_struct,  void *p_buffer, uint8_t size)
{
    p_struct->In = 0U;
    p_struct->Out = 0U;
    p_struct->Count = 0U;
    //8 or 32 bits
    p_struct->pBuff = p_buffer;
    p_struct->BuffSize = size;
}

eTEST_APP_RingBufferError_t TEST_APP_RingBuffer_WriteByte(TEST_APP_RingBuffer_t *p_struct,  uint8_t *p_data)
{
    if(RingBuffer_isFull(p_struct)) {
#ifdef _TEST_APP_DEBUG_
        LOG("Ring Buffer overflow error");
#endif//_TEST_APP_DEBUG_
        return RING_BUFF_OVERFLOW_ERR;
    } else {
        *((uint8_t *)p_struct->pBuff + p_struct->In++) = *p_data;
        p_struct->In %= p_struct->BuffSize;
        p_struct->Count++;
        return RING_BUFF_NO_ERROR;
    }
}

eTEST_APP_RingBufferError_t TEST_APP_RingBuffer_Write(TEST_APP_RingBuffer_t *p_struct,  uint32_t *p_data)
{
    if(RingBuffer_isFull(p_struct)) {
#ifdef _TEST_APP_DEBUG_
        LOG("Ring Buffer overflow error");
#endif//_TEST_APP_DEBUG_
        return RING_BUFF_OVERFLOW_ERR;
    } else {
        *((uint32_t *)p_struct->pBuff + p_struct->In++) = *p_data;
        p_struct->In %= p_struct->BuffSize;
        p_struct->Count++;
        return RING_BUFF_NO_ERROR;
    }
}

eTEST_APP_RingBufferError_t TEST_APP_RingBuffer_ReadByte(TEST_APP_RingBuffer_t *p_struct,  uint8_t *p_data)
{
    if(RingBuffer_isEmpty(p_struct)) {
#ifdef _TEST_APP_DEBUG_
        LOG("Ring Buffer underflow error");
#endif//_TEST_APP_DEBUG_
        return RING_BUFF_UNDERFLOW_ERR;
    } else {
        *p_data = *((uint8_t *)p_struct->pBuff + p_struct->Out++);
        p_struct->Out %= p_struct->BuffSize;
        p_struct->Count--;
        return RING_BUFF_NO_ERROR;
    }
}

eTEST_APP_RingBufferError_t TEST_APP_RingBuffer_Read(TEST_APP_RingBuffer_t *p_struct,  uint32_t *p_data)
{
    if(RingBuffer_isEmpty(p_struct)) {
#ifdef _TEST_APP_DEBUG_
        LOG("Ring Buffer underflow error");
#endif//_TEST_APP_DEBUG_
        return RING_BUFF_UNDERFLOW_ERR;
    } else {
        *p_data = *((uint32_t *)p_struct->pBuff + p_struct->Out++);
        p_struct->Out %= p_struct->BuffSize;
        p_struct->Count--;
        return RING_BUFF_NO_ERROR;
    }
}

uint8_t TEST_APP_RingBuffer_GetCount(TEST_APP_RingBuffer_t *p_struct)
{
    return p_struct->Count;
}

void TEST_APP_RingBuffer_Reset(TEST_APP_RingBuffer_t *p_struct)
{
    p_struct->Out = p_struct->In;
    p_struct->Count = 0U;
}

//================================================================================
//Private
//================================================================================

static bool RingBuffer_isEmpty(TEST_APP_RingBuffer_t *p_struct)
{
    return p_struct->Count == 0U;
}

static bool RingBuffer_isFull(TEST_APP_RingBuffer_t *p_struct)
{
    return p_struct->Count == p_struct->BuffSize;
}
