//********************************************************************************
//ringbuffer.c
//********************************************************************************
#include <stdbool.h>
#include "ringbuffer.h"

#ifdef _APP_DEBUG_
#include "assert.h"
#endif//_APP_DEBUG_

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

static bool RingBuffer_isEmpty(RingBuffer_t *p_struct);

static bool RingBuffer_isFull(RingBuffer_t *p_struct);

//================================================================================
//Public
//================================================================================

void RingBuffer_Init(RingBuffer_t *p_struct,  void *p_buffer, uint8_t size)
{
    p_struct->In = 0U;
    p_struct->Out = 0U;
    p_struct->Count = 0U;
    p_struct->pBuff = p_buffer;
    p_struct->BuffSize = size;
}

int8_t RingBuffer_WriteByte(RingBuffer_t *p_struct,  uint8_t *p_data)
{
    if(RingBuffer_isFull(p_struct)) {
#ifdef _APP_DEBUG_
        LOG("Ring Buffer overflow error");
#endif//_APP_DEBUG_
        return RING_BUFF_OVERFLOW_ERR;
    } else {
        *((uint8_t *)p_struct->pBuff + p_struct->In++) = *p_data;
        p_struct->In %= p_struct->BuffSize;
        p_struct->Count++;
        return RING_BUFF_OK;
    }
}

int8_t RingBuffer_Write(RingBuffer_t *p_struct,  uint32_t *p_data)
{
    if(RingBuffer_isFull(p_struct)) {
#ifdef _APP_DEBUG_
        LOG("Ring Buffer overflow error");
#endif//_APP_DEBUG_
        return RING_BUFF_OVERFLOW_ERR;
    } else {
        *((uint32_t *)p_struct->pBuff + p_struct->In++) = *p_data;
        p_struct->In %= p_struct->BuffSize;
        p_struct->Count++;
        return RING_BUFF_OK;
    }
}

int8_t RingBuffer_ReadByte(RingBuffer_t *p_struct,  uint8_t *p_data)
{
    if(RingBuffer_isEmpty(p_struct)) {
#ifdef _APP_DEBUG_
        LOG("Ring Buffer underflow error");
#endif//_APP_DEBUG_
        return RING_BUFF_UNDERFLOW_ERR;
    } else {
        *p_data = *((uint8_t *)p_struct->pBuff + p_struct->Out++);
        p_struct->Out %= p_struct->BuffSize;
        p_struct->Count--;
        return RING_BUFF_OK;
    }
}

int8_t RingBuffer_Read(RingBuffer_t *p_struct,  uint32_t *p_data)
{
    if(RingBuffer_isEmpty(p_struct)) {
#ifdef _APP_DEBUG_
        LOG("Ring Buffer underflow error");
#endif//_APP_DEBUG_
        return RING_BUFF_UNDERFLOW_ERR;
    } else {
        *p_data = *((uint32_t *)p_struct->pBuff + p_struct->Out++);
        p_struct->Out %= p_struct->BuffSize;
        p_struct->Count--;
        return RING_BUFF_OK;
    }
}

uint8_t RingBuffer_GetCount(RingBuffer_t *p_struct)
{
    return p_struct->Count;
}

void RingBuffer_Reset(RingBuffer_t *p_struct)
{
    p_struct->Out = p_struct->In;
    p_struct->Count = 0U;
}

//================================================================================
//Private
//================================================================================

static bool RingBuffer_isEmpty(RingBuffer_t *p_struct)
{
    return p_struct->Count == 0U;
}

static bool RingBuffer_isFull(RingBuffer_t *p_struct)
{
    return p_struct->Count == p_struct->BuffSize;
}
