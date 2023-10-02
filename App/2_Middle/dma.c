//********************************************************************************
//dma.c
//********************************************************************************
#include "dma.h"
#include "arm_dma.h"

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

//================================================================================
//Public
//================================================================================

void DMA1_Channel1_IRQHandler(void)
{
    TEST_APP_ARM_DMA_IRQHandlerChannel(DMA1_CHANNEL1);
}

void DMA1_Channel2_IRQHandler(void)
{
    TEST_APP_ARM_DMA_IRQHandlerChannel(DMA1_CHANNEL2);
}

void DMA1_Channel3_IRQHandler(void)
{
    TEST_APP_ARM_DMA_IRQHandlerChannel(DMA1_CHANNEL3);
}

void DMA1_Channel4_IRQHandler(void)
{
    TEST_APP_ARM_DMA_IRQHandlerChannel(DMA1_CHANNEL4);
}

void DMA1_Channel5_IRQHandler(void)
{
    TEST_APP_ARM_DMA_IRQHandlerChannel(DMA1_CHANNEL5);
}

void DMA1_Channel6_IRQHandler(void)
{
    TEST_APP_ARM_DMA_IRQHandlerChannel(DMA1_CHANNEL6);
}

void DMA1_Channel7_IRQHandler(void)
{
    TEST_APP_ARM_DMA_IRQHandlerChannel(DMA1_CHANNEL7);
}

void DMA2_Channel1_IRQHandler(void)
{
    TEST_APP_ARM_DMA_IRQHandlerChannel(DMA2_CHANNEL1);
}

void DMA2_Channel2_IRQHandler(void)
{
    TEST_APP_ARM_DMA_IRQHandlerChannel(DMA2_CHANNEL2);
}

void DMA2_Channel3_IRQHandler(void)
{
    TEST_APP_ARM_DMA_IRQHandlerChannel(DMA2_CHANNEL3);
}

void DMA2_Channel4_5_IRQHandler(void)
{
    TEST_APP_ARM_DMA_IRQHandlerChannel_y_z(DMA2_CHANNEL4, DMA2_CHANNEL5);
}

void DMA2_Channel6_7_IRQHandler(void)
{
    TEST_APP_ARM_DMA_IRQHandlerChannel_y_z(DMA2_CHANNEL6, DMA2_CHANNEL7);
}

//===============================================================================
//Private
//================================================================================

