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
    TEST_APP_ARM_DMA_IRQHandlerChannel(TEST_APP_ARM_DMA1_CHAN1);
}

void DMA1_Channel2_IRQHandler(void)
{
    TEST_APP_ARM_DMA_IRQHandlerChannel(TEST_APP_ARM_DMA1_CHAN2);
}

void DMA1_Channel3_IRQHandler(void)
{
    TEST_APP_ARM_DMA_IRQHandlerChannel(TEST_APP_ARM_DMA1_CHAN3);
}

void DMA1_Channel4_IRQHandler(void)
{
    TEST_APP_ARM_DMA_IRQHandlerChannel(TEST_APP_ARM_DMA1_CHAN4);
}

void DMA1_Channel5_IRQHandler(void)
{
    TEST_APP_ARM_DMA_IRQHandlerChannel(TEST_APP_ARM_DMA1_CHAN5);
}

void DMA1_Channel6_IRQHandler(void)
{
    TEST_APP_ARM_DMA_IRQHandlerChannel(TEST_APP_ARM_DMA1_CHAN6);
}

void DMA1_Channel7_IRQHandler(void)
{
    TEST_APP_ARM_DMA_IRQHandlerChannel(TEST_APP_ARM_DMA1_CHAN7);
}

void DMA2_Channel1_IRQHandler(void)
{
    TEST_APP_ARM_DMA_IRQHandlerChannel(TEST_APP_ARM_DMA2_CHAN1);
}

void DMA2_Channel2_IRQHandler(void)
{
    TEST_APP_ARM_DMA_IRQHandlerChannel(TEST_APP_ARM_DMA2_CHAN2);
}

void DMA2_Channel3_IRQHandler(void)
{
    TEST_APP_ARM_DMA_IRQHandlerChannel(TEST_APP_ARM_DMA2_CHAN3);
}

void DMA2_Channel4_5_IRQHandler(void)
{
    TEST_APP_ARM_DMA_IRQHandlerChannel_y_z(TEST_APP_ARM_DMA2_CHAN4, TEST_APP_ARM_DMA2_CHAN5);
}

void DMA2_Channel6_7_IRQHandler(void)
{
    TEST_APP_ARM_DMA_IRQHandlerChannel_y_z(TEST_APP_ARM_DMA2_CHAN6, TEST_APP_ARM_DMA2_CHAN7);
}

//===============================================================================
//Private
//================================================================================

