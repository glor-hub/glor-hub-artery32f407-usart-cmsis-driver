//********************************************************************************
//usart.c
//********************************************************************************
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "usart.h"
#include "arm_driver.h"
#include "arm_usart.h"
#include "arm_gpio.h"
#include "arm_clock.h"
#include "ringbuffer.h"

#ifdef _TEST_APP_DEBUG_
#include "assert.h"
#include "LCD_2004.h"
#endif//_TEST_APP_DEBUG_

extern TEST_APP_ARM_USART_Driver_t *pARM_USART_Driver[TEST_APP_ARM_USART_TYPES];

//********************************************************************************
//Macros
//********************************************************************************

#define USART_TEST_MESSAGE_BUFF_SIZE 256

//********************************************************************************
//Enums
//********************************************************************************

//********************************************************************************
//Typedefs
//********************************************************************************

//********************************************************************************
//Prototypes
//********************************************************************************

//********************************************************************************
//Variables
//********************************************************************************

static char USART_TEST_MESSAGE[TEST_APP_ARM_USART_TYPES][USART_TEST_MESSAGE_BUFF_SIZE] = {
    {"Hello, World, from USART1!\n"},
    {"Hello, World, from USART2!\n"},
    {"Hello, World, from USART3!\n"},
    {"Hello, World, from UART4!\n"},
    {"Hello, World, from UART5!\n"},
    {"Hello, World, from USART6!\n"},
    {"Hello, World, from UART7!\n"},
    {"Hello, World, from UART8!\n"}
};

//================================================================================
//Public
//================================================================================

/********************************************************************************
gpio_pin_def - pin definitions default/remap1:

    UART4:
            default: uart4_tx(pc10), uart4_rx(pc11);
            remap1:  uart4_tx(pa0), uart4_rx(pa1);

    UART5:
            default: uart5_tx(pc12), uart5_rx(pd2);
            remap1:  uart5_tx(pb9), uart5_rx(pb8);

    UART7:
            default: uart7_tx(pe8), uart7_rx(pe7);
            remap1:  uart7_tx(pb4), uart7_rx(pb3);

    UART8:
            default: uart8_tx(pe1), uart8_rx(pe0);
            remap1:  uart8_tx(pc2), uart8_rx(pc3);

********************************************************************************/

void UART4_IRQHandler()
{
    TEST_APP_ARM_USART_IRQHandler(TEST_APP_ARM_UART4);
}

void UART5_IRQHandler()
{
    TEST_APP_ARM_USART_IRQHandler(TEST_APP_ARM_UART5);
}

void UART7_IRQHandler()
{
    TEST_APP_ARM_USART_IRQHandler(TEST_APP_ARM_UART7);
}

void UART8_IRQHandler()
{
    TEST_APP_ARM_USART_IRQHandler(TEST_APP_ARM_UART8);
}

error_status TEST_APP_USART_Init(void)
{
    uint32_t drv_status = TEST_APP_ARM_DRIVER_NO_ERROR;
    eTEST_APP_ARM_USART_Types_t usart;
    TEST_APP_ARM_USART_Driver_t *p_drv = pARM_USART_Driver[TEST_APP_ARM_USART1];
    for(usart = TEST_APP_ARM_USART1; usart < TEST_APP_ARM_USART_TYPES;
        usart++) {
        if(p_drv[usart].GetStatus().DrvStateOn) {
            //UART7 remap pin configuration is not used: the debug port uses these pins
            if(usart == TEST_APP_ARM_UART7) {
                drv_status |= p_drv[usart].Initialize(TEST_APP_ARM_USART_BAUDRATE_57600, USART_DATA_8BITS,
                                                      USART_STOP_1_BIT, USART_PARITY_NONE,
                                                      TEST_APP_ARM_USART_GPIO_PIN_DEF_TYPE_DEFAULT);
            } else {
                drv_status |= p_drv[usart].Initialize(TEST_APP_ARM_USART_BAUDRATE_57600, USART_DATA_8BITS,
                                                      USART_STOP_1_BIT, USART_PARITY_NONE,
                                                      TEST_APP_ARM_USART_GPIO_PIN_DEF_TYPE_REMAP1);
            }
        }
    }
    return TEST_APP_ARM_DRIVER_isReady(drv_status) ? SUCCESS : ERROR;
}

error_status TEST_APP_USART_Uninit(void)
{
    uint32_t drv_status = TEST_APP_ARM_DRIVER_NO_ERROR;

    eTEST_APP_ARM_USART_Types_t usart;
    TEST_APP_ARM_USART_Driver_t *p_drv = pARM_USART_Driver[TEST_APP_ARM_USART1];
    for(usart = TEST_APP_ARM_USART1; usart < TEST_APP_ARM_USART_TYPES;
        usart++) {
        if(!p_drv[usart].GetStatus().DrvStateOn) {
            drv_status |= p_drv[usart].Uninitialize();
        }
    }
    return TEST_APP_ARM_DRIVER_isReady(drv_status) ? SUCCESS : ERROR;
}

void TEST_APP_USART_cb(void)
{
    eTEST_APP_ARM_USART_Types_t usart;
    TEST_APP_ARM_USART_Driver_t *p_drv = pARM_USART_Driver[TEST_APP_ARM_USART1];
    for(usart = TEST_APP_ARM_USART1; usart < TEST_APP_ARM_USART_TYPES;
        usart++) {
        if(p_drv[usart].GetStatus().DrvStateOn) {
            p_drv[usart].Event_cb();
        }
    }
}

int8_t TEST_APP_USART_printf(TEST_APP_ARM_USART_Driver_t *p_drv, char *fmt, ...)
{
    uint32_t drv_status = TEST_APP_ARM_DRIVER_NO_ERROR;
    int8_t res, len;
    char *strbuff = (char *)p_drv->GetTransfer().pTxData;
    va_list args;
    va_start(args, fmt);
    while(p_drv->GetStatus().XferStatus.TxBusy);
    res = vsprintf(strbuff, fmt, args);
    len = strlen(strbuff);
    drv_status |= p_drv->Send(strbuff, len);
    va_end(args);
    if(drv_status == TEST_APP_ARM_DRIVER_NO_ERROR) {
        return res;
    } else {
        return -1;
    }
}

#ifdef _TEST_APP_DEBUG_
error_status TEST_APP_USART_Test(void)
{
    uint32_t drv_status = TEST_APP_ARM_DRIVER_NO_ERROR;
    uint8_t *pbuff_rx;
    eTEST_APP_ARM_USART_Types_t usart;
    TEST_APP_ARM_USART_Driver_t **pp_drv = &pARM_USART_Driver[TEST_APP_ARM_USART1];
    for(usart = TEST_APP_ARM_USART1; usart < TEST_APP_ARM_USART_TYPES;
        usart++) {
        if(pp_drv[usart]->GetStatus().DrvStateOn) {
            TEST_APP_USART_printf(pp_drv[usart], USART_TEST_MESSAGE[usart]);
            pbuff_rx = pp_drv[usart]->GetTransfer().pRxData;
            drv_status |= pp_drv[usart]->Recieve(pbuff_rx, 8);
        }
    }
    return TEST_APP_ARM_DRIVER_isReady(drv_status) ? SUCCESS : ERROR;
}
#endif//_TEST_APP_DEBUG_

//================================================================================
//Private
//================================================================================



