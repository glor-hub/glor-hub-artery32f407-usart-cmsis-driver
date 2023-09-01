#ifndef _USART_H_
#define _USART_H_

#include "at32f403a_407.h"
#include "app.h"
#include "arm_usart.h"

error_status USART_Init(void);
error_status USART_Uninitialize(ARM_USART_Driver_t *p_drv);

#ifdef _APP_DEBUG_
error_status USART_Test(void);
#endif//_APP_DEBUG_

void USART_cb(void);
int8_t USART_printf(ARM_USART_Driver_t *p_drv, char *fmt, ...);

//select UARTx
//use DMA if necessary
//use DMA circular buffer if necessary (default - linear buffer)

/*******************************************
UART4

For DMA:
DMA2 CHANNEL5 (default config) - Tx
DMA2_CHANNEL3 (default config) - Rx
********************************************/
#define _UART4_PERIPH_ENABLE_
#define _UART4_TX_USE_DMA_
#define _UART4_TX_DMA_CIRCULAR_MODE_
#define _UART4_RX_USE_DMA_
#define _UART4_RX_DMA_CIRCULAR_MODE_

/*******************************************
UART5

For DMA:
DMA1 CHANNEL4 (with flexible mapping) - Tx
DMA1_CHANNEL5 (with flexible mapping) - Rx

********************************************/
#define _UART5_PERIPH_
#define _UART5_TX_USE_DMA_
#define _UART5_TX_DMA_CIRCULAR_MODE_
#define _UART5_RX_USE_DMA_
#define _UART5_RX_DMA_CIRCULAR_MODE_

/*******************************************
UART7

For DMA:
DMA1 CHANNEL2 (with flexible mapping) - Tx
DMA1_CHANNEL3 (with flexible mapping) - Rx
********************************************/
#define _UART7_PERIPH_
#define _UART7_TX_USE_DMA_
#define _UART7_TX_DMA_CIRCULAR_MODE_
#define _UART7_RX_USE_DMA_
#define _UART7_RX_DMA_CIRCULAR_MODE_

/*******************************************
UART8

For DMA:
DMA2 CHANNEL6 (with flexible mapping) - Tx
DMA2_CHANNEL4 (with flexible mapping) - Rx
********************************************/
#define _UART8_PERIPH_
#define _UART8_TX_USE_DMA_
#define _UART8_TX_DMA_CIRCULAR_MODE_
#define _UART8_RX_USE_DMA_
#define _UART8_RX_DMA_CIRCULAR_MODE_

#endif //_USART_H_ 
