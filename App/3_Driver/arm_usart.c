//********************************************************************************
//arm_usart.c
//********************************************************************************
#include <string.h>
#include "arm_driver.h"
#include "arm_usart.h"
#include "arm_clock.h"
#include "arm_gpio.h"
#include "app.h"
#include "arm_dma.h"
#include "timer.h"

#ifdef _TEST_APP_DEBUG_
#include "assert.h"
#include "LCD_2004.h"
#endif//_TEST_APP_DEBUG_

//********************************************************************************
//Macros
//********************************************************************************

#define ARM_USART_EVENT_BUFF_SIZE 8

#define ARM_USART_TX_BUFF_SIZE 256
#define ARM_USART_RX_BUFF_SIZE 256

#define ARM_USART_DMA_FLEX_MODE_ENABLE  TRUE
#define ARM_USART_DMA_FLEX_MODE_DISABLE FALSE

#define ARM_USART_TIMEOUT_MSEC 200

//********************************************************************************
//Enums
//********************************************************************************



//********************************************************************************
//Typedefs
//********************************************************************************

typedef enum {
    ARM_USART_TX_CHAN = 0,
    ARM_USART_RX_CHAN,
    ARM_USART_CHANS
} eARM_USART_Chans_t;


//********************************************************************************
//Prototypes
//********************************************************************************

static void  ARM_USART_SetResources(TEST_APP_ARM_USART_Resources_t *p_res, eTEST_APP_ARM_USART_Types_t usart_type,
                                    uint32_t baudrate,
                                    usart_data_bit_num_type data_bit,
                                    usart_stop_bit_num_type stop_bit,
                                    usart_parity_selection_type parity,
                                    eTEST_APP_ARM_USART_PinDefTypes_t gpio_pin_def_type);
static uint32_t ARM_USART_GPIO_Config(eTEST_APP_ARM_USART_Types_t usart_type, confirm_state new_state);

static uint32_t ARM_USART_Initialize(eTEST_APP_ARM_USART_Types_t usart_type,
                                     uint32_t baudrate,
                                     usart_data_bit_num_type data_bit,
                                     usart_stop_bit_num_type stop_bit,
                                     usart_parity_selection_type parity,
                                     eTEST_APP_ARM_USART_PinDefTypes_t gpio_pin_def_type);

static uint32_t ARM_USART_Initialize_1(uint32_t baudrate,
                                       usart_data_bit_num_type dataBit,
                                       usart_stop_bit_num_type stopBit,
                                       usart_parity_selection_type parity,
                                       eTEST_APP_ARM_USART_PinDefTypes_t gpio_pin_def_type);
static uint32_t ARM_USART_Initialize_2(uint32_t baudrate,
                                       usart_data_bit_num_type dataBit,
                                       usart_stop_bit_num_type stopBit,
                                       usart_parity_selection_type parity,
                                       eTEST_APP_ARM_USART_PinDefTypes_t gpio_pin_def_type);
static uint32_t ARM_USART_Initialize_3(uint32_t baudrate,
                                       usart_data_bit_num_type dataBit,
                                       usart_stop_bit_num_type stopBit,
                                       usart_parity_selection_type parity,
                                       eTEST_APP_ARM_USART_PinDefTypes_t gpio_pin_def_type);
static uint32_t ARM_USART_Initialize_4(uint32_t baudrate,
                                       usart_data_bit_num_type dataBit,
                                       usart_stop_bit_num_type stopBit,
                                       usart_parity_selection_type parity,
                                       eTEST_APP_ARM_USART_PinDefTypes_t gpio_pin_def_type);
static uint32_t ARM_USART_Initialize_5(uint32_t baudrate,
                                       usart_data_bit_num_type dataBit,
                                       usart_stop_bit_num_type stopBit,
                                       usart_parity_selection_type parity,
                                       eTEST_APP_ARM_USART_PinDefTypes_t gpio_pin_def_type);
static uint32_t ARM_USART_Initialize_6(uint32_t baudrate,
                                       usart_data_bit_num_type dataBit,
                                       usart_stop_bit_num_type stopBit,
                                       usart_parity_selection_type parity,
                                       eTEST_APP_ARM_USART_PinDefTypes_t gpio_pin_def_type);
static uint32_t ARM_USART_Initialize_7(uint32_t baudrate,
                                       usart_data_bit_num_type dataBit,
                                       usart_stop_bit_num_type stopBit,
                                       usart_parity_selection_type parity,
                                       eTEST_APP_ARM_USART_PinDefTypes_t gpio_pin_def_type);
static uint32_t ARM_USART_Initialize_8(uint32_t baudrate,
                                       usart_data_bit_num_type dataBit,
                                       usart_stop_bit_num_type stopBit,
                                       usart_parity_selection_type parity,
                                       eTEST_APP_ARM_USART_PinDefTypes_t gpio_pin_def_type);

static uint32_t ARM_USART_Uninitialize(eTEST_APP_ARM_USART_Types_t usart_type);
static uint32_t ARM_USART_Uninitialize_1(void);
static uint32_t ARM_USART_Uninitialize_2(void);
static uint32_t ARM_USART_Uninitialize_3(void);
static uint32_t ARM_USART_Uninitialize_4(void);
static uint32_t ARM_USART_Uninitialize_5(void);
static uint32_t ARM_USART_Uninitialize_6(void);
static uint32_t ARM_USART_Uninitialize_7(void);
static uint32_t ARM_USART_Uninitialize_8(void);

static void ARM_USART_Event_cb(eTEST_APP_ARM_USART_Types_t usart_type);
static void ARM_USART_Event_cb_1(void);
static void ARM_USART_Event_cb_2(void);
static void ARM_USART_Event_cb_3(void);
static void ARM_USART_Event_cb_4(void);
static void ARM_USART_Event_cb_5(void);
static void ARM_USART_Event_cb_6(void);
static void ARM_USART_Event_cb_7(void);
static void ARM_USART_Event_cb_8(void);

static void ARM_USART_DMA_Event_cb(uint32_t event,
                                   eTEST_APP_ARM_USART_Types_t usart_type,
                                   eARM_USART_Chans_t chan_type);
static void ARM_USART1_DMA_TxEvent_cb(uint32_t event);
static void ARM_USART1_DMA_RxEvent_cb(uint32_t event);
static void ARM_USART2_DMA_TxEvent_cb(uint32_t event);
static void ARM_USART2_DMA_RxEvent_cb(uint32_t event);
static void ARM_USART3_DMA_TxEvent_cb(uint32_t event);
static void ARM_USART3_DMA_RxEvent_cb(uint32_t event);
static void ARM_UART4_DMA_TxEvent_cb(uint32_t event);
static void ARM_UART4_DMA_RxEvent_cb(uint32_t event);
static void ARM_UART5_DMA_TxEvent_cb(uint32_t event);
static void ARM_UART5_DMA_RxEvent_cb(uint32_t event);
static void ARM_USART6_DMA_TxEvent_cb(uint32_t event);
static void ARM_USART6_DMA_RxEvent_cb(uint32_t event);
static void ARM_UART7_DMA_TxEvent_cb(uint32_t event);
static void ARM_UART7_DMA_RxEvent_cb(uint32_t event);
static void ARM_UART8_DMA_TxEvent_cb(uint32_t event);
static void ARM_UART8_DMA_RxEvent_cb(uint32_t event);

static uint8_t ARM_USART_ReadByte(eTEST_APP_ARM_USART_Types_t usart_type);

static void ARM_USART_WriteByte(eTEST_APP_ARM_USART_Types_t usart_type, uint8_t *pByte);

static uint32_t ARM_USART_Send(eTEST_APP_ARM_USART_Types_t usart_type,
                               void *pdata, uint32_t num);
static uint32_t ARM_USART_Send_1(void *pdata, uint32_t num);
static uint32_t ARM_USART_Send_2(void *pdata, uint32_t num);
static uint32_t ARM_USART_Send_3(void *pdata, uint32_t num);
static uint32_t ARM_USART_Send_4(void *pdata, uint32_t num);
static uint32_t ARM_USART_Send_5(void *pdata, uint32_t num);
static uint32_t ARM_USART_Send_6(void *pdata, uint32_t num);
static uint32_t ARM_USART_Send_7(void *pdata, uint32_t num);
static uint32_t ARM_USART_Send_8(void *pdata, uint32_t num);


static uint32_t ARM_USART_Recieve(eTEST_APP_ARM_USART_Types_t usart_type,
                                  void *pdata, uint32_t num);
static uint32_t ARM_USART_Recieve_1(void *pdata, uint32_t num);
static uint32_t ARM_USART_Recieve_2(void *pdata, uint32_t num);
static uint32_t ARM_USART_Recieve_3(void *pdata, uint32_t num);
static uint32_t ARM_USART_Recieve_4(void *pdata, uint32_t num);
static uint32_t ARM_USART_Recieve_5(void *pdata, uint32_t num);
static uint32_t ARM_USART_Recieve_6(void *pdata, uint32_t num);
static uint32_t ARM_USART_Recieve_7(void *pdata, uint32_t num);
static uint32_t ARM_USART_Recieve_8(void *pdata, uint32_t num);



static TEST_APP_ARM_USART_Transfer_t ARM_USART_GetTransfer(eTEST_APP_ARM_USART_Types_t usart_type);
static TEST_APP_ARM_USART_Transfer_t ARM_USART_GetTransfer_1(void);
static TEST_APP_ARM_USART_Transfer_t ARM_USART_GetTransfer_2(void);
static TEST_APP_ARM_USART_Transfer_t ARM_USART_GetTransfer_3(void);
static TEST_APP_ARM_USART_Transfer_t ARM_USART_GetTransfer_4(void);
static TEST_APP_ARM_USART_Transfer_t ARM_USART_GetTransfer_5(void);
static TEST_APP_ARM_USART_Transfer_t ARM_USART_GetTransfer_6(void);
static TEST_APP_ARM_USART_Transfer_t ARM_USART_GetTransfer_7(void);
static TEST_APP_ARM_USART_Transfer_t ARM_USART_GetTransfer_8(void);

static TEST_APP_ARM_USART_Status_t ARM_USART_GetStatus(eTEST_APP_ARM_USART_Types_t usart_type);
static TEST_APP_ARM_USART_Status_t ARM_USART_GetStatus_1(void);
static TEST_APP_ARM_USART_Status_t ARM_USART_GetStatus_2(void);
static TEST_APP_ARM_USART_Status_t ARM_USART_GetStatus_3(void);
static TEST_APP_ARM_USART_Status_t ARM_USART_GetStatus_4(void);
static TEST_APP_ARM_USART_Status_t ARM_USART_GetStatus_5(void);
static TEST_APP_ARM_USART_Status_t ARM_USART_GetStatus_6(void);
static TEST_APP_ARM_USART_Status_t ARM_USART_GetStatus_7(void);
static TEST_APP_ARM_USART_Status_t ARM_USART_GetStatus_8(void);

//********************************************************************************
//Variables
//********************************************************************************

TEST_APP_ARM_USART_Driver_t Driver_USART1 = {
    ARM_USART_Initialize_1,
    ARM_USART_Uninitialize_1,
    ARM_USART_Event_cb_1,
    ARM_USART_Send_1,
    ARM_USART_Recieve_1,
    ARM_USART_GetTransfer_1,
    ARM_USART_GetStatus_1
};

TEST_APP_ARM_USART_Driver_t Driver_USART2 = {
    ARM_USART_Initialize_2,
    ARM_USART_Uninitialize_2,
    ARM_USART_Event_cb_2,
    ARM_USART_Send_2,
    ARM_USART_Recieve_2,
    ARM_USART_GetTransfer_2,
    ARM_USART_GetStatus_2
};
TEST_APP_ARM_USART_Driver_t Driver_USART3 = {
    ARM_USART_Initialize_3,
    ARM_USART_Uninitialize_3,
    ARM_USART_Event_cb_3,
    ARM_USART_Send_3,
    ARM_USART_Recieve_3,
    ARM_USART_GetTransfer_3,
    ARM_USART_GetStatus_3
};
TEST_APP_ARM_USART_Driver_t Driver_UART4 = {
    ARM_USART_Initialize_4,
    ARM_USART_Uninitialize_4,
    ARM_USART_Event_cb_4,
    ARM_USART_Send_4,
    ARM_USART_Recieve_4,
    ARM_USART_GetTransfer_4,
    ARM_USART_GetStatus_4
};
TEST_APP_ARM_USART_Driver_t Driver_UART5 = {
    ARM_USART_Initialize_5,
    ARM_USART_Uninitialize_5,
    ARM_USART_Event_cb_5,
    ARM_USART_Send_5,
    ARM_USART_Recieve_5,
    ARM_USART_GetTransfer_5,
    ARM_USART_GetStatus_5
};
TEST_APP_ARM_USART_Driver_t Driver_USART6 = {
    ARM_USART_Initialize_6,
    ARM_USART_Uninitialize_6,
    ARM_USART_Event_cb_6,
    ARM_USART_Send_6,
    ARM_USART_Recieve_6,
    ARM_USART_GetTransfer_6,
    ARM_USART_GetStatus_6
};
TEST_APP_ARM_USART_Driver_t Driver_UART7 = {
    ARM_USART_Initialize_7,
    ARM_USART_Uninitialize_7,
    ARM_USART_Event_cb_7,
    ARM_USART_Send_7,
    ARM_USART_Recieve_7,
    ARM_USART_GetTransfer_7,
    ARM_USART_GetStatus_7
};
TEST_APP_ARM_USART_Driver_t Driver_UART8 = {
    ARM_USART_Initialize_8,
    ARM_USART_Uninitialize_8,
    ARM_USART_Event_cb_8,
    ARM_USART_Send_8,
    ARM_USART_Recieve_8,
    ARM_USART_GetTransfer_8,
    ARM_USART_GetStatus_8
};

TEST_APP_ARM_USART_Driver_t *pARM_USART_Driver[TEST_APP_ARM_USART_TYPES] = {
    &Driver_USART1,
    &Driver_USART2,
    &Driver_USART3,
    &Driver_UART4,
    &Driver_UART5,
    &Driver_USART6,
    &Driver_UART7,
    &Driver_UART8
};

static void (*pTEST_APP_ARM_USART_DMA_cb[TEST_APP_ARM_USART_TYPES][ARM_USART_CHANS])(uint32_t event) = {
    {ARM_USART1_DMA_TxEvent_cb,     ARM_USART1_DMA_RxEvent_cb},
    {ARM_USART2_DMA_TxEvent_cb,     ARM_USART2_DMA_RxEvent_cb},
    {ARM_USART3_DMA_TxEvent_cb,     ARM_USART3_DMA_RxEvent_cb},
    {ARM_UART4_DMA_TxEvent_cb,      ARM_UART4_DMA_RxEvent_cb},
    {ARM_UART5_DMA_TxEvent_cb,      ARM_UART5_DMA_RxEvent_cb},
    {ARM_USART6_DMA_TxEvent_cb,     ARM_USART6_DMA_RxEvent_cb},
    {ARM_UART7_DMA_TxEvent_cb,      ARM_UART7_DMA_RxEvent_cb},
    {ARM_UART8_DMA_TxEvent_cb,      ARM_UART8_DMA_RxEvent_cb}
};

static TEST_APP_ARM_USART_Resources_t ARM_USART_Resources[TEST_APP_ARM_USART_TYPES];
static uint32_t ARM_USART_EventBuff[TEST_APP_ARM_USART_TYPES][ARM_USART_EVENT_BUFF_SIZE];

static uint32_t ARM_USART_TxBuff[TEST_APP_ARM_USART_TYPES][ARM_USART_TX_BUFF_SIZE];
static uint32_t ARM_USART_RxBuff[TEST_APP_ARM_USART_TYPES][ARM_USART_RX_BUFF_SIZE];

static TEST_APP_ARM_USART_GPIO_t ARM_USART_GPIO_Def[TEST_APP_ARM_USART_TYPES][TEST_APP_ARM_USART_GPIO_PIN_DEF_TYPES] = {
    {   {TEST_APP_ARM_GPIO_PORTA, GPIO_PINS_9,  TEST_APP_ARM_GPIO_PORTA, GPIO_PINS_10},
        {TEST_APP_ARM_GPIO_PORTB, GPIO_PINS_6,  TEST_APP_ARM_GPIO_PORTB, GPIO_PINS_7}
    },
    {   {TEST_APP_ARM_GPIO_PORTA, GPIO_PINS_2,  TEST_APP_ARM_GPIO_PORTA, GPIO_PINS_3},
        {TEST_APP_ARM_GPIO_PORTD, GPIO_PINS_5,  TEST_APP_ARM_GPIO_PORTD, GPIO_PINS_6}
    },
    {   {TEST_APP_ARM_GPIO_PORTB, GPIO_PINS_10, TEST_APP_ARM_GPIO_PORTB, GPIO_PINS_11},
        {TEST_APP_ARM_GPIO_PORTD, GPIO_PINS_8,  TEST_APP_ARM_GPIO_PORTD, GPIO_PINS_9}
    },
    {   {TEST_APP_ARM_GPIO_PORTC, GPIO_PINS_10, TEST_APP_ARM_GPIO_PORTC, GPIO_PINS_11},
        {TEST_APP_ARM_GPIO_PORTA, GPIO_PINS_0,  TEST_APP_ARM_GPIO_PORTA, GPIO_PINS_1}
    },
    {   {TEST_APP_ARM_GPIO_PORTC, GPIO_PINS_12, TEST_APP_ARM_GPIO_PORTD, GPIO_PINS_2},
        {TEST_APP_ARM_GPIO_PORTB, GPIO_PINS_9,  TEST_APP_ARM_GPIO_PORTB, GPIO_PINS_8}
    },
    {   {TEST_APP_ARM_GPIO_PORTC, GPIO_PINS_6,  TEST_APP_ARM_GPIO_PORTC, GPIO_PINS_7},
        {TEST_APP_ARM_GPIO_PORTA, GPIO_PINS_4,  TEST_APP_ARM_GPIO_PORTA, GPIO_PINS_5}
    },
    {   {TEST_APP_ARM_GPIO_PORTE, GPIO_PINS_8,  TEST_APP_ARM_GPIO_PORTE, GPIO_PINS_7},
        {TEST_APP_ARM_GPIO_PORTB, GPIO_PINS_4,  TEST_APP_ARM_GPIO_PORTB, GPIO_PINS_3}
    },
    {   {TEST_APP_ARM_GPIO_PORTE, GPIO_PINS_1,  TEST_APP_ARM_GPIO_PORTE, GPIO_PINS_0},
        {TEST_APP_ARM_GPIO_PORTC, GPIO_PINS_2,  TEST_APP_ARM_GPIO_PORTC, GPIO_PINS_3}
    }
};

//USART GPIO multiplex I/O (IOMUX) remapping definitions
uint32_t ARM_USART_GPIO_IOMUX_Remap_Def[TEST_APP_ARM_USART_TYPES] = {
    USART1_GMUX_0001,
    USART2_GMUX_0001,
    USART3_GMUX_0001,
    UART4_GMUX_0010,
    UART5_GMUX_0001,
    USART6_GMUX,
    UART7_GMUX,
    UART8_GMUX
};

static usart_type *pARM_USART_Register[TEST_APP_ARM_USART_TYPES] = {
    USART1,
    USART2,
    USART3,
    UART4,
    UART5,
    USART6,
    UART7,
    UART8
};

static IRQn_Type ARM_USART_IrqNumber[TEST_APP_ARM_USART_TYPES] = {
    USART1_IRQn,
    USART2_IRQn,
    USART3_IRQn,
    UART4_IRQn,
    UART5_IRQn,
    USART6_IRQn,
    UART7_IRQn,
    UART8_IRQn
};

static eTEST_APP_TimerTypes_t ARM_USART_TimeoutTimer[TEST_APP_ARM_USART_TYPES][ARM_USART_CHANS] = {
    {TIMER_USART1_TIMEOUT_TX,    TIMER_USART1_TIMEOUT_RX},
    {TIMER_USART2_TIMEOUT_TX,    TIMER_USART2_TIMEOUT_RX},
    {TIMER_USART3_TIMEOUT_TX,    TIMER_USART3_TIMEOUT_RX},
    {TIMER_UART4_TIMEOUT_TX,     TIMER_UART4_TIMEOUT_RX},
    {TIMER_UART5_TIMEOUT_TX,     TIMER_UART5_TIMEOUT_RX},
    {TIMER_USART6_TIMEOUT_TX,    TIMER_USART6_TIMEOUT_RX},
    {TIMER_UART7_TIMEOUT_TX,     TIMER_UART7_TIMEOUT_RX},
    {TIMER_UART8_TIMEOUT_TX,     TIMER_UART8_TIMEOUT_RX,}
};


//USART with DMA
static eTEST_APP_ARM_DMA_Chan_t ARM_USART_DMA_ChanDef[TEST_APP_ARM_USART_TYPES][ARM_USART_CHANS] = {
    {TEST_APP_ARM_DMA_CHAN_UNDEFINED,   TEST_APP_ARM_DMA_CHAN_UNDEFINED},
    {TEST_APP_ARM_DMA_CHAN_UNDEFINED,   TEST_APP_ARM_DMA_CHAN_UNDEFINED},
    {TEST_APP_ARM_DMA_CHAN_UNDEFINED,   TEST_APP_ARM_DMA_CHAN_UNDEFINED},
    {TEST_APP_ARM_DMA2_CHAN5,           TEST_APP_ARM_DMA2_CHAN3},
    {TEST_APP_ARM_DMA1_CHAN5,           TEST_APP_ARM_DMA1_CHAN4},
    {TEST_APP_ARM_DMA_CHAN_UNDEFINED,   TEST_APP_ARM_DMA_CHAN_UNDEFINED},
    {TEST_APP_ARM_DMA1_CHAN3,           TEST_APP_ARM_DMA1_CHAN2},
    {TEST_APP_ARM_DMA2_CHAN6,           TEST_APP_ARM_DMA2_CHAN4}
};

static confirm_state ARM_USART_DMA_FlexModeEnable[TEST_APP_ARM_USART_TYPES][ARM_USART_CHANS] = {
    {ARM_USART_DMA_FLEX_MODE_DISABLE,    ARM_USART_DMA_FLEX_MODE_DISABLE},
    {ARM_USART_DMA_FLEX_MODE_DISABLE,    ARM_USART_DMA_FLEX_MODE_DISABLE},
    {ARM_USART_DMA_FLEX_MODE_DISABLE,    ARM_USART_DMA_FLEX_MODE_DISABLE},
    {ARM_USART_DMA_FLEX_MODE_DISABLE,    ARM_USART_DMA_FLEX_MODE_DISABLE},
    {ARM_USART_DMA_FLEX_MODE_ENABLE,     ARM_USART_DMA_FLEX_MODE_ENABLE},
    {ARM_USART_DMA_FLEX_MODE_DISABLE,    ARM_USART_DMA_FLEX_MODE_DISABLE},
    {ARM_USART_DMA_FLEX_MODE_ENABLE,     ARM_USART_DMA_FLEX_MODE_ENABLE},
    {ARM_USART_DMA_FLEX_MODE_ENABLE,     ARM_USART_DMA_FLEX_MODE_ENABLE}
};

static dma_flexible_request_type ARM_USART_DMA_FlexPeriphReq[TEST_APP_ARM_USART_TYPES][ARM_USART_CHANS] = {
    {DMA_FLEXIBLE_UART1_TX, DMA_FLEXIBLE_UART1_RX},
    {DMA_FLEXIBLE_UART2_TX, DMA_FLEXIBLE_UART2_RX},
    {DMA_FLEXIBLE_UART3_TX, DMA_FLEXIBLE_UART3_RX},
    {DMA_FLEXIBLE_UART4_TX, DMA_FLEXIBLE_UART4_RX},
    {DMA_FLEXIBLE_UART5_TX, DMA_FLEXIBLE_UART5_RX},
    {DMA_FLEXIBLE_UART6_TX, DMA_FLEXIBLE_UART6_RX},
    {DMA_FLEXIBLE_UART7_TX, DMA_FLEXIBLE_UART7_RX},
    {DMA_FLEXIBLE_UART8_TX, DMA_FLEXIBLE_UART8_RX}
};



//================================================================================
//Public
//================================================================================

void TEST_APP_ARM_USART_StartUp(void)
{
    TEST_APP_ARM_USART_Resources_t *p_res = &ARM_USART_Resources[TEST_APP_ARM_USART1];
#ifdef _TEST_APP_UART4_ENABLE_
    (p_res[TEST_APP_ARM_UART4]).Status.DrvStateOn = TRUE;
    (p_res[TEST_APP_ARM_UART4]).Status.DrvStatus = TEST_APP_ARM_DRIVER_NO_ERROR;
    (p_res[TEST_APP_ARM_UART4]).Status.DrvFlag = 0x0000;
#ifdef _TEST_APP_UART4_TX_USE_DMA_
    (p_res[TEST_APP_ARM_UART4]).DMA.TxEnable = TRUE;
#endif //_TEST_APP_UART4_TX_USE_DMA_
#ifdef _TEST_APP_UART4_RX_USE_DMA_
    (p_res[TEST_APP_ARM_UART4]).DMA.RxEnable = TRUE;
#endif //_TEST_APP_UART4_RX_USE_DMA_    
#else
#if (defined _TEST_APP_UART4_TX_USE_DMA_ || defined _TEST_APP_UART4_RX_USE_DMA_)
#error "UART4 DMA configuration error"
#endif //(defined _TEST_APP_UART4_TX_USE_DMA_ || defined _TEST_APP_UART4_RX_USE_DMA_)
#endif //_TEST_APP_UART4_ENABLE_

#ifdef _TEST_APP_UART5_ENABLE_
    (p_res[TEST_APP_ARM_UART5]).Status.DrvStateOn = TRUE;
    (p_res[TEST_APP_ARM_UART5]).Status.DrvStatus = TEST_APP_ARM_DRIVER_NO_ERROR;
    (p_res[TEST_APP_ARM_UART5]).Status.DrvFlag = 0x0000;
#ifdef _TEST_APP_UART5_TX_USE_DMA_
    (p_res[TEST_APP_ARM_UART5]).DMA.TxEnable = TRUE;
#endif //_TEST_APP_UART5_TX_USE_DMA_
#ifdef _TEST_APP_UART5_RX_USE_DMA_
    (p_res[TEST_APP_ARM_UART5]).DMA.RxEnable = TRUE;
#endif //_TEST_APP_UART5_RX_USE_DMA_    
#else
#if (defined _TEST_APP_UART5_TX_USE_DMA_ || defined _TEST_APP_UART5_RX_USE_DMA_)
#error "UART5 DMA configuration error"
#endif //(defined _TEST_APP_UART5_TX_USE_DMA_ || defined _TEST_APP_UART5_RX_USE_DMA_)
#endif //_TEST_APP_UART5_ENABLE_

#ifdef _TEST_APP_UART7_ENABLE_
    (p_res[TEST_APP_ARM_UART7]).Status.DrvStateOn = TRUE;
    (p_res[TEST_APP_ARM_UART7]).Status.DrvStatus = TEST_APP_ARM_DRIVER_NO_ERROR;
    (p_res[TEST_APP_ARM_UART7]).Status.DrvFlag = 0x0000;
#ifdef _TEST_APP_UART7_TX_USE_DMA_
    (p_res[TEST_APP_ARM_UART7]).DMA.TxEnable = TRUE;
#endif //_TEST_APP_UART7_TX_USE_DMA_
#ifdef _TEST_APP_UART7_RX_USE_DMA_
    (p_res[TEST_APP_ARM_UART7]).DMA.RxEnable = TRUE;
#endif //_TEST_APP_UART7_RX_USE_DMA_    
#else
#if (defined _TEST_APP_UART7_TX_USE_DMA_ || defined _TEST_APP_UART7_RX_USE_DMA_)
#error "UART7 DMA configuration error"
#endif //(defined _TEST_APP_UART7_TX_USE_DMA_ || defined _TEST_APP_UART7_RX_USE_DMA_)
#endif //_TEST_APP_UART7_ENABLE_

#ifdef _TEST_APP_UART8_ENABLE_
    (p_res[TEST_APP_ARM_UART8]).Status.DrvStateOn = TRUE;
    (p_res[TEST_APP_ARM_UART8]).Status.DrvStatus = TEST_APP_ARM_DRIVER_NO_ERROR;
    (p_res[TEST_APP_ARM_UART8]).Status.DrvFlag = 0x0000;
#ifdef _TEST_APP_UART8_TX_USE_DMA_
    (p_res[TEST_APP_ARM_UART8]).DMA.TxEnable = TRUE;
#endif //_TEST_APP_UART8_TX_USE_DMA_
#ifdef _TEST_APP_UART8_RX_USE_DMA_
    (p_res[TEST_APP_ARM_UART8]).DMA.RxEnable = TRUE;
#endif //_TEST_APP_UART8_RX_USE_DMA_    
#else
#if (defined _TEST_APP_UART8_TX_USE_DMA_ || defined _TEST_APP_UART8_RX_USE_DMA_)
#error "UART8 DMA configuration error"
#endif //(defined _TEST_APP_UART8_TX_USE_DMA_ || defined _TEST_APP_UART8_RX_USE_DMA_)
#endif //_TEST_APP_UART8_ENABLE_
}

void TEST_APP_ARM_USART_IRQHandler(eTEST_APP_ARM_USART_Types_t usart_type)
{
    uint32_t event = 0;
    TEST_APP_ARM_USART_Resources_t *p_res = &ARM_USART_Resources[usart_type];
    if(p_res->Status.XferStatus.RxBusy == 1) {
        if(usart_flag_get(pARM_USART_Register[usart_type], USART_RDBF_FLAG) == SET) {
            *((uint8_t *)p_res->Transfer.pRxData + p_res->Transfer.RxCnt) = ARM_USART_ReadByte(usart_type);
            p_res->Transfer.RxCnt++;
        }
        if(usart_flag_get(pARM_USART_Register[usart_type], USART_ROERR_FLAG) == SET) {
            p_res->Status.XferStatus.RxOverflow = 1;
            event |= TEST_APP_ARM_USART_EVENT_RX_OVERFLOW;
        }
        if(usart_flag_get(pARM_USART_Register[usart_type], USART_FERR_FLAG) == SET) {
            p_res->Status.XferStatus.RxFramingError = 1;
            event |= TEST_APP_ARM_USART_EVENT_RX_FRAMING_ERROR;
        }
        if(usart_flag_get(pARM_USART_Register[usart_type], USART_NERR_FLAG) == SET) {
            p_res->Status.XferStatus.RxNoiseError = 1;
            event |= TEST_APP_ARM_USART_EVENT_RX_NOISE_ERROR;
        }
        if(usart_flag_get(pARM_USART_Register[usart_type], USART_PERR_FLAG) == SET) {
            p_res->Status.XferStatus.RxParityError = 1;
            event |= TEST_APP_ARM_USART_EVENT_RX_PARITY_ERROR;
        }
        if(p_res->Transfer.RxCnt == p_res->Transfer.RxNum) {
            event |= TEST_APP_ARM_USART_EVENT_RX_COMPLETE;
            usart_interrupt_enable(pARM_USART_Register[usart_type], USART_RDBF_INT, FALSE);
            usart_interrupt_enable(pARM_USART_Register[usart_type], USART_ERR_INT, FALSE);
            usart_interrupt_enable(pARM_USART_Register[usart_type], USART_PERR_INT, FALSE);
            p_res->Status.XferStatus.RxBusy = 0;
        }
    }
    if(p_res->Status.XferStatus.TxBusy == 1) {
        if(usart_flag_get(pARM_USART_Register[usart_type], USART_TDBE_FLAG) == SET) {
            ARM_USART_WriteByte(usart_type,
                                ((uint8_t *)p_res->Transfer.pTxData + p_res->Transfer.TxCnt));
            p_res->Transfer.TxCnt++;
            if(p_res->Transfer.TxCnt == p_res->Transfer.TxNum) {
                while(usart_flag_get(pARM_USART_Register[usart_type], USART_TDC_FLAG != SET));
                event |= TEST_APP_ARM_USART_EVENT_TX_COMPLETE;
                usart_interrupt_enable(pARM_USART_Register[usart_type], USART_TDBE_INT, FALSE);
                p_res->Status.XferStatus.TxBusy = 0;
            }
        }
    }
    if(event) {
        TEST_APP_RingBuffer_Write(&(p_res->Event), &event);
    }
}

//================================================================================
//Private
//================================================================================

static void  ARM_USART_SetResources(TEST_APP_ARM_USART_Resources_t *p_res, eTEST_APP_ARM_USART_Types_t usart_type,
                                    uint32_t baudrate,
                                    usart_data_bit_num_type data_bit,
                                    usart_stop_bit_num_type stop_bit,
                                    usart_parity_selection_type parity,
                                    eTEST_APP_ARM_USART_PinDefTypes_t gpio_pin_def_type)
{
    p_res->usart_type = usart_type;
    p_res->IrqNum = ARM_USART_IrqNumber[usart_type];
    p_res->GpioPinDefType = gpio_pin_def_type;
    p_res->Config.BaudRate = baudrate;
    p_res->Config.DataBit = data_bit;
    p_res->Config.StopBit = stop_bit;
    p_res->Config.Parity = parity;

    TEST_APP_RingBuffer_Init(&(p_res->Event), &ARM_USART_EventBuff[usart_type], ARM_USART_EVENT_BUFF_SIZE);
    p_res->Transfer.pTxData = &ARM_USART_TxBuff[usart_type];
    p_res->Transfer.pRxData = &ARM_USART_RxBuff[usart_type];;
    p_res->Transfer.TxNum = 0;
    p_res->Transfer.RxNum = 0;
    p_res->Transfer.TxCnt = 0;
    p_res->Transfer.RxCnt = 0;
    memcpy(&p_res->Gpio, &ARM_USART_GPIO_Def[usart_type][gpio_pin_def_type],
           sizeof(TEST_APP_ARM_USART_GPIO_t));
    p_res->DMA.TxChan = ARM_USART_DMA_ChanDef[usart_type][ARM_USART_TX_CHAN];
    p_res->DMA.RxChan = ARM_USART_DMA_ChanDef[usart_type][ARM_USART_RX_CHAN];
    p_res->DMA.TxEvent_cb = pTEST_APP_ARM_USART_DMA_cb[usart_type][ARM_USART_TX_CHAN];
    p_res->DMA.RxEvent_cb = pTEST_APP_ARM_USART_DMA_cb[usart_type][ARM_USART_RX_CHAN];
    p_res->DMA.TxFlexModeEnable = ARM_USART_DMA_FlexModeEnable[usart_type][ARM_USART_TX_CHAN];
    p_res->DMA.RxFlexModeEnable = ARM_USART_DMA_FlexModeEnable[usart_type][ARM_USART_RX_CHAN];
    p_res->DMA.TxFlexPeriphReq = ARM_USART_DMA_FlexPeriphReq[usart_type][ARM_USART_TX_CHAN];
    p_res->DMA.RxFlexPeriphReq = ARM_USART_DMA_FlexPeriphReq[usart_type][ARM_USART_RX_CHAN];
}

static uint32_t ARM_USART_GPIO_Config(eTEST_APP_ARM_USART_Types_t usart_type, confirm_state new_state)
{
    TEST_APP_ARM_USART_Resources_t *p_res = &ARM_USART_Resources[usart_type];
    if(new_state) {
        if(!TEST_APP_ARM_CRM_PeriphClockEnable(TEST_APP_PERIPH_GPIO, p_res->Gpio.TxPort, TRUE)) {
#ifdef _TEST_APP_DEBUG_
            LOG("USART configuration error");
#endif//_TEST_APP_DEBUG_        
            return TEST_APP_ARM_DRIVER_ERROR;
        }
        if(!TEST_APP_ARM_CRM_PeriphClockEnable(TEST_APP_PERIPH_GPIO, p_res->Gpio.RxPort, TRUE)) {
#ifdef _TEST_APP_DEBUG_
            LOG("USART configuration error");
#endif//_TEST_APP_DEBUG_
            return TEST_APP_ARM_DRIVER_ERROR;
        }
        //enable GPIO IOMUX clock (for pin remapping)
        if(!(p_res->GpioPinDefType == TEST_APP_ARM_USART_GPIO_PIN_DEF_TYPE_DEFAULT)) {
            crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
            gpio_pin_remap_config(ARM_USART_GPIO_IOMUX_Remap_Def[usart_type], TRUE);
            //in remap mode configure both Tx and Rx as multiplexed function mode
            TEST_APP_ARM_GPIO_Config(p_res->Gpio.TxPort, p_res->Gpio.TxPin, GPIO_MODE_MUX, GPIO_OUTPUT_PUSH_PULL,
                                     GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_STRONGER);
            TEST_APP_ARM_GPIO_Config(p_res->Gpio.RxPort, p_res->Gpio.RxPin, GPIO_MODE_MUX, GPIO_OUTPUT_PUSH_PULL,
                                     GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_STRONGER);
        } else {
            //configure Tx as multiplexed function mode
            TEST_APP_ARM_GPIO_Config(p_res->Gpio.TxPort, p_res->Gpio.TxPin, GPIO_MODE_MUX, GPIO_OUTPUT_PUSH_PULL,
                                     GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_STRONGER);
            //configure Rx as input, use any out_type and drive_strength for input i/o:
            TEST_APP_ARM_GPIO_Config(p_res->Gpio.RxPort, p_res->Gpio.RxPin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                                     GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
        }
    } else {
        //release pins
        TEST_APP_ARM_GPIO_Release(p_res->Gpio.TxPort, p_res->Gpio.TxPin);
        TEST_APP_ARM_GPIO_Release(p_res->Gpio.RxPort, p_res->Gpio.RxPin);
        //disable remap (for pins remapping release)
        if(!(p_res->GpioPinDefType == TEST_APP_ARM_USART_GPIO_PIN_DEF_TYPE_DEFAULT)) {
            gpio_pin_remap_config(ARM_USART_GPIO_IOMUX_Remap_Def[usart_type], FALSE);
        }
    }
    return TEST_APP_ARM_DRIVER_NO_ERROR;
}

static uint32_t ARM_USART_Initialize(eTEST_APP_ARM_USART_Types_t usart_type,
                                     uint32_t baudrate,
                                     usart_data_bit_num_type data_bit,
                                     usart_stop_bit_num_type stop_bit,
                                     usart_parity_selection_type parity,
                                     eTEST_APP_ARM_USART_PinDefTypes_t gpio_pin_def_type)
{
    TEST_APP_ARM_USART_Resources_t *p_res = &ARM_USART_Resources[usart_type];
    uint32_t drv_status = TEST_APP_ARM_DRIVER_NO_ERROR;
    ARM_USART_SetResources(p_res, usart_type, baudrate, data_bit, stop_bit, parity, gpio_pin_def_type);
    if(!(TEST_APP_ARM_CRM_PeriphClockEnable(TEST_APP_PERIPH_USART, p_res->usart_type, TRUE))) {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
        return TEST_APP_ARM_DRIVER_ERROR;
    }
//asynchronous mode is default
    TEST_APP_ARM_CRM_PeriphReset(TEST_APP_PERIPH_USART, p_res->usart_type, TRUE);
    TEST_APP_ARM_CRM_PeriphReset(TEST_APP_PERIPH_USART, p_res->usart_type, FALSE);
    usart_enable(pARM_USART_Register[usart_type], TRUE);
    usart_init(pARM_USART_Register[usart_type], p_res->Config.BaudRate,
               p_res->Config.DataBit, p_res->Config.StopBit);
    usart_parity_selection_config(pARM_USART_Register[usart_type],
                                  p_res->Config.Parity);
    p_res->Status.DrvFlag |= TEST_APP_ARM_USART_DRIVER_FLAG_CONFIGURATED;
    if(p_res->DMA.TxEnable) {
        //clock and reset DMA
        if(!TEST_APP_ARM_DMA_Init(p_res->DMA.TxChan, p_res->DMA.TxEvent_cb)) {
#ifdef _TEST_APP_DEBUG_
            LOG("USART DMA driver error");
#endif//_TEST_APP_DEBUG_
            p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
            return TEST_APP_ARM_DRIVER_ERROR;
        }
        if(p_res->DMA.TxFlexModeEnable) {
            TEST_APP_ARM_DMA_FlexibleConfig(p_res->DMA.TxChan, p_res->DMA.TxFlexPeriphReq);
        }
        TEST_APP_DMA_ClearAndEnableIRQ(p_res->DMA.TxChan);
        usart_dma_transmitter_enable(pARM_USART_Register[usart_type], TRUE);
        p_res->Status.DrvFlag |= TEST_APP_ARM_USART_DRIVER_FLAG_DMA_TX_ENABLED;

    }
    if(p_res->DMA.RxEnable) {
        //clock and reset DMA
        if(!TEST_APP_ARM_DMA_Init(p_res->DMA.RxChan, p_res->DMA.RxEvent_cb)) {
#ifdef _TEST_APP_DEBUG_
            LOG("USART DMA driver error");
#endif//_TEST_APP_DEBUG_
            p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
            return TEST_APP_ARM_DRIVER_ERROR;
        }
        if(p_res->DMA.RxFlexModeEnable) {
            TEST_APP_ARM_DMA_FlexibleConfig(p_res->DMA.RxChan, p_res->DMA.RxFlexPeriphReq);
        }
        TEST_APP_DMA_ClearAndEnableIRQ(p_res->DMA.RxChan);
        usart_dma_receiver_enable(pARM_USART_Register[usart_type], TRUE);
        p_res->Status.DrvFlag |= TEST_APP_ARM_USART_DRIVER_FLAG_DMA_RX_ENABLED;
    }
    usart_transmitter_enable(pARM_USART_Register[usart_type], TRUE);
    p_res->Status.DrvFlag |= TEST_APP_ARM_USART_DRIVER_FLAG_TX_ENABLED;
    usart_receiver_enable(pARM_USART_Register[usart_type], TRUE);
    p_res->Status.DrvFlag |= TEST_APP_ARM_USART_DRIVER_FLAG_RX_ENABLED;
    drv_status |= ARM_USART_GPIO_Config(usart_type, TRUE);
//clear and enable UARTx IRQ
    NVIC_ClearPendingIRQ(p_res->IrqNum);
    NVIC_EnableIRQ(p_res->IrqNum);
    p_res->Status.DrvFlag |= TEST_APP_ARM_USART_DRIVER_FLAG_INITIALIZED;
    p_res->Status.DrvStatus |= drv_status;
    return drv_status;
}

static uint32_t ARM_USART_Uninitialize(eTEST_APP_ARM_USART_Types_t usart_type)
{
    TEST_APP_ARM_USART_Resources_t *p_res = &ARM_USART_Resources[usart_type];
    uint32_t drv_status = TEST_APP_ARM_DRIVER_NO_ERROR;
    if(p_res->Status.DrvFlag & TEST_APP_ARM_USART_DRIVER_FLAG_INITIALIZED) {
        //disable and clear UARTx IRQ
        NVIC_DisableIRQ(p_res->IrqNum);
        NVIC_ClearPendingIRQ(p_res->IrqNum);
        if(p_res->DMA.TxEnable) {
            usart_dma_transmitter_enable(pARM_USART_Register[usart_type], FALSE);
            p_res->Status.DrvFlag &= ~TEST_APP_ARM_USART_DRIVER_FLAG_DMA_TX_ENABLED;

            TEST_APP_DMA_DisableAndClearIRQ(p_res->DMA.TxChan);
            TEST_APP_DMA_Enable(p_res->DMA.TxChan, FALSE);
        }
        if(p_res->DMA.RxEnable) {
            usart_dma_receiver_enable(pARM_USART_Register[usart_type], FALSE);
            p_res->Status.DrvFlag &= ~TEST_APP_ARM_USART_DRIVER_FLAG_DMA_TX_ENABLED;
            TEST_APP_DMA_DisableAndClearIRQ(p_res->DMA.RxChan);
            TEST_APP_DMA_Enable(p_res->DMA.RxChan, FALSE);
        }
        usart_transmitter_enable(pARM_USART_Register[usart_type], FALSE);
        p_res->Status.DrvFlag &= ~TEST_APP_ARM_USART_DRIVER_FLAG_TX_ENABLED;
        usart_receiver_enable(pARM_USART_Register[usart_type], FALSE);
        p_res->Status.DrvFlag &= ~TEST_APP_ARM_USART_DRIVER_FLAG_RX_ENABLED;
        drv_status |= ARM_USART_GPIO_Config(usart_type, FALSE);
        usart_enable(pARM_USART_Register[usart_type], FALSE);
        TEST_APP_ARM_CRM_PeriphReset(TEST_APP_PERIPH_USART, p_res->usart_type, TRUE);
        p_res->Status.DrvFlag &= ~TEST_APP_ARM_USART_DRIVER_FLAG_CONFIGURATED;
        if(!(TEST_APP_ARM_CRM_PeriphClockEnable(TEST_APP_PERIPH_USART, p_res->usart_type, FALSE))) {
            p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
            return TEST_APP_ARM_DRIVER_ERROR;
        }
        memset(p_res, 0, sizeof(TEST_APP_ARM_USART_Resources_t));
        p_res->Status.DrvFlag &= ~TEST_APP_ARM_USART_DRIVER_FLAG_INITIALIZED;
    } else {
#ifdef _TEST_APP_DEBUG_
        LOG("USART not been initialized");
#endif//_TEST_APP_DEBUG_
    }
    p_res->Status.DrvStatus |= drv_status;
    return drv_status;
}

static void ARM_USART_Event_cb(eTEST_APP_ARM_USART_Types_t usart_type)
{
    TEST_APP_ARM_USART_Resources_t *p_res = &ARM_USART_Resources[usart_type];
    uint32_t event = 0;
    TEST_APP_RingBuffer_Read(&(p_res->Event), &event);
    if(event & TEST_APP_ARM_USART_EVENT_RX_COMPLETE) {
        p_res->Transfer.RxCnt = 0;
        p_res->Transfer.RxNum = 0;
#ifdef _TEST_APP_DEBUG_
        TEST_APP_ARM_USART_Driver_t *p_drv = pARM_USART_Driver[TEST_APP_ARM_USART1];
        p_drv[usart_type].Send(p_res->Transfer.pRxData, 8);
        TEST_APP_LCD2004_Printf(0, 0, SET, "%s", p_res->Transfer.pRxData);
        LOG("USART recieved test data");
#endif//_TEST_APP_DEBUG_ 
    }
    if(event & TEST_APP_ARM_USART_EVENT_TX_COMPLETE) {
        p_res->Transfer.TxCnt = 0;
        p_res->Transfer.TxNum = 0;
#ifdef _TEST_APP_DEBUG_
        LOG("USART transmitted test data");
#endif//_TEST_APP_DEBUG_                 
    }
    if(event & TEST_APP_ARM_USART_EVENT_RX_OVERFLOW) {
#ifdef _TEST_APP_DEBUG_
        LOG("USART receiver overflow");
#endif//_TEST_APP_DEBUG_                
    }
    if(event & TEST_APP_ARM_USART_EVENT_RX_FRAMING_ERROR) {
#ifdef _TEST_APP_DEBUG_
        LOG("USART receiver framing error");
#endif//_TEST_APP_DEBUG_                
    }
    if(event & TEST_APP_ARM_USART_EVENT_RX_NOISE_ERROR) {
#ifdef _TEST_APP_DEBUG_
        LOG("USART receiver noise error");
#endif//_TEST_APP_DEBUG_                
    }
    if(event & TEST_APP_ARM_USART_EVENT_RX_PARITY_ERROR) {
#ifdef _TEST_APP_DEBUG_
        LOG("USART receiver parity error");
#endif//_TEST_APP_DEBUG_
    }
}

static void ARM_USART_DMA_Event_cb(uint32_t event,
                                   eTEST_APP_ARM_USART_Types_t usart_type,
                                   eARM_USART_Chans_t chan_type)
{
    TEST_APP_ARM_USART_Resources_t *p_res = &ARM_USART_Resources[usart_type];
    switch(chan_type) {
        case ARM_USART_TX_CHAN: {
            if(event & TEST_APP_ARM_DMA_EVENT_FULL_DATA) {
                p_res->Status.XferStatus.TxBusy = 0;
                p_res->Transfer.TxCnt = 0;
                p_res->Transfer.TxNum = 0;

#ifdef _TEST_APP_DEBUG_
                LOG("USART transmitted test data via DMA");
#endif//_TEST_APP_DEBUG_
            }
            if(event & TEST_APP_ARM_DMA_EVENT_DATA_ERROR) {
#ifdef _TEST_APP_DEBUG_
                LOG("USART DMA data error");
#endif//_TEST_APP_DEBUG_
            }
            break;
        }
        case ARM_USART_RX_CHAN: {
            if(event & TEST_APP_ARM_DMA_EVENT_FULL_DATA) {
                p_res->Status.XferStatus.RxBusy = 0;
                p_res->Transfer.RxCnt = 0;
                p_res->Transfer.RxNum = 0;
                TEST_APP_ARM_USART_Driver_t *p_drv = pARM_USART_Driver[TEST_APP_ARM_USART1];
                p_drv[usart_type].Send(p_res->Transfer.pRxData, 8);
                TEST_APP_LCD2004_Printf(0, 0, SET, "%s", p_res->Transfer.pRxData);
#ifdef _TEST_APP_DEBUG_
                TEST_APP_LCD2004_Printf(0, 0, SET, "%s", p_res->Transfer.pRxData);
                LOG("USART recieved test data via DMA");
#endif//_TEST_APP_DEBUG_
            }
            if(event & TEST_APP_ARM_DMA_EVENT_DATA_ERROR) {
#ifdef _TEST_APP_DEBUG_
                LOG("USART DMA data error");
#endif//_TEST_APP_DEBUG_
            }
            break;
        }
    }
}

static uint32_t ARM_USART_Recieve(eTEST_APP_ARM_USART_Types_t usart_type, void *pdata, uint32_t num)
{
    TEST_APP_ARM_USART_Resources_t *p_res = &ARM_USART_Resources[usart_type];
    if(!(p_res->Status.DrvFlag & TEST_APP_ARM_USART_DRIVER_FLAG_RX_ENABLED)) {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
        return TEST_APP_ARM_DRIVER_ERROR;
    }
    if(num == 0) {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR_PARAMETER;
        return TEST_APP_ARM_DRIVER_ERROR_PARAMETER;
    }
    if(p_res->Status.XferStatus.RxBusy) {
        TimerEnable(ARM_USART_TimeoutTimer[usart_type][ARM_USART_RX_CHAN], ARM_USART_TIMEOUT_MSEC);
        while(p_res->Status.XferStatus.RxBusy && (!TimerTestFlag(ARM_USART_TimeoutTimer[usart_type][ARM_USART_RX_CHAN])));
        if(TimerTestFlag(ARM_USART_TimeoutTimer[usart_type][ARM_USART_RX_CHAN])) {
            p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR_BUSY;
            return TEST_APP_ARM_DRIVER_ERROR_BUSY;
        } else {
            p_res->Status.DrvStatus &= ~TEST_APP_ARM_DRIVER_ERROR_BUSY;
        }
        TimerDisable(ARM_USART_TimeoutTimer[usart_type][ARM_USART_RX_CHAN]);
    }
    p_res->Status.XferStatus.RxBusy = 1;
    usart_interrupt_enable(pARM_USART_Register[usart_type], USART_ERR_INT, TRUE);
    usart_interrupt_enable(pARM_USART_Register[usart_type], USART_PERR_INT, TRUE);
    if(p_res->DMA.RxEnable) {
        TEST_APP_DMA_Enable(p_res->DMA.RxChan, FALSE);
        TEST_APP_ARM_DMA_Config(p_res->DMA.RxChan,
                                (uint32_t)(&pARM_USART_Register[usart_type]->dt),
                                (uint32_t)(volatile void *)pdata,
                                num, DMA_DIR_PERIPHERAL_TO_MEMORY,
                                TEST_APP_ARM_DMA_LOOP_MODE_DISABLE,
                                DMA_PRIORITY_LOW);
        TEST_APP_DMA_InterruptEnable(p_res->DMA.RxChan, TEST_APP_ARM_DMA_ERR_DATA_TRANSFER_INT,
                                     TRUE);
        TEST_APP_DMA_InterruptEnable(p_res->DMA.RxChan, TEST_APP_ARM_DMA_FULL_DATA_TRANSFER_INT,
                                     TRUE);
        TEST_APP_DMA_Enable(p_res->DMA.RxChan, TRUE);
    } else {
        p_res->Transfer.RxCnt = 0;
        p_res->Transfer.RxNum = num;
        p_res->Transfer.pRxData = pdata;
        usart_interrupt_enable(pARM_USART_Register[usart_type], USART_RDBF_INT, TRUE);
    }
    return TEST_APP_ARM_DRIVER_NO_ERROR;
}

static uint32_t ARM_USART_Send(eTEST_APP_ARM_USART_Types_t usart_type, void *pdata, uint32_t num)
{
    TEST_APP_ARM_USART_Resources_t *p_res = &ARM_USART_Resources[usart_type];
    if(!(p_res->Status.DrvFlag & TEST_APP_ARM_USART_DRIVER_FLAG_TX_ENABLED)) {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
        return TEST_APP_ARM_DRIVER_ERROR;
    }
    if(num == 0) {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR_PARAMETER;
        return TEST_APP_ARM_DRIVER_ERROR_PARAMETER;
    }

    if(p_res->Status.XferStatus.TxBusy) {
        TimerEnable(ARM_USART_TimeoutTimer[usart_type][ARM_USART_TX_CHAN], ARM_USART_TIMEOUT_MSEC);
        while(p_res->Status.XferStatus.TxBusy && (!TimerTestFlag(ARM_USART_TimeoutTimer[usart_type][ARM_USART_TX_CHAN])));
        if(TimerTestFlag(ARM_USART_TimeoutTimer[usart_type][ARM_USART_TX_CHAN])) {
            p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR_BUSY;
            return TEST_APP_ARM_DRIVER_ERROR_BUSY;
        } else {
            p_res->Status.DrvStatus &= ~TEST_APP_ARM_DRIVER_ERROR_BUSY;
        }
        TimerDisable(ARM_USART_TimeoutTimer[usart_type][ARM_USART_TX_CHAN]);
    }
    p_res->Status.XferStatus.TxBusy = 1;
    if(p_res->DMA.TxEnable) {
        TEST_APP_DMA_Enable(p_res->DMA.TxChan, FALSE);
        TEST_APP_ARM_DMA_Config(p_res->DMA.TxChan,
                                (uint32_t)(&pARM_USART_Register[usart_type]->dt),
                                (uint32_t)pdata,
                                num, DMA_DIR_MEMORY_TO_PERIPHERAL,
                                TEST_APP_ARM_DMA_LOOP_MODE_DISABLE,
                                DMA_PRIORITY_LOW);
        TEST_APP_DMA_InterruptEnable(p_res->DMA.TxChan, TEST_APP_ARM_DMA_ERR_DATA_TRANSFER_INT,
                                     TRUE);
        TEST_APP_DMA_InterruptEnable(p_res->DMA.TxChan, TEST_APP_ARM_DMA_FULL_DATA_TRANSFER_INT,
                                     TRUE);
        TEST_APP_DMA_Enable(p_res->DMA.TxChan, TRUE);
    } else {
        p_res->Transfer.TxCnt = 0;
        p_res->Transfer.TxNum = num;
        p_res->Transfer.pTxData = pdata;
        usart_interrupt_enable(pARM_USART_Register[usart_type], USART_TDBE_INT, TRUE);
    }
    return TEST_APP_ARM_DRIVER_NO_ERROR;
}

static uint8_t ARM_USART_ReadByte(eTEST_APP_ARM_USART_Types_t usart_type)
{
    uint8_t byte;
    byte = pARM_USART_Register[usart_type]->dt;
    return byte;
}

static void ARM_USART_WriteByte(eTEST_APP_ARM_USART_Types_t usart_type, uint8_t *pByte)
{
    pARM_USART_Register[usart_type]->dt = *pByte;
}

static TEST_APP_ARM_USART_Transfer_t ARM_USART_GetTransfer(eTEST_APP_ARM_USART_Types_t usart_type)
{
    TEST_APP_ARM_USART_Resources_t *p_res = &ARM_USART_Resources[usart_type];
    TEST_APP_ARM_USART_Transfer_t transfer;
    transfer.pTxData = p_res->Transfer.pTxData;
    transfer.pRxData = p_res->Transfer.pRxData;
    transfer.TxNum = p_res->Transfer.TxNum;
    transfer.RxNum = p_res->Transfer.RxNum;
    transfer.TxCnt = p_res->Transfer.TxCnt;
    transfer.RxCnt = p_res->Transfer.RxCnt;
    return transfer;
}

static TEST_APP_ARM_USART_Status_t ARM_USART_GetStatus(eTEST_APP_ARM_USART_Types_t usart_type)
{
    TEST_APP_ARM_USART_Resources_t *p_res = &ARM_USART_Resources[usart_type];
    TEST_APP_ARM_USART_Status_t status;
    status.DrvStateOn = p_res->Status.DrvStateOn;
    status.DrvFlag = p_res->Status.DrvFlag;
    status.DrvStatus = p_res->Status.DrvStatus;
    status.XferStatus.TxBusy = p_res->Status.XferStatus.TxBusy;
    status.XferStatus.RxBusy = p_res->Status.XferStatus.RxBusy;
    status.XferStatus.TxUnderflow = p_res->Status.XferStatus.TxUnderflow;
    status.XferStatus.RxOverflow = p_res->Status.XferStatus.RxOverflow;
    status.XferStatus.RxBreak = p_res->Status.XferStatus.RxBreak;
    status.XferStatus.RxFramingError = p_res->Status.XferStatus.RxFramingError;
    status.XferStatus.RxParityError = p_res->Status.XferStatus.RxParityError;
    return status;
}


static uint32_t ARM_USART_Initialize_1(uint32_t baudrate,
                                       usart_data_bit_num_type dataBit,
                                       usart_stop_bit_num_type stopBit,
                                       usart_parity_selection_type parity,
                                       eTEST_APP_ARM_USART_PinDefTypes_t gpio_pin_def_type)
{
    return TEST_APP_ARM_DRIVER_ERROR_UNSUPPORTED;
}


static uint32_t ARM_USART_Initialize_2(uint32_t baudrate,
                                       usart_data_bit_num_type dataBit,
                                       usart_stop_bit_num_type stopBit,
                                       usart_parity_selection_type parity,
                                       eTEST_APP_ARM_USART_PinDefTypes_t gpio_pin_def_type)
{
    return TEST_APP_ARM_DRIVER_ERROR_UNSUPPORTED;
}

static uint32_t ARM_USART_Initialize_3(uint32_t baudrate,
                                       usart_data_bit_num_type dataBit,
                                       usart_stop_bit_num_type stopBit,
                                       usart_parity_selection_type parity,
                                       eTEST_APP_ARM_USART_PinDefTypes_t gpio_pin_def_type)
{
    return TEST_APP_ARM_DRIVER_ERROR_UNSUPPORTED;
}

static uint32_t ARM_USART_Initialize_4(uint32_t baudrate,
                                       usart_data_bit_num_type dataBit,
                                       usart_stop_bit_num_type stopBit,
                                       usart_parity_selection_type parity,
                                       eTEST_APP_ARM_USART_PinDefTypes_t gpio_pin_def_type)
{
    return ARM_USART_Initialize(TEST_APP_ARM_UART4,
                                baudrate,
                                dataBit,
                                stopBit,
                                parity,
                                gpio_pin_def_type);
}

static uint32_t ARM_USART_Initialize_5(uint32_t baudrate,
                                       usart_data_bit_num_type dataBit,
                                       usart_stop_bit_num_type stopBit,
                                       usart_parity_selection_type parity,
                                       eTEST_APP_ARM_USART_PinDefTypes_t gpio_pin_def_type)
{
    return ARM_USART_Initialize(TEST_APP_ARM_UART5,
                                baudrate,
                                dataBit,
                                stopBit,
                                parity,
                                gpio_pin_def_type);
}

static uint32_t ARM_USART_Initialize_6(uint32_t baudrate,
                                       usart_data_bit_num_type dataBit,
                                       usart_stop_bit_num_type stopBit,
                                       usart_parity_selection_type parity,
                                       eTEST_APP_ARM_USART_PinDefTypes_t gpio_pin_def_type)
{
    return TEST_APP_ARM_DRIVER_ERROR_UNSUPPORTED;
}

static uint32_t ARM_USART_Initialize_7(uint32_t baudrate,
                                       usart_data_bit_num_type dataBit,
                                       usart_stop_bit_num_type stopBit,
                                       usart_parity_selection_type parity,
                                       eTEST_APP_ARM_USART_PinDefTypes_t gpio_pin_def_type)
{
    return ARM_USART_Initialize(TEST_APP_ARM_UART7,
                                baudrate,
                                dataBit,
                                stopBit,
                                parity,
                                gpio_pin_def_type);
}

static uint32_t ARM_USART_Initialize_8(uint32_t baudrate,
                                       usart_data_bit_num_type dataBit,
                                       usart_stop_bit_num_type stopBit,
                                       usart_parity_selection_type parity,
                                       eTEST_APP_ARM_USART_PinDefTypes_t gpio_pin_def_type)
{
    return ARM_USART_Initialize(TEST_APP_ARM_UART8,
                                baudrate,
                                dataBit,
                                stopBit,
                                parity,
                                gpio_pin_def_type);
}

static uint32_t ARM_USART_Uninitialize_1(void)
{
    return TEST_APP_ARM_DRIVER_ERROR_UNSUPPORTED;
}

static uint32_t ARM_USART_Uninitialize_2(void)
{
    return TEST_APP_ARM_DRIVER_ERROR_UNSUPPORTED;
}

static uint32_t ARM_USART_Uninitialize_3(void)
{
    return TEST_APP_ARM_DRIVER_ERROR_UNSUPPORTED;
}

static uint32_t ARM_USART_Uninitialize_4(void)
{
    return ARM_USART_Uninitialize(TEST_APP_ARM_UART4);
}

static uint32_t ARM_USART_Uninitialize_5(void)
{
    return ARM_USART_Uninitialize(TEST_APP_ARM_UART5);
}

static uint32_t ARM_USART_Uninitialize_6(void)
{
    return TEST_APP_ARM_DRIVER_ERROR_UNSUPPORTED;
}

static uint32_t ARM_USART_Uninitialize_7(void)
{
    return ARM_USART_Uninitialize(TEST_APP_ARM_UART7);
}

static uint32_t ARM_USART_Uninitialize_8(void)
{
    return ARM_USART_Uninitialize(TEST_APP_ARM_UART8);
}

static void ARM_USART_Event_cb_1(void)
{
#ifdef _TEST_APP_DEBUG_
    LOG("USART driver unsupported");
#endif//_TEST_APP_DEBUG_
}
static void ARM_USART_Event_cb_2(void)
{
#ifdef _TEST_APP_DEBUG_
    LOG("USART driver unsupported");
#endif//_TEST_APP_DEBUG_   
}
static void ARM_USART_Event_cb_3(void)
{
#ifdef _TEST_APP_DEBUG_
    LOG("USART driver unsupported");
#endif//_TEST_APP_DEBUG_
}
static void ARM_USART_Event_cb_4(void)
{
    ARM_USART_Event_cb(TEST_APP_ARM_UART4);
}
static void ARM_USART_Event_cb_5(void)
{
    ARM_USART_Event_cb(TEST_APP_ARM_UART5);
}
static void ARM_USART_Event_cb_6(void)
{
#ifdef _TEST_APP_DEBUG_
    LOG("USART driver unsupported");
#endif//_TEST_APP_DEBUG_
}
static void ARM_USART_Event_cb_7(void)
{
    ARM_USART_Event_cb(TEST_APP_ARM_UART7);
}
static void ARM_USART_Event_cb_8(void)
{
    ARM_USART_Event_cb(TEST_APP_ARM_UART8);
}

static void ARM_USART1_DMA_TxEvent_cb(uint32_t event)
{
    ARM_USART_DMA_Event_cb(event, TEST_APP_ARM_USART1, ARM_USART_TX_CHAN);
}

static void ARM_USART1_DMA_RxEvent_cb(uint32_t event)
{
    ARM_USART_DMA_Event_cb(event, TEST_APP_ARM_USART1, ARM_USART_RX_CHAN);
}

static void ARM_USART2_DMA_TxEvent_cb(uint32_t event)
{
    ARM_USART_DMA_Event_cb(event, TEST_APP_ARM_USART2, ARM_USART_TX_CHAN);
}

static void ARM_USART2_DMA_RxEvent_cb(uint32_t event)
{
    ARM_USART_DMA_Event_cb(event, TEST_APP_ARM_USART2, ARM_USART_RX_CHAN);
}

static void ARM_USART3_DMA_TxEvent_cb(uint32_t event)
{
    ARM_USART_DMA_Event_cb(event, TEST_APP_ARM_USART3, ARM_USART_TX_CHAN);
}

static void ARM_USART3_DMA_RxEvent_cb(uint32_t event)
{
    ARM_USART_DMA_Event_cb(event, TEST_APP_ARM_USART3, ARM_USART_RX_CHAN);
}

static void ARM_UART4_DMA_TxEvent_cb(uint32_t event)
{
    ARM_USART_DMA_Event_cb(event, TEST_APP_ARM_UART4, ARM_USART_TX_CHAN);
}

static void ARM_UART4_DMA_RxEvent_cb(uint32_t event)
{
    ARM_USART_DMA_Event_cb(event, TEST_APP_ARM_UART4, ARM_USART_RX_CHAN);
}

static void ARM_UART5_DMA_TxEvent_cb(uint32_t event)
{
    ARM_USART_DMA_Event_cb(event, TEST_APP_ARM_UART5, ARM_USART_TX_CHAN);
}

static void ARM_UART5_DMA_RxEvent_cb(uint32_t event)
{
    ARM_USART_DMA_Event_cb(event, TEST_APP_ARM_UART5, ARM_USART_RX_CHAN);
}

static void ARM_USART6_DMA_TxEvent_cb(uint32_t event)
{
    ARM_USART_DMA_Event_cb(event, TEST_APP_ARM_USART6, ARM_USART_TX_CHAN);
}

static void ARM_USART6_DMA_RxEvent_cb(uint32_t event)
{
    ARM_USART_DMA_Event_cb(event, TEST_APP_ARM_USART6, ARM_USART_RX_CHAN);
}

static void ARM_UART7_DMA_TxEvent_cb(uint32_t event)
{
    ARM_USART_DMA_Event_cb(event, TEST_APP_ARM_UART7, ARM_USART_TX_CHAN);
}

static void ARM_UART7_DMA_RxEvent_cb(uint32_t event)
{
    ARM_USART_DMA_Event_cb(event, TEST_APP_ARM_UART7, ARM_USART_RX_CHAN);
}

static void ARM_UART8_DMA_TxEvent_cb(uint32_t event)
{
    ARM_USART_DMA_Event_cb(event, TEST_APP_ARM_UART8, ARM_USART_TX_CHAN);
}

static void ARM_UART8_DMA_RxEvent_cb(uint32_t event)
{
    ARM_USART_DMA_Event_cb(event, TEST_APP_ARM_UART8, ARM_USART_RX_CHAN);
}

static uint32_t ARM_USART_Send_1(void *pdata, uint32_t num)
{
    return TEST_APP_ARM_DRIVER_ERROR_UNSUPPORTED;
}
static uint32_t ARM_USART_Send_2(void *pdata, uint32_t num)
{
    return TEST_APP_ARM_DRIVER_ERROR_UNSUPPORTED;
}
static uint32_t ARM_USART_Send_3(void *pdata, uint32_t num)
{
    return TEST_APP_ARM_DRIVER_ERROR_UNSUPPORTED;
}
static uint32_t ARM_USART_Send_4(void *pdata, uint32_t num)
{
    return ARM_USART_Send(TEST_APP_ARM_UART4, pdata, num);
}
static uint32_t ARM_USART_Send_5(void *pdata, uint32_t num)
{
    return ARM_USART_Send(TEST_APP_ARM_UART5, pdata, num);
}
static uint32_t ARM_USART_Send_6(void *pdata, uint32_t num)
{
    return TEST_APP_ARM_DRIVER_ERROR_UNSUPPORTED;
}
static uint32_t ARM_USART_Send_7(void *pdata, uint32_t num)
{
    return ARM_USART_Send(TEST_APP_ARM_UART7, pdata, num);
}
static uint32_t ARM_USART_Send_8(void *pdata, uint32_t num)
{
    return ARM_USART_Send(TEST_APP_ARM_UART8, pdata, num);
}

static uint32_t ARM_USART_Recieve_1(void *pdata, uint32_t num)
{
    return TEST_APP_ARM_DRIVER_ERROR_UNSUPPORTED;
}
static uint32_t ARM_USART_Recieve_2(void *pdata, uint32_t num)
{
    return TEST_APP_ARM_DRIVER_ERROR_UNSUPPORTED;
}
static uint32_t ARM_USART_Recieve_3(void *pdata, uint32_t num)
{
    return TEST_APP_ARM_DRIVER_ERROR_UNSUPPORTED;
}
static uint32_t ARM_USART_Recieve_4(void *pdata, uint32_t num)
{
    return ARM_USART_Recieve(TEST_APP_ARM_UART4, pdata, num);
}
static uint32_t ARM_USART_Recieve_5(void *pdata, uint32_t num)
{
    return ARM_USART_Recieve(TEST_APP_ARM_UART5, pdata, num);
}
static uint32_t ARM_USART_Recieve_6(void *pdata, uint32_t num)
{
    return TEST_APP_ARM_DRIVER_ERROR_UNSUPPORTED;
}
static uint32_t ARM_USART_Recieve_7(void *pdata, uint32_t num)
{
    return ARM_USART_Recieve(TEST_APP_ARM_UART7, pdata, num);
}
static uint32_t ARM_USART_Recieve_8(void *pdata, uint32_t num)
{
    return ARM_USART_Recieve(TEST_APP_ARM_UART8, pdata, num);
}

static TEST_APP_ARM_USART_Transfer_t ARM_USART_GetTransfer_1(void)
{
    return ARM_USART_GetTransfer(TEST_APP_ARM_USART1);
}
static TEST_APP_ARM_USART_Transfer_t ARM_USART_GetTransfer_2(void)
{
    return ARM_USART_GetTransfer(TEST_APP_ARM_USART2);
}
static TEST_APP_ARM_USART_Transfer_t ARM_USART_GetTransfer_3(void)
{
    return ARM_USART_GetTransfer(TEST_APP_ARM_USART3);
}
static TEST_APP_ARM_USART_Transfer_t ARM_USART_GetTransfer_4(void)
{
    return ARM_USART_GetTransfer(TEST_APP_ARM_UART4);
}
static TEST_APP_ARM_USART_Transfer_t ARM_USART_GetTransfer_5(void)
{
    return ARM_USART_GetTransfer(TEST_APP_ARM_UART5);
}
static TEST_APP_ARM_USART_Transfer_t ARM_USART_GetTransfer_6(void)
{
    return ARM_USART_GetTransfer(TEST_APP_ARM_USART6);
}
static TEST_APP_ARM_USART_Transfer_t ARM_USART_GetTransfer_7(void)
{
    return ARM_USART_GetTransfer(TEST_APP_ARM_UART7);
}
static TEST_APP_ARM_USART_Transfer_t ARM_USART_GetTransfer_8(void)
{
    return ARM_USART_GetTransfer(TEST_APP_ARM_UART8);
}

static TEST_APP_ARM_USART_Status_t ARM_USART_GetStatus_1(void)
{
    return ARM_USART_GetStatus(TEST_APP_ARM_USART1);
}
static TEST_APP_ARM_USART_Status_t ARM_USART_GetStatus_2(void)
{
    return ARM_USART_GetStatus(TEST_APP_ARM_USART2);
}
static TEST_APP_ARM_USART_Status_t ARM_USART_GetStatus_3(void)
{
    return ARM_USART_GetStatus(TEST_APP_ARM_USART3);
}
static TEST_APP_ARM_USART_Status_t ARM_USART_GetStatus_4(void)
{
    return ARM_USART_GetStatus(TEST_APP_ARM_UART4);
}
static TEST_APP_ARM_USART_Status_t ARM_USART_GetStatus_5(void)
{
    return ARM_USART_GetStatus(TEST_APP_ARM_UART5);
}
static TEST_APP_ARM_USART_Status_t ARM_USART_GetStatus_6(void)
{
    return ARM_USART_GetStatus(TEST_APP_ARM_USART6);
}
static TEST_APP_ARM_USART_Status_t ARM_USART_GetStatus_7(void)
{
    return ARM_USART_GetStatus(TEST_APP_ARM_UART7);
}
static TEST_APP_ARM_USART_Status_t ARM_USART_GetStatus_8(void)
{
    return ARM_USART_GetStatus(TEST_APP_ARM_UART8);
}

