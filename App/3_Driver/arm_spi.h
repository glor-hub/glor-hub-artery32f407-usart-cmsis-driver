#ifndef _ARM_SPI_H_
#define _ARM_SPI_H_

#include "at32f403a_407.h"
#include <stdbool.h>
#include "ringbuffer.h"
#include "arm_gpio.h"

/********************************************

This driver version does not support:
- multiple slave devices
- multiple master devices
- CRC transmission and error checking
- the use of DMA

********************************************

Select SPIx
Configure SPIx using:
 - parameters of the arrays in arm_spi.c,
 - parameters of function TEST_APP_SPI_Init() in spi.c

********************************************/


/*******************************************
SPI1

********************************************/
#define _TEST_APP_SPI1_ENABLED_ (1)

/*******************************************
SPI2

********************************************/
#define _TEST_APP_SPI2_ENABLED_ (1)

/*******************************************
SPI3

********************************************/
#define _TEST_APP_SPI3_ENABLED_ (1)

/*******************************************
SPI4

********************************************/
#define _TEST_APP_SPI4_ENABLED_ (1)

/*******************************************

********************************************/

typedef enum {
    TEST_APP_ARM_SPI1 = 0,
    TEST_APP_ARM_SPI2,
    TEST_APP_ARM_SPI3,
    TEST_APP_ARM_SPI4,
    TEST_APP_ARM_SPI_TYPES
} eTEST_APP_ARM_SPI_Types_t;

typedef enum {
    ARM_SPI_MISO_PIN = 0,
    ARM_SPI_MOSI_PIN,
    ARM_SPI_CS_PIN,
    ARM_SPI_SCLK_PIN,
    ARM_SPI_PIN_TYPES
} eARM_SPI_PinTypes_t;

typedef enum {
    TEST_APP_ARM_SPI_FULL_DUPLEX_MASTER_MODE = 0,
    TEST_APP_ARM_SPI_FULL_DUPLEX_SLAVE_MODE,
    //full-duplex master with the use of single line MOSI:
    TEST_APP_ARM_SPI_TRANSMIT_ONLY_MASTER_MODE,
    //full-duplex slave with the use of single line MISO:
    TEST_APP_ARM_SPI_TRANSMIT_ONLY_SLAVE_MODE,
    TEST_APP_ARM_SPI_HALF_DUPLEX_TRANSMIT_MASTER_MODE,
    TEST_APP_ARM_SPI_HALF_DUPLEX_TRANSMIT_SLAVE_MODE,
    TEST_APP_ARM_SPI_HALF_DUPLEX_RECIEVE_MASTER_MODE,
    TEST_APP_ARM_SPI_HALF_DUPLEX_RECIEVE_SLAVE_MODE,
    TEST_APP_ARM_SPI_RECIEVE_ONLY_MASTER_MODE,
    TEST_APP_ARM_SPI_RECIEVE_ONLY_SLAVE_MODE,
    TEST_APP_ARM_SPI_MODE_TYPES
} eTEST_APP_ARM_SPI_ModeTypes_t;

typedef enum {
    TEST_APP_ARM_SPI_GPIO_PIN_DEF_TYPE_REMAP1 = 0,
    TEST_APP_ARM_SPI_GPIO_PIN_DEF_TYPE_REMAP2,
    TEST_APP_ARM_SPI_GPIO_PIN_DEF_TYPE_DEFAULT,
    TEST_APP_ARM_SPI_GPIO_PIN_DEF_TYPES,
    TEST_APP_ARM_SPI_GPIO_PIN_REMAP_CONFIG_NOT_DEFINED,
} eTEST_APP_ARM_SPI_GPIO_PinDefTypes_t;

#define TEST_APP_ARM_SPI_GPIO_PIN_DEF_REMAP_TYPES (TEST_APP_ARM_SPI_GPIO_PIN_DEF_TYPES-1)

typedef enum {
    TEST_APP_ARM_SPI_GPIO_PIN_ANALOG_INPUT_MODE_ENABLE = 0,
    TEST_APP_ARM_SPI_GPIO_PIN_ANALOG_INPUT_MODE_DISABLE
} eTEST_APP_ARM_SPI_PinAnalogInputMode_t;

//SPI Driver state
#define TEST_APP_ARM_SPI_DRIVER_FLAG_INITIALIZED       (uint32_t)(1U << 0)
#define TEST_APP_ARM_SPI_DRIVER_FLAG_CONFIGURATED      (uint32_t)(1U << 1)
#define TEST_APP_ARM_SPI_DRIVER_FLAG_ENABLED           (uint32_t)(1U << 2)

//SPI Event
#define TEST_APP_ARM_SPI_EVENT_RX_COMPLETE      ((uint32_t)1UL << 0)//Receive completed
#define TEST_APP_ARM_SPI_EVENT_TX_COMPLETE      ((uint32_t)1UL << 1)//Transmit completed
#define TEST_APP_ARM_SPI_EVENT_RX_OVERFLOW      ((uint32_t)1UL << 2)//Receive data overflow
#define TEST_APP_ARM_SPI_EVENT_MODE_FAULT       ((uint32_t)1UL << 3)//Master mode error

//CS confirm every world enable in software mode
#define TEST_APP_ARM_SPI_CS_CONFIRM_EVERY_WORLD_ENABLE  TRUE
#define TEST_APP_ARM_SPI_CS_CONFIRM_EVERY_WORLD_DISABLE FALSE

//MISO pin analog input mode enable
#define TEST_APP_ARM_SPI_MISO_ANALOG_INPUT_MODE_ENABLE  TRUE
#define TEST_APP_ARM_SPI_MISO_ANALOG_INPUT_MODE_DISABLE FALSE

//SPI clock latch format modes
typedef enum {
    TEST_APP_ARM_SPI_CLOCK_POLAR_LOW_RISING_EDGE = 0,
    TEST_APP_ARM_SPI_CLOCK_POLAR_LOW_FALLING_EDGE,
    TEST_APP_ARM_SPI_CLOCK_POLAR_HIGH_RISING_EDGE,
    TEST_APP_ARM_SPI_CLOCK_POLAR_HIGH_FALLING_EDGE
} eTEST_APP_ARM_SPI_ClockLatchTypes_t;

//SPI CS (use as general gpio) active level in software mode
typedef enum {
    TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_LOW = 0,
    TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_HIGH
} eTEST_APP_ARM_SPI_CSActiveLevel_t;

typedef struct {
    spi_init_type                           InitStruct;
    eTEST_APP_ARM_SPI_ClockLatchTypes_t     ClockType;
    eTEST_APP_ARM_SPI_CSActiveLevel_t       CSActiveLevel;
    confirm_state                           CSConfEveryWorldEn;
    uint32_t                                DelayAfterCS_usec;
    confirm_state                           MISOAnalogInputModeEn;
} TEST_APP_ARM_SPI_Config_t;

typedef struct {
    eTEST_APP_ARM_GPIO_Ports_t  Port;
    uint32_t                    Pin;
} TEST_APP_ARM_SPI_GPIO_t;

typedef struct {
    void                    *pTxData;
    void                    *pRxData;
    uint32_t                TxNum;
    uint32_t                RxNum;
    volatile uint32_t       TxCnt;         // Number of data received
    volatile uint32_t       RxCnt;         // Number of data sent
} TEST_APP_ARM_SPI_Transfer_t;

typedef struct {
    volatile uint8_t TxBusy;
    volatile uint8_t RxBusy;
    volatile uint8_t RxOverflow;
    volatile uint8_t ModeFault;
} TEST_APP_ARM_SPI_XferStatus_t;

typedef struct {
    confirm_state DrvStateOn;
    uint32_t DrvStatus;
    uint32_t DrvFlag;
    TEST_APP_ARM_SPI_XferStatus_t XferStatus;
} TEST_APP_ARM_SPI_Status_t;

typedef struct {
    eTEST_APP_ARM_SPI_ModeTypes_t           mode;
    IRQn_Type                               IrqNum;         // SPI IRQ Number
    eTEST_APP_ARM_SPI_GPIO_PinDefTypes_t    GpioPinDefType; //pin remapping or not
    TEST_APP_RingBuffer_t                   Event;
    TEST_APP_ARM_SPI_Config_t               Config;
    TEST_APP_ARM_SPI_GPIO_t                 Gpio[ARM_SPI_PIN_TYPES];
    TEST_APP_ARM_SPI_Transfer_t             Transfer;
    TEST_APP_ARM_SPI_Status_t               Status;
} TEST_APP_ARM_SPI_Resources_t;

typedef struct {
    uint32_t (*Initialize)(eTEST_APP_ARM_SPI_ModeTypes_t mode,
                           spi_mclk_freq_div_type mclk_freq_div,
                           spi_frame_bit_num_type data_bit_num,
                           spi_first_bit_type data_first_bit,
                           eTEST_APP_ARM_SPI_GPIO_PinDefTypes_t gpio_pin_def_type);
    uint32_t (*Uninitialize)(void);
    void (*DisableOnTransfer)(void);
    void (*Event_cb)(void);
    uint32_t (*Send_Recieve)(void *ptxdata, void *prxdata,
                             uint32_t num);
    uint32_t (*Send)(void *pdata, uint32_t num);
    uint32_t (*Recieve)(void *pdata, uint32_t num);
    TEST_APP_ARM_SPI_Transfer_t (*GetTransfer)(void);
    TEST_APP_ARM_SPI_Status_t (*GetStatus)(void);
} TEST_APP_ARM_SPI_Driver_t;

void TEST_APP_ARM_SPI_StartUp(void);
void TEST_APP_ARM_SPI_IRQHandler(eTEST_APP_ARM_SPI_Types_t spi);

#endif //_ARM_SPI_H_ 
