#ifndef _ARM_SPI_H_
#define _ARM_SPI_H_

#include "at32f403a_407.h"
#include <stdbool.h>
#include "ringbuffer.h"

//select SPIx

/*******************************************
SPI1

********************************************/
#define _TEST_APP_SPI1_PERIPH_ENABLE_

/*******************************************/

/*******************************************
SPI2

********************************************/
#define _TEST_APP_SPI2_PERIPH_ENABLE_

/*******************************************/

/*******************************************
SPI3

********************************************/
#define _TEST_APP_SPI3_PERIPH_ENABLE_

/*******************************************/

/*******************************************
SPI4

********************************************/
#define _TEST_APP_SPI4_PERIPH_ENABLE_

/*******************************************/

#define TEST_APP_ARM_SPI_TX_BUFF_SIZE 256
#define TEST_APP_ARM_SPI_RX_BUFF_SIZE 256

#define TEST_APP_ARM_SPI_EVENT_BUFF_SIZE 8

// SPI Driver flags
#define TEST_APP_ARM_SPI_FLAG_INITIALIZED          (uint32_t)(1U << 0)
#define TEST_APP_ARM_SPI_FLAG_CONFIGURATED         (uint32_t)(1U << 1)

//SPI transfer format modes
typedef enum {
    TEST_APP_ARM_SPI_CLOCK_POLAR_LOW_RISING_EDGE = 0,
    TEST_APP_ARM_SPI_CLOCK_POLAR_LOW_FALLING_EDGE,
    TEST_APP_ARM_SPI_CLOCK_POLAR_HIGH_RISING_EDGE,
    TEST_APP_ARM_SPI_CLOCK_POLAR_HIGH_FALLING_EDGE
} eTEST_APP_ARM_SPI_ClockTypes_t;

//SPI transfer format modes
typedef enum {
    TEST_APP_ARM_SPI_CS_LEVEL_LOW = 0,
    TEST_APP_ARM_SPI_CS_LEVEL_HIGH
} eTEST_APP_ARM_SPI_CSLevels_t;

typedef struct {
    spi_master_slave_mode_type      Mode;
    spi_transmission_mode_type      XferMode;
    spi_half_duplex_direction_type  HalfDuplexDir;
    spi_mclk_freq_div_type          MclkFreqDiv;
    eTEST_APP_ARM_SPI_ClockTypes_t  ClkType;
    spi_cs_mode_type                CsMode;
    eTEST_APP_ARM_SPI_CSLevels_t    CsLevel;
    spi_frame_bit_num_type          DataBitNum;
    spi_first_bit_type              DataFirstBit;
} TEST_APP_ARM_SPI_Config_t;

typedef struct {
    gpio_type               *pTxGpio;
    gpio_type               *pRxGpio;
    uint32_t                 TxPin;
    uint32_t                 RxPin;
} TEST_APP_ARM_SPI_GPIO_t;

typedef struct {
    uint8_t Busy;
    uint8_t DataLost;
    uint8_t ModeFault;
} TEST_APP_ARM_SPI_XferStatus_t;

typedef struct {
    void                    *pTxData;
    void                    *pRxData;
    volatile uint32_t       TxNum;
    volatile uint32_t       RxNum;
    volatile uint32_t       TxCnt;         // Number of data received
    volatile uint32_t       RxCnt;         // Number of data sent
} TEST_APP_ARM_SPI_Transfer_t;

typedef struct {
    uint32_t DrvStatus;
    uint32_t DrvFlag;
    TEST_APP_ARM_SPI_XferStatus_t XferStatus;
} TEST_APP_ARM_SPI_Status_t;



typedef struct {
    spi_type                        *pSPIx;
    IRQn_Type                       IrqNum;  // SPI IRQ Number
    TEST_APP_RingBuffer_t           Event;
    TEST_APP_ARM_SPI_Config_t       Config;
    TEST_APP_ARM_SPI_GPIO_t         Gpio;
    TEST_APP_ARM_SPI_Transfer_t     Transfer;
    TEST_APP_ARM_SPI_Status_t       Status;
} TEST_APP_ARM_SPI_Resources_t;

typedef struct {
    uint32_t (*Initialize)(spi_master_slave_mode_type mode, usart_data_bit_num_type DataBit,
                           usart_stop_bit_num_type StopBit,
                           usart_parity_selection_type parity);
    // uint32_t (*Uninitialize)(void);
    // void (*Event_cb)(void);
    // uint32_t (*Send)(void *pdata, uint32_t num);
    // uint32_t (*Recieve)(void *pdata, uint32_t num);
    // ARM_SPI_Transfer_t (*GetTransfer)(void);
    // ARM_SPI_Status_t (*GetStatus)(void);
} TEST_APP_ARM_SPI_Driver_t;

uint32_t TEST_APP_ARM_SPI_Init(TEST_APP_ARM_SPI_Resources_t *p_res);
uint32_t TEST_APP_ARM_SPI_SetResources(TEST_APP_ARM_SPI_Resources_t *p_res,
                                       spi_type *p_spix,
                                       void *p_event_buff,
                                       void *p_tx_buff, void *p_rx_buff,                                        spi_master_slave_mode_type mode,
                                       spi_transmission_mode_type xfer_mode,
                                       spi_half_duplex_direction_type dir,
                                       spi_mclk_freq_div_type mclk_div,
                                       eTEST_APP_ARM_SPI_ClockTypes_t clk_type,
                                       spi_cs_mode_type cs_mode,
                                       spi_software_cs_level_type cs_level,
                                       spi_frame_bit_num_type data_bit_num,
                                       spi_first_bit_type data_first_bit);

#endif //_ARM_SPI_H_ 
