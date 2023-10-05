//********************************************************************************
//spi.c
//********************************************************************************
#include "spi.h"
#include "arm_driver.h"
#include "arm_spi.h"

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
//Prototypes
//********************************************************************************
#ifdef _TEST_APP_SPI1_PERIPH_ENABLE_
static uint32_t     SPI1_Initialize(spi_master_slave_mode_type mode,
                                    spi_transmission_mode_type xfer_mode,
                                    spi_half_duplex_direction_type dir,
                                    spi_mclk_freq_div_type mclk_div,
                                    eTEST_APP_ARM_SPI_ClockTypes_t clk_type,
                                    spi_cs_mode_type cs_mode,
                                    eTEST_APP_ARM_SPI_CSLevels_t cs_level,
                                    spi_frame_bit_num_type data_bit_num,
                                    spi_first_bit_type data_first_bit);
static uint32_t     SPI1_Uninitialize(void);
static void         SPI1_cb(void);
static uint32_t     SPI1_Recieve(void *pbuff, uint32_t num);
static uint32_t     SPI1_Send(void *pbuff, uint32_t num);
static TEST_APP_ARM_SPI_Status_t    SPI1_GetStatus(void);
static TEST_APP_ARM_SPI_Transfer_t  SPI1_GetTransfer(void);
#endif//_TEST_APP_SPI1_PERIPH_ENABLE_


//********************************************************************************
//Variables
//********************************************************************************

#ifdef _TEST_APP_SPI1_PERIPH_ENABLE_
static TEST_APP_ARM_SPI_Driver_t SPI1_Driver = {
    SPI1_Initialize,
    SPI1_Uninitialize,
    SPI1_cb,
    SPI1_Send,
    SPI1_Recieve,
    SPI1_GetTransfer,
    SPI1_GetStatus,
};
static TEST_APP_ARM_SPI_Resources_t     SPI1_Resources;
static uint32_t                         SPI1_EventBuff[TEST_APP_ARM_SPI_EVENT_BUFF_SIZE];
static uint8_t                          SPI1_Tx_Buff[TEST_APP_ARM_SPI_TX_BUFF_SIZE];
static uint8_t                          SPI1_Rx_Buff[TEST_APP_ARM_SPI_RX_BUFF_SIZE];
#endif _TEST_APP_SPI1_PERIPH_ENABLE_


//================================================================================
//Public
//================================================================================

// SPI1 test for ADC AD7685 (Fclk AD7685 max = 59MHz)

error_status TEST_APP_SPI_Init(void)
{
    uint32_t status_ready = TEST_APP_ARM_DRIVER_NO_ERROR;

#ifdef _TEST_APP_SPI1_PERIPH_ENABLE_
    status_ready |= TEST_APP_SPI_Initialize(&SPI1_Driver,
                                            SPI_MODE_MASTER,
                                            SPI_TRANSMIT_SIMPLEX_RX,
                                            SPI_HALF_DUPLEX_DIRECTION_RX, //not used
                                            SPI_MCLK_DIV_4, //30 MHz (APB2 bus - 120 MHz)
                                            TEST_APP_ARM_SPI_CLOCK_POLAR_LOW_FALLING_EDGE,
                                            SPI_CS_HARDWARE_MODE,
                                            TEST_APP_ARM_SPI_CS_LEVEL_HIGH,
                                            SPI_FRAME_16BIT,
                                            SPI_FIRST_BIT_MSB);
#endif//_TEST_APP_SPI1_PERIPH_ENABLE_
    return TEST_APP_ARM_DRIVER_isReady(status_ready) ? SUCCESS : ERROR;
}

/********************************************************************************
SPI configuration:

mode            - master/slave mode;
xfer_mode       - transfer modes:
                    2-wire full-duplex,
                    1-wire simplex-rx (only rx),
                    1-wire half-duplex-rx,
                    1 wire half-duplex-tx;
dir             - half-duplex direction;
mclk_div        - master clock frequency division: 2,4,8,16,...1024;
clk_type        - types:
                    clock polarity low, rising edge,
                    clock polarity low, falling edge,
                    clock polarity high, rising edge,
                    clock polarity high, rising edge;
cs_mode         - hardware/software mode;
cs_level        - high/low level;
data_bit_num    - 8/16 bit;
data_first_bit  - MSB/LSB;
********************************************************************************/

uint32_t TEST_APP_SPI_Initialize(TEST_APP_ARM_spi_Driver_t *p_drv, uint32_t baud_rate, usart_data_bit_num_type data_bit,
                                 usart_stop_bit_num_type stop_bit, usart_parity_selection_type parity)
{
    uint32_t status_ready;
    status_ready = p_drv->Initialize(mode, xfer_mode, dir, mclk_div, clk_type,
                                     cs_mode, cs_level, data_bit_num, data_first_bit);
    return status_ready;
}

#ifdef _TEST_APP_SPI1_PERIPH_ENABLE_
static uint32_t SPI1_Initialize(spi_master_slave_mode_type mode,
                                spi_transmission_mode_type xfer_mode,
                                spi_half_duplex_direction_type dir,
                                spi_mclk_freq_div_type mclk_div,
                                eTEST_APP_ARM_SPI_ClockTypes_t clk_type,
                                spi_cs_mode_type cs_mode,
                                eTEST_APP_ARM_SPI_CSLevels_t cs_level,
                                spi_frame_bit_num_type data_bit_num,
                                spi_first_bit_type data_first_bit)
{
    uint32_t status_ready = TEST_APP_ARM_DRIVER_NO_ERROR;
    TEST_APP_ARM_SPI_Resources_t *p_res = &SPI1_Resources;
    status_ready |= TEST_APP_ARM_SPI_SetResources(p_res, SPI1, SPI1_EventBuff,
                    SPI1_Tx_Buff, SPI1_Rx_Buff,
                    mode, xfer_mode, dir, mclk_div, clk_type,
                    cs_mode, cs_level, data_bit_num, data_first_bit);
    status_ready |= TEST_APP_ARM_SPI_Init(p_res);
    return status_ready;
}
#endif//_TEST_APP_SPI1_PERIPH_ENABLE_


//================================================================================
//Private
//================================================================================
