//********************************************************************************
//spi.c
//********************************************************************************
#include <string.h>
#include "spi.h"
#include "arm_driver.h"
#include "arm_spi.h"
#include "usart.h"

#ifdef _TEST_APP_DEBUG_
#include "assert.h"
#include "LCD_2004.h"
#endif//_TEST_APP_DEBUG_

extern TEST_APP_ARM_SPI_Driver_t *pARM_SPI_Driver[TEST_APP_ARM_SPI_TYPES];
extern TEST_APP_ARM_USART_Driver_t *pARM_USART_Driver[TEST_APP_ARM_USART_TYPES];

//********************************************************************************
//Macros
//********************************************************************************
#define SPI_TEST_DATA_BUFF_SIZE 256

#define SPI_AD7685_ADC_NUM 8

#define SPI_AD7685_DELAY_USEC_AFTER_CS ((uint32_t)10)

#define SPI_NO_DELAY_USEC_AFTER_CS ((uint32_t)0)

//********************************************************************************
//Enums
//********************************************************************************

//********************************************************************************
//Typedefs
//********************************************************************************

//********************************************************************************
//Prototypes
//********************************************************************************

error_status ARM_SPI_TestFullDuplexMode(eTEST_APP_ARM_SPI_Types_t spi_master_type,
                                        eTEST_APP_ARM_SPI_Types_t spi_slave_type);

//********************************************************************************
//Variables
//********************************************************************************

extern TEST_APP_ARM_SPI_Driver_t *pARM_SPI_Driver[TEST_APP_ARM_SPI_TYPES];

static char ARM_SPI_DataMasterTx[SPI_TEST_DATA_BUFF_SIZE] = {"Hello, slave"};
static char ARM_SPI_DataSlaveTx[SPI_TEST_DATA_BUFF_SIZE] = {"Hello, master"};

//================================================================================
//Public
//================================================================================


/********************************************************************************
SPI configuration:

mode            - master/slave mode;
xfer_mode       - transfer modes:
                    2-wire unidirectional full-duplex (SPI_TRANSMIT_FULL_DUPLEX),
                    1-wire bidirectional half-duplex-rx (SPI_TRANSMIT_HALF_DUPLEX_RX),
                    1-wire bidirectional half-duplex-tx (SPI_TRANSMIT_HALF_DUPLEX_TX),
                    1-wire unidirectional simplex-rx for rx only (SPI_TRANSMIT_SIMPLEX_RX)
                    1-wire unidirectional simplex-tx for tx only (SPI_TRANSMIT_FULL_DUPLEX_TX_ONLY),

1-wire unidirectional simplex-rx transmit-only mode is similar to full-duplex mode:
the data are transmitted on the transmit pin (MOSI in master mode or MISO in slave
mode) and the receive pin (MISO in master mode or MOSI in slave mode) can be used as a
general-purpose IO. In this case, the application just needs to ignore the Rx buffer (if
the data register is read, it does not contain the received value).

mclk_div        - master clock frequency division: 2,4,8,16,...1024;
clk_type        - types:
                    clock polarity low, rising edge,
                    clock polarity low, falling edge,
                    clock polarity high, rising edge,
                    clock polarity high, rising edge;
cs_mode         - hardware/software mode;
cs_active_level - in software mode only
cs_conf_every_world - CS used for synchronization in software mode only
data_bit_num    - 8/16 bit;
data_first_bit  - MSB/LSB;
gpio_pin_def - pin definitions default/remap1/remap2:
    SPI1:
            default: spi1_cs(pa4),  spi1_sck(pa5), spi1_miso(pa6), spi1_mosi(pa7);
            remap1:  spi1_cs(pa15), spi1_sck(pb3), spi1_miso(pb4), spi1_mosi(pb5);
            remap2:  -
    SPI2:
            default: spi2_cs(pb12), spi2_sck(pb13), spi2_miso(pb14), spi2_mosi(pb15);
            remap1:  -
            remap2:  -
    SPI3:
            default: spi3_cs(pa15), spi3_sck(pb3), spi3_miso(pb4), spi3_mosi(pb5);
            remap1:  spi3_cs(pa4),  spi3_sck(pc10), spi3_miso(pc11), spi3_mosi(pc12);
            remap2:  -
    SPI4:
            default: spi4_cs(pe4), spi4_sck(pe2), spi4_miso(pe5), spi4_mosi(pe6);
            remap1:  spi4_cs(pe12), spi4_sck(pe11), spi4_miso(pe13), spi4_mosi(pe14);
            remap2:  spi4_cs(pb6), spi4_sck(pb7), spi4_miso(pb8), spi4_mosi(pb9);


********************************************************************************/

void SPI1_IRQHandler(void)
{
    TEST_APP_ARM_SPI_IRQHandler(TEST_APP_ARM_SPI1);
}

void SPI2_I2S2EXT_IRQHandler(void)
{
    TEST_APP_ARM_SPI_IRQHandler(TEST_APP_ARM_SPI2);
}

void SPI3_I2S3EXT_IRQHandler(void)
{
    TEST_APP_ARM_SPI_IRQHandler(TEST_APP_ARM_SPI3);
}

void SPI4_IRQHandler(void)
{
    TEST_APP_ARM_SPI_IRQHandler(TEST_APP_ARM_SPI4);
}

/*
error_status TEST_APP_SPI_Init(void)
{
    uint32_t drv_status = TEST_APP_ARM_DRIVER_NO_ERROR;
    eTEST_APP_ARM_SPI_Types_t spi_type;
    TEST_APP_ARM_SPI_Driver_t *p_drv = pARM_SPI_Driver[TEST_APP_ARM_SPI1];
    for(spi_type = TEST_APP_ARM_SPI1; spi_type < TEST_APP_ARM_SPI_TYPES;
        spi_type++) {
        if(p_drv[spi_type].GetStatus().DrvStateOn) {
            switch(spi_type) {
                case TEST_APP_ARM_SPI1: {
                    drv_status |= p_drv[spi_type].Initialize(
                                      SPI_MODE_MASTER,
                                      SPI_TRANSMIT_FULL_DUPLEX,
                                      SPI_MCLK_DIV_4, //30 MHz(APB2 bus-120 MHz)
                                      TEST_APP_ARM_SPI_CLOCK_POLAR_LOW_FALLING_EDGE,
                                      SPI_CS_HARDWARE_MODE,
                                      TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_LOW,
                                      TEST_APP_ARM_SPI_CS_CONFIRM_EVERY_WORLD_ENABLE,
                                      SPI_FRAME_8BIT,
                                      SPI_FIRST_BIT_MSB,
                                      TEST_APP_ARM_SPI_PIN_DEF_TYPE_DEFAULT);
                    break;
                }
                case TEST_APP_ARM_SPI2: {
                    drv_status |= p_drv[spi_type].Initialize(
                                      SPI_MODE_MASTER,
                                      SPI_TRANSMIT_SIMPLEX_RX,
                                      SPI_MCLK_DIV_4, //30 MHz(APB2 bus-120 MHz)
                                      TEST_APP_ARM_SPI_CLOCK_POLAR_LOW_FALLING_EDGE,
                                      SPI_CS_SOFTWARE_MODE,
                                      TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_HIGH,
                                      TEST_APP_ARM_SPI_CS_CONFIRM_EVERY_WORLD_DISABLE,
                                      SPI_FRAME_16BIT,
                                      SPI_FIRST_BIT_MSB,
                                      TEST_APP_ARM_SPI_PIN_DEF_TYPE_REMAP1);
                    break;
                }
                case TEST_APP_ARM_SPI3: {
                    drv_status |= p_drv[spi_type].Initialize(
                                      SPI_MODE_SLAVE,
                                      SPI_TRANSMIT_FULL_DUPLEX,
                                      SPI_MCLK_DIV_4, //30 MHz(APB2 bus-120 MHz)
                                      TEST_APP_ARM_SPI_CLOCK_POLAR_LOW_FALLING_EDGE,
                                      SPI_CS_HARDWARE_MODE,
                                      TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_LOW,
                                      TEST_APP_ARM_SPI_CS_CONFIRM_EVERY_WORLD_ENABLE,
                                      SPI_FRAME_8BIT,
                                      SPI_FIRST_BIT_MSB,
                                      TEST_APP_ARM_SPI_PIN_DEF_TYPE_DEFAULT);
                    break;
                }
                case TEST_APP_ARM_SPI4: {
                    drv_status |= p_drv[spi_type].Initialize(
                                      SPI_MODE_MASTER,
                                      SPI_TRANSMIT_SIMPLEX_RX,
                                      SPI_MCLK_DIV_4, //30 MHz(APB2 bus-120 MHz)
                                      TEST_APP_ARM_SPI_CLOCK_POLAR_LOW_FALLING_EDGE,
                                      SPI_CS_SOFTWARE_MODE,
                                      TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_HIGH,
                                      TEST_APP_ARM_SPI_CS_CONFIRM_EVERY_WORLD_ENABLE,
                                      SPI_FRAME_16BIT,
                                      SPI_FIRST_BIT_MSB,
                                      TEST_APP_ARM_SPI_PIN_DEF_TYPE_DEFAULT);
                    break;
                }
                default: {
                    break;
                }
            }
        }
    }
    return TEST_APP_ARM_DRIVER_isReady(drv_status) ? SUCCESS : ERROR;
}

*/

void TEST_APP_SPI_cb(void)
{
    eTEST_APP_ARM_SPI_Types_t spi_type;
    TEST_APP_ARM_SPI_Driver_t *p_drv = pARM_SPI_Driver[TEST_APP_ARM_SPI1];
    for(spi_type = TEST_APP_ARM_SPI1; spi_type < TEST_APP_ARM_SPI_TYPES;
        spi_type++) {
        if(p_drv[spi_type].GetStatus().DrvStateOn) {
            p_drv[spi_type].Event_cb();
        }
    }
}

error_status TEST_APP_SPI_Test(void)
{
    error_status test_result;
    test_result = ARM_SPI_TestFullDuplexMode(TEST_APP_ARM_SPI1, TEST_APP_ARM_SPI3);
    return test_result;
}


//================================================================================
//Private
//================================================================================

//spi1, spi3
error_status ARM_SPI_TestFullDuplexMode(eTEST_APP_ARM_SPI_Types_t spi_master_type,
                                        eTEST_APP_ARM_SPI_Types_t spi_slave_type)
{
    uint32_t drv_status = TEST_APP_ARM_DRIVER_NO_ERROR;
    TEST_APP_ARM_SPI_Driver_t *p_drv = pARM_SPI_Driver[TEST_APP_ARM_SPI1];
    if(p_drv[spi_master_type].GetStatus().DrvStateOn &&
       p_drv[spi_slave_type].GetStatus().DrvStateOn) {
        drv_status |= p_drv[spi_master_type].Initialize(
                          SPI_MODE_MASTER,
                          SPI_TRANSMIT_FULL_DUPLEX,
                          SPI_MCLK_DIV_4, //30 MHz(APB2 bus-120 MHz)
                          TEST_APP_ARM_SPI_CLOCK_POLAR_LOW_FALLING_EDGE,
                          SPI_CS_SOFTWARE_MODE,
                          TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_LOW,
                          TEST_APP_ARM_SPI_CS_CONFIRM_EVERY_WORLD_ENABLE,
                          SPI_NO_DELAY_USEC_AFTER_CS,
                          SPI_FRAME_8BIT,
                          SPI_FIRST_BIT_MSB,
                          TEST_APP_ARM_SPI_PIN_DEF_TYPE_DEFAULT);
        drv_status |= p_drv[spi_slave_type].Initialize(
                          SPI_MODE_SLAVE,
                          SPI_TRANSMIT_SIMPLEX_RX,
                          SPI_MCLK_DIV_4, //30 MHz(APB2 bus-120 MHz)
                          TEST_APP_ARM_SPI_CLOCK_POLAR_LOW_FALLING_EDGE,
                          SPI_CS_SOFTWARE_MODE,
                          TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_HIGH,
                          TEST_APP_ARM_SPI_CS_CONFIRM_EVERY_WORLD_DISABLE,
                          SPI_NO_DELAY_USEC_AFTER_CS,
                          SPI_FRAME_8BIT,
                          SPI_FIRST_BIT_MSB,
                          TEST_APP_ARM_SPI_PIN_DEF_TYPE_DEFAULT);
    } else {
        drv_status |= TEST_APP_ARM_DRIVER_ERROR;
    }
    char *pdata;
    pdata = (char *)p_drv[spi_slave_type].GetTransfer().pTxData;
    memcpy(pdata, ARM_SPI_DataSlaveTx, SPI_TEST_DATA_BUFF_SIZE);
    p_drv[spi_slave_type].Send(pdata, SPI_TEST_DATA_BUFF_SIZE);
    pdata = (char *)p_drv[spi_master_type].GetTransfer().pTxData;
    memcpy(pdata, ARM_SPI_DataMasterTx, SPI_TEST_DATA_BUFF_SIZE);
    p_drv[spi_master_type].Send(pdata, SPI_TEST_DATA_BUFF_SIZE);
    while(p_drv[spi_master_type].GetStatus().XferStatus.TxBusy);
    if(memcmp((char *)p_drv[spi_master_type].GetTransfer().pRxData,
              ARM_SPI_DataSlaveTx,
              SPI_TEST_DATA_BUFF_SIZE)) {
        drv_status |= TEST_APP_ARM_DRIVER_ERROR;
    }
    if(memcmp((char *)p_drv[spi_slave_type].GetTransfer().pRxData,
              ARM_SPI_DataMasterTx,
              SPI_TEST_DATA_BUFF_SIZE)) {
        drv_status |= TEST_APP_ARM_DRIVER_ERROR;
    }
    TEST_APP_USART_printf(pARM_USART_Driver[TEST_APP_ARM_UART4], "Master recieved");
    TEST_APP_USART_printf(pARM_USART_Driver[TEST_APP_ARM_UART4],
                          (char *)p_drv[spi_master_type].GetTransfer().pRxData);
    TEST_APP_USART_printf(pARM_USART_Driver[TEST_APP_ARM_UART4], "Slave recieved");
    TEST_APP_USART_printf(pARM_USART_Driver[TEST_APP_ARM_UART4],
                          (char *)p_drv[spi_slave_type].GetTransfer().pRxData);
    return TEST_APP_ARM_DRIVER_isReady(drv_status) ? SUCCESS : ERROR;
}

//spi2
error_status ARM_SPI_TestAD7685(eTEST_APP_ARM_SPI_Types_t spi_master_type)
{
    uint32_t drv_status = TEST_APP_ARM_DRIVER_NO_ERROR;
    TEST_APP_ARM_SPI_Driver_t *p_drv = pARM_SPI_Driver[TEST_APP_ARM_SPI1];
    if(p_drv[spi_master_type].GetStatus().DrvStateOn) {
        drv_status |= p_drv[spi_master_type].Initialize(
                          SPI_MODE_MASTER,
                          SPI_TRANSMIT_SIMPLEX_RX,
                          SPI_MCLK_DIV_4, //30 MHz(APB2 bus-120 MHz)
                          TEST_APP_ARM_SPI_CLOCK_POLAR_LOW_FALLING_EDGE,
                          SPI_CS_SOFTWARE_MODE,
                          TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_HIGH,
                          TEST_APP_ARM_SPI_CS_CONFIRM_EVERY_WORLD_DISABLE,
                          SPI_AD7685_DELAY_USEC_AFTER_CS,
                          SPI_FRAME_16BIT,
                          SPI_FIRST_BIT_MSB,
                          TEST_APP_ARM_SPI_PIN_DEF_TYPE_REMAP1);
    } else {
        drv_status |= TEST_APP_ARM_DRIVER_ERROR;
    }
    uint16_t *pdata;
    pdata = (uint16_t *)p_drv[spi_master_type].GetTransfer().pRxData;
    return TEST_APP_ARM_DRIVER_isReady(drv_status) ? SUCCESS : ERROR;
}
