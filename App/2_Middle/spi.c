//********************************************************************************
//spi.c
//********************************************************************************
#include <string.h>
#include <stdio.h>
#include "spi.h"
#include "arm_driver.h"
#include "arm_spi.h"
#include "usart.h"

#ifdef _TEST_APP_DEBUG_
#include "assert.h"
#include "LCD_2004.h"
#endif//_TEST_APP_DEBUG_

//********************************************************************************
//Macros
//********************************************************************************

#define SPI_TEST_DATA_pBUFF_SIZE 256

//********************************************************************************
//Enums
//********************************************************************************

//********************************************************************************
//Typedefs
//********************************************************************************

//********************************************************************************
//Prototypes
//********************************************************************************

error_status ARM_SPI_TestFullDuplexMode(eTEST_APP_ARM_SPI_Types_t spi_master,
                                        eTEST_APP_ARM_SPI_Types_t spi_slave);

//********************************************************************************
//Variables
//********************************************************************************

extern TEST_APP_ARM_SPI_Driver_t *pARM_SPI_Driver[TEST_APP_ARM_SPI_TYPES];

static uint8_t SPI_TestDataMaster[] =  {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10};
static uint8_t SPI_TestDataSlave[] =  {0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59};

//================================================================================
//Public
//================================================================================


/********************************************************************************
SPI configuration:

mode                    - TEST_APP_ARM_SPI_FULL_DUPLEX_MASTER_MODE,
                        - TEST_APP_ARM_SPI_FULL_DUPLEX_SLAVE_MODE,
                        - TEST_APP_ARM_SPI_HALF_DUPLEX_TRANSMIT_MASTER_MODE,
                        - TEST_APP_ARM_SPI_HALF_DUPLEX_RECIEVE_MASTER_MODE,
                        - TEST_APP_ARM_SPI_HALF_DUPLEX_TRANSMIT_SLAVE_MODE,
                        - TEST_APP_ARM_SPI_HALF_DUPLEX_RECIEVE_SLAVE_MODE,
                        - TEST_APP_ARM_SPI_TRANSMIT_ONLY_MASTER_MODE,
                        - TEST_APP_ARM_SPI_RECIEVE_ONLY_MASTER_MODE,
                        - TEST_APP_ARM_SPI_TRANSMIT_ONLY_SLAVE_MODE,
                        - TEST_APP_ARM_SPI_RECIEVE_ONLY_SLAVE_MODE,

mclk_freq_division      - master clock frequency division: 2,4,8,16,...1024,
                            set in ARM_SPI_MasterClockFrequencyDiv[]);

cs_mode_selection       - hardware/software mode,
                            set in ARM_SPI_CSMode[]);

data_bit_num            - 8/16 bit;

data_first_bit          - MSB/LSB;

clock_type              - clock polarity low, rising edge,
                          clock polarity low, falling edge,
                          clock polarity high, rising edge,
                          clock polarity high, rising edge;
                           set in ARM_SPI_ClockLatchType[]);

cs_active_level         - in software mode only
                            set in ARM_SPI_CSActiveLevel[]);

cs_conf_every_world_en  - CS used for synchronization in software mode only
                            set in ARM_SPI_CSConfirmEveryWorld[]);

delay_afterCS_usec - delay by master after CS (e.g. time for conversation in ADC)
                            set in ARM_SPI_MasterDelayAfter_CS[]);

miso_analog_input_mode_en - (e.g. for data from ADC)
                            set in ARM_SPI_MISOAnalogInput[]);

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


error_status TEST_APP_SPI_Init(void)
{
    uint32_t drv_status = TEST_APP_ARM_DRIVER_NO_ERROR;
    // eTEST_APP_ARM_SPI_Types_t spi;
    // TEST_APP_ARM_SPI_Driver_t *p_drv = pARM_SPI_Driver[TEST_APP_ARM_SPI1];
    // for(spi = TEST_APP_ARM_SPI1; spi < TEST_APP_ARM_SPI_TYPES;
    //     spi++) {
    //     if(p_drv[spi].GetStatus().DrvStateOn) {
    //         switch(spi) {
    //             case TEST_APP_ARM_SPI1: {
    //                 drv_status |= p_drv[spi].Initialize(
    //                                   TEST_APP_ARM_SPI_FULL_DUPLEX_MASTER_MODE,
    //                                   SPI_FRAME_8BIT,
    //                                   SPI_FIRST_BIT_MSB,
    //                                   TEST_APP_ARM_SPI_GPIO_PIN_DEF_TYPE_DEFAULT);
    //                 break;
    //             }
    //             case TEST_APP_ARM_SPI2: {
    //                 drv_status |= p_drv[spi].Initialize(
    //                                   TEST_APP_ARM_SPI_FULL_DUPLEX_SLAVE_MODE,
    //                                   SPI_FRAME_8BIT,
    //                                   SPI_FIRST_BIT_MSB,
    //                                   TEST_APP_ARM_SPI_GPIO_PIN_DEF_TYPE_DEFAULT);
    //                 break;
    //             }
    //             case TEST_APP_ARM_SPI3: {
    //                 drv_status |= p_drv[spi].Initialize(
    //                                   TEST_APP_ARM_SPI_RECIEVE_ONLY_MASTER_MODE,
    //                                   SPI_FRAME_16BIT,
    //                                   SPI_FIRST_BIT_MSB,
    //                                   TEST_APP_ARM_SPI_GPIO_PIN_DEF_TYPE_DEFAULT);
    //                 break;
    //             }
    //             case TEST_APP_ARM_SPI4: {
    //                 drv_status |= p_drv[spi].Initialize(
    //                                   TEST_APP_ARM_SPI_RECIEVE_ONLY_MASTER_MODE,
    //                                   SPI_FRAME_16BIT,
    //                                   SPI_FIRST_BIT_MSB,
    //                                   TEST_APP_ARM_SPI_GPIO_PIN_DEF_TYPE_DEFAULT);
    //                 break;
    //             }
    //             default: {
    //                 break;
    //             }
    //         }
    //     }
    // }
    return TEST_APP_ARM_DRIVER_isReady(drv_status) ? SUCCESS : ERROR;
}



void TEST_APP_SPI_cb(void)
{
    eTEST_APP_ARM_SPI_Types_t spi;
    TEST_APP_ARM_SPI_Driver_t *p_drv = pARM_SPI_Driver[TEST_APP_ARM_SPI1];
    for(spi = TEST_APP_ARM_SPI1; spi < TEST_APP_ARM_SPI_TYPES;
        spi++) {
        if(p_drv[spi].GetStatus().DrvStateOn) {
            p_drv[spi].Event_cb();
        }
    }
}

error_status TEST_APP_SPI_Test(void)
{
    error_status test_result;
    test_result = ARM_SPI_TestFullDuplexMode(TEST_APP_ARM_SPI2, TEST_APP_ARM_SPI1);
    return test_result;
}

//================================================================================
//Private
//================================================================================

#ifdef _TEST_APP_DEBUG_
error_status ARM_SPI_TestFullDuplexMode(eTEST_APP_ARM_SPI_Types_t spi_master,
                                        eTEST_APP_ARM_SPI_Types_t spi_slave)
{

    TEST_APP_ARM_SPI_Driver_t *p_drv_mst = pARM_SPI_Driver[spi_master];
    TEST_APP_ARM_SPI_Driver_t *p_drv_slv = pARM_SPI_Driver[spi_slave];
    uint32_t drv_status = TEST_APP_ARM_DRIVER_NO_ERROR;
    if(p_drv_mst->GetStatus().DrvStateOn &&
       p_drv_slv->GetStatus().DrvStateOn) {
        drv_status |= p_drv_mst->Initialize(
                          TEST_APP_ARM_SPI_FULL_DUPLEX_MASTER_MODE,
                          SPI_FRAME_8BIT,
                          SPI_FIRST_BIT_MSB,
                          TEST_APP_ARM_SPI_GPIO_PIN_DEF_TYPE_DEFAULT);
        drv_status |= p_drv_slv->Initialize(
                          TEST_APP_ARM_SPI_FULL_DUPLEX_SLAVE_MODE,
                          SPI_FRAME_8BIT,
                          SPI_FIRST_BIT_MSB,
                          TEST_APP_ARM_SPI_GPIO_PIN_DEF_TYPE_DEFAULT);
    } else {
        drv_status |= TEST_APP_ARM_DRIVER_ERROR;
    }
    uint8_t *pbuff_rx_master = (uint8_t *)(p_drv_mst->GetTransfer().pRxData);
    uint8_t *pbuff_rx_slave = (uint8_t *)(p_drv_slv->GetTransfer().pRxData);
    p_drv_slv->Send_Recieve(SPI_TestDataSlave, pbuff_rx_slave, sizeof(SPI_TestDataSlave));
    p_drv_mst->Send_Recieve(SPI_TestDataMaster, pbuff_rx_master, sizeof(SPI_TestDataMaster));
    while(p_drv_mst->GetStatus().XferStatus.TxBusy);
    while(p_drv_slv->GetStatus().XferStatus.RxBusy);
    if(memcmp((uint8_t *)SPI_TestDataMaster,
              (uint8_t *)pbuff_rx_slave,
              sizeof(SPI_TestDataMaster)) ||
       memcmp((uint8_t *)SPI_TestDataSlave,
              (uint8_t *)pbuff_rx_master,
              sizeof(SPI_TestDataSlave))
      ) {
        LOG("SPI FullDuplex Mode transfer error");
        drv_status |= TEST_APP_ARM_DRIVER_ERROR;
    }
    if(drv_status == TEST_APP_ARM_DRIVER_NO_ERROR) {
        LOG("SPI in FullDuplex Mode successfully swap test data");
    }
    return TEST_APP_ARM_DRIVER_isReady(drv_status) ? SUCCESS : ERROR;
}
#endif//_TEST_APP_DEBUG_

