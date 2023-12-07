//********************************************************************************
//arm_spi.c
//********************************************************************************
#include <string.h>
#include "arm_driver.h"
#include "arm_spi.h"
#include "arm_clock.h"
#include "arm_gpio.h"

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

static uint32_t ARM_SPI_GPIO_Config(TEST_APP_ARM_SPI_Resources_t *p_res, confirm_state new_state);

//================================================================================
//Public
//================================================================================

uint32_t TEST_APP_ARM_SPI_Init(TEST_APP_ARM_SPI_Resources_t *p_res)
{
    uint32_t drv_status = TEST_APP_ARM_DRIVER_NO_ERROR;
    uint32_t drv_flag = p_res->Status.DrvFlag;
    if(!(TEST_APP_ARM_CRM_SPI_ClockEnable(p_res->pSPIx, TRUE))) {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
        return TEST_APP_ARM_DRIVER_ERROR;
    }
    spi_i2s_reset(p_res->pSPIx);
    spi_init(p_res->pSPIx, p_res->Config.p_init_struct);
    drv_flag |= TEST_APP_ARM_SPI_FLAG_CONFIGURATED;
    spi_enable(p_res->pSPIx, TRUE);
    drv_flag |= TEST_APP_ARM_SPI_FLAG_ENABLED;
    drv_status |= ARM_SPI_GPIO_Config(p_res, TRUE);
//clear and enable SPIx IRQ
    NVIC_ClearPendingIRQ(p_res->IrqNum);
    NVIC_EnableIRQ(p_res->IrqNum);
    drv_flag |= TEST_APP_ARM_SPI_FLAG_INITIALIZED;
    p_res->Status.DrvStatus |= drv_status;
    p_res->Status.DrvFlag |= drv_flag;
    return drv_status;
}

uint32_t TEST_APP_ARM_SPI_Uninit(TEST_APP_ARM_SPI_Resources_t *p_res)
{
    uint32_t drv_status = TEST_APP_ARM_DRIVER_NO_ERROR;
    uint32_t drv_flag = p_res->Status.DrvFlag;
    if(drv_flag & TEST_APP_ARM_SPI_FLAG_INITIALIZED) {
        //disable and clear UARTx IRQ
        NVIC_DisableIRQ(p_res->IrqNum);
        NVIC_ClearPendingIRQ(p_res->IrqNum);
        drv_status |= ARM_SPI_GPIO_Config(p_res, FALSE);
        spi_i2s_reset(p_res->pSPIx);
        drv_flag &= ~TEST_APP_ARM_SPI_FLAG_CONFIGURATED;
        if(!(TEST_APP_ARM_CRM_SPI_ClockEnable(p_res->pSPIx, FALSE))) {
            p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
            return TEST_APP_ARM_DRIVER_ERROR;
        }
        memset(p_res, 0, sizeof(TEST_APP_ARM_SPI_Resources_t));
        drv_flag &= ~TEST_APP_ARM_SPI_FLAG_INITIALIZED;
    } else {
#ifdef _TEST_APP_DEBUG_
        LOG("SPI not been initialized");
#endif//_TEST_APP_DEBUG_
    }
    p_res->Status.DrvStatus |= drv_status;
    p_res->Status.DrvFlag |= drv_flag;
    return drv_status;
}

uint32_t TEST_APP_ARM_SPI_SetResources(TEST_APP_ARM_SPI_Resources_t *p_res,
                                       spi_type *p_spix,
                                       void *p_event_buff,
                                       void *p_tx_buff, void *p_rx_buff,
                                       spi_master_slave_mode_type mode,
                                       spi_transmission_mode_type xfer_mode,
                                       spi_half_duplex_direction_type dir,
                                       spi_mclk_freq_div_type mclk_div,
                                       eTEST_APP_ARM_SPI_ClockLatchTypes_t clk_type,
                                       spi_cs_mode_type cs_mode,
                                       eTEST_APP_ARM_SPI_CSActiveLevel_t cs_active_level,
                                       eTEST_APP_ARM_SPI_CSConfirmEveryWorld_t cs_conf_every_world,
                                       spi_frame_bit_num_type data_bit_num,
                                       spi_first_bit_type data_first_bit,
                                       uint32_t gpio_pin_def)
{
    p_res->pSPIx = p_spix;
    TEST_APP_RingBuffer_Init(&(p_res->Event), p_event_buff, TEST_APP_ARM_SPI_EVENT_BUFF_SIZE);
    p_res->Gpio.PinDef = gpio_pin_def;
    p_res->Config.p_init_struct->master_slave_mode = mode;
    p_res->Config.p_init_struct->transmission_mode = xfer_mode;
    p_res->Config.HalfDuplexDir = dir;
    p_res->Config.p_init_struct->mclk_freq_division = mclk_div;
    p_res->Config.p_init_struct->cs_mode_selection = cs_mode;
    p_res->Config.p_init_struct->frame_bit_num = data_bit_num;
    p_res->Config.p_init_struct->first_bit_transmission = data_first_bit;
    p_res->Config.ClkType = clk_type;
    if((xfer_mode == SPI_HALF_DUPLEX_DIRECTION_RX) || (xfer_mode == SPI_HALF_DUPLEX_DIRECTION_TX)) {
        spi_half_duplex_direction_set(p_res->pSPIx, p_res->Config.HalfDuplexDir);
    }
    switch(p_res->Config.ClkType) {
        case TEST_APP_ARM_SPI_CLOCK_POLAR_LOW_RISING_EDGE: {
            p_res->Config.p_init_struct->clock_polarity = SPI_CLOCK_POLARITY_LOW;
            p_res->Config.p_init_struct->clock_phase = SPI_CLOCK_PHASE_1EDGE;
        }
        break;
        case TEST_APP_ARM_SPI_CLOCK_POLAR_LOW_FALLING_EDGE: {
            p_res->Config.p_init_struct->clock_polarity = SPI_CLOCK_POLARITY_LOW;
            p_res->Config.p_init_struct->clock_phase = SPI_CLOCK_PHASE_2EDGE;

            break;
        }
        case TEST_APP_ARM_SPI_CLOCK_POLAR_HIGH_RISING_EDGE: {
            p_res->Config.p_init_struct->clock_polarity = SPI_CLOCK_POLARITY_HIGH;
            p_res->Config.p_init_struct->clock_phase = SPI_CLOCK_PHASE_2EDGE;

            break;
        }
        case TEST_APP_ARM_SPI_CLOCK_POLAR_HIGH_FALLING_EDGE: {
            p_res->Config.p_init_struct->clock_polarity = SPI_CLOCK_POLARITY_HIGH;
            p_res->Config.p_init_struct->clock_phase = SPI_CLOCK_PHASE_1EDGE;

            break;
        }
        default: {
            break;
        }
    }
    p_res->Config.CSActiveLevel = cs_active_level;
    p_res->Config.CSConfEveryWorldEn = cs_conf_every_world;
    if((p_res->Config.CSConfEveryWorldEn == TEST_APP_ARM_SPI_CS_CONFIRM_EVERY_WORLD_ENABLE) &&
       (p_res->Config.p_init_struct->master_slave_mode == SPI_MODE_SLAVE ||
        p_res->Config.p_init_struct->cs_mode_selection == SPI_CS_HARDWARE_MODE)) {
        return TEST_APP_ARM_DRIVER_ERROR_PARAMETER;
    }
    p_res->Status.DrvStatus = TEST_APP_ARM_DRIVER_NO_ERROR;
    p_res->Status.DrvFlag = 0U;
    p_res->Status.XferStatus.TxBusy = 0;
    p_res->Status.XferStatus.RxBusy = 0;
    p_res->Status.XferStatus.RxOverflow = 0;
    p_res->Status.XferStatus.ModeFault = 0;
    p_res->Transfer.pTxData = p_tx_buff;
    p_res->Transfer.pRxData = p_rx_buff;
    p_res->Transfer.TxNum = 0;
    p_res->Transfer.RxNum = 0;
    p_res->Transfer.TxCnt = 0;
    p_res->Transfer.RxCnt = 0;
    /****************************************************************
     SPI1
    ***************************************************************/
    if(p_res->pSPIx == SPI1) {
        p_res->IrqNum = SPI1_IRQn;
        switch(p_res->Gpio.PinDef) {
            case TEST_APP_ARM_SPI_GPIO_PIN_DEF_DEFAULT: {
                p_res->Gpio.pCSGpio = GPIOA;
                p_res->Gpio.CSPin = GPIO_PINS_4;
                p_res->Gpio.pMOSIGpio = GPIOA;
                p_res->Gpio.MOSIPin = GPIO_PINS_7;
                p_res->Gpio.pMISOGpio = GPIOA;
                p_res->Gpio.MISOPin = GPIO_PINS_6;
                p_res->Gpio.pSCLKGpio = GPIOA;
                p_res->Gpio.SCLKPin = GPIO_PINS_5;
                break;
            }
            case TEST_APP_ARM_SPI1_GPIO_PIN_DEF_REMAP1: {
                p_res->Gpio.pCSGpio = GPIOA;
                p_res->Gpio.CSPin = GPIO_PINS_15;
                p_res->Gpio.pMOSIGpio = GPIOB;
                p_res->Gpio.MOSIPin = GPIO_PINS_5;
                p_res->Gpio.pMISOGpio = GPIOB;
                p_res->Gpio.MISOPin = GPIO_PINS_4;
                p_res->Gpio.pSCLKGpio = GPIOB;
                p_res->Gpio.SCLKPin = GPIO_PINS_3;
                break;
            }
            default: {
                break;
            }
        }
        /****************************************************************
         SPI2
        ***************************************************************/
    } else if(p_res->pSPIx == SPI2) {
        p_res->IrqNum = SPI2_I2S2EXT_IRQn;
        switch(p_res->Gpio.PinDef) {
            case TEST_APP_ARM_SPI_GPIO_PIN_DEF_DEFAULT: {
                p_res->Gpio.pCSGpio = GPIOB;
                p_res->Gpio.CSPin = GPIO_PINS_12;
                p_res->Gpio.pMOSIGpio = GPIOB;
                p_res->Gpio.MOSIPin = GPIO_PINS_15;
                p_res->Gpio.pMISOGpio = GPIOB;
                p_res->Gpio.MISOPin = GPIO_PINS_14;
                p_res->Gpio.pSCLKGpio = GPIOB;
                p_res->Gpio.SCLKPin = GPIO_PINS_13;
                break;
            }
            default: {
                break;
            }
        }
        /****************************************************************
         SPI3
        ***************************************************************/
    } else if(p_res->pSPIx == SPI3) {
        p_res->IrqNum = SPI3_I2S3EXT_IRQn;
        switch(p_res->Gpio.PinDef) {
            case TEST_APP_ARM_SPI_GPIO_PIN_DEF_DEFAULT: {
                p_res->Gpio.pCSGpio = GPIOA;
                p_res->Gpio.CSPin = GPIO_PINS_15;
                p_res->Gpio.pMOSIGpio = GPIOB;
                p_res->Gpio.MOSIPin = GPIO_PINS_5;
                p_res->Gpio.pMISOGpio = GPIOB;
                p_res->Gpio.MISOPin = GPIO_PINS_4;
                p_res->Gpio.pSCLKGpio = GPIOB;
                p_res->Gpio.SCLKPin = GPIO_PINS_3;
                break;
            }
            case TEST_APP_ARM_SPI3_GPIO_PIN_DEF_REMAP1: {
                p_res->Gpio.pCSGpio = GPIOA;
                p_res->Gpio.CSPin = GPIO_PINS_4;
                p_res->Gpio.pMOSIGpio = GPIOC;
                p_res->Gpio.MOSIPin = GPIO_PINS_12;
                p_res->Gpio.pMISOGpio = GPIOC;
                p_res->Gpio.MISOPin = GPIO_PINS_11;
                p_res->Gpio.pSCLKGpio = GPIOC;
                p_res->Gpio.SCLKPin = GPIO_PINS_10;
                break;
            }
            default: {
                break;
            }
        }
        /****************************************************************
        SPI4
        ***************************************************************/
    } else if(p_res->pSPIx == SPI4) {
        p_res->IrqNum = SPI4_IRQn;
        switch(p_res->Gpio.PinDef) {
            case TEST_APP_ARM_SPI_GPIO_PIN_DEF_DEFAULT: {
                p_res->Gpio.pCSGpio = GPIOE;
                p_res->Gpio.CSPin = GPIO_PINS_4;
                p_res->Gpio.pMOSIGpio = GPIOE;
                p_res->Gpio.MOSIPin = GPIO_PINS_6;
                p_res->Gpio.pMISOGpio = GPIOE;
                p_res->Gpio.MISOPin = GPIO_PINS_5;
                p_res->Gpio.pSCLKGpio = GPIOE;
                p_res->Gpio.SCLKPin = GPIO_PINS_2;
                break;
            }
            case TEST_APP_ARM_SPI4_GPIO_PIN_DEF_REMAP1: {
                p_res->Gpio.pCSGpio = GPIOE;
                p_res->Gpio.CSPin = GPIO_PINS_12;
                p_res->Gpio.pMOSIGpio = GPIOE;
                p_res->Gpio.MOSIPin = GPIO_PINS_14;
                p_res->Gpio.pMISOGpio = GPIOE;
                p_res->Gpio.MISOPin = GPIO_PINS_13;
                p_res->Gpio.pSCLKGpio = GPIOE;
                p_res->Gpio.SCLKPin = GPIO_PINS_11;
                break;
            }
            case TEST_APP_ARM_SPI4_GPIO_PIN_DEF_REMAP2: {
                p_res->Gpio.pCSGpio = GPIOB;
                p_res->Gpio.CSPin = GPIO_PINS_6;
                p_res->Gpio.pMOSIGpio = GPIOB;
                p_res->Gpio.MOSIPin = GPIO_PINS_9;
                p_res->Gpio.pMISOGpio = GPIOB;
                p_res->Gpio.MISOPin = GPIO_PINS_8;
                p_res->Gpio.pSCLKGpio = GPIOB;
                p_res->Gpio.SCLKPin = GPIO_PINS_7;
                break;
            }
            default: {
                break;
            }
        }
    } else {
#ifdef _TEST_APP_DEBUG_
        LOG("SPI configuration error");
#endif//_TEST_APP_DEBUG_
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR_UNSUPPORTED;
        return TEST_APP_ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    return TEST_APP_ARM_DRIVER_NO_ERROR;
}

void TEST_APP_ARM_SPI_SetCSActiveLevel(TEST_APP_ARM_SPI_Resources_t *p_res, confirm_state new_state)
{
    if(new_state) {
        if(p_res->Config.CSActiveLevel == TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_LOW) {
            TEST_APP_ARM_GPIO_BitsReset(p_res->Gpio.pCSGpio, p_res->Gpio.CSPin);
        } else {
            TEST_APP_ARM_GPIO_BitsSet(p_res->Gpio.pCSGpio, p_res->Gpio.CSPin);
        }
    } else {
        if(p_res->Config.CSActiveLevel == TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_LOW) {
            TEST_APP_ARM_GPIO_BitsSet(p_res->Gpio.pCSGpio, p_res->Gpio.CSPin);
        } else {
            TEST_APP_ARM_GPIO_BitsReset(p_res->Gpio.pCSGpio, p_res->Gpio.CSPin);
        }
    }
}

void TEST_APP_ARM_SPI_IRQHandler(TEST_APP_ARM_SPI_Resources_t *p_res)
{
    uint32_t event = 0;
    if(p_res->Status.XferStatus.RxBusy == 1) {
        if(spi_i2s_flag_get(p_res->pSPIx, SPI_I2S_RDBF_FLAG) == SET) {
            if(p_res->Config.p_init_struct->frame_bit_num == SPI_FRAME_8BIT) {
                *((uint8_t *)p_res->Transfer.pRxData + p_res->Transfer.RxCnt) =
                    TEST_APP_ARM_SPI_ReadData(p_res);
            } else {
                *((uint16_t *)p_res->Transfer.pRxData + p_res->Transfer.RxCnt) =
                    TEST_APP_ARM_SPI_ReadData(p_res);
            }
            if(p_res->Config.CSConfEveryWorldEn == TEST_APP_ARM_SPI_CS_CONFIRM_EVERY_WORLD_ENABLE) {
                if(p_res->Config.CSActiveLevel == TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_LOW) {
                    TEST_APP_ARM_GPIO_BitsReset(p_res->Gpio.pCSGpio, p_res->Gpio.CSPin);
                    TEST_APP_ARM_GPIO_BitsSet(p_res->Gpio.pCSGpio, p_res->Gpio.CSPin);
                } else {
                    TEST_APP_ARM_GPIO_BitsSet(p_res->Gpio.pCSGpio, p_res->Gpio.CSPin);
                    TEST_APP_ARM_GPIO_BitsReset(p_res->Gpio.pCSGpio, p_res->Gpio.CSPin);
                }
            }
            p_res->Transfer.RxCnt++;
        }
        if(spi_i2s_flag_get(p_res->pSPIx, SPI_I2S_ROERR_FLAG) == SET) {
            p_res->Status.XferStatus.RxOverflow = 1;
            event |= TEST_APP_ARM_SPI_EVENT_RX_OVERFLOW;
        }
        if(spi_i2s_flag_get(p_res->pSPIx, SPI_MMERR_FLAG) == SET) {
            p_res->Status.XferStatus.ModeFault = 1;
            event |= TEST_APP_ARM_SPI_EVENT_MODE_FAULT;
        }
        if(p_res->Transfer.RxCnt == p_res->Transfer.RxNum) {
            event |= TEST_APP_ARM_SPI_EVENT_RX_COMPLETE;
            p_res->Status.XferStatus.RxBusy = 0;
            spi_i2s_interrupt_enable(p_res->pSPIx, SPI_I2S_RDBF_INT, FALSE);
            spi_i2s_interrupt_enable(p_res->pSPIx, SPI_I2S_ERROR_INT, FALSE);
        }
    }
    if(p_res->Status.XferStatus.TxBusy == 1) {
        if(spi_i2s_flag_get(p_res->pSPIx, SPI_I2S_TDBE_FLAG) == SET) {
            if(p_res->Config.p_init_struct->frame_bit_num == SPI_FRAME_8BIT) {
                TEST_APP_ARM_SPI_WriteData(p_res, (uint8_t *)p_res->Transfer.pTxData +
                                           p_res->Transfer.TxCnt);
            } else {
                TEST_APP_ARM_SPI_WriteData(p_res, (uint16_t *)p_res->Transfer.pTxData +
                                           p_res->Transfer.TxCnt);
            }
        }
        p_res->Transfer.TxCnt++;
        if(spi_i2s_flag_get(p_res->pSPIx, SPI_MMERR_FLAG) == SET) {
            p_res->Status.XferStatus.ModeFault = 1;
            event |= TEST_APP_ARM_SPI_EVENT_MODE_FAULT;
        }
        if(p_res->Transfer.TxCnt == p_res->Transfer.TxNum) {
            event |= TEST_APP_ARM_SPI_EVENT_TX_COMPLETE;
            p_res->Status.XferStatus.TxBusy = 0;
            spi_i2s_interrupt_enable(p_res->pSPIx, SPI_I2S_TDBE_INT, FALSE);
            spi_i2s_interrupt_enable(p_res->pSPIx, SPI_I2S_ERROR_INT, FALSE);
        }
    }
    if(event) {
        TEST_APP_RingBuffer_Write(&(p_res->Event), &event);
    }
}


void TEST_APP_ARM_SPI_cb(TEST_APP_ARM_SPI_Resources_t *p_res)
{
    uint32_t event = 0;
    TEST_APP_RingBuffer_Read(&(p_res->Event), &event);
    if(event & TEST_APP_ARM_SPI_EVENT_RX_COMPLETE) {
        p_res->Transfer.RxCnt = 0;
        p_res->Transfer.RxNum = 0;
        p_res->Status.XferStatus.RxBusy = 0;
        while(p_res->Status.XferStatus.TxBusy == 0);
        if(p_res->Config.p_init_struct->cs_mode_selection == SPI_CS_SOFTWARE_MODE) {
            if(p_res->Config.p_init_struct->master_slave_mode == SPI_MODE_MASTER) {
                if(p_res->Config.CSActiveLevel == TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_LOW) {
                    TEST_APP_ARM_GPIO_BitsSet(p_res->Gpio.pCSGpio, p_res->Gpio.CSPin);
                } else {
                    TEST_APP_ARM_GPIO_BitsReset(p_res->Gpio.pCSGpio, p_res->Gpio.CSPin);
                }
            } else {
                if(p_res->Config.CSActiveLevel == TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_LOW) {
                    spi_software_cs_internal_level_set(p_res->pSPIx, SPI_SWCS_INTERNAL_LEVEL_HIGHT);
                } else {
                    spi_software_cs_internal_level_set(p_res->pSPIx, SPI_SWCS_INTERNAL_LEVEL_LOW);
                }
            }
        }
#ifdef _TEST_APP_DEBUG_
        TEST_APP_LCD2004_Printf(0, 0, SET, "%d", p_res->Transfer.pRxData);
        LOG("SPI recieved test data");
#endif//_TEST_APP_DEBUG_ 
    }
    if(event & TEST_APP_ARM_SPI_EVENT_TX_COMPLETE) {
        p_res->Transfer.TxCnt = 0;
        p_res->Transfer.TxNum = 0;
        p_res->Status.XferStatus.TxBusy = 0;
        while(p_res->Status.XferStatus.RxBusy == 0);
        if(p_res->Config.p_init_struct->cs_mode_selection == SPI_CS_SOFTWARE_MODE) {
            if(p_res->Config.p_init_struct->master_slave_mode == SPI_MODE_MASTER) {
                if(p_res->Config.CSActiveLevel == TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_LOW) {
                    TEST_APP_ARM_GPIO_BitsSet(p_res->Gpio.pCSGpio, p_res->Gpio.CSPin);
                } else {
                    TEST_APP_ARM_GPIO_BitsReset(p_res->Gpio.pCSGpio, p_res->Gpio.CSPin);
                }
            } else {
                if(p_res->Config.CSActiveLevel == TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_LOW) {
                    spi_software_cs_internal_level_set(p_res->pSPIx, SPI_SWCS_INTERNAL_LEVEL_HIGHT);
                } else {
                    spi_software_cs_internal_level_set(p_res->pSPIx, SPI_SWCS_INTERNAL_LEVEL_LOW);
                }
            }
        }
    }
    if(event & TEST_APP_ARM_SPI_EVENT_RX_OVERFLOW) {
#ifdef _TEST_APP_DEBUG_
        LOG("SPI receiver overflow");
#endif//_TEST_APP_DEBUG_                
    }
    if(event & TEST_APP_ARM_SPI_EVENT_MODE_FAULT) {
#ifdef _TEST_APP_DEBUG_
        LOG("SPI receiver mode fault error");
#endif//_TEST_APP_DEBUG_                
    }
}

uint32_t TEST_APP_ARM_SPI_Send(TEST_APP_ARM_SPI_Resources_t *p_res, void *pdata, uint32_t num)
{
    if(!(p_res->Status.DrvFlag & TEST_APP_ARM_SPI_FLAG_ENABLED)) {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
        return TEST_APP_ARM_DRIVER_ERROR;
    }
    if(num == 0) {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR_PARAMETER;
        return TEST_APP_ARM_DRIVER_ERROR_PARAMETER;
    }
    if(p_res->Status.XferStatus.TxBusy) {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR_BUSY;
        return TEST_APP_ARM_DRIVER_ERROR_BUSY;
    } else {
        p_res->Status.DrvStatus &= ~TEST_APP_ARM_DRIVER_ERROR_BUSY;
    }

    p_res->Status.XferStatus.TxBusy = 1;
    p_res->Transfer.TxCnt = 0;
    p_res->Transfer.TxNum = num;
    p_res->Transfer.pTxData = pdata;
    switch(p_res->Config.p_init_struct->transmission_mode) {
        case SPI_TRANSMIT_FULL_DUPLEX:
        case SPI_TRANSMIT_HALF_DUPLEX_TX: {
            break;
        }
        case SPI_TRANSMIT_SIMPLEX_RX:
        case SPI_TRANSMIT_HALF_DUPLEX_RX: {
            p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
            return TEST_APP_ARM_DRIVER_ERROR;
        }
        default: {
            p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
            return TEST_APP_ARM_DRIVER_ERROR;
        }
    }
    if(p_res->Config.p_init_struct->cs_mode_selection == SPI_CS_SOFTWARE_MODE) {
        if(p_res->Config.p_init_struct->master_slave_mode == SPI_MODE_MASTER) {
            if(p_res->Config.CSActiveLevel == TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_LOW) {
                TEST_APP_ARM_GPIO_BitsReset(p_res->Gpio.pCSGpio, p_res->Gpio.CSPin);
            } else {
                TEST_APP_ARM_GPIO_BitsSet(p_res->Gpio.pCSGpio, p_res->Gpio.CSPin);
            }
        } else {
            if(p_res->Config.CSActiveLevel == TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_LOW) {
                spi_software_cs_internal_level_set(p_res->pSPIx, SPI_SWCS_INTERNAL_LEVEL_LOW);
            } else {
                spi_software_cs_internal_level_set(p_res->pSPIx, SPI_SWCS_INTERNAL_LEVEL_HIGHT);
            }
        }
    }
    spi_i2s_interrupt_enable(p_res->pSPIx, SPI_I2S_TDBE_INT, TRUE);
    spi_i2s_interrupt_enable(p_res->pSPIx, SPI_I2S_ERROR_INT, TRUE);
    return TEST_APP_ARM_DRIVER_NO_ERROR;
}

uint32_t TEST_APP_ARM_SPI_Recieve(TEST_APP_ARM_SPI_Resources_t *p_res, void *pdata, uint32_t num)
{
    if(!(p_res->Status.DrvFlag & TEST_APP_ARM_SPI_FLAG_ENABLED)) {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
        return TEST_APP_ARM_DRIVER_ERROR;
    }
    if(num == 0) {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR_PARAMETER;
        return TEST_APP_ARM_DRIVER_ERROR_PARAMETER;
    }
    if(p_res->Status.XferStatus.RxBusy) {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR_BUSY;
        return TEST_APP_ARM_DRIVER_ERROR_BUSY;
    } else {
        p_res->Status.DrvStatus &= ~TEST_APP_ARM_DRIVER_ERROR_BUSY;
    }
    p_res->Status.XferStatus.RxBusy = 1;
    p_res->Transfer.RxCnt = 0;
    p_res->Transfer.RxNum = num;
    p_res->Transfer.pRxData = pdata;
    switch(p_res->Config.p_init_struct->transmission_mode) {
        case SPI_TRANSMIT_FULL_DUPLEX:
        case SPI_TRANSMIT_SIMPLEX_RX:
        case SPI_TRANSMIT_HALF_DUPLEX_RX: {
            break;
        }
        case SPI_TRANSMIT_HALF_DUPLEX_TX: {
            p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
            return TEST_APP_ARM_DRIVER_ERROR;
        }
        default: {
            p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
            return TEST_APP_ARM_DRIVER_ERROR;
        }
    }
    if(p_res->Config.p_init_struct->cs_mode_selection == SPI_CS_SOFTWARE_MODE) {
        if(p_res->Config.p_init_struct->master_slave_mode == SPI_MODE_MASTER) {
            if(p_res->Config.CSActiveLevel == TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_LOW) {
                TEST_APP_ARM_GPIO_BitsReset(p_res->Gpio.pCSGpio, p_res->Gpio.CSPin);
            } else {
                TEST_APP_ARM_GPIO_BitsSet(p_res->Gpio.pCSGpio, p_res->Gpio.CSPin);
            }
        } else {
            if(p_res->Config.CSActiveLevel == TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_LOW) {
                spi_software_cs_internal_level_set(p_res->pSPIx, SPI_SWCS_INTERNAL_LEVEL_LOW);
            } else {
                spi_software_cs_internal_level_set(p_res->pSPIx, SPI_SWCS_INTERNAL_LEVEL_HIGHT);
            }
        }
    }
    spi_i2s_interrupt_enable(p_res->pSPIx, SPI_I2S_RDBF_INT, TRUE);
    spi_i2s_interrupt_enable(p_res->pSPIx, SPI_I2S_ERROR_INT, TRUE);
    return TEST_APP_ARM_DRIVER_NO_ERROR;
}

void TEST_APP_ARM_SPI_WriteData(TEST_APP_ARM_SPI_Resources_t *p_res, void *pdata)
{
    uint16_t data;
    if(p_res->Config.p_init_struct->frame_bit_num == SPI_FRAME_8BIT) {
        data = *(uint8_t *)pdata;
    } else {
        data = *(uint16_t *)pdata;
    }
    p_res->pSPIx->dt = (uint16_t)data;
}

uint16_t TEST_APP_ARM_SPI_ReadData(TEST_APP_ARM_SPI_Resources_t *p_res)
{
    return (uint16_t)p_res->pSPIx->dt;
}

TEST_APP_ARM_SPI_Status_t TEST_APP_ARM_SPI_GetStatus(TEST_APP_ARM_SPI_Resources_t *p_res)
{
    TEST_APP_ARM_SPI_Status_t status;
    status.DrvStatus = p_res->Status.DrvStatus;
    status.DrvFlag = p_res->Status.DrvFlag;
    status.XferStatus.TxBusy = p_res->Status.XferStatus.TxBusy;
    status.XferStatus.RxBusy = p_res->Status.XferStatus.RxBusy;
    status.XferStatus.RxOverflow = p_res->Status.XferStatus.RxOverflow;
    status.XferStatus.ModeFault = p_res->Status.XferStatus.ModeFault;
    return status;
}

TEST_APP_ARM_SPI_Transfer_t TEST_APP_ARM_SPI_GetTransfer(TEST_APP_ARM_SPI_Resources_t *p_res)
{
    TEST_APP_ARM_SPI_Transfer_t transfer;
    transfer.pTxData = p_res->Transfer.pTxData;
    transfer.pRxData = p_res->Transfer.pRxData;
    transfer.TxNum = p_res->Transfer.TxNum;
    transfer.RxNum = p_res->Transfer.RxNum;
    transfer.TxCnt = p_res->Transfer.TxCnt;
    transfer.RxCnt = p_res->Transfer.RxCnt;
    return transfer;
}

//================================================================================
//Private
//================================================================================

static uint32_t ARM_SPI_GPIO_Config(TEST_APP_ARM_SPI_Resources_t *p_res, confirm_state new_state)
{
    if(new_state) {
//enable GPIO clock
        if(!TEST_APP_ARM_CRM_GPIO_ClockEnable(p_res->Gpio.pCSGpio, TRUE)) {
#ifdef _TEST_APP_DEBUG_
            LOG("SPI configuration error");
#endif//_TEST_APP_DEBUG_        
            p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
            return TEST_APP_ARM_DRIVER_ERROR;
        }
        if(!TEST_APP_ARM_CRM_GPIO_ClockEnable(p_res->Gpio.pSCLKGpio, TRUE)) {
#ifdef _TEST_APP_DEBUG_
            LOG("SPI configuration error");
#endif//_TEST_APP_DEBUG_
            p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
            return TEST_APP_ARM_DRIVER_ERROR;
        }
        if(!TEST_APP_ARM_CRM_GPIO_ClockEnable(p_res->Gpio.pMISOGpio, TRUE)) {
#ifdef _TEST_APP_DEBUG_
            LOG("SPI configuration error");
#endif//_TEST_APP_DEBUG_
            p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
            return TEST_APP_ARM_DRIVER_ERROR;
        }
        if(!TEST_APP_ARM_CRM_GPIO_ClockEnable(p_res->Gpio.pMOSIGpio, TRUE)) {
#ifdef _TEST_APP_DEBUG_
            LOG("SPI configuration error");
#endif//_TEST_APP_DEBUG_
            p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
            return TEST_APP_ARM_DRIVER_ERROR;
        }
//enable Gpio IOMUX clock (for pin remapping)
        if(!(p_res->Gpio.PinDef == TEST_APP_ARM_SPI_GPIO_PIN_DEF_DEFAULT)) {
            crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
            gpio_pin_remap_config(p_res->Gpio.PinDef, TRUE);
        }
//configure GPIO i/o
        if(p_res->Config.p_init_struct->master_slave_mode == SPI_MODE_MASTER) {
            TEST_APP_ARM_GPIO_Config(p_res->Gpio.pSCLKGpio, p_res->Gpio.SCLKPin, GPIO_MODE_MUX, GPIO_OUTPUT_PUSH_PULL,
                                     GPIO_PULL_DOWN, GPIO_DRIVE_STRENGTH_STRONGER);
            if(p_res->Config.p_init_struct->cs_mode_selection == SPI_CS_HARDWARE_MODE) {
                TEST_APP_ARM_GPIO_Config(p_res->Gpio.pCSGpio, p_res->Gpio.CSPin, GPIO_MODE_MUX, GPIO_OUTPUT_PUSH_PULL,
                                         GPIO_PULL_DOWN, GPIO_DRIVE_STRENGTH_STRONGER);
            } else {
                TEST_APP_ARM_GPIO_Config(p_res->Gpio.pCSGpio, p_res->Gpio.CSPin, GPIO_MODE_OUTPUT, GPIO_OUTPUT_PUSH_PULL,
                                         GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_STRONGER);
            }
            switch(p_res->Config.p_init_struct->transmission_mode) {
                case SPI_TRANSMIT_FULL_DUPLEX: {
                    TEST_APP_ARM_GPIO_Config(p_res->Gpio.pMOSIGpio, p_res->Gpio.MOSIPin, GPIO_MODE_MUX, GPIO_OUTPUT_PUSH_PULL,
                                             GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
                    TEST_APP_ARM_GPIO_Config(p_res->Gpio.pMISOGpio, p_res->Gpio.MISOPin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                                             GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
                    break;
                }
                case SPI_TRANSMIT_SIMPLEX_RX: {
                    TEST_APP_ARM_GPIO_Config(p_res->Gpio.pMISOGpio, p_res->Gpio.MISOPin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                                             GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
                    break;
                }
                case SPI_TRANSMIT_HALF_DUPLEX_RX:
                case SPI_TRANSMIT_HALF_DUPLEX_TX: {
                    TEST_APP_ARM_GPIO_Config(p_res->Gpio.pMOSIGpio, p_res->Gpio.MOSIPin, GPIO_MODE_MUX, GPIO_OUTPUT_PUSH_PULL,
                                             GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
                    break;
                }
                default: {
                    return TEST_APP_ARM_DRIVER_ERROR;
                }
            }
        }
        if(p_res->Config.p_init_struct->master_slave_mode == SPI_MODE_SLAVE) {
            TEST_APP_ARM_GPIO_Config(p_res->Gpio.pSCLKGpio, p_res->Gpio.SCLKPin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                                     GPIO_PULL_DOWN, GPIO_DRIVE_STRENGTH_STRONGER);
            if(p_res->Config.p_init_struct->cs_mode_selection == SPI_CS_HARDWARE_MODE) {
                TEST_APP_ARM_GPIO_Config(p_res->Gpio.pCSGpio, p_res->Gpio.CSPin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                                         GPIO_PULL_DOWN, GPIO_DRIVE_STRENGTH_STRONGER);
            } else {
                TEST_APP_ARM_GPIO_Config(p_res->Gpio.pCSGpio, p_res->Gpio.CSPin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                                         GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
            }
            switch(p_res->Config.p_init_struct->transmission_mode) {
                case SPI_TRANSMIT_FULL_DUPLEX: {
                    TEST_APP_ARM_GPIO_Config(p_res->Gpio.pMOSIGpio, p_res->Gpio.MOSIPin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                                             GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
                    TEST_APP_ARM_GPIO_Config(p_res->Gpio.pMISOGpio, p_res->Gpio.MISOPin, GPIO_MODE_MUX, GPIO_OUTPUT_PUSH_PULL,
                                             GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
                    break;
                }
                case SPI_TRANSMIT_SIMPLEX_RX: {
                    TEST_APP_ARM_GPIO_Config(p_res->Gpio.pMOSIGpio, p_res->Gpio.MOSIPin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                                             GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
                    break;
                }
                case SPI_TRANSMIT_HALF_DUPLEX_RX:
                case SPI_TRANSMIT_HALF_DUPLEX_TX: {
                    TEST_APP_ARM_GPIO_Config(p_res->Gpio.pMISOGpio, p_res->Gpio.MISOPin, GPIO_MODE_MUX, GPIO_OUTPUT_PUSH_PULL,
                                             GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
                    break;
                }
                default: {
                    return TEST_APP_ARM_DRIVER_ERROR;
                }
            }
        }
    } else {
//set i/o default configuration
        if(p_res->Config.p_init_struct->cs_mode_selection == SPI_CS_HARDWARE_MODE) {
            TEST_APP_ARM_GPIO_Config(p_res->Gpio.pCSGpio, p_res->Gpio.CSPin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                                     GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
        }
        TEST_APP_ARM_GPIO_Config(p_res->Gpio.pSCLKGpio, p_res->Gpio.SCLKPin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                                 GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
        if(p_res->Config.p_init_struct->master_slave_mode == SPI_MODE_MASTER) {
            switch(p_res->Config.p_init_struct->transmission_mode) {
                case SPI_TRANSMIT_FULL_DUPLEX: {
                    TEST_APP_ARM_GPIO_Config(p_res->Gpio.pMOSIGpio, p_res->Gpio.MOSIPin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                                             GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
                    TEST_APP_ARM_GPIO_Config(p_res->Gpio.pMISOGpio, p_res->Gpio.MISOPin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                                             GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
                    break;
                }
                case SPI_TRANSMIT_SIMPLEX_RX: {
                    TEST_APP_ARM_GPIO_Config(p_res->Gpio.pMISOGpio, p_res->Gpio.MISOPin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                                             GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
                    break;
                }
                case SPI_TRANSMIT_HALF_DUPLEX_RX:
                case SPI_TRANSMIT_HALF_DUPLEX_TX: {
                    TEST_APP_ARM_GPIO_Config(p_res->Gpio.pMOSIGpio, p_res->Gpio.MOSIPin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                                             GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
                    break;
                }
                default: {
                    return TEST_APP_ARM_DRIVER_ERROR;
                }
            }
            if(p_res->Config.p_init_struct->master_slave_mode == SPI_MODE_SLAVE) {
                switch(p_res->Config.p_init_struct->transmission_mode) {
                    case SPI_TRANSMIT_FULL_DUPLEX: {
                        TEST_APP_ARM_GPIO_Config(p_res->Gpio.pMOSIGpio, p_res->Gpio.MOSIPin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                                                 GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
                        TEST_APP_ARM_GPIO_Config(p_res->Gpio.pMISOGpio, p_res->Gpio.MISOPin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                                                 GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
                        break;
                    }
                    case SPI_TRANSMIT_SIMPLEX_RX: {
                        TEST_APP_ARM_GPIO_Config(p_res->Gpio.pMOSIGpio, p_res->Gpio.MOSIPin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                                                 GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
                        break;
                    }
                    case SPI_TRANSMIT_HALF_DUPLEX_RX:
                    case SPI_TRANSMIT_HALF_DUPLEX_TX: {
                        TEST_APP_ARM_GPIO_Config(p_res->Gpio.pMISOGpio, p_res->Gpio.MISOPin, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                                                 GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
                        break;
                    }
                    default: {
                        return TEST_APP_ARM_DRIVER_ERROR;
                    }
                }
            }
//Gpio IOMUX clock disable (for pin remapping release)
            if(!(p_res->Gpio.PinDef == TEST_APP_ARM_SPI_GPIO_PIN_DEF_DEFAULT)) {
                gpio_pin_remap_config(p_res->Gpio.PinDef, FALSE);
            }
        }
    }
    return TEST_APP_ARM_DRIVER_NO_ERROR;
}
