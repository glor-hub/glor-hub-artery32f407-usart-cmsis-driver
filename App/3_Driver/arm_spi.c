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
    drv_status |= ARM_SPI_GPIO_Config(p_res, TRUE);
//clear and enable UARTx IRQ
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
                                       spi_mclk_freq_div_type mclk_div,
                                       eTEST_APP_ARM_SPI_ClockLatchTypes_t clk_type,
                                       spi_cs_mode_type cs_mode,
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
    p_res->Config.p_init_struct->mclk_freq_division = mclk_div;
    p_res->Config.p_init_struct->cs_mode_selection = cs_mode;
    p_res->Config.p_init_struct->frame_bit_num = data_bit_num;
    p_res->Config.p_init_struct->first_bit_transmission = data_first_bit;
    p_res->Config.ClkType = clk_type;
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
    p_res->Config.CSConfEveryWorldEn = cs_conf_every_world;
    p_res->Status.DrvStatus = TEST_APP_ARM_DRIVER_NO_ERROR;
    p_res->Status.DrvFlag = 0U;
    p_res->Status.XferStatus.Busy = 0;
    p_res->Status.XferStatus.DataLost = 0;
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


//================================================================================
//Private
//================================================================================

static uint32_t ARM_SPI_GPIO_Config(TEST_APP_ARM_SPI_Resources_t *p_res, confirm_state new_state)
{
    if(new_state) {
//enable GPIO clock
        if(p_res->Config.p_init_struct->cs_mode_selection == SPI_CS_HARDWARE_MODE) {
            if(!TEST_APP_ARM_CRM_GPIO_ClockEnable(p_res->Gpio.pCSGpio, TRUE)) {
#ifdef _TEST_APP_DEBUG_
                LOG("SPI configuration error");
#endif//_TEST_APP_DEBUG_        
                p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
                return TEST_APP_ARM_DRIVER_ERROR;
            }
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
