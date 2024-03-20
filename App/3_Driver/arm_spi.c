//********************************************************************************
//arm_spi.c
//********************************************************************************
#include <string.h>
#include "arm_driver.h"
#include "arm_spi.h"
#include "arm_clock.h"
#include "arm_gpio.h"
#include "software_timer.h"
#include "systick_timer.h"

//********************************************************************************
//Macros
//********************************************************************************

#define ARM_SPI_EVENT_BUFF_SIZE 8

#define ARM_SPI_TX_BUFF_SIZE 256
#define ARM_SPI_RX_BUFF_SIZE 256

#define ARM_SPI_TIMEOUT_MSEC 200

//********************************************************************************
//Enums
//********************************************************************************

typedef enum {
    ARM_SPI_TRANSFER_TX = 0,
    ARM_SPI_TRANSFER_RX,
    ARM_SPI_TRANSFER_TYPES
} eARM_SPI_TransferTypes_t;


//********************************************************************************
//Typedefs
//********************************************************************************


//********************************************************************************
//Prototypes
//********************************************************************************

static void  ARM_SPI_SetResources(eTEST_APP_ARM_SPI_Types_t spi_type,
                                  spi_master_slave_mode_type mode,
                                  spi_transmission_mode_type xfer_mode,
                                  spi_mclk_freq_div_type mclk_div,
                                  eTEST_APP_ARM_SPI_ClockLatchTypes_t clk_type,
                                  spi_cs_mode_type cs_mode,
                                  eTEST_APP_ARM_SPI_CSActiveLevel_t cs_active_level,
                                  eTEST_APP_ARM_SPI_CSConfirmEveryWorld_t cs_conf_every_world,
                                  uint32_t spi_delay_after_cs_usec,
                                  spi_frame_bit_num_type data_bit_num,
                                  spi_first_bit_type data_first_bit,
                                  eTEST_APP_ARM_SPI_PinDefTypes_t gpio_pin_def_type);
static uint32_t ARM_SPI_GPIO_Config(eTEST_APP_ARM_SPI_Types_t spi_type, confirm_state new_state);

static uint32_t ARM_SPI_Initialize(eTEST_APP_ARM_SPI_Types_t spi_type,
                                   spi_master_slave_mode_type mode,
                                   spi_transmission_mode_type xfer_mode,
                                   spi_mclk_freq_div_type mclk_div,
                                   eTEST_APP_ARM_SPI_ClockLatchTypes_t clk_type,
                                   spi_cs_mode_type cs_mode,
                                   eTEST_APP_ARM_SPI_CSActiveLevel_t cs_active_level,
                                   eTEST_APP_ARM_SPI_CSConfirmEveryWorld_t cs_conf_every_world,
                                   uint32_t spi_delay_after_cs_usec,
                                   spi_frame_bit_num_type data_bit_num,
                                   spi_first_bit_type data_first_bit,
                                   eTEST_APP_ARM_SPI_PinDefTypes_t gpio_pin_def_type);

static uint32_t ARM_SPI_Initialize_1(spi_master_slave_mode_type mode,
                                     spi_transmission_mode_type xfer_mode,
                                     spi_mclk_freq_div_type mclk_div,
                                     eTEST_APP_ARM_SPI_ClockLatchTypes_t clk_type,
                                     spi_cs_mode_type cs_mode,
                                     eTEST_APP_ARM_SPI_CSActiveLevel_t cs_active_level,
                                     eTEST_APP_ARM_SPI_CSConfirmEveryWorld_t cs_conf_every_world,
                                     uint32_t spi_delay_after_cs_usec,
                                     spi_frame_bit_num_type data_bit_num,
                                     spi_first_bit_type data_first_bit,
                                     eTEST_APP_ARM_SPI_PinDefTypes_t gpio_pin_def_type);
static uint32_t ARM_SPI_Initialize_2(spi_master_slave_mode_type mode,
                                     spi_transmission_mode_type xfer_mode,
                                     spi_mclk_freq_div_type mclk_div,
                                     eTEST_APP_ARM_SPI_ClockLatchTypes_t clk_type,
                                     spi_cs_mode_type cs_mode,
                                     eTEST_APP_ARM_SPI_CSActiveLevel_t cs_active_level,
                                     eTEST_APP_ARM_SPI_CSConfirmEveryWorld_t cs_conf_every_world,
                                     uint32_t spi_delay_after_cs_usec,
                                     spi_frame_bit_num_type data_bit_num,
                                     spi_first_bit_type data_first_bit,
                                     eTEST_APP_ARM_SPI_PinDefTypes_t gpio_pin_def_type);
static uint32_t ARM_SPI_Initialize_3(spi_master_slave_mode_type mode,
                                     spi_transmission_mode_type xfer_mode,
                                     spi_mclk_freq_div_type mclk_div,
                                     eTEST_APP_ARM_SPI_ClockLatchTypes_t clk_type,
                                     spi_cs_mode_type cs_mode,
                                     eTEST_APP_ARM_SPI_CSActiveLevel_t cs_active_level,
                                     eTEST_APP_ARM_SPI_CSConfirmEveryWorld_t cs_conf_every_world,
                                     uint32_t spi_delay_after_cs_usec,
                                     spi_frame_bit_num_type data_bit_num,
                                     spi_first_bit_type data_first_bit,
                                     eTEST_APP_ARM_SPI_PinDefTypes_t gpio_pin_def_type);
static uint32_t ARM_SPI_Initialize_4(spi_master_slave_mode_type mode,
                                     spi_transmission_mode_type xfer_mode,
                                     spi_mclk_freq_div_type mclk_div,
                                     eTEST_APP_ARM_SPI_ClockLatchTypes_t clk_type,
                                     spi_cs_mode_type cs_mode,
                                     eTEST_APP_ARM_SPI_CSActiveLevel_t cs_active_level,
                                     eTEST_APP_ARM_SPI_CSConfirmEveryWorld_t cs_conf_every_world,
                                     uint32_t spi_delay_after_cs_usec,
                                     spi_frame_bit_num_type data_bit_num,
                                     spi_first_bit_type data_first_bit,
                                     eTEST_APP_ARM_SPI_PinDefTypes_t gpio_pin_def_type);

static uint32_t ARM_SPI_Uninitialize(eTEST_APP_ARM_SPI_Types_t spi_type);
static uint32_t ARM_SPI_Uninitialize_1(void);
static uint32_t ARM_SPI_Uninitialize_2(void);
static uint32_t ARM_SPI_Uninitialize_3(void);
static uint32_t ARM_SPI_Uninitialize_4(void);

static void ARM_SPI_Event_cb(eTEST_APP_ARM_SPI_Types_t spi_type);
static void ARM_SPI_Event_cb_1(void);
static void ARM_SPI_Event_cb_2(void);
static void ARM_SPI_Event_cb_3(void);
static void ARM_SPI_Event_cb_4(void);

static void ARM_SPI_SoftwareSelectSlave(eTEST_APP_ARM_SPI_Types_t spi_type);
static void ARM_SPI_SoftwareReleaseSlave(eTEST_APP_ARM_SPI_Types_t spi_type);

static uint16_t ARM_SPI_ReadData(eTEST_APP_ARM_SPI_Types_t spi_type);

static void ARM_SPI_WriteData(eTEST_APP_ARM_SPI_Types_t spi_type, void *pdata);

static uint32_t ARM_SPI_Send(eTEST_APP_ARM_SPI_Types_t spi_type,
                             void *pdata, uint32_t num);
static uint32_t ARM_SPI_Send_1(void *pdata, uint32_t num);
static uint32_t ARM_SPI_Send_2(void *pdata, uint32_t num);
static uint32_t ARM_SPI_Send_3(void *pdata, uint32_t num);
static uint32_t ARM_SPI_Send_4(void *pdata, uint32_t num);

static uint32_t ARM_SPI_Recieve(eTEST_APP_ARM_SPI_Types_t spi_type,
                                void *pdata, uint32_t num);
static uint32_t ARM_SPI_Recieve_1(void *pdata, uint32_t num);
static uint32_t ARM_SPI_Recieve_2(void *pdata, uint32_t num);
static uint32_t ARM_SPI_Recieve_3(void *pdata, uint32_t num);
static uint32_t ARM_SPI_Recieve_4(void *pdata, uint32_t num);

static TEST_APP_ARM_SPI_Transfer_t ARM_SPI_GetTransfer(eTEST_APP_ARM_SPI_Types_t spi_type);
static TEST_APP_ARM_SPI_Transfer_t ARM_SPI_GetTransfer_1(void);
static TEST_APP_ARM_SPI_Transfer_t ARM_SPI_GetTransfer_2(void);
static TEST_APP_ARM_SPI_Transfer_t ARM_SPI_GetTransfer_3(void);
static TEST_APP_ARM_SPI_Transfer_t ARM_SPI_GetTransfer_4(void);

static TEST_APP_ARM_SPI_Status_t ARM_SPI_GetStatus(eTEST_APP_ARM_SPI_Types_t spi_type);
static TEST_APP_ARM_SPI_Status_t ARM_SPI_GetStatus_1(void);
static TEST_APP_ARM_SPI_Status_t ARM_SPI_GetStatus_2(void);
static TEST_APP_ARM_SPI_Status_t ARM_SPI_GetStatus_3(void);
static TEST_APP_ARM_SPI_Status_t ARM_SPI_GetStatus_4(void);


//********************************************************************************
//Variables
//********************************************************************************

TEST_APP_ARM_SPI_Driver_t Driver_SPI1 = {
    ARM_SPI_Initialize_1,
    ARM_SPI_Uninitialize_1,
    ARM_SPI_Event_cb_1,
    ARM_SPI_Send_1,
    ARM_SPI_Recieve_1,
    ARM_SPI_GetTransfer_1,
    ARM_SPI_GetStatus_1
};

TEST_APP_ARM_SPI_Driver_t Driver_SPI2 = {
    ARM_SPI_Initialize_2,
    ARM_SPI_Uninitialize_2,
    ARM_SPI_Event_cb_2,
    ARM_SPI_Send_2,
    ARM_SPI_Recieve_2,
    ARM_SPI_GetTransfer_2,
    ARM_SPI_GetStatus_2
};
TEST_APP_ARM_SPI_Driver_t Driver_SPI3 = {
    ARM_SPI_Initialize_3,
    ARM_SPI_Uninitialize_3,
    ARM_SPI_Event_cb_3,
    ARM_SPI_Send_3,
    ARM_SPI_Recieve_3,
    ARM_SPI_GetTransfer_3,
    ARM_SPI_GetStatus_3
};
TEST_APP_ARM_SPI_Driver_t Driver_SPI4 = {
    ARM_SPI_Initialize_4,
    ARM_SPI_Uninitialize_4,
    ARM_SPI_Event_cb_4,
    ARM_SPI_Send_4,
    ARM_SPI_Recieve_4,
    ARM_SPI_GetTransfer_4,
    ARM_SPI_GetStatus_4
};

TEST_APP_ARM_SPI_Driver_t *pARM_SPI_Driver[TEST_APP_ARM_SPI_TYPES] = {
    &Driver_SPI1,
    &Driver_SPI2,
    &Driver_SPI3,
    &Driver_SPI4
};

static spi_type *pARM_SPI_Register[TEST_APP_ARM_SPI_TYPES] = {
    SPI1,
    SPI2,
    SPI3,
    SPI4
};

static IRQn_Type ARM_SPI_IrqNumber[TEST_APP_ARM_SPI_TYPES] = {
    SPI1_IRQn,
    SPI2_I2S2EXT_IRQn,
    SPI3_I2S3EXT_IRQn,
    SPI4_IRQn
};

static eTEST_APP_SOFTWARE_TIMER_TimerTypes_t ARM_SPI_TimeoutTimer[TEST_APP_ARM_SPI_TYPES][ARM_SPI_TRANSFER_TYPES] = {
    { TEST_APP_SOFTWARE_TIMER_SPI1_TIMEOUT_TX,     TEST_APP_SOFTWARE_TIMER_SPI1_TIMEOUT_RX},
    { TEST_APP_SOFTWARE_TIMER_SPI2_TIMEOUT_TX,     TEST_APP_SOFTWARE_TIMER_SPI2_TIMEOUT_RX},
    { TEST_APP_SOFTWARE_TIMER_SPI3_TIMEOUT_TX,     TEST_APP_SOFTWARE_TIMER_SPI3_TIMEOUT_RX},
    { TEST_APP_SOFTWARE_TIMER_SPI4_TIMEOUT_TX,     TEST_APP_SOFTWARE_TIMER_SPI4_TIMEOUT_RX}
};

static TEST_APP_ARM_SPI_Resources_t ARM_SPI_Resources[TEST_APP_ARM_SPI_TYPES];
static uint32_t ARM_SPI_EventBuff[TEST_APP_ARM_SPI_TYPES][ARM_SPI_EVENT_BUFF_SIZE];


static uint32_t ARM_SPI_TxBuff[TEST_APP_ARM_SPI_TYPES][ARM_SPI_TX_BUFF_SIZE];
static uint32_t ARM_SPI_RxBuff[TEST_APP_ARM_SPI_TYPES][ARM_SPI_RX_BUFF_SIZE];

static TEST_APP_ARM_SPI_GPIO_t ARM_SPI_GPIO_Def[TEST_APP_ARM_SPI_TYPES][TEST_APP_ARM_SPI_PIN_DEF_TYPES][ARM_SPI_PIN_TYPES] = {

    //SPI1
    {
        //PIN_DEF_TYPE_DEFAULT
        {
            //MISO
            {TEST_APP_ARM_GPIO_PORTA, GPIO_PINS_6},
            //MOSI
            {TEST_APP_ARM_GPIO_PORTA, GPIO_PINS_7},
            //CS
            {TEST_APP_ARM_GPIO_PORTA, GPIO_PINS_4},
            //SCLK
            {TEST_APP_ARM_GPIO_PORTA, GPIO_PINS_5}
        },

        //PIN_DEF_TYPE_REMAP1
        {
            //MISO
            {TEST_APP_ARM_GPIO_PORTB, GPIO_PINS_4},
            //MOSI
            {TEST_APP_ARM_GPIO_PORTB, GPIO_PINS_5},
            //CS
            {TEST_APP_ARM_GPIO_PORTA, GPIO_PINS_15},
            //SCLK
            {TEST_APP_ARM_GPIO_PORTB, GPIO_PINS_3}
        },

        //PIN_DEF_TYPE_REMAP2
        {
            //MISO
            {TEST_APP_ARM_GPIO_PORT_NOT_DEFINED, TEST_APP_ARM_GPIO_PIN_NOT_DEFINED},
            //MOSI
            {TEST_APP_ARM_GPIO_PORT_NOT_DEFINED, TEST_APP_ARM_GPIO_PIN_NOT_DEFINED},
            //CS
            {TEST_APP_ARM_GPIO_PORT_NOT_DEFINED, TEST_APP_ARM_GPIO_PIN_NOT_DEFINED},
            //SCLK
            {TEST_APP_ARM_GPIO_PORT_NOT_DEFINED, TEST_APP_ARM_GPIO_PIN_NOT_DEFINED}
        }
    },

//SPI2
    {
        //PIN_DEF_TYPE_DEFAULT
        {
            //MISO
            {TEST_APP_ARM_GPIO_PORTB, GPIO_PINS_14},
            //MOSI
            {TEST_APP_ARM_GPIO_PORTB, GPIO_PINS_15},
            //CS
            {TEST_APP_ARM_GPIO_PORTB, GPIO_PINS_12},
            //SCLK
            {TEST_APP_ARM_GPIO_PORTB, GPIO_PINS_13}
        },

        //PIN_DEF_TYPE_REMAP1
        {
            //MISO
            {TEST_APP_ARM_GPIO_PORT_NOT_DEFINED, TEST_APP_ARM_GPIO_PIN_NOT_DEFINED},
            //MOSI
            {TEST_APP_ARM_GPIO_PORT_NOT_DEFINED, TEST_APP_ARM_GPIO_PIN_NOT_DEFINED},
            //CS
            {TEST_APP_ARM_GPIO_PORT_NOT_DEFINED, TEST_APP_ARM_GPIO_PIN_NOT_DEFINED},
            //SCLK
            {TEST_APP_ARM_GPIO_PORT_NOT_DEFINED, TEST_APP_ARM_GPIO_PIN_NOT_DEFINED}
        },

        //PIN_DEF_TYPE_REMAP2
        {
            //MISO
            {TEST_APP_ARM_GPIO_PORT_NOT_DEFINED, TEST_APP_ARM_GPIO_PIN_NOT_DEFINED},
            //MOSI
            {TEST_APP_ARM_GPIO_PORT_NOT_DEFINED, TEST_APP_ARM_GPIO_PIN_NOT_DEFINED},
            //CS
            {TEST_APP_ARM_GPIO_PORT_NOT_DEFINED, TEST_APP_ARM_GPIO_PIN_NOT_DEFINED},
            //SCLK
            {TEST_APP_ARM_GPIO_PORT_NOT_DEFINED, TEST_APP_ARM_GPIO_PIN_NOT_DEFINED}
        }
    },

//SPI3
    {
        //PIN_DEF_TYPE_DEFAULT
        {
            //MISO
            {TEST_APP_ARM_GPIO_PORTB, GPIO_PINS_4},
            //MOSI
            {TEST_APP_ARM_GPIO_PORTB, GPIO_PINS_5},
            //CS
            {TEST_APP_ARM_GPIO_PORTA, GPIO_PINS_15},
            //SCLK
            {TEST_APP_ARM_GPIO_PORTB, GPIO_PINS_3}
        },

        //PIN_DEF_TYPE_REMAP1
        {
            //MISO
            {TEST_APP_ARM_GPIO_PORTC, GPIO_PINS_11},
            //MOSI
            {TEST_APP_ARM_GPIO_PORTC, GPIO_PINS_12},
            //CS
            {TEST_APP_ARM_GPIO_PORTA, GPIO_PINS_4},
            //SCLK
            {TEST_APP_ARM_GPIO_PORTC, GPIO_PINS_10}
        },

        //PIN_DEF_TYPE_REMAP2
        {
            //MISO
            {TEST_APP_ARM_GPIO_PORT_NOT_DEFINED, TEST_APP_ARM_GPIO_PIN_NOT_DEFINED},
            //MOSI
            {TEST_APP_ARM_GPIO_PORT_NOT_DEFINED, TEST_APP_ARM_GPIO_PIN_NOT_DEFINED},
            //CS
            {TEST_APP_ARM_GPIO_PORT_NOT_DEFINED, TEST_APP_ARM_GPIO_PIN_NOT_DEFINED},
            //SCLK
            {TEST_APP_ARM_GPIO_PORT_NOT_DEFINED, TEST_APP_ARM_GPIO_PIN_NOT_DEFINED}
        }
    },

//SPI4
    {
        //PIN_DEF_TYPE_DEFAULT
        {
            //MISO
            {TEST_APP_ARM_GPIO_PORTE, GPIO_PINS_5},
            //MOSI
            {TEST_APP_ARM_GPIO_PORTE, GPIO_PINS_6},
            //CS
            {TEST_APP_ARM_GPIO_PORTE, GPIO_PINS_4},
            //SCLK
            {TEST_APP_ARM_GPIO_PORTE, GPIO_PINS_2}
        },

        //PIN_DEF_TYPE_REMAP1
        {
            //MISO
            {TEST_APP_ARM_GPIO_PORTE, GPIO_PINS_13},
            //MOSI
            {TEST_APP_ARM_GPIO_PORTE, GPIO_PINS_14},
            //CS
            {TEST_APP_ARM_GPIO_PORTE, GPIO_PINS_12},
            //SCLK
            {TEST_APP_ARM_GPIO_PORTE, GPIO_PINS_11}
        },

        //PIN_DEF_TYPE_REMAP2
        {
            //MISO
            {TEST_APP_ARM_GPIO_PORTB, GPIO_PINS_8},
            //MOSI
            {TEST_APP_ARM_GPIO_PORTB, GPIO_PINS_9},
            //CS
            {TEST_APP_ARM_GPIO_PORTB, GPIO_PINS_6},
            //SCLK
            {TEST_APP_ARM_GPIO_PORTB, GPIO_PINS_7}
        }
    }

};

//SPI GPIO multiplex I/O (IOMUX) remapping definitions
uint32_t ARM_SPI_GPIO_IOMUX_Remap_Def[TEST_APP_ARM_SPI_TYPES][TEST_APP_ARM_SPI_PIN_DEF_REMAP_TYPES] = {
    {SPI1_GMUX_0001,                                    TEST_APP_ARM_SPI_PIN_REMAP_CONFIG_NOT_DEFINED},
    {TEST_APP_ARM_SPI_PIN_REMAP_CONFIG_NOT_DEFINED,     TEST_APP_ARM_SPI_PIN_REMAP_CONFIG_NOT_DEFINED},
    {SPI3_GMUX_0001,                                    TEST_APP_ARM_SPI_PIN_REMAP_CONFIG_NOT_DEFINED},
    {SPI4_GMUX_0001,                                    SPI4_GMUX_0010}
};


//================================================================================
//Public
//================================================================================

void TEST_APP_ARM_SPI_StartUp(void)
{
    TEST_APP_ARM_SPI_Resources_t *p_res = &ARM_SPI_Resources[TEST_APP_ARM_SPI1];
#if _TEST_APP_SPI1_ENABLED_ > 0
    (p_res[TEST_APP_ARM_SPI1]).Status.DrvStateOn = TRUE;
    (p_res[TEST_APP_ARM_SPI1]).Status.DrvStatus = TEST_APP_ARM_DRIVER_NO_ERROR;
    (p_res[TEST_APP_ARM_SPI1]).Status.DrvFlag = 0x0000;
#endif //_TEST_APP_SPI1_ENABLED_
#if _TEST_APP_SPI2_ENABLED_ > 0
    (p_res[TEST_APP_ARM_SPI2]).Status.DrvStateOn = TRUE;
    (p_res[TEST_APP_ARM_SPI2]).Status.DrvStatus = TEST_APP_ARM_DRIVER_NO_ERROR;
    (p_res[TEST_APP_ARM_SPI2]).Status.DrvFlag = 0x0000;
#endif //_TEST_APP_SPI2_ENABLED_
#if _TEST_APP_SPI3_ENABLED_ > 0
    (p_res[TEST_APP_ARM_SPI3]).Status.DrvStateOn = TRUE;
    (p_res[TEST_APP_ARM_SPI3]).Status.DrvStatus = TEST_APP_ARM_DRIVER_NO_ERROR;
    (p_res[TEST_APP_ARM_SPI3]).Status.DrvFlag = 0x0000;
#endif //_TEST_APP_SPI3_ENABLED_
#if _TEST_APP_SPI4_ENABLED_ > 0
    (p_res[TEST_APP_ARM_SPI4]).Status.DrvStateOn = TRUE;
    (p_res[TEST_APP_ARM_SPI4]).Status.DrvStatus = TEST_APP_ARM_DRIVER_NO_ERROR;
    (p_res[TEST_APP_ARM_SPI4]).Status.DrvFlag = 0x0000;
#endif //_TEST_APP_SPI4_ENABLED_
}

void TEST_APP_ARM_SPI_IRQHandler(eTEST_APP_ARM_SPI_Types_t spi_type)
{
    uint32_t event = 0;
    TEST_APP_ARM_SPI_Resources_t *p_res = &ARM_SPI_Resources[spi_type];
    if(p_res->Status.XferStatus.RxBusy == 1) {
        if(spi_i2s_flag_get(pARM_SPI_Register[spi_type], SPI_I2S_RDBF_FLAG) == SET) {
            if(p_res->Config.InitStruct.frame_bit_num == SPI_FRAME_8BIT) {
                *((uint8_t *)p_res->Transfer.pRxData + p_res->Transfer.RxCnt) =
                    ARM_SPI_ReadData(spi_type);
            } else {
                *((uint16_t *)p_res->Transfer.pRxData + p_res->Transfer.RxCnt) =
                    ARM_SPI_ReadData(spi_type);
            }
            p_res->Transfer.RxCnt++;
            if((p_res->Config.InitStruct.transmission_mode != SPI_TRANSMIT_FULL_DUPLEX) &&
               (p_res->Config.InitStruct.cs_mode_selection == SPI_CS_SOFTWARE_MODE) &&
               (p_res->Config.CSConfEveryWorldEn == TEST_APP_ARM_SPI_CS_CONFIRM_EVERY_WORLD_ENABLE)) {
                ARM_SPI_SoftwareReleaseSlave(spi_type);
//do delay
                ARM_SPI_SoftwareSelectSlave(spi_type);
            }
        }
        if(spi_i2s_flag_get(pARM_SPI_Register[spi_type], SPI_I2S_ROERR_FLAG) == SET) {
            p_res->Status.XferStatus.RxOverflow = 1;
            event |= TEST_APP_ARM_SPI_EVENT_RX_OVERFLOW;
        }
        if(spi_i2s_flag_get(pARM_SPI_Register[spi_type], SPI_MMERR_FLAG) == SET) {
            p_res->Status.XferStatus.ModeFault = 1;
            event |= TEST_APP_ARM_SPI_EVENT_MODE_FAULT;
        }
        if(p_res->Transfer.RxCnt == p_res->Transfer.RxNum) {
            spi_i2s_interrupt_enable(pARM_SPI_Register[spi_type], SPI_I2S_RDBF_INT, FALSE);
            spi_i2s_interrupt_enable(pARM_SPI_Register[spi_type], SPI_I2S_ERROR_INT, FALSE);
            p_res->Status.XferStatus.RxBusy = 0;
            event |= TEST_APP_ARM_SPI_EVENT_RX_COMPLETE;
        }
    }
    if(p_res->Status.XferStatus.TxBusy == 1) {
        if(spi_i2s_flag_get(pARM_SPI_Register[spi_type], SPI_I2S_TDBE_FLAG) == SET) {
            if((p_res->Config.InitStruct.transmission_mode == SPI_TRANSMIT_FULL_DUPLEX) &&
               (p_res->Config.InitStruct.cs_mode_selection == SPI_CS_SOFTWARE_MODE) &&
               (p_res->Config.CSConfEveryWorldEn == TEST_APP_ARM_SPI_CS_CONFIRM_EVERY_WORLD_ENABLE)) {
                ARM_SPI_SoftwareReleaseSlave(spi_type);
//do delay
                ARM_SPI_SoftwareSelectSlave(spi_type);
            }
            if(p_res->Config.InitStruct.frame_bit_num == SPI_FRAME_8BIT) {
                ARM_SPI_WriteData(spi_type, (uint8_t *)p_res->Transfer.pTxData +
                                  p_res->Transfer.TxCnt);
            } else {
                ARM_SPI_WriteData(spi_type, (uint16_t *)p_res->Transfer.pTxData +
                                  p_res->Transfer.TxCnt);
            }
        }
        p_res->Transfer.TxCnt++;
        if(spi_i2s_flag_get(pARM_SPI_Register[spi_type], SPI_MMERR_FLAG) == SET) {
            p_res->Status.XferStatus.ModeFault = 1;
            event |= TEST_APP_ARM_SPI_EVENT_MODE_FAULT;
        }
        if(p_res->Transfer.TxCnt == p_res->Transfer.TxNum) {
            while(spi_i2s_flag_get(pARM_SPI_Register[spi_type], SPI_I2S_BF_FLAG) == SET);
            spi_i2s_interrupt_enable(pARM_SPI_Register[spi_type], SPI_I2S_TDBE_INT, FALSE);
            spi_i2s_interrupt_enable(pARM_SPI_Register[spi_type], SPI_I2S_ERROR_INT, FALSE);
            p_res->Status.XferStatus.TxBusy = 0;
            event |= TEST_APP_ARM_SPI_EVENT_TX_COMPLETE;
        }
    }
    if(event) {
        TEST_APP_RingBuffer_Write(&(p_res->Event), &event);
    }
}

//================================================================================
//Private
//================================================================================

static void  ARM_SPI_SetResources(eTEST_APP_ARM_SPI_Types_t spi_type,
                                  spi_master_slave_mode_type mode,
                                  spi_transmission_mode_type xfer_mode,
                                  spi_mclk_freq_div_type mclk_div,
                                  eTEST_APP_ARM_SPI_ClockLatchTypes_t clk_type,
                                  spi_cs_mode_type cs_mode,
                                  eTEST_APP_ARM_SPI_CSActiveLevel_t cs_active_level,
                                  eTEST_APP_ARM_SPI_CSConfirmEveryWorld_t cs_conf_every_world,
                                  uint32_t spi_delay_after_cs_usec,
                                  spi_frame_bit_num_type data_bit_num,
                                  spi_first_bit_type data_first_bit,
                                  eTEST_APP_ARM_SPI_PinDefTypes_t gpio_pin_def_type)
{
    TEST_APP_ARM_SPI_Resources_t *p_res = &ARM_SPI_Resources[spi_type];
    p_res->IrqNum = ARM_SPI_IrqNumber[spi_type];
    p_res->GpioPinDefType = gpio_pin_def_type;
    p_res->Config.InitStruct.master_slave_mode = mode;
    p_res->Config.InitStruct.transmission_mode = xfer_mode;
    p_res->Config.InitStruct.mclk_freq_division = mclk_div;
    p_res->Config.InitStruct.cs_mode_selection = cs_mode;
    p_res->Config.InitStruct.frame_bit_num = data_bit_num;
    p_res->Config.InitStruct.first_bit_transmission = data_first_bit;
    p_res->Config.ClockType = clk_type;
    p_res->Config.CSActiveLevel = cs_active_level;
    p_res->Config.CSConfEveryWorldEn = cs_conf_every_world;
    p_res->Config.DelayAfterCS_usec = spi_delay_after_cs_usec;
    switch(clk_type) {
        case TEST_APP_ARM_SPI_CLOCK_POLAR_LOW_RISING_EDGE: {
            p_res->Config.InitStruct.clock_polarity = SPI_CLOCK_POLARITY_LOW;
            p_res->Config.InitStruct.clock_phase = SPI_CLOCK_PHASE_1EDGE;
        }
        break;
        case TEST_APP_ARM_SPI_CLOCK_POLAR_LOW_FALLING_EDGE: {
            p_res->Config.InitStruct.clock_polarity = SPI_CLOCK_POLARITY_LOW;
            p_res->Config.InitStruct.clock_phase = SPI_CLOCK_PHASE_2EDGE;

            break;
        }
        case TEST_APP_ARM_SPI_CLOCK_POLAR_HIGH_RISING_EDGE: {
            p_res->Config.InitStruct.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
            p_res->Config.InitStruct.clock_phase = SPI_CLOCK_PHASE_2EDGE;

            break;
        }
        case TEST_APP_ARM_SPI_CLOCK_POLAR_HIGH_FALLING_EDGE: {
            p_res->Config.InitStruct.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
            p_res->Config.InitStruct.clock_phase = SPI_CLOCK_PHASE_1EDGE;

            break;
        }
        default: {
            break;
        }
    }
    if((p_res->Config.CSConfEveryWorldEn == TEST_APP_ARM_SPI_CS_CONFIRM_EVERY_WORLD_ENABLE) &&
       (p_res->Config.InitStruct.master_slave_mode == SPI_MODE_SLAVE ||
        p_res->Config.InitStruct.cs_mode_selection == SPI_CS_HARDWARE_MODE)) {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR_PARAMETER;
        return;
    }
    TEST_APP_RingBuffer_Init(&(p_res->Event), &ARM_SPI_EventBuff[spi_type], ARM_SPI_EVENT_BUFF_SIZE);
    p_res->Transfer.pTxData = &ARM_SPI_TxBuff[spi_type];
    p_res->Transfer.pRxData = &ARM_SPI_RxBuff[spi_type];;
    p_res->Transfer.TxNum = 0;
    p_res->Transfer.RxNum = 0;
    p_res->Transfer.TxCnt = 0;
    p_res->Transfer.RxCnt = 0;
    memcpy(&p_res->Gpio[ARM_SPI_MISO_PIN], &ARM_SPI_GPIO_Def[spi_type][gpio_pin_def_type][ARM_SPI_MISO_PIN],
           ARM_SPI_PIN_TYPES * sizeof(TEST_APP_ARM_SPI_GPIO_t));

}

static uint32_t ARM_SPI_GPIO_Config(eTEST_APP_ARM_SPI_Types_t spi_type, confirm_state new_state)
{
    eARM_SPI_PinTypes_t pin_type;
    gpio_mode_type gpio_input_mode;
    TEST_APP_ARM_SPI_Resources_t *p_res = &ARM_SPI_Resources[spi_type];
    if(new_state) {
        for(pin_type = ARM_SPI_MISO_PIN; pin_type < ARM_SPI_PIN_TYPES; pin_type++) {
            if(!TEST_APP_ARM_CRM_PeriphClockEnable(TEST_APP_PERIPH_GPIO, p_res->Gpio[pin_type].Port, TRUE)) {
#ifdef _TEST_APP_DEBUG_
                LOG("SPI configuration error");
#endif//_TEST_APP_DEBUG_        
                return TEST_APP_ARM_DRIVER_ERROR;
            }
        }

        //for pin remapping - enable GPIO IOMUX clock
        switch(p_res->GpioPinDefType) {
            case TEST_APP_ARM_SPI_PIN_DEF_TYPE_DEFAULT: {
                gpio_input_mode = GPIO_MODE_INPUT;
                break;
            }
            case TEST_APP_ARM_SPI_PIN_DEF_TYPE_REMAP1:
            case TEST_APP_ARM_SPI_PIN_DEF_TYPE_REMAP2: {
                if(ARM_SPI_GPIO_IOMUX_Remap_Def[spi_type][p_res->GpioPinDefType] == TEST_APP_ARM_SPI_PIN_REMAP_CONFIG_NOT_DEFINED) {
                    p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR_PARAMETER;
                    return TEST_APP_ARM_DRIVER_ERROR_PARAMETER;
                }
                crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
                gpio_pin_remap_config(ARM_SPI_GPIO_IOMUX_Remap_Def[spi_type][p_res->GpioPinDefType], TRUE);
                gpio_input_mode = GPIO_MODE_MUX;
                break;
            }
        }
        switch(p_res->Config.InitStruct.master_slave_mode) {
            case SPI_MODE_MASTER: {

                //SCLK
                switch(p_res->Config.InitStruct.clock_polarity) {
                    case SPI_CLOCK_POLARITY_LOW: {
                        TEST_APP_ARM_GPIO_Config(p_res->Gpio[ARM_SPI_SCLK_PIN].Port,
                                                 p_res->Gpio[ARM_SPI_SCLK_PIN].Pin,
                                                 GPIO_MODE_MUX, GPIO_OUTPUT_PUSH_PULL,
                                                 GPIO_PULL_DOWN, GPIO_DRIVE_STRENGTH_STRONGER);
                    }
                    case SPI_CLOCK_POLARITY_HIGH: {
                        TEST_APP_ARM_GPIO_Config(p_res->Gpio[ARM_SPI_SCLK_PIN].Port,
                                                 p_res->Gpio[ARM_SPI_SCLK_PIN].Pin,
                                                 GPIO_MODE_MUX, GPIO_OUTPUT_PUSH_PULL,
                                                 GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
                    }
                    default: {
                        break;
                    }
                }

                //CS
                switch(p_res->Config.InitStruct.cs_mode_selection) {
                    case SPI_CS_HARDWARE_MODE: {
                        TEST_APP_ARM_GPIO_Config(p_res->Gpio[ARM_SPI_CS_PIN].Port,
                                                 p_res->Gpio[ARM_SPI_CS_PIN].Pin,
                                                 GPIO_MODE_MUX, GPIO_OUTPUT_PUSH_PULL,
                                                 GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
                    }
                    case SPI_CS_SOFTWARE_MODE: {
                        if(p_res->Config.CSActiveLevel == TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_LOW) {
                            TEST_APP_ARM_GPIO_Config(p_res->Gpio[ARM_SPI_CS_PIN].Port,
                                                     p_res->Gpio[ARM_SPI_CS_PIN].Pin,
                                                     GPIO_MODE_OUTPUT, GPIO_OUTPUT_PUSH_PULL,
                                                     GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
                        } else {
                            TEST_APP_ARM_GPIO_Config(p_res->Gpio[ARM_SPI_CS_PIN].Port,
                                                     p_res->Gpio[ARM_SPI_CS_PIN].Pin,
                                                     GPIO_MODE_OUTPUT, GPIO_OUTPUT_PUSH_PULL,
                                                     GPIO_PULL_DOWN, GPIO_DRIVE_STRENGTH_STRONGER);
                        }
                    }
                    default: {
                        break;
                    }
                }

                //MOSI, MISO
                switch(p_res->Config.InitStruct.transmission_mode) {
                    case SPI_TRANSMIT_FULL_DUPLEX:
                    case SPI_TRANSMIT_FULL_DUPLEX_TX_ONLY: {
                        TEST_APP_ARM_GPIO_Config(p_res->Gpio[ARM_SPI_MOSI_PIN].Port,
                                                 p_res->Gpio[ARM_SPI_MOSI_PIN].Pin,
                                                 GPIO_MODE_MUX, GPIO_OUTPUT_PUSH_PULL,
                                                 GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
                        TEST_APP_ARM_GPIO_Config(p_res->Gpio[ARM_SPI_MISO_PIN].Port,
                                                 p_res->Gpio[ARM_SPI_MISO_PIN].Pin,
                                                 GPIO_MODE_MUX, GPIO_OUTPUT_PUSH_PULL,
                                                 GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
                        break;
                    }
                    case SPI_TRANSMIT_SIMPLEX_RX: {
                        TEST_APP_ARM_GPIO_Config(p_res->Gpio[ARM_SPI_MISO_PIN].Port,
                                                 p_res->Gpio[ARM_SPI_MISO_PIN].Pin,
                                                 GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                                                 GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
                        break;
                    }
                    case SPI_TRANSMIT_HALF_DUPLEX_RX:
                    case SPI_TRANSMIT_HALF_DUPLEX_TX: {
                        TEST_APP_ARM_GPIO_Config(p_res->Gpio[ARM_SPI_MOSI_PIN].Port,
                                                 p_res->Gpio[ARM_SPI_MOSI_PIN].Pin,
                                                 GPIO_MODE_MUX, GPIO_OUTPUT_PUSH_PULL,
                                                 GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
                        break;
                    }
                    default: {
                        break;
                    }
                }
            }
            case SPI_MODE_SLAVE: {
                //SCLK
                switch(p_res->Config.InitStruct.clock_polarity) {
                    case SPI_CLOCK_POLARITY_LOW: {
                        TEST_APP_ARM_GPIO_Config(p_res->Gpio[ARM_SPI_SCLK_PIN].Port,
                                                 p_res->Gpio[ARM_SPI_SCLK_PIN].Pin,
                                                 gpio_input_mode, GPIO_OUTPUT_PUSH_PULL,
                                                 GPIO_PULL_DOWN, GPIO_DRIVE_STRENGTH_STRONGER);
                    }
                    case SPI_CLOCK_POLARITY_HIGH: {
                        TEST_APP_ARM_GPIO_Config(p_res->Gpio[ARM_SPI_SCLK_PIN].Port,
                                                 p_res->Gpio[ARM_SPI_SCLK_PIN].Pin,
                                                 gpio_input_mode, GPIO_OUTPUT_PUSH_PULL,
                                                 GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
                    }
                    default: {
                        break;
                    }
                }

                //CS
                switch(p_res->Config.InitStruct.cs_mode_selection) {
                    case SPI_CS_HARDWARE_MODE: {
                        TEST_APP_ARM_GPIO_Config(p_res->Gpio[ARM_SPI_CS_PIN].Port,
                                                 p_res->Gpio[ARM_SPI_CS_PIN].Pin,
                                                 gpio_input_mode, GPIO_OUTPUT_PUSH_PULL,
                                                 GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
                    }
                    case SPI_CS_SOFTWARE_MODE: {
                    }
                    default: {
                        break;
                    }
                }

                //MOSI, MISO
                switch(p_res->Config.InitStruct.transmission_mode) {
                    case SPI_TRANSMIT_FULL_DUPLEX:
                    case SPI_TRANSMIT_FULL_DUPLEX_TX_ONLY: {
                        TEST_APP_ARM_GPIO_Config(p_res->Gpio[ARM_SPI_MOSI_PIN].Port,
                                                 p_res->Gpio[ARM_SPI_MOSI_PIN].Pin,
                                                 gpio_input_mode, GPIO_OUTPUT_PUSH_PULL,
                                                 GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
                        TEST_APP_ARM_GPIO_Config(p_res->Gpio[ARM_SPI_MISO_PIN].Port,
                                                 p_res->Gpio[ARM_SPI_MISO_PIN].Pin,
                                                 GPIO_MODE_MUX, GPIO_OUTPUT_PUSH_PULL,
                                                 GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
                        break;
                    }
                    case SPI_TRANSMIT_SIMPLEX_RX: {
                        TEST_APP_ARM_GPIO_Config(p_res->Gpio[ARM_SPI_MOSI_PIN].Port,
                                                 p_res->Gpio[ARM_SPI_MOSI_PIN].Pin,
                                                 gpio_input_mode, GPIO_OUTPUT_PUSH_PULL,
                                                 GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
                        break;
                    }
                    case SPI_TRANSMIT_HALF_DUPLEX_RX:
                    case SPI_TRANSMIT_HALF_DUPLEX_TX: {
                        TEST_APP_ARM_GPIO_Config(p_res->Gpio[ARM_SPI_MISO_PIN].Port,
                                                 p_res->Gpio[ARM_SPI_MISO_PIN].Pin,
                                                 GPIO_MODE_MUX, GPIO_OUTPUT_PUSH_PULL,
                                                 GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
                        break;
                    }
                    default: {
                        break;
                    }
                }
            }
            default: {
                break;
            }
        }
    }

    else {
        //release pins

        //SCLK
        TEST_APP_ARM_GPIO_ReleaseBits(p_res->Gpio[ARM_SPI_SCLK_PIN].Port,
                                      p_res->Gpio[ARM_SPI_SCLK_PIN].Pin);
        switch(p_res->Config.InitStruct.master_slave_mode) {
            case SPI_MODE_MASTER: {

                //CS
                switch(p_res->Config.InitStruct.cs_mode_selection) {
                    case SPI_CS_HARDWARE_MODE:
                    case SPI_CS_SOFTWARE_MODE: {
                        TEST_APP_ARM_GPIO_ReleaseBits(p_res->Gpio[ARM_SPI_CS_PIN].Port,
                                                      p_res->Gpio[ARM_SPI_CS_PIN].Pin);
                    }
                    default: {
                        break;
                    }
                }

                //MOSI, MISO
                switch(p_res->Config.InitStruct.transmission_mode) {
                    case SPI_TRANSMIT_FULL_DUPLEX:
                    case SPI_TRANSMIT_FULL_DUPLEX_TX_ONLY: {
                        TEST_APP_ARM_GPIO_ReleaseBits(p_res->Gpio[ARM_SPI_MOSI_PIN].Port,
                                                      p_res->Gpio[ARM_SPI_MOSI_PIN].Pin);
                        TEST_APP_ARM_GPIO_ReleaseBits(p_res->Gpio[ARM_SPI_MISO_PIN].Port,
                                                      p_res->Gpio[ARM_SPI_MISO_PIN].Pin);
                        break;
                    }
                    case SPI_TRANSMIT_SIMPLEX_RX: {
                        TEST_APP_ARM_GPIO_ReleaseBits(p_res->Gpio[ARM_SPI_MISO_PIN].Port,
                                                      p_res->Gpio[ARM_SPI_MISO_PIN].Pin);
                        break;
                    }
                    case SPI_TRANSMIT_HALF_DUPLEX_RX:
                    case SPI_TRANSMIT_HALF_DUPLEX_TX: {
                        TEST_APP_ARM_GPIO_ReleaseBits(p_res->Gpio[ARM_SPI_MOSI_PIN].Port,
                                                      p_res->Gpio[ARM_SPI_MOSI_PIN].Pin);
                        break;
                    }
                    default: {
                        return TEST_APP_ARM_DRIVER_ERROR;
                    }
                }
            }
            case SPI_MODE_SLAVE: {

                //CS
                switch(p_res->Config.InitStruct.cs_mode_selection) {
                    case SPI_CS_HARDWARE_MODE: {
                        TEST_APP_ARM_GPIO_ReleaseBits(p_res->Gpio[ARM_SPI_CS_PIN].Port,
                                                      p_res->Gpio[ARM_SPI_CS_PIN].Pin);
                        case SPI_CS_SOFTWARE_MODE: {
                        }
                        default: {
                            break;
                        }
                    }
                }

                //MOSI, MISO
                switch(p_res->Config.InitStruct.transmission_mode) {
                    case SPI_TRANSMIT_FULL_DUPLEX:
                    case SPI_TRANSMIT_FULL_DUPLEX_TX_ONLY: {
                        TEST_APP_ARM_GPIO_ReleaseBits(p_res->Gpio[ARM_SPI_MOSI_PIN].Port,
                                                      p_res->Gpio[ARM_SPI_MOSI_PIN].Pin);
                        TEST_APP_ARM_GPIO_ReleaseBits(p_res->Gpio[ARM_SPI_MISO_PIN].Port,
                                                      p_res->Gpio[ARM_SPI_MISO_PIN].Pin);
                        break;
                    }
                    case SPI_TRANSMIT_SIMPLEX_RX: {
                        TEST_APP_ARM_GPIO_ReleaseBits(p_res->Gpio[ARM_SPI_MOSI_PIN].Port,
                                                      p_res->Gpio[ARM_SPI_MOSI_PIN].Pin);
                        break;
                    }
                    case SPI_TRANSMIT_HALF_DUPLEX_RX:
                    case SPI_TRANSMIT_HALF_DUPLEX_TX: {
                        TEST_APP_ARM_GPIO_ReleaseBits(p_res->Gpio[ARM_SPI_MISO_PIN].Port,
                                                      p_res->Gpio[ARM_SPI_MISO_PIN].Pin);
                        break;
                    }
                    default: {
                        return TEST_APP_ARM_DRIVER_ERROR;
                    }
                }
            }
            default: {
                break;
            }
        }
    }
    //disable remap (for pins remapping release)
    if(!(p_res->GpioPinDefType == TEST_APP_ARM_SPI_PIN_DEF_TYPE_DEFAULT)) {
        gpio_pin_remap_config(ARM_SPI_GPIO_IOMUX_Remap_Def[spi_type][p_res->GpioPinDefType], FALSE);
    }

    return TEST_APP_ARM_DRIVER_NO_ERROR;

}

static uint32_t ARM_SPI_Initialize(eTEST_APP_ARM_SPI_Types_t spi_type,
                                   spi_master_slave_mode_type mode,
                                   spi_transmission_mode_type xfer_mode,
                                   spi_mclk_freq_div_type mclk_div,
                                   eTEST_APP_ARM_SPI_ClockLatchTypes_t clk_type,
                                   spi_cs_mode_type cs_mode,
                                   eTEST_APP_ARM_SPI_CSActiveLevel_t cs_active_level,
                                   eTEST_APP_ARM_SPI_CSConfirmEveryWorld_t cs_conf_every_world,
                                   uint32_t spi_delay_after_cs_usec,
                                   spi_frame_bit_num_type data_bit_num,
                                   spi_first_bit_type data_first_bit,
                                   eTEST_APP_ARM_SPI_PinDefTypes_t gpio_pin_def_type)
{
    TEST_APP_ARM_SPI_Resources_t *p_res = &ARM_SPI_Resources[spi_type];
    uint32_t drv_status = TEST_APP_ARM_DRIVER_NO_ERROR;
    ARM_SPI_SetResources(spi_type, mode, xfer_mode, mclk_div, clk_type,
                         cs_mode, cs_active_level, cs_conf_every_world,
                         spi_delay_after_cs_usec,
                         data_bit_num, data_first_bit, gpio_pin_def_type);
    if(!(TEST_APP_ARM_CRM_PeriphClockEnable(TEST_APP_PERIPH_SPI, spi_type, TRUE))) {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
        return TEST_APP_ARM_DRIVER_ERROR;
    }
    TEST_APP_ARM_CRM_PeriphReset(TEST_APP_PERIPH_SPI, spi_type, TRUE);
    TEST_APP_ARM_CRM_PeriphReset(TEST_APP_PERIPH_SPI, spi_type, FALSE);
    drv_status |= ARM_SPI_GPIO_Config(spi_type, TRUE);
    spi_init(pARM_SPI_Register[spi_type], &p_res->Config.InitStruct);
    p_res->Status.DrvFlag |= TEST_APP_ARM_SPI_DRIVER_FLAG_CONFIGURATED;
//clear and enable SPIx IRQ
    NVIC_ClearPendingIRQ(p_res->IrqNum);
    NVIC_EnableIRQ(p_res->IrqNum);
    p_res->Status.DrvFlag |= TEST_APP_ARM_SPI_DRIVER_FLAG_INITIALIZED;
    p_res->Status.DrvStatus |= drv_status;
    spi_enable(pARM_SPI_Register[spi_type], TRUE);
    p_res->Status.DrvFlag |= TEST_APP_ARM_SPI_DRIVER_FLAG_ENABLED;
    return drv_status;
}

static uint32_t ARM_SPI_Uninitialize(eTEST_APP_ARM_SPI_Types_t spi_type)
{
    TEST_APP_ARM_SPI_Resources_t *p_res = &ARM_SPI_Resources[spi_type];
    uint32_t drv_status = TEST_APP_ARM_DRIVER_NO_ERROR;
    if(p_res->Status.DrvFlag & TEST_APP_ARM_SPI_DRIVER_FLAG_INITIALIZED) {
        spi_enable(pARM_SPI_Register[spi_type], FALSE);
        //disable and clear SPIx IRQ
        NVIC_DisableIRQ(p_res->IrqNum);
        NVIC_ClearPendingIRQ(p_res->IrqNum);
        drv_status |= ARM_SPI_GPIO_Config(spi_type, FALSE);
        TEST_APP_ARM_CRM_PeriphReset(TEST_APP_PERIPH_SPI, spi_type, TRUE);
        p_res->Status.DrvFlag &= ~TEST_APP_ARM_SPI_DRIVER_FLAG_CONFIGURATED;
        if(!(TEST_APP_ARM_CRM_PeriphClockEnable(TEST_APP_PERIPH_SPI, spi_type, FALSE))) {
            p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
            return TEST_APP_ARM_DRIVER_ERROR;
        }
        memset(p_res, 0, sizeof(TEST_APP_ARM_SPI_Resources_t));
        p_res->Status.DrvFlag &= ~TEST_APP_ARM_SPI_DRIVER_FLAG_INITIALIZED;
    } else {
#ifdef _TEST_APP_DEBUG_
        LOG("SPI not been initialized");
#endif//_TEST_APP_DEBUG_
    }
    p_res->Status.DrvStatus |= drv_status;
    return drv_status;
}

static void ARM_SPI_Event_cb(eTEST_APP_ARM_SPI_Types_t spi_type)
{
    TEST_APP_ARM_SPI_Resources_t *p_res = &ARM_SPI_Resources[spi_type];
    uint32_t event = 0;
    TEST_APP_RingBuffer_Read(&(p_res->Event), &event);
    if(event & TEST_APP_ARM_SPI_EVENT_RX_COMPLETE) {
        switch(p_res->Config.InitStruct.transmission_mode) {
            case SPI_TRANSMIT_SIMPLEX_RX:
            case SPI_TRANSMIT_HALF_DUPLEX_RX:  {
                if(p_res->Config.InitStruct.cs_mode_selection == SPI_CS_SOFTWARE_MODE) {
                    ARM_SPI_SoftwareReleaseSlave(spi_type);
                }
                break;
            }
            default: {
                break;
            }
        }
        p_res->Transfer.RxCnt = 0;
        p_res->Transfer.RxNum = 0;
#ifdef _TEST_APP_DEBUG_
        TEST_APP_ARM_SPI_Driver_t *p_drv = pARM_SPI_Driver[TEST_APP_ARM_SPI1];
        p_drv[spi_type].Send(p_res->Transfer.pRxData, 8);
        TEST_APP_LCD2004_Printf(0, 0, SET, "%s", p_res->Transfer.pRxData);
        LOG("SPI recieved test data");
#endif//_TEST_APP_DEBUG_ 
    }
    if(event & TEST_APP_ARM_SPI_EVENT_TX_COMPLETE) {
        switch(p_res->Config.InitStruct.transmission_mode) {
            case SPI_TRANSMIT_FULL_DUPLEX:
            case SPI_TRANSMIT_FULL_DUPLEX_TX_ONLY:
            case SPI_TRANSMIT_HALF_DUPLEX_TX: {
                if(p_res->Config.InitStruct.cs_mode_selection == SPI_CS_SOFTWARE_MODE) {
                    ARM_SPI_SoftwareReleaseSlave(spi_type);
                }
                break;
            }
            default: {
                break;
            }
        }
        p_res->Transfer.TxCnt = 0;
        p_res->Transfer.TxNum = 0;
#ifdef _TEST_APP_DEBUG_
        LOG("SPI transmitted test data");
#endif//_TEST_APP_DEBUG_                 
    }
    if(event & TEST_APP_ARM_SPI_EVENT_RX_OVERFLOW) {
        if(p_res->Config.InitStruct.transmission_mode != SPI_TRANSMIT_FULL_DUPLEX_TX_ONLY) {
#ifdef _TEST_APP_DEBUG_
            LOG("SPI receiver overflow");
#endif//_TEST_APP_DEBUG_                
        }
    }
    if(event & TEST_APP_ARM_SPI_EVENT_MODE_FAULT) {
#ifdef _TEST_APP_DEBUG_
        LOG("SPI master mode error");
#endif//_TEST_APP_DEBUG_                
    }
}

static void ARM_SPI_SoftwareSelectSlave(eTEST_APP_ARM_SPI_Types_t spi_type)
{
    TEST_APP_ARM_SPI_Resources_t *p_res = &ARM_SPI_Resources[spi_type];
    if(p_res->Config.InitStruct.master_slave_mode == SPI_MODE_MASTER) {
        if(p_res->Config.CSActiveLevel == TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_LOW) {
            TEST_APP_ARM_GPIO_ResetBits(p_res->Gpio[ARM_SPI_CS_PIN].Port, p_res->Gpio[ARM_SPI_CS_PIN].Pin);
        } else {
            TEST_APP_ARM_GPIO_SetBits(p_res->Gpio[ARM_SPI_CS_PIN].Port, p_res->Gpio[ARM_SPI_CS_PIN].Pin);
        }
        //slave mode
    } else {
        if(p_res->Config.CSActiveLevel == TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_LOW) {
            spi_software_cs_internal_level_set(pARM_SPI_Register[spi_type], SPI_SWCS_INTERNAL_LEVEL_LOW);
        } else {
            spi_software_cs_internal_level_set(pARM_SPI_Register[spi_type], SPI_SWCS_INTERNAL_LEVEL_HIGHT);
        }
    }
}

static void ARM_SPI_SoftwareReleaseSlave(eTEST_APP_ARM_SPI_Types_t spi_type)
{
    TEST_APP_ARM_SPI_Resources_t *p_res = &ARM_SPI_Resources[spi_type];
    if(p_res->Config.InitStruct.master_slave_mode == SPI_MODE_MASTER) {
        if(p_res->Config.CSActiveLevel == TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_LOW) {
            TEST_APP_ARM_GPIO_SetBits(p_res->Gpio[ARM_SPI_CS_PIN].Port, p_res->Gpio[ARM_SPI_CS_PIN].Pin);
        } else {
            TEST_APP_ARM_GPIO_ResetBits(p_res->Gpio[ARM_SPI_CS_PIN].Port, p_res->Gpio[ARM_SPI_CS_PIN].Pin);
        }
        //slave mode
    } else {
        if(p_res->Config.CSActiveLevel == TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_LOW) {
            spi_software_cs_internal_level_set(pARM_SPI_Register[spi_type], SPI_SWCS_INTERNAL_LEVEL_HIGHT);
        } else {
            spi_software_cs_internal_level_set(pARM_SPI_Register[spi_type], SPI_SWCS_INTERNAL_LEVEL_LOW);
        }
    }
}

static uint32_t ARM_SPI_Recieve(eTEST_APP_ARM_SPI_Types_t spi_type, void *pdata, uint32_t num)
{
    TEST_APP_ARM_SPI_Resources_t *p_res = &ARM_SPI_Resources[spi_type];
    if(!(p_res->Status.DrvFlag & TEST_APP_ARM_SPI_DRIVER_FLAG_ENABLED)) {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
        return TEST_APP_ARM_DRIVER_ERROR;
    }
    if(num == 0) {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR_PARAMETER;
        return TEST_APP_ARM_DRIVER_ERROR_PARAMETER;
    }
    if(p_res->Status.XferStatus.RxBusy) {
        TEST_APP_SOFTWARE_TIMER_TimerEnable(ARM_SPI_TimeoutTimer[spi_type][ARM_SPI_TRANSFER_RX], ARM_SPI_TIMEOUT_MSEC);
        while(p_res->Status.XferStatus.RxBusy && (!TEST_APP_SOFTWARE_TIMER_TimerTestFlag(ARM_SPI_TimeoutTimer[spi_type][ARM_SPI_TRANSFER_RX])));
        if(TEST_APP_SOFTWARE_TIMER_TimerTestFlag(ARM_SPI_TimeoutTimer[spi_type][ARM_SPI_TRANSFER_RX])) {
            p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR_BUSY;
            return TEST_APP_ARM_DRIVER_ERROR_BUSY;
        } else {
            p_res->Status.DrvStatus &= ~TEST_APP_ARM_DRIVER_ERROR_BUSY;
        }
        TEST_APP_SOFTWARE_TIMER_TimerDisable(ARM_SPI_TimeoutTimer[spi_type][ARM_SPI_TRANSFER_RX]);
    } else {
        p_res->Status.DrvStatus &= ~TEST_APP_ARM_DRIVER_ERROR_BUSY;
    }
    if(p_res->Config.InitStruct.cs_mode_selection == SPI_CS_SOFTWARE_MODE) {
        ARM_SPI_SoftwareSelectSlave(spi_type);
    }
    if(p_res->Config.DelayAfterCS_usec != 0) {
        TEST_APP_SYSTICK_TIMER_DoDelay_ms(1);
    }
    switch(p_res->Config.InitStruct.transmission_mode) {
        case SPI_TRANSMIT_SIMPLEX_RX:
        case SPI_TRANSMIT_HALF_DUPLEX_RX: {
            p_res->Status.XferStatus.RxBusy = 1;
            spi_i2s_interrupt_enable(pARM_SPI_Register[spi_type], SPI_I2S_ERROR_INT, TRUE);
            spi_i2s_interrupt_enable(pARM_SPI_Register[spi_type], SPI_I2S_RDBF_INT, TRUE);
            break;
        }
        default: {
            break;
        }
    }
    return TEST_APP_ARM_DRIVER_NO_ERROR;
}

static uint32_t ARM_SPI_Send(eTEST_APP_ARM_SPI_Types_t spi_type, void *pdata, uint32_t num)
{
    TEST_APP_ARM_SPI_Resources_t *p_res = &ARM_SPI_Resources[spi_type];
    if(!(p_res->Status.DrvFlag & TEST_APP_ARM_SPI_DRIVER_FLAG_ENABLED)) {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR;
        return TEST_APP_ARM_DRIVER_ERROR;
    }
    if(num == 0) {
        p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR_PARAMETER;
        return TEST_APP_ARM_DRIVER_ERROR_PARAMETER;
    }
    if(p_res->Status.XferStatus.TxBusy) {
        TEST_APP_SOFTWARE_TIMER_TimerEnable(ARM_SPI_TimeoutTimer[spi_type][ARM_SPI_TRANSFER_TX], ARM_SPI_TIMEOUT_MSEC);
        while(p_res->Status.XferStatus.TxBusy && (!TEST_APP_SOFTWARE_TIMER_TimerTestFlag(ARM_SPI_TimeoutTimer[spi_type][ARM_SPI_TRANSFER_TX])));
        if(TEST_APP_SOFTWARE_TIMER_TimerTestFlag(ARM_SPI_TimeoutTimer[spi_type][ARM_SPI_TRANSFER_TX])) {
            p_res->Status.DrvStatus |= TEST_APP_ARM_DRIVER_ERROR_BUSY;
            return TEST_APP_ARM_DRIVER_ERROR_BUSY;
        } else {
            p_res->Status.DrvStatus &= ~TEST_APP_ARM_DRIVER_ERROR_BUSY;
        }
        TEST_APP_SOFTWARE_TIMER_TimerDisable(ARM_SPI_TimeoutTimer[spi_type][ARM_SPI_TRANSFER_TX]);
    } else {
        p_res->Status.DrvStatus &= ~TEST_APP_ARM_DRIVER_ERROR_BUSY;
    }
    if(p_res->Config.InitStruct.cs_mode_selection == SPI_CS_SOFTWARE_MODE) {
        ARM_SPI_SoftwareSelectSlave(spi_type);
    }
    switch(p_res->Config.InitStruct.transmission_mode) {
        case SPI_TRANSMIT_FULL_DUPLEX: {
            p_res->Status.XferStatus.TxBusy = 1;
            p_res->Status.XferStatus.RxBusy = 1;
            spi_i2s_interrupt_enable(pARM_SPI_Register[spi_type], SPI_I2S_ERROR_INT, TRUE);
            spi_i2s_interrupt_enable(pARM_SPI_Register[spi_type], SPI_I2S_TDBE_INT | SPI_I2S_RDBF_INT, TRUE);
            break;
        }
        case SPI_TRANSMIT_FULL_DUPLEX_TX_ONLY: {
            p_res->Status.XferStatus.TxBusy = 1;
            // in SPI_TRANSMIT_FULL_DUPLEX_TX_ONLY after sending two data worlds the ROERR flag  is set
            spi_i2s_interrupt_enable(pARM_SPI_Register[spi_type], SPI_I2S_TDBE_INT, TRUE);
            break;
        }
        case SPI_TRANSMIT_HALF_DUPLEX_TX: {
            p_res->Status.XferStatus.TxBusy = 1;
            spi_i2s_interrupt_enable(pARM_SPI_Register[spi_type], SPI_I2S_ERROR_INT, TRUE);
            spi_i2s_interrupt_enable(pARM_SPI_Register[spi_type], SPI_I2S_TDBE_INT, TRUE);
            break;
        }

        default: {
            break;
        }
    }
    return TEST_APP_ARM_DRIVER_NO_ERROR;
}

static void ARM_SPI_WriteData(eTEST_APP_ARM_SPI_Types_t spi_type, void *pdata)
{
    uint16_t data;
    TEST_APP_ARM_SPI_Resources_t *p_res = &ARM_SPI_Resources[spi_type];
    if(p_res->Config.InitStruct.frame_bit_num == SPI_FRAME_8BIT) {
        data = *((uint8_t *)pdata);
        data &= 0x00FF;
    } else {
        data = *(uint16_t *)pdata;
    }
    pARM_SPI_Register[spi_type]->dt = data;
}

static uint16_t ARM_SPI_ReadData(eTEST_APP_ARM_SPI_Types_t spi_type)
{
    return (uint16_t)pARM_SPI_Register[spi_type]->dt;
}

static TEST_APP_ARM_SPI_Transfer_t ARM_SPI_GetTransfer(eTEST_APP_ARM_SPI_Types_t spi_type)
{
    TEST_APP_ARM_SPI_Resources_t *p_res = &ARM_SPI_Resources[spi_type];
    TEST_APP_ARM_SPI_Transfer_t transfer;
    transfer.pTxData = p_res->Transfer.pTxData;
    transfer.pRxData = p_res->Transfer.pRxData;
    transfer.TxNum = p_res->Transfer.TxNum;
    transfer.RxNum = p_res->Transfer.RxNum;
    transfer.TxCnt = p_res->Transfer.TxCnt;
    transfer.RxCnt = p_res->Transfer.RxCnt;
    return transfer;
}

static TEST_APP_ARM_SPI_Status_t ARM_SPI_GetStatus(eTEST_APP_ARM_SPI_Types_t spi_type)
{
    TEST_APP_ARM_SPI_Resources_t *p_res = &ARM_SPI_Resources[spi_type];
    TEST_APP_ARM_SPI_Status_t status;
    status.DrvStateOn = p_res->Status.DrvStateOn;
    status.DrvFlag = p_res->Status.DrvFlag;
    status.DrvStatus = p_res->Status.DrvStatus;
    status.XferStatus.TxBusy = p_res->Status.XferStatus.TxBusy;
    status.XferStatus.RxBusy = p_res->Status.XferStatus.RxBusy;
    status.XferStatus.RxOverflow = p_res->Status.XferStatus.RxOverflow;
    status.XferStatus.ModeFault = p_res->Status.XferStatus.ModeFault;
    return status;
}


static uint32_t ARM_SPI_Initialize_1(spi_master_slave_mode_type mode,
                                     spi_transmission_mode_type xfer_mode,
                                     spi_mclk_freq_div_type mclk_div,
                                     eTEST_APP_ARM_SPI_ClockLatchTypes_t clk_type,
                                     spi_cs_mode_type cs_mode,
                                     eTEST_APP_ARM_SPI_CSActiveLevel_t cs_active_level,
                                     eTEST_APP_ARM_SPI_CSConfirmEveryWorld_t cs_conf_every_world,
                                     uint32_t spi_delay_after_cs_usec,
                                     spi_frame_bit_num_type data_bit_num,
                                     spi_first_bit_type data_first_bit,
                                     eTEST_APP_ARM_SPI_PinDefTypes_t gpio_pin_def_type)
{
    return ARM_SPI_Initialize(TEST_APP_ARM_SPI1, mode, xfer_mode, mclk_div, clk_type, cs_mode,
                              cs_active_level, cs_conf_every_world,
                              spi_delay_after_cs_usec,
                              data_bit_num, data_first_bit, gpio_pin_def_type);
}


static uint32_t ARM_SPI_Initialize_2(spi_master_slave_mode_type mode,
                                     spi_transmission_mode_type xfer_mode,
                                     spi_mclk_freq_div_type mclk_div,
                                     eTEST_APP_ARM_SPI_ClockLatchTypes_t clk_type,
                                     spi_cs_mode_type cs_mode,
                                     eTEST_APP_ARM_SPI_CSActiveLevel_t cs_active_level,
                                     eTEST_APP_ARM_SPI_CSConfirmEveryWorld_t cs_conf_every_world,
                                     uint32_t spi_delay_after_cs_usec,
                                     spi_frame_bit_num_type data_bit_num,
                                     spi_first_bit_type data_first_bit,
                                     eTEST_APP_ARM_SPI_PinDefTypes_t gpio_pin_def_type)
{
    return ARM_SPI_Initialize(TEST_APP_ARM_SPI2, mode, xfer_mode, mclk_div, clk_type, cs_mode,
                              cs_active_level, cs_conf_every_world,
                              spi_delay_after_cs_usec,
                              data_bit_num, data_first_bit, gpio_pin_def_type);
}

static uint32_t ARM_SPI_Initialize_3(spi_master_slave_mode_type mode,
                                     spi_transmission_mode_type xfer_mode,
                                     spi_mclk_freq_div_type mclk_div,
                                     eTEST_APP_ARM_SPI_ClockLatchTypes_t clk_type,
                                     spi_cs_mode_type cs_mode,
                                     eTEST_APP_ARM_SPI_CSActiveLevel_t cs_active_level,
                                     eTEST_APP_ARM_SPI_CSConfirmEveryWorld_t cs_conf_every_world,
                                     uint32_t spi_delay_after_cs_usec,
                                     spi_frame_bit_num_type data_bit_num,
                                     spi_first_bit_type data_first_bit,
                                     eTEST_APP_ARM_SPI_PinDefTypes_t gpio_pin_def_type)
{
    return ARM_SPI_Initialize(TEST_APP_ARM_SPI3, mode, xfer_mode, mclk_div, clk_type, cs_mode,
                              cs_active_level, cs_conf_every_world,
                              spi_delay_after_cs_usec,
                              data_bit_num, data_first_bit, gpio_pin_def_type);
}

static uint32_t ARM_SPI_Initialize_4(spi_master_slave_mode_type mode,
                                     spi_transmission_mode_type xfer_mode,
                                     spi_mclk_freq_div_type mclk_div,
                                     eTEST_APP_ARM_SPI_ClockLatchTypes_t clk_type,
                                     spi_cs_mode_type cs_mode,
                                     eTEST_APP_ARM_SPI_CSActiveLevel_t cs_active_level,
                                     eTEST_APP_ARM_SPI_CSConfirmEveryWorld_t cs_conf_every_world,
                                     uint32_t spi_delay_after_cs_usec,
                                     spi_frame_bit_num_type data_bit_num,
                                     spi_first_bit_type data_first_bit,
                                     eTEST_APP_ARM_SPI_PinDefTypes_t gpio_pin_def_type)
{
    return ARM_SPI_Initialize(TEST_APP_ARM_SPI4, mode, xfer_mode, mclk_div, clk_type, cs_mode,
                              cs_active_level, cs_conf_every_world,
                              spi_delay_after_cs_usec,
                              data_bit_num, data_first_bit, gpio_pin_def_type);
}

static uint32_t ARM_SPI_Uninitialize_1(void)
{
    return ARM_SPI_Uninitialize(TEST_APP_ARM_SPI1);
}

static uint32_t ARM_SPI_Uninitialize_2(void)
{
    return ARM_SPI_Uninitialize(TEST_APP_ARM_SPI2);
}

static uint32_t ARM_SPI_Uninitialize_3(void)
{
    return ARM_SPI_Uninitialize(TEST_APP_ARM_SPI3);
}

static uint32_t ARM_SPI_Uninitialize_4(void)
{
    return ARM_SPI_Uninitialize(TEST_APP_ARM_SPI4);
}

static void ARM_SPI_Event_cb_1(void)
{
    ARM_SPI_Event_cb(TEST_APP_ARM_SPI1);
}
static void ARM_SPI_Event_cb_2(void)
{
    ARM_SPI_Event_cb(TEST_APP_ARM_SPI2);
}
static void ARM_SPI_Event_cb_3(void)
{
    ARM_SPI_Event_cb(TEST_APP_ARM_SPI3);
}
static void ARM_SPI_Event_cb_4(void)
{
    ARM_SPI_Event_cb(TEST_APP_ARM_SPI4);
}

static uint32_t ARM_SPI_Send_1(void *pdata, uint32_t num)
{
    return ARM_SPI_Send(TEST_APP_ARM_SPI1, pdata, num);
}
static uint32_t ARM_SPI_Send_2(void *pdata, uint32_t num)
{
    return ARM_SPI_Send(TEST_APP_ARM_SPI2, pdata, num);
}
static uint32_t ARM_SPI_Send_3(void *pdata, uint32_t num)
{
    return ARM_SPI_Send(TEST_APP_ARM_SPI3, pdata, num);
}
static uint32_t ARM_SPI_Send_4(void *pdata, uint32_t num)
{
    return ARM_SPI_Send(TEST_APP_ARM_SPI4, pdata, num);
}
static uint32_t ARM_SPI_Recieve_1(void *pdata, uint32_t num)
{
    return ARM_SPI_Recieve(TEST_APP_ARM_SPI1, pdata, num);
}
static uint32_t ARM_SPI_Recieve_2(void *pdata, uint32_t num)
{
    return ARM_SPI_Recieve(TEST_APP_ARM_SPI2, pdata, num);
}
static uint32_t ARM_SPI_Recieve_3(void *pdata, uint32_t num)
{
    return ARM_SPI_Recieve(TEST_APP_ARM_SPI3, pdata, num);
}
static uint32_t ARM_SPI_Recieve_4(void *pdata, uint32_t num)
{
    return ARM_SPI_Recieve(TEST_APP_ARM_SPI4, pdata, num);
}
static TEST_APP_ARM_SPI_Transfer_t ARM_SPI_GetTransfer_1(void)
{
    return ARM_SPI_GetTransfer(TEST_APP_ARM_SPI1);
}
static TEST_APP_ARM_SPI_Transfer_t ARM_SPI_GetTransfer_2(void)
{
    return ARM_SPI_GetTransfer(TEST_APP_ARM_SPI2);
}
static TEST_APP_ARM_SPI_Transfer_t ARM_SPI_GetTransfer_3(void)
{
    return ARM_SPI_GetTransfer(TEST_APP_ARM_SPI3);
}
static TEST_APP_ARM_SPI_Transfer_t ARM_SPI_GetTransfer_4(void)
{
    return ARM_SPI_GetTransfer(TEST_APP_ARM_SPI4);
}
static TEST_APP_ARM_SPI_Status_t ARM_SPI_GetStatus_1(void)
{
    return ARM_SPI_GetStatus(TEST_APP_ARM_SPI1);
}
static TEST_APP_ARM_SPI_Status_t ARM_SPI_GetStatus_2(void)
{
    return ARM_SPI_GetStatus(TEST_APP_ARM_SPI2);
}
static TEST_APP_ARM_SPI_Status_t ARM_SPI_GetStatus_3(void)
{
    return ARM_SPI_GetStatus(TEST_APP_ARM_SPI3);
}
static TEST_APP_ARM_SPI_Status_t ARM_SPI_GetStatus_4(void)
{
    return ARM_SPI_GetStatus(TEST_APP_ARM_SPI4);
}
