//********************************************************************************
//arm_gpio.c
//********************************************************************************

#include "at32f403a_407.h"
#include "arm_gpio.h"
#include "arm_clock.h"

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

static gpio_type *pARM_GPIO_pRegisters[TEST_APP_ARM_GPIO_PORTS] = {

    GPIOA,
    GPIOB,
    GPIOC,
    GPIOD,
    GPIOE
};

//********************************************************************************
//Prototypes
//********************************************************************************


//================================================================================
//Public
//================================================================================


void TEST_APP_ARM_GPIO_Config(eTEST_APP_ARM_GPIO_Ports_t port, uint32_t pins,
                              gpio_mode_type mode, gpio_output_type out_type,
                              gpio_pull_type pull, gpio_drive_type drive_strength)
{
    gpio_type *pReg = pARM_GPIO_pRegisters[port];
    gpio_init_type cfg;
    cfg.gpio_pins  = pins;
    cfg.gpio_out_type = out_type;
    cfg.gpio_pull = pull;
    cfg.gpio_mode = mode;
    cfg.gpio_drive_strength = drive_strength;
    gpio_init(pReg, &cfg);
}

void TEST_APP_ARM_GPIO_ReleaseBits(eTEST_APP_ARM_GPIO_Ports_t port, uint32_t pins)
{
    TEST_APP_ARM_GPIO_Config(port, pins, GPIO_MODE_INPUT, GPIO_OUTPUT_PUSH_PULL,
                             GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
}

void TEST_APP_ARM_GPIO_SetBits(eTEST_APP_ARM_GPIO_Ports_t port, uint32_t pins)
{
    gpio_type *pReg = pARM_GPIO_pRegisters[port];
    gpio_bits_set(pReg, pins);
}

void TEST_APP_ARM_GPIO_ResetBits(eTEST_APP_ARM_GPIO_Ports_t port, uint32_t pins)
{
    gpio_type *pReg = pARM_GPIO_pRegisters[port];
    gpio_bits_reset(pReg, pins);
}

flag_status TEST_APP_ARM_GPIO_ReadInputDataBit(eTEST_APP_ARM_GPIO_Ports_t port, uint32_t pins)
{
    flag_status status = RESET;
    gpio_type *pReg = pARM_GPIO_pRegisters[port];
    status = gpio_input_data_bit_read(pReg, pins);
    return status;
}

uint16_t TEST_APP_ARM_GPIO_ReadInputData(eTEST_APP_ARM_GPIO_Ports_t port)
{
    uint16_t data;
    gpio_type *pReg = pARM_GPIO_pRegisters[port];
    data = gpio_input_data_read(pReg);
    return data;
}

flag_status TEST_APP_ARM_GPIO_ReadOutputDataBit(eTEST_APP_ARM_GPIO_Ports_t port, uint32_t pins)
{
    flag_status status = RESET;
    gpio_type *pReg = pARM_GPIO_pRegisters[port];
    status = gpio_output_data_bit_read(pReg, pins);
    return status;
}

uint16_t TEST_APP_ARM_GPIO_ReadOutputData(eTEST_APP_ARM_GPIO_Ports_t port)
{
    uint16_t data;
    gpio_type *pReg = pARM_GPIO_pRegisters[port];
    data = gpio_output_data_read(pReg);
    return data;
}

//================================================================================
//Private
//================================================================================
