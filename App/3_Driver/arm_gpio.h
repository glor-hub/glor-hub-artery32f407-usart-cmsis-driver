#ifndef _ARM_GPIO_H_
#define _ARM_GPIO_H_

#include "at32f403a_407.h"

typedef enum {
    TEST_APP_ARM_GPIO_PORTA = 0,
    TEST_APP_ARM_GPIO_PORTB,
    TEST_APP_ARM_GPIO_PORTC,
    TEST_APP_ARM_GPIO_PORTD,
    TEST_APP_ARM_GPIO_PORTE,
    TEST_APP_ARM_GPIO_PORTS,
    TEST_APP_ARM_GPIO_PORT_NOT_DEFINED
} eTEST_APP_ARM_GPIO_Ports_t;

#define TEST_APP_ARM_GPIO_PIN_NOT_DEFINED   (uint32_t)0x0000

void TEST_APP_ARM_GPIO_Config(eTEST_APP_ARM_GPIO_Ports_t port, uint32_t pins,
                              gpio_mode_type mode, gpio_output_type out_type,
                              gpio_pull_type pull, gpio_drive_type drive_strength);
void TEST_APP_ARM_GPIO_ReleaseBits(eTEST_APP_ARM_GPIO_Ports_t port, uint32_t pins);

void TEST_APP_ARM_GPIO_SetBits(eTEST_APP_ARM_GPIO_Ports_t port, uint32_t pins);
void TEST_APP_ARM_GPIO_ResetBits(eTEST_APP_ARM_GPIO_Ports_t port, uint32_t pins);
flag_status TEST_APP_ARM_GPIO_ReadInputDataBit(eTEST_APP_ARM_GPIO_Ports_t port, uint32_t pins);

uint16_t TEST_APP_ARM_GPIO_ReadInputData(eTEST_APP_ARM_GPIO_Ports_t port);

flag_status TEST_APP_ARM_GPIO_ReadOutputDataBit(eTEST_APP_ARM_GPIO_Ports_t port, uint32_t pins);

uint16_t TEST_APP_ARM_GPIO_ReadOutputData(eTEST_APP_ARM_GPIO_Ports_t port);


#endif //_ARM_GPIO_H_ 
