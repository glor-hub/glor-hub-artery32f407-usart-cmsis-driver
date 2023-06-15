#ifndef _ARM_GPIO_H_
#define _ARM_GPIO_H_

#include "at32f403a_407.h"

void ARM_GPIO_Config(gpio_type *pGPIO_X, uint32_t pins, gpio_mode_type mode, gpio_output_type out_type, gpio_pull_type pull, gpio_drive_type drive_strength);

void ARM_GPIO_BitsSet(gpio_type *pGPIO_X, uint32_t pins);

void ARM_GPIO_BitsReset(gpio_type *pGPIO_X, uint32_t pins);

#endif //_ARM_GPIO_H_ 
