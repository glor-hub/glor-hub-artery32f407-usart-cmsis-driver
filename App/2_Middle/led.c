//********************************************************************************
//led.c
//********************************************************************************

#include "at32f403a_407.h"
#include "led.h"
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


//********************************************************************************
//Prototypes
//********************************************************************************

//================================================================================
//Public
//================================================================================

void LED_Init(eLedColors color)
{
    ARM_CRM_ClockPeriphEnable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
    switch(color) {
        case LED_RED: {
            ARM_GPIO_Init(GPIOD, GPIO_PINS_13, GPIO_MODE_OUTPUT, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_MODERATE);
            break;
        }
        case LED_YELLOW: {
            ARM_GPIO_Init(GPIOD, GPIO_PINS_14, GPIO_MODE_OUTPUT, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_MODERATE);
            break;
        }
        case LED_GREEN: {
            ARM_GPIO_Init(GPIOD, GPIO_PINS_15, GPIO_MODE_OUTPUT, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_MODERATE);
            break;
        }
        case LED_ALL: {
            ARM_GPIO_Init(GPIOD, GPIO_PINS_13, GPIO_MODE_OUTPUT, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
            ARM_GPIO_Init(GPIOD, GPIO_PINS_14, GPIO_MODE_OUTPUT, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_MODERATE);
            ARM_GPIO_Init(GPIOD, GPIO_PINS_15, GPIO_MODE_OUTPUT, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_MODERATE);
            break;
        }
        default:
            break;
    }
}
void LED_Driver(eLedColors color, uint8_t led_state)
{
    switch(color) {
        case LED_RED: {
            if(!led_state) {
                ARM_GPIO_BitsSet(GPIOD, GPIO_PINS_13);
            } else {
                ARM_GPIO_BitsReset(GPIOD, GPIO_PINS_13);
            }
            break;
        }
        case LED_YELLOW: {
            if(!led_state) {
                ARM_GPIO_BitsSet(GPIOD, GPIO_PINS_14);
            } else {
                ARM_GPIO_BitsReset(GPIOD, GPIO_PINS_14);
            }
            break;
        }
        case LED_GREEN: {
            if(!led_state) {
                ARM_GPIO_BitsSet(GPIOD, GPIO_PINS_15);
            } else {
                ARM_GPIO_BitsReset(GPIOD, GPIO_PINS_15);
            }
            break;
        }
        case LED_ALL: {
            if(!led_state) {
                ARM_GPIO_BitsSet(GPIOD, GPIO_PINS_13 | GPIO_PINS_14 | GPIO_PINS_15);
            } else {
                ARM_GPIO_BitsReset(GPIOD, GPIO_PINS_13 | GPIO_PINS_14 | GPIO_PINS_15);
            }
            break;
        }
        default:
            break;
    }
}

//================================================================================
//Private
//================================================================================

