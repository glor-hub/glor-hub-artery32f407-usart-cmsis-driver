//********************************************************************************
//led.c
//********************************************************************************

#include "led.h"
#include "arm_gpio.h"
#include "arm_clock.h"
#include "timer.h"

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
    ARM_CRM_GPIO_ClockEnable(GPIOD, TRUE);
    switch(color) {
        case LED_RED: {
            ARM_GPIO_Config(GPIOD, GPIO_PINS_13, GPIO_MODE_OUTPUT, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
            break;
        }
        case LED_YELLOW: {
            ARM_GPIO_Config(GPIOD, GPIO_PINS_14, GPIO_MODE_OUTPUT, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
            break;
        }
        case LED_GREEN: {
            ARM_GPIO_Config(GPIOD, GPIO_PINS_15, GPIO_MODE_OUTPUT, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
            break;
        }
        case LEDS: {
            ARM_GPIO_Config(GPIOD, GPIO_PINS_13 | GPIO_PINS_14 | GPIO_PINS_15, GPIO_MODE_OUTPUT, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
            break;
        }
        default: {
            break;
        }
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
        case LEDS: {
            if(!led_state) {
                ARM_GPIO_BitsSet(GPIOD, GPIO_PINS_13 | GPIO_PINS_14 | GPIO_PINS_15);
            } else {
                ARM_GPIO_BitsReset(GPIOD, GPIO_PINS_13 | GPIO_PINS_14 | GPIO_PINS_15);
            }
            break;
        }
        default: {
            break;
        }
    }
}

void LED_Test(void)
{
    LED_Init(LEDS);
    LED_Driver(LEDS, FALSE);
    TimerDoDelay_ms(500);
    LED_Driver(LED_RED, TRUE);
    TimerDoDelay_ms(500);
    LED_Driver(LED_RED, FALSE);
    LED_Driver(LED_YELLOW, TRUE);
    TimerDoDelay_ms(500);
    LED_Driver(LED_YELLOW, FALSE);
    LED_Driver(LED_GREEN, TRUE);
    TimerDoDelay_ms(500);
    LED_Driver(LED_GREEN, FALSE);
    TimerDoDelay_ms(500);
    LED_Driver(LEDS, TRUE);
    TimerDoDelay_ms(500);
}

//================================================================================
//Private
//================================================================================

