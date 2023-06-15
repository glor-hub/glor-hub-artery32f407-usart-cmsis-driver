//********************************************************************************
//LCD_2004.c
//Driver for LCD2004 (20x4) using 4-bit operation mode.

//********************************************************************************
#include "LCD_2004.h"
#include "at32f403a_407.h"
#include "common.h"
#include "arm_gpio.h"
#include "arm_clock.h"
#include "timer.h"

//********************************************************************************
//Macros
//********************************************************************************

#define LCD_PORT GPIO_PORT_E
#define LCD_PORT_REG_ADDR (gpio_type *)GPIOE

// five volt tolerant pins only
#define LCD_PIN_RS GPIO_PINS_9
#define LCD_PIN_RW GPIO_PINS_10
#define LCD_PIN_E GPIO_PINS_11
// using 4-bit operation mode
#define LCD_PIN_D4 GPIO_PINS_12
#define LCD_PIN_D5 GPIO_PINS_13
#define LCD_PIN_D6 GPIO_PINS_14
#define LCD_PIN_D7 GPIO_PINS_15
#define LCD_PIN_MASK (LCD_PIN_RS | LCD_PIN_RW | LCD_PIN_E | LCD_PIN_D4 | LCD_PIN_D5 | LCD_PIN_D6 | LCD_PIN_D7)

//commands code
#define LCD_CMD_CLEAR_DISPLAY           (uint8_t)(0x01)
#define LCD_CMD_DISPLAY_RETURN_HOME     (uint8_t)(0x02)
#define LCD_CMD_DISPLAY_OFF             (uint8_t)(0x08)
#define LCD_CMD_DISPLAY_ON              (uint8_t)(0x0C) //non cursor
#define LCD_CMD_CURSOR_ON               (uint8_t)(0x0A)
#define LCD_CMD_BLINK_CURSOR_SET        (uint8_t)(0x09)
//set 4-bit operation and select 2-line (4-line) display and 5x8 dot character font
#define LCD_CMD_SET_4BIT_INTERFACE_MODE (uint8_t)(0x28)
//set entry mode: cursor shift to right, non display shift at the time of write to the DD/CGRAM
#define LCD_CMD_SET_ENTRY_MODE          (uint8_t)(0x06)
//cursor shift without write to the DD/CGRAM
#define LCD_CMD_CURSOR_SHIFT_LEFT       (uint8_t)(0x10)
#define LCD_CMD_CURSOR_SHIFT_RIGHT      (uint8_t)(0x14)
//display shift without write to the DD/CGRAM
#define LCD_CMD_DISPLAY_SHIFT_LEFT      (uint8_t)(0x18)
#define LCD_CMD_DISPLAY_SHIFT_RIGHT     (uint8_t)(0x1C)

//********************************************************************************
//Enums
//********************************************************************************

typedef enum {
    LCD_SEND_DATA = 0,
    LCD_SEND_CMD,
    NUM_InstrTypes
} eLCD_InstrTypes;


//********************************************************************************
//Typedefs
//********************************************************************************

//********************************************************************************
//Variables
//********************************************************************************



//********************************************************************************
//Prototypes
//********************************************************************************

static void LCD_WriteLowTetradByte(uint8_t byte);
static void LCD_SendByte(uint8_t byte, eLCD_InstrTypes instr_type);
static void LCD_DoDelay_us(uint32_t time);
static void LCD_GPIO_Init(void);
static void LCD_Strobe(void);

//================================================================================
//Public
//================================================================================

void LCD_Init(void)
{
    LCD_GPIO_Init();
    TimerDoDelay_ms(40);
    ARM_GPIO_BitsReset(LCD_PORT_REG_ADDR, LCD_PIN_RS);
    ARM_GPIO_BitsReset(LCD_PORT_REG_ADDR, LCD_PIN_RW);
    LCD_WriteLowTetradByte(0x03);
    LCD_Strobe();
    TimerDoDelay_ms(5);
    LCD_WriteLowTetradByte(0x03);
    LCD_Strobe();
    LCD_DoDelay_us(100);
    LCD_WriteLowTetradByte(0x03);
    LCD_Strobe();
    TimerDoDelay_ms(1);
    LCD_WriteLowTetradByte(0x02);
    LCD_Strobe();
    TimerDoDelay_ms(1);
    LCD_SendByte(LCD_CMD_SET_4BIT_INTERFACE_MODE, LCD_SEND_CMD);
    TimerDoDelay_ms(1);
    LCD_SendByte(LCD_CMD_DISPLAY_OFF, LCD_SEND_CMD);
    TimerDoDelay_ms(1);
    LCD_SendByte(LCD_CMD_CLEAR_DISPLAY, LCD_SEND_CMD);
    TimerDoDelay_ms(2);
    LCD_SendByte(LCD_CMD_SET_ENTRY_MODE, LCD_SEND_CMD);
    TimerDoDelay_ms(1);
    LCD_SendByte(LCD_CMD_DISPLAY_ON | LCD_CMD_CURSOR_ON | LCD_CMD_BLINK_CURSOR_SET, LCD_SEND_CMD);
    TimerDoDelay_ms(1);
    LCD_SendByte(LCD_CMD_DISPLAY_RETURN_HOME, LCD_SEND_CMD);
    TimerDoDelay_ms(2);
    // LCD_SendByte('s', LCD_SEND_DATA);
}


//================================================================================
//Private
//================================================================================

static void LCD_GPIO_Init(void)
{
    switch(LCD_PORT) {
        case GPIO_PORT_A: {
            ARM_CRM_ClockPeriphEnable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
            break;
        }
        case GPIO_PORT_B: {
            ARM_CRM_ClockPeriphEnable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
            break;
        }
        case GPIO_PORT_C: {
            ARM_CRM_ClockPeriphEnable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
            break;
        }
        case GPIO_PORT_D: {
            ARM_CRM_ClockPeriphEnable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
            break;
        }
        case GPIO_PORT_E: {
            ARM_CRM_ClockPeriphEnable(CRM_GPIOE_PERIPH_CLOCK, TRUE);
            break;
        }
        default: {
            break;
        }
    }

    ARM_GPIO_Config(LCD_PORT_REG_ADDR, LCD_PIN_RS | LCD_PIN_RW | LCD_PIN_E | LCD_PIN_D4 |
                    LCD_PIN_D5 | LCD_PIN_D6 | LCD_PIN_D7, GPIO_MODE_OUTPUT,
                    GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
}


static void LCD_WriteLowTetradByte(uint8_t byte)
{
    if((byte >> 0) & 0x01) {
        ARM_GPIO_BitsSet(LCD_PORT_REG_ADDR, LCD_PIN_D4);
    } else {
        ARM_GPIO_BitsReset(LCD_PORT_REG_ADDR, LCD_PIN_D4);
    }
    if((byte >> 1) & 0x01) {
        ARM_GPIO_BitsSet(LCD_PORT_REG_ADDR, LCD_PIN_D5);
    } else {
        ARM_GPIO_BitsReset(LCD_PORT_REG_ADDR, LCD_PIN_D5);
    }
    if((byte >> 2) & 0x01) {
        ARM_GPIO_BitsSet(LCD_PORT_REG_ADDR, LCD_PIN_D6);
    } else {
        ARM_GPIO_BitsReset(LCD_PORT_REG_ADDR, LCD_PIN_D6);
    }
    if((byte >> 3) & 0x01) {
        ARM_GPIO_BitsSet(LCD_PORT_REG_ADDR, LCD_PIN_D7);
    } else {
        ARM_GPIO_BitsReset(LCD_PORT_REG_ADDR, LCD_PIN_D7);
    }
}

static void LCD_Strobe(void)
{
    ARM_GPIO_BitsSet(LCD_PORT_REG_ADDR, LCD_PIN_E);
    LCD_DoDelay_us(1);
    ARM_GPIO_BitsReset(LCD_PORT_REG_ADDR, LCD_PIN_E);
}


static void LCD_SendByte(uint8_t byte, eLCD_InstrTypes instr_type)
{
    switch(instr_type) {
        case LCD_SEND_CMD: {
            ARM_GPIO_BitsReset(LCD_PORT_REG_ADDR, LCD_PIN_RS);
            break;
        }
        case LCD_SEND_DATA: {
            ARM_GPIO_BitsSet(LCD_PORT_REG_ADDR, LCD_PIN_RS);
            break;
        }
        default: {
            break;
        }
    }
    ARM_GPIO_BitsReset(LCD_PORT_REG_ADDR, LCD_PIN_RW);
    LCD_WriteLowTetradByte(byte >> 4);
    LCD_DoDelay_us(1);
    LCD_Strobe();
    LCD_DoDelay_us(1);
    LCD_WriteLowTetradByte(byte);
    LCD_Strobe();
    LCD_DoDelay_us(40);
}

static void LCD_DoDelay_us(uint32_t time)
{
    uint32_t i, counter;
    counter = time * 100;
    for(i = 0; i < counter; i++) {
    }
}
