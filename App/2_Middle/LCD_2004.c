//********************************************************************************
//LCD_2004.c
//Driver for LCD2004 (20x4) using 4-bit operation mode.

//********************************************************************************
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "LCD_2004.h"
#include "common.h"
#include "arm_gpio.h"
#include "arm_clock.h"
#include "timer.h"

//********************************************************************************
//Macros
//********************************************************************************

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

//command code
#define LCD_CMD_CLEAR_DISPLAY           (uint8_t)(0x01)
#define LCD_CMD_DISPLAY_RETURN_HOME     (uint8_t)(0x02)
#define LCD_CMD_DISPLAY_OFF             (uint8_t)(0x08)
#define LCD_CMD_DISPLAY_ON              (uint8_t)(0x0C)
#define LCD_CMD_CURSOR_ON               (uint8_t)(0x0E)
#define LCD_CMD_BLINK_CURSOR_SET        (uint8_t)(0x0F)
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
//set DDRAM address
#define LCD_CMD_SET_DDRAM_ADDR          (uint8_t)(0x80)
//set DDRAM address
#define LCD_CMD_SET_CGRAM_ADDR          (uint8_t)(0x40)

//DDRAM base address for rows
#define LCD_DDRAM_ADDR_ROW1 (uint8_t)(0x00)
#define LCD_DDRAM_ADDR_ROW2 (uint8_t)(0x40)
#define LCD_DDRAM_ADDR_ROW3 (uint8_t)(0x14)
#define LCD_DDRAM_ADDR_ROW4 (uint8_t)(0x54)

//********************************************************************************
//Enums
//********************************************************************************



//********************************************************************************
//Typedefs
//********************************************************************************

typedef enum {
    LCD_ROW1 = 0,
    LCD_ROW2,
    LCD_ROW3,
    LCD_ROW4,
    LCD_ROWS
} eLCD_Rows;

typedef enum {
    LCD_COLUMN1 = 0,
    LCD_COLUMN2,
    LCD_COLUMN3,
    LCD_COLUMN4,
    LCD_COLUMN5,
    LCD_COLUMN6,
    LCD_COLUMN7,
    LCD_COLUMN8,
    LCD_COLUMN9,
    LCD_COLUMN10,
    LCD_COLUMN11,
    LCD_COLUMN12,
    LCD_COLUMN13,
    LCD_COLUMN14,
    LCD_COLUMN15,
    LCD_COLUMN16,
    LCD_COLUMN17,
    LCD_COLUMN18,
    LCD_COLUMN19,
    LCD_COLUMN20,
    LCD_COLUMNS
} eLCD_Columns;


#define LCD_NUM_POS (LCD_ROWS*LCD_COLUMNS)

//********************************************************************************
//Variables
//********************************************************************************


static char LCD_Buff[LCD_ROWS][LCD_COLUMNS];
static const uint8_t LCD_RowAddrMap[LCD_ROWS] = {
    LCD_DDRAM_ADDR_ROW1,
    LCD_DDRAM_ADDR_ROW2,
    LCD_DDRAM_ADDR_ROW3,
    LCD_DDRAM_ADDR_ROW4
};

//********************************************************************************
//Prototypes
//********************************************************************************

static void LCD_WriteLowTetradByte(uint8_t byte);
static void LCD_SendCmd(uint8_t cmd);
static void LCD_SendData(uint8_t data);
static void LCD_DoDelay_us(uint32_t time);
static void LCD_GPIO_Init(void);
static void LCD_DoStrobe(void);
static void LCD_BuffPuts(uint8_t row, uint8_t col, char *str);
static void LCD_Refresh(void);
static void LCD_BuffUp(void);
static void LCD_ClearDisplay(void);
static void LCD_ShowCursor(void);
static void LCD_HideCursor(void);
static void LCD_BuffClear(void);
static void LCD_SetPos(uint8_t row, uint8_t col);

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
    LCD_DoStrobe();
    TimerDoDelay_ms(5);
    LCD_WriteLowTetradByte(0x03);
    LCD_DoStrobe();
    LCD_DoDelay_us(100);
    LCD_WriteLowTetradByte(0x03);
    LCD_DoStrobe();
    TimerDoDelay_ms(1);
    LCD_WriteLowTetradByte(0x02);
    LCD_DoStrobe();
    TimerDoDelay_ms(1);
    LCD_SendCmd(LCD_CMD_SET_4BIT_INTERFACE_MODE);
    TimerDoDelay_ms(1);
    LCD_SendCmd(LCD_CMD_DISPLAY_OFF);
    TimerDoDelay_ms(1);
    LCD_ClearDisplay();
    TimerDoDelay_ms(2);
    LCD_SendCmd(LCD_CMD_SET_ENTRY_MODE);
    TimerDoDelay_ms(1);
    LCD_SendCmd(LCD_CMD_DISPLAY_ON);
    LCD_HideCursor();
#ifdef _APP_DEBUG_
    LCD_ShowCursor();
#endif//_APP_DEBUG_
    TimerDoDelay_ms(1);
    LCD_SendCmd(LCD_CMD_DISPLAY_RETURN_HOME);
    TimerDoDelay_ms(2);
    LCD_BuffClear();

}

int8_t LCD_Printf(uint8_t row, uint8_t col, flag_status refresh, char *fmt, ...)
{
    int8_t res;
    char strbuff[256];
    va_list args;
    va_start(args, fmt);
    res = vsprintf(strbuff, fmt, args);
    LCD_BuffPuts(row, col, strbuff);
    va_end(args);
    if(refresh) {
        LCD_Refresh();
    }
    return res;
}

void LCD_Test(void)
{
    uint8_t i, j, k;
    k = 0;
    LCD_BuffClear();
    for(i = 0; i < LCD_ROWS; i++) {
        for(j = 0; j < LCD_COLUMNS / 2; j++) {
            LCD_Printf(i, 2 * j, SET, "%d", k++);
        }
    }
    TimerDoDelay_ms(3000);
    LCD_BuffClear();
    LCD_Printf(0, 0, SET, "%s",
               "The ARM Cortex-M is a group of 32-bit RISC ARM processor cores.");
    TimerDoDelay_ms(3000);
}

//================================================================================
//Private
//================================================================================

static void LCD_GPIO_Init(void)
{
    ARM_CRM_GPIO_ClockEnable(LCD_PORT_REG_ADDR, TRUE);
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

static void LCD_DoStrobe(void)
{
    ARM_GPIO_BitsSet(LCD_PORT_REG_ADDR, LCD_PIN_E);
    LCD_DoDelay_us(1);
    ARM_GPIO_BitsReset(LCD_PORT_REG_ADDR, LCD_PIN_E);
}


static void LCD_SendCmd(uint8_t cmd)
{
    ARM_GPIO_BitsReset(LCD_PORT_REG_ADDR, LCD_PIN_RS);
    ARM_GPIO_BitsReset(LCD_PORT_REG_ADDR, LCD_PIN_RW);
    LCD_WriteLowTetradByte(cmd >> 4);
    LCD_DoDelay_us(1);
    LCD_DoStrobe();
    LCD_DoDelay_us(1);
    LCD_WriteLowTetradByte(cmd);
    LCD_DoStrobe();
    LCD_DoDelay_us(40);
}

static void LCD_SendData(uint8_t data)
{
    ARM_GPIO_BitsSet(LCD_PORT_REG_ADDR, LCD_PIN_RS);
    ARM_GPIO_BitsReset(LCD_PORT_REG_ADDR, LCD_PIN_RW);
    LCD_WriteLowTetradByte(data >> 4);
    LCD_DoDelay_us(1);
    LCD_DoStrobe();
    LCD_DoDelay_us(1);
    LCD_WriteLowTetradByte(data);
    LCD_DoStrobe();
    LCD_DoDelay_us(40);
}

static void LCD_DoDelay_us(uint32_t time)
{
    uint32_t i, counter;
    counter = time * 100;
    for(i = 0; i < counter; i++) {
    }
}

static void LCD_Refresh(void)
{
    uint8_t row, col;
    for(row = LCD_ROW1; row < LCD_ROWS ; row++) {
        LCD_SetPos(row, LCD_COLUMN1);
        for(col = LCD_COLUMN1; col < LCD_COLUMNS; col++) {
            LCD_SendData(LCD_Buff[row][col]);
        }

    }
}

static void LCD_BuffUp(void)
{
    uint8_t row;
    for(row = LCD_ROW1; row < LCD_ROW4; row++) {
        memcpy(&LCD_Buff[row], &LCD_Buff[row + 1], LCD_COLUMNS);
    }
    row = LCD_ROW4;
    memset(&LCD_Buff[row], ' ', LCD_COLUMNS);
}

static void LCD_BuffPuts(uint8_t row, uint8_t col, char *str)
{
    char ch = *str;
    if(row > LCD_ROWS) {
        return;
    }
    if(col > LCD_COLUMNS) {
        return;
    }
    LCD_SetPos(row, col);
    while(ch) {
        if(col == LCD_COLUMNS) {
            col = LCD_COLUMN1;
            row++;
        }
        if(row == LCD_ROWS) {
            LCD_BuffUp();
            row = LCD_ROW4;
        }
        LCD_Buff[row][col] = ch;
        col++;
        str++;
        ch = *str;
    }
}

static void LCD_BuffClear(void)
{
    memset(LCD_Buff, ' ', sizeof(LCD_Buff));
}

static void LCD_ClearDisplay(void)
{
    LCD_SendCmd(LCD_CMD_CLEAR_DISPLAY);
}

static void LCD_ShowCursor(void)
{
    LCD_SendCmd(LCD_CMD_CURSOR_ON);
}

static void LCD_HideCursor(void)
{
    LCD_SendCmd(LCD_CMD_DISPLAY_ON);
}

static void LCD_SetPos(uint8_t row, uint8_t col)
{
    if(row > LCD_ROWS) {
        return;
    }
    if(col > LCD_COLUMNS) {
        return;
    }
    LCD_SendCmd(LCD_CMD_SET_DDRAM_ADDR | LCD_RowAddrMap[row] | col);
}
