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
#include "gpio.h"

//********************************************************************************
//Macros
//********************************************************************************

#define LCD2004_PORT   GPIO_LCD2004_PORT

// five volt tolerant pins only
#define LCD2004_PIN_RS          GPIO_LCD2004_PIN_RS
#define LCD2004_PIN_RW          GPIO_LCD2004_PIN_RW
#define LCD2004_PIN_E           GPIO_LCD2004_PIN_E
// using 4-bit operation mode
#define LCD2004_PIN_D4          GPIO_LCD2004_PIN_D4
#define LCD2004_PIN_D5          GPIO_LCD2004_PIN_D5
#define LCD2004_PIN_D6          GPIO_LCD2004_PIN_D6
#define LCD2004_PIN_D7          GPIO_LCD2004_PIN_D7
#define LCD2004_PIN_MASK (LCD2004_PIN_RS | LCD2004_PIN_RW | LCD2004_PIN_E | LCD2004_PIN_D4 | LCD2004_PIN_D5 | LCD2004_PIN_D6 | LCD2004_PIN_D7)

//command code
#define LCD2004_CMD_CLEAR_DISPLAY           (uint8_t)(0x01)
#define LCD2004_CMD_DISPLAY_RETURN_HOME     (uint8_t)(0x02)
#define LCD2004_CMD_DISPLAY_OFF             (uint8_t)(0x08)
#define LCD2004_CMD_DISPLAY_ON              (uint8_t)(0x0C)
#define LCD2004_CMD_CURSOR_ON               (uint8_t)(0x0E)
#define LCD2004_CMD_BLINK_CURSOR_SET        (uint8_t)(0x0F)
//set 4-bit operation and select 2-line (4-line) display and 5x8 dot character font
#define LCD2004_CMD_SET_4BIT_INTERFACE_MODE (uint8_t)(0x28)
//set entry mode: cursor shift to right, non display shift at the time of write to the DD/CGRAM
#define LCD2004_CMD_SET_ENTRY_MODE          (uint8_t)(0x06)
//cursor shift without write to the DD/CGRAM
#define LCD2004_CMD_CURSOR_SHIFT_LEFT       (uint8_t)(0x10)
#define LCD2004_CMD_CURSOR_SHIFT_RIGHT      (uint8_t)(0x14)
//display shift without write to the DD/CGRAM
#define LCD2004_CMD_DISPLAY_SHIFT_LEFT      (uint8_t)(0x18)
#define LCD2004_CMD_DISPLAY_SHIFT_RIGHT     (uint8_t)(0x1C)
//set DDRAM address
#define LCD2004_CMD_SET_DDRAM_ADDR          (uint8_t)(0x80)
//set DDRAM address
#define LCD2004_CMD_SET_CGRAM_ADDR          (uint8_t)(0x40)

//DDRAM base address for rows
#define LCD2004_DDRAM_ADDR_ROW1 (uint8_t)(0x00)
#define LCD2004_DDRAM_ADDR_ROW2 (uint8_t)(0x40)
#define LCD2004_DDRAM_ADDR_ROW3 (uint8_t)(0x14)
#define LCD2004_DDRAM_ADDR_ROW4 (uint8_t)(0x54)

//********************************************************************************
//Enums
//********************************************************************************



//********************************************************************************
//Typedefs
//********************************************************************************

typedef enum {
    LCD2004_ROW1 = 0,
    LCD2004_ROW2,
    LCD2004_ROW3,
    LCD2004_ROW4,
    LCD2004_ROWS
} eLCD2004_Rows;

typedef enum {
    LCD2004_COLUMN1 = 0,
    LCD2004_COLUMN2,
    LCD2004_COLUMN3,
    LCD2004_COLUMN4,
    LCD2004_COLUMN5,
    LCD2004_COLUMN6,
    LCD2004_COLUMN7,
    LCD2004_COLUMN8,
    LCD2004_COLUMN9,
    LCD2004_COLUMN10,
    LCD2004_COLUMN11,
    LCD2004_COLUMN12,
    LCD2004_COLUMN13,
    LCD2004_COLUMN14,
    LCD2004_COLUMN15,
    LCD2004_COLUMN16,
    LCD2004_COLUMN17,
    LCD2004_COLUMN18,
    LCD2004_COLUMN19,
    LCD2004_COLUMN20,
    LCD2004_COLUMNS
} eLCD2004_Columns;


#define LCD2004_NUM_POS (LCD2004_ROWS*LCD2004_COLUMNS)

//********************************************************************************
//Variables
//********************************************************************************


static char LCD2004_Buff[LCD2004_ROWS][LCD2004_COLUMNS];
static const uint8_t LCD2004_RowAddrMap[LCD2004_ROWS] = {
    LCD2004_DDRAM_ADDR_ROW1,
    LCD2004_DDRAM_ADDR_ROW2,
    LCD2004_DDRAM_ADDR_ROW3,
    LCD2004_DDRAM_ADDR_ROW4
};

//********************************************************************************
//Prototypes
//********************************************************************************

static void LCD2004_WriteLowTetradByte(uint8_t byte);
static void LCD2004_SendCmd(uint8_t cmd);
static void LCD2004_SendData(uint8_t data);
static void LCD2004_DoDelay_us(uint32_t time);
static void LCD2004_GPIO_Init(void);
static void LCD2004_DoStrobe(void);
static void LCD2004_BuffPuts(uint8_t row, uint8_t col, char *str);
static void LCD2004_Refresh(void);
static void LCD2004_BuffUp(void);
static void LCD2004_ClearDisplay(void);
static void LCD2004_ShowCursor(void);
static void LCD2004_HideCursor(void);
static void LCD2004_BuffClear(void);
static void LCD2004_SetPos(uint8_t row, uint8_t col);

//================================================================================
//Public
//================================================================================

void TEST_APP_LCD2004_Init(void)
{
    LCD2004_GPIO_Init();
    TimerDoDelay_ms(40);
    TEST_APP_ARM_GPIO_BitsReset(LCD2004_PORT, LCD2004_PIN_RS);
    TEST_APP_ARM_GPIO_BitsReset(LCD2004_PORT, LCD2004_PIN_RW);
    LCD2004_WriteLowTetradByte(0x03);
    LCD2004_DoStrobe();
    TimerDoDelay_ms(5);
    LCD2004_WriteLowTetradByte(0x03);
    LCD2004_DoStrobe();
    LCD2004_DoDelay_us(100);
    LCD2004_WriteLowTetradByte(0x03);
    LCD2004_DoStrobe();
    TimerDoDelay_ms(1);
    LCD2004_WriteLowTetradByte(0x02);
    LCD2004_DoStrobe();
    TimerDoDelay_ms(1);
    LCD2004_SendCmd(LCD2004_CMD_SET_4BIT_INTERFACE_MODE);
    TimerDoDelay_ms(1);
    LCD2004_SendCmd(LCD2004_CMD_DISPLAY_OFF);
    TimerDoDelay_ms(1);
    LCD2004_ClearDisplay();
    TimerDoDelay_ms(2);
    LCD2004_SendCmd(LCD2004_CMD_SET_ENTRY_MODE);
    TimerDoDelay_ms(1);
    LCD2004_SendCmd(LCD2004_CMD_DISPLAY_ON);
    LCD2004_HideCursor();
#ifdef _TEST_APP_DEBUG_
    LCD2004_ShowCursor();
#endif//_TEST_APP_DEBUG_
    TimerDoDelay_ms(1);
    LCD2004_SendCmd(LCD2004_CMD_DISPLAY_RETURN_HOME);
    TimerDoDelay_ms(2);
    LCD2004_BuffClear();

}

int8_t TEST_APP_LCD2004_Printf(uint8_t row, uint8_t col, flag_status refresh, char *fmt, ...)
{
    int8_t res;
    char strbuff[256];
    va_list args;
    va_start(args, fmt);
    res = vsprintf(strbuff, fmt, args);
    LCD2004_BuffPuts(row, col, strbuff);
    va_end(args);
    if(refresh) {
        LCD2004_Refresh();
    }
    return res;
}

void TEST_APP_LCD2004_Test(void)
{
    uint8_t i, j, k;
    k = 0;
    LCD2004_BuffClear();
    for(i = 0; i < LCD2004_ROWS; i++) {
        for(j = 0; j < LCD2004_COLUMNS / 2; j++) {
            TEST_APP_LCD2004_Printf(i, 2 * j, SET, "%d", k++);
        }
    }
    TimerDoDelay_ms(3000);
    LCD2004_BuffClear();
    TEST_APP_LCD2004_Printf(0, 0, SET, "%s",
                            "The ARM Cortex-M is a group of 32-bit RISC ARM processor cores.");
    TimerDoDelay_ms(3000);
}

//================================================================================
//Private
//================================================================================

static void LCD2004_GPIO_Init(void)
{
    TEST_APP_ARM_CRM_PeriphClockEnable(TEST_APP_PERIPH_GPIO, LCD2004_PORT, TRUE);
    TEST_APP_ARM_GPIO_Config(LCD2004_PORT, LCD2004_PIN_RS | LCD2004_PIN_RW | LCD2004_PIN_E | LCD2004_PIN_D4 |
                             LCD2004_PIN_D5 | LCD2004_PIN_D6 | LCD2004_PIN_D7, GPIO_MODE_OUTPUT,
                             GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_NONE, GPIO_DRIVE_STRENGTH_MODERATE);
}


static void LCD2004_WriteLowTetradByte(uint8_t byte)
{
    if((byte >> 0) & 0x01) {
        TEST_APP_ARM_GPIO_BitsSet(LCD2004_PORT, LCD2004_PIN_D4);
    } else {
        TEST_APP_ARM_GPIO_BitsReset(LCD2004_PORT, LCD2004_PIN_D4);
    }
    if((byte >> 1) & 0x01) {
        TEST_APP_ARM_GPIO_BitsSet(LCD2004_PORT, LCD2004_PIN_D5);
    } else {
        TEST_APP_ARM_GPIO_BitsReset(LCD2004_PORT, LCD2004_PIN_D5);
    }
    if((byte >> 2) & 0x01) {
        TEST_APP_ARM_GPIO_BitsSet(LCD2004_PORT, LCD2004_PIN_D6);
    } else {
        TEST_APP_ARM_GPIO_BitsReset(LCD2004_PORT, LCD2004_PIN_D6);
    }
    if((byte >> 3) & 0x01) {
        TEST_APP_ARM_GPIO_BitsSet(LCD2004_PORT, LCD2004_PIN_D7);
    } else {
        TEST_APP_ARM_GPIO_BitsReset(LCD2004_PORT, LCD2004_PIN_D7);
    }
}

static void LCD2004_DoStrobe(void)
{
    TEST_APP_ARM_GPIO_BitsSet(LCD2004_PORT, LCD2004_PIN_E);
    LCD2004_DoDelay_us(1);
    TEST_APP_ARM_GPIO_BitsReset(LCD2004_PORT, LCD2004_PIN_E);
}


static void LCD2004_SendCmd(uint8_t cmd)
{
    TEST_APP_ARM_GPIO_BitsReset(LCD2004_PORT, LCD2004_PIN_RS);
    TEST_APP_ARM_GPIO_BitsReset(LCD2004_PORT, LCD2004_PIN_RW);
    LCD2004_WriteLowTetradByte(cmd >> 4);
    LCD2004_DoDelay_us(1);
    LCD2004_DoStrobe();
    LCD2004_DoDelay_us(1);
    LCD2004_WriteLowTetradByte(cmd);
    LCD2004_DoStrobe();
    LCD2004_DoDelay_us(40);
}

static void LCD2004_SendData(uint8_t data)
{
    TEST_APP_ARM_GPIO_BitsSet(LCD2004_PORT, LCD2004_PIN_RS);
    TEST_APP_ARM_GPIO_BitsReset(LCD2004_PORT, LCD2004_PIN_RW);
    LCD2004_WriteLowTetradByte(data >> 4);
    LCD2004_DoDelay_us(1);
    LCD2004_DoStrobe();
    LCD2004_DoDelay_us(1);
    LCD2004_WriteLowTetradByte(data);
    LCD2004_DoStrobe();
    LCD2004_DoDelay_us(40);
}

static void LCD2004_DoDelay_us(uint32_t time)
{
    uint32_t i, counter;
    counter = time * 100;
    for(i = 0; i < counter; i++) {
    }
}

static void LCD2004_Refresh(void)
{
    uint8_t row, col;
    for(row = LCD2004_ROW1; row < LCD2004_ROWS ; row++) {
        LCD2004_SetPos(row, LCD2004_COLUMN1);
        for(col = LCD2004_COLUMN1; col < LCD2004_COLUMNS; col++) {
            LCD2004_SendData(LCD2004_Buff[row][col]);
        }

    }
}

static void LCD2004_BuffUp(void)
{
    uint8_t row;
    for(row = LCD2004_ROW1; row < LCD2004_ROW4; row++) {
        memcpy(&LCD2004_Buff[row], &LCD2004_Buff[row + 1], LCD2004_COLUMNS);
    }
    row = LCD2004_ROW4;
    memset(&LCD2004_Buff[row], ' ', LCD2004_COLUMNS);
}

static void LCD2004_BuffPuts(uint8_t row, uint8_t col, char *str)
{
    char ch = *str;
    if(row > LCD2004_ROWS) {
        return;
    }
    if(col > LCD2004_COLUMNS) {
        return;
    }
    LCD2004_SetPos(row, col);
    while(ch) {
        if(col == LCD2004_COLUMNS) {
            col = LCD2004_COLUMN1;
            row++;
        }
        if(row == LCD2004_ROWS) {
            LCD2004_BuffUp();
            row = LCD2004_ROW4;
        }
        LCD2004_Buff[row][col] = ch;
        col++;
        str++;
        ch = *str;
    }
}

static void LCD2004_BuffClear(void)
{
    memset(LCD2004_Buff, ' ', sizeof(LCD2004_Buff));
}

static void LCD2004_ClearDisplay(void)
{
    LCD2004_SendCmd(LCD2004_CMD_CLEAR_DISPLAY);
}

static void LCD2004_ShowCursor(void)
{
    LCD2004_SendCmd(LCD2004_CMD_CURSOR_ON);
}

static void LCD2004_HideCursor(void)
{
    LCD2004_SendCmd(LCD2004_CMD_DISPLAY_ON);
}

static void LCD2004_SetPos(uint8_t row, uint8_t col)
{
    if(row > LCD2004_ROWS) {
        return;
    }
    if(col > LCD2004_COLUMNS) {
        return;
    }
    LCD2004_SendCmd(LCD2004_CMD_SET_DDRAM_ADDR | LCD2004_RowAddrMap[row] | col);
}
