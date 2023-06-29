#ifndef _LCD_2004_H_
#define _LCD_2004_H_

#include "at32f403a_407.h"

void LCD_Init(void);
int8_t LCD_Printf(uint8_t row, uint8_t col, flag_status refresh, char *fmt, ...);
void LCD_Test(void);

#endif //_LCD_2004_H_ 
