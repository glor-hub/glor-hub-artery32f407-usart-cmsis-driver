#ifndef _LCD_2004_H_
#define _LCD_2004_H_

#include "at32f403a_407.h"

void TEST_APP_LCD2004_Init(void);
int8_t TEST_APP_LCD2004_Printf(uint8_t row, uint8_t col, flag_status refresh, char *fmt, ...);
void TEST_APP_LCD2004_Test(void);

#endif //_LCD_2004_H_ 
