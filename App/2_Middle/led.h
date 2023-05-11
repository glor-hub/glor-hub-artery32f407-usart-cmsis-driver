#ifndef _LED_H_
#define _LED_H_

#include "at32f403a_407.h"

typedef enum {
    LED_RED = 0,
    LED_YELLOW,
    LED_GREEN,
    LED_ALL
} eLedColors;

void LED_Init(eLedColors color);
void LED_Driver(eLedColors color, uint8_t led_state);

#endif //_LED_H_