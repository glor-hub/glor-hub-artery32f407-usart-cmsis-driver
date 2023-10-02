#ifndef _LED_H_
#define _LED_H_

#include "at32f403a_407.h"

typedef enum {
    LED_RED = 0,
    LED_YELLOW,
    LED_GREEN,
    LEDS
} eTEST_APP_LedColors_t;

void TEST_APP_LED_Init(eTEST_APP_LedColors_t color);
void TEST_APP_LED_Driver(eTEST_APP_LedColors_t color, uint8_t led_state);
void TEST_APP_LED_Test(void);

#endif //_LED_H_
