#ifndef _TIMER_H_
#define _TIMER_H_

typedef enum {
    TIMER_DELAY = 0,
    TIMER_USART1_TIMEOUT_TX,
    TIMER_USART1_TIMEOUT_RX,
    TIMER_USART2_TIMEOUT_TX,
    TIMER_USART2_TIMEOUT_RX,
    TIMER_USART3_TIMEOUT_TX,
    TIMER_USART3_TIMEOUT_RX,
    TIMER_UART4_TIMEOUT_TX,
    TIMER_UART4_TIMEOUT_RX,
    TIMER_UART5_TIMEOUT_TX,
    TIMER_UART5_TIMEOUT_RX,
    TIMER_USART6_TIMEOUT_TX,
    TIMER_USART6_TIMEOUT_RX,
    TIMER_UART7_TIMEOUT_TX,
    TIMER_UART7_TIMEOUT_RX,
    TIMER_UART8_TIMEOUT_TX,
    TIMER_UART8_TIMEOUT_RX,
    NUM_TIMERS
} eTEST_APP_TimerTypes_t;

error_status TimerInit(void);
void TimerEnable(eTEST_APP_TimerTypes_t type, uint32_t time);
void TimerDisable(eTEST_APP_TimerTypes_t type);
flag_status TimerTestFlag(eTEST_APP_TimerTypes_t type);
confirm_state TimerTestSet(eTEST_APP_TimerTypes_t type);
void TimerDoDelay_ms(uint32_t time);

#endif //_TIMER_H_
