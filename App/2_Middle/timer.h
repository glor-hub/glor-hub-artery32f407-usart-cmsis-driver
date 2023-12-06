#ifndef _TIMER_H_
#define _TIMER_H_

typedef enum {
    TIMER_DELAY = 0,
    TIMER_USART_TIMEOUT,
    NUM_TIMERS
} eTEST_APP_TimerTypes_t;

error_status TimerInit(void);
void TimerEnable(eTEST_APP_TimerTypes_t type, uint32_t time);
void TimerDisable(eTEST_APP_TimerTypes_t type);
flag_status TimerTestFlag(eTEST_APP_TimerTypes_t type);
confirm_state TimerTestSet(eTEST_APP_TimerTypes_t type);
void TimerDoDelay_ms(uint32_t time);

#endif //_TIMER_H_
