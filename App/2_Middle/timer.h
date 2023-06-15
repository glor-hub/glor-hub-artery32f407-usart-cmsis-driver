#ifndef _TIMER_H_
#define _TIMER_H_

typedef enum {
    TIMER_DELAY = 0,
    NUM_TIMERS
} eTimerTypes;

error_status TimerInit(void);
void TimerEnable(eTimerTypes type, uint32_t time);
void TimerDisable(eTimerTypes type);
flag_status TimerTestFlag(eTimerTypes type);
confirm_state TimerTestSet(eTimerTypes type);
void TimerDoDelay_ms(uint32_t time);

#endif //_TIMER_H_
