#ifndef _ASSERT_H_
#define _ASSERT_H_

void AssertConfig(void);
void AssertFailed(uint8_t *func, uint8_t *file, uint32_t line);
void Logger(uint8_t *func, uint8_t *file, uint8_t line, char *report);

#define ASSERT(expr) expr ? (void)0 : AssertFailed((uint8_t *)__FUNCTION__, (uint8_t *)__FILE__, __LINE__);

#define LOG(report) Logger((uint8_t *)__FUNCTION__, (uint8_t *)__FILE__, __LINE__, report)


#endif //_ASSERT_H_ 
