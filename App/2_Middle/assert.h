#ifndef _ASSERT_H_
#define _ASSERT_H_

void AssertConfig(void);
void AssertFailed(char *func, char *file, uint32_t line);
void Logger(char *func, char *file, uint32_t line, char *report);

#define ASSERT(expr) expr ? (void)0 : AssertFailed((char *)__FUNCTION__, (char *)__FILE__, __LINE__);

#define LOG(report) Logger((char *)__FUNCTION__, (char *)__FILE__, __LINE__, report)


#endif //_ASSERT_H_ 
