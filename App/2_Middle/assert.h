#ifndef _ASSERT_H_
#define _ASSERT_H_

void TEST_APP_AssertConfig(void);
void TEST_APP_AssertFailed(char *func, char *file, uint32_t line);
void TEST_APP_Logger(char *func, char *file, uint32_t line, char *report);

#define ASSERT(expr) expr ? (void)0 : TEST_APP_AssertFailed((char *)__FUNCTION__, (char *)__FILE__, __LINE__);

#define LOG(report) TEST_APP_Logger((char *)__FUNCTION__, (char *)__FILE__, __LINE__, report)


#endif //_ASSERT_H_ 
