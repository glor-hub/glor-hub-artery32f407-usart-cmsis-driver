#ifndef _SPI_H_
#define _SPI_H_


#include "at32f403a_407.h"
#include "app.h"
#include "arm_spi.h"

//************************************************************************
//Macros
//************************************************************************

#define SPI_NO_DELAY_USEC_AFTER_CS      ((uint32_t)0)

//for AD7685
#define SPI_AD7685_ADC_NUM 8
#define SPI_AD7685_DELAY_USEC_AFTER_CS  ((uint32_t)10)
#define SPI_AD7685_CS_ACTIVE_LEVEL TEST_APP_ARM_SPI_CS_ACTIVE_LEVEL_HIGH

//************************************************************************
//Prototypes
//************************************************************************
error_status TEST_APP_SPI_Test(void);
void TEST_APP_SPI_cb(void);
error_status TEST_APP_SPI_Init(void);

#endif //_SPI_H_ 
