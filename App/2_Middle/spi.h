#ifndef _SPI_H_
#define _SPI_H_

error_status TEST_APP_SPI_Init(void);
uint32_t TEST_APP_SPI_Initialize(&SPI1_Driver, TEST_APP_ARM_SPI_BAUDRATE_57600, SPI_DATA_8BITS,
                                 SPI_STOP_1_BIT, SPI_PARITY_NONE);

#endif //_SPI_H_ 
