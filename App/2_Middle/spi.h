#ifndef _SPI_H_
#define _SPI_H_


#include "at32f403a_407.h"
#include "app.h"
#include "arm_spi.h"

error_status TEST_APP_SPI_Init(void);
uint32_t TEST_APP_SPI_Initialize(TEST_APP_ARM_SPI_Driver_t *p_drv, spi_master_slave_mode_type mode,
                                 spi_transmission_mode_type xfer_mode,
                                 spi_half_duplex_direction_type dir,
                                 spi_mclk_freq_div_type mclk_div,
                                 eTEST_APP_ARM_SPI_ClockLatchTypes_t clk_type,
                                 spi_cs_mode_type cs_mode,
                                 eTEST_APP_ARM_SPI_CSActiveLevel_t cs_active_level,
                                 eTEST_APP_ARM_SPI_CSConfirmEveryWorld_t cs_conf_every_world,
                                 spi_frame_bit_num_type data_bit_num,
                                 spi_first_bit_type data_first_bit,
                                 uint32_t gpio_pin_def);
uint32_t TEST_APP_SPI_Uninitialize(TEST_APP_ARM_SPI_Driver_t *p_drv);
void TEST_APP_SPI_cb(void);
error_status TEST_APP_SPI_Test(void);
#endif //_SPI_H_ 
