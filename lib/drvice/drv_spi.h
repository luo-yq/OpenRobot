/*
 *  drv_spi.h
 *
 *  Created on: 2016. 7.13.
 *      Author: Baram, PBHP
 */

#ifndef DRV_SPI_H
#define DRV_SPI_H


#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f7xx.h"
#include <stdint.h>
#include <stdbool.h>







int     drv_spi_init(void);
void    drv_spi_enable_dma(SPI_HandleTypeDef* hspi);
uint8_t drv_spi_is_dma_tx_done(SPI_HandleTypeDef* hspi);
void    drv_spi_start_dma_tx(SPI_HandleTypeDef* hspi, uint8_t *p_buf, uint32_t length);

#ifdef __cplusplus
}
#endif


#endif
