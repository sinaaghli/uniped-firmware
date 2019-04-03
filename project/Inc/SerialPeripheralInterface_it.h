//
// Created by mrshannon on 4/2/19.
//


#ifndef SERIALPERIPHERALINTERFACE_IT_H
#define SERIALPERIPHERALINTERFACE_IT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include "stm32f4xx_hal_spi.h"

void spi_rx_complete_it(SPI_HandleTypeDef *spi);

#ifdef __cplusplus
}
#endif

#endif //SERIALPERIPHERALINTERFACE_IT_H
