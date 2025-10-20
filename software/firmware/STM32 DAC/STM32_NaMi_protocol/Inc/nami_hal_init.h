#ifndef INC_NAMI_HAL_INIT_H_
#define INC_NAMI_HAL_INIT_H_

#include "main.h"
#include "nami_protocol.h"

void nami_hal_register_spi_callbacks(const NaMi_Hardware_Config* hw_config);
void nami_hal_reset_w5500(const NaMi_Hardware_Config* hw_config);

#endif /* INC_NAMI_HAL_INIT_H_ */
