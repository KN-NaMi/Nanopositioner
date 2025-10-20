#ifndef INC_NAMI_PROTOCOL_H_
#define INC_NAMI_PROTOCOL_H_

#include "main.h"
#include "cJSON.h"

typedef struct {
    SPI_HandleTypeDef* spi_handle;
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef* rst_port;
    uint16_t rst_pin;
} NaMi_Hardware_Config;

typedef struct {
    const char* device_id;
    const char* type;
    const char* version;
    const char* software_version;
    uint16_t tcp_port;
} NaMi_Device_Info;

typedef struct {
    uint8_t mac[6];
    uint8_t ip[4];
    uint8_t subnet_mask[4];
    uint8_t gateway[4];
} NaMi_Network_Config;

typedef void (*NaMi_CommandHandler)(cJSON *command, char* session_id);

int8_t NaMi_Init(
    const NaMi_Hardware_Config* hw_config,
    const NaMi_Network_Config* net_config,
    const NaMi_Device_Info* dev_info,
    NaMi_CommandHandler cmd_handler
);

void NaMi_Process(void);

#endif /* INC_NAMI_PROTOCOL_H_ */