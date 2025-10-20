#include "../../STM32_NaMi_protocol/Inc/nami_hal_init.h"
#include "../../STM32_NaMi_protocol/Vendor/WIZnet/wizchip_conf.h"

static SPI_HandleTypeDef* g_spi_handle = NULL;
static GPIO_TypeDef* g_cs_port = NULL;
static uint16_t g_cs_pin = 0;
static GPIO_TypeDef* g_rst_port = NULL;
static uint16_t g_rst_pin = 0;

static void nami_cs_select(void)
{
    HAL_GPIO_WritePin(g_cs_port, g_cs_pin, GPIO_PIN_RESET);
}

static void nami_cs_deselect(void)
{
    HAL_GPIO_WritePin(g_cs_port, g_cs_pin, GPIO_PIN_SET);
}

static void nami_spi_write_byte(uint8_t byte)
{
    HAL_SPI_Transmit(g_spi_handle, &byte, 1, HAL_MAX_DELAY);
}

static uint8_t nami_spi_read_byte(void)
{
    uint8_t byte;
    HAL_SPI_Receive(g_spi_handle, &byte, 1, HAL_MAX_DELAY);
    return byte;
}

void nami_hal_register_spi_callbacks(const NaMi_Hardware_Config* hw_config)
{
    g_spi_handle = hw_config->spi_handle;
    g_cs_port = hw_config->cs_port;
    g_cs_pin = hw_config->cs_pin;
    g_rst_port = hw_config->rst_port;
    g_rst_pin = hw_config->rst_pin;

    reg_wizchip_cs_cbfunc(nami_cs_select, nami_cs_deselect);
    reg_wizchip_spi_cbfunc(nami_spi_read_byte, nami_spi_write_byte);
}

void nami_hal_reset_w5500(const NaMi_Hardware_Config* hw_config)
{
    HAL_GPIO_WritePin(hw_config->rst_port, hw_config->rst_pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(hw_config->rst_port, hw_config->rst_pin, GPIO_PIN_SET);
    HAL_Delay(10);
}