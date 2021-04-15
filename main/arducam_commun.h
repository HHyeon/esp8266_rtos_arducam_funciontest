

#ifndef ___ARDUCAM_COMMUN__H__
#define ___ARDUCAM_COMMUN__H__


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_system.h"

esp_err_t i2c_write(uint8_t reg, uint8_t *data, size_t len);
esp_err_t i2c_read(uint8_t reg, uint8_t *data, size_t len);

void spi_write_reg(uint8_t addr, uint8_t data);
uint8_t spi_read_reg(uint8_t addr);
void setBitOrder(uint8_t bitOrder);

#endif