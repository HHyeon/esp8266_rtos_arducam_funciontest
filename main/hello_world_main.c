/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "string.h"

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/spi.h"
#include "esp8266_peri.h"


#define I2C_DEV_ADDR 0x68

esp_err_t i2c_write(uint8_t reg, uint8_t *data, size_t len)
{
	int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, I2C_DEV_ADDR << 1 | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, reg, 1);
    i2c_master_write(cmd, data, len, 1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

	return ret;
}

esp_err_t i2c_read(uint8_t reg, uint8_t *data, size_t len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, I2C_DEV_ADDR << 1 | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, reg, 1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, I2C_DEV_ADDR << 1 | I2C_MASTER_READ, 11);
    i2c_master_read(cmd, data, len, 2); // nack
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}


inline void setDataBits(uint16_t bits) {
    const uint32_t mask = ~((SPIMMOSI << SPILMOSI) | (SPIMMISO << SPILMISO));
    bits--;
    SPI1U1 = ((SPI1U1 & mask) | ((bits << SPILMOSI) | (bits << SPILMISO)));
}


uint8_t spi_transfer(uint8_t data) {
    while(SPI1CMD & SPIBUSY) {}
    // reset to 8Bit mode
    setDataBits(8);
    SPI1W0 = data;
    SPI1CMD |= SPIBUSY;
    while(SPI1CMD & SPIBUSY) {}
    return (uint8_t) (SPI1W0 & 0xff);
}

void spi_write_reg(uint8_t addr, uint8_t data)
{
	gpio_set_level(GPIO_NUM_5, 0);

	spi_transfer(addr|0x80);
	spi_transfer(data);
	
	gpio_set_level(GPIO_NUM_5, 1);
}


uint8_t spi_read_reg(uint8_t addr)
{
	gpio_set_level(GPIO_NUM_5, 0);

	spi_transfer(addr);
	uint8_t result = spi_transfer(0x00);
	
	gpio_set_level(GPIO_NUM_5, 1);

	return result;
}


void setBitOrder(uint8_t bitOrder) {
	if(bitOrder == 1/*MSBFIRST*/) {
	    SPI1C &= ~(SPICWBO | SPICRBO);
	} else {
	    SPI1C |= (SPICWBO | SPICRBO);
	}
}






void IRAM_ATTR spi_periodic_sending_task(void *arg)
{
	while(1)
	{
		static uint32_t level=0;
		gpio_set_level(GPIO_NUM_2, level^=1);

		uint8_t testdata[4] = {0xAA,0xBB,0xCC,0xDD};

		uint8_t res;

		for(int i=0;i<4;i++)
		{
			spi_write_reg(0x00, testdata[i]);
			res = spi_read_reg(0x00);
			printf("send 0x%x recv 0x%x - %s\n", testdata[i], res, res == testdata[i] ? "SUCCESS" : "FAILED");
		}

		
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}


void app_main()
{
	gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
	gpio_set_intr_type(GPIO_NUM_5, GPIO_INTR_DISABLE);
	gpio_set_pull_mode(GPIO_NUM_5, GPIO_PULLUP_ONLY);
	gpio_set_level(GPIO_NUM_5, 1);
	gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
	gpio_set_intr_type(GPIO_NUM_2, GPIO_INTR_DISABLE);
	gpio_set_pull_mode(GPIO_NUM_2, GPIO_FLOATING);

	spi_config_t spi_config;
    spi_config.interface.val = SPI_DEFAULT_INTERFACE;
	spi_config.mode = SPI_MASTER_MODE;
	spi_config.clk_div =SPI_2MHz_DIV;
	spi_init(HSPI_HOST, &spi_config);
	
    SPI1C = 0;
    SPI1U = SPIUMOSI | SPIUDUPLEX | SPIUSSE;
    SPI1U1 = (7 << SPILMOSI) | (7 << SPILMISO);
    SPI1C1 = 0;
	

	setBitOrder(1);
	
    xTaskCreate(spi_periodic_sending_task, "spi_periodic_sending_task", 4096, NULL, 3, NULL);

	return;

    int i2c_master_port = I2C_NUM_0;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 4;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = 2;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 300;

	ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
	ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
	
	uint8_t regval;
	while(1)
	{
		if(i2c_write(0x6B, &regval, 1) == ESP_OK && 
			i2c_read(0x75, &regval, 1) == ESP_OK)
		{
			printf("whoami - 0x%02x\n", regval);
		}
		else
		{
			printf("read failed\n");
		}

		vTaskDelay(3000 / portTICK_RATE_MS);
	}
}
