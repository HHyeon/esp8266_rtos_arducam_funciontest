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

static esp_err_t spi_master_write_byte(uint8_t *data)
{
	spi_trans_t trans = {0};
	uint32_t buf[16];
    uint16_t cmd = 0;
    uint32_t addr = 0;

	for(uint8_t i=0;i<16;i++)
		buf[i] = 0x12345678;
	
    trans.mosi = buf;
    trans.bits.mosi = 8*64;
    trans.bits.addr = 0;	
	trans.cmd = &cmd;
	trans.addr = &addr;

	spi_trans(HSPI_HOST, &trans);

	return ESP_OK;
}


void spi_periodic_sending_task(void *arg)
{
	while(1)
	{
		static uint32_t level=0;
		gpio_set_level(GPIO_NUM_2, level^=1);

		uint8_t data[4] = {0xAA, 0xBB, 0xCC, 0xDD};
		
		spi_master_write_byte(data);
		
		vTaskDelay(500 / portTICK_RATE_MS);
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
    spi_config.intr_enable.val = SPI_MASTER_DEFAULT_INTR_ENABLE;
	spi_config.mode = SPI_MASTER_MODE;
	spi_config.clk_div =SPI_5MHz_DIV;
	spi_config.event_cb = NULL;
	spi_init(HSPI_HOST, &spi_config);
	
    xTaskCreate(spi_periodic_sending_task, "spi_periodic_sending_task", 4096, NULL, 5, NULL);

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
