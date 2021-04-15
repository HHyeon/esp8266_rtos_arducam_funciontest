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

#include "driver/gpio.h"
#include "arducam_commun.h"

#include "driver/i2c.h"
#include "driver/spi.h"
#include "driver/gpio.h"



void arducam_camera_init()
{
	while(1)
	{
		
		uint8_t error = 0;
		uint8_t testdata[4] = {0xAA,0xBB,0xCC,0xDD};

		for(int i=0;i<4;i++)
		{
			spi_write_reg(0x00, testdata[i]);
			if(spi_read_reg(0x00) != testdata[i])
			{
				error = 1;
				break;
			}
		}
		
		if(error)
		{
			printf("spi test communication error...\n");
			vTaskDelay(1000 / portTICK_RATE_MS);
			continue;
		}
		
		vTaskDelay(100 / portTICK_RATE_MS);
		spi_write_reg(0x07,0x80);
		vTaskDelay(100 / portTICK_RATE_MS);
		spi_write_reg(0x07,0x00);
		vTaskDelay(100 / portTICK_RATE_MS);
		
		uint8_t sensoridH, sensoridL, txreg;
		txreg = 0x01;		
		
		i2c_write(0xff, &txreg, 1);		
		i2c_read(0x0A, &sensoridH, 1);
		i2c_read(0x0B, &sensoridL, 1);
		
		if(sensoridH!=0x26 || sensoridL!=0x42)
		{
			printf("cam sensor id not match...\n");
			vTaskDelay(1000 / portTICK_RATE_MS);
			continue;
		}
		
		arducam_sensor_default_init();
		
		break;

	}
}


void spi_periodic_sending_task(void *arg)
{
	while(1)
	{
		vTaskDelay(3000 / portTICK_RATE_MS);
	}
}


void app_main()
{
	printf("\n --------- esp8266 started ---------\n");
	
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 4;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = 2;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 300;

	i2c_driver_install(I2C_NUM_0, conf.mode);
	i2c_param_config(I2C_NUM_0, &conf);
	
	gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
	gpio_set_intr_type(GPIO_NUM_5, GPIO_INTR_DISABLE);
	gpio_set_pull_mode(GPIO_NUM_5, GPIO_PULLUP_ONLY);
	gpio_set_level(GPIO_NUM_5, 1);
	
	spi_config_t spi_config;
    spi_config.interface.val = SPI_DEFAULT_INTERFACE;
	spi_config.mode = SPI_MASTER_MODE;
	spi_config.clk_div = SPI_8MHz_DIV;
	spi_init(HSPI_HOST, &spi_config);
	
    SPI1C = 0;
    SPI1U = SPIUMOSI | SPIUDUPLEX | SPIUSSE;
    SPI1U1 = (7 << SPILMOSI) | (7 << SPILMISO);
    SPI1C1 = 0;
	setBitOrder(1);

    xTaskCreate(spi_periodic_sending_task, "spi_periodic_sending_task", 2048, NULL, 5, NULL);

	arducam_camera_init();
	printf("arducam_camera_init SUCCESSFUL\n");
	
	
	while(1)
	{
		printf("capture start\n");
		
		spi_write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
		spi_write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
		
		while( !(spi_read_reg(ARDUCHIP_TRIG) & CAP_DONE_MASK))
			vTaskDelay(50 / portTICK_RATE_MS);
		
		uint32_t buffered_fifosize=0;
		
		buffered_fifosize |= spi_read_reg(FIFO_SIZE3);
		buffered_fifosize <<= 8;
		buffered_fifosize |= spi_read_reg(FIFO_SIZE2);
		buffered_fifosize <<= 8;
		buffered_fifosize |= spi_read_reg(FIFO_SIZE1);
		
		printf("captured size : %d bytes\n", buffered_fifosize);
		
		CAM_CS_BEGIN;
		
		spi_transfer(BURST_FIFO_READ);
		
		uint8_t bytenow=0,bytepast=0;
		uint32_t imgsiz = 0;
		while(buffered_fifosize--)
		{
			bytenow = spi_transfer(0x00);
			imgsiz++;
			if(bytenow == 0xD9 && bytepast == 0xFF)
			{
				printf("FFD9 found\n");
				break;
			}		
			bytepast = bytenow;
		}
		CAM_CS_END;
		
		spi_write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
		
		
		printf("capture end img siz : %d\n", imgsiz);
		
		vTaskDelay(5000 / portTICK_RATE_MS);
	}
	
}
