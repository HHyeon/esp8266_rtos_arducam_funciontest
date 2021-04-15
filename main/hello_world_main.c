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




void spi_periodic_sending_task(void *arg)
{
	while(1)
	{
		vTaskDelay(3000 / portTICK_RATE_MS);
	}
}


static inline void arducam_camera_init(void)
{
	gpio_set_direction(ARDUCAM_CS_PIN, GPIO_MODE_OUTPUT);
	gpio_set_level(ARDUCAM_CS_PIN, 1);
	
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 4;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = 5;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 300;

	i2c_driver_install(I2C_NUM_0, conf.mode);
	i2c_param_config(I2C_NUM_0, &conf);
	
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

	uint8_t res;
	while((res=ardu_cam_init())!=ESP_OK)
	{
		if(res == 1)
			printf("spi test communication error...\n");
		else if(res == 2)
			printf("cam sensor id not match...\n");
			
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}



void app_main()
{
	printf("\n --------- esp8266 started ---------\n");
	gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_NUM_2, 1);
	arducam_camera_init();
	printf("arducam_camera_init SUCCESSFUL\n");

    xTaskCreate(spi_periodic_sending_task, "spi_periodic_sending_task", 2048, NULL, 5, NULL);
	
	while(1)
	{
		printf("capture start\n");
		gpio_set_level(GPIO_NUM_2, 0);
		
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
		
		
		gpio_set_level(GPIO_NUM_2, 1);
		printf("capture end img siz : %d\n", imgsiz);
		
		vTaskDelay(5000 / portTICK_RATE_MS);
	}
	
}
