
#include "arducam_commun.h"

#include "driver/i2c.h"
#include "driver/spi.h"
#include "driver/gpio.h"
#include "esp8266_peri.h"

#define I2C_DEV_ADDR 0x30

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


inline void setDataBits(uint16_t bits)
{
    const uint32_t mask = ~((SPIMMOSI << SPILMOSI) | (SPIMMISO << SPILMISO));
    bits--;
    SPI1U1 = ((SPI1U1 & mask) | ((bits << SPILMOSI) | (bits << SPILMISO)));
}


uint8_t spi_transfer(uint8_t data)
{
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


void setBitOrder(uint8_t bitOrder)
{
	if(bitOrder == 1/*MSBFIRST*/) {
	    SPI1C &= ~(SPICWBO | SPICRBO);
	} else {
	    SPI1C |= (SPICWBO | SPICRBO);
	}
}


