
#include "arducam_commun.h"

#include "ov2640_regs.h"

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


void i2c_writes(const struct sensor_reg reglist[])
{
  uint8_t reg_addr = 0;
  uint8_t reg_val = 0;
  const struct sensor_reg *next = reglist;
  while ((reg_addr != 0xff) | (reg_val != 0xff))
  {
    reg_addr = next->reg;
    reg_val = next->val;
    i2c_write(reg_addr, &reg_val, 1);
    next++;
  }
}

void arducam_sensor_default_init()
{
	uint8_t data;
	
	data = 0x01;
	i2c_write(0xff, &data, 1);
	data = 0x80;
	i2c_write(0x12, &data, 1);
	vTaskDelay(100 / portTICK_RATE_MS);
	// printf("OV2640_JPEG_INIT\n");
	i2c_writes(OV2640_JPEG_INIT);
	// printf("OV2640_YUV422\n");
	i2c_writes(OV2640_YUV422);
	// printf("OV2640_JPEG\n");
	i2c_writes(OV2640_JPEG);
	data = 0x01;
	i2c_write(0xff, &data, 1);
	data = 0x00;
	i2c_write(0x15, &data, 1);
	// printf("OV2640_1600x1200_JPEG\n");
	i2c_writes(OV2640_1600x1200_JPEG);
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
	CAM_CS_BEGIN;

	spi_transfer(addr|0x80);
	spi_transfer(data);
	
	CAM_CS_END;
}


uint8_t spi_read_reg(uint8_t addr)
{
	CAM_CS_BEGIN;

	spi_transfer(addr);
	uint8_t result = spi_transfer(0x00);
	
	CAM_CS_END;

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


