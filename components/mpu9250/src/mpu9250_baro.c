/*
 * mpu9250_baro.c
 *
 *  Created on: 30 apr 2022
 *      Author: andrea
 */

#include <mpu9250_spi.h>
#include <mpu9250_baro.h>
#include <driver/i2c.h>

/************************************************************************
 ************** P R I V A T E  I M P L E M E N T A T I O N **************
 ************************************************************************/
static esp_err_t WriteBmp388Register(mpu9250_handle_t mpu9250_handle, uint8_t reg,
		uint8_t data) {
	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV1_ADDR, BMP388_ADDRESS));
	ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV1_REG, reg));
	ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV1_DO, data));
	uint8_t slv_en = (I2C_SLV_EN | 0x01);
	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV1_CTRL, slv_en));
	return ESP_OK;
}

static esp_err_t ReadBmp388Register(mpu9250_handle_t mpu9250_handle, uint8_t reg) {
	uint8_t addr = (BMP388_ADDRESS | MPU9250_I2C_READ_FLAG);
	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV1_ADDR, addr));
	ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV1_REG, reg));
	uint8_t slv_en = (I2C_SLV_EN | 0x01);
	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV1_CTRL, slv_en));
	return ESP_OK;
}

static esp_err_t ReadBmp388Registers(mpu9250_handle_t mpu9250_handle, uint8_t reg, uint8_t count) {
	uint8_t addr = (BMP388_ADDRESS | MPU9250_I2C_READ_FLAG);
	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV1_ADDR, addr));
	ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV1_REG, reg));
	uint8_t slv_en = (I2C_SLV_EN | count);
	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV1_CTRL, slv_en));
	return ESP_OK;
}

static esp_err_t mpu9250_baro_prepare(mpu9250_handle_t mpu9250_handle) {

	return ESP_OK;
}

/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/
esp_err_t mpu9250_baro_test(mpu9250_handle_t mpu9250_handle) {
	uint8_t bmp388_whoAmI = 0x00;
	esp_err_t result = ReadBmp388Register(mpu9250_handle, BMP388_REG_CHIP_ID);
	if(result == ESP_OK) {
		vTaskDelay(pdMS_TO_TICKS(10));
		result = mpu9250_read8(mpu9250_handle, MPU9250_EXT_SENS_DATA_08,&bmp388_whoAmI);
		if(result == ESP_OK) {
			if (bmp388_whoAmI != BMP388_CHIP_ID) {
				printf("BMP388 Error reading chip ID: [%d]\n",bmp388_whoAmI);
				result = ESP_FAIL;
			} else {
				printf("BMP388 communication OK: [%d]\n",bmp388_whoAmI);
			}
		}
	}
	return result;
}

esp_err_t mpu9250_baro_init(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_baro_prepare(mpu9250_handle));
	return ESP_OK;
}
