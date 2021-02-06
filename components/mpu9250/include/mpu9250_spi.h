/*
 * mpu9250_spi.h
 *
 *  Created on: 6 feb 2021
 *      Author: andrea
 */

#ifndef COMPONENTS_MPU9250_INCLUDE_MPU9250_SPI_H_
#define COMPONENTS_MPU9250_INCLUDE_MPU9250_SPI_H_
#include <mpu9250.h>

esp_err_t mpu9250_read8(mpu9250_handle_t mpu9250_handle, uint8_t reg, uint8_t* val);
esp_err_t mpu9250_write8(mpu9250_handle_t mpu9250_handle, uint8_t reg, uint8_t val);
esp_err_t mpu9250_read_buff(mpu9250_handle_t mpu9250_handle, uint8_t reg, void* buff, uint8_t length);
esp_err_t mpu9250_read_buff(mpu9250_handle_t mpu9250_handle, uint8_t reg, void* buff, uint8_t length);
esp_err_t mpu9250_write_buff(mpu9250_handle_t mpu9250_handle, uint8_t reg, void* buff, uint8_t length);
esp_err_t mpu9250_spi_init(mpu9250_handle_t mpu9250_handle);

#endif /* COMPONENTS_MPU9250_INCLUDE_MPU9250_SPI_H_ */
