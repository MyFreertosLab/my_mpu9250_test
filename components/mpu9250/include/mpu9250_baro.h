/*
 * mpu9250_baro.h
 *
 *  Created on: 30 apr 2022
 *      Author: andrea
 */

#ifndef COMPONENTS_DRIVERS_MPU9250_INCLUDE_MPU9250_BARO_H_
#define COMPONENTS_DRIVERS_MPU9250_INCLUDE_MPU9250_BARO_H_

#include <mpu9250.h>

#define BMP388_ADDRESS   0x76
#define BMP388_CHIP_ID   0x50

#define BMP388_REG_CHIP_ID 0x00


esp_err_t mpu9250_baro_init(mpu9250_handle_t mpu9250_handle);


#endif /* COMPONENTS_DRIVERS_MPU9250_INCLUDE_MPU9250_BARO_H_ */
