/*
 * mpu9250_calibrator.h
 *
 *  Created on: 20 feb 2021
 *      Author: andrea
 */

#ifndef COMPONENTS_MPU9250_INCLUDE_MPU9250_CALIBRATOR_H_
#define COMPONENTS_MPU9250_INCLUDE_MPU9250_CALIBRATOR_H_
#include <mpu9250.h>

#define MPU9250_CAL_ACCEL_INDEX         0
#define MPU9250_CAL_GYRO_INDEX          1
#define MPU9250_CAL_MAX_KSAMPLE_CYCLES 10

/*********************************
******* CALIBRATOR HANDLE ********
*********************************/
typedef struct mpu9250_cal_s {
	mpu9250_handle_t mpu9250_handle;
	uint8_t found[2][3];

} mpu9250_cal_t;

typedef mpu9250_cal_t* mpu9250_cal_handle_t;

esp_err_t mpu9250_calibrate(mpu9250_cal_handle_t mpu9250_cal_handle);


#endif /* COMPONENTS_MPU9250_INCLUDE_MPU9250_CALIBRATOR_H_ */
