/*
 * mpu9250_calibrator.c
 *
 *  Created on: 20 feb 2021
 *      Author: andrea
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <mpu9250_accel.h>
#include <mpu9250_gyro.h>
#include <mpu9250_calibrator.h>
#include <math.h>

static esp_err_t mpu9250_cal_discard_messages(mpu9250_handle_t mpu9250_handle, uint16_t num_msgs) {
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );
	printf("Discarding %d Samples ... \n", num_msgs);
	for(uint16_t i = 0; i < num_msgs; i++) {
		ulTaskNotifyTake( pdTRUE,xMaxBlockTime );
	}
	return ESP_OK;
}

static esp_err_t mpu9250_cal_display_messages(mpu9250_handle_t mpu9250_handle, uint16_t num_msgs) {
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );
	for(uint16_t i = 0; i < num_msgs; i++) {
		ulTaskNotifyTake( pdTRUE,xMaxBlockTime );
		ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));
		if(i%100 == 0) {
			printf("Acc [%d][%d][%d]\n",
					mpu9250_handle->raw_data.data_s_xyz.accel_data_x,
					mpu9250_handle->raw_data.data_s_xyz.accel_data_y,
					mpu9250_handle->raw_data.data_s_xyz.accel_data_z
				  );
			printf("Gyro [%d][%d][%d]\n",
					mpu9250_handle->raw_data.data_s_xyz.gyro_data_x,
					mpu9250_handle->raw_data.data_s_xyz.gyro_data_y,
					mpu9250_handle->raw_data.data_s_xyz.gyro_data_z
				  );
		}
	}
	return ESP_OK;
}

static esp_err_t mpu9250_cal_load_offset(mpu9250_cal_handle_t mpu9250_cal_handle) {
	ESP_ERROR_CHECK(mpu9250_acc_load_offset(mpu9250_cal_handle->mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_gyro_load_offset(mpu9250_cal_handle->mpu9250_handle));

	printf("Acc offsets: [%d][%d][%d]\n", mpu9250_cal_handle->mpu9250_handle->accel.cal.offset.xyz.x, mpu9250_cal_handle->mpu9250_handle->accel.cal.offset.xyz.y,mpu9250_cal_handle->mpu9250_handle->accel.cal.offset.xyz.z);
	printf("Gyro offsets: [%d][%d][%d]\n", mpu9250_cal_handle->mpu9250_handle->gyro.cal.offset.xyz.x, mpu9250_cal_handle->mpu9250_handle->gyro.cal.offset.xyz.y,mpu9250_cal_handle->mpu9250_handle->gyro.cal.offset.xyz.z);

	return ESP_OK;
}

static esp_err_t mpu9250_cal_init(mpu9250_cal_handle_t mpu9250_cal_handle) {
	ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_cal_handle->mpu9250_handle, mpu9250_cal_handle->mpu9250_handle->accel.fsr=INV_FSR_16G));
	ESP_ERROR_CHECK(mpu9250_gyro_set_fsr(mpu9250_cal_handle->mpu9250_handle, mpu9250_cal_handle->mpu9250_handle->gyro.fsr=INV_FSR_2000DPS));

	ESP_ERROR_CHECK(mpu9250_cal_load_offset(mpu9250_cal_handle));

	return ESP_OK;
}

static esp_err_t mpu9250_cal_calc_means(mpu9250_cal_handle_t mpu9250_cal_handle, uint8_t cycles) {
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );
	int64_t acc_sum[3] = {0,0,0};
	int64_t gyro_sum[3] = {0,0,0};

	for(uint8_t i = 0; i < 3; i++) {
		mpu9250_cal_handle->mpu9250_handle->accel.cal.means[mpu9250_cal_handle->mpu9250_handle->accel.fsr].array[i] = 0;
		mpu9250_cal_handle->mpu9250_handle->gyro.cal.means[mpu9250_cal_handle->mpu9250_handle->gyro.fsr].array[i] = 0;
	}

	printf("Calculating Means with %d000 samples (wait for %d seconds)... \n", cycles, cycles);
	for(int j = 0; j < cycles*1000; j++) {
			ulTaskNotifyTake( pdTRUE,xMaxBlockTime );
			ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_cal_handle->mpu9250_handle));
			for(uint8_t i = 0; i < 3; i++) {
				acc_sum[i] += mpu9250_cal_handle->mpu9250_handle->raw_data.data_s_vector.accel[i];
				gyro_sum[i] += mpu9250_cal_handle->mpu9250_handle->raw_data.data_s_vector.gyro[i];
			}
	}
	for(uint8_t i = 0; i < 3; i++) {
		mpu9250_cal_handle->mpu9250_handle->accel.cal.means[mpu9250_cal_handle->mpu9250_handle->accel.fsr].array[i] = acc_sum[i]/(cycles*1000);
		mpu9250_cal_handle->mpu9250_handle->gyro.cal.means[mpu9250_cal_handle->mpu9250_handle->gyro.fsr].array[i] = gyro_sum[i]/(cycles*1000);
	}
	return ESP_OK;
}

static esp_err_t mpu9250_cal_calc_var(mpu9250_cal_handle_t mpu9250_cal_handle, uint8_t cycles) {
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );

	uint64_t acc_sum[3] = {0,0,0};
	uint64_t gyro_sum[3] = {0,0,0};

	printf("Calculating Var with %d000 samples (wait for %d seconds)... \n", cycles, cycles);
	for(int j = 0; j < cycles*1000; j++) {
			ulTaskNotifyTake( pdTRUE,xMaxBlockTime );
			ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_cal_handle->mpu9250_handle));
			for(uint8_t i = 0; i < 3; i++) {
				acc_sum[i] += (mpu9250_cal_handle->mpu9250_handle->raw_data.data_s_vector.accel[i] - mpu9250_cal_handle->mpu9250_handle->accel.cal.means[mpu9250_cal_handle->mpu9250_handle->accel.fsr].array[i])*
						      (mpu9250_cal_handle->mpu9250_handle->raw_data.data_s_vector.accel[i] - mpu9250_cal_handle->mpu9250_handle->accel.cal.means[mpu9250_cal_handle->mpu9250_handle->accel.fsr].array[i]);
				gyro_sum[i] += (mpu9250_cal_handle->mpu9250_handle->raw_data.data_s_vector.gyro[i] - mpu9250_cal_handle->mpu9250_handle->gyro.cal.means[mpu9250_cal_handle->mpu9250_handle->gyro.fsr].array[i])*
					 	       (mpu9250_cal_handle->mpu9250_handle->raw_data.data_s_vector.gyro[i] - mpu9250_cal_handle->mpu9250_handle->gyro.cal.means[mpu9250_cal_handle->mpu9250_handle->gyro.fsr].array[i]);
			}
	}

	for(uint8_t i = 0; i < 3; i++) {
		mpu9250_cal_handle->mpu9250_handle->accel.cal.var[mpu9250_cal_handle->mpu9250_handle->accel.fsr].array[i] = (uint16_t)(acc_sum[i]/(cycles*1000));
		mpu9250_cal_handle->mpu9250_handle->gyro.cal.var[mpu9250_cal_handle->mpu9250_handle->gyro.fsr].array[i] = (uint16_t)(gyro_sum[i]/(cycles*1000));
	}
	printf("Acc_var: [%d][%d][%d]\n",
			mpu9250_cal_handle->mpu9250_handle->accel.cal.var[mpu9250_cal_handle->mpu9250_handle->accel.fsr].array[0],
			mpu9250_cal_handle->mpu9250_handle->accel.cal.var[mpu9250_cal_handle->mpu9250_handle->accel.fsr].array[1],
			mpu9250_cal_handle->mpu9250_handle->accel.cal.var[mpu9250_cal_handle->mpu9250_handle->accel.fsr].array[2]);
	printf("Gyro_var: [%d][%d][%d]\n",
			mpu9250_cal_handle->mpu9250_handle->gyro.cal.var[mpu9250_cal_handle->mpu9250_handle->gyro.fsr].array[0],
			mpu9250_cal_handle->mpu9250_handle->gyro.cal.var[mpu9250_cal_handle->mpu9250_handle->gyro.fsr].array[1],
			mpu9250_cal_handle->mpu9250_handle->gyro.cal.var[mpu9250_cal_handle->mpu9250_handle->gyro.fsr].array[2]);
	return ESP_OK;
}

static esp_err_t mpu9250_cal_calc_sqm(mpu9250_cal_handle_t mpu9250_cal_handle) {
	for(uint8_t i = 0; i < 3; i++) {
		mpu9250_cal_handle->mpu9250_handle->accel.cal.sqm[mpu9250_cal_handle->mpu9250_handle->accel.fsr].array[i] = sqrt(mpu9250_cal_handle->mpu9250_handle->accel.cal.var[mpu9250_cal_handle->mpu9250_handle->accel.fsr].array[i]);
		mpu9250_cal_handle->mpu9250_handle->gyro.cal.sqm[mpu9250_cal_handle->mpu9250_handle->gyro.fsr].array[i] = sqrt(mpu9250_cal_handle->mpu9250_handle->gyro.cal.var[mpu9250_cal_handle->mpu9250_handle->gyro.fsr].array[i]);
	}
	printf("Acc_sqm: [%d][%d][%d]\n",
			mpu9250_cal_handle->mpu9250_handle->accel.cal.sqm[mpu9250_cal_handle->mpu9250_handle->accel.fsr].array[0],
			mpu9250_cal_handle->mpu9250_handle->accel.cal.sqm[mpu9250_cal_handle->mpu9250_handle->accel.fsr].array[1],
			mpu9250_cal_handle->mpu9250_handle->accel.cal.sqm[mpu9250_cal_handle->mpu9250_handle->accel.fsr].array[2]);
	printf("Gyro_sqm: [%d][%d][%d]\n",
			mpu9250_cal_handle->mpu9250_handle->gyro.cal.sqm[mpu9250_cal_handle->mpu9250_handle->gyro.fsr].array[0],
			mpu9250_cal_handle->mpu9250_handle->gyro.cal.sqm[mpu9250_cal_handle->mpu9250_handle->gyro.fsr].array[1],
			mpu9250_cal_handle->mpu9250_handle->gyro.cal.sqm[mpu9250_cal_handle->mpu9250_handle->gyro.fsr].array[2]);
	return ESP_OK;
}

static esp_err_t mpu9250_cal_calc_bias(mpu9250_cal_handle_t mpu9250_cal_handle) {
	mpu9250_handle_t mpu9250_handle = mpu9250_cal_handle->mpu9250_handle;
	// Calc Bias
	{
		printf("Calculating Biases ... \n");

		memset(mpu9250_handle->accel.cal.var[mpu9250_handle->accel.fsr].array, 0, sizeof(mpu9250_handle->accel.cal.var[mpu9250_handle->accel.fsr].array));       //Zero out var
		memset(mpu9250_handle->accel.cal.sqm[mpu9250_handle->accel.fsr].array, 0, sizeof(mpu9250_handle->accel.cal.sqm[mpu9250_handle->accel.fsr].array));       //Zero out sqm
		memset(mpu9250_handle->gyro.cal.var[mpu9250_handle->gyro.fsr].array, 0, sizeof(mpu9250_handle->gyro.cal.var[mpu9250_handle->gyro.fsr].array));       //Zero out var
		memset(mpu9250_handle->gyro.cal.sqm[mpu9250_handle->gyro.fsr].array, 0, sizeof(mpu9250_handle->gyro.cal.sqm[mpu9250_handle->gyro.fsr].array));       //Zero out sqm

		ESP_ERROR_CHECK(mpu9250_cal_calc_var(mpu9250_cal_handle, MPU9250_CAL_MAX_KSAMPLE_CYCLES));
		ESP_ERROR_CHECK(mpu9250_cal_calc_sqm(mpu9250_cal_handle));

	}
	return ESP_OK;
}

static esp_err_t mpu9250_cal_calc_biases(mpu9250_cal_handle_t mpu9250_cal_handle) {
	mpu9250_handle_t mpu9250_handle = mpu9250_cal_handle->mpu9250_handle;
	ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_16G));
	ESP_ERROR_CHECK(mpu9250_gyro_set_fsr(mpu9250_handle, INV_FSR_2000DPS));
	ESP_ERROR_CHECK(mpu9250_cal_discard_messages(mpu9250_handle, 10000));
	ESP_ERROR_CHECK(mpu9250_cal_calc_bias(mpu9250_cal_handle));

	ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_8G));
	ESP_ERROR_CHECK(mpu9250_gyro_set_fsr(mpu9250_handle, INV_FSR_1000DPS));
	ESP_ERROR_CHECK(mpu9250_cal_discard_messages(mpu9250_handle, 10000));
	ESP_ERROR_CHECK(mpu9250_cal_calc_means(mpu9250_cal_handle, MPU9250_CAL_MAX_KSAMPLE_CYCLES));
	ESP_ERROR_CHECK(mpu9250_cal_calc_bias(mpu9250_cal_handle));

	ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_4G));
	ESP_ERROR_CHECK(mpu9250_gyro_set_fsr(mpu9250_handle, INV_FSR_500DPS));
	ESP_ERROR_CHECK(mpu9250_cal_discard_messages(mpu9250_handle, 10000));
	ESP_ERROR_CHECK(mpu9250_cal_calc_means(mpu9250_cal_handle, MPU9250_CAL_MAX_KSAMPLE_CYCLES));
	ESP_ERROR_CHECK(mpu9250_cal_calc_bias(mpu9250_cal_handle));

	ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_2G));
	ESP_ERROR_CHECK(mpu9250_gyro_set_fsr(mpu9250_handle, INV_FSR_250DPS));
	ESP_ERROR_CHECK(mpu9250_cal_discard_messages(mpu9250_handle, 10000));
	ESP_ERROR_CHECK(mpu9250_cal_calc_means(mpu9250_cal_handle, MPU9250_CAL_MAX_KSAMPLE_CYCLES));
	ESP_ERROR_CHECK(mpu9250_cal_calc_bias(mpu9250_cal_handle));

	return ESP_OK;
}

static esp_err_t mpu9250_cal_set_found_offset(uint8_t* found, uint8_t precision, int16_t relative_val, int16_t offset, int16_t* target) {
	if((*found == 0)) {
		if(abs(offset - relative_val) >> precision == 0) {
			*found = 1;
		}
		else {
			*target -= (offset - relative_val);
		}
	}
	return ESP_OK;
}
static esp_err_t mpu9250_cal_diaplay_status(mpu9250_cal_handle_t mpu9250_cal_handle) {
	printf("Accel Founds [%d,%d,%d]\n", mpu9250_cal_handle->found[MPU9250_CAL_ACCEL_INDEX][X_POS], mpu9250_cal_handle->found[MPU9250_CAL_ACCEL_INDEX][Y_POS], mpu9250_cal_handle->found[MPU9250_CAL_ACCEL_INDEX][Z_POS]);
	printf("Gyro  Founds [%d,%d,%d]\n", mpu9250_cal_handle->found[MPU9250_CAL_GYRO_INDEX][X_POS], mpu9250_cal_handle->found[MPU9250_CAL_GYRO_INDEX][Y_POS], mpu9250_cal_handle->found[MPU9250_CAL_GYRO_INDEX][Z_POS]);
	return ESP_OK;
}
static esp_err_t mpu9250_cal_calc_offset(mpu9250_cal_handle_t mpu9250_cal_handle) {
	mpu9250_handle_t mpu9250_handle = mpu9250_cal_handle->mpu9250_handle;
	memset(mpu9250_cal_handle->found, 0, sizeof(mpu9250_cal_handle->found));

	while(true) {
		ESP_ERROR_CHECK(mpu9250_cal_load_offset(mpu9250_cal_handle));

		printf("Calculating Offsets ... \n");
		ESP_ERROR_CHECK(mpu9250_cal_calc_means(mpu9250_cal_handle, MPU9250_CAL_MAX_KSAMPLE_CYCLES));

		for(uint8_t i = 0; i<3; i++) {
			ESP_ERROR_CHECK(mpu9250_cal_set_found_offset(&mpu9250_cal_handle->found[MPU9250_CAL_ACCEL_INDEX][i], 1, (i==Z_POS ? mpu9250_cal_handle->mpu9250_handle->accel.lsb : 0 ), mpu9250_cal_handle->mpu9250_handle->accel.cal.means[mpu9250_cal_handle->mpu9250_handle->accel.fsr].array[i], &mpu9250_cal_handle->mpu9250_handle->accel.cal.offset.array[i]));
			ESP_ERROR_CHECK(mpu9250_cal_set_found_offset(&mpu9250_cal_handle->found[MPU9250_CAL_GYRO_INDEX][i], 0, 0, mpu9250_cal_handle->mpu9250_handle->gyro.cal.means[mpu9250_cal_handle->mpu9250_handle->gyro.fsr].array[i], &mpu9250_cal_handle->mpu9250_handle->gyro.cal.offset.array[i]));
		}

		uint8_t found_all = 0;
		for(uint8_t i = 0; i < 2; i++) {
			for(uint8_t j = 0; j < 3; j++) {
				found_all += mpu9250_cal_handle->found[i][j];
			}
		}
		ESP_ERROR_CHECK(mpu9250_acc_set_offset(mpu9250_handle, mpu9250_handle->accel.cal.offset.xyz.x, mpu9250_handle->accel.cal.offset.xyz.y, mpu9250_handle->accel.cal.offset.xyz.z));
		ESP_ERROR_CHECK(mpu9250_gyro_set_offset(mpu9250_handle, mpu9250_handle->gyro.cal.offset.xyz.x, mpu9250_handle->gyro.cal.offset.xyz.y, mpu9250_handle->gyro.cal.offset.xyz.z));

		ESP_ERROR_CHECK(mpu9250_cal_discard_messages(mpu9250_handle, 10000));
		ESP_ERROR_CHECK(mpu9250_cal_display_messages(mpu9250_handle, 1000));
		ESP_ERROR_CHECK(mpu9250_cal_diaplay_status(mpu9250_cal_handle));

		if(found_all == 6) {
			break;
		}
	}
	return ESP_OK;
}

/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/
esp_err_t mpu9250_calibrate(mpu9250_cal_handle_t mpu9250_cal_handle) {
	ESP_ERROR_CHECK(mpu9250_cal_init(mpu9250_cal_handle));
	ESP_ERROR_CHECK(mpu9250_cal_discard_messages(mpu9250_cal_handle->mpu9250_handle, 10000));

	ESP_ERROR_CHECK(mpu9250_cal_calc_offset(mpu9250_cal_handle));
	ESP_ERROR_CHECK(mpu9250_cal_calc_biases(mpu9250_cal_handle));

	return ESP_OK;
}

