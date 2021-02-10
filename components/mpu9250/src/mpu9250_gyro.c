/*
 * mpu9250_gyro.c
 *
 *  Created on: 6 feb 2021
 *      Author: andrea
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <mpu9250_spi.h>
#include <mpu9250_gyro.h>
#include <math.h>

/************************************************************************
 ************** P R I V A T E  I M P L E M E N T A T I O N **************
 ************************************************************************/

static esp_err_t mpu9250_gyro_save_fsr(mpu9250_handle_t mpu9250_handle) {
	uint8_t gyro_conf = 0;
	uint8_t gyro_conf_req = mpu9250_handle->gyro_fsr;
    ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_GYRO_CONFIG, &gyro_conf));
    uint8_t toWrite = (gyro_conf & MPU9250_GYRO_FSR_MASK) | ((mpu9250_handle->gyro_fsr << MPU9250_GYRO_FSR_LBIT)& (~MPU9250_GYRO_FSR_MASK));
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_GYRO_CONFIG, toWrite));
    ESP_ERROR_CHECK(mpu9250_gyro_load_fsr(mpu9250_handle));

	switch(mpu9250_handle->gyro_fsr) {
	case INV_FSR_250DPS: {
		printf("MPU9250: GyroFSR 250dps\n");
		break;
	}
	case INV_FSR_500DPS: {
		printf("MPU9250: GyroFSR 500dps\n");
		break;
	}
	case INV_FSR_1000DPS: {
		printf("MPU9250: GyroFSR 1000dps\n");
		break;
	}
	case INV_FSR_2000DPS: {
		printf("MPU9250: GyroFSR 2000dps\n");
		break;
	}
	default: {
		printf("MPU9250: GyroFSR UNKNOWN [%d]\n", mpu9250_handle->gyro_fsr);
		break;
	}
	}
    return (mpu9250_handle->gyro_fsr == gyro_conf_req ? ESP_OK : ESP_FAIL);
}

static esp_err_t mpu9250_gyro_calc_means(mpu9250_handle_t mpu9250_handle, int16_t* gyro_means, uint8_t cycles) {
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );
	printf("Calculating Gyro Means with %d000 samples (wait for %d seconds)... \n", cycles, cycles);
	gyro_means[0] = 0;
	gyro_means[1] = 0;
	gyro_means[2] = 0;

	int64_t gyro_means_64[3] = {0,0,0};
	for(int j = 0; j < cycles; j++) {
		uint16_t max_samples = 1000;
		int64_t gyro_sum[3] = {0,0,0};
		for(int i = 0; i < max_samples; i++) {
			ulTaskNotifyTake( pdTRUE,xMaxBlockTime );
			ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));
			gyro_sum[0] += mpu9250_handle->raw_data.data_s_xyz.gyro_data_x;
			gyro_sum[1] += mpu9250_handle->raw_data.data_s_xyz.gyro_data_y;
			gyro_sum[2] += mpu9250_handle->raw_data.data_s_xyz.gyro_data_z;
		}

		// offsets respect vertical attitude
		gyro_means_64[0] += gyro_sum[0]/max_samples;
		gyro_means_64[1] += gyro_sum[1]/max_samples;
		gyro_means_64[2] += gyro_sum[2]/max_samples;
	}
	gyro_means[0] = gyro_means_64[0]/cycles;
	gyro_means[1] = gyro_means_64[1]/cycles;
	gyro_means[2] = gyro_means_64[2]/cycles;
	return ESP_OK;
}

static esp_err_t mpu9250_gyro_calc_var(mpu9250_handle_t mpu9250_handle, int16_t* gyro_means, uint16_t* gyro_var, uint8_t cycles) {
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );

	printf("Calculating Gyro Var with %d000 samples (wait for %d seconds)... \n", cycles, cycles);
	gyro_var[0] = 0.0f;
	gyro_var[1] = 0.0f;
	gyro_var[2] = 0.0f;
	uint64_t gyro_var_64[3] = {0,0,0};

	for(int j = 0; j < cycles; j++) {
		uint16_t max_samples = 1000;
		int64_t gyro_sum[3] = {0,0,0};
		for(int i = 0; i < max_samples; i++) {
			ulTaskNotifyTake( pdTRUE,xMaxBlockTime );
			ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));
			gyro_sum[0] += (mpu9250_handle->raw_data.data_s_xyz.gyro_data_x - gyro_means[0])*(mpu9250_handle->raw_data.data_s_xyz.gyro_data_x - gyro_means[0]);
			gyro_sum[1] += (mpu9250_handle->raw_data.data_s_xyz.gyro_data_y - gyro_means[1])*(mpu9250_handle->raw_data.data_s_xyz.gyro_data_y - gyro_means[1]);
			gyro_sum[2] += (mpu9250_handle->raw_data.data_s_xyz.gyro_data_z - gyro_means[2])*(mpu9250_handle->raw_data.data_s_xyz.gyro_data_z - gyro_means[2]);
		}

		// offsets respect vertical attitude
		gyro_var_64[0] += gyro_sum[0]/max_samples;
		gyro_var_64[1] += gyro_sum[1]/max_samples;
		gyro_var_64[2] += gyro_sum[2]/max_samples;
	}

	gyro_var[0] = gyro_var_64[0]/(cycles);
	gyro_var[1] = gyro_var_64[1]/(cycles);
	gyro_var[2] = gyro_var_64[2]/(cycles);
	printf("Gyro_var: [%d][%d][%d]\n", gyro_var[0], gyro_var[1],gyro_var[2]);

	return ESP_OK;
}
static esp_err_t mpu9250_gyro_calc_sqm(mpu9250_handle_t mpu9250_handle, int16_t* gyro_means, int16_t* gyro_sqm, uint8_t cycles) {
	ESP_ERROR_CHECK(mpu9250_gyro_calc_var(mpu9250_handle, gyro_means, mpu9250_handle->gyro_var[mpu9250_handle->gyro_fsr].array, 60));
	gyro_sqm[0] = sqrt(mpu9250_handle->gyro_var[mpu9250_handle->gyro_fsr].xyz.x);
	gyro_sqm[1] = sqrt(mpu9250_handle->gyro_var[mpu9250_handle->gyro_fsr].xyz.y);
	gyro_sqm[2] = sqrt(mpu9250_handle->gyro_var[mpu9250_handle->gyro_fsr].xyz.z);

	printf("Gyro_sqm: [%d][%d][%d]\n", gyro_sqm[0], gyro_sqm[1],gyro_sqm[2]);

	return ESP_OK;
}

static esp_err_t mpu9250_gyro_calc_bias(mpu9250_handle_t mpu9250_handle) {
	// Calc Bias
	{
		int16_t gyro_means[3] = {0,0,0};
		printf("Calculating Gyro Bias ... \n");
		memset(mpu9250_handle->gyro_sqm[mpu9250_handle->gyro_fsr].array, 0, sizeof(mpu9250_handle->gyro_sqm[mpu9250_handle->gyro_fsr].array));       //Zero out sqm
		ESP_ERROR_CHECK(mpu9250_gyro_calc_sqm(mpu9250_handle, gyro_means, mpu9250_handle->gyro_sqm[mpu9250_handle->gyro_fsr].array, 60));
		printf("Gyro SQM: [%d][%d][%d]\n",
				mpu9250_handle->gyro_sqm[mpu9250_handle->gyro_fsr].array[0],
				mpu9250_handle->gyro_sqm[mpu9250_handle->gyro_fsr].array[1],
				mpu9250_handle->gyro_sqm[mpu9250_handle->gyro_fsr].array[2]);

	}
	return ESP_OK;
}

static esp_err_t mpu9250_gyro_calc_biases(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_gyro_set_fsr(mpu9250_handle, INV_FSR_250DPS));
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 10000));
	ESP_ERROR_CHECK(mpu9250_gyro_calc_bias(mpu9250_handle));

	ESP_ERROR_CHECK(mpu9250_gyro_set_fsr(mpu9250_handle, INV_FSR_500DPS));
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 10000));
	ESP_ERROR_CHECK(mpu9250_gyro_calc_bias(mpu9250_handle));

	ESP_ERROR_CHECK(mpu9250_gyro_set_fsr(mpu9250_handle, INV_FSR_1000DPS));
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 10000));
	ESP_ERROR_CHECK(mpu9250_gyro_calc_bias(mpu9250_handle));

	ESP_ERROR_CHECK(mpu9250_gyro_set_fsr(mpu9250_handle, INV_FSR_2000DPS));
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 10000));
	ESP_ERROR_CHECK(mpu9250_gyro_calc_bias(mpu9250_handle));

	return ESP_OK;
}

static esp_err_t mpu9250_gyro_calc_lsb(mpu9250_handle_t mpu9250_handle) {
	mpu9250_handle->gyro_lsb = (32768 >> (mpu9250_handle->gyro_fsr + 1));
	return ESP_OK;
}


static esp_err_t mpu9250_gyro_save_offset(mpu9250_handle_t mpu9250_handle) {
	uint8_t buff[8];
	buff[0] = mpu9250_handle->gyro_offset.xyz.x >> 8;
	buff[1] = mpu9250_handle->gyro_offset.xyz.x & 0x00FF;
	buff[2] = mpu9250_handle->gyro_offset.xyz.y >> 8;
	buff[3] = mpu9250_handle->gyro_offset.xyz.y & 0x00FF;
	buff[4] = mpu9250_handle->gyro_offset.xyz.z >> 8;
	buff[5] = mpu9250_handle->gyro_offset.xyz.z & 0x00FF;

	esp_err_t ret = mpu9250_write_buff(mpu9250_handle, MPU9250_XG_OFFSET_H, buff, 6*8);
	return ret;
}

static esp_err_t mpu9250_gyro_calc_offset(mpu9250_handle_t mpu9250_handle) {
	uint8_t fsr_original = mpu9250_handle->gyro_fsr;
	if(mpu9250_handle->gyro_fsr != INV_FSR_2000DPS) {
		mpu9250_handle->gyro_fsr=INV_FSR_2000DPS;
		ESP_ERROR_CHECK(mpu9250_gyro_save_fsr(mpu9250_handle));
	}


	// Original Gyro offsets: Gyro offsets: [455][-11][-17]]
	ESP_ERROR_CHECK(mpu9250_gyro_load_offset(mpu9250_handle));
	printf("Gyro offsets: [%d][%d][%d]\n", mpu9250_handle->gyro_offset.xyz.x, mpu9250_handle->gyro_offset.xyz.y,mpu9250_handle->gyro_offset.xyz.z);

	// dicard 10000 samples
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 10000));
	uint8_t found[3] = {0,0,0};
	int16_t offsets[3] = {0,0,0};
	while(true) {
		printf("Calculating Gyro Offset ... \n");
		uint16_t max_means = 60;
		int16_t gyro_means[3] = {0,0,0};
		ESP_ERROR_CHECK(mpu9250_gyro_calc_means(mpu9250_handle, gyro_means, max_means));

		if((found[X_POS] == 0)) {
			if(abs(gyro_means[X_POS]) == 0) {
				found[X_POS] = 1;
				offsets[X_POS] = mpu9250_handle->gyro_offset.xyz.x;
				printf("FOUND X OFFSET\n");
			}
			else {
				mpu9250_handle->gyro_offset.xyz.z -= gyro_means[X_POS];
			}
		} else {
			mpu9250_handle->gyro_offset.xyz.z = offsets[X_POS];
		}
		if((found[Y_POS] == 0)) {
			if(abs(gyro_means[Y_POS]) == 0) {
				found[Y_POS] = 1;
				offsets[Y_POS] = mpu9250_handle->gyro_offset.xyz.y;
				printf("FOUND Y OFFSET\n");
			}
			else {
				mpu9250_handle->gyro_offset.xyz.z -= gyro_means[Y_POS];
			}
		} else {
			mpu9250_handle->gyro_offset.xyz.z = offsets[Y_POS];
		}
		if((found[Z_POS] == 0)) {
			if(abs(gyro_means[Z_POS]) == 0) {
				found[Z_POS] = 1;
				offsets[Z_POS] = mpu9250_handle->gyro_offset.xyz.y;
				printf("FOUND Z OFFSET\n");
			}
			else {
				mpu9250_handle->gyro_offset.xyz.z -= gyro_means[Z_POS];
			}
		} else {
			mpu9250_handle->gyro_offset.xyz.z = offsets[Z_POS];
		}

		printf("Gyro offsets: [%d][%d][%d]\n", mpu9250_handle->gyro_offset.xyz.x, mpu9250_handle->gyro_offset.xyz.y,mpu9250_handle->gyro_offset.xyz.z);
		printf("Gyro means: [%d][%d][%d]\n", gyro_means[0], gyro_means[1],gyro_means[2]);

		if((found[0] == 1) && (found[1] == 1) && (found[2] == 1)) {
			break;
		}

		ESP_ERROR_CHECK(mpu9250_gyro_save_offset(mpu9250_handle));
		ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 10000));
		ESP_ERROR_CHECK(mpu9250_display_messages(mpu9250_handle, 1000));
	}
	ESP_ERROR_CHECK(mpu9250_gyro_calc_biases(mpu9250_handle));

	if(fsr_original != mpu9250_handle->gyro_fsr) {
		ESP_ERROR_CHECK(mpu9250_gyro_set_fsr(mpu9250_handle, fsr_original));
		ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 10000));
	}
	return ESP_OK;
}

/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/

esp_err_t mpu9250_gyro_load_fsr(mpu9250_handle_t mpu9250_handle) {
	uint8_t gyro_conf = 0;
    ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_GYRO_CONFIG, &gyro_conf));
    mpu9250_handle->gyro_fsr = (gyro_conf & (~MPU9250_GYRO_FSR_MASK)) >> MPU9250_GYRO_FSR_LBIT;
    ESP_ERROR_CHECK(mpu9250_gyro_calc_lsb(mpu9250_handle));
	return ESP_OK;
}

esp_err_t mpu9250_gyro_set_fsr(mpu9250_handle_t mpu9250_handle, uint8_t fsr) {
	mpu9250_handle->gyro_fsr=fsr;
	ESP_ERROR_CHECK(mpu9250_gyro_save_fsr(mpu9250_handle));
	return ESP_OK;
}

esp_err_t mpu9250_gyro_load_offset(mpu9250_handle_t mpu9250_handle) {
	uint8_t buff[6];
	esp_err_t ret = mpu9250_read_buff(mpu9250_handle, MPU9250_XG_OFFSET_H, buff, 6*8);
	mpu9250_handle->gyro_offset.xyz.x = (buff[0] << 8) + buff[1];
	mpu9250_handle->gyro_offset.xyz.y = (buff[2] << 8) + buff[3];
	mpu9250_handle->gyro_offset.xyz.z = (buff[4] << 8) + buff[5];
	return ret;
}
esp_err_t mpu9250_gyro_set_offset(mpu9250_handle_t mpu9250_handle, int16_t xoff, int16_t yoff, int16_t zoff) {
	mpu9250_handle->gyro_offset.xyz.x = xoff;
	mpu9250_handle->gyro_offset.xyz.y = yoff;
	mpu9250_handle->gyro_offset.xyz.z = zoff;
	return mpu9250_gyro_save_offset(mpu9250_handle);
}

esp_err_t mpu9250_gyro_calibrate(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_gyro_calc_offset(mpu9250_handle));
	return ESP_OK;
}
