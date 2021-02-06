/*
 * mpu9250_accel.c
 *
 *  Created on: 6 feb 2021
 *      Author: andrea
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <mpu9250_spi.h>
#include <mpu9250_accel.h>
#include <math.h>

/************************************************************************
 ************** P R I V A T E  I M P L E M E N T A T I O N **************
 ************************************************************************/

static esp_err_t mpu9250_acc_save_fsr(mpu9250_handle_t mpu9250_handle) {
	uint8_t acc_conf = 0;
	uint8_t acc_conf_req = mpu9250_handle->acc_fsr;
    ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_ACCEL_CONFIG, &acc_conf));
    uint8_t toWrite = (acc_conf & MPU9250_ACC_FSR_MASK) | ((mpu9250_handle->acc_fsr << MPU9250_ACC_FSR_LBIT)& (~MPU9250_ACC_FSR_MASK));
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_ACCEL_CONFIG, toWrite));
    ESP_ERROR_CHECK(mpu9250_acc_load_fsr(mpu9250_handle));

	switch(mpu9250_handle->acc_fsr) {
	case INV_FSR_2G: {
		printf("MPU9250: AccFSR 2g\n");
		break;
	}
	case INV_FSR_4G: {
		printf("MPU9250: AccFSR 4g\n");
		break;
	}
	case INV_FSR_8G: {
		printf("MPU9250: AccFSR 8g\n");
		break;
	}
	case INV_FSR_16G: {
		printf("MPU9250: AccFSR 16g\n");
		break;
	}
	default: {
		printf("MPU9250: AccFSR UNKNOWN [%d]\n", mpu9250_handle->acc_fsr);
		break;
	}
	}
    return (mpu9250_handle->acc_fsr == acc_conf_req ? ESP_OK : ESP_FAIL);
}

static esp_err_t mpu9250_acc_calc_means(mpu9250_handle_t mpu9250_handle, int32_t* acc_means, uint8_t cycles) {
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );
	printf("Calculating Acc Means with %d000 samples (wait for %d seconds)... \n", cycles, cycles);
	acc_means[0] = 0;
	acc_means[1] = 0;
	acc_means[2] = 0;

	for(int j = 0; j < cycles; j++) {
		uint16_t max_samples = 1000;
		int64_t acc_sum[3] = {0,0,0};
		for(int i = 0; i < max_samples; i++) {
			ulTaskNotifyTake( pdTRUE,xMaxBlockTime );
			ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));
			acc_sum[0] += mpu9250_handle->raw_data.data_s_xyz.accel_data_x;
			acc_sum[1] += mpu9250_handle->raw_data.data_s_xyz.accel_data_y;
			acc_sum[2] += mpu9250_handle->raw_data.data_s_xyz.accel_data_z;
		}

		// offsets respect vertical attitude
		acc_means[0] += acc_sum[0]/max_samples;
		acc_means[1] += acc_sum[1]/max_samples;
		acc_means[2] += acc_sum[2]/max_samples;
	}
	acc_means[0] = acc_means[0]/cycles;
	acc_means[1] = acc_means[1]/cycles;
	acc_means[2] = acc_means[2]/cycles;
	return ESP_OK;
}

static esp_err_t mpu9250_acc_calc_sqm(mpu9250_handle_t mpu9250_handle, int32_t* acc_means, int16_t* acc_sqm, uint8_t cycles) {
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );

	printf("Calculating Acc sqm with %d000 samples (wait for %d seconds)... \n", cycles, cycles);
	acc_sqm[0] = 0.0f;
	acc_sqm[1] = 0.0f;
	acc_sqm[2] = 0.0f;

	for(int j = 0; j < cycles; j++) {
		uint16_t max_samples = 1000;
		int64_t acc_sum[3] = {0,0,0};
		printf("New SQM Cycle:[%d]\n", j);
		for(int i = 0; i < max_samples; i++) {
			ulTaskNotifyTake( pdTRUE,xMaxBlockTime );
			ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));
			acc_sum[0] += (mpu9250_handle->raw_data.data_s_xyz.accel_data_x - acc_means[0])*(mpu9250_handle->raw_data.data_s_xyz.accel_data_x - acc_means[0]);
			acc_sum[1] += (mpu9250_handle->raw_data.data_s_xyz.accel_data_y - acc_means[1])*(mpu9250_handle->raw_data.data_s_xyz.accel_data_y - acc_means[1]);
			acc_sum[2] += (mpu9250_handle->raw_data.data_s_xyz.accel_data_z - acc_means[2])*(mpu9250_handle->raw_data.data_s_xyz.accel_data_z - acc_means[2]);
//			if(i%100 == 0) {
//				printf("SQM: X(R[%d] M[%d] D[%d]) Y(R[%d] M[%d] D[%d]) Z(R[%d] M[%d] D[%d])\n",
//						mpu9250_handle->raw_data.data_s_xyz.accel_data_x,
//						acc_means[0],
//						(mpu9250_handle->raw_data.data_s_xyz.accel_data_x - acc_means[0]),
//						mpu9250_handle->raw_data.data_s_xyz.accel_data_y,
//						acc_means[1],
//						(mpu9250_handle->raw_data.data_s_xyz.accel_data_y - acc_means[1]),
//						mpu9250_handle->raw_data.data_s_xyz.accel_data_z,
//						acc_means[2],
//						(mpu9250_handle->raw_data.data_s_xyz.accel_data_z - acc_means[2])
//					  );
//			}
		}

		// offsets respect vertical attitude
		acc_sqm[0] += acc_sum[0]/max_samples;
		acc_sqm[1] += acc_sum[1]/max_samples;
		acc_sqm[2] += acc_sum[2]/max_samples;
	}

	acc_sqm[0] = sqrt(acc_sqm[0]/(cycles));
	acc_sqm[1] = sqrt(acc_sqm[1]/(cycles));
	acc_sqm[2] = sqrt(acc_sqm[2]/(cycles));
	printf("Acc_sqm: [%d][%d][%d]\n", acc_sqm[0], acc_sqm[1],acc_sqm[2]);

	return ESP_OK;
}

static esp_err_t mpu9250_acc_calc_bias(mpu9250_handle_t mpu9250_handle) {
	// Calc Bias
	{
		int32_t acc_means[3] = {0,0,0};
		printf("Calculating Acc SQM ... \n");
		memset(mpu9250_handle->acc_sqm[mpu9250_handle->acc_fsr].array, 0, sizeof(mpu9250_handle->acc_sqm[mpu9250_handle->acc_fsr].array));       //Zero out sqm
		acc_means[2] = mpu9250_handle->acc_lsb;
		ESP_ERROR_CHECK(mpu9250_acc_calc_sqm(mpu9250_handle, acc_means, mpu9250_handle->acc_sqm[mpu9250_handle->acc_fsr].array, 60));
		printf("Acc SQM: [%d][%d][%d]\n",
				mpu9250_handle->acc_sqm[mpu9250_handle->acc_fsr].array[0],
				mpu9250_handle->acc_sqm[mpu9250_handle->acc_fsr].array[1],
				mpu9250_handle->acc_sqm[mpu9250_handle->acc_fsr].array[2]);

	}
	return ESP_OK;
}

static esp_err_t mpu9250_acc_calc_biases(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_16G));
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 5000));
	ESP_ERROR_CHECK(mpu9250_acc_calc_bias(mpu9250_handle));

	ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_8G));
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 5000));
	ESP_ERROR_CHECK(mpu9250_acc_calc_bias(mpu9250_handle));

	ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_4G));
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 5000));
	ESP_ERROR_CHECK(mpu9250_acc_calc_bias(mpu9250_handle));

	ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_2G));
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 5000));
	ESP_ERROR_CHECK(mpu9250_acc_calc_bias(mpu9250_handle));

	return ESP_OK;
}

static esp_err_t mpu9250_acc_calc_lsb(mpu9250_handle_t mpu9250_handle) {
	mpu9250_handle->acc_lsb = (32768 >> (mpu9250_handle->acc_fsr + 1));
	return ESP_OK;
}


static esp_err_t mpu9250_acc_save_offset(mpu9250_handle_t mpu9250_handle) {
	uint8_t buff[8];
	buff[0] = mpu9250_handle->acc_offset.xyz.x >> 8;
	buff[1] = mpu9250_handle->acc_offset.xyz.x & 0x00FF;
	buff[2] = 0;
	buff[3] = mpu9250_handle->acc_offset.xyz.y >> 8;
	buff[4] = mpu9250_handle->acc_offset.xyz.y & 0x00FF;
	buff[5] = 0;
	buff[6] = mpu9250_handle->acc_offset.xyz.z >> 8;
	buff[7] = mpu9250_handle->acc_offset.xyz.z & 0x00FF;

	esp_err_t ret = mpu9250_write_buff(mpu9250_handle, MPU9250_XA_OFFSET_H, buff, 8*8);
	return ret;
}

static esp_err_t mpu9250_acc_calc_offset(mpu9250_handle_t mpu9250_handle) {
	uint8_t fsr_original = mpu9250_handle->acc_fsr;
	if(mpu9250_handle->acc_fsr != INV_FSR_16G) {
		mpu9250_handle->acc_fsr=INV_FSR_16G;
		ESP_ERROR_CHECK(mpu9250_acc_save_fsr(mpu9250_handle));
	}


	// Original Acc offsets: [6684][-5436][9684]
	ESP_ERROR_CHECK(mpu9250_acc_load_offset(mpu9250_handle));
	printf("Acc offsets: [%d][%d][%d]\n", mpu9250_handle->acc_offset.xyz.x, mpu9250_handle->acc_offset.xyz.y,mpu9250_handle->acc_offset.xyz.z);

	// dicard 10000 samples
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 10000));

	printf("Calculating Acc Offset ... \n");
	uint16_t max_means = 60;
	int32_t acc_means[3] = {0,0,0};
	ESP_ERROR_CHECK(mpu9250_acc_calc_means(mpu9250_handle, acc_means, max_means));

	mpu9250_handle->acc_offset.xyz.x -= acc_means[0];
	mpu9250_handle->acc_offset.xyz.y -= acc_means[1];
	mpu9250_handle->acc_offset.xyz.z -= (acc_means[2] - mpu9250_handle->acc_lsb);

	printf("Acc offsets: [%d][%d][%d]\n", mpu9250_handle->acc_offset.xyz.x, mpu9250_handle->acc_offset.xyz.y,mpu9250_handle->acc_offset.xyz.z);
	printf("Acc means: [%d][%d][%d]\n", acc_means[0], acc_means[1],acc_means[2]);

	ESP_ERROR_CHECK(mpu9250_acc_save_offset(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 5000));
	ESP_ERROR_CHECK(mpu9250_display_messages(mpu9250_handle, 1000));
	ESP_ERROR_CHECK(mpu9250_acc_calc_biases(mpu9250_handle));

	if(fsr_original != mpu9250_handle->acc_fsr) {
		ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, fsr_original));
		ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 5000));
	}
	return ESP_OK;
}

/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/

esp_err_t mpu9250_acc_load_fsr(mpu9250_handle_t mpu9250_handle) {
	uint8_t acc_conf = 0;
    ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_ACCEL_CONFIG, &acc_conf));
    mpu9250_handle->acc_fsr = (acc_conf & (~MPU9250_ACC_FSR_MASK)) >> MPU9250_ACC_FSR_LBIT;
    ESP_ERROR_CHECK(mpu9250_acc_calc_lsb(mpu9250_handle));
	return ESP_OK;
}

esp_err_t mpu9250_acc_set_fsr(mpu9250_handle_t mpu9250_handle, uint8_t fsr) {
	mpu9250_handle->acc_fsr=fsr;
	ESP_ERROR_CHECK(mpu9250_acc_save_fsr(mpu9250_handle));
	return ESP_OK;
}

esp_err_t mpu9250_acc_load_offset(mpu9250_handle_t mpu9250_handle) {
	uint8_t buff[8];
	esp_err_t ret = mpu9250_read_buff(mpu9250_handle, MPU9250_XA_OFFSET_H, buff, 8*8);
	mpu9250_handle->acc_offset.xyz.x = (buff[0] << 8) + buff[1];
	mpu9250_handle->acc_offset.xyz.y = (buff[3] << 8) + buff[4];
	mpu9250_handle->acc_offset.xyz.z = (buff[6] << 8) + buff[7];
	return ret;
}
esp_err_t mpu9250_acc_set_offset(mpu9250_handle_t mpu9250_handle, int16_t xoff, int16_t yoff, int16_t zoff) {
	mpu9250_handle->acc_offset.xyz.x = xoff;
	mpu9250_handle->acc_offset.xyz.y = yoff;
	mpu9250_handle->acc_offset.xyz.z = zoff;
	return mpu9250_acc_save_offset(mpu9250_handle);
}

esp_err_t mpu9250_acc_calibrate(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_acc_calc_offset(mpu9250_handle));
	return ESP_OK;
}

