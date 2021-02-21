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
	uint8_t gyro_conf_req = mpu9250_handle->gyro.fsr;
    ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_GYRO_CONFIG, &gyro_conf));
    uint8_t toWrite = (gyro_conf & MPU9250_GYRO_FSR_MASK) | ((mpu9250_handle->gyro.fsr << MPU9250_GYRO_FSR_LBIT)& (~MPU9250_GYRO_FSR_MASK));
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_GYRO_CONFIG, toWrite));
    ESP_ERROR_CHECK(mpu9250_gyro_load_fsr(mpu9250_handle));

	switch(mpu9250_handle->gyro.fsr) {
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
		printf("MPU9250: GyroFSR UNKNOWN [%d]\n", mpu9250_handle->gyro.fsr);
		break;
	}
	}
    return (mpu9250_handle->gyro.fsr == gyro_conf_req ? ESP_OK : ESP_FAIL);
}

static esp_err_t mpu9250_gyro_init_kalman_filter(mpu9250_handle_t mpu9250_handle) {
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );

	for(uint8_t i = 0; i < 3; i++) {
		mpu9250_handle->gyro.cal.kalman[i].X = mpu9250_handle->gyro.lsb;
		mpu9250_handle->gyro.cal.kalman[i].sample=0;
		mpu9250_handle->gyro.cal.kalman[i].P=1.0f;
		mpu9250_handle->gyro.cal.kalman[i].Q=1.5;
		mpu9250_handle->gyro.cal.kalman[i].K=0.0f;
		mpu9250_handle->gyro.cal.kalman[i].R=mpu9250_handle->gyro.cal.var[mpu9250_handle->gyro.fsr].array[i];
	}
	return ESP_OK;
}

static esp_err_t mpu9250_gyro_calc_lsb(mpu9250_handle_t mpu9250_handle) {
	switch(mpu9250_handle->gyro.fsr) {
	case(INV_FSR_2000DPS): {
		mpu9250_handle->gyro.lsb = 16.384;
		break;
	}
	case(INV_FSR_1000DPS): {
		mpu9250_handle->gyro.lsb = 32.768;
		break;
	}
	case(INV_FSR_500DPS): {
		mpu9250_handle->gyro.lsb = 65.536;
		break;
	}
	case(INV_FSR_250DPS): {
		mpu9250_handle->gyro.lsb = 131.072;
		break;
	}
	}
	return ESP_OK;
}


static esp_err_t mpu9250_gyro_save_offset(mpu9250_handle_t mpu9250_handle) {
	uint8_t buff[8];
	buff[0] = mpu9250_handle->gyro.cal.offset.xyz.x >> 8;
	buff[1] = mpu9250_handle->gyro.cal.offset.xyz.x & 0x00FF;
	buff[2] = mpu9250_handle->gyro.cal.offset.xyz.y >> 8;
	buff[3] = mpu9250_handle->gyro.cal.offset.xyz.y & 0x00FF;
	buff[4] = mpu9250_handle->gyro.cal.offset.xyz.z >> 8;
	buff[5] = mpu9250_handle->gyro.cal.offset.xyz.z & 0x00FF;

	esp_err_t ret = mpu9250_write_buff(mpu9250_handle, MPU9250_XG_OFFSET_H, buff, 6*8);
	return ret;
}

static esp_err_t mpu9250_gyro_load_statistics(mpu9250_handle_t mpu9250_handle) {
	mpu9250_handle->gyro.cal.offset.array[X_POS]=358;
	mpu9250_handle->gyro.cal.offset.array[Y_POS]=-35;
	mpu9250_handle->gyro.cal.offset.array[Z_POS]=-15;

	mpu9250_handle->gyro.cal.var[INV_FSR_250DPS].array[X_POS]=196;
	mpu9250_handle->gyro.cal.var[INV_FSR_250DPS].array[Y_POS]=215;
	mpu9250_handle->gyro.cal.var[INV_FSR_250DPS].array[Z_POS]=305;
	mpu9250_handle->gyro.cal.sqm[INV_FSR_250DPS].array[X_POS]=14;
	mpu9250_handle->gyro.cal.sqm[INV_FSR_250DPS].array[Y_POS]=14;
	mpu9250_handle->gyro.cal.sqm[INV_FSR_250DPS].array[Z_POS]=17;

	mpu9250_handle->gyro.cal.var[INV_FSR_500DPS].array[X_POS]=51;
	mpu9250_handle->gyro.cal.var[INV_FSR_500DPS].array[Y_POS]=48;
	mpu9250_handle->gyro.cal.var[INV_FSR_500DPS].array[Z_POS]=68;
	mpu9250_handle->gyro.cal.sqm[INV_FSR_500DPS].array[X_POS]=7;
	mpu9250_handle->gyro.cal.sqm[INV_FSR_500DPS].array[Y_POS]=6;
	mpu9250_handle->gyro.cal.sqm[INV_FSR_500DPS].array[Z_POS]=8;

	mpu9250_handle->gyro.cal.var[INV_FSR_1000DPS].array[X_POS]=12;
	mpu9250_handle->gyro.cal.var[INV_FSR_1000DPS].array[Y_POS]=11;
	mpu9250_handle->gyro.cal.var[INV_FSR_1000DPS].array[Z_POS]=15;
	mpu9250_handle->gyro.cal.sqm[INV_FSR_1000DPS].array[X_POS]=3;
	mpu9250_handle->gyro.cal.sqm[INV_FSR_1000DPS].array[Y_POS]=3;
	mpu9250_handle->gyro.cal.sqm[INV_FSR_1000DPS].array[Z_POS]=3;

	mpu9250_handle->gyro.cal.var[INV_FSR_2000DPS].array[X_POS]=3;
	mpu9250_handle->gyro.cal.var[INV_FSR_2000DPS].array[Y_POS]=2;
	mpu9250_handle->gyro.cal.var[INV_FSR_2000DPS].array[Z_POS]=2;
	mpu9250_handle->gyro.cal.sqm[INV_FSR_2000DPS].array[X_POS]=1;
	mpu9250_handle->gyro.cal.sqm[INV_FSR_2000DPS].array[Y_POS]=1;
	mpu9250_handle->gyro.cal.sqm[INV_FSR_2000DPS].array[Z_POS]=1;

	return ESP_OK;
}

static esp_err_t mpu9250_gyro_filter_data(mpu9250_handle_t mpu9250_handle) {
	for(uint8_t i = 0; i < 3; i++) {
		if(mpu9250_handle->gyro.cal.kalman[i].P > 0.01) {
			mpu9250_cb_means(&mpu9250_handle->gyro.cb[i], &mpu9250_handle->gyro.cal.kalman[i].sample);
			mpu9250_handle->gyro.cal.kalman[i].P = mpu9250_handle->gyro.cal.kalman[i].P+mpu9250_handle->gyro.cal.kalman[i].Q;
			mpu9250_handle->gyro.cal.kalman[i].K = mpu9250_handle->gyro.cal.kalman[i].P/(mpu9250_handle->gyro.cal.kalman[i].P+mpu9250_handle->gyro.cal.kalman[i].R);
			mpu9250_handle->gyro.cal.kalman[i].X = mpu9250_handle->gyro.cal.kalman[i].X + mpu9250_handle->gyro.cal.kalman[i].K*(mpu9250_handle->gyro.cal.kalman[i].sample - mpu9250_handle->gyro.cal.kalman[i].X);
			mpu9250_handle->gyro.cal.kalman[i].P=(1-mpu9250_handle->gyro.cal.kalman[i].K)*mpu9250_handle->gyro.cal.kalman[i].P;
		}
	}
	return ESP_OK;
}

static esp_err_t mpu9250_gyro_calc_rpy(mpu9250_handle_t mpu9250_handle) {
	// angolo di rotazione: w(i)=domega(i)*dt espresso in rad
	double w[3] = {0.0f,0.0f,0.0f};
	for(uint8_t i = 0; i < 3; i++) {
		w[i] = (double)(mpu9250_handle->gyro.cal.kalman[i].X)/(double)mpu9250_handle->gyro.lsb/(double)1000.0f/(double)360.0f*(double)PI_2;
	}

	mpu9250_handle->gyro.rpy.xyz.x += w[X_POS];
	mpu9250_handle->gyro.rpy.xyz.y += w[Y_POS];
	mpu9250_handle->gyro.rpy.xyz.z += w[Z_POS];

	return ESP_OK;
}

/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/
esp_err_t mpu9250_gyro_init(mpu9250_handle_t mpu9250_handle) {
	mpu9250_gyro_load_statistics(mpu9250_handle); // set offset, var, sqm, P, K
	mpu9250_gyro_save_offset(mpu9250_handle);
	mpu9250_gyro_init_kalman_filter(mpu9250_handle); // this reset P,K
	mpu9250_handle->gyro.rpy.xyz.x = 0;
	mpu9250_handle->gyro.rpy.xyz.y = 0;
	mpu9250_handle->gyro.rpy.xyz.z = 0;
	return ESP_OK;
}

esp_err_t mpu9250_gyro_load_fsr(mpu9250_handle_t mpu9250_handle) {
	uint8_t gyro_conf = 0;
    ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_GYRO_CONFIG, &gyro_conf));
    mpu9250_handle->gyro.fsr = (gyro_conf & (~MPU9250_GYRO_FSR_MASK)) >> MPU9250_GYRO_FSR_LBIT;
    ESP_ERROR_CHECK(mpu9250_gyro_calc_lsb(mpu9250_handle));
	return ESP_OK;
}

esp_err_t mpu9250_gyro_set_fsr(mpu9250_handle_t mpu9250_handle, uint8_t fsr) {
	mpu9250_handle->gyro.fsr=fsr;
	ESP_ERROR_CHECK(mpu9250_gyro_save_fsr(mpu9250_handle));
	return ESP_OK;
}

esp_err_t mpu9250_gyro_load_offset(mpu9250_handle_t mpu9250_handle) {
	uint8_t buff[6];
	esp_err_t ret = mpu9250_read_buff(mpu9250_handle, MPU9250_XG_OFFSET_H, buff, 6*8);
	mpu9250_handle->gyro.cal.offset.xyz.x = (buff[0] << 8) + buff[1];
	mpu9250_handle->gyro.cal.offset.xyz.y = (buff[2] << 8) + buff[3];
	mpu9250_handle->gyro.cal.offset.xyz.z = (buff[4] << 8) + buff[5];
	return ret;
}
esp_err_t mpu9250_gyro_set_offset(mpu9250_handle_t mpu9250_handle, int16_t xoff, int16_t yoff, int16_t zoff) {
	mpu9250_handle->gyro.cal.offset.xyz.x = xoff;
	mpu9250_handle->gyro.cal.offset.xyz.y = yoff;
	mpu9250_handle->gyro.cal.offset.xyz.z = zoff;
	return mpu9250_gyro_save_offset(mpu9250_handle);
}

esp_err_t mpu9250_gyro_update_state(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_gyro_filter_data(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_gyro_calc_rpy(mpu9250_handle));
	return ESP_OK;
}

