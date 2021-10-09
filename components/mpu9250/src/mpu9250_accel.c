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
#include "esp_system.h"
#include <nvs_flash.h>
#include <nvs.h>

/************************************************************************
 ************** P R I V A T E  I M P L E M E N T A T I O N **************
 ************************************************************************/

static esp_err_t mpu9250_acc_save_fsr(mpu9250_handle_t mpu9250_handle) {
	uint8_t acc_conf = 0;
	uint8_t acc_conf_req = mpu9250_handle->accel.fsr;
    ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_ACCEL_CONFIG, &acc_conf));
    uint8_t toWrite = (acc_conf & MPU9250_ACC_FSR_MASK) | ((mpu9250_handle->accel.fsr << MPU9250_ACC_FSR_LBIT)& (~MPU9250_ACC_FSR_MASK));
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_ACCEL_CONFIG, toWrite));
    ESP_ERROR_CHECK(mpu9250_acc_load_fsr(mpu9250_handle));

	switch(mpu9250_handle->accel.fsr) {
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
		printf("MPU9250: AccFSR UNKNOWN [%d]\n", mpu9250_handle->accel.fsr);
		break;
	}
	}
    return (mpu9250_handle->accel.fsr == acc_conf_req ? ESP_OK : ESP_FAIL);
}

static esp_err_t mpu9250_acc_init_kalman_filter(mpu9250_handle_t mpu9250_handle) {
	for(uint8_t i = 0; i < 3; i++) {
		mpu9250_handle->accel.cal.kalman[i].X = mpu9250_handle->accel.lsb;
		mpu9250_handle->accel.cal.kalman[i].sample=0;
		mpu9250_handle->accel.cal.kalman[i].P=1.0f;
		mpu9250_handle->accel.cal.kalman[i].Q=1.5;
		mpu9250_handle->accel.cal.kalman[i].K=0.0f;
		mpu9250_handle->accel.cal.kalman[i].R=mpu9250_handle->accel.cal.var[mpu9250_handle->accel.fsr].array[i];
	}
	return ESP_OK;
}

static esp_err_t mpu9250_acc_calc_lsb(mpu9250_handle_t mpu9250_handle) {
	mpu9250_handle->accel.lsb = (32768 >> (mpu9250_handle->accel.fsr + 1));
	return ESP_OK;
}


static esp_err_t mpu9250_acc_save_offset(mpu9250_handle_t mpu9250_handle) {
	uint8_t buff[8];
	buff[0] = mpu9250_handle->accel.cal.offset.xyz.x >> 8;
	buff[1] = mpu9250_handle->accel.cal.offset.xyz.x & 0x00FF;
	buff[2] = 0;
	buff[3] = mpu9250_handle->accel.cal.offset.xyz.y >> 8;
	buff[4] = mpu9250_handle->accel.cal.offset.xyz.y & 0x00FF;
	buff[5] = 0;
	buff[6] = mpu9250_handle->accel.cal.offset.xyz.z >> 8;
	buff[7] = mpu9250_handle->accel.cal.offset.xyz.z & 0x00FF;

	esp_err_t ret = mpu9250_write_buff(mpu9250_handle, MPU9250_XA_OFFSET_H, buff, 8*8);
	return ret;
}

static esp_err_t mpu9250_acc_load_default_calibration_data(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_acc_load_offset(mpu9250_handle));

	mpu9250_handle->accel.cal.var[INV_FSR_2G].array[X_POS]=97;
	mpu9250_handle->accel.cal.var[INV_FSR_2G].array[Y_POS]=140;
	mpu9250_handle->accel.cal.var[INV_FSR_2G].array[Z_POS]=133;
	mpu9250_handle->accel.cal.sqm[INV_FSR_2G].array[X_POS]=9;
	mpu9250_handle->accel.cal.sqm[INV_FSR_2G].array[Y_POS]=11;
	mpu9250_handle->accel.cal.sqm[INV_FSR_2G].array[Z_POS]=11;

	mpu9250_handle->accel.cal.var[INV_FSR_4G].array[X_POS]=11;
	mpu9250_handle->accel.cal.var[INV_FSR_4G].array[Y_POS]=11;
	mpu9250_handle->accel.cal.var[INV_FSR_4G].array[Z_POS]=31;
	mpu9250_handle->accel.cal.sqm[INV_FSR_4G].array[X_POS]=3;
	mpu9250_handle->accel.cal.sqm[INV_FSR_4G].array[Y_POS]=3;
	mpu9250_handle->accel.cal.sqm[INV_FSR_4G].array[Z_POS]=5;

	mpu9250_handle->accel.cal.var[INV_FSR_8G].array[X_POS]=3;
	mpu9250_handle->accel.cal.var[INV_FSR_8G].array[Y_POS]=3;
	mpu9250_handle->accel.cal.var[INV_FSR_8G].array[Z_POS]=8;
	mpu9250_handle->accel.cal.sqm[INV_FSR_8G].array[X_POS]=1;
	mpu9250_handle->accel.cal.sqm[INV_FSR_8G].array[Y_POS]=1;
	mpu9250_handle->accel.cal.sqm[INV_FSR_8G].array[Z_POS]=2;

	mpu9250_handle->accel.cal.var[INV_FSR_16G].array[X_POS]=1;
	mpu9250_handle->accel.cal.var[INV_FSR_16G].array[Y_POS]=1;
	mpu9250_handle->accel.cal.var[INV_FSR_16G].array[Z_POS]=1;
	mpu9250_handle->accel.cal.sqm[INV_FSR_16G].array[X_POS]=1;
	mpu9250_handle->accel.cal.sqm[INV_FSR_16G].array[Y_POS]=1;
	mpu9250_handle->accel.cal.sqm[INV_FSR_16G].array[Z_POS]=1;
	printf("Accel: loaded default calibration data\n");

	return ESP_OK;
}

static esp_err_t mpu9250_acc_load_calibration_data(mpu9250_handle_t mpu9250_handle) {
    nvs_handle_t my_handle;
    uint8_t flashed = 0;
    ESP_ERROR_CHECK(nvs_open("ACC_CAL", NVS_READWRITE, &my_handle));
    esp_err_t err = nvs_get_u8(my_handle, "FLASHED", &flashed);
    if(err == ESP_ERR_NVS_NOT_FOUND) {
    	return mpu9250_acc_load_default_calibration_data(mpu9250_handle);
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "X_OFF", &mpu9250_handle->accel.cal.offset.array[X_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "Y_OFF", &mpu9250_handle->accel.cal.offset.array[Y_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "Z_OFF", &mpu9250_handle->accel.cal.offset.array[Z_POS]));

    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "X_VAR_2G", &mpu9250_handle->accel.cal.var[INV_FSR_2G].array[X_POS]));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "Y_VAR_2G", &mpu9250_handle->accel.cal.var[INV_FSR_2G].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "Z_VAR_2G", &mpu9250_handle->accel.cal.var[INV_FSR_2G].array[Z_POS]));

    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "X_VAR_4G", &mpu9250_handle->accel.cal.var[INV_FSR_4G].array[X_POS]));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "Y_VAR_4G", &mpu9250_handle->accel.cal.var[INV_FSR_4G].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "Z_VAR_4G", &mpu9250_handle->accel.cal.var[INV_FSR_4G].array[Z_POS]));

    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "X_VAR_8G", &mpu9250_handle->accel.cal.var[INV_FSR_8G].array[X_POS]));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "Y_VAR_8G", &mpu9250_handle->accel.cal.var[INV_FSR_8G].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "Z_VAR_8G", &mpu9250_handle->accel.cal.var[INV_FSR_8G].array[Z_POS]));

    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "X_VAR_16G", &mpu9250_handle->accel.cal.var[INV_FSR_16G].array[X_POS]));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "Y_VAR_16G", &mpu9250_handle->accel.cal.var[INV_FSR_16G].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "Z_VAR_16G", &mpu9250_handle->accel.cal.var[INV_FSR_16G].array[Z_POS]));

    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "X_SQM_2G", &mpu9250_handle->accel.cal.sqm[INV_FSR_2G].array[X_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "Y_SQM_2G", &mpu9250_handle->accel.cal.sqm[INV_FSR_2G].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "Z_SQM_2G", &mpu9250_handle->accel.cal.sqm[INV_FSR_2G].array[Z_POS]));

    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "X_SQM_4G", &mpu9250_handle->accel.cal.sqm[INV_FSR_4G].array[X_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "Y_SQM_4G", &mpu9250_handle->accel.cal.sqm[INV_FSR_4G].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "Z_SQM_4G", &mpu9250_handle->accel.cal.sqm[INV_FSR_4G].array[Z_POS]));

    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "X_SQM_8G", &mpu9250_handle->accel.cal.sqm[INV_FSR_8G].array[X_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "Y_SQM_8G", &mpu9250_handle->accel.cal.sqm[INV_FSR_8G].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "Z_SQM_8G", &mpu9250_handle->accel.cal.sqm[INV_FSR_8G].array[Z_POS]));

    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "X_SQM_16G", &mpu9250_handle->accel.cal.sqm[INV_FSR_16G].array[X_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "Y_SQM_16G", &mpu9250_handle->accel.cal.sqm[INV_FSR_16G].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "Z_SQM_16G", &mpu9250_handle->accel.cal.sqm[INV_FSR_16G].array[Z_POS]));

    printf("Accel: loaded calibration data from NVS ... \n");
    // Close
    nvs_close(my_handle);
	return ESP_OK;
}

static esp_err_t mpu9250_acc_calc_rpy(mpu9250_handle_t mpu9250_handle) {
	double modq = sqrt((double)(mpu9250_handle->accel.cal.kalman[X_POS].X*mpu9250_handle->accel.cal.kalman[X_POS].X+
			    mpu9250_handle->accel.cal.kalman[Y_POS].X*mpu9250_handle->accel.cal.kalman[Y_POS].X+
				mpu9250_handle->accel.cal.kalman[Z_POS].X*mpu9250_handle->accel.cal.kalman[Z_POS].X));

	// range of values: [-pi/2,+pi/2] rad
	// Accel X angle is Pitch
	// Accel Y angle is Roll
	mpu9250_handle->accel.rpy.xyz.x = PI_HALF - acos((double)mpu9250_handle->accel.cal.kalman[Y_POS].X/modq);
	mpu9250_handle->accel.rpy.xyz.y = - PI_HALF + acos((double)mpu9250_handle->accel.cal.kalman[X_POS].X/modq);
	mpu9250_handle->accel.rpy.xyz.z = acos((double)mpu9250_handle->accel.cal.kalman[Z_POS].X/modq);
	return ESP_OK;
}

static esp_err_t mpu9250_acc_filter_data(mpu9250_handle_t mpu9250_handle) {
	for(uint8_t i = 0; i < 3; i++) {
		if(mpu9250_handle->accel.cal.kalman[i].P > 0.01) {
			/*
			 *     X(k)=X(k-1)
			 *     P(k)=P(k-1)+Q
			 *     K(k)=P(k)/(P(k)+R)
			 *     X(k)=X(k)+K(k)*(Sample(k)-X(k))
			 *     P(k)=(1-K(k))*P(k)
			 */
			mpu9250_cb_means(&mpu9250_handle->accel.cb[i], &mpu9250_handle->accel.cal.kalman[i].sample);
			mpu9250_handle->accel.cal.kalman[i].P = mpu9250_handle->accel.cal.kalman[i].P+mpu9250_handle->accel.cal.kalman[i].Q;
			mpu9250_handle->accel.cal.kalman[i].K = mpu9250_handle->accel.cal.kalman[i].P/(mpu9250_handle->accel.cal.kalman[i].P+mpu9250_handle->accel.cal.kalman[i].R);
			mpu9250_handle->accel.cal.kalman[i].X = mpu9250_handle->accel.cal.kalman[i].X + mpu9250_handle->accel.cal.kalman[i].K*(mpu9250_handle->accel.cal.kalman[i].sample - mpu9250_handle->accel.cal.kalman[i].X);
			mpu9250_handle->accel.cal.kalman[i].P = (1-mpu9250_handle->accel.cal.kalman[i].K)*mpu9250_handle->accel.cal.kalman[i].P;
		}
	}
	return ESP_OK;
}

/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/
esp_err_t mpu9250_acc_init(mpu9250_handle_t mpu9250_handle) {
	mpu9250_acc_load_calibration_data(mpu9250_handle); // set offset, var, sqm, P, K
	mpu9250_acc_save_offset(mpu9250_handle);
	mpu9250_acc_init_kalman_filter(mpu9250_handle);
	mpu9250_handle->accel.rpy.xyz.x = 0;
	mpu9250_handle->accel.rpy.xyz.y = 0;
	mpu9250_handle->accel.rpy.xyz.z = 0;

	return ESP_OK;
}

esp_err_t mpu9250_acc_load_fsr(mpu9250_handle_t mpu9250_handle) {
	uint8_t acc_conf = 0;
    ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_ACCEL_CONFIG, &acc_conf));
    mpu9250_handle->accel.fsr = (acc_conf & (~MPU9250_ACC_FSR_MASK)) >> MPU9250_ACC_FSR_LBIT;
    ESP_ERROR_CHECK(mpu9250_acc_calc_lsb(mpu9250_handle));
	return ESP_OK;
}

esp_err_t mpu9250_acc_set_fsr(mpu9250_handle_t mpu9250_handle, uint8_t fsr) {
	mpu9250_handle->accel.fsr=fsr;
	ESP_ERROR_CHECK(mpu9250_acc_save_fsr(mpu9250_handle));
	return ESP_OK;
}

esp_err_t mpu9250_acc_load_offset(mpu9250_handle_t mpu9250_handle) {
	uint8_t buff[8];
	esp_err_t ret = mpu9250_read_buff(mpu9250_handle, MPU9250_XA_OFFSET_H, buff, 8*8);
	mpu9250_handle->accel.cal.offset.xyz.x = (buff[0] << 8) + buff[1];
	mpu9250_handle->accel.cal.offset.xyz.y = (buff[3] << 8) + buff[4];
	mpu9250_handle->accel.cal.offset.xyz.z = (buff[6] << 8) + buff[7];
	return ret;
}
esp_err_t mpu9250_acc_set_offset(mpu9250_handle_t mpu9250_handle, int16_t xoff, int16_t yoff, int16_t zoff) {
	mpu9250_handle->accel.cal.offset.xyz.x = xoff;
	mpu9250_handle->accel.cal.offset.xyz.y = yoff;
	mpu9250_handle->accel.cal.offset.xyz.z = zoff;
	return mpu9250_acc_save_offset(mpu9250_handle);
}

esp_err_t mpu9250_acc_update_state(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_acc_filter_data(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_acc_calc_rpy(mpu9250_handle));
	return ESP_OK;
}

esp_err_t mpu9250_acc_save_calibration_data (mpu9250_handle_t mpu9250_handle) {
    nvs_handle_t my_handle;
    ESP_ERROR_CHECK(nvs_open("ACC_CAL", NVS_READWRITE, &my_handle));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "FLASHED", 1));
    ESP_ERROR_CHECK(nvs_set_i16(my_handle, "X_OFF", mpu9250_handle->accel.cal.offset.array[X_POS]));
    ESP_ERROR_CHECK(nvs_set_i16(my_handle, "Y_OFF", mpu9250_handle->accel.cal.offset.array[Y_POS]));
    ESP_ERROR_CHECK(nvs_set_i16(my_handle, "Z_OFF", mpu9250_handle->accel.cal.offset.array[Z_POS]));

    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "X_VAR_2G", mpu9250_handle->accel.cal.var[INV_FSR_2G].array[X_POS]));
    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "Y_VAR_2G", mpu9250_handle->accel.cal.var[INV_FSR_2G].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "Z_VAR_2G", mpu9250_handle->accel.cal.var[INV_FSR_2G].array[Z_POS]));

    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "X_VAR_4G", mpu9250_handle->accel.cal.var[INV_FSR_4G].array[X_POS]));
    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "Y_VAR_4G", mpu9250_handle->accel.cal.var[INV_FSR_4G].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "Z_VAR_4G", mpu9250_handle->accel.cal.var[INV_FSR_4G].array[Z_POS]));

    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "X_VAR_8G", mpu9250_handle->accel.cal.var[INV_FSR_8G].array[X_POS]));
    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "Y_VAR_8G", mpu9250_handle->accel.cal.var[INV_FSR_8G].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "Z_VAR_8G", mpu9250_handle->accel.cal.var[INV_FSR_8G].array[Z_POS]));

    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "X_VAR_16G", mpu9250_handle->accel.cal.var[INV_FSR_16G].array[X_POS]));
    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "Y_VAR_16G", mpu9250_handle->accel.cal.var[INV_FSR_16G].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "Z_VAR_16G", mpu9250_handle->accel.cal.var[INV_FSR_16G].array[Z_POS]));

    ESP_ERROR_CHECK(nvs_set_i16(my_handle, "X_SQM_2G", mpu9250_handle->accel.cal.sqm[INV_FSR_2G].array[X_POS]));
    ESP_ERROR_CHECK(nvs_set_i16(my_handle, "Y_SQM_2G", mpu9250_handle->accel.cal.sqm[INV_FSR_2G].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_set_i16(my_handle, "Z_SQM_2G", mpu9250_handle->accel.cal.sqm[INV_FSR_2G].array[Z_POS]));

    ESP_ERROR_CHECK(nvs_set_i16(my_handle, "X_SQM_4G", mpu9250_handle->accel.cal.sqm[INV_FSR_4G].array[X_POS]));
    ESP_ERROR_CHECK(nvs_set_i16(my_handle, "Y_SQM_4G", mpu9250_handle->accel.cal.sqm[INV_FSR_4G].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_set_i16(my_handle, "Z_SQM_4G", mpu9250_handle->accel.cal.sqm[INV_FSR_4G].array[Z_POS]));

    ESP_ERROR_CHECK(nvs_set_i16(my_handle, "X_SQM_8G", mpu9250_handle->accel.cal.sqm[INV_FSR_8G].array[X_POS]));
    ESP_ERROR_CHECK(nvs_set_i16(my_handle, "Y_SQM_8G", mpu9250_handle->accel.cal.sqm[INV_FSR_8G].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_set_i16(my_handle, "Z_SQM_8G", mpu9250_handle->accel.cal.sqm[INV_FSR_8G].array[Z_POS]));

    ESP_ERROR_CHECK(nvs_set_i16(my_handle, "X_SQM_16G", mpu9250_handle->accel.cal.sqm[INV_FSR_16G].array[X_POS]));
    ESP_ERROR_CHECK(nvs_set_i16(my_handle, "Y_SQM_16G", mpu9250_handle->accel.cal.sqm[INV_FSR_16G].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_set_i16(my_handle, "Z_SQM_16G", mpu9250_handle->accel.cal.sqm[INV_FSR_16G].array[Z_POS]));

    printf("Accel Committing updates in NVS ... \n");
    ESP_ERROR_CHECK(nvs_commit(my_handle));

    // Close
    nvs_close(my_handle);

	return ESP_OK;
}

