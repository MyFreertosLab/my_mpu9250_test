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
	for(uint8_t i = X_POS; i <= Z_POS; i++) {
		mpu9250_handle->accel.kalman[i].X = mpu9250_handle->accel.lsb;
		mpu9250_handle->accel.kalman[i].sample=0;
		mpu9250_handle->accel.kalman[i].P=1.0f;
		mpu9250_handle->accel.kalman[i].Q=1.5;
		mpu9250_handle->accel.kalman[i].K=0.0f;
		mpu9250_handle->accel.kalman[i].R=mpu9250_handle->accel.var[mpu9250_handle->accel.fsr].array[i];
	}
	return ESP_OK;
}

static esp_err_t mpu9250_acc_calc_means(mpu9250_handle_t mpu9250_handle, int16_t* acc_means, uint8_t cycles) {
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );
	printf("Calculating Acc Means with %d000 samples (wait for %d seconds)... \n", cycles, cycles);
	acc_means[0] = 0;
	acc_means[1] = 0;
	acc_means[2] = 0;

	int64_t acc_means_64[3] = {0,0,0};
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
		acc_means_64[0] += acc_sum[0]/max_samples;
		acc_means_64[1] += acc_sum[1]/max_samples;
		acc_means_64[2] += acc_sum[2]/max_samples;
	}
	acc_means[0] = acc_means_64[0]/cycles;
	acc_means[1] = acc_means_64[1]/cycles;
	acc_means[2] = acc_means_64[2]/cycles;
	return ESP_OK;
}

static esp_err_t mpu9250_acc_calc_var(mpu9250_handle_t mpu9250_handle, int16_t* acc_means, uint16_t* accel_var, uint8_t cycles) {
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );

	printf("Calculating Acc Var with %d000 samples (wait for %d seconds)... \n", cycles, cycles);
	accel_var[0] = 0.0f;
	accel_var[1] = 0.0f;
	accel_var[2] = 0.0f;
	uint64_t accel_var_64[3] = {0,0,0};

	for(int j = 0; j < cycles; j++) {
		uint16_t max_samples = 1000;
		int64_t acc_sum[3] = {0,0,0};
		for(int i = 0; i < max_samples; i++) {
			ulTaskNotifyTake( pdTRUE,xMaxBlockTime );
			ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));
			acc_sum[0] += (mpu9250_handle->raw_data.data_s_xyz.accel_data_x - acc_means[0])*(mpu9250_handle->raw_data.data_s_xyz.accel_data_x - acc_means[0]);
			acc_sum[1] += (mpu9250_handle->raw_data.data_s_xyz.accel_data_y - acc_means[1])*(mpu9250_handle->raw_data.data_s_xyz.accel_data_y - acc_means[1]);
			acc_sum[2] += (mpu9250_handle->raw_data.data_s_xyz.accel_data_z - acc_means[2])*(mpu9250_handle->raw_data.data_s_xyz.accel_data_z - acc_means[2]);
//			if(i%100 == 0) {
//				printf("Err Var [%d][%d][%d]\n", (mpu9250_handle->raw_data.data_s_xyz.accel_data_x - acc_means[0]), (mpu9250_handle->raw_data.data_s_xyz.accel_data_y - acc_means[1]), (mpu9250_handle->raw_data.data_s_xyz.accel_data_z - acc_means[2]));
//				printf("Sum Var [%lld][%lld][%lld]\n", acc_sum[0], acc_sum[1], acc_sum[2]);
//			}
		}

		// offsets respect vertical attitude
		accel_var_64[0] += acc_sum[0]/max_samples;
		accel_var_64[1] += acc_sum[1]/max_samples;
		accel_var_64[2] += acc_sum[2]/max_samples;
	}

	accel_var[0] = accel_var_64[0]/(cycles);
	accel_var[1] = accel_var_64[1]/(cycles);
	accel_var[2] = accel_var_64[2]/(cycles);
	printf("Acc_var: [%d][%d][%d]\n", accel_var[0], accel_var[1],accel_var[2]);

	return ESP_OK;
}
static esp_err_t mpu9250_acc_calc_sqm(mpu9250_handle_t mpu9250_handle, int16_t* acc_means, int16_t* accel_sqm, uint8_t cycles) {
	ESP_ERROR_CHECK(mpu9250_acc_calc_var(mpu9250_handle, acc_means, mpu9250_handle->accel.var[mpu9250_handle->accel.fsr].array, 60));
	accel_sqm[0] = sqrt(mpu9250_handle->accel.var[mpu9250_handle->accel.fsr].xyz.x);
	accel_sqm[1] = sqrt(mpu9250_handle->accel.var[mpu9250_handle->accel.fsr].xyz.y);
	accel_sqm[2] = sqrt(mpu9250_handle->accel.var[mpu9250_handle->accel.fsr].xyz.z);

	printf("Acc_sqm: [%d][%d][%d]\n", accel_sqm[0], accel_sqm[1],accel_sqm[2]);

	return ESP_OK;
}

static esp_err_t mpu9250_acc_calc_bias(mpu9250_handle_t mpu9250_handle) {
	// Calc Bias
	{
		int16_t acc_means[3] = {0,0,0};
		printf("Calculating Acc Bias ... \n");
		memset(mpu9250_handle->accel.sqm[mpu9250_handle->accel.fsr].array, 0, sizeof(mpu9250_handle->accel.sqm[mpu9250_handle->accel.fsr].array));       //Zero out sqm
		acc_means[2] = mpu9250_handle->accel.lsb;
		ESP_ERROR_CHECK(mpu9250_acc_calc_sqm(mpu9250_handle, acc_means, mpu9250_handle->accel.sqm[mpu9250_handle->accel.fsr].array, 60));
		printf("Acc SQM: [%d][%d][%d]\n",
				mpu9250_handle->accel.sqm[mpu9250_handle->accel.fsr].array[0],
				mpu9250_handle->accel.sqm[mpu9250_handle->accel.fsr].array[1],
				mpu9250_handle->accel.sqm[mpu9250_handle->accel.fsr].array[2]);

	}
	return ESP_OK;
}

static esp_err_t mpu9250_acc_calc_biases(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_16G));
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 10000));
	ESP_ERROR_CHECK(mpu9250_acc_calc_bias(mpu9250_handle));

	ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_8G));
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 10000));
	ESP_ERROR_CHECK(mpu9250_acc_calc_bias(mpu9250_handle));

	ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_4G));
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 10000));
	ESP_ERROR_CHECK(mpu9250_acc_calc_bias(mpu9250_handle));

	ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_2G));
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 10000));
	ESP_ERROR_CHECK(mpu9250_acc_calc_bias(mpu9250_handle));

	return ESP_OK;
}

static esp_err_t mpu9250_acc_calc_lsb(mpu9250_handle_t mpu9250_handle) {
	mpu9250_handle->accel.lsb = (32768 >> (mpu9250_handle->accel.fsr + 1));
	return ESP_OK;
}


static esp_err_t mpu9250_acc_save_offset(mpu9250_handle_t mpu9250_handle) {
	uint8_t buff[8];
	buff[0] = mpu9250_handle->accel.offset.xyz.x >> 8;
	buff[1] = mpu9250_handle->accel.offset.xyz.x & 0x00FF;
	buff[2] = 0;
	buff[3] = mpu9250_handle->accel.offset.xyz.y >> 8;
	buff[4] = mpu9250_handle->accel.offset.xyz.y & 0x00FF;
	buff[5] = 0;
	buff[6] = mpu9250_handle->accel.offset.xyz.z >> 8;
	buff[7] = mpu9250_handle->accel.offset.xyz.z & 0x00FF;

	esp_err_t ret = mpu9250_write_buff(mpu9250_handle, MPU9250_XA_OFFSET_H, buff, 8*8);
	return ret;
}

static esp_err_t mpu9250_acc_calc_offset(mpu9250_handle_t mpu9250_handle) {
	uint8_t fsr_original = mpu9250_handle->accel.fsr;
	if(mpu9250_handle->accel.fsr != INV_FSR_16G) {
		mpu9250_handle->accel.fsr=INV_FSR_16G;
		ESP_ERROR_CHECK(mpu9250_acc_save_fsr(mpu9250_handle));
	}


	// Original Acc offsets: [6684][-5436][9684]
	// Last: Acc offsets: [6812][-5443][9674]
	ESP_ERROR_CHECK(mpu9250_acc_load_offset(mpu9250_handle));
	printf("Acc offsets: [%d][%d][%d]\n", mpu9250_handle->accel.offset.xyz.x, mpu9250_handle->accel.offset.xyz.y,mpu9250_handle->accel.offset.xyz.z);

	// dicard 10000 samples
	ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 10000));
	uint8_t found[3] = {0,0,0};
	int16_t offsets[3] = {0,0,0};
	while(true) {
		printf("Calculating Acc Offset ... \n");
		uint16_t max_means = 60;
		int16_t acc_means[3] = {0,0,0};
		ESP_ERROR_CHECK(mpu9250_acc_calc_means(mpu9250_handle, acc_means, max_means));

		if((found[X_POS] == 0)) {
			if(abs(acc_means[X_POS]) >> 1 == 0) {
				found[X_POS] = 1;
				offsets[X_POS] = mpu9250_handle->accel.offset.xyz.x;
				printf("FOUND X OFFSET\n");
			}
			else {
				mpu9250_handle->accel.offset.xyz.x -= acc_means[X_POS];
			}
		} else {
			mpu9250_handle->accel.offset.xyz.x = offsets[X_POS];
		}
		if((found[Y_POS] == 0)) {
			if(abs(acc_means[Y_POS]) >> 1 == 0) {
				found[Y_POS] = 1;
				offsets[Y_POS] = mpu9250_handle->accel.offset.xyz.y;
				printf("FOUND Y OFFSET\n");
			}
			else {
				mpu9250_handle->accel.offset.xyz.y -= acc_means[Y_POS];
			}
		} else {
			mpu9250_handle->accel.offset.xyz.y = offsets[Y_POS];
		}
		if((found[Z_POS] == 0)) {
			if(abs(acc_means[Z_POS] - mpu9250_handle->accel.lsb) >> 1 == 0) {
				found[Z_POS] = 1;
				offsets[Z_POS] = mpu9250_handle->accel.offset.xyz.z;
				printf("FOUND Z OFFSET\n");
			}
			else {
				mpu9250_handle->accel.offset.xyz.z -= (acc_means[Z_POS]  - mpu9250_handle->accel.lsb);
			}
		} else {
			mpu9250_handle->accel.offset.xyz.z = offsets[Z_POS];
		}

		printf("Acc offsets: [%d][%d][%d]\n", mpu9250_handle->accel.offset.xyz.x, mpu9250_handle->accel.offset.xyz.y,mpu9250_handle->accel.offset.xyz.z);
		printf("Acc means: [%d][%d][%d]\n", acc_means[X_POS], acc_means[Y_POS],acc_means[Z_POS]);

		if((found[0] == 1) && (found[1] == 1) && (found[2] == 1)) {
			break;
		}

		ESP_ERROR_CHECK(mpu9250_acc_save_offset(mpu9250_handle));
		ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 10000));
		ESP_ERROR_CHECK(mpu9250_display_messages(mpu9250_handle, 1000));
	}
	ESP_ERROR_CHECK(mpu9250_acc_calc_biases(mpu9250_handle));

	if(fsr_original != mpu9250_handle->accel.fsr) {
		ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, fsr_original));
		ESP_ERROR_CHECK(mpu9250_discard_messages(mpu9250_handle, 10000));
	}
	return ESP_OK;
}

static esp_err_t mpu9250_acc_load_statistics(mpu9250_handle_t mpu9250_handle) {
	mpu9250_handle->accel.offset.array[X_POS]=6995;
	mpu9250_handle->accel.offset.array[Y_POS]=-5411;
	mpu9250_handle->accel.offset.array[Z_POS]=9684;

	mpu9250_handle->accel.var[INV_FSR_2G].array[X_POS]=197;
	mpu9250_handle->accel.var[INV_FSR_2G].array[Y_POS]=209;
	mpu9250_handle->accel.var[INV_FSR_2G].array[Z_POS]=247;
	mpu9250_handle->accel.sqm[INV_FSR_2G].array[X_POS]=14;
	mpu9250_handle->accel.sqm[INV_FSR_2G].array[Y_POS]=14;
	mpu9250_handle->accel.sqm[INV_FSR_2G].array[Z_POS]=15;

	mpu9250_handle->accel.var[INV_FSR_4G].array[X_POS]=50;
	mpu9250_handle->accel.var[INV_FSR_4G].array[Y_POS]=50;
	mpu9250_handle->accel.var[INV_FSR_4G].array[Z_POS]=59;
	mpu9250_handle->accel.sqm[INV_FSR_4G].array[X_POS]=7;
	mpu9250_handle->accel.sqm[INV_FSR_4G].array[Y_POS]=7;
	mpu9250_handle->accel.sqm[INV_FSR_4G].array[Z_POS]=7;

	mpu9250_handle->accel.var[INV_FSR_8G].array[X_POS]=12;
	mpu9250_handle->accel.var[INV_FSR_8G].array[Y_POS]=12;
	mpu9250_handle->accel.var[INV_FSR_8G].array[Z_POS]=13;
	mpu9250_handle->accel.sqm[INV_FSR_8G].array[X_POS]=3;
	mpu9250_handle->accel.sqm[INV_FSR_8G].array[Y_POS]=3;
	mpu9250_handle->accel.sqm[INV_FSR_8G].array[Z_POS]=3;

	mpu9250_handle->accel.var[INV_FSR_16G].array[X_POS]=3;
	mpu9250_handle->accel.var[INV_FSR_16G].array[Y_POS]=2;
	mpu9250_handle->accel.var[INV_FSR_16G].array[Z_POS]=3;
	mpu9250_handle->accel.sqm[INV_FSR_16G].array[X_POS]=1;
	mpu9250_handle->accel.sqm[INV_FSR_16G].array[Y_POS]=1;
	mpu9250_handle->accel.sqm[INV_FSR_16G].array[Z_POS]=1;

	return ESP_OK;
}
/*
 Acc offsets: [6995][-5411][9684]
Acc means: [0][0][2047]
MPU9250: AccFSR 16g
Discarding 10000 Samples ...
Calculating Acc Bias ...
Calculating Acc Var with 60000 samples (wait for 60 seconds)...
Acc_var: [10][10][24]
Acc_sqm: [3][3][4]
Acc SQM: [3][3][4]
MPU9250: AccFSR 8g
Discarding 10000 Samples ...
Calculating Acc Bias ...
Calculating Acc Var with 60000 samples (wait for 60 seconds)...
Acc_var: [46][49][100]
Acc_sqm: [6][7][10]
Acc SQM: [6][7][10]
MPU9250: AccFSR 4g
Discarding 10000 Samples ...
Calculating Acc Bias ...
Calculating Acc Var with 60000 samples (wait for 60 seconds)...
Acc_var: [290][333][400]
Acc_sqm: [17][18][20]
Acc SQM: [17][18][20]
MPU9250: AccFSR 2g
Discarding 10000 Samples ...
Calculating Acc Bias ...
Calculating Acc Var with 60000 samples (wait for 60 seconds)...
Acc_var: [1822][1938][1682]
Acc_sqm: [42][44][41]
Acc SQM: [42][44][41]
MPU9250: AccFSR 16g

 */

/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/
esp_err_t mpu9250_acc_init(mpu9250_handle_t mpu9250_handle) {
	mpu9250_acc_load_statistics(mpu9250_handle); // set offset, var, sqm, P, K
	mpu9250_acc_save_offset(mpu9250_handle);
	mpu9250_acc_init_kalman_filter(mpu9250_handle);
	mpu9250_handle->accel.roll = 0;
	mpu9250_handle->accel.pitch = 0;
	mpu9250_handle->accel.yaw = 0;

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
	mpu9250_handle->accel.offset.xyz.x = (buff[0] << 8) + buff[1];
	mpu9250_handle->accel.offset.xyz.y = (buff[3] << 8) + buff[4];
	mpu9250_handle->accel.offset.xyz.z = (buff[6] << 8) + buff[7];
	return ret;
}
esp_err_t mpu9250_acc_set_offset(mpu9250_handle_t mpu9250_handle, int16_t xoff, int16_t yoff, int16_t zoff) {
	mpu9250_handle->accel.offset.xyz.x = xoff;
	mpu9250_handle->accel.offset.xyz.y = yoff;
	mpu9250_handle->accel.offset.xyz.z = zoff;
	return mpu9250_acc_save_offset(mpu9250_handle);
}

esp_err_t mpu9250_acc_calibrate(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_acc_calc_offset(mpu9250_handle));
	mpu9250_acc_init_kalman_filter(mpu9250_handle);

	return ESP_OK;
}

esp_err_t mpu9250_acc_calc_rpy(mpu9250_handle_t mpu9250_handle) {
	int64_t modq = mpu9250_handle->accel.kalman[X_POS].X*mpu9250_handle->accel.kalman[X_POS].X+
			    mpu9250_handle->accel.kalman[Y_POS].X*mpu9250_handle->accel.kalman[Y_POS].X+
				mpu9250_handle->accel.kalman[Z_POS].X*mpu9250_handle->accel.kalman[Z_POS].X;
	mpu9250_handle->accel.roll = 3.141592654f/2.0f - acos((double)mpu9250_handle->accel.kalman[Y_POS].X/sqrt((double)modq));
	mpu9250_handle->accel.pitch = - 3.141592654f/2.0f + acos((double)mpu9250_handle->accel.kalman[X_POS].X/sqrt((double)modq));
	mpu9250_handle->accel.yaw = acos((double)mpu9250_handle->accel.kalman[Z_POS].X/sqrt((double)modq));
	return ESP_OK;
}
esp_err_t mpu9250_acc_filter_data(mpu9250_handle_t mpu9250_handle) {
	for(uint8_t i = X_POS; i <= Z_POS; i++) {
		if(mpu9250_handle->accel.kalman[i].P > 0.01) {
			/*
			 *     X(k)=X(k-1)
			 *     P(k)=P(k-1)+Q
			 *     K(k)=P(k)/(P(k)+R)
			 *     X(k)=X(k)+K(k)*(Sample(k)-X(k))
			 *     P(k)=(1-K(k))*P(k)
			 */
			mpu9250_cb_means(&mpu9250_handle->accel.cb[i], &mpu9250_handle->accel.kalman[i].sample);
			mpu9250_handle->accel.kalman[i].P = mpu9250_handle->accel.kalman[i].P+mpu9250_handle->accel.kalman[i].Q;
			mpu9250_handle->accel.kalman[i].K = mpu9250_handle->accel.kalman[i].P/(mpu9250_handle->accel.kalman[i].P+mpu9250_handle->accel.kalman[i].R);
			mpu9250_handle->accel.kalman[i].X = mpu9250_handle->accel.kalman[i].X + mpu9250_handle->accel.kalman[i].K*(mpu9250_handle->accel.kalman[i].sample - mpu9250_handle->accel.kalman[i].X);
			mpu9250_handle->accel.kalman[i].P = (1-mpu9250_handle->accel.kalman[i].K)*mpu9250_handle->accel.kalman[i].P;
		}
	}
	return ESP_OK;
}

