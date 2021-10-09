/*
 * mpu9250_mag.c
 *
 *  Created on: 25 set 2021
 *      Author: andrea
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <mpu9250_spi.h>
#include <mpu9250_mag.h>
#include <math.h>
#include "esp_system.h"
#include <nvs_flash.h>
#include <nvs.h>

/************************************************************************
 ************** P R I V A T E  I M P L E M E N T A T I O N **************
 ************************************************************************/

static esp_err_t WriteAk8963Register(mpu9250_handle_t mpu9250_handle, uint8_t reg,
		uint8_t data) {
	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_ADDR, AK8963_ADDRESS));
	ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_REG, reg));
	ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_DO, data));
	uint8_t slv0_en = (I2C_SLV0_EN | 0x01);
	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_CTRL, slv0_en));
	return ESP_OK;
}

static esp_err_t ReadAk8963Register(mpu9250_handle_t mpu9250_handle, uint8_t reg) {
	uint8_t addr = (AK8963_ADDRESS | MPU9250_I2C_READ_FLAG);
	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_ADDR, addr));
	ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_REG, reg));
	uint8_t slv0_en = (I2C_SLV0_EN | 0x01);
	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_CTRL, slv0_en));
	return ESP_OK;
}
static esp_err_t ReadAk8963Registers(mpu9250_handle_t mpu9250_handle, uint8_t reg, uint8_t count) {
	uint8_t addr = (AK8963_ADDRESS | MPU9250_I2C_READ_FLAG);
	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_ADDR, addr));
	ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_REG, reg));
	uint8_t slv0_en = (I2C_SLV0_EN | count);
	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_CTRL, slv0_en));
	return ESP_OK;
}

static esp_err_t mpu9250_mag_calc_precision_factor(mpu9250_handle_t mpu9250_handle) {
	switch(mpu9250_handle->mag.precision) {
	case(INV_MAG_PRECISION_14_BITS): {
		mpu9250_handle->mag.precision_factor = 0.6f;
		printf("MPU9250: Mag Precision 0.6 \n");
		break;
	}
	case(INV_MAG_PRECISION_16_BITS): {
		mpu9250_handle->mag.precision_factor = 0.15f;
		printf("MPU9250: Mag Precision 0.15 \n");
		break;
	}
	default: {
		printf("MPU9250: Mag Precision UNKNOWN [%d]\n", mpu9250_handle->mag.precision);
		return ESP_FAIL;
	}
	}
	return ESP_OK;
}

static esp_err_t mpu9250_mag_save_precision(mpu9250_handle_t mpu9250_handle) {
	uint8_t mag_conf = 0;

	// read cntl1
    ESP_ERROR_CHECK(ReadAk8963Register(mpu9250_handle, AK8963_CNTL1));
    vTaskDelay(pdMS_TO_TICKS(10));
	ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_EXT_SENS_DATA_00, &mag_conf));
	printf("MAG SAVE mag_conf=[%d]\n", mag_conf);

	// calc configuration for requested precision
	switch(mpu9250_handle->mag.precision) {
	case INV_MAG_PRECISION_14_BITS: {
		mag_conf &= ~(AK8963_PRECISION_MASK);
		printf("MAG SAVE 14BITS mag_conf=[%d]\n", mag_conf);
		break;
	}
	case INV_MAG_PRECISION_16_BITS: {
		mag_conf |= AK8963_PRECISION_MASK;
		printf("MAG SAVE 16BITS mag_conf=[%d]\n", mag_conf);
		break;
	}
	default: {
		printf("MPU9250: Mag Precision UNKNOWN [%d]\n", mpu9250_handle->mag.precision);
		return ESP_FAIL;
	}
	}

    ESP_ERROR_CHECK(
    		WriteAk8963Register(mpu9250_handle, AK8963_CNTL1, mag_conf));
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_ERROR_CHECK(
    		mpu9250_mag_calc_precision_factor(mpu9250_handle));

	return ESP_OK;
}

static esp_err_t mpu9250_mag_init_kalman_filter(mpu9250_handle_t mpu9250_handle) {
	for(uint8_t i = 0; i < 3; i++) {
		mpu9250_handle->mag.cal.kalman[i].initialized = 0;
		mpu9250_handle->mag.cal.kalman[i].X = 0.0f;
		mpu9250_handle->mag.cal.kalman[i].sample=0;
		mpu9250_handle->mag.cal.kalman[i].P=1.0f;
		mpu9250_handle->mag.cal.kalman[i].Q=1.0;
		mpu9250_handle->mag.cal.kalman[i].K=0.0f;
		mpu9250_handle->mag.cal.kalman[i].R=mpu9250_handle->mag.cal.var[mpu9250_handle->mag.precision].array[i];
	}
	return ESP_OK;
}


/*
* Mag soff: [30.87274,-20.66476,20.37856]
* Mag sf2: [1.06298,1.01071,0.92631]
* Mag offset calculated at 0.15 precision
*
* Mag soff: [7.09354,-4.60390,1.97933]
* Mag sf2: [0.95312,0.90874,1.13813]
* Mag offset calculated at 0.6 precision
*
*/
static esp_err_t mpu9250_mag_load_default_calibration_data(mpu9250_handle_t mpu9250_handle) {
	mpu9250_handle->mag.cal.var[INV_MAG_PRECISION_14_BITS].array[X_POS]=1.0f;
	mpu9250_handle->mag.cal.var[INV_MAG_PRECISION_14_BITS].array[Y_POS]=1.0f;
	mpu9250_handle->mag.cal.var[INV_MAG_PRECISION_14_BITS].array[Z_POS]=1.0f;
	mpu9250_handle->mag.cal.sqm[INV_MAG_PRECISION_14_BITS].array[X_POS]=1.0f;
	mpu9250_handle->mag.cal.sqm[INV_MAG_PRECISION_14_BITS].array[Y_POS]=1.0f;
	mpu9250_handle->mag.cal.sqm[INV_MAG_PRECISION_14_BITS].array[Z_POS]=1.0f;

	mpu9250_handle->mag.cal.var[INV_MAG_PRECISION_16_BITS].array[X_POS]=1.0f;
	mpu9250_handle->mag.cal.var[INV_MAG_PRECISION_16_BITS].array[Y_POS]=1.0f;
	mpu9250_handle->mag.cal.var[INV_MAG_PRECISION_16_BITS].array[Z_POS]=1.0f;
	mpu9250_handle->mag.cal.sqm[INV_MAG_PRECISION_16_BITS].array[X_POS]=1.0f;
	mpu9250_handle->mag.cal.sqm[INV_MAG_PRECISION_16_BITS].array[Y_POS]=1.0f;
	mpu9250_handle->mag.cal.sqm[INV_MAG_PRECISION_16_BITS].array[Z_POS]=1.0f;

	mpu9250_handle->mag.cal.scaled_offset[INV_MAG_PRECISION_14_BITS].array[X_POS] = 0.0f;
	mpu9250_handle->mag.cal.scaled_offset[INV_MAG_PRECISION_14_BITS].array[Y_POS]= 0.0f;
	mpu9250_handle->mag.cal.scaled_offset[INV_MAG_PRECISION_14_BITS].array[Z_POS]= 0.0f;
	mpu9250_handle->mag.cal.scaled_offset[INV_MAG_PRECISION_16_BITS].array[X_POS] = 0.0f;
	mpu9250_handle->mag.cal.scaled_offset[INV_MAG_PRECISION_16_BITS].array[Y_POS]= 0.0f;
	mpu9250_handle->mag.cal.scaled_offset[INV_MAG_PRECISION_16_BITS].array[Z_POS]= 0.0f;

	mpu9250_handle->mag.cal.scale_factors2[INV_MAG_PRECISION_14_BITS].array[X_POS] = 1.0f;
	mpu9250_handle->mag.cal.scale_factors2[INV_MAG_PRECISION_14_BITS].array[Y_POS]= 1.0f;
	mpu9250_handle->mag.cal.scale_factors2[INV_MAG_PRECISION_14_BITS].array[Z_POS]= 1.0f;
	mpu9250_handle->mag.cal.scale_factors2[INV_MAG_PRECISION_16_BITS].array[X_POS] = 1.0f;
	mpu9250_handle->mag.cal.scale_factors2[INV_MAG_PRECISION_16_BITS].array[Y_POS]= 1.0f;
	mpu9250_handle->mag.cal.scale_factors2[INV_MAG_PRECISION_16_BITS].array[Z_POS]= 1.0f;

	printf("Mag: loaded default calibration data\n");
	return ESP_OK;
}

static esp_err_t mpu9250_mag_load_calibration_data(mpu9250_handle_t mpu9250_handle) {
    nvs_handle_t my_handle;
    uint8_t flashed = 0;
    ESP_ERROR_CHECK(nvs_open("MAG_CAL", NVS_READWRITE, &my_handle));
    esp_err_t err = nvs_get_u8(my_handle, "FLASHED", &flashed);
    if(err == ESP_ERR_NVS_NOT_FOUND) {
    	return mpu9250_mag_load_default_calibration_data(mpu9250_handle);
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(nvs_get_u8(my_handle, "X_ASA", &mpu9250_handle->mag.cal.asa.array[X_POS]));
    ESP_ERROR_CHECK(nvs_get_u8(my_handle, "Y_ASA", &mpu9250_handle->mag.cal.asa.array[Y_POS]));
    ESP_ERROR_CHECK(nvs_get_u8(my_handle, "Z_ASA", &mpu9250_handle->mag.cal.asa.array[Z_POS]));

    ESP_ERROR_CHECK(nvs_get_u32(my_handle, "X_OFF_14_BIT", ((uint32_t*)&mpu9250_handle->mag.cal.scaled_offset[INV_MAG_PRECISION_14_BITS].array[X_POS])));
    ESP_ERROR_CHECK(nvs_get_u32(my_handle, "Y_OFF_14_BIT", ((uint32_t*)&mpu9250_handle->mag.cal.scaled_offset[INV_MAG_PRECISION_14_BITS].array[Y_POS])));
    ESP_ERROR_CHECK(nvs_get_u32(my_handle, "Z_OFF_14_BIT", ((uint32_t*)&mpu9250_handle->mag.cal.scaled_offset[INV_MAG_PRECISION_14_BITS].array[Z_POS])));
    ESP_ERROR_CHECK(nvs_get_u32(my_handle, "X_OFF_16_BIT", ((uint32_t*)&mpu9250_handle->mag.cal.scaled_offset[INV_MAG_PRECISION_16_BITS].array[X_POS])));
    ESP_ERROR_CHECK(nvs_get_u32(my_handle, "Y_OFF_16_BIT", ((uint32_t*)&mpu9250_handle->mag.cal.scaled_offset[INV_MAG_PRECISION_16_BITS].array[Y_POS])));
    ESP_ERROR_CHECK(nvs_get_u32(my_handle, "Z_OFF_16_BIT", ((uint32_t*)&mpu9250_handle->mag.cal.scaled_offset[INV_MAG_PRECISION_16_BITS].array[Z_POS])));

    ESP_ERROR_CHECK(nvs_get_u32(my_handle, "X_SF2_14_BIT", ((uint32_t*)&mpu9250_handle->mag.cal.scale_factors2[INV_MAG_PRECISION_14_BITS].array[X_POS])));
    ESP_ERROR_CHECK(nvs_get_u32(my_handle, "Y_SF2_14_BIT", ((uint32_t*)&mpu9250_handle->mag.cal.scale_factors2[INV_MAG_PRECISION_14_BITS].array[Y_POS])));
    ESP_ERROR_CHECK(nvs_get_u32(my_handle, "Z_SF2_14_BIT", ((uint32_t*)&mpu9250_handle->mag.cal.scale_factors2[INV_MAG_PRECISION_14_BITS].array[Z_POS])));
    ESP_ERROR_CHECK(nvs_get_u32(my_handle, "X_SF2_16_BIT", ((uint32_t*)&mpu9250_handle->mag.cal.scale_factors2[INV_MAG_PRECISION_16_BITS].array[X_POS])));
    ESP_ERROR_CHECK(nvs_get_u32(my_handle, "Y_SF2_16_BIT", ((uint32_t*)&mpu9250_handle->mag.cal.scale_factors2[INV_MAG_PRECISION_16_BITS].array[Y_POS])));
    ESP_ERROR_CHECK(nvs_get_u32(my_handle, "Z_SF2_16_BIT", ((uint32_t*)&mpu9250_handle->mag.cal.scale_factors2[INV_MAG_PRECISION_16_BITS].array[Z_POS])));

    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "X_VAR_14_BIT", &mpu9250_handle->mag.cal.var[INV_MAG_PRECISION_14_BITS].array[X_POS]));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "Y_VAR_14_BIT", &mpu9250_handle->mag.cal.var[INV_MAG_PRECISION_14_BITS].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "Z_VAR_14_BIT", &mpu9250_handle->mag.cal.var[INV_MAG_PRECISION_14_BITS].array[Z_POS]));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "X_VAR_16_BIT", &mpu9250_handle->mag.cal.var[INV_MAG_PRECISION_16_BITS].array[X_POS]));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "Y_VAR_16_BIT", &mpu9250_handle->mag.cal.var[INV_MAG_PRECISION_16_BITS].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "Z_VAR_16_BIT", &mpu9250_handle->mag.cal.var[INV_MAG_PRECISION_16_BITS].array[Z_POS]));

    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "X_SQM_14_BIT", &mpu9250_handle->mag.cal.sqm[INV_MAG_PRECISION_14_BITS].array[X_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "Y_SQM_14_BIT", &mpu9250_handle->mag.cal.sqm[INV_MAG_PRECISION_14_BITS].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "Z_SQM_14_BIT", &mpu9250_handle->mag.cal.sqm[INV_MAG_PRECISION_14_BITS].array[Z_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "X_SQM_16_BIT", &mpu9250_handle->mag.cal.sqm[INV_MAG_PRECISION_16_BITS].array[X_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "Y_SQM_16_BIT", &mpu9250_handle->mag.cal.sqm[INV_MAG_PRECISION_16_BITS].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "Z_SQM_16_BIT", &mpu9250_handle->mag.cal.sqm[INV_MAG_PRECISION_16_BITS].array[Z_POS]));

	printf("Mag: loaded calibration data from NVS \n");

    // Close
    nvs_close(my_handle);

	return ESP_OK;
}

static esp_err_t mpu9250_mag_filter_data(mpu9250_handle_t mpu9250_handle) {
	for(uint8_t i = 0; i < 3; i++) {
		if(mpu9250_handle->mag.cal.kalman[i].P > 0.01) {
			mpu9250_handle->mag.cal.kalman[i].sample = mpu9250_handle->raw_data.data_s_vector.mag[i];
			if(mpu9250_handle->mag.cal.kalman[i].initialized == 0) {
				mpu9250_handle->mag.cal.kalman[i].initialized = 1;
				mpu9250_handle->mag.cal.kalman[i].X = mpu9250_handle->mag.cal.kalman[i].sample;
			}
			mpu9250_handle->mag.cal.kalman[i].P = mpu9250_handle->mag.cal.kalman[i].P+mpu9250_handle->mag.cal.kalman[i].Q;
			mpu9250_handle->mag.cal.kalman[i].K = mpu9250_handle->mag.cal.kalman[i].P/(mpu9250_handle->mag.cal.kalman[i].P+mpu9250_handle->mag.cal.kalman[i].R);
			mpu9250_handle->mag.cal.kalman[i].X = mpu9250_handle->mag.cal.kalman[i].X + mpu9250_handle->mag.cal.kalman[i].K*(mpu9250_handle->mag.cal.kalman[i].sample - mpu9250_handle->mag.cal.kalman[i].X);
			mpu9250_handle->mag.cal.kalman[i].P=(1-mpu9250_handle->mag.cal.kalman[i].K)*mpu9250_handle->mag.cal.kalman[i].P;
		}
	}
	return ESP_OK;
}

static esp_err_t mpu9250_mag_calc_rpy(mpu9250_handle_t mpu9250_handle) {
	// I can not with mag only
	return ESP_OK;
}

static esp_err_t mpu9250_mag_prepare(mpu9250_handle_t mpu9250_handle) {
	/******************************************************************************************
	 * AK8963
	 ******************************************************************************************/
	// Read AK8963 who am i
	uint8_t ak8963_whoAmI = 0xFF;
	ESP_ERROR_CHECK(
			ReadAk8963Register(mpu9250_handle, AK8963_WIA));
    vTaskDelay(pdMS_TO_TICKS(10));
	ESP_ERROR_CHECK(
			mpu9250_read8(mpu9250_handle, MPU9250_EXT_SENS_DATA_00, &ak8963_whoAmI));

	if (ak8963_whoAmI != AK8963_WHO_AM_I_RESULT) {
		printf("MPU9250: AK8963 CONFIGURATION FAILED: [%d]\n", ak8963_whoAmI);
		return ESP_FAIL;
	} else {
		printf("MPU9250: (AK8963 communication OK)\n");
	}

    // Read calibration data in fuse mode
    ESP_ERROR_CHECK(
    		WriteAk8963Register(mpu9250_handle, AK8963_CNTL1, AK8963_FUSE_ROM));
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_ERROR_CHECK(ReadAk8963Registers(mpu9250_handle, AK8963_ASAX, 3));
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_ERROR_CHECK(mpu9250_read_buff(mpu9250_handle, MPU9250_EXT_SENS_DATA_00, mpu9250_handle->mag.cal.asa.array, 3*8));

    uint8_t asax = mpu9250_handle->mag.cal.asa.array[0];
    uint8_t asay = mpu9250_handle->mag.cal.asa.array[1];
    uint8_t asaz = mpu9250_handle->mag.cal.asa.array[2];
    mpu9250_handle->mag.cal.asa.array[0] =  asax;
    mpu9250_handle->mag.cal.asa.array[1] =  asay;
    mpu9250_handle->mag.cal.asa.array[2] =  asaz;

    for(uint8_t i = 0; i < 3; i++) {
    	mpu9250_handle->mag.cal.scale_factors.array[i] = (((float)(mpu9250_handle->mag.cal.asa.array[i] - 128))/2.0f / 128.0f + 1.0f) * (4912.0f / 32768.0f);
    }
    printf("mag_asa [%d, %d, %d\n", mpu9250_handle->mag.cal.asa.array[0], mpu9250_handle->mag.cal.asa.array[1], mpu9250_handle->mag.cal.asa.array[2]);
    printf("mag_scale [%5.5f, %5.5f, %5.5f\n", mpu9250_handle->mag.cal.scale_factors.array[0], mpu9250_handle->mag.cal.scale_factors.array[1], mpu9250_handle->mag.cal.scale_factors.array[2]);

    ESP_ERROR_CHECK(
    		WriteAk8963Register(mpu9250_handle, AK8963_CNTL1, AK8963_PWR_DOWN));
    vTaskDelay(pdMS_TO_TICKS(100));

    // Read AK8963 who am i
    ak8963_whoAmI = 0xFF;
    ESP_ERROR_CHECK(ReadAk8963Register(mpu9250_handle, AK8963_WIA));
    vTaskDelay(pdMS_TO_TICKS(10));
	ESP_ERROR_CHECK(
			mpu9250_read8(mpu9250_handle, MPU9250_EXT_SENS_DATA_00, &ak8963_whoAmI));

    if(ak8963_whoAmI != AK8963_WHO_AM_I_RESULT) {
    	printf("MPU9250: AK8963 CONFIGURATION FAILED: [%d]\n", ak8963_whoAmI);
    	return ESP_FAIL;
    } else {
    	printf("MPU9250: (AK8963 communication OK)\n");
    }
    ESP_ERROR_CHECK(mpu9250_mag_set_mode2_with_precision(mpu9250_handle, INV_MAG_PRECISION_16_BITS));
	printf("MPU9250: (AK8963 configuration OK)\n");

	return ESP_OK;

}

/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/
esp_err_t mpu9250_mag_init(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_mag_prepare(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_mag_load_calibration_data(mpu9250_handle)); // set offset, var, sqm, P, K
	ESP_ERROR_CHECK(mpu9250_mag_init_kalman_filter(mpu9250_handle)); // this reset P,K
	mpu9250_handle->mag.rpy.xyz.x = 0;
	mpu9250_handle->mag.rpy.xyz.y = 0;
	mpu9250_handle->mag.rpy.xyz.z = 0;

	printf("Mag offsets: [%2.3f,%2.3f,%2.3f]\n", mpu9250_handle->mag.cal.scaled_offset[mpu9250_handle->mag.precision].array[X_POS], mpu9250_handle->mag.cal.scaled_offset[mpu9250_handle->mag.precision].array[Y_POS], mpu9250_handle->mag.cal.scaled_offset[mpu9250_handle->mag.precision].array[Z_POS]);
	printf("Mag factors2: [%2.3f,%2.3f,%2.3f]\n", mpu9250_handle->mag.cal.scale_factors2[mpu9250_handle->mag.precision].array[X_POS], mpu9250_handle->mag.cal.scale_factors2[mpu9250_handle->mag.precision].array[Y_POS], mpu9250_handle->mag.cal.scale_factors2[mpu9250_handle->mag.precision].array[Z_POS]);
	ESP_ERROR_CHECK(mpu9250_mag_set_continuous_reading(mpu9250_handle));
	return ESP_OK;
}

esp_err_t mpu9250_mag_load_precision(mpu9250_handle_t mpu9250_handle) {
	uint8_t mag_conf = 0;

	// read cntl1
    ESP_ERROR_CHECK(ReadAk8963Register(mpu9250_handle, AK8963_CNTL1));
    vTaskDelay(pdMS_TO_TICKS(10));
	ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_EXT_SENS_DATA_00, &mag_conf));

	mpu9250_handle->mag.precision = ((AK8963_PRECISION_MASK & mag_conf) ? INV_MAG_PRECISION_16_BITS : INV_MAG_PRECISION_14_BITS);

	switch(mpu9250_handle->mag.precision) {
	case INV_MAG_PRECISION_14_BITS: {
		printf("MPU9250: Mag Precision 0.6 \n");
		break;
	}
	case INV_MAG_PRECISION_16_BITS: {
		printf("MPU9250: Mag Precision 0.15 \n");
		break;
	}
	default: {
		printf("MPU9250: Mag Precision UNKNOWN [%d]\n", mpu9250_handle->mag.precision);
		break;
	}
	}
	return ESP_OK;
}

esp_err_t mpu9250_mag_set_precision(mpu9250_handle_t mpu9250_handle, uint8_t precision) {
	mpu9250_handle->mag.precision=precision;
	ESP_ERROR_CHECK(mpu9250_mag_save_precision(mpu9250_handle));
	return ESP_OK;
}

esp_err_t mpu9250_mag_scale_data_in_body_frame(mpu9250_handle_t mpu9250_handle) {
	// allinea gli assi del mag con quelli del body frame e scala i valori con i fattori di scala di fabbrica e quelli corretti
	// applica gli scaled_offset calcolati sui valori già scalati e con assi convertiti
	mpu9250_handle->mag.body_frame_data.array[X_POS] = mpu9250_handle->mag.cal.kalman[Y_POS].X*mpu9250_handle->mag.cal.scale_factors.array[Y_POS]*mpu9250_handle->mag.cal.scale_factors2[mpu9250_handle->mag.precision].array[X_POS] - mpu9250_handle->mag.cal.scaled_offset[mpu9250_handle->mag.precision].array[X_POS];
	mpu9250_handle->mag.body_frame_data.array[Y_POS] = -(mpu9250_handle->mag.cal.kalman[X_POS].X*mpu9250_handle->mag.cal.scale_factors.array[X_POS]*mpu9250_handle->mag.cal.scale_factors2[mpu9250_handle->mag.precision].array[Y_POS]) - mpu9250_handle->mag.cal.scaled_offset[mpu9250_handle->mag.precision].array[Y_POS];
	mpu9250_handle->mag.body_frame_data.array[Z_POS] = -mpu9250_handle->mag.cal.kalman[Z_POS].X*mpu9250_handle->mag.cal.scale_factors.array[Z_POS]*mpu9250_handle->mag.cal.scale_factors2[mpu9250_handle->mag.precision].array[Z_POS] - mpu9250_handle->mag.cal.scaled_offset[mpu9250_handle->mag.precision].array[Z_POS];
	mpu9250_handle->mag.module = sqrt(mpu9250_handle->mag.body_frame_data.array[X_POS]*mpu9250_handle->mag.body_frame_data.array[X_POS]+mpu9250_handle->mag.body_frame_data.array[Y_POS]*mpu9250_handle->mag.body_frame_data.array[Y_POS]+mpu9250_handle->mag.body_frame_data.array[Z_POS]*mpu9250_handle->mag.body_frame_data.array[Z_POS]);
	return ESP_OK;
}

esp_err_t mpu9250_mag_update_state(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_mag_filter_data(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_mag_scale_data_in_body_frame(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_mag_calc_rpy(mpu9250_handle));
	return ESP_OK;
}

esp_err_t mpu9250_mag_save_calibration_data(mpu9250_handle_t mpu9250_handle) {
    nvs_handle_t my_handle;
    ESP_ERROR_CHECK(nvs_open("MAG_CAL", NVS_READWRITE, &my_handle));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "FLASHED", 1));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "X_ASA", mpu9250_handle->mag.cal.asa.array[X_POS]));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "Y_ASA", mpu9250_handle->mag.cal.asa.array[Y_POS]));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, "Z_ASA", mpu9250_handle->mag.cal.asa.array[Z_POS]));

    // in inertial frame axis
    ESP_ERROR_CHECK(nvs_set_u32(my_handle, "X_OFF_14_BIT", *((uint32_t*)&mpu9250_handle->mag.cal.scaled_offset[INV_MAG_PRECISION_14_BITS].array[X_POS])));
    ESP_ERROR_CHECK(nvs_set_u32(my_handle, "Y_OFF_14_BIT", *((uint32_t*)&mpu9250_handle->mag.cal.scaled_offset[INV_MAG_PRECISION_14_BITS].array[Y_POS])));
    ESP_ERROR_CHECK(nvs_set_u32(my_handle, "Z_OFF_14_BIT", *((uint32_t*)&mpu9250_handle->mag.cal.scaled_offset[INV_MAG_PRECISION_14_BITS].array[Z_POS])));
    ESP_ERROR_CHECK(nvs_set_u32(my_handle, "X_OFF_16_BIT", *((uint32_t*)&mpu9250_handle->mag.cal.scaled_offset[INV_MAG_PRECISION_16_BITS].array[X_POS])));
    ESP_ERROR_CHECK(nvs_set_u32(my_handle, "Y_OFF_16_BIT", *((uint32_t*)&mpu9250_handle->mag.cal.scaled_offset[INV_MAG_PRECISION_16_BITS].array[Y_POS])));
    ESP_ERROR_CHECK(nvs_set_u32(my_handle, "Z_OFF_16_BIT", *((uint32_t*)&mpu9250_handle->mag.cal.scaled_offset[INV_MAG_PRECISION_16_BITS].array[Z_POS])));

    // in inertial frame axis
    ESP_ERROR_CHECK(nvs_set_u32(my_handle, "X_SF2_14_BIT", *((uint32_t*)&mpu9250_handle->mag.cal.scale_factors2[INV_MAG_PRECISION_14_BITS].array[X_POS])));
    ESP_ERROR_CHECK(nvs_set_u32(my_handle, "Y_SF2_14_BIT", *((uint32_t*)&mpu9250_handle->mag.cal.scale_factors2[INV_MAG_PRECISION_14_BITS].array[Y_POS])));
    ESP_ERROR_CHECK(nvs_set_u32(my_handle, "Z_SF2_14_BIT", *((uint32_t*)&mpu9250_handle->mag.cal.scale_factors2[INV_MAG_PRECISION_14_BITS].array[Z_POS])));
    ESP_ERROR_CHECK(nvs_set_u32(my_handle, "X_SF2_16_BIT", *((uint32_t*)&mpu9250_handle->mag.cal.scale_factors2[INV_MAG_PRECISION_16_BITS].array[X_POS])));
    ESP_ERROR_CHECK(nvs_set_u32(my_handle, "Y_SF2_16_BIT", *((uint32_t*)&mpu9250_handle->mag.cal.scale_factors2[INV_MAG_PRECISION_16_BITS].array[Y_POS])));
    ESP_ERROR_CHECK(nvs_set_u32(my_handle, "Z_SF2_16_BIT", *((uint32_t*)&mpu9250_handle->mag.cal.scale_factors2[INV_MAG_PRECISION_16_BITS].array[Z_POS])));

    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "X_VAR_14_BIT", mpu9250_handle->mag.cal.var[INV_MAG_PRECISION_14_BITS].array[X_POS]));
    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "Y_VAR_14_BIT", mpu9250_handle->mag.cal.var[INV_MAG_PRECISION_14_BITS].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "Z_VAR_14_BIT", mpu9250_handle->mag.cal.var[INV_MAG_PRECISION_14_BITS].array[Z_POS]));
    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "X_VAR_16_BIT", mpu9250_handle->mag.cal.var[INV_MAG_PRECISION_16_BITS].array[X_POS]));
    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "Y_VAR_16_BIT", mpu9250_handle->mag.cal.var[INV_MAG_PRECISION_16_BITS].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "Z_VAR_16_BIT", mpu9250_handle->mag.cal.var[INV_MAG_PRECISION_16_BITS].array[Z_POS]));

    ESP_ERROR_CHECK(nvs_set_i16(my_handle, "X_SQM_14_BIT", mpu9250_handle->mag.cal.sqm[INV_MAG_PRECISION_14_BITS].array[X_POS]));
    ESP_ERROR_CHECK(nvs_set_i16(my_handle, "Y_SQM_14_BIT", mpu9250_handle->mag.cal.sqm[INV_MAG_PRECISION_14_BITS].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_set_i16(my_handle, "Z_SQM_14_BIT", mpu9250_handle->mag.cal.sqm[INV_MAG_PRECISION_14_BITS].array[Z_POS]));
    ESP_ERROR_CHECK(nvs_set_i16(my_handle, "X_SQM_16_BIT", mpu9250_handle->mag.cal.sqm[INV_MAG_PRECISION_16_BITS].array[X_POS]));
    ESP_ERROR_CHECK(nvs_set_i16(my_handle, "Y_SQM_16_BIT", mpu9250_handle->mag.cal.sqm[INV_MAG_PRECISION_16_BITS].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_set_i16(my_handle, "Z_SQM_16_BIT", mpu9250_handle->mag.cal.sqm[INV_MAG_PRECISION_16_BITS].array[Z_POS]));

    printf("Mag: Committing updates in NVS ... \n");
    ESP_ERROR_CHECK(nvs_commit(my_handle));

    // Close
    nvs_close(my_handle);

	return ESP_OK;
}

esp_err_t mpu9250_mag_set_mode2_with_precision(mpu9250_handle_t mpu9250_handle, uint8_t precision) {
	// Configure AK8963 Continuous Mode
	switch(precision) {
	case INV_MAG_PRECISION_14_BITS: {
	    ESP_ERROR_CHECK(
	    		WriteAk8963Register(mpu9250_handle, AK8963_CNTL1, 0x06));  // mode2 (100Hz) 14 bits
		break;
	}
	case INV_MAG_PRECISION_16_BITS: {
	    ESP_ERROR_CHECK(
	    		WriteAk8963Register(mpu9250_handle, AK8963_CNTL1, 0x16));  // mode2 (100Hz) 16bits
		break;
	}
	default: {
		return ESP_FAIL;
	}
	}
    mpu9250_handle->mag.precision = precision;
    ESP_ERROR_CHECK(mpu9250_mag_calc_precision_factor(mpu9250_handle));

    return ESP_OK;
}


esp_err_t mpu9250_mag_set_continuous_reading(mpu9250_handle_t mpu9250_handle) {
    // Mpu9250 start reading AK8963
    vTaskDelay(pdMS_TO_TICKS(100));
    ReadAk8963Registers(mpu9250_handle, AK8963_ST1, 8);
	printf("MPU9250: continuous reading for AK8963 on SLV0\n");
	return ESP_OK;
}

