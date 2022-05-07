/*
 * mpu9250_baro.c
 *
 *  Created on: 30 apr 2022
 *      Author: andrea
 */

#include <mpu9250_spi.h>
#include <mpu9250.h>
#include <mpu9250_baro.h>
#include <driver/i2c.h>
#include <math.h>

/************************************************************************
 ************** P R I V A T E  I M P L E M E N T A T I O N **************
 ************************************************************************/
static esp_err_t WriteBmp388Register(mpu9250_handle_t mpu9250_handle, uint8_t reg,
		uint8_t data) {
	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_ADDR, BMP388_ADDRESS));
	ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_REG, reg));
	ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_DO, data));
	uint8_t slv_en = (I2C_SLV_EN | 0x01);
	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_CTRL, slv_en));
	return ESP_OK;
}

static esp_err_t ReadBmp388Register(mpu9250_handle_t mpu9250_handle, uint8_t reg) {
	uint8_t addr = (BMP388_ADDRESS | MPU9250_I2C_READ_FLAG);
	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_ADDR, addr));
	ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_REG, reg));
	uint8_t slv_en = (I2C_SLV_EN | 0x01);
	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_CTRL, slv_en));
	return ESP_OK;
}

static esp_err_t ReadBmp388Registers(mpu9250_handle_t mpu9250_handle, uint8_t reg, uint8_t count) {
	uint8_t addr = (BMP388_ADDRESS | MPU9250_I2C_READ_FLAG);
	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV1_ADDR, addr));
	ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV1_REG, reg));
	uint8_t slv_en = (I2C_SLV_EN | count);
	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV1_CTRL, slv_en));
	return ESP_OK;
}

esp_err_t mpu9250_baro_parse_coefficients(mpu9250_handle_t mpu9250_handle, uint8_t* buff) {
	mpu9250_handle->baro.cal.par_t1   = ((buff[1] << 8) | buff[0]);
	mpu9250_handle->baro.cal.par_t2   = ((buff[3] << 8) | buff[2]);
	mpu9250_handle->baro.cal.par_t3   = buff[4];
	mpu9250_handle->baro.cal.par_p1   = ((buff[6] << 8) | buff[5]);
	mpu9250_handle->baro.cal.par_p2   = ((buff[8] << 8) | buff[7]);
	mpu9250_handle->baro.cal.par_p3   = buff[9];
	mpu9250_handle->baro.cal.par_p4   = buff[10];
	mpu9250_handle->baro.cal.par_p5   = ((buff[12] << 8) | buff[11]);
	mpu9250_handle->baro.cal.par_p6   = ((buff[14] << 8) | buff[13]);
	mpu9250_handle->baro.cal.par_p7   = buff[15];
	mpu9250_handle->baro.cal.par_p8   = buff[16];
	mpu9250_handle->baro.cal.par_p9   = ((buff[18] << 8) | buff[17]);
	mpu9250_handle->baro.cal.par_p10  = buff[19];
	mpu9250_handle->baro.cal.par_p11  = buff[20];

	double temp_var  = 0.00390625f; // 2^(-8)
	mpu9250_handle->baro.cal.q_par_t1 = ((double)mpu9250_handle->baro.cal.par_t1/temp_var);
    temp_var = 1073741824.0f; // 2^30
	mpu9250_handle->baro.cal.q_par_t2 = ((double)mpu9250_handle->baro.cal.par_t2/temp_var);
    temp_var = 281474976710656.0f; // 2^48
	mpu9250_handle->baro.cal.q_par_t3 = ((double)mpu9250_handle->baro.cal.par_t3/temp_var);
    temp_var = 1048576.0f; // 2^20
	mpu9250_handle->baro.cal.q_par_p1 = (((double)mpu9250_handle->baro.cal.par_p1 - (16384))/temp_var);
	temp_var = 536870912.0f; // 2^29
	mpu9250_handle->baro.cal.q_par_p2 = (((double)mpu9250_handle->baro.cal.par_p2 - (16384))/temp_var);
    temp_var = 4294967296.0f; // 2^32
	mpu9250_handle->baro.cal.q_par_p3 = ((double)mpu9250_handle->baro.cal.par_p3/temp_var);
    temp_var = 137438953472.0f; // 2^37
	mpu9250_handle->baro.cal.q_par_p4 = ((double)mpu9250_handle->baro.cal.par_p4/temp_var);
    temp_var = 0.125f; // 2^(-3)
	mpu9250_handle->baro.cal.q_par_p5 = ((double)mpu9250_handle->baro.cal.par_p5/temp_var);
    temp_var = 64.0f; // 2^6
	mpu9250_handle->baro.cal.q_par_p6 = ((double)mpu9250_handle->baro.cal.par_p6/temp_var);
    temp_var = 256.0f; // 2^8
	mpu9250_handle->baro.cal.q_par_p7 = ((double)mpu9250_handle->baro.cal.par_p7/temp_var);
    temp_var = 32768.0f; // 2^15
	mpu9250_handle->baro.cal.q_par_p8 = ((double)mpu9250_handle->baro.cal.par_p8/temp_var);
    temp_var = 281474976710656.0f; // 2^48
	mpu9250_handle->baro.cal.q_par_p9 = ((double)mpu9250_handle->baro.cal.par_p9/temp_var);
    temp_var = 281474976710656.0f; // 2^48
	mpu9250_handle->baro.cal.q_par_p10 = ((double)mpu9250_handle->baro.cal.par_p10/temp_var);
    temp_var = 36893488147419103232.0f; // 2^65
	mpu9250_handle->baro.cal.q_par_p11 = ((double)mpu9250_handle->baro.cal.par_p11/temp_var);

	printf("BMP388: coefficients:\n");
	printf("BMP388: t1-t3:[%5.9f,%5.9f,%5.9f]\n", mpu9250_handle->baro.cal.q_par_t1,mpu9250_handle->baro.cal.q_par_t2,mpu9250_handle->baro.cal.q_par_t3);
	printf("BMP388: p1-p3:[%5.9f,%5.9f,%5.9f]\n", mpu9250_handle->baro.cal.q_par_p1,mpu9250_handle->baro.cal.q_par_p2,mpu9250_handle->baro.cal.q_par_p3);
	printf("BMP388: p4-p6:[%5.9f,%5.9f,%5.9f]\n", mpu9250_handle->baro.cal.q_par_p4,mpu9250_handle->baro.cal.q_par_p5,mpu9250_handle->baro.cal.q_par_p6);
	printf("BMP388: p7-p9:[%5.9f,%5.9f,%5.9f]\n", mpu9250_handle->baro.cal.q_par_p7,mpu9250_handle->baro.cal.q_par_p8,mpu9250_handle->baro.cal.q_par_p9);
	printf("BMP388: p10-p11:[%5.9f,%5.9f]\n", mpu9250_handle->baro.cal.q_par_p10,mpu9250_handle->baro.cal.q_par_p11);

	return ESP_OK;
}


/*
 * Mode: Normal
 * Resolution: Standard
 * Temperature Oversampling: NO
 * Pressure Oversampling: x8
 * IIR Coeffcient: 2
 * IDD: 570
 * ODR: 50Hz
 * RMS Noise: 11
 *
 */
static esp_err_t mpu9250_baro_prepare(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_baro_load_coefficients(mpu9250_handle));

	ESP_ERROR_CHECK(mpu9250_baro_set_oversampling(mpu9250_handle, BMP388_NO_OVERSAMPLING, BMP388_OVERSAMPLING_8X));
	vTaskDelay(pdMS_TO_TICKS(10));

	ESP_ERROR_CHECK(mpu9250_baro_set_IIR_coefficient(mpu9250_handle, BMP388_IIR_FILTER_COEFF_3));
	vTaskDelay(pdMS_TO_TICKS(10));

	ESP_ERROR_CHECK(mpu9250_baro_set_output_data_rate(mpu9250_handle, BMP388_ODR_50_HZ));
	vTaskDelay(pdMS_TO_TICKS(10));

	ESP_ERROR_CHECK(mpu9250_baro_set_pwr_ctrl(mpu9250_handle, BMP388_PWR_CTRL_NORMAL_MODE | BMP388_PWR_CTRL_TEMPERATURE_ENABLED | BMP388_PWR_CTRL_PRESSURE_ENABLED));
	vTaskDelay(pdMS_TO_TICKS(10));

	return ESP_OK;
}


static void mpu9250_baro_compensate_temperature(mpu9250_handle_t mpu9250_handle)
{
    uint32_t uncomp_temp = mpu9250_handle->raw_data.data_s_xyz.temperature;
    double partial_data1;
    double partial_data2;

    partial_data1 = (double)(uncomp_temp - mpu9250_handle->baro.cal.q_par_t1);
    partial_data2 = (double)(partial_data1 * mpu9250_handle->baro.cal.q_par_t2);

    mpu9250_handle->baro.temperature = partial_data2 + (partial_data1 * partial_data1) * mpu9250_handle->baro.cal.q_par_t3;
}

static void mpu9250_baro_compensate_pressure(mpu9250_handle_t mpu9250_handle)
{

//	BMP388: coefficients:
//	BMP388: t1-t3:[7120896,000000000,0,000017137,-0,000000000]
//	BMP388: p1-p3:[0,000389099,-0,000004224,0,000000008]
//	BMP388: p4-p6:[0,000000000,202464,000000000,481,406250000]
//	BMP388: p7-p9:[-0,050781250,-0,000305176,0,000000000]
//	BMP388: p10-p11:[0,000000000,-0,000000000]
//  RP[7198273]
//  T[24,69668107]

    /* Temporary variables used for compensation */
    double partial_data1;
    double partial_data2;
    double partial_data3;
    double partial_data4;
    double partial_out1;
    double partial_out2;
    double dPressure = mpu9250_handle->raw_data.data_s_xyz.pressure;

    partial_data1 = mpu9250_handle->baro.cal.q_par_p6 * mpu9250_handle->baro.temperature;
    partial_data2 = mpu9250_handle->baro.cal.q_par_p7 * mpu9250_handle->baro.temperature*mpu9250_handle->baro.temperature;
    partial_data3 = mpu9250_handle->baro.cal.q_par_p8 * mpu9250_handle->baro.temperature*mpu9250_handle->baro.temperature*mpu9250_handle->baro.temperature;
    partial_out1 = mpu9250_handle->baro.cal.q_par_p5 + partial_data1 + partial_data2 + partial_data3;
    partial_data1 = mpu9250_handle->baro.cal.q_par_p2 * mpu9250_handle->baro.temperature;
    partial_data2 = mpu9250_handle->baro.cal.q_par_p3 * mpu9250_handle->baro.temperature*mpu9250_handle->baro.temperature;
    partial_data3 = mpu9250_handle->baro.cal.q_par_p4 * mpu9250_handle->baro.temperature*mpu9250_handle->baro.temperature*mpu9250_handle->baro.temperature;
    partial_out2 = dPressure *
                   (mpu9250_handle->baro.cal.q_par_p1 + partial_data1 + partial_data2 + partial_data3);
    partial_data1 = dPressure*dPressure;
    partial_data2 = mpu9250_handle->baro.cal.q_par_p9 + mpu9250_handle->baro.cal.q_par_p10 * mpu9250_handle->baro.temperature;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + dPressure*dPressure*dPressure * mpu9250_handle->baro.cal.q_par_p11;
    mpu9250_handle->baro.pressure = partial_out1 + partial_out2 + partial_data4;

}

/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/
esp_err_t mpu9250_baro_set_oversampling(mpu9250_handle_t mpu9250_handle, uint8_t osr_t, uint8_t osr_p) {
	ESP_ERROR_CHECK(WriteBmp388Register(mpu9250_handle, BMP388_REG_OSR, ((osr_t << 3) | osr_p)));
	return ESP_OK;
}
esp_err_t mpu9250_baro_get_oversampling(mpu9250_handle_t mpu9250_handle, uint8_t* osr) {
	ESP_ERROR_CHECK(ReadBmp388Register(mpu9250_handle, BMP388_REG_OSR));
	vTaskDelay(pdMS_TO_TICKS(10));
	ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_EXT_SENS_DATA_00,osr));
	return ESP_OK;
}
esp_err_t mpu9250_baro_set_IIR_coefficient(mpu9250_handle_t mpu9250_handle, uint8_t iir_coeff) {
	ESP_ERROR_CHECK(WriteBmp388Register(mpu9250_handle, BMP388_REG_CONFIG, iir_coeff));
	return ESP_OK;
}
esp_err_t mpu9250_baro_get_IIR_coefficient(mpu9250_handle_t mpu9250_handle, uint8_t* config) {
	ESP_ERROR_CHECK(ReadBmp388Register(mpu9250_handle, BMP388_REG_CONFIG));
	vTaskDelay(pdMS_TO_TICKS(10));
	ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_EXT_SENS_DATA_00,config));
	return ESP_OK;
}
esp_err_t mpu9250_baro_set_output_data_rate(mpu9250_handle_t mpu9250_handle, uint8_t odr_hz) {
	ESP_ERROR_CHECK(WriteBmp388Register(mpu9250_handle, BMP388_REG_ODR, odr_hz));
	return ESP_OK;
}
esp_err_t mpu9250_baro_get_output_data_rate(mpu9250_handle_t mpu9250_handle, uint8_t* odr) {
	ESP_ERROR_CHECK(ReadBmp388Register(mpu9250_handle, BMP388_REG_ODR));
	vTaskDelay(pdMS_TO_TICKS(10));
	ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_EXT_SENS_DATA_00,odr));
	return ESP_OK;
}
esp_err_t mpu9250_baro_set_pwr_ctrl(mpu9250_handle_t mpu9250_handle, uint8_t value) {
	ESP_ERROR_CHECK(WriteBmp388Register(mpu9250_handle, BMP388_REG_PWR_CTRL, value));
	return ESP_OK;
}
esp_err_t mpu9250_baro_get_pwr_ctrl(mpu9250_handle_t mpu9250_handle, uint8_t* pwr_ctrl) {
	ESP_ERROR_CHECK(ReadBmp388Register(mpu9250_handle, BMP388_REG_PWR_CTRL));
	vTaskDelay(pdMS_TO_TICKS(10));
	ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_EXT_SENS_DATA_00,pwr_ctrl));
	return ESP_OK;
}
esp_err_t mpu9250_baro_load_coefficients(mpu9250_handle_t mpu9250_handle) {
	uint8_t buff[BMP388_NUM_BYTES_CALIB_DATA] = { '\0' };
	for(uint8_t i = 0; i < BMP388_NUM_BYTES_CALIB_DATA; i++) {
		ESP_ERROR_CHECK(ReadBmp388Register(mpu9250_handle, BMP388_REG_CALIB_DATA+i));
		vTaskDelay(pdMS_TO_TICKS(10));
		ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_EXT_SENS_DATA_00,(void*)(buff+i)));
	}
	ESP_ERROR_CHECK(mpu9250_baro_parse_coefficients(mpu9250_handle, buff));
	return ESP_OK;
}

esp_err_t mpu9250_baro_test(mpu9250_handle_t mpu9250_handle) {
	uint8_t bmp388_whoAmI = 0x00;
	esp_err_t result = ReadBmp388Register(mpu9250_handle, BMP388_REG_CHIP_ID);
	if(result == ESP_OK) {
		vTaskDelay(pdMS_TO_TICKS(10));
		result = mpu9250_read8(mpu9250_handle, MPU9250_EXT_SENS_DATA_00,&bmp388_whoAmI);
		if(result == ESP_OK) {
			if (bmp388_whoAmI != BMP388_CHIP_ID) {
				printf("BMP388 Error reading chip ID: [%d]\n",bmp388_whoAmI);
				result = ESP_FAIL;
			} else {
				printf("BMP388 communication OK: [%d]\n",bmp388_whoAmI);
			}
		}
	}
	return result;
}


esp_err_t mpu9250_baro_set_continuous_reading(mpu9250_handle_t mpu9250_handle) {
    // Mpu9250 start reading BMP388
    vTaskDelay(pdMS_TO_TICKS(100));
    ReadBmp388Registers(mpu9250_handle, BMP388_REG_DATA, 6);
	printf("MPU9250: continuous reading for BMP388 on SLV1\n");
	return ESP_OK;
}

esp_err_t mpu9250_baro_calc_altitude(mpu9250_handle_t mpu9250_handle) {
	float atmospheric = mpu9250_handle->baro.pressure / 100.0F;
	mpu9250_handle->baro.altitude = 44330.0 * (1.0 - pow(atmospheric / BMP388_SEA_LEVEL_PRESSURE_HPA, 0.1903));
	return ESP_OK;
}
esp_err_t mpu9250_baro_compensate(mpu9250_handle_t mpu9250_handle) {
	mpu9250_baro_compensate_temperature(mpu9250_handle);
	mpu9250_baro_compensate_pressure(mpu9250_handle);
	mpu9250_baro_calc_altitude(mpu9250_handle);
	return ESP_OK;
}
esp_err_t mpu9250_baro_init(mpu9250_handle_t mpu9250_handle) {
	esp_err_t result = mpu9250_baro_test(mpu9250_handle);
	mpu9250_handle->baro.present = 0x00;
	if(result == ESP_OK) {
		ESP_ERROR_CHECK(mpu9250_baro_prepare(mpu9250_handle));
		mpu9250_handle->baro.present = 0x01;
		uint8_t val = 0x00;
		ESP_ERROR_CHECK(mpu9250_baro_get_oversampling(mpu9250_handle, &val));
		vTaskDelay(pdMS_TO_TICKS(10));
		printf("BMP388: OSR=[%d]\n", val);
		ESP_ERROR_CHECK(mpu9250_baro_get_IIR_coefficient(mpu9250_handle, &val));
		vTaskDelay(pdMS_TO_TICKS(10));
		printf("BMP388: CONFIG=[%d]\n", val);
		ESP_ERROR_CHECK(mpu9250_baro_get_output_data_rate(mpu9250_handle, &val));
		vTaskDelay(pdMS_TO_TICKS(10));
		printf("BMP388: ODR=[%d]\n", val);
		ESP_ERROR_CHECK(mpu9250_baro_get_pwr_ctrl(mpu9250_handle, &val));
		vTaskDelay(pdMS_TO_TICKS(10));
		printf("BMP388: PWR_CTRL=[%d]\n", val);
	}
	return result;
}

