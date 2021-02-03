/**
 *  @addtogroup  DRIVERS Sensor Driver Layer
 *  @brief       Hardware drivers for mp9250
 *
 *  @{
 *      @file       mpu9250.c
 *      @brief      An SPI-based driver for mpu9250
 *      @details    This driver currently works for the following devices:
 *                  MPU6500
 *                  MPU9250 (or MPU6500 w/ AK8963 on the auxiliary bus)
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <mpu9250.h>
#include <driver/gpio.h>


/************************************************************************
 ************** P R I V A T E  I M P L E M E N T A T I O N **************
 ************************************************************************/

//This function is called with every interrupt
static void IRAM_ATTR mpu9250_isr(void* mpu9250_handle)
{
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(((mpu9250_handle_t)mpu9250_handle)->data_ready_task_handle, &xHigherPriorityTaskWoken);
	if( xHigherPriorityTaskWoken) {
		portYIELD_FROM_ISR(); // this wakes up sample_timer_task immediately
	}
}

static esp_err_t mpu9250_read8(mpu9250_handle_t mpu9250_handle, uint8_t reg, uint8_t* val) {
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length = 8;                    //Transaction length is in bits.
	t.addr = (MPU9250_READ_FLAG | reg);
	t.flags = SPI_TRANS_USE_RXDATA;
	ret = spi_device_polling_transmit(((mpu9250_handle_t)mpu9250_handle)->device_handle, &t);  //Transmit!
	if (ret == ESP_OK) {
	  	  *val=t.rx_data[0];
	}
	return ret;
}

static esp_err_t mpu9250_write8(mpu9250_handle_t mpu9250_handle, uint8_t reg, uint8_t val) {
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length = 8;                    //Transaction length is in bits.
	t.addr = reg;
	t.tx_data[0] = val;
	t.flags = SPI_TRANS_USE_TXDATA;
	ret = spi_device_polling_transmit(((mpu9250_handle_t)mpu9250_handle)->device_handle, &t);  //Transmit!
	return ret;
}

static esp_err_t mpu9250_read_buff(mpu9250_handle_t mpu9250_handle, uint8_t reg, void* buff, uint8_t length) {
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length = length;                    //Transaction length is in bits.
	t.addr = (MPU9250_READ_FLAG | reg);
	t.rx_buffer=buff;
	ret = spi_device_polling_transmit(((mpu9250_handle_t)mpu9250_handle)->device_handle, &t);  //Transmit!
	return ret;
}

static esp_err_t mpu9250_write_buff(mpu9250_handle_t mpu9250_handle, uint8_t reg, void* buff, uint8_t length) {
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length = length;              //Transaction length is in bits.
	t.addr = reg;
	t.tx_buffer=buff;
	ret = spi_device_polling_transmit(((mpu9250_handle_t)mpu9250_handle)->device_handle, &t);  //Transmit!
	return ret;
}

/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/

esp_err_t mpu9250_init(mpu9250_handle_t mpu9250_handle) {
	mpu9250_handle->data_ready_task_handle=xTaskGetCurrentTaskHandle();

    // set Configuration Register
	printf("MPU9250: Gyro bandwidth 184Hz\n");
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_CONFIG, 0x01)); // gyro bandwidth 184Hz

    // set Gyro Configuration Register
	printf("MPU9250: Gyro +-250deg/sec\n");
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_GYRO_CONFIG, 0x0)); // +-250deg/sec

    // set Acc Conf1 Register
	mpu9250_handle->acc_fsr=INV_FSR_16G;
	ESP_ERROR_CHECK(mpu9250_save_acc_fsr(mpu9250_handle));

    // set Acc Conf2 Register
	printf("MPU9250: Accel bandwidth 92Hz\n");
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_ACCEL_CONFIG_2, 0x02)); // bandwidth 92Hz

    // set PwrMgmt2 Register
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_PWR_MGMT_2, 0x00));

    // set PwrMgmt1 Register
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_PWR_MGMT_1, 0x01)); // clock, no cycle

    // set Int Status Register
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_INT_STATUS, 0x00)); // reset all interrupts?

    // set Int Enable Register
	printf("MPU9250: Enable Interrupts\n");
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_INT_ENABLE, 0x01)); // data ready int

	// prepare GPIO Interrupt
	printf("MPU9250: Gpio interrupt pin [%d]\n", mpu9250_handle->int_pin);
    gpio_config_t io_conf={
        .intr_type=GPIO_PIN_INTR_POSEDGE,
        .mode=GPIO_MODE_INPUT,
        .pull_down_en=1,
        .pin_bit_mask=(1<<mpu9250_handle->int_pin)
    };
    gpio_config(&io_conf);
	printf("MPU9250: Gpio interrupt pin configured\n");

	printf("MPU9250: Configuring gpio interrupts ....\n");
	ESP_ERROR_CHECK(gpio_set_intr_type(mpu9250_handle->int_pin, GPIO_PIN_INTR_POSEDGE));
	ESP_ERROR_CHECK(gpio_install_isr_service(0));
	ESP_ERROR_CHECK(gpio_isr_handler_add(mpu9250_handle->int_pin, mpu9250_isr, (void*)mpu9250_handle));
	printf("MPU9250: Gpio interrupts configured ..\n");

    return ESP_OK;
}

esp_err_t mpu9250_load_int_status(mpu9250_handle_t mpu9250_handle) {
	return mpu9250_read8(mpu9250_handle, MPU9250_INT_STATUS, &((mpu9250_handle_t)mpu9250_handle)->int_status);
}

esp_err_t mpu9250_load_whoami(mpu9250_handle_t mpu9250_handle) {
	esp_err_t ret = ESP_FAIL;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));        //Zero out the transaction
	t.length = 8;                    //Transaction length is in bits.
	t.addr = (MPU9250_READ_FLAG | MPU9250_WHO_AM_I);
	t.flags = SPI_TRANS_USE_RXDATA;
	ret = spi_device_polling_transmit(mpu9250_handle->device_handle, &t);  //Transmit!
	mpu9250_handle->whoami = (ret == ESP_OK ? t.rx_data[0] : 0);
	return ret;
}
esp_err_t mpu9250_load_acc_fsr(mpu9250_handle_t mpu9250_handle) {
	uint8_t acc_conf = 0;
    ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_ACCEL_CONFIG, &acc_conf));
    mpu9250_handle->acc_fsr = (acc_conf & (~MPU9250_ACC_FSR_MASK)) >> MPU9250_ACC_FSR_LBIT;
    ESP_ERROR_CHECK(mpu9250_calc_acc_lsb(mpu9250_handle));
	return ESP_OK;
}
esp_err_t mpu9250_save_acc_fsr(mpu9250_handle_t mpu9250_handle) {
	uint8_t acc_conf = 0;
	uint8_t acc_conf_req = mpu9250_handle->acc_fsr;
    ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_ACCEL_CONFIG, &acc_conf));
    uint8_t toWrite = (acc_conf & MPU9250_ACC_FSR_MASK) | ((mpu9250_handle->acc_fsr << MPU9250_ACC_FSR_LBIT)& (~MPU9250_ACC_FSR_MASK));
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_ACCEL_CONFIG, toWrite));
    ESP_ERROR_CHECK(mpu9250_load_acc_fsr(mpu9250_handle));

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

esp_err_t mpu9250_test_connection(mpu9250_handle_t mpu9250_handle) {
	mpu9250_load_whoami(mpu9250_handle);
	return (mpu9250_handle->whoami == MPU9250_ID ? ESP_OK : ESP_FAIL);

}

esp_err_t mpu9250_load_raw_data(mpu9250_handle_t mpu9250_handle) {
	uint8_t buff[26];
	esp_err_t ret = mpu9250_read_buff(mpu9250_handle, MPU9250_ACCEL_XOUT_H, buff, 26*8);
	mpu9250_handle->raw_data.data_s_xyz.accel_data_x = ((buff[0] << 8) + buff[1]) - mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.x;
	mpu9250_handle->raw_data.data_s_xyz.accel_data_y = ((buff[2] << 8) + buff[3]) - mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.y;
	mpu9250_handle->raw_data.data_s_xyz.accel_data_z = ((buff[4] << 8) + buff[5]) - mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.z;
	return ret;
}
esp_err_t mpu9250_load_acc_offset(mpu9250_handle_t mpu9250_handle) {
	uint8_t buff[8];
	esp_err_t ret = mpu9250_read_buff(mpu9250_handle, MPU9250_XA_OFFSET_H, buff, 8*8);
	mpu9250_handle->acc_offset.xyz.x = (buff[0] << 8) + buff[1];
	mpu9250_handle->acc_offset.xyz.y = (buff[3] << 8) + buff[4];
	mpu9250_handle->acc_offset.xyz.z = (buff[6] << 8) + buff[7];
	return ret;
}
esp_err_t mpu9250_save_acc_offset(mpu9250_handle_t mpu9250_handle) {
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

esp_err_t mpu9250_calc_acc_bias(mpu9250_handle_t mpu9250_handle) {
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );
	// Calc Bias
	{
		// init values
		int32_t acc_sum[3] = {0,0,0};
		mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.x = 0;
		mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.y = 0;
		mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.z = 0;

		printf("Calculating Acc Bias ... \n");
		for(int i = 0; i < 30000; i++) {
			ulTaskNotifyTake( pdTRUE,xMaxBlockTime );
			ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));
			acc_sum[0] += mpu9250_handle->raw_data.data_s_xyz.accel_data_x;
			acc_sum[1] += mpu9250_handle->raw_data.data_s_xyz.accel_data_y;
			acc_sum[2] += mpu9250_handle->raw_data.data_s_xyz.accel_data_z;
		}

		// offsets respect vertical attitude
		mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.x = acc_sum[0]/30000;
		mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.y = acc_sum[1]/30000;
		mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.z = (acc_sum[2]/30000 - mpu9250_handle->acc_lsb);
		printf("Acc bias: [%d][%d][%d]\n", mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.x, mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.y,mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.z);
	}
	return ESP_OK;
}

esp_err_t mpu9250_calc_acc_biases(mpu9250_handle_t mpu9250_handle) {
	mpu9250_handle->acc_fsr=INV_FSR_8G;
	ESP_ERROR_CHECK(mpu9250_save_acc_fsr(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_calc_acc_bias(mpu9250_handle));

	mpu9250_handle->acc_fsr=INV_FSR_4G;
	ESP_ERROR_CHECK(mpu9250_save_acc_fsr(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_calc_acc_bias(mpu9250_handle));

	mpu9250_handle->acc_fsr=INV_FSR_2G;
	ESP_ERROR_CHECK(mpu9250_save_acc_fsr(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_calc_acc_bias(mpu9250_handle));

	return ESP_OK;
}

esp_err_t mpu9250_calc_acc_lsb(mpu9250_handle_t mpu9250_handle) {
	mpu9250_handle->acc_lsb = (32768 >> (mpu9250_handle->acc_fsr + 1));
	return ESP_OK;
}

esp_err_t mpu9250_calc_acc_offset(mpu9250_handle_t mpu9250_handle) {
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );

	if(mpu9250_handle->acc_fsr != INV_FSR_16G) {
		mpu9250_handle->acc_fsr=INV_FSR_16G;
		ESP_ERROR_CHECK(mpu9250_save_acc_fsr(mpu9250_handle));
	}


	// Original Acc offsets: [6684][-5436][9684]
//	mpu9250_handle->acc_offset.xyz.x = 6684;
//	mpu9250_handle->acc_offset.xyz.y = -5436;
//	mpu9250_handle->acc_offset.xyz.z = 9684;
//	ESP_ERROR_CHECK(mpu9250_save_acc_offset(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_load_acc_offset(mpu9250_handle));
	printf("Acc offsets: [%d][%d][%d]\n", mpu9250_handle->acc_offset.xyz.x, mpu9250_handle->acc_offset.xyz.y,mpu9250_handle->acc_offset.xyz.z);

	mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.x = 0;
	mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.y = 0;
	mpu9250_handle->acc_bias[mpu9250_handle->acc_fsr].xyz.z = 0;

	// dicard 10000 samples
	printf("Discarding 10000 Samples ... \n");
	for(int i = 0; i < 10000; i++) {
		ulTaskNotifyTake( pdTRUE,xMaxBlockTime );
	}

	// Offset means
	{
		int32_t acc_sum[3] = {0,0,0};
		printf("Calculating Acc Means ... \n");
		for(int i = 0; i < 30000; i++) {
			ulTaskNotifyTake( pdTRUE,xMaxBlockTime );
			ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));
			acc_sum[0] += mpu9250_handle->raw_data.data_s_xyz.accel_data_x;
			acc_sum[1] += mpu9250_handle->raw_data.data_s_xyz.accel_data_y;
			acc_sum[2] += mpu9250_handle->raw_data.data_s_xyz.accel_data_z;
		}

		// offsets respect vertical attitude
		mpu9250_handle->acc_offset.xyz.x += acc_sum[0]/30000;
		mpu9250_handle->acc_offset.xyz.y += acc_sum[1]/30000;
		mpu9250_handle->acc_offset.xyz.z += (acc_sum[2]/30000 - mpu9250_handle->acc_lsb);
		printf("Acc offsets: [%d][%d][%d]\n", mpu9250_handle->acc_offset.xyz.x, mpu9250_handle->acc_offset.xyz.y,mpu9250_handle->acc_offset.xyz.z);
		ESP_ERROR_CHECK(mpu9250_save_acc_offset(mpu9250_handle));
	}

	return ESP_OK;
}
