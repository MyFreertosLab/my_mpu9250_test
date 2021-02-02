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

uint32_t counter = 0;

//This function is called with every interrupt
static void IRAM_ATTR mpu9250_isr(void* mpu9250_handle)
{
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	counter++;
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

esp_err_t mpu9250_init(mpu9250_handle_t mpu9250_handle) {
	mpu9250_handle->data_ready_task_handle=xTaskGetCurrentTaskHandle();

    // set Configuration Register
	printf("MPU9250: Gyro bandwidth 184Hz\n");
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_CONFIG, 0x01)); // gyro bandwidth 184Hz

    // set Gyro Configuration Register
	printf("MPU9250: Gyro +-250deg/sec\n");
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_GYRO_CONFIG, 0x0)); // +-250deg/sec

    // set Acc Conf1 Register
	printf("MPU9250: Accel +-16g\n");
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_ACCEL_CONFIG, 0x18));

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

esp_err_t mpu9250_set_acc_8g(mpu9250_handle_t mpu9250_handle) {
    // set Acc Conf1 Register
	printf("MPU9250: Accel +-8g\n");
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_ACCEL_CONFIG, 0x10));
    return ESP_OK;
}

esp_err_t mpu9250_test_connection(mpu9250_handle_t mpu9250_handle) {
	mpu9250_load_whoami(mpu9250_handle);
	return (mpu9250_handle->whoami == MPU9250_ID ? ESP_OK : ESP_FAIL);

}

esp_err_t mpu9250_load_raw_data(mpu9250_handle_t mpu9250_handle) {
	uint8_t buff[26];
	esp_err_t ret = mpu9250_read_buff(mpu9250_handle, MPU9250_ACCEL_XOUT_H, buff, 26*8);
	mpu9250_handle->raw_data.data_s_xyz.accel_data_x = ((buff[0] << 8) + buff[1]) - mpu9250_handle->acc_bias.xyz.x;
	mpu9250_handle->raw_data.data_s_xyz.accel_data_y = ((buff[2] << 8) + buff[3]) - mpu9250_handle->acc_bias.xyz.y;
	mpu9250_handle->raw_data.data_s_xyz.accel_data_z = ((buff[4] << 8) + buff[5]) - mpu9250_handle->acc_bias.xyz.z;
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
	buff[1] = mpu9250_handle->acc_offset.xyz.x & 0x0F;
	buff[2] = 0;
	buff[3] = mpu9250_handle->acc_offset.xyz.y >> 8;
	buff[4] = mpu9250_handle->acc_offset.xyz.y & 0x0F;
	buff[5] = 0;
	buff[6] = mpu9250_handle->acc_offset.xyz.z >> 8;
	buff[7] = mpu9250_handle->acc_offset.xyz.z & 0x00FF;

	esp_err_t ret = mpu9250_write_buff(mpu9250_handle, MPU9250_XA_OFFSET_H, buff, 8*8);
	return ret;
}
