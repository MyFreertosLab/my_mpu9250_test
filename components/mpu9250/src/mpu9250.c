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
	counter++;
//    xSemaphoreTake(((mpu9250_handle_t)mpu9250_handle)->spi_mutex, portMAX_DELAY);

	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length = 8;                    //Transaction length is in bits.
	t.addr = (MPU9250_READ_FLAG | MPU9250_INT_STATUS);
	t.flags = SPI_TRANS_USE_RXDATA;
	ret = spi_device_polling_transmit(((mpu9250_handle_t)mpu9250_handle)->device_handle, &t);  //Transmit!
	if (ret == ESP_OK) {
		((mpu9250_handle_t)mpu9250_handle)->int_status=t.rx_data[0];
	}
//    xSemaphoreGive(((mpu9250_handle_t)mpu9250_handle)->spi_mutex);
}

esp_err_t mpu9250_write8(mpu9250_handle_t mpu9250_handle, uint8_t reg, uint8_t val) {
//    xSemaphoreTake(((mpu9250_handle_t)mpu9250_handle)->spi_mutex, portMAX_DELAY);
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length = 8;                    //Transaction length is in bits.
	t.addr = reg;
	t.tx_data[0] = val;
	t.flags = SPI_TRANS_USE_TXDATA;
	ret = spi_device_polling_transmit(((mpu9250_handle_t)mpu9250_handle)->device_handle, &t);  //Transmit!
//    xSemaphoreGive(((mpu9250_handle_t)mpu9250_handle)->spi_mutex);
	return ret;
}
esp_err_t mpu9250_init(mpu9250_handle_t mpu9250_handle) {
	printf("PREPARE GPIO [%d]\n", mpu9250_handle->int_pin);

	// prepare GPIO
    gpio_config_t io_conf={
        .intr_type=GPIO_PIN_INTR_POSEDGE,
        .mode=GPIO_MODE_INPUT,
        .pull_down_en=1,
        .pin_bit_mask=(1<<mpu9250_handle->int_pin)
    };
    gpio_config(&io_conf);

	ESP_ERROR_CHECK(gpio_set_intr_type(mpu9250_handle->int_pin, GPIO_PIN_INTR_POSEDGE));
	ESP_ERROR_CHECK(gpio_install_isr_service(0));
	ESP_ERROR_CHECK(gpio_isr_handler_add(mpu9250_handle->int_pin, mpu9250_isr, (void*)mpu9250_handle));

    // set Configuration Register
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_CONFIG, 0x01)); // gyro bandwidth 184Hz

    // set Gyro Configuration Register
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_GYRO_CONFIG, 0x0)); // +-250deg/sec

    // set Acc Conf1 Register
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_ACCEL_CONFIG, 0x01)); // +-4g

    // set Acc Conf2 Register
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_ACCEL_CONFIG_2, 0x02)); // bandwidth 92Hz

    // set PwrMgmt2 Register
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_PWR_MGMT_2, 0x00));

    // set PwrMgmt1 Register
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_PWR_MGMT_1, 0x01)); // clock, no cycle

    // set Int Status Register
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_INT_STATUS, 0x00)); // reset all interrupts?

    // set Int Enable Register
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_INT_ENABLE, 0x01)); // data ready int

    return ESP_OK;
}

esp_err_t mpu9250_whoami(mpu9250_handle_t mpu9250_handle) {
//    xSemaphoreTake(((mpu9250_handle_t)mpu9250_handle)->spi_mutex, portMAX_DELAY);
	esp_err_t ret;
	spi_transaction_t t;
	printf("Keep Alive ...\n");
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length = 8;                    //Transaction length is in bits.
	t.addr = (MPU9250_READ_FLAG | MPU9250_WHO_AM_I);
	t.flags = SPI_TRANS_USE_RXDATA;
	ret = spi_device_polling_transmit(mpu9250_handle->device_handle, &t);  //Transmit!
	if (ret == ESP_OK) {
		printf("MPU9250 ID: [%d][bits: %d]\n", (t.rx_data[0]), t.rxlength);
	} else {
		printf("ERROR: [%d]\n", ret);
	}
//    xSemaphoreGive(((mpu9250_handle_t)mpu9250_handle)->spi_mutex);
	return ret;
}
esp_err_t mpu9250_get_data(mpu9250_handle_t mpu9250_handle, mpu9250_raw_data_buff_t mpu9250_raw_data_buff) {
	printf("COUNTER [%d]\n", counter);
    return ESP_OK;
}
