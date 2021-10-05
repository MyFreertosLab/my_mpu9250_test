/*
 * mpu9250_spi.c
 *
 *  Created on: 6 feb 2021
 *      Author: andrea
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <driver/gpio.h>
#include <mpu9250_spi.h>

// TODO: definire in Kdconfig
#ifdef CONFIG_IDF_TARGET_ESP32
#define MY_SPI_MPU9250_HOST HSPI_HOST
#define PIN_NUM_MISO GPIO_NUM_12
#define PIN_NUM_MOSI GPIO_NUM_13
#define PIN_NUM_CLK  GPIO_NUM_14
#define PIN_NUM_CS   GPIO_NUM_15
#define PIN_NUM_INT  GPIO_NUM_2

#elif defined CONFIG_IDF_TARGET_ESP32S2
#define MY_SPI_MPU9250_HOST SPI2_HOST
#define PIN_NUM_MISO GPIO_NUM_37
#define PIN_NUM_MOSI GPIO_NUM_35
#define PIN_NUM_CLK  GPIO_NUM_36
#define PIN_NUM_CS   GPIO_NUM_34
#define PIN_NUM_INT  GPIO_NUM_2
#endif

esp_err_t mpu9250_read8(mpu9250_handle_t mpu9250_handle, uint8_t reg, uint8_t* val) {
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

esp_err_t mpu9250_write8(mpu9250_handle_t mpu9250_handle, uint8_t reg, uint8_t val) {
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

esp_err_t mpu9250_read_buff(mpu9250_handle_t mpu9250_handle, uint8_t reg, void* buff, uint8_t length) {
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length = length;                    //Transaction length is in bits.
	t.addr = (MPU9250_READ_FLAG | reg);
	t.rx_buffer=buff;
	ret = spi_device_polling_transmit(((mpu9250_handle_t)mpu9250_handle)->device_handle, &t);  //Transmit!
	return ret;
}

esp_err_t mpu9250_write_buff(mpu9250_handle_t mpu9250_handle, uint8_t reg, void* buff, uint8_t length) {
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length = length;              //Transaction length is in bits.
	t.addr = reg;
	t.tx_buffer=buff;
	ret = spi_device_polling_transmit(((mpu9250_handle_t)mpu9250_handle)->device_handle, &t);  //Transmit!
	return ret;
}

esp_err_t mpu9250_spi_init(mpu9250_handle_t mpu9250_handle) {
	printf("MPU9250: Initializing SPI ... \n");
    memset(mpu9250_handle, 0, sizeof(mpu9250_init_t));

    mpu9250_handle->buscfg.miso_io_num = PIN_NUM_MISO;
    mpu9250_handle->buscfg.mosi_io_num = PIN_NUM_MOSI;
    mpu9250_handle->buscfg.sclk_io_num = PIN_NUM_CLK;
    mpu9250_handle->buscfg.quadwp_io_num = -1;
    mpu9250_handle->buscfg.quadhd_io_num = -1;
    mpu9250_handle->buscfg.max_transfer_sz = 256;

    mpu9250_handle->devcfg.spics_io_num = PIN_NUM_CS;
    mpu9250_handle->devcfg.clock_speed_hz = SPI_MASTER_FREQ_20M; //Clock out at 20 MHz
    mpu9250_handle->devcfg.address_bits = 8;
    mpu9250_handle->devcfg.mode = 3; //SPI mode 3
    mpu9250_handle->devcfg.queue_size = 7;  //We want to be able to queue 7 transactions at a time

    mpu9250_handle->int_pin=PIN_NUM_INT;

	printf("MPU9250: miso [%d]\n", mpu9250_handle->buscfg.miso_io_num);
	printf("MPU9250: mosi [%d]\n", mpu9250_handle->buscfg.mosi_io_num);
	printf("MPU9250: cs [%d]\n", mpu9250_handle->devcfg.spics_io_num);
	printf("MPU9250: freq [%d]\n", mpu9250_handle->devcfg.clock_speed_hz);
	printf("MPU9250: flags [%d]\n", mpu9250_handle->devcfg.flags);

	//Initialize the SPI bus
	ESP_ERROR_CHECK(spi_bus_initialize(MY_SPI_MPU9250_HOST, &mpu9250_handle->buscfg, 0));
	//Attach the MPU9250 to the SPI bus
	ESP_ERROR_CHECK(spi_bus_add_device(MY_SPI_MPU9250_HOST, &mpu9250_handle->devcfg, &(mpu9250_handle->device_handle)));
	printf("MPU9250: SPI initialized\n");
	return ESP_OK;
}

