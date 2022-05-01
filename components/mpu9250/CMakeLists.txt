set(mpu9250_srcs "src/mpu9250.c" "src/mpu9250_spi.c" "src/mpu9250_accel.c" "src/mpu9250_gyro.c" "src/mpu9250_mag.c" "src/mpu9250_mag_calibrator.c" "src/mpu9250_calibrator.c" "src/mpu9250_baro.c")
idf_component_register(SRCS "${mpu9250_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES main nvs_flash app_update
                      )

