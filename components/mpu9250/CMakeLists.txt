set(mpu9250_srcs "src/mpu9250.c" "src/mpu9250_spi.c" "src/mpu9250_accel.c" "src/mpu9250_gyro.c" "src/mpu9250_calibrator.c")

idf_component_register(SRCS "${mpu9250_srcs}"
                       INCLUDE_DIRS "include")
