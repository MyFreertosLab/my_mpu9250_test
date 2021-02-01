set(mpu9250_srcs "src/mpu9250.c")

idf_component_register(SRCS "${mpu9250_srcs}"
                       INCLUDE_DIRS "include")
