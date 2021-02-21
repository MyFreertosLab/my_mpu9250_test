ESP-IDF MPU-9250 Test
=====================

This is a MPU9250 Test on ESP32 connected with SPI2 interface

The software performs a static calibration which consists of:
1) place the sensor on a reference plane without vibrations
2) start the test and wait for it to converge to the optimal calibration

Calibration is performed in two stages. Each of them applied simultaneously on gyroscope and accelerometer

Phase 1: Calculation of the offsets on all axes. This phase is iterative. Calculation cycles and offsets assignment are performed until an average error equal to zero is obtained.

Phase 2: Variance and mean square deviation calculation for each available 'Free Scale Resolution (FSR)'. The result of this step is used in the Kalman Filter to calculate the gain K.

A Kalman filter is used to estimate the measurements. The gain K is calculated using the variance obtained in step 2.
The sampling frequency used is 1KHz.

The sampling frequency used is 1KHz.

Below are the SPI connections between ESP32 and MPU9250 and the calibration phases.

