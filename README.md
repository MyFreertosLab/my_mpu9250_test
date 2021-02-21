ESP-IDF MPU-9250 Test
=====================

This is a MPU9250 Test on ESP32 connected with SPI2 interface

The software performs a static calibration which consists of:
1) place the sensor on a reference plane without vibrations
2) start the test and wait for it to converge to the optimal calibration

Calibration is performed in two stages. Each of them applied simultaneously on gyroscope and accelerometer

Phase 1: Calculation of the offsets on all axes. This phase is iterative. Calculation cycles and offsets assignment are performed until an average error equal to zero is obtained.

Phase 2: Variance and mean square deviation calculation for each available 'Free Scale Resolution (FSR)'. The result of this step is used in the Kalman Filter to calculate the gain K.

A Kalman filter is used to estimate the measurements. The gain K is calculated using the variance obtained in Phase 2.

The sampling frequency used is 1KHz.

Below are the SPI connections between ESP32 and MPU9250 and the calibration phases.
<h1>SPI Connections</h1>
<p align="left">
  <img src="https://github.com/MyFreertosLab/my_mpu9250_test/blob/master/images/esp32ToImu-Miso-Mosi-Sclk.jpg" width="300" title="hover text">
  <img src="https://github.com/MyFreertosLab/my_mpu9250_test/blob/master/images/esp32ToImu-Ncs-Int.jpg" width="300" title="hover text">
  <img src="https://github.com/MyFreertosLab/my_mpu9250_test/blob/master/images/imuToEsp32.jpg" width="300" title="hover text">
</p>

<h1>Calibration</h1>
<h2>Phase 1</h2>
<p align="left">
  <img src="https://github.com/MyFreertosLab/my_mpu9250_test/blob/master/images/convergence-start.jpg" width="300" title="Start with command: 'idf.py monitor'">
  <img src="https://github.com/MyFreertosLab/my_mpu9250_test/blob/master/images/convergence-1.jpg" width="300" title="Offset Calc Cycles">
  <img src="https://github.com/MyFreertosLab/my_mpu9250_test/blob/master/images/convergence-2.jpg" width="300" title="Convergence OK">
</p>

<h1>Calibration</h1>
<h2>Phase 2</h2>
<p align="left">
  <img src="https://github.com/MyFreertosLab/my_mpu9250_test/blob/master/images/biases-for-each-FSR.jpg" width="300" title="Variance and Mean Square Deviation calculation">
</p>

<h1>Test</h1>
<p align="left">
  <img src="https://github.com/MyFreertosLab/my_mpu9250_test/blob/master/images/test-1.jpg" width="300" title="MPU9250 positioned with an exact 60 degree inclination">
  <img src="https://github.com/MyFreertosLab/my_mpu9250_test/blob/master/images/test-2.jpg" width="300" title="Misurements">
</p>

<h1>TODO List</h1>
1) Estimation of Scale Factors 
2) Calibration for Magnetometer
