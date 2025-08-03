# 310_esp32_freertos_icm_20948

with ESP32,PlatformIO with Arduino framework and FreeRTOS

[SparkFun 9DoF IMU Breakout - ICM-20948 (Qwiic)](https://www.sparkfun.com/sparkfun-9dof-imu-breakout-icm-20948-qwiic.html)

## Code
1- Sample : main_ICM_20948.cpp : pio run -t upload -e main_ICM_20948
2- Sample with FreeRtos : main_ICM_20948_FreeRTOS.cpp : pio run -t upload -e main_ICM_20948_FreeRTOS
2- Sample with FreeRtos and Mavlink : main_ICM_20948_FreeRTOS_Mavlink.cpp : pio run -t upload -e main_ICM_20948_FreeRTOS_Mavlink

## References 
1- [Sparkun]  (https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/tree/main)
2- [Wolfgang Ewald : ICM-20948](https://wolles-elektronikkiste.de/en/icm-20948-9-axis-sensor-part-i)
3- [How to use MAVLink on Arduino ESP32 with PlatformIO](https://github.com/technopolistv/ESP32-MAVLink-Arduino-Example/tree/main)
AHRS For Madgwick
4- [PaulStoffregen's Madgwick AHRS implementation](https://github.com/arduino-libraries/MadgwickAHRS/tree/master)
5- [Read and Plot Real-Time Data from BNO055 Sensor in NDOF Mode](https://www.mathworks.com/help/matlab/supportpkg/read-and-plot-real-time-data-from-bno055-sensor-in-ndof-mode.html)
