// To Upload and run : pio run -t upload -e [env_defined_in_platformio.ini] in our case :
// pio run -t upload -e main_ICM_20948

// Data  Pin : SDA	GPIO 21
// Clock Pin : SCL	GPIO 22

#include <Wire.h>

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
// I2C parameters
static const int32_t I2C_SDA_PIN = 21;
static const int32_t I2C_SCL_PIN = 22;
static const uint32_t I2C_FREQUENCY_HZ = 400000U; // in Hz // Set I2C frequency to 400 kHz

// For esp_timer_get_time()
#include "esp_timer.h"
// Add Logger
#include "esp_log.h"
static const char *LOGGER_TAG = "main_ICM_20948";

ICM_20948_I2C myICM;       // Create an ICM_20948_I2C object
#define ICM20948_ADDR 0x68 // ICM20948_ADDR : 0x68 is default I2C address

void setup()
{
  Serial.begin(115200);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY_HZ); // SDA, SCL, Frequency in H

  // Initialize the sensor
  myICM.begin(Wire, ICM20948_ADDR); // 0x68 is default I2C address

  if (myICM.status != ICM_20948_Stat_Ok)
  {
    ESP_LOGE(LOGGER_TAG, "ICM20948 : IMU initialization unsuccessful");
    ESP_LOGE(LOGGER_TAG, "Check wiring, address, and sensor power");
    while (1)
      ; // Halt
  }
  ESP_LOGI(LOGGER_TAG, "ICM-20948 connected");
}


void loop()
{
  int64_t start_us = 0;
  int64_t end_us = 0;
  int64_t execution_time_us = 0;

  // Ask for new data
  if (myICM.dataReady())
  {
    start_us = esp_timer_get_time(); // Time before transmission

    myICM.getAGMT(); // Accel, Gyro, Mag, Temp

    end_us = esp_timer_get_time(); // Time after transmission
    execution_time_us = end_us - start_us;

    Serial.printf("Acc (MG)  : X: %5.2f, Y: %5.2f, Z: %5.2f,", myICM.accX(), myICM.accY(), myICM.accZ());
    Serial.printf("Gyr (DPS) : X: %5.2f, Y: %5.2f, Z: %5.2f,", myICM.gyrX(), myICM.gyrY(), myICM.gyrZ());
    Serial.printf("Mag (uT)  : X: %5.2f, Y: %5.2f, Z: %5.2f,", myICM.magX(), myICM.magY(), myICM.magZ());
    Serial.printf("Temp (°C) : %5.2f", myICM.temp());
    Serial.println("");

    ESP_LOGD(LOGGER_TAG, "Execution Time: %lld us, Frequency : %5.2f Hz", execution_time_us, 1.0 / (execution_time_us / 1000000.0f)); 
  }
  delay(100);
}
