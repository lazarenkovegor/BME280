# BME280
BME280/BMP280 driver for ESP32

Async mode example:
```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "BME280.h"
static const char *TAG = "example";

#define I2C_HOST  0
#define I2C_CLOCK_HZ    (400 * 1000)
#define I2C_PIN_NUM_SDA           14
#define I2C_PIN_NUM_SCL           15
#define BME280_I2C_HW_ADDR            0x76


void app_main(void)
{
    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_PIN_NUM_SDA,
        .scl_io_num = I2C_PIN_NUM_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_CLOCK_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_HOST, i2c_conf.mode, 0, 0, 0));


    ESP_LOGI(TAG, "Initialize BME280");

    bme280_t s;
    bme280_init(&s,I2C_HOST,BME280_I2C_HW_ADDR);
    bme280_set_config(&s, OVERSAMPLING_16,OVERSAMPLING_16,OVERSAMPLING_16,FILTER_16);
    bme280_start_async(&s, STANDBY_125_MS); //start async measure loop with 125ms standby interval

    while (1) {
      bme280_measurement_result_t res = bme280_read_last_result(&s); //fast read and decode the last async value
      
      ESP_LOGD(TAG, "status is %d", bme280_status(&s)); 
      ESP_LOGD(TAG, "raw temperature %ld", res.adc_raw.temperature);
      ESP_LOGD(TAG, "raw pressure %ld", res.adc_raw.pressure);
      ESP_LOGD(TAG, "raw humidity %ld", res.adc_raw.humidity);
      ESP_LOGD(TAG, "temperature %.1f degC", res.temperature_degC);
      ESP_LOGD(TAG, "humidity %.0f %%RH", res.humidity_prh);   
      ESP_LOGD(TAG, "pressure %.0f mm", res.pressure_mm);   

      vTaskDelay(pdMS_TO_TICKS(1000));

    }
}

```


Sync mode example:
```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "BME280.h"
static const char *TAG = "example";

#define I2C_HOST  0
#define I2C_CLOCK_HZ    (400 * 1000)
#define I2C_PIN_NUM_SDA           14
#define I2C_PIN_NUM_SCL           15
#define BME280_I2C_HW_ADDR            0x76


void app_main(void)
{
    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_PIN_NUM_SDA,
        .scl_io_num = I2C_PIN_NUM_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_CLOCK_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_HOST, i2c_conf.mode, 0, 0, 0));


    ESP_LOGI(TAG, "Initialize BME280");

    bme280_t s;
    bme280_init(&s,I2C_HOST,BME280_I2C_HW_ADDR);
    bme280_set_config(&s, OVERSAMPLING_16,OVERSAMPLING_16,OVERSAMPLING_16,FILTER_16);

    while (1) {
      bme280_measurement_result_t res = bme280_measure_now(&s); //blocking operation
      
      ESP_LOGD(TAG, "status is %d", bme280_status(&s));
      ESP_LOGD(TAG, "raw temperature %ld", res.adc_raw.temperature);
      ESP_LOGD(TAG, "raw pressure %ld", res.adc_raw.pressure);
      ESP_LOGD(TAG, "raw humidity %ld", res.adc_raw.humidity);
      ESP_LOGD(TAG, "temperature %.1f degC", res.temperature_degC);
      ESP_LOGD(TAG, "humidity %.0f %%RH", res.humidity_prh);   
      ESP_LOGD(TAG, "pressure %.0f mm", res.pressure_mm);   

      vTaskDelay(pdMS_TO_TICKS(1000));

    }
}

```
