#ifndef BME280_H_  
#define BME280_H_

#include "driver/i2c.h"


typedef struct {
  int32_t  temperature; 
  int32_t  pressure;
  int32_t  humidity;
} adc_values_t;


typedef struct {
  adc_values_t adc_raw;
  float  temperature_degC; 
  float  pressure_mm;
  float  humidity_prh;
} bme280_measurement_result_t;


typedef void (*bme280_measurement_callback_t) (bme280_measurement_result_t);



typedef enum {
    OVERSAMPLING_SKIPPED,
    OVERSAMPLING_1,
    OVERSAMPLING_2,
    OVERSAMPLING_4,
    OVERSAMPLING_8,
    OVERSAMPLING_16
} bme280_oversampling_t;


typedef enum {
    MODE_SLEEP,
    MODE_FORCED,
    MODE_NORMAL = 3
} bme280_mode_t;


typedef enum {
    STANDBY_0_5_MS,
    STANDBY_62_5_MS,
    STANDBY_125_MS,
    STANDBY_250_MS,
    STANDBY_500_MS,
    STANDBY_1000_MS,
    STANDBY_10_MS,
    STANDBY_20_MS
} bme280_standby_t;


typedef enum {
    FILTER_OFF,
    FILTER_2,
    FILTER_4,
    FILTER_8,
    FILTER_16
} bme280_filter_t;

#pragma pack(push, 1)
typedef struct {
    union {
        uint8_t  tp_block_raw[26];//0x88
        struct {
                    uint16_t dig_T1;  //0x88, 0x89 
                    int16_t dig_T2;    //0x8A, 0x8B  
                    int16_t dig_T3;    //0x8C, 0x8D
                    uint16_t dig_P1;  //0x8E, 0x8F
                    int16_t dig_P2;    // 0x90,0x91
                    int16_t dig_P3;    //0x92,0x93
                    int16_t dig_P4;    //0x94, 0x95 
                    int16_t dig_P5;    //0x96, 0x97 
                    int16_t dig_P6;    //0x98, 0x99 
                    int16_t dig_P7;    //0x9A, 0x9B 
                    int16_t dig_P8;    //0x9C, 0x9D
                    int16_t dig_P9;    //0x9E, 0x9F
                    uint8_t _;        //0xA0
                    uint8_t dig_H1;   //0xA1
                };
    };
  
    union {
        uint8_t h_block_raw[8]; // 0xE1
        struct {
            int16_t   dig_H2; // 0xE1 ,0xE2
            uint8_t   dig_H3; //0xE3
            int16_t   dig_H4:12; //0xE4
            int16_t   dig_H5:12;
            int8_t    dig_H6; //0xE7
        };
    };

   union {
    uint8_t config_block_raw[2]; //0xF4
    struct {
            union { 
                    uint8_t ctrl_meas; //0xF4
                    struct {
                        bme280_mode_t mode:2;
                        bme280_oversampling_t osrs_p:3;
                        bme280_oversampling_t osrs_t:3;
                    };  
            };

            union { 
                    uint8_t config; //0xF5
                    struct {
                        bool spi3w_en:1;
                        unsigned:1;
                        bme280_filter_t filter:3;
                        bme280_standby_t t_sb:3;
                    };  
            };
    };
   };

   union {
        uint8_t ctrl_hum; //0xF2
        struct {
                bme280_oversampling_t osrs_h:3;
                unsigned:0;
            };  
   };

    i2c_port_t i2c_num;
    uint8_t device_address;
    uint16_t measure_time_ms;
} bme280_t;
#pragma pack(pop) 





typedef uint8_t bme280_status_t;
#define BME280_STATUS_READY bme280_status_t(0); 
#define BME280_STATUS_MASK_MEASURING bme280_status_t(8); 
#define BME280_STATUS_MASK_IM_UPDATE bme280_status_t(1); 


void bme280_init(bme280_t *sensor, i2c_port_t i2c_num, uint8_t device_address);

bme280_status_t bme280_status(bme280_t *sensor) ;

bme280_measurement_result_t bme280_measure_now(bme280_t *sensor);

void bme280_start_async(bme280_t *sensor, bme280_standby_t interval);
void bme280_stop(bme280_t *sensor);

bme280_measurement_result_t bme280_read_last_result(bme280_t *sensor);

void bme280_set_config( bme280_t *sensor, 
                        bme280_oversampling_t osrs_h, 
                        bme280_oversampling_t osrs_t, 
                        bme280_oversampling_t osrs_p,
                        bme280_filter_t filter
                        );

void bme280_reset(bme280_t *sensor);

#endif