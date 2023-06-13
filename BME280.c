#include <stdio.h>
#include "BME280.h"
#include "driver/i2c.h"
#include "esp_log.h"

void  bme280_write(bme280_t *sensor, uint8_t addr , uint8_t value) {
    uint8_t buf[2] = {addr,value};
    ESP_ERROR_CHECK(i2c_master_write_to_device (sensor->i2c_num, sensor->device_address, buf,sizeof(buf), 100/portTICK_PERIOD_MS));
}


void  _bme280_read(bme280_t *sensor, uint8_t addr , uint8_t *to, size_t len) {
   ESP_ERROR_CHECK(i2c_master_write_read_device (sensor->i2c_num, sensor->device_address, &addr,1, to, len, 100/portTICK_PERIOD_MS));
}

#define bme280_read(sensor, addr, destination) _bme280_read(sensor,addr,    \
    _Generic((destination),uint8_t*: destination,   default: &destination)  \
    , sizeof(destination)) 



void bme280_reset(bme280_t *sensor)
{
    bme280_write(sensor,0xE0,0xB6);
}


void bme280_test(bme280_t *sensor)
{    
    uint8_t chip_id;
    bme280_read(sensor,0xD0, chip_id);
    assert(chip_id == 0x60);
}


uint32_t oversampling_value(bme280_oversampling_t val) {
    switch (val) {
    case OVERSAMPLING_SKIPPED:       return 0;
    case OVERSAMPLING_1:             return 1;
    case OVERSAMPLING_2:             return 2;
    case OVERSAMPLING_4:             return 4;
    case OVERSAMPLING_8:             return 8;
    default:                         return 16;
    }
}


void bme280_calc_measure_time(bme280_t *sensor) {
    uint32_t tmp = 1250;

    if (sensor->osrs_t != OVERSAMPLING_SKIPPED) 
        tmp+=2300*oversampling_value(sensor->osrs_t);

    if (sensor->osrs_p != OVERSAMPLING_SKIPPED) 
        tmp+=2300*oversampling_value(sensor->osrs_p)  +575;

    if (sensor->osrs_h != OVERSAMPLING_SKIPPED) 
        tmp+=2300*oversampling_value(sensor->osrs_h)  +575;
    
    sensor->measure_time_ms = tmp / 1000;
}


void bme280_read_config(bme280_t *sensor)
{    
    bme280_read(sensor,0xF4, sensor->config_block_raw);
    bme280_read(sensor,0xF2, sensor->ctrl_hum);
    bme280_calc_measure_time(sensor);
}

void bme280_write_config(bme280_t *sensor)
{    
    bme280_calc_measure_time(sensor);
    bme280_write(sensor,0xF5, sensor->config);
    bme280_write(sensor,0xF2, sensor->ctrl_hum);
    bme280_write(sensor,0xF4, sensor->ctrl_meas);
}


void bme280_init(bme280_t *sensor, i2c_port_t i2c_num, uint8_t device_address)
{
    sensor->i2c_num = i2c_num;
    sensor->device_address = device_address;

    bme280_test(sensor);
    bme280_read(sensor,0x88, sensor->tp_block_raw);
    bme280_read(sensor,0xE1, sensor->h_block_raw);
    sensor->dig_H4 = sensor->dig_H4 << 4 |  ((uint8_t)(sensor->dig_H4 >>8) & 0xF); //convert byte order to little-endian
    bme280_read_config(sensor); 
}

bme280_status_t bme280_status(bme280_t *sensor) 
{
    uint8_t status;

    bme280_read(sensor,0xF3, status);

    return status;
}

void bme280_set_mode(bme280_t *sensor, bme280_mode_t mode) {
    sensor->mode = mode;
    bme280_write(sensor,0xF4, sensor->ctrl_meas);
}


void bme280_set_config (bme280_t *sensor, 
                        bme280_oversampling_t osrs_h, 
                        bme280_oversampling_t osrs_t, 
                        bme280_oversampling_t osrs_p,
                        bme280_filter_t filter
                        ) {
    sensor->osrs_h = osrs_h;
    sensor->osrs_t = osrs_t;  
    sensor->osrs_p = osrs_p; 
    sensor->filter = filter;

    bme280_write_config(sensor);
}
 

typedef union {
        uint8_t raw[8];// 0xF7
        struct {
            uint8_t press_msb;// 0xF7
            uint8_t press_lsb;// 0xF8
            uint8_t :4;// 0xF9
            uint8_t press_xlsb:4;
            uint8_t temp_msb;// 0xFA 
            uint8_t temp_lsb;// 0xFB 
            uint8_t :4;// 0xFC 
            uint8_t temp_xlsb:4;
            uint8_t hum_msb;// 0xFD 
            uint8_t hum_lsb;// 0xFE 
        };
} raw_adc_t;





adc_values_t bme280_read_measure_result(bme280_t *sensor) {
    raw_adc_t d;

    bme280_read(sensor,0xF7,d.raw);

    adc_values_t res = {
        .temperature    =   d.temp_msb << 12  |  d.temp_lsb << 4  | d.temp_xlsb, 
        .pressure       =   d.press_msb << 12  |  d.press_lsb << 4  | d.press_xlsb, 
        .humidity       =   d.hum_msb << 8 | d.hum_lsb} ;
    return res;
}




// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
int32_t BME280_compensate_T_int32(bme280_t *s, int32_t adc_T, int32_t* t_fine)
{
    int32_t var1 = ((((adc_T>>3) - ((int32_t)s->dig_T1<<1))) * ((int32_t)s->dig_T2)) >> 11;
    int32_t var2 = (((((adc_T>>4) - ((int32_t)s->dig_T1)) * ((adc_T>>4) - ((int32_t)s->dig_T1))) >> 12) * ((int32_t)s->dig_T3)) >> 14;
    
    *t_fine = var1 + var2;
    
    return (*t_fine * 5 + 128) >> 8;
}



// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t BME280_compensate_P_int64(bme280_t *s,int32_t adc_P, int32_t* t_fine)
{
    int64_t var1 = ((int64_t)*t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)s->dig_P6;
    var2 = var2 + ((var1*(int64_t)s->dig_P5)<<17);
    var2 = var2 + (((int64_t)s->dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)s->dig_P3)>>8) + ((var1 * (int64_t)s->dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)s->dig_P1)>>33;
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    
    int64_t p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)s->dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)s->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)s->dig_P7)<<4);

    return (uint32_t)p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
uint32_t bme280_compensate_H_int32(bme280_t *s, int32_t adc_H, int32_t* t_fine)
{
    int32_t v_x1_u32r = (*t_fine - ((int32_t)76800));

    v_x1_u32r = (((((adc_H << 14) - (((int32_t)s->dig_H4) << 20) - (((int32_t)s->dig_H5) *
    v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r *   ((int32_t)s->dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)s->dig_H3)) >> 11) +
    ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)s->dig_H2) +   8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *   ((int32_t)s->dig_H1)) >> 4));


    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (uint32_t)(v_x1_u32r>>12);
}


bme280_measurement_result_t decode_results(bme280_t *sensor, adc_values_t raw) {
    int32_t t_fine;

    bme280_measurement_result_t res = { 
        .adc_raw = raw,
        .temperature_degC = BME280_compensate_T_int32(sensor,raw.temperature, &t_fine) / 100.,
        .humidity_prh = bme280_compensate_H_int32(sensor, raw.humidity, &t_fine) / 1024. ,
        .pressure_mm = BME280_compensate_P_int64(sensor, raw.pressure, &t_fine)/ 34128.782828956 // 256÷0.007501
    };

    return res;
}


bme280_measurement_result_t bme280_read_last_result(bme280_t *sensor) {
    return decode_results(sensor,bme280_read_measure_result(sensor));
}

bme280_measurement_result_t bme280_measure_now(bme280_t *sensor)  {
  bme280_set_mode(sensor, MODE_FORCED);
  vTaskDelay(pdMS_TO_TICKS(sensor->measure_time_ms));

  return bme280_read_last_result(sensor);
}


void bme280_start_async(bme280_t *sensor, bme280_standby_t interval) {
  bme280_set_mode(sensor, MODE_SLEEP);
  sensor->t_sb = interval;
  bme280_write(sensor,0xF5, sensor->config);
  bme280_set_mode(sensor, MODE_NORMAL);
  vTaskDelay(pdMS_TO_TICKS(sensor->measure_time_ms));
}

void bme280_stop(bme280_t *sensor){
  bme280_set_mode(sensor, MODE_SLEEP);
}

