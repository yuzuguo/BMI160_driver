//
// Created by kerner on 11/30/17.
//
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <stdbool.h>
#include <signal.h>
#include <zconf.h>
#include <linux/i2c-dev.h>

#include "bmi160.h"

// accel_scale is +/- 4g, 4 * 9.8 / 32767, unit is m/s^2
#define ACCEL_TO_METER 0.001196326

// gyro_scale is +/- 1000 deg/s, 1000 / (57.3 * 32767), unit is radian
#define GYRO_TO_RADIAN 0.000532609

#define LogDataToFile

static uint64_t GetCurrentTimeMilliSec();
static int8_t user_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data,
                            uint16_t len);
static int8_t user_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data,
                             uint16_t len);
static void user_delay_ms(uint32_t period);

static bool is_exit = false;
static void exit_handler(int s) { is_exit = true; }

int main(int argc, char **argv) {
  // install SIGNAL handler
  signal(SIGINT, exit_handler);

  struct bmi160_dev sensor;

  sensor.id = BMI160_I2C_ADDR;
  sensor.interface = BMI160_I2C_INTF;
  sensor.read = user_i2c_read;
  sensor.write = user_i2c_write;
  sensor.delay_ms = user_delay_ms;

  int8_t rslt = BMI160_OK;
  rslt = bmi160_init(&sensor);
  /* After the above function call, accel and gyro parameters in the device
  structure
  are set with default values, found in the datasheet of the sensor */

  rslt = BMI160_OK;

  /* Select the Output data rate, range of accelerometer sensor */
  sensor.accel_cfg.odr = BMI160_ACCEL_ODR_200HZ;
  sensor.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
  sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

  /* Select the power mode of accelerometer sensor */
  sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

  /* Select the Output data rate, range of Gyroscope sensor */
  sensor.gyro_cfg.odr = BMI160_ACCEL_ODR_200HZ;
  sensor.gyro_cfg.range = BMI160_GYRO_RANGE_1000_DPS;
  sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

  /* Select the power mode of Gyroscope sensor */
  sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

  /* Set the sensor configuration */
  rslt = bmi160_set_sens_conf(&sensor);

  rslt = BMI160_OK;
  struct bmi160_sensor_data accel;
  struct bmi160_sensor_data gyro;


  // open file
#ifdef LogDataToFile
  // write to /tmp directory will be much faster!
  FILE * log_file = fopen("/tmp/imu.csv", "w");
  if(log_file == NULL) {
    printf("open log file error!\n");
    goto error;
  }
#endif


  while (!is_exit) {

    uint64_t start = GetCurrentTimeMilliSec();
/* To read both Accel and Gyro data along with time*/
    rslt = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL), &accel, &gyro, &sensor);

    if (rslt != BMI160_OK) {
      continue;
    }

    // process data
    const uint64_t timestamp = GetCurrentTimeMilliSec();
    const double accelX = accel.x * ACCEL_TO_METER;
    const double accelY = accel.y * ACCEL_TO_METER;
    const double accelZ = accel.z * ACCEL_TO_METER;
    const double gyroX = gyro.x * GYRO_TO_RADIAN;
    const double gyroY = gyro.y * GYRO_TO_RADIAN;
    const double gyroZ = gyro.z * GYRO_TO_RADIAN;

    printf(
        "the accelX is %f \n the accelY is %f \nthe accelZ is %f \nthe gyroX "
        "is %f \nthe gyroY is %f \nthe gyroZ is %f \n",
        accelX, accelY, accelZ, gyroX, gyroY, gyroZ);


#ifdef LogDataToFile
    fprintf(log_file, "%ld %f %f %f %ld %f %f %f\n",timestamp, accelX, accelY,accelZ, timestamp, gyroX, gyroY, gyroZ);
#endif

    uint64_t used_time = GetCurrentTimeMilliSec() - start;
    const static int period = 5; // period 5ms, 200Hz
    if(used_time < period){
      usleep((__useconds_t) ((period - used_time) * 1000));
    }

    error:
#ifdef LogDataToFile
    if(log_file!=NULL) {
      fclose(log_file);
    }
#endif
    // close I2C device
    // release any other things
  }
}

static void user_delay_ms(uint32_t period) {
  uint64_t start = GetCurrentTimeMilliSec();
  while (GetCurrentTimeMilliSec() - start < period)
    ;
}

static uint64_t GetCurrentTimeMilliSec() {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  uint64_t cur_milli_sec =
      (uint64_t)((uint64_t)t.tv_nsec / 1000000 + (((uint64_t)t.tv_sec) * 1000));
  return cur_milli_sec;
}

static int8_t user_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data,
                            uint16_t len) {
  // TODO implement, use wiringPi
}
static int8_t user_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data,
                             uint16_t len) {
  // TODO implement, use wiringPi
}