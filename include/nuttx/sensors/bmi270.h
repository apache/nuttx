/****************************************************************************
 * include/nuttx/sensors/bmi270.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_BMI270_H
#define __INCLUDE_NUTTX_SENSORS_BMI270_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * struct 6-axis data
 ****************************************************************************/

struct accel_t
{
  int16_t x;
  int16_t y;
  int16_t z;
};

struct gyro_t
{
  int16_t x;
  int16_t y;
  int16_t z;
};

struct accel_gyro_st_s
{
  struct gyro_t  gyro;
  struct accel_t accel;
  uint32_t sensor_time;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: bmi270_register
 *
 * Description:
 *   Register the BMI270 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/accel0"
 *   dev     - An instance of the SPI or I2C interface to use to communicate
 *             with BMI270
 *   addr    - (I2C only) I2C address
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI270_I2C
struct i2c_master_s;
int bmi270_register(FAR const char *devpath, FAR struct i2c_master_s *dev,
                    uint8_t addr);
#else /* CONFIG_BMI270_SPI */
struct spi_dev_s;
int bmi270_register(FAR const char *devpath, FAR struct spi_dev_s *dev);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_BMI270_H */
