/****************************************************************************
 * include/nuttx/sensors/bmi160.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NUTTX_SENSORS_BMI160_H
#define __INCLUDE_NUTTX_SENSORS_BMI160_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

#if defined(CONFIG_SENSORS_BMI160) || defined(CONFIG_SENSORS_BMI160_SCU)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BMI160_SPI_MAXFREQUENCY 10000000

/* Configuration ************************************************************/

/* Power mode */

#define BMI160_PM_SUSPEND     (0x00)
#define BMI160_PM_NORMAL      (0x01)
#define BMI160_PM_LOWPOWER    (0x02)
#define BMI160_PM_FASTSTARTUP (0x03)

/* Output data rate */

#define BMI160_ACCEL_ODR_0_78HZ (0x01)
#define BMI160_ACCEL_ODR_1_56HZ (0x02)
#define BMI160_ACCEL_ODR_3_12HZ (0x03)
#define BMI160_ACCEL_ODR_6_25HZ (0x04)
#define BMI160_ACCEL_ODR_12_5HZ (0x05)
#define BMI160_ACCEL_ODR_25HZ   (0x06)
#define BMI160_ACCEL_ODR_50HZ   (0x07)
#define BMI160_ACCEL_ODR_100HZ  (0x08)
#define BMI160_ACCEL_ODR_200HZ  (0x09)
#define BMI160_ACCEL_ODR_400HZ  (0x0A)
#define BMI160_ACCEL_ODR_800HZ  (0x0B)
#define BMI160_ACCEL_ODR_1600HZ (0x0C)

/* IOCTL Commands ***********************************************************/

#define SNIOC_ENABLESC     _SNIOC(0x0001) /* Arg: uint8_t value */
#define SNIOC_READSC       _SNIOC(0x0002) /* Arg: int16_t* pointer */
#define SNIOC_SETACCPM     _SNIOC(0x0003) /* Arg: uint8_t value */
#define SNIOC_SETACCODR    _SNIOC(0x0004) /* Arg: uint8_t value */

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

struct spi_dev_s;
struct i2c_master_s;

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
 * Name: bmi160_register
 *
 * Description:
 *   Register the BMI160 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/accel0"
 *   dev     - An instance of the SPI or I2C interface to use to communicate
 *             with BMI160
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI160_I2C
#  ifdef CONFIG_SENSORS_BMI160_UORB
int bmi160_register_uorb(int devno, FAR struct i2c_master_s *dev);
#  else
int bmi160_register(FAR const char *devpath, FAR struct i2c_master_s *dev);
#  endif /* CONFIG_SENSORS_BMI160_UORB */
#else /* CONFIG_BMI160_SPI */
#  ifdef CONFIG_SENSORS_BMI160_UORB
int bmi160_register_uorb(int devno, FAR struct spi_dev_s *dev);
#  else
int bmi160_register(FAR const char *devpath, FAR struct spi_dev_s *dev);
#  endif /* CONFIG_SENSORS_BMI160_UORB */
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_BMI160 */
#endif /* __INCLUDE_NUTTX_SENSORS_BMI160_H */
