/****************************************************************************
 * include/nuttx/sensors/bmi088.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_BMI088_H
#define __INCLUDE_NUTTX_SENSORS_BMI088_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#if defined(CONFIG_SENSORS_BMI088)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BMI088_SPI_MAXFREQUENCY 1000000

/* Configuration ************************************************************/

/* Power mode */

#define BMI088_PM_SUSPEND       (0x00)
#define BMI088_PM_NORMAL        (0x01)
#define BMI088_PM_LOWPOWER      (0x02)
#define BMI088_PM_FASTSTARTUP   (0x03)

/* Output data rate */

#define BMI088_ACCEL_ODR_0_78HZ (0x01)
#define BMI088_ACCEL_ODR_1_56HZ (0x02)
#define BMI088_ACCEL_ODR_3_12HZ (0x03)
#define BMI088_ACCEL_ODR_6_25HZ (0x04)
#define BMI088_ACCEL_ODR_12_5HZ (0x05)
#define BMI088_ACCEL_ODR_25HZ   (0x06)
#define BMI088_ACCEL_ODR_50HZ   (0x07)
#define BMI088_ACCEL_ODR_100HZ  (0x08)
#define BMI088_ACCEL_ODR_200HZ  (0x09)
#define BMI088_ACCEL_ODR_400HZ  (0x0A)
#define BMI088_ACCEL_ODR_800HZ  (0x0B)
#define BMI088_ACCEL_ODR_1600HZ (0x0C)

/* IOCTL Commands ***********************************************************/

#define SNIOC_ACC_ENABLE        _SNIOC(0x0001)
#define SNIOC_ACC_DISABLE       _SNIOC(0x0002)

#define SNIOC_GYRO_ENABLE       _SNIOC(0x0101)
#define SNIOC_GYRO_DISABLE      _SNIOC(0x0102)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * struct 6-axis data
 ****************************************************************************/

#pragma pack(1)

struct acc_source_t
{
  int16_t x;
  int16_t y;
  int16_t z;
};

struct acc_t
{
  float x;
  float y;
  float z;
};

struct gyro_source_t
{
  int16_t x;
  int16_t y;
  int16_t z;
};

struct gyro_t
{
  float x;
  float y;
  float z;
};

struct acc_gyro_st_s
{
  struct acc_source_t   acc_source;
  uint32_t              sensor_time;
  struct gyro_source_t  gyro_source;
  struct acc_t          accel;
  struct gyro_t         gyro;
};

#pragma pack()

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
 * Name: bmi088_register
 *
 * Description:
 *   Register the BMI088 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/accel0"
 *   dev     - An instance of the SPI or I2C interface to use to communicate
 *             with BMI088
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI088_I2C
#  ifdef CONFIG_SENSORS_BMI088_UORB
int bmi088_register_acc_uorb(int devno, FAR struct i2c_master_s *dev);
int bmi088_register_gyro_uorb(int devno, FAR struct i2c_master_s *dev);
#  else
int bmi088_acc_register (FAR const char *devpath,
                         FAR struct i2c_master_s *dev);
int bmi088_gyro_register(FAR const char *devpath,
                         FAR struct i2c_master_s *dev);
#  endif /* CONFIG_SENSORS_BMI088_UORB */
#else /* CONFIG_BMI088_SPI */
#  ifdef CONFIG_SENSORS_BMI088_UORB
int bmi088_register_acc_uorb(int devno, FAR struct spi_dev_s *dev);
int bmi088_register_gyro_uorb(int devno, FAR struct spi_dev_s *dev);
#  else
int bmi088_acc_register (FAR const char *devpath,
                         FAR struct spi_dev_s *dev);
int bmi088_gyro_register(FAR const char *devpath,
                         FAR struct spi_dev_s *dev);
#  endif /* CONFIG_SENSORS_BMI088_UORB */
#endif

#endif /* CONFIG_SENSORS_BMI088 */
#endif /* __INCLUDE_NUTTX_SENSORS_BMI088_H */
