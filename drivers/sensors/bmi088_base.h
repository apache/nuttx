/****************************************************************************
 * drivers/sensors/bmi088_base.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_BMI088_COMMOM_H
#define __INCLUDE_NUTTX_SENSORS_BMI088_COMMOM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/bmi088.h>

#include <stdlib.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <fixedmath.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEVID_ACC                                 0x1E
#define DEVID_GYRO                                0x0F

/* I2C  ACC Address
 *
 * NOTE: If SDO1 pin is pulled to VDDIO, use 0x19
 */
#ifdef CONFIG_BMI088_I2C_ACC_ADDR_18
#define BMI088_I2C_ACC_ADDR                       0x18
#else
#define BMI088_I2C_ACC_ADDR                       0x19
#endif

/* I2C  GY Address
 *
 * NOTE: If SDO2 pin is pulled to VDDIO, use 0x69
 */

#ifdef CONFIG_BMI088_I2C_GY_ADDR_68
#define BMI088_I2C_GY_ADDR                        0x68
#else
#define BMI088_I2C_GY_ADDR                        0x69
#endif

#define BMI088_I2C_FREQ                           400000

#define BMI088_SPI_DEV_ACC                        0x00
#define BMI088_SPI_DEV_GYRO                       0x01

/* Accelerometer Registers */

#define BMI088_ACC_CHIP_ID                        0x00
#define BMI088_ACC_ERR_REG                        0x02
#define BMI088_ACC_STATUS                         0x03
#define BMI088_ACC_X_LSB                          0x12
#define BMI088_ACC_X_MSB                          0x13
#define BMI088_ACC_Y_LSB                          0x14
#define BMI088_ACC_Y_MSB                          0x15
#define BMI088_ACC_Z_LSB                          0x16
#define BMI088_ACC_Z_MSB                          0x17
#define BMI088_SENSORTIME_0                       0x18
#define BMI088_SENSORTIME_1                       0x19
#define BMI088_SENSORTIME_2                       0x1A
#define BMI088_ACC_INT_STAT_1                     0x1D
#define BMI088_TEMP_MSB                           0x22
#define BMI088_TEMP_LSB                           0x23
#define BMI088_ACC_CONF                           0x40
#define BMI088_ACC_RANGE                          0x41
#define BMI088_INT1_IO_CONF                       0x53
#define BMI088_INT2_IO_CONF                       0x54
#define BMI088_INT1_INT2_MAP_DATA                 0x58
#define BMI088_ACC_SELF_TEST                      0x6D
#define BMI088_ACC_PWR_CONF                       0x7C
#define BMI088_ACC_PWR_CTRL                       0x7D
#define BMI088_ACC_SOFTRESET                      0x7E

/* Gyroscope Registers */

#define BMI088_GYRO_CHIP_ID                       0x00
#define BMI088_GYRO_X_LSB                         0x02
#define BMI088_GYRO_X_MSB                         0x03
#define BMI088_GYRO_Y_LSB                         0x04
#define BMI088_GYRO_Y_MSB                         0x05
#define BMI088_GYRO_Z_LSB                         0x06
#define BMI088_GYRO_Z_MSB                         0x07
#define BMI088_GYRO_INT_STAT_1                    0x0A
#define BMI088_GYRO_RANGE                         0x0F
#define BMI088_GYRO_BANDWIDTH                     0x10
#define BMI088_GYRO_LPM1                          0x11
#define BMI088_GYRO_SOFTRESET                     0x14
#define BMI088_GYRO_INT_CTRL                      0x15
#define BMI088_INT3_INT4_IO_CONF                  0x16
#define BMI088_INT3_INT4_IO_MAP                   0x18
#define BMI088_GYRO_SELF_TEST                     0x3C

/* additional constants */

#define BMI088_SOFTRESET_CMD                      0xB6
#define BMI088_ACC_ENABLE                         0x04
#define BMI088_GYRO_PM_NORMAL                     0x00
#define BMI088_GYRO_PM_SUSPEND                    0x80
#define BMI088_GYRO_PM_DEEPSUSPEND                0x20

/* ACC_ERR_REG (0x02) Accelerometer Error Register */

#define BMI088_ACC_ERR_REG_NO_ERROR               (0x00)
#define BMI088_ACC_ERR_REG_ERROR_OCCURRED         (0x01)

/* ACC_CONF (0x40) Accelerometer configuration register */

#define BMI088_ACC_CONF_BWP_OSR4                  (0x00 << 4)
#define BMI088_ACC_CONF_BWP_OSR2                  (0x01 << 4)
#define BMI088_ACC_CONF_BWP_NORMAL                (0x02 << 4)
#define BMI088_ACC_CONF_ODR_12_5                  (0x05 << 0)
#define BMI088_ACC_CONF_ODR_25                    (0x06 << 0)
#define BMI088_ACC_CONF_ODR_50                    (0x07 << 0)
#define BMI088_ACC_CONF_ODR_100                   (0x08 << 0)
#define BMI088_ACC_CONF_ODR_200                   (0x09 << 0)
#define BMI088_ACC_CONF_ODR_400                   (0x0A << 0)
#define BMI088_ACC_CONF_ODR_800                   (0x0B << 0)
#define BMI088_ACC_CONF_ODR_1600                  (0x0C << 0)

/* ACC_RANGE (0x41) Accelerometer Range Setting Register */

#define BMI088_ACC_RANGE_3G                       (0x00 << 0)
#define BMI088_ACC_RANGE_6G                       (0x01 << 0)
#define BMI088_ACC_RANGE_12G                      (0x02 << 0)
#define BMI088_ACC_RANGE_24G                      (0x03 << 0)

/* INT1_IO_CONF (0x53) Configures the input/output pin INT1 */

#define BMI088_INT1_IO_CONF_INT1_INPUT            (0x01<<4)
#define BMI088_INT1_IO_CONF_INT1_OUTPUT           (0x01<<3)
#define BMI088_INT1_IO_CONF_INT1_PP               (0x00<<2)
#define BMI088_INT1_IO_CONF_INT1_OD               (0x01<<2)
#define BMI088_INT1_IO_CONF_INT1_LVL              (0x00<<1)
#define BMI088_INT1_IO_CONF_INT1_HL               (0x01<<1)

/* INT1_IO_CONF (0x54) Configures the input/output pin INT1 */

#define BMI088_INT2_IO_CONF_INT2_INPUT            (0x01<<4)
#define BMI088_INT2_IO_CONF_INT2_OUTPUT           (0x01<<3)
#define BMI088_INT2_IO_CONF_INT2_PP               (0x00<<2)
#define BMI088_INT2_IO_CONF_INT2_OD               (0x01<<2)
#define BMI088_INT2_IO_CONF_INT2_LVL              (0x00<<1)
#define BMI088_INT2_IO_CONF_INT2_HL               (0x01<<1)

/* ACC_SELF_TEST (0x6D) Enables the sensor self-test signal */

#define BMI088_ACC_SELF_TEST_OFF                  (0x00 << 0)
#define BMI088_ACC_SELF_TEST_POSITIVE             (0x0D << 0)
#define BMI088_ACC_SELF_TEST_NEGATIVE             (0x09 << 0)

/* ACC_PWR_CONF (0x7C) */

#define BMI088_ACC_PWR_CONF_ACTIVE_MODE           (0x00 << 0)
#define BMI088_ACC_PWR_CONF_SUSPEND_MODE          (0x03 << 0)

/* ACC_PWR_CTRL (0x7D) Switches accelerometer ON or OFF. */

#define BMI088_ACC_PWR_CTRL_ACC_DISABLE           (0x00 << 0)
#define BMI088_ACC_PWR_CTRL_ACC_ENABLE            (0x04 << 0)

/* GYRO_RANGE (0x0F) Angular rate range and resolution. */

#define BMI088_GYRO_RANGE_2000DPS                 (0x00 << 0)
#define BMI088_GYRO_RANGE_1000DPS                 (0x01 << 0)
#define BMI088_GYRO_RANGE_500DPS                  (0x02 << 0)
#define BMI088_GYRO_RANGE_250DPS                  (0x03 << 0)
#define BMI088_GYRO_RANGE_125DPS                  (0x04 << 0)

/* GYRO_BANDWIDTH (0x10)  */

#define BMI088_GYRO_BANDWIDTH_2000HZ_532HZ        (0x00 << 0)
#define BMI088_GYRO_BANDWIDTH_2000HZ_230HZ        (0x01 << 0)
#define BMI088_GYRO_BANDWIDTH_1000HZ_116HZ        (0x02 << 0)
#define BMI088_GYRO_BANDWIDTH_400HZ_47HZ          (0x03 << 0)
#define BMI088_GYRO_BANDWIDTH_200HZ_23HZ          (0x04 << 0)
#define BMI088_GYRO_BANDWIDTH_100HZ_12HZ          (0x05 << 0)
#define BMI088_GYRO_BANDWIDTH_200HZ_64HZ          (0x06 << 0)
#define BMI088_GYRO_BANDWIDTH_100HZ_32HZ          (0x07 << 0)

/* GYRO_LPM1 (0x11) Selection of the main power modes. */

#define BMI088_GYRO_LPM1_NORMAL_MODE              (0x00 << 0)
#define BMI088_GYRO_LPM1_SUSPEND_MODE             (0x80 << 0)
#define BMI088_GYRO_LPM1_DEEP_SUSPEND_MODE        (0x20 << 0)

/* GYRO_INT_CTRL (0x15) Control the data ready interrupt generation. */

#define BMI088_GYRO_INT_CTRL_DATA_RDY_INT_DISABLE (0x00 << 0)
#define BMI088_GYRO_INT_CTRL_DATA_RDY_INT_ENABLE  (0x80 << 0)

/* INT3_INT4_IO_MAP (0x18)  */

#define BMI088_INT3_INT4_IO_MAP_NONE              (0x00 << 0)
#define BMI088_INT3_INT4_IO_MAP_INT3_ONLY         (0x01 << 0)
#define BMI088_INT3_INT4_IO_MAP_INT4_ONLY         (0x80 << 0)
#define BMI088_INT3_INT4_IO_MAP_INT3_AND_INT4     (0x81 << 0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct bmi088_dev_s
{
#ifdef CONFIG_SENSORS_BMI088_I2C
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
  int freq;                     /* Frequency <= 3.4MHz */

#else /* CONFIG_SENSORS_BMI088_SPI */
  FAR struct spi_dev_s *spi;    /* SPI interface */

#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

uint8_t bmi088_get_acc_reg8(FAR struct bmi088_dev_s *priv, uint8_t regaddr);
uint8_t bmi088_get_gyro_reg8(FAR struct bmi088_dev_s *priv, uint8_t regaddr);
void bmi088_put_acc_reg8(FAR struct bmi088_dev_s *priv, uint8_t regaddr,
                        uint8_t regval);
void bmi088_put_gyro_reg8(FAR struct bmi088_dev_s *priv, uint8_t regaddr,
                        uint8_t regval);
void bmi088_get_acc_regs(FAR struct bmi088_dev_s *priv, uint8_t regaddr,
                        uint8_t *regval, int len);
void bmi088_get_gyro_regs(FAR struct bmi088_dev_s *priv, uint8_t regaddr,
                        uint8_t *regval, int len);

#endif /* __INCLUDE_NUTTX_SENSORS_BMI088_COMMOM_H */
