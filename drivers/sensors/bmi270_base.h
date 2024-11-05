/****************************************************************************
 * drivers/sensors/bmi270_base.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_BMI270_BASE_H
#define __INCLUDE_NUTTX_SENSORS_BMI270_BASE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/sensors/bmi270.h>
#ifdef CONFIG_SENSORS_BMI270_I2C
#  include <nuttx/i2c/i2c_master.h>
#else
#  include <nuttx/spi/spi.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BMI270_SPI_MAXFREQUENCY 10000000
#define BMI270_I2C_FREQ         400000

#define DEVID                   0x24

#define BMI270_CHIP_ID          (0x00) /* Chip ID */
#define BMI270_ERROR            (0x02) /* Error register */
#define BMI270_PMU_STAT         (0x03) /* Current power mode */
#define BMI270_DATA_0           (0x04) /* MAG X  7:0 (LSB) */
#define BMI270_DATA_1           (0x05) /* MAG X 15:8 (MSB) */
#define BMI270_DATA_2           (0x06) /* MAG Y  7:0 (LSB) */
#define BMI270_DATA_3           (0x07) /* MAG Y 15:8 (MSB) */
#define BMI270_DATA_4           (0x08) /* MAG Z  7:0 (LSB) */
#define BMI270_DATA_5           (0x09) /* MAG Z 15:8 (MSB) */
#define BMI270_DATA_6           (0x0a) /* RHALL  7:0 (LSB) */
#define BMI270_DATA_7           (0x0b) /* RHALL 15:8 (MSB) */

#define BMI270_DATA_8           (0x0c) /* ACC X  7:0 (LSB) */
#define BMI270_DATA_9           (0x0d) /* ACC X 15:8 (MSB) */
#define BMI270_DATA_10          (0x0e) /* ACC Y  7:0 (LSB) */
#define BMI270_DATA_11          (0x0f) /* ACC Y 15:8 (MSB) */
#define BMI270_DATA_12          (0x10) /* ACC Z  7:0 (LSB) */
#define BMI270_DATA_13          (0x11) /* ACC Z 15:8 (MSB) */

#define BMI270_DATA_14          (0x12) /* GYR X  7:0 (LSB) */
#define BMI270_DATA_15          (0x13) /* GYR X 15:8 (MSB) */
#define BMI270_DATA_16          (0x14) /* GYR Y  7:0 (LSB) */
#define BMI270_DATA_17          (0x15) /* GYR Y 15:8 (MSB) */
#define BMI270_DATA_18          (0x16) /* GYR Z  7:0 (LSB) */
#define BMI270_DATA_19          (0x17) /* GYR Z 15:8 (MSB) */
#define BMI270_SENSORTIME_0     (0x18) /* Sensor time 0 */
#define BMI270_SENSORTIME_1     (0x19) /* Sensor time 1 */
#define BMI270_SENSORTIME_2     (0x1A) /* Sensor time 2 */
#define BMI270_EVENT            (0x1B) /* Sensor event flags */
#define BMI270_INTR_STAT_0      (0x1C) /* Interrupt status */
#define BMI270_INTR_STAT_1      (0x1D)
#define BMI270_SC_OUT_0         (0x1E) /* Step counting value */
#define BMI270_SC_OUT_1         (0x1f) /* Step counting value */
#define BMI270_WR_GEST_ACT      (0x20) /* Wrist gesture and activity detection */
#define BMI270_INTERNAL_STAT    (0x21) /* Internal status */
#define BMI270_TEMPERATURE_0    (0x22) /* Temperature */
#define BMI270_TEMPERATURE_1    (0x23)
#define BMI270_FIFO_LENGTH_0    (0x24) /* FIFO length */
#define BMI270_FIFO_LENGTH_1    (0x25)
#define BMI270_FIFO_DATA        (0x26)
#define BMI270_FEAT_PAGE        (0x2f) /* Page number for feature configuration and output registers */
                                       /* TODO: Features 0x30-0x3f */
#define BMI270_ACC_CONFIG       (0x40) /* ACCEL config for ODR, bandwidth and undersampling */
#define BMI270_ACC_RANGE        (0x41) /* ACCEL range */
#define BMI270_GYR_CONFIG       (0x42) /* GYRO config for ODR and bandwidth */
#define BMI270_GYR_RANGE        (0x43) /* GYRO range */
#define BMI270_AUX_CONFIG       (0x44) /* AUX config for ODR */
#define BMI270_FIFO_DOWN        (0x45) /* GYRO and ACCEL downsampling rates for FIFO */
#define BMI270_FIFO_WTM_0       (0x46) /* FIFO Watermark level */
#define BMI270_FIFO_WTM_1       (0x47) /* FIFO Watermark level */
#define BMI270_FIFO_CONFIG_0    (0x48) /* FIFO config */
#define BMI270_FIFO_CONFIG_1    (0x49)
#define BMI270_SATURATION       (0x4A) /* Saturation */
#define BMI270_AUX_DEV_ID       (0x4B) /* Auxiliary interface device_id */
#define BMI270_AUX_IF_CONF      (0x4C) /* Auxiliary interface configuration */
#define BMI270_AUX_RD_ADDR      (0x4C) /* Auxiliary interface read address */
#define BMI270_AUX_WR_ADDR      (0x4E) /* Auxiliary interface write address */
#define BMI270_AUX_WR_DATA      (0x4F) /* Auxiliary interface write data */
#define BMI270_ERR_REG_MSK      (0x52)
#define BMI270_INT1_IO_CTRL     (0x53) /* INT pin configuration */
#define BMI270_INT2_IO_CTRL     (0x54)
#define BMI270_INT_LATCH        (0x55) /* Configure interrupt modes */
#define BMI270_INT1_MAP_FEAT    (0x56) /* Interrupt/Feature mapping on INT1 */
#define BMI270_INT2_MAP_FEAT    (0x57) /* Interrupt/Feature mapping on INT2 */
#define BMI270_INT_MAP_DATA     (0x58) /* Data Interrupt mapping for both INT pins */
#define BMI270_INIT_CTRL        (0x59) /* Start initialization */
#define BMI270_INIT_ADDR_0      (0x5b) /* Base address of the initialization data */
#define BMI270_INIT_ADDR_1      (0x5c)
#define BMI270_INIT_DATA        (0x5e) /* Initialization register */
#define BMI270_INTERNAL_ERROR   (0x5f)
#define BMI270_AUX_IF_TRIM      (0x68) /* Auxiliary interface trim */
#define BMI270_GYR_CRT_CONF     (0x69) /* Component Retrimming for Gyroscope */
#define BMI270_NMV_CONFIG       (0x6A) /* NVM Configuration */
#define BMI270_IF_CONFIG        (0x6B) /* Serial interface configuration */
#define BMI270_DRV              (0x6C) /* Drive strength control */
#define BMI270_ACC_SELF_TEST    (0x6D) /* Acc self test */
#define BMI270_GYR_SELF_TEST    (0x6E) /* Gyro self test */
#define BMI270_NV_CONFIG        (0x70) /* SPI/I2C selection */
#define BMI270_OFFSET_0         (0x71) /* ACCEL and GYRO offset */
#define BMI270_OFFSET_1         (0x72)
#define BMI270_OFFSET_2         (0x73)
#define BMI270_OFFSET_3         (0x74)
#define BMI270_OFFSET_4         (0x75)
#define BMI270_OFFSET_5         (0x76)
#define BMI270_OFFSET_6         (0x77)
#define BMI270_PWR_CONF         (0x7C) /* Power mode configuration */
#define BMI270_PWR_CTRL         (0x7D) /* Power mode control */
#define BMI270_CMD              (0x7e) /* Command register */

/* Register 0x21 - INTERNAL_STATUS */

#define INTSTAT_MSG_MASK        (7)
#define INTSTAT_MSG_NOTINIT     (0x00)
#define INTSTAT_MSG_INITOK      (0x01)

/* Register 0x40 - ACCEL_CONFIG accel bandwidth */

#define ACCEL_OSR4_AVG1         (0 << 4)
#define ACCEL_OSR2_AVG2         (1 << 4)
#define ACCEL_NORMAL_AVG4       (2 << 4)
#define ACCEL_CIC_AVG8          (3 << 4)
#define ACCEL_RES_AVG2          (4 << 4)
#define ACCEL_RES_AVG4          (5 << 4)
#define ACCEL_RES_AVG8          (6 << 4)
#define ACCEL_RES_AVG16         (7 << 4)
#define ACCEL_RES_AVG32         (8 << 4)
#define ACCEL_RES_AVG64         (9 << 4)
#define ACCEL_RES_AVG128        (10 << 4)

#define ACCEL_ODR_0_78HZ        (0x01)
#define ACCEL_ODR_1_56HZ        (0x02)
#define ACCEL_ODR_3_12HZ        (0x03)
#define ACCEL_ODR_6_25HZ        (0x04)
#define ACCEL_ODR_12_5HZ        (0x05)
#define ACCEL_ODR_25HZ          (0x06)
#define ACCEL_ODR_50HZ          (0x07)
#define ACCEL_ODR_100HZ         (0x08)
#define ACCEL_ODR_200HZ         (0x09)
#define ACCEL_ODR_400HZ         (0x0A)
#define ACCEL_ODR_800HZ         (0x0B)
#define ACCEL_ODR_1600HZ        (0x0C)

/* Register 0x41 - ACC_RANGE accel range */

#define ACCEL_RANGE_2G          (0x00)
#define ACCEL_RANGE_4G          (0x01)
#define ACCEL_RANGE_8G          (0x02)
#define ACCEL_RANGE_16G         (0x03)

/* Register 0x42 - GYRO_CONFIG accel bandwidth */

#define GYRO_OSR4_MODE          (0 << 4)
#define GYRO_OSR2_MODE          (1 << 4)
#define GYRO_NORMAL_MODE        (2 << 4)
#define GYRO_CIC_MODE           (3 << 4)

#define GYRO_ODR_25HZ           (0x06)
#define GYRO_ODR_50HZ           (0x07)
#define GYRO_ODR_100HZ          (0x08)
#define GYRO_ODR_200HZ          (0x09)
#define GYRO_ODR_400HZ          (0x0A)
#define GYRO_ODR_800HZ          (0x0B)
#define GYRO_ODR_1600HZ         (0x0C)
#define GYRO_ODR_3200HZ         (0x0D)

/* Register 0x43 - GYR_RANGE gyr range */

#define GYRO_RANGE_2000         (0x00)
#define GYRO_RANGE_1000         (0x01)
#define GYRO_RANGE_500          (0x02)
#define GYRO_RANGE_250          (0x03)
#define GYRO_RANGE_125          (0x04)

/* Register 0x7d - PWR_CONF */

#define PWRCONF_APS_ON          (1 << 0)
#define PWRCONF_FSW_ON          (1 << 1)
#define PWRCONF_FUP_ON          (1 << 2)

/* Register 0x7d - PWR_CTRL */

#define PWRCTRL_AUX_EN          (1 << 0)
#define PWRCTRL_GYR_EN          (1 << 1)
#define PWRCTRL_ACC_EN          (1 << 2)
#define PWRCTRL_TEMP_EN         (1 << 3)

/* Register 0x7e - CMD */

#define CMD_SOFTRESET           (0xB6)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct bmi270_dev_s
{
#ifdef CONFIG_SENSORS_BMI270_I2C
  FAR struct i2c_master_s *i2c;  /* I2C interface */
  uint8_t                  addr; /* I2C address */
  int                      freq; /* Frequency <= 3.4MHz */
#else /* CONFIG_SENSORS_BMI270_SPI */
  FAR struct spi_dev_s    *spi;  /* SPI interface */
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const uint8_t g_bmi270_config_file[];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

uint8_t bmi270_getreg8(FAR struct bmi270_dev_s *priv, uint8_t regaddr);
void bmi270_putreg8(FAR struct bmi270_dev_s *priv, uint8_t regaddr,
                    uint8_t regval);
void bmi270_getregs(FAR struct bmi270_dev_s *priv, uint8_t regaddr,
                    FAR uint8_t *regval, int len);
void bmi270_putregs(FAR struct bmi270_dev_s *priv, uint8_t regaddr,
                    FAR uint8_t *regval, int len);

void bmi270_set_normal_imu(FAR struct bmi270_dev_s *priv);
int bmi270_init_seq(FAR struct bmi270_dev_s *priv);
int bmi270_checkid(FAR struct bmi270_dev_s *priv);

#endif  /* __INCLUDE_NUTTX_SENSORS_BMI270_BASE_H */
