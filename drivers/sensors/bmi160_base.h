/****************************************************************************
 * drivers/sensors/bmi160_base.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_BMI160_COMMOM_H
#define __INCLUDE_NUTTX_SENSORS_BMI160_COMMOM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/bmi160.h>

#include <stdlib.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <fixedmath.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEVID               0xd1

/* I2C  Address
 *
 * NOTE: If SDO pin is pulled to VDDIO, use 0x69
 */

#ifdef CONFIG_BMI160_I2C_ADDR_68
#define BMI160_I2C_ADDR     0x68
#else
#define BMI160_I2C_ADDR     0x69
#endif

#define BMI160_I2C_FREQ     400000

#define BMI160_CHIP_ID          (0x00) /* Chip ID */
#define BMI160_ERROR            (0x02) /* Error register */
#define BMI160_PMU_STAT         (0x03) /* Current power mode */
#define BMI160_DATA_0           (0x04) /* MAG X  7:0 (LSB) */
#define BMI160_DATA_1           (0x05) /* MAG X 15:8 (MSB) */
#define BMI160_DATA_2           (0x06) /* MAG Y  7:0 (LSB) */
#define BMI160_DATA_3           (0x07) /* MAG Y 15:8 (MSB) */
#define BMI160_DATA_4           (0x08) /* MAG Z  7:0 (LSB) */
#define BMI160_DATA_5           (0x09) /* MAG Z 15:8 (MSB) */
#define BMI160_DATA_6           (0x0A) /* RHALL  7:0 (LSB) */
#define BMI160_DATA_7           (0x0B) /* RHALL 15:8 (MSB) */
#define BMI160_DATA_8           (0x0C) /* GYR X  7:0 (LSB) */
#define BMI160_DATA_9           (0x0D) /* GYR X 15:8 (MSB) */
#define BMI160_DATA_10          (0x0E) /* GYR Y  7:0 (LSB) */
#define BMI160_DATA_11          (0x0F) /* GYR Y 15:8 (MSB) */
#define BMI160_DATA_12          (0x10) /* GYR Z  7:0 (LSB) */
#define BMI160_DATA_13          (0x11) /* GYR Z 15:8 (MSB) */
#define BMI160_DATA_14          (0x12) /* ACC X  7:0 (LSB) */
#define BMI160_DATA_15          (0x13) /* ACC X 15:8 (MSB) */
#define BMI160_DATA_16          (0x14) /* ACC Y  7:0 (LSB) */
#define BMI160_DATA_17          (0x15) /* ACC Y 15:8 (MSB) */
#define BMI160_DATA_18          (0x16) /* ACC Z  7:0 (LSB) */
#define BMI160_DATA_19          (0x17) /* ACC Z 15:8 (MSB) */
#define BMI160_SENSORTIME_0     (0x18) /* Sensor time 0 */
#define BMI160_SENSORTIME_1     (0x19) /* Sensor time 1 */
#define BMI160_SENSORTIME_2     (0x1A) /* Sensor time 2 */
#define BMI160_STAT             (0x1B) /* Status register */
#define BMI160_INTR_STAT_0      (0x1C) /* Interrupt status */
#define BMI160_INTR_STAT_1      (0x1D)
#define BMI160_INTR_STAT_2      (0x1E)
#define BMI160_INTR_STAT_3      (0x1F)
#define BMI160_TEMPERATURE_0    (0x20) /* Temperature */
#define BMI160_TEMPERATURE_1    (0x21)
#define BMI160_FIFO_LENGTH_0    (0x22) /* FIFO length */
#define BMI160_FIFO_LENGTH_1    (0x23)
#define BMI160_FIFO_DATA        (0x24)
#define BMI160_ACCEL_CONFIG     (0x40) /* ACCEL config for ODR, bandwidth and undersampling */
#define BMI160_ACCEL_RANGE      (0x41) /* ACCEL range */
#define BMI160_GYRO_CONFIG      (0x42) /* GYRO config for ODR and bandwidth */
#define BMI160_GYRO_RANGE       (0x43) /* GYRO range */
#define BMI160_MAG_CONFIG       (0x44) /* MAG config for ODR */
#define BMI160_FIFO_DOWN        (0x45) /* GYRO and ACCEL downsampling rates for FIFO */
#define BMI160_FIFO_CONFIG_0    (0x46) /* FIFO config */
#define BMI160_FIFO_CONFIG_1    (0x47)
#define BMI160_MAG_IF_0         (0x4B) /* MAG interface */
#define BMI160_MAG_IF_1         (0x4C)
#define BMI160_MAG_IF_2         (0x4D)
#define BMI160_MAG_IF_3         (0x4E)
#define BMI160_MAG_IF_4         (0x4F)
#define BMI160_INTR_ENABLE_0    (0x50) /* Interrupt enable */
#define BMI160_INTR_ENABLE_1    (0x51)
#define BMI160_INTR_ENABLE_2    (0x52)
#define BMI160_INTR_OUT_CTRL    (0x53)
#define BMI160_INTR_LATCH       (0x54) /* Latch duration */
#define BMI160_INTR_MAP_0       (0x55) /* Map interrupt */
#define BMI160_INTR_MAP_1       (0x56)
#define BMI160_INTR_MAP_2       (0x57)
#define BMI160_INTR_DATA_0      (0x58) /* Data source */
#define BMI160_INTR_DATA_1      (0x59)
#define BMI160_INTR_LOWHIGH_0   (0x5A) /* Threshold interrupt */
#define BMI160_INTR_LOWHIGH_1   (0x5B)
#define BMI160_INTR_LOWHIGH_2   (0x5C)
#define BMI160_INTR_LOWHIGH_3   (0x5D)
#define BMI160_INTR_LOWHIGH_4   (0x5E)
#define BMI160_INTR_MOTION_0    (0x5F)
#define BMI160_INTR_MOTION_1    (0x60)
#define BMI160_INTR_MOTION_2    (0x61)
#define BMI160_INTR_MOTION_3    (0x62)
#define BMI160_INTR_TAP_0       (0x63)
#define BMI160_INTR_TAP_1       (0x64)
#define BMI160_INTR_ORIENT_0    (0x65)
#define BMI160_INTR_ORIENT_1    (0x66)
#define BMI160_INTR_FLAT_0      (0x67)
#define BMI160_INTR_FLAT_1      (0x68)
#define BMI160_FOC_CONFIG       (0x69) /* Fast offset configuration */
#define BMI160_CONFIG           (0x6A) /* Miscellaneous configuration */
#define BMI160_IF_CONFIG        (0x6B) /* Serial interface configuration */
#define BMI160_PMU_TRIGGER      (0x6C) /* GYRO power mode trigger */
#define BMI160_SELF_TEST        (0x6D) /* Self test */
#define BMI160_NV_CONFIG        (0x70) /* SPI/I2C selection */
#define BMI160_OFFSET_0         (0x71) /* ACCEL and GYRO offset */
#define BMI160_OFFSET_1         (0x72)
#define BMI160_OFFSET_2         (0x73)
#define BMI160_OFFSET_3         (0x74)
#define BMI160_OFFSET_4         (0x75)
#define BMI160_OFFSET_5         (0x76)
#define BMI160_OFFSET_6         (0x77)
#define BMI160_STEP_COUNT_0     (0x78) /* Step counter interrupt */
#define BMI160_STEP_COUNT_1     (0x79)
#define BMI160_STEP_CONFIG_0    (0x7A) /* Step counter configuration */
#define BMI160_STEP_CONFIG_1    (0x7B)
#define BMI160_CMD              (0x7e) /* Command register */

/* Register 0x40 - ACCEL_CONFIG accel bandwidth */

#define ACCEL_OSR4_AVG1   (0 << 4)
#define ACCEL_OSR2_AVG2   (1 << 4)
#define ACCEL_NORMAL_AVG4 (2 << 4)
#define ACCEL_CIC_AVG8    (3 << 4)
#define ACCEL_RES_AVG2    (4 << 4)
#define ACCEL_RES_AVG4    (5 << 4)
#define ACCEL_RES_AVG8    (6 << 4)
#define ACCEL_RES_AVG16   (7 << 4)
#define ACCEL_RES_AVG32   (8 << 4)
#define ACCEL_RES_AVG64   (9 << 4)
#define ACCEL_RES_AVG128  (10 << 4)

#define ACCEL_ODR_0_78HZ      (0x01)
#define ACCEL_ODR_1_56HZ      (0x02)
#define ACCEL_ODR_3_12HZ      (0x03)
#define ACCEL_ODR_6_25HZ      (0x04)
#define ACCEL_ODR_12_5HZ      (0x05)
#define ACCEL_ODR_25HZ        (0x06)
#define ACCEL_ODR_50HZ        (0x07)
#define ACCEL_ODR_100HZ       (0x08)
#define ACCEL_ODR_200HZ       (0x09)
#define ACCEL_ODR_400HZ       (0x0A)
#define ACCEL_ODR_800HZ       (0x0B)
#define ACCEL_ODR_1600HZ      (0x0C)

/* Register 0x42 - GYRO_CONFIG accel bandwidth */

#define GYRO_OSR4_MODE   (0x00 << 4)
#define GYRO_OSR2_MODE   (0x01 << 4)
#define GYRO_NORMAL_MODE (0x02 << 4)
#define GYRO_CIC_MODE    (0x03 << 4)

#define GYRO_ODR_25HZ         (0x06)
#define GYRO_ODR_50HZ         (0x07)
#define GYRO_ODR_100HZ        (0x08)
#define GYRO_ODR_200HZ        (0x09)
#define GYRO_ODR_400HZ        (0x0A)
#define GYRO_ODR_800HZ        (0x0B)
#define GYRO_ODR_1600HZ       (0x0C)
#define GYRO_ODR_3200HZ       (0x0D)

/* Register 0x7b STEP_CONFIG_1 */

#define STEP_CNT_EN           (1 << 3)

/* Register 0x7e - CMD */

#define ACCEL_PM_SUSPEND      (0X10)
#define ACCEL_PM_NORMAL       (0x11)
#define ACCEL_PM_LOWPOWER     (0X12)
#define GYRO_PM_SUSPEND       (0x14)
#define GYRO_PM_NORMAL        (0x15)
#define GYRO_PM_FASTSTARTUP   (0x17)
#define MAG_PM_SUSPEND        (0x18)
#define MAG_PM_NORMAL         (0x19)
#define MAG_PM_LOWPOWER       (0x1A)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct bmi160_dev_s
{
#ifdef CONFIG_SENSORS_BMI160_I2C
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
  int freq;                     /* Frequency <= 3.4MHz */

#else /* CONFIG_SENSORS_BMI160_SPI */
  FAR struct spi_dev_s *spi;    /* SPI interface */

#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

uint8_t bmi160_getreg8(FAR struct bmi160_dev_s *priv, uint8_t regaddr);
void bmi160_putreg8(FAR struct bmi160_dev_s *priv, uint8_t regaddr,
                    uint8_t regval);
uint16_t bmi160_getreg16(FAR struct bmi160_dev_s *priv, uint8_t regaddr);
void bmi160_getregs(FAR struct bmi160_dev_s *priv, uint8_t regaddr,
                    uint8_t *regval, int len);

int bmi160_checkid(FAR struct bmi160_dev_s *priv);

#endif /* __INCLUDE_NUTTX_SENSORS_BMI160_COMMOM_H */
