/****************************************************************************
 * drivers/sensors/qmi8658_base.h
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

#ifndef __DRIVERS_SENSORS_QMI8658_BASE_H
#define __DRIVERS_SENSORS_QMI8658_BASE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/sensors/qmi8658.h>
#include <debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* QMI8658 Register Addresses */
#define QMI8658_REG_WHOAMI        (0x00)  /* Chip ID register */
#define QMI8658_REG_REVISION      (0x01)  /* Revision register */
#define QMI8658_REG_CTRL1         (0x02)  /* Control register 1 */
#define QMI8658_REG_CTRL2         (0x03)  /* Control register 2 (Accelerometer) */
#define QMI8658_REG_CTRL3         (0x04)  /* Control register 3 (Gyroscope) */
#define QMI8658_REG_CTRL5         (0x06)  /* Control register 5 (LPF settings) */
#define QMI8658_REG_CTRL7         (0x08)  /* Control register 7 (Sensor enable) */
#define QMI8658_REG_CTRL8         (0x09)  /* Control register 8 (Motion detection) */
#define QMI8658_REG_CTRL9         (0x0A)  /* Control register 9 (Commands) */
#define QMI8658_REG_CAL1_L        (0x0B)  /* Calibration register 1 low */
#define QMI8658_REG_CAL1_H        (0x0C)  /* Calibration register 1 high */
#define QMI8658_REG_CAL2_L        (0x0D)  /* Calibration register 2 low */
#define QMI8658_REG_CAL2_H        (0x0E)  /* Calibration register 2 high */
#define QMI8658_REG_CAL3_L        (0x0F)  /* Calibration register 3 low */
#define QMI8658_REG_CAL3_H        (0x10)  /* Calibration register 3 high */
#define QMI8658_REG_CAL4_L        (0x11)  /* Calibration register 4 low */
#define QMI8658_REG_CAL4_H        (0x12)  /* Calibration register 4 high */
#define QMI8658_REG_FIFO_WTM_TH   (0x13)  /* FIFO watermark threshold */
#define QMI8658_REG_FIFO_CTRL     (0x14)  /* FIFO control */
#define QMI8658_REG_FIFO_COUNT    (0x15)  /* FIFO sample count */
#define QMI8658_REG_FIFO_STATUS   (0x16)  /* FIFO status */
#define QMI8658_REG_FIFO_DATA     (0x17)  /* FIFO data */
#define QMI8658_REG_STATUS_INT    (0x2D)  /* Status interrupt */
#define QMI8658_REG_STATUS0       (0x2E)  /* Status register 0 */
#define QMI8658_REG_STATUS1       (0x2F)  /* Status register 1 */
#define QMI8658_REG_TIMESTAMP_L   (0x30)  /* Timestamp low */
#define QMI8658_REG_TIMESTAMP_M   (0x31)  /* Timestamp middle */
#define QMI8658_REG_TIMESTAMP_H   (0x32)  /* Timestamp high */
#define QMI8658_REG_TEMPERATURE_L (0x33)  /* Temperature low */
#define QMI8658_REG_TEMPERATURE_H (0x34)  /* Temperature high */
#define QMI8658_REG_AX_L          (0x35)  /* Accelerometer X low */
#define QMI8658_REG_AX_H          (0x36)  /* Accelerometer X high */
#define QMI8658_REG_AY_L          (0x37)  /* Accelerometer Y low */
#define QMI8658_REG_AY_H          (0x38)  /* Accelerometer Y high */
#define QMI8658_REG_AZ_L          (0x39)  /* Accelerometer Z low */
#define QMI8658_REG_AZ_H          (0x3A)  /* Accelerometer Z high */
#define QMI8658_REG_GX_L          (0x3B)  /* Gyroscope X low */
#define QMI8658_REG_GX_H          (0x3C)  /* Gyroscope X high */
#define QMI8658_REG_GY_L          (0x3D)  /* Gyroscope Y low */
#define QMI8658_REG_GY_H          (0x3E)  /* Gyroscope Y high */
#define QMI8658_REG_GZ_L          (0x3F)  /* Gyroscope Z low */
#define QMI8658_REG_GZ_H          (0x40)  /* Gyroscope Z high */
#define QMI8658_REG_COD_STATUS    (0x46)  /* Calibration-on-demand status */
#define QMI8658_REG_DQW_L         (0x49)  /* COD quaternion W low */
#define QMI8658_REG_DQW_H         (0x4A)  /* COD quaternion W high */
#define QMI8658_REG_DQX_L         (0x4B)  /* COD quaternion X low */
#define QMI8658_REG_DQX_H         (0x4C)  /* COD quaternion X high */
#define QMI8658_REG_RST_RESULT    (0x4D)  /* Reset result register */
#define QMI8658_REG_DQY_L         (0x4E)  /* COD quaternion Y low */
#define QMI8658_REG_DQY_H         (0x4F)  /* COD quaternion Y high */
#define QMI8658_REG_DQZ_L         (0x50)  /* COD quaternion Z low */
#define QMI8658_REG_DQZ_H         (0x51)  /* COD quaternion Z high */
#define QMI8658_REG_DVX_L         (0x52)  /* Self-test X low */
#define QMI8658_REG_DVX_H         (0x53)  /* Self-test X high */
#define QMI8658_REG_DVY_L         (0x54)  /* Self-test Y low */
#define QMI8658_REG_DVY_H         (0x55)  /* Self-test Y high */
#define QMI8658_REG_DVZ_L         (0x56)  /* Self-test Z low */
#define QMI8658_REG_DVZ_H         (0x57)  /* Self-test Z high */
#define QMI8658_REG_TAP_STATUS    (0x59)  /* Tap status */
#define QMI8658_REG_STEP_CNT_LOW  (0x5A)  /* Step counter low */
#define QMI8658_REG_STEP_CNT_MID  (0x5B)  /* Step counter middle */
#define QMI8658_REG_STEP_CNT_HIGH (0x5C)  /* Step counter high */
#define QMI8658_REG_RESET         (0x60)  /* Reset register */

/* Default values */
#define QMI8658_REG_WHOAMI_DEFAULT (0x05)
#define QMI8658_REG_STATUS_DEFAULT (0x03)
#define QMI8658_REG_RESET_DEFAULT  (0xB0)
#define QMI8658_REG_RST_RESULT_VAL (0x80)

/* Control register bit definitions */

/* CTRL1 - Control Register 1 */
#define QMI8658_CTRL1_ACC_EN       (1 << 0)
#define QMI8658_CTRL1_POWER_DOWN   (1 << 1)
#define QMI8658_CTRL1_FIFO_INT_EN  (1 << 2)
#define QMI8658_CTRL1_INT1_EN      (1 << 3)
#define QMI8658_CTRL1_INT2_EN      (1 << 4)
#define QMI8658_CTRL1_ADDR_AI_EN   (1 << 6)

/* CTRL5 - Control Register 5 (Low-pass filters) */
#define QMI8658_ACCEL_LPF_MASK (0xF9)
#define QMI8658_GYRO_LPF_MASK  (0x9F)

/* CTRL7 - Control Register 7 (Enable/Disable) */
#define QMI8658_CTRL7_ACC_EN    (1 << 0)  /* Accelerometer enable */
#define QMI8658_CTRL7_GYRO_EN   (1 << 1)  /* Gyroscope enable */
#define QMI8658_CTRL7_RESERVED  (0x7C)    /* Reserved bits 2-6 */
#define QMI8658_CTRL7_SYNC_MODE (1 << 7)  /* Synchronization mode */

/* CTRL7 bit masks */
#define QMI8658_CTRL7_ACC_EN_MASK  (0x01)    /* Accelerometer enable mask */
#define QMI8658_CTRL7_GYRO_EN_MASK (0x02)    /* Gyroscope enable mask */
#define QMI8658_CTRL7_EN_MASK      (0x03)    /* Enable bits mask */

/* CTRL9 - Control Register 9 (Commands) */
#define QMI8658_CTRL9_CMD_ACK      (0x00)    /* Acknowledge command */
#define QMI8658_CTRL9_CMD_RST_FIFO (0x04)    /* Reset FIFO command */
#define QMI8658_CTRL9_CMD_REQ_FIFO (0x05)    /* Request FIFO data command */
#define QMI8658_CTRL9_CMD_CALIB    (0xA2)    /* Calibration command */

/* STATUS0 - Status Register 0 (Data ready status) */
#define QMI8658_STATUS0_ACC_DRDY  (1 << 0)  /* Accelerometer data ready */
#define QMI8658_STATUS0_GYRO_DRDY (1 << 1)  /* Gyroscope data ready */
#define QMI8658_STATUS0_RESERVED  (0xFC)    /* Reserved bits 2-7 */

/* STATUS_INT - Status Interrupt Register */
#define QMI8658_STATUS_INT_AVAIL    (1 << 0)  /* Interrupt available */
#define QMI8658_STATUS_INT_LOCKED   (1 << 1)  /* Interrupt locked */
#define QMI8658_STATUS_INT_RESERVED (0x7C)    /* Reserved bits 2-6 */
#define QMI8658_STATUS_INT_CMD_DONE (1 << 7)  /* Command done interrupt */

/* Legacy compatibility definitions */
#define STATUS0_ACCEL_AVAIL       QMI8658_STATUS0_ACC_DRDY
#define STATUS0_GYRO_AVAIL        QMI8658_STATUS0_GYRO_DRDY
#define STATUS_INT_CTRL9_CMD_DONE QMI8658_STATUS_INT_CMD_DONE
#define STATUS_INT_LOCKED         QMI8658_STATUS_INT_LOCKED
#define STATUS_INT_AVAIL          QMI8658_STATUS_INT_AVAIL

/* Low-Pass Filter Modes */
#define QMI8658_LPF_MODE_0         (0x00)
#define QMI8658_LPF_MODE_1         (0x01)
#define QMI8658_LPF_MODE_2         (0x02)
#define QMI8658_LPF_MODE_3         (0x03)
#define QMI8658_LPF_OFF            (0x04)

/* Sample Synchronization Modes */
#define QMI8658_SYNC_MODE          (0x00)
#define QMI8658_ASYNC_MODE         (0x01)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* QMI8658 configuration structure */

struct qmi8658_config_s
{
  uint8_t acc_range;
  uint8_t gyro_range;
  uint8_t acc_enable;
  uint8_t gyro_enable;
  uint8_t temp_enable;
  uint8_t lp_mode;
};

/* Scale factors structure */

struct qmi8658_scale_factors_s
{
  float acc_scale;
  float gyro_scale;
  float temp_scale;
};

/****************************************************************************
 * Private Type
 ****************************************************************************/

struct qmi8658_dev_s
{
  FAR struct i2c_master_s *i2c;
  uint8_t addr;
  int freq;

  uint8_t acc_range;
  uint8_t gyro_range;
  uint8_t acc_odr;
  uint8_t gyro_odr;
  uint8_t acc_lpf;
  uint8_t gyro_lpf;
  uint8_t sample_mode;

  mutex_t dev_lock;

#ifdef CONFIG_SENSORS_QMI8658_UORB
  bool accel_enabled;
  bool gyro_enabled;
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* I2C Register Operations */

int qmi8658_readreg8(FAR struct qmi8658_dev_s *priv, uint8_t regaddr,
                     FAR uint8_t *regval);
int qmi8658_writereg8(FAR struct qmi8658_dev_s *priv, uint8_t regaddr,
                      uint8_t regval);
int qmi8658_modifyreg8(FAR struct qmi8658_dev_s *priv, uint8_t regaddr,
                       uint8_t clearbits, uint8_t setbits);
int qmi8658_readregs(FAR struct qmi8658_dev_s *priv, uint8_t regaddr,
                     FAR uint8_t *buffer, uint8_t len);

/* Basic Operations */

int qmi8658_checkid(FAR struct qmi8658_dev_s *priv);
int qmi8658_reset(FAR struct qmi8658_dev_s *priv);

/* Sensor Configuration */

int qmi8658_config_accelerometer(FAR struct qmi8658_dev_s *priv,
                                 uint8_t range,
                                 uint8_t odr,
                                 uint8_t lpf_mode);
int qmi8658_config_gyroscope(FAR struct qmi8658_dev_s *priv,
                             uint8_t range,
                             uint8_t odr,
                             uint8_t lpf_mode);

/* Sensor Control */

int qmi8658_set_accelerometer(FAR struct qmi8658_dev_s *priv, bool enable);
int qmi8658_set_gyroscope(FAR struct qmi8658_dev_s *priv, bool enable);

/* Sampling Mode */

int qmi8658_set_sample_mode(FAR struct qmi8658_dev_s *priv, bool sync);

/* Data Reading Functions */

int qmi8658_read_accel(FAR struct qmi8658_dev_s *priv,
                       FAR int16_t *data);
int qmi8658_read_gyro(FAR struct qmi8658_dev_s *priv,
                      FAR int16_t *data);
int qmi8658_read_temp(FAR struct qmi8658_dev_s *priv,
                      FAR int16_t *temperature);
int qmi8658_read_all(FAR struct qmi8658_dev_s *priv,
                     FAR struct qmi8658_data_s *data);

/* Status Functions */

int qmi8658_get_data_ready(FAR struct qmi8658_dev_s *priv);

/* Range Configuration Functions */

int qmi8658_set_acc_range(FAR struct qmi8658_dev_s *priv, uint8_t range);
int qmi8658_set_gyro_range(FAR struct qmi8658_dev_s *priv, uint8_t range);
int qmi8658_get_config(FAR struct qmi8658_dev_s *priv,
                       FAR struct qmi8658_config_s *config);
int qmi8658_get_scale_factors(FAR struct qmi8658_dev_s *priv,
                              FAR struct qmi8658_scale_factors_s *factors);

/* Initialization */

int qmi8658_initialize(FAR struct qmi8658_dev_s *priv);

#endif /* __DRIVERS_SENSORS_QMI8658_BASE_H */
