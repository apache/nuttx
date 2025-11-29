/****************************************************************************
 * drivers/sensors/qmi8658_uorb.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/compiler.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/ioctl.h>
#include <nuttx/sensors/qmi8658.h>
#include <debug.h>
#include <errno.h>

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

/* CTRL5 - Control Register 5 (Low-pass filters */
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

/* Scale factors are defined in nuttx/sensors/qmi8658.h */

/* Sensor indices */

enum qmi8658_idx_e
{
  QMI8658_ACCEL_IDX = 0,
  QMI8658_GYRO_IDX,
  QMI8658_MAX_IDX
};

/****************************************************************************
 * Private Types
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

  bool accel_enabled;
  bool gyro_enabled;
};

struct qmi8658_sensor_s
{
  struct sensor_lowerhalf_s lower;
#ifdef CONFIG_SENSORS_QMI8658_POLL
  struct work_s work;
  uint32_t interval;
#endif
  float scale;
  FAR struct qmi8658_dev_s *dev;
  bool enabled;
};

struct qmi8658_uorb_dev_s
{
  struct qmi8658_dev_s base;
  struct qmi8658_sensor_s priv[QMI8658_MAX_IDX];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Register Operations */

static int qmi8658_readreg8(FAR struct qmi8658_dev_s *priv, uint8_t regaddr,
                             FAR uint8_t *regval);
static int qmi8658_writereg8(FAR struct qmi8658_dev_s *priv, uint8_t regaddr,
                              uint8_t regval);
static int qmi8658_modifyreg8(FAR struct qmi8658_dev_s *priv,
                              uint8_t regaddr, uint8_t clearbits,
                              uint8_t setbits);
static int qmi8658_readregs(FAR struct qmi8658_dev_s *priv, uint8_t regaddr,
                            FAR uint8_t *buffer, uint8_t len);

/* Basic Operations */

static int qmi8658_checkid(FAR struct qmi8658_dev_s *priv);
static int qmi8658_reset(FAR struct qmi8658_dev_s *priv);

/* Sensor Configuration */

static int qmi8658_config_accelerometer(FAR struct qmi8658_dev_s *priv,
                                         uint8_t range,
                                         uint8_t odr,
                                         uint8_t lpf_mode);
static int qmi8658_config_gyroscope(FAR struct qmi8658_dev_s *priv,
                                     uint8_t range,
                                     uint8_t odr,
                                     uint8_t lpf_mode);

/* Sensor Control */

static int qmi8658_set_accelerometer(FAR struct qmi8658_dev_s *priv,
                                     bool enable);
static int qmi8658_set_gyroscope(FAR struct qmi8658_dev_s *priv,
                                 bool enable);

/* Sampling Mode */

static int qmi8658_set_sample_mode(FAR struct qmi8658_dev_s *priv,
                                   bool sync);

/* Data Reading Functions */

static int qmi8658_read_accel(FAR struct qmi8658_dev_s *priv,
                              FAR int16_t *data);
static int qmi8658_read_gyro(FAR struct qmi8658_dev_s *priv,
                             FAR int16_t *data);
static int qmi8658_read_temp(FAR struct qmi8658_dev_s *priv,
                             FAR int16_t *temperature);

/* Scale Factors Functions */

static int qmi8658_get_scale_factors(FAR struct qmi8658_dev_s *priv,
  FAR struct qmi8658_scale_factors_s *factors);

/* Initialization */

static int qmi8658_initialize(FAR struct qmi8658_dev_s *priv);

/* Sensor methods */

static int qmi8658_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable);
#ifdef CONFIG_SENSORS_QMI8658_POLL
static int qmi8658_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *period_us);
#else
static int qmi8658_fetch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep,
                         FAR char *buffer, size_t buflen);
#endif

#ifdef CONFIG_SENSORS_QMI8658_POLL
static void qmi8658_accel_worker(FAR void *arg);
static void qmi8658_gyro_worker(FAR void *arg);
#endif

static int qmi8658_read_imu(FAR struct qmi8658_dev_s *dev,
                            FAR struct sensor_accel *accel,
                            FAR struct sensor_gyro *gyro);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_sensor_ops =
{
  NULL,                   /* open */
  NULL,                   /* close */
  qmi8658_activate,       /* activate */
#ifdef CONFIG_SENSORS_QMI8658_POLL
  qmi8658_set_interval,   /* set_interval */
#else
  NULL,                   /* set_interval */
#endif
  NULL,                   /* batch */
#ifdef CONFIG_SENSORS_QMI8658_POLL
  NULL,                   /* fetch */
#else
  qmi8658_fetch,          /* fetch */
#endif
  NULL,                   /* flush */
  NULL,                   /* selftest */
  NULL,                   /* set_calibvalue */
  NULL,                   /* calibrate */
  NULL,                   /* get_info */
  NULL,                   /* control */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qmi8658_readreg8
 *
 * Description:
 *   Read from an 8-bit QMI8658 register
 *
 ****************************************************************************/

static int qmi8658_readreg8(FAR struct qmi8658_dev_s *priv, uint8_t regaddr,
                            FAR uint8_t *regval)
{
  struct i2c_config_s config;
  int ret;

  sninfo("Reading reg 0x%02x (addr=0x%02x, freq=%d)\n",
         regaddr, priv->addr, priv->freq);

  config.frequency = priv->freq;
  config.address   = priv->addr;
  config.addrlen   = 7;

  ret = i2c_write(priv->i2c, &config, &regaddr, 1);
  if (ret < 0)
    {
      snerr(" i2c_write failed: %d\n", ret);
      return ret;
    }

  ret = i2c_read(priv->i2c, &config, regval, 1);
  if (ret < 0)
    {
      snerr(" i2c_read failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: qmi8658_writereg8
 *
 * Description:
 *   Write to an 8-bit QMI8658 register
 *
 ****************************************************************************/

static int qmi8658_writereg8(FAR struct qmi8658_dev_s *priv, uint8_t regaddr,
                             uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t data[2];
  int ret;

  sninfo("Writing reg 0x%02x = 0x%02x (addr=0x%02x, freq=%d)\n",
         regaddr, regval, priv->addr, priv->freq);

  config.frequency = priv->freq;
  config.address   = priv->addr;
  config.addrlen   = 7;

  data[0] = regaddr;
  data[1] = regval;

  ret = i2c_write(priv->i2c, &config, data, 2);
  if (ret < 0)
    {
      snerr(" i2c_write failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: qmi8658_modifyreg8
 *
 * Description:
 *   Modify an 8-bit QMI8658 register (clear and set bits)
 *
 ****************************************************************************/

static int qmi8658_modifyreg8(FAR struct qmi8658_dev_s *priv,
                              uint8_t regaddr, uint8_t clearbits,
                              uint8_t setbits)
{
  uint8_t regval;
  int ret;

  ret = qmi8658_readreg8(priv, regaddr, &regval);
  if (ret < 0)
    {
      return ret;
    }

  regval &= ~clearbits;
  regval |= setbits;

  ret = qmi8658_writereg8(priv, regaddr, regval);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: qmi8658_readregs
 *
 * Description:
 *   Read multiple consecutive registers using address auto-increment
 *
 ****************************************************************************/

static int qmi8658_readregs(FAR struct qmi8658_dev_s *priv, uint8_t regaddr,
                            FAR uint8_t *buffer, uint8_t len)
{
  struct i2c_config_s config;
  int ret;

  sninfo("Reading %d registers starting from 0x%02x "
         "(addr=0x%02x, freq=%d)\n",
         len, regaddr, priv->addr, priv->freq);

  config.frequency = priv->freq;
  config.address   = priv->addr;
  config.addrlen   = 7;

  ret = i2c_write(priv->i2c, &config, &regaddr, 1);
  if (ret < 0)
    {
      snerr(" i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Read multiple consecutive registers (auto-increment enabled) */

  ret = i2c_read(priv->i2c, &config, buffer, len);
  if (ret < 0)
    {
      snerr(" i2c_read failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: qmi8658_checkid
 *
 * Description:
 *   Check if the QMI8658 chip ID is correct
 *
 ****************************************************************************/

static int qmi8658_checkid(FAR struct qmi8658_dev_s *priv)
{
  uint8_t chip_id;
  int ret;
  int ret_lock;

  ret_lock = nxmutex_lock(&priv->dev_lock);
  if (ret_lock < 0)
    {
      return ret_lock;
    }

  ret = qmi8658_readreg8(priv, QMI8658_REG_WHOAMI, &chip_id);
  if (ret < 0)
    {
      snerr(" Failed to read chip ID\n");
      goto err_unlock;
    }

  if (chip_id != QMI8658_REG_WHOAMI_DEFAULT)
    {
      snerr(" Invalid chip ID: 0x%02x (expected 0x%02x)\n",
             chip_id, QMI8658_REG_WHOAMI_DEFAULT);
      ret = -ENODEV;
      goto err_unlock;
    }

  sninfo("QMI8658 chip ID verified: 0x%02x\n", chip_id);
  ret = OK;

err_unlock:
  nxmutex_unlock(&priv->dev_lock);
  return ret;
}

/****************************************************************************
 * Name: qmi8658_reset
 *
 * Description:
 *   Reset the QMI8658 sensor with proper verification
 *
 ****************************************************************************/

static int qmi8658_reset(FAR struct qmi8658_dev_s *priv)
{
  int ret;
  int ret_lock;
  uint8_t chip_id;

  ret_lock = nxmutex_lock(&priv->dev_lock);
  if (ret_lock < 0)
    {
      return ret_lock;
    }

  /* Issue device reset command */

  ret = qmi8658_writereg8(priv, QMI8658_REG_RESET,
                          QMI8658_REG_RESET_DEFAULT);
  if (ret < 0)
    {
      snerr("Failed to write reset command to QMI8658: %d\n", ret);
      goto err_unlock;
    }

  /* Wait for reset to complete (15ms minimum delay per datasheet) */

  up_mdelay(15);

  /* Verify device responsiveness after reset */

  ret = qmi8658_readreg8(priv, QMI8658_REG_WHOAMI, &chip_id);
  if (ret < 0)
    {
      snerr("Failed to read chip ID after reset: %d\n", ret);
      goto err_unlock;
    }

  if (chip_id != QMI8658_REG_WHOAMI_DEFAULT)
    {
      snerr("Invalid chip ID after reset: 0x%02x (expected "
            "0x%02x)\n", chip_id, QMI8658_REG_WHOAMI_DEFAULT);
      ret = -EIO;
      goto err_unlock;
    }

  /* Enable address auto-increment */

  ret = qmi8658_modifyreg8(priv, QMI8658_REG_CTRL1, 0,
                           QMI8658_CTRL1_ADDR_AI_EN);
  if (ret < 0)
    {
      snerr("Failed to enable address auto-increment: %d\n", ret);
      goto err_unlock;
    }

  sninfo("QMI8658 reset completed successfully (chip ID: 0x%02x)\n",
         chip_id);

err_unlock:
  nxmutex_unlock(&priv->dev_lock);
  return ret;
}

/****************************************************************************
 * Name: qmi8658_config_accelerometer
 *
 * Description:
 *   Configure accelerometer settings (range, ODR, low-pass filter)
 *
 ****************************************************************************/

static int qmi8658_config_accelerometer(FAR struct qmi8658_dev_s *priv,
                                        uint8_t range, uint8_t odr,
                                        uint8_t lpf_mode)
{
  int ret;
  int ret_lock;

  ret_lock = nxmutex_lock(&priv->dev_lock);
  if (ret_lock < 0)
    {
      return ret_lock;
    }

  sninfo("Configuring accelerometer range: %d (0x%02x)\n",
          range, range << 4);
  ret = qmi8658_modifyreg8(priv, QMI8658_REG_CTRL2, 0x30,
                           (range << 4));
  if (ret < 0)
    {
      snerr(" Failed to configure accelerometer range\n");
      goto err_unlock;
    }

  priv->acc_range = range;

  sninfo("Configuring accelerometer ODR: %d (0x%02x)\n", odr, odr);
  ret = qmi8658_modifyreg8(priv, QMI8658_REG_CTRL2, 0x0f, odr);
  if (ret < 0)
    {
      snerr(" Failed to configure accelerometer ODR\n");
      goto err_unlock;
    }

  uint8_t ctrl2_odr_val;
  qmi8658_readreg8(priv, QMI8658_REG_CTRL2, &ctrl2_odr_val);
  sninfo("CTRL2 after ODR config: 0x%02x\n", ctrl2_odr_val);

  priv->acc_odr = odr;

  uint8_t ctrl2_val;
  ret = qmi8658_readreg8(priv, QMI8658_REG_CTRL2, &ctrl2_val);
  if (ret < 0)
    {
      snerr(" Failed to read CTRL2 register\n");
    }
  else
    {
      sninfo("CTRL2 after accel config: 0x%02x\n", ctrl2_val);
    }

  if (lpf_mode != QMI8658_LPF_OFF)
    {
      ret = qmi8658_modifyreg8(priv, QMI8658_REG_CTRL5,
                               QMI8658_ACCEL_LPF_MASK, (lpf_mode << 1));
      if (ret < 0)
        {
          snerr(" Failed to configure accelerometer LPF\n");
          goto err_unlock;
        }
    }

  priv->acc_lpf = lpf_mode;
  ret = OK;

err_unlock:
  nxmutex_unlock(&priv->dev_lock);
  return ret;
}

/****************************************************************************
 * Name: qmi8658_config_gyroscope
 *
 * Description:
 *   Configure gyroscope settings (range, ODR, low-pass filter)
 *
 ****************************************************************************/

static int qmi8658_config_gyroscope(FAR struct qmi8658_dev_s *priv,
                                    uint8_t range, uint8_t odr,
                                    uint8_t lpf_mode)
{
  int ret;
  int ret_lock;

  ret_lock = nxmutex_lock(&priv->dev_lock);
  if (ret_lock < 0)
    {
      return ret_lock;
    }

  ret = qmi8658_modifyreg8(priv, QMI8658_REG_CTRL3, 0x30,
                           (range << 4));
  if (ret < 0)
    {
      snerr(" Failed to configure gyroscope range\n");
      goto err_unlock;
    }

  priv->gyro_range = range;

  ret = qmi8658_modifyreg8(priv, QMI8658_REG_CTRL3, 0x0f, odr);
  if (ret < 0)
    {
      snerr(" Failed to configure gyroscope ODR\n");
      goto err_unlock;
    }

  priv->gyro_odr = odr;

  if (lpf_mode != QMI8658_LPF_OFF)
    {
      ret = qmi8658_modifyreg8(priv, QMI8658_REG_CTRL5,
                                QMI8658_GYRO_LPF_MASK, (lpf_mode << 5));
      if (ret < 0)
        {
          snerr(" Failed to configure gyroscope LPF\n");
          goto err_unlock;
        }
    }

  priv->gyro_lpf = lpf_mode;
  ret = OK;

err_unlock:
  nxmutex_unlock(&priv->dev_lock);
  return ret;
}

/****************************************************************************
 * Name: qmi8658_set_accelerometer
 *
 * Description:
 *   Enable or disable the accelerometer
 *
 ****************************************************************************/

static int qmi8658_set_accelerometer(FAR struct qmi8658_dev_s *priv,
                                     bool enable)
{
  int ret;
  int ret_lock;

  ret_lock = nxmutex_lock(&priv->dev_lock);
  if (ret_lock < 0)
    {
      return ret_lock;
    }

  if (enable)
    {
      uint8_t ctrl7_val;

      ret = qmi8658_readreg8(priv, QMI8658_REG_CTRL7, &ctrl7_val);
      if (ret < 0)
        {
          snerr(" Failed to read CTRL7 register\n");
          goto err_unlock;
        }

      sninfo("CTRL7 before enable: 0x%02x\n", ctrl7_val);

      ret = qmi8658_modifyreg8(priv, QMI8658_REG_CTRL7, 0,
                               QMI8658_CTRL7_ACC_EN);
      if (ret < 0)
        {
          snerr(" Failed to enable accelerometer\n");
          goto err_unlock;
        }

      ret = qmi8658_readreg8(priv, QMI8658_REG_CTRL7, &ctrl7_val);
      if (ret < 0)
        {
          snerr(" Failed to read CTRL7 register after enable\n");
          goto err_unlock;
        }

      sninfo("CTRL7 after enable: 0x%02x\n", ctrl7_val);

      priv->accel_enabled = true;
      sninfo("Accelerometer enabled\n");
    }
  else
    {
      ret = qmi8658_modifyreg8(priv, QMI8658_REG_CTRL7,
                               QMI8658_CTRL7_ACC_EN, 0);
      if (ret < 0)
        {
          snerr(" Failed to disable accelerometer\n");
          goto err_unlock;
        }

      priv->accel_enabled = false;
      sninfo("Accelerometer disabled\n");
    }

  ret = OK;

err_unlock:
  nxmutex_unlock(&priv->dev_lock);
  return ret;
}

/****************************************************************************
 * Name: qmi8658_set_gyroscope
 *
 * Description:
 *   Enable or disable the gyroscope
 *
 ****************************************************************************/

static int qmi8658_set_gyroscope(FAR struct qmi8658_dev_s *priv, bool enable)
{
  int ret;
  int ret_lock;

  ret_lock = nxmutex_lock(&priv->dev_lock);
  if (ret_lock < 0)
    {
      return ret_lock;
    }

  if (enable)
    {
      ret = qmi8658_modifyreg8(priv, QMI8658_REG_CTRL7, 0,
                               QMI8658_CTRL7_GYRO_EN);
      if (ret < 0)
        {
          snerr(" Failed to enable gyroscope\n");
          goto err_unlock;
        }

      priv->gyro_enabled = true;
      sninfo("Gyroscope enabled\n");
    }
  else
    {
      ret = qmi8658_modifyreg8(priv, QMI8658_REG_CTRL7,
                               QMI8658_CTRL7_GYRO_EN, 0);
      if (ret < 0)
        {
          snerr(" Failed to disable gyroscope\n");
          goto err_unlock;
        }

      priv->gyro_enabled = false;
      sninfo("Gyroscope disabled\n");
    }

  ret = OK;

err_unlock:
  nxmutex_unlock(&priv->dev_lock);
  return ret;
}

/****************************************************************************
 * Name: qmi8658_set_sample_mode
 *
 * Description:
 *   Enable or disable synchronous sampling mode. When disabled, the sensor
 *   operates in asynchronous mode where accelerometer and gyroscope data
 *   are sampled independently.
 *
 ****************************************************************************/

static int qmi8658_set_sample_mode(FAR struct qmi8658_dev_s *priv, bool sync)
{
  int ret;
  int ret_lock;

  ret_lock = nxmutex_lock(&priv->dev_lock);
  if (ret_lock < 0)
    {
      return ret_lock;
    }

  if (sync)
    {
      ret = qmi8658_modifyreg8(priv, QMI8658_REG_CTRL7, 0,
                               QMI8658_CTRL7_SYNC_MODE);
      if (ret < 0)
        {
          snerr(" Failed to enable synchronous mode\n");
          goto err_unlock;
        }

      priv->sample_mode = QMI8658_SYNC_MODE;
      sninfo("Synchronous mode enabled\n");
    }
  else
    {
      ret = qmi8658_modifyreg8(priv, QMI8658_REG_CTRL7,
                               QMI8658_CTRL7_SYNC_MODE, 0);
      if (ret < 0)
        {
          snerr(" Failed to disable synchronous mode\n");
          goto err_unlock;
        }

      priv->sample_mode = QMI8658_ASYNC_MODE;
      sninfo("Synchronous mode disabled\n");
    }

  ret = OK;

err_unlock:
  nxmutex_unlock(&priv->dev_lock);
  return ret;
}

/****************************************************************************
 * Name: qmi8658_read_accel
 *
 * Description:
 *   Read accelerometer data (16-bit values) using batch reading
 *
 ****************************************************************************/

static int qmi8658_read_accel(FAR struct qmi8658_dev_s *priv,
                              FAR int16_t *data)
{
  uint8_t buffer[6];
  int ret;
  int ret_lock;

  ret_lock = nxmutex_lock(&priv->dev_lock);
  if (ret_lock < 0)
    {
      return ret_lock;
    }

  ret = qmi8658_readregs(priv, QMI8658_REG_AX_L, buffer, 6);
  if (ret < 0)
    {
      snerr(" Failed to read accelerometer data registers\n");
      goto err_unlock;
    }

  data[0] = (int16_t)((buffer[1] << 8) | buffer[0]);
  data[1] = (int16_t)((buffer[3] << 8) | buffer[2]);
  data[2] = (int16_t)((buffer[5] << 8) | buffer[4]);

  sninfo("Accel: X=%d, Y=%d, Z=%d (raw: %02x%02x, %02x%02x, %02x%02x)\n",
          data[0], data[1], data[2],
          buffer[1], buffer[0], buffer[3], buffer[2],
          buffer[5], buffer[4]);

  ret = OK;

err_unlock:
  nxmutex_unlock(&priv->dev_lock);
  return ret;
}

/****************************************************************************
 * Name: qmi8658_read_gyro
 *
 * Description:
 *   Read gyroscope data (16-bit values) using batch reading
 *
 ****************************************************************************/

static int qmi8658_read_gyro(FAR struct qmi8658_dev_s *priv,
                             FAR int16_t *data)
{
  uint8_t buffer[6];
  int ret;
  int ret_lock;

  ret_lock = nxmutex_lock(&priv->dev_lock);
  if (ret_lock < 0)
    {
      return ret_lock;
    }

  ret = qmi8658_readregs(priv, QMI8658_REG_GX_L, buffer, 6);
  if (ret < 0)
    {
      snerr(" Failed to read gyroscope data registers\n");
      goto err_unlock;
    }

  /* Convert to 16-bit signed values */

  data[0] = (int16_t)((buffer[1] << 8) | buffer[0]);
  data[1] = (int16_t)((buffer[3] << 8) | buffer[2]);
  data[2] = (int16_t)((buffer[5] << 8) | buffer[4]);

  ret = OK;

err_unlock:
  nxmutex_unlock(&priv->dev_lock);
  return ret;
}

/****************************************************************************
 * Name: qmi8658_read_temp
 *
 * Description:
 *   Read temperature data from QMI8658 sensor
 *
 * Input Parameters:
 *   priv        - Pointer to QMI8658 device structure
 *   temperature - Pointer to temperature data (16-bit signed)
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 * Notes:
 *   Temperature sensor is always enabled. Use QMI8658_TEMP_SCALE (256.0f)
 *   to convert raw value to degrees Celsius.
 *
 ****************************************************************************/

static int qmi8658_read_temp(FAR struct qmi8658_dev_s *priv,
                             FAR int16_t *temperature)
{
  uint8_t buffer[2];
  int ret;
  int ret_lock;

  ret_lock = nxmutex_lock(&priv->dev_lock);
  if (ret_lock < 0)
    {
      return ret_lock;
    }

  /* Read temperature registers (0x33-0x34) */

  ret = qmi8658_readregs(priv, QMI8658_REG_TEMPERATURE_L, buffer, 2);
  if (ret < 0)
    {
      snerr(" Failed to read temperature data registers\n");
      goto err_unlock;
    }

  /* Convert to 16-bit signed value (little-endian) */

  *temperature = (int16_t)((buffer[1] << 8) | buffer[0]);

  sninfo("Temperature: %d (raw value)\n", *temperature);
  ret = OK;

err_unlock:
  nxmutex_unlock(&priv->dev_lock);
  return ret;
}

/****************************************************************************
 * Name: qmi8658_initialize
 *
 * Description:
 *   Initialize the QMI8658 sensor with default configuration
 *
 ****************************************************************************/

static int qmi8658_initialize(FAR struct qmi8658_dev_s *priv)
{
  int ret;

  /* Initialize mutex */

  nxmutex_init(&priv->dev_lock);

  ret = qmi8658_checkid(priv);
  if (ret < 0)
    {
      return ret;
    }

  ret = qmi8658_reset(priv);
  if (ret < 0)
    {
      return ret;
    }

  ret = qmi8658_config_accelerometer(priv, QMI8658_ACC_FS_4G,
                                     QMI8658_ACC_ODR_1000Hz,
                                     QMI8658_LPF_MODE_0);
  if (ret < 0)
    {
      return ret;
    }

  ret = qmi8658_config_gyroscope(priv, QMI8658_GYRO_FS_1024DPS,
                                 QMI8658_GYRO_ODR_896_8Hz,
                                 QMI8658_LPF_MODE_0);
  if (ret < 0)
    {
      return ret;
    }

  ret = qmi8658_set_accelerometer(priv, true);
  if (ret < 0)
    {
      return ret;
    }

  ret = qmi8658_set_gyroscope(priv, true);
  if (ret < 0)
    {
      return ret;
    }

  ret = qmi8658_set_sample_mode(priv, true);
  if (ret < 0)
    {
      return ret;
    }

  sninfo("QMI8658 initialization completed successfully\n");
  return OK;
}

/****************************************************************************
 * Name: qmi8658_get_scale_factors
 *
 * Description:
 *   Get current scale factors for data conversion
 *
 ****************************************************************************/

static int qmi8658_get_scale_factors(FAR struct qmi8658_dev_s *priv,
  FAR struct qmi8658_scale_factors_s *factors)
{
  if (!priv || !factors)
    {
      return -EINVAL;
    }

  switch (priv->acc_range)
    {
    case QMI8658_ACC_FS_2G:
      factors->acc_scale = QMI8658_ACC_SCALE_2G;
      break;
    case QMI8658_ACC_FS_4G:
      factors->acc_scale = QMI8658_ACC_SCALE_4G;
      break;
    case QMI8658_ACC_FS_8G:
      factors->acc_scale = QMI8658_ACC_SCALE_8G;
      break;
    case QMI8658_ACC_FS_16G:
      factors->acc_scale = QMI8658_ACC_SCALE_16G;
      break;
    default:
      factors->acc_scale = QMI8658_ACC_SCALE_2G;
      break;
    }

  switch (priv->gyro_range)
    {
    case QMI8658_GYRO_FS_16DPS:
      factors->gyro_scale = QMI8658_GYRO_SCALE_16DPS;
      break;
    case QMI8658_GYRO_FS_32DPS:
      factors->gyro_scale = QMI8658_GYRO_SCALE_32DPS;
      break;
    case QMI8658_GYRO_FS_64DPS:
      factors->gyro_scale = QMI8658_GYRO_SCALE_64DPS;
      break;
    case QMI8658_GYRO_FS_128DPS:
      factors->gyro_scale = QMI8658_GYRO_SCALE_128DPS;
      break;
    case QMI8658_GYRO_FS_256DPS:
      factors->gyro_scale = QMI8658_GYRO_SCALE_256DPS;
      break;
    case QMI8658_GYRO_FS_512DPS:
      factors->gyro_scale = QMI8658_GYRO_SCALE_512DPS;
      break;
    case QMI8658_GYRO_FS_1024DPS:
      factors->gyro_scale = QMI8658_GYRO_SCALE_1024DPS;
      break;

    default:
      factors->gyro_scale = QMI8658_GYRO_SCALE_64DPS;
      break;
    }

  factors->temp_scale = QMI8658_TEMP_SCALE;

  return OK;
}

/****************************************************************************
 * Name: qmi8658_read_imu
 *
 * Description:
 *   Read accelerometer and/or gyroscope data from the QMI8658 sensor.
 *   This function reads raw sensor data and applies appropriate scaling
 *   factors to convert to physical units. Temperature data is also
 *   read for sensor compensation.
 *
 * Input Parameters:
 *   dev   - Pointer to the QMI8658 device structure
 *   accel - Pointer to accelerometer data structure (NULL if not needed)
 *   gyro  - Pointer to gyroscope data structure (NULL if not needed)
 *
 * Returned Value:
 *   OK on success; negated errno on failure
 *
 *   -EINVAL - Invalid device pointer
 *   -EIO    - I2C communication failure
 *
 * Assumptions:
 *   The device must be properly initialized before calling this function.
 *   Either accel or gyro (or both) must be non-NULL.
 *
 ****************************************************************************/

static int qmi8658_read_imu(FAR struct qmi8658_dev_s *dev,
                            FAR struct sensor_accel *accel,
                            FAR struct sensor_gyro *gyro)
{
  struct qmi8658_scale_factors_s scale_factors;
  int16_t accel_data[3];
  int16_t gyro_data[3];
  int16_t temperature;
  int ret;

  if (!dev)
    {
      return -EINVAL;
    }

  ret = qmi8658_get_scale_factors(dev, &scale_factors);
  if (ret < 0)
    {
      return ret;
    }

  /* Read temperature data (shared by both accel and gyro) */

  ret = qmi8658_read_temp(dev, &temperature);
  if (ret < 0)
    {
      return ret;
    }

  if (accel)
    {
      /* Read accelerometer data */

      ret = qmi8658_read_accel(dev, accel_data);
      if (ret < 0)
        {
          return ret;
        }

      accel->x = (float)accel_data[0] / scale_factors.acc_scale;
      accel->y = (float)accel_data[1] / scale_factors.acc_scale;
      accel->z = (float)accel_data[2] / scale_factors.acc_scale;
      accel->temperature = (float)temperature / scale_factors.temp_scale;
      accel->timestamp = sensor_get_timestamp();
    }

  if (gyro)
    {
      /* Read gyroscope data */

      ret = qmi8658_read_gyro(dev, gyro_data);
      if (ret < 0)
        {
          return ret;
        }

      gyro->x = (float)gyro_data[0] / scale_factors.gyro_scale;
      gyro->y = (float)gyro_data[1] / scale_factors.gyro_scale;
      gyro->z = (float)gyro_data[2] / scale_factors.gyro_scale;
      gyro->temperature = (float)temperature / scale_factors.temp_scale;
      gyro->timestamp = sensor_get_timestamp();
    }

  return OK;
}

/****************************************************************************
 * Name: qmi8658_activate
 *
 * Description:
 *   Enable or disable the sensor and manage polling work if enabled.
 *   When enabling a sensor in polling mode, this function starts a
 *   periodic work queue job to read sensor data at the configured
 *   interval. When disabling, it cancels any pending work.
 *
 * Input Parameters:
 *   lower - Pointer to the sensor lower-half structure
 *   filep - Pointer to the file structure (unused)
 *   enable - true to enable sensor, false to disable
 *
 * Returned Value:
 *   OK on success; negated errno on failure
 *
 *   -EINVAL - Invalid pointer parameters
 *   -EIO    - Work queue operation failed
 *
 * Assumptions:
 *   This function is called from the sensor framework when applications
 *   open/close the sensor device.
 *
 *   In polling mode, the work queue must be available for scheduling
 *   periodic sensor reads.
 *
 ****************************************************************************/

static int qmi8658_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable)
{
  FAR struct qmi8658_sensor_s *priv = (FAR struct qmi8658_sensor_s *)lower;
  FAR struct qmi8658_uorb_dev_s *dev =
    (FAR struct qmi8658_uorb_dev_s *)priv->dev;
  int ret = OK;

  if (!priv || !dev)
    {
      return -EINVAL;
    }

  int sensor_idx = priv - &dev->priv[0];

  priv->enabled = enable;

#ifdef CONFIG_SENSORS_QMI8658_POLL
  if (enable)
    {
      if (priv->interval > 0)
        {
          uint32_t delay = priv->interval / USEC_PER_TICK;
          if (sensor_idx == QMI8658_ACCEL_IDX)
            {
              ret = work_queue(HPWORK, &priv->work, qmi8658_accel_worker,
                               priv, delay);
            }
          else if (sensor_idx == QMI8658_GYRO_IDX)
            {
              ret = work_queue(HPWORK, &priv->work, qmi8658_gyro_worker,
                               priv, delay);
            }
        }
    }
  else
    {
      work_cancel(HPWORK, &priv->work);
    }
#endif

  return ret;
}

#ifdef CONFIG_SENSORS_QMI8658_POLL
/****************************************************************************
 * Name: qmi8658_set_interval
 *
 * Description:
 *   Set the polling interval for sensor data acquisition.
 *   This function updates the interval at which sensor data will be
 *   read when polling mode is enabled. The interval is specified
 *   in microseconds.
 *
 * Input Parameters:
 *   lower     - Pointer to the sensor lower-half structure
 *   filep     - Pointer to the file structure (unused)
 *   period_us - Pointer to the polling period in microseconds
 *
 * Returned Value:
 *   OK on success; negated errno on failure
 *
 *   -EINVAL - Invalid pointer parameters
 *
 * Assumptions:
 *   This function is called from the sensor framework when applications
 *   set the sensor polling interval via IOCTL or similar interface.
 *
 *   The new interval takes effect immediately for the next polling cycle.
 *
 *   interval_us should be greater than 0 for meaningful operation.
 *
 *   CONFIG_SENSORS_QMI8658_POLL must be enabled for this function
 *   to be available.
 *
 ****************************************************************************/

static int qmi8658_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *period_us)
{
  FAR struct qmi8658_sensor_s *priv = (FAR struct qmi8658_sensor_s *)lower;

  if (!priv || !period_us)
    {
      return -EINVAL;
    }

  priv->interval = *period_us;

  return OK;
}
#endif

#ifndef CONFIG_SENSORS_QMI8658_POLL
/****************************************************************************
 * Name: qmi8658_fetch
 *
 * Description:
 *   Fetch sensor data when polling mode is disabled.
 *   This function reads accelerometer or gyroscope data from the
 *   QMI8658 sensor and returns it to the caller. The function
 *   determines which sensor data to read based on the sensor index.
 *
 * Input Parameters:
 *   lower  - Pointer to the sensor lower-half structure
 *   filep  - Pointer to the file structure (unused)
 *   buffer - Buffer to store the sensor data
 *   buflen - Size of the buffer in bytes
 *
 * Returned Value:
 *   Number of bytes read on success; negated errno on failure
 *
 *   -EINVAL - Invalid pointer parameters or insufficient buffer size
 *   -EIO    - I2C communication failure
 *
 *   For accelerometer: sizeof(struct sensor_accel) bytes
 *   For gyroscope: sizeof(struct sensor_gyro) bytes
 *
 * Assumptions:
 *   This function is called when polling mode is not enabled.
 *   The application is responsible for providing a sufficiently
 *   large buffer to hold the sensor data structure.
 *
 *   The function detects the sensor type based on the sensor index
 *   and reads the appropriate data (accelerometer or gyroscope).
 *
 *   This is a blocking read operation that communicates with the
 *   hardware via I2C.
 *
 *   CONFIG_SENSORS_QMI8658_POLL must be disabled for this function
 *   to be available.
 *
 ****************************************************************************/

static int qmi8658_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep,
                        FAR char *buffer, size_t buflen)
{
  FAR struct qmi8658_sensor_s *priv = (FAR struct qmi8658_sensor_s *)lower;
  FAR struct qmi8658_uorb_dev_s *dev =
    (FAR struct qmi8658_uorb_dev_s *)priv->dev;
  int ret;

  if (!priv || !dev || !buffer)
    {
      return -EINVAL;
    }

  int sensor_idx = priv - &dev->priv[0];

  if (sensor_idx == QMI8658_ACCEL_IDX)
    {
      struct sensor_accel accel;
      ret = qmi8658_read_imu(&dev->base, &accel, NULL);
      if (ret < 0)
        {
          return ret;
        }

      if (buflen < sizeof(accel))
        {
          return -EINVAL;
        }

      memcpy(buffer, &accel, sizeof(accel));
      return sizeof(accel);
    }
  else if (sensor_idx == QMI8658_GYRO_IDX)
    {
      struct sensor_gyro gyro;
      ret = qmi8658_read_imu(&dev->base, NULL, &gyro);
      if (ret < 0)
        {
          return ret;
        }

      if (buflen < sizeof(gyro))
        {
          return -EINVAL;
        }

      memcpy(buffer, &gyro, sizeof(gyro));
      return sizeof(gyro);
    }

  return -EINVAL;
}
#endif

#ifdef CONFIG_SENSORS_QMI8658_POLL
/****************************************************************************
 * Name: qmi8658_accel_worker
 *
 * Description:
 *   Worker function for accelerometer data polling. This function is
 *   scheduled by the work queue to periodically read accelerometer
 *   data from the QMI8658 sensor and push it to the sensor framework.
 *
 * Input Parameters:
 *   arg - Pointer to the qmi8658_sensor_s structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The sensor is enabled and has a valid polling interval.
 *   The work queue is available for rescheduling.
 *
 ****************************************************************************/

static void qmi8658_accel_worker(FAR void *arg)
{
  FAR struct qmi8658_sensor_s *priv = (FAR struct qmi8658_sensor_s *)arg;
  FAR struct qmi8658_uorb_dev_s *dev =
    (FAR struct qmi8658_uorb_dev_s *)priv->dev;
  struct sensor_accel accel;
  int ret;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(dev != NULL);

  if (priv->enabled && priv->interval > 0)
    {
      uint32_t delay = priv->interval / USEC_PER_TICK;
      work_queue(HPWORK, &priv->work, qmi8658_accel_worker, priv, delay);
    }

  ret = qmi8658_read_imu(&dev->base, &accel, NULL);
  if (ret < 0)
    {
      return;
    }

  if (priv->lower.push_event && priv->lower.priv)
    {
      priv->lower.push_event(priv->lower.priv, &accel, sizeof(accel));
    }
}

/****************************************************************************
 * Name: qmi8658_gyro_worker
 *
 * Description:
 *   Worker function for gyroscope data polling. This function is
 *   scheduled by the work queue to periodically read gyroscope
 *   data from the QMI8658 sensor and push it to the sensor framework.
 *
 * Input Parameters:
 *   arg - Pointer to the qmi8658_sensor_s structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The sensor is enabled and has a valid polling interval.
 *   The work queue is available for rescheduling.
 *
 ****************************************************************************/

static void qmi8658_gyro_worker(FAR void *arg)
{
  FAR struct qmi8658_sensor_s *priv = (FAR struct qmi8658_sensor_s *)arg;
  FAR struct qmi8658_uorb_dev_s *dev =
    (FAR struct qmi8658_uorb_dev_s *)priv->dev;
  struct sensor_gyro gyro;
  int ret;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(dev != NULL);

  if (priv->enabled && priv->interval > 0)
    {
      uint32_t delay = priv->interval / USEC_PER_TICK;
      work_queue(HPWORK, &priv->work, qmi8658_gyro_worker, priv, delay);
    }

  ret = qmi8658_read_imu(&dev->base, NULL, &gyro);
  if (ret < 0)
    {
      return;
    }

  if (priv->lower.push_event && priv->lower.priv)
    {
      priv->lower.push_event(priv->lower.priv, &gyro, sizeof(gyro));
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qmi8658_uorb_register
 *
 * Description:
 *   Register the QMI8658 IMU sensor device with the NuttX sensor
 *   framework. This function initializes the device, sets up the
 *   accelerometer and gyroscope sensors, and registers them with
 *   the sensor subsystem.
 *
 * Input Parameters:
 *   devno - Device number for sensor registration
 *   i2c   - Pointer to the I2C master interface
 *   addr  - I2C slave address of the QMI8658 device
 *
 * Returned Value:
 *   OK on success; negated errno on failure
 *
 *   -EINVAL - Invalid I2C pointer
 *   -ENOMEM - Memory allocation failure
 *   -EIO    - Device initialization or registration failure
 *
 * Assumptions:
 *   The I2C bus is properly configured and available.
 *   The QMI8658 device is connected and powered.
 *
 ****************************************************************************/

int qmi8658_uorb_register(int devno, FAR struct i2c_master_s *i2c,
                          uint8_t addr)
{
  FAR struct qmi8658_uorb_dev_s *dev;
  struct sensor_lowerhalf_s *lower;
  struct qmi8658_scale_factors_s scale_factors;
  int ret = OK;
  int i;

  if (!i2c)
    {
      return -EINVAL;
    }

  dev = (FAR struct qmi8658_uorb_dev_s *)
        kmm_zalloc(sizeof(struct qmi8658_uorb_dev_s));
  if (!dev)
    {
      return -ENOMEM;
    }

  dev->base.i2c = i2c;
  dev->base.addr = addr;
  dev->base.freq = CONFIG_QMI8658_I2C_FREQUENCY;

  for (i = 0; i < QMI8658_MAX_IDX; i++)
    {
      FAR struct qmi8658_sensor_s *sensor = &dev->priv[i];

      sensor->dev = &dev->base;
      sensor->enabled = false;
#ifdef CONFIG_SENSORS_QMI8658_POLL
      sensor->interval = CONFIG_SENSORS_QMI8658_POLL_INTERVAL;

      memset(&sensor->work, 0, sizeof(sensor->work));
#endif

      lower = &sensor->lower;
      lower->type = (i == QMI8658_ACCEL_IDX) ? SENSOR_TYPE_ACCELEROMETER :
                                              SENSOR_TYPE_GYROSCOPE;
      lower->nbuffer = 2;
      lower->ops = &g_sensor_ops;

      ret = qmi8658_get_scale_factors(&dev->base, &scale_factors);
      if (ret < 0)
        {
          snerr("Failed to get scale factors: %d\n", ret);
          goto errout;
        }

      sensor->scale = (i == QMI8658_ACCEL_IDX) ? scale_factors.acc_scale :
                                                  scale_factors.gyro_scale;
    }

  ret = qmi8658_initialize(&dev->base);
  if (ret < 0)
    {
      snerr("Failed to initialize QMI8658: %d\n", ret);
      goto errout;
    }

  for (i = 0; i < QMI8658_MAX_IDX; i++)
    {
      FAR struct qmi8658_sensor_s *sensor = &dev->priv[i];
      FAR const char *devname;

      devname = (i == QMI8658_ACCEL_IDX) ? "qmi8658_accel" : "qmi8658_gyro";

      ret = sensor_register(&sensor->lower, devno);
      if (ret < 0)
        {
          snerr("Failed to register %s: %d\n", devname, ret);
          goto errout;
        }
    }

  return ret;

errout:
  for (i = 0; i < QMI8658_MAX_IDX; i++)
    {
      if (dev->priv[i].lower.type != 0)
        {
          sensor_unregister(&dev->priv[i].lower, devno);
        }
    }

  kmm_free(dev);

  return ret;
}
