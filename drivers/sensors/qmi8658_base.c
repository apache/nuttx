/****************************************************************************
 * drivers/sensors/qmi8658_base.c
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

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/sensors/qmi8658.h>
#include "qmi8658_base.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

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

int qmi8658_readreg8(FAR struct qmi8658_dev_s *priv,
                     uint8_t regaddr, FAR uint8_t *regval)
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

int qmi8658_writereg8(FAR struct qmi8658_dev_s *priv,
                       uint8_t regaddr, uint8_t regval)
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

int qmi8658_modifyreg8(FAR struct qmi8658_dev_s *priv, uint8_t regaddr,
                       uint8_t clearbits, uint8_t setbits)
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

int qmi8658_readregs(FAR struct qmi8658_dev_s *priv, uint8_t regaddr,
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qmi8658_checkid
 *
 * Description:
 *   Check if the QMI8658 chip ID is correct
 *
 ****************************************************************************/

int qmi8658_checkid(FAR struct qmi8658_dev_s *priv)
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

int qmi8658_reset(FAR struct qmi8658_dev_s *priv)
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
 * Name: qmi8658_set_gyro_range
 *
 * Description:
 *   Set gyroscope range dynamically
 *
 ****************************************************************************/

int qmi8658_set_gyro_range(FAR struct qmi8658_dev_s *priv, uint8_t range)
{
  int ret;
  int ret_lock;

  if (range > QMI8658_GYRO_FS_1024DPS)
    {
      snerr(" Invalid gyroscope range: %u\n", range);
      return -EINVAL;
    }

  ret_lock = nxmutex_lock(&priv->dev_lock);
  if (ret_lock < 0)
    {
      return ret_lock;
    }

  ret = qmi8658_modifyreg8(priv, QMI8658_REG_CTRL3,
                           0x70, (range << 4));  /* Clear bits 4-6, set new range */
  if (ret < 0)
    {
      snerr(" Failed to set gyroscope range\n");
      goto err_unlock;
    }

  priv->gyro_range = range;

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

int qmi8658_config_accelerometer(FAR struct qmi8658_dev_s *priv,
                                 uint8_t range,
                                 uint8_t odr,
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

int qmi8658_config_gyroscope(FAR struct qmi8658_dev_s *priv,
                             uint8_t range,
                             uint8_t odr,
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

int qmi8658_set_accelerometer(FAR struct qmi8658_dev_s *priv, bool enable)
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

#ifdef CONFIG_SENSORS_QMI8658_UORB
      priv->accel_enabled = true;
#endif
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

#ifdef CONFIG_SENSORS_QMI8658_UORB
      priv->accel_enabled = false;
#endif
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

int qmi8658_set_gyroscope(FAR struct qmi8658_dev_s *priv, bool enable)
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

#ifdef CONFIG_SENSORS_QMI8658_UORB
      priv->gyro_enabled = true;
#endif
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

#ifdef CONFIG_SENSORS_QMI8658_UORB
      priv->gyro_enabled = false;
#endif
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

int qmi8658_set_sample_mode(FAR struct qmi8658_dev_s *priv, bool sync)
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

int qmi8658_read_accel(FAR struct qmi8658_dev_s *priv,
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

  sninfo("Accel: X=%d, Y=%d, Z=%d (raw bytes: %02x%02x, %02x%02x, "
         "%02x%02x)\n",
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

int qmi8658_read_gyro(FAR struct qmi8658_dev_s *priv,
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

int qmi8658_read_temp(FAR struct qmi8658_dev_s *priv,
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
 * Name: qmi8658_read_all
 *
 * Description:
 *   Read all sensor data in one call
 *
 ****************************************************************************/

int qmi8658_read_all(FAR struct qmi8658_dev_s *priv,
                       FAR struct qmi8658_data_s *data)
{
  int ret;
  int ret_lock;
  uint8_t buffer[14];  /* Temperature (0x33-0x34) + Accel (0x35-0x3A) + Gyro (0x3B-0x40) */

  ret_lock = nxmutex_lock(&priv->dev_lock);
  if (ret_lock < 0)
    {
      return ret_lock;
    }

  /* Read all sensor data in one batch read using auto-increment */

  ret = qmi8658_readregs(priv, QMI8658_REG_TEMPERATURE_L, buffer, 14);
  if (ret < 0)
    {
      snerr(" Failed to read all sensor data registers\n");
      goto err_unlock;
    }

  data->temperature = (int16_t)((buffer[1] << 8) | buffer[0]);
  data->accel_x = (int16_t)((buffer[3] << 8) | buffer[2]);
  data->accel_y = (int16_t)((buffer[5] << 8) | buffer[4]);
  data->accel_z = (int16_t)((buffer[7] << 8) | buffer[6]);

  data->gyro_x = (int16_t)((buffer[9] << 8) | buffer[8]);
  data->gyro_y = (int16_t)((buffer[11] << 8) | buffer[10]);
  data->gyro_z = (int16_t)((buffer[13] << 8) | buffer[12]);

  sninfo("All sensors read: Temp=%d, Accel=(%d,%d,%d), Gyro=(%d,%d,%d)\n",
         data->temperature,
         data->accel_x, data->accel_y, data->accel_z,
         data->gyro_x, data->gyro_y, data->gyro_z);

  ret = OK;

err_unlock:
  nxmutex_unlock(&priv->dev_lock);
  return ret;
}

/****************************************************************************
 * Name: qmi8658_get_data_ready
 *
 * Description:
 *   Check if new sensor data is ready
 *
 ****************************************************************************/

int qmi8658_get_data_ready(FAR struct qmi8658_dev_s *priv)
{
  uint8_t status_int;
  int ret;
  int ret_lock;
  int result;

  ret_lock = nxmutex_lock(&priv->dev_lock);
  if (ret_lock < 0)
    {
      return ret_lock;
    }

  ret = qmi8658_readreg8(priv, QMI8658_REG_STATUS_INT, &status_int);
  if (ret < 0)
    {
      goto err_unlock;
    }

  result = (status_int & QMI8658_STATUS_INT_AVAIL) != 0;

#ifdef CONFIG_SENSORS_QMI8658_UORB
  sninfo("STATUS_INT register: 0x%02x (accel_en=%d, gyro_en=%d)\n",
         status_int, priv->accel_enabled, priv->gyro_enabled);
#else
  sninfo("STATUS_INT register: 0x%02x\n", status_int);
#endif

  ret = result;

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

int qmi8658_initialize(FAR struct qmi8658_dev_s *priv)
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

  ret = qmi8658_config_accelerometer(priv, QMI8658_ACC_FS_2G,
                                     QMI8658_ACC_ODR_1000Hz,
                                     QMI8658_LPF_MODE_0);
  if (ret < 0)
    {
      return ret;
    }

  ret = qmi8658_config_gyroscope(priv, QMI8658_GYRO_FS_64DPS,
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
 * Name: qmi8658_set_acc_range
 *
 * Description:
 *   Set accelerometer range dynamically
 *
 ****************************************************************************/

int qmi8658_set_acc_range(FAR struct qmi8658_dev_s *priv, uint8_t range)
{
  int ret;
  int ret_lock;

  if (range > QMI8658_ACC_FS_16G)
    {
      snerr(" Invalid accelerometer range: %u\n",
             range);
      return -EINVAL;
    }

  ret_lock = nxmutex_lock(&priv->dev_lock);
  if (ret_lock < 0)
    {
      return ret_lock;
    }

  sninfo("Setting accelerometer range: %d (0x%02x shifted)\n",
         range, range << 4);
  ret = qmi8658_modifyreg8(priv, QMI8658_REG_CTRL2,
                           0x30, range << 4);  /* Clear bits 4-5, set new range */

  uint8_t ctrl2_val;
  qmi8658_readreg8(priv, QMI8658_REG_CTRL2, &ctrl2_val);
  sninfo("CTRL2 after range change: 0x%02x\n", ctrl2_val);
  if (ret < 0)
    {
      snerr(" Failed to set accelerometer range\n");
      goto err_unlock;
    }

  priv->acc_range = range;

err_unlock:
  nxmutex_unlock(&priv->dev_lock);
  return ret;
}

/****************************************************************************
 * Name: qmi8658_get_config
 *
 * Description:
 *   Get current sensor configuration
 *
 ****************************************************************************/

int qmi8658_get_config(FAR struct qmi8658_dev_s *priv,
                       FAR struct qmi8658_config_s *config)
{
  if (!priv || !config)
    {
      return -EINVAL;
    }

  config->acc_range = priv->acc_range;
  config->gyro_range = priv->gyro_range;
#ifdef CONFIG_SENSORS_QMI8658_UORB
  config->acc_enable = priv->accel_enabled ? 1 : 0;
  config->gyro_enable = priv->gyro_enabled ? 1 : 0;
#else
  config->acc_enable = 0;  /* Unknown for non-uorb variant */
  config->gyro_enable = 0; /* Unknown for non-uorb variant */
#endif
  config->temp_enable = 1;  /* Temperature is always enabled */
  config->lp_mode = 0;      /* Current implementation doesn't use LP mode */

  return OK;
}

/****************************************************************************
 * Name: qmi8658_get_scale_factors
 *
 * Description:
 *   Get current scale factors for data conversion
 *
 ****************************************************************************/

int qmi8658_get_scale_factors(FAR struct qmi8658_dev_s *priv,
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
