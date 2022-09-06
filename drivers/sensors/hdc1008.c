/****************************************************************************
 * drivers/sensors/hdc1008.c
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

#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <time.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/hdc1008.h>
#include <nuttx/random.h>
#include <nuttx/signal.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_HDC1008)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_HDC1008_DEBUG
#  define hdc1008_dbg(x, ...)    _info(x, ##__VA_ARGS__)
#else
#  define hdc1008_dbg(x, ...)    sninfo(x, ##__VA_ARGS__)
#endif

#ifndef CONFIG_SHT21_I2C_FREQUENCY
#  define CONFIG_SHT21_I2C_FREQUENCY 400000
#endif

/* Macros to convert raw temperature and humidity to real values. Temperature
 * is scaled by 100, humidity by 10.
 */

#define RAW_TO_TEMP(x) (((x) * 16500 / 65536) - 4000)
#define RAW_TO_RH(x)   ((x) * 1000 / 65536)

/* Resolution for measurements. 8-bit is only valid for humidity. */

#define CONFIGURATION_RES_14BIT  0x00
#define CONFIGURATION_RES_11BIT  0x01
#define CONFIGURATION_RES_8BIT   0x02

/* HDC1008 registers */

#define HDC1008_REG_TEMPERATURE     0x00
#define HDC1008_REG_HUMIDITY        0x01
#define HDC1008_REG_CONFIGURATION   0x02
#define HDC1008_REG_SERIALID_0      0xFB /* Bits 0-15: Bit 24-39 of serial */
#define HDC1008_REG_SERIALID_1      0xFC /* Bits 0-15: Bit 8-23 of serial */
#define HDC1008_REG_SERIALID_2      0xFD /* Bits 7-15: Bit 0-7 of serial */

/* Configuration register bits */

#define HDC1008_CONFIGURATION_HRES_SHIFT      (8)       /* Bits 8-9: Humidity resolution */
#define HDC1008_CONFIGURATION_HRES_MASK       (0x03 << HDC1008_CONFIGURATION_HRES_SHIFT)
#  define HDC1008_CONFIGURATION_HRES_14BIT    (CONFIGURATION_RES_14BIT << HDC1008_CONFIGURATION_HRES_SHIFT)
#  define HDC1008_CONFIGURATION_HRES_11BIT    (CONFIGURATION_RES_11BIT << HDC1008_CONFIGURATION_HRES_SHIFT)
#  define HDC1008_CONFIGURATION_HRES_8BIT     (CONFIGURATION_RES_8BIT  << HDC1008_CONFIGURATION_HRES_SHIFT)
#define HDC1008_CONFIGURATION_TRES_SHIFT      (10)      /* Bit 10: Temperature resolution */
#define HDC1008_CONFIGURATION_TRES_MASK       (0x01 << HDC1008_CONFIGURATION_TRES_SHIFT)
#  define HDC1008_CONFIGURATION_TRES_14BIT    (CONFIGURATION_RES_14BIT << HDC1008_CONFIGURATION_TRES_SHIFT)
#  define HDC1008_CONFIGURATION_TRES_11BIT    (CONFIGURATION_RES_11BIT << HDC1008_CONFIGURATION_TRES_SHIFT)
#define HDC1008_CONFIGURATION_BTST            (1 << 11) /* Bit 11: Battery status */
#define HDC1008_CONFIGURATION_MODE            (1 << 12) /* Bit 12: Mode of acquisition */
#define HDC1008_CONFIGURATION_HEAT_SHIFT      (13)      /* Bit 13: Heater */
#define HDC1008_CONFIGURATION_HEAT_MASK       (0x01 << HDC1008_CONFIGURATION_HEAT_SHIFT)
#  define HDC1008_CONFIGURATION_HEAT_DISABLE  (0x00 << HDC1008_CONFIGURATION_HEAT_SHIFT)
#  define HDC1008_CONFIGURATION_HEAT_ENABLE   (0x01 << HDC1008_CONFIGURATION_HEAT_SHIFT)
                                                        /* Bit 14: Reserved */
#define HDC1008_CONFIGURATION_RST             (1 << 15) /* Bit 15: Software reset bit */

/****************************************************************************
 * Private
 ****************************************************************************/

struct hdc1008_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  bool unlinked;                /* True, driver has been unlinked */
#endif
  uint8_t mode;                 /* Acquisition mode */
  uint16_t configuration;       /* Configuration shadow register */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  int16_t crefs;                /* Number of open references */
#endif
  mutex_t devlock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     hdc1008_open(FAR struct file *filep);
static int     hdc1008_close(FAR struct file *filep);
#endif
static ssize_t hdc1008_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t hdc1008_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int     hdc1008_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     hdc1008_unlink(FAR struct inode *inode);
#endif
static int hdc1008_getreg(FAR struct hdc1008_dev_s *priv, uint8_t regaddr,
                          FAR uint16_t *regval);
static int hdc1008_putreg(FAR struct hdc1008_dev_s *priv, uint8_t regaddr,
                          FAR uint16_t regval);
static int hdc1008_reset(FAR struct hdc1008_dev_s *priv);
static int hdc1008_measure_trh(FAR struct hdc1008_dev_s *priv, int *t,
                               int *h);
static int hdc1008_measure_current_mode(struct hdc1008_dev_s *priv,
                                        struct hdc1008_conv_data_s *data);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_hdc1008fops =
{
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  hdc1008_open,     /* open */
  hdc1008_close,    /* close */
#else
  NULL,             /* open */
  NULL,             /* close */
#endif
  hdc1008_read,     /* read */
  hdc1008_write,    /* write */
  NULL,             /* seek */
  hdc1008_ioctl,    /* ioctl */
  NULL              /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , hdc1008_unlink  /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hdc1008_measure_trh
 *
 * Description:
 *   Read both temperature and humidity from the sensor
 *
 ****************************************************************************/

static int hdc1008_measure_trh(FAR struct hdc1008_dev_s *priv, int *t,
                               int *h)
{
  struct i2c_config_s config;
  uint8_t buf[4];
  uint8_t reg = HDC1008_REG_TEMPERATURE;
  int tmp;
  int ret;

  config.frequency = CONFIG_HDC1008_I2C_FREQUENCY;
  config.address = priv->addr;
  config.addrlen = 7;

  ret = i2c_write(priv->i2c, &config, &reg, 1);

  /* Wait for measurement to complete. Max should be about 20 ms if measuring
   * both temperature and humidity.
   */

  nxsig_usleep(20000);

  ret = i2c_read(priv->i2c, &config, buf, 4);
  if (ret < 0)
    {
      return ret;
    }

  /* Convert raw data from sensor to temperature/humidity */

  tmp = ((int)buf[0] << 8) | (int)buf[1];
  tmp = RAW_TO_TEMP(tmp);
  *t = tmp;

  tmp = ((int)buf[2] << 8) | (int)buf[3];
  tmp = RAW_TO_RH(tmp);
  *h = tmp;

  return 0;
}

/****************************************************************************
 * Name: hdc1008_measure_t_or_rh
 *
 * Description:
 *   Read both temperature and humidity from the sensor
 *
 ****************************************************************************/

static int hdc1008_measure_t_or_rh(FAR struct hdc1008_dev_s *priv,
                                   bool temperature, int *val)
{
  struct i2c_config_s config;
  uint8_t buf[4];
  uint8_t reg;
  int tmp;
  int ret;

  config.frequency = CONFIG_HDC1008_I2C_FREQUENCY;
  config.address = priv->addr;
  config.addrlen = 7;

  reg = temperature ? HDC1008_REG_TEMPERATURE : HDC1008_REG_HUMIDITY;

  ret = i2c_write(priv->i2c, &config, &reg, 1);

  /* Wait for measurement to complete. 10 ms wait should give enough
   * margin for either temperature/humidity at maximum resolution.
   */

  nxsig_usleep(10000);

  ret = i2c_read(priv->i2c, &config, buf, 2);
  if (ret < 0)
    {
      return ret;
    }

  /* Convert raw data from sensor to temperature/humidity */

  tmp = ((int)buf[0] << 8) | (int)buf[1];
  tmp = temperature ? RAW_TO_TEMP(tmp) : RAW_TO_RH(tmp);
  *val = tmp;

  return 0;
}

/****************************************************************************
 * Name: hdc1008_set_operational_mode
 *
 * Description:
 *   Configure the HDC1008 for measuring humitidy, temperature or both.
 *
 ****************************************************************************/

static int hdc1008_set_operational_mode(struct hdc1008_dev_s *priv,
                                        uint8_t mode)
{
  int ret;
  uint16_t reg;

  ret = hdc1008_getreg(priv, HDC1008_REG_CONFIGURATION, &reg);
  if (ret < 0)
    {
      return ret;
    }

  switch (mode)
    {
      case HDC1008_MEAS_TEMPERATURE:
        {
          reg &= ~HDC1008_CONFIGURATION_MODE;
          break;
        }

      case HDC1008_MEAS_HUMIDITY:
        {
          reg &= ~HDC1008_CONFIGURATION_MODE;
          break;
        }

      case HDC1008_MEAS_T_AND_RH:
        {
          reg |= HDC1008_CONFIGURATION_MODE;
          break;
        }

      default:
        return -EINVAL;
    }

  priv->mode = mode;

  return hdc1008_putreg(priv, HDC1008_REG_CONFIGURATION, reg);
}

/****************************************************************************
 * Name: hdc1008_set_temperature_resolution
 *
 * Description:
 *   Configure the HDC1008 temperature resolution
 *
 ****************************************************************************/

static int hdc1008_set_temperature_resolution(struct hdc1008_dev_s *priv,
                                              uint8_t resolution)
{
  int ret;
  uint16_t reg;

  ret = hdc1008_getreg(priv, HDC1008_REG_CONFIGURATION, &reg);
  if (ret < 0)
    {
      return ret;
    }

  reg &= ~HDC1008_CONFIGURATION_TRES_MASK;
  switch (resolution)
    {
      case 11:
        {
          reg |= HDC1008_CONFIGURATION_TRES_11BIT;
          break;
        }

      case 14:
        {
          reg |= HDC1008_CONFIGURATION_TRES_14BIT;
          break;
        }

      default:
        return -EINVAL;
    }

  return hdc1008_putreg(priv, HDC1008_REG_CONFIGURATION, reg);
}

/****************************************************************************
 * Name: hdc1008_set_humidity_resolution
 *
 * Description:
 *   Configure the HDC1008 humidity resolution
 *
 ****************************************************************************/

static int hdc1008_set_humidity_resolution(struct hdc1008_dev_s *priv,
                                           uint8_t resolution)
{
  int ret;
  uint16_t reg;

  ret = hdc1008_getreg(priv, HDC1008_REG_CONFIGURATION, &reg);
  if (ret < 0)
    {
      return ret;
    }

  reg &= ~HDC1008_CONFIGURATION_HRES_MASK;
  switch (resolution)
    {
      case 8:
        {
          reg |= HDC1008_CONFIGURATION_HRES_8BIT;
          break;
        }

      case 11:
        {
          reg |= HDC1008_CONFIGURATION_HRES_11BIT;
          break;
        }

      case 14:
        {
          reg |= HDC1008_CONFIGURATION_HRES_14BIT;
          break;
        }

      default:
        return -EINVAL;
    }

  return hdc1008_putreg(priv, HDC1008_REG_CONFIGURATION, reg);
}

/****************************************************************************
 * Name: hdc1008_set_heater_mode
 *
 * Description:
 *   Configure the HDC1008 heater mode
 *
 ****************************************************************************/

static int hdc1008_set_heater_mode(struct hdc1008_dev_s *priv,
                                   uint8_t mode)
{
  int ret;
  uint16_t reg;

  ret = hdc1008_getreg(priv, HDC1008_REG_CONFIGURATION, &reg);
  if (ret < 0)
    {
      return ret;
    }

  reg &= ~HDC1008_CONFIGURATION_HEAT_MASK;
  if (mode == 0)
    {
      reg |= HDC1008_CONFIGURATION_HEAT_DISABLE;
    }
  else if (mode == 1)
    {
      reg |= HDC1008_CONFIGURATION_HEAT_ENABLE;
    }
  else
    {
      return -EINVAL;
    }

  return hdc1008_putreg(priv, HDC1008_REG_CONFIGURATION, reg);
}

/****************************************************************************
 * Name: hdc1008_getreg
 *
 * Description:
 *   Get value of 16-bit register
 *
 ****************************************************************************/

static int hdc1008_getreg(FAR struct hdc1008_dev_s *priv, uint8_t regaddr,
                          FAR uint16_t *regval)
{
  int ret;
  struct i2c_config_s config;
  uint8_t buf[2];

  config.frequency = CONFIG_HDC1008_I2C_FREQUENCY;
  config.address = priv->addr;
  config.addrlen = 7;

  ret = i2c_write(priv->i2c, &config, &regaddr, 1);
  if (ret < 0)
    {
      return ret;
    }

  ret = i2c_read(priv->i2c, &config, buf, 2);
  if (ret < 0)
    {
      return ret;
    }

  *regval = (buf[0] << 8) | buf[1];
  return ret;
}

/****************************************************************************
 * Name: hdc1008_putreg
 *
 * Description:
 *   Set value of 16-bit register
 *
 ****************************************************************************/

static int hdc1008_putreg(FAR struct hdc1008_dev_s *priv, uint8_t regaddr,
                          FAR uint16_t regval)
{
  struct i2c_config_s config;
  uint8_t buf[3];

  config.frequency = CONFIG_HDC1008_I2C_FREQUENCY;
  config.address = priv->addr;
  config.addrlen = 7;

  hdc1008_dbg("addr=%02x; reg=%02x; val=%04x\n", priv->addr, regaddr,
              regval);

  buf[0] = regaddr;
  buf[1] = (uint8_t)((regval >> 8) & 0xff);
  buf[2] = (uint8_t)(regval & 0xff);
  return i2c_write(priv->i2c, &config, buf, 3);
}

/****************************************************************************
 * Name: hdc1008_open
 *
 * Description:
 *   This function is called whenever the hdc1008 device is opened.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int hdc1008_open(FAR struct file *filep)
{
  FAR struct inode *inode        = filep->f_inode;
  FAR struct hdc1008_dev_s *priv =
    (FAR struct hdc1008_dev_s *)inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Increases the reference count */

  ++priv->crefs;
  DEBUGASSERT(priv->crefs > 0);

  nxmutex_unlock(&priv->devlock);
  return 0;
}
#endif

/****************************************************************************
 * Name: hdc1008_close
 *
 * Description:
 *   This routine is called when the hdc1008 device is closed.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int hdc1008_close(FAR struct file *filep)
{
  FAR struct inode *inode        = filep->f_inode;
  FAR struct hdc1008_dev_s *priv =
    (FAR struct hdc1008_dev_s *)inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Decrement the reference count */

  DEBUGASSERT(priv->crefs > 0);
  --priv->crefs;

  /* If the reference count is zero, and the driver has been unlinked, free
   * the memory.
   */

  if ((priv->crefs <= 0) && priv->unlinked)
    {
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      return OK;
    }

  nxmutex_unlock(&priv->devlock);
  return OK;
}
#endif

/****************************************************************************
 * Name: hdc1008_read
 *
 * Description:
 *   Called when the driver is read from.
 *
 ****************************************************************************/

static ssize_t hdc1008_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode        = filep->f_inode;
  FAR struct hdc1008_dev_s *priv =
    (FAR struct hdc1008_dev_s *)inode->i_private;
  int ret;
  int len = 0;
  struct hdc1008_conv_data_s data;

  DEBUGASSERT(filep != NULL);

  /* Sanity check of input buffer argument */

  if (buffer == NULL)
    {
      return -EINVAL;
    }

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (priv->unlinked)
    {
      /* The driver is unliked, access is not allowed */

      nxmutex_unlock(&priv->devlock);
      return -ENODEV;
    }
#endif

  ret = hdc1008_measure_current_mode(priv, &data);
  if (ret < 0)
    {
      nxmutex_unlock(&priv->devlock);
      return ret;
    }

  if (priv->mode == HDC1008_MEAS_TEMPERATURE)
    {
      len = snprintf(buffer, buflen, "%d.%02d", data.temperature / 100,
                     data.temperature % 100);
    }
  else if (priv->mode == HDC1008_MEAS_HUMIDITY)
    {
      len = snprintf(buffer, buflen, "%d.%d", data.humidity / 10,
                     data.humidity % 10);
    }
  else if (priv->mode == HDC1008_MEAS_T_AND_RH)
    {
      len = snprintf(buffer, buflen, "%d.%02d %d.%d",
                     data.temperature / 100, data.temperature % 100,
                     data.humidity / 10, data.humidity % 10);
    }
  else
    {
      nxmutex_unlock(&priv->devlock);
      return -EINVAL;
    }

  nxmutex_unlock(&priv->devlock);
  return len;
}

/****************************************************************************
 * Name: hdc1008_measure_current_mode
 *
 * Description:
 *   Measure from sensor according to current mode (temperature, humidity or
 *   both).
 *
 ****************************************************************************/

static int hdc1008_measure_current_mode(struct hdc1008_dev_s *priv,
                                        struct hdc1008_conv_data_s *data)
{
  int ret;
  int temperature;
  int humidity;

  switch (priv->mode)
    {
      case HDC1008_MEAS_TEMPERATURE:
        ret = hdc1008_measure_t_or_rh(priv, true, &temperature);
        if (ret < 0)
          {
            return ret;
          }

        data->temperature = temperature;
        break;

      case HDC1008_MEAS_HUMIDITY:
        ret = hdc1008_measure_t_or_rh(priv, false, &humidity);
        if (ret < 0)
          {
            return ret;
          }

        data->humidity = humidity;
        break;

      case HDC1008_MEAS_T_AND_RH:
        ret = hdc1008_measure_trh(priv, &temperature, &humidity);
        if (ret < 0)
          {
            return ret;
          }

        data->temperature = temperature;
        data->humidity = humidity;
        break;

      default:
        ret = -EINVAL;
        break;
    }

    return ret;
}

/****************************************************************************
 * Name: hdc1008_write
 *
 * Description:
 *   Called when the driver is written to (not supported).
 *
 ****************************************************************************/

static ssize_t hdc1008_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: hdc1008_ioctl
 *
 * Description:
 *   Called when an ioctl() operation is made for the driver.
 *
 ****************************************************************************/

static int hdc1008_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode        = filep->f_inode;
  FAR struct hdc1008_dev_s *priv =
    (FAR struct hdc1008_dev_s *)inode->i_private;
  int ret;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (priv->unlinked)
    {
      /* The driver is unliked, access is not allowed */

      nxmutex_unlock(&priv->devlock);
      return -ENODEV;
    }
#endif

  switch (cmd)
    {
      case SNIOC_RESET:
        {
          ret = hdc1008_reset(priv);
          hdc1008_dbg("reset ret: %d\n", ret);
          break;
        }

      case SNIOC_SET_OPERATIONAL_MODE:
        {
          ret = hdc1008_set_operational_mode(priv, (uint8_t)arg);
          hdc1008_dbg("opmode ret: %d\n", ret);
          break;
        }

      case SNIOC_SET_RESOLUTION_T:
        {
          ret = hdc1008_set_temperature_resolution(priv, (uint8_t)arg);
          hdc1008_dbg("tres ret: %d\n", ret);
          break;
        }

      case SNIOC_SET_RESOLUTION_RH:
        {
          ret = hdc1008_set_humidity_resolution(priv, (uint8_t)arg);
          hdc1008_dbg("hres ret: %d\n", ret);
          break;
        }

      case SNIOC_SET_HEATER_MODE:
        {
          ret = hdc1008_set_heater_mode(priv, (uint8_t)arg);
          hdc1008_dbg("heater ret: %d\n", ret);
          break;
        }

      case SNIOC_GET_CONFIGURATION:
        {
          uint16_t reg;
          ret = hdc1008_getreg(priv, HDC1008_REG_CONFIGURATION, &reg);
          if (ret >= 0)
            {
              *(uint16_t *)arg = reg;
            }

          hdc1008_dbg("read config ret: %d\n", ret);
          break;
        }

      case SNIOC_MEASURE:
        {
          struct hdc1008_conv_data_s data;
          ret = hdc1008_measure_current_mode(priv, &data);
          if (ret >= 0)
            memcpy((struct hdc1008_conv_data_s *)arg, &data, sizeof(data));
          break;
        }

      default:
        {
          hdc1008_dbg("unrecognized cmd: %d\n", cmd);
          _info("unknown cmd: %d\n", cmd);
          ret = -ENOTTY;
          break;
        }
    }

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: hdc1008_unlink
 *
 * Description:
 *   Called when the driver is unlinked.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int hdc1008_unlink(FAR struct inode *inode)
{
  FAR struct hdc1008_dev_s *priv;
  int ret;

  DEBUGASSERT((inode != NULL) && (inode->i_private != NULL));
  priv = (FAR struct hdc1008_dev_s *)inode->i_private;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Are there any open references to the driver? */

  if (priv->crefs <= 0)
    {
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      return OK;
    }

  /* We still have open references, mark driver as unliked and wait for
   * everyone to close their instance.
   */

  priv->unlinked = true;
  nxmutex_unlock(&priv->devlock);
  return OK;
}
#endif

/****************************************************************************
 * Name: hdc1008_reset
 *
 * Description:
 *   Perform a software reset of the sensor
 *
 ****************************************************************************/

static int hdc1008_reset(FAR struct hdc1008_dev_s *priv)
{
  int ret;
  int count = 10;
  uint16_t reg;

  ret = hdc1008_getreg(priv, HDC1008_REG_CONFIGURATION, &reg);
  if (ret < 0)
    {
      return ret;
    }

  ret = hdc1008_putreg(priv, HDC1008_REG_CONFIGURATION,
                       reg | HDC1008_CONFIGURATION_RST);
  if (ret < 0)
    {
      return ret;
    }

  /* Now we wait until the sensor has reset */

  do
    {
      ret = hdc1008_getreg(priv, HDC1008_REG_CONFIGURATION, &reg);
      nxsig_usleep(1000);
      --count;
    }
  while ((reg & HDC1008_CONFIGURATION_RST) && (ret == OK) && count);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hdc1008_register
 *
 * Description:
 *   Register the HDC1008 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the HDC1008
 *   addr    - The I2C address of the HDC1008. The I2C address is
 *             configurable by two address pins, in the range of 0x40-0x43
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int hdc1008_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     uint8_t addr)
{
  FAR struct hdc1008_dev_s *priv;
  uint16_t reg;
  int ret;

  DEBUGASSERT(i2c != NULL);

  /* Initialize the driver structure */

  priv =
    (FAR struct hdc1008_dev_s *)kmm_zalloc(sizeof(struct hdc1008_dev_s));

  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate memory for driver data\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = addr;

  nxmutex_init(&priv->devlock);

  /* Register the driver */

  ret = register_driver(devpath, &g_hdc1008fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
    }

  sninfo("driver registered\n");

  /* 15 ms max startup time according to datasheet, let's wait just to be
   * sure that it is ready.
   */

  nxsig_usleep(15000);

  /* Set default configuration */

  ret = hdc1008_getreg(priv, HDC1008_REG_CONFIGURATION, &reg);
  if (ret < 0)
    {
      snerr("ERROR: failed to read default configuration\n");
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      return ret;
    }

  /* Enforce sensor default configuration at startup */

  reg = HDC1008_CONFIGURATION_MODE;
  ret = hdc1008_putreg(priv, HDC1008_REG_CONFIGURATION, reg);
  if (ret < 0)
    {
      snerr("ERROR: failed to set default configuration: %d\n", ret);
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      return ret;
    }

  priv->mode = HDC1008_MEAS_T_AND_RH;

  return ret;
}
#endif /* CONFIG_I2C && CONFIG_SENSORS_HDC1008 */
