/****************************************************************************
 * drivers/sensors/sht3x.c
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
#include <nuttx/sensors/sht3x.h>
#include <nuttx/random.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_SHT3X)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SHT3X_DEBUG
#  define sht3x_dbg(x, ...)    _info(x, ##__VA_ARGS__)
#else
#  define sht3x_dbg(x, ...)    sninfo(x, ##__VA_ARGS__)
#endif

#ifndef CONFIG_SHT3X_I2C_FREQUENCY
#  define CONFIG_SHT3X_I2C_FREQUENCY 400000
#endif

/* Commands */

#define SHT3X_READ_SERIALNBR  0x3780 /* read serial number */
#define SHT3X_READ_STATUS     0xF32D /* read status register */
#define SHT3X_CLEAR_STATUS    0x3041 /* clear status register */
#define SHT3X_HEATER_ENABLE   0x306D /* enabled heater */
#define SHT3X_HEATER_DISABLE  0x3066 /* disable heater */
#define SHT3X_SOFT_RESET      0x30A2 /* soft reset */
#define SHT3X_MEAS_CLOCKSTR_H 0x2C06 /* measurement: clock stretching, high repeatability */
#define SHT3X_MEAS_CLOCKSTR_M 0x2C0D /* measurement: clock stretching, medium repeatability */
#define SHT3X_MEAS_CLOCKSTR_L 0x2C10 /* measurement: clock stretching, low repeatability */
#define SHT3X_MEAS_POLLING_H  0x2400 /* measurement: polling, high repeatability */
#define SHT3X_MEAS_POLLING_M  0x240B /* measurement: polling, medium repeatability */
#define SHT3X_MEAS_POLLING_L  0x2416 /* measurement: polling, low repeatability */
#define SHT3X_MEAS_PERI_05_H  0x2032 /* measurement: periodic 0.5 mps, high repeatability */
#define SHT3X_MEAS_PERI_05_M  0x2024 /* measurement: periodic 0.5 mps, medium repeatability */
#define SHT3X_MEAS_PERI_05_L  0x202F /* measurement: periodic 0.5 mps, low repeatability */
#define SHT3X_MEAS_PERI_1_H   0x2130 /* measurement: periodic 1 mps, high repeatability */
#define SHT3X_MEAS_PERI_1_M   0x2126 /* measurement: periodic 1 mps, medium repeatability */
#define SHT3X_MEAS_PERI_1_L   0x212D /* measurement: periodic 1 mps, low repeatability */
#define SHT3X_MEAS_PERI_2_H   0x2236 /* measurement: periodic 2 mps, high repeatability */
#define SHT3X_MEAS_PERI_2_M   0x2220 /* measurement: periodic 2 mps, medium repeatability */
#define SHT3X_MEAS_PERI_2_L   0x222B /* measurement: periodic 2 mps, low repeatability */
#define SHT3X_MEAS_PERI_4_H   0x2334 /* measurement: periodic 4 mps, high repeatability */
#define SHT3X_MEAS_PERI_4_M   0x2322 /* measurement: periodic 4 mps, medium repeatability */
#define SHT3X_MEAS_PERI_4_L   0x2329 /* measurement: periodic 4 mps, low repeatability */
#define SHT3X_MEAS_PERI_10_H  0x2737 /* measurement: periodic 10 mps, high repeatability */
#define SHT3X_MEAS_PERI_10_M  0x2721 /* measurement: periodic 10 mps, medium repeatability */
#define SHT3X_MEAS_PERI_10_L  0x272A /* measurement: periodic 10 mps, low repeatability */
#define SHT3X_FETCH_DATA      0xE000 /* readout measurements for periodic mode */
#define SHT3X_R_AL_LIM_LS     0xE102 /* read alert limits, low set */
#define SHT3X_R_AL_LIM_LC     0xE109 /* read alert limits, low clear */
#define SHT3X_R_AL_LIM_HS     0xE11F /* read alert limits, high set */
#define SHT3X_R_AL_LIM_HC     0xE114 /* read alert limits, high clear */
#define SHT3X_W_AL_LIM_HS     0x611D /* write alert limits, high set */
#define SHT3X_W_AL_LIM_HC     0x6116 /* write alert limits, high clear */
#define SHT3X_W_AL_LIM_LC     0x610B /* write alert limits, low clear */
#define SHT3X_W_AL_LIM_LS     0x6100 /* write alert limits, low set */
#define SHT3X_NO_SLEEP        0x303E /* disable sleep */

#define SHT3X_DEFAULT_MEAS_MODE SHT3X_MEAS_PERI_1_H

/****************************************************************************
 * Private
 ****************************************************************************/

struct sht3x_dev_s
{
  FAR struct i2c_master_s *i2c;  /* I2C interface */
  uint8_t addr;                  /* I2C address */
  bool valid;                    /* If cached readings are valid */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  bool unlinked;                 /* True, driver has been unlinked */
#endif
  struct timespec last_update;   /* Last time when sensor was read */
  struct sht3x_meas_data_s data; /* Cached sensor values */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  int16_t crefs;                 /* Number of open references */
#endif
  mutex_t devlock;
};

struct sht3x_word_s
{
  uint8_t data[2];
  uint8_t crc;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int     sht3x_do_transfer(FAR struct i2c_master_s *i2c,
                                 FAR struct i2c_msg_s *msgv,
                                 size_t nmsg);
static int     sht3x_write_cmd(FAR struct sht3x_dev_s *priv, uint16_t cmd,
                               FAR struct sht3x_word_s *params,
                               unsigned int num_params);
static int     sht3x_read_cmd(FAR struct sht3x_dev_s *priv, uint16_t cmd,
                              FAR struct sht3x_word_s *words,
                              unsigned int num_words);

/* Character driver methods */

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     sht3x_open(FAR struct file *filep);
static int     sht3x_close(FAR struct file *filep);
#endif
static ssize_t sht3x_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t sht3x_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int     sht3x_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     sht3x_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_sht3xfops =
{
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  sht3x_open,     /* open */
  sht3x_close,    /* close */
#else
  NULL,           /* open */
  NULL,           /* close */
#endif
  sht3x_read,     /* read */
  sht3x_write,    /* write */
  NULL,           /* seek */
  sht3x_ioctl,    /* ioctl */
  NULL            /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , sht3x_unlink  /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sht3x_do_transfer
 ****************************************************************************/

static int sht3x_do_transfer(FAR struct i2c_master_s *i2c,
                             FAR struct i2c_msg_s *msgv,
                             size_t nmsg)
{
  int ret = ret = I2C_TRANSFER(i2c, msgv, nmsg);
  if (ret >= 0)
    {
      return OK;
    }

  sht3x_dbg("transfer failed: %d\n", ret);
  return ret;
}

/****************************************************************************
 * Name: sht3x_write_cmd
 ****************************************************************************/

static int sht3x_write_cmd(FAR struct sht3x_dev_s *priv, uint16_t cmd,
                           FAR struct sht3x_word_s *params,
                           unsigned int num_params)
{
  struct i2c_msg_s msg[2];
  uint8_t cmd_buf[2];
  int ret;

  cmd_buf[0] = cmd >> 8;
  cmd_buf[1] = cmd >> 0;

  msg[0].frequency = CONFIG_SHT3X_I2C_FREQUENCY;
  msg[0].addr = priv->addr;
  msg[0].flags = 0;
  msg[0].buffer = cmd_buf;
  msg[0].length = 2;

  if (num_params)
    {
      msg[1].frequency = CONFIG_SHT3X_I2C_FREQUENCY;
      msg[1].addr = priv->addr;
      msg[1].flags = I2C_M_NOSTART;
      msg[1].buffer = (FAR uint8_t *)params;
      msg[1].length = num_params * sizeof(*params);
    }

  ret = sht3x_do_transfer(priv->i2c, msg, (num_params) ? 2 : 1);

  sht3x_dbg("cmd: 0x%04X num_params: %d ret: %d\n",
            cmd, num_params, ret);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: sht3x_read_cmd
 ****************************************************************************/

static int sht3x_read_cmd(FAR struct sht3x_dev_s *priv, uint16_t cmd,
                          FAR struct sht3x_word_s *words,
                          unsigned int num_words)
{
  struct i2c_msg_s msg[1];
  uint8_t addr_buf[2];
  int ret;

  addr_buf[0] = cmd >> 8;
  addr_buf[1] = cmd >> 0;

  msg[0].frequency = CONFIG_SHT3X_I2C_FREQUENCY;
  msg[0].addr = priv->addr;
  msg[0].flags = 0;
  msg[0].buffer = addr_buf;
  msg[0].length = 2;

  ret = sht3x_do_transfer(priv->i2c, msg, 1);

  sht3x_dbg("cmd: 0x%04X ret: %d\n", cmd, ret);

  if (ret < 0)
    {
      return ret;
    }

  msg[0].frequency = CONFIG_SHT3X_I2C_FREQUENCY;
  msg[0].addr = priv->addr;
  msg[0].flags = I2C_M_READ;
  msg[0].buffer = (FAR uint8_t *)words;
  msg[0].length = num_words * sizeof(*words);

  ret = sht3x_do_transfer(priv->i2c, msg, 1);

  sht3x_dbg("read cmd: 0x%04X num_params: %d ret: %d\n",
            cmd, num_words, ret);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: sht3x_crc_word
 ****************************************************************************/

static uint8_t sht3x_crc_word(uint16_t word)
{
  static const uint8_t crc_table[16] =
  {
    0x00, 0x31, 0x62, 0x53, 0xc4, 0xf5, 0xa6, 0x97,
    0xb9, 0x88, 0xdb, 0xea, 0x7d, 0x4c, 0x1f, 0x2e
  };

  uint8_t crc = 0xff;

  crc ^= word >> 8;
  crc  = (crc << 4) ^ crc_table[crc >> 4];
  crc  = (crc << 4) ^ crc_table[crc >> 4];
  crc ^= word >> 0;
  crc  = (crc << 4) ^ crc_table[crc >> 4];
  crc  = (crc << 4) ^ crc_table[crc >> 4];

  return crc;
}

/****************************************************************************
 * Name: sht3x_data_word2uint16
 ****************************************************************************/

static uint16_t sht3x_data_word2uint16(FAR const struct sht3x_word_s *word)
{
  return (word[0].data[0] << 8) | (word[0].data[1]);
}

/****************************************************************************
 * Name: sht3x_check_data_crc
 ****************************************************************************/

static int sht3x_check_data_crc(FAR const struct sht3x_word_s *words,
                                unsigned int num_words)
{
  while (num_words)
    {
      if (sht3x_crc_word(sht3x_data_word2uint16(words)) != words->crc)
        {
          return -1;
        }

      num_words--;
      words++;
    }

  return 0;
}

/****************************************************************************
 * Name: sht3x_softreset
 *
 * Description:
 *   Reset the SHT3x sensor.
 *
 ****************************************************************************/

static inline int sht3x_softreset(FAR struct sht3x_dev_s *priv)
{
  return sht3x_write_cmd(priv, SHT3X_SOFT_RESET, NULL, 0);
}

/****************************************************************************
 * Name: sht3x_temp_to_celsius
 *
 * Description:
 *   Convert raw temperature value to degrees Celsius.
 *
 ****************************************************************************/

static inline float sht3x_temp_to_celsius(uint16_t raw)
{
  /* Calculate temperature [°C]
   * T = -45 + 175 * rawValue / (2^16-1)
   */

  return 175.0f * (float)raw / 65535.0f - 45.0f;
}

/****************************************************************************
 * Name: sht3x_rh_to_percent
 *
 * Description:
 *   Convert raw humidity value to relative humidity [%].
 *
 ****************************************************************************/

static inline float sht3x_rh_to_percent(uint16_t raw)
{
  /* Calculate relative humidity [%RH]
   * RH = rawValue / (2^16-1) * 100
   */

  return 100.0f * (float)raw / 65535.0f;
}

/****************************************************************************
 * Name: sht3x_read_values
 ****************************************************************************/

static int sht3x_read_values(FAR struct sht3x_dev_s *priv,
                             FAR struct sht3x_meas_data_s *out)
{
  struct sht3x_word_s data[2];
  uint16_t temp16;
  uint16_t rh16;
  struct timespec ts;
  int ret;

  clock_systime_timespec(&ts);

  /* Read the raw data */

  ret = sht3x_read_cmd(priv, SHT3X_FETCH_DATA, data, 2);
  if (ret < 0)
    {
      sht3x_dbg("ERROR: sht3x_read_cmd failed: %d\n", ret);
      return ret;
    }

  if (sht3x_check_data_crc(data, 2) < 0)
    {
      sht3x_dbg("ERROR: sht3x_read_values crc failed\n");
      ret = -EIO;
      return ret;
    }

  temp16 = sht3x_data_word2uint16(data);
  rh16 = sht3x_data_word2uint16(&data[1]);
  add_sensor_randomness(ts.tv_nsec ^ ((int)temp16 << 16 | rh16));

  priv->data.temperature = sht3x_temp_to_celsius(temp16);
  priv->data.humidity = sht3x_rh_to_percent(rh16);
  priv->last_update = ts;
  priv->valid = true;

  *out = priv->data;
  return OK;
}

/****************************************************************************
 * Name: sht3x_open
 *
 * Description:
 *   This function is called whenever the SHT3x device is opened.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int sht3x_open(FAR struct file *filep)
{
  FAR struct inode       *inode = filep->f_inode;
  FAR struct sht3x_dev_s *priv  = inode->i_private;
  int ret;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Increment the count of open references on the driver */

  priv->crefs++;
  DEBUGASSERT(priv->crefs > 0);

  nxmutex_unlock(&priv->devlock);
  return OK;
}
#endif

/****************************************************************************
 * Name: sht3x_close
 *
 * Description:
 *   This routine is called when the SHT3x device is closed.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int sht3x_close(FAR struct file *filep)
{
  FAR struct inode       *inode = filep->f_inode;
  FAR struct sht3x_dev_s *priv  = inode->i_private;
  int ret;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Decrement the count of open references on the driver */

  DEBUGASSERT(priv->crefs > 0);
  priv->crefs--;

  /* If the count has decremented to zero and the driver has been unlinked,
   * then free memory now.
   */

  if (priv->crefs <= 0 && priv->unlinked)
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
 * Name: sht3x_read
 ****************************************************************************/

static ssize_t sht3x_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  /* Use ioctl to read from sensor */

  return -ENOSYS;
}

/****************************************************************************
 * Name: sht3x_write
 ****************************************************************************/

static ssize_t sht3x_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: sht3x_ioctl
 ****************************************************************************/

static int sht3x_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct sht3x_dev_s *priv = inode->i_private;
  int ret;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (priv->unlinked)
    {
      /* Do not allow operations on unlinked sensors. This allows
       * sensor use on hot swappable I2C bus.
       */

      nxmutex_unlock(&priv->devlock);
      return -ENODEV;
    }
#endif

  switch (cmd)
    {
      /* Soft reset the SHT3x, Arg: None */

      case SNIOC_RESET:
        {
          ret = sht3x_softreset(priv);
          sht3x_dbg("softreset ret: %d\n", ret);
        }
        break;

      /* Start periodic measurement mode, Arg: None */

      case SNIOC_START:
        {
          ret = sht3x_write_cmd(priv, SHT3X_DEFAULT_MEAS_MODE, NULL, 0);
          if (ret < 0)
            {
              sht3x_dbg("cannot start periodic measurement mode: %d\n", ret);
            }
        }
        break;

      case SNIOC_READ_CONVERT_DATA:
        {
          FAR struct sht3x_meas_data_s *data =
            (FAR struct sht3x_meas_data_s *)arg;

          ret = sht3x_read_values(priv, data);
          if (ret < 0)
            {
              sht3x_dbg("cannot read data: %d\n", ret);
            }
        }
        break;

      default:
        sht3x_dbg("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: sht3x_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int sht3x_unlink(FAR struct inode *inode)
{
  FAR struct sht3x_dev_s *priv;
  int ret;

  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv = (FAR struct sht3x_dev_s *)inode->i_private;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Are there open references to the driver data structure? */

  if (priv->crefs <= 0)
    {
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      return OK;
    }

  /* No... just mark the driver as unlinked and free the resources when
   * the last client closes their reference to the driver.
   */

  priv->unlinked = true;
  nxmutex_unlock(&priv->devlock);
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sht3x_register
 *
 * Description:
 *   Register the SHT3x character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the SHT3x
 *   addr    - The I2C address of the SHT3x.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sht3x_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                   uint8_t addr)
{
  FAR struct sht3x_dev_s *priv;
  int ret;

  DEBUGASSERT(i2c != NULL);

  /* Initialize the device structure */

  priv = (FAR struct sht3x_dev_s *)kmm_zalloc(sizeof(struct sht3x_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;

  nxmutex_init(&priv->devlock);

  /* Register the character driver */

  ret = register_driver(devpath, &g_sht3xfops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
    }

  return ret;
}
#endif /* CONFIG_I2C && CONFIG_SENSORS_SHT3X */
