/****************************************************************************
 * drivers/sensors/scd41.c
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

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <time.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/scd41.h>
#include <nuttx/random.h>

#if defined(CONFIG_SENSORS_SCD41)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SCD41_DEBUG
#  define scd41_dbg(x, ...)    _info(x, ##__VA_ARGS__)
#else
#  define scd41_dbg(x, ...)    sninfo(x, ##__VA_ARGS__)
#endif

#ifndef CONFIG_SCD41_I2C_FREQUENCY
#  define CONFIG_SCD41_I2C_FREQUENCY 100000
#endif

#define SCD41_I2C_RETRIES 3

/* SCD41 command words */

#define SCD41_CMD_START_MEASUREMENT     0x21b1
#define SCD41_CMD_STOP_MEASUREMENT      0x3f86
#define SCD41_CMD_GET_DATA_READY        0xe4b8
#define SCD41_CMD_READ_MEASUREMENT      0xec05
#define SCD41_CMD_SET_ASC               0x2416
#define SCD41_CMD_GET_ASC               0x2313
#define SCD41_CMD_SET_FRC               0x362f
#define SCD41_CMD_SET_TEMP_OFFSET       0x241d
#define SCD41_CMD_GET_TEMP_OFFSET       0x2318
#define SCD41_CMD_SET_ALT_COMPENSATION  0x2427
#define SCD41_CMD_GET_ALT_COMPENSATION  0x2322
#define SCD41_CMD_SET_PRESSURE_COMP     0xe000
#define SCD41_CMD_SOFT_RESET            0x3646
#define SCD41_CMD_LOWPOWER_MODE         0x21ac
#define SCD41_CMD_PERSIST_SETTINGS      0x3615
#define SCD41_CMD_GET_SERIAL_NUMBER     0x3682
#define SCD41_CMD_START_SELFTEST        0x3639
#define SCD41_CMD_FACTORY_RESET         0x3632
#define SCD41_CMD_ONESHOT_MEASUREMENT   0x219d
#define SCD41_CMD_ONESHOT_MEASURE_RHT   0x2196

#define SCD41_DEFAULT_MEASUREMENT_INTERVAL  5 /* seconds */
#define SCD41_DEFAULT_PRESSURE_COMPENSATION 0
#define SCD41_DEFAULT_ALTITUDE_COMPENSATION 0
#define SCD41_DEFAULT_TEMPERATURE_OFFSET    0

/****************************************************************************
 * Private
 ****************************************************************************/

struct scd41_dev_s
{
#ifdef CONFIG_SCD41_I2C
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
#endif
  bool valid;                   /* If cached readings are valid */
  bool started;                 /* If continuous measurement is enabled */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  bool unlinked;                /* True, driver has been unlinked */
#endif
  struct timespec last_update;  /* Last time when sensor was read */
  float co2;                    /* Cached CO₂ (PPM) */
  float temperature;            /* Cached temperature (°C) */
  float humidity;               /* Cached humidity (RH%) */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  int16_t crefs;                /* Number of open references */
#endif
  sem_t devsem;
  uint16_t pressure_comp;       /* Pressure compensation in mbar (non-zero
                                 * value overrides altitude compensation). */
  uint16_t altitude_comp;       /* Altitude compensation in meters */
  uint16_t interval;            /* Background measurement interval in
                                 * seconds (2 to 1800). */
  uint16_t temperature_offset;  /* Temperature offset in 0.01 Kelvin. */
};

struct scd41_word_s
{
  uint8_t data[2];
  uint8_t crc;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IO Helpers */

#ifdef CONFIG_SCD41_I2C
static int scd41_do_transfer(FAR struct i2c_master_s *i2c,
                             FAR struct i2c_msg_s *msgv,
                             size_t nmsg);
#endif
static int scd41_write_cmd(FAR struct scd41_dev_s *priv, uint16_t cmd,
                           FAR struct scd41_word_s *params,
                           unsigned int num_params);
static int scd41_read_words(FAR struct scd41_dev_s *priv,
                            FAR struct scd41_word_s *words,
                            unsigned int num_words);

/* Data conversion */

static uint8_t scd41_crc_word(uint16_t word);
static void scd41_set_command_param(FAR struct scd41_word_s *param,
                                    uint16_t value);
static int scd41_check_data_crc(FAR const struct scd41_word_s *words,
                                unsigned int num_words);
static uint16_t scd41_data_word2uint16(FAR const struct scd41_word_s *word);

/* Driver features */

static int scd41_read_values(FAR struct scd41_dev_s *priv, FAR float *temp,
                             FAR float *rh, FAR float *co2, bool wait);
static int scd41_configure(FAR struct scd41_dev_s *priv, bool start);

/* Character driver methods */

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     scd41_open(FAR struct file *filep);
static int     scd41_close(FAR struct file *filep);
#endif
static ssize_t scd41_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t scd41_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int     scd41_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     scd41_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_scd41fops =
{
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  scd41_open,     /* open */
  scd41_close,    /* close */
#else
  NULL,           /* open */
  NULL,           /* close */
#endif
  scd41_read,     /* read */
  scd41_write,    /* write */
  NULL,           /* seek */
  scd41_ioctl,    /* ioctl */
  NULL            /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , scd41_unlink /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: scd41_do_transfer
 ****************************************************************************/

#ifdef CONFIG_SCD41_I2C
static int scd41_do_transfer(FAR struct i2c_master_s *i2c,
                             FAR struct i2c_msg_s *msgv,
                             size_t nmsg)
{
  int ret = -EIO;
  int retries;

  for (retries = 0; retries < SCD41_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(i2c, msgv, nmsg);
      if (ret >= 0)
        {
          return 0;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */

#ifdef CONFIG_I2C_RESET
          if (retries == SCD41_I2C_RETRIES - 1)
            {
              break;
            }

          ret = I2C_RESET(i2c);
          if (ret < 0)
            {
              scd41_dbg("I2C_RESET failed: %d\n", ret);
              return ret;
            }
#endif
        }
    }

  scd41_dbg("xfer failed: %d\n", ret);
  return ret;
}
#endif

/****************************************************************************
 * Name: scd41_write_cmd
 ****************************************************************************/

static int scd41_write_cmd(FAR struct scd41_dev_s *priv, uint16_t cmd,
                           FAR struct scd41_word_s *params,
                           unsigned int num_params)
{
#ifdef CONFIG_SCD41_I2C
  struct i2c_msg_s msg[2];
  uint8_t cmd_buf[2];
  int ret;

  cmd_buf[0] = cmd >> 8;
  cmd_buf[1] = cmd >> 0;

  msg[0].frequency = CONFIG_SCD41_I2C_FREQUENCY;
  msg[0].addr = priv->addr;
  msg[0].flags = 0;
  msg[0].buffer = cmd_buf;
  msg[0].length = 2;

  if (num_params)
    {
      msg[1].frequency = CONFIG_SCD41_I2C_FREQUENCY;
      msg[1].addr = priv->addr;
      msg[1].flags = I2C_M_NOSTART;
      msg[1].buffer = (FAR uint8_t *)params;
      msg[1].length = num_params * sizeof(*params);
    }

  ret = scd41_do_transfer(priv->i2c, msg, (num_params) ? 2 : 1);

  scd41_dbg("cmd: 0x%04X num_params: %d ret: %d\n",
            cmd, num_params, ret);
  return (ret >= 0) ? OK : ret;
#else
  /* UART mode not implemented yet. */

  return -ENODEV;
#endif
}

/****************************************************************************
 * Name: scd41_read_words
 ****************************************************************************/

static int scd41_read_words(FAR struct scd41_dev_s *priv,
                            FAR struct scd41_word_s *words,
                            unsigned int num_words)
{
#ifdef CONFIG_SCD41_I2C
  struct i2c_msg_s msg[1];
  int ret;

  msg[0].frequency = CONFIG_SCD41_I2C_FREQUENCY;
  msg[0].addr = priv->addr;
  msg[0].flags = I2C_M_READ;
  msg[0].buffer = (FAR uint8_t *)words;
  msg[0].length = num_words * sizeof(*words);

  ret = scd41_do_transfer(priv->i2c, msg, 1);

  scd41_dbg("num_words: %d ret: %d\n", num_words, ret);
  return (ret >= 0) ? OK : ret;
#else
  /* UART mode not implemented yet. */

  return -ENODEV;
#endif
}

/****************************************************************************
 * Name: scd41_crc_word
 ****************************************************************************/

static uint8_t scd41_crc_word(uint16_t word)
{
  static const uint8_t crc_table[16] =
  {
    0x00, 0x31, 0x62, 0x53, 0xc4, 0xf5, 0xa6, 0x97,
    0xb9, 0x88, 0xdb, 0xea, 0x7d, 0x4c, 0x1f, 0x2e
  };

  uint8_t crc = 0xff;

  crc ^= word >> 8;
  crc = (crc << 4) ^ crc_table[crc >> 4];
  crc = (crc << 4) ^ crc_table[crc >> 4];
  crc ^= word >> 0;
  crc = (crc << 4) ^ crc_table[crc >> 4];
  crc = (crc << 4) ^ crc_table[crc >> 4];

  return crc;
}

/****************************************************************************
 * Name: scd41_set_command_param
 ****************************************************************************/

static void scd41_set_command_param(FAR struct scd41_word_s *param,
                                    uint16_t value)
{
  param->data[0] = value >> 8;
  param->data[1] = value >> 0;
  param->crc = scd41_crc_word(value);
}

/****************************************************************************
 * Name: scd41_data_word2uint16
 ****************************************************************************/

static uint16_t scd41_data_word2uint16(FAR const struct scd41_word_s *word)
{
  return (word[0].data[0] << 8) | (word[0].data[1]);
}

/****************************************************************************
 * Name: scd41_crc_word
 ****************************************************************************/

static int scd41_check_data_crc(FAR const struct scd41_word_s *words,
                                unsigned int num_words)
{
  while (num_words)
    {
      if (scd41_crc_word(scd41_data_word2uint16(words)) != words->crc)
        {
          return -1;
        }

      num_words--;
      words++;
    }

  return 0;
}

/****************************************************************************
 * Name: scd41_softreset
 *
 * Description:
 *   Reset the SCD41 sensor. This takes less than 2000 ms.
 *
 ****************************************************************************/

static int scd41_softreset(FAR struct scd41_dev_s *priv)
{
  int ret;

  ret = scd41_write_cmd(priv, SCD41_CMD_SOFT_RESET, NULL, 0);
  if (ret < 0)
    {
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Name: has_time_passed
 *
 * Description:
 *   Return true if curr >= start + secs_since_start
 *
 ****************************************************************************/

static bool has_time_passed(struct timespec curr,
                            struct timespec start,
                            unsigned int secs_since_start)
{
  if ((long)((start.tv_sec + secs_since_start) - curr.tv_sec) == 0)
    {
      return start.tv_nsec <= curr.tv_nsec;
    }

  return (long)((start.tv_sec + secs_since_start) - curr.tv_sec) <= 0;
}

/****************************************************************************
 * Name: scd41_read_values
 ****************************************************************************/

static int scd41_read_values(FAR struct scd41_dev_s *priv, FAR float *temp,
                             FAR float *rh, FAR float *co2, bool wait)
{
  struct scd41_word_s data[3];
  struct timespec ts;
  int ret;

  clock_systime_timespec(&ts);

  if (wait || !priv->valid ||
      has_time_passed(ts, priv->last_update,
                      SCD41_DEFAULT_MEASUREMENT_INTERVAL))
    {
      while (1)
        {
          /* Wait data to be ready. */

          ret = scd41_write_cmd(priv, SCD41_CMD_GET_DATA_READY, NULL, 0);
          if (ret < 0)
            {
              scd41_dbg("ERROR: scd41_write_cmd failed: %d\n", ret);
              return ret;
            }

          ret = scd41_read_words(priv, data, 1);
          if (ret < 0)
            {
              scd41_dbg("ERROR: scd41_read_words failed: %d\n", ret);
              return ret;
            }

          if (scd41_check_data_crc(data, 1) < 0)
            {
              scd41_dbg("ERROR: scd41_read_words crc failed\n");
              ret = -EIO;
              return ret;
            }

          if ((scd41_data_word2uint16(data) & 0x07ff) == 0x0000)
            {
              if (!wait)
                {
                  scd41_dbg("ERROR: data not ready\n");
                  ret = -EAGAIN;
                  return ret;
                }

              ret = nxsig_usleep(500 * 1000);
              if (ret == -EINTR)
                {
                  return ret;
                }
            }
          else
            {
              break;
            }
        }

      /* Read the raw data */

      ret = scd41_write_cmd(priv, SCD41_CMD_READ_MEASUREMENT, NULL, 0);
      if (ret < 0)
        {
          scd41_dbg("ERROR: scd41_write_cmd failed: %d\n", ret);
          return ret;
        }

      ret = scd41_read_words(priv, data, 3);
      if (ret < 0)
        {
          scd41_dbg("ERROR: scd41_read_words failed: %d\n", ret);
          return ret;
        }

      if (scd41_check_data_crc(data, 3) < 0)
        {
          scd41_dbg("ERROR: scd41_read_words crc failed\n");
          ret = -EIO;
          return ret;
        }

      add_sensor_randomness((data[0].crc << 16) ^ (data[1].crc << 8) ^
                            data[2].crc);

      priv->co2 = (float)scd41_data_word2uint16(data + 0);
      uint16_t tval = scd41_data_word2uint16(data + 1);
      priv->temperature = (float)((tval * 175.0) / 65536.0 - 45.0);
      uint16_t hval = scd41_data_word2uint16(data + 2);
      priv->humidity = (float)(hval * 100.0 / 65536.0);
      priv->last_update = ts;
      priv->valid = true;
    }

  *temp = priv->temperature;
  *rh = priv->humidity;
  *co2 = priv->co2;
  return OK;
}

/****************************************************************************
 * Name: scd41_configure
 ****************************************************************************/

static int scd41_configure(FAR struct scd41_dev_s *priv, bool start)
{
  struct scd41_word_s param;
  int ret = OK;

  /* Set altitude compensation. */

  scd41_set_command_param(&param, priv->altitude_comp);
  scd41_write_cmd(priv, SCD41_CMD_SET_ALT_COMPENSATION, &param, 1);

  /* Set temperature offset. */

  scd41_set_command_param(&param, priv->temperature_offset);
  scd41_write_cmd(priv, SCD41_CMD_SET_TEMP_OFFSET, &param, 1);

  /* Set pressure compensation. */

  scd41_set_command_param(&param, priv->pressure_comp);
  scd41_write_cmd(priv, SCD41_CMD_SET_PRESSURE_COMP, &param, 1);

  if (start)
    {
      /* Start measurements. */

      ret = scd41_write_cmd(priv, SCD41_CMD_START_MEASUREMENT, NULL, 0);
      if (ret >= 0)
        {
          priv->started = true;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: scd41_open
 *
 * Description:
 *   This function is called whenever the SCD41x device is opened.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int scd41_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct scd41_dev_s *priv = inode->i_private;
  int ret = OK;

  /* Get exclusive access */

  ret = nxsem_wait_uninterruptible(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Increment the count of open references on the driver */

  priv->crefs++;
  DEBUGASSERT(priv->crefs > 0);

  if (priv->crefs == 1)
    {
      ret = scd41_configure(priv, true);
    }

  if (ret < 0)
    {
      priv->crefs--;
    }

  nxsem_post(&priv->devsem);
  return ret;
}
#endif

/****************************************************************************
 * Name: scd41_close
 *
 * Description:
 *   This routine is called when the SCD41 device is closed.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int scd41_close(FAR struct file *filep)
{
  FAR struct inode       *inode = filep->f_inode;
  FAR struct scd41_dev_s *priv  = inode->i_private;
  int ret;

  /* Get exclusive access */

  ret = nxsem_wait_uninterruptible(&priv->devsem);
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
      nxsem_destroy(&priv->devsem);
      kmm_free(priv);
      return OK;
    }

  nxsem_post(&priv->devsem);
  return OK;
}
#endif

/****************************************************************************
 * Name: scd41_read
 ****************************************************************************/

static ssize_t scd41_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct scd41_dev_s *priv = inode->i_private;
  ssize_t length = 0;
  float temp;
  float rh;
  float co2;
  unsigned int temp100;
  unsigned int rh100;
  unsigned int co2_100;
  int ret;

  /* Get exclusive access */

  ret = nxsem_wait_uninterruptible(&priv->devsem);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (priv->unlinked)
    {
      /* Do not allow operations on unlinked sensors. This allows
       * sensor use on hot swappable I2C bus.
       */

      nxsem_post(&priv->devsem);
      return -ENODEV;
    }
#endif

  if (!priv->started)
    {
      return -ENODATA;
    }

  ret = scd41_read_values(priv, &temp, &rh, &co2,
                          !(filep->f_oflags & O_NONBLOCK));
  if (ret < 0)
    {
      scd41_dbg("cannot read data: %d\n", ret);
    }
  else
    {
      /* This interface is mainly intended for easy debugging in nsh. */

      temp100 = abs(temp * 100);
      rh100 = abs(rh * 100);
      co2_100 = abs(co2 * 100);

      length = snprintf(buffer, buflen, "%s%u.%02u %s%u.%02u %s%u.%02u\n",
                        co2 < 0 ? "-" : "", co2_100 / 100, co2_100 % 100,
                        temp < 0 ? "-" : "", temp100 / 100, temp100 % 100,
                        rh < 0 ? "-" : "", rh100 / 100, rh100 % 100);
      if (length > buflen)
        {
          length = buflen;
        }
    }

  nxsem_post(&priv->devsem);
  return length;
}

/****************************************************************************
 * Name: scd41_write
 ****************************************************************************/

static ssize_t scd41_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: scd41_ioctl
 ****************************************************************************/

static int scd41_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct scd41_dev_s *priv = inode->i_private;
  struct scd41_word_s param;
  int ret;

  /* Get exclusive access */

  ret = nxsem_wait_uninterruptible(&priv->devsem);
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

      nxsem_post(&priv->devsem);
      return -ENODEV;
    }
#endif

  switch (cmd)
    {
      /* Soft reset the SCD41, Arg: None */

      case SNIOC_RESET:
        {
          priv->interval = SCD41_DEFAULT_MEASUREMENT_INTERVAL;
          priv->pressure_comp = SCD41_DEFAULT_PRESSURE_COMPENSATION;
          priv->altitude_comp = SCD41_DEFAULT_ALTITUDE_COMPENSATION;
          priv->temperature_offset = SCD41_DEFAULT_TEMPERATURE_OFFSET;

          ret = scd41_softreset(priv);
          scd41_dbg("softreset ret: %d\n", ret);

          scd41_configure(priv, priv->started);
        }
        break;

      /* Start background measurement, Arg: None */

      case SNIOC_START:
        {
          /* Start measurements. */

          ret = scd41_write_cmd(priv, SCD41_CMD_START_MEASUREMENT, NULL, 0);
          if (ret >= 0)
            {
              priv->started = true;
            }
        }
        break;

      /* Stop background measurement, Arg: None */

      case SNIOC_STOP:
        {
          /* Stop measurements. */

          ret = scd41_write_cmd(priv, SCD41_CMD_STOP_MEASUREMENT, NULL, 0);
          if (ret >= 0)
            {
              priv->started = false;
            }
        }
        break;

      /* Set temperature offset value, Arg: uint16_t */

      case SNIOC_SET_TEMP_OFFSET:
        {
          if (arg > UINT16_MAX)
            {
              ret = -EINVAL;
              break;
            }

          /* Actual Toffset = temperature_offset * 175.0 / 2^16 */

          priv->temperature_offset = arg;
          scd41_set_command_param(&param, priv->temperature_offset);
          ret = scd41_write_cmd(priv, SCD41_CMD_SET_TEMP_OFFSET, &param, 1);
        }
        break;

      /* Set pressure compensation value, Arg: uint16_t */

      case SNIOC_SET_PRESSURE_COMP:
        {
          if (arg > UINT16_MAX)
            {
              ret = -EINVAL;
              break;
            }

          priv->pressure_comp = arg;
          scd41_set_command_param(&param, priv->pressure_comp);
          ret = scd41_write_cmd(priv, SCD41_CMD_SET_PRESSURE_COMP,
                                &param, 1);
        }
        break;

      /* Set altitude compensation value, Arg: uint16_t */

      case SNIOC_SET_ALTITUDE_COMP:
        {
          if (arg > UINT16_MAX)
            {
              ret = -EINVAL;
              break;
            }

          priv->altitude_comp = arg;
          scd41_set_command_param(&param, priv->altitude_comp);
          ret = scd41_write_cmd(priv, SCD41_CMD_SET_ALT_COMPENSATION,
                                &param, 1);
        }
        break;

      /* Set Forced Recalibration value (FRC), Arg: uint16_t */

      case SNIOC_SET_FRC:
        {
          if (arg > UINT16_MAX)
            {
              ret = -EINVAL;
              break;
            }

          scd41_set_command_param(&param, arg);
          ret = scd41_write_cmd(priv, SCD41_CMD_SET_FRC, &param, 1);
        }
        break;

      /* (De-)Activate Automatic Self-Calibration (ASC), Arg: bool */

      case SNIOC_ENABLE_ASC:
        {
          if (arg != !!arg) /* 0 or 1 */
            {
              ret = -EINVAL;
              break;
            }

          scd41_set_command_param(&param, arg);
          ret = scd41_write_cmd(priv, SCD41_CMD_SET_ASC, &param, 1);
        }
        break;

      /* Read sensor data, Arg: struct scd41_conv_data_s *data */

      case SNIOC_READ_CONVERT_DATA:
        {
          float temp;
          float rh;
          float co2;

          ret = scd41_read_values(priv, &temp, &rh, &co2, false);
          if (ret < 0)
            {
              scd41_dbg("cannot read data: %d\n", ret);
            }
          else
            {
              FAR struct scd41_conv_data_s *data =
                (FAR struct scd41_conv_data_s *)arg;

              data->temperature = temp;
              data->humidity = rh;
              data->co2 = co2;
            }
        }
        break;

      default:
        scd41_dbg("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  nxsem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: scd41_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int scd41_unlink(FAR struct inode *inode)
{
  FAR struct scd41_dev_s *priv;
  int ret;

  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv = (FAR struct scd41_dev_s *)inode->i_private;

  /* Get exclusive access */

  ret = nxsem_wait_uninterruptible(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Are there open references to the driver data structure? */

  if (priv->crefs <= 0)
    {
      nxsem_destroy(&priv->devsem);
      kmm_free(priv);
      return OK;
    }

  /* No... just mark the driver as unlinked and free the resources when
   * the last client closes their reference to the driver.
   */

  priv->unlinked = true;
  nxsem_post(&priv->devsem);
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_SCD41_I2C
/****************************************************************************
 * Name: scd41_register_i2c
 *
 * Description:
 *   Register the SCD41 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/co2_0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the SCD41
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int scd41_register_i2c(FAR const char *devpath, FAR struct i2c_master_s *i2c)
{
  FAR struct scd41_dev_s *priv;
  int ret;

  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(scd41_crc_word(0xbeef) == 0x92);

  /* Initialize the device structure */

  priv = (FAR struct scd41_dev_s *)kmm_zalloc(sizeof(struct scd41_dev_s));
  if (priv == NULL)
    {
      scd41_dbg("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = CONFIG_SCD41_ADDR;
  priv->started = false;
  priv->interval = SCD41_DEFAULT_MEASUREMENT_INTERVAL;
  priv->pressure_comp = SCD41_DEFAULT_PRESSURE_COMPENSATION;
  priv->altitude_comp = SCD41_DEFAULT_ALTITUDE_COMPENSATION;
  priv->temperature_offset = SCD41_DEFAULT_TEMPERATURE_OFFSET;

  nxsem_init(&priv->devsem, 0, 1);

  /* Register the character driver */

  ret = register_driver(devpath, &g_scd41fops, 0666, priv);
  if (ret < 0)
    {
      scd41_dbg("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}
#endif /* CONFIG_SCD41_I2C */

#endif /* CONFIG_SENSORS_SCD41 */
