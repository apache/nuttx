/****************************************************************************
 * drivers/sensors/scd30.c
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
#include <nuttx/sensors/scd30.h>
#include <nuttx/random.h>

#if defined(CONFIG_SENSORS_SCD30)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SCD30_DEBUG
#  define scd30_dbg(x, ...)    _info(x, ##__VA_ARGS__)
#else
#  define scd30_dbg(x, ...)    sninfo(x, ##__VA_ARGS__)
#endif

#ifndef CONFIG_SCD30_I2C_FREQUENCY
#  define CONFIG_SCD30_I2C_FREQUENCY 100000
#endif

#define SCD30_I2C_RETRIES 3

/* SCD30 command words */

#define SCD30_CMD_START_MEASUREMENT     0x0010
#define SCD30_CMD_STOP_MEASUREMENT      0x0104
#define SCD30_CMD_SET_INTERVAL          0x4600
#define SCD30_CMD_GET_DATA_READY        0x0202
#define SCD30_CMD_READ_MEASUREMENT      0x0300
#define SCD30_CMD_SET_ASC               0x5306
#define SCD30_CMD_SET_FRC               0x5204
#define SCD30_CMD_SET_TEMP_OFFSET       0x5403
#define SCD30_CMD_SET_ALT_COMPENSATION  0x5102
#define SCD30_CMD_SOFT_RESET            0xd304

#define SCD30_DEFAULT_MEASUREMENT_INTERVAL  2 /* seconds */
#define SCD30_DEFAULT_PRESSURE_COMPENSATION 0
#define SCD30_DEFAULT_ALTITUDE_COMPENSATION 0
#define SCD30_DEFAULT_TEMPERATURE_OFFSET    0

/****************************************************************************
 * Private
 ****************************************************************************/

struct scd30_dev_s
{
#ifdef CONFIG_SCD30_I2C
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

struct scd30_word_s
{
  uint8_t data[2];
  uint8_t crc;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IO Helpers */

#ifdef CONFIG_SCD30_I2C
static int scd30_do_transfer(FAR struct i2c_master_s *i2c,
                             FAR struct i2c_msg_s *msgv,
                             size_t nmsg);
#endif
static int scd30_write_cmd(FAR struct scd30_dev_s *priv, uint16_t cmd,
                           FAR struct scd30_word_s *params,
                           unsigned int num_params);
static int scd30_read_words(FAR struct scd30_dev_s *priv,
                            FAR struct scd30_word_s *words,
                            unsigned int num_words);

/* Data conversion */

static uint8_t scd30_crc_word(uint16_t word);
static void scd30_set_command_param(FAR struct scd30_word_s *param,
                                    uint16_t value);
static int scd30_check_data_crc(FAR const struct scd30_word_s *words,
                                unsigned int num_words);
static uint16_t scd30_data_word2uint16(FAR const struct scd30_word_s *word);
static float scd30_data_words_to_float(
                                FAR const struct scd30_word_s words[2]);

/* Driver features */

static int scd30_read_values(FAR struct scd30_dev_s *priv, FAR float *temp,
                             FAR float *rh, FAR float *co2, bool wait);
static int scd30_configure(FAR struct scd30_dev_s *priv, bool start);

/* Character driver methods */

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     scd30_open(FAR struct file *filep);
static int     scd30_close(FAR struct file *filep);
#endif
static ssize_t scd30_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t scd30_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int     scd30_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     scd30_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_scd30fops =
{
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  scd30_open,     /* open */
  scd30_close,    /* close */
#else
  NULL,           /* open */
  NULL,           /* close */
#endif
  scd30_read,     /* read */
  scd30_write,    /* write */
  NULL,           /* seek */
  scd30_ioctl,    /* ioctl */
  NULL            /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , scd30_unlink /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: scd30_do_transfer
 ****************************************************************************/

#ifdef CONFIG_SCD30_I2C
static int scd30_do_transfer(FAR struct i2c_master_s *i2c,
                             FAR struct i2c_msg_s *msgv,
                             size_t nmsg)
{
  int ret = -EIO;
  int retries;

  for (retries = 0; retries < SCD30_I2C_RETRIES; retries++)
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
          if (retries == SCD30_I2C_RETRIES - 1)
            {
              break;
            }

          ret = I2C_RESET(i2c);
          if (ret < 0)
            {
              scd30_dbg("I2C_RESET failed: %d\n", ret);
              return ret;
            }
#endif
        }
    }

  scd30_dbg("xfer failed: %d\n", ret);
  return ret;
}
#endif

/****************************************************************************
 * Name: scd30_write_cmd
 ****************************************************************************/

static int scd30_write_cmd(FAR struct scd30_dev_s *priv, uint16_t cmd,
                           FAR struct scd30_word_s *params,
                           unsigned int num_params)
{
#ifdef CONFIG_SCD30_I2C
  struct i2c_msg_s msg[2];
  uint8_t cmd_buf[2];
  int ret;

  cmd_buf[0] = cmd >> 8;
  cmd_buf[1] = cmd >> 0;

  msg[0].frequency = CONFIG_SCD30_I2C_FREQUENCY;
  msg[0].addr = priv->addr;
  msg[0].flags = 0;
  msg[0].buffer = cmd_buf;
  msg[0].length = 2;

  if (num_params)
    {
      msg[1].frequency = CONFIG_SCD30_I2C_FREQUENCY;
      msg[1].addr = priv->addr;
      msg[1].flags = I2C_M_NOSTART;
      msg[1].buffer = (FAR uint8_t *)params;
      msg[1].length = num_params * sizeof(*params);
    }

  ret = scd30_do_transfer(priv->i2c, msg, (num_params) ? 2 : 1);

  scd30_dbg("cmd: 0x%04X num_params: %d ret: %d\n",
            cmd, num_params, ret);
  return (ret >= 0) ? OK : ret;
#else
  /* UART mode not implemented yet. */

  return -ENODEV;
#endif
}

/****************************************************************************
 * Name: scd30_read_words
 ****************************************************************************/

static int scd30_read_words(FAR struct scd30_dev_s *priv,
                            FAR struct scd30_word_s *words,
                            unsigned int num_words)
{
#ifdef CONFIG_SCD30_I2C
  struct i2c_msg_s msg[1];
  int ret;

  msg[0].frequency = CONFIG_SCD30_I2C_FREQUENCY;
  msg[0].addr = priv->addr;
  msg[0].flags = I2C_M_READ;
  msg[0].buffer = (FAR uint8_t *)words;
  msg[0].length = num_words * sizeof(*words);

  ret = scd30_do_transfer(priv->i2c, msg, 1);

  scd30_dbg("num_words: %d ret: %d\n", num_words, ret);
  return (ret >= 0) ? OK : ret;
#else
  /* UART mode not implemented yet. */

  return -ENODEV;
#endif
}

/****************************************************************************
 * Name: scd30_crc_word
 ****************************************************************************/

static uint8_t scd30_crc_word(uint16_t word)
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
 * Name: scd30_set_command_param
 ****************************************************************************/

static void scd30_set_command_param(FAR struct scd30_word_s *param,
                                    uint16_t value)
{
  param->data[0] = value >> 8;
  param->data[1] = value >> 0;
  param->crc = scd30_crc_word(value);
}

/****************************************************************************
 * Name: scd30_data_words_to_float
 ****************************************************************************/

static float scd30_data_words_to_float(
  FAR const struct scd30_word_s words[2])
{
  uint8_t data[4];
  float value;

  data[3] = words[0].data[0];
  data[2] = words[0].data[1];
  data[1] = words[1].data[0];
  data[0] = words[1].data[1];
  memcpy(&value, data, 4);
  return value;
}

/****************************************************************************
 * Name: scd30_data_word2uint16
 ****************************************************************************/

static uint16_t scd30_data_word2uint16(FAR const struct scd30_word_s *word)
{
  return (word[0].data[0] << 8) | (word[0].data[1]);
}

/****************************************************************************
 * Name: scd30_crc_word
 ****************************************************************************/

static int scd30_check_data_crc(FAR const struct scd30_word_s *words,
                                unsigned int num_words)
{
  while (num_words)
    {
      if (scd30_crc_word(scd30_data_word2uint16(words)) != words->crc)
        {
          return -1;
        }

      num_words--;
      words++;
    }

  return 0;
}

/****************************************************************************
 * Name: scd30_softreset
 *
 * Description:
 *   Reset the SCD30 sensor. This takes less than 2000 ms.
 *
 ****************************************************************************/

static int scd30_softreset(FAR struct scd30_dev_s *priv)
{
  int ret;

  ret = scd30_write_cmd(priv, SCD30_CMD_SOFT_RESET, NULL, 0);
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
 * Name: scd30_read_values
 ****************************************************************************/

static int scd30_read_values(FAR struct scd30_dev_s *priv, FAR float *temp,
                             FAR float *rh, FAR float *co2, bool wait)
{
  struct scd30_word_s data[6];
  struct timespec ts;
  int ret;

  clock_systime_timespec(&ts);

  if (wait || !priv->valid ||
      has_time_passed(ts, priv->last_update,
                      SCD30_DEFAULT_MEASUREMENT_INTERVAL))
    {
      while (1)
        {
          /* Wait data to be ready. */

          ret = scd30_write_cmd(priv, SCD30_CMD_GET_DATA_READY, NULL, 0);
          if (ret < 0)
            {
              scd30_dbg("ERROR: scd30_write_cmd failed: %d\n", ret);
              return ret;
            }

          ret = scd30_read_words(priv, data, 1);
          if (ret < 0)
            {
              scd30_dbg("ERROR: scd30_read_words failed: %d\n", ret);
              return ret;
            }

          if (scd30_check_data_crc(data, 1) < 0)
            {
              scd30_dbg("ERROR: scd30_read_words crc failed\n");
              ret = -EIO;
              return ret;
            }

          if (scd30_data_word2uint16(data) != 0x0001)
            {
              if (!wait)
                {
                  scd30_dbg("ERROR: data not ready\n");
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

      ret = scd30_write_cmd(priv, SCD30_CMD_READ_MEASUREMENT, NULL, 0);
      if (ret < 0)
        {
          scd30_dbg("ERROR: scd30_write_cmd failed: %d\n", ret);
          return ret;
        }

      ret = scd30_read_words(priv, data, 6);
      if (ret < 0)
        {
          scd30_dbg("ERROR: scd30_read_words failed: %d\n", ret);
          return ret;
        }

      if (scd30_check_data_crc(data, 6) < 0)
        {
          scd30_dbg("ERROR: scd30_read_words crc failed\n");
          ret = -EIO;
          return ret;
        }

      add_sensor_randomness(((data[0].crc ^ data[1].crc) << 0) ^
                            ((data[2].crc ^ data[3].crc) << 8) ^
                            ((data[4].crc ^ data[5].crc) << 16));

      priv->co2 = scd30_data_words_to_float(data + 0);
      priv->temperature = scd30_data_words_to_float(data + 2);
      priv->humidity = scd30_data_words_to_float(data + 4);
      priv->last_update = ts;
      priv->valid = true;
    }

  *temp = priv->temperature;
  *rh = priv->humidity;
  *co2 = priv->co2;
  return OK;
}

/****************************************************************************
 * Name: scd30_configure
 ****************************************************************************/

static int scd30_configure(FAR struct scd30_dev_s *priv, bool start)
{
  struct scd30_word_s param;
  int ret;

  /* Set measurement interval. */

  scd30_set_command_param(&param, priv->interval);
  scd30_write_cmd(priv, SCD30_CMD_SET_INTERVAL, &param, 1);

  /* Set altitude compensation. */

  scd30_set_command_param(&param, priv->altitude_comp);
  scd30_write_cmd(priv, SCD30_CMD_SET_ALT_COMPENSATION, &param, 1);

  /* Set temperature offset. */

  scd30_set_command_param(&param, priv->temperature_offset);
  scd30_write_cmd(priv, SCD30_CMD_SET_TEMP_OFFSET, &param, 1);

  if (!start)
    {
      /* Stop measurements. */

      ret = scd30_write_cmd(priv, SCD30_CMD_STOP_MEASUREMENT, &param, 1);
      if (ret >= 0)
        {
          priv->started = false;
        }
    }
  else
    {
      /* Start measurements (and set pressure compensation). */

      scd30_set_command_param(&param, priv->pressure_comp);
      ret = scd30_write_cmd(priv, SCD30_CMD_START_MEASUREMENT, &param, 1);
      if (ret >= 0)
        {
          priv->started = true;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: scd30_open
 *
 * Description:
 *   This function is called whenever the SCD30x device is opened.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int scd30_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct scd30_dev_s *priv = inode->i_private;
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
      ret = scd30_configure(priv, true);
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
 * Name: scd30_close
 *
 * Description:
 *   This routine is called when the SCD30 device is closed.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int scd30_close(FAR struct file *filep)
{
  FAR struct inode       *inode = filep->f_inode;
  FAR struct scd30_dev_s *priv  = inode->i_private;
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
 * Name: scd30_read
 ****************************************************************************/

static ssize_t scd30_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct scd30_dev_s *priv = inode->i_private;
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

  ret = scd30_read_values(priv, &temp, &rh, &co2,
                          !(filep->f_oflags & O_NONBLOCK));
  if (ret < 0)
    {
      scd30_dbg("cannot read data: %d\n", ret);
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
 * Name: scd30_write
 ****************************************************************************/

static ssize_t scd30_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: scd30_ioctl
 ****************************************************************************/

static int scd30_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct scd30_dev_s *priv = inode->i_private;
  struct scd30_word_s param;
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
      /* Soft reset the SCD30, Arg: None */

      case SNIOC_RESET:
        {
          priv->interval = SCD30_DEFAULT_MEASUREMENT_INTERVAL;
          priv->pressure_comp = SCD30_DEFAULT_PRESSURE_COMPENSATION;
          priv->altitude_comp = SCD30_DEFAULT_ALTITUDE_COMPENSATION;
          priv->temperature_offset = SCD30_DEFAULT_TEMPERATURE_OFFSET;

          ret = scd30_softreset(priv);
          scd30_dbg("softreset ret: %d\n", ret);

          scd30_configure(priv, priv->started);
        }
        break;

      /* Start background measurement, Arg: None */

      case SNIOC_START:
        {
          /* Start measurements (and set pressure compensation). */

          scd30_set_command_param(&param, priv->pressure_comp);
          ret = scd30_write_cmd(priv, SCD30_CMD_START_MEASUREMENT,
                                &param, 1);
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

          ret = scd30_write_cmd(priv, SCD30_CMD_STOP_MEASUREMENT, NULL, 0);
          if (ret >= 0)
            {
              priv->started = false;
            }
        }
        break;

      /* Set background measurement interval, Arg: uint16_t */

      case SNIOC_SET_INTERVAL:
        {
          if (arg < 2 && arg > 1800)
            {
              ret = -EINVAL;
              break;
            }

          priv->interval = arg;
          scd30_set_command_param(&param, priv->interval);
          ret = scd30_write_cmd(priv, SCD30_CMD_SET_INTERVAL, &param, 1);
        }
        break;

      /* Set temperature offset value, Arg: uint16_t */

      case SNIOC_SET_TEMP_OFFSET:
        {
          if (arg < 0 && arg > UINT16_MAX)
            {
              ret = -EINVAL;
              break;
            }

          priv->temperature_offset = arg;
          scd30_set_command_param(&param, priv->temperature_offset);
          ret = scd30_write_cmd(priv, SCD30_CMD_SET_TEMP_OFFSET, &param, 1);
        }
        break;

      /* Set pressure compensation value, Arg: uint16_t */

      case SNIOC_SET_PRESSURE_COMP:
        {
          if (arg != 0 && arg < 700 && arg > 1200)
            {
              ret = -EINVAL;
              break;
            }

          priv->pressure_comp = arg;
          if (priv->started)
            {
              scd30_set_command_param(&param, priv->pressure_comp);
              ret = scd30_write_cmd(priv, SCD30_CMD_START_MEASUREMENT,
                                    &param, 1);
            }
          else
            {
              ret = 0;
            }
        }
        break;

      /* Set altitude compensation value, Arg: uint16_t */

      case SNIOC_SET_ALTITUDE_COMP:
        {
          if (arg < 0 && arg > UINT16_MAX)
            {
              ret = -EINVAL;
              break;
            }

          priv->altitude_comp = arg;
          scd30_set_command_param(&param, priv->altitude_comp);
          ret = scd30_write_cmd(priv, SCD30_CMD_SET_ALT_COMPENSATION,
                                &param, 1);
        }
        break;

      /* Set Forced Recalibration value (FRC), Arg: uint16_t */

      case SNIOC_SET_FRC:
        {
          if (arg < 0 && arg > UINT16_MAX)
            {
              ret = -EINVAL;
              break;
            }

          scd30_set_command_param(&param, arg);
          ret = scd30_write_cmd(priv, SCD30_CMD_SET_FRC, &param, 1);
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

          scd30_set_command_param(&param, arg);
          ret = scd30_write_cmd(priv, SCD30_CMD_SET_ASC, &param, 1);
        }
        break;

      /* Read sensor data, Arg: struct scd30_conv_data_s *data */

      case SNIOC_READ_CONVERT_DATA:
        {
          float temp;
          float rh;
          float co2;

          ret = scd30_read_values(priv, &temp, &rh, &co2, false);
          if (ret < 0)
            {
              scd30_dbg("cannot read data: %d\n", ret);
            }
          else
            {
              FAR struct scd30_conv_data_s *data =
                (FAR struct scd30_conv_data_s *)arg;

              data->temperature = temp;
              data->humidity = rh;
              data->co2 = co2;
            }
        }
        break;

      default:
        scd30_dbg("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  nxsem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: scd30_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int scd30_unlink(FAR struct inode *inode)
{
  FAR struct scd30_dev_s *priv;
  int ret;

  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv = (FAR struct scd30_dev_s *)inode->i_private;

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

#ifdef CONFIG_SCD30_I2C
/****************************************************************************
 * Name: scd30_register_i2c
 *
 * Description:
 *   Register the SCD30 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/co2_0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the SCD30
 *   addr    - The I2C address of the SCD30. The I2C address of SCD30 is
 *             always 0x61.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int scd30_register_i2c(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                       uint8_t addr)
{
  FAR struct scd30_dev_s *priv;
  int ret;

  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(addr == CONFIG_SCD30_ADDR);
  DEBUGASSERT(scd30_crc_word(0xbeef) == 0x92);

  /* Initialize the device structure */

  priv = (FAR struct scd30_dev_s *)kmm_zalloc(sizeof(struct scd30_dev_s));
  if (priv == NULL)
    {
      scd30_dbg("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;
  priv->started = false;
  priv->interval = SCD30_DEFAULT_MEASUREMENT_INTERVAL;
  priv->pressure_comp = SCD30_DEFAULT_PRESSURE_COMPENSATION;
  priv->altitude_comp = SCD30_DEFAULT_ALTITUDE_COMPENSATION;
  priv->temperature_offset = SCD30_DEFAULT_TEMPERATURE_OFFSET;

  nxsem_init(&priv->devsem, 0, 1);

  /* Register the character driver */

  ret = register_driver(devpath, &g_scd30fops, 0666, priv);
  if (ret < 0)
    {
      scd30_dbg("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}
#endif /* CONFIG_SCD30_I2C */

#endif /* CONFIG_SENSORS_SCD30 */
