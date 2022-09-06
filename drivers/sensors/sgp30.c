/****************************************************************************
 * drivers/sensors/sgp30.c
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
#include <nuttx/sensors/sgp30.h>
#include <nuttx/mutex.h>
#include <nuttx/random.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_SGP30)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SGP30_DEBUG
#  define sgp30_dbg(x, ...)    _info(x, ##__VA_ARGS__)
#else
#  define sgp30_dbg(x, ...)    sninfo(x, ##__VA_ARGS__)
#endif

#ifndef CONFIG_SGP30_I2C_FREQUENCY
#  define CONFIG_SGP30_I2C_FREQUENCY 100000
#endif

#define SGP30_I2C_RETRIES 3
#define SGP30_INIT_RETRIES 5
#define SGP30_INIT_LIMIT_MS 10

/****************************************************************************
 * Private
 ****************************************************************************/

struct sgp30_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  bool unlinked;                /* True, driver has been unlinked */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  int16_t crefs;                /* Number of open references */
#endif
  struct timespec last_update;
  mutex_t devlock;
};

struct sgp30_word_s
{
  uint8_t data[2];
  uint8_t crc;
};

struct sgp30_cmd_s
{
  uint16_t address;
  unsigned int read_delay_usec;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IO Helpers */

static int sgp30_do_transfer(FAR struct i2c_master_s *i2c,
                             FAR struct i2c_msg_s *msgv,
                             size_t nmsg);
static int sgp30_write_cmd(FAR struct sgp30_dev_s *priv,
                           struct sgp30_cmd_s cmd,
                           FAR struct sgp30_word_s *params,
                           unsigned int num_params);
static int sgp30_read_cmd(FAR struct sgp30_dev_s *priv,
                          struct sgp30_cmd_s cmd,
                          FAR struct sgp30_word_s *words,
                          unsigned int num_words);

/* Data conversion */

static uint8_t sgp30_crc_word(uint16_t word);
static void sgp30_set_command_param(FAR struct sgp30_word_s *param,
                                    uint16_t value);
static int sgp30_check_data_crc(FAR const struct sgp30_word_s *words,
                                unsigned int num_words);
static uint16_t sgp30_data_word_to_uint16(
                                FAR const struct sgp30_word_s *word);

/* Driver features */

static int sgp30_measure_airq(FAR struct sgp30_dev_s *priv,
                              FAR struct sgp30_conv_data_s *data);
static int sgp30_measure_raw(FAR struct sgp30_dev_s *priv,
                             FAR struct sgp30_raw_data_s *data);

/* Character driver methods */

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int sgp30_open(FAR struct file *filep);
static int sgp30_close(FAR struct file *filep);
#endif
static ssize_t sgp30_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t sgp30_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int sgp30_ioctl(FAR struct file *filep, int cmd,
                       unsigned long arg);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int sgp30_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_sgp30fops =
{
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  sgp30_open,     /* open */
  sgp30_close,    /* close */
#else
  NULL,           /* open */
  NULL,           /* close */
#endif
  sgp30_read,     /* read */
  sgp30_write,    /* write */
  NULL,           /* seek */
  sgp30_ioctl,    /* ioctl */
  NULL            /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , sgp30_unlink /* unlink */
#endif
};

/* SGP30 command words */

static const struct sgp30_cmd_s SGP30_CMD_INIT_AIR_QUALITY =
{
  0x2003, 0
};

static const struct sgp30_cmd_s SGP30_CMD_MEASURE_AIR_QUALITY =
{
  0x2008, 12000
};

static const struct sgp30_cmd_s SGP30_CMD_GET_BASELINE =
{
  0x2015, 10000
};

static const struct sgp30_cmd_s SGP30_CMD_SET_BASELINE =
{
  0x201e, 10000
};

static const struct sgp30_cmd_s SGP30_CMD_SET_HUMIDITY =
{
  0x2061, 10000
};

static const struct sgp30_cmd_s SGP30_CMD_MEASURE_TEST =
{
  0x2032, 220000
};

static const struct sgp30_cmd_s SGP30_CMD_GET_FEATURE_SET_VERSION =
{
  0x202f, 2000
};

static const struct sgp30_cmd_s SGP30_CMD_MEASURE_RAW_SIGNALS =
{
  0x2050, 25000
};

static const struct sgp30_cmd_s SGP30_CMD_GET_SERIAL_ID =
{
  0x3682, 500
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sgp30_do_transfer
 ****************************************************************************/

static int sgp30_do_transfer(FAR struct i2c_master_s *i2c,
                             FAR struct i2c_msg_s *msgv,
                             size_t nmsg)
{
  int ret = -EIO;
  int retries;

  for (retries = 0; retries < SGP30_I2C_RETRIES; retries++)
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
          if (retries == SGP30_I2C_RETRIES - 1)
            {
              break;
            }

          ret = I2C_RESET(i2c);
          if (ret < 0)
            {
              sgp30_dbg("I2C_RESET failed: %d\n", ret);
              return ret;
            }
#endif
        }
    }

  sgp30_dbg("xfer failed: %d\n", ret);
  return ret;
}

/****************************************************************************
 * Name: sgp30_write_cmd
 ****************************************************************************/

static int sgp30_write_cmd(FAR struct sgp30_dev_s *priv,
                           struct sgp30_cmd_s cmd,
                           FAR struct sgp30_word_s *params,
                           unsigned int num_params)
{
  struct i2c_msg_s msg[2];
  uint8_t addr_buf[2];
  int ret;

  addr_buf[0] = cmd.address >> 8;
  addr_buf[1] = cmd.address >> 0;

  msg[0].frequency = CONFIG_SGP30_I2C_FREQUENCY;
  msg[0].addr = priv->addr;
  msg[0].flags = 0;
  msg[0].buffer = addr_buf;
  msg[0].length = 2;

  if (num_params)
    {
      msg[1].frequency = CONFIG_SGP30_I2C_FREQUENCY;
      msg[1].addr = priv->addr;
      msg[1].flags = I2C_M_NOSTART;
      msg[1].buffer = (FAR uint8_t *)params;
      msg[1].length = num_params * sizeof(*params);
    }

  ret = sgp30_do_transfer(priv->i2c, msg, (num_params) ? 2 : 1);

  sgp30_dbg("cmd: 0x%04X num_params: %d ret: %d\n",
            cmd.address, num_params, ret);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: sgp30_read_cmd
 ****************************************************************************/

static int sgp30_read_cmd(FAR struct sgp30_dev_s *priv,
                          struct sgp30_cmd_s cmd,
                          FAR struct sgp30_word_s *words,
                          unsigned int num_words)
{
  struct i2c_msg_s msg[1];
  uint8_t addr_buf[2];
  int ret;

  addr_buf[0] = cmd.address >> 8;
  addr_buf[1] = cmd.address >> 0;

  msg[0].frequency = CONFIG_SGP30_I2C_FREQUENCY;
  msg[0].addr = priv->addr;
  msg[0].flags = 0;
  msg[0].buffer = addr_buf;
  msg[0].length = 2;

  ret = sgp30_do_transfer(priv->i2c, msg, 1);

  sgp30_dbg("cmd: 0x%04X delay: %uus ret: %d\n", cmd.address,
            cmd.read_delay_usec, ret);

  if (ret < 0)
    {
      return ret;
    }

  up_udelay(cmd.read_delay_usec + 100);

  msg[0].frequency = CONFIG_SGP30_I2C_FREQUENCY;
  msg[0].addr = priv->addr;
  msg[0].flags = I2C_M_READ;
  msg[0].buffer = (FAR uint8_t *)words;
  msg[0].length = num_words * sizeof(*words);

  ret = sgp30_do_transfer(priv->i2c, msg, 1);

  sgp30_dbg("read cmd: 0x%04X num_params: %d ret: %d\n",
            cmd.address, num_words, ret);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: sgp30_crc_word
 ****************************************************************************/

static uint8_t sgp30_crc_word(uint16_t word)
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
 * Name: sgp30_soft_reset
 ****************************************************************************/

static int sgp30_soft_reset(FAR struct sgp30_dev_s *priv)
{
  struct i2c_msg_s msg[1];
  uint8_t buf[1];
  int ret = 0;

  buf[0] = CONFIG_SGP30_RESET_SECOND_BYTE;

  msg[0].frequency = CONFIG_SGP30_I2C_FREQUENCY;
  msg[0].addr = CONFIG_SGP30_RESET_ADDR;
  msg[0].flags = 0;
  msg[0].buffer = buf;
  msg[0].length = 1;

  ret = sgp30_do_transfer(priv->i2c, msg, 1);

  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: sgp30_set_command_param
 ****************************************************************************/

static void sgp30_set_command_param(FAR struct sgp30_word_s *param,
                                    uint16_t value)
{
  param->data[0] = value >> 8;
  param->data[1] = value >> 0;
  param->crc = sgp30_crc_word(value);
}

/****************************************************************************
 * Name: sgp30_data_word_to_uint16
 ****************************************************************************/

static uint16_t sgp30_data_word_to_uint16(
    FAR const struct sgp30_word_s *word)
{
  return (word[0].data[0] << 8) | (word[0].data[1]);
}

/****************************************************************************
 * Name: sgp30_crc_word
 ****************************************************************************/

static int sgp30_check_data_crc(FAR const struct sgp30_word_s *words,
                                unsigned int num_words)
{
  while (num_words)
    {
      if (sgp30_crc_word(sgp30_data_word_to_uint16(words)) != words->crc)
        {
          return -1;
        }

      num_words--;
      words++;
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

static bool has_time_passed(FAR struct timespec *curr,
                            FAR struct timespec *start,
                            unsigned int secs_since_start)
{
  if ((long)((start->tv_sec + secs_since_start) - curr->tv_sec) == 0)
    {
      return start->tv_nsec <= curr->tv_nsec;
    }

  return (long)((start->tv_sec + secs_since_start) - curr->tv_sec) <= 0;
}

/****************************************************************************
 * Name: time_has_passed_ms
 *
 * Description:
 *   Return true if curr >= start + msecs_since_start
 *
 ****************************************************************************/

static bool time_has_passed_ms(FAR struct timespec *curr,
                               FAR struct timespec *start,
                               unsigned int msecs_since_start)
{
  uint32_t start_msec = start->tv_nsec / (1000 * 1000);
  uint32_t curr_msec = curr->tv_nsec / (1000 * 1000);

  if (start->tv_sec < curr->tv_sec)
    {
      curr_msec += 1000 * ((long)(curr->tv_sec - start->tv_sec));
    }

  return (start_msec + msecs_since_start) <= curr_msec;
}

/****************************************************************************
 * Name: sgp30_measure_airq
 ****************************************************************************/

static int sgp30_measure_airq(FAR struct sgp30_dev_s *priv,
                              FAR struct sgp30_conv_data_s *data)
{
  struct sgp30_word_s words[2];
  int ret;

  /* Read the data */

  ret = sgp30_read_cmd(priv, SGP30_CMD_MEASURE_AIR_QUALITY, words, 2);
  if (ret < 0)
    {
      sgp30_dbg("ERROR: sgp30_write_cmd failed: %d\n", ret);
      return ret;
    }

  if (sgp30_check_data_crc(words, 2) < 0)
    {
      sgp30_dbg("ERROR: sgp30_read_words crc failed\n");
      ret = -EIO;
      return ret;
    }

  add_sensor_randomness(words[0].crc ^ (words[1].crc << 8));
  data->co2eq_ppm = sgp30_data_word_to_uint16(words + 0);
  data->tvoc_ppb = sgp30_data_word_to_uint16(words + 1);

  return OK;
}

/****************************************************************************
 * Name: sgp30_measure_raw
 ****************************************************************************/

static int sgp30_measure_raw(FAR struct sgp30_dev_s *priv,
                             FAR struct sgp30_raw_data_s *data)
{
  struct sgp30_word_s words[2];
  int ret;

  /* Read the data */

  ret = sgp30_read_cmd(priv, SGP30_CMD_MEASURE_RAW_SIGNALS, words, 2);
  if (ret < 0)
    {
      sgp30_dbg("ERROR: sgp30_write_cmd failed: %d\n", ret);
      return ret;
    }

  if (sgp30_check_data_crc(words, 2) < 0)
    {
      sgp30_dbg("ERROR: sgp30_read_words crc failed\n");
      ret = -EIO;
      return ret;
    }

  add_sensor_randomness(words[0].crc ^ (words[1].crc << 8));
  data->h2_signal = sgp30_data_word_to_uint16(words + 0);
  data->ethanol_signal = sgp30_data_word_to_uint16(words + 1);

  return OK;
}

/****************************************************************************
 * Name: sgp30_open
 *
 * Description:
 *   This function is called whenever the SGP30x device is opened.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int sgp30_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sgp30_dev_s *priv = inode->i_private;
  int ret = OK;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Increment the count of open references on the driver */

  priv->crefs++;
  DEBUGASSERT(priv->crefs > 0);

  if (priv->crefs == 1)
    {
      struct sgp30_word_s serial[3];
      struct sgp30_word_s buf[1];

      if (sgp30_read_cmd(priv, SGP30_CMD_GET_SERIAL_ID, serial, 3) >= 0
          && sgp30_check_data_crc(serial, 3) >= 0 &&
          sgp30_read_cmd(priv,
                         SGP30_CMD_GET_FEATURE_SET_VERSION, buf, 1) >= 0
          && sgp30_check_data_crc(buf, 1) >= 0)
        {
          struct timespec start;
          struct timespec curr;
          sgp30_dbg("serial id: %04x-%04x-%04x\n",
                    sgp30_data_word_to_uint16(serial + 0),
                    sgp30_data_word_to_uint16(serial + 1),
                    sgp30_data_word_to_uint16(serial + 2));
          sgp30_dbg("feature set version: %d\n",
                    sgp30_data_word_to_uint16(buf));

          add_sensor_randomness((buf[0].crc << 24) ^ (serial[0].crc << 16) ^
                                (serial[1].crc << 8) ^ (serial[2].crc << 0));

          clock_systime_timespec(&start);
          ret = sgp30_write_cmd(priv, SGP30_CMD_INIT_AIR_QUALITY, NULL, 0);
          if (ret < 0)
            {
              sgp30_dbg("sgp30_write_cmd(SGP30_CMD_INIT_AIR_QUALITY)"
                         " failed, %d\n", ret);
            }
          else
            {
              uint32_t repeat = SGP30_INIT_RETRIES;
              clock_systime_timespec(&curr);
              sgp30_dbg("sgp30_write_cmd(SGP30_CMD_INIT_AIR_QUALITY)\n");
              while (repeat-- &&
                     time_has_passed_ms(&curr, &start, SGP30_INIT_LIMIT_MS))
                {
                  /* Infrequently the SGP30_CMD_INIT_AIR_QUALITY message
                   * delivery takes suspiciously long time
                   * (SGP30_INIT_LIMIT_MS or more) and in these cases the
                   * TVOC values will never reach the correct level (not
                   * even after 24 hours).
                   * If this delay is detected, the sensor is given a
                   * "General Call" soft reset as described in the SGP30
                   * datasheet and initialization is tried again after
                   * CONFIG_SGP30_RESET_DELAY_US.
                   */

                  ret = sgp30_soft_reset(priv);
                  if (ret < 0)
                    {
                      sgp30_dbg("sgp30_soft_reset failed, %d\n", ret);
                      return ret;
                    }

                  nxsig_usleep(CONFIG_SGP30_RESET_DELAY_US);

                  clock_systime_timespec(&start);
                  ret = sgp30_write_cmd(priv, SGP30_CMD_INIT_AIR_QUALITY,
                                        NULL, 0);
                  clock_systime_timespec(&curr);
                  if (ret < 0)
                    {
                      sgp30_dbg("sgp30_write_cmd(SGP30_CMD_INIT_AIR_QUALITY)"
                                 " failed, %d\n", ret);
                    }
                }
            }
        }
      else
        {
          ret = -ENODEV;
        }

      if (ret < 0)
        {
          ret = -ENODEV;
          priv->crefs--;
        }
    }

  nxmutex_unlock(&priv->devlock);
  return ret;
}
#endif

/****************************************************************************
 * Name: sgp30_close
 *
 * Description:
 *   This routine is called when the SGP30 device is closed.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int sgp30_close(FAR struct file *filep)
{
  FAR struct inode       *inode = filep->f_inode;
  FAR struct sgp30_dev_s *priv  = inode->i_private;
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
 * Name: sgp30_read
 ****************************************************************************/

static ssize_t sgp30_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sgp30_dev_s *priv = inode->i_private;
  ssize_t length = 0;
  struct timespec ts;
  struct timespec ts_sleep;
  struct sgp30_conv_data_s data;
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

  /* Note: For correct operation, SGP30 documentation tells host
   *       to run measurement command every 1 second.
   */

  clock_systime_timespec(&ts);

  while (!has_time_passed(&ts, &priv->last_update, 1))
    {
      if (filep->f_oflags & O_NONBLOCK)
        {
          nxmutex_unlock(&priv->devlock);
          return -EAGAIN;
        }

      ts_sleep.tv_sec = priv->last_update.tv_sec + 1 - ts.tv_sec;
      ts_sleep.tv_nsec = priv->last_update.tv_nsec - ts.tv_nsec;
      while (ts_sleep.tv_nsec < 0)
        {
          ts_sleep.tv_sec--;
          ts_sleep.tv_nsec += NSEC_PER_SEC;
        }

      if ((long)ts_sleep.tv_sec >= 0)
        {
          ret = nxsig_nanosleep(&ts_sleep, NULL);
          if (ret == -EINTR)
            {
              nxmutex_unlock(&priv->devlock);
              return -EINTR;
            }
        }

      clock_systime_timespec(&ts);
    }

  ret = sgp30_measure_airq(priv, &data);
  if (ret < 0)
    {
      sgp30_dbg("cannot read data: %d\n", ret);
    }
  else
    {
      /* This interface is mainly intended for easy debugging in nsh. */

      length = snprintf(buffer, buflen, "%u %u\n",
                        data.co2eq_ppm, data.tvoc_ppb);
      if (length > buflen)
        {
          length = buflen;
        }

      priv->last_update = ts;
    }

  nxmutex_unlock(&priv->devlock);
  return length;
}

/****************************************************************************
 * Name: sgp30_write
 ****************************************************************************/

static ssize_t sgp30_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: sgp30_ioctl
 ****************************************************************************/

static int sgp30_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sgp30_dev_s *priv = inode->i_private;
  struct sgp30_word_s buf[2];
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
      /* Soft reset the SGP30, Arg: None */

      case SNIOC_RESET:
        {
          /* Note: After INIT_AIR_QUALITY, host should write baseline
           *       values (SNIOC_SET_BASELINE) from non-volatile memory if
           *       stored values are less than week old. Without baseline
           *       values, sensor enters 12hr long early operation phase.
           */

          ret = sgp30_write_cmd(priv, SGP30_CMD_INIT_AIR_QUALITY, NULL, 0);
          sgp30_dbg("reset ret: %d\n", ret);
        }
        break;

      /* Get sensor baseline, Arg: struct sgp30_baseline_s * */

      case SNIOC_GET_BASELINE:
        {
          FAR struct sgp30_baseline_s *baseline =
              (FAR struct sgp30_baseline_s *)arg;

          /* Note: SGP30 documentation tells host should read baseline values
           *       every 1 hour and save to non-volatile memory along with
           *       timestamp.
           */

          ret = sgp30_read_cmd(priv, SGP30_CMD_GET_BASELINE, buf, 2);
          if (ret >= 0)
            {
              if (sgp30_check_data_crc(buf, 2) < 0)
                {
                  ret = -EIO;
                  break;
                }

              baseline->co2eq_baseline = sgp30_data_word_to_uint16(buf + 0);
              baseline->tvoc_baseline = sgp30_data_word_to_uint16(buf + 1);
            }
        }
        break;

      /* Set sensor baseline, Arg: struct sgp30_baseline_s * */

      case SNIOC_SET_BASELINE:
        {
          FAR const struct sgp30_baseline_s *baseline =
              (FAR const struct sgp30_baseline_s *)arg;

          sgp30_set_command_param(buf + 0, baseline->co2eq_baseline);
          sgp30_set_command_param(buf + 1, baseline->tvoc_baseline);
          ret = sgp30_write_cmd(priv, SGP30_CMD_SET_BASELINE, buf, 2);
        }
        break;

      /* Set humidity compensation value, Arg: uint16_t */

      case SNIOC_SET_HUMIDITY:
        {
          /* Input is absolute humidity in unit "mg/m^3". */

          if (arg >= 256000)
            {
              ret = -EINVAL;
              break;
            }

          arg = arg * 256 / 1000; /* scale to range 0..65535 */

          sgp30_set_command_param(buf, arg);
          ret = sgp30_write_cmd(priv, SGP30_CMD_SET_HUMIDITY, buf, 1);
        }
        break;

      /* Read sensor data, Arg: struct sgp30_conv_data_s *data */

      case SNIOC_READ_CONVERT_DATA:
        {
          FAR struct sgp30_conv_data_s *data =
            (FAR struct sgp30_conv_data_s *)arg;

          /* Note: For correct operation, SGP30 documentation tells host
           *       to run measurement command every 1 second.
           */

          ret = sgp30_measure_airq(priv, data);
          if (ret < 0)
            {
              sgp30_dbg("cannot read data: %d\n", ret);
            }
        }
        break;

      /* Read raw data, Arg: struct sgp30_raw_data_s *data */

      case SNIOC_READ_RAW_DATA:
        {
          FAR struct sgp30_raw_data_s *data =
            (FAR struct sgp30_raw_data_s *)arg;

          ret = sgp30_measure_raw(priv, data);
          if (ret < 0)
            {
              sgp30_dbg("cannot read data: %d\n", ret);
            }
        }
        break;

      /* Run selftest, Arg: None. */

      case SNIOC_START_SELFTEST:
        {
          ret = sgp30_write_cmd(priv, SGP30_CMD_MEASURE_TEST, NULL, 0);
          if (ret >= 0)
            {
              if (sgp30_check_data_crc(buf, 1) < 0)
                {
                  sgp30_dbg("crc error\n");
                  ret = -EIO;
                }
              else if (sgp30_data_word_to_uint16(buf) != 0xd400)
                {
                  sgp30_dbg("self-test failed, 0x%04x\n",
                            sgp30_data_word_to_uint16(buf));
                  ret = -EFAULT;
                }
            }
        }
        break;

      default:
        sgp30_dbg("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: sgp30_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int sgp30_unlink(FAR struct inode *inode)
{
  FAR struct sgp30_dev_s *priv;
  int ret;

  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv = (FAR struct sgp30_dev_s *)inode->i_private;

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
 * Name: sgp30_register
 *
 * Description:
 *   Register the SGP30 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/gas0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the SGP30
 *   addr    - The I2C address of the SGP30. The I2C address of SGP30 is
 *             always 0x58.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sgp30_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                   uint8_t addr)
{
  FAR struct sgp30_dev_s *priv;
  int ret;

  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(addr == CONFIG_SGP30_ADDR);
  DEBUGASSERT(sgp30_crc_word(0xbeef) == 0x92);

  /* Initialize the device structure */

  priv = (FAR struct sgp30_dev_s *)kmm_zalloc(sizeof(struct sgp30_dev_s));
  if (priv == NULL)
    {
      sgp30_dbg("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;

  nxmutex_init(&priv->devlock);

  /* Register the character driver */

  ret = register_driver(devpath, &g_sgp30fops, 0666, priv);
  if (ret < 0)
    {
      sgp30_dbg("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}
#endif /* CONFIG_I2C && CONFIG_SENSORS_SGP30 */
