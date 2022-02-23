/****************************************************************************
 * drivers/sensors/sps30.c
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
#include <nuttx/sensors/sps30.h>
#include <nuttx/random.h>

#if defined(CONFIG_SENSORS_SPS30)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SPS30_DEBUG
#  define sps30_dbg(x, ...)    _info(x, ##__VA_ARGS__)
#else
#  define sps30_dbg(x, ...)    sninfo(x, ##__VA_ARGS__)
#endif

#ifndef CONFIG_SPS30_I2C_FREQUENCY
#  define CONFIG_SPS30_I2C_FREQUENCY 100000
#endif

#define SPS30_MEASUREMENT_INTERVAL 1      /* one second, fixed in hw */
#define SPS30_MEASUREMENT_MODE     0x0300

#define SPS30_I2C_RETRIES 3

/* SPS30 command words */

#define SPS30_CMD_START_MEASUREMENT          0x0010
#define SPS30_CMD_STOP_MEASUREMENT           0x0104
#define SPS30_CMD_GET_DATA_READY             0x0202
#define SPS30_CMD_READ_MEASUREMENT           0x0300
#define SPS30_CMD_SET_AUTO_CLEANING_INTERVAL 0x8004
#define SPS30_CMD_START_FAN_CLEANING         0x5607
#define SPS30_CMD_READ_ARTICLE_CODE          0xd025
#define SPS30_CMD_READ_SERIAL_NUMBER         0xd033
#define SPS30_CMD_SOFT_RESET                 0xd304

/****************************************************************************
 * Private
 ****************************************************************************/

struct sps30_dev_s
{
#ifdef CONFIG_SPS30_I2C
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
#endif
  bool valid;                   /* If cached readings are valid */
  bool started;                 /* If continuous measurement is enabled */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  bool unlinked;                /* True, driver has been unlinked */
#endif
  struct timespec last_update;  /* Last time when sensor was read */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  int16_t crefs;                /* Number of open references */
#endif
  sem_t devsem;

  /* Cached sensor values */

  struct sps30_conv_data_s data;
};

struct sps30_word_s
{
  uint8_t data[2];
  uint8_t crc;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IO Helpers */

#ifdef CONFIG_SPS30_I2C
static int sps30_do_transfer(FAR struct i2c_master_s *i2c,
                             FAR struct i2c_msg_s *msgv,
                             size_t nmsg);
#endif
static int sps30_write_cmd(FAR struct sps30_dev_s *priv, uint16_t cmd,
                           FAR struct sps30_word_s *params,
                           unsigned int num_params);
static int sps30_read_cmd(FAR struct sps30_dev_s *priv, uint16_t cmd,
                          FAR struct sps30_word_s *words,
                          unsigned int num_words);

/* Data conversion */

static uint8_t sps30_crc_word(uint16_t word);
static void sps30_set_command_param(FAR struct sps30_word_s *param,
                                    uint16_t value);
static int sps30_check_data_crc(FAR const struct sps30_word_s *words,
                                unsigned int num_words);
static uint16_t sps30_data_word2uint16(FAR const struct sps30_word_s *word);
static float sps30_data_words2float(FAR const struct sps30_word_s words[2]);

/* Driver features */

static int sps30_read_values(FAR struct sps30_dev_s *priv,
                             FAR struct sps30_conv_data_s *out, bool wait);
static int sps30_configure(FAR struct sps30_dev_s *priv, bool start);
static int sps30_read_dev_info(FAR struct sps30_dev_s *priv, uint16_t cmd,
                               char *out, size_t outlen);

/* Character driver methods */

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int sps30_open(FAR struct file *filep);
static int sps30_close(FAR struct file *filep);
#endif
static ssize_t sps30_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t sps30_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int sps30_ioctl(FAR struct file *filep, int cmd,
                       unsigned long arg);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int sps30_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_sps30fops =
{
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  sps30_open,     /* open */
  sps30_close,    /* close */
#else
  NULL,           /* open */
  NULL,           /* close */
#endif
  sps30_read,     /* read */
  sps30_write,    /* write */
  NULL,           /* seek */
  sps30_ioctl,    /* ioctl */
  NULL            /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , sps30_unlink /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sps30_do_transfer
 ****************************************************************************/

#ifdef CONFIG_SPS30_I2C
static int sps30_do_transfer(FAR struct i2c_master_s *i2c,
                             FAR struct i2c_msg_s *msgv,
                             size_t nmsg)
{
  int ret = -EIO;
  int retries;

  for (retries = 0; retries < SPS30_I2C_RETRIES; retries++)
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
          if (retries == SPS30_I2C_RETRIES - 1)
            {
              break;
            }

          ret = I2C_RESET(i2c);
          if (ret < 0)
            {
              sps30_dbg("I2C_RESET failed: %d\n", ret);
              return ret;
            }
#endif
        }
    }

  sps30_dbg("xfer failed: %d\n", ret);
  return ret;
}
#endif

/****************************************************************************
 * Name: sps30_write_cmd
 ****************************************************************************/

static int sps30_write_cmd(FAR struct sps30_dev_s *priv, uint16_t cmd,
                           FAR struct sps30_word_s *params,
                           unsigned int num_params)
{
#ifdef CONFIG_SPS30_I2C
  struct i2c_msg_s msg[2];
  uint8_t cmd_buf[2];
  int ret;

  cmd_buf[0] = cmd >> 8;
  cmd_buf[1] = cmd >> 0;

  msg[0].frequency = CONFIG_SPS30_I2C_FREQUENCY;
  msg[0].addr = priv->addr;
  msg[0].flags = 0;
  msg[0].buffer = cmd_buf;
  msg[0].length = 2;

  if (num_params)
    {
      msg[1].frequency = CONFIG_SPS30_I2C_FREQUENCY;
      msg[1].addr = priv->addr;
      msg[1].flags = I2C_M_NOSTART;
      msg[1].buffer = (FAR uint8_t *)params;
      msg[1].length = num_params * sizeof(*params);
    }

  ret = sps30_do_transfer(priv->i2c, msg, (num_params) ? 2 : 1);

  sps30_dbg("cmd: 0x%04X num_params: %d ret: %d\n",
            cmd, num_params, ret);
  return (ret >= 0) ? OK : ret;
#else
  /* UART mode not implemented yet. */

  return -ENODEV;
#endif
}

/****************************************************************************
 * Name: sps30_read_words
 ****************************************************************************/

static int sps30_read_cmd(FAR struct sps30_dev_s *priv, uint16_t cmd,
                          FAR struct sps30_word_s *words,
                          unsigned int num_words)
{
#ifdef CONFIG_SPS30_I2C
  struct i2c_msg_s msg[1];
  uint8_t addr_buf[2];
  int ret;

  addr_buf[0] = cmd >> 8;
  addr_buf[1] = cmd >> 0;

  msg[0].frequency = CONFIG_SPS30_I2C_FREQUENCY;
  msg[0].addr = priv->addr;
  msg[0].flags = 0;
  msg[0].buffer = addr_buf;
  msg[0].length = 2;

  ret = sps30_do_transfer(priv->i2c, msg, 1);

  sps30_dbg("cmd: 0x%04X ret: %d\n", cmd, ret);

  if (ret < 0)
    {
      return ret;
    }

  msg[0].frequency = CONFIG_SPS30_I2C_FREQUENCY;
  msg[0].addr = priv->addr;
  msg[0].flags = I2C_M_READ;
  msg[0].buffer = (FAR uint8_t *)words;
  msg[0].length = num_words * sizeof(*words);

  ret = sps30_do_transfer(priv->i2c, msg, 1);

  sps30_dbg("read cmd: 0x%04X num_params: %d ret: %d\n",
            cmd, num_words, ret);
  return (ret >= 0) ? OK : ret;
#else
  /* UART mode not implemented yet. */

  return -ENODEV;
#endif
}

/****************************************************************************
 * Name: sps30_crc_word
 ****************************************************************************/

static uint8_t sps30_crc_word(uint16_t word)
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
 * Name: sps30_set_command_param
 ****************************************************************************/

static void sps30_set_command_param(FAR struct sps30_word_s *param,
                                    uint16_t value)
{
  param->data[0] = value >> 8;
  param->data[1] = value >> 0;
  param->crc = sps30_crc_word(value);
}

/****************************************************************************
 * Name: sps30_data_words2float
 ****************************************************************************/

static float sps30_data_words2float(FAR const struct sps30_word_s words[2])
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
 * Name: sps30_data_word2uint16
 ****************************************************************************/

static uint16_t sps30_data_word2uint16(FAR const struct sps30_word_s *word)
{
  return (word[0].data[0] << 8) | (word[0].data[1]);
}

/****************************************************************************
 * Name: sps30_crc_word
 ****************************************************************************/

static int sps30_check_data_crc(FAR const struct sps30_word_s *words,
                                unsigned int num_words)
{
  while (num_words)
    {
      if (sps30_crc_word(sps30_data_word2uint16(words)) != words->crc)
        {
          return -1;
        }

      num_words--;
      words++;
    }

  return 0;
}

/****************************************************************************
 * Name: sps30_softreset
 *
 * Description:
 *   Reset the SPS30 sensor. This takes less than 2000 ms.
 *
 ****************************************************************************/

static int sps30_softreset(FAR struct sps30_dev_s *priv)
{
  int ret;

  ret = sps30_write_cmd(priv, SPS30_CMD_SOFT_RESET, NULL, 0);
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
 * Name: sps30_read_values
 ****************************************************************************/

static int sps30_read_values(FAR struct sps30_dev_s *priv,
                             FAR struct sps30_conv_data_s *out, bool wait)
{
  struct sps30_word_s data[20];
  struct timespec ts;
  int ret;

  clock_gettime(CLOCK_REALTIME, &ts);

  if (wait || !priv->valid ||
      has_time_passed(ts, priv->last_update, SPS30_MEASUREMENT_INTERVAL))
    {
      while (1)
        {
          /* Wait data to be ready. */

          ret = sps30_read_cmd(priv, SPS30_CMD_GET_DATA_READY, data, 1);
          if (ret < 0)
            {
              sps30_dbg("ERROR: sps30_read_cmd failed: %d\n", ret);
              return ret;
            }

          if (sps30_check_data_crc(data, 1) < 0)
            {
              sps30_dbg("ERROR: sps30_read_words crc failed\n");
              ret = -EIO;
              return ret;
            }

          if (sps30_data_word2uint16(data) != 0x0001)
            {
              if (!wait)
                {
                  sps30_dbg("ERROR: data not ready\n");
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

      ret = sps30_read_cmd(priv, SPS30_CMD_READ_MEASUREMENT, data, 20);
      if (ret < 0)
        {
          sps30_dbg("ERROR: sps30_read_cmd failed: %d\n", ret);
          return ret;
        }

      if (sps30_check_data_crc(data, 20) < 0)
        {
          sps30_dbg("ERROR: sps30_read_words crc failed\n");
          ret = -EIO;
          return ret;
        }

      add_sensor_randomness(((data[0].crc ^ data[1].crc) << 0) ^
                            ((data[2].crc ^ data[3].crc) << 8) ^
                            ((data[4].crc ^ data[5].crc) << 16) ^
                            ((data[6].crc ^ data[7].crc) << 24) ^
                            ((data[8].crc ^ data[9].crc) << 0) ^
                            ((data[10].crc ^ data[11].crc) << 8) ^
                            ((data[12].crc ^ data[13].crc) << 16) ^
                            ((data[14].crc ^ data[15].crc) << 24) ^
                            ((data[16].crc ^ data[17].crc) << 0) ^
                            ((data[18].crc ^ data[19].crc) << 8));

      priv->data.mass_concenration_pm1_0 =
          sps30_data_words2float(data + 0);
      priv->data.mass_concenration_pm2_5 =
          sps30_data_words2float(data + 2);
      priv->data.mass_concenration_pm4_0 =
          sps30_data_words2float(data + 4);
      priv->data.mass_concenration_pm10 =
          sps30_data_words2float(data + 6);
      priv->data.number_concenration_pm0_5 =
          sps30_data_words2float(data + 8);
      priv->data.number_concenration_pm1_0 =
          sps30_data_words2float(data + 10);
      priv->data.number_concenration_pm2_5 =
          sps30_data_words2float(data + 12);
      priv->data.number_concenration_pm4_0 =
          sps30_data_words2float(data + 14);
      priv->data.number_concenration_pm10 =
          sps30_data_words2float(data + 16);
      priv->data.typical_particle_size =
          sps30_data_words2float(data + 18);
      priv->last_update = ts;
      priv->valid = true;

      /* Wait data to be ready. */

      ret = sps30_read_cmd(priv, SPS30_CMD_GET_DATA_READY, data, 1);
      if (ret < 0)
        {
          sps30_dbg("ERROR: sps30_read_cmd failed: %d\n", ret);
        }
    }

  *out = priv->data;
  return OK;
}

/****************************************************************************
 * Name: sps30_read_dev_info
 ****************************************************************************/

static int sps30_read_dev_info(FAR struct sps30_dev_s *priv, uint16_t cmd,
                               char *out, size_t outlen)
{
  struct sps30_word_s buf[16];
  int ret;

  ret = sps30_read_cmd(priv, cmd, buf, 16);
  if (ret < 0)
    {
      sps30_dbg("ERROR: sps30_read_cmd failed: %d\n", ret);
      return ret;
    }

  if (sps30_check_data_crc(buf, 16) < 0)
    {
      sps30_dbg("ERROR: sps30_read_words crc failed\n");
      ret = -EIO;
      return ret;
    }

  ret = 0;
  while (outlen && ret < 32)
    {
      *out = buf[ret / 2].data[ret % 2];
      ret++;
      out++;
      outlen--;
    }

  if (outlen)
    {
      *out = '\0';
    }

  return ret;
}

/****************************************************************************
 * Name: sps30_configure
 ****************************************************************************/

static int sps30_configure(FAR struct sps30_dev_s *priv, bool start)
{
  struct sps30_word_s param;
  int ret;

  if (!start)
    {
      /* Stop measurements. */

      ret = sps30_write_cmd(priv, SPS30_CMD_STOP_MEASUREMENT, &param, 1);
      if (ret >= 0)
        {
          priv->started = false;
        }
    }
  else
    {
      /* Start measurements (and set pressure compensation). */

      sps30_set_command_param(&param, SPS30_MEASUREMENT_MODE);
      ret = sps30_write_cmd(priv, SPS30_CMD_START_MEASUREMENT, &param, 1);
      if (ret >= 0)
        {
          priv->started = true;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: sps30_open
 *
 * Description:
 *   This function is called whenever the SPS30x device is opened.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int sps30_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sps30_dev_s *priv  = inode->i_private;
  union article_u
    {
      uint32_t u32[8];
      char c[32];
    };

  union article_u code;
  union article_u sn;

  int ret;

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
      /* Read device information. */

      ret = sps30_read_dev_info(priv, SPS30_CMD_READ_ARTICLE_CODE, code.c,
                                sizeof(code.c));
      if (ret >= 0)
        {
          ret = sps30_read_dev_info(priv, SPS30_CMD_READ_SERIAL_NUMBER, sn.c,
                                    sizeof(sn.c));
          if (ret >= 0)
            {
              static int once;

              if (!once)
                {
                  once = 1;
                  up_rngaddentropy(RND_SRC_SENSOR, code.u32,
                                   (strlen(code.c) + 3) / 4);
                  up_rngaddentropy(RND_SRC_SENSOR, sn.u32,
                                   (strlen(sn.c) + 3) / 4);
                }

              sps30_dbg("article code: \"%.*s\"\n", 32, code.c);
              sps30_dbg("serial number: \"%.*s\"\n", 32, sn.c);

              /* Start sensor. */

              ret = sps30_configure(priv, true);
            }
        }
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
 * Name: sps30_close
 *
 * Description:
 *   This routine is called when the SPS30 device is closed.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int sps30_close(FAR struct file *filep)
{
  FAR struct inode       *inode = filep->f_inode;
  FAR struct sps30_dev_s *priv  = inode->i_private;
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
 * Name: sps30_read
 ****************************************************************************/

static ssize_t sps30_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sps30_dev_s *priv = inode->i_private;
  ssize_t length = 0;
  struct sps30_conv_data_s data;
  unsigned int data100[10];
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

  if (!priv->started)
    {
      return -ENODATA;
    }

  ret = sps30_read_values(priv, &data, !(filep->f_oflags & O_NONBLOCK));
  if (ret < 0)
    {
      sps30_dbg("cannot read data: %d\n", ret);
    }
  else
    {
      /* This interface is mainly intended for easy debugging in nsh. */

      data100[0] = abs(data.mass_concenration_pm1_0 * 100);
      data100[1] = abs(data.mass_concenration_pm2_5 * 100);
      data100[2] = abs(data.mass_concenration_pm4_0 * 100);
      data100[3] = abs(data.mass_concenration_pm10 * 100);
      data100[4] = abs(data.number_concenration_pm0_5 * 100);
      data100[5] = abs(data.number_concenration_pm1_0 * 100);
      data100[6] = abs(data.number_concenration_pm2_5 * 100);
      data100[7] = abs(data.number_concenration_pm4_0 * 100);
      data100[8] = abs(data.number_concenration_pm10 * 100);
      data100[9] = abs(data.typical_particle_size * 100);

      length = snprintf(buffer, buflen,
                        "%u.%02u %u.%02u %u.%02u %u.%02u %u.%02u "
                        "%u.%02u %u.%02u %u.%02u %u.%02u %u.%02u\n",
                        data100[0] / 100, data100[0] % 100,
                        data100[1] / 100, data100[1] % 100,
                        data100[2] / 100, data100[2] % 100,
                        data100[3] / 100, data100[3] % 100,
                        data100[4] / 100, data100[4] % 100,
                        data100[5] / 100, data100[5] % 100,
                        data100[6] / 100, data100[6] % 100,
                        data100[7] / 100, data100[7] % 100,
                        data100[8] / 100, data100[8] % 100,
                        data100[9] / 100, data100[9] % 100);
      if (length > buflen)
        {
          length = buflen;
        }
    }

  nxsem_post(&priv->devsem);
  return length;
}

/****************************************************************************
 * Name: sps30_write
 ****************************************************************************/

static ssize_t sps30_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: sps30_ioctl
 ****************************************************************************/

static int sps30_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sps30_dev_s *priv = inode->i_private;
  struct sps30_word_s param;
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
      /* Soft reset the SPS30, Arg: None */

      case SNIOC_RESET:
        {
          ret = sps30_softreset(priv);
          sps30_dbg("softreset ret: %d\n", ret);

          sps30_configure(priv, priv->started);
        }
        break;

      /* Start background measurement, Arg: None */

      case SNIOC_START:
        {
          /* Start measurements (and set pressure compensation). */

          sps30_set_command_param(&param, SPS30_MEASUREMENT_MODE);
          ret = sps30_write_cmd(priv, SPS30_CMD_START_MEASUREMENT,
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

          ret = sps30_write_cmd(priv, SPS30_CMD_STOP_MEASUREMENT, NULL, 0);
          if (ret >= 0)
            {
              priv->started = false;
            }
        }
        break;

      /* Set fan auto cleaning interval measurement interval, Arg: uint32_t */

      case SNIOC_SET_CLEAN_INTERVAL:
        {
          if (arg != (uint32_t)arg)
            {
              ret = -EINVAL;
              break;
            }

          if (arg > 0 && arg < 15)
            {
              arg = 15;
            }

          sps30_set_command_param(&param, arg);
          ret = sps30_write_cmd(priv, SPS30_CMD_SET_AUTO_CLEANING_INTERVAL,
                                &param, 1);
        }
        break;

      /* Start fan cleaning, Arg: None */

      case SNIOC_START_FAN_CLEANING:
        {
          if (!priv->started)
            {
              ret = -EBUSY;
              break;
            }

          ret = sps30_write_cmd(priv, SPS30_CMD_START_FAN_CLEANING, NULL, 0);
        }
        break;

      /* Read sensor data, Arg: struct sps30_conv_data_s *data */

      case SNIOC_READ_CONVERT_DATA:
        {
          FAR struct sps30_conv_data_s *data =
            (FAR struct sps30_conv_data_s *)arg;

          ret = sps30_read_values(priv, data, false);
          if (ret < 0)
            {
              sps30_dbg("cannot read data: %d\n", ret);
            }
        }
        break;

      default:
        sps30_dbg("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  nxsem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: sps30_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int sps30_unlink(FAR struct inode *inode)
{
  FAR struct sps30_dev_s *priv;
  int ret;

  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv = (FAR struct sps30_dev_s *)inode->i_private;

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

#ifdef CONFIG_SPS30_I2C
/****************************************************************************
 * Name: sps30_register_i2c
 *
 * Description:
 *   Register the SPS30 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register e.g., "/dev/particle0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the SPS30
 *   addr    - The I2C address of the SPS30. The I2C address of SPS30 is
 *             always 0x69.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sps30_register_i2c(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                       uint8_t addr)
{
  FAR struct sps30_dev_s *priv;
  int ret;

  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(addr == CONFIG_SPS30_ADDR);
  DEBUGASSERT(sps30_crc_word(0xbeef) == 0x92);

  /* Initialize the device structure */

  priv = (FAR struct sps30_dev_s *)kmm_zalloc(sizeof(struct sps30_dev_s));
  if (priv == NULL)
    {
      sps30_dbg("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;
  priv->started = false;

  nxsem_init(&priv->devsem, 0, 1);

  /* Register the character driver */

  ret = register_driver(devpath, &g_sps30fops, 0666, priv);
  if (ret < 0)
    {
      sps30_dbg("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}
#endif /* CONFIG_SPS30_I2C */

#endif /* CONFIG_SENSORS_SPS30 */
