/****************************************************************************
 * drivers/sensors/max44009.c
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
#include <sys/types.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <poll.h>
#include <stdio.h>

#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/random.h>

#include <nuttx/sensors/max44009.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_MAX44009
#  define max44009_dbg(x, ...)      _info(x, ##__VA_ARGS__)
#else
#  define max44009_dbg(x, ...)      sninfo(x, ##__VA_ARGS__)
#endif

#ifndef CONFIG_MAX44009_I2C_FREQUENCY
#  define CONFIG_MAX44009_I2C_FREQUENCY 400000
#endif

/* Registers */

#define MAX44009_INT_STS            0x0
#define MAX44009_INT_EN             0x01
#define MAX44009_CONFIG             0x02
#define MAX44009_LUX_HBYTE          0x03
#define MAX44009_LUX_LBYTE          0x04
#define MAX44009_UP_THRESH_BYTE     0x05
#define MAX44009_LOW_THRESH_BYTE    0x06
#define MAX44009_THRESH_TIMER       0x07

/* Other constants */

#define MAX44009_I2C_RETRIES        10

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct max44009_dev_s
{
  FAR struct max44009_config_s *config;
  sem_t dev_sem;
  FAR struct i2c_master_s *i2c;
  uint8_t addr;
  uint8_t cref;
  bool int_pending;
  struct pollfd *fds[CONFIG_MAX44009_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int max44009_open(FAR struct file *filep);
static int max44009_close(FAR struct file *filep);
static ssize_t max44009_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
static ssize_t max44009_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen);
static int max44009_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg);
static int max44009_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup);

static int max44009_read_data(FAR struct max44009_dev_s *priv,
                              FAR struct max44009_data_s *data);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_alsops =
{
  max44009_open,   /* open */
  max44009_close,  /* close */
  max44009_read,   /* read */
  max44009_write,  /* write */
  NULL,            /* seek */
  max44009_ioctl,  /* ioctl */
  max44009_poll    /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL           /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int max44009_do_transfer(FAR struct max44009_dev_s *dev,
                                FAR struct i2c_msg_s *msgv,
                                size_t nmsg)
{
  int ret = -EIO;
  int retries;

  for (retries = 0; retries < MAX44009_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(dev->i2c, msgv, nmsg);
      if (ret >= 0)
        {
          return 0;
        }
      else
        {
#ifdef CONFIG_I2C_RESET
          /* Some error. Try to reset I2C bus and keep trying. */

          if (retries == MAX44009_I2C_RETRIES - 1)
            {
              break;
            }

          ret = I2C_RESET(dev->i2c);
          if (ret < 0)
            {
              max44009_dbg("I2C_RESET failed: %d\n", ret);
              return ret;
            }
#endif
        }
    }

  max44009_dbg("xfer failed: %d\n", ret);
  return ret;
}

static int max44009_write_reg8(FAR struct max44009_dev_s *dev,
                               FAR const uint8_t *command)
{
  struct i2c_msg_s msgv[2] =
  {
    {
      .frequency = CONFIG_MAX44009_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = 0,
      .buffer    = (void *)&command[0],
      .length    = 1
    },
    {
      .frequency = CONFIG_MAX44009_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = I2C_M_NOSTART,
      .buffer    = (void *)&command[1],
      .length    = 1
    }
  };

  return max44009_do_transfer(dev, msgv, 2);
}

static int max44009_read_reg8(FAR struct max44009_dev_s *dev,
                              FAR uint8_t *command,
                              FAR uint8_t *value)
{
  struct i2c_msg_s msgv[2] =
  {
    {
      .frequency = CONFIG_MAX44009_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = 0,
      .buffer    = command,
      .length    = 1
    },
    {
      .frequency = CONFIG_MAX44009_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = I2C_M_READ,
      .buffer    = value,
      .length    = 1
    }
  };

  return max44009_do_transfer(dev, msgv, 2);
}

static int max44009_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct max44009_dev_s *priv;
  unsigned int use_count;
  int ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct max44009_dev_s *)inode->i_private;

  ret = nxsem_wait_uninterruptible(&priv->dev_sem);
  if (ret < 0)
    {
      return ret;
    }

  use_count = priv->cref + 1;
  if (use_count == 1)
    {
      /* First user, do power on. */

      ret = priv->config->set_power(priv->config, true);
      if (ret < 0)
        {
          max44009_dbg("Cannot power on sensor: %d\n", ret);
          goto out_sem;
        }

      priv->config->irq_enable(priv->config, true);
      priv->cref = use_count;
    }
  else
    {
      DEBUGASSERT(use_count < UINT8_MAX && use_count > priv->cref);

      priv->cref = use_count;
      ret = 0;
    }

  max44009_dbg("Sensor is powered on\n");

out_sem:
  nxsem_post(&priv->dev_sem);
  return ret;
}

static int max44009_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct max44009_dev_s *priv;
  int use_count;
  int ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct max44009_dev_s *)inode->i_private;

  ret = nxsem_wait_uninterruptible(&priv->dev_sem);
  if (ret < 0)
    {
      return ret;
    }

  use_count = priv->cref - 1;
  if (use_count == 0)
    {
      priv->config->irq_enable(priv->config, false);

      /* Last user, do power off. */

      priv->config->set_power(priv->config, false);
      priv->cref = use_count;
    }
  else
    {
      DEBUGASSERT(use_count > 0);

      priv->cref = use_count;
    }

  max44009_dbg("CLOSED\n");
  nxsem_post(&priv->dev_sem);
  return OK;
}

static ssize_t max44009_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen)
{
  FAR struct inode *inode;
  FAR struct max44009_dev_s *priv;
  ssize_t length = 0;
  int ret;
  struct max44009_data_s data;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct max44009_dev_s *)inode->i_private;

  ret = nxsem_wait_uninterruptible(&priv->dev_sem);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  ret = max44009_read_data(priv, &data);
  if (ret < 0)
    {
      max44009_dbg("failed to read the sensor\n");
    }
  else
    {
      /* This interface is mainly intended for easy debugging in nsh. */

      length = snprintf(buffer, buflen, "%u.%03hu", data.lux, data.mlux);
      if (length > buflen)
        {
          length = buflen;
        }
    }

  nxsem_post(&priv->dev_sem);
  return length;
}

static ssize_t max44009_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen)
{
  ssize_t length = 0;
  return length;
}

static int max44009_set_interrupt_bit(FAR struct max44009_dev_s *priv,
                                      bool is_activated)
{
  int ret;
  uint8_t cmd[2];

  cmd[0] = MAX44009_INT_EN;
  cmd[1] = is_activated ? (1 << 0) : 0;

  ret = max44009_write_reg8(priv, cmd);
  if (ret < 0)
    {
      max44009_dbg("Cannot set interrupt bit\n");
    }

  return ret;
}

static int max44009_set_manual_mode(FAR struct max44009_dev_s *priv,
                                    bool is_manual)
{
  int ret;
  uint8_t value = 0;
  const uint8_t manual_bit = (1 << 6);
  uint8_t cmd[2];

  cmd[0] = MAX44009_CONFIG;
  ret    = max44009_read_reg8(priv, &cmd[0], &value);
  if (ret < 0)
    {
      max44009_dbg("Cannot read config register\n");
      goto fail;
    }

  if (is_manual)
    {
      value |= manual_bit;
    }
  else
    {
      value &= ~manual_bit;
    }

  cmd[1] = value;
  ret    = max44009_write_reg8(priv, cmd);
  if (ret < 0)
    {
      max44009_dbg("Cannot set manual bit in config register\n");
    }

fail:
  return ret;
}

static int max44009_set_continuous_mode(FAR struct max44009_dev_s *priv,
                                        bool is_cont)
{
  int ret;
  uint8_t value = 0;
  const uint8_t cont_bit = (1 << 7);
  uint8_t cmd[2];

  cmd[0] = MAX44009_CONFIG;
  ret    = max44009_read_reg8(priv, &cmd[0], &value);
  if (ret < 0)
    {
      max44009_dbg("Cannot read config register\n");
      goto fail;
    }

  if (is_cont)
    {
      value |= cont_bit;
    }
  else
    {
      value &= ~cont_bit;
    }

  cmd[1] = value;
  ret    = max44009_write_reg8(priv, cmd);
  if (ret < 0)
    {
      max44009_dbg("Cannot set cont bit in config register\n");
    }

fail:
  return ret;
}

static int max44009_set_current_div_ratio(FAR struct max44009_dev_s *priv,
                                          bool is_cdr)
{
  int ret;
  uint8_t value = 0;
  const uint8_t cdr_bit = (1 << 3);
  uint8_t cmd[2];

  cmd[0] = MAX44009_CONFIG;
  ret    = max44009_read_reg8(priv, &cmd[0], &value);
  if (ret < 0)
    {
      max44009_dbg("Cannot read config register\n");
      goto fail;
    }

  if (is_cdr)
    {
      value |= cdr_bit;
    }
  else
    {
      value &= ~cdr_bit;
    }

  cmd[1] = value;
  ret    = max44009_write_reg8(priv, cmd);
  if (ret < 0)
    {
      max44009_dbg("Cannot set cdr bit in config register\n");
    }

fail:
  return ret;
}

static int max44009_set_integration_time(FAR struct max44009_dev_s *priv,
                                         enum max44009_integration_time_e
                                         integration_time)
{
  int ret;
  uint8_t value = 0;
  const uint8_t tim_bits = integration_time << 0;
  const uint8_t tim_mask = 0x07;
  uint8_t cmd[2];

  cmd[0] = MAX44009_CONFIG;
  ret    = max44009_read_reg8(priv, &cmd[0], &value);
  if (ret < 0)
    {
      max44009_dbg("Cannot read config register\n");
      goto fail;
    }

  value &= ~tim_mask;
  value |= tim_bits;

  cmd[1] = value;
  ret    = max44009_write_reg8(priv, cmd);
  if (ret < 0)
    {
      max44009_dbg("Cannot set tim bits in config register\n");
    }

fail:
  return ret;
}

static int max44009_set_threshold_timer(FAR struct max44009_dev_s *priv,
                                        uint8_t threshold_timer)
{
  int ret;
  uint8_t cmd[2];

  cmd[0] = MAX44009_THRESH_TIMER;
  cmd[1] = threshold_timer;

  ret = max44009_write_reg8(priv, cmd);
  if (ret < 0)
    {
      max44009_dbg("Threshold timer cannot be set\n");
    }

  return ret;
}

static int max44009_set_threshold(FAR struct max44009_dev_s *priv,
                                  FAR struct max44009_threshold_s *settings)
{
  int ret;
  uint8_t cmd[2];

  cmd[0] = MAX44009_UP_THRESH_BYTE;
  cmd[1] = settings->upper_threshold;
  ret    = max44009_write_reg8(priv, cmd);
  if (ret < 0)
    {
      max44009_dbg("Cannot set upper threshold\n");
      goto fail;
    }

  cmd[0] = MAX44009_LOW_THRESH_BYTE;
  cmd[1] = settings->lower_threshold;
  ret    = max44009_write_reg8(priv, cmd);
  if (ret < 0)
    {
      max44009_dbg("Cannot set lower threshold\n");
      goto fail;
    }

  ret = max44009_set_threshold_timer(priv, settings->threshold_timer);
  if (ret < 0)
    {
      max44009_dbg("Cannot set threshold timer\n");
      goto fail;
    }

  ret = max44009_set_interrupt_bit(priv, true);
  if (ret < 0)
    {
      max44009_dbg("Cannot set interrupt\n");
      goto fail;
    }

fail:
  return ret;
}

static int max44009_selftest(FAR struct max44009_dev_s *priv,
                             FAR struct max44009_data_s * data)
{
  int ret;
  uint8_t reg_addr = MAX44009_THRESH_TIMER;
  uint8_t value = 0;

  ret = max44009_set_threshold_timer(priv, data->test_value);
  if (ret < 0)
    {
      max44009_dbg("Cannot write test-value\n");
      goto fail;
    }

  ret = max44009_read_reg8(priv, &reg_addr, &value);
  if (ret < 0)
    {
      max44009_dbg("Cannot read written value\n");
      goto fail;
    }

  if (value != data->test_value)
    {
      max44009_dbg("Test failed\n");
      ret = -EIO;
    }

fail:
  return ret;
}

static int max44009_init_device(FAR struct max44009_dev_s *priv,
                                FAR struct max44009_init_s * settings)
{
  int ret;

  ret = max44009_set_manual_mode(priv, settings->is_manual);
  if (ret < 0)
    {
      max44009_dbg("Cannot init manual mode\n");
      goto fail;
    }

  ret = max44009_set_continuous_mode(priv, settings->is_cont);
  if (ret < 0)
    {
      max44009_dbg("Cannot init cont mode\n");
      goto fail;
    }

  if (settings->is_manual)
    {
      ret = max44009_set_current_div_ratio(priv, settings->is_cdr);
      if (ret < 0)
        {
          max44009_dbg("Cannot init cdr mode\n");
          goto fail;
        }

      ret = max44009_set_integration_time(priv, settings->integr_time);
      if (ret < 0)
        {
          max44009_dbg("Cannot init tim mode\n");
          goto fail;
        }
    }

fail:
  return ret;
}

static int max44009_read_data(FAR struct max44009_dev_s *priv,
                              FAR struct max44009_data_s *data)
{
  int ret;
  uint8_t lvalue;
  uint8_t hvalue;
  uint8_t reg_addr;
  uint32_t val;

  reg_addr = MAX44009_INT_STS;
  ret = max44009_read_reg8(priv, &reg_addr, &hvalue);
  if (ret < 0)
    {
      max44009_dbg("Cannot read interrupt status register\n");
    }

  reg_addr = MAX44009_LUX_HBYTE;
  ret = max44009_read_reg8(priv, &reg_addr, &hvalue);
  if (ret < 0)
    {
      max44009_dbg("Cannot read high bits from lux register\n");
      return ret;
    }

  /* LUX HBYTE has (starting with MSB): E3.E2.E1.E0.M7.M6.M5.M4
   * LUX LBYTE has                    : --.--.--.--.M3.M2.M1.M0
   *
   * E[3..0] = Exponent, M[7..0]: Mantissa.
   *
   * Lux can be calculated as (full resolution):
   *     (M[7..0] << E[3..0]) * 0.045.
   *
   * Lux can also be calculated using only HBYTE:
   *     (M[7..4] << E[3..0]) * 0.72
   *       == (M[7..4] << E[3..0]) * 2^4 * 0.045
   *       == (M[7..4] << E[3..0]) * (1 << 4) * 0.045
   *       == (M[7..4] << (E[3..0] + 4)) * 0.045
   */

  reg_addr = MAX44009_LUX_LBYTE;
  ret = max44009_read_reg8(priv, &reg_addr, &lvalue);
  if (ret < 0)
    {
      max44009_dbg("Cannot read low bits from lux register\n");
      return ret;
    }

  /* Merge HBYTE and LBYTE to 16-bit integer:
   *   --.--.--.--.E3.E2.E1.E0.M7.M6.M5.M4.M3.M2.M1.M0
   */

  data->raw_value = (hvalue << 4) | (lvalue & 0xf);

  /* Add raw value to entropy pool. */

  add_sensor_randomness(data->raw_value);

  /* Convert raw value to lux and millilux. */

  val = data->raw_value & 0xff;
  val = val << ((data->raw_value & 0x0f00) >> 8);

  /* lux is the raw output multiplied by 0.045. */

  data->lux  = (val * 45) / 1000;
  data->mlux = (val * 45) % 1000;

  return ret;
}

static int max44009_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct max44009_dev_s *priv;
  int ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct max44009_dev_s *)inode->i_private;

  ret = nxsem_wait_uninterruptible(&priv->dev_sem);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
    case SNIOC_INIT:
      ret = max44009_init_device(priv, (FAR struct max44009_init_s *)arg);
      break;

    case SNIOC_THRESHOLD:
      ret = max44009_set_threshold(priv,
                                   (FAR struct max44009_threshold_s *)arg);
      break;

    case SNIOC_READ_RAW_DATA:
    case SNIOC_READ_CONVERT_DATA:
      ret = max44009_read_data(priv, (FAR struct max44009_data_s *)arg);
      break;

    case SNIOC_START_SELFTEST:
      ret = max44009_selftest(priv, (FAR struct max44009_data_s *)arg);
      break;

    default:
      ret = -ENOTTY;
      break;
    }

  nxsem_post(&priv->dev_sem);
  return ret;
}

static void max44009_notify(FAR struct max44009_dev_s *priv)
{
  DEBUGASSERT(priv != NULL);

  int i;

  /* If there are threads waiting on poll() for data to become available,
   * then wake them up now.  NOTE: we wake up all waiting threads because we
   * do not know that they are going to do.  If they all try to read the
   * data, then some make end up blocking after all.
   */

  for (i = 0; i < CONFIG_MAX44009_NPOLLWAITERS; i++)
    {
      FAR struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= POLLIN;
          max44009_dbg("Report events: %08" PRIx32 "\n", fds->revents);
          nxsem_post(fds->sem);
          priv->int_pending = false;
        }
    }
}

static int max44009_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup)
{
  FAR struct inode *inode;
  FAR struct max44009_dev_s *priv;
  int ret = OK;
  int i;

  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct max44009_dev_s *)inode->i_private;

  ret = nxsem_wait_uninterruptible(&priv->dev_sem);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto out;
        }

      /* This is a request to set up the poll.  Find an available slot for
       * the poll structure reference.
       */

      for (i = 0; i < CONFIG_MAX44009_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_MAX44009_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
          goto out;
        }

      if (priv->int_pending)
        {
          max44009_notify(priv);
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);

      /* Remove all memory of the poll setup */

      *slot = NULL;
      fds->priv = NULL;
    }

out:
  nxsem_post(&priv->dev_sem);
  return ret;
}

static int max44009_int_handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct max44009_dev_s *priv = (FAR struct max44009_dev_s *)arg;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  flags = enter_critical_section();
  priv->int_pending = true;
  leave_critical_section(flags);
  max44009_notify(priv);
  max44009_dbg("MAX44009 interrupt\n");

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int max44009_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                      uint8_t addr, FAR struct max44009_config_s *config)
{
  struct max44009_dev_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(struct max44009_dev_s));
  if (!priv)
    {
      max44009_dbg("Memory cannot be allocated for ALS sensor\n");
      return -ENOMEM;
    }

  priv->addr = addr;
  priv->i2c = i2c;
  priv->config = config;
  nxsem_init(&priv->dev_sem, 0, 1);

  ret = register_driver(devpath, &g_alsops, 0666, priv);
  max44009_dbg("Registered with %d\n", ret);
  if (ret < 0)
    {
      kmm_free(priv);
      max44009_dbg("Error occurred during the driver registering\n");
      return ret;
    }

  priv->config->irq_attach(priv->config, max44009_int_handler, priv);
  priv->config->irq_enable(priv->config, false);
  return ret;
}
