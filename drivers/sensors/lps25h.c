/****************************************************************************
 * drivers/sensors/lps25h.c
 *
 *   Copyright (C) 2014-2017 Haltian Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <sys/types.h>
#include <debug.h>
#include <stdio.h>
#include <errno.h>
#include <nuttx/kmalloc.h>
#include <nuttx/random.h>

#include <nuttx/sensors/lps25h.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_LPS25H
#  define lps25h_dbg(x, ...)      _info(x, ##__VA_ARGS__)
#else
#  define lps25h_dbg(x, ...)      sninfo(x, ##__VA_ARGS__)
#endif

#ifndef CONFIG_LPS25H_I2C_FREQUENCY
#  define CONFIG_LPS25H_I2C_FREQUENCY     400000
#endif

#define LPS25H_PRESSURE_INTERNAL_DIVIDER  4096

/* 'AN4450 - Hardware and software guidelines for use of LPS25H pressure
 * sensors' - '6.2 One-shot mode conversion time estimation' gives estimates
 * for conversion times:
 *
 * Typical conversion time ≈ 62*(Pavg+Tavg) + 975 μs
 *  ex: Tavg = 64; Pavg = 512; Typ. conversation time ≈ 36.7 ms (compatible with
 *                                                               ODT=25 Hz)
 *  ex: Tavg = 32; Pavg = 128; Typ. conversation time ≈ 10.9 ms
 *  The formula is accurate within +/- 3% at room temperature
 *
 * Set timeout to 2 * max.conversation time (2*36.7*1.03 = 76 ms).
 */

#define LPS25H_RETRY_TIMEOUT_MSECS        76
#define LPS25H_MAX_RETRIES                5

#define LPS25H_I2C_RETRIES                10

/* Registers */

#define LPS25H_REF_P_XL         0x08
#define LPS25H_REF_P_L          0x09
#define LPS25H_REF_P_H          0x0a
#define LPS25H_WHO_AM_I         0x0f
#define LPS25H_RES_CONF         0x10
#define LPS25H_CTRL_REG1        0x20
#define LPS25H_CTRL_REG2        0x21
#define LPS25H_CTRL_REG3        0x22
#define LPS25H_CTRL_REG4        0x23
#define LPS25H_INT_CFG          0x24
#define LPS25H_INT_SOURCE       0x25
#define LPS25H_STATUS_REG       0x27
#define LPS25H_PRESS_POUT_XL    0x28
#define LPS25H_PRESS_OUT_L      0x29
#define LPS25H_PRESS_OUT_H      0x2a
#define LPS25H_TEMP_OUT_L       0x2b
#define LPS25H_TEMP_OUT_H       0x2c
#define LPS25H_FIFO_CTRL        0x2e
#define LPS25H_FIFO_STATUS      0x2f
#define LPS25H_THS_P_L          0x30
#define LPS25H_THS_P_H          0x31
#define LPS25H_RPDS_L           0x39
#define LPS25H_RPDS_H           0x3a

/* Bits in registers */

#define LPS25H_AUTO_ZERO        (1 << 2)
#define LPS25H_BDU              (1 << 2)
#define LPS25H_DIFF_EN          (1 << 3)
#define LPS25H_FIFO_EN          (1 << 6)
#define LPS25H_WTM_EN           (1 << 5)
#define LPS25H_FIFO_MEAN_DEC    (1 << 4)
#define LPS25H_PD               (1 << 7)
#define LPS25H_ONE_SHOT         (1 << 0)
#define LPS25H_INT_H_L          (1 << 7)
#define LPS25H_PP_OD            (1 << 6)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct lps25h_dev_s
{
  struct i2c_master_s *i2c;
  uint8_t addr;
  bool irqenabled;
  volatile bool int_pending;
  sem_t devsem;
  sem_t waitsem;
  lps25h_config_t *config;
};

enum LPS25H_RES_CONF_AVG_PRES
{
  PRES_AVG_8 = 0,
  PRES_AVG_32,
  PRES_AVG_128,
  PRES_AVG_512
};

enum LPS25H_RES_CONF_AVG_TEMP
{
  TEMP_AVG_8 = 0,
  TEMP_AVG_16,
  TEMP_AVG_32,
  TEMP_AVG_64
};

enum LPS25H_CTRL_REG1_ODR
{
  CTRL_REG1_ODR_ONE_SHOT = 0,
  CTRL_REG1_ODR_1Hz,
  CTRL_REG1_ODR_7Hz,
  CTRL_REG1_ODR_12_5Hz,
  CTRL_REG1_ODR_25Hz
};

enum LPS25H_CTRL_REG4_P1
{
  P1_DRDY = 0x1,
  P1_OVERRUN = 0x02,
  P1_WTM = 0x04,
  P1_EMPTY = 0x08
};

enum LPS25H_FIFO_CTRL_MODE
{
  BYPASS_MODE = 0x0,
  FIFO_STOP_WHEN_FULL,
  STREAM_NEWEST_IN_FIFO,
  STREAM_DEASSERTED,
  BYPASS_DEASSERTED_STREAM,
  FIFO_MEAN = 0x06,
  BYPASS_DEASSERTED_FIFO
};

enum LPS25H_FIFO_CTRL_WTM
{
  SAMPLE_2 = 0x01,
  SAMPLE_4 = 0x03,
  SAMPLE_8 = 0x07,
  SAMPLE_16 = 0x0f,
  SAMPLE_32 = 0x1f
};

enum LPS25H_INT_CFG_OP
{
  PH_E = 0x1,
  PL_E = 0x2,
  LIR = 0x4
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

static int lps25h_open(FAR struct file *filep);
static int lps25h_close(FAR struct file *filep);
static ssize_t lps25h_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t lps25h_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int lps25h_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

static int lps25h_configure_dev(FAR struct lps25h_dev_s *dev);
static int lps25h_read_pressure(FAR struct lps25h_dev_s *dev,
                                FAR lps25h_pressure_data_t *pres);
static int lps25h_read_temper(FAR struct lps25h_dev_s *dev,
                              FAR lps25h_temper_data_t *temper);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_lps25hops =
{
  lps25h_open,   /* open */
  lps25h_close,  /* close */
  lps25h_read,   /* read */
  lps25h_write,  /* write */
  NULL,          /* seek */
  lps25h_ioctl   /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , NULL         /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL         /* unlink */
 #endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int lps25h_do_transfer(FAR struct lps25h_dev_s *dev,
                              FAR struct i2c_msg_s *msgv,
                              size_t nmsg)
{
  int ret = -EIO;
  int retries;

  for (retries = 0; retries < LPS25H_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(dev->i2c, msgv, nmsg);
      if (ret >= 0)
        {
          return 0;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */
#ifdef CONFIG_I2C_RESET
          if (retries == LPS25H_I2C_RETRIES - 1)
            {
              break;
            }

          ret = I2C_RESET(dev->i2c);
          if (ret < 0)
            {
              lps25h_dbg("I2C_RESET failed: %d\n", ret);
              return ret;
            }
#endif
        }
    }

  lps25h_dbg("xfer failed: %d\n", ret);
  return ret;
}

static int lps25h_write_reg8(struct lps25h_dev_s *dev, uint8_t reg_addr,
                             const uint8_t value)
{
  struct i2c_msg_s msgv[2] =
  {
    {
      .frequency = CONFIG_LPS25H_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = 0,
      .buffer    = &reg_addr,
      .length    = 1
    },
    {
      .frequency = CONFIG_LPS25H_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = I2C_M_NOSTART,
      .buffer    = (void *)&value,
      .length    = 1
    }
  };

  return lps25h_do_transfer(dev, msgv, 2);
}

static int lps25h_read_reg8(FAR struct lps25h_dev_s *dev,
                            FAR uint8_t *reg_addr,
                            FAR uint8_t *value)
{
  struct i2c_msg_s msgv[2] =
  {
    {
      .frequency = CONFIG_LPS25H_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = 0,
      .buffer    = reg_addr,
      .length    = 1
    },
    {
      .frequency = CONFIG_LPS25H_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = I2C_M_READ,
      .buffer    = value,
      .length    = 1
    }
  };

  return lps25h_do_transfer(dev, msgv, 2);
}

static int lps25h_power_on_off(FAR struct lps25h_dev_s *dev, bool on)
{
  int ret;
  uint8_t value;

  value = on ? LPS25H_PD : 0;
  ret = lps25h_write_reg8(dev, LPS25H_CTRL_REG1, value);
  return ret;
}

static int lps25h_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lps25h_dev_s *dev = inode->i_private;
  uint8_t value = 0;
  uint8_t addr = LPS25H_WHO_AM_I;
  int32_t ret;

  /* Get exclusive access */

  do
    {
      ret = nxsem_wait(&dev->devsem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);

  dev->config->set_power(dev->config, true);
  ret = lps25h_read_reg8(dev, &addr, &value);
  if (ret < 0)
    {
      lps25h_dbg("Cannot read device's ID\n");
      dev->config->set_power(dev->config, false);
      goto out;
    }

  lps25h_dbg("WHO_AM_I: 0x%2x\n", value);

  dev->config->irq_enable(dev->config, true);
  dev->irqenabled = true;

out:
  nxsem_post(&dev->devsem);
  return ret;
}

static int lps25h_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lps25h_dev_s *dev = inode->i_private;
  int ret;

  /* Get exclusive access */

  do
    {
      ret = nxsem_wait(&dev->devsem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);

  dev->config->irq_enable(dev->config, false);
  dev->irqenabled = false;
  ret = lps25h_power_on_off(dev, false);
  dev->config->set_power(dev->config, false);
  lps25h_dbg("CLOSED\n");

  nxsem_post(&dev->devsem);
  return ret;
}

static ssize_t lps25h_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lps25h_dev_s *dev = inode->i_private;
  lps25h_pressure_data_t data;
  ssize_t length = 0;
  int ret;

  /* Get exclusive access */

  do
    {
      ret = nxsem_wait(&dev->devsem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);

  ret = lps25h_configure_dev(dev);
  if (ret < 0)
    {
      lps25h_dbg("cannot configure sensor: %d\n", ret);
      goto out;
    }

  ret = lps25h_read_pressure(dev, &data);
  if (ret < 0)
    {
      lps25h_dbg("cannot read data: %d\n", ret);
    }
  else
    {
      /* This interface is mainly intended for easy debugging in nsh. */

      length = snprintf(buffer, buflen, "%u\n", data.pressure_Pa);
      if (length > buflen)
        {
          length = buflen;
        }
    }

out:
  nxsem_post(&dev->devsem);
  return length;
}

static ssize_t lps25h_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  ssize_t length = 0;

  return length;
}

static void lps25h_notify(FAR struct lps25h_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);

  dev->int_pending = true;
  nxsem_post(&dev->waitsem);
}

static int lps25h_int_handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct lps25h_dev_s *dev = (FAR struct lps25h_dev_s *)arg;

  DEBUGASSERT(dev != NULL);

  lps25h_notify(dev);
  lps25h_dbg("lps25h interrupt\n");
  return OK;
}

static int lps25h_configure_dev(FAR struct lps25h_dev_s *dev)
{
  int ret = 0;

  ret = lps25h_power_on_off(dev, false);
  if (ret < 0)
    {
      return ret;
    }

  /* Enable FIFO */

  ret = lps25h_write_reg8(dev, LPS25H_CTRL_REG2, LPS25H_FIFO_EN);
  if (ret < 0)
    {
      return ret;
    }

  ret = lps25h_write_reg8(dev, LPS25H_FIFO_CTRL, (BYPASS_MODE << 5));
  if (ret < 0)
    {
      return ret;
    }

  ret = lps25h_write_reg8(dev, LPS25H_CTRL_REG4, P1_DRDY);
  if (ret < 0)
    {
      return ret;
    }

  /* Write CTRL_REG1 to turn device on */

  ret = lps25h_write_reg8(dev, LPS25H_CTRL_REG1,
                          LPS25H_PD | (CTRL_REG1_ODR_1Hz << 4));

  return ret;
}

static int lps25h_one_shot(FAR struct lps25h_dev_s *dev)
{
  int ret = ERROR;
  int retries;
  struct timespec abstime;
  irqstate_t flags;

  if (!dev->irqenabled)
    {
      lps25h_dbg("IRQ disabled!\n");
    }

  /* Retry one-shot measurement multiple times. */

  for (retries = 0; retries < LPS25H_MAX_RETRIES; retries++)
    {
      /* Power off so we start from a known state. */

      ret = lps25h_power_on_off(dev, false);
      if (ret < 0)
        {
          return ret;
        }

      /* Initiate a one shot mode measurement */

      ret = lps25h_write_reg8(dev, LPS25H_CTRL_REG2, LPS25H_ONE_SHOT);
      if (ret < 0)
        {
          return ret;
        }

      /* Power on to start measurement. */

      ret = lps25h_power_on_off(dev, true);
      if (ret < 0)
        {
          return ret;
        }

      (void)clock_gettime(CLOCK_REALTIME, &abstime);
      abstime.tv_sec += (LPS25H_RETRY_TIMEOUT_MSECS / 1000);
      abstime.tv_nsec += (LPS25H_RETRY_TIMEOUT_MSECS % 1000) * 1000 * 1000;
      while (abstime.tv_nsec >= (1000 * 1000 * 1000))
        {
          abstime.tv_sec++;
          abstime.tv_nsec -= 1000 * 1000 * 1000;
        }

      while ((ret = nxsem_timedwait(&dev->waitsem, &abstime)) < 0)
        {
          if (ret == -EINTR)
            {
              continue;
            }
          else if (ret == -ETIMEDOUT)
            {
              uint8_t reg = LPS25H_CTRL_REG2;
              uint8_t value;

              /* In 'AN4450 - Hardware and software guidelines for use of
               * LPS25H pressure sensors' - '4.3 One-shot mode measurement
               * sequence', one-shot mode example is given where interrupt line
               * is not used, but CTRL_REG2 is polled until ONE_SHOT bit is
               * unset (as it is self-clearing). Check ONE_SHOT bit status here
               * to see if we just missed interrupt.
               */

              ret = lps25h_read_reg8(dev, &reg, &value);
              if (ret < 0)
                {
                  break;
                }

              if ((value & LPS25H_ONE_SHOT) == 0)
                {
                  /* One-shot completed. */

                  ret = OK;
                  break;
                }
            }
          else
            {
              /* Some unknown mystery error */

              DEBUGASSERT(false);
              return ret;
            }
        }

      if (ret == OK)
        {
          break;
        }

      lps25h_dbg("Retrying one-shot measurement: retries=%d\n", retries);
    }

  if (ret != OK)
    {
       return -ETIMEDOUT;
    }

  flags = enter_critical_section();
  dev->int_pending = false;
  leave_critical_section(flags);

  return ret;
}

static int lps25h_read_pressure(FAR struct lps25h_dev_s *dev,
                                FAR lps25h_pressure_data_t *pres)
{
  int ret;
  uint8_t pres_addr_h = LPS25H_PRESS_OUT_H;
  uint8_t pres_addr_l = LPS25H_PRESS_OUT_L;
  uint8_t pres_addr_xl = LPS25H_PRESS_POUT_XL;
  uint8_t pres_value_h = 0;
  uint8_t pres_value_l = 0;
  uint8_t pres_value_xl = 0;
  int32_t pres_res = 0;

  ret = lps25h_one_shot(dev);
  if (ret < 0)
    {
      return ret;
    }

  ret = lps25h_read_reg8(dev, &pres_addr_h, &pres_value_h);
  if (ret < 0)
    {
      return ret;
    }

  ret = lps25h_read_reg8(dev, &pres_addr_l, &pres_value_l);
  if (ret < 0)
    {
      return ret;
    }

  ret = lps25h_read_reg8(dev, &pres_addr_xl, &pres_value_xl);
  if (ret < 0)
    {
      return ret;
    }

  pres_res = ((int32_t) pres_value_h << 16) |
             ((int16_t) pres_value_l << 8) |
             pres_value_xl;

  /* Add to entropy pool. */

  add_sensor_randomness(pres_res);

  /* Convert to more usable format. */

  pres->pressure_int_hP = pres_res / LPS25H_PRESSURE_INTERNAL_DIVIDER;
  pres->pressure_Pa = (uint64_t)pres_res * 100000 / LPS25H_PRESSURE_INTERNAL_DIVIDER;
  pres->raw_data = pres_res;
  lps25h_dbg("Pressure: %u Pa\n", pres->pressure_Pa);

  return ret;
}

static int lps25h_read_temper(FAR struct lps25h_dev_s *dev,
                              FAR lps25h_temper_data_t *temper)
{
  int ret;
  uint8_t temper_addr_h = LPS25H_TEMP_OUT_H;
  uint8_t temper_addr_l = LPS25H_TEMP_OUT_L;
  uint8_t temper_value_h = 0;
  uint8_t temper_value_l = 0;
  int32_t temper_res;
  int16_t raw_data;

  ret = lps25h_read_reg8(dev, &temper_addr_h, &temper_value_h);
  if (ret < 0)
    {
      return ret;
    }

  ret = lps25h_read_reg8(dev, &temper_addr_l, &temper_value_l);
  if (ret < 0)
    {
      return ret;
    }

  raw_data = (temper_value_h << 8) | temper_value_l;

  /* Add to entropy pool. */

  add_sensor_randomness(raw_data);

  /* T(⁰C) = 42.5 + (raw / 480)
   * =>
   * T(⁰C) * scale = (425 * 48 + raw) * scale / 480;
   */

  temper_res = (425 * 48 + raw_data);
  temper_res *= LPS25H_TEMPER_DIVIDER;
  temper_res /= 480;

  temper->int_temper = temper_res;
  temper->raw_data = raw_data;
  lps25h_dbg("Temperature: %d\n", temper_res);

  return ret;
}

static int lps25h_who_am_i(struct lps25h_dev_s *dev,
                           lps25h_who_am_i_data * who_am_i_data)
{
  uint8_t who_addr = LPS25H_WHO_AM_I;
  return lps25h_read_reg8(dev, &who_addr, &who_am_i_data->who_am_i);
}

static int lps25h_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lps25h_dev_s *dev = inode->i_private;
  int ret;

  /* Get exclusive access */

  do
    {
      ret = nxsem_wait(&dev->devsem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);

  switch (cmd)
    {
    case SNIOC_CFGR:
      ret = lps25h_configure_dev(dev);
      break;

    case SNIOC_PRESSURE_OUT:
      ret = lps25h_read_pressure(dev, (lps25h_pressure_data_t *) arg);
      break;

    case SNIOC_TEMPERATURE_OUT:
      /* NOTE: call SNIOC_PRESSURE_OUT before this one,
       * or results are bogus.
       */

      ret = lps25h_read_temper(dev, (lps25h_temper_data_t *) arg);
      break;

    case SNIOC_SENSOR_OFF:
      ret = lps25h_power_on_off(dev, false);
      break;

    case SNIOC_GET_DEV_ID:
      ret = lps25h_who_am_i(dev, (lps25h_who_am_i_data *) arg);
      break;

    default:
      ret = -ENOTTY;
      break;
    }

  nxsem_post(&dev->devsem);
  return ret;
}

int lps25h_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                    uint8_t addr, FAR lps25h_config_t *config)
{
  int ret = 0;
  FAR struct lps25h_dev_s *dev;

  dev = (struct lps25h_dev_s *)kmm_zalloc(sizeof(struct lps25h_dev_s));
  if (!dev)
    {
      lps25h_dbg("Memory cannot be allocated for LPS25H sensor\n");
      return -ENOMEM;
    }

  nxsem_init(&dev->devsem, 0, 1);
  nxsem_init(&dev->waitsem, 0, 0);

  dev->addr = addr;
  dev->i2c = i2c;
  dev->config = config;

  if (dev->config->irq_clear)
    {
      dev->config->irq_clear(dev->config);
    }

  ret = register_driver(devpath, &g_lps25hops, 0666, dev);

  lps25h_dbg("Registered with %d\n", ret);

  if (ret < 0)
    {
      kmm_free(dev);
      lps25h_dbg("Error occurred during the driver registering\n");
      return ERROR;
    }

  dev->config->irq_attach(config, lps25h_int_handler, dev);
  dev->config->irq_enable(config, false);
  dev->irqenabled = false;
  return OK;
}
