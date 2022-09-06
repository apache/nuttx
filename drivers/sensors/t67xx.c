/****************************************************************************
 * drivers/sensors/t67xx.c
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

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <time.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/random.h>

#include <nuttx/sensors/t67xx.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_I2C)
#  error i2c support required
#endif

/* This driver is tested with Telaire T6713. Likely works for Telaire T6703
 * and other models as well, but this has not been verified. No test
 * animals were gassed during testing.
 */

/* Modbus registers */

#define T67XX_REG_FWREV           0x1389  /* Firmware revision */
#define T67XX_REG_STATUS          0x138a  /* Status */
#define T67XX_REG_GASPPM          0x138b  /* Gas parts per million */
#define T67XX_REG_RESET           0x03e8  /* Reset device */
#define T67XX_REG_SPCAL           0x03ec  /* Single point calibration */
#define T67XX_REG_SLAVEADDR       0x0fa5  /* Slave address */
#define T67XX_REG_ABCLOGIC        0x03ee  /* ABC Logic enable/disable */

/* Status register bits */

#define T67XX_STATUS_ERROR        (1 << 0)  /* Error condition */
#define T67XX_STATUS_FLASH_ERROR  (1 << 1)  /* Flash error */
#define T67XX_STATUS_CALIB_ERROR  (1 << 2)  /* Calibration error */
                                            /* [3:7] reserved */
#define T67XX_STATUS_RS232        (1 << 8)  /* RS-232 error */
#define T67XX_STATUS_RS485        (1 << 9)  /* RS-485 error */
#define T67XX_STATUS_I2C          (1 << 10) /* I2C error */
#define T67XX_STATUS_WARMUP       (1 << 11) /* Warm-up mode */
                                            /* [12:14] reserved */
#define T67XX_STATUS_SPCAL        (1 << 15) /* Single point calibration */

/* Command bits */

#define RESET_SENSOR              0xff00    /* Reset sensor */

#define SPCAL_START               0xff00    /* Start SP calibration */
#define SPCAL_STOP                0x0000    /* Stop SP calibration */

#define ABCLOGIC_ENABLE           0xff00    /* Enable ABC Logic */
#define ABCLOGIC_DISABLE          0x0000    /* Disable ABC Logic */

/* Required times before sensor reaches either minimal or full
 * operational accuracy.
 */

#define T67XX_UPTIME_MINIMAL_SEC  120             /* two minutes */

#define T67XX_UPTIME_FULL_SEC     (10 * 60)       /* ten minutes */

#define T67XX_UPTIME_ABCLOGIC_SEC (24 * 60 * 60)  /* a day */

/* I2C constants */

#define T67XX_I2C_FREQUENCY       100000    /* Only supported I2C speed */

#define T67XX_I2C_DELAY_MSEC      10        /* 5-10 ms recommended, 10 ms
                                             * is always safe. */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct t67xx_dev_s
{
  FAR struct i2c_master_s *i2c;  /* I2C interface */
  uint8_t addr;                  /* I2C address */
  struct timespec boot_time;     /* When sensor was booted */
  mutex_t devlock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int t67xx_read16(FAR struct t67xx_dev_s *priv, uint16_t regaddr,
                        FAR uint16_t *regvalue);
static int t67xx_write16(FAR struct t67xx_dev_s *priv, uint16_t regaddr,
                         FAR uint16_t regvalue, bool read_reply);

/* Driver features */

static int t67xx_check_status(FAR struct t67xx_dev_s *priv,
                              bool *warming_up, bool *calibrating);
static int t67xx_read_fwrev(FAR struct t67xx_dev_s *priv, uint16_t *rev);
static int t67xx_read_gas_ppm(FAR struct t67xx_dev_s *priv,
                              FAR struct t67xx_value_s *buffer);
static int t67xx_spcal(FAR struct t67xx_dev_s *priv, bool start);
static int t67xx_abclogic(FAR struct t67xx_dev_s *priv, bool enable);
static int t67xx_reset(FAR struct t67xx_dev_s *priv);

/* Character driver methods */

static ssize_t t67xx_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t t67xx_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int     t67xx_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_t67xxfops =
{
  NULL,           /* open */
  NULL,           /* close */
  t67xx_read,     /* read */
  t67xx_write,    /* write */
  NULL,           /* seek */
  t67xx_ioctl,    /* ioctl */
  NULL            /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL          /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t67xx_read16
 *
 * Description:
 *   Read 16-bit register
 *
 ****************************************************************************/

static int t67xx_read16(FAR struct t67xx_dev_s *priv, uint16_t regaddr,
                        FAR uint16_t *regvalue)
{
  struct i2c_config_s config;
  uint8_t buf[5];
  uint8_t rxbuf[4] =
  {
    0
  };

  int ret;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(regvalue != NULL);

  /* Set up the I2C configuration. */

  config.frequency = T67XX_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Set up the Modbus read request. */

  buf[0] = 4;
  buf[1] = (regaddr >> 8) & 0xff;
  buf[2] =  regaddr       & 0xff;
  buf[3] = 0;
  buf[4] = 1;

  /* Write the Modbus read request. */

  ret = i2c_write(priv->i2c, &config, buf, sizeof(buf));
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Wait and read response. */

  up_mdelay(T67XX_I2C_DELAY_MSEC);

  ret = i2c_read(priv->i2c, &config, rxbuf, sizeof(rxbuf));
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  if (rxbuf[0] != 4 || rxbuf[1] != 2)
    {
#ifdef CONFIG_DEBUG_SENSORS_ERROR
      lib_dumpbuffer("ERROR: sensor wrong data", rxbuf, sizeof(rxbuf));
#endif
      return -EIO;
    }

  *regvalue = ((uint16_t)rxbuf[2] << 8) | rxbuf[3];
  return ret;
}

/****************************************************************************
 * Name: t67xx_write16
 *
 * Description:
 *   Write 16-bit register. If 'reply' is true, sensor response is read
 *   for verification.
 *
 ****************************************************************************/

static int t67xx_write16(FAR struct t67xx_dev_s *priv, uint16_t regaddr,
                         FAR uint16_t regvalue, bool reply)
{
  struct i2c_config_s config;
  uint8_t buf[5];
  uint8_t rxbuf[5] =
  {
    0
  };

  int ret;

  DEBUGASSERT(priv != NULL);

  /* Set up the I2C configuration. */

  config.frequency = T67XX_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Set up the Modbus write request. */

  buf[0] = 5;
  buf[1] = (regaddr >> 8)  & 0xff;
  buf[2] =  regaddr        & 0xff;
  buf[3] = (regvalue >> 8) & 0xff;
  buf[4] =  regvalue       & 0xff;

  /* Write the Modbus write request. */

  ret = i2c_write(priv->i2c, &config, buf, sizeof(buf));
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* There is no reply for Reset command, exit early. */

  if (!reply)
    {
      return ret;
    }

  /* Wait and read response. */

  up_mdelay(T67XX_I2C_DELAY_MSEC);

  ret = i2c_read(priv->i2c, &config, rxbuf, sizeof(rxbuf));
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  if (memcmp(rxbuf, buf, sizeof(rxbuf)) != 0)
    {
#ifdef CONFIG_DEBUG_SENSORS_ERROR
      lib_dumpbuffer("ERROR: sensor wrong data", rxbuf, sizeof(rxbuf));
#endif
      return -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: t67xx_check_status
 *
 * Description:
 *   Check status register.
 *
 ****************************************************************************/

static int t67xx_check_status(FAR struct t67xx_dev_s *priv,
                              bool *warming_up, bool *calibrating)
{
  uint16_t status;
  int ret;

  /* Read the status register. */

  ret = t67xx_read16(priv, T67XX_REG_STATUS, &status);
  if (ret < 0)
    {
      snerr("ERROR: sensor cannot read status: %d\n", ret);
      return ret;
    }

  sninfo("status 0x%04x\n", status);

  /* We ignore I2C error here, as it is checked for every bus
   * operation anyway.
   */

  if (status & (T67XX_STATUS_ERROR |
                T67XX_STATUS_FLASH_ERROR | T67XX_STATUS_CALIB_ERROR))
    {
      snerr("ERROR: sensor error 0x%04x\n", status);
      return ERROR;
    }

  *warming_up  = status & T67XX_STATUS_WARMUP;
  *calibrating = status & T67XX_STATUS_SPCAL;

  return OK;
}

/****************************************************************************
 * Name: t67xx_read_fwrev
 *
 * Description:
 *   Check firmware revision register.
 *
 ****************************************************************************/

static int t67xx_read_fwrev(FAR struct t67xx_dev_s *priv, uint16_t *rev)
{
  uint16_t fwrev;
  int ret;

  /* Read the FW revision register. */

  ret = t67xx_read16(priv, T67XX_REG_FWREV, &fwrev);
  if (ret < 0)
    {
      snerr("ERROR: sensor cannot read reg: %d\n", ret);
      return ret;
    }

  sninfo("sensor FW rev: 0x%04x\n", fwrev);
  if (rev)
    {
      *rev = fwrev;
    }

  return OK;
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
 * Name: t67xx_read_gas_ppm
 *
 * Description:
 *   Read the current carbon dioxide level.
 *
 ****************************************************************************/

static int t67xx_read_gas_ppm(FAR struct t67xx_dev_s *priv,
                              FAR struct t67xx_value_s *buffer)
{
  uint16_t ppm;
  bool warming_up;
  bool calibrating;
  struct timespec ts;
  int ret;

  clock_systime_timespec(&ts);

  if (!has_time_passed(ts, priv->boot_time, T67XX_UPTIME_MINIMAL_SEC))
    {
      snwarn("WARN: sensor not ready yet\n");
      return -EAGAIN;
    }

  /* Check sensor status. */

  ret = t67xx_check_status(priv, &warming_up, &calibrating);
  if (ret != OK)
    {
      return ret;
    }

  /* Sanity check warm-up behavior. */

  if (!has_time_passed(ts, priv->boot_time, T67XX_UPTIME_FULL_SEC))
    {
      if (!warming_up)
        {
          snwarn("WARN: sensor not fully warm-up\n");
          warming_up = true;
        }
    }
  else if (warming_up)
    {
      snwarn("WARN: sensor still warming up after %d secs\n",
              T67XX_UPTIME_FULL_SEC);
    }

  /* Read the CO2 level. */

  ret = t67xx_read16(priv, T67XX_REG_GASPPM, &ppm);
  if (ret < 0)
    {
      return ret;
    }

  add_sensor_randomness(ts.tv_nsec ^ (int)ppm);

  buffer->gas_ppm     = ppm;
  buffer->warming_up  = warming_up;
  buffer->calibrating = calibrating;

  sninfo("ppm %d, warming up %d, calibrating %d\n",
         (int)ppm, warming_up, calibrating);

  return OK;
}

/****************************************************************************
 * Name: t67xx_spcal
 *
 * Description:
 *   Start or stop sensor single-point calibration.
 *
 ****************************************************************************/

static int t67xx_spcal(FAR struct t67xx_dev_s *priv, bool start)
{
  int ret;

  ret = t67xx_write16(priv, T67XX_REG_SPCAL,
                      start ? SPCAL_START : SPCAL_STOP, true);
  if (ret < 0)
    {
      snerr("ERROR: sp calibration failed: %d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: t67xx_abclogic
 *
 * Description:
 *   Start or stop sensor ABC Logic feature.
 *
 *   Automatic Background Logic, is a self-calibration technique that is
 *   designed to be used in applications where concentrations will drop to
 *   outside ambient conditions (400 ppm) at least three times in a 7 days.
 *
 *   With ABC Logic enabled, the sensor will typically reach its operational
 *   accuracy after 24 hours of continuous operation at a condition that it
 *   was exposed to ambient reference levels of air at 400 ppm CO2. Sensor
 *   will maintain accuracy specifications with ABC Logic enabled, given
 *   that it is at least four times in 21 days exposed to the reference
 *   value and this reference value is the lowest concentration to which
 *   the sensor is exposed. ABC Logic requires continuous operation of the
 *   sensor for periods of at least 24 hours.
 *
 *   Currently driver does not try to check for these constraints.
 *
 ****************************************************************************/

static int t67xx_abclogic(FAR struct t67xx_dev_s *priv, bool enable)
{
  int ret;

  ret = t67xx_write16(priv, T67XX_REG_ABCLOGIC,
                      enable ? ABCLOGIC_ENABLE : ABCLOGIC_DISABLE, true);
  if (ret < 0)
    {
      snerr("ERROR: ABC Logic failed: %d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: t67xx_reset
 *
 * Description:
 *   Reset sensor. Data-sheet does not recommend using this.
 *
 ****************************************************************************/

static int t67xx_reset(FAR struct t67xx_dev_s *priv)
{
  int ret;

  ret = t67xx_write16(priv, T67XX_REG_RESET, RESET_SENSOR, false);
  if (ret < 0)
    {
      snerr("ERROR: reset failed: %d\n", ret);
      return ret;
    }

  /* Sensor uptime starting again from zero. */

  clock_systime_timespec(&priv->boot_time);

  return ret;
}

/****************************************************************************
 * Name: t67xx_read
 ****************************************************************************/

static ssize_t t67xx_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct inode         *inode = filep->f_inode;
  FAR struct t67xx_dev_s   *priv  = inode->i_private;
  FAR struct t67xx_value_s *ptr;
  ssize_t nsamples;
  int i;
  int ret;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* How many samples were requested to get? */

  nsamples = buflen / sizeof(struct t67xx_value_s);
  ptr      = (FAR struct t67xx_value_s *)buffer;

  sninfo("buflen: %d nsamples: %d\n", buflen, nsamples);

  /* Get the requested number of samples */

  for (i = 0; i < nsamples; i++)
    {
      struct t67xx_value_s gas_ppm;

      /* Read the next struct t67xx_value_s */

      ret = t67xx_read_gas_ppm(priv, &gas_ppm);
      if (ret < 0)
        {
          snerr("ERROR: t67xx_read_gas_ppm failed: %d\n", ret);
          nxmutex_unlock(&priv->devlock);
          return (ssize_t)ret;
        }

      /* Save the value in the user buffer */

      *ptr++ = gas_ppm;
    }

  nxmutex_unlock(&priv->devlock);
  return nsamples * sizeof(struct t67xx_value_s);
}

/****************************************************************************
 * Name: t67xx_write
 ****************************************************************************/

static ssize_t t67xx_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: t67xx_ioctl
 ****************************************************************************/

static int t67xx_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode       *inode = filep->f_inode;
  FAR struct t67xx_dev_s *priv  = inode->i_private;
  int ret;

  /* Get exclusive access */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
      /* Soft reset the sensor, Arg: None */

      case SNIOC_RESET:
        ret = t67xx_reset(priv);
        sninfo("reset ret: %d\n", ret);
        break;

      /* Dump registers, currently just FWREV reg, Arg: None */

      case SNIOC_DUMP_REGS:
        ret = t67xx_read_fwrev(priv, NULL);
        break;

      /* Calibrate the sensor, Arg: uint8_t value */

      case SNIOC_SPCALIB:
        ret = t67xx_spcal(priv, !!arg);
        sninfo("spcal ret: %d\n", ret);
        break;

      /* Enable/disable the ABC Logic feature, Arg: uint8_t value */

      case SNIOC_ABCLOGIC:
        ret = t67xx_abclogic(priv, !!arg);
        sninfo("abclogic ret: %d\n", ret);
        break;

      default:
        sninfo("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t67xx_register
 *
 * Description:
 *   Register the T67XX character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/co2_0"
 *   i2c - An instance of the I2C interface to use to communicate with T67XX
 *   addr - The I2C address of the T67XX. For T6713 this is initially 0x15
 *          but it can be changed by the SLAVE ADDRESS command.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int t67xx_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                   uint8_t addr)
{
  FAR struct t67xx_dev_s *priv;
  int ret;

  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(addr == T67XX_I2C_ADDR);

  /* Initialize the t67xx device structure. */

  priv = (FAR struct t67xx_dev_s *)kmm_malloc(sizeof(struct t67xx_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;

  nxmutex_init(&priv->devlock);

  clock_systime_timespec(&priv->boot_time);

  /* Register the character driver. */

  ret = register_driver(devpath, &g_t67xxfops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      goto errout;
    }

  sninfo("(addr=0x%02x) registered at %s\n", priv->addr, devpath);
  return ret;

errout:
  kmm_free(priv);
  return ret;
}
