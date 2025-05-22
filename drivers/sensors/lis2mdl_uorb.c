/****************************************************************************
 * drivers/sensors/lis2mdl_uorb.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with
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
#include <nuttx/nuttx.h>

#include <debug.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>

#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/random.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/lis2mdl.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/signal.h>
#include <nuttx/wqueue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The value that should be in the "who am I" register */

#define WHO_AM_I_VAL 0x40

/* Registers */

#define REG_OFFSET_X_REG_L 0x45
#define REG_OFFSET_X_REG_H 0x46
#define REG_OFFSET_Y_REG_L 0x47
#define REG_OFFSET_Y_REG_H 0x48
#define REG_OFFSET_Z_REG_L 0x49
#define REG_OFFSET_Z_REG_H 0x4a
#define REG_WHO_AM_I 0x4f
#define REG_CFG_REG_A 0x60
#define REG_CFG_REG_B 0x61
#define REG_CFG_REG_C 0x62
#define REG_INT_CRTL_REG 0x63
#define REG_INT_SOURCE_REG 0x64
#define REG_INT_THS_L_REG 0x65
#define REG_INT_THS_H_REG 0x66
#define REG_STATUS_REG 0x67
#define REG_OUTX_L_REG 0x68
#define REG_OUTX_H_REG 0x69
#define REG_OUTY_L_REG 0x6a
#define REG_OUTY_H_REG 0x6b
#define REG_OUTZ_L_REG 0x6c
#define REG_OUTZ_H_REG 0x6d
#define REG_TEMP_OUT_L_REG 0x6e
#define REG_TEMP_OUT_H_REG 0x6f

/* Important register bits */

#define BIT_LOWPWR 4      /* Low power mode */
#define BIT_SOFT_RST 5    /* Soft reset */
#define BIT_REBOOT 6      /* Memory reboot */
#define BIT_TEMP_COMP 7   /* Temperature compensation */
#define BIT_SELF_TEST 1   /* Self test enabled */
#define BIT_ZYXDA 3       /* New XYZ data available */
#define BIT_DRDY 0        /* Data ready signal enable */
#define BIT_OFFSET_CANC 1 /* Offset cancellation enable */
#define BIT_LPF 0         /* Low pass filter enable */

/* Unit conversion of Gauss to micro Tesla */

#define GAUSS_TO_UTESLA 100.0f

/* Sensitivity of magnetometer (1.5 milligauss/LSB) */

#define MAG_SENSITIVITY 0.0015f

/* The number of samples required during a self-test */

#define SELFTEST_SAMPLES 50

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Valid ODR settings */

enum lis2mdl_odr_e
{
  ODR_10HZ = 0x0,
  ODR_20HZ = 0x1,
  ODR_50HZ = 0x2,
  ODR_100HZ = 0x3,
};

/* Valid mode settings */

enum lis2mdl_mode_e
{
  MODE_CONT = 0x0,   /* Continuous measurement */
  MODE_SINGLE = 0x1, /* Single measurement, then idle */
  MODE_IDLE = 0x2,   /* Idle */
};

/* Represents the main LIS2MDL device with a lower half for each data type */

struct lis2mdl_dev_s
{
  struct sensor_lowerhalf_s lower; /* UORB lower-half */
  FAR struct i2c_master_s *i2c;    /* I2C interface */
  uint8_t addr;                    /* I2C address */
  sem_t run;                       /* Polling cycle lock */
  mutex_t devlock;                 /* Exclusive access to device */
  enum lis2mdl_odr_e odr;          /* Configured ODR */
  bool lowpower;                   /* True if in low-power mode */
  bool offsets;                    /* True if offset cancellations
                                    * are enabled */
  bool enabled;                    /* Whether the sensor is
                                    * enabled */
  bool interrupts;                 /* Whether or not
                                    * interrupts are enabled */
  enum lis2mdl_mode_e mode;        /* Operation mode */
  struct work_s work;              /* Interrupt work queue
                                    * structure */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int lis2mdl_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable);
static int lis2mdl_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *period_us);
static int lis2mdl_selftest(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, unsigned long arg);
static int lis2mdl_get_info(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            FAR struct sensor_device_info_s *info);
static int lis2mdl_control(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, int cmd,
                           unsigned long arg);
static int lis2mdl_set_calibvalue(FAR struct sensor_lowerhalf_s *lower,
                                  FAR struct file *filep, unsigned long arg);
#ifdef CONFIG_SENSORS_LIS2MDL_FETCH
static int lis2mdl_fetch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep, FAR char *buffer,
                         size_t buflen);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Convert ODR settings to intervals in microseconds */

static const uint32_t ODR_TO_INTERVAL[] =
{
  [ODR_10HZ] = 100000,
  [ODR_20HZ] = 50000,
  [ODR_50HZ] = 20000,
  [ODR_100HZ] = 10000,
};

/* Current consumption from operating mode
 * Indexed by: CURRENT_CONS[odr][lp][offsets];
 * Current values in mA.
 */

static const float CURRENT_CONS[][2][2] =
{
    [ODR_10HZ] =
        {
            {0.1f, 0.12f},
            {0.025f, 0.05f},
        },
    [ODR_20HZ] =
        {
            {0.2f, 0.235f},
            {0.05f, 0.1f},
        },
    [ODR_50HZ] =
        {
            {0.475f, 0.575f},
            {0.125f, 0.235f},
        },
    [ODR_100HZ] =
        {
            {0.95f, 1.13f},
            {0.25f, 0.46f},
        },
};

/* Sensor operations */

static const struct sensor_ops_s g_sensor_ops =
{
  .activate = lis2mdl_activate,
  .set_interval = lis2mdl_set_interval,
  .selftest = lis2mdl_selftest,
  .get_info = lis2mdl_get_info,
  .control = lis2mdl_control,
  .set_calibvalue = lis2mdl_set_calibvalue,
#ifdef CONFIG_SENSORS_LIS2MDL_FETCH
  .fetch = lis2mdl_fetch,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lis2mdl_read_reg
 *
 * Description:
 *   Read `nbytes` from the register at `addr` into `buf`.
 ****************************************************************************/

static int lis2mdl_read_reg(FAR struct lis2mdl_dev_s *dev, uint8_t addr,
                            void *buf, uint8_t nbytes)
{
  struct i2c_msg_s readcmd[2] = {
      {
          .frequency = CONFIG_SENSORS_LIS2MDL_I2C_FREQUENCY,
          .addr = dev->addr,
          .flags = I2C_M_NOSTOP,
          .buffer = &addr,
          .length = sizeof(addr),
      },
      {
          .frequency = CONFIG_SENSORS_LIS2MDL_I2C_FREQUENCY,
          .addr = dev->addr,
          .flags = I2C_M_READ,
          .buffer = buf,
          .length = nbytes,
      },
  };

  return I2C_TRANSFER(dev->i2c, readcmd, 2);
}

/****************************************************************************
 * Name: lis2mdl_write_reg
 *
 * Description:
 *   Write `nbytes` from `buf` to the registers starting at `addr`.
 ****************************************************************************/

static int lis2mdl_write_reg(FAR struct lis2mdl_dev_s *dev, uint8_t addr,
                             void *buf, uint8_t nbytes)
{
  struct i2c_msg_s writecmd[2] = {
      {
          .frequency = CONFIG_SENSORS_LIS2MDL_I2C_FREQUENCY,
          .addr = dev->addr,
          .flags = I2C_M_NOSTOP,
          .buffer = &addr,
          .length = sizeof(addr),
      },
      {
          .frequency = CONFIG_SENSORS_LIS2MDL_I2C_FREQUENCY,
          .addr = dev->addr,
          .flags = I2C_M_NOSTART,
          .buffer = buf,
          .length = nbytes,
      },
  };

  return I2C_TRANSFER(dev->i2c, writecmd, 2);
}

/****************************************************************************
 * Name: lis2mdl_read_data
 *
 * Description:
 *   Read temperature & mag data from the LIS2MDL into the mag struct.
 ****************************************************************************/

static int lis2mdl_read_data(FAR struct lis2mdl_dev_s *dev,
                             FAR struct sensor_mag *data)
{
  int16_t raw_data[4]; /* X, Y, Z, temperature */
  int err;

  /* Read magnetometer axes followed by temperature */

  err = lis2mdl_read_reg(dev, REG_OUTX_L_REG, raw_data, sizeof(raw_data));
  if (err < 0)
    {
      return err;
    }

  data->timestamp = sensor_get_timestamp();
  data->temperature = ((float)(raw_data[3]) / 256.0f) + 25.0f;
  data->x = (float)(raw_data[0]) * MAG_SENSITIVITY * GAUSS_TO_UTESLA;
  data->y = (float)(raw_data[1]) * MAG_SENSITIVITY * GAUSS_TO_UTESLA;
  data->z = (float)(raw_data[2]) * MAG_SENSITIVITY * GAUSS_TO_UTESLA;
  data->status = 0; /* TODO what is this for? */

  /* It is possible to check status register to see if there even is new
   * data and return an error if there is not, but I have opted to avoid
   * this check. Such a check requires extra overhead in the form of slow
   * I2C communication. Plus, in interrupt-driven mode, this function should
   * only be called when there has been an interrupt (or when doing
   * driver-internal sampling for a self-test).
   */

  return err;
}

/****************************************************************************
 * Name: lis2mdl_push_data
 *
 * Description:
 *    Reads some data with exclusive device access and pushed it to the UORB
 *    upper half.
 ****************************************************************************/

static int lis2mdl_push_data(FAR struct lis2mdl_dev_s *dev)
{
  int err;
  struct sensor_mag data;

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  if (!dev->enabled)
    {
      err = -EAGAIN;
      goto unlock_ret;
    }

  err = lis2mdl_read_data(dev, &data);
  if (err < 0)
    {
      goto unlock_ret;
    }

  dev->lower.push_event(dev->lower.priv, &data, sizeof(data));

unlock_ret:
  nxmutex_unlock(&dev->devlock);
  return err;
}

/****************************************************************************
 * Name: lis2mdl_set_bit
 *
 * Description:
 *   Set or clear bit number `bit` of register `addr`, depending on `val`.
 *
 * Arguments:
 *   addr - The address of the register to modify
 *   bit - The bit number to set/clear
 *   val - True to set the bit, false to clear it
 ****************************************************************************/

static int lis2mdl_set_bit(FAR struct lis2mdl_dev_s *dev, uint8_t addr,
                           uint8_t bit, bool val)
{
  int err;
  uint8_t regval;

  /* Get the current value of the configuration register */

  err = lis2mdl_read_reg(dev, addr, &regval, sizeof(regval));
  if (err < 0)
    {
      return err;
    }

  /* Set or clear the bit */

  regval &= ~(1 << bit);

  if (val)
    {
      regval |= (1 << bit);
    }

  return lis2mdl_write_reg(dev, addr, &regval, sizeof(regval));
}

/****************************************************************************
 * Name: lis2mdl_set_odr
 *
 * Description:
 *   Set the output data rate of the LIS2MDL.
 ****************************************************************************/

static int lis2mdl_set_odr(FAR struct lis2mdl_dev_s *dev,
                           enum lis2mdl_odr_e odr)
{
  int err;
  uint8_t regval;

  /* Get the current value of the configuration register */

  err = lis2mdl_read_reg(dev, REG_CFG_REG_A, &regval, sizeof(regval));
  if (err < 0)
    {
      return err;
    }

  /* Add in the bits for ODR */

  regval &= (0xf3);                        /* Clear ODR bits */
  regval |= (((uint8_t)(odr) & 0x3) << 2); /* Set ODR bits */

  return lis2mdl_write_reg(dev, REG_CFG_REG_A, &regval, sizeof(regval));
}

/****************************************************************************
 * Name: lis2mdl_set_mode
 *
 * Description:
 *   Set the operation mode of the LIS2MDL.
 ****************************************************************************/

static int lis2mdl_set_mode(FAR struct lis2mdl_dev_s *dev,
                            enum lis2mdl_mode_e mode)
{
  int err;
  uint8_t regval;

  /* Get the current value of the configuration register */

  err = lis2mdl_read_reg(dev, REG_CFG_REG_A, &regval, sizeof(regval));
  if (err < 0)
    {
      return err;
    }

  /* Add in the bits for mode */

  regval &= (~0x3);                  /* Clear mode bits */
  regval |= ((uint8_t)(mode) & 0x3); /* Set mode bits */

  return lis2mdl_write_reg(dev, REG_CFG_REG_A, &regval, sizeof(regval));
}

/****************************************************************************
 * Name: lis2mdl_low_power
 *
 * Description:
 *   Put the LIS2MDL sensor in low power mode or high resolution mode.
 *
 * Arguments:
 *   lowpower - True to enter low power mode, false otherwise.
 ****************************************************************************/

static int lis2mdl_low_power(FAR struct lis2mdl_dev_s *dev, bool lowpower)
{
  int err;

  /* Don't send request if low power mode already matches request */

  if (dev->lowpower == lowpower)
    {
      return 0;
    }

  err = lis2mdl_set_bit(dev, REG_CFG_REG_A, BIT_LOWPWR, lowpower);
  if (err < 0)
    {
      return err;
    }

  dev->lowpower = lowpower;
  return err;
}

/****************************************************************************
 * Name: lis2mdl_offset_enable
 *
 * Description:
 *   Enable offset corrections on the LIS2MDL.
 *
 * Arguments:
 *   enable - True to enable offsets, false otherwise.
 ****************************************************************************/

static int lis2mdl_offset_enable(FAR struct lis2mdl_dev_s *dev, bool enable)
{
  int err;

  /* Don't send request if offset enable already matches request */

  if (enable == dev->offsets)
    {
      return 0;
    }

  err = lis2mdl_set_bit(dev, REG_CFG_REG_B, BIT_OFFSET_CANC, enable);
  if (err < 0)
    {
      return err;
    }

  dev->offsets = enable;
  return err;
}

/****************************************************************************
 * Name: lis2mdl_enable_interrupts
 *
 * Description:
 *   Enable offset corrections on the LIS2MDL.
 *
 * Arguments:
 *   enable - True to enable offsets, false otherwise.
 ****************************************************************************/

static int lis2mdl_enable_interrupts(FAR struct lis2mdl_dev_s *dev,
                                     bool enable)
{
  int err;

  /* Don't send request if request already matches current state */

  if (enable == dev->interrupts)
    {
      return 0;
    }

  err = lis2mdl_set_bit(dev, REG_CFG_REG_C, BIT_DRDY, enable);
  if (err < 0)
    {
      return err;
    }

  dev->interrupts = enable;
  return err;
}

/****************************************************************************
 * Name: lis2mdl_temp_compensation
 *
 * Description:
 *   Enable/disable magnetometer temperature compensation.
 *
 * Arguments:
 *   enable - True to enable temperature compensation, false otherwise
 ****************************************************************************/

static int lis2mdl_temp_compensation(FAR struct lis2mdl_dev_s *dev,
                                     bool enable)
{
  return lis2mdl_set_bit(dev, REG_CFG_REG_A, BIT_TEMP_COMP, enable);
}

/****************************************************************************
 * Name: lis2mdl_enable_lpf
 *
 * Description:
 *   Enable/disable magnetometer low pass filter.
 *
 * Arguments:
 *   enable - True to enable the low pass filter, false otherwise
 ****************************************************************************/

static int lis2mdl_enable_lpf(FAR struct lis2mdl_dev_s *dev, bool enable)
{
  return lis2mdl_set_bit(dev, REG_CFG_REG_B, BIT_LPF, enable);
}

/****************************************************************************
 * Name: lis2mdl_reboot
 *
 * Description:
 *   Reboot the sensor.
 ****************************************************************************/

static int lis2mdl_reboot(FAR struct lis2mdl_dev_s *dev)
{
  return lis2mdl_set_bit(dev, REG_CFG_REG_A, BIT_REBOOT, true);
}

/****************************************************************************
 * Name: lis2mdl_reset
 *
 * Description:
 *   Perform a soft reset of the sensor.
 ****************************************************************************/

static int lis2mdl_reset(FAR struct lis2mdl_dev_s *dev)
{
  return lis2mdl_set_bit(dev, REG_CFG_REG_A, BIT_SOFT_RST, true);
}

/****************************************************************************
 * Name: lis2mdl_data_avail
 *
 * Description:
 *   Check if data is available. If yes, return 0. If no, return -EAGAIN.
 ****************************************************************************/

static int lis2mdl_data_avail(FAR struct lis2mdl_dev_s *dev)
{
  int err;
  uint8_t regval;

  /* Get the current value of the status register */

  err = lis2mdl_read_reg(dev, REG_STATUS_REG, &regval, sizeof(regval));
  if (err < 0)
    {
      return err;
    }

  /* Check if bit is set */

  if (regval & (1 << BIT_ZYXDA))
    {
      return 0;
    }

  return -EAGAIN;
}

/****************************************************************************
 * Name: lis2mdl_get_info
 ****************************************************************************/

static int lis2mdl_get_info(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            FAR struct sensor_device_info_s *info)
{
  FAR struct lis2mdl_dev_s *dev =
      container_of(lower, FAR struct lis2mdl_dev_s, lower);

  DEBUGASSERT(dev != NULL);

  info->version = 0;
  info->power = CURRENT_CONS[dev->odr][dev->lowpower][dev->offsets];
  memcpy(info->name, "LIS2MDL", sizeof("LIS2MDL"));
  memcpy(info->name, "STMicro", sizeof("STMicro"));

  info->min_delay = ODR_TO_INTERVAL[ODR_10HZ];
  info->max_delay = ODR_TO_INTERVAL[ODR_10HZ];
  info->fifo_reserved_event_count = 0;
  info->fifo_max_event_count = 0;
  info->max_range = 49.152f * GAUSS_TO_UTESLA; /* +/- 49.152 gauss */
  info->resolution = MAG_SENSITIVITY * GAUSS_TO_UTESLA;

  return 0;
}

/****************************************************************************
 * Name: lis2mdl_set_interval
 ****************************************************************************/

static int lis2mdl_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *period_us)
{
  FAR struct lis2mdl_dev_s *dev =
      container_of(lower, FAR struct lis2mdl_dev_s, lower);
  enum lis2mdl_odr_e odr;
  int err = 0;

  DEBUGASSERT(dev != NULL);

  if (*period_us >= 100000)
    {
      /* 10Hz or lower, set 10Hz */

      odr = ODR_10HZ;
    }
  else if (*period_us >= 50000 && *period_us < 100000)
    {
      /* 10 - 20Hz, set 20Hz */

      odr = ODR_20HZ;
    }
  else if (*period_us >= 20000 && *period_us < 50000)
    {
      /* 20 - 50Hz, set 50Hz */

      odr = ODR_50HZ;
    }
  else
    {
      /* >50Hz, set 100Hz */

      odr = ODR_100HZ;
    }

  err = lis2mdl_set_odr(dev, odr);
  if (err < 0)
    {
      return err;
    }

  *period_us = ODR_TO_INTERVAL[odr];
  dev->odr = odr; /* Record ODR */

  return err;
}

/****************************************************************************
 * Name: lis2mdl_activate
 ****************************************************************************/

static int lis2mdl_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable)
{
  FAR struct lis2mdl_dev_s *dev =
      container_of(lower, FAR struct lis2mdl_dev_s, lower);
  bool start_thread = false;
  int err = 0;

  /* Start the collection thread if not already enabled */

  if (enable && !dev->enabled)
    {
      start_thread = true;

      /* Enable temperature compensation (required for proper operation as
       * per 8.5 in datasheet)
       */

      err = lis2mdl_temp_compensation(dev, true);
      if (err < 0)
        {
          return err;
        }

      /* Put the sensor in high resolution mode */

      err = lis2mdl_low_power(dev, false);
      if (err < 0)
        {
          return err;
        }

      /* Start up the sensor for continuous measurement */

      err = lis2mdl_set_mode(dev, MODE_CONT);
      if (err < 0)
        {
          return err;
        }
    }

  /* Stop the sensor to save power if we're disabling */

  if (!enable && dev->enabled)
    {
      err = lis2mdl_set_mode(dev, MODE_IDLE);
      if (err < 0)
        {
          return err;
        }

      /* Put the sensor in low power mode */

      err = lis2mdl_low_power(dev, true);
      if (err < 0)
        {
          return err;
        }
    }

  dev->enabled = enable; /* Mark state */

  /* Start thread */

  if (start_thread)
    {
      return nxsem_post(&dev->run);
    }

  return 0;
}

/****************************************************************************
 * Name: lis2mdl_selftest
 *
 * Description:
 *   Self test performed according to AN5069 from STMicro.
 ****************************************************************************/

static int lis2mdl_selftest(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, unsigned long arg)
{
  FAR struct lis2mdl_dev_s *dev =
      container_of(lower, FAR struct lis2mdl_dev_s, lower);
  int err = 0;
  enum lis2mdl_mode_e prev_mode = dev->mode;
  bool prev_int = dev->interrupts;
  struct sensor_mag data;
  struct sensor_mag pre_avg;
  struct sensor_mag post_avg;

  /* TODO: should I put this in a fast ODR just for the self-test? */

  memset(&pre_avg, 0, sizeof(pre_avg));
  memset(&post_avg, 0, sizeof(post_avg));

  sninfo("LIS2MDL begin selftest.");

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  /* Turn off interrupts for this self-test if they were enabled */

  if (prev_int)
    {
      err = lis2mdl_enable_interrupts(dev, false);
      if (err < 0)
        {
          goto end_test;
          snerr("LIS2MDL failed to stop interrupts for selftest.");
        }
    }

  /* Enter continuous mode */

  err = lis2mdl_set_mode(dev, MODE_CONT);
  if (err < 0)
    {
      snerr("LIS2MDL failed to enter continuous mode for selftest.");
      goto end_test;
    }

  /* Perform samples without self-test enabled */

  for (uint8_t i = 0; i < SELFTEST_SAMPLES; i++)
    {
      /* Check for data */

      if (!lis2mdl_data_avail(dev))
        {
          goto end_test;
        }

      err = lis2mdl_read_data(dev, &data);
      if (err < 0)
        {
          goto end_test;
        }

      /* Add up samples */

      pre_avg.x += data.x;
      pre_avg.y += data.y;
      pre_avg.z += data.z;

      /* Wait for the measurement interval */

      nxsig_usleep(ODR_TO_INTERVAL[dev->odr]);
    }

  sninfo("LIS2MDL regular samples complete.");

  /* Perform samples with self-test enabled */

  err = lis2mdl_set_bit(dev, REG_CFG_REG_C, BIT_SELF_TEST, true);
  if (err < 0)
    {
      snerr("LIS2MDL failed to enter self-test mode.");
      goto end_test;
    }

  sninfo("LIS2MDL waiting for self-test.");
  nxsig_usleep(60000); /* Wait 60ms as per AN5069 */
  sninfo("LIS2MDL waiting for self-test over.");

  for (uint8_t i = 0; i < SELFTEST_SAMPLES; i++)
    {
      /* Check for data */

      if (!lis2mdl_data_avail(dev))
        {
          goto end_test;
        }

      err = lis2mdl_read_data(dev, &data);
      if (err < 0)
        {
          goto end_test;
        }

      /* Add up samples */

      post_avg.x += data.x;
      post_avg.y += data.y;
      post_avg.z += data.z;

      /* Wait for the measurement interval */

      nxsig_usleep(ODR_TO_INTERVAL[dev->odr]);
    }

  sninfo("LIS2MDL self-test samples complete.");

  /* Turn off self-test for normal operation */

  err = lis2mdl_set_bit(dev, REG_CFG_REG_C, BIT_SELF_TEST, true);
  if (err < 0)
    {
      goto end_test;
    }

  /* Go back to previous operation mode */

  err = lis2mdl_set_mode(dev, prev_mode);
  if (err < 0)
    {
      goto end_test;
    }

  /* Turn interrupts back on if they were enabled */

  if (prev_int)
    {
      err = lis2mdl_enable_interrupts(dev, true);
      if (err < 0)
        {
          snerr("LIS2MDL failed to re-enable interrupts.");
          goto end_test;
        }
    }

  /* Average samples */

  pre_avg.x /= SELFTEST_SAMPLES;
  pre_avg.y /= SELFTEST_SAMPLES;
  pre_avg.z /= SELFTEST_SAMPLES;

  post_avg.x /= SELFTEST_SAMPLES;
  post_avg.y /= SELFTEST_SAMPLES;
  post_avg.z /= SELFTEST_SAMPLES;

  /* Calculate diffs */

  data.x = post_avg.x - pre_avg.x;
  data.y = post_avg.y - pre_avg.y;
  data.z = post_avg.z - pre_avg.z;

  /* Check if difference is within tolerance. */

  if (data.x < 0.015f * GAUSS_TO_UTESLA || data.x > 0.5f * GAUSS_TO_UTESLA ||
      data.y < 0.015f * GAUSS_TO_UTESLA || data.y > 0.5f * GAUSS_TO_UTESLA ||
      data.z < 0.015f * GAUSS_TO_UTESLA || data.z > 0.5f * GAUSS_TO_UTESLA)
    {
      snerr("LIS2MDL failed self-test (%.2fX, %.2fY, %.2fZ)", data.x, data.y,
            data.z);
      err = -EAGAIN;
    }

end_test:
  nxmutex_unlock(&dev->devlock);
  sninfo("LIS2MDL selftest complete.");
  return err;
}

/****************************************************************************
 * Name: lis2mdl_set_calibvalue
 *
 * Description:
 *   Sets the hard-iron offset calibration values. Argument is three floats
 *   representing X, Y, Z offsets in micro-teslas (SI units)
 ****************************************************************************/

static int lis2mdl_set_calibvalue(FAR struct sensor_lowerhalf_s *lower,
                                  FAR struct file *filep, unsigned long arg)
{
  FAR struct lis2mdl_dev_s *dev =
      container_of(lower, FAR struct lis2mdl_dev_s, lower);
  int err = 0;
  int16_t hardirons[3];

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  /* Get user calibration values */

  FAR float *vals = (float *)(arg);
  if (vals == NULL)
    {
      err = -EINVAL;
      goto early_ret;
    }

  /* Convert to offsets in 1.5 milligauss per LSB */

  hardirons[0] = (int16_t)(vals[0] * (1.0f / GAUSS_TO_UTESLA) * 1000.0f);
  hardirons[1] = (int16_t)(vals[1] * (1.0f / GAUSS_TO_UTESLA) * 1000.0f);
  hardirons[2] = (int16_t)(vals[2] * (1.0f / GAUSS_TO_UTESLA) * 1000.0f);

  /* Set values */

  err = lis2mdl_write_reg(dev, REG_OFFSET_X_REG_L, hardirons,
                          sizeof(hardirons));

  if (err < 0)
    {
      goto early_ret;
    }

  /* Enable offset cancellation */

  err = lis2mdl_offset_enable(dev, true);

early_ret:
  nxmutex_unlock(&dev->devlock);
  return err;
}

/****************************************************************************
 * Name: lis2mdl_fetch
 ****************************************************************************/

#ifdef CONFIG_SENSORS_LIS2MDL_FETCH
static int lis2mdl_fetch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  FAR struct lis2mdl_dev_s *dev =
      container_of(lower, FAR struct lis2mdl_dev_s, lower);
  return lis2mdl_push_data(dev);
}
#endif /* CONFIG_SENSORS_LIS2MDL_FETCH */

/****************************************************************************
 * Name: lis2mdl_control
 ****************************************************************************/

static int lis2mdl_control(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR struct lis2mdl_dev_s *dev =
      container_of(lower, FAR struct lis2mdl_dev_s, lower);
  int err = 0;

  err = nxmutex_lock(&dev->devlock);
  if (err)
    {
      return err;
    }

  switch (cmd)
    {
      /* Get the WHOAMI register value */

    case SNIOC_WHO_AM_I:
      {
        uint8_t *id = ((uint8_t *)(arg));
        if (id == NULL)
          {
            err = -EINVAL;
            break;
          }

        err = lis2mdl_read_reg(dev, REG_WHO_AM_I, id, 1);
      }
      break;

      /* Set low power mode if `arg` is truthy */

    case SNIOC_SET_POWER_MODE:
      {
        err = lis2mdl_low_power(dev, arg);
      }
      break;

      /* Soft reset */

    case SNIOC_RESET:
      {
        err = lis2mdl_reset(dev);
      }
      break;

      /* Reboot memory contents */

    case SNIOC_SENSOR_OFF:
      {
        err = lis2mdl_reboot(dev);
      }
      break;

      /* Enable/disable temperature compensation */

    case SNIOC_SET_TEMP_OFFSET:
      {
        err = lis2mdl_temp_compensation(dev, arg);
      }
      break;

    case SNIOC_LPF:
      {
        err = lis2mdl_enable_lpf(dev, arg);
      }
      break;

    default:
      err = -EINVAL;
      snerr("Unknown command for LIS2MDL: %d\n", cmd);
      break;
    }

  nxmutex_unlock(&dev->devlock);
  return err;
}

/****************************************************************************
 * Name: lis2mdl_worker
 *
 * Description:
 *   High priority worker thread to read data upon an interrupt and push it
 *   to the UORB upper half.
 ****************************************************************************/

static void lis2mdl_worker(FAR void *arg)
{
  FAR struct lis2mdl_dev_s *dev = (FAR struct lis2mdl_dev_s *)(arg);
  DEBUGASSERT(dev != NULL);
  lis2mdl_push_data(dev);
}

/****************************************************************************
 * Name: lis2mdl_int_handler
 *
 * Description:
 *   Interrupt handler for the LIS2MDL. `arg` should contain a reference
 *   to the LIS2MDL device instance.
 ****************************************************************************/

static int lis2mdl_int_handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct lis2mdl_dev_s *dev = (FAR struct lis2mdl_dev_s *)(arg);
  int err;
  (void)context;

  DEBUGASSERT(dev != NULL);

  /* Start the high priority worker thread */

  err = work_queue(HPWORK, &dev->work, &lis2mdl_worker, dev, 0);

  if (err < 0)
    {
      snerr("Could not queue LIS2MDL work queue.");
    }

  return err;
}

/****************************************************************************
 * Name: lis2mdl_thread
 *
 * Description:
 *   Kernel thread to poll the LIS2MDL, if not configured with interrupt
 *   handling.
 ****************************************************************************/

static int lis2mdl_thread(int argc, char **argv)
{
  FAR struct lis2mdl_dev_s *dev =
      (FAR struct lis2mdl_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
  int err = 0;

  while (true)
    {
      /* If the sensor is disabled we wait indefinitely */

      if (!dev->enabled)
        {
          err = nxsem_wait(&dev->run);
          if (err < 0)
            {
              continue;
            }
        }

      /* If the sensor is enabled, grab some data */

      err = lis2mdl_push_data(dev);
      if (err < 0)
        {
          return err;
        }

      /* Wait for next measurement cycle */

      nxsig_usleep(ODR_TO_INTERVAL[dev->odr]);
    }

  return err;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lis2mdl_register
 *
 * Description:
 *   Register the LIS2MDL device as a UORB sensor.
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the LIS2MDL.
 *   addr    - The I2C address of the LIS2MDL. Should always be 0x1e.
 *   devno   - The device number to use for the topic (i.e. /dev/mag0)
 *   attach  - A function which is called by this driver to attach the
 *             LIS2MDL interrupt handler to an IRQ. Pass NULL to operate
 *             in polling mode. This function should return 0 on success
 *             and a negated error code otherwise.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lis2mdl_register(FAR struct i2c_master_s *i2c, int devno, uint8_t addr,
                     lis2mdl_attach attach)
{
  FAR struct lis2mdl_dev_s *priv;
  int err;
  FAR char *argv[2];
  char arg1[32];

  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(addr == 0x1e);

  /* High priority work queue is required for interrupt driven mode */

#if !defined(CONFIG_SCHED_HPWORK)
  if (attach != NULL)
    {
      snerr("LIS2MDL driver needs `CONFIG_SCHED_HPWORK` in order to be "
            "interrupt driven.");
      return -ENOSYS;
    }
#endif

  /* Initialize the device structure */

  priv = kmm_zalloc(sizeof(struct lis2mdl_dev_s));
  if (priv == NULL)
    {
      snerr("Failed to allocate LIS2MDL instance.\n");
      return -ENOMEM;
    }

  memset(priv, 0, sizeof(struct lis2mdl_dev_s));

  priv->i2c = i2c;
  priv->addr = addr;

  err = nxmutex_init(&priv->devlock);
  if (err < 0)
    {
      snerr("Failed to register LIS2MDL driver: %d\n", err);
      goto del_mem;
    }

  err = nxsem_init(&priv->run, 0, 0);
  if (err < 0)
    {
      snerr("Failed to register LIS2MDL driver: %d\n", err);
      goto del_mutex;
    }

  /* Register UORB Sensor */

  priv->lower.ops = &g_sensor_ops;
  priv->lower.type = SENSOR_TYPE_MAGNETIC_FIELD;
  priv->enabled = false;
  priv->lowpower = false;
  priv->offsets = false;
  priv->interrupts = false;
  priv->odr = ODR_10HZ; /* 10Hz (0.1s) default ODR */

  err = sensor_register(&priv->lower, devno);
  if (err < 0)
    {
      snerr("Failed to register LIS2MDL driver: %d\n", err);
      goto del_sem;
    }

  /* If attach function is NULL, we operate in polling mode using a kernel
   * thread. Otherwise, we attach our interrupt handler and operate in
   * interrupt driven mode.
   */

  if (attach != NULL)
    {
      /* Register interrupt handler */

      err = attach(lis2mdl_int_handler, priv);

      if (err < 0)
        {
          snerr("Failed to attach LIS2MDL interrupt handler: %d\n", err);
          goto sensor_unreg;
        }

      /* Configure the LIS2MDL to output an interrupt when data is ready */

      err = lis2mdl_enable_interrupts(priv, true);
      if (err < 0)
        {
          snerr("Failed to enable LIS2MDL interrupts: %d\n", err);
          goto sensor_unreg;
        }

      sninfo("Registered LIS2MDL driver in interrupt driven mode.\n");
    }
  else
    {
      /* Polling thread */

      snprintf(arg1, 16, "%p", priv);
      argv[0] = arg1;
      argv[1] = NULL;
      err = kthread_create("lis2mdl_thread", SCHED_PRIORITY_DEFAULT,
                           CONFIG_SENSORS_LIS2MDL_THREAD_STACKSIZE,
                           lis2mdl_thread, argv);
      if (err < 0)
        {
          snerr("Failed to create the LIS2MDL notification kthread.\n");
          goto sensor_unreg;
        }

      sninfo("Registered LIS2MDL driver with kernel polling thread.\n");
    }

  if (err < 0)
    {
    sensor_unreg:
      sensor_unregister(&priv->lower, devno);
    del_sem:
      nxsem_destroy(&priv->run);
    del_mutex:
      nxmutex_destroy(&priv->devlock);
    del_mem:
      kmm_free(priv);
      return err;
    }

  sninfo("Registered LIS2MDL driver.\n");
  return err;
}
