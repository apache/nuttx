/****************************************************************************
 * drivers/sensors/nau7802.c
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
#include <nuttx/sensors/nau7802.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/signal.h>
#include <nuttx/wqueue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Registers */

#define REG_PU_CTRL 0x00 // power up control
#define REG_CTRL_1 0x01  // control/config reg 1
#define REG_CTRL_2 0x02  // control/config reg 2

#define REG_GCAL1_B3 0x6 // gain calibration registers
#define REG_GCAL1_B2 0x7
#define REG_GCAL1_B1 0x8
#define REG_GCAL1_B0 0x9

#define REG_ADCO_B2 0x12 // data bit 23 to 16
#define REG_ADCO_B1 0x13 // data bit 15 to 8
#define REG_ADCO_B0 0x14 // data bit 7 to 0
#define REG_ADC 0x15     // ADC / chopper control
#define REG_PGA 0x1B     // PGA control
#define REG_POWER 0x1C   // power control

/* Bits for the PU_CTRL register */

#define BIT_RR 0x0    // register reset
#define BIT_PUD 0x1   // power up digital
#define BIT_PUA 0x2   // power up analog
#define BIT_PUR 0x3   // power up ready
#define BIT_CS 0x4    // cycle start
#define BIT_CR 0x5    // cycle ready
#define BIT_AVVDS 0x7 // AVDDS source select

/* Bits for the CTRL_2 register */

#define CAL_START 0x2
#define CAL_ERR 0x3

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* ODR to Interval */

static const uint32_t ODR_TO_INTERVAL[] =
{
    [NAU7802_ODR_10HZ] = 100000,
    [NAU7802_ODR_20HZ] = 50000,
    [NAU7802_ODR_40HZ] = 25000,
    [NAU7802_ODR_80HZ] = 12500,
    [NAU7802_ODR_320HZ] = 3125
  };

typedef struct
{
  struct sensor_lowerhalf_s lower;
  FAR struct i2c_master_s *i2c;
  uint8_t addr;
  sem_t run;
  mutex_t devlock;
  bool enabled;
  nau7802_odr_e odr;
} nau7802_dev_s;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nau7802_read_reg
 *
 * Description:
 *   Read `nbytes` from the register at `addr` into `buf`.
 *
 ****************************************************************************/

static int nau7802_read_reg(FAR nau7802_dev_s *dev, uint8_t addr, void *buf,
                            uint8_t nbytes)
{
  struct i2c_msg_s readcmd[2] = {
      {
          .frequency = CONFIG_SENSORS_NAU7802_I2C_FREQUENCY,
          .addr = dev->addr,
          .flags = I2C_M_NOSTOP,
          .buffer = &addr,
          .length = sizeof(addr),
      },
      {
          .frequency = CONFIG_SENSORS_NAU7802_I2C_FREQUENCY,
          .addr = dev->addr,
          .flags = I2C_M_READ,
          .buffer = buf,
          .length = nbytes,
      },
  };

  return I2C_TRANSFER(dev->i2c, readcmd, 2);
}

/****************************************************************************
 * Name: nau7802_write_reg
 *
 * Description:
 *   Write `nbytes` from `buf` to the registers starting at `addr`.
 *
 ****************************************************************************/

static int nau7802_write_reg(FAR nau7802_dev_s *dev, uint8_t addr, void *buf,
                             uint8_t nbytes)
{
  struct i2c_msg_s writecmd[2] = {
      {
          .frequency = CONFIG_SENSORS_NAU7802_I2C_FREQUENCY,
          .addr = dev->addr,
          .flags = I2C_M_NOSTOP,
          .buffer = &addr,
          .length = sizeof(addr),
      },
      {
          .frequency = CONFIG_SENSORS_NAU7802_I2C_FREQUENCY,
          .addr = dev->addr,
          .flags = I2C_M_NOSTART,
          .buffer = buf,
          .length = nbytes,
      },
  };

  return I2C_TRANSFER(dev->i2c, writecmd, 2);
}

/****************************************************************************
 * Name: nau7802_set_bits
 *
 * Description:
 *   Helper function to set bits in a register.
 *
 * Arguments:
 *   addr - The address of the register to modify
 *   n_bits - The number of bits to set
 *   n_bit_shifts - The number of bits to shift
 *   value - The value to set
 *
 ****************************************************************************/

static int nau7802_set_bits(FAR nau7802_dev_s *dev, uint8_t addr,
                            uint8_t n_bits, uint8_t n_bit_shifts,
                            uint8_t value)
{
  int err = 0;
  uint8_t reg_val;

  err = nau7802_read_reg(dev, addr, &reg_val, sizeof(reg_val));
  if (err < 0)
    {
      return err;
    }

  uint8_t mask = ((1 << n_bits) - 1);

  reg_val &= ~(mask << n_bit_shifts);

  reg_val |= ((value & mask) << n_bit_shifts);

  return nau7802_write_reg(dev, addr, &reg_val, sizeof(reg_val));
}

/****************************************************************************
 * Name: nau7802_read_bit
 ****************************************************************************/

static int nau7802_read_bit(FAR nau7802_dev_s *dev, uint8_t addr,
                                          uint8_t bit, bool *val)
{
  int err = 0;
  uint8_t reg_val;

  err = nau7802_read_reg(dev, addr, &reg_val, sizeof(reg_val));
  if (err < 0)
    {
      return err;
    }

  *val = (reg_val >> bit) & 1;
  return err;
}

/****************************************************************************
 * Name: nau7802_reset
 ****************************************************************************/

static int nau7802_reset(FAR nau7802_dev_s *dev)
{
  int err = 0;
  err = nau7802_set_bits(dev, REG_PU_CTRL, 1, BIT_RR, 1);
  if (err < 0)
    {
      return err;
    }

  err = nau7802_set_bits(dev, REG_PU_CTRL, 1, BIT_RR, 0);
  if (err < 0)
    {
      return err;
    }

  err = nau7802_set_bits(dev, REG_PU_CTRL, 1, BIT_PUD, 1);
  if (err < 0)
    {
      return err;
    }

  /* waiting 200 micoroseconds for the power up */

  usleep(200);

  uint8_t reg_val;
  err = nau7802_read_reg(dev, REG_PU_CTRL, &reg_val, sizeof(reg_val));
  if (err < 0)
    {
      return err;
    }

  /* check if power up is successful */

  if (((reg_val >> BIT_PUR) & 1) == 1)
    {
      return 0;
    }
  else
    {
      snerr("Power up failed\n");
      return -1;
    }
}

/****************************************************************************
 * Name: nau7802_enable
 *
 * Description:
 *  Enable or disable the NAU7802.
 *
 ****************************************************************************/

static int nau7802_enable(FAR nau7802_dev_s *dev, bool enable)
{
  int err = 0;
  if (!enable)
    {
      err = nau7802_set_bits(dev, REG_PU_CTRL, 1, BIT_PUA, 0);
      if (err < 0)
        {
          return err;
        }

      err = nau7802_set_bits(dev, REG_PU_CTRL, 1, BIT_PUD, 0);
      if (err < 0)
        {
          return err;
        }

      return err;
    }

  err = nau7802_set_bits(dev, REG_PU_CTRL, 1, BIT_PUD, 1);
  if (err < 0)
    {
      return err;
    }

  err = nau7802_set_bits(dev, REG_PU_CTRL, 1, BIT_PUA, 1);
  if (err < 0)
    {
      return err;
    }

  usleep(600000);

  err = nau7802_set_bits(dev, REG_PU_CTRL, 1, BIT_CS, 1);
  if (err < 0)
    {
      return err;
    }

  bool reg_val;
  err = nau7802_read_bit(dev, REG_PU_CTRL, BIT_PUR, &reg_val);
  if (err < 0 || !reg_val)
    {
      return err;
    }

  return err;
}

/****************************************************************************
 * Name: nau7802_data_available
 *
 * Description:
 *  Check if data is available over I2C.
 *
 ****************************************************************************/

static int nau7802_data_available(FAR nau7802_dev_s *dev, bool *val)
{
  return nau7802_read_bit(dev, REG_PU_CTRL, BIT_CR, val);
}

/****************************************************************************
 * Name: nau7802_read_data
 *
 * Description:
 *  Read the ADC data from the NAU7802 into the sensor_force structure.
 *
 ****************************************************************************/

static int nau7802_read_data(FAR nau7802_dev_s *dev,
                             FAR struct sensor_force *data)
{
  uint8_t msb;
  uint8_t mid;
  uint8_t lsb;
  int32_t value;
  int err = 0;

  err = nau7802_read_reg(dev, REG_ADCO_B2, &msb, sizeof(msb));
  if (err < 0)
    {
      return err;
    }

  nau7802_read_reg(dev, REG_ADCO_B1, &mid, sizeof(mid));
  if (err < 0)
    {
      return err;
    }

  nau7802_read_reg(dev, REG_ADCO_B0, &lsb, sizeof(lsb));
  if (err < 0)
    {
      return err;
    }

  /* Combine into 24-bit value and sign extend to 32-bit */

  value = (int32_t)((uint32_t)msb << 16 | (uint32_t)mid << 8 | lsb);

  /* Sign extend if negative (MSB is set) */

  if (value & 0x800000)
    {
      value |= 0xff000000;
    }

  data->timestamp = sensor_get_timestamp();
  data->event = 0;
  data->force = value;
  return 0;
}

/****************************************************************************
 * Name: nau7802_set_ldo
 *
 * Description:
 *  Set the LDO voltage.
 *
 ****************************************************************************/

static int nau7802_set_ldo(FAR nau7802_dev_s *dev, nau7802_ldo_e voltage)
{
  if (voltage == NAU7802_LDO_V_EXTERNAL)
    {
      return nau7802_set_bits(dev, REG_PU_CTRL, 1, BIT_AVVDS, 0);
    }

  int err = 0;
  err = nau7802_set_bits(dev, REG_PU_CTRL, 1, BIT_AVVDS, 1);
  if (err < 0)
    {
      return err;
    }

  return nau7802_set_bits(dev, REG_CTRL_1, 3, 3, voltage);
}

/****************************************************************************
 * Name: nau7802_set_gain
 ****************************************************************************/

static int nau7802_set_gain(FAR nau7802_dev_s *dev, nau7802_gain_e gain)
{
  return nau7802_set_bits(dev, REG_CTRL_1, 3, 0, gain);
}

/****************************************************************************
 * Name: nau7802_set_interval
 ****************************************************************************/

static int nau7802_set_interval(FAR nau7802_dev_s *dev,
                                      nau7802_odr_e rate)
{
  dev->odr = rate;
  return nau7802_set_bits(dev, REG_CTRL_2, 3, 4, rate);
}

/****************************************************************************
 * Name: nau7802_push_data
 *
 * Description:
 *    Reads some data with exclusive device access and pushed it to the UORB
 *    upper half.
 *
 ****************************************************************************/

static int nau7802_push_data(FAR nau7802_dev_s *dev)
{
  int err = 0;
  struct sensor_force data;

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

  bool data_ready;
  err = nau7802_data_available(dev, &data_ready);
  if (err < 0 || !data_ready)
    {
      goto unlock_ret;
    }

  err = nau7802_read_data(dev, &data);
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
 * Name: nau7802_get_calibvalue
 *
 * Description:
 *  Get the gain calibration value.
 *
 ****************************************************************************/

static int nau7802_get_calibvalue(FAR nau7802_dev_s *dev, unsigned long arg)
{
  uint32_t *calibvalue = (uint32_t *)arg;
  uint8_t reg_b3;
  uint8_t reg_b2;
  uint8_t reg_b1;
  uint8_t reg_b0;

  int err = 0;

  err = nau7802_read_reg(dev, REG_GCAL1_B3, &reg_b3, sizeof(reg_b3));
  if (err < 0)
    {
      return err;
    }

  err = nau7802_read_reg(dev, REG_GCAL1_B2, &reg_b2, sizeof(reg_b2));
  if (err < 0)
    {
      return err;
    }

  err = nau7802_read_reg(dev, REG_GCAL1_B1, &reg_b1, sizeof(reg_b1));
  if (err < 0)
    {
      return err;
    }

  err = nau7802_read_reg(dev, REG_GCAL1_B0, &reg_b0, sizeof(reg_b0));
  if (err < 0)
    {
      return err;
    }

  *calibvalue = (reg_b3 << 24) | (reg_b2 << 16) | (reg_b1 << 8) | reg_b0;
  return 0;
}

/****************************************************************************
 * Name: nau7802_set_calibvalue
 *
 * Description:
 *  Set the gain calibration value.
 *
 ****************************************************************************/

static int nau7802_set_calibvalue(FAR struct sensor_lowerhalf_s *lower,
                                  FAR struct file *filep, unsigned long arg)
{
  FAR nau7802_dev_s *dev = container_of(lower, FAR nau7802_dev_s, lower);
  uint32_t calibvalue = (uint32_t)arg;
  uint8_t reg_b3 = (calibvalue >> 24) & 0xff;
  uint8_t reg_b2 = (calibvalue >> 16) & 0xff;
  uint8_t reg_b1 = (calibvalue >> 8) & 0xff;
  uint8_t reg_b0 = calibvalue & 0xff;

  int err = 0;
  err = nau7802_write_reg(dev, REG_GCAL1_B3, &reg_b3, sizeof(reg_b3));
  if (err < 0)
    {
      return err;
    }

  err = nau7802_write_reg(dev, REG_GCAL1_B2, &reg_b2, sizeof(reg_b2));
  if (err < 0)
    {
      return err;
    }

  err = nau7802_write_reg(dev, REG_GCAL1_B1, &reg_b1, sizeof(reg_b1));
  if (err < 0)
    {
      return err;
    }

  err = nau7802_write_reg(dev, REG_GCAL1_B0, &reg_b0, sizeof(reg_b0));
  if (err < 0)
    {
      return err;
    }

  return 0;
}

/****************************************************************************
 * Name: nau7802_calibrate
 *
 * Description:
 *   Perform either an INTERNAL, OFFSET or GAIN calibration.
 *   The gain calibration value is saved and can be retrieved via the
 *   SNIOC_GET_GAIN_CALIBVALUE command
 *
 ****************************************************************************/

static int nau7802_calibrate(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep, unsigned long int arg)
{
  FAR nau7802_dev_s *dev = container_of(lower, FAR nau7802_dev_s, lower);
  nau7802_calibmode_e mode = (nau7802_calibmode_e)arg;
  int err = 0;

  /* Choosing calibration mode */

  err = nau7802_set_bits(dev, REG_CTRL_2, 2, 0, mode);
  if (err < 0)
    {
      return err;
    }

  /* Start calibration */

  err = nau7802_set_bits(dev, REG_CTRL_2, 1, CAL_START, 1);
  if (err < 0)
    {
      return err;
    }

  /* Wait for calibration to complete */

  bool reg_val;
  do
    {
      err = nau7802_read_bit(dev, REG_CTRL_2, CAL_START, &reg_val);
      if (err < 0)
        {
          return err;
        }

      usleep(10000);
    }
  while (reg_val);

  /* Check calibration error bit */

  err = nau7802_read_bit(dev, REG_CTRL_2, CAL_ERR, &reg_val);
  if (err < 0)
    {
      return err;
    }

  if (reg_val)
    {
      snerr("Calibration failed\n");
      return -1;
    }

  return 0;
}

/****************************************************************************
 * Name: nau7802_control
 ****************************************************************************/

static int nau7802_control(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR nau7802_dev_s *dev = container_of(lower, FAR nau7802_dev_s, lower);
  int err = 0;

  err = nxmutex_lock(&dev->devlock);
  if (err)
    {
      return err;
    }

  switch (cmd)
    {
    case SNIOC_RESET:
      err = nau7802_reset(dev);
      break;

    case SNIOC_SET_GAIN:
      err = nau7802_set_gain(dev, arg);
      break;

    case SNIOC_SET_LDO:
      err = nau7802_set_ldo(dev, arg);
      break;

    case SNIOC_GET_CALIBVALUE:
      err = nau7802_get_calibvalue(dev, arg);
      break;

    default:
      err = -EINVAL;
      snerr("Unknown command for NAU7802: %d\n", cmd);
      break;
    }

  nxmutex_unlock(&dev->devlock);
  return err;
}

/****************************************************************************
 * Name: nau7802_activate
 ****************************************************************************/

static int nau7802_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable)
{
  FAR nau7802_dev_s *dev = container_of(lower, FAR nau7802_dev_s, lower);
  bool start_thread = false;
  int err = 0;

  /* Start the collection thread if not already enabled */

  if (enable && !dev->enabled)
    {
      start_thread = true;

      err = nau7802_reset(dev);
      if (err < 0)
        {
          return err;
        }

      err = nau7802_enable(dev, true);
      if (err < 0)
        {
          return err;
        }

      uint8_t reg_val;
      err = nau7802_read_reg(dev, 0x1f, &reg_val, sizeof(reg_val));
      if (err < 0 || (reg_val & 0xf) != 0xf)
        {
          snerr("Could not read the revision register\n");
          return err;
        }

      err = nau7802_set_ldo(dev, NAU7802_LDO_V_3V0);
      if (err < 0)
        {
          return err;
        }

      err = nau7802_set_gain(dev, NAU7802_GAIN_128);
      if (err < 0)
        {
          return err;
        }

      err = nau7802_set_interval(dev, NAU7802_ODR_10HZ);
      if (err < 0)
        {
          return err;
        }

      /* Disable ADC chopper */

      err = nau7802_set_bits(dev, REG_ADC, 2, 4, 0x3);
      if (err < 0)
        {
          return err;
        }

      /* Use low ESR caps */

      err = nau7802_set_bits(dev, REG_PGA, 1, 6, 0);
      if (err < 0)
        {
          return err;
        }

      /* PGA stabilizer cap on output */

      err = nau7802_set_bits(dev, REG_POWER, 1, 7, 1);
      if (err < 0)
        {
          return err;
        }
    }

  dev->enabled = enable;

  if (start_thread)
    {
      return nxsem_post(&dev->run);
    }

  return 0;
}

/****************************************************************************
 * Name: nau7802_get_info
 ****************************************************************************/

static int nau7802_get_info(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            FAR struct sensor_device_info_s *info)
{
  info->version = 0;
  info->power = 2.1f;
  memcpy(info->name, "NAU7802", sizeof("NAU7802"));
  memcpy(info->vendor, "Nuvoton", sizeof("Nuvoton"));

  info->min_delay = 3125;   /* For 320 ODR (fastest rate) */
  info->max_delay = 100000; /* For 10 ODR (slowest rate) */
  info->fifo_reserved_event_count = 0;
  info->fifo_max_event_count = 0;
  info->max_range = 1.0f;
  info->resolution = 0;

  return 0;
}

/****************************************************************************
 * Name: nau7802_thread
 *
 * Description:
 *   Kernel thread to poll the NAU7802
 *
 ****************************************************************************/

static int nau7802_thread(int argc, FAR char *argv[])
{
  FAR nau7802_dev_s *dev =
      (FAR nau7802_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
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

      err = nau7802_push_data(dev);
      if (err < 0)
        {
          return err;
        }

      /* Wait for next measurement cycle */

      nxsig_usleep(ODR_TO_INTERVAL[dev->odr]);
    }

  return err;
}

static const struct sensor_ops_s g_sensor_ops =
{
  .activate = nau7802_activate,
  .get_info = nau7802_get_info,
  .control = nau7802_control,
  .calibrate = nau7802_calibrate,
  .set_calibvalue = nau7802_set_calibvalue
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nau7802_register
 *
 * Description:
 *   Register the NAU7802 device as a UORB sensor.
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the NAU7802.
 *   addr    - The I2C address of the NAU7802. Should always be 0x2a.
 *   devno   - The device number to use for the topic (i.e. /dev/mag0)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int nau7802_register(FAR struct i2c_master_s *i2c, int devno, uint8_t addr)
{
  int err;
  FAR nau7802_dev_s *priv = kmm_zalloc(sizeof(nau7802_dev_s));
  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(addr == 0x2a);

  if (priv == NULL)
    {
      snerr("Failed to allocate nau7802 instance\n");
      return -ENOMEM;
    }

  err = nxmutex_init(&priv->devlock);
  if (err < 0)
    {
      snerr("Failed to register nau7802 driver: %d\n", err);
      goto del_mem;
    }

  err = nxsem_init(&priv->run, 0, 0);
  if (err < 0)
    {
      snerr("Failed to register nau7802 driver: %d\n", err);
      goto del_mutex;
    }

  priv->i2c = i2c;
  priv->addr = addr;
  priv->lower.ops = &g_sensor_ops;
  priv->lower.type = SENSOR_TYPE_FORCE;
  priv->enabled = false;
  priv->odr = NAU7802_ODR_10HZ; /* 10Hz (0.1s) default ODR */

  err = sensor_register(&priv->lower, devno);
  if (err < 0)
    {
      snerr("Failed to register nau7802 driver: %d\n", err);
      goto del_sem;
    }

  FAR char *argv[2];
  char arg1[32];
  snprintf(arg1, 16, "%p", priv);
  argv[0] = arg1;
  argv[1] = NULL;

  err = kthread_create("nau7802_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_SENSORS_NAU7802_THREAD_STACKSIZE,
                       nau7802_thread, argv);
  if (err < 0)
    {
      snerr("Failed to create the nau7802 notification kthread\n");
      goto sensor_unreg;
    }

  sninfo("Registered nau7802 driver with kernel polling thread\n");

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

  sninfo("Registered NAU7802 driver\n");
  return err;
}
