/****************************************************************************
 * drivers/sensors/tc74.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <string.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/tc74.h>
#include <nuttx/wqueue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_TC74_I2C_FREQUENCY
#  define CONFIG_TC74_I2C_FREQUENCY 100000
#endif

/* TC74 Register Addresses */

#define TC74_TEMP_REG              0x00     /* Temperature Register (read-only) */
#define TC74_CONFIG_REG            0x01     /* Configuration Register (read/write) */

/* Configuration Register Bit Definitions */

#define TC74_CONFIG_STANDBY        (1 << 7) /* Bit 7: 1=Standby mode, 0=Normal mode */

/* One reading a second by default until requested otherwise */

#define TC74_DEFAULT_INTERVAL      1000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tc74_dev_s
{
  struct sensor_lowerhalf_s lower;    /* Must be first */
  FAR struct i2c_master_s  *i2c;
  uint8_t                   addr;
  uint32_t                  interval; /* Microseconds between readings */
  struct work_s             work;
  bool                      enabled;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int tc74_activate(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep, bool enable);
static int tc74_set_interval(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep,
                             FAR uint32_t *period_us);
static int tc74_get_info(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep,
                         FAR struct sensor_device_info_s *info);
static void tc74_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_tc74_ops =
{
  .activate     = tc74_activate,
  .set_interval = tc74_set_interval,
  .get_info     = tc74_get_info,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc74_delay
 *
 * Description:
 *   The requeue delay in ticks for the interval in force, never zero: a
 *   zero delay would requeue the worker without it ever yielding.
 *
 * Input Parameters:
 *   priv - The driver state
 *
 * Returned Value:
 *   The delay in clock ticks, at least one.
 *
 ****************************************************************************/

static clock_t tc74_delay(FAR struct tc74_dev_s *priv)
{
  clock_t ticks = priv->interval / USEC_PER_TICK;

  return ticks > 0 ? ticks : 1;
}

/****************************************************************************
 * Name: tc74_putreg8
 *
 * Description:
 *   Write an 8-bit value to a TC74 register.
 *
 * Input Parameters:
 *   priv    - The driver state
 *   regaddr - The register address
 *   regval  - The value to write
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int tc74_putreg8(FAR struct tc74_dev_s *priv,
                        uint8_t regaddr, uint8_t regval)
{
  struct i2c_msg_s msg;
  uint8_t buffer[2];
  int ret;

  buffer[0] = regaddr;
  buffer[1] = regval;

  msg.frequency = CONFIG_TC74_I2C_FREQUENCY;
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = buffer;
  msg.length    = 2;

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      snerr("ERROR: I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: tc74_readraw
 *
 * Description:
 *   Read the temperature register and return it as the signed 8-bit count.
 *
 * Input Parameters:
 *   priv - The driver state
 *   raw  - Where to return the signed 8-bit temperature in degrees C
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.
 *
 ****************************************************************************/

static int tc74_readraw(FAR struct tc74_dev_s *priv, FAR int8_t *raw)
{
  struct i2c_msg_s msg[2];
  uint8_t regaddr = TC74_TEMP_REG;
  uint8_t val;
  int ret;

  msg[0].frequency = CONFIG_TC74_I2C_FREQUENCY;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = CONFIG_TC74_I2C_FREQUENCY;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = &val;
  msg[1].length    = 1;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("ERROR: cannot read temperature: %d\n", ret);
      return ret;
    }

  *raw = (int8_t)val;
  return OK;
}

/****************************************************************************
 * Name: tc74_worker
 *
 * Description:
 *   Take one reading and publish it, then requeue for the next.
 *
 * Input Parameters:
 *   arg - The driver state, as passed to work_queue()
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void tc74_worker(FAR void *arg)
{
  FAR struct tc74_dev_s *priv = arg;
  struct sensor_temp temp;
  int8_t raw;

  DEBUGASSERT(priv != NULL);

  /* Queue the next reading first, so that a failed transfer costs one
   * sample rather than the whole stream.
   */

  work_queue(LPWORK, &priv->work, tc74_worker, priv,
             tc74_delay(priv));

  if (tc74_readraw(priv, &raw) < 0)
    {
      return;
    }

  temp.temperature = sensor_data_itof(raw);
  temp.timestamp   = sensor_get_timestamp();

  priv->lower.push_event(priv->lower.priv, &temp, sizeof(temp));
}

/****************************************************************************
 * Name: tc74_activate
 *
 * Description:
 *   Start or stop the reading stream.
 *
 * Input Parameters:
 *   lower  - The sensor lower half
 *   filep  - The file that asked, unused
 *   enable - True to start reading, false to stop
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.
 *
 ****************************************************************************/

static int tc74_activate(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep, bool enable)
{
  FAR struct tc74_dev_s *priv = (FAR struct tc74_dev_s *)lower;

  if (enable == priv->enabled)
    {
      return OK;
    }

  if (enable)
    {
      /* Wake up TC74 from standby mode */

      tc74_putreg8(priv, TC74_CONFIG_REG, 0);

      work_queue(LPWORK, &priv->work, tc74_worker, priv,
                 tc74_delay(priv));
    }
  else
    {
      work_cancel(LPWORK, &priv->work);

      /* Place TC74 into low-power standby mode */

      tc74_putreg8(priv, TC74_CONFIG_REG, TC74_CONFIG_STANDBY);
    }

  priv->enabled = enable;
  return OK;
}

/****************************************************************************
 * Name: tc74_set_interval
 *
 * Description:
 *   Set how often to read the part.
 *
 * Input Parameters:
 *   lower     - The sensor lower half
 *   filep     - The file that asked, unused
 *   period_us - The interval wanted, updated to the interval granted
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.
 *
 ****************************************************************************/

static int tc74_set_interval(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep,
                             FAR uint32_t *period_us)
{
  FAR struct tc74_dev_s *priv = (FAR struct tc74_dev_s *)lower;

  priv->interval = *period_us;
  return OK;
}

/****************************************************************************
 * Name: tc74_get_info
 *
 * Description:
 *   Describe the part: vendor, range, and resolution.
 *
 * Input Parameters:
 *   lower - The sensor lower half
 *   filep - The file that asked, unused
 *   info  - Where to return the description
 *
 * Returned Value:
 *   Zero on success.
 *
 ****************************************************************************/

static int tc74_get_info(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep,
                         FAR struct sensor_device_info_s *info)
{
  info->version                   = 0;
  info->power                     = 0.2f;    /* 200 uA operating current */
  info->max_range                 = 125.0f;  /* Specified -40C to +125C */
  info->resolution                = 1.0f;    /* 8-bit signed, 1C step */
  info->min_delay                 = 0;
  info->max_delay                 = 0;
  info->fifo_reserved_event_count = 0;
  info->fifo_max_event_count      = 0;
  strlcpy(info->name, "TC74", sizeof(info->name));
  strlcpy(info->vendor, "Microchip", sizeof(info->vendor));
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc74_register
 *
 * Description:
 *   Register the TC74 as a uORB temperature sensor.
 *
 * Input Parameters:
 *   devno - The topic number, giving /dev/uorb/sensor_temp<devno>
 *   i2c   - The bus the part is on
 *   addr  - The 7-bit bus address (e.g., TC74_ADDR_A0 through TC74_ADDR_A7)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 *
 ****************************************************************************/

int tc74_register(int devno, FAR struct i2c_master_s *i2c, uint8_t addr)
{
  FAR struct tc74_dev_s *priv;
  int ret;

  DEBUGASSERT(i2c != NULL);

  priv = kmm_zalloc(sizeof(struct tc74_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->i2c        = i2c;
  priv->addr       = addr;
  priv->interval   = TC74_DEFAULT_INTERVAL;
  priv->lower.ops  = &g_tc74_ops;
  priv->lower.type = SENSOR_TYPE_TEMPERATURE;

  /* Put sensor in standby by default until activated */

  tc74_putreg8(priv, TC74_CONFIG_REG, TC74_CONFIG_STANDBY);

  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: cannot register: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  sninfo("TC74 at %02x registered as sensor_temp%d\n", addr, devno);
  return OK;
}
