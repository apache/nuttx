/****************************************************************************
 * drivers/sensors/ina226_uorb.c
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

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>
#include <nuttx/sensors/ina226.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/wqueue.h>
#include <nuttx/debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_INA226_I2C_FREQUENCY
#  define CONFIG_INA226_I2C_FREQUENCY 400000
#endif

/* The bus voltage register counts 1.25mV a step, and the shunt voltage
 * register counts 2.5uV a step and is signed, because a part watching a
 * rail that can be fed from either end has to say which way the current
 * is going.
 */

#define INA226_BUS_UV_PER_LSB    1250
#define INA226_SHUNT_NV_PER_LSB  2500

/* A reading every tenth of a second by default: slow enough not to sit on
 * the bus, quick enough to see a board change what it is doing.
 */

#define INA226_DEFAULT_INTERVAL  100000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ina226_dev_uorb_s;

/* One published quantity.  Each carries a way back to the device, since
 * the framework hands the lower half back and nothing else.
 */

struct ina226_topic_s
{
  struct sensor_lowerhalf_s     lower;
  FAR struct ina226_dev_uorb_s *dev;
  uint32_t                      interval; /* What this topic asked for */
};

struct ina226_dev_uorb_s
{
  struct ina226_topic_s     voltage;
  struct ina226_topic_s     current;
  struct ina226_topic_s     power;
  FAR struct i2c_master_s  *i2c;
  int32_t                   shunt_uohms;
  uint16_t                  config;
  uint8_t                   addr;
  uint32_t                  interval;
  struct work_s             work;
  int                       enabled;   /* How many topics are running    */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ina226_uorb_activate(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep, bool enable);
static int ina226_uorb_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                    FAR struct file *filep,
                                    FAR uint32_t *period_us);
static void ina226_uorb_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_ina226_uorb_ops =
{
  .activate     = ina226_uorb_activate,
  .set_interval = ina226_uorb_set_interval,
};

/* How long one conversion takes, in microseconds, indexed by the value of
 * the VBUSCT and VSHCT fields, which share an encoding.  A sample is both
 * conversions, repeated as many times as the averaging field asks for, so
 * the two tables together say how fast this part can be read.
 */

static const uint16_t g_ina226_convtime[8] =
{
  140, 204, 332, 588, 1100, 2116, 4156, 8244
};

static const uint16_t g_ina226_average[8] =
{
  1, 4, 16, 64, 128, 256, 512, 1024
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ina226_uorb_read16
 *
 * Description:
 *   Read a sixteen bit register: the register address is written, then
 *   two bytes are read back with a repeated start.
 *
 * Input Parameters:
 *   priv    - The driver state
 *   regaddr - Register to read
 *   value   - Where to return the register's contents
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 *
 ****************************************************************************/

static int ina226_uorb_read16(FAR struct ina226_dev_uorb_s *priv,
                              uint8_t regaddr, FAR uint16_t *value)
{
  struct i2c_msg_s msg[2];
  uint8_t buffer[2];
  int ret;

  msg[0].frequency = CONFIG_INA226_I2C_FREQUENCY;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = CONFIG_INA226_I2C_FREQUENCY;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buffer;
  msg[1].length    = 2;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("ERROR: read of %u failed: %d\n", regaddr, ret);
      return ret;
    }

  /* This part sends the high byte first. */

  *value = ((uint16_t)buffer[0] << 8) | buffer[1];
  return OK;
}

/****************************************************************************
 * Name: ina226_uorb_sampletime
 *
 * Description:
 *   How long the part takes to produce one sample with the configuration
 *   it was given: both conversions, times the averaging.
 *
 * Input Parameters:
 *   priv - The driver state
 *
 * Returned Value:
 *   The sample time in microseconds.
 *
 ****************************************************************************/

static uint32_t ina226_uorb_sampletime(FAR struct ina226_dev_uorb_s *priv)
{
  uint32_t bus;
  uint32_t shunt;
  uint32_t avg;

  bus   = g_ina226_convtime[(priv->config & INA226_CONFIG_VBUSCT_MASK) >>
                            INA226_CONFIG_VBUSCT_SHIFT];
  shunt = g_ina226_convtime[(priv->config & INA226_CONFIG_VSHCT_MASK) >>
                            INA226_CONFIG_VSHCT_SHIFT];
  avg   = g_ina226_average[(priv->config & INA226_CONFIG_AVG_MASK) >>
                           INA226_CONFIG_AVG_SHIFT];

  return (bus + shunt) * avg;
}

/****************************************************************************
 * Name: ina226_uorb_write16
 *
 * Description:
 *   Write a sixteen bit register: the address and the two data bytes go
 *   out as one transfer.
 *
 * Input Parameters:
 *   priv    - The driver state
 *   regaddr - Register to write
 *   value   - Value to write
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 *
 ****************************************************************************/

static int ina226_uorb_write16(FAR struct ina226_dev_uorb_s *priv,
                               uint8_t regaddr, uint16_t value)
{
  struct i2c_msg_s msg[2];
  uint8_t buffer[2];

  buffer[0] = value >> 8;
  buffer[1] = value & 0xff;

  msg[0].frequency = CONFIG_INA226_I2C_FREQUENCY;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = CONFIG_INA226_I2C_FREQUENCY;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_NOSTART;
  msg[1].buffer    = buffer;
  msg[1].length    = 2;

  return I2C_TRANSFER(priv->i2c, msg, 2);
}

/****************************************************************************
 * Name: ina226_uorb_worker
 *
 * Description:
 *   Read the part once and publish all three quantities from it, sharing
 *   one timestamp.  Reading per topic instead would put three transfers on
 *   the bus for one sample and leave the values describing three different
 *   instants, which is exactly the wrong property for a power measurement.
 *
 * Input Parameters:
 *   arg - The driver state, as a work queue argument
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void ina226_uorb_worker(FAR void *arg)
{
  FAR struct ina226_dev_uorb_s *priv = arg;
  struct sensor_voltage voltage;
  struct sensor_current current;
  struct sensor_power power;
  uint64_t timestamp;
  uint16_t reg;
  int64_t tmp;
  int32_t bus_uv;
  int32_t shunt_ua;

  DEBUGASSERT(priv != NULL);

  /* Queue the next reading first, so a failed transfer costs one sample
   * rather than the stream.
   */

  work_queue(HPWORK, &priv->work, ina226_uorb_worker, priv,
             priv->interval / USEC_PER_TICK);

  if (ina226_uorb_read16(priv, INA226_REG_BUS_VOLTAGE, &reg) < 0)
    {
      return;
    }

  bus_uv = (int32_t)((uint32_t)reg * INA226_BUS_UV_PER_LSB);

  if (ina226_uorb_read16(priv, INA226_REG_SHUNT_VOLTAGE, &reg) < 0)
    {
      return;
    }

  /* The drop across the shunt gives the current through it.  Widened to
   * 64 bits first: a large reading times a nanovolt scale overflows a
   * 32 bit accumulator long before the result would.
   *
   *   I(uA) = U(nV) / R(uohms) * 1000
   */

  tmp = (int64_t)(int16_t)reg * INA226_SHUNT_NV_PER_LSB * 1000;
  shunt_ua = (int32_t)(tmp / priv->shunt_uohms);

  timestamp = sensor_get_timestamp();

  voltage.timestamp = timestamp;
  voltage.voltage   = sensor_data_divi(sensor_data_itof(bus_uv), 1000000);

  current.timestamp = timestamp;
  current.current   = sensor_data_divi(sensor_data_itof(shunt_ua), 1000000);

  /* Watts, from the two above.  This part can compute power itself, but
   * only once its calibration register has been given a current scale,
   * and the product of two values already read costs nothing.
   */

  power.timestamp = timestamp;
  power.power     = sensor_data_mul(voltage.voltage, current.current);

  priv->voltage.lower.push_event(priv->voltage.lower.priv,
                                 &voltage, sizeof(voltage));
  priv->current.lower.push_event(priv->current.lower.priv,
                                 &current, sizeof(current));
  priv->power.lower.push_event(priv->power.lower.priv,
                               &power, sizeof(power));
}

/****************************************************************************
 * Name: ina226_uorb_activate
 *
 * Description:
 *   The three topics share one worker, so it runs while any of them is
 *   subscribed and stops when the last goes away.
 *
 * Input Parameters:
 *   lower  - The topic being enabled or disabled
 *   filep  - Unused
 *   enable - True to subscribe, false to unsubscribe
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 *
 ****************************************************************************/

static int ina226_uorb_activate(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep, bool enable)
{
  FAR struct ina226_topic_s *topic =
    container_of(lower, struct ina226_topic_s, lower);
  FAR struct ina226_dev_uorb_s *priv = topic->dev;

  if (enable)
    {
      if (priv->enabled++ == 0)
        {
          /* Wake the part and start sampling. */

          ina226_uorb_write16(priv, INA226_REG_CONFIG,
                              priv->config | INA226_CONFIG_MODE_SBCONT);

          work_queue(HPWORK, &priv->work, ina226_uorb_worker, priv,
                     priv->interval / USEC_PER_TICK);
        }
    }
  else if (priv->enabled > 0 && --priv->enabled == 0)
    {
      work_cancel(HPWORK, &priv->work);

      ina226_uorb_write16(priv, INA226_REG_CONFIG,
                          priv->config | INA226_CONFIG_MODE_PWRDOWN);
    }

  return OK;
}

/****************************************************************************
 * Name: ina226_uorb_set_interval
 *
 * Description:
 *   Ask for a new sampling interval on one topic.  The period is raised
 *   to what the part and the worker can actually deliver, and every
 *   topic's own interval is re-examined so the worker keeps running at
 *   the shortest of the three.
 *
 * Input Parameters:
 *   lower     - The topic whose interval is changing
 *   filep     - Unused
 *   period_us - The interval asked for; updated to what is granted
 *
 * Returned Value:
 *   Zero (OK), always.
 *
 ****************************************************************************/

static int ina226_uorb_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                    FAR struct file *filep,
                                    FAR uint32_t *period_us)
{
  FAR struct ina226_topic_s *topic =
    container_of(lower, struct ina226_topic_s, lower);
  FAR struct ina226_dev_uorb_s *priv = topic->dev;
  uint32_t floor;

  /* Nothing is gained by asking faster than the part converts: the same
   * reading comes back twice.  Nor can the worker be woken sooner than a
   * clock tick, and a period that rounds down to no delay at all would
   * leave it re-queueing itself with the bus never idle.  Report back
   * what will actually happen rather than accepting either.
   */

  floor = ina226_uorb_sampletime(priv);
  if (floor < USEC_PER_TICK)
    {
      floor = USEC_PER_TICK;
    }

  if (*period_us < floor)
    {
      *period_us = floor;
    }

  /* One worker feeds all three, so the shortest interval any topic asks
   * for is the one they all get.
   */

  topic->interval = *period_us;

  priv->interval = priv->voltage.interval;
  if (priv->current.interval < priv->interval)
    {
      priv->interval = priv->current.interval;
    }

  if (priv->power.interval < priv->interval)
    {
      priv->interval = priv->power.interval;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ina226_register_uorb
 *
 * Description:
 *   Register the INA226 as uORB voltage, current and power sensors.
 *
 * Input Parameters:
 *   devno    - The topic number shared by all three
 *   i2c      - The bus the part is on
 *   addr     - The bus address
 *   shuntval - The shunt resistance in micro-ohms
 *   config   - The value for the configuration register, averaging and
 *              conversion times; the mode bits are managed here
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 *
 ****************************************************************************/

int ina226_register_uorb(int devno, FAR struct i2c_master_s *i2c,
                         uint8_t addr, int32_t shuntval, uint16_t config)
{
  FAR struct ina226_dev_uorb_s *priv;
  int ret;

  DEBUGASSERT(i2c != NULL);

  if (shuntval <= 0)
    {
      snerr("ERROR: a shunt of %" PRId32 " micro-ohms is not a resistor\n",
            shuntval);
      return -EINVAL;
    }

  priv = kmm_zalloc(sizeof(struct ina226_dev_uorb_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->i2c         = i2c;
  priv->addr        = addr;
  priv->shunt_uohms = shuntval;
  priv->config      = config & ~INA226_CONFIG_MODE_MASK;
  priv->interval    = INA226_DEFAULT_INTERVAL;

  priv->voltage.dev        = priv;
  priv->voltage.interval   = INA226_DEFAULT_INTERVAL;
  priv->voltage.lower.ops  = &g_ina226_uorb_ops;
  priv->voltage.lower.type = SENSOR_TYPE_VOLTAGE;

  priv->current.dev        = priv;
  priv->current.interval   = INA226_DEFAULT_INTERVAL;
  priv->current.lower.ops  = &g_ina226_uorb_ops;
  priv->current.lower.type = SENSOR_TYPE_CURRENT;

  priv->power.dev          = priv;
  priv->power.interval     = INA226_DEFAULT_INTERVAL;
  priv->power.lower.ops    = &g_ina226_uorb_ops;
  priv->power.lower.type   = SENSOR_TYPE_POWER;

  /* Leave the part asleep until something subscribes. */

  ret = ina226_uorb_write16(priv, INA226_REG_CONFIG,
                            priv->config | INA226_CONFIG_MODE_PWRDOWN);
  if (ret < 0)
    {
      snerr("ERROR: %02x does not answer: %d\n", addr, ret);
      goto errout;
    }

  ret = sensor_register(&priv->voltage.lower, devno);
  if (ret < 0)
    {
      goto errout;
    }

  ret = sensor_register(&priv->current.lower, devno);
  if (ret < 0)
    {
      goto errout_voltage;
    }

  ret = sensor_register(&priv->power.lower, devno);
  if (ret < 0)
    {
      goto errout_current;
    }

  sninfo("INA226 at %02x registered as topics %d, shunt %" PRId32 "uohm\n",
         addr, devno, shuntval);
  return OK;

errout_current:
  sensor_unregister(&priv->current.lower, devno);
errout_voltage:
  sensor_unregister(&priv->voltage.lower, devno);
errout:
  kmm_free(priv);
  return ret;
}

