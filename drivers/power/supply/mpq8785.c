/****************************************************************************
 * drivers/power/supply/mpq8785.c
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
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/power/mpq8785.h>
#include <nuttx/power/regulator.h>
#include <nuttx/nuttx.h>
#ifdef CONFIG_SENSORS
#  include <nuttx/sensors/sensor.h>
#  include <nuttx/wqueue.h>
#endif
#include <nuttx/debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PMBus command codes.  All of these are from the specification rather than
 * particular to this part, which is worth knowing when reading the manual:
 * the datasheet describes what each one holds here, not what it is for.
 */

#define MPQ8785_OPERATION        0x01
#define MPQ8785_ON_OFF_CONFIG    0x02
#define MPQ8785_VOUT_MODE        0x20
#define MPQ8785_VOUT_COMMAND     0x21
#define MPQ8785_VOUT_MAX         0x24
#define MPQ8785_STATUS_WORD      0x79
#define MPQ8785_READ_VIN         0x88
#define MPQ8785_READ_VOUT        0x8b
#define MPQ8785_READ_IOUT        0x8c
#define MPQ8785_READ_TEMPERATURE 0x8d

/* VOUT_MODE bits 6:5 choose how a voltage is written and read back:
 * linear, VID, or direct.  VID is selected, which fixes the reference
 * step at 1.5625mV and forces the parameter bits to zero.
 */

#define MPQ8785_MODE_SEL_SHIFT   5
#define MPQ8785_MODE_SEL_MASK    (3 << MPQ8785_MODE_SEL_SHIFT)
#define MPQ8785_MODE_SEL_LINEAR  (0 << MPQ8785_MODE_SEL_SHIFT)
#define MPQ8785_MODE_SEL_VID     (1 << MPQ8785_MODE_SEL_SHIFT)

/* VOUT_COMMAND is twelve bits wide; the reference DAC behind it is ten.
 * The register will therefore accept values the part will not produce,
 * which is why the range this driver publishes comes from the electrical
 * table and not from the width of the field.
 */

#define MPQ8785_VOUT_MASK        0x0fff

/* OPERATION bit 7 turns the output on.  ON_OFF_CONFIG decides whether that
 * bit is obeyed at all, so it is read rather than assumed.
 */

#define MPQ8785_OPERATION_ON     0x80

/* Telemetry, each one a whole number of its unit per count.  Temperature
 * arrives in the low byte only, the high byte reads as zero.
 */

#define MPQ8785_VIN_UV_PER_LSB   25000
#define MPQ8785_IOUT_UA_PER_LSB  62500
#define MPQ8785_TEMP_MC_PER_LSB  1000

/* The reference DAC steps 1.5625mV, expressed here as a fraction to keep
 * it exact: a microvolt is not a fine enough unit to hold it.
 */

#define MPQ8785_LSB_NUM          15625
#define MPQ8785_LSB_DEN          10

/* A reading every tenth of a second by default. */

#define MPQ8785_DEFAULT_INTERVAL 100000

#ifndef CONFIG_MPQ8785_I2C_FREQUENCY
#  define CONFIG_MPQ8785_I2C_FREQUENCY 100000
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mpq8785_dev_s;

#ifdef CONFIG_SENSORS
/* One published quantity, with a way back to the part. */

struct mpq8785_topic_s
{
  struct sensor_lowerhalf_s lower;
  FAR struct mpq8785_dev_s *dev;
};
#endif

struct mpq8785_dev_s
{
  FAR struct i2c_master_s *i2c;
  struct regulator_desc_s  desc;
  FAR struct regulator_dev_s *rdev;
  mutex_t lock;
  uint8_t addr;
  char    name[16];

#ifdef CONFIG_SENSORS
  /* The output side and, separately, what the part is being fed. */

  struct mpq8785_topic_s vout;
  struct mpq8785_topic_s iout;
  struct mpq8785_topic_s power;
  struct mpq8785_topic_s temp;
  struct mpq8785_topic_s vin;
  struct work_s          work;
  uint32_t               interval;
#endif
  int                    enabled;
  bool                   published;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int mpq8785_list_voltage(FAR struct regulator_dev_s *rdev,
                                unsigned int selector);
static int mpq8785_set_voltage_sel(FAR struct regulator_dev_s *rdev,
                                   unsigned int selector);
static int mpq8785_get_voltage_sel(FAR struct regulator_dev_s *rdev);
static int mpq8785_enable(FAR struct regulator_dev_s *rdev);
static int mpq8785_disable(FAR struct regulator_dev_s *rdev);
static int mpq8785_is_enabled(FAR struct regulator_dev_s *rdev);
static int mpq8785_describe(FAR struct regulator_dev_s *rdev,
                            FAR char *extra, size_t len);

#ifdef CONFIG_SENSORS
static int mpq8785_sensor_activate(FAR struct sensor_lowerhalf_s *lower,
                                   FAR struct file *filep, bool enable);
static int mpq8785_sensor_interval(FAR struct sensor_lowerhalf_s *lower,
                                   FAR struct file *filep,
                                   FAR uint32_t *period_us);
static void mpq8785_worker(FAR void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct regulator_ops_s g_mpq8785_ops =
{
  .list_voltage    = mpq8785_list_voltage,
  .set_voltage_sel = mpq8785_set_voltage_sel,
  .get_voltage_sel = mpq8785_get_voltage_sel,
  .enable          = mpq8785_enable,
  .disable         = mpq8785_disable,
  .is_enabled      = mpq8785_is_enabled,
  .describe        = mpq8785_describe,
};

#ifdef CONFIG_SENSORS
static const struct sensor_ops_s g_mpq8785_sensor_ops =
{
  .activate     = mpq8785_sensor_activate,
  .set_interval = mpq8785_sensor_interval,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpq8785_getreg
 *
 * Description:
 *   Read a PMBus register: the command byte is written, then the data
 *   read back with a repeated start.  Words arrive low byte first.
 *
 * Input Parameters:
 *   priv   - The driver state
 *   cmd    - PMBus command code to read
 *   value  - Where to return the value
 *   nbytes - Register width, one or two bytes
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.
 *
 ****************************************************************************/

static int mpq8785_getreg(FAR struct mpq8785_dev_s *priv, uint8_t cmd,
                          FAR uint16_t *value, int nbytes)
{
  struct i2c_msg_s msg[2];
  uint8_t buffer[2];
  int ret;

  msg[0].frequency = CONFIG_MPQ8785_I2C_FREQUENCY;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &cmd;
  msg[0].length    = 1;

  msg[1].frequency = CONFIG_MPQ8785_I2C_FREQUENCY;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buffer;
  msg[1].length    = nbytes;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      pwrerr("ERROR: read of %02x failed: %d\n", cmd, ret);
      return ret;
    }

  *value = nbytes == 1 ? buffer[0] : (uint16_t)buffer[0] |
                                     ((uint16_t)buffer[1] << 8);
  return OK;
}

/****************************************************************************
 * Name: mpq8785_putreg
 *
 * Description:
 *   Write a PMBus register: the command byte and the data go out as one
 *   transfer, low byte first.
 *
 * Input Parameters:
 *   priv   - The driver state
 *   cmd    - PMBus command code to write
 *   value  - The value to write
 *   nbytes - Register width, one or two bytes
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.
 *
 ****************************************************************************/

static int mpq8785_putreg(FAR struct mpq8785_dev_s *priv, uint8_t cmd,
                          uint16_t value, int nbytes)
{
  struct i2c_msg_s msg[2];
  uint8_t buffer[2];
  int ret;

  buffer[0] = value & 0xff;
  buffer[1] = value >> 8;

  msg[0].frequency = CONFIG_MPQ8785_I2C_FREQUENCY;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &cmd;
  msg[0].length    = 1;

  msg[1].frequency = CONFIG_MPQ8785_I2C_FREQUENCY;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_NOSTART;
  msg[1].buffer    = buffer;
  msg[1].length    = nbytes;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      pwrerr("ERROR: write of %02x failed: %d\n", cmd, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: mpq8785_modreg
 *
 * Description:
 *   Read a register, clear and set bits in it, and write it back, leaving
 *   the bits named by neither mask as they were found.  The caller holds
 *   the lock, so the read and the write are one operation.
 *
 * Input Parameters:
 *   priv   - The part
 *   cmd    - The PMBus command
 *   clear  - Bits to clear
 *   set    - Bits to set
 *   nbytes - Register width, 1 or 2
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.
 *
 ****************************************************************************/

static int mpq8785_modreg(FAR struct mpq8785_dev_s *priv, uint8_t cmd,
                          uint16_t clear, uint16_t set, int nbytes)
{
  uint16_t value;
  int ret;

  ret = mpq8785_getreg(priv, cmd, &value, nbytes);
  if (ret < 0)
    {
      return ret;
    }

  return mpq8785_putreg(priv, cmd, (value & ~clear) | set, nbytes);
}

/****************************************************************************
 * Name: mpq8785_list_voltage
 *
 * Description:
 *   The voltage a selector stands for.  Selectors count from the
 *   registered floor in whole steps, so this is arithmetic rather than
 *   a table.
 *
 * Input Parameters:
 *   rdev     - The regulator being asked
 *   selector - The selector to convert
 *
 * Returned Value:
 *   The voltage in microvolts, or a negated errno if the selector is
 *   outside the range this device was registered with.
 *
 ****************************************************************************/

static int mpq8785_list_voltage(FAR struct regulator_dev_s *rdev,
                                unsigned int selector)
{
  if (selector >= rdev->desc->n_voltages)
    {
      return -EINVAL;
    }

  return rdev->desc->min_uv + selector * rdev->desc->uv_step;
}

/****************************************************************************
 * Name: mpq8785_set_voltage_sel
 *
 * Description:
 *   Move the rail.  The framework has already checked the selector against
 *   the range this device was registered with, which is the board's range
 *   rather than the part's.
 *
 * Input Parameters:
 *   rdev     - The regulator being set
 *   selector - Which voltage to move to
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.
 *
 ****************************************************************************/

static int mpq8785_set_voltage_sel(FAR struct regulator_dev_s *rdev,
                                   unsigned int selector)
{
  FAR struct mpq8785_dev_s *priv = rdev->priv;
  int uv;
  int reg;
  int ret;

  uv = mpq8785_list_voltage(rdev, selector);
  if (uv < 0)
    {
      return uv;
    }

  /* Microvolts to DAC counts, at 1.5625mV a count.  Every voltage a
   * selector can name is a whole number of counts by construction.
   */

  reg = (uv * MPQ8785_LSB_DEN) / MPQ8785_LSB_NUM;

  nxmutex_lock(&priv->lock);
  ret = mpq8785_putreg(priv, MPQ8785_VOUT_COMMAND,
                       reg & MPQ8785_VOUT_MASK, 2);
  nxmutex_unlock(&priv->lock);

  pwrinfo("%s: %d uV, DAC %d\n", priv->name, uv, reg);
  return ret;
}

/****************************************************************************
 * Name: mpq8785_get_voltage_sel
 *
 * Description:
 *   Read the rail back and report it as a selector.  The DAC is read
 *   rather than remembered, so a rail the boot loader set reports as it
 *   actually is.
 *
 * Input Parameters:
 *   rdev - The regulator being read
 *
 * Returned Value:
 *   The selector, or a negated errno if the part could not be read, or
 *   -ERANGE if it reports a voltage outside the registered range.
 *
 ****************************************************************************/

static int mpq8785_get_voltage_sel(FAR struct regulator_dev_s *rdev)
{
  FAR struct mpq8785_dev_s *priv = rdev->priv;
  uint16_t reg;
  int uv;
  int ret;

  nxmutex_lock(&priv->lock);
  ret = mpq8785_getreg(priv, MPQ8785_VOUT_COMMAND, &reg, 2);
  nxmutex_unlock(&priv->lock);

  if (ret < 0)
    {
      return ret;
    }

  uv = ((reg & MPQ8785_VOUT_MASK) * MPQ8785_LSB_NUM) / MPQ8785_LSB_DEN;

  /* The rail may be somewhere this driver would not have put it, because
   * the boot loader set it and nothing has asked for a change.  A voltage
   * outside the range this device was registered with has no selector, and
   * naming the nearest one would report a voltage the rail is not at, so
   * this reports that it does not know: /proc/regulator shows uv:- and a
   * consumer gets an error rather than a plausible wrong answer.
   */

  if (uv < (int)rdev->desc->min_uv || uv > (int)rdev->desc->max_uv)
    {
      pwrwarn("WARNING: %s: %d uV is outside %" PRIu32 " to %" PRIu32 "\n",
              priv->name, uv, rdev->desc->min_uv, rdev->desc->max_uv);
      return -ERANGE;
    }

  return (uv - rdev->desc->min_uv) / rdev->desc->uv_step;
}

/****************************************************************************
 * Name: mpq8785_enable
 *
 * Description:
 *   Switch the rail on, by setting the ON bit of OPERATION and leaving
 *   the rest of that register as it was found.
 *
 * Input Parameters:
 *   rdev - The regulator being enabled
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.
 *
 ****************************************************************************/

static int mpq8785_enable(FAR struct regulator_dev_s *rdev)
{
  FAR struct mpq8785_dev_s *priv = rdev->priv;
  int ret;

  nxmutex_lock(&priv->lock);
  ret = mpq8785_modreg(priv, MPQ8785_OPERATION, 0, MPQ8785_OPERATION_ON, 1);
  nxmutex_unlock(&priv->lock);

  return ret;
}

/****************************************************************************
 * Name: mpq8785_disable
 *
 * Description:
 *   Switch the rail off, clearing the ON bit of OPERATION and leaving
 *   the rest of that register as it was found.
 *
 * Input Parameters:
 *   rdev - The regulator being disabled
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.
 *
 ****************************************************************************/

static int mpq8785_disable(FAR struct regulator_dev_s *rdev)
{
  FAR struct mpq8785_dev_s *priv = rdev->priv;
  int ret;

  nxmutex_lock(&priv->lock);
  ret = mpq8785_modreg(priv, MPQ8785_OPERATION, MPQ8785_OPERATION_ON, 0, 1);
  nxmutex_unlock(&priv->lock);

  return ret;
}

/****************************************************************************
 * Name: mpq8785_is_enabled
 *
 * Description:
 *   Report whether the rail is switched on, from the ON bit of
 *   OPERATION.
 *
 * Input Parameters:
 *   rdev - The regulator being queried
 *
 * Returned Value:
 *   One if the rail is on, zero if it is off, or a negated errno if the
 *   part could not be read.
 *
 ****************************************************************************/

static int mpq8785_is_enabled(FAR struct regulator_dev_s *rdev)
{
  FAR struct mpq8785_dev_s *priv = rdev->priv;
  uint16_t operation;
  int ret;

  nxmutex_lock(&priv->lock);
  ret = mpq8785_getreg(priv, MPQ8785_OPERATION, &operation, 1);
  nxmutex_unlock(&priv->lock);

  if (ret < 0)
    {
      return ret;
    }

  return (operation & MPQ8785_OPERATION_ON) != 0;
}

#ifdef CONFIG_SENSORS

/****************************************************************************
 * Name: mpq8785_worker
 *
 * Description:
 *   Read the part once and publish everything it said from that one
 *   reading, sharing a timestamp.  The power is the product of a voltage
 *   and a current measured at the same instant, which it would not be if
 *   each topic fetched its own.
 *
 * Input Parameters:
 *   arg - The driver state, as a work queue argument
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void mpq8785_worker(FAR void *arg)
{
  FAR struct mpq8785_dev_s *priv = arg;
  struct sensor_voltage vout;
  struct sensor_current iout;
  struct sensor_power power;
  struct sensor_temp temp;
  struct sensor_voltage vin;
  uint64_t timestamp;
  uint16_t raw;
  int ret;

  DEBUGASSERT(priv != NULL);

  work_queue(HPWORK, &priv->work, mpq8785_worker, priv,
             priv->interval / USEC_PER_TICK);

  nxmutex_lock(&priv->lock);

  ret = mpq8785_getreg(priv, MPQ8785_READ_VOUT, &raw, 2);
  if (ret >= 0)
    {
      vout.voltage = sensor_data_divi(
                       sensor_data_muli(sensor_data_itof(raw),
                                        MPQ8785_LSB_NUM),
                       MPQ8785_LSB_DEN * 1000000);
      ret = mpq8785_getreg(priv, MPQ8785_READ_IOUT, &raw, 2);
    }

  if (ret >= 0)
    {
      iout.current = sensor_data_divi(
                       sensor_data_muli(sensor_data_itof(raw),
                                        MPQ8785_IOUT_UA_PER_LSB),
                       1000000);
      ret = mpq8785_getreg(priv, MPQ8785_READ_TEMPERATURE, &raw, 1);
    }

  if (ret >= 0)
    {
      temp.temperature = sensor_data_itof(raw);
      ret = mpq8785_getreg(priv, MPQ8785_READ_VIN, &raw, 2);
    }

  nxmutex_unlock(&priv->lock);

  if (ret < 0)
    {
      return;
    }

  vin.voltage = sensor_data_divi(
                  sensor_data_muli(sensor_data_itof(raw),
                                   MPQ8785_VIN_UV_PER_LSB),
                  1000000);

  power.power = sensor_data_mul(vout.voltage, iout.current);

  timestamp       = sensor_get_timestamp();
  vout.timestamp  = timestamp;
  iout.timestamp  = timestamp;
  power.timestamp = timestamp;
  temp.timestamp  = timestamp;
  vin.timestamp   = timestamp;

  priv->vout.lower.push_event(priv->vout.lower.priv, &vout, sizeof(vout));
  priv->iout.lower.push_event(priv->iout.lower.priv, &iout, sizeof(iout));
  priv->power.lower.push_event(priv->power.lower.priv,
                               &power, sizeof(power));
  priv->temp.lower.push_event(priv->temp.lower.priv, &temp, sizeof(temp));
  priv->vin.lower.push_event(priv->vin.lower.priv, &vin, sizeof(vin));
}

/****************************************************************************
 * Name: mpq8785_sensor_activate
 *
 * Description:
 *   All the topics share one worker, so it runs while any of them is
 *   subscribed.  The part is not switched off when they all go away: it is
 *   supplying something, and stopping the readings must not stop the rail.
 *
 * Input Parameters:
 *   lower  - The topic being enabled or disabled
 *   filep  - Unused
 *   enable - True to subscribe, false to unsubscribe
 *
 * Returned Value:
 *   Zero (OK), always.
 *
 ****************************************************************************/

static int mpq8785_sensor_activate(FAR struct sensor_lowerhalf_s *lower,
                                   FAR struct file *filep, bool enable)
{
  FAR struct mpq8785_topic_s *topic =
    container_of(lower, struct mpq8785_topic_s, lower);
  FAR struct mpq8785_dev_s *priv = topic->dev;

  if (enable)
    {
      if (priv->enabled++ == 0)
        {
          work_queue(HPWORK, &priv->work, mpq8785_worker, priv,
                     priv->interval / USEC_PER_TICK);
        }
    }
  else if (priv->enabled > 0 && --priv->enabled == 0)
    {
      work_cancel(HPWORK, &priv->work);
    }

  return OK;
}

/****************************************************************************
 * Name: mpq8785_sensor_interval
 *
 * Description:
 *   Set one topic's interval.  Unlike a part with several independent
 *   readings, this one worker samples everything on a single schedule,
 *   so the last topic to ask sets the pace for all of them.
 *
 *   The part has no conversion time to wait out: its telemetry registers
 *   are continuously updated, and a read is just an I2C transaction.  The
 *   floor here is the scheduler's own: a delay of less than a tick rounds
 *   down to zero, which would leave the worker re-queueing itself with
 *   the bus never idle.
 *
 * Input Parameters:
 *   lower     - The topic whose interval is changing
 *   filep     - Unused
 *   period_us - The interval asked for; raised to a tick if shorter
 *
 * Returned Value:
 *   Zero (OK), always.
 *
 ****************************************************************************/

static int mpq8785_sensor_interval(FAR struct sensor_lowerhalf_s *lower,
                                   FAR struct file *filep,
                                   FAR uint32_t *period_us)
{
  FAR struct mpq8785_topic_s *topic =
    container_of(lower, struct mpq8785_topic_s, lower);

  if (*period_us < USEC_PER_TICK)
    {
      *period_us = USEC_PER_TICK;
    }

  topic->dev->interval = *period_us;
  return OK;
}
#endif /* CONFIG_SENSORS */

/****************************************************************************
 * Name: mpq8785_describe
 *
 * Description:
 *   Report what this part measures and the framework has no field for, for
 *   /proc/regulator: what it is being fed, what it is delivering, how warm
 *   it is and whether it has latched a fault.  The rail's voltage and range
 *   are the framework's own and are not repeated here.
 *
 * Input Parameters:
 *   rdev  - The regulator being described
 *   extra - Where to write the key:value text
 *   len   - Size of extra
 *
 * Returned Value:
 *   Zero on success, or a negated errno if the part could not be read, in
 *   which case the rail is listed without these fields rather than with a
 *   line of zeroes.
 *
 ****************************************************************************/

static int mpq8785_describe(FAR struct regulator_dev_s *rdev,
                            FAR char *extra, size_t len)
{
  FAR struct mpq8785_dev_s *priv = rdev->priv;
  int32_t vin_uv = 0;
  int32_t iout_ua = 0;
  int32_t temp_mc = 0;
  uint16_t raw;
  int ret;

  nxmutex_lock(&priv->lock);

  ret = mpq8785_getreg(priv, MPQ8785_READ_VIN, &raw, 2);
  if (ret >= 0)
    {
      vin_uv = (int32_t)raw * MPQ8785_VIN_UV_PER_LSB;
      ret = mpq8785_getreg(priv, MPQ8785_READ_IOUT, &raw, 2);
    }

  if (ret >= 0)
    {
      iout_ua = (int32_t)raw * MPQ8785_IOUT_UA_PER_LSB;

      /* Temperature is a byte; the upper half of the word reads zero. */

      ret = mpq8785_getreg(priv, MPQ8785_READ_TEMPERATURE, &raw, 1);
    }

  if (ret >= 0)
    {
      temp_mc = (int32_t)raw * MPQ8785_TEMP_MC_PER_LSB;
      ret = mpq8785_getreg(priv, MPQ8785_STATUS_WORD, &raw, 2);
    }

  nxmutex_unlock(&priv->lock);

  if (ret < 0)
    {
      return ret;
    }

  snprintf(extra, len,
           "vin:%" PRId32 " iout:%" PRId32 " temp:%" PRId32
           " status:0x%04x",
           vin_uv, iout_ua, temp_mc, raw);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpq8785_initialize
 *
 * Description:
 *   Bind the part to an I2C bus and register it as a regulator.  Board
 *   logic calls this once the bus exists.
 *
 *   The caller's limits are intersected with what the part can produce,
 *   so a board may narrow the range and never widen it.  Nothing is
 *   applied: the rail is described, not moved, because whatever is
 *   supplying a running processor was chosen by the boot loader.
 *
 * Input Parameters:
 *   i2c    - The bus the part is on
 *   config - The board's description of the part; see
 *            struct mpq8785_config_s
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.
 *
 ****************************************************************************/

int mpq8785_initialize(FAR struct i2c_master_s *i2c,
                       FAR const struct mpq8785_config_s *config)
{
  FAR struct mpq8785_dev_s *priv;
  uint32_t min_uv;
  uint32_t max_uv;
  uint16_t mode;
  int ret;

  DEBUGASSERT(i2c != NULL && config != NULL && config->name != NULL);

  /* Take the caller's limits, held inside what the part can produce.  A
   * board may only narrow this: the intersection cannot be wider than the
   * electrical specification whatever is passed in, and zero means the
   * caller has nothing to say and gets the part's own range.
   */

  min_uv = config->min_uv != 0 ? config->min_uv : MPQ8785_PART_MIN_UV;
  max_uv = config->max_uv != 0 ? config->max_uv : MPQ8785_PART_MAX_UV;

  if (min_uv < MPQ8785_PART_MIN_UV)
    {
      min_uv = MPQ8785_PART_MIN_UV;
    }

  if (max_uv > MPQ8785_PART_MAX_UV)
    {
      max_uv = MPQ8785_PART_MAX_UV;
    }

  if (min_uv > max_uv)
    {
      pwrerr("ERROR: %s: %" PRIu32 " to %" PRIu32 " uV is not a range\n",
             config->name, min_uv, max_uv);
      return -EINVAL;
    }

  priv = kmm_zalloc(sizeof(struct mpq8785_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = config->addr;
  strlcpy(priv->name, config->name, sizeof(priv->name));
  nxmutex_init(&priv->lock);

  /* Select VID, which is what makes a count of the reference DAC worth
   * 1.5625mV.  The parameter bits are forced to zero by the part in this
   * mode, so the whole register is the mode.
   */

  ret = mpq8785_putreg(priv, MPQ8785_VOUT_MODE, MPQ8785_MODE_SEL_VID, 1);
  if (ret < 0)
    {
      goto errout;
    }

  /* Read it back.  A part that did not take the mode would report volts
   * against a different step, and every number after this would be wrong
   * by a quarter without anything looking broken.
   */

  ret = mpq8785_getreg(priv, MPQ8785_VOUT_MODE, &mode, 1);
  if (ret < 0)
    {
      goto errout;
    }

  if ((mode & MPQ8785_MODE_SEL_MASK) != MPQ8785_MODE_SEL_VID)
    {
      pwrerr("ERROR: %s: VOUT_MODE reads %02x, VID was not selected\n",
             config->name, mode);
      ret = -EIO;
      goto errout;
    }

  priv->desc.name       = priv->name;
  priv->desc.min_uv     = min_uv;
  priv->desc.max_uv     = max_uv;
  priv->desc.uv_step    = MPQ8785_STEP_UV;
  priv->desc.n_voltages = (max_uv - min_uv) / MPQ8785_STEP_UV + 1;

  /* Registered as already on, and nothing applied.  This part is supplying
   * something that is running: the boot loader chose a voltage that works,
   * and coming up is not the moment to have an opinion about it.
   */

  priv->desc.always_on = 1;
  priv->desc.apply_uv  = 0;

  priv->rdev = regulator_register(&priv->desc, &g_mpq8785_ops, priv);
  if (priv->rdev == NULL)
    {
      pwrerr("ERROR: %s: cannot register\n", config->name);
      ret = -EINVAL;
      goto errout;
    }

#ifdef CONFIG_SENSORS
  /* Publish the readings too, which is the half of this part that is
   * useful before anything wants to change a voltage.
   *
   * Five topics over two numbers.  What the part produces and what it is
   * fed are both voltages and cannot share one number, so the output side
   * takes config->devno and the input voltage takes config->vin_devno.
   */

  if (!config->nosensor)
    {
      priv->interval = MPQ8785_DEFAULT_INTERVAL;

      priv->vout.dev        = priv;
      priv->vout.lower.ops  = &g_mpq8785_sensor_ops;
      priv->vout.lower.type = SENSOR_TYPE_VOLTAGE;

      priv->iout.dev        = priv;
      priv->iout.lower.ops  = &g_mpq8785_sensor_ops;
      priv->iout.lower.type = SENSOR_TYPE_CURRENT;

      priv->power.dev        = priv;
      priv->power.lower.ops  = &g_mpq8785_sensor_ops;
      priv->power.lower.type = SENSOR_TYPE_POWER;

      priv->temp.dev        = priv;
      priv->temp.lower.ops  = &g_mpq8785_sensor_ops;
      priv->temp.lower.type = SENSOR_TYPE_TEMPERATURE;

      priv->vin.dev        = priv;
      priv->vin.lower.ops  = &g_mpq8785_sensor_ops;
      priv->vin.lower.type = SENSOR_TYPE_VOLTAGE;

      ret = sensor_register(&priv->vout.lower, config->devno);
      if (ret >= 0)
        {
          ret = sensor_register(&priv->iout.lower, config->devno);
        }

      if (ret >= 0)
        {
          ret = sensor_register(&priv->power.lower, config->devno);
        }

      if (ret >= 0)
        {
          ret = sensor_register(&priv->temp.lower, config->devno);
        }

      if (ret >= 0)
        {
          ret = sensor_register(&priv->vin.lower, config->vin_devno);
        }

      if (ret < 0)
        {
          pwrerr("ERROR: %s: cannot publish the readings: %d\n",
                 config->name, ret);
          regulator_unregister(priv->rdev);
          goto errout;
        }

      priv->published = true;
    }
#endif

  pwrinfo("%s: %" PRIu32 " to %" PRIu32 " uV in %u steps\n",
          priv->name, min_uv, max_uv, priv->desc.n_voltages);
  return OK;

errout:
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return ret;
}

