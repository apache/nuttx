/****************************************************************************
 * drivers/sensors/tmp112_uorb.c
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
#include <stdio.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/tmp112.h>
#include <nuttx/wqueue.h>
#include <nuttx/debug.h>

#if defined(CONFIG_SENSORS_TMP112) && defined(CONFIG_SENSORS_TMP112_UORB)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_TMP112_I2C_FREQUENCY
#  define CONFIG_TMP112_I2C_FREQUENCY 400000
#endif

/* One reading a second, until something asks for another rate.  This part
 * measures the room rather than anything that moves quickly.
 */

#define TMP112_DEFAULT_INTERVAL  1000000

/* Twelve bits, a sixteenth of a degree each. */

#define TMP112_SIGN_BIT          0x0800
#define TMP112_SIGN_EXTEND       0xf000
#define TMP112_LSB_NUM           1
#define TMP112_LSB_DEN           16

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tmp112_dev_uorb_s
{
  struct sensor_lowerhalf_s lower;   /* Must be first                     */
  FAR struct i2c_master_s  *i2c;
  uint8_t                   addr;
  uint32_t                  interval; /* Microseconds between readings    */
  struct work_s             work;
  bool                      enabled;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int tmp112_uorb_activate(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep, bool enable);
static int tmp112_uorb_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                    FAR struct file *filep,
                                    FAR uint32_t *period_us);
static int tmp112_uorb_get_info(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR struct sensor_device_info_s *info);
static void tmp112_uorb_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_tmp112_uorb_ops =
{
  .activate     = tmp112_uorb_activate,
  .set_interval = tmp112_uorb_set_interval,
  .get_info     = tmp112_uorb_get_info,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tmp112_uorb_delay
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

static clock_t tmp112_uorb_delay(FAR struct tmp112_dev_uorb_s *priv)
{
  clock_t ticks = priv->interval / USEC_PER_TICK;

  return ticks > 0 ? ticks : 1;
}

/****************************************************************************
 * Name: tmp112_uorb_readraw
 *
 * Description:
 *   Read the temperature register and return it as the twelve bit signed
 *   count the part holds.
 *
 * Input Parameters:
 *   priv - The driver state
 *   raw  - Where to return the count, sign extended from twelve bits
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.
 *
 ****************************************************************************/

static int tmp112_uorb_readraw(FAR struct tmp112_dev_uorb_s *priv,
                               FAR int16_t *raw)
{
  struct i2c_msg_s msg[2];
  uint8_t regaddr = TMP112_REG_TEMP;
  uint8_t buffer[2];
  uint16_t value;
  int ret;

  msg[0].frequency = CONFIG_TMP112_I2C_FREQUENCY;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = CONFIG_TMP112_I2C_FREQUENCY;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buffer;
  msg[1].length    = 2;

  /* The pointer and the read go out as one transaction.  The part converts
   * continuously and this register always holds the last completed
   * conversion, so there is nothing to wait for between the two.
   */

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("ERROR: cannot read the temperature: %d\n", ret);
      return ret;
    }

  /* The part sends the high byte first, and the reading occupies the top
   * twelve bits of the pair.
   */

  value = ((uint16_t)buffer[0] << 4) | (buffer[1] >> 4);

  /* Sign extend from twelve bits.  The character mode driver does not do
   * this, so it reads anything below freezing as a large positive number;
   * the part itself is specified down to -40C.
   */

  if ((value & TMP112_SIGN_BIT) != 0)
    {
      value |= TMP112_SIGN_EXTEND;
    }

  *raw = (int16_t)value;
  return OK;
}

/****************************************************************************
 * Name: tmp112_uorb_worker
 *
 * Description:
 *   Take one reading and publish it, then requeue for the next.  The
 *   requeue happens first so a failed transfer costs one sample rather
 *   than ending the stream.
 *
 * Input Parameters:
 *   arg - The driver state, as passed to work_queue()
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void tmp112_uorb_worker(FAR void *arg)
{
  FAR struct tmp112_dev_uorb_s *priv = arg;
  struct sensor_temp temp;
  int16_t raw;

  DEBUGASSERT(priv != NULL);

  /* Queue the next reading first, so that a failed transfer costs one
   * sample rather than the whole stream.
   */

  work_queue(LPWORK, &priv->work, tmp112_uorb_worker, priv,
             tmp112_uorb_delay(priv));

  if (tmp112_uorb_readraw(priv, &raw) < 0)
    {
      return;
    }

  temp.temperature = sensor_data_divi(sensor_data_itof(raw),
                                      TMP112_LSB_DEN);
  temp.timestamp   = sensor_get_timestamp();

  priv->lower.push_event(priv->lower.priv, &temp, sizeof(temp));
}

/****************************************************************************
 * Name: tmp112_uorb_activate
 *
 * Description:
 *   Start or stop the reading stream.  The part measures whether or not
 *   anything is listening, so this only starts and stops the work that
 *   collects and publishes.
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

static int tmp112_uorb_activate(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep, bool enable)
{
  FAR struct tmp112_dev_uorb_s *priv =
    (FAR struct tmp112_dev_uorb_s *)lower;

  if (enable == priv->enabled)
    {
      return OK;
    }

  if (enable)
    {
      work_queue(LPWORK, &priv->work, tmp112_uorb_worker, priv,
                 tmp112_uorb_delay(priv));
    }
  else
    {
      work_cancel(LPWORK, &priv->work);
    }

  priv->enabled = enable;
  return OK;
}

/****************************************************************************
 * Name: tmp112_uorb_set_interval
 *
 * Description:
 *   Set how often to read the part.
 *
 *   The interval is taken as asked.  The upper half rejects a lower half
 *   that hands back a longer interval than it was given, so clamping here
 *   would fail the request rather than grant a slower rate.  Reading faster
 *   than the part converts repeats a value, which costs bus traffic and
 *   nothing else.
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

static int tmp112_uorb_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                    FAR struct file *filep,
                                    FAR uint32_t *period_us)
{
  FAR struct tmp112_dev_uorb_s *priv =
    (FAR struct tmp112_dev_uorb_s *)lower;

  priv->interval = *period_us;
  return OK;
}

/****************************************************************************
 * Name: tmp112_uorb_get_info
 *
 * Description:
 *   Describe the part to a consumer that asks: what it is, who makes it, and
 *   the range and resolution its readings carry.  Without this a consumer
 *   would have to know it was talking to a TMP112 to know what the numbers
 *   mean.
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

static int tmp112_uorb_get_info(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR struct sensor_device_info_s *info)
{
  info->version    = 0;
  info->power      = 0.01f;    /* 10uA quiescent */
  info->max_range  = 125.0f;   /* Specified -40C to +125C */
  info->resolution = 0.0625f;  /* Twelve bits over 128C */
  info->min_delay  = 0;
  info->max_delay  = 0;
  info->fifo_reserved_event_count = 0;
  info->fifo_max_event_count      = 0;
  strlcpy(info->name, "TMP112", sizeof(info->name));
  strlcpy(info->vendor, "Texas Instruments", sizeof(info->vendor));
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tmp112_register_uorb
 *
 * Description:
 *   Register the TMP112 as a uORB temperature sensor.
 *
 * Input Parameters:
 *   devno - The topic number, giving /dev/uorb/sensor_temp<devno>
 *   i2c   - The bus the part is on
 *   addr  - The bus address, 0x48 to 0x4b as the pin straps say
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 *
 ****************************************************************************/

int tmp112_register_uorb(int devno, FAR struct i2c_master_s *i2c,
                         uint8_t addr)
{
  FAR struct tmp112_dev_uorb_s *priv;
  int ret;

  DEBUGASSERT(i2c != NULL);

  priv = kmm_zalloc(sizeof(struct tmp112_dev_uorb_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->i2c        = i2c;
  priv->addr       = addr;
  priv->interval   = TMP112_DEFAULT_INTERVAL;
  priv->lower.ops  = &g_tmp112_uorb_ops;
  priv->lower.type = SENSOR_TYPE_TEMPERATURE;

  /* The part measures continuously out of reset, so there is nothing to
   * configure before it will answer.
   */

  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: cannot register: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  sninfo("TMP112 at %02x registered as sensor_temp%d\n", addr, devno);
  return OK;
}

#endif /* CONFIG_SENSORS_TMP112 && CONFIG_SENSORS_TMP112_UORB */
