/****************************************************************************
 * arch/arm/src/rm57/rm57_adc_temp.c
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

/* Device nodes for the RM57L843's three on-die temperature sensors.
 *
 * The sensors themselves live in the ADC driver - they are ADC channels,
 * and rm57_adc_readtemp() converts and calibrates them.  This file is only
 * the adaptation to a file interface, and it offers two of them, chosen by
 * CONFIG_RM57_ADC_TEMP_UORB.  The arrangement follows
 * arch/risc-v/src/common/espressif/esp_temperature_sensor.c, which makes the
 * same offer for the same reason: publishing through the sensor framework
 * drags in the whole UORB upper half, which is a lot to ask of a board that
 * wants nothing more than to read its own die temperature.
 *
 * Without CONFIG_RM57_ADC_TEMP_UORB - the default - each enabled sensor is
 * a plain character device, /dev/temp<devno>, and one read() hands back a
 * whole struct rm57_adc_temp_s: the raw code, the calibrated value, and the
 * flag saying whether the calibration was usable.  Nothing outside this
 * chip understands that record, but nothing is lost from it either, and it
 * is the same record ANIOC_RM57_GET_TEMP returns.  A read runs a fresh
 * conversion every time and never reports end of file, so `cat /dev/temp0`
 * will happily spin forever; read a fixed length instead.
 *
 * With CONFIG_RM57_ADC_TEMP_UORB the sensors are registered with the sensor
 * framework as /dev/uorb/sensor_temp<devno> instead, which costs the
 * calibration flag - struct sensor_temp has nowhere to put it - so an
 * uncalibrated sensor has to fail the read rather than qualify it.
 *
 * Two choices in the UORB path are worth explaining.
 *
 * The type is SENSOR_TYPE_TEMPERATURE rather than
 * SENSOR_TYPE_AMBIENT_TEMPERATURE.  These sensors measure the junction, not
 * the room, so that is the honest description; it also happens to be the
 * only one the sensortest application can open, because its node-name table
 * knows "temp" and not "ambient_temp".
 *
 * The data model is fetch-on-demand: a conversion is run when someone reads,
 * rather than by a worker thread pushing samples.  Temperature moves slowly
 * and the conversion is cheap, so a background thread would only burn cycles
 * and steal the ADC more often than necessary.  Because ops->fetch is set,
 * the upper half puts notify_event into the union that would otherwise hold
 * push_event, and push_event must never be called from here.
 *
 * That choice comes with a constraint worth stating plainly, because it is
 * not obvious and it is easy to "fix" wrongly: a fetch-model sensor only
 * serves non-blocking readers.  sensor_read() sends a blocking reader to
 * wait on a per-user semaphore that starts at zero and that only
 * notify_event posts, and a fetch driver has nothing to post it from - the
 * conversion it would report is precisely what the reader is blocked
 * waiting for.  Posting from activate() does not help either: the upper
 * half calls activate() before it has created the semaphore or linked the
 * user into its list, so the post lands nowhere.  sensor_poll() covers the
 * gap the other way instead, returning POLLIN immediately for an O_NONBLOCK
 * fetch driver, which is how sensortest works.  So `cat` on one of these
 * nodes will hang; that is the framework's fetch model, not a bug here, and
 * the reference driver behaves the same way.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nuttx.h>
#include <nuttx/fs/fs.h>

#ifdef CONFIG_RM57_ADC_TEMP_UORB
#  include <nuttx/sensors/sensor.h>
#  include <nuttx/uorb.h>
#  include <fixedmath.h>
#endif

#include "rm57_adc.h"

#ifdef CONFIG_RM57_ADC_TEMPSENSOR

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rm57_tempsensor_s
{
#ifdef CONFIG_RM57_ADC_TEMP_UORB
  struct sensor_lowerhalf_s lower;  /* Reached with container_of() */

  /* An uncalibrated sensor fails every fetch for the same reason, and a
   * reader that ignores the error retries as fast as it can.  Complain once
   * rather than at the speed of a 9600 baud console.
   */

  bool    warned;
#endif
  uint8_t sensor;                   /* Which on-die sensor, 0-2 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_RM57_ADC_TEMP_UORB
static int rm57_temp_activate(struct sensor_lowerhalf_s *lower,
                              struct file *filep, bool enable);
static int rm57_temp_fetch(struct sensor_lowerhalf_s *lower,
                           struct file *filep, char *buffer,
                           size_t buflen);
static int rm57_temp_getinfo(struct sensor_lowerhalf_s *lower,
                             struct file *filep,
                             struct sensor_device_info_s *info);
#else
static ssize_t rm57_temp_read(struct file *filep, char *buffer,
                              size_t buflen);
#endif

static void rm57_adc_temp_probe(int sensor);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_RM57_ADC_TEMP_UORB
static const struct sensor_ops_s g_rm57_temp_ops =
{
  .activate = rm57_temp_activate,
  .fetch    = rm57_temp_fetch,
  .get_info = rm57_temp_getinfo,

  /* set_interval is deliberately absent.  The ADC has no free-running rate
   * for these channels - a conversion happens when someone reads - so there
   * is no period to set.  Leaving it NULL makes SNIOC_SET_INTERVAL a silent
   * success in the upper half, which is what a caller like sensortest
   * expects; returning -ENOTSUP from a stub would be a lie about a period
   * that simply does not exist.
   */
};
#else
static const struct file_operations g_rm57_temp_fops =
{
  .read = rm57_temp_read,
};
#endif

static struct rm57_tempsensor_s g_tempsensors[RM57_ADC_NTEMPSENSORS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_RM57_ADC_TEMP_UORB

/****************************************************************************
 * Name: rm57_temp_celsius
 *
 * Description:
 *   Convert milli-Celsius to the sensor framework's data type.  Under the
 *   fixed point configuration the whole and fractional parts have to be
 *   scaled separately: itob16() of a milli-Celsius value would overflow the
 *   16.16 integer part well inside the device's operating range.
 *
 ****************************************************************************/

static sensor_data_t rm57_temp_celsius(int32_t millicel)
{
#ifdef CONFIG_SENSORS_USE_B16
  /* C truncates toward zero and gives the remainder the dividend's sign, so
   * the two halves stay consistent below freezing.
   */

  return itob16(millicel / 1000) + b16divi(itob16(millicel % 1000), 1000);
#else
  return (sensor_data_t)millicel / 1000.0f;
#endif
}

/****************************************************************************
 * Name: rm57_temp_activate
 *
 * Description:
 *   Called by the upper half when the first subscriber opens the node and
 *   again when the last one closes it.  The sensors are muxed onto their
 *   channels once at ADC bring-up and cost nothing while idle, so there is
 *   genuinely no hardware to switch here.
 *
 *   The op is kept rather than left NULL because arming a fresh complaint
 *   about missing calibration for each new subscriber is worth the few
 *   lines - see rm57_temp_fetch().
 *
 ****************************************************************************/

static int rm57_temp_activate(struct sensor_lowerhalf_s *lower,
                              struct file *filep, bool enable)
{
  struct rm57_tempsensor_s *priv =
    container_of(lower, struct rm57_tempsensor_s, lower);

  if (enable)
    {
      priv->warned = false;
    }

  return OK;
}

/****************************************************************************
 * Name: rm57_temp_fetch
 *
 * Description:
 *   Run one conversion and hand back a sensor_temp record.  Note the return
 *   type is int, not ssize_t: struct sensor_ops_s declares it that way.
 *
 ****************************************************************************/

static int rm57_temp_fetch(struct sensor_lowerhalf_s *lower,
                           struct file *filep, char *buffer, size_t buflen)
{
  struct rm57_tempsensor_s *priv =
    container_of(lower, struct rm57_tempsensor_s, lower);
  struct rm57_adc_temp_s temp;
  struct sensor_temp report;
  int ret;

  if (buffer == NULL || buflen < sizeof(report))
    {
      return -EINVAL;
    }

  ret = rm57_adc_readtemp(priv->sensor, &temp);
  if (ret < 0)
    {
      return ret;
    }

  if (!temp.calibrated)
    {
      /* Without factory calibration the raw code cannot be turned into a
       * temperature, and struct sensor_temp has no way to say so.  Report
       * the failure rather than publishing a number that means nothing;
       * the character device and the ioctl both hand back the raw code and
       * let the caller decide.  rm57_adc_temp_register() has already logged
       * the calibration points, so the reason is on the console once.
       */

      if (!priv->warned)
        {
          priv->warned = true;
          snerr("ERROR: Sensor %d has no usable OTP calibration (raw %u)\n",
                priv->sensor, temp.raw);
        }

      return -ENODATA;
    }

  report.timestamp   = sensor_get_timestamp();
  report.temperature = rm57_temp_celsius(temp.millicel);

  memcpy(buffer, &report, sizeof(report));

  return sizeof(report);
}

/****************************************************************************
 * Name: rm57_temp_getinfo
 *
 * Description:
 *   Describe the sensor to the framework.  The value fields are
 *   sensor_data_t, which is a float only when CONFIG_SENSORS_USE_B16 is off;
 *   under B16 a float literal would be truncated to a raw fixed point word,
 *   turning 130 degrees into 130/65536 of one.  So they go through the same
 *   conversion the readings do.
 *
 ****************************************************************************/

static int rm57_temp_getinfo(struct sensor_lowerhalf_s *lower,
                             struct file *filep,
                             struct sensor_device_info_s *info)
{
  struct rm57_tempsensor_s *priv =
    container_of(lower, struct rm57_tempsensor_s, lower);

  memset(info, 0, sizeof(*info));

  /* Operating junction temperature range, SPNS215 recommended operating
   * conditions.  min_delay is -1 because this is a one-shot sensor: it has
   * no free-running rate, a conversion happens when someone reads.
   */

  info->version    = 1;
  info->max_range  = rm57_temp_celsius(130000);
  info->min_delay  = -1;
  info->max_delay  = 0;
  info->resolution = rm57_temp_celsius(0); /* Set by the calibration slope */
  info->power      = rm57_temp_celsius(0);

  snprintf(info->name, sizeof(info->name), "temp%d", priv->sensor + 1);
  strlcpy(info->vendor, "Texas Instruments", sizeof(info->vendor));

  return OK;
}

#else /* CONFIG_RM57_ADC_TEMP_UORB */

/****************************************************************************
 * Name: rm57_temp_read
 *
 * Description:
 *   Run one conversion and hand back the whole struct rm57_adc_temp_s.  A
 *   partial record would be useless, so a short buffer is an error rather
 *   than a truncated read, and the file position is ignored: every read is
 *   a fresh measurement and there is no end of file to reach.
 *
 ****************************************************************************/

static ssize_t rm57_temp_read(struct file *filep, char *buffer,
                              size_t buflen)
{
  struct inode *inode = filep->f_inode;
  struct rm57_tempsensor_s *priv = inode->i_private;
  struct rm57_adc_temp_s temp;
  int ret;

  if (buffer == NULL || buflen < sizeof(temp))
    {
      return -EINVAL;
    }

  ret = rm57_adc_readtemp(priv->sensor, &temp);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(buffer, &temp, sizeof(temp));
  return (ssize_t)sizeof(temp);
}

#endif /* CONFIG_RM57_ADC_TEMP_UORB */

/****************************************************************************
 * Name: rm57_adc_temp_probe
 *
 * Description:
 *   Log one sensor's factory calibration as it was found in OTP, and take
 *   one reading through it.
 *
 *   This is here for bring-up.  A sensor that reports itself uncalibrated
 *   gives the application no way to tell blank OTP from a curve the driver
 *   rejected - and under the UORB interface it gives nothing at all, because
 *   struct sensor_temp has no room for the qualification and the read simply
 *   fails.  Logging the points once at registration answers that question
 *   without a debugger.
 *
 *   It doubles as the check on errata L2FMC#5: these are the only OTP reads
 *   the port makes, and if the region were mapped as normal memory they
 *   would return instruction opcodes instead of data.  Temperatures in the
 *   250-450K range are what says the strongly-ordered mapping is doing its
 *   job.
 *
 *   The trial conversion catches the other way a node can be born dead: a
 *   sensor lives on a particular ADC module, and that module is only brought
 *   up by one of its own conversion groups being registered.  Enable
 *   RM57_ADC2 with no ADC2 group and sensors 2 and 3 will return -ENODEV
 *   from every read, silently, forever.  Better to say so once here.
 *
 ****************************************************************************/

static void rm57_adc_temp_probe(int sensor)
{
  uint16_t val[RM57_ADC_TEMPCAL_NPOINTS];
  uint16_t kelvin[RM57_ADC_TEMPCAL_NPOINTS];
  struct rm57_adc_temp_s temp;
  int ret;
  int i;

  ret = rm57_adc_tempcal_points(sensor, val, kelvin);

  for (i = 0; i < RM57_ADC_TEMPCAL_NPOINTS; i++)
    {
      sninfo("Sensor %d OTP point %d: %uK (%d mC) reads %u\n",
             sensor + 1, i + 1, kelvin[i],
             (int)kelvin[i] * 1000 - 273150, val[i]);
    }

  if (ret < 0)
    {
      snwarn("WARNING: Sensor %d calibration unusable (%d); readings will "
             "be raw only\n", sensor + 1, ret);
    }

  ret = rm57_adc_readtemp(sensor, &temp);
  if (ret < 0)
    {
      snwarn("WARNING: Sensor %d trial conversion failed: %d\n",
             sensor + 1, ret);
    }
  else
    {
      sninfo("Sensor %d reads raw %u, %" PRId32 " mC%s\n", sensor + 1,
             temp.raw, temp.millicel,
             temp.calibrated ? "" : " (uncalibrated)");
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_adc_temp_register
 *
 * Description:
 *   Register one on-die temperature sensor as /dev/uorb/sensor_temp<devno>
 *   or, without CONFIG_RM57_ADC_TEMP_UORB, as
 *   CONFIG_RM57_ADC_TEMP_PATH<devno>.
 *
 ****************************************************************************/

int rm57_adc_temp_register(int sensor, int devno)
{
  struct rm57_tempsensor_s *priv;
#ifndef CONFIG_RM57_ADC_TEMP_UORB
  char devpath[32];
#endif
  int ret;

  if (sensor < 0 || sensor >= RM57_ADC_NTEMPSENSORS)
    {
      return -EINVAL;
    }

  priv = &g_tempsensors[sensor];
  priv->sensor = (uint8_t)sensor;

  rm57_adc_temp_probe(sensor);

#ifdef CONFIG_RM57_ADC_TEMP_UORB
  priv->warned     = false;
  priv->lower.ops  = &g_rm57_temp_ops;
  priv->lower.type = SENSOR_TYPE_TEMPERATURE;

  /* nbuffer is forced to zero by the upper half for a fetch-model driver;
   * set it anyway so the intent is not mistaken for an oversight.
   */

  priv->lower.nbuffer = 0;

  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register temp sensor %d as devno %d: %d\n",
            sensor, devno, ret);
      return ret;
    }

  sninfo("Temperature sensor %d registered as /dev/uorb/sensor_temp%d\n",
         sensor + 1, devno);
#else
  snprintf(devpath, sizeof(devpath), "%s%d", CONFIG_RM57_ADC_TEMP_PATH,
           devno);

  /* Read only: there is nothing a writer could say to a temperature */

  ret = register_driver(devpath, &g_rm57_temp_fops, 0444, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register temp sensor %d as %s: %d\n",
           sensor, devpath, ret);
      return ret;
    }

  sninfo("Temperature sensor %d registered as %s\n", sensor + 1, devpath);
#endif

  return OK;
}

#endif /* CONFIG_RM57_ADC_TEMPSENSOR */
