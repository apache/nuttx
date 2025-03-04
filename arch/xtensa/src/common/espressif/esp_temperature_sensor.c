/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_temperature_sensor.c
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
#include <nuttx/nuttx.h>

#ifdef CONFIG_ESPRESSIF_TEMP
#include <stdio.h>
#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/can/can.h>
#include <nuttx/signal.h>
#include <nuttx/kthread.h>
#ifdef CONFIG_ESPRESSIF_TEMP_UORB
#include <nuttx/sensors/sensor.h>
#endif

#include "xtensa.h"
#include "espressif/esp_temperature_sensor.h"

#include "esp_clk.h"

#include "periph_ctrl.h"
#include "soc/gpio_sig_map.h"
#include "soc/reg_base.h"
#include "soc/periph_defs.h"
#include "hal/regi2c_ctrl.h"
#include "hal/temperature_sensor_ll.h"
#include "hal/temperature_sensor_types.h"
#include "soc/temperature_sensor_periph.h"
#include "esp_efuse_rtc_calib.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP_TEMP_MIN_INTERVAL 30000

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Temperature sensor states */

enum esp_tempstate_e
{
  TEMP_SENSOR_INIT,
  TEMP_SENSOR_ENABLE,
};

/* Temperature sensor Private Data */

struct esp_temp_priv_s
{
#ifndef CONFIG_ESPRESSIF_TEMP_UORB
  const struct file_operations *ops;                     /* Standard file operations */
#endif
  const temperature_sensor_attribute_t *tsens_attribute; /* Attribute struct of the common layer */
  struct esp_temp_sensor_config_t cfg;                   /* Configuration struct of the common layer */
  temperature_sensor_clk_src_t clk_src;                  /* Clock source to use */
  int module;                                            /* Peripheral module */
  int refs;                                              /* Reference count */
  mutex_t lock;                                          /* Mutual exclusion mutex */
#ifdef CONFIG_ESPRESSIF_TEMP_UORB
  struct sensor_lowerhalf_s lower;                       /* Lower half sensor driver. */
  uint32_t interval;                                     /* Sensor acquisition interval. */
#ifdef CONFIG_ESPRESSIF_TEMP_UORB_POLL
  bool                       enabled;
  sem_t                      run;
#endif
#endif
  /* Temperature sensor work state (see enum esp_tempstate_e) */

  volatile enum esp_tempstate_e tempstate;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int inline temperature_sensor_accuracy_compare(const void *p1,
                                                      const void *p2);
static int temperature_sensor_attribute_table_sort(void);
static int temperature_sensor_choose_best_range(
                                      struct esp_temp_priv_s *priv);
static int temperature_sensor_read_delta_t(void);
static float temperature_sensor_parse_raw_value(uint32_t tsens_raw,
                                                const int dac_offset);
static void esp_temperature_sensor_enable(struct esp_temp_priv_s *priv);
static void esp_temperature_sensor_disable(struct esp_temp_priv_s *priv);
static void esp_temp_sensor_register(struct esp_temp_priv_s *priv);
static int esp_temperature_sensor_install(struct esp_temp_priv_s *priv,
  struct esp_temp_sensor_config_t cfg);
static void esp_temperature_sensor_uninstall(struct esp_temp_priv_s *priv);
static int esp_temperature_sensor_get_celsius(struct esp_temp_priv_s *priv,
                                              int *buffer);
#ifdef CONFIG_ESPRESSIF_TEMP_UORB
static int esp_temperature_sensor_set_interval(
  struct sensor_lowerhalf_s *lower,
  struct file *filep,
  uint32_t *period_us);
static int esp_temperature_sensor_activate(
  struct sensor_lowerhalf_s *lower,
  struct file *filep,
  bool enable);
#ifndef CONFIG_ESPRESSIF_TEMP_UORB_POLL
static int esp_temperature_sensor_fetch(struct sensor_lowerhalf_s *lower,
                                        struct file *filep,
                                        char *buffer, size_t buflen);
#endif
#else
static int esp_temperature_sensor_read(struct file *filep,
                                       char *buffer,
                                       size_t buflen);
#endif /* CONFIG_ESPRESSIF_TEMP_UORB */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static float g_delta_t = NAN;

#ifndef CONFIG_ESPRESSIF_TEMP_UORB
static const struct file_operations g_esp_temp_sensor_fops =
{
  NULL,                               /* open */
  NULL,                               /* close */
  esp_temperature_sensor_read,        /* read */
  NULL,                               /* write */
};
#else
static const struct sensor_ops_s g_esp_temp_sensor_sops =
{
  .activate     = esp_temperature_sensor_activate,     /* Enable/disable sensor. */
#ifndef CONFIG_ESPRESSIF_TEMP_UORB_POLL
  .fetch        = esp_temperature_sensor_fetch,
#endif
  .set_interval = esp_temperature_sensor_set_interval, /* Set output data period. */
};
#endif /* CONFIG_ESPRESSIF_TEMP_UORB */

struct esp_temp_priv_s esp_temp_priv =
{
#ifndef CONFIG_ESPRESSIF_TEMP_UORB
  .ops = &g_esp_temp_sensor_fops,
#endif
  .tsens_attribute = NULL,
  .cfg =
  {
    0
  },
  .clk_src = TEMPERATURE_SENSOR_CLK_SRC_DEFAULT,
  .module = PERIPH_TEMPSENSOR_MODULE,
  .refs = 0,
  .lock = NXMUTEX_INITIALIZER,
#ifdef CONFIG_ESPRESSIF_TEMP_UORB
  .lower =
  {
    0
  },
  .interval = ESP_TEMP_MIN_INTERVAL,
#endif
  .tempstate = 0,
};

static temperature_sensor_attribute_t
  g_tsens_attributes[TEMPERATURE_SENSOR_ATTR_RANGE_NUM];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: temperature_sensor_accuracy_compare
 *
 * Description:
 *   This function compares error values to choose least error rated
 *   measurement range for the temperature sensor.
 *
 * Input Parameters:
 *   p1 - First value to compare with other value
 *   p2 - Second value to compare with outher value
 *
 * Returned Value:
 *   Returns -1 if the first value has smaller error rate; 1 otherwise
 *
 ****************************************************************************/

static int inline temperature_sensor_accuracy_compare(const void *p1,
                                                      const void *p2)
{
  return ((*(temperature_sensor_attribute_t *)p1).error_max <
      (*(temperature_sensor_attribute_t *)p2).error_max) ? -1 : 1;
}

/****************************************************************************
 * Name: temperature_sensor_attribute_table_sort
 *
 * Description:
 *   This function sorts temperature sensor attributes regarding error rate.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns OK on success; a negated errno value on failure
 *
 ****************************************************************************/

static int temperature_sensor_attribute_table_sort(void)
{
  for (int i = 0 ; i < TEMPERATURE_SENSOR_ATTR_RANGE_NUM; i++)
    {
      g_tsens_attributes[i] = temperature_sensor_attributes[i];
    }

  /* Sort from small to large by error_max */

  qsort(g_tsens_attributes, TEMPERATURE_SENSOR_ATTR_RANGE_NUM,
        sizeof(g_tsens_attributes[0]), temperature_sensor_accuracy_compare);
  return OK;
}

/****************************************************************************
 * Name: temperature_sensor_choose_best_range
 *
 * Description:
 *   This function selects least error rated temperature sensor attribute
 *   for given mesurement range.
 *
 * Input Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 * Returned Value:
 *   Returns OK on success; a negated errno value on failure
 *
 ****************************************************************************/

static int temperature_sensor_choose_best_range(struct esp_temp_priv_s *priv)
{
  for (int i = 0 ; i < TEMPERATURE_SENSOR_ATTR_RANGE_NUM; i++)
    {
      if ((priv->cfg.range_min >= g_tsens_attributes[i].range_min) &&
          (priv->cfg.range_max <= g_tsens_attributes[i].range_max))
        {
          priv->tsens_attribute = &g_tsens_attributes[i];
          break;
        }
    }

  if (priv->tsens_attribute == NULL)
    {
      snerr("Out of testing range");
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: temperature_sensor_read_delta_t
 *
 * Description:
 *   This function reads the temperature sensor calibration number delta_T
 *   stored in the efuse.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns OK on success; a negated errno value on failure
 *
 ****************************************************************************/

static int temperature_sensor_read_delta_t(void)
{
  if (esp_efuse_rtc_calib_get_tsens_val(&g_delta_t) != OK)
    {
      snwarn("Calibration failed");
      g_delta_t = 0;
      return ERROR;
    }

  sninfo("delta_T = %f", g_delta_t);
  return OK;
}

/****************************************************************************
 * Name: temperature_sensor_parse_raw_value
 *
 * Description:
 *   This function converts raw temperature sensor data value
 *   to degrees Celsius
 *
 * Input Parameters:
 *   tsens_raw  - Raw value of temperature sensor data
 *   dac_offset - Offset value of DAC to compensate temperature measurement
 *
 * Returned Value:
 *   Temperature sensor data that is converted to degrees Celsius
 *
 ****************************************************************************/

static float temperature_sensor_parse_raw_value(uint32_t tsens_raw,
                                                const int dac_offset)
{
  if (isnan(g_delta_t))
    {
      temperature_sensor_read_delta_t();
    }

  return (TEMPERATURE_SENSOR_LL_ADC_FACTOR * (float)tsens_raw -
            TEMPERATURE_SENSOR_LL_DAC_FACTOR * dac_offset -
                TEMPERATURE_SENSOR_LL_OFFSET_FACTOR) - g_delta_t / 10.0;
}

/****************************************************************************
 * Name: esp_temperature_sensor_enable
 *
 * Description:
 *   This function enables the temperature sensor
 *
 * Input Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_temperature_sensor_enable(struct esp_temp_priv_s *priv)
{
  temperature_sensor_ll_clk_enable(true);
  temperature_sensor_ll_clk_sel(priv->clk_src);
  temperature_sensor_ll_enable(true);
  priv->tempstate = TEMP_SENSOR_ENABLE;
}

/****************************************************************************
 * Name: esp_temperature_sensor_disable
 *
 * Description:
 *   This function disables the temperature sensor
 *
 * Input Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_temperature_sensor_disable(struct esp_temp_priv_s *priv)
{
  temperature_sensor_ll_enable(false);
  priv->tempstate = TEMP_SENSOR_INIT;
}

/****************************************************************************
 * Name: esp_temp_sensor_register
 *
 * Description:
 *   This function registers the internal temperature sensor.
 *
 * Input Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_temp_sensor_register(struct esp_temp_priv_s *priv)
{
#ifndef CONFIG_ESPRESSIF_TEMP_UORB
  register_driver(CONFIG_ESPRESSIF_TEMP_PATH, &g_esp_temp_sensor_fops,
                  0666, priv);
#else
  sensor_register(&priv->lower, CONFIG_ESPRESSIF_TEMP_PATH_DEVNO);
#endif /* CONFIG_ESPRESSIF_TEMP_UORB */
}

/****************************************************************************
 * Name: esp_temperature_sensor_install
 *
 * Description:
 *   This function installs temperature sensor driver
 *
 * Input Parameters:
 *   priv - Pointer to the internal driver state structure.
 *   cfg - Configuration of measurement range for the temperature sensor
 *
 * Returned Value:
 *   Returns OK on success; a negated errno value on failure
 *
 ****************************************************************************/

static int esp_temperature_sensor_install(struct esp_temp_priv_s *priv,
                                        struct esp_temp_sensor_config_t cfg)
{
  int ret;
  periph_module_enable(priv->module);
  periph_module_reset(priv->module);

  ret = temperature_sensor_attribute_table_sort();
  if (ret < 0)
    {
      snerr("Table sort failed");
      goto err;
    }

  priv->cfg.range_min = cfg.range_min;
  priv->cfg.range_max = cfg.range_max;
  ret = temperature_sensor_choose_best_range(priv);
  if (ret < 0)
    {
      snerr("Cannot select the correct range");
      goto err;
    }

  regi2c_saradc_enable();
  temperature_sensor_ll_set_range(priv->tsens_attribute->reg_val);
  esp_temperature_sensor_disable(priv); /* Disable the sensor by default */

  priv->tempstate = TEMP_SENSOR_INIT;

  return OK;

err:
  esp_temperature_sensor_uninstall(priv);
  return ret;
}

/****************************************************************************
 * Name: esp_temperature_sensor_uninstall
 *
 * Description:
 *   Read temperature sensor data that is converted to degrees Celsius.
 *
 * Input Parameters:
 *   priv - Pointer to the internal driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_temperature_sensor_uninstall(struct esp_temp_priv_s *priv)
{
  regi2c_saradc_disable();
  periph_module_disable(priv->module);
}

/****************************************************************************
 * Name: esp_temperature_sensor_get_celsius
 *
 * Description:
 *   Read temperature sensor data that is converted to degrees Celsius.
 *
 * Input Parameters:
 *   priv   - Pointer to the internal driver state structure.
 *   buffer - Buffer to save temperature sensor value in degrees Celsius
 *
 * Returned Value:
 *   Returns OK on success; a negated errno value on failure
 *
 ****************************************************************************/

static int esp_temperature_sensor_get_celsius(struct esp_temp_priv_s *priv,
                                              int *buffer)
{
  uint32_t tsens_out;
  float *out = (float *)buffer;

  nxmutex_lock(&priv->lock);
  if (priv == NULL)
    {
      snwarn("Temperature sensor has not been installed");
      goto err;
    }

  esp_temperature_sensor_enable(priv);
  if (priv->tempstate != TEMP_SENSOR_ENABLE)
    {
      snwarn("Temperature sensor not enabled");
      goto err;
    }

  tsens_out = temperature_sensor_ll_get_raw_value();
  sninfo("Temperature sensor raw value: %ld", (long int)tsens_out);

  *out = temperature_sensor_parse_raw_value(tsens_out,
                                     priv->tsens_attribute->offset);
  esp_temperature_sensor_disable(priv);

  if (*out < priv->tsens_attribute->range_min ||
      *out > priv->tsens_attribute->range_max)
    {
      snwarn("Temperature sensor value is out of range");
      goto err;
    }

  nxmutex_unlock(&priv->lock);
  return OK;

err:
  nxmutex_unlock(&priv->lock);
  return ERROR;
}

#ifndef CONFIG_ESPRESSIF_TEMP_UORB
/****************************************************************************
 * Name: esp_temperature_sensor_read
 *
 * Description:
 *   Read temperature sensor data that is converted to degrees Celsius.
 *
 * Input Parameters:
 *   filep  - The pointer of file, represents each user using the sensor
 *   buffer - Buffer to save temperature sensor value in degrees Celsius
 *   buflen - Length of the buffer
 *
 * Returned Value:
 *   Returns OK on success; a negated errno value on failure
 *
 ****************************************************************************/

static int esp_temperature_sensor_read(struct file *filep,
                                       char *buffer,
                                       size_t buflen)
{
  struct inode *inode = filep->f_inode;
  struct esp_temp_priv_s *priv = inode->i_private;

  return esp_temperature_sensor_get_celsius(priv, (int *)buffer);
}
#else
#ifndef CONFIG_ESPRESSIF_TEMP_UORB_POLL
static int esp_temperature_sensor_fetch(struct sensor_lowerhalf_s *lower,
                                        struct file *filep,
                                        char *buffer, size_t buflen)
{
  struct esp_temp_priv_s *priv =
    container_of (lower, struct esp_temp_priv_s, lower);

  int ret;
  struct sensor_temp temp;
  float val = 0.0;

  if (buflen != sizeof(temp))
    {
      return -EINVAL;
    }

  esp_temperature_sensor_get_celsius(priv, (int *)&val);
  sninfo("temp = %d\n", (int)val);

  temp.timestamp = sensor_get_timestamp();
  temp.temperature = (int)val;

  memcpy(buffer, &temp, sizeof(temp));

  return buflen;
}
#endif /* CONFIG_ESPRESSIF_TEMP_UORB_POLL */

#ifdef CONFIG_ESPRESSIF_TEMP_UORB_POLL
/****************************************************************************
 * Name: esp_temperature_sensor_thread
 *
 * Description:
 *   Thread for performing interval measurement read.
 *
 * Parameter:
 *   argc - Number opf arguments
 *   argv - Pointer to argument list
 *
 ****************************************************************************/

static int esp_temperature_sensor_thread(int argc, char **argv)
{
  struct esp_temp_priv_s *priv =
      (struct esp_temp_priv_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
  struct sensor_temp temp;
  uint8_t data[8];
  int ret;
  float val = 0.0;

  while (true)
    {
      if ((!priv->enabled))
        {
          /* Waiting to be woken up */

          ret = nxsem_wait(&priv->run);
          if (ret < 0)
            {
              continue;
            }
        }

      /* Read sensor */

      if (priv->enabled)
        {
          esp_temperature_sensor_get_celsius(priv, (int *)&val);
          sninfo("temp = %d\n", (int)val);

          temp.timestamp = sensor_get_timestamp();
          temp.temperature = (int)val;

          priv->lower.push_event(priv->lower.priv, &temp, sizeof(temp));
        }

      /* Sleeping thread before fetching the next sensor data */

      nxsig_usleep(priv->interval);
    }

  return OK;
}
#endif /* CONFIG_ESPRESSIF_TEMP_UORB_POLL */

/****************************************************************************
 * Name: esp_temperature_sensor_set_interval
 *
 * Description:
 *   Set the sensor output data period in microseconds for a given sensor.
 *   If *period_us < min_delay it will be replaced by min_delay.
 *
 * Input Parameters:
 *   lower     - The instance of lower half sensor driver.
 *   filep     - The pointer of file, represents each user using the sensor.
 *   period_us - The time between report data, in us. It may by overwrite
 *               by lower half driver.
 *
 * Returned Value:
 *   Return OK(0) if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

static int esp_temperature_sensor_set_interval(
  struct sensor_lowerhalf_s *lower,
  struct file *filep,
  uint32_t *period_us)
{
  struct esp_temp_priv_s *priv = (struct esp_temp_priv_s *)lower;

  if (*period_us < ESP_TEMP_MIN_INTERVAL)
    {
      priv->interval = ESP_TEMP_MIN_INTERVAL;
      *period_us = priv->interval;
    }
  else
    {
      priv->interval = *period_us;
    }

  return OK;
}

/****************************************************************************
 * Name: esp_temperature_sensor_activate
 *
 * Description:
 *   Enable or disable sensor device. when enable sensor, sensor will
 *   work in  current mode(if not set, use default mode). when disable
 *   sensor, it will disable sense path and stop convert.
 *
 * Input Parameters:
 *   lower  - The instance of lower half sensor driver.
 *   filep  - The pointer of file, represents each user using the sensor.
 *   enable - true(enable) and false(disable).
 *
 * Returned Value:
 *   Return OK(0)  if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

static int esp_temperature_sensor_activate(
  struct sensor_lowerhalf_s *lower,
  struct file *filep,
  bool enable)
{
  struct esp_temp_priv_s *priv = (struct esp_temp_priv_s *)lower;
#ifdef CONFIG_ESPRESSIF_TEMP_UORB_POLL
  bool start_thread = false;
#endif

  /* Set accel output data rate. */

  if (enable)
    {
#ifdef CONFIG_ESPRESSIF_TEMP_UORB_POLL
      if (!priv->enabled)
        {
          start_thread = true;
        }
#endif

      esp_temperature_sensor_enable(priv);
    }
  else
    {
      /* Set suspend mode to sensors. */

        esp_temperature_sensor_disable(priv);
    }

#ifdef CONFIG_ESPRESSIF_TEMP_UORB_POLL
  priv->enabled = enable;

  if (start_thread)
    {
      /* Wake up the thread */

      nxsem_post(&priv->run);
    }
#endif

  return OK;
}
#endif /* CONFIG_ESPRESSIF_TEMP_UORB */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_temperature_sensor_initialize
 *
 * Description:
 *   This function initializes the internal temperature sensor with the
 *   provided configuration.
 *
 * Input Parameters:
 *   cfg - Configuration of measurement range for the temperature sensor
 *
 * Returned Value:
 *   Returns OK on success; a negated errno value on failure
 *
 ****************************************************************************/

int esp_temperature_sensor_initialize(struct esp_temp_sensor_config_t cfg)
{
  int ret = 0;
  struct esp_temp_priv_s *priv = &esp_temp_priv;
#ifdef CONFIG_ESPRESSIF_TEMP_UORB_POLL
  char *argv[2];
  char arg1[32];
#endif

  nxmutex_lock(&priv->lock);

  if (priv->refs++ != 0)
    {
      nxmutex_unlock(&priv->lock);
      sninfo("Temperature sensor previously initialized."
             "Handler: %p\n", priv);
    }
  else
    {
      ret = esp_temperature_sensor_install(priv, cfg);
      if (ret != OK)
        {
          snerr("Temperature sensor initialization failed!");
          nxmutex_unlock(&priv->lock);
          return ret;
        }
    }

#ifdef CONFIG_ESPRESSIF_TEMP_UORB
  priv->lower.ops = &g_esp_temp_sensor_sops;
  priv->lower.type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  priv->interval = ESP_TEMP_MIN_INTERVAL;
#ifdef CONFIG_ESPRESSIF_TEMP_UORB_POLL
  priv->enabled = false;
  priv->interval = CONFIG_ESPRESSIF_TEMP_UORB_POLL_INTERVAL;

  nxsem_init(&priv->run, 0, 0);
#endif
#endif

  esp_temp_sensor_register(priv);
  nxmutex_unlock(&priv->lock);

  sninfo("Temperature sensor initialized! Handler: %p\n", priv);

#ifdef CONFIG_ESPRESSIF_TEMP_UORB_POLL
  /* Create thread for polling sensor data */

  snprintf(arg1, 16, "%p", priv);
  argv[0] = arg1;
  argv[1] = NULL;

  ret = kthread_create("esp_temperature_sensor_thread",
                       SCHED_PRIORITY_DEFAULT,
                       CONFIG_ESPRESSIF_TEMP_THREAD_STACKSIZE,
                       esp_temperature_sensor_thread,
                       argv);
  if (ret < 0)
    {
      kmm_free(priv);
      return ret;
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: esp_temperature_sensor_uninitialize
 *
 * Description:
 *   This function uninitializes the internal temperature sensor.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns OK on success; a negated errno value on failure
 *
 ****************************************************************************/

int esp_temperature_sensor_uninitialize(void)
{
  struct esp_temp_priv_s *priv = &esp_temp_priv;

  nxmutex_lock(&priv->lock);
  esp_temperature_sensor_uninstall(priv);
  nxmutex_unlock(&priv->lock);
  return OK;
}
#endif /* CONFIG_ESPRESSIF_TEMP */
