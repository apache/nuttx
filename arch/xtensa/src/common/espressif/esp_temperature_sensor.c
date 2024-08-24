/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_temperature_sensor.c
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
#include <nuttx/arch.h>

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
  const struct file_operations *ops;                     /* Standard file operations */
  const temperature_sensor_attribute_t *tsens_attribute; /* Attribute struct of the common layer */
  struct esp_temp_sensor_config_t cfg;                   /* Configuration struct of the common layer */
  temperature_sensor_clk_src_t clk_src;                  /* Clock source to use */
  int module;                                            /* Peripheral module */
  int refs;                                              /* Reference count */
  mutex_t lock;                                          /* Mutual exclusion mutex */

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
static int esp_temperature_sensor_get_celsius(struct file *filep,
                                              char *buffer,
                                              size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static float g_delta_t = NAN;

static const struct file_operations g_esp_temp_sensor_fops =
{
  NULL,                               /* open */
  NULL,                               /* close */
  esp_temperature_sensor_get_celsius, /* read */
  NULL,                               /* write */
};

struct esp_temp_priv_s esp_temp_priv =
{
  .ops = &g_esp_temp_sensor_fops,
  .tsens_attribute = NULL,
  .cfg =
  {
    0
  },
  .clk_src = TEMPERATURE_SENSOR_CLK_SRC_DEFAULT,
  .module = PERIPH_TEMPSENSOR_MODULE,
  .refs = 0,
  .lock = NXMUTEX_INITIALIZER,
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
      syslog(LOG_ERR, "Out of testing range");
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
      syslog(LOG_WARNING, "Calibration failed");
      g_delta_t = 0;
      return ERROR;
    }

  syslog(LOG_INFO, "delta_T = %f", g_delta_t);
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
 *   This function registers the internal temperature sensor to
 *   /dev/temp driver.
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
  register_driver(CONFIG_ESPRESSIF_TEMP_PATH, &g_esp_temp_sensor_fops,
                  0666, priv);
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
      syslog(LOG_ERR, "Table sort failed");
      goto err;
    }

  priv->cfg.range_min = cfg.range_min;
  priv->cfg.range_max = cfg.range_max;
  ret = temperature_sensor_choose_best_range(priv);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Cannot select the correct range");
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
 *   filep  - The pointer of file, represents each user using the sensor
 *   buffer - Buffer to save temperature sensor value in degrees Celsius
 *   buflen - Length of the buffer
 *
 * Returned Value:
 *   Returns OK on success; a negated errno value on failure
 *
 ****************************************************************************/

static int esp_temperature_sensor_get_celsius(struct file *filep,
                                              char *buffer,
                                              size_t buflen)
{
  uint32_t tsens_out;
  struct inode *inode = filep->f_inode;
  struct esp_temp_priv_s *priv = inode->i_private;
  int *out = (int *)buffer;

  nxmutex_lock(&priv->lock);
  if (priv == NULL)
    {
      syslog(LOG_WARNING, "Temperature sensor has not been installed");
      goto err;
    }

  esp_temperature_sensor_enable(priv);
  if (priv->tempstate != TEMP_SENSOR_ENABLE)
    {
      syslog(LOG_WARNING, "Temperature sensor not enabled");
      goto err;
    }

  tsens_out = temperature_sensor_ll_get_raw_value();
  syslog(LOG_INFO, "Temperature sensor raw value: %d", tsens_out);

  *out = temperature_sensor_parse_raw_value(tsens_out,
                                     priv->tsens_attribute->offset);
  esp_temperature_sensor_disable(priv);

  if (*out < priv->tsens_attribute->range_min ||
      *out > priv->tsens_attribute->range_max)
    {
      syslog(LOG_WARNING, "Temperature sensor value is out of range");
      goto err;
    }

  nxmutex_unlock(&priv->lock);
  return OK;

err:
  nxmutex_unlock(&priv->lock);
  return ERROR;
}

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

  nxmutex_lock(&priv->lock);

  if (priv->refs++ != 0)
    {
      nxmutex_unlock(&priv->lock);
      syslog(LOG_INFO, "Temperature sensor previously initialized."
             "Handler: %p\n", priv);
    }
  else
    {
      ret = esp_temperature_sensor_install(priv, cfg);
      if (ret != OK)
        {
          syslog(LOG_ERR, "Temperature sensor initialization failed!");
          nxmutex_unlock(&priv->lock);
          return ret;
        }
    }

  esp_temp_sensor_register(priv);
  nxmutex_unlock(&priv->lock);

  syslog(LOG_INFO, "Temperature sensor initialized! Handler: %p\n", priv);

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
