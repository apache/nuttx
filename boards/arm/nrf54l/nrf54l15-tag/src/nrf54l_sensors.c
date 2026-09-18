/****************************************************************************
 * boards/arm/nrf54l/nrf54l15-tag/src/nrf54l_sensors.c
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
#include <syslog.h>

#ifdef CONFIG_SENSORS_BMI270
#  include <nuttx/sensors/bmi270.h>
#  include "nrf54l_spi.h"
#endif
#ifdef CONFIG_SENSORS_BME688
#  include <nuttx/sensors/bme688.h>
#endif
#ifdef CONFIG_SENSORS_ADXL367
#  include <nuttx/sensors/adxl367.h>
#endif

#include "nrf54l_i2c.h"
#include "nrf54l15-tag.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BME688
/****************************************************************************
 * Name: nrf54l_bme688_init
 ****************************************************************************/

static int nrf54l_bme688_init(struct i2c_master_s *i2c)
{
  struct bme688_config_s config =
  {
    0
  };

  config.temp_os         = BME688_OS_2X;
#ifndef CONFIG_BME688_DISABLE_PRESS_MEAS
  config.press_os        = BME688_OS_16X;
#endif
#ifndef CONFIG_BME688_DISABLE_HUM_MEAS
  config.hum_os          = BME688_OS_1X;
#endif
#ifdef CONFIG_BME688_ENABLE_IIR_FILTER
  config.filter_coef    = BME688_FILTER_COEF3;
#endif
#ifndef CONFIG_BME688_DISABLE_GAS_MEAS
  config.target_temp     = 300;
  config.heater_duration = 100;
  config.nb_conv         = 0;
  config.amb_temp        = 30;
#endif

  return bme688_register(0, i2c, &config);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_sensors_init
 *
 * Description:
 *   Initialize and register the onboard sensors.
 *
 ****************************************************************************/

int nrf54l_sensors_init(void)
{
#ifdef CONFIG_SENSORS_BMI270
  struct spi_dev_s *spi;
#endif
#if defined(CONFIG_SENSORS_BME688) || defined(CONFIG_SENSORS_ADXL367)
  struct i2c_master_s *i2c;
#endif
  int ret = OK;
  int result = OK;

#ifdef CONFIG_SENSORS_BMI270
  spi = nrf54l_spibus_initialize(2);
  if (spi == NULL)
    {
      ret = -ENODEV;
    }
  else
    {
#ifdef CONFIG_SENSORS_BMI270_UORB
      ret = bmi270_register_uorb(0, spi);
#else
      ret = bmi270_register("/dev/bmi270", spi);
#endif
    }

  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: BMI270 registration failed: %d\n", ret);
      result = ret;
    }
#endif

#if defined(CONFIG_SENSORS_BME688) || defined(CONFIG_SENSORS_ADXL367)
  i2c = nrf54l_i2cbus_initialize(1);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

#ifdef CONFIG_SENSORS_BME688
  ret = nrf54l_bme688_init(i2c);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: BME688 registration failed: %d\n", ret);
      result = ret;
    }
#endif

#ifdef CONFIG_SENSORS_ADXL367
  ret = adxl367_register(1, i2c, 0x1d);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: ADXL367 registration failed: %d\n", ret);
      result = ret;
    }
#endif
#endif

  UNUSED(ret);
  return result;
}
