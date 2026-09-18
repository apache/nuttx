/****************************************************************************
 * boards/arm/nrf54l/nrf54l15-tag/src/nrf54l_bringup.c
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

#include <syslog.h>

#include <nuttx/fs/fs.h>

#ifdef CONFIG_INPUT_BUTTONS_LOWER
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_USERLED_LOWER
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_TIMER
#  include "nrf54l_tim_lowerhalf.h"
#endif

#include "nrf54l15-tag.h"

#ifdef CONFIG_NRF54L_SOFTDEVICE_CONTROLLER
#  include "nrf54l_sdc.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_bringup
 *
 * Description:
 *   Mount optional file systems and register enabled board devices.
 *
 ****************************************************************************/

int nrf54l_bringup(void)
{
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, NRF54L_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: procfs mount failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_USERLED_LOWER
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: LED registration failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS_LOWER
  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Button registration failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SYSTEM_I2CTOOL
  ret = nrf54l_i2ctool();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: I2C registration failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_NRF54L_SAADC
  ret = nrf54l_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: ADC registration failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_TIMER) && defined(CONFIG_NRF54L_TIMER0)
  /* Configure TIMER driver */

  ret = nrf54l_timer_initialize("/dev/timer0", 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize timer driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS
  ret = nrf54l_sensors_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Sensor registration failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_NRF54L_SOFTDEVICE_CONTROLLER
  /* Select antenna 1 on the SKY13348 RF switch */

  nrf54l_gpio_config(GPIO_ANT2);
  nrf54l_gpio_config(GPIO_ANT1);

  /* Register the Bluetooth controller */

  ret = nrf54l_sdc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: SDC initialization failed: %d\n", ret);
      return ret;
    }
#endif

  UNUSED(ret);
  return OK;
}
