/****************************************************************************
 * boards/arm/nrf53/nrf5340-dk/src/nrf53_bringup.c
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

#include <sys/types.h>
#include <syslog.h>

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_NRF53_SOFTDEVICE_CONTROLLER
#  include "nrf53_sdc.h"
#endif

#ifdef CONFIG_RPTUN
#  include "nrf53_rptun.h"
#endif

#include "nrf5340-dk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NRF53_TIMER (0)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int nrf53_bringup(void)
{
  int ret;

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  ret = userled_lower_initialize(CONFIG_EXAMPLES_LEDS_DEVPATH);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_RPTUN
#ifdef CONFIG_NRF53_APPCORE
  nrf53_rptun_init("nrf53-shmem", "appcore");
#else
  nrf53_rptun_init("nrf53-shmem", "netcore");
#endif
#endif

#if defined(CONFIG_TIMER) && defined(CONFIG_NRF53_TIMER)
  /* Configure TIMER driver */

  ret = nrf53_timer_driver_setup("/dev/timer0", NRF53_TIMER);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_PWM
  /* Configure PWM driver */

  ret = nrf53_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize PWM driver: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_ADC
  /* Configure ADC driver */

  ret = nrf53_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize ADC driver: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_NRF53_SOFTDEVICE_CONTROLLER
  ret = nrf53_sdc_initialize();

  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: nrf53_sdc_initialize() failed: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
