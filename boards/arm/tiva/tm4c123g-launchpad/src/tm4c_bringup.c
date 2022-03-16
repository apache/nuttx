/****************************************************************************
 * boards/arm/tiva/tm4c123g-launchpad/src/tm4c_bringup.c
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

#include "tm4c123g-launchpad.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tm4c_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int tm4c_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_TIVA_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = tm4c_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: tm4c_adc_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_TIVA_CAN
  /* Initialize CAN module and register the CAN driver(s) */

  ret = tm4c_can_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: tm4c_can_setup failed %d\n", ret);
    }
#endif

#ifdef HAVE_AT24
  /* Initialize the AT24 driver */

  ret = tm4c_at24_automount(AT24_MINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: tm4c_at24_automount failed: %d\n", ret);
      return ret;
    }
#endif /* HAVE_AT24 */

#ifdef CONFIG_TIVA_TIMER
  /* Initialize the timer driver */

  ret = tiva_timer_configure();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: tiva_timer_configure failed: %d\n", ret);
      return ret;
    }
#endif /* CONFIG_TIVA_TIMER */

#ifdef CONFIG_CAN_MCP2515
  /* Configure and initialize the MCP2515 CAN device */

  ret = tiva_mcp2515initialize("/dev/can0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_mcp2515initialize() failed: %d\n", ret);
      return ret;
    }
#endif

  UNUSED(ret);
  return ret;
}
