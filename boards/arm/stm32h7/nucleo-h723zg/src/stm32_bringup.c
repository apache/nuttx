/****************************************************************************
 * boards/arm/stm32h7/nucleo-h723zg/src/stm32_bringup.c
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

#include <sys/types.h>
#include <syslog.h>
#include <errno.h>

#include <arch/board/board.h>

#include <nuttx/fs/fs.h>

#ifdef CONFIG_CDCACM
#  include <nuttx/usb/cdcacm.h>
#endif

#include "nucleo-h723zg.h"

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#if defined(CONFIG_USERLED) && !defined(CONFIG_ARCH_LEDS)
#  include <nuttx/leds/userled.h>
#endif

#include "stm32.h"
#include "board_common.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = OK;

  /* Initialize the board features common to all STM32 boards */

  ret = stm32_common_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_common_initialize() failed: %d\n",
             ret);
    }

  UNUSED(ret);

#ifdef CONFIG_INPUT_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif /* CONFIG_INPUT_BUTTONS */

#if defined(CONFIG_USERLED) && !defined(CONFIG_ARCH_LEDS)
  /* Register the USERLED driver */

  ret = userled_lower_initialize(LED_DRIVER_PATH);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif /* CONFIG_USERLED */

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = stm32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_adc_setup failed: %d\n", ret);
    }
#endif /* CONFIG_ADC */

#ifdef CONFIG_DEV_GPIO
  /* Register the GPIO driver */

  ret = stm32_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
      return ret;
    }
#endif

#if defined(CONFIG_CDCACM) && !defined(CONFIG_CDCACM_CONSOLE) && \
    !defined(CONFIG_CDCACM_COMPOSITE)
  /* Initialize CDCACM */

  syslog(LOG_INFO, "Initialize CDCACM device\n");

  ret = cdcacm_initialize(0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: cdcacm_initialize failed: %d\n", ret);
    }
#endif /* CONFIG_CDCACM & !CONFIG_CDCACM_CONSOLE */

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = stm32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_pwm_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_NET_OA_TC6
  stm32_oa_tc6_initialize();
#endif

  return OK;
}
