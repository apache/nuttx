/****************************************************************************
 * boards/arm/stm32h5/nucleo-h563zi/src/stm32_bringup.c
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
#include <nuttx/debug.h>

#include <nuttx/input/buttons.h>
#include <nuttx/leds/userled.h>
#include <nuttx/board.h>

#include "nucleo-h563zi.h"

#include <arch/board/board.h>

#include "board_common.h"

#if defined(CONFIG_CAPTURE) && defined(CONFIG_STM32_TIMX_CAP)
#  include <errno.h>
#  include <nuttx/timers/capture.h>
#  include "stm32_capture.h"
#  define HAVE_CAPTURE 1
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_capture_setup
 *
 * Description:
 *   Initialize and register the capture drivers of the timers that are
 *   configured for capture.  They are registered as /dev/cap0, /dev/cap1,
 *   etc. in the order of the timer numbers.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef HAVE_CAPTURE
static int stm32_capture_setup(void)
{
  struct cap_lowerhalf_s *lower[] =
    {
#ifdef CONFIG_STM32_TIM1_CAP
      stm32_cap_initialize(1),
#endif
#ifdef CONFIG_STM32_TIM2_CAP
      stm32_cap_initialize(2),
#endif
#ifdef CONFIG_STM32_TIM3_CAP
      stm32_cap_initialize(3),
#endif
#ifdef CONFIG_STM32_TIM4_CAP
      stm32_cap_initialize(4),
#endif
#ifdef CONFIG_STM32_TIM5_CAP
      stm32_cap_initialize(5),
#endif
#ifdef CONFIG_STM32_TIM8_CAP
      stm32_cap_initialize(8),
#endif
#ifdef CONFIG_STM32_TIM12_CAP
      stm32_cap_initialize(12),
#endif
#ifdef CONFIG_STM32_TIM15_CAP
      stm32_cap_initialize(15),
#endif
    };

  size_t count = sizeof(lower) / sizeof(lower[0]);
  size_t i;

  for (i = 0; i < count; i++)
    {
      if (lower[i] == NULL)
        {
          syslog(LOG_ERR, "ERROR: Failed to initialize a capture timer\n");
          return -ENODEV;
        }
    }

  /* This will register "/dev/cap0" ... "/dev/cap<count-1>" */

  return cap_register_multiple("/dev/cap", lower, count);
}
#endif /* HAVE_CAPTURE */

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
  int ret;

  /* Initialize the board features common to all STM32 boards */

  ret = stm32_common_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_common_initialize() failed: %d\n",
             ret);
    }

#if !defined(CONFIG_ARCH_LEDS) && defined(CONFIG_USERLED_LOWER)
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS
#ifdef CONFIG_INPUT_BUTTONS_LOWER
  iinfo("Initializing button driver\n");

  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      ierr("ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#else
  /* Enable BUTTON support for some other purpose */

  board_button_initialize();
#endif
#endif /* CONFIG_INPUT_BUTTONS */

#ifdef CONFIG_ADC
  ret = stm32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_adc_setup failed: %d\n", ret);
    }
#endif /* CONFIG_ADC*/

#ifdef CONFIG_DAC
  ret = stm32_dac_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_dac_setup failed: %d\n", ret);
    }
#endif /* CONFIG_DAC */

#ifdef CONFIG_STM32_DTS
  /* devno == 0 creates /dev/sensor_temp0 */

  ret = stm32_dts_setup(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_adc_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_STM32_SPI
  /* Cannot call at board init because irq_attach would be called before
   * before irq_initialize is called.
   */

  stm32_spiinitialize();

#ifdef CONFIG_SPI_DRIVER
  stm32_spiregister();
#endif
#endif /* CONFIG_STM32_SPI */

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = stm32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_pwm_setup() failed: %d\n", ret);
    }
#endif

#ifdef HAVE_CAPTURE
  /* Initialize the capture drivers and register them as /dev/capN */

  ret = stm32_capture_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_capture_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST
  ret = stm32_usbhost_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize USB host: %d\n", ret);
      return ret;
    }
#endif

  UNUSED(ret);
  return OK;
}
