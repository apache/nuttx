/****************************************************************************
 * boards/arm/stm32/stm32f334-disco/src/stm32_appinit.c
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

#include <nuttx/board.h>
#include <nuttx/leds/userled.h>

#include "stm32f334-disco.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef HAVE_LEDS
#undef HAVE_DAC

#if !defined(CONFIG_ARCH_LEDS) && defined(CONFIG_USERLED_LOWER)
#  define HAVE_LEDS 1
#endif

#if defined(CONFIG_DAC)
#  define HAVE_DAC1 1
#  define HAVE_DAC2 1
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initialization logic and the
 *         matching application logic.  The value could be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
  int ret;

#if !defined(CONFIG_DRIVERS_POWERLED) && !defined(CONFIG_DRIVERS_SMPS)
#ifdef HAVE_LEDS
  /* Register the LED driver */

  ret = userled_lower_initialize(LED_DRIVER_PATH);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = stm32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_adc_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_DAC
  /* Initialize DAC and register the DAC driver. */

  ret = stm32_dac_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_dac_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_COMP
  /* Initialize COMP and register the COMP driver. */

  ret = stm32_comp_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_comp_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_OPAMP
  /* Initialize OPAMP and register the OPAMP driver. */

  ret = stm32_opamp_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_opamp_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_STM32_HRTIM
  /* Initialize HRTIM and register the HRTIM driver. */

  ret = stm32_hrtim_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_hrtim_setup failed: %d\n", ret);
    }
#endif
#endif

#ifdef CONFIG_DRIVERS_POWERLED
  /* Initialize powerled and register the powerled driver */

  ret = stm32_powerled_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_powerled_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_DRIVERS_SMPS
  /* Initialize smps and register the smps driver */

  ret = stm32_smps_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_smps_setup failed: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
