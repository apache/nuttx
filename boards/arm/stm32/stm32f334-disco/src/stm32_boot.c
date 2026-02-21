/****************************************************************************
 * boards/arm/stm32/stm32f334-disco/src/stm32_boot.c
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

#include <nuttx/board.h>
#include <nuttx/leds/userled.h>
#include <arch/board/board.h>

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
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after all memory
 *   has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void stm32_boardinitialize(void)
{
#ifdef CONFIG_ARCH_LEDS
  /* Configure on-board LEDs if LED support has been selected. */

  board_autoled_initialize();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called immediately after up_initialize() is called and just before
 *   the initial application is started.  This additional initialization
 *   phase may be used, for example, to initialize board-specific device
 *   drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  int ret;

#if !defined(CONFIG_DRIVERS_POWERLED) && !defined(CONFIG_DRIVERS_SMPS)
#ifdef HAVE_LEDS
  /* Register the LED driver */

  ret = userled_lower_initialize(LED_DRIVER_PATH);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
      return;
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
}
#endif
