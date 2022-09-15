/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_capture.c
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
#include <debug.h>
#include <nuttx/timers/capture.h>
#include <arch/board/board.h>

#include "chip.h"

#include "stm32.h"
#include "stm32_capture.h"
#include "arm_internal.h"

#include "stm32f4discovery.h"

#if defined(CONFIG_CAPTURE)
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Capture
 *
 * The stm32f4discovery has no real on-board pwm capture devices, but the
 * board can be configured to capture pwm using TIM3 CH2 PB5.
 */

#define HAVE_CAPTURE 1

#ifndef CONFIG_CAPTURE
#  undef HAVE_CAPTURE
#endif

#ifndef CONFIG_STM32_TIM3
#  undef HAVE_CAPTURE
#endif

#ifndef CONFIG_STM32_TIM3_CAP
#  undef HAVE_CAPTURE
#endif

#if !defined(CONFIG_STM32_TIM3_CHANNEL) || CONFIG_STM32_TIM3_CHANNEL != STM32F4DISCOVERY_CAPTURECHANNEL
#  undef HAVE_CAPTURE
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_capture_setup
 *
 * Description:
 *   Initialize and register the pwm capture driver.
 *
 * Input parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/capture0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_capture_setup(const char *devpath)
{
#ifdef HAVE_CAPTURE
  struct cap_lowerhalf_s *capture;
  int ret;

  capture = stm32_cap_initialize(STM32F4DISCOVERY_CAPTURETIMER);

  /* Then register the pwm capture sensor */

  ret = cap_register(devpath, capture);
  if (ret < 0)
    {
      mtrerr("ERROR: Error registering capture\n");
    }

  return ret;
#else
  return -ENODEV;
#endif
}

#endif /* CONFIG_CAPTURE */
