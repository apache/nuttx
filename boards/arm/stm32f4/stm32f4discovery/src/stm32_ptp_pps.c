/****************************************************************************
 * boards/arm/stm32f4/stm32f4discovery/src/stm32_ptp_pps.c
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

#include <nuttx/debug.h>
#include <nuttx/timers/capture.h>
#include <nuttx/timers/ptp_pps.h>
#include <arch/board/board.h>

#include "stm32_capture.h"
#include "stm32f4discovery.h"

#ifdef CONFIG_PTP_PPS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32_TIM5_CAP
#  error "The PPS clock needs CONFIG_STM32_TIM5_CAP"
#endif

#if CONFIG_STM32_TIM5_CHANNEL != STM32F4DISCOVERY_PPSCHANNEL
#  error "The PPS is on TIM5 channel 2, set CONFIG_STM32_TIM5_CHANNEL=2"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ptp_pps_setup
 *
 * Description:
 *   Register a PPS disciplined PTP clock on the TIM5 capture.  The free
 *   running TIM5 counter is the counter of the clock, the PPS on PA1
 *   (TIM5 CH2) is its external time stamp input 0.
 *
 * Input parameters:
 *   devno - The device number, the clock is registered as /dev/ptp<devno>
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_ptp_pps_setup(int devno)
{
  struct cap_lowerhalf_s *capture;

  capture = stm32_cap_initialize(STM32F4DISCOVERY_PPSTIMER);
  if (capture == NULL)
    {
      return -ENODEV;
    }

  return ptp_pps_register(capture, CONFIG_STM32_TIM5_CLOCK, devno);
}

#endif /* CONFIG_PTP_PPS */
