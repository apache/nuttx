/****************************************************************************
 * arch/arm/src/gd32w51x/gd32w51x_pm.h
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

#ifndef __ARCH_ARM_SRC_GD32W51X_GD32W51X_PM_H
#define __ARCH_ARM_SRC_GD32W51X_GD32W51X_PM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_PM

/****************************************************************************
 * Name: gd32w51x_pmstop
 *
 * Description:
 *   Enter the deep-sleep mode: the core and bus clocks stop, SRAM and
 *   registers keep their contents.  Any EXTI interrupt wakes the chip up,
 *   and so does the RTC wake-up timer after
 *   CONFIG_GD32W51X_PM_WAKEUP_MS milliseconds (when it is not zero).  On
 *   return the clock tree is set up again and the system time is
 *   corrected with the time spent asleep.
 *
 *   Called with the interrupts disabled.
 *
 ****************************************************************************/

void gd32w51x_pmstop(void);

#endif /* CONFIG_PM */

#ifdef CONFIG_RTC

/****************************************************************************
 * Name: gd32w51x_rtc_setwakeup
 *
 * Description:
 *   Start the RTC wake-up timer to raise its interrupt (EXTI line 24) in
 *   'msec' milliseconds, at most about 32 s, or stop it if 'msec' is 0.
 *
 ****************************************************************************/

int gd32w51x_rtc_setwakeup(uint32_t msec);

/****************************************************************************
 * Name: gd32w51x_rtc_resync
 *
 * Description:
 *   Wait for the calendar shadow registers to be updated, which they are
 *   not in deep-sleep.
 *
 ****************************************************************************/

int gd32w51x_rtc_resync(void);

#endif /* CONFIG_RTC */

#endif /* __ARCH_ARM_SRC_GD32W51X_GD32W51X_PM_H */
