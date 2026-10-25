/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_pm.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_PM_H
#define __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_PM_H

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
 * Name: gd32vw55x_pmstop
 *
 * Description:
 *   Enter the deep-sleep mode until the next timer event: the core and bus
 *   clocks stop, SRAM and registers keep their contents.  The RTC wake-up
 *   timer ends it at the next system timer alarm, at most
 *   CONFIG_GD32VW55X_PM_WAKEUP_MS milliseconds later, and so does any EXTI
 *   interrupt.  On return the clock tree is set up again and the system
 *   timer is advanced by the time spent asleep.  Without the RTC, or when
 *   the next alarm is too close, the core only waits for an interrupt.
 *
 *   Called with the interrupts disabled.
 *
 ****************************************************************************/

void gd32vw55x_pmstop(void);

#endif /* CONFIG_PM */

#ifdef CONFIG_RTC

/****************************************************************************
 * Name: gd32vw55x_rtc_setwakeup
 *
 * Description:
 *   Start the RTC wake-up timer to raise its interrupt (EXTI line 21) in
 *   'usec' microseconds, at most about 30 s, or stop it if 'usec' is 0.
 *
 ****************************************************************************/

int gd32vw55x_rtc_setwakeup(uint32_t usec);

/****************************************************************************
 * Name: gd32vw55x_rtc_timeofday
 *
 * Description:
 *   Wait for the start of the next sub-second unit of the calendar, and
 *   return the time of day in these units, and their frequency in 'hz'.
 *   The time is right after deep-sleep, without waiting for the shadow
 *   registers.
 *
 ****************************************************************************/

uint32_t gd32vw55x_rtc_timeofday(uint32_t *hz);

#endif /* CONFIG_RTC */

#endif /* __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_PM_H */
