/****************************************************************************
 * arch/arm64/src/qemu/qemu_rtc.c
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

#include <nuttx/arch.h>
#include <nuttx/timers/pl031.h>
#include <nuttx/timers/rtc.h>
#include <nuttx/timers/arch_rtc.h>

#include "arm64_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Register the PL031 RTC of the QEMU virt machine.  QEMU sets it from the
 *   host clock (-rtc base=utc by default), so the system time starts at the
 *   host's time.
 *
 ****************************************************************************/

int up_rtc_initialize(void)
{
  FAR struct rtc_lowerhalf_s *lower;

#ifdef CONFIG_QEMU_RTC_PL031_SYNC
  /* The PL031 counts whole seconds, so the system time would start up to a
   * second late.  Wait until the second changes: the time read next is
   * then correct to a few milliseconds.  Register 0 is the data register.
   */

  uint32_t second = getreg32(CONFIG_QEMU_RTC_PL031_BASE);

  while (getreg32(CONFIG_QEMU_RTC_PL031_BASE) == second);
#endif

  lower = pl031_initialize(CONFIG_QEMU_RTC_PL031_BASE,
                           CONFIG_QEMU_RTC_PL031_IRQ);
  up_rtc_set_lowerhalf(lower, true);
  return rtc_initialize(0, lower);
}
