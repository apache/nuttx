/****************************************************************************
 * boards/arm/rtl8721f/rtl8721f_evb/src/rtl8721f_timer.c
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

#include <syslog.h>
#include <errno.h>

#include "ameba_timer.h"
#include "rtl8721f_rtl8721f_evb.h"

#ifdef CONFIG_AMEBA_TIMER

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtl8721f_timer_initialize
 *
 * Description:
 *   Register the board's general-purpose timers.  Both timers are internal
 *   (no board wiring), so this just binds the two exposed instances to their
 *   device nodes: /dev/timer0 is TIM1 and /dev/timer1 is TIM2, both 32-bit
 *   basic timers at 32.768 kHz.  TIM0 is reserved by the ROM as the system
 *   timer and is not exposed.  A failure on either is logged but does not
 *   abort the other.
 *
 ****************************************************************************/

int rtl8721f_timer_initialize(void)
{
  int ret;

  ret = ameba_timer_initialize("/dev/timer0", 0);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: ameba_timer_initialize(/dev/timer0) failed: %d\n", ret);
    }

  ret = ameba_timer_initialize("/dev/timer1", 1);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: ameba_timer_initialize(/dev/timer1) failed: %d\n", ret);
    }

  return ret;
}

#endif /* CONFIG_AMEBA_TIMER */
