/****************************************************************************
 * boards/arm/cxd56xx/spresense/src/cxd56_clock.c
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

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <arch/board/board.h>
#include <arch/chip/pm.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct pm_cpu_freqlock_s g_hv_lock =
  PM_CPUFREQLOCK_INIT(PM_CPUFREQLOCK_TAG('C','P',0), PM_CPUFREQLOCK_FLAG_HV);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_clock_initialize
 ****************************************************************************/

void board_clock_initialize(void)
{
  up_pm_acquire_freqlock(&g_hv_lock);
}

/****************************************************************************
 * Name: board_clock_enable
 ****************************************************************************/

void board_clock_enable(void)
{
  up_pm_release_freqlock(&g_hv_lock);
}
