/****************************************************************************
 * sched/wdog/wd_initialize.c
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

#include <nuttx/list.h>

#include "wdog/wdog.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The g_wdactivelist data structure is a singly linked list ordered by
 * watchdog expiration time. When watchdog timers expire,the functions on
 * this linked list are removed and the function is called.
 */

#undef g_wdactivelist
DEFINE_PER_CPU_BSS_BMP(struct list_node, g_wdactivelist);
#define g_wdactivelist this_cpu_var_bmp(g_wdactivelist)

#ifdef CONFIG_HRTIMER
#  undef g_wdtimer
DEFINE_PER_CPU_BSS_BMP(struct hrtimer_s, g_wdtimer);
#  define g_wdtimer this_cpu_var_bmp(g_wdtimer)
#endif

#  undef g_wdtimernested
DEFINE_PER_CPU_BSS_BMP(bool, g_wdtimernested);
#  define g_wdtimernested this_cpu_var_bmp(g_wdtimernested)

#ifdef CONFIG_HRTIMER

#endif

#if defined(CONFIG_SCHED_TICKLESS) || defined(CONFIG_HRTIMER)

#undef g_wdexpired
DEFINE_PER_CPU_BSS_BMP(clock_t, g_wdexpired);
#  define g_wdexpired this_cpu_var_bmp(g_wdexpired)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wdlist_initialize
 *
 * Description:
 *   This function do list initialize, list should prefer runtime init.
 *
 ****************************************************************************/

void wdlist_initialize(void)
{
  list_initialize(&g_wdactivelist);
}
