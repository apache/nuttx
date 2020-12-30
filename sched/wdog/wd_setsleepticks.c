/****************************************************************************
 * sched/wdog/wd_setsleepticks.c
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

#include <nuttx/wdog.h>
#include <nuttx/irq.h>

#include "wdog/wdog.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wd_setsleepticks
 *
 * Description:
 *   This function will update watchdogs' lag.
 *
 * Input Parameters:
 *   ticks - sleep ticks
 *
 * Returned Value:
 *   Zero means success.
 *
 ****************************************************************************/

int wd_setsleepticks(clock_t ticks)
{
  irqstate_t flags;
  FAR struct wdog_s * curr;

  flags = enter_critical_section();

  for (curr = (FAR struct wdog_s *)(g_wdactivelist.head);
       curr; curr = curr->next)
    {
      curr->lag -= ticks;
    }

  leave_critical_section(flags);
  return 0;
}
