/****************************************************************************
 * libs/libc/stdlib/lib_getloadavg.c
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

#include <stdlib.h>
#include <sys/sysinfo.h>

#ifdef CONFIG_HAVE_DOUBLE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getloadavg
 *
 * Description:
 *   Get the system load averaged over the last 1, 5 and 15 minutes (the
 *   BSD and Linux function).  NuttX has one CPU load, the share of time
 *   that the CPU did not run the idle task (clock_cpuload(), through
 *   sysinfo()), so the three values are the same: from 0.0 (idle) to 1.0
 *   (always busy).
 *
 * Input Parameters:
 *   loadavg - The array for the averages
 *   nelem   - The number of averages to get (at most 3)
 *
 * Returned Value:
 *   The number of averages in loadavg, or -1 if the load is not available
 *   (CONFIG_SCHED_CPULOAD_NONE).  Programs that can use the load average
 *   take -1 as "unknown".
 *
 ****************************************************************************/

int getloadavg(FAR double loadavg[], int nelem)
{
#ifndef CONFIG_SCHED_CPULOAD_NONE
  struct sysinfo info;
  int i;

  if (nelem < 0 || sysinfo(&info) < 0)
    {
      return -1;
    }

  if (nelem > 3)
    {
      nelem = 3;
    }

  for (i = 0; i < nelem; i++)
    {
      loadavg[i] = (double)info.loads[i] / (1 << SI_LOAD_SHIFT);
    }

  return nelem;
#else
  UNUSED(loadavg);
  UNUSED(nelem);
  return -1;
#endif
}

#endif /* CONFIG_HAVE_DOUBLE */
