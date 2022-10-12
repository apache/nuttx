/****************************************************************************
 * sched/sched/sched_sysinfo.c
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

#include <sys/sysinfo.h>
#include <sys/types.h>
#include <errno.h>
#include <string.h>

#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/pgalloc.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sysinfo
 *
 * Description:
 *   sysinfo() returns certain statistics on memory and swap usage,
 *   as well as the load average.
 *
 ****************************************************************************/

int sysinfo(FAR struct sysinfo *info)
{
#ifdef CONFIG_SCHED_CPULOAD
  struct cpuload_s cpuload;
#endif
#ifdef CONFIG_MM_PGALLOC
  struct pginfo_s pginfo;
#endif
  struct mallinfo minfo;

  if (info == NULL)
    {
      set_errno(EINVAL);
      return -1;
    }

  memset(info, 0, sizeof(*info));

#ifdef CONFIG_SCHED_CPULOAD
  clock_cpuload(0, &cpuload);

  /* On the simulator, you may hit cpuload.total == 0, but probably never
   * on real hardware.
   */

  if (cpuload.total)
    {
      info->loads[0] = ((cpuload.total - cpuload.active) <<
                         SI_LOAD_SHIFT) / cpuload.total;
      info->loads[1] = info->loads[0];
      info->loads[2] = info->loads[0];
    }
#endif

#ifdef MM_KERNEL_USRHEAP_INIT
  minfo = kumm_mallinfo();

  info->totalram += minfo.arena;
  info->freeram  += minfo.fordblks;
#endif

#ifdef CONFIG_MM_KERNEL_HEAP
  minfo = kmm_mallinfo();

  info->totalram += minfo.arena;
  info->freeram  += minfo.fordblks;
#endif

#ifdef CONFIG_MM_PGALLOC
  mm_pginfo(&pginfo);

  info->totalram += pginfo.ntotal << MM_PGSHIFT;
  info->freeram  += pginfo.nfree  << MM_PGSHIFT;
#endif

  info->uptime   = TICK2SEC(clock_systime_ticks());
  info->procs    = CONFIG_SMP_NCPUS;
  info->mem_unit = 1;

  return 0;
}
