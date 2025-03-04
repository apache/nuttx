/****************************************************************************
 * sched/sched/sched_profil.c
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

#include <errno.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/spinlock.h>
#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PROFTICK NSEC2TICK(NSEC_PER_SEC / CONFIG_SCHED_PROFILE_TICKSPERSEC)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct profinfo_s
{
  FAR unsigned short *counter; /* Histogram PC sample array */
  uintptr_t lowpc;             /* Range to be profiled */
  uintptr_t highpc;            /* Range to be profiled */
  unsigned int scale;          /* Scale value of bins */
  struct wdog_s timer;         /* Timer for profiling */
  spinlock_t lock;             /* Lock for this structure */
};

#ifdef CONFIG_SMP
static int profil_timer_handler_cpu(FAR void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct profinfo_s g_prof;

#ifdef CONFIG_SMP
static struct smp_call_data_s g_call_data =
SMP_CALL_INITIALIZER(profil_timer_handler_cpu, &g_prof);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int profil_timer_handler_cpu(FAR void *arg)
{
  FAR struct profinfo_s *prof = (FAR struct profinfo_s *)arg;
  uintptr_t pc = up_getusrpc(NULL);
  irqstate_t flags;

  flags = spin_lock_irqsave(&prof->lock);
  if (pc >= prof->lowpc && pc < prof->highpc)
    {
      size_t idx = (pc - prof->lowpc) / 2;
#if UINTMAX_MAX > SIZE_MAX
      idx = (uintmax_t)idx * prof->scale / 65536;
#else
      idx = idx / 65536 * prof->scale + idx % 65536 * prof->scale / 65536;
#endif
      prof->counter[idx]++;
    }

  spin_unlock_irqrestore(&prof->lock, flags);
  return OK;
}

static void profil_timer_handler(wdparm_t arg)
{
  FAR struct profinfo_s *prof = (FAR struct profinfo_s *)(uintptr_t)arg;

#ifdef CONFIG_SMP
  cpu_set_t cpus = (1 << CONFIG_SMP_NCPUS) - 1;
  CPU_CLR(this_cpu(), &cpus);
  nxsched_smp_call_async(cpus, &g_call_data);
#endif

  profil_timer_handler_cpu(prof);
  wd_start(&prof->timer, PROFTICK, profil_timer_handler, arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: profil
 *
 * Description:
 *   This routine provides a means to find out in what areas your
 *   program spends most of its time.  The argument buf points to
 *   bufsiz bytes of core.  the user's program counter (PC) is
 *   examined SCHED_PROFILE_TICKSPERSEC times in every second:
 *   offset is subtracted and the result is multiplied by scale
 *   and divided by 65536.  If the resulting value is less than
 *   bufsiz, then the corresponding entry in buf is incremented.
 *   If buf is NULL, profiling is disabled.
 *
 * Input Parameters:
 *   buf    - Buffer to record the hitting count
 *   bufsiz - Size of buffer in bytes
 *   offset - The lowest address to be sampled
 *   scale  - Multiply address by scale / 65536
 *
 * Returned Value:
 *   Zero (OK) if successful. Otherwise, ERROR is returned and
 *   errno is set to indicate the error.
 *
 ****************************************************************************/

int profil(FAR unsigned short *buf, size_t bufsiz,
           size_t offset, unsigned int scale)
{
  FAR struct profinfo_s *prof = &g_prof;
  irqstate_t flags;
  uintptr_t highpc;

  if (scale > 65536)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  if (buf == NULL || scale == 0)
    {
      wd_cancel(&prof->timer);
      return OK;
    }

  memset(buf, 0, bufsiz);
  highpc = (uintmax_t)bufsiz * 65536 / scale;

  flags = spin_lock_irqsave(&prof->lock);
  prof->counter = buf;
  prof->lowpc   = offset;
  prof->highpc  = offset + highpc;
  prof->scale   = scale;
  spin_unlock_irqrestore(&prof->lock, flags);

  wd_start(&prof->timer, PROFTICK, profil_timer_handler,
           (wdparm_t)(uintptr_t)prof);
  return OK;
}
