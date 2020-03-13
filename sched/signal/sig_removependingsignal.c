/****************************************************************************
 * sched/signal/sig_removependingsignal.c
 *
 *   Copyright (C) 2007, 2009, 2013-2014, 2016-2017 Gregory Nutt. All
 *     rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <assert.h>
#include <debug.h>
#include <sched.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/kmalloc.h>

#include "signal/signal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_remove_pendingsignal
 *
 * Description:
 *   Remove the specified signal from the signal pending list
 *
 ****************************************************************************/

FAR sigpendq_t *nxsig_remove_pendingsignal(FAR struct tcb_s *stcb, int signo)
{
  FAR struct task_group_s *group = stcb->group;
  FAR sigpendq_t *currsig;
  FAR sigpendq_t *prevsig;
  irqstate_t  flags;

  DEBUGASSERT(group);

  flags = enter_critical_section();

  for (prevsig = NULL,
       currsig = (FAR sigpendq_t *)group->tg_sigpendingq.head;
       (currsig && currsig->info.si_signo != signo);
       prevsig = currsig, currsig = currsig->flink);

  if (currsig)
    {
      if (prevsig)
        {
          sq_remafter((FAR sq_entry_t *)prevsig, &group->tg_sigpendingq);
        }
      else
        {
          sq_remfirst(&group->tg_sigpendingq);
        }
    }

  leave_critical_section(flags);

  return currsig;
}
