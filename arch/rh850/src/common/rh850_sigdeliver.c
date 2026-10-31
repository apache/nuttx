/****************************************************************************
 * arch/rh850/src/common/rh850_sigdeliver.c
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

#include <assert.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/queue.h>
#include <nuttx/sched.h>

#include "sched/sched.h"
#include "rh850_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rh850_sigdeliver
 *
 * Description:
 *   Run the pending signal actions of the current task, then restore the
 *   frame saved by up_schedule_sigaction().
 *
 ****************************************************************************/

void rh850_sigdeliver(void)
{
  struct tcb_s *rtcb = this_task();
  uintptr_t *regs = rtcb->xcp.saved_regs;

  board_autoled_on(LED_SIGNAL);

  sinfo("rtcb=%p sigdeliver=%p sigpendactionq.head=%p\n",
        rtcb, rtcb->sigdeliver, rtcb->sigpendactionq.head);
  DEBUGASSERT(rtcb->sigdeliver != NULL);

retry:
#ifndef CONFIG_SUPPRESS_INTERRUPTS
  up_irq_enable();
#endif

  (rtcb->sigdeliver)(rtcb);

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  up_irq_save();
#endif

  if (!sq_empty(&rtcb->sigpendactionq) &&
      (rtcb->flags & TCB_FLAG_SIGNAL_ACTION) == 0)
    {
      goto retry;
    }

  rtcb->sigdeliver = NULL;
  board_autoled_off(LED_SIGNAL);

  rtcb->xcp.regs = regs;
  rh850_fullcontextrestore();
}
