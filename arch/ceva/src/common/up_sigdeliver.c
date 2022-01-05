/****************************************************************************
 * arch/ceva/src/common/up_sigdeliver.c
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

#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "sched/sched.h"
#include "up_internal.h"

#ifndef CONFIG_DISABLE_SIGNALS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_sigdeliver
 *
 * Description:
 *   This is the a signal handling trampoline.  When a signal action was
 *   posted.  The task context was mucked with and forced to branch to this
 *   location with interrupts unchanged.
 *
 ****************************************************************************/

void up_sigdeliver(void)
{
  struct tcb_s  *rtcb = this_task();
  uint32_t *regs = rtcb->xcp.saved_regs;
  sig_deliver_t sigdeliver;

  /* Save the errno.  This must be preserved throughout the signal handling
   * so that the user code final gets the correct errno value (probably
   * EINTR).
   */

  int saved_errno = rtcb->pterrno;

  sinfo("rtcb=%p sigdeliver=%p sigpendactionq.head=%p\n",
        rtcb, rtcb->xcp.sigdeliver, rtcb->sigpendactionq.head);
  DEBUGASSERT(rtcb->xcp.sigdeliver != NULL);

  /* Get a local copy of the sigdeliver function pointer. We do this so that
   * we can nullify the sigdeliver function pointer in the TCB and accept
   * more signal deliveries while processing the current pending signals.
   */

  sigdeliver           = (sig_deliver_t)rtcb->xcp.sigdeliver;
  rtcb->xcp.sigdeliver = NULL;

  /* Deliver the signal */

  sigdeliver(rtcb);

  /* Output any debug messages BEFORE restoring errno (because they may
   * alter errno), then disable interrupts again and restore the original
   * errno that is needed by the user logic (it is probably EINTR).
   */

  sinfo("Resuming\n");

  rtcb->pterrno = saved_errno;

  /* Then restore the correct state for this thread of
   * execution.
   */

  up_fullcontextrestore(regs);
}

#endif /* !CONFIG_DISABLE_SIGNALS */
