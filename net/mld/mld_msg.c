/****************************************************************************
 * net/mld/mld_msg.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of CITEL Technologies Ltd nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY CITEL TECHNOLOGIES AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL CITEL TECHNOLOGIES OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/mld.h>

#include "devif/devif.h"
#include "mld/mld.h"

#ifdef CONFIG_NET_MLD

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mld_schedmsg
 *
 * Description:
 *   Schedule a message to be send at the next driver polling interval.
 *
 * Assumptions:
 *   This function may be called in most any context.
 *
 ****************************************************************************/

void mld_schedmsg(FAR struct mld_group_s *group, uint8_t msgtype)
{
  /* The following should be atomic */

  net_lock();
  DEBUGASSERT(!IS_MLD_SCHEDMSG(group->flags));
  group->msgtype = msgtype;
  SET_MLD_SCHEDMSG(group->flags);
  net_unlock();
}

/****************************************************************************
 * Name: mld_waitmsg
 *
 * Description:
 *   Schedule a message to be send at the next driver polling interval and
 *   block, waiting for the message to be sent.
 *
 ****************************************************************************/

void mld_waitmsg(FAR struct mld_group_s *group, uint8_t msgtype)
{
  int ret;

  /* Schedule to send the message */

  net_lock();
  DEBUGASSERT(!IS_MLD_WAITMSG(group->flags));
  SET_MLD_WAITMSG(group->flags);
  mld_schedmsg(group, msgtype);

  /* Then wait for the message to be sent */

  while (IS_MLD_SCHEDMSG(group->flags))
    {
      /* Wait for the semaphore to be posted */

      while ((ret = net_lockedwait(&group->sem)) < 0)
        {
          /* The only error that should occur from net_lockedwait() is if
           * the wait is awakened by a signal.
           */

          DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
        }

      UNUSED(ret);
    }

  /* The message has been sent and we are no longer waiting */

  CLR_MLD_WAITMSG(group->flags);
  net_unlock();
}

#endif /* CONFIG_NET_MLD */
