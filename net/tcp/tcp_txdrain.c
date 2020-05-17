/****************************************************************************
 * net/tcp/tcp_txdrain.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/semaphore.h>

#include "tcp/tcp.h"

#if defined(CONFIG_NET_TCP_WRITE_BUFFERS) && defined(CONFIG_NET_TCP_NOTIFIER)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: txdrain_worker
 *
 * Description:
 *   Called with the write buffers have all been sent.
 *
 * Input Parameters:
 *   arg     - The notifier entry.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void txdrain_worker(FAR void *arg)
{
  FAR sem_t *waitsem = (FAR sem_t *)arg;

  DEBUGASSERT(waitsem != NULL);

  /* Then just post the semaphore, waking up tcp_txdrain() */

  nxsem_post(waitsem);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_txdrain
 *
 * Description:
 *   Wait for all write buffers to be sent (or for a timeout to occur).
 *
 * Input Parameters:
 *   psock   - An instance of the internal socket structure.
 *   timeout - The relative time when the timeout will occur
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

int tcp_txdrain(FAR struct socket *psock, unsigned int timeout)
{
  FAR struct tcp_conn_s *conn;
  sem_t waitsem;
  int ret;

  DEBUGASSERT(psock != NULL && psock->s_crefs > 0 && psock->s_conn != NULL);
  DEBUGASSERT(psock->s_type == SOCK_STREAM);

  conn = (FAR struct tcp_conn_s *)psock->s_conn;

  /* Initialize the wait semaphore */

  nxsem_init(&waitsem, 0, 0);
  nxsem_set_protocol(&waitsem, SEM_PRIO_NONE);

  /* The following needs to be done with the network stable */

  net_lock();

  /* Get a notification when the write buffers are drained */

  ret = tcp_writebuffer_notifier_setup(txdrain_worker, conn, &waitsem);

  /* The special return value of 0 means that there is no Tx data to be
   * drained.  Otherwise it is a special 'key' that can be used to teardown
   * the notification later
   */

  if (ret > 0)
    {
      /* Save the drain key */

      int drain_key = ret;

      /* Also get a notification if we lose the connection.
       * REVISIT:  This really should not be necessary.  tcp_send() should
       * detect the disconnection event and signal the txdrain but I do not
       * see that TCP disconnection event.  This assures that the txdrain
       * operation will not wait for the full timeout in that case.
       */

      ret = tcp_disconnect_notifier_setup(txdrain_worker, conn, &waitsem);

      /* Zero is a special value that means that no connection has been
       * established.  Otherwise it is a special 'key' that can be used
       * to teardown the notification later
       */

      if (ret > 0)
        {
          /* Save the disconnect key */

          int disconn_key = ret;

          /* There is pending write data and the socket is connected..
           * wait for it to drain or be be disconnected.
           */

          ret = net_timedwait_uninterruptible(&waitsem, timeout);

          /* Tear down the disconnect notifier */

          tcp_notifier_teardown(disconn_key);
        }

      /* Tear down the disconnect notifier */

      tcp_notifier_teardown(drain_key);
    }

  net_unlock();
  nxsem_destroy(&waitsem);
  return ret;
}

#endif /* CONFIG_NET_TCP_WRITE_BUFFERS && CONFIG_NET_TCP_NOTIFIER */
