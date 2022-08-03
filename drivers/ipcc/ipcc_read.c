/****************************************************************************
 * drivers/ipcc/ipcc_read.c
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
#include <nuttx/ipcc.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <sys/types.h>

#include "ipcc_priv.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipcc_rxfree_notify
 *
 * Description:
 *   Notifies all blocked threads or those waiting in poll/select that
 *   there is data on buffer to perform reading.
 *
 * Input Parameters:
 *   ipcc - pointer to driver instance
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   This function can be called from interrupt handler from lower half.
 *
 ****************************************************************************/

void ipcc_rxfree_notify(FAR struct ipcc_driver_s *priv)
{
  int semval;

  if (priv == NULL)
    {
      /* priv can be NULL when ipcc lower half is initialized but
       * upper half has not yet been initialized, and rx interrupt
       * has been received. In such case we don't wake any reader,
       * because since ipcc is not yet initialized there cannot be
       * any readers yet. We can safely return here, first read()
       * to this ipcc channel will immediately read data.
       */

      return;
    }

  if ((nxsem_get_value(&priv->rxsem, &semval) == 0) && semval > 0)
    {
      /* Notify all poll/select waiters that they can read from the driver.
       * Do it only when there are no already blocked readers to avoid
       * situation where thread that is polling gets notified only to
       * be blocked in read() because another thread have read from
       * buffer before polling thread could.
       */

      ipcc_pollnotify(priv, POLLIN);
      return;
    }

  /* Notify all blocked readers that data is available to read */

  do
    {
      nxsem_post(&priv->rxsem);
    }
  while (nxsem_get_value(&priv->rxsem, &semval) == 0 && semval <= 0);
}

/****************************************************************************
 * Name: ipcc_read
 *
 * Description:
 *   Reads data from the IPCC lower driver. Will block if no data is
 *   available, unless O_NONBLOCK flag is set.
 *
 * Input Parameters:
 *   filep - file on vfs associated with the driver
 *   buffer - buffer where read data should be copied to
 *   buflen - size of the buffer... buffer
 *
 * Returned Value:
 *   Will return number of bytes read or negated errno
 *
 * Assumptions/Limitations:
 *
 ****************************************************************************/

ssize_t ipcc_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  FAR struct ipcc_driver_s *priv;
  ssize_t nread;
  int ret;
  int flags;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  priv = filep->f_inode->i_private;

  /* Get exclusive access to driver */

  if ((ret = nxsem_wait(&priv->exclsem)))
    {
      /* nxsem_wait() will return on signal, we did not start
       * any transfer yet, so we can safely return with error
       */

      return ret;
    }

  for (; ; )
    {
      /* Disable interrupts, or we might get in situation when we:
       * - read 0 bytes from circbuf
       * - interrupt comes in
       *   - it copies data to buffer and notifies blocked readers
       *     (in this case sem count is 0, so no sem_post() is called)
       * - interrupt ends
       * - since we are in blocking mode, and we read 0 from buffer
       *   we call sem_wait() and we hang in there cause rx interrupt
       *   will not be triggered again.
       */

      flags = enter_critical_section();
#ifdef CONFIG_IPCC_BUFFERED
      /* Data is buffered in interrupt handler, so we simply
       * have to return buffers content to the user
       */

      if (circbuf_used(&priv->ipcc->rxbuf))
        {
          /* There is some data on buffer, we are sure we won't block
           * so immediately leave critical section to not block other
           * more important drivers
           */

          leave_critical_section(flags);
        }

      if ((nread = circbuf_read(&priv->ipcc->rxbuf, buffer, buflen)) > 0)
        {
          /* got some data */

          if (priv->ipcc->overflow)
            {
              /* We tried to buffer data in previous interrupt, but
               * it failed due to rxbuf being full. Now that we took
               * some data from buffer, we can try to buffer data again
               */

              priv->ipcc->ops.buffer_data(priv->ipcc, &priv->ipcc->rxbuf);

              if ((size_t)nread < buflen)
                {
                  /* There is still some space left on buffer, and
                   * we just added new data to buffer, get more data
                   * for the user
                   */

                  nread += circbuf_read(&priv->ipcc->rxbuf, buffer + nread,
                                        buflen - nread);
                }
            }

          /* return number of bytes read to the caller */

          nxsem_post(&priv->exclsem);
          return nread;
        }
#else /* CONFIG_IPCC_BUFFERED */

      /* Unbuffered read, read data directly from lower driver */

      if ((nread = priv->ipcc->ops.read(priv->ipcc, buffer, buflen)) != 0)
        {
          /* Got some data, return number of bytes read to the caller
           *   --or--
           * read() returned error in which case return errno value
           */

          leave_critical_section(flags);
          nxsem_post(&priv->exclsem);
          return nread;
        }
#endif /* CONFIG_IPCC_BUFFERED */

      /* no data on the buffer, should we block? */

      if (filep->f_oflags & O_NONBLOCK)
        {
          /* No, we should not block, so inform caller that
           * no data could be read
           */

          leave_critical_section(flags);
          nxsem_post(&priv->exclsem);
          return -EAGAIN;
        }

      /* We are in blocking mode, so we have to wait for data to arrive.
       * nxsem_wait() will atomically leave critical section for us so
       * we don't have to do it.
       */

      nxsem_post(&priv->exclsem);
      if ((ret = nxsem_wait(&priv->rxsem)))
        {
          /* We were interrupted by signal, but we have not written
           * any data to caller's buffer, so we return with error
           */

          return ret;
        }

      /* Data should now be available, but it's possible that
       * another thread will read all data from buffer before
       * we can do it, so we will stay in the loop until we
       * manage to read something - or interrupt signal occurs
       *
       * We have released exclusive lock to driver when we were
       * waiting for data, so now let's retake it.
       */

      nxsem_wait(&priv->exclsem);
      continue;
    }
}
