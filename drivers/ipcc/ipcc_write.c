/****************************************************************************
 * drivers/ipcc/ipcc_write.c
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
 * Name: ipcc_txfree_notify
 *
 * Description:
 *   Notifies all blocked threads or those waiting in poll/select that
 *   there is place on buffer to perform writing.
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

void ipcc_txfree_notify(FAR struct ipcc_driver_s *priv)
{
  int semval;

  if (priv == NULL)
    {
      /* priv can be NULL when ipcc lower half is initialized but
       * upper half has not yet been initialized, and tx interrupt
       * has been received. In such case we don't wake any writers,
       * because since ipcc is not yet initialized there cannot be
       * any writers yet. We can safely return here, first write()
       * to this ipcc channel will immediately write data.
       */

      return;
    }

  if ((nxsem_get_value(&priv->txsem, &semval) == 0) && semval > 0)
    {
      /* Notify all poll/select waiters that they can write to the driver.
       * Do it only when there are no already blocked writers to avoid
       * situation where thread that is polling gets notified only to
       * be blocked in write() because another thread have written to
       * buffer before polling thread could.
       */

      ipcc_pollnotify(priv, POLLOUT);
      return;
    }

  /* Notify all blocked writers that data is available to write */

  do
    {
      nxsem_post(&priv->txsem);
    }
  while (nxsem_get_value(&priv->txsem, &semval) == 0 && semval <= 0);
}

/****************************************************************************
 * Name: ipcc_write
 *
 * Description:
 *   Writes data to IPCC memory so that another CPU can read the contents.
 *   Will block untill whole buffer is copied unless signal is received
 *   or O_NONBLOCK flag is set.
 *
 * Input Parameters:
 *   filep - file on vfs associated with the driver
 *   buffer - buffer to copy to IPCC memory
 *   buflen - size of the buffer... buffer
 *
 * Returned Value:
 *   Number of successfully written bytes into the IPCC memory or netagted
 *   errno when no data could be written.
 *
 * Assumptions/Limitations:
 *
 ****************************************************************************/

ssize_t ipcc_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen)
{
  FAR struct ipcc_driver_s *priv;
  ssize_t nwritten;
  int ret;
  int flags;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  priv = filep->f_inode->i_private;

  /* Get exclusive access to driver */

  if ((ret = nxmutex_lock(&priv->lock)))
    {
      /* nxsem_wait() will return on signal, we did not start
       * any transfer yet, so we can safely return with error
       */

      return ret;
    }

  flags = enter_critical_section();

  for (nwritten = 0; ; )
    {
#ifdef CONFIG_IPCC_BUFFERED
      /* Buffered write, if buffer is empty try to write directly to
       * IPCC memory, else buffer data in circbuf - it will be written
       * in interrupt handler when IPCC tx channel is free.
       */

      if (circbuf_used(&priv->ipcc->txbuf) == 0)
        {
          /* Write buffer is empty, we can try and write data directly
           * to IPCC memory thus omitting copying to buffer.
           */

          nwritten += priv->ipcc->ops.write(priv->ipcc, buffer + nwritten,
                                           buflen - nwritten);
          if (nwritten == (ssize_t)buflen || nwritten < 0)
            {
              /* We've managed to write whole buffer to IPCC memory,
               * there is nothing else for use to do
               *   --or--
               * lower half driver returned error during write,
               *
               * either way we return with nwritten which will either
               * be number of bytes written or negated errno.
               */

              nxmutex_unlock(&priv->lock);
              leave_critical_section(flags);
              return nwritten;
            }
        }

      /* Either, there is already some data on the txbuffer, which
       * means IPCC is occupied, or txbuffer is empty, but we could
       * not write whole buffer to IPCC memory. In either case we
       * copy what is left in data to buffer.
       */

      nwritten += circbuf_write(&priv->ipcc->txbuf, buffer + nwritten,
                                (ssize_t)buflen - nwritten);

      /* Notify lower half that new data on circ buffer is available */

      priv->ipcc->ops.write_notify(priv->ipcc);

      if (nwritten == (ssize_t)buflen)
        {
          /* All outstanding data has been copied to txbuffer, we're done */

          nxmutex_unlock(&priv->lock);
          leave_critical_section(flags);
          return nwritten;
        }

#else /* CONFIG_IPCC_BUFFERED */
      /* Unbuffered write, write data directly to lower driver */

      nwritten += priv->ipcc->ops.write(priv->ipcc, buffer + nwritten,
                                        buflen - nwritten);
      if (nwritten == (ssize_t)buflen)
        {
          /* We've managed to write whole buffer to IPCC memory,
           * there is nothing else for use to do
           *   --or--
           * lower half driver returned error during write,
           *
           * either way we return with nwritten which will either
           * be number of bytes written or negated errno.
           */

          nxmutex_unlock(&priv->lock);
          leave_critical_section(flags);
          return nwritten;
        }

#endif /* CONFIG_IPCC_BUFFERED */

      /* No space left on buffer, should we block? */

      if (filep->f_oflags & O_NONBLOCK)
        {
          /* No, we should not block, return number of bytes written or
           * -EAGAIN when we did not write anything
           */

          nxmutex_unlock(&priv->lock);
          leave_critical_section(flags);
          return nwritten ? nwritten : -EAGAIN;
        }

      /* We are in blocking mode, so we have to wait for space
       * to write data
       */

      nxmutex_unlock(&priv->lock);

      if ((ret = nxsem_wait(&priv->txsem)))
        {
          leave_critical_section(flags);

          /* We were interrupted by signal, return error or number
           * of bytes written
           */

          return nwritten ? nwritten : -EINTR;
        }

      /* Space should now be available, but it's possible that
       * another thread will write data to txbuffer before
       * we can do it, so we will stay in the loop until we
       * manage to write whole buffer - or interrupt signal occurs.
       *
       * We have released exclusive lock to driver when we were
       * waiting for data, so now let's retake it.
       */

      nxmutex_lock(&priv->lock);
    }

  leave_critical_section(flags);
}
