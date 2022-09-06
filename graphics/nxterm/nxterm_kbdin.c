/****************************************************************************
 * graphics/nxterm/nxterm_kbdin.c
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

#include <inttypes.h>
#include <fcntl.h>
#include <sched.h>
#include <assert.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>

#include "nxterm.h"

#ifdef CONFIG_NXTERM_NXKBDIN

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxterm_pollnotify
 ****************************************************************************/

static void nxterm_pollnotify(FAR struct nxterm_state_s *priv,
                              pollevent_t eventset)
{
  irqstate_t flags;
  int i;

  /* This function may be called from an interrupt handler */

  for (i = 0; i < CONFIG_NXTERM_NPOLLWAITERS; i++)
    {
      flags = enter_critical_section();
      poll_notify(&priv->fds[i], 1, eventset);
      leave_critical_section(flags);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxterm_read
 *
 * Description:
 *   The optional NxTerm read method
 *
 ****************************************************************************/

ssize_t nxterm_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct nxterm_state_s *priv;
  ssize_t nread;
  char ch;
  int ret;

  /* Recover our private state structure */

  DEBUGASSERT(filep && filep->f_priv);
  priv = (FAR struct nxterm_state_s *)filep->f_priv;

  /* Get exclusive access to the driver structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      gerr("ERROR: nxmutex_lock failed\n");
      return ret;
    }

  /* Loop until something is read */

  for (nread = 0; nread < len; )
    {
      /* Get the next byte from the buffer */

      if (priv->head == priv->tail)
        {
          /* The circular buffer is empty. Did we read anything? */

          if (nread > 0)
            {
              /* Yes.. break out to return what we have.  */

              break;
            }

          /* If the driver was opened with O_NONBLOCK option, then
           * don't wait. Just return EGAIN.
           */

          if (filep->f_oflags & O_NONBLOCK)
            {
              nread = -EAGAIN;
              break;
            }

          /* Otherwise, wait for something to be written to the circular
           * buffer. Increment the number of waiters so that the
           * nxterm_write() will not that it needs to post the semaphore
           * to wake us up.
           */

          sched_lock();
          priv->nwaiters++;
          nxmutex_unlock(&priv->lock);

          /* We may now be pre-empted!  But that should be okay because we
           * have already incremented nwaiters.  Pre-emption is disabled
           * but will be re-enabled while we are waiting.
           */

          ret = nxsem_wait(&priv->waitsem);

          /* Pre-emption will be disabled when we return.  So the
           * decrementing nwaiters here is safe.
           */

          priv->nwaiters--;
          sched_unlock();

          /* Did we successfully get the waitsem? */

          if (ret >= 0)
            {
              /* Yes... then retake the mutual exclusion mutex */

              ret = nxmutex_lock(&priv->lock);
            }

          /* Was the mutex wait successful? Did we successful re-take the
           * mutual exclusion mutex?
           */

          if (ret < 0)
            {
              /* No.. One of the two nxmutex_lock's failed. */

              gerr("ERROR: nxmutex_lock failed\n");

              /* Were we awakened by a signal?  Did we read anything before
               * we received the signal?
               */

              if (ret != -EINTR || nread >= 0)
                {
                  /* Yes.. return the error. */

                  nread = ret;
                }

              /* Break out to return what we have.  Note, we can't exactly
               * "break" out because whichever error occurred, we do not hold
               * the exclusion mutex.
               */

              goto errout_without_lock;
            }
        }
      else
        {
          /* The circular buffer is not empty, get the next byte from the
           * tail index.
           */

          ch = priv->rxbuffer[priv->tail];

          /* Increment the tail index and re-enable interrupts */

          if (++priv->tail >= CONFIG_NXTERM_KBDBUFSIZE)
            {
              priv->tail = 0;
            }

          /* Add the character to the user buffer */

          buffer[nread] = ch;
          nread++;
        }
    }

  /* Relinquish the mutual exclusion mutex */

  nxmutex_unlock(&priv->lock);

  /* Notify all poll/select waiters that they can write to the FIFO */

errout_without_lock:
  if (nread > 0)
    {
      nxterm_pollnotify(priv, POLLOUT);
    }

  /* Return the number of characters actually read */

  return nread;
}

/****************************************************************************
 * Name: nxterm_poll
 ****************************************************************************/

int nxterm_poll(FAR struct file *filep, FAR struct pollfd *fds, bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct nxterm_state_s *priv;
  pollevent_t eventset;
  int ret;
  int i;

  /* Some sanity checking */

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  /* Get exclusive access to the driver structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      gerr("ERROR: nxmutex_lock failed\n");
      return ret;
    }

  /* Are we setting up the poll?  Or tearing it down? */

  if (setup)
    {
      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_NXTERM_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv       = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_NXTERM_NPOLLWAITERS)
        {
          gerr("ERROR: Too many poll waiters\n");

          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should immediately notify on any of the requested events?
       * This driver is always available for transmission.
       */

      eventset = POLLOUT;

      /* Check if the receive buffer is empty */

      if (priv->head != priv->tail)
        {
          eventset |= POLLIN;
        }

      nxterm_pollnotify(priv, eventset);
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;

#ifdef CONFIG_DEBUG_GRAPHICS
      if (!slot)
        {
          gerr("ERROR: No slot\n");

          ret = -EIO;
          goto errout;
        }
#endif

      /* Remove all memory of the poll setup */

      *slot     = NULL;
      fds->priv = NULL;
    }

errout:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: nxterm_kbdin
 *
 * Description:
 *  This function should be driven by the window kbdin callback function
 *  (see nx.h).  When the NxTerm is the top window and keyboard input is
 *  received on the top window, that window callback should be directed to
 *  this function.  This function will buffer the keyboard data and makE
 *  it available to the NxTerm as stdin.
 *
 *  If CONFIG_NXTERM_NXKBDIN is not selected, then the NxTerm will
 *  receive its input from stdin (/dev/console).  This works great but
 *  cannot be shared between different windows.  Chaos will ensue if you
 *  try to support multiple NxTerm windows without CONFIG_NXTERM_NXKBDIN
 *
 * Input Parameters:
 *   handle - A handle previously returned by nx_register, nxtk_register, or
 *     nxtool_register.
 *   buffer   - The array of characters
 *   buflen  - The number of characters that are available in buffer[]
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxterm_kbdin(NXTERM handle, FAR const uint8_t *buffer, uint8_t buflen)
{
  FAR struct nxterm_state_s *priv;
  ssize_t nwritten;
  int nexthead;
  char ch;
  int ret;

  ginfo("buflen=%" PRId8 "\n", buflen);
  DEBUGASSERT(handle);

  /* Get the reference to the driver structure from the handle */

  priv = (FAR struct nxterm_state_s *)handle;

  /* Get exclusive access to the driver structure */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      gerr("ERROR: nxmutex_lock failed\n");
      return;
    }

  /* Loop until all of the bytes have been written.  This function may be
   * called from an interrupt handler!  Semaphores cannot be used!
   *
   * The write logic only needs to modify the head index.  Therefore,
   * there is a difference in the way that head and tail are protected:
   * tail is protected with a semaphore; tail is protected by disabling
   * interrupts.
   */

  for (nwritten = 0; nwritten < buflen; nwritten++)
    {
      /* Add the next character */

      ch = buffer[nwritten];

      /* Calculate the write index AFTER the next byte is add to the ring
       * buffer
       */

      nexthead = priv->head + 1;
      if (nexthead >= CONFIG_NXTERM_KBDBUFSIZE)
        {
          nexthead = 0;
        }

      /* Would the next write overflow the circular buffer? */

      if (nexthead == priv->tail)
        {
          /* Yes... Return an indication that nothing was saved in
           * the buffer.
           */

          gerr("ERROR: Keyboard data overrun\n");
          break;
        }

      /* No... copy the byte */

      priv->rxbuffer[priv->head] = ch;
      priv->head = nexthead;
    }

  /* Was anything written? */

  if (nwritten > 0)
    {
      int i;

      /* Are there threads waiting for read data? */

      sched_lock();

      /* Notify all poll/select waiters that they can read from the FIFO */

      nxterm_pollnotify(priv, POLLIN);

      for (i = 0; i < priv->nwaiters; i++)
        {
          /* Yes.. Notify all of the waiting readers that more data is
           * available
           */

          nxsem_post(&priv->waitsem);
        }

      sched_unlock();
    }

  nxmutex_unlock(&priv->lock);
}

#endif /* CONFIG_NXTERM_NXKBDIN */
