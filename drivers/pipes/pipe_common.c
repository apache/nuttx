/****************************************************************************
 * drivers/pipes/pipe_common.c
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

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/param.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sched.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <nuttx/debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/lib/lib.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/fs/uio.h>

#include "pipe_common.h"

#ifdef CONFIG_PIPES

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_DEV_PIPEDUMP will dump the contents of each transfer into and out
 * of the pipe.
 */

#ifdef CONFIG_DEV_PIPEDUMP
#  define pipe_dumpbuffer(m,a,n) lib_dumpbuffer(m,a,n)
#else
#  define pipe_dumpbuffer(m,a,n)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pipecommon_wakeup
 ****************************************************************************/

static void pipecommon_wakeup(FAR sem_t *sem)
{
  int sval;

  if (nxsem_get_value(sem, &sval) >= 0)
    {
      while (sval++ <= 0)
        {
          nxsem_post(sem);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pipecommon_allocdev
 ****************************************************************************/

FAR struct pipe_dev_s *pipecommon_allocdev(size_t bufsize)
{
  FAR struct pipe_dev_s *dev;

  DEBUGASSERT(bufsize <= CONFIG_DEV_PIPE_MAXSIZE);

  /* Allocate a private structure to manage the pipe */

  dev = kmm_zalloc(sizeof(struct pipe_dev_s));
  if (dev)
    {
      /* Initialize the circular buffer */

      if (circbuf_init(&dev->d_buffer, NULL, bufsize) < 0)
        {
          kmm_free(dev);
          return NULL;
        }

      /* Initialize the private structure */

      rspin_lock_init(&dev->d_bflock);
      nxsem_init(&dev->d_rdsem, 0, 0);
      nxsem_init(&dev->d_wrsem, 0, 0);
      dev->d_bufsize = bufsize;
    }

  return dev;
}

/****************************************************************************
 * Name: pipecommon_freedev
 ****************************************************************************/

void pipecommon_freedev(FAR struct pipe_dev_s *dev)
{
  DEBUGASSERT(!rspin_lock_is_locked(&dev->d_bflock));
  nxsem_destroy(&dev->d_rdsem);
  nxsem_destroy(&dev->d_wrsem);
  circbuf_uninit(&dev->d_buffer);
  kmm_free(dev);
}

/****************************************************************************
 * Name: pipecommon_open
 ****************************************************************************/

int pipecommon_open(FAR struct file *filep)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct pipe_dev_s *dev   = inode->i_private;
  int                    ret   = OK;
  irqstate_t             flags;

  DEBUGASSERT(dev != NULL);

  /* Make sure that we have exclusive access to the device structure.  The
   * nxrmutex_lock() call should fail if we are awakened by a signal or if
   * the thread was canceled.
   */

  flags = rspin_lock_irqsave_nopreempt(&dev->d_bflock);

  dev->d_crefs++;

  /* If opened for writing, increment the count of writers on the pipe
   * instance.
   */

  if ((filep->f_oflags & O_ACCMODE) != O_RDONLY)
    {
      dev->d_nwriters++;

      /* If this is the first writer, then the n-readers semaphore
       * indicates the number of readers waiting for the first writer.
       * Wake them all up!
       */

      if (dev->d_nwriters == 1)
        {
          pipecommon_wakeup(&dev->d_rdsem);
        }
    }

  while ((filep->f_oflags & O_NONBLOCK) == 0 &&       /* Blocking */
         (filep->f_oflags & O_ACCMODE) == O_WRONLY && /* Write-only */
         dev->d_nreaders < 1 &&                       /* No readers on the pipe */
         circbuf_is_empty(&dev->d_buffer))            /* Buffer is empty */
    {
      /* If opened for write-only, then wait for at least one reader
       * on the pipe.
       */

      rspin_unlock_irqrestore_nopreempt(&dev->d_bflock, flags);

      /* NOTE: d_wrsem is normally used to check if the write buffer is full
       * and wait for it being read and being able to receive more data. But,
       * until the first reader has opened the pipe, the meaning is different
       * and it is used prevent O_WRONLY open calls from returning until
       * there is at least one reader on the pipe.
       */

      ret = nxsem_wait(&dev->d_wrsem);
      if (ret < 0)
        {
          ferr("ERROR: nxsem_wait failed: %d\n", ret);

          /* Immediately close the pipe that we just opened */

          pipecommon_close(filep);
          return ret;
        }

      /* The nxrmutex_lock() call should fail if we are awakened by a
       * signal or if the task is canceled.
       */

      flags = rspin_lock_irqsave_nopreempt(&dev->d_bflock);
    }

  /* If opened for reading, increment the count of reader on on the pipe
   * instance.
   */

  if ((filep->f_oflags & O_ACCMODE) != O_WRONLY)
    {
      dev->d_nreaders++;

      /* If this is the first reader, then the n-writers semaphore
       * indicates the number of writers waiting for the first reader.
       * Wake them all up.
       */

      if (dev->d_nreaders == 1)
        {
          pipecommon_wakeup(&dev->d_wrsem);
        }
    }

  while ((filep->f_oflags & O_NONBLOCK) == 0 &&       /* Blocking */
         (filep->f_oflags & O_ACCMODE) == O_RDONLY && /* Read-only */
         dev->d_nwriters < 1 &&                       /* No writers on the pipe */
         circbuf_is_empty(&dev->d_buffer))            /* Buffer is empty */
    {
      /* If opened for read-only, then wait for either at least one writer
       * on the pipe.
       */

      rspin_unlock_irqrestore_nopreempt(&dev->d_bflock, flags);

      /* NOTE: d_rdsem is normally used when the read logic waits for more
       * data to be written.  But until the first writer has opened the
       * pipe, the meaning is different: it is used prevent O_RDONLY open
       * calls from returning until there is at least one writer on the pipe.
       * This is required both by spec and also because it prevents
       * subsequent read() calls from returning end-of-file because there is
       * no writer on the pipe.
       */

      ret = nxsem_wait(&dev->d_rdsem);
      if (ret < 0)
        {
          ferr("ERROR: nxsem_wait failed: %d\n", ret);

          /* Immediately close the pipe that we just opened */

          pipecommon_close(filep);
          return ret;
        }

      /* The nxrmutex_lock() call should fail if we are awakened by a
       * signal or if the task is canceled.
       */

      flags = rspin_lock_irqsave_nopreempt(&dev->d_bflock);
    }

  rspin_unlock_irqrestore_nopreempt(&dev->d_bflock, flags);
  return ret;
}

/****************************************************************************
 * Name: pipecommon_close
 ****************************************************************************/

int pipecommon_close(FAR struct file *filep)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct pipe_dev_s *dev   = inode->i_private;
  irqstate_t             flags;

  DEBUGASSERT(dev && dev->d_crefs > 0);

  /* Make sure that we have exclusive access to the device structure.
   * NOTE: close() is supposed to return EINTR if interrupted, however
   * I've never seen anyone check that.
   */

  flags = rspin_lock_irqsave_nopreempt(&dev->d_bflock);

  /* Decrement the number of references on the pipe.  Check if there are
   * still outstanding references to the pipe.
   */

  /* Check if the decremented inode reference count would go to zero */

  dev->d_crefs--;
  if (dev->d_crefs > 0)
    {
      /* More references.. If opened for writing, decrement the count of
       * writers on the pipe instance.
       */

      if ((filep->f_oflags & O_ACCMODE) != O_RDONLY)
        {
          /* If there are no longer any writers on the pipe, then notify all
           * of the waiting readers that they must return end-of-file.
           */

          if (--dev->d_nwriters <= 0)
            {
              /* Inform poll readers that other end closed.  Match
               * pipecommon_poll(), which reports POLLHUP only under
               * buffer policy 0: under policy 1 the data stays valid for
               * the next reader, so the loss of the last writer is not an
               * end-of-file condition.  A datagram socket opens and closes
               * its sending FIFO for every packet, so reporting POLLHUP
               * here would flag a hangup on each datagram.
               */

              if (PIPE_IS_POLICY_0(dev->d_flags))
                {
                  poll_notify(dev->d_fds, CONFIG_DEV_PIPE_NPOLLWAITERS,
                              POLLHUP);
                }

              pipecommon_wakeup(&dev->d_rdsem);
            }
        }

      /* If opened for reading, decrement the count of readers on the pipe
       * instance.
       */

      if ((filep->f_oflags & O_ACCMODE) != O_WRONLY)
        {
          if (--dev->d_nreaders <= 0)
            {
              if (PIPE_IS_POLICY_0(dev->d_flags))
                {
                  /* Inform poll writers that other end closed. */

                  poll_notify(dev->d_fds, CONFIG_DEV_PIPE_NPOLLWAITERS,
                              POLLERR);
                  pipecommon_wakeup(&dev->d_wrsem);
                }
            }
        }
    }

  /* What is the buffer management policy?  Do we free the buffer when the
   * last client closes the pipe policy 0, or when the buffer becomes empty.
   * In the latter case, the buffer data will remain valid and can be
   * obtained when the pipe is re-opened.
   */

  else if (PIPE_IS_POLICY_0(dev->d_flags) ||
           circbuf_is_empty(&dev->d_buffer))
    {
      /* Policy 0 or the buffer is empty ... Reset the buffer now. */

      circbuf_reset(&dev->d_buffer);

      /* And reset all counts and indices */

      dev->d_nwriters = 0;
      dev->d_nreaders = 0;

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
      /* If, in addition, we have been unlinked, then also need to free the
       * device structure as well to prevent a memory leak.
       */

      if (PIPE_IS_UNLINKED(dev->d_flags))
        {
          rspin_unlock_irqrestore_nopreempt(&dev->d_bflock, flags);
          pipecommon_freedev(dev);
          return OK;
        }
#endif
    }

  rspin_unlock_irqrestore_nopreempt(&dev->d_bflock, flags);
  return OK;
}

/****************************************************************************
 * Name: pipecommon_read
 ****************************************************************************/

ssize_t pipecommon_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct pipe_dev_s *dev   = inode->i_private;
  ssize_t                nread = 0;
  irqstate_t             flags;
  int                    ret;

  DEBUGASSERT(dev);

  if (len == 0)
    {
      return 0;
    }

  /* Make sure that we have exclusive access to the device structure */

  flags = rspin_lock_irqsave_nopreempt(&dev->d_bflock);

  /* If the pipe is empty, then wait for something to be written to it */

  while (circbuf_is_empty(&dev->d_buffer))
    {
      /* If there are no writers on the pipe, then return end of file */

      if (dev->d_nwriters <= 0 && PIPE_IS_POLICY_0(dev->d_flags))
        {
          rspin_unlock_irqrestore_nopreempt(&dev->d_bflock, flags);
          return 0;
        }

      /* If O_NONBLOCK was set, then return EGAIN */

      if (filep->f_oflags & O_NONBLOCK)
        {
          rspin_unlock_irqrestore_nopreempt(&dev->d_bflock, flags);
          return -EAGAIN;
        }

      /* Otherwise, wait for something to be written to the pipe */

      rspin_unlock_irqrestore_nopreempt(&dev->d_bflock, flags);
      ret = nxsem_wait(&dev->d_rdsem);

      if (ret < 0)
        {
          ferr("ERROR: nxsem_wait failed: %d\n", ret);
          return ret;
        }

      flags = rspin_lock_irqsave_nopreempt(&dev->d_bflock);
    }

  /* Then return whatever is available in the pipe (which is at least one
   * byte).
   */

  nread = circbuf_read(&dev->d_buffer, buffer, len);

  /* Notify all poll/select waiters that they can write to the
   * FIFO when buffer can accept more than d_polloutthrd bytes.
   */

  if (circbuf_used(&dev->d_buffer) <= (dev->d_bufsize - dev->d_polloutthrd))
    {
      poll_notify(dev->d_fds, CONFIG_DEV_PIPE_NPOLLWAITERS, POLLOUT);
    }

  /* Notify all waiting writers that bytes have been removed from the
   * buffer.
   */

  pipecommon_wakeup(&dev->d_wrsem);

  rspin_unlock_irqrestore_nopreempt(&dev->d_bflock, flags);
  pipe_dumpbuffer("From PIPE:", buffer, nread);
  return nread;
}

/****************************************************************************
 * Name: pipecommon_write
 ****************************************************************************/

ssize_t pipecommon_write(FAR struct file *filep, FAR const char *buffer,
                         size_t len)
{
  FAR struct inode      *inode    = filep->f_inode;
  FAR struct pipe_dev_s *dev      = inode->i_private;
  ssize_t                nwritten = 0;
  ssize_t                last;
  irqstate_t             flags;
  int                    ret;

  DEBUGASSERT(dev);
  pipe_dumpbuffer("To PIPE:", (FAR uint8_t *)buffer, len);

  /* Handle zero-length writes */

  if (len == 0)
    {
      return 0;
    }

  /* At present, this method cannot be called from interrupt handlers.  That
   * is because it calls nxrmutex_lock() and nxrmutex_lock() cannot be called
   * form interrupt level. This actually happens fairly commonly
   * IF [a-z]err() is called from interrupt handlers and stdout is being
   * redirected via a pipe.  In that case, the debug output will try to go
   * out the pipe (interrupt handlers should use the _err() APIs).
   *
   * On the other hand, it would be very valuable to be able to feed the pipe
   * from an interrupt handler!  TODO:  Consider disabling interrupts instead
   * of taking semaphores so that pipes can be written from interrupt
   * handlers.
   */

  DEBUGASSERT(up_interrupt_context() == false);

  /* Make sure that we have exclusive access to the device structure */

  flags = rspin_lock_irqsave_nopreempt(&dev->d_bflock);

  /* Loop until all of the bytes have been written */

  last = 0;
  for (; ; )
    {
      /* REVISIT:  "If all file descriptors referring to the read end of a
       * pipe have been closed, then a write will cause a SIGPIPE signal to
       * be generated for the calling process.  If the calling process is
       * ignoring this signal, then write(2) fails with the error EPIPE."
       */

      if (dev->d_nreaders <= 0 && PIPE_IS_POLICY_0(dev->d_flags))
        {
          rspin_unlock_irqrestore_nopreempt(&dev->d_bflock, flags);
          return nwritten == 0 ? -EPIPE : nwritten;
        }

      /* Would the next write overflow the circular buffer? */

      if (!circbuf_is_full(&dev->d_buffer))
        {
          /* Loop until all of the bytes have been written */

          nwritten += circbuf_write(&dev->d_buffer,
                                    buffer + nwritten, len - nwritten);

          if ((size_t)nwritten == len)
            {
              /* Notify all poll/select waiters that they can read from the
               * FIFO when buffer used exceeds poll threshold.
               */

              if (circbuf_used(&dev->d_buffer) > dev->d_pollinthrd)
                {
                  poll_notify(dev->d_fds, CONFIG_DEV_PIPE_NPOLLWAITERS,
                              POLLIN);
                }

              /* Yes.. Notify all of the waiting readers that more data is
               * available.
               */

              pipecommon_wakeup(&dev->d_rdsem);

              /* Return the number of bytes written */

              rspin_unlock_irqrestore_nopreempt(&dev->d_bflock, flags);
              return len;
            }
        }
      else
        {
          /* There is not enough room for the next byte.  Was anything
           * written in this pass?
           */

          if (last < nwritten)
            {
              /* Notify all poll/select waiters that they can read from the
               * FIFO.
               */

              poll_notify(dev->d_fds, CONFIG_DEV_PIPE_NPOLLWAITERS, POLLIN);

              /* Yes.. Notify all of the waiting readers that more data is
               * available.
               */

              pipecommon_wakeup(&dev->d_rdsem);
            }

          last = nwritten;

          /* If O_NONBLOCK was set, then return partial bytes written or
           * EGAIN.
           */

          if (filep->f_oflags & O_NONBLOCK)
            {
              if (nwritten == 0)
                {
                  nwritten = -EAGAIN;
                }

              rspin_unlock_irqrestore_nopreempt(&dev->d_bflock, flags);
              return nwritten;
            }

          /* There is more to be written.. wait for data to be removed from
           * the pipe
           */

          rspin_unlock_irqrestore_nopreempt(&dev->d_bflock, flags);
          ret = nxsem_wait(&dev->d_wrsem);
          if (ret < 0)
            {
              /* Either call nxsem_wait may fail because a signal was
               * received or if the task was canceled.
               */

              return nwritten == 0 ? (ssize_t)ret : nwritten;
            }

          flags = rspin_lock_irqsave_nopreempt(&dev->d_bflock);
        }
    }
}

/****************************************************************************
 * Name: pipecommon_writev
 *
 * Description:
 *   Gather write.  In the default byte-stream mode this behaves like a
 *   sequence of single-buffer writes: each segment is copied in order,
 *   short writes and blocking happen exactly as they would for a plain
 *   write(), and a request larger than the whole buffer is admitted a
 *   piece at a time as the reader drains it.  When PIPE_FLAG_ATOMIC is set,
 *   the whole request is admitted all-or-nothing: no byte becomes visible
 *   to the reader until every segment has been copied while holding
 *   d_bflock.  A single writev() then carries one complete message (a
 *   datagram whose header and payload live in separate segments), so a
 *   reader never observes a partial frame; a request that cannot ever fit
 *   the buffer is rejected with -EMSGSIZE instead of being written in part.
 *
 ****************************************************************************/

ssize_t pipecommon_writev(FAR struct file *filep, FAR struct uio *uio)
{
  FAR struct inode       *inode = filep->f_inode;
  FAR struct pipe_dev_s  *dev   = inode->i_private;
  FAR const struct iovec *iov;
  size_t                  offset;
  size_t                  total;
  irqstate_t              flags;
  int                     iovcnt;
  int                     ret;

  DEBUGASSERT(dev);
  DEBUGASSERT(up_interrupt_context() == false);

  total = uio->uio_resid;
  if (total == 0)
    {
      return 0;
    }

  if (!PIPE_IS_ATOMIC(dev->d_flags))
    {
      ssize_t ntotal = 0;
      ssize_t last;

      /* Byte-stream gather write: copy each segment in order, applying the
       * same short-write, blocking and error reporting rules a plain
       * write() would.  A request larger than the buffer is written a piece
       * at a time as the reader drains it, so there is no upper bound here.
       */

      iov    = uio->uio_iov;
      iovcnt = uio->uio_iovcnt;
      offset = uio->uio_offset_in_iov;

      flags = rspin_lock_irqsave_nopreempt(&dev->d_bflock);

      last = 0;
      while (iovcnt > 0)
        {
          FAR const uint8_t *base = (FAR const uint8_t *)iov->iov_base;
          size_t             len  = iov->iov_len - offset;

          if (len == 0)
            {
              offset = 0;
              iov++;
              iovcnt--;
              continue;
            }

          /* REVISIT:  "If all file descriptors referring to the read end of
           * a pipe have been closed, then a write will cause a SIGPIPE
           * signal to be generated for the calling process.  If the calling
           * process is ignoring this signal, then write(2) fails with the
           * error EPIPE."
           */

          if (dev->d_nreaders <= 0 && PIPE_IS_POLICY_0(dev->d_flags))
            {
              rspin_unlock_irqrestore_nopreempt(&dev->d_bflock, flags);
              if (ntotal == 0)
                {
                  return -EPIPE;
                }

              uio_advance(uio, ntotal);
              return ntotal;
            }

          /* Would the next write overflow the circular buffer? */

          if (!circbuf_is_full(&dev->d_buffer))
            {
              size_t nwritten;

              nwritten = circbuf_write(&dev->d_buffer, base + offset, len);
              ntotal += nwritten;
              offset += nwritten;

              if (nwritten == len)
                {
                  offset = 0;
                  iov++;
                  iovcnt--;
                }

              if (iovcnt == 0)
                {
                  /* Notify all poll/select waiters that they can read from
                   * the FIFO when buffer used exceeds poll threshold.
                   */

                  if (circbuf_used(&dev->d_buffer) > dev->d_pollinthrd)
                    {
                      poll_notify(dev->d_fds, CONFIG_DEV_PIPE_NPOLLWAITERS,
                                  POLLIN);
                    }

                  pipecommon_wakeup(&dev->d_rdsem);

                  rspin_unlock_irqrestore_nopreempt(&dev->d_bflock, flags);
                  uio_advance(uio, ntotal);
                  return ntotal;
                }
            }
          else
            {
              /* There is not enough room for the next byte.  Was anything
               * written in this pass?
               */

              if (last < ntotal)
                {
                  poll_notify(dev->d_fds, CONFIG_DEV_PIPE_NPOLLWAITERS,
                              POLLIN);
                  pipecommon_wakeup(&dev->d_rdsem);
                }

              last = ntotal;

              /* If O_NONBLOCK was set, then return partial bytes written or
               * EAGAIN.
               */

              if (filep->f_oflags & O_NONBLOCK)
                {
                  rspin_unlock_irqrestore_nopreempt(&dev->d_bflock, flags);
                  if (ntotal == 0)
                    {
                      return -EAGAIN;
                    }

                  uio_advance(uio, ntotal);
                  return ntotal;
                }

              /* There is more to be written.. wait for data to be removed
               * from the pipe.
               */

              rspin_unlock_irqrestore_nopreempt(&dev->d_bflock, flags);
              ret = nxsem_wait(&dev->d_wrsem);
              if (ret < 0)
                {
                  /* nxsem_wait may fail because a signal was received or
                   * because the task was canceled.
                   */

                  if (ntotal == 0)
                    {
                      return (ssize_t)ret;
                    }

                  uio_advance(uio, ntotal);
                  return ntotal;
                }

              flags = rspin_lock_irqsave_nopreempt(&dev->d_bflock);
            }
        }

      rspin_unlock_irqrestore_nopreempt(&dev->d_bflock, flags);
      uio_advance(uio, ntotal);
      return ntotal;
    }

  /* A request bigger than the whole buffer can never be admitted
   * atomically, so reporting -EAGAIN would make the caller retry forever
   * and blocking would wait for space that cannot appear.
   */

  if (total > dev->d_bufsize)
    {
      return -EMSGSIZE;
    }

  flags = rspin_lock_irqsave_nopreempt(&dev->d_bflock);

  for (; ; )
    {
      if (dev->d_nreaders <= 0 && PIPE_IS_POLICY_0(dev->d_flags))
        {
          rspin_unlock_irqrestore_nopreempt(&dev->d_bflock, flags);
          return -EPIPE;
        }

      if (circbuf_space(&dev->d_buffer) >= total)
        {
          break;
        }

      if (filep->f_oflags & O_NONBLOCK)
        {
          rspin_unlock_irqrestore_nopreempt(&dev->d_bflock, flags);
          return -EAGAIN;
        }

      /* Wait for the reader to free enough room for the whole request.
       * Nothing has been written yet, so there is no partial state to
       * publish or to roll back here.
       */

      rspin_unlock_irqrestore_nopreempt(&dev->d_bflock, flags);
      ret = nxsem_wait(&dev->d_wrsem);
      if (ret < 0)
        {
          return ret;
        }

      flags = rspin_lock_irqsave_nopreempt(&dev->d_bflock);
    }

  /* Room for the whole request is guaranteed from here to the notify
   * below, and d_bflock is held throughout, so the reader observes the
   * request as a single unit.  Only uio_iov[0] carries an offset.
   */

  iov    = uio->uio_iov;
  iovcnt = uio->uio_iovcnt;
  offset = uio->uio_offset_in_iov;

  while (iovcnt-- > 0)
    {
      size_t len = iov->iov_len - offset;

      if (len > 0)
        {
          pipe_dumpbuffer("To PIPE:",
                          (FAR uint8_t *)iov->iov_base + offset, len);
          circbuf_write(&dev->d_buffer,
                        (FAR const uint8_t *)iov->iov_base + offset, len);
        }

      offset = 0;
      iov++;
    }

  if (circbuf_used(&dev->d_buffer) > dev->d_pollinthrd)
    {
      poll_notify(dev->d_fds, CONFIG_DEV_PIPE_NPOLLWAITERS, POLLIN);
    }

  pipecommon_wakeup(&dev->d_rdsem);

  rspin_unlock_irqrestore_nopreempt(&dev->d_bflock, flags);

  uio_advance(uio, total);
  return total;
}

/****************************************************************************
 * Name: pipecommon_poll
 ****************************************************************************/

int pipecommon_poll(FAR struct file *filep, FAR struct pollfd_s *fds,
                    bool setup)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct pipe_dev_s *dev   = inode->i_private;
  int                    ret   = OK;
  pollevent_t            eventset;
  pipe_ndx_t             nbytes;
  irqstate_t             flags;
  int                    i;

  DEBUGASSERT(dev && fds);

  /* Are we setting up the poll?  Or tearing it down? */

  flags = rspin_lock_irqsave_nopreempt(&dev->d_bflock);

  if (setup)
    {
      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_DEV_PIPE_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!dev->d_fds[i])
            {
              /* Bind the poll structure and this slot */

              dev->d_fds[i] = fds;
              fds->priv     = &dev->d_fds[i];
              break;
            }
        }

      if (i >= CONFIG_DEV_PIPE_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret       = -EBUSY;
          goto errout;
        }

      /* Should immediately notify on any of the requested events?
       * First, determine how many bytes are in the buffer
       */

      nbytes = circbuf_used(&dev->d_buffer);

      /* Notify the POLLOUT event if the pipe buffer can accept
       * more than d_polloutthrd bytes, but only if
       * there is readers.
       */

      eventset = 0;
      if ((filep->f_oflags & O_ACCMODE) != O_RDONLY &&
          nbytes < dev->d_bufsize - dev->d_polloutthrd)
        {
          eventset |= POLLOUT;
        }

      /* Notify the POLLIN event if buffer used exceeds poll threshold */

      if ((filep->f_oflags & O_ACCMODE) != O_WRONLY &&
          nbytes > dev->d_pollinthrd)
        {
          eventset |= POLLIN;
        }

      /* Notify the POLLHUP event if the pipe is empty,
       * while no writers and policy 0.
       */

      if (nbytes == 0 &&
          dev->d_nwriters <= 0 &&
          PIPE_IS_POLICY_0(dev->d_flags))
        {
          eventset |= POLLHUP;
        }

      /* Change POLLOUT to POLLERR, if no readers and policy 0. */

      if ((eventset & POLLOUT) &&
          PIPE_IS_POLICY_0(dev->d_flags) &&
          dev->d_nreaders <= 0)
        {
          eventset |= POLLERR;
        }

      poll_notify(&fds, 1, eventset);
    }
  else
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd_s **slot = (FAR struct pollfd_s **)fds->priv;

#ifdef CONFIG_DEBUG_FEATURES
      if (!slot)
        {
          ret = -EIO;
          goto errout;
        }
#endif

      /* Remove all memory of the poll setup */

      *slot     = NULL;
      fds->priv = NULL;
    }

errout:
  rspin_unlock_irqrestore_nopreempt(&dev->d_bflock, flags);
  return ret;
}

/****************************************************************************
 * Name: pipecommon_ioctl
 ****************************************************************************/

int pipecommon_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct pipe_dev_s *dev   = inode->i_private;
  int                    ret   = -EINVAL;
  irqstate_t             flags;

#ifdef CONFIG_DEBUG_FEATURES
  /* Some sanity checking */

  if (dev == NULL)
    {
      return -EBADF;
    }
#endif

  flags = rspin_lock_irqsave(&dev->d_bflock);

  switch (cmd)
    {
      case PIPEIOC_POLICY:
        {
          if (arg != 0)
            {
              PIPE_POLICY_1(dev->d_flags);
            }
          else
            {
              PIPE_POLICY_0(dev->d_flags);
            }

          ret = OK;
        }
        break;

      case PIPEIOC_ATOMICWRITE:
        {
          if (arg != 0)
            {
              PIPE_ATOMIC_1(dev->d_flags);
            }
          else
            {
              PIPE_ATOMIC_0(dev->d_flags);
            }

          ret = OK;
        }
        break;

      case PIPEIOC_POLLINTHRD:
        {
          pipe_ndx_t threshold = (pipe_ndx_t)arg;

          if (threshold >= dev->d_bufsize)
            {
              ret = -EINVAL;
              break;
            }

          dev->d_pollinthrd = threshold;
          ret = OK;
        }
        break;

      case PIPEIOC_POLLOUTTHRD:
        {
          pipe_ndx_t threshold = (pipe_ndx_t)arg;

          if (threshold >= dev->d_bufsize)
            {
              ret = -EINVAL;
              break;
            }

          dev->d_polloutthrd = threshold;
          ret = OK;
        }
        break;

      case PIPEIOC_PEEK:
        {
          FAR struct pipe_peek_s *peek = (FAR struct pipe_peek_s *)arg;

          DEBUGASSERT(peek && peek->buf);

          ret = circbuf_peekat(&dev->d_buffer,
                               dev->d_buffer.tail + peek->offset,
                               peek->buf, peek->size);
        }
        break;

      case PIPEIOC_SETSIZE:
        {
          size_t size = (size_t)arg;
          FAR void *newbuf;
          FAR void *oldbuf;

          if (size == 0)
            {
              ret = -EINVAL;
              break;
            }

          size = MIN(size, CONFIG_DEV_PIPE_MAXSIZE);

          if (size == dev->d_bufsize)
            {
              ret = OK;
              break;
            }

          rspin_unlock_irqrestore(&dev->d_bflock, flags);
          newbuf = lib_malloc(size);
          if (!newbuf)
            {
              return -ENOMEM;
            }

          flags = rspin_lock_irqsave(&dev->d_bflock);
          oldbuf = circbuf_reinit(&dev->d_buffer, newbuf, size);
          dev->d_bufsize = size;
          rspin_unlock_irqrestore(&dev->d_bflock, flags);
          lib_free(oldbuf);
          flags = rspin_lock_irqsave(&dev->d_bflock);
        }
        break;

      case PIPEIOC_GETSIZE:
        {
          ret = dev->d_bufsize;
        }
        break;

      case FIONWRITE:  /* Number of bytes waiting in send queue */
      case FIONREAD:   /* Number of bytes available for reading */
        {
          *(FAR int *)((uintptr_t)arg) = circbuf_used(&dev->d_buffer);
          ret = 0;
        }
        break;

      /* Free space in buffer */

      case FIONSPACE:
        {
          *(FAR int *)((uintptr_t)arg) = circbuf_space(&dev->d_buffer);
          ret = 0;
        }
        break;

      case BIOC_FLUSH:
        ret = -EINVAL;
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  rspin_unlock_irqrestore(&dev->d_bflock, flags);
  return ret;
}

/****************************************************************************
 * Name: pipecommon_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
int pipecommon_unlink(FAR struct inode *inode)
{
  FAR struct pipe_dev_s *dev;
  irqstate_t             flags;

  DEBUGASSERT(inode->i_private);
  dev = inode->i_private;

  flags = rspin_lock_irqsave(&dev->d_bflock);

  if (dev->d_crefs <= 0)
    {
      rspin_unlock_irqrestore(&dev->d_bflock, flags);
      pipecommon_freedev(dev);
      return OK;
    }

  /* Mark the pipe unlinked */

  PIPE_UNLINK(dev->d_flags);
  rspin_unlock_irqrestore(&dev->d_bflock, flags);

  return OK;
}
#endif

#endif /* CONFIG_PIPES */
