/****************************************************************************
 * drivers/pipes/pipe_common.c
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
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sched.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>

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
 * Name: pipecommon_bufferused
 ****************************************************************************/

static pipe_ndx_t pipecommon_bufferused(FAR struct pipe_dev_s *dev)
{
  if (dev->d_wrndx >= dev->d_rdndx)
    {
      return dev->d_wrndx - dev->d_rdndx;
    }
  else
    {
      return dev->d_bufsize + dev->d_wrndx - dev->d_rdndx;
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

  dev = (FAR struct pipe_dev_s *)kmm_malloc(sizeof(struct pipe_dev_s));
  if (dev)
    {
      /* Initialize the private structure */

      memset(dev, 0, sizeof(struct pipe_dev_s));
      nxmutex_init(&dev->d_bflock);
      nxsem_init(&dev->d_rdsem, 0, 0);
      nxsem_init(&dev->d_wrsem, 0, 0);
      dev->d_bufsize = bufsize + 1; /* +1 to compensate the full indicator */
    }

  return dev;
}

/****************************************************************************
 * Name: pipecommon_freedev
 ****************************************************************************/

void pipecommon_freedev(FAR struct pipe_dev_s *dev)
{
  nxmutex_destroy(&dev->d_bflock);
  nxsem_destroy(&dev->d_rdsem);
  nxsem_destroy(&dev->d_wrsem);
  kmm_free(dev);
}

/****************************************************************************
 * Name: pipecommon_open
 ****************************************************************************/

int pipecommon_open(FAR struct file *filep)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct pipe_dev_s *dev   = inode->i_private;
  int                    sval;
  int                    ret;

  DEBUGASSERT(dev != NULL);

  /* Make sure that we have exclusive access to the device structure.  The
   * nxmutex_lock() call should fail if we are awakened by a signal or if the
   * thread was canceled.
   */

  ret = nxmutex_lock(&dev->d_bflock);
  if (ret < 0)
    {
      ferr("ERROR: nxmutex_lock failed: %d\n", ret);
      return ret;
    }

  /* If this the first reference on the device, then allocate the buffer.
   * In the case of policy 1, the buffer already be present when the pipe
   * is first opened.
   */

  if (inode->i_crefs == 1 && dev->d_buffer == NULL)
    {
      dev->d_buffer = (FAR uint8_t *)kmm_malloc(dev->d_bufsize);
      if (!dev->d_buffer)
        {
          nxmutex_unlock(&dev->d_bflock);
          return -ENOMEM;
        }
    }

  /* If opened for writing, increment the count of writers on the pipe
   * instance.
   */

  if ((filep->f_oflags & O_WROK) != 0)
    {
      dev->d_nwriters++;

      /* If this is the first writer, then the n-readers semaphore
       * indicates the number of readers waiting for the first writer.
       * Wake them all up!
       */

      if (dev->d_nwriters == 1)
        {
          while (nxsem_get_value(&dev->d_rdsem, &sval) == 0 && sval <= 0)
            {
              nxsem_post(&dev->d_rdsem);
            }
        }
    }

  while ((filep->f_oflags & O_NONBLOCK) == 0 &&     /* Blocking */
         (filep->f_oflags & O_RDWR) == O_WRONLY &&  /* Write-only */
         dev->d_nreaders < 1 &&                     /* No readers on the pipe */
         dev->d_wrndx == dev->d_rdndx)              /* Buffer is empty */
    {
      /* If opened for write-only, then wait for at least one reader
       * on the pipe.
       */

      nxmutex_unlock(&dev->d_bflock);

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

      /* The nxmutex_lock() call should fail if we are awakened by a
       * signal or if the task is canceled.
       */

      ret = nxmutex_lock(&dev->d_bflock);
      if (ret < 0)
        {
          ferr("ERROR: nxmutex_lock failed: %d\n", ret);

          /* Immediately close the pipe that we just opened */

          pipecommon_close(filep);
          return ret;
        }
    }

  /* If opened for reading, increment the count of reader on on the pipe
   * instance.
   */

  if ((filep->f_oflags & O_RDOK) != 0)
    {
      dev->d_nreaders++;

      /* If this is the first reader, then the n-writers semaphore
       * indicates the number of writers waiting for the first reader.
       * Wake them all up.
       */

      if (dev->d_nreaders == 1)
        {
          while (nxsem_get_value(&dev->d_wrsem, &sval) == 0 && sval <= 0)
            {
              nxsem_post(&dev->d_wrsem);
            }
        }
    }

  while ((filep->f_oflags & O_NONBLOCK) == 0 &&     /* Blocking */
         (filep->f_oflags & O_RDWR) == O_RDONLY &&  /* Read-only */
         dev->d_nwriters < 1 &&                     /* No writers on the pipe */
         dev->d_wrndx == dev->d_rdndx)              /* Buffer is empty */
    {
      /* If opened for read-only, then wait for either at least one writer
       * on the pipe.
       */

      nxmutex_unlock(&dev->d_bflock);

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

      /* The nxmutex_lock() call should fail if we are awakened by a
       * signal or if the task is canceled.
       */

      ret = nxmutex_lock(&dev->d_bflock);
      if (ret < 0)
        {
          ferr("ERROR: nxmutex_lock failed: %d\n", ret);

          /* Immediately close the pipe that we just opened */

          pipecommon_close(filep);
          return ret;
        }
    }

  nxmutex_unlock(&dev->d_bflock);
  return ret;
}

/****************************************************************************
 * Name: pipecommon_close
 ****************************************************************************/

int pipecommon_close(FAR struct file *filep)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct pipe_dev_s *dev   = inode->i_private;
  int                    sval;
  int                    ret;

  DEBUGASSERT(dev && filep->f_inode->i_crefs > 0);

  /* Make sure that we have exclusive access to the device structure.
   * NOTE: close() is supposed to return EINTR if interrupted, however
   * I've never seen anyone check that.
   */

  ret = nxmutex_lock(&dev->d_bflock);
  if (ret < 0)
    {
      /* The close will not be performed if the task was canceled */

      return ret;
    }

  /* Decrement the number of references on the pipe.  Check if there are
   * still outstanding references to the pipe.
   */

  /* Check if the decremented inode reference count would go to zero */

  if (inode->i_crefs > 1)
    {
      /* More references.. If opened for writing, decrement the count of
       * writers on the pipe instance.
       */

      if ((filep->f_oflags & O_WROK) != 0)
        {
          /* If there are no longer any writers on the pipe, then notify all
           * of the waiting readers that they must return end-of-file.
           */

          if (--dev->d_nwriters <= 0)
            {
              /* Inform poll readers that other end closed. */

              poll_notify(dev->d_fds, CONFIG_DEV_PIPE_NPOLLWAITERS, POLLHUP);

              while (nxsem_get_value(&dev->d_rdsem, &sval) == 0 && sval <= 0)
                {
                  nxsem_post(&dev->d_rdsem);
                }
            }
        }

      /* If opened for reading, decrement the count of readers on the pipe
       * instance.
       */

      if ((filep->f_oflags & O_RDOK) != 0)
        {
          if (--dev->d_nreaders <= 0)
            {
              if (PIPE_IS_POLICY_0(dev->d_flags))
                {
                  /* Inform poll writers that other end closed. */

                  poll_notify(dev->d_fds, CONFIG_DEV_PIPE_NPOLLWAITERS,
                              POLLERR);
                  while (nxsem_get_value(&dev->d_wrsem, &sval) == 0 &&
                         sval <= 0)
                    {
                      nxsem_post(&dev->d_wrsem);
                    }
                }
            }
        }
    }

  /* What is the buffer management policy?  Do we free the buffer when the
   * last client closes the pipe policy 0, or when the buffer becomes empty.
   * In the latter case, the buffer data will remain valid and can be
   * obtained when the pipe is re-opened.
   */

  else if (PIPE_IS_POLICY_0(dev->d_flags) || dev->d_wrndx == dev->d_rdndx)
    {
      /* Policy 0 or the buffer is empty ... deallocate the buffer now. */

      kmm_free(dev->d_buffer);
      dev->d_buffer = NULL;

      /* And reset all counts and indices */

      dev->d_wrndx    = 0;
      dev->d_rdndx    = 0;
      dev->d_nwriters = 0;
      dev->d_nreaders = 0;

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
      /* If, in addition, we have been unlinked, then also need to free the
       * device structure as well to prevent a memory leak.
       */

      if (PIPE_IS_UNLINKED(dev->d_flags))
        {
          pipecommon_freedev(dev);
          return OK;
        }
#endif
    }

  nxmutex_unlock(&dev->d_bflock);
  return OK;
}

/****************************************************************************
 * Name: pipecommon_read
 ****************************************************************************/

ssize_t pipecommon_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct pipe_dev_s *dev   = inode->i_private;
#ifdef CONFIG_DEV_PIPEDUMP
  FAR uint8_t           *start = (FAR uint8_t *)buffer;
#endif
  ssize_t                nread = 0;
  int                    sval;
  int                    ret;

  DEBUGASSERT(dev);

  if (len == 0)
    {
      return 0;
    }

  /* Make sure that we have exclusive access to the device structure */

  ret = nxmutex_lock(&dev->d_bflock);
  if (ret < 0)
    {
      /* May fail because a signal was received or if the task was
       * canceled.
       */

      return ret;
    }

  /* If the pipe is empty, then wait for something to be written to it */

  while (dev->d_wrndx == dev->d_rdndx)
    {
      /* If there are no writers on the pipe, then return end of file */

      if (dev->d_nwriters <= 0)
        {
          nxmutex_unlock(&dev->d_bflock);
          return 0;
        }

      /* If O_NONBLOCK was set, then return EGAIN */

      if (filep->f_oflags & O_NONBLOCK)
        {
          nxmutex_unlock(&dev->d_bflock);
          return -EAGAIN;
        }

      /* Otherwise, wait for something to be written to the pipe */

      nxmutex_unlock(&dev->d_bflock);
      ret = nxsem_wait(&dev->d_rdsem);

      if (ret < 0 || (ret = nxmutex_lock(&dev->d_bflock)) < 0)
        {
          /* May fail because a signal was received or if the task was
           * canceled.
           */

          return ret;
        }
    }

  /* Then return whatever is available in the pipe (which is at least one
   * byte).
   */

  nread = 0;
  while ((size_t)nread < len && dev->d_wrndx != dev->d_rdndx)
    {
      *buffer++ = dev->d_buffer[dev->d_rdndx];
      if (++dev->d_rdndx >= dev->d_bufsize)
        {
          dev->d_rdndx = 0;
        }

      nread++;
    }

  /* Notify all poll/select waiters that they can write to the
   * FIFO when buffer can accept more than d_polloutthrd bytes.
   */

  if (pipecommon_bufferused(dev) < (dev->d_bufsize - 1 - dev->d_polloutthrd))
    {
      poll_notify(dev->d_fds, CONFIG_DEV_PIPE_NPOLLWAITERS, POLLOUT);
    }

  /* Notify all waiting writers that bytes have been removed from the
   * buffer.
   */

  while (nxsem_get_value(&dev->d_wrsem, &sval) == 0 && sval <= 0)
    {
      nxsem_post(&dev->d_wrsem);
    }

  nxmutex_unlock(&dev->d_bflock);
  pipe_dumpbuffer("From PIPE:", start, nread);
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
  int                    nxtwrndx;
  int                    sval;
  int                    ret;

  DEBUGASSERT(dev);
  pipe_dumpbuffer("To PIPE:", (FAR uint8_t *)buffer, len);

  /* Handle zero-length writes */

  if (len == 0)
    {
      return 0;
    }

  /* At present, this method cannot be called from interrupt handlers.  That
   * is because it calls nxmutex_lock() and nxmutex_lock() cannot be called
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

  ret = nxmutex_lock(&dev->d_bflock);
  if (ret < 0)
    {
      /* May fail because a signal was received or if the task was
       * canceled.
       */

      return ret;
    }

  /* Loop until all of the bytes have been written */

  last = 0;
  for (; ; )
    {
      /* REVISIT:  "If all file descriptors referring to the read end of a
       * pipe have been closed, then a write will cause a SIGPIPE signal to
       * be generated for the calling process.  If the calling process is
       * ignoring this signal, then write(2) fails with the error EPIPE."
       */

      if (dev->d_nreaders <= 0)
        {
          nxmutex_unlock(&dev->d_bflock);
          return nwritten == 0 ? -EPIPE : nwritten;
        }

      /* Calculate the write index AFTER the next byte is written */

      nxtwrndx = dev->d_wrndx + 1;
      if (nxtwrndx >= dev->d_bufsize)
        {
          nxtwrndx = 0;
        }

      /* Would the next write overflow the circular buffer? */

      if (nxtwrndx != dev->d_rdndx)
        {
          /* No... copy the byte */

          dev->d_buffer[dev->d_wrndx] = *buffer++;
          dev->d_wrndx = nxtwrndx;

          /* Is the write complete? */

          nwritten++;
          if ((size_t)nwritten >= len)
            {
              /* Notify all poll/select waiters that they can read from the
               * FIFO when buffer used exceeds poll threshold.
               */

              if (pipecommon_bufferused(dev) > dev->d_pollinthrd)
                {
                  poll_notify(dev->d_fds, CONFIG_DEV_PIPE_NPOLLWAITERS,
                              POLLIN);
                }

              /* Yes.. Notify all of the waiting readers that more data is
               * available.
               */

              while (nxsem_get_value(&dev->d_rdsem, &sval) == 0 && sval <= 0)
                {
                  nxsem_post(&dev->d_rdsem);
                }

              /* Return the number of bytes written */

              nxmutex_unlock(&dev->d_bflock);
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

              while (nxsem_get_value(&dev->d_rdsem, &sval) == 0 && sval <= 0)
                {
                  nxsem_post(&dev->d_rdsem);
                }
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

              nxmutex_unlock(&dev->d_bflock);
              return nwritten;
            }

          /* There is more to be written.. wait for data to be removed from
           * the pipe
           */

          nxmutex_unlock(&dev->d_bflock);
          ret = nxsem_wait(&dev->d_wrsem);
          if (ret < 0 || (ret = nxmutex_lock(&dev->d_bflock)) < 0)
            {
              /* Either call nxsem_wait may fail because a signal was
               * received or if the task was canceled.
               */

              return nwritten == 0 ? (ssize_t)ret : nwritten;
            }
        }
    }
}

/****************************************************************************
 * Name: pipecommon_poll
 ****************************************************************************/

int pipecommon_poll(FAR struct file *filep, FAR struct pollfd *fds,
                    bool setup)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct pipe_dev_s *dev   = inode->i_private;
  pollevent_t            eventset;
  pipe_ndx_t             nbytes;
  int                    ret;
  int                    i;

  DEBUGASSERT(dev && fds);

  /* Are we setting up the poll?  Or tearing it down? */

  ret = nxmutex_lock(&dev->d_bflock);
  if (ret < 0)
    {
      return ret;
    }

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

      nbytes = pipecommon_bufferused(dev);

      /* Notify the POLLOUT event if the pipe buffer can accept
       * more than d_polloutthrd bytes, but only if
       * there is readers.
       */

      eventset = 0;
      if ((filep->f_oflags & O_WROK) &&
          nbytes < (dev->d_bufsize - 1 - dev->d_polloutthrd))
        {
          eventset |= POLLOUT;
        }

      /* Notify the POLLIN event if buffer used exceeds poll threshold */

      if ((filep->f_oflags & O_RDOK) && (nbytes > dev->d_pollinthrd))
        {
          eventset |= POLLIN;
        }

      /* Notify the POLLHUP event if the pipe is empty and no writers */

      if (nbytes == 0 && dev->d_nwriters <= 0)
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

      poll_notify(dev->d_fds, CONFIG_DEV_PIPE_NPOLLWAITERS, eventset);
    }
  else
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;

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
  nxmutex_unlock(&dev->d_bflock);
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

#ifdef CONFIG_DEBUG_FEATURES
  /* Some sanity checking */

  if (dev == NULL)
    {
      return -EBADF;
    }
#endif

  ret = nxmutex_lock(&dev->d_bflock);
  if (ret < 0)
    {
      return ret;
    }

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

      case FIONWRITE:  /* Number of bytes waiting in send queue */
      case FIONREAD:   /* Number of bytes available for reading */
        {
          int count;

          /* Determine the number of bytes written to the buffer.  This is,
           * of course, also the number of bytes that may be read from the
           * buffer.
           *
           *   d_rdndx - index to remove next byte from the buffer
           *   d_wrndx - Index to next location to add a byte to the buffer.
           */

          if (dev->d_wrndx < dev->d_rdndx)
            {
              count = (dev->d_bufsize - dev->d_rdndx) + dev->d_wrndx;
            }
          else
            {
              count = dev->d_wrndx - dev->d_rdndx;
            }

          *(FAR int *)((uintptr_t)arg) = count;
          ret = 0;
        }
        break;

      /* Free space in buffer */

      case FIONSPACE:
        {
          int count;

          /* Determine the number of bytes free in the buffer.
           *
           *   d_rdndx - index to remove next byte from the buffer
           *   d_wrndx - Index to next location to add a byte to the buffer.
           */

          if (dev->d_wrndx < dev->d_rdndx)
            {
              count = (dev->d_rdndx - dev->d_wrndx) - 1;
            }
          else
            {
              count = ((dev->d_bufsize - dev->d_wrndx) + dev->d_rdndx) - 1;
            }

          *(FAR int *)((uintptr_t)arg) = count;
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

  nxmutex_unlock(&dev->d_bflock);
  return ret;
}

/****************************************************************************
 * Name: pipecommon_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
int pipecommon_unlink(FAR struct inode *inode)
{
  FAR struct pipe_dev_s *dev;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct pipe_dev_s *)inode->i_private;

  /* Mark the pipe unlinked */

  PIPE_UNLINK(dev->d_flags);

  /* Are the any open references to the driver? */

  if (inode->i_crefs == 1)
    {
      /* No.. free the buffer (if there is one) */

      if (dev->d_buffer)
        {
          kmm_free(dev->d_buffer);
        }

      /* And free the device structure. */

      pipecommon_freedev(dev);
    }

  return OK;
}
#endif

#endif /* CONFIG_PIPES */
