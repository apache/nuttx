/****************************************************************************
 * drivers/pipe_common.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * Compilation Switches
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <string.h>
#include <sched.h>
#include <semaphore.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <nuttx/fs.h>

#include "pipe_common.h"

#if CONFIG_DEV_PIPE_SIZE > 0

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void pipecommon_semtake(sem_t *sem);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pipecommon_semtake
 ****************************************************************************/

static void pipecommon_semtake(sem_t *sem)
{
  while (sem_wait(sem) != 0)
    {
      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pipecommon_allocdev
 ****************************************************************************/

FAR struct pipe_dev_s *pipecommon_allocdev(void)
{
 struct pipe_dev_s *dev;

  /* Allocate a private structure to manage the pipe */

  dev = (struct pipe_dev_s *)malloc(sizeof(struct pipe_dev_s));
  if (dev)
    {
      /* Initialize the private structure */

      memset(dev, 0, sizeof(struct pipe_dev_s));
      sem_init(&dev->d_bfsem, 0, 1);
      sem_init(&dev->d_rdsem, 0, 0);
      sem_init(&dev->d_wrsem, 0, 0);
    }
  return dev;
}

/****************************************************************************
 * Name: pipecommon_freedev
 ****************************************************************************/

void pipecommon_freedev(FAR struct pipe_dev_s *dev)
{
   sem_destroy(&dev->d_bfsem);
   sem_destroy(&dev->d_rdsem);
   sem_destroy(&dev->d_wrsem);
   free(dev);
}

/****************************************************************************
 * Name: pipecommon_open
 ****************************************************************************/

int pipecommon_open(FAR struct file *filep)
{
  struct inode      *inode = filep->f_inode;
  struct pipe_dev_s *dev   = inode->i_private;
  int                sval;
 
  /* Some sanity checking */
#if CONFIG_DEBUG
  if (!dev)
    {
       return -EBADF;
    }
#endif
  /* Make sure that we have exclusive access to the device structure */

  if (sem_wait(&dev->d_bfsem) == 0)
    {
      /* If this the first reference on the device, then allocate the buffer */

      if (dev->d_refs == 0)
        {
          dev->d_buffer = (ubyte*)malloc(CONFIG_DEV_PIPE_SIZE);
          if (!dev->d_buffer)
            {
              (void)sem_post(&dev->d_bfsem);
              return -ENOMEM;
            }
        }

      /* Increment the reference count on the pipe instance */

      dev->d_refs++;

      /* If opened for writing, increment the count of writers on on the pipe instance */

      if ((filep->f_oflags & O_WROK) != 0)
        {
          dev->d_nwriters++;

          /* If this this is the first writer, then the read semaphore indicates the
           * number of readers waiting for the first writer.  Wake them all up.
           */
          if (dev->d_nwriters == 1)
            {
              while (sem_getvalue(&dev->d_rdsem, &sval) == 0 && sval < 0)
                {
                  sem_post(&dev->d_rdsem);
                }
            }
        }

      /* If opened for read-only, then wait for at least one writer on the pipe */

      sched_lock();
      (void)sem_post(&dev->d_bfsem);
      if ((filep->f_oflags & O_RDWR) == O_RDONLY && dev->d_nwriters < 1)
        {
          /* NOTE: d_rdsem is normally used when the read logic waits for more
           * data to be written.  But until the first writer has opened the
           * pipe, the meaning is different: it is used prevent O_RDONLY open
           * calls from returning until there is at least one writer on the pipe.
           * This is required both by spec and also because it prevents
           * subsequent read() calls from returning end-of-file because there is
           * no writer on the pipe.
           */

          pipecommon_semtake(&dev->d_rdsem);
        }
      sched_unlock();
      return OK;
  }
  return ERROR;
}

/****************************************************************************
 * Name: pipecommon_close
 ****************************************************************************/

int pipecommon_close(FAR struct file *filep)
{
  struct inode      *inode = filep->f_inode;
  struct pipe_dev_s *dev   = inode->i_private;
  int                sval;

  /* Some sanity checking */
#if CONFIG_DEBUG
  if (!dev)
    {
       return -EBADF;
    }
#endif

  /* Make sure that we have exclusive access to the device structure.
   * NOTE: close() is supposed to return EINTR if interrupted, however
   * I've never seen anyone check that.
   */

  pipecommon_semtake(&dev->d_bfsem);

  /* Check if the decremented reference count would go to zero */

  if (dev->d_refs > 1)
    {
       /* No.. then just decrement the reference count */

       dev->d_refs--;

      /* If opened for writing, decrement the count of writers on on the pipe instance */

      if ((filep->f_oflags & O_WROK) != 0)
        {
          /* If there are no longer any writers on the pipe, then notify all of the
           * waiting readers that they must return end-of-file.
           */

          if (--dev->d_nwriters <= 0)
            {
              while (sem_getvalue(&dev->d_rdsem, &sval) == 0 && sval < 0)
                {
                  sem_post(&dev->d_rdsem);
                }
            }
        }
    }
  else
    {
      /* Yes... deallocate the buffer */

      free(dev->d_buffer);
      dev->d_buffer = NULL;

      /* And reset all counts and indices */
 
      dev->d_wrndx    = 0;
      dev->d_rdndx    = 0;
      dev->d_refs     = 0;
      dev->d_nwriters = 0;
   }

  sem_post(&dev->d_bfsem);
  return OK;
}

/****************************************************************************
 * Name: pipecommon_read
 ****************************************************************************/

ssize_t pipecommon_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  struct inode      *inode  = filep->f_inode;
  struct pipe_dev_s *dev    = inode->i_private;
  ssize_t            nread  = 0;
  int                sval;
  int                ret;

  /* Some sanity checking */
#if CONFIG_DEBUG
  if (!dev)
    {
      return -ENODEV;
    }
#endif

  /* Make sure that we have exclusive access to the device structure */

  if (sem_wait(&dev->d_bfsem) < 0)
    {
      return ERROR;
    }

  /* If the pipe is empty, then wait for something to be written to it */

  while (dev->d_wrndx == dev->d_rdndx)
    {
      /* If O_NONBLOCK was set, then return EGAIN */

      if (filep->f_oflags & O_NONBLOCK)
        {
          sem_post(&dev->d_bfsem);
          return -EAGAIN;
        }

      /* If there are no writers on the pipe, then return end of file */

      if (dev->d_nwriters <= 0)
        {
          sem_post(&dev->d_bfsem);
          return 0;
        }

      /* Otherwise, wait for something to be written to the pipe */

      sched_lock();
      sem_post(&dev->d_bfsem);
      ret = sem_wait(&dev->d_rdsem);
      sched_unlock();
      if (ret < 0  || sem_wait(&dev->d_bfsem) < 0) 
        {
          return ERROR;
        }
    }

  /* Then return whatever is available in the pipe (which is at least one byte) */

  nread = 0;
  while (nread < len && dev->d_wrndx != dev->d_rdndx)
    {
      *buffer++ = dev->d_buffer[dev->d_rdndx];
      if (++dev->d_rdndx >= CONFIG_DEV_PIPE_SIZE)
        {
          dev->d_rdndx = 0; 
        }
      nread++;
    }

  /* Notify all waiting writers that bytes have been removed from the buffer */

  while (sem_getvalue(&dev->d_wrsem, &sval) == 0 && sval < 0)
    {
      sem_post(&dev->d_wrsem);
    }

  sem_post(&dev->d_bfsem);
  return nread;	    
}

/****************************************************************************
 * Name: pipecommon_write
 ****************************************************************************/

ssize_t pipecommon_write(FAR struct file *filep, FAR const char *buffer, size_t len)
{
  struct inode      *inode    = filep->f_inode;
  struct pipe_dev_s *dev      = inode->i_private;
  ssize_t            nwritten = 0;
  ssize_t            last;
  int                nxtwrndx;
  int                sval;

  /* Some sanity checking */
#if CONFIG_DEBUG
  if (!dev)
    {
      return -ENODEV;
    }
#endif

  /* Make sure that we have exclusive access to the device structure */

  if (sem_wait(&dev->d_bfsem) < 0)
    {
      return ERROR;
    }

  /* Loop until all of the bytes have been written */

  last = 0;
  for (;;)
    {
      /* Calculate the write index AFTER the next byte is written */

      nxtwrndx = dev->d_wrndx + 1;
      if (nxtwrndx >= CONFIG_DEV_PIPE_SIZE)
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

          if (++nwritten >= len)
            {
              /* Yes.. Notify all of the waiting readers that more data is available */

              while (sem_getvalue(&dev->d_rdsem, &sval) == 0 && sval < 0)
                {
                  sem_post(&dev->d_rdsem);
                }

              /* Return the number of bytes written */

              sem_post(&dev->d_bfsem);
              return len;
            }
        }
      else
        {
          /* There is not enough room for the next byte.  Was anything written in this pass? */

          if (last < nwritten)
            {
              /* Yes.. Notify all of the waiting readers that more data is available */

              while (sem_getvalue(&dev->d_rdsem, &sval) == 0 && sval < 0)
                {
                  sem_post(&dev->d_rdsem);
                }
            }
          last = nwritten;

          /* If O_NONBLOCK was set, then return partial bytes written or EGAIN */

          if (filep->f_oflags & O_NONBLOCK)
            {
              if (nwritten == 0)
                {
                  nwritten = -EAGAIN;
                }
              sem_post(&dev->d_bfsem);
              return nwritten;
            }

          /* There is more to be written.. wait for data to be removed from the pipe */

          sched_lock();
          sem_post(&dev->d_bfsem);
          pipecommon_semtake(&dev->d_wrsem);
          sched_unlock();
          pipecommon_semtake(&dev->d_bfsem);
        }
    }
}

#endif /* CONFIG_DEV_PIPE_SIZE > 0 */
