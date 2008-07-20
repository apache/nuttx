/****************************************************************************
 * drivers/fifo.c
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
#include <errno.h>
#include <assert.h>
#include <nuttx/fs.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#ifndef CONFIG_DEV_FIFO_SIZE
#  define CONFIG_DEV_FIFO_SIZE 1024
#endif

/* Maximum number of open's supported on FIFO */

#define CONFIG_DEV_FIFO_MAXUSER 255

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Make the FIFO index as small as possible for the configured FIFO size */
 
#if CONFIG_DEV_FIFO_SIZE > 65535
typedef uint32 fifo_ndx_t;  /* 32-bit index */
#elif CONFIG_DEV_FIFO_SIZE > 255
typedef uint16 fifo_ndx_t;  /* 16-bit index */
#else
typedef ubyte fifo_ndx_t;   /*  8-bit index */
#endif

struct fifo_dev_s
{
  sem_t      d_bfsem;       /* Used to serialize access to d_buffer and indices */
  sem_t      d_rdsem;       /* Empty buffer - Reader waits for data write */
  sem_t      d_wrsem;       /* Full buffer - Writer waits for data read */
  fifo_ndx_t d_wrndx;       /* Index in d_buffer to save next byte written */
  fifo_ndx_t d_rdndx;      /* Index in d_buffer to return the next byte read */
  ubyte      d_refs;        /* References counts on FIFO (limited to 255) */
  ubyte      d_nreaders;    /* Number of readers waiting for write data to empty buffer */
  ubyte      d_nwriters;    /* Number of writers wiating for data read out of full buffer */
  ubyte      d_buffer[CONFIG_DEV_FIFO_SIZE];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline FAR struct fifo_dev_s *fifo_allocdev(void);
static inline void fifo_freedev(FAR struct fifo_dev_s *dev);
static void    fifo_semtake(sem_t *sem);

static int     fifo_open(FAR struct file *filep);
static int     fifo_close(FAR struct file *filep);
static ssize_t fifo_read(FAR struct file *, FAR char *, size_t);
static ssize_t fifo_write(FAR struct file *, FAR const char *, size_t);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct file_operations fifo_fops =
{
  fifo_open,  /* open */
  fifo_close, /* close */
  fifo_read,  /* read */
  fifo_write, /* write */
  0,          /* seek */
  0           /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fifo_allocdev
 ****************************************************************************/
static inline FAR struct fifo_dev_s *fifo_allocdev(void)
{
 struct fifo_dev_s *dev;

  /* Allocate a private structure to manage the FIFO */

  dev = (struct fifo_dev_s *)malloc(sizeof(struct fifo_dev_s));
  if (dev)
    {
      /* Initialize the private structure */

      sem_init(&dev->d_bfsem, 0, 1);
      sem_init(&dev->d_rdsem, 0, 0);
      sem_init(&dev->d_wrsem, 0, 0);
      dev->d_wrndx    = 0;
      dev->d_rdndx   = 0;
      dev->d_refs     = 0;
      dev->d_nreaders = 0;
      dev->d_nwriters = 0;
    }
  return dev;
}

/****************************************************************************
 * Name: fifo_freedev
 ****************************************************************************/
static inline void fifo_freedev(FAR struct fifo_dev_s *dev)
{
   sem_destroy(&dev->d_bfsem);
   sem_destroy(&dev->d_rdsem);
   sem_destroy(&dev->d_wrsem);
   free(dev);
}

/****************************************************************************
 * Name: fifo_semtake
 ****************************************************************************/
static void fifo_semtake(sem_t *sem)
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
 * Name: fifo_open
 ****************************************************************************/
static int fifo_open(FAR struct file *filep)
{
  struct inode      *inode = filep->f_inode;
  struct fifo_dev_s *dev   = inode->i_private;
 
  /* The device structure will be allocated the first time that the FIFO is
   * opened
   */

  if (!dev)
    {
      /* Allocate and initialize a new device structure instance */

      dev = fifo_allocdev();
      if (!dev)
        {
          return -ENOMEM;
        }

      /* Set the private data on the inode to this instance */

      inode->i_private = dev;
    }

  /* Make sure that we have exclusive access to the device structure */

  if (sem_wait(&dev->d_bfsem) == 0)
    {
      /* Increment the reference count on the fifo instance */

      dev->d_refs++;
      (void)sem_post(&dev->d_bfsem);
      return OK;
  }
  return ERROR;
}

/****************************************************************************
 * Name: fifo_close
 ****************************************************************************/
static int fifo_close(FAR struct file *filep)
{
  struct inode      *inode = filep->f_inode;
  struct fifo_dev_s *dev   = inode->i_private;

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

  fifo_semtake(&dev->d_bfsem);

  /* Check if the decremented reference count would be zero */

  if (dev->d_refs > 1)
    {
       /* No.. then just decrement the reference count */

       dev->d_refs--;
       sem_post(&dev->d_bfsem);
    }
  else
    {
       /* Then nothing else can be holding the semaphore, so it is save to */

       inode->i_private = NULL;
       sem_post(&dev->d_bfsem);

       /* Then free the fifo structure instance */

       fifo_freedev(dev);
    }
  return OK;
}

/****************************************************************************
 * Name: fifo_read
 ****************************************************************************/
static ssize_t fifo_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  struct inode      *inode  = filep->f_inode;
  struct fifo_dev_s *dev    = inode->i_private;
  ssize_t            nread  = 0;
  ssize_t            nrqstd = len;
  int                ret;

  /* Some sanity checking */
#if CONFIG_DEBUG
  if (dev)
    {
      return -ENODEV;
    }
#endif

  /* Make sure that we have exclusive access to the device structure */

  if (sem_wait(&dev->d_bfsem) < 0)
    {
      return ERROR;
    }

  /* Loop until all of the bytes have been read */

  for (;;)
    {
      ssize_t nbytes;

      /* Is there data available at the end of the buffer?  When the write
       * index is smaller than the read index, then all of the data from the
       * read index to the end of the buffer is available for reading.
       */

      if (dev->d_rdndx > dev->d_wrndx)
        {
          /* Yes.. How many bytes are available at the end of the circular buffer */

          nbytes = CONFIG_DEV_FIFO_SIZE - dev->d_rdndx;
          if (nbytes > 0)
            {
              /* Read bytes from the end of the buffer.  Are there more than we
               * need?
               */

              if (nbytes > nrqstd)
                {
                  nbytes = nrqstd;
                }

              /* Copy the correct number of bytes */

              memcpy(&buffer[nread], &dev->d_buffer[dev->d_rdndx], nbytes);

              /* Then update all indices and counts */

              nread              += nbytes;
              nrqstd             -= nbytes;
              dev->d_rdndx += nbytes;
              if (dev->d_rdndx >= CONFIG_DEV_FIFO_SIZE)
                {
                  dev->d_rdndx = 0;
                }
           }
       }

      /* We have read some or all of the data at the end of the buffer. Do we still need more? */
 
      if (nrqstd > 0)
        {
          /* Yes... then we now that we have consumed all of the bytes at the end of the
           * buffer. How many bytes are available at the end of the circular buffer>
           */

          nbytes = dev->d_wrndx - dev->d_rdndx;
          if (nbytes > 0)
            {
              /* Read bytes from the beginning of the buffer.  Are there more than we
               * need?
               */

              if (nbytes > nrqstd)
                {
                  nbytes = nrqstd;
                }

              /* Copy the correct number of bytes */

              memcpy(&buffer[nread], &dev->d_buffer[dev->d_rdndx], nbytes);

              /* Then update all indices and counts */

              nread              += nbytes;
              nrqstd             -= nbytes;
              dev->d_rdndx  += nbytes;
            }
        }

      /* Was anything read? */

      if (nread > 0)
        {
          /* Yes.. Notify the waiting writers that space is again available */

          if (dev->d_nwriters > 0)
            {
              sem_post(&dev->d_wrsem);
            }

          /* Return the number of bytes read */

          sem_post(&dev->d_bfsem);
          return nread;
        }
      else
        {
          /* No.. wait for data to be added to the FIFO */

          dev->d_nreaders++;
          sched_lock();
          sem_post(&dev->d_bfsem);
          ret = sem_wait(&dev->d_rdsem);
          sched_unlock();
          fifo_semtake(&dev->d_bfsem);
          dev->d_nreaders--;

          if (ret != 0)
            {
              return ERROR;
            }
        }
    }
}

/****************************************************************************
 * Name: fifo_write
 ****************************************************************************/
static ssize_t fifo_write(FAR struct file *filep, FAR const char *buffer, size_t len)
{
  struct inode      *inode    = filep->f_inode;
  struct fifo_dev_s *dev      = inode->i_private;
  const char        *src;
  ssize_t            nwritten = 0;
  ssize_t            ntowrite = len;
 
  /* Some sanity checking */
#if CONFIG_DEBUG
  if (dev)
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

  src = buffer;
  for (;;)
    {
      ssize_t nbytes;

      /* Is there space available at the end of the buffer?  When the write
       * index is greater than the read index, then all of the data from the
       * write index to the end of the buffer is available for writing.
       */

      if (dev->d_wrndx >= dev->d_rdndx)
        {
          /* How many bytes are available at the end of the circular buffer */

          nbytes = CONFIG_DEV_FIFO_SIZE - dev->d_wrndx;
          if (nbytes > 0)
            {
              /* Write bytes to the end of the buffer.  Is there more space than we
               * need?
               */

              if (nbytes > ntowrite)
                {
                  nbytes = ntowrite;
                }

              /* Copy the correct number of bytes */

              memcpy(&dev->d_buffer[dev->d_wrndx], &src[nwritten], nbytes);

              /* Then update all indices and counts */

              nwritten          += nbytes;
              ntowrite          -= nbytes;
              dev->d_wrndx += nbytes;
              if (dev->d_wrndx >= CONFIG_DEV_FIFO_SIZE)
                {
                  dev->d_wrndx = 0;
                }
           }
       }

      /* Do we still need to write more data? */
 
      if (ntowrite > 0)
        {
          /* Yes... How many bytes are available at the end of the circular buffer */

          nbytes =  dev->d_rdndx - dev->d_wrndx - 1;
          if (nbytes > 0)
            {
              /* Write bytes to the beginning of the buffer.  Is there more space than we
               * need?
               */

              if (nbytes > ntowrite)
                {
                  nbytes = ntowrite;
                }

              /* Copy the correct number of bytes */

              memcpy(&dev->d_buffer[dev->d_wrndx], &src[nwritten], nbytes);

              /* Then update all indices and counts */

              nwritten          += nbytes;
              ntowrite          -= nbytes;
              dev->d_wrndx += nbytes;
            }
        }

        /* Was anything written? */

        if (nwritten > 0)
          {
            /* Yes.. Notify the waiting reades that more data is available */

            if (dev->d_nreaders > 0)
              {
                sem_post(&dev->d_rdsem);
              }
          }

        /* Was everything written? */

        if (ntowrite <= 0  )
          {
 	    /* Return the number of bytes written */

            sem_post(&dev->d_bfsem);
            return len;
          }

        /* There is more to be writtend.. wait for data to be removed from the FIFO */

        dev->d_nwriters++;
        sched_lock();
        sem_post(&dev->d_bfsem);
        fifo_semtake(&dev->d_wrsem);
        sched_unlock();
        fifo_semtake(&dev->d_bfsem);
        dev->d_nwriters--;

        src = &src[nwritten];
        nwritten = 0;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mkfifo
 *
 * Description:
 *   mkfifo() makes a FIFO device driver file with name 'pathname.'  Unlike
 *   Linux, a NuttX FIFO is not a special file type but simply a device driver
 *   instance.  'mode' specifies the FIFO's permissions. 
 *
 *   Once the FIFO has been created by mkfifo(), any thread can open it for
 *   reading or writing, in the same way as an ordinary file.  NuttX FIFOs need
 *   not be open at both ends before input or output operations on it.
 *
 *   In NuttX pipes are built on top of FIFOs
 *
 * Inputs:
 *   pathname - The full path to the FIFO instance to attach to or to create
 *     (if not already created).
 *   mode - Ignored for now
 *
 * Return:
 *   0 is returned on success; otherwise, -1 is returned with errno set
 *   appropriately.
 *
 ****************************************************************************/
int mkfifo(FAR const char *pathname, mode_t mode)
{
  return register_driver(pathname, &fifo_fops, mode, NULL);
}
