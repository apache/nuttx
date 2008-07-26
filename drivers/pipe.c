/****************************************************************************
 * drivers/pipe.c
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
#include <nuttx/fs.h>
#include <stdio.h>
#include <unistd.h>
#include <semaphore.h>
#include <fcntl.h>
#include <errno.h>

#include "pipe-common.h"

#if CONFIG_DEV_PIPE_SIZE > 0

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define MAX_PIPES 32

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int pipe_close(FAR struct file *filep);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct file_operations pipe_fops =
{
  pipecommon_open,   /* open */
  pipe_close,        /* close */
  pipecommon_read,   /* read */
  pipecommon_write,  /* write */
  0,                 /* seek */
  0                  /* ioctl */
};

static sem_t  g_pipesem = { 1 };
static uint32 g_pipeset = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pipe_allocate
 ****************************************************************************/
static inline int pipe_allocate(void)
{
  int pipeno;
  int ret = sem_wait(&g_pipesem);
  if (ret >= 0)
    {
      ret = -ENFILE;
      for (pipeno = 0; pipeno < MAX_PIPES; pipeno++)
        {
          if ((g_pipeset & (1 << pipeno)) == 0)
            {
              g_pipeset |= (1 << pipeno);
              ret = pipeno;
              break;
            }
        }
      (void)sem_post(&g_pipesem);
    }
  return ret;
}

/****************************************************************************
 * Name: pipe_free
 ****************************************************************************/
static inline void pipe_free(int pipeno)
{
  int ret = sem_wait(&g_pipesem);
  if (ret == 0)
    {
      g_pipeset &= ~(1 << pipeno);
      (void)sem_post(&g_pipesem);
    }
}

/****************************************************************************
 * Name: pipe_close
 ****************************************************************************/
static int pipe_close(FAR struct file *filep)
{
  struct inode      *inode = filep->f_inode;
  struct pipe_dev_s *dev   = inode->i_private;
  ubyte              pipeno;
  int                ret;

  /* Some sanity checking */
#if CONFIG_DEBUG
  if (!dev)
    {
       return -EBADF;
    }
#endif
  pipeno = dev->s.d_pipeno;

  /* Perform common close operations */

  ret =  pipecommon_close(filep);
  if (ret == 0 && !inode->i_private)
    {
      char devname[16];
      sprintf(devname, "/dev/pipe%d", pipeno);
      unlink(devname);
      pipe_free(pipeno);
    }
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pipe
 *
 * Description:
 *   pipe() creates a pair of file descriptors, pointing to a pipe inode, and
 *   places them in the array pointed to by 'filedes'. filedes[0] is for reading,
 *   filedes[1] is for writing. 
 *
 * Inputs:
 *   filedes[2] - The user provided array in which to catch the pipe file
 *   descriptors
 *
 * Return:
 *   0 is returned on success; otherwise, -1 is returned with errno set
 *   appropriately.
 *
 ****************************************************************************/
int pipe(int filedes[2])
{
  struct pipe_dev_s *dev;
  char devname[16];
  int pipeno;
  int err;
  int ret;

  /* Allocate a minor number for the pipe device */

  pipeno = pipe_allocate();
  if (pipeno < 0)
    {
      err = -pipeno;
      goto errout;
    }

  /* Allocate and initialize a new device structure instance */

  dev = pipecommon_allocdev();
  if (!dev)
    {
      pipe_free(pipeno);
      err = ENOMEM;
      goto errout;
    }
  dev->s.d_pipeno = pipeno;

  /* Create a pathname to the pipe device */

  sprintf(devname, "/dev/pipe%d", pipeno);

  /* Register the pipe device */

  ret = register_driver(devname, &pipe_fops, 0666, (void*)dev);
  if (ret != 0)
    {
      err = -ret;
      goto errout_with_dev;
    }

  /* Get a write file descriptor */

  filedes[1] = open(devname, O_WRONLY);
  if (filedes[1] < 0)
    {
      err = -filedes[1];
      goto errout_with_driver;
    }

  /* Get a read file descriptor */

  filedes[0] = open(devname, O_RDONLY);
  if (filedes[0] < 0)
    {
      err = -filedes[0];
      goto errout_with_wrfd;
    }

  return OK;

errout_with_wrfd:
  close(filedes[1]);
errout_with_driver:
  unregister_driver(devname);
errout_with_dev:
  pipecommon_freedev(dev);
errout:
  errno = err;
  return ERROR;
}

#endif /* CONFIG_DEV_PIPE_SIZE > 0 */
