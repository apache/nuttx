/****************************************************************************
 * drivers/pipes/pipe.c
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

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>

#include "pipe_common.h"

#if CONFIG_DEV_PIPE_SIZE > 0

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

static const struct file_operations g_pipe_fops =
{
  pipecommon_open,     /* open */
  pipe_close,          /* close */
  pipecommon_read,     /* read */
  pipecommon_write,    /* write */
  NULL,                /* seek */
  pipecommon_ioctl,    /* ioctl */
  NULL,                /* mmap */
  NULL,                /* truncate */
  pipecommon_poll      /* poll */
};

static mutex_t g_pipelock = NXMUTEX_INITIALIZER;
static int     g_pipeno;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pipe_allocate
 ****************************************************************************/

static inline int pipe_allocate(void)
{
  int ret;

  ret = nxmutex_lock(&g_pipelock);
  if (ret < 0)
    {
      return ret;
    }

  ret = g_pipeno++;
  if (g_pipeno < 0)
    {
      g_pipeno = 0;
    }

  nxmutex_unlock(&g_pipelock);
  return ret;
}

/****************************************************************************
 * Name: pipe_close
 ****************************************************************************/

static int pipe_close(FAR struct file *filep)
{
  FAR struct inode *inode    = filep->f_inode;
  FAR struct pipe_dev_s *dev = inode->i_private;
  int ret;

  DEBUGASSERT(dev);

  /* Perform common close operations */

  ret = pipecommon_close(filep);
  if (ret == 0 && inode->i_crefs == 1)
    {
      /* Release the pipe when there are no further open references to it. */

      pipecommon_freedev(dev);
    }

  return ret;
}

/****************************************************************************
 * Name: pipe_register
 ****************************************************************************/

static int pipe_register(size_t bufsize, int flags,
                         FAR char *devname, size_t namesize)
{
  FAR struct pipe_dev_s *dev;
  int pipeno;
  int ret;

  /* Allocate a minor number for the pipe device */

  pipeno = pipe_allocate();
  if (pipeno < 0)
    {
      return pipeno;
    }

  /* Create a pathname to the pipe device */

  snprintf(devname, namesize, CONFIG_DEV_PIPE_VFS_PATH"/%d", pipeno);

  /* Allocate and initialize a new device structure instance */

  dev = pipecommon_allocdev(bufsize);
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  /* Register the pipe device */

  ret = register_driver(devname, &g_pipe_fops, 0666, (FAR void *)dev);
  if (ret != 0)
    {
      pipecommon_freedev(dev);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pipe2
 *
 * Description:
 *   pipe2() creates a pair of file descriptors, pointing to a pipe inode,
 *   and  places them in the array pointed to by 'fd'. fd[0] is for reading,
 *   fd[1] is for writing.
 *
 * Input Parameters:
 *   fd[2] - The user provided array in which to catch the pipe file
 *   descriptors
 *   flags - The file status flags.
 *
 * Returned Value:
 *   0 is returned on success; -1 (ERROR) is returned on a failure
 *   with the errno value set appropriately.
 *
 ****************************************************************************/

int file_pipe(FAR struct file *filep[2], size_t bufsize, int flags)
{
  char devname[32];
  int ret;

  /* Register a new pipe device */

  ret = pipe_register(bufsize, flags, devname, sizeof(devname));
  if (ret < 0)
    {
      return ret;
    }

  /* Get a write file descriptor */

  ret = file_open(filep[1], devname, O_WRONLY | flags);
  if (ret < 0)
    {
      goto errout_with_driver;
    }

  /* Get a read file descriptor */

  ret = file_open(filep[0], devname, O_RDONLY | flags);
  if (ret < 0)
    {
      goto errout_with_wrfd;
    }

  /* Remove the pipe name from file system */

  unregister_driver(devname);
  return OK;

errout_with_wrfd:
  file_close(filep[1]);

errout_with_driver:
  unregister_driver(devname);
  return ret;
}

int pipe2(int fd[2], int flags)
{
  char devname[32];
  int ret;

  /* Register a new pipe device */

  ret = pipe_register(CONFIG_DEV_PIPE_SIZE, flags, devname, sizeof(devname));
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  /* Get a write file descriptor */

  fd[1] = open(devname, O_WRONLY | flags);
  if (fd[1] < 0)
    {
      goto errout_with_driver;
    }

  /* Get a read file descriptor */

  fd[0] = open(devname, O_RDONLY | flags);
  if (fd[0] < 0)
    {
      goto errout_with_wrfd;
    }

  /* Remove the pipe name from file system */

  unregister_driver(devname);
  return OK;

errout_with_wrfd:
  nx_close(fd[1]);

errout_with_driver:
  unregister_driver(devname);
  return ERROR;
}

#endif /* CONFIG_DEV_PIPE_SIZE > 0 */
