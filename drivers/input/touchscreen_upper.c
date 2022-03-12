/****************************************************************************
 * drivers/input/touchscreen_upper.c
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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <poll.h>
#include <fcntl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mm/circbuf.h>
#include <nuttx/input/touchscreen.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure is for touchscreen upper half driver */

struct touch_upperhalf_s
{
  uint8_t                      open_count;   /* Number of times the device has been opened */
  uint32_t                     buff_nums;    /* Number of touch point structure */
  uint8_t                      nwaiters;     /* Number of threads waiting for touch point data */
  sem_t                        exclsem;      /* Manages exclusive access to this structure */
  sem_t                        buffersem;    /* Used to wait for the availability of data */
  struct circbuf_s             buffer;       /* Store touch point data in circle buffer */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

  FAR struct pollfd            *fds[CONFIG_INPUT_TOUCHSCREEN_NPOLLWAITERS];
  FAR struct touch_lowerhalf_s *lower;       /* A pointer of lower half instance */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void    touch_notify(FAR struct touch_upperhalf_s *upper,
                            pollevent_t eventset);
static int     touch_open(FAR struct file *filep);
static int     touch_close(FAR struct file *filep);
static ssize_t touch_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t touch_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int     touch_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);
static int     touch_poll(FAR struct file *filep, FAR struct pollfd *fds,
                          bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_touch_fops =
{
  touch_open,     /* open */
  touch_close,    /* close */
  touch_read,     /* read */
  touch_write,    /* write */
  NULL,           /* seek */
  touch_ioctl,    /* ioctl */
  touch_poll      /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL          /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void touch_notify(FAR struct touch_upperhalf_s *upper,
                         pollevent_t eventset)
{
  FAR struct pollfd *fd;
  int i;

  for (i = 0; i < CONFIG_INPUT_TOUCHSCREEN_NPOLLWAITERS; i++)
    {
      fd = upper->fds[i];
      if (fd)
        {
          fd->revents |= (fd->events & eventset);
          if (fd->revents != 0)
            {
              /* report event log */

              int semcount;
              nxsem_get_value(fd->sem, &semcount);
              if (semcount < 1)
                {
                   nxsem_post(fd->sem);
                }
            }
        }
    }
}

/****************************************************************************
 * Name: touch_open
 ****************************************************************************/

static int touch_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct touch_upperhalf_s *upper = inode->i_private;
  FAR struct touch_lowerhalf_s *lower = upper->lower;
  int ret;
  int tmp;

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  tmp = upper->open_count + 1;
  if (tmp == 0)
    {
      ret = -EMFILE;
      goto err_out;
    }
  else if (tmp == 1)
    {
      ret = circbuf_init(&upper->buffer, NULL, upper->buff_nums *
                         SIZEOF_TOUCH_SAMPLE_S(lower->maxpoint));
      if (ret < 0)
        {
          goto err_out;
        }
    }

  upper->open_count = tmp;

err_out:
  nxsem_post(&upper->exclsem);
  return ret;
}

/****************************************************************************
 * Name: touch_close
 ****************************************************************************/

static int touch_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct touch_upperhalf_s *upper = inode->i_private;
  int ret;

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  if (upper->open_count == 1)
    {
      upper->open_count--;
      circbuf_uninit(&upper->buffer);
    }

  nxsem_post(&upper->exclsem);
  return ret;
}

/****************************************************************************
 * Name: touch_write
 ****************************************************************************/

static ssize_t touch_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  FAR struct inode *inode             = filep->f_inode;
  FAR struct touch_upperhalf_s *upper = inode->i_private;
  FAR struct touch_lowerhalf_s *lower = upper->lower;

  if (!lower->write)
    {
      return -ENOSYS;
    }

  return lower->write(lower, buffer, buflen);
}

/****************************************************************************
 * Name: touch_read
 ****************************************************************************/

static ssize_t touch_read(FAR struct file *filep, FAR char *buffer,
                          size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct touch_upperhalf_s *upper = inode->i_private;
  int ret;

  if (!buffer || !len)
    {
      return -EINVAL;
    }

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  while (circbuf_is_empty(&upper->buffer))
    {
      if (filep->f_oflags & O_NONBLOCK)
        {
          ret = -EAGAIN;
          goto out;
        }
      else
        {
          nxsem_post(&upper->exclsem);
          ret = nxsem_wait_uninterruptible(&upper->buffersem);
          if (ret < 0)
            {
              return ret;
            }

          ret = nxsem_wait(&upper->exclsem);
          if (ret < 0)
            {
              return ret;
            }
        }
    }

  ret = circbuf_read(&upper->buffer, buffer, len);

out:
  nxsem_post(&upper->exclsem);
  return ret;
}

static int touch_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct touch_upperhalf_s *upper = inode->i_private;
  FAR struct touch_lowerhalf_s *lower = upper->lower;
  int ret;

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  if (lower->control)
    {
      ret = lower->control(lower, cmd, arg);
    }
  else
    {
      ret = -ENOTTY;
    }

  nxsem_post(&upper->exclsem);
  return ret;
}

static int touch_poll(FAR struct file *filep,
                      struct pollfd *fds, bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct touch_upperhalf_s *upper = inode->i_private;
  pollevent_t eventset = 0;
  int ret;
  int i;

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      for (i = 0; i < CONFIG_INPUT_TOUCHSCREEN_NPOLLWAITERS; i++)
        {
          if (NULL == upper->fds[i])
            {
              upper->fds[i] = fds;
              fds->priv = &upper->fds[i];
              break;
            }
        }

      if (i >= CONFIG_INPUT_TOUCHSCREEN_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret       = -EBUSY;
          goto errout;
        }

      if (!circbuf_is_empty(&upper->buffer))
        {
          eventset |= (fds->events & POLLIN);
        }

      if (eventset)
        {
          touch_notify(upper, eventset);
        }
    }
  else if (fds->priv)
    {
      for (i = 0; i < CONFIG_INPUT_TOUCHSCREEN_NPOLLWAITERS; i++)
        {
          if (fds == upper->fds[i])
            {
              upper->fds[i] = NULL;
              fds->priv = NULL;
              break;
            }
        }
    }

errout:
  nxsem_post(&upper->exclsem);
  return ret;
}

/****************************************************************************
 * Public Function
 ****************************************************************************/

void touch_event(FAR void *priv, FAR const struct touch_sample_s *sample)
{
  FAR struct touch_upperhalf_s *upper = priv;
  int semcount;

  if (nxsem_wait(&upper->exclsem) < 0)
    {
      return;
    }

  circbuf_overwrite(&upper->buffer, sample,
                    SIZEOF_TOUCH_SAMPLE_S(sample->npoints));
  touch_notify(upper, POLLIN);
  nxsem_get_value(&upper->buffersem, &semcount);
  if (semcount < 1)
    {
      nxsem_post(&upper->buffersem);
    }

  nxsem_post(&upper->exclsem);
}

int touch_register(FAR struct touch_lowerhalf_s *lower,
                   FAR const char *path, uint8_t buff_nums)
{
  FAR struct touch_upperhalf_s *upper;
  int ret;

  iinfo("Registering %s\n", path);

  if (lower == NULL || !buff_nums)
    {
      ierr("ERROR: invalid touchscreen device\n");
      return -EINVAL;
    }

  upper = kmm_zalloc(sizeof(struct touch_upperhalf_s));
  if (!upper)
    {
      ierr("ERROR: Failed to mem alloc!\n");
      return -ENOMEM;
    }

  upper->lower     = lower;
  upper->buff_nums = buff_nums;

  nxsem_init(&upper->exclsem, 0, 1);
  nxsem_init(&upper->buffersem, 0, 0);

  nxsem_set_protocol(&upper->buffersem, SEM_PRIO_NONE);

  lower->priv = upper;

  ret = register_driver(path, &g_touch_fops, 0666, upper);
  if (ret < 0)
    {
      goto err_out;
    }

  return ret;
err_out:
  nxsem_destroy(&upper->exclsem);
  nxsem_destroy(&upper->buffersem);
  kmm_free(upper);
  return ret;
}

void touch_unregister(FAR struct touch_lowerhalf_s *lower,
                      FAR const char *path)
{
  FAR struct touch_upperhalf_s *upper;

  DEBUGASSERT(lower != NULL);
  DEBUGASSERT(lower->priv != NULL);

  upper = lower->priv;

  iinfo("UnRegistering %s\n", path);
  unregister_driver(path);

  nxsem_destroy(&upper->exclsem);
  nxsem_destroy(&upper->buffersem);

  kmm_free(upper);
}
