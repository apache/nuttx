/****************************************************************************
 * drivers/sent/sent.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <nuttx/poll.h>
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>
#include <nuttx/semaphore.h>
#include <nuttx/sent/sent.h>
#include <nuttx/spinlock.h>
#include <nuttx/circbuf.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct sent_upperhalf_s
{
  FAR struct sent_lowerhalf_s *lower;          /* The handle of lower half driver */
  uint8_t                      crefs;          /* The number of times the device has been opened */
  spinlock_t                   spinlock;       /* Manages exclusive access to file operations */
  struct circbuf_s             buffer;         /* The circular buffer of sent fast data */
  sem_t                        buffersem;      /* Semaphore to signal data availability */
  FAR struct pollfd_s         *pollfd;         /* Polling file descriptors */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     sent_open(FAR struct file *filep);
static int     sent_close(FAR struct file *filep);
static ssize_t sent_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen);
static int     sent_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg);
static int     sent_poll(FAR struct file *filep, FAR struct pollfd_s *fds,
                         bool setup);
static void    sent_push_frame(FAR void *priv,
                               FAR const struct sent_frame_s *frame);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_sent_fops =
{
  sent_open,  /* open */
  sent_close, /* close */
  sent_read,  /* read  */
  NULL,       /* write */
  NULL,       /* seek  */
  sent_ioctl, /* ioctl */
  NULL,       /* mmap */
  NULL,       /* truncate */
  sent_poll,  /* poll  */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int sent_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sent_upperhalf_s *upper = inode->i_private;
  FAR struct sent_lowerhalf_s *lower = upper->lower;
  irqstate_t flags;
  int ret = OK;

  flags = spin_lock_irqsave(&upper->spinlock);

  /* Increment the reference count */

  if (upper->crefs == 0)
    {
      if (lower->ops->activate)
        {
          ret = lower->ops->activate(lower, sent_push_frame, upper);
          if (ret < 0)
            {
              spin_unlock_irqrestore(&upper->spinlock, flags);
              return ret;
            }
        }
    }

  upper->crefs++;

  spin_unlock_irqrestore(&upper->spinlock, flags);
  return ret;
}

static int sent_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sent_upperhalf_s *upper = inode->i_private;
  FAR struct sent_lowerhalf_s *lower = upper->lower;
  irqstate_t flags;
  int ret = OK;

  flags = spin_lock_irqsave(&upper->spinlock);

  /* Decrement the reference count */

  DEBUGASSERT(upper->crefs > 0);
  upper->crefs--;
  if (upper->crefs == 0)
    {
      if (lower->ops->activate)
        {
          ret = lower->ops->activate(lower, NULL, NULL);
        }
    }

  spin_unlock_irqrestore(&upper->spinlock, flags);
  return ret;
}

static void sent_push_frame(FAR void *priv,
                            FAR const struct sent_frame_s *frame)
{
  FAR struct sent_upperhalf_s *upper = priv;
  ssize_t ret;
  int semcount;
  irqstate_t flags;

  flags = spin_lock_irqsave_nopreempt(&upper->spinlock);
  ret = circbuf_overwrite(&upper->buffer, frame, sizeof(*frame));
  if (ret < 0)
    {
      spin_unlock_irqrestore_nopreempt(&upper->spinlock, flags);
      return;
    }

  if (upper->pollfd && (upper->pollfd->events & POLLIN))
    {
      poll_notify(&upper->pollfd, 1, POLLIN);
    }

  nxsem_get_value(&upper->buffersem, &semcount);
  if (semcount < 1)
    {
      nxsem_post(&upper->buffersem);
    }

  spin_unlock_irqrestore_nopreempt(&upper->spinlock, flags);
}

static ssize_t sent_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sent_upperhalf_s *upper = inode->i_private;
  irqstate_t flags;
  ssize_t ret;

  if (buffer == NULL || buflen == 0)
    {
      return -EINVAL;
    }

  /* Read integral count of sent_frame_s entries */

  buflen /= sizeof(struct sent_frame_s);
  if (buflen == 0)
    {
      return -EINVAL;
    }

  flags = spin_lock_irqsave(&upper->spinlock);
  while (circbuf_is_empty(&upper->buffer))
    {
      if (filep->f_oflags & O_NONBLOCK)
        {
          spin_unlock_irqrestore(&upper->spinlock, flags);
          return -ENODATA;
        }

      spin_unlock_irqrestore(&upper->spinlock, flags);
      nxsem_wait_uninterruptible(&upper->buffersem);
      flags = spin_lock_irqsave(&upper->spinlock);
    }

  ret = circbuf_read(&upper->buffer, buffer,
                     buflen * sizeof(struct sent_frame_s));

  spin_unlock_irqrestore(&upper->spinlock, flags);
  return ret;
}

static int sent_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sent_upperhalf_s *upper = inode->i_private;
  FAR struct sent_lowerhalf_s *lower = upper->lower;
  int ret = -ENOTTY;

  switch (cmd)
    {
      case SENTIOC_TRIGGER_SPC:
        if (lower->ops->trigger_spc)
          {
            ret = lower->ops->trigger_spc(lower);
          }

        break;

      default:
        if (lower->ops->control)
          {
            ret = lower->ops->control(lower, cmd, arg);
          }

        break;
    }

  return ret;
}

static int sent_poll(FAR struct file *filep, FAR struct pollfd_s *fds,
                     bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sent_upperhalf_s *upper = inode->i_private;
  irqstate_t flags;
  int ret = 0;

  flags = spin_lock_irqsave_nopreempt(&upper->spinlock);

  if (setup)
    {
      if (upper->pollfd != NULL)
        {
          ret = -EBUSY;
          goto errout;
        }

      upper->pollfd = fds;
      fds->revents = 0;
      if (!circbuf_is_empty(&upper->buffer))
        {
          if (fds->events & POLLIN)
            {
              poll_notify(&upper->pollfd, 1, POLLIN);
            }
        }
    }
  else
    {
      if (upper->pollfd == fds)
        {
          upper->pollfd = NULL;
        }
    }

errout:
  spin_unlock_irqrestore_nopreempt(&upper->spinlock, flags);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sent_register
 *
 * Description:
 *   This function is used to register a SENT device driver instance.
 *   It creates and initializes the upper-half driver structure,
 *   links it with the provided lower-half driver, and registers the
 *   device node under the /dev directory (e.g., /dev/sent0_channel0).
 *
 *   The function allocates and initializes the necessary resources,
 *   including spin locks and a circular buffer for data reception.
 *   If registration succeeds, the SENT device can then be accessed
 *   via standard file  operations defined in g_sent_fops.
 *
 * Input Parameters:
 *   path    - The device path to register (e.g., "/dev/sent0_channel0").
 *   lower   - Pointer to the lower-half driver structure that provides the
 *             hardware-specific implementations and callbacks for the SENT
 *             peripheral.
 *
 * Returned Value:
 *   OK if the driver was successfully registered; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int sent_register(const char *path, FAR struct sent_lowerhalf_s *lower)
{
  FAR struct sent_upperhalf_s *upper;
  int ret;

  DEBUGASSERT(lower != NULL);

  upper = kmm_zalloc(sizeof(struct sent_upperhalf_s));
  if (!upper)
    {
      return -ENOMEM;
    }

  ret = circbuf_init(&upper->buffer, NULL, CONFIG_SENT_RING_BUFFER_SIZE *
                     sizeof(struct sent_frame_s));
  if (ret < 0)
    {
      goto errout;
    }

  spin_lock_init(&upper->spinlock);
  nxsem_init(&upper->buffersem, 0, 0);
  upper->lower = lower;
  ret = register_driver(path, &g_sent_fops, 0666, upper);
  if (ret < 0)
    {
      nxsem_destroy(&upper->buffersem);
      circbuf_uninit(&upper->buffer);
      goto errout;
    }

  return OK;

errout:
  kmm_free(upper);
  return ret;
}

