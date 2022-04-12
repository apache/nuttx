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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/input/touchscreen.h>
#include <nuttx/kmalloc.h>
#include <nuttx/list.h>
#include <nuttx/mm/circbuf.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct touch_openpriv_s
{
  struct circbuf_s   circbuf; /* Store touch point data in circle buffer */
  struct list_node   node;    /* Opened file buffer linked list node */
  FAR struct pollfd *fds;     /* Polling structure of waiting thread */
  sem_t              waitsem; /* Used to wait for the availability of data */
  sem_t              locksem; /* Manages exclusive access to this structure */
};

/* This structure is for touchscreen upper half driver */

struct touch_upperhalf_s
{
  uint8_t          nums;               /* Number of touch point structure */
  sem_t            exclsem;            /* Manages exclusive access to this structure */
  struct list_node head;               /* Opened file buffer chain header node */
  FAR struct touch_lowerhalf_s *lower; /* A pointer of lower half instance */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void    touch_notify(FAR struct touch_openpriv_s *openpriv,
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

static void touch_notify(FAR struct touch_openpriv_s *openpriv,
                         pollevent_t eventset)
{
  FAR struct pollfd *fd = openpriv->fds;

  fd = openpriv->fds;
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

/****************************************************************************
 * Name: touch_open
 ****************************************************************************/

static int touch_open(FAR struct file *filep)
{
  FAR struct touch_openpriv_s  *openpriv;
  FAR struct inode             *inode = filep->f_inode;
  FAR struct touch_upperhalf_s *upper = inode->i_private;
  FAR struct touch_lowerhalf_s *lower = upper->lower;
  int ret;

  openpriv = kmm_zalloc(sizeof(struct touch_openpriv_s));
  if (openpriv == NULL)
    {
      return -ENOMEM;
    }

  ret = circbuf_init(&openpriv->circbuf, NULL,
                     upper->nums * SIZEOF_TOUCH_SAMPLE_S(lower->maxpoint));
  if (ret < 0)
    {
      kmm_free(openpriv);
      return ret;
    }

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      circbuf_uninit(&openpriv->circbuf);
      kmm_free(openpriv);
      return ret;
    }

  nxsem_init(&openpriv->waitsem, 0, 0);
  nxsem_init(&openpriv->locksem, 0, 1);
  nxsem_set_protocol(&openpriv->waitsem, SEM_PRIO_NONE);
  list_add_tail(&upper->head, &openpriv->node);

  /* Save the buffer node pointer so that it can be used directly
   * in the read operation.
   */

  filep->f_priv = openpriv;
  nxsem_post(&upper->exclsem);
  return ret;
}

/****************************************************************************
 * Name: touch_close
 ****************************************************************************/

static int touch_close(FAR struct file *filep)
{
  FAR struct touch_openpriv_s  *openpriv = filep->f_priv;
  FAR struct inode             *inode    = filep->f_inode;
  FAR struct touch_upperhalf_s *upper    = inode->i_private;
  int ret;

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  list_delete(&openpriv->node);
  circbuf_uninit(&openpriv->circbuf);
  nxsem_destroy(&openpriv->waitsem);
  nxsem_destroy(&openpriv->locksem);
  kmm_free(openpriv);

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
  FAR struct touch_openpriv_s *openpriv = filep->f_priv;
  int ret;

  if (!buffer || !len)
    {
      return -EINVAL;
    }

  ret = nxsem_wait(&openpriv->locksem);
  if (ret < 0)
    {
      return ret;
    }

  while (circbuf_is_empty(&openpriv->circbuf))
    {
      if (filep->f_oflags & O_NONBLOCK)
        {
          ret = -EAGAIN;
          goto out;
        }
      else
        {
          nxsem_post(&openpriv->locksem);
          ret = nxsem_wait_uninterruptible(&openpriv->waitsem);
          if (ret < 0)
            {
              return ret;
            }

          ret = nxsem_wait(&openpriv->locksem);
          if (ret < 0)
            {
              return ret;
            }
        }
    }

  ret = circbuf_read(&openpriv->circbuf, buffer, len);

out:
  nxsem_post(&openpriv->locksem);
  return ret;
}

static int touch_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode             *inode = filep->f_inode;
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

static int touch_poll(FAR struct file *filep, struct pollfd *fds, bool setup)
{
  FAR struct touch_openpriv_s *openpriv = filep->f_priv;
  pollevent_t eventset = 0;
  int ret;

  ret = nxsem_wait(&openpriv->locksem);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      if (openpriv->fds == NULL)
        {
          openpriv->fds = fds;
          fds->priv     = &openpriv->fds;
        }
      else
        {
          ret = -EBUSY;
          goto errout;
        }

      if (!circbuf_is_empty(&openpriv->circbuf))
        {
          eventset |= (fds->events & POLLIN);
        }

      if (eventset)
        {
          touch_notify(openpriv, POLLIN);
        }
    }
  else if (fds->priv)
    {
      openpriv->fds = NULL;
      fds->priv   = NULL;
    }

errout:
  nxsem_post(&openpriv->locksem);
  return ret;
}

/****************************************************************************
 * Public Function
 ****************************************************************************/

void touch_event(FAR void *priv, FAR const struct touch_sample_s *sample)
{
  FAR struct touch_upperhalf_s *upper = priv;
  FAR struct touch_openpriv_s  *openpriv;
  int semcount;

  if (nxsem_wait(&upper->exclsem) < 0)
    {
      return;
    }

  list_for_every_entry(&upper->head, openpriv, struct touch_openpriv_s, node)
    {
      circbuf_overwrite(&openpriv->circbuf, sample,
                        SIZEOF_TOUCH_SAMPLE_S(sample->npoints));

      nxsem_get_value(&openpriv->waitsem, &semcount);
      if (semcount < 1)
        {
          nxsem_post(&openpriv->waitsem);
        }

      touch_notify(openpriv, POLLIN);
    }

  nxsem_post(&upper->exclsem);
}

int touch_register(FAR struct touch_lowerhalf_s *lower,
                   FAR const char *path, uint8_t nums)
{
  FAR struct touch_upperhalf_s *upper;
  int ret;

  iinfo("Registering %s\n", path);

  if (lower == NULL || nums == 0)
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

  lower->priv  = upper;
  upper->lower = lower;
  upper->nums  = nums;
  list_initialize(&upper->head);
  nxsem_init(&upper->exclsem, 0, 1);

  ret = register_driver(path, &g_touch_fops, 0666, upper);
  if (ret < 0)
    {
      nxsem_destroy(&upper->exclsem);
      kmm_free(upper);
      return ret;
    }

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
  kmm_free(upper);
}
