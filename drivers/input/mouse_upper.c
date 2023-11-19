/****************************************************************************
 * drivers/input/mouse_upper.c
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

#include <debug.h>
#include <fcntl.h>
#include <poll.h>

#include <nuttx/input/mouse.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/list.h>
#include <nuttx/mm/circbuf.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mouse_openpriv_s
{
  struct circbuf_s   circbuf; /* Store mouse point data in circle buffer */
  struct list_node   node;    /* Opened file buffer linked list node */
  FAR struct pollfd *fds;     /* Polling structure of waiting thread */
  sem_t              waitsem; /* Used to wait for the availability of data */
  mutex_t            lock;    /* Manages exclusive access to this structure */
};

/* This structure is for mouse upper half driver */

struct mouse_upperhalf_s
{
  uint8_t          nums;               /* Number of mouse point structure */
  mutex_t          lock;               /* Manages exclusive access to this structure */
  struct list_node head;               /* Opened file buffer chain header node */
  FAR struct mouse_lowerhalf_s *lower; /* A pointer of lower half instance */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     mouse_open(FAR struct file *filep);
static int     mouse_close(FAR struct file *filep);
static ssize_t mouse_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static int     mouse_poll(FAR struct file *filep, FAR struct pollfd *fds,
                          bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_mouse_fops =
{
  mouse_open,     /* open */
  mouse_close,    /* close */
  mouse_read,     /* read */
  NULL,           /* write */
  NULL,           /* seek */
  NULL,           /* ioctl */
  NULL,           /* mmap */
  NULL,           /* truncate */
  mouse_poll      /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mouse_open
 ****************************************************************************/

static int mouse_open(FAR struct file *filep)
{
  FAR struct mouse_openpriv_s  *openpriv;
  FAR struct inode             *inode = filep->f_inode;
  FAR struct mouse_upperhalf_s *upper = inode->i_private;
  int ret;

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  openpriv = kmm_zalloc(sizeof(struct mouse_openpriv_s));
  if (openpriv == NULL)
    {
      nxmutex_unlock(&upper->lock);
      return -ENOMEM;
    }

  ret = circbuf_init(&openpriv->circbuf, NULL,
                     upper->nums * sizeof(struct mouse_report_s));
  if (ret < 0)
    {
      kmm_free(openpriv);
      nxmutex_unlock(&upper->lock);
      return ret;
    }

  nxsem_init(&openpriv->waitsem, 0, 0);
  nxmutex_init(&openpriv->lock);
  list_add_tail(&upper->head, &openpriv->node);

  /* Save the buffer node pointer so that it can be used directly
   * in the read operation.
   */

  filep->f_priv = openpriv;
  nxmutex_unlock(&upper->lock);
  return ret;
}

/****************************************************************************
 * Name: mouse_close
 ****************************************************************************/

static int mouse_close(FAR struct file *filep)
{
  FAR struct mouse_openpriv_s  *openpriv = filep->f_priv;
  FAR struct inode             *inode    = filep->f_inode;
  FAR struct mouse_upperhalf_s *upper    = inode->i_private;
  int ret;

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  list_delete(&openpriv->node);
  circbuf_uninit(&openpriv->circbuf);
  nxsem_destroy(&openpriv->waitsem);
  nxmutex_destroy(&openpriv->lock);
  kmm_free(openpriv);

  nxmutex_unlock(&upper->lock);
  return ret;
}

/****************************************************************************
 * Name: mouse_read
 ****************************************************************************/

static ssize_t mouse_read(FAR struct file *filep, FAR char *buffer,
                          size_t len)
{
  FAR struct mouse_openpriv_s *openpriv = filep->f_priv;
  ssize_t ret;

  if (!buffer || !len)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&openpriv->lock);
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
          nxmutex_unlock(&openpriv->lock);
          ret = nxsem_wait_uninterruptible(&openpriv->waitsem);
          if (ret < 0)
            {
              return ret;
            }

          ret = nxmutex_lock(&openpriv->lock);
          if (ret < 0)
            {
              return ret;
            }
        }
    }

  ret = circbuf_read(&openpriv->circbuf, buffer, len);

out:
  nxmutex_unlock(&openpriv->lock);
  return ret;
}

/****************************************************************************
 * Name: mouse_poll
 ****************************************************************************/

static int mouse_poll(FAR struct file *filep,
                      FAR struct pollfd *fds, bool setup)
{
  FAR struct mouse_openpriv_s *openpriv = filep->f_priv;
  pollevent_t eventset = 0;
  int ret;

  ret = nxmutex_lock(&openpriv->lock);
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
          eventset |= POLLIN;
        }

      poll_notify(&fds, 1, eventset);
    }
  else if (fds->priv)
    {
      openpriv->fds = NULL;
      fds->priv     = NULL;
    }

errout:
  nxmutex_unlock(&openpriv->lock);
  return ret;
}

/****************************************************************************
 * Public Function
 ****************************************************************************/

/****************************************************************************
 * Name: mouse_event
 ****************************************************************************/

void mouse_event(FAR void *priv, FAR const struct mouse_report_s *sample)
{
  FAR struct mouse_upperhalf_s *upper = priv;
  FAR struct mouse_openpriv_s  *openpriv;
  int semcount;

  if (nxmutex_lock(&upper->lock) < 0)
    {
      return;
    }

  list_for_every_entry(&upper->head, openpriv, struct mouse_openpriv_s, node)
    {
      circbuf_overwrite(&openpriv->circbuf, sample,
                        sizeof(struct mouse_report_s));

      nxsem_get_value(&openpriv->waitsem, &semcount);
      if (semcount < 1)
        {
          nxsem_post(&openpriv->waitsem);
        }

      if (openpriv->fds && openpriv->fds->fd >= 0)
        {
          poll_notify(&openpriv->fds, 1, POLLIN);
        }
    }

  nxmutex_unlock(&upper->lock);
}

/****************************************************************************
 * Name: mouse_register
 ****************************************************************************/

int mouse_register(FAR struct mouse_lowerhalf_s *lower,
                   FAR const char *path, uint8_t nums)
{
  FAR struct mouse_upperhalf_s *upper;
  int ret;

  iinfo("Registering %s\n", path);

  if (lower == NULL || nums == 0)
    {
      ierr("ERROR: invalid mouse device\n");
      return -EINVAL;
    }

  upper = kmm_zalloc(sizeof(struct mouse_upperhalf_s));
  if (!upper)
    {
      ierr("ERROR: Failed to mem alloc!\n");
      return -ENOMEM;
    }

  lower->priv  = upper;
  upper->lower = lower;
  upper->nums  = nums;
  list_initialize(&upper->head);
  nxmutex_init(&upper->lock);

  ret = register_driver(path, &g_mouse_fops, 0666, upper);
  if (ret < 0)
    {
      nxmutex_destroy(&upper->lock);
      kmm_free(upper);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: mouse_unregister
 ****************************************************************************/

void mouse_unregister(FAR struct mouse_lowerhalf_s *lower,
                      FAR const char *path)
{
  FAR struct mouse_upperhalf_s *upper;

  DEBUGASSERT(lower != NULL);
  DEBUGASSERT(lower->priv != NULL);

  upper = lower->priv;
  iinfo("UnRegistering %s\n", path);
  unregister_driver(path);

  nxmutex_destroy(&upper->lock);
  kmm_free(upper);
}
