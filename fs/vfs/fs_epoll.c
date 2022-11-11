/****************************************************************************
 * fs/vfs/fs_epoll.c
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

#include <sys/epoll.h>

#include <inttypes.h>
#include <stdint.h>
#include <poll.h>
#include <errno.h>
#include <string.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>

#include "inode/inode.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct epoll_head
{
  int size;
  int occupied;
  int crefs;
  mutex_t lock;
  FAR epoll_data_t *data;
  FAR struct pollfd *poll;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int epoll_do_open(FAR struct file *filep);
static int epoll_do_close(FAR struct file *filep);
static int epoll_do_poll(FAR struct file *filep,
                         FAR struct pollfd *fds, bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_epoll_ops =
{
  epoll_do_open,    /* open */
  epoll_do_close,   /* close */
  NULL,             /* read */
  NULL,             /* write */
  NULL,             /* seek */
  NULL,             /* ioctl */
  epoll_do_poll     /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL            /* unlink */
#endif
};

static struct inode g_epoll_inode =
{
  NULL,                   /* i_parent */
  NULL,                   /* i_peer */
  NULL,                   /* i_child */
  1,                      /* i_crefs */
  FSNODEFLAG_TYPE_DRIVER, /* i_flags */
  {
    &g_epoll_ops          /* u */
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR struct epoll_head *epoll_head_from_fd(int fd)
{
  FAR struct file *filep;
  int ret;

  /* Get file pointer by file descriptor */

  ret = fs_getfilep(fd, &filep);
  if (ret < 0)
    {
      set_errno(-ret);
      return NULL;
    }

  /* Check fd come from us */

  if (!filep->f_inode || filep->f_inode->u.i_ops != &g_epoll_ops)
    {
      set_errno(EBADF);
      return NULL;
    }

  return (FAR struct epoll_head *)filep->f_priv;
}

static int epoll_do_open(FAR struct file *filep)
{
  FAR struct epoll_head *eph = filep->f_priv;
  int ret;

  ret = nxmutex_lock(&eph->lock);
  if (ret < 0)
    {
      return ret;
    }

  eph->crefs++;
  nxmutex_unlock(&eph->lock);
  return ret;
}

static int epoll_do_close(FAR struct file *filep)
{
  FAR struct epoll_head *eph = filep->f_priv;
  int ret;

  ret = nxmutex_lock(&eph->lock);
  if (ret < 0)
    {
      return ret;
    }

  eph->crefs--;
  nxmutex_unlock(&eph->lock);
  if (eph->crefs <= 0)
    {
      nxmutex_destroy(&eph->lock);
      kmm_free(eph);
    }

  return ret;
}

static int epoll_do_poll(FAR struct file *filep,
                         FAR struct pollfd *fds, bool setup)
{
  return OK;
}

static int epoll_do_create(int size, int flags)
{
  FAR struct epoll_head *eph;
  int reserve = size + 1;
  int fd;

  eph = (FAR struct epoll_head *)
        kmm_zalloc(sizeof(struct epoll_head) +
                   sizeof(epoll_data_t) * reserve +
                   sizeof(struct pollfd) * reserve);
  if (eph == NULL)
    {
      set_errno(ENOMEM);
      return ERROR;
    }

  nxmutex_init(&eph->lock);

  eph->size = size;
  eph->data = (FAR epoll_data_t *)(eph + 1);
  eph->poll = (FAR struct pollfd *)(eph->data + reserve);

  /* Alloc the file descriptor */

  fd = file_allocate(&g_epoll_inode, flags, 0, eph, 0, true);
  if (fd < 0)
    {
      nxmutex_destroy(&eph->lock);
      kmm_free(eph);
      set_errno(-fd);
      return ERROR;
    }

  /* Setup the first pollfd for internal use */

  eph->poll[0].fd = fd;
  eph->poll[0].events = POLLIN;

  return fd;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: epoll_create
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int epoll_create(int size)
{
  return epoll_do_create(size, 0);
}

/****************************************************************************
 * Name: epoll_create1
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int epoll_create1(int flags)
{
  return epoll_do_create(CONFIG_FS_NEPOLL_DESCRIPTORS, flags);
}

/****************************************************************************
 * Name: epoll_close
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void epoll_close(int epfd)
{
  close(epfd);
}

/****************************************************************************
 * Name: epoll_ctl
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int epoll_ctl(int epfd, int op, int fd, FAR struct epoll_event *ev)
{
  FAR struct epoll_head *eph;
  int ret;
  int i;

  eph = epoll_head_from_fd(epfd);
  if (eph == NULL)
    {
      return ERROR;
    }

  ret = nxmutex_lock(&eph->lock);
  if (ret < 0)
    {
      goto err_without_lock;
    }

  switch (op)
    {
      case EPOLL_CTL_ADD:
        finfo("%08x CTL ADD(%d): fd=%d ev=%08" PRIx32 "\n",
              epfd, eph->occupied, fd, ev->events);
        if (eph->occupied >= eph->size)
          {
            ret = -ENOMEM;
            goto err;
          }

        for (i = 1; i <= eph->occupied; i++)
          {
            if (eph->poll[i].fd == fd)
              {
                ret = -EEXIST;
                goto err;
              }
          }

        eph->data[++eph->occupied]      = ev->data;
        eph->poll[eph->occupied].events = ev->events;
        eph->poll[eph->occupied].fd     = fd;

        break;

      case EPOLL_CTL_DEL:
        for (i = 1; i <= eph->occupied; i++)
          {
            if (eph->poll[i].fd == fd)
              {
                if (i != eph->occupied)
                  {
                    memmove(&eph->data[i], &eph->data[i + 1],
                            sizeof(epoll_data_t) * (eph->occupied - i));
                    memmove(&eph->poll[i], &eph->poll[i + 1],
                            sizeof(struct pollfd) * (eph->occupied - i));
                  }

                break;
              }
          }

        if (i > eph->occupied)
          {
            ret = -ENOENT;
            goto err;
          }

        eph->occupied--;
        break;

      case EPOLL_CTL_MOD:
        finfo("%08x CTL MOD(%d): fd=%d ev=%08" PRIx32 "\n",
              epfd, eph->occupied, fd, ev->events);
        for (i = 1; i <= eph->occupied; i++)
          {
            if (eph->poll[i].fd == fd)
              {
                eph->data[i]        = ev->data;
                eph->poll[i].events = ev->events;
                break;
              }
          }

        if (i > eph->occupied)
          {
            ret = -ENOENT;
            goto err;
          }

        break;

      default:
        ret = -EINVAL;
        goto err;
    }

  poll_notify(&eph->poll, 1, POLLIN);
  nxmutex_unlock(&eph->lock);
  return OK;

err:
  nxmutex_unlock(&eph->lock);
err_without_lock:
  set_errno(-ret);
  return ERROR;
}

/****************************************************************************
 * Name: epoll_pwait
 ****************************************************************************/

int epoll_pwait(int epfd, FAR struct epoll_event *evs,
                int maxevents, int timeout, FAR const sigset_t *sigmask)
{
  FAR struct epoll_head *eph;
  struct timespec expire;
  struct timespec curr;
  struct timespec diff;
  int counter;
  int rc;
  int i;

  eph = epoll_head_from_fd(epfd);
  if (eph == NULL)
    {
      return ERROR;
    }

  if (timeout >= 0)
    {
      expire.tv_sec  = timeout / 1000;
      expire.tv_nsec = timeout % 1000 * 1000;

      clock_systime_timespec(&curr);
      clock_timespec_add(&curr, &expire, &expire);
    }

again:
  if (timeout < 0)
    {
      rc = ppoll(eph->poll, eph->occupied + 1, NULL, sigmask);
    }
  else
    {
      clock_systime_timespec(&curr);
      clock_timespec_subtract(&expire, &curr, &diff);

      rc = ppoll(eph->poll, eph->occupied + 1, &diff, sigmask);
    }

  if (rc <= 0)
    {
      return rc;
    }
  else if (eph->poll[0].revents != 0)
    {
      if (--rc == 0)
        {
          goto again;
        }
    }

  if (rc > maxevents)
    {
      rc = maxevents;
    }

  /* Iterate over non NULL event fds */

  for (i = 0, counter = 1; i < rc && counter <= eph->occupied; counter++)
    {
      if (eph->poll[counter].revents != 0)
        {
          evs[i].data     = eph->data[counter];
          evs[i++].events = eph->poll[counter].revents;
          if (eph->poll[counter].events & EPOLLONESHOT)
            {
              eph->poll[counter].events = 0; /* Disable oneshot internally */
            }
        }
    }

  return i;
}

/****************************************************************************
 * Name: epoll_wait
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int epoll_wait(int epfd, FAR struct epoll_event *evs,
               int maxevents, int timeout)
{
  return epoll_pwait(epfd, evs, maxevents, timeout, NULL);
}
