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
#include <nuttx/list.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>

#include "inode/inode.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct epoll_node_s
{
  struct list_node      node;
  epoll_data_t          data;
  struct pollfd         pfd;
};

typedef struct epoll_node_s epoll_node_t;

struct epoll_head_s
{
  int                   size;
  int                   crefs;
  mutex_t               lock;
  sem_t                 sem;
  struct list_node      setup;    /* The setup list, store all the setuped
                                   * epoll node.
                                   */
  struct list_node      teardown; /* The teardown list, store all the epoll
                                   * node notified after epoll_wait finish,
                                   * these epoll node should be setup again
                                   * to check the pending poll notification.
                                   */
  struct list_node      free;     /* The free list, store all the freed epoll
                                   * node.
                                   */
  struct list_node      extend;   /* The extend list, store all the malloced
                                   * first node, used to free the malloced
                                   * memory in epoll_do_close().
                                   */
};

typedef struct epoll_head_s epoll_head_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int epoll_do_open(FAR struct file *filep);
static int epoll_do_close(FAR struct file *filep);
static int epoll_do_poll(FAR struct file *filep,
                         FAR struct pollfd *fds, bool setup);
static int epoll_setup(FAR epoll_head_t *eph);
static int epoll_teardown(FAR epoll_head_t *eph, FAR struct epoll_event *evs,
                          int maxevents);

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

static FAR epoll_head_t *epoll_head_from_fd(int fd)
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

  return (FAR epoll_head_t *)filep->f_priv;
}

static int epoll_do_open(FAR struct file *filep)
{
  FAR epoll_head_t *eph = filep->f_priv;
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
  FAR epoll_head_t *eph = filep->f_priv;
  FAR epoll_node_t *epn;
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
      list_for_every_entry(&eph->setup, epn, epoll_node_t, node)
        {
          poll_fdsetup(epn->pfd.fd, &epn->pfd, false);
        }

      list_for_every_entry(&eph->extend, epn, epoll_node_t, node)
        {
          kmm_free(epn);
        }

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
  FAR epoll_head_t *eph;
  FAR epoll_node_t *epn;
  int fd;

  size = size <= 0 ? 1 : size;
  eph = kmm_zalloc(sizeof(epoll_head_t) + sizeof(epoll_node_t) * size);
  if (eph == NULL)
    {
      set_errno(ENOMEM);
      return ERROR;
    }

  eph->size = size;
  nxmutex_init(&eph->lock);
  nxsem_init(&eph->sem, 0, 0);

  /* List initialize */

  epn = (FAR epoll_node_t *)(eph + 1);

  list_initialize(&eph->setup);
  list_initialize(&eph->teardown);
  list_initialize(&eph->extend);
  list_initialize(&eph->free);
  for (int i = 0; i < size; i++)
    {
      list_add_tail(&eph->free, &epn[i].node);
    }

  /* Alloc the file descriptor */

  fd = file_allocate(&g_epoll_inode, flags, 0, eph, 0, true);
  if (fd < 0)
    {
      nxmutex_destroy(&eph->lock);
      kmm_free(eph);
      set_errno(-fd);
      return ERROR;
    }

  return fd;
}

static int epoll_setup(FAR epoll_head_t *eph)
{
  FAR epoll_node_t *tepn;
  FAR epoll_node_t *epn;
  int ret;

  ret = nxmutex_lock(&eph->lock);
  if (ret < 0)
    {
      return ret;
    }

  list_for_every_entry_safe(&eph->teardown, epn, tepn, epoll_node_t, node)
    {
      /* Setup again to check the notified pollfd last epoll_wait() to
       * cover the situation several poll event pending on one fd.
       */

      epn->pfd.revents = 0;
      ret = poll_fdsetup(epn->pfd.fd, &epn->pfd, true);
      if (ret < 0)
        {
          ferr("epoll setup failed, fd=%d, events=%08" PRIx32 ", ret=%d\n",
               epn->pfd.fd, epn->pfd.events, ret);
          break;
        }

      list_delete(&epn->node);
      list_add_tail(&eph->setup, &epn->node);
    }

  nxmutex_unlock(&eph->lock);
  return ret;
}

static int epoll_teardown(FAR epoll_head_t *eph, FAR struct epoll_event *evs,
                          int maxevents)
{
  FAR epoll_node_t *tepn;
  FAR epoll_node_t *epn;
  int i = 0;

  nxmutex_lock(&eph->lock);

  list_for_every_entry_safe(&eph->setup, epn, tepn, epoll_node_t, node)
    {
      if (epn->pfd.revents != 0 && i < maxevents)
        {
          evs[i].data     = epn->data;
          evs[i++].events = epn->pfd.revents;
          poll_fdsetup(epn->pfd.fd, &epn->pfd, false);
          list_delete(&epn->node);
          if ((epn->pfd.events & EPOLLONESHOT) != 0)
            {
              list_add_tail(&eph->free, &epn->node);
            }
          else
            {
              list_add_tail(&eph->teardown, &epn->node);
            }
        }
    }

  nxmutex_unlock(&eph->lock);
  return i;
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
  FAR struct list_node *extend;
  FAR epoll_head_t *eph;
  FAR epoll_node_t *epn;
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
        finfo("%p CTL ADD: fd=%d ev=%08" PRIx32 "\n", eph, fd, ev->events);

        /* Check repetition */

        list_for_every_entry(&eph->setup, epn, epoll_node_t, node)
          {
            if (epn->pfd.fd == fd)
              {
                ret = -EEXIST;
                goto err;
              }
          }

        if (list_is_empty(&eph->free))
          {
            /* Malloc new epoll node, insert the first list_node to the
             * extend list and insert the remaining epoll nodes to the free
             * list.
             */

            extend = kmm_zalloc(sizeof(*extend) +
                                sizeof(epoll_node_t) * eph->size);
            if (extend == NULL)
              {
                ret = -ENOMEM;
                goto err;
              }

            list_add_tail(&eph->extend, extend);
            epn = (FAR epoll_node_t *)(extend + 1);
            for (i = 0; i < eph->size; i++)
              {
                list_add_tail(&eph->free, &epn[i].node);
              }

            eph->size += eph->size;
          }

        epn = list_remove_head_type(&eph->free, epoll_node_t, node);
        epn->data        = ev->data;
        epn->pfd.events  = ev->events;
        epn->pfd.fd      = fd;
        epn->pfd.arg     = &eph->sem;
        epn->pfd.cb      = poll_default_cb;
        epn->pfd.revents = 0;

        ret = poll_fdsetup(fd, &epn->pfd, true);
        if (ret < 0)
          {
            list_add_tail(&eph->free, &epn->node);
            goto err;
          }

        list_add_tail(&eph->setup, &epn->node);
        break;

      case EPOLL_CTL_DEL:
        finfo("%p CTL DEL: fd=%d\n", eph, fd);
        list_for_every_entry(&eph->setup, epn, epoll_node_t, node)
          {
            if (epn->pfd.fd == fd)
              {
                poll_fdsetup(fd, &epn->pfd, false);
                list_delete(&epn->node);
                list_add_tail(&eph->free, &epn->node);
                goto out;
              }
          }

        list_for_every_entry(&eph->teardown, epn, epoll_node_t, node)
          {
            if (epn->pfd.fd == fd)
              {
                list_delete(&epn->node);
                list_add_tail(&eph->free, &epn->node);
                goto out;
              }
          }

        break;

      case EPOLL_CTL_MOD:
        finfo("%p CTL MOD: fd=%d ev=%08" PRIx32 "\n", eph, fd, ev->events);
        list_for_every_entry(&eph->setup, epn, epoll_node_t, node)
          {
            if (epn->pfd.fd == fd)
              {
                if (epn->pfd.events != ev->events)
                  {
                    poll_fdsetup(fd, &epn->pfd, false);

                    epn->data        = ev->data;
                    epn->pfd.events  = ev->events;
                    epn->pfd.fd      = fd;
                    epn->pfd.revents = 0;

                    ret = poll_fdsetup(fd, &epn->pfd, true);
                    if (ret < 0)
                      {
                        goto err;
                      }
                  }

                break;
              }
          }

        break;

      default:
        ret = -EINVAL;
        goto err;
    }

out:
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
  FAR epoll_head_t *eph;
  sigset_t oldsigmask;
  int ret;

  eph = epoll_head_from_fd(epfd);
  if (eph == NULL)
    {
      return ERROR;
    }

  ret = epoll_setup(eph);
  if (ret < 0)
    {
      goto err;
    }

  /* Wait the poll ready */

  nxsig_procmask(SIG_SETMASK, sigmask, &oldsigmask);

  if (timeout == 0)
    {
      ret = OK;
    }
  else if (timeout > 0)
    {
      clock_t ticks;
#if (MSEC_PER_TICK * USEC_PER_MSEC) != USEC_PER_TICK && \
    defined(CONFIG_HAVE_LONG_LONG)
      ticks = (((unsigned long long)timeout * USEC_PER_MSEC) +
                (USEC_PER_TICK - 1)) /
              USEC_PER_TICK;
#else
      ticks = ((unsigned int)timeout + (MSEC_PER_TICK - 1)) /
              MSEC_PER_TICK;
#endif

      ret = nxsem_tickwait(&eph->sem, ticks);
      if (ret == -ETIMEDOUT)
        {
          ret = OK;
        }
    }
  else
    {
      ret = nxsem_wait(&eph->sem);
    }

  nxsig_procmask(SIG_SETMASK, &oldsigmask, NULL);
  if (ret < 0)
    {
      goto err;
    }

  return epoll_teardown(eph, evs, maxevents);

err:
  set_errno(-ret);
  return ERROR;
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
  FAR epoll_head_t *eph;
  int ret;

  eph = epoll_head_from_fd(epfd);
  if (eph == NULL)
    {
      return ERROR;
    }

  ret = epoll_setup(eph);
  if (ret < 0)
    {
      goto err;
    }

  /* Wait the poll ready */

  if (timeout == 0)
    {
      ret = OK;
    }
  else if (timeout > 0)
    {
      clock_t ticks;
#if (MSEC_PER_TICK * USEC_PER_MSEC) != USEC_PER_TICK && \
    defined(CONFIG_HAVE_LONG_LONG)
      ticks = (((unsigned long long)timeout * USEC_PER_MSEC) +
                (USEC_PER_TICK - 1)) /
              USEC_PER_TICK;
#else
      ticks = ((unsigned int)timeout + (MSEC_PER_TICK - 1)) /
              MSEC_PER_TICK;
#endif

      ret = nxsem_tickwait(&eph->sem, ticks);
      if (ret == -ETIMEDOUT)
        {
          ret = OK;
        }
    }
  else
    {
      ret = nxsem_wait(&eph->sem);
    }

  if (ret < 0)
    {
      goto err;
    }

  return epoll_teardown(eph, evs, maxevents);

err:
  set_errno(-ret);
  return ERROR;
}
