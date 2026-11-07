/****************************************************************************
 * fs/vfs/fs_epoll.c
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

#include <sys/epoll.h>

#include <inttypes.h>
#include <stdint.h>
#include <poll.h>
#include <errno.h>
#include <string.h>
#include <nuttx/debug.h>

#include <nuttx/nuttx.h>
#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/list.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/spinlock.h>
#include <nuttx/tls.h>

#include "inode/inode.h"
#include "fs_heap.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct epoll_node_s
{
  struct list_node         node;   /* Used to link in the setup/oneshot/free
                                    * lists (ownership chain). */
  struct list_node         rnode;  /* Used to link in the ready list. */
  epoll_data_t             data;
  bool                     queued; /* Whether the node is in the ready list. */

  /* Suppress synchronous rearm notify. */

  bool                     suppress;
  struct pollfd_s          pfd;
  FAR struct file         *filep;
  FAR struct epoll_head_s *eph;
};

typedef struct epoll_node_s epoll_node_t;

struct epoll_head_s
{
  int                   size;
  int                   crefs;
  mutex_t               lock;
  spinlock_t            ready_lock;
  sem_t                 sem;
  struct list_node      setup;   /* The setup list, store all the persistently
                                  * registered epoll node (driver poll kept
                                  * armed across epoll_wait()).
                                  */
  struct list_node      ready;   /* The ready list, store all the epoll node
                                  * notified by the driver, pending to be
                                  * delivered by epoll_wait().
                                  */
  struct list_node      oneshot; /* The oneshot list, store all the epoll node
                                  * notified after epoll_wait and with
                                  * EPOLLONESHOT events, these oneshot epoll
                                  * nodes can be reset by epoll_ctl (move
                                  * from oneshot list to the setup list).
                                  */
  struct list_node      free;    /* The free list, store all the freed epoll
                                  * node.
                                  */
  struct list_node      extend;  /* The extend list, store all the malloced
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
                         FAR struct pollfd_s *fds, bool setup);
static void epoll_refresh_node(FAR epoll_head_t *eph, FAR epoll_node_t *epn);
static int epoll_collect(FAR epoll_head_t *eph, FAR struct epoll_event *evs,
                         int maxevents);
static void epoll_node_dequeue(FAR epoll_head_t *eph, FAR epoll_node_t *epn);
static void epoll_node_ready(FAR epoll_head_t *eph, FAR epoll_node_t *epn);
static int epoll_arm_node(FAR epoll_head_t *eph, FAR epoll_node_t *epn,
                          pollevent_t events);

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
  NULL,             /* mmap */
  NULL,             /* truncate */
  epoll_do_poll     /* poll */
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

static FAR epoll_head_t *epoll_head_from_fd(int fd, FAR struct file **filep)
{
  int ret;

  /* Get file pointer by file descriptor */

  ret = file_get(fd, filep);
  if (ret < 0)
    {
      set_errno(-ret);
      return NULL;
    }

  /* Check fd come from us */

  if ((*filep)->f_inode->u.i_ops != &g_epoll_ops)
    {
      file_put(*filep);
      set_errno(EBADF);
      return NULL;
    }

  return (*filep)->f_priv;
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
  FAR epoll_node_t *tmp;
  irqstate_t flags;
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
          file_poll(epn->filep, &epn->pfd, false);
          file_put(epn->filep);
        }

      list_for_every_entry(&eph->oneshot, epn, epoll_node_t, node)
        {
          file_put(epn->filep);
        }

      /* Drop any node still pending in the ready list (its node link is
       * already handled through the setup/oneshot lists above).
       */

      flags = spin_lock_irqsave(&eph->ready_lock);
      list_for_every_entry_safe(&eph->ready, epn, tmp, epoll_node_t, rnode)
        {
          list_delete(&epn->rnode);
        }

      spin_unlock_irqrestore(&eph->ready_lock, flags);

      list_for_every_entry_safe(&eph->extend, epn, tmp, epoll_node_t, node)
        {
          list_delete(&epn->node);
          fs_heap_free(epn);
        }

      fs_heap_free(eph);
    }

  return ret;
}

static int epoll_do_poll(FAR struct file *filep,
                         FAR struct pollfd_s *fds, bool setup)
{
  return OK;
}

static int epoll_do_create(int size, int flags)
{
  FAR epoll_head_t *eph;
  FAR epoll_node_t *epn;
  int fd;
  int i;

  size = size <= 0 ? 1 : size;
  eph = fs_heap_zalloc(sizeof(epoll_head_t) + sizeof(epoll_node_t) * size);
  if (eph == NULL)
    {
      set_errno(ENOMEM);
      return ERROR;
    }

  eph->size = size;
  nxmutex_init(&eph->lock);
  nxsem_init(&eph->sem, 0, 0);
  spin_lock_init(&eph->ready_lock);

  /* List initialize */

  epn = (FAR epoll_node_t *)(eph + 1);

  list_initialize(&eph->setup);
  list_initialize(&eph->ready);
  list_initialize(&eph->oneshot);
  list_initialize(&eph->extend);
  list_initialize(&eph->free);
  for (i = 0; i < size; i++)
    {
      list_add_tail(&eph->free, &epn[i].node);
    }

  eph->crefs++;

  /* Alloc the file descriptor */

  fd = file_allocate_from_inode(&g_epoll_inode, flags, 0, eph, 0);
  if (fd < 0)
    {
      nxmutex_destroy(&eph->lock);
      fs_heap_free(eph);
      set_errno(-fd);
      return ERROR;
    }

  return fd;
}

/****************************************************************************
 * Name: epoll_refresh_node
 *
 * Description:
 *   Re-query the current readiness of a level-triggered node.  The driver
 *   poll is torn down before revents is cleared so an asynchronous
 *   poll_notify() cannot race with the core-side revents reset while the
 *   driver still owns this pollfd.
 *
 ****************************************************************************/

static void epoll_refresh_node(FAR epoll_head_t *eph, FAR epoll_node_t *epn)
{
  irqstate_t flags;

  /* Drop the driver reference before clearing revents. */

  file_poll(epn->filep, &epn->pfd, false);

  /* Clear the accumulated revents and ready state, ask the driver to
   * report whether the state is still ready.
   */

  flags = spin_lock_irqsave(&eph->ready_lock);
  epn->queued      = false;
  epn->pfd.revents = 0;
  spin_unlock_irqrestore(&eph->ready_lock, flags);

  epn->suppress = true;
  file_poll(epn->filep, &epn->pfd, true);
  epn->suppress = false;
}

/****************************************************************************
 * Name: epoll_collect
 *
 * Description:
 *   Deliver all the events currently pending in the ready list.  For each
 *   EPOLLONESHOT node, drop its driver poll registration and park it into
 *   the oneshot list until it is re-armed by epoll_ctl().  For every other
 *   (level-triggered) node, keep the registration and re-query whether it
 *   is still ready so that it is reported again by the next epoll_wait().
 *
 * Input Parameters:
 *   eph       - The epoll head pointer
 *   evs       - The epoll events array
 *   maxevents - The epoll events array size
 *
 * Returned Value:
 *   Return the number of fd that delivered to the user.
 *
 ****************************************************************************/

static int epoll_collect(FAR epoll_head_t *eph, FAR struct epoll_event *evs,
                         int maxevents)
{
  FAR epoll_node_t *epn;
  struct list_node  deliver;
  struct list_node  refresh;
  irqstate_t        flags;
  int               i = 0;

  list_initialize(&deliver);
  list_initialize(&refresh);

  nxmutex_lock(&eph->lock);

  /* Phase 1: atomically move at most maxevents ready nodes out of the
   * ready list into a private queue, so nodes not delivered to userspace
   * stay queued for the next epoll_wait().
   *
   * Keep epn->queued true while a node is on the private deliver/refresh
   * queues.  The same rnode link is still in use there, so a concurrent
   * driver notification must not enqueue it into eph->ready again.  The
   * epoll mutex also prevents epoll_ctl() from trying to dequeue a node
   * from these private queues.
   */

  flags = spin_lock_irqsave(&eph->ready_lock);
  while (i < maxevents && !list_is_empty(&eph->ready))
    {
      epn = container_of(list_remove_head(&eph->ready), epoll_node_t, rnode);
      list_add_tail(&deliver, &epn->rnode);
      i++;
    }

  if (!list_is_empty(&eph->ready))
    {
      int semcount;

      nxsem_get_value(&eph->sem, &semcount);
      if (semcount < 1)
        {
          nxsem_post(&eph->sem);
        }
    }

  spin_unlock_irqrestore(&eph->ready_lock, flags);
  i = 0;

  /* Phase 2: classify and deliver. */

  while (!list_is_empty(&deliver))
    {
      epn = container_of(list_remove_head(&deliver), epoll_node_t, rnode);

      if (epn->pfd.revents != 0 && i < maxevents)
        {
          evs[i].data   = epn->data;
          evs[i++].events = epn->pfd.revents;
        }

      if ((epn->pfd.events & EPOLLONESHOT) != 0)
        {
          /* EPOLLONESHOT: tear down the driver poll and park the node until
           * it is re-armed by epoll_ctl(EPOLL_CTL_MOD).
           */

          file_poll(epn->filep, &epn->pfd, false);
          flags = spin_lock_irqsave(&eph->ready_lock);
          epn->queued = false;
          spin_unlock_irqrestore(&eph->ready_lock, flags);

          list_delete(&epn->node);
          list_add_tail(&eph->oneshot, &epn->node);
        }
      else
        {
          /* Level-triggered: keep ownership stable and re-query readiness
           * below while still holding eph->lock.  The driver may notify this
           * node again during refresh, but the callback only takes the ready
           * spinlock, so it cannot deadlock with the epoll mutex.
           */

          list_add_tail(&refresh, &epn->rnode);
        }
    }

  /* Phase 3: re-query the level-triggered nodes while holding eph->lock so
   * epoll_ctl(DEL) or close cannot recycle a node that is still linked in
   * the private refresh list.
   */

  while (!list_is_empty(&refresh))
    {
      epn = container_of(list_remove_head(&refresh), epoll_node_t, rnode);
      epoll_refresh_node(eph, epn);
    }

  nxmutex_unlock(&eph->lock);

  return i;
}

/****************************************************************************
 * Name: epoll_node_dequeue
 *
 * Description:
 *   Remove a node from the ready list if it is currently queued.  Caller
 *   must hold the eph->lock mutex.
 *
 ****************************************************************************/

static void epoll_node_dequeue(FAR epoll_head_t *eph, FAR epoll_node_t *epn)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&eph->ready_lock);
  if (epn->queued)
    {
      epn->queued = false;
      list_delete(&epn->rnode);
    }

  spin_unlock_irqrestore(&eph->ready_lock, flags);
}

/****************************************************************************
 * Name: epoll_node_ready
 *
 * Description:
 *   Put a node back into the ready list if it still has pending revents.
 *   Caller may hold eph->lock, but not eph->ready_lock.
 *
 ****************************************************************************/

static void epoll_node_ready(FAR epoll_head_t *eph, FAR epoll_node_t *epn)
{
  irqstate_t flags;
  int semcount;

  flags = spin_lock_irqsave(&eph->ready_lock);
  if (epn->pfd.revents != 0 && !epn->queued)
    {
      epn->queued = true;
      list_add_tail(&eph->ready, &epn->rnode);
      nxsem_get_value(&eph->sem, &semcount);
      if (semcount < 1)
        {
          nxsem_post(&eph->sem);
        }
    }

  spin_unlock_irqrestore(&eph->ready_lock, flags);
}

/****************************************************************************
 * Name: epoll_arm_node
 *
 * Description:
 *   (Re-)arm a node: clear its pending ready state, update the expected
 *   events and register (or refresh) its driver poll.  If the driver does
 *   not support the persistent model yet, fall back to the legacy teardown
 *   + set-up cycle.  Caller must hold the eph->lock mutex.
 *
 ****************************************************************************/

static int epoll_arm_node(FAR epoll_head_t *eph, FAR epoll_node_t *epn,
                          pollevent_t events)
{
  pollevent_t old_events;
  pollevent_t old_revents;
  bool was_bound;
  int ret;

  old_events  = epn->pfd.events;
  old_revents = epn->pfd.revents;
  was_bound   = epn->pfd.priv != NULL;

  epoll_node_dequeue(eph, epn);

  if (was_bound)
    {
      /* Tear down with the old event mask before switching masks.  Some
       * lower-half implementations use fds->events to decide which nested
       * poll binding to remove.
       */

      file_poll(epn->filep, &epn->pfd, false);
    }

  epn->pfd.events  = events | POLLALWAYS;
  epn->pfd.revents = 0;

  ret = file_poll(epn->filep, &epn->pfd, true);
  if (ret < 0)
    {
      if (was_bound)
        {
          int ret2;

          /* Keep EPOLL_CTL_MOD failure atomic as far as possible: restore
           * the old event mask and old registration before returning the
           * original error to the caller.
           */

          epn->pfd.events  = old_events;
          epn->pfd.revents = old_revents;
          ret2 = file_poll(epn->filep, &epn->pfd, true);
          if (ret2 >= 0)
            {
              epoll_node_ready(eph, epn);
            }
        }

      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: epoll_cleanup
 *
 * Description:
 *   Cleanup the epoll operation.
 *
 ****************************************************************************/

static void epoll_cleanup(FAR void *arg)
{
  file_put(arg);
}

/****************************************************************************
 * Name: epoll_default_cb
 *
 * Description:
 *   The default epoll callback function, this function do the final step of
 *   poll notification: enqueue the notified node into the ready list and
 *   wake up the waiter.
 *
 *   NOTE: this callback may be invoked from interrupt context (e.g. a
 *   driver's poll notification triggered by a timer), so it only takes the
 *   spin lock protecting the ready list and the semaphore, never the mutex.
 *
 * Input Parameters:
 *   fds - The fds
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void epoll_default_cb(FAR struct pollfd_s *fds)
{
  FAR epoll_node_t *epn = fds->arg;
  FAR epoll_head_t *eph = epn->eph;
  irqstate_t flags;
  int semcount;

  flags = spin_lock_irqsave(&eph->ready_lock);
  if (epn->suppress)
    {
      fds->revents = 0;
      spin_unlock_irqrestore(&eph->ready_lock, flags);
      return;
    }

  if (fds->revents != 0)
    {
      if (!epn->queued)
        {
          epn->queued = true;
          list_add_tail(&eph->ready, &epn->rnode);
          nxsem_get_value(&eph->sem, &semcount);
          if (semcount < 1)
            {
              nxsem_post(&eph->sem);
            }
        }
    }

  spin_unlock_irqrestore(&eph->ready_lock, flags);
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
  FAR struct file *filep;
  FAR epoll_head_t *eph;
  FAR epoll_node_t *epn;
  int ret;
  int i;

  eph = epoll_head_from_fd(epfd, &filep);
  if (eph == NULL)
    {
      set_errno(EBADF);
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

        list_for_every_entry(&eph->oneshot, epn, epoll_node_t, node)
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

            extend = fs_heap_zalloc(sizeof(*extend) +
                                2 * sizeof(epoll_node_t) * eph->size);
            if (extend == NULL)
              {
                ret = -ENOMEM;
                goto err;
              }

            eph->size *= 2;
            list_add_tail(&eph->extend, extend);
            epn = (FAR epoll_node_t *)(extend + 1);
            for (i = 0; i < eph->size; i++)
              {
                list_add_tail(&eph->free, &epn[i].node);
              }
          }

        epn = container_of(list_remove_head(&eph->free), epoll_node_t, node);
        epn->eph       = eph;
        epn->queued    = false;
        epn->suppress  = false;
        epn->data      = ev->data;
        epn->pfd.fd    = fd;
        epn->pfd.arg   = epn;
        epn->pfd.cb    = epoll_default_cb;

        ret = file_get(fd, &epn->filep);
        if (ret < 0)
          {
            list_add_tail(&eph->free, &epn->node);
            goto err;
          }

        ret = epoll_arm_node(eph, epn, ev->events);
        if (ret < 0)
          {
            file_put(epn->filep);
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
                epoll_node_dequeue(eph, epn);
                file_poll(epn->filep, &epn->pfd, false);
                file_put(epn->filep);
                list_delete(&epn->node);
                list_add_tail(&eph->free, &epn->node);
                goto out;
              }
          }

        list_for_every_entry(&eph->oneshot, epn, epoll_node_t, node)
          {
            if (epn->pfd.fd == fd)
              {
                epoll_node_dequeue(eph, epn);
                file_put(epn->filep);
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
                epn->data = ev->data;
                if (epn->pfd.events != (ev->events | POLLALWAYS))
                  {
                    /* The event mask changed: re-arm (refresh) the still
                     * registered poll with the new mask.
                     */

                    ret = epoll_arm_node(eph, epn, ev->events);
                    if (ret < 0)
                      {
                        goto err;
                      }
                  }

                goto out;
              }
          }

        list_for_every_entry(&eph->oneshot, epn, epoll_node_t, node)
          {
            if (epn->pfd.fd == fd)
              {
                epn->data = ev->data;
                ret = epoll_arm_node(eph, epn, ev->events);
                if (ret < 0)
                  {
                    goto err;
                  }

                list_delete(&epn->node);
                list_add_tail(&eph->setup, &epn->node);
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
  file_put(filep);
  return OK;
err:
  nxmutex_unlock(&eph->lock);
err_without_lock:
  file_put(filep);
  set_errno(-ret);
  return ERROR;
}

/****************************************************************************
 * Name: epoll_pwait
 ****************************************************************************/

int epoll_pwait(int epfd, FAR struct epoll_event *evs,
                int maxevents, int timeout, FAR const sigset_t *sigmask)
{
  FAR struct file *filep;
  FAR epoll_head_t *eph;
#ifndef CONFIG_DISABLE_ALL_SIGNALS
  sigset_t oldsigmask;
#endif
  int ret;

  eph = epoll_head_from_fd(epfd, &filep);
  if (eph == NULL)
    {
      goto out;
    }

  /* All the registered driver polls are persistently armed, so there is no
   * need to (re)register anything before waiting.  Just wait until a driver
   * notification wakes us up.
   */

retry:

  /* Wait the poll ready.  Without signals, there is no mask to set. */

#ifndef CONFIG_DISABLE_ALL_SIGNALS
  nxsig_procmask(SIG_SETMASK, sigmask, &oldsigmask);
#endif

  /* Push a cancellation point onto the stack.  This will be called if
   * the thread is canceled.
   */

  tls_cleanup_push(tls_get_info(), epoll_cleanup, filep);
  if (timeout == 0)
    {
      ret = -ETIMEDOUT;
    }
  else if (timeout > 0)
    {
      ret = nxsem_tickwait(&eph->sem, MSEC2TICK(timeout));
    }
  else
    {
      ret = nxsem_wait(&eph->sem);
    }

  /* Pop the cancellation point */

  tls_cleanup_pop(tls_get_info(), 0);

#ifndef CONFIG_DISABLE_ALL_SIGNALS
  nxsig_procmask(SIG_SETMASK, &oldsigmask, NULL);
#endif
  if (ret < 0 && ret != -ETIMEDOUT)
    {
      goto err;
    }
  else /* ret >= 0 or ret == -ETIMEDOUT */
    {
      int num = epoll_collect(eph, evs, maxevents);

      if (num == 0 && ret >= 0)
        {
          goto retry;
        }

      ret = num;
    }

  file_put(filep);
  return ret;

err:
  file_put(filep);
  set_errno(-ret);
out:
  ferr("epoll wait failed:%d, timeout:%d\n", errno, timeout);
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
  FAR struct file *filep;
  FAR epoll_head_t *eph;
  int ret;

  eph = epoll_head_from_fd(epfd, &filep);
  if (eph == NULL)
    {
      goto out;
    }

  /* All the registered driver polls are persistently armed, so there is no
   * need to (re)register anything before waiting.  Just wait until a driver
   * notification wakes us up.
   */

retry:

  /* Push a cancellation point onto the stack.  This will be called if
   * the thread is canceled.
   */

  tls_cleanup_push(tls_get_info(), epoll_cleanup, filep);

  /* Wait the poll ready */

  if (timeout == 0)
    {
      ret = -ETIMEDOUT;
    }
  else if (timeout > 0)
    {
      ret = nxsem_tickwait(&eph->sem, MSEC2TICK(timeout));
    }
  else
    {
      ret = nxsem_wait(&eph->sem);
    }

  /* Pop the cancellation point */

  tls_cleanup_pop(tls_get_info(), 0);

  if (ret < 0 && ret != -ETIMEDOUT)
    {
      goto err;
    }
  else /* ret >= 0 or ret == -ETIMEDOUT */
    {
      int num = epoll_collect(eph, evs, maxevents);

      if (num == 0 && ret >= 0)
        {
          goto retry;
        }

      ret = num;
    }

  file_put(filep);
  return ret;

err:
  file_put(filep);
  set_errno(-ret);
out:
  ferr("epoll wait failed:%d, timeout:%d\n", errno, timeout);
  return ERROR;
}
