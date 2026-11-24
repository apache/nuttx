/****************************************************************************
 * drivers/rpmsg/rpmsg.c
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

#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>
#include <sys/param.h>
#include <time.h>

#include <metal/sys.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/list.h>
#include <nuttx/mm/mempool.h>
#include <nuttx/mutex.h>
#include <nuttx/nuttx.h>
#include <nuttx/percpu.h>
#include <nuttx/queue.h>
#include <nuttx/rwsem.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>

#include "rpmsg.h"
#include "rpmsg_procfs.h"
#include "rpmsg_router.h"
#include "rpmsg_test.h"
#include "rpmsg_wakelock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_RPMSG_WQUEUE_GLOBAL
#  define rpmsg_get_wqueues(rpmsg) g_rpmsg_wqueues
#else
#  define rpmsg_get_wqueues(rpmsg) ((rpmsg)->wqueues)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rpmsg_bind_s
{
  char              name[RPMSG_NAME_SIZE];
  uint32_t          dest;
  struct list_node  node;
  size_t            extra_len;
  uint8_t           extra[];
};

struct rpmsg_cb_s
{
  FAR void              *priv;
  rpmsg_dev_cb_t        device_created;
  rpmsg_dev_cb_t        device_destroy;
  rpmsg_match_cb_t      ns_match;
  rpmsg_bind_cb_t       ns_bind;
  rpmsg_bind_extra_cb_t ns_bind_extra;
  dq_entry_t            node;
};

struct rpmsg_ioctl_s
{
  FAR const char *cpuname;
  int             cmd;
  unsigned long   arg;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rpmsg_dev_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static DEFINE_PER_CPU_BSS_BMP(dq_queue_t, g_rpmsg_cb);
#define g_rpmsg_cb this_cpu_var_bmp(g_rpmsg_cb)

static DEFINE_PER_CPU_BSS_BMP(dq_queue_t, g_rpmsg);
#define g_rpmsg this_cpu_var_bmp(g_rpmsg)

static DEFINE_PER_CPU_BMP(rw_semaphore_t, g_rpmsg_lock) = RWSEM_INITIALIZER;
#define g_rpmsg_lock this_cpu_var_bmp(g_rpmsg_lock)

#if CONFIG_RPMSG_POOL_COUNT > 0
MEMPOOL_DEFINE(g_rpmsg_pool, CONFIG_RPMSG_POOL_SIZE,
               CONFIG_RPMSG_POOL_COUNT, CONFIG_RPMSG_POOL_COUNT, 0);
#endif

#ifdef CONFIG_RPMSG_WQUEUE_GLOBAL
static DEFINE_PER_CPU_BSS_BMP(struct rpmsg_wqueue_s,
                              g_rpmsg_wqueues[CONFIG_RPMSG_WQUEUE_NUMBER]);
#define g_rpmsg_wqueues this_cpu_var_bmp(g_rpmsg_wqueues)
static DEFINE_PER_CPU_BMP(spinlock_t, g_rpmsg_wqueue_lock) = SP_UNLOCKED;
#define g_rpmsg_wqueue_lock this_cpu_var_bmp(g_rpmsg_wqueue_lock)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline FAR struct rpmsg_s *
rpmsg_get_by_rdev(FAR struct rpmsg_device *rdev)
{
  FAR struct rpmsg_s *rpmsg = NULL;

  if (rdev)
    {
      rpmsg = (FAR struct rpmsg_s *)
        ((FAR char *)rdev - sizeof(struct rpmsg_s));
    }

  return rpmsg;
}

static int rpmsg_dev_ioctl_(FAR struct rpmsg_s *rpmsg, int cmd,
                            unsigned long arg)
{
  int ret = -ENOTTY;

  switch (cmd)
    {
      case RPMSGIOC_PANIC:
        if (rpmsg->ops->panic)
          {
            rpmsg->ops->panic(rpmsg);
            ret = OK;
          }

        break;
#if CONFIG_RPMSG_DUMP_LEVEL > RPMSG_DUMP_DISABLE
      case RPMSGIOC_DUMP:
        if (rpmsg->ops->dump)
          {
            rpmsg->ops->dump(rpmsg, (bool)arg);
            ret = OK;
          }

        break;
#endif
#ifdef CONFIG_RPMSG_PING
      case RPMSGIOC_PING:
        ret = rpmsg_ping(&rpmsg->ping, (FAR const struct rpmsg_ping_s *)arg);
        break;
#endif
#ifdef CONFIG_RPMSG_TEST
      case RPMSGIOC_TEST:
        ret = rpmsg_test(&rpmsg->test, arg);
        break;
#endif
      case RPMSGIOC_RUNNING:
        *(FAR bool *)arg = rpmsg_is_running(rpmsg_get_rdev_by_rpmsg(rpmsg));
        ret = OK;
        break;
#ifdef CONFIG_RPMSG_WAKELOCK
      case RPMSGIOC_ACQUIRE_WAKELOCK:
      case RPMSGIOC_RELEASE_WAKELOCK:
        ret = rpmsg_wakelock(&rpmsg->wakelock, cmd, arg);
        break;
#endif
#ifdef CONFIG_RPMSG_CHAR
      case RPMSG_CREATE_EPT_IOCTL:
      case RPMSG_CREATE_DEV_IOCTL:
        ret = rpmsg_char_create_eptdev(
          &rpmsg->chr, (FAR const struct rpmsg_endpoint_info *)arg);
        break;
      case RPMSG_RELEASE_DEV_IOCTL:
        rpmsg_char_destroy_eptdev(
          &rpmsg->chr, (FAR const struct rpmsg_endpoint_info *)arg);
        ret = OK;
        break;
#endif
      default:
        if (rpmsg->ops->ioctl)
          {
            ret = rpmsg->ops->ioctl(rpmsg, cmd, arg);
          }

        break;
    }

  return ret;
}

#if CONFIG_RPMSG_DUMP_LEVEL > RPMSG_DUMP_MANUAL
/****************************************************************************
 * Name: rpmsg_panic_notifier
 ****************************************************************************/

static int rpmsg_panic_notifier(FAR struct notifier_block *nb,
                                unsigned long action, FAR void *data)
{
  FAR struct rpmsg_s *rpmsg = container_of(nb, struct rpmsg_s, nbpanic);
  int ret = OK;

  if (action == PANIC_KERNEL_FINAL)
    {
      ret = rpmsg_dev_ioctl_(rpmsg, RPMSGIOC_DUMP,
              CONFIG_RPMSG_DUMP_LEVEL >= RPMSG_DUMP_PANIC_VERBOSE);
    }

  return ret;
}
#endif

static int rpmsg_dev_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR struct rpmsg_s *rpmsg =
    (FAR struct rpmsg_s *)filep->f_inode->i_private;

  return rpmsg_dev_ioctl_(rpmsg, cmd, arg);
}

static int rpmsg_init_workrx(FAR struct rpmsg_s *rpmsg, uint16_t nrx)
{
  FAR struct rpmsg_work_s *workrx;
  uint16_t i;
  int ret = OK;

  if (nrx != 0u)
    {
      workrx = kmm_zalloc(nrx * sizeof(struct rpmsg_work_s));
      if (workrx != NULL)
        {
          list_initialize(&rpmsg->freerx);
          for (i = 0; i < nrx; i++)
            {
              list_add_tail(&rpmsg->freerx, &workrx[i].work.node);
            }

          rpmsg->workrx = workrx;
        }
      else
        {
          ret = -ENOMEM;
        }
    }

  return ret;
}

static void rpmsg_deinit_workrx(FAR struct rpmsg_s *rpmsg)
{
  if (rpmsg->workrx)
    {
      kmm_free(rpmsg->workrx);
      rpmsg->workrx = NULL;
    }
}

static FAR struct rpmsg_wqueue_s *
rpmsg_get_current_wqueue(FAR struct rpmsg_s *rpmsg)
{
  FAR struct rpmsg_wqueue_s *wqueues = rpmsg_get_wqueues(rpmsg);
  FAR struct rpmsg_wqueue_s *wqueue = NULL;
  int i;

  for (i = 0; i < CONFIG_RPMSG_WQUEUE_NUMBER; i++)
    {
      if (work_queue_in_queue(wqueues[i].kwqueue))
        {
          wqueue = &wqueues[i];
          break;
        }
    }

  return wqueue;
}

static int rpmsg_create_wqueues(FAR struct rpmsg_wqueue_s *wqueues,
                                FAR const char *name, uint8_t priority)
{
  FAR struct rpmsg_wqueue_s *wqueue;
  char wqname[64];
  int ret = OK;
  int i;

  for (i = 0; i < CONFIG_RPMSG_WQUEUE_NUMBER; i++)
    {
      snprintf(wqname, sizeof(wqname), "rpmsg-%.*s-%d", RPMSG_NAME_SIZE,
               name, i);
      wqueue = &wqueues[i];
      wqueue->kwqueue = work_queue_create(wqname, priority, NULL,
                                          CONFIG_RPMSG_WQUEUE_STACKSIZE, 1);
      if (wqueue->kwqueue == NULL)
        {
          rpmsgerr("rpmsg wqueue [%d] create failed\n", i);
#if CONFIG_RPMSG_WQUEUE_NUMBER > 1
          while (--i >= 0)
            {
              work_queue_free(wqueues[i].kwqueue);
              wqueues[i].kwqueue = NULL;
            }
#endif

          ret = -ENOMEM;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: rpmsg_bind_alloc / rpmsg_bind_free
 *
 * Description:
 *   Allocate/free a name-service bind record holding extra_len bytes of
 *   service defined data.  When the rpmsg pool is enabled
 *   (RPMSG_POOL_COUNT > 0) and its block size is large enough to hold the
 *   record, it is taken from the rpmsg pool to avoid heap allocation.
 *   Otherwise it falls back to the heap.
 *
 *   Both must be called with the same extra_len so that they agree on the
 *   allocator: the pool does not check whether a block belongs to it, and
 *   releasing a heap pointer into it would corrupt it.  When
 *   RPMSG_POOL_COUNT <= 0 the rpmsg pool degenerates into alloca() and must
 *   not be used for these long-lived list nodes, so the heap is used
 *   unconditionally.
 *
 ****************************************************************************/

static inline_function FAR void *rpmsg_bind_alloc(size_t extra_len)
{
  size_t size = sizeof(struct rpmsg_bind_s) + extra_len;
  FAR void *bind;

#if CONFIG_RPMSG_POOL_COUNT > 0
  if (CONFIG_RPMSG_POOL_SIZE >= size)
    {
      bind = rpmsg_pool_alloc(size);
    }
  else
#endif
    {
      bind = kmm_malloc(size);
    }

  return bind;
}

static inline_function void rpmsg_bind_free(FAR void *bind, size_t extra_len)
{
#if CONFIG_RPMSG_POOL_COUNT > 0
  if (CONFIG_RPMSG_POOL_SIZE >= sizeof(struct rpmsg_bind_s) + extra_len)
    {
      rpmsg_pool_free(bind);
    }
  else
#endif
    {
      kmm_free(bind);
    }
}

static int rpmsg_register_callback_(FAR void *priv,
                                    rpmsg_dev_cb_t device_created,
                                    rpmsg_dev_cb_t device_destroy,
                                    rpmsg_match_cb_t ns_match,
                                    rpmsg_bind_cb_t ns_bind,
                                    rpmsg_bind_extra_cb_t ns_bind_extra)
{
  FAR struct rpmsg_bind_s *bind;
  FAR struct rpmsg_s *rpmsg;
  FAR struct rpmsg_cb_s *cb;
  int ret = -ENOMEM;
  bool again;

  cb = kmm_zalloc(sizeof(struct rpmsg_cb_s));
  if (cb != NULL)
    {
      cb->priv           = priv;
      cb->device_created = device_created;
      cb->device_destroy = device_destroy;
      cb->ns_match       = ns_match;
      cb->ns_bind        = ns_bind;
      cb->ns_bind_extra  = ns_bind_extra;

      down_write(&g_rpmsg_lock);
      dq_addlast(&cb->node, &g_rpmsg_cb);
      downgrade_write(&g_rpmsg_lock);
      dq_for_every_entry(&g_rpmsg, rpmsg, struct rpmsg_s, node)
        {
          FAR struct rpmsg_device *rdev = rpmsg_get_rdev_by_rpmsg(rpmsg);

          if (!rpmsg->init)
            {
              continue;
            }

          if (device_created)
            {
              device_created(rdev, priv);
            }

          if (ns_bind == NULL && ns_bind_extra == NULL)
            {
              continue;
            }

          DEBUGASSERT(ns_match != NULL);

          again = true;
          nxrmutex_lock(&rpmsg->lock);
          while (again)
            {
              again = false;
              list_for_every_entry(&rpmsg->bind, bind,
                                   struct rpmsg_bind_s, node)
                {
                  if (ns_match(rdev, priv, bind->name, bind->dest))
                    {
                      list_delete(&bind->node);
                      nxrmutex_unlock(&rpmsg->lock);

                      if (ns_bind_extra)
                        {
                          ns_bind_extra(rdev, priv, bind->name, bind->dest,
                                        bind->extra_len ?
                                        bind->extra : NULL,
                                        bind->extra_len);
                        }
                      else
                        {
                          ns_bind(rdev, priv, bind->name, bind->dest);
                        }

                      rpmsg_bind_free(bind, bind->extra_len);
                      again = true;

                      nxrmutex_lock(&rpmsg->lock);
                      break;
                    }
                }
            }

          nxrmutex_unlock(&rpmsg->lock);
        }

      up_read(&g_rpmsg_lock);
      ret = 0;
    }

  return ret;
}

static void rpmsg_unregister_callback_(FAR void *priv,
                                       rpmsg_dev_cb_t device_created,
                                       rpmsg_dev_cb_t device_destroy,
                                       rpmsg_match_cb_t ns_match,
                                       rpmsg_bind_cb_t ns_bind,
                                       rpmsg_bind_extra_cb_t ns_bind_extra)
{
  FAR struct rpmsg_cb_s *cb;
  FAR struct rpmsg_s *rpmsg;

  down_write(&g_rpmsg_lock);
  dq_for_every_entry(&g_rpmsg_cb, cb, struct rpmsg_cb_s, node)
    {
      if (cb->priv == priv &&
          cb->device_created == device_created &&
          cb->device_destroy == device_destroy &&
          cb->ns_match == ns_match &&
          cb->ns_bind == ns_bind &&
          cb->ns_bind_extra == ns_bind_extra)
        {
          dq_rem(&cb->node, &g_rpmsg_cb);
          kmm_free(cb);
          break;
        }
    }

  downgrade_write(&g_rpmsg_lock);
  if (device_destroy)
    {
      dq_for_every_entry(&g_rpmsg, rpmsg, struct rpmsg_s, node)
        {
          FAR struct rpmsg_device *rdev = rpmsg_get_rdev_by_rpmsg(rpmsg);

          if (rpmsg->init)
            {
              device_destroy(rdev, priv);
            }
        }
    }

  up_read(&g_rpmsg_lock);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void rpmsg_rx_worker(FAR void *arg)
{
  FAR struct rpmsg_work_s *work = arg;
  FAR struct rpmsg_s *rpmsg = work->rpmsg;
  FAR struct rpmsg_device *rdev;
  rpmsg_worker_t worker = work->worker;

  arg = work->arg;
  rdev = rpmsg_get_rdev_by_rpmsg(rpmsg);
  metal_mutex_acquire(&rdev->lock);
  list_add_tail(&work->rpmsg->freerx, &work->work.node);
  metal_mutex_release(&rdev->lock);
  DEBUGASSERT(worker != NULL);
  worker(rpmsg, arg);
}

void rpmsg_initialize(void)
{
  rpmsg_procfs_initialize();
#ifdef CONFIG_RPMSG_WQUEUE_GLOBAL
  DEBUGVERIFY(rpmsg_create_wqueues(g_rpmsg_wqueues, "glb",
              CONFIG_RPMSG_WQUEUE_PRIORITY));
#endif
}

int rpmsg_tickwait(FAR struct rpmsg_endpoint *ept, FAR sem_t *sem,
                   clock_t delay)
{
  FAR struct rpmsg_wqueue_s *wqueue = NULL;
  FAR struct rpmsg_s *rpmsg;
  clock_t end;
  int ret = -EINVAL;

  if (ept && sem)
    {
      rpmsg = rpmsg_get_by_rdev(ept->rdev);
      if (rpmsg && rpmsg->ops->wait)
        {
          wqueue = rpmsg_get_current_wqueue(rpmsg);
        }

      if (wqueue == NULL)
        {
          ret = delay >= WDOG_MAX_DELAY ?
                nxsem_wait_uninterruptible(sem) :
                nxsem_tickwait_uninterruptible(sem, delay);
        }
      else
        {
          end = delay >= WDOG_MAX_DELAY ? 0 :
                clock_systime_ticks() + delay;
          for (; ; )
            {
              ret = nxsem_trywait(sem);
              if (ret >= 0)
                {
                  break;
                }

              if (delay < WDOG_MAX_DELAY)
                {
                  clock_t now = clock_systime_ticks();

                  if (clock_compare(end, now))
                    {
                      ret = -ETIMEDOUT;
                      break;
                    }

                  delay = end - now;
                }

              /* Keep running the queued works until the semaphore is
               * posted: a deeper level than the limit only waits, so the
               * wait still ends.
               */

              rpmsg->ops->wait(rpmsg, delay);
              if (wqueue->recursive < CONFIG_RPMSG_RECURSIVE_LIMIT)
                {
                  wqueue->recursive++;
                  work_queue_dispatch(wqueue->kwqueue);
                  wqueue->recursive--;
                }
            }
        }
    }

  return ret;
}

int rpmsg_post(FAR struct rpmsg_endpoint *ept, FAR sem_t *sem)
{
  FAR struct rpmsg_s *rpmsg;
  int semcount = 0;
  int ret = -EINVAL;

  if (ept && sem)
    {
      rpmsg = rpmsg_get_by_rdev(ept->rdev);
      if (!rpmsg || !rpmsg->ops->post)
        {
          ret = nxsem_post(sem);
        }
      else
        {
          nxsem_get_value(sem, &semcount);
          ret = nxsem_post(sem);
          if (semcount >= 0)
            {
              ret = rpmsg->ops->post(rpmsg);
            }
        }
    }

  return ret;
}

FAR const char *rpmsg_get_local_cpuname(FAR struct rpmsg_device *rdev)
{
  FAR struct rpmsg_s *rpmsg = rpmsg_get_by_rdev(rdev);
  FAR const char *cpuname = NULL;

  if (rpmsg != NULL)
    {
      cpuname = rpmsg->local_cpuname[0] ? rpmsg->local_cpuname :
                CONFIG_RPMSG_LOCAL_CPUNAME;
    }

  return cpuname;
}

FAR const char *rpmsg_get_cpuname(FAR struct rpmsg_device *rdev)
{
  FAR struct rpmsg_s *rpmsg = rpmsg_get_by_rdev(rdev);

  return rpmsg ? rpmsg->cpuname : NULL;
}

int rpmsg_get_signals(FAR struct rpmsg_device *rdev)
{
  FAR struct rpmsg_s *rpmsg = rpmsg_get_by_rdev(rdev);

  return atomic_read(&rpmsg->signals);
}

int rpmsg_get_timestamp(FAR struct rpmsg_device *rdev, FAR const void *data,
                        FAR struct rpmsg_timestamp_s *ts)
{
  FAR struct rpmsg_s *rpmsg = rpmsg_get_by_rdev(rdev);
  int ret = -EINVAL;

  if (rpmsg && data && ts && rpmsg->ops->get_timestamp)
    {
      ret = rpmsg->ops->get_timestamp(rpmsg, data, ts);
    }

  return ret;
}

bool rpmsg_support_alloc_buf(FAR struct rpmsg_device *rdev)
{
  FAR struct rpmsg_s *rpmsg = rpmsg_get_by_rdev(rdev);

  return rpmsg && rpmsg->ops->alloc_buf && rpmsg->ops->free_buf;
}

FAR void *rpmsg_alloc_buf(FAR struct rpmsg_device *rdev, size_t size,
                          size_t align)
{
  FAR struct rpmsg_s *rpmsg = rpmsg_get_by_rdev(rdev);
  FAR void *buf = NULL;

  if (rpmsg && rpmsg->ops->alloc_buf)
    {
      buf = rpmsg->ops->alloc_buf(rpmsg, size, align);
    }

  return buf;
}

void rpmsg_free_buf(FAR struct rpmsg_device *rdev, FAR void *addr)
{
  FAR struct rpmsg_s *rpmsg = rpmsg_get_by_rdev(rdev);

  if (rpmsg && rpmsg->ops->free_buf)
    {
      rpmsg->ops->free_buf(rpmsg, addr);
    }
}

#if CONFIG_RPMSG_POOL_COUNT > 0
FAR void *rpmsg_pool_alloc(size_t size)
{
  DEBUGASSERT(size <= CONFIG_RPMSG_POOL_SIZE);
  return mempool_allocate(&g_rpmsg_pool, UINT_MAX);
}

void rpmsg_pool_free(FAR void *addr)
{
  mempool_release(&g_rpmsg_pool, addr);
}
#endif

int rpmsg_register_callback(FAR void *priv,
                            rpmsg_dev_cb_t device_created,
                            rpmsg_dev_cb_t device_destroy,
                            rpmsg_match_cb_t ns_match,
                            rpmsg_bind_cb_t ns_bind)
{
  return rpmsg_register_callback_(priv, device_created, device_destroy,
                                  ns_match, ns_bind, NULL);
}

void rpmsg_unregister_callback(FAR void *priv,
                               rpmsg_dev_cb_t device_created,
                               rpmsg_dev_cb_t device_destroy,
                               rpmsg_match_cb_t ns_match,
                               rpmsg_bind_cb_t ns_bind)
{
  rpmsg_unregister_callback_(priv, device_created, device_destroy,
                             ns_match, ns_bind, NULL);
}

int rpmsg_register_callback_extra(FAR void *priv,
                                  rpmsg_dev_cb_t device_created,
                                  rpmsg_dev_cb_t device_destroy,
                                  rpmsg_match_cb_t ns_match,
                                  rpmsg_bind_extra_cb_t ns_bind_extra)
{
  return rpmsg_register_callback_(priv, device_created, device_destroy,
                                  ns_match, NULL, ns_bind_extra);
}

void rpmsg_unregister_callback_extra(FAR void *priv,
                                     rpmsg_dev_cb_t device_created,
                                     rpmsg_dev_cb_t device_destroy,
                                     rpmsg_match_cb_t ns_match,
                                     rpmsg_bind_extra_cb_t ns_bind_extra)
{
  rpmsg_unregister_callback_(priv, device_created, device_destroy,
                             ns_match, NULL, ns_bind_extra);
}

void rpmsg_ns_bind_extra(FAR struct rpmsg_device *rdev,
                         FAR const char *name, uint32_t dest,
                         FAR const void *extra, size_t extra_len)
{
  FAR struct rpmsg_s *rpmsg = rpmsg_get_by_rdev(rdev);
  FAR struct rpmsg_bind_s *bind;
  FAR struct rpmsg_cb_s *cb;
  bool matched = false;

  down_read(&g_rpmsg_lock);
  dq_for_every_entry(&g_rpmsg_cb, cb, struct rpmsg_cb_s, node)
    {
      if (cb->ns_match && cb->ns_match(rdev, cb->priv, name, dest))
        {
          rpmsg_bind_extra_cb_t ns_bind_extra = cb->ns_bind_extra;
          rpmsg_bind_cb_t ns_bind = cb->ns_bind;
          FAR void *cb_priv = cb->priv;

          up_read(&g_rpmsg_lock);

          if (ns_bind_extra)
            {
              ns_bind_extra(rdev, cb_priv, name, dest, extra, extra_len);
            }
          else
            {
              ns_bind(rdev, cb_priv, name, dest);
            }

          matched = true;
          break;
        }
    }

  if (!matched)
    {
      bind = rpmsg_bind_alloc(extra_len);
      if (bind != NULL)
        {
          bind->dest = dest;
          bind->extra_len = extra_len;
          strlcpy(bind->name, name, RPMSG_NAME_SIZE);
          if (extra_len > 0)
            {
              memcpy(bind->extra, extra, extra_len);
            }

          nxrmutex_lock(&rpmsg->lock);
          list_add_tail(&rpmsg->bind, &bind->node);
          nxrmutex_unlock(&rpmsg->lock);
        }

      up_read(&g_rpmsg_lock);
    }
}

void rpmsg_ns_unbind(FAR struct rpmsg_device *rdev,
                     FAR const char *name, uint32_t dest)
{
  FAR struct rpmsg_s *rpmsg = rpmsg_get_by_rdev(rdev);
  FAR struct rpmsg_bind_s *bind;

  nxrmutex_lock(&rpmsg->lock);
  list_for_every_entry(&rpmsg->bind, bind, struct rpmsg_bind_s, node)
    {
      if (bind->dest == dest && !strncmp(bind->name, name, RPMSG_NAME_SIZE))
        {
          list_delete(&bind->node);
          rpmsg_bind_free(bind, bind->extra_len);
          break;
        }
    }

  nxrmutex_unlock(&rpmsg->lock);
}

void rpmsg_device_created(FAR struct rpmsg_s *rpmsg)
{
  FAR struct rpmsg_device *rdev = rpmsg_get_rdev_by_rpmsg(rpmsg);
  FAR struct rpmsg_cb_s *tmp;
  FAR struct rpmsg_cb_s *cb;

  down_write(&g_rpmsg_lock);
  dq_for_every_entry_safe(&g_rpmsg_cb, cb, tmp, struct rpmsg_cb_s, node)
    {
      if (cb->device_created)
        {
          cb->device_created(rdev, cb->priv);
        }
    }

  rpmsg->init = true;
  up_write(&g_rpmsg_lock);

#ifdef CONFIG_RPMSG_PING
  rpmsg_ping_init(rdev, &rpmsg->ping);
#endif
#ifdef CONFIG_RPMSG_TEST
  rpmsg_test_init(rdev, &rpmsg->test);
#endif
#ifdef CONFIG_RPMSG_WAKELOCK
  rpmsg_wakelock_init(rdev, &rpmsg->wakelock);
#endif
#ifdef CONFIG_RPMSG_HEAP
  rpmsg_heap_init(rdev, &rpmsg->heap);
#endif
}

void rpmsg_device_destroy(FAR struct rpmsg_s *rpmsg)
{
  FAR struct rpmsg_device *rdev = rpmsg_get_rdev_by_rpmsg(rpmsg);
  FAR struct rpmsg_bind_s *bind_tmp;
  FAR struct rpmsg_bind_s *bind;
  FAR struct rpmsg_cb_s *cb_tmp;
  FAR struct rpmsg_cb_s *cb;
  FAR struct rpmsg_endpoint *ept;
  FAR struct metal_list *node;
  FAR struct metal_list *tmp;

#ifdef CONFIG_RPMSG_PING
  rpmsg_ping_deinit(&rpmsg->ping);
#endif

#ifdef CONFIG_RPMSG_TEST
  rpmsg_test_deinit(&rpmsg->test);
#endif

#ifdef CONFIG_RPMSG_WAKELOCK
  rpmsg_wakelock_deinit(&rpmsg->wakelock);
#endif

#ifdef CONFIG_RPMSG_HEAP
  rpmsg_heap_deinit(&rpmsg->heap);
#endif

  nxrmutex_lock(&rpmsg->lock);
  list_for_every_entry_safe(&rpmsg->bind, bind, bind_tmp,
                            struct rpmsg_bind_s, node)
    {
      list_delete(&bind->node);
      rpmsg_bind_free(bind, bind->extra_len);
    }

  nxrmutex_unlock(&rpmsg->lock);

  /* Broadcast device_destroy to all registers */

  down_write(&g_rpmsg_lock);
  rpmsg->init = false;
  downgrade_write(&g_rpmsg_lock);
  dq_for_every_entry_safe(&g_rpmsg_cb, cb, cb_tmp, struct rpmsg_cb_s, node)
    {
      if (cb->device_destroy)
        {
          cb->device_destroy(rdev, cb->priv);
        }
    }

  up_read(&g_rpmsg_lock);

  /* Release all ept attached to current rpmsg device.  rdev->lock is not
   * held: rpmsg_destroy_ept() sends NS_DESTROY and may wait for a tx buffer.
   */

  metal_list_for_each_safe(&rdev->endpoints, tmp, node)
    {
      ept = container_of(node, struct rpmsg_endpoint, node);
      if (ept->ns_unbind_cb)
        {
          /* Unregister first so ept->rdev cannot outlive the device.
           * Without release_cb the callback may free ept, so it is not
           * touched afterwards.
           */

          if (ept->release_cb)
            {
              rpmsg_ept_incref(ept);
              rpmsg_unregister_endpoint(ept);
              ept->ns_unbind_cb(ept);
              rpmsg_ept_decref(ept);
            }
          else
            {
              rpmsg_unregister_endpoint(ept);
              ept->ns_unbind_cb(ept);
            }
        }
      else
        {
          rpmsg_destroy_ept(ept);
        }
    }
}

int rpmsg_register(FAR const char *path, FAR struct rpmsg_s *rpmsg,
                   FAR const struct rpmsg_ops_s *ops, uint16_t nrx)
{
  static const struct file_operations g_rpmsg_dev_ops =
  {
    NULL,             /* open */
    NULL,             /* close */
    NULL,             /* read */
    NULL,             /* write */
    NULL,             /* seek */
    rpmsg_dev_ioctl,  /* ioctl */
  };

  struct metal_init_params params = METAL_INIT_DEFAULTS;
  int ret;

  ret = metal_init(&params);
  if (ret >= 0)
    {
      rpmsg_char_init(&rpmsg->chr);

      ret = register_driver(path, &g_rpmsg_dev_ops, 0200, rpmsg);
      if (ret >= 0)
        {
          ret = rpmsg_init_workrx(rpmsg, nrx);
          if (ret >= 0)
            {
              list_initialize(&rpmsg->bind);
              nxrmutex_init(&rpmsg->lock);
              rpmsg->ops = ops;
              atomic_or(&rpmsg->signals, RPMSG_SIGNAL_RUNNING);

              /* Add priv to list */

              down_write(&g_rpmsg_lock);
              dq_addlast(&rpmsg->node, &g_rpmsg);
              up_write(&g_rpmsg_lock);

#if CONFIG_RPMSG_DUMP_LEVEL > RPMSG_DUMP_MANUAL
              rpmsg->nbpanic.notifier_call = rpmsg_panic_notifier;
              rpmsg->nbpanic.priority = INT_MIN + 2;
              panic_notifier_chain_register(&rpmsg->nbpanic);
#endif
            }
          else
            {
              unregister_driver(path);
              rpmsg_char_deinit(&rpmsg->chr);
              metal_finish();
            }
        }
      else
        {
          rpmsg_char_deinit(&rpmsg->chr);
          metal_finish();
        }
    }

  return ret;
}

void rpmsg_unregister(FAR const char *path, FAR struct rpmsg_s *rpmsg)
{
#if CONFIG_RPMSG_DUMP_LEVEL > RPMSG_DUMP_MANUAL
  panic_notifier_chain_unregister(&rpmsg->nbpanic);
#endif

  down_write(&g_rpmsg_lock);
  dq_rem(&rpmsg->node, &g_rpmsg);
  up_write(&g_rpmsg_lock);

  rpmsg_deinit_workrx(rpmsg);
  nxrmutex_destroy(&rpmsg->lock);
  unregister_driver(path);
  rpmsg_char_deinit(&rpmsg->chr);
  metal_finish();
}

#if CONFIG_RPMSG_DUMP_LEVEL > RPMSG_DUMP_DISABLE
/****************************************************************************
 * Name: rpmsg_dump_epts
 ****************************************************************************/

void rpmsg_dump_epts(FAR struct rpmsg_device *rdev)
{
  FAR struct rpmsg_endpoint *ept;
  FAR struct metal_list *node;

  syslog(LOG_EMERG, "  rpmsg ept list:\n");
  metal_list_for_each(&rdev->endpoints, node)
    {
      ept = container_of(node, struct rpmsg_endpoint, node);
      syslog(LOG_EMERG, "    ept %s %p: addr=%" PRIu32 " "
             "dest=%" PRIu32 " refcnt=%u priority=%" PRIu8
             " priv=%p\n", ept->name, ept, ept->addr, ept->dest_addr,
             atomic_load(&ept->refcnt), ept->priority, ept->priv);
    }
}
#endif

static int rpmsg_ioctl_foreach_cb(FAR struct rpmsg_s *rpmsg, FAR void *arg)
{
  FAR struct rpmsg_ioctl_s *info = (FAR struct rpmsg_ioctl_s *)arg;
  int ret = OK;

  if (!info->cpuname || !strcmp(rpmsg->cpuname, info->cpuname))
    {
      ret = rpmsg_dev_ioctl_(rpmsg, info->cmd, info->arg);
    }

  return ret;
}

int rpmsg_ioctl(FAR const char *cpuname, int cmd, unsigned long arg)
{
  struct rpmsg_ioctl_s info;

  info.cpuname = cpuname;
  info.cmd     = cmd;
  info.arg     = arg;

  return rpmsg_foreach(rpmsg_ioctl_foreach_cb, &info);
}

int rpmsg_panic(FAR const char *cpuname)
{
  return rpmsg_ioctl(cpuname, RPMSGIOC_PANIC, 0);
}

void rpmsg_dump_all(void)
{
  rpmsg_ioctl(NULL, RPMSGIOC_DUMP, 0);
}

int rpmsg_foreach(rpmsg_foreach_t handler, FAR void *args)
{
  bool needlock = !up_interrupt_context() && !sched_idletask();
  FAR struct rpmsg_s *rpmsg;
  int ret = OK;

  if (needlock)
    {
      down_read(&g_rpmsg_lock);
    }

  dq_for_every_entry(&g_rpmsg, rpmsg, struct rpmsg_s, node)
    {
      ret = handler(rpmsg, args);
      if (ret < 0)
        {
          break;
        }
    }

  if (needlock)
    {
      up_read(&g_rpmsg_lock);
    }

  return ret;
}

void rpmsg_modify_signals(FAR struct rpmsg_s *rpmsg,
                          int setflags, int clrflags)
{
  FAR struct rpmsg_device *rdev = rpmsg_get_rdev_by_rpmsg(rpmsg);
  FAR struct rpmsg_endpoint *ept;
  FAR struct metal_list *node;
  bool needlock;

  atomic_and_acquire(&rpmsg->signals, ~clrflags);
  atomic_or_acquire(&rpmsg->signals, setflags);

  /* Send signal to Router Hub */

  needlock = !up_interrupt_context() && !sched_idletask();
  if (needlock)
    {
      metal_mutex_acquire(&rdev->lock);
    }

  metal_list_for_each(&rdev->endpoints, node)
    {
      ept = container_of(node, struct rpmsg_endpoint, node);
      if (!strncmp(ept->name, RPMSG_ROUTER_NAME,
                   RPMSG_ROUTER_NAME_LEN))
        {
          rpmsg_ept_incref(ept);
          ept->cb(ept, NULL, 0, RPMSG_ADDR_ANY, NULL);
          rpmsg_ept_decref(ept);
        }
    }

  if (needlock)
    {
      metal_mutex_release(&rdev->lock);
    }
}

int rpmsg_init_wqueues(FAR struct rpmsg_s *rpmsg, uint8_t priority)
{
  rpmsg->priority = priority;

#ifndef CONFIG_RPMSG_WQUEUE_GLOBAL
  return rpmsg_create_wqueues(rpmsg->wqueues, rpmsg->cpuname, priority);
#endif
  return OK;
}

void rpmsg_deinit_wqueues(FAR struct rpmsg_s *rpmsg)
{
#ifndef CONFIG_RPMSG_WQUEUE_GLOBAL
  int i;

  for (i = 0; i < CONFIG_RPMSG_WQUEUE_NUMBER; i++)
    {
      if (rpmsg->wqueues[i].kwqueue != NULL)
        {
          work_queue_free(rpmsg->wqueues[i].kwqueue);
        }
    }
#endif
}

int rpmsg_queue_work(FAR struct rpmsg_s *rpmsg, int priority,
                     FAR struct work_s *work, worker_t worker, FAR void *arg)
{
  FAR struct rpmsg_wqueue_s *wqueues = rpmsg_get_wqueues(rpmsg);
#ifdef CONFIG_RPMSG_WQUEUE_GLOBAL
  irqstate_t flags;
#endif
  int idx = 0;
  int i;

  priority = priority - RPMSG_PRIO_DEFAULT + rpmsg->priority;
  priority = CLAMP(priority, SCHED_PRIORITY_MIN, SCHED_PRIORITY_MAX);

#ifdef CONFIG_RPMSG_WQUEUE_GLOBAL
  flags = spin_lock_irqsave(&g_rpmsg_wqueue_lock);
#endif
  for (i = 0; i < CONFIG_RPMSG_WQUEUE_NUMBER; i++)
    {
      if (priority == wqueues[i].priority)
        {
          idx = i;
          break;
        }

      if (wqueues[i].priority == 0)
        {
          DEBUGVERIFY(work_queue_setpriority_wq(wqueues[i].kwqueue,
                                                priority));
          wqueues[i].priority = priority;
          idx = i;
          break;
        }

      if (i != idx && abs(priority - wqueues[i].priority) <
          abs(priority - wqueues[idx].priority))
        {
          idx = i;
        }
    }

#ifdef CONFIG_RPMSG_WQUEUE_GLOBAL
  spin_unlock_irqrestore(&g_rpmsg_wqueue_lock, flags);
#endif

  return work_queue_wq(wqueues[idx].kwqueue, work, worker, arg, 0);
}
