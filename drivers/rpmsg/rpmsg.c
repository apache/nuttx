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
#include <syslog.h>
#include <time.h>

#include <metal/sys.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/list.h>
#include <nuttx/mm/mempool.h>
#include <nuttx/mutex.h>
#include <nuttx/nuttx.h>
#include <nuttx/rwsem.h>
#include <nuttx/semaphore.h>
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

#define RPMSG_RECURSIVE_LIMIT    8
#define RPMSG_DEFAULT_WQUEUE_IDX (CONFIG_RPMSG_WQUEUE_NUMBER / 2)

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
};

struct rpmsg_cb_s
{
  FAR void          *priv;
  rpmsg_dev_cb_t    device_created;
  rpmsg_dev_cb_t    device_destroy;
  rpmsg_match_cb_t  ns_match;
  rpmsg_bind_cb_t   ns_bind;
  struct list_node  node;
};

struct rpmsg_ioctl_s
{
  FAR const char *cpuname;
  int             cmd;
  unsigned long   arg;
};

struct rpmsg_work_s
{
  struct work_s       work;
  rpmsg_worker_t      worker;
  FAR struct rpmsg_s *rpmsg;
  FAR void           *arg;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rpmsg_dev_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct list_node g_rpmsg_cb = LIST_INITIAL_VALUE(g_rpmsg_cb);
static struct list_node g_rpmsg = LIST_INITIAL_VALUE(g_rpmsg);

static rw_semaphore_t g_rpmsg_lock = RWSEM_INITIALIZER;

#if CONFIG_RPMSG_POOL_COUNT > 0
MEMPOOL_DEFINE(g_rpmsg_pool, CONFIG_RPMSG_POOL_SIZE,
               CONFIG_RPMSG_POOL_COUNT, CONFIG_RPMSG_POOL_COUNT, 0);
#endif

#ifdef CONFIG_RPMSG_WQUEUE_GLOBAL
static struct rpmsg_wqueue_s g_rpmsg_wqueues[CONFIG_RPMSG_WQUEUE_NUMBER];
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
      case RPMSGIOC_DUMP:
        if (rpmsg->ops->dump)
          {
            rpmsg->ops->dump(rpmsg, (bool)arg);
            ret = OK;
          }

        break;
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

#if CONFIG_RPMSG_PANIC_DUMP_LEVEL > RPMSG_PANIC_DUMP_DISABLE
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
              CONFIG_RPMSG_PANIC_DUMP_LEVEL >= RPMSG_PANIC_DUMP_VERBOSE);
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

static void rpmsg_rx_worker(FAR void *arg)
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

static int rpmsg_create_wqueues(FAR struct rpmsg_wqueue_s *wqueues,
                                FAR const char *name)
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
      wqueue->kwqueue = work_queue_create(wqname,
                                          CONFIG_RPMSG_WQUEUE_PRIORITY + i,
                                          NULL,
                                          CONFIG_RPMSG_WQUEUE_STACKSIZE,
                                          1);
      if (wqueue->kwqueue == NULL)
        {
          rpmsgerr("rpmsg wqueue [%d] create failed\n", i);
          while (--i >= 0)
            {
              work_queue_free(wqueues[i].kwqueue);
            }

          ret = -ENOMEM;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void rpmsg_initialize(void)
{
  rpmsg_procfs_initialize();
#ifdef CONFIG_RPMSG_WQUEUE_GLOBAL
  rpmsg_create_wqueues(g_rpmsg_wqueues, "glb");
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
              if (wqueue->recursive < RPMSG_RECURSIVE_LIMIT)
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

      down_write(&g_rpmsg_lock);
      list_add_tail(&g_rpmsg_cb, &cb->node);
      downgrade_write(&g_rpmsg_lock);
      list_for_every_entry(&g_rpmsg, rpmsg, struct rpmsg_s, node)
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

          if (ns_bind == NULL)
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
                      ns_bind(rdev, priv, bind->name, bind->dest);

                      kmm_free(bind);
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

void rpmsg_unregister_callback(FAR void *priv,
                               rpmsg_dev_cb_t device_created,
                               rpmsg_dev_cb_t device_destroy,
                               rpmsg_match_cb_t ns_match,
                               rpmsg_bind_cb_t ns_bind)
{
  FAR struct rpmsg_cb_s *cb;
  FAR struct rpmsg_s *rpmsg;

  down_write(&g_rpmsg_lock);
  list_for_every_entry(&g_rpmsg_cb, cb, struct rpmsg_cb_s, node)
    {
      if (cb->priv == priv &&
          cb->device_created == device_created &&
          cb->device_destroy == device_destroy &&
          cb->ns_match == ns_match &&
          cb->ns_bind == ns_bind)
        {
          list_delete(&cb->node);
          kmm_free(cb);
          break;
        }
    }

  downgrade_write(&g_rpmsg_lock);
  if (device_destroy)
    {
      list_for_every_entry(&g_rpmsg, rpmsg, struct rpmsg_s, node)
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

void rpmsg_ns_bind(FAR struct rpmsg_device *rdev,
                   FAR const char *name, uint32_t dest)
{
  FAR struct rpmsg_s *rpmsg = rpmsg_get_by_rdev(rdev);
  FAR struct rpmsg_bind_s *bind;
  FAR struct rpmsg_cb_s *cb;
  bool matched = false;

  down_read(&g_rpmsg_lock);
  list_for_every_entry(&g_rpmsg_cb, cb, struct rpmsg_cb_s, node)
    {
      if (cb->ns_match && cb->ns_match(rdev, cb->priv, name, dest))
        {
          rpmsg_bind_cb_t ns_bind = cb->ns_bind;
          FAR void *cb_priv = cb->priv;

          up_read(&g_rpmsg_lock);

          ns_bind(rdev, cb_priv, name, dest);
          matched = true;
          break;
        }
    }

  if (!matched)
    {
      bind = kmm_malloc(sizeof(struct rpmsg_bind_s));
      if (bind != NULL)
        {
          bind->dest = dest;
          strlcpy(bind->name, name, RPMSG_NAME_SIZE);

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
          kmm_free(bind);
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
  list_for_every_entry_safe(&g_rpmsg_cb, cb, tmp, struct rpmsg_cb_s, node)
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
      kmm_free(bind);
    }

  nxrmutex_unlock(&rpmsg->lock);

  /* Broadcast device_destroy to all registers */

  down_write(&g_rpmsg_lock);
  rpmsg->init = false;
  downgrade_write(&g_rpmsg_lock);
  list_for_every_entry_safe(&g_rpmsg_cb, cb, cb_tmp, struct rpmsg_cb_s, node)
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
          ept->ns_unbind_cb(ept);
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
              atomic_set(&rpmsg->signals, RPMSG_SIGNAL_RUNNING);

              /* Add priv to list */

              down_write(&g_rpmsg_lock);
              list_add_tail(&g_rpmsg, &rpmsg->node);
              up_write(&g_rpmsg_lock);

#if CONFIG_RPMSG_PANIC_DUMP_LEVEL > RPMSG_PANIC_DUMP_DISABLE
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
#if CONFIG_RPMSG_PANIC_DUMP_LEVEL > RPMSG_PANIC_DUMP_DISABLE
  panic_notifier_chain_unregister(&rpmsg->nbpanic);
#endif

  down_write(&g_rpmsg_lock);
  list_delete(&rpmsg->node);
  up_write(&g_rpmsg_lock);

  rpmsg_deinit_workrx(rpmsg);
  nxrmutex_destroy(&rpmsg->lock);
  unregister_driver(path);
  rpmsg_char_deinit(&rpmsg->chr);
  metal_finish();
}

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
             "dest=%" PRIu32 " refcnt=%" PRIu32 " priority=%" PRIu8
             " priv=%p\n", ept->name, ept, ept->addr, ept->dest_addr,
             ept->refcnt, ept->priority, ept->priv);
    }
}

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

  list_for_every_entry(&g_rpmsg, rpmsg, struct rpmsg_s, node)
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

#ifndef CONFIG_RPMSG_WQUEUE_GLOBAL
int rpmsg_init_wqueues(FAR struct rpmsg_s *rpmsg)
{
  return rpmsg_create_wqueues(rpmsg->wqueues, rpmsg->cpuname);
}

void rpmsg_deinit_wqueues(FAR struct rpmsg_s *rpmsg)
{
  int i;

  for (i = 0; i < CONFIG_RPMSG_WQUEUE_NUMBER; i++)
    {
      if (rpmsg->wqueues[i].kwqueue != NULL)
        {
          work_queue_free(rpmsg->wqueues[i].kwqueue);
        }
    }
}
#endif

int rpmsg_queue_work(FAR struct rpmsg_s *rpmsg, uint8_t priority,
                     FAR struct work_s *work, worker_t worker, FAR void *arg)
{
  FAR struct rpmsg_wqueue_s *wqueues = rpmsg_get_wqueues(rpmsg);
  int idx;

  idx = (int)priority - RPMSG_PRIO_DEFAULT + RPMSG_DEFAULT_WQUEUE_IDX;
  if (idx < 0)
    {
      idx = 0;
    }
  else if (idx >= CONFIG_RPMSG_WQUEUE_NUMBER)
    {
      idx = CONFIG_RPMSG_WQUEUE_NUMBER - 1;
    }

  return work_queue_wq(wqueues[idx].kwqueue, work, worker, arg, 0);
}

void rpmsg_queue_rx_work(FAR struct rpmsg_s *rpmsg, uint8_t priority,
                         rpmsg_worker_t worker, FAR void *arg)
{
  FAR struct rpmsg_work_s *work;

  /* Called by the transport rx dispatch with rdev->lock held */

  work = list_remove_head_type(&rpmsg->freerx, struct rpmsg_work_s,
                               work.node);
  DEBUGASSERT(work != NULL);
  work->arg = arg;
  work->rpmsg = rpmsg;
  work->worker = worker;

  rpmsg_queue_work(rpmsg, priority, &work->work, rpmsg_rx_worker, work);
}
