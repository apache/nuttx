/****************************************************************************
 * drivers/rpmsg/rpmsg.h
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

#ifndef __DRIVERS_RPMSG_RPMSG_H
#define __DRIVERS_RPMSG_RPMSG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifdef CONFIG_RPMSG

#include <nuttx/list.h>
#include <nuttx/panic_notifier.h>
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/wqueue.h>

#include "rpmsg_char.h"
#include "rpmsg_ping.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RPMSG_PANIC_DUMP_DISABLE 0
#define RPMSG_PANIC_DUMP_INFO    1
#define RPMSG_PANIC_DUMP_VERBOSE 2

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Forward declarations */

struct rpmsg_s;

CODE typedef int (*rpmsg_foreach_t)(FAR struct rpmsg_s *rpmsg,
                                    FAR void *args);
CODE typedef void (*rpmsg_worker_t)(FAR struct rpmsg_s *rpmsg,
                                    FAR void *arg);

struct rpmsg_wqueue_s
{
  FAR struct kwork_wqueue_s    *kwqueue;
  int                          recursive;
  int                          priority;
};

struct rpmsg_work_s
{
  struct work_s                work;
  FAR struct rpmsg_s           *rpmsg;
  rpmsg_worker_t               worker;
  FAR void                     *arg;
};

struct rpmsg_s
{
  bool                         init;
  struct list_node             bind;
  rmutex_t                     lock;
  struct list_node             node;
  char                         local_cpuname[RPMSG_NAME_SIZE];
  char                         cpuname[RPMSG_NAME_SIZE];
  FAR const struct rpmsg_ops_s *ops;
#ifndef CONFIG_RPMSG_WQUEUE_GLOBAL
  struct rpmsg_wqueue_s        wqueues[CONFIG_RPMSG_WQUEUE_NUMBER];
#endif
  int                          priority;
  FAR void                     *workrx;
  struct list_node             freerx;
#ifdef CONFIG_RPMSG_PING
  struct rpmsg_ping_dev_s      ping;
#endif
#ifdef CONFIG_RPMSG_TEST
  struct rpmsg_endpoint        test;
#endif
#ifdef CONFIG_RPMSG_WAKELOCK
  struct rpmsg_endpoint        wakelock;
#endif
#ifdef CONFIG_RPMSG_HEAP
  struct rpmsg_endpoint        heap;
#endif
#ifdef CONFIG_RPMSG_CHAR
  struct rpmsg_char_s          chr;
#endif
  atomic_t                     signals;
#if CONFIG_RPMSG_PANIC_DUMP_LEVEL > RPMSG_PANIC_DUMP_DISABLE
  struct notifier_block        nbpanic;
#endif
};

/**
 * struct rpmsg_ops_s - Rpmsg device operations
 * wait: wait sem.
 * post: post sem.
 * get_cpuname: get cpu name.
 */

struct rpmsg_ops_s
{
  CODE int (*wait)(FAR struct rpmsg_s *rpmsg, clock_t delay);
  CODE int (*post)(FAR struct rpmsg_s *rpmsg);
  CODE int (*ioctl)(FAR struct rpmsg_s *rpmsg, int cmd, unsigned long arg);
  CODE void (*panic)(FAR struct rpmsg_s *rpmsg);
  CODE void (*dump)(FAR struct rpmsg_s *rpmsg, bool verbose);
  CODE int (*get_timestamp)(FAR struct rpmsg_s *rpmsg, FAR const void *data,
                            FAR struct rpmsg_timestamp_s *ts);
  CODE FAR void *(*alloc_buf)(FAR struct rpmsg_s *rpmsg, size_t size,
                              size_t align);
  CODE void (*free_buf)(FAR struct rpmsg_s *rpmsg, FAR void *addr);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void rpmsg_modify_signals(FAR struct rpmsg_s *rpmsg,
                          int setflags, int clrflags);

void rpmsg_ns_bind(FAR struct rpmsg_device *rdev,
                   FAR const char *name, uint32_t dest);
void rpmsg_ns_unbind(FAR struct rpmsg_device *rdev,
                     FAR const char *name, uint32_t dest);

void rpmsg_device_created(FAR struct rpmsg_s *rpmsg);
void rpmsg_device_destroy(FAR struct rpmsg_s *rpmsg);

int rpmsg_register(FAR const char *path, FAR struct rpmsg_s *rpmsg,
                   FAR const struct rpmsg_ops_s *ops, uint16_t nrx);
void rpmsg_unregister(FAR const char *path, FAR struct rpmsg_s *rpmsg);

int rpmsg_init_wqueues(FAR struct rpmsg_s *rpmsg, uint8_t priority);
void rpmsg_deinit_wqueues(FAR struct rpmsg_s *rpmsg);

int rpmsg_queue_work(FAR struct rpmsg_s *rpmsg, int priority,
                     FAR struct work_s *work, worker_t worker,
                     FAR void *arg);
void rpmsg_rx_worker(FAR void *arg);

int rpmsg_foreach(rpmsg_foreach_t handler, FAR void *args);
void rpmsg_dump_epts(FAR struct rpmsg_device *rdev);

#ifdef CONFIG_RPMSG_HEAP
int rpmsg_heap_init(FAR struct rpmsg_device *rdev,
                    FAR struct rpmsg_endpoint *ept);
void rpmsg_heap_deinit(FAR struct rpmsg_endpoint *ept);
FAR void *rpmsg_heap_alloc(FAR struct rpmsg_endpoint *ept,
                           size_t size, size_t align);
void rpmsg_heap_free(FAR struct rpmsg_endpoint *ept, FAR void *addr);
#else
#  define rpmsg_heap_alloc(ept, size, align) (NULL)
#  define rpmsg_heap_free(ept, addr)
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_get_rdev_by_rpmsg
 *
 * Description:
 *   Return the rpmsg device that follows struct rpmsg_s in memory.
 *
 ****************************************************************************/

static inline FAR struct rpmsg_device *
rpmsg_get_rdev_by_rpmsg(FAR struct rpmsg_s *rpmsg)
{
  if (!rpmsg)
    {
      return NULL;
    }

  return (FAR struct rpmsg_device *)(rpmsg + 1);
}

/****************************************************************************
 * Name: rpmsg_queue_rx_work
 *
 * Description:
 *   Queue a received message to an rx worker.  Inline to keep the rx
 *   dispatch, which may recurse, one stack frame shorter.
 *
 ****************************************************************************/

static inline_function
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

#endif /* CONFIG_RPMSG */
#endif /* __DRIVERS_RPMSG_RPMSG_H */
