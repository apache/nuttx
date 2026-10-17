/****************************************************************************
 * net/bt/compat/src/kernel/timeout.c
 *
 * Copyright (c) ByteDance Ltd. and/or its affiliates.
 *
 * Author: chao an <anchao.archer@bytedance.com>
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

/* SPDX-License-Identifier: Apache-2.0 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <sched.h>
#include <string.h>
#include <nuttx/net/bluetooth/compat/kernel.h>

/* Global system work queue - uses NuttX HP work queue */

struct k_work_q k_sys_work_q =
{
  .qid = HPWORK,
  .thread_id = INVALID_PROCESS_ID,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int k_work_queue_native(struct k_work_q *queue,
                               struct work_s *work,
                               worker_t worker, void *arg,
                               clock_t delay)
{
  if (queue != NULL && queue->wqueue != NULL)
    {
      return work_queue_wq(queue->wqueue, work, worker, arg, delay);
    }

  return work_queue(queue != NULL ? queue->qid : HPWORK,
                    work, worker, arg, delay);
}

static int k_work_cancel_native(struct k_work *work, bool sync)
{
  struct k_work_q *queue = work->queue;
  int ret;

  if (queue != NULL && queue->wqueue != NULL)
    {
      ret = sync ? work_cancel_sync_wq(queue->wqueue, &work->work) :
            work_cancel_wq(queue->wqueue, &work->work);
    }
  else
    {
      ret =
        sync ? work_cancel_sync(queue != NULL ? queue->qid : HPWORK,
                                &work->work) :
        work_cancel(queue != NULL ? queue->qid : HPWORK,
                    &work->work);
    }

  if (ret == 0)
    {
      work->flags &= ~(K_WORK_QUEUED | K_WORK_DELAYED);
    }

  return ret;
}

static void k_work_handler_wrapper(void *arg)
{
  struct k_work *work = arg;
  pid_t previous_thread_id =
    work->queue != NULL ? work->queue->thread_id : INVALID_PROCESS_ID;

  if (work->queue != NULL)
    {
      work->queue->thread_id = nxsched_gettid();
    }

  work->flags &= ~(K_WORK_QUEUED | K_WORK_DELAYED);
  work->flags |= K_WORK_RUNNING;

  work->handler(work);

  work->flags &= ~K_WORK_RUNNING;

  if (work->queue != NULL)
    {
      work->queue->thread_id = previous_thread_id;
    }
}

void k_work_init(struct k_work *work, k_work_handler_t handler)
{
  memset(work, 0, sizeof(*work));
  work->handler = handler;
  work->queue = &k_sys_work_q;
}

void k_work_init_delayable(struct k_work_delayable *dwork,
                           k_work_handler_t handler)
{
  memset(dwork, 0, sizeof(*dwork));
  dwork->work.handler = handler;
  dwork->work.queue = &k_sys_work_q;
  dwork->queue = &k_sys_work_q;
}

int k_work_submit(struct k_work *work)
{
  return k_work_submit_to_queue(&k_sys_work_q, work);
}

int k_work_submit_to_queue(struct k_work_q *queue, struct k_work *work)
{
  int ret;

  work->queue = queue != NULL ? queue : &k_sys_work_q;
  work->flags |= K_WORK_QUEUED;
  work->flags &= ~K_WORK_DELAYED;

  ret = k_work_queue_native(work->queue, &work->work,
                            k_work_handler_wrapper, work, 0);
  if (ret < 0)
    {
      work->flags &= ~K_WORK_QUEUED;
    }

  return ret;
}

int k_work_schedule(struct k_work_delayable *dwork, clock_t delay)
{
  if ((dwork->work.flags & (K_WORK_QUEUED | K_WORK_DELAYED)) != 0)
    {
      return 0;
    }

  return k_work_reschedule(dwork, delay);
}

int k_work_schedule_for_queue(struct k_work_q *queue,
                              struct k_work_delayable *dwork,
                              clock_t delay)
{
  dwork->queue = queue != NULL ? queue : &k_sys_work_q;
  dwork->work.queue = dwork->queue;

  return k_work_schedule(dwork, delay);
}

int k_work_cancel_delayable(struct k_work_delayable *dwork)
{
  return k_work_cancel_native(&dwork->work, false);
}

int k_work_cancel_delayable_sync(struct k_work_delayable *dwork,
                                 struct k_work_sync *sync)
{
  (void)sync;
  return k_work_cancel_native(&dwork->work, true);
}

int k_work_reschedule(struct k_work_delayable *dwork, clock_t delay)
{
  int ret;

  (void)k_work_cancel_delayable(dwork);

  if (dwork->work.queue == NULL)
    {
      dwork->work.queue = dwork->queue != NULL ? dwork->queue :
                          &k_sys_work_q;
    }

  dwork->queue = dwork->work.queue;

  if (delay < 0)
    {
      dwork->work.flags &= ~(K_WORK_QUEUED | K_WORK_DELAYED);
      return 0;
    }

  dwork->work.flags |= delay > 0 ? K_WORK_DELAYED : K_WORK_QUEUED;
  dwork->work.flags &= delay > 0 ? ~K_WORK_QUEUED : ~K_WORK_DELAYED;

  ret = k_work_queue_native(dwork->work.queue, &dwork->work.work,
                            k_work_handler_wrapper, &dwork->work, delay);
  if (ret < 0)
    {
      dwork->work.flags &= ~(K_WORK_QUEUED | K_WORK_DELAYED);
    }

  return ret;
}

int k_work_reschedule_for_queue(struct k_work_q *queue,
                                struct k_work_delayable *dwork,
                                clock_t delay)
{
  dwork->queue = queue != NULL ? queue : &k_sys_work_q;
  dwork->work.queue = dwork->queue;

  return k_work_reschedule(dwork, delay);
}

bool k_work_is_pending(const struct k_work *work)
{
  return (work->flags & (K_WORK_QUEUED | K_WORK_DELAYED)) != 0;
}

int k_work_busy_get(const struct k_work *work)
{
  return work->flags;
}

int k_work_flush(struct k_work *work, struct k_work_sync *sync)
{
  (void)sync;

  while (k_work_busy_get(work) != 0)
    {
      sched_yield();
    }

  return 0;
}

int k_work_cancel(struct k_work *work)
{
  return k_work_cancel_native(work, false);
}

int k_work_cancel_sync(struct k_work *work, struct k_work_sync *sync)
{
  (void)sync;
  return k_work_cancel_native(work, true);
}

struct k_work_delayable *k_work_delayable_from_work(struct k_work *work)
{
  return CONTAINER_OF(work, struct k_work_delayable, work);
}

int k_work_delayable_busy_get(const struct k_work_delayable *dwork)
{
  return k_work_busy_get(&dwork->work);
}

clock_t k_work_delayable_remaining_get(const struct k_work_delayable *dwork)
{
  clock_t remaining;

  if ((dwork->work.flags & K_WORK_DELAYED) == 0)
    {
      return 0;
    }

  remaining = work_timeleft(&dwork->work.work);

  return remaining > 0 ? remaining : 0;
}

pid_t k_work_queue_thread_get(struct k_work_q *queue)
{
  return queue != NULL ? queue->thread_id : INVALID_PROCESS_ID;
}

void k_work_queue_init(struct k_work_q *queue)
{
  if (queue != NULL)
    {
      memset(queue, 0, sizeof(*queue));
      queue->qid = HPWORK;
      queue->thread_id = INVALID_PROCESS_ID;
    }
}

int k_work_queue_start(struct k_work_q *queue, void *stack,
                       size_t stack_size, int prio,
                       const struct k_work_queue_config *cfg)
{
  const char *name = cfg != NULL
                     && cfg->name != NULL ? cfg->name : "bt_wq";

  if (queue == NULL)
    {
      return -EINVAL;
    }

  queue->wqueue = work_queue_create(name, prio, stack, stack_size, 1);
  if (queue->wqueue == NULL)
    {
      return -ENOMEM;
    }

  return 0;
}

/* ===== Condvar ===== */

int k_condvar_init(struct k_condvar *condvar)
{
  return pthread_cond_init(&condvar->cond, NULL);
}

int k_condvar_wait(struct k_condvar *condvar, rmutex_t *mutex,
                   clock_t timeout)
{
  (void)mutex;
  (void)timeout;
  return -ENOSYS;
}

int k_condvar_broadcast(struct k_condvar *condvar)
{
  return pthread_cond_broadcast(&condvar->cond);
}
