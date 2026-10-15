/****************************************************************************
 * include/nuttx/net/bluetooth/compat/kernel.h
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
#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_KERNEL_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_KERNEL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <sched.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/sched.h>
#include <nuttx/wqueue.h>
#include <nuttx/spinlock.h>
#include <nuttx/clock.h>
#include <nuttx/list.h>
#include <nuttx/mutex.h>

#include "sys/util.h"
#include "toolchain.h"
#include "init.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ===== FIFO / Queue ===== */

struct k_fifo
{
  struct list_node data_q;
  sem_t sem;
  spinlock_t lock; /* Use raw spinlock for ISR safety */
  unsigned int waiters;
  unsigned int poll_waiters;
  unsigned int cancel_waiters;
  unsigned int poll_cancel_waiters;
};

void k_fifo_init(struct k_fifo *fifo);
void k_fifo_cancel_wait(struct k_fifo *fifo);
void k_fifo_put(struct k_fifo *fifo, void *data);
void *k_fifo_get(struct k_fifo *fifo, clock_t timeout);
bool k_fifo_is_empty(struct k_fifo *fifo);
void *k_fifo_peek_head(struct k_fifo *fifo);
void z_k_queue_prepend_fifo(struct k_fifo *fifo, void *data);
bool z_k_queue_remove_fifo(struct k_fifo *fifo, void *data);

#define k_queue k_fifo
#define k_queue_init(queue) k_fifo_init(queue)
#define k_queue_append(queue, data) k_fifo_put(queue, data)
#define k_queue_get(queue, timeout) k_fifo_get(queue, timeout)
#define k_queue_peek_head(queue) k_fifo_peek_head(queue)
#define k_queue_prepend(queue, data) z_k_queue_prepend_fifo(queue, data)
#define k_queue_remove(queue, data) z_k_queue_remove_fifo(queue, data)

#define K_FIFO_DEFINE(name)                                                                        \
struct k_fifo name = {                                                                           \
        .data_q = LIST_INITIAL_VALUE(name.data_q),                                               \
        .sem = SEM_INITIALIZER(0),                                                               \
}

/* ===== Work Queue ===== */

/* Work flags for k_work */
#define K_WORK_QUEUED BIT(0)
#define K_WORK_DELAYED BIT(1)
#define K_WORK_RUNNING BIT(2)

struct k_work
{
  struct work_s work;
  void (*handler)(struct k_work *work);
  struct k_work_q *queue;
  uint32_t flags;
};

struct k_work_delayable
{
  struct k_work work;
  struct k_work_q *queue;
};

struct k_work_q
{
  FAR struct kwork_wqueue_s *wqueue;
  int qid;
  pid_t thread_id;
};

struct k_work_queue_config
{
  const char *name;
  bool no_yield;
  bool essential;
};

/* Work queue synchronization */

struct k_work_sync
{
  sem_t sem;
};

typedef void (*k_work_handler_t)(struct k_work *work);

#define Z_WORK_INITIALIZER(work_handler) {.handler = (k_work_handler_t)(work_handler)}

#define Z_WORK_DELAYABLE_INITIALIZER(work_handler) {.work = Z_WORK_INITIALIZER(work_handler)}

#define K_WORK_DEFINE(work, work_handler) struct k_work work = {.handler = (work_handler)}

#define K_WORK_DELAYABLE_DEFINE(dwork, work_handler)                                               \
struct k_work_delayable dwork = {.work = {.handler = (work_handler)}}

void k_work_init(struct k_work *work, k_work_handler_t handler);
void k_work_init_delayable(struct k_work_delayable *dwork,
                           k_work_handler_t handler);
int k_work_submit(struct k_work *work);
int k_work_submit_to_queue(struct k_work_q *queue, struct k_work *work);
int k_work_schedule(struct k_work_delayable *dwork, clock_t delay);
int k_work_schedule_for_queue(
  struct k_work_q *queue, struct k_work_delayable *dwork, clock_t delay);
int k_work_cancel_delayable(struct k_work_delayable *dwork);
int k_work_cancel_delayable_sync(struct k_work_delayable *dwork,
                                 struct k_work_sync *sync);
int k_work_reschedule(struct k_work_delayable *dwork, clock_t delay);
int k_work_reschedule_for_queue(
  struct k_work_q *queue, struct k_work_delayable *dwork, clock_t delay);
bool k_work_is_pending(const struct k_work *work);
int k_work_busy_get(const struct k_work *work);
static inline bool k_work_delayable_is_pending(
  const struct k_work_delayable *dwork)
{
  return k_work_is_pending(&dwork->work);
}
int k_work_flush(struct k_work *work, struct k_work_sync *sync);

int k_work_cancel(struct k_work *work);
int k_work_cancel_sync(struct k_work *work, struct k_work_sync *sync);
struct k_work_delayable *k_work_delayable_from_work(struct k_work *work);
int k_work_delayable_busy_get(const struct k_work_delayable *dwork);
clock_t k_work_delayable_remaining_get(const struct k_work_delayable *dwork);
pid_t k_work_queue_thread_get(struct k_work_q *queue);
void k_work_queue_init(struct k_work_q *queue);
int k_work_queue_start(
  struct k_work_q *queue, void *stack, size_t stack_size, int prio,
  const struct k_work_queue_config *cfg);

#define K_PRIO_COOP(prio) (prio)

extern struct k_work_q k_sys_work_q;

/* ===== Memory Slab ===== */

struct k_mem_slab
{
  char *buffer;
  char *free_list;
  size_t block_size;
  uint32_t num_blocks;
  uint32_t num_used;
  bool initialized;
  spinlock_t lock;
};

void k_mem_slab_init(
  struct k_mem_slab *slab, void *buffer, size_t block_size,
  uint32_t num_blocks);
int k_mem_slab_alloc(struct k_mem_slab *slab, void **mem, clock_t timeout);
void k_mem_slab_free(struct k_mem_slab *slab, void *mem);
uint32_t k_mem_slab_num_free_get(struct k_mem_slab *slab);

#define K_MEM_SLAB_DEFINE(name, slab_block_size, slab_num_blocks, slab_align)                      \
static char __aligned(slab_align) _k_mem_slab_buf_##name[(slab_block_size) * (slab_num_blocks)]; \
struct k_mem_slab name = {                                                                       \
        .buffer = _k_mem_slab_buf_##name,                                                        \
        .block_size = (slab_block_size),                                                         \
        .num_blocks = (slab_num_blocks),                                                         \
        .initialized = false,                                                                    \
}

#define K_MEM_SLAB_DEFINE_TYPE(name, type, num_blocks, ...)                                        \
K_MEM_SLAB_DEFINE(name, sizeof(type), num_blocks, __alignof__(type))

#define K_MEM_SLAB_DEFINE_STATIC_TYPE(name, type, num_blocks, ...)                                 \
K_MEM_SLAB_DEFINE(name, sizeof(type), num_blocks, __alignof__(type))

#define K_MEM_SLAB_DEFINE_STATIC(name, slab_block_size, slab_num_blocks, slab_align)               \
static K_MEM_SLAB_DEFINE(name, slab_block_size, slab_num_blocks, slab_align)

/* ===== Poll ===== */
#define K_POLL_TYPE_FIFO_DATA_AVAILABLE 4

#define K_POLL_STATE_FIFO_DATA_AVAILABLE 0x004
#define K_POLL_STATE_CANCELLED 0x020
#define K_POLL_STATE_NOT_READY 0x080

#define K_POLL_MODE_NOTIFY_ONLY 0

struct k_poll_event
{
  int type;
  uint32_t mode;
  uint32_t state;
  union
  {
    void *obj;
    struct k_fifo *fifo;
  };
  int tag;
};

#define K_POLL_EVENT_STATIC_INITIALIZER(_type, _mode, _obj, _tag)                                  \
{.type = (_type),                                                                                \
 .mode = (_mode),                                                                                \
 .state = K_POLL_STATE_NOT_READY,                                                                \
 .obj = (void *)(_obj),                                                                          \
 .tag = (_tag)}

int k_poll(struct k_poll_event *events, int num_events, clock_t timeout);

/* ===== Miscellaneous ===== */

struct k_condvar
{
  pthread_cond_t cond;
};

#define Z_CONDVAR_INITIALIZER(obj) {.cond = PTHREAD_COND_INITIALIZER}
int k_condvar_init(struct k_condvar *condvar);
int k_condvar_wait(struct k_condvar *condvar,
                   rmutex_t *mutex, clock_t timeout);
int k_condvar_broadcast(struct k_condvar *condvar);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_KERNEL_H */
