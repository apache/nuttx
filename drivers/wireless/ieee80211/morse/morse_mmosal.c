/****************************************************************************
 * drivers/wireless/ieee80211/morse/morse_mmosal.c
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

/* Implementation of the Morse Micro Operating System Abstraction Layer
 * (mmosal) on top of the NuttX kernel services.  This is the OS interface
 * used by the Morse Micro host library (libmorse).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <sched.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/sched.h>
#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>

#include "mmosal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timers run on the low priority work queue if available */

#ifdef CONFIG_SCHED_LPWORK
#  define MORSE_TIMER_WORK  LPWORK
#else
#  define MORSE_TIMER_WORK  HPWORK
#endif

/* Extra stack added to every mmosal task: the stack sizes requested by
 * libmorse are tuned for FreeRTOS and do not account for the NuttX
 * syslog/exception overhead.
 */

#define MORSE_TASK_STACK_EXTRA  2048

#define MORSE_TASK_PRIO_STEP    10

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mmosal_task
{
  mmosal_task_fn_t fn;
  FAR void *arg;
};

struct mmosal_mutex
{
  mutex_t lock;
};

struct mmosal_sem
{
  sem_t sem;
  unsigned int max;
};

struct mmosal_semb
{
  sem_t sem;
};

struct mmosal_queue
{
  sem_t items;              /* Number of items in the queue */
  sem_t spaces;             /* Number of free slots */
  size_t itemsize;
  size_t nitems;
  size_t head;
  size_t tail;
  uint8_t buf[1];
};

struct mmosal_timer
{
  struct work_s work;
  timer_callback_t callback;
  FAR void *arg;
  uint32_t period;          /* Timer period in ms */
  bool reload;              /* Auto reload */
  volatile bool active;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static irqstate_t g_critical_flags;
static int g_critical_nest;
static irqstate_t g_irq_flags;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t ms2tick(uint32_t ms)
{
  return (uint32_t)MSEC2TICK((clock_t)ms);
}

/* Wait on a NuttX semaphore with an mmosal style timeout.  Returns true if
 * the semaphore was taken.
 */

static bool sem_wait_ms(FAR sem_t *sem, uint32_t timeout_ms)
{
  int ret;

  if (timeout_ms == UINT32_MAX)
    {
      ret = nxsem_wait_uninterruptible(sem);
    }
  else if (timeout_ms == 0)
    {
      ret = nxsem_trywait(sem);
    }
  else
    {
      ret = nxsem_tickwait_uninterruptible(sem, ms2tick(timeout_ms));
    }

  return ret >= 0;
}

/* Post a semaphore only if its count is below 'max'.  Safe to be called
 * from interrupt context.
 */

static bool sem_post_max(FAR sem_t *sem, int max)
{
  irqstate_t flags;
  bool ret = false;
  int value;

  flags = enter_critical_section();
  if (nxsem_get_value(sem, &value) >= 0 && value < max)
    {
      ret = nxsem_post(sem) >= 0;
    }

  leave_critical_section(flags);
  return ret;
}

static int morse_task_main(int argc, FAR char *argv[])
{
  FAR struct mmosal_task *task;

  DEBUGASSERT(argc == 2);
  task = (FAR struct mmosal_task *)((uintptr_t)strtoul(argv[1], NULL, 16));

  task->fn(task->arg);

  kmm_free(task);
  return 0;
}

static void morse_timer_worker(FAR void *arg)
{
  FAR struct mmosal_timer *timer = arg;

  if (!timer->active)
    {
      return;
    }

  if (timer->reload)
    {
      work_queue(MORSE_TIMER_WORK, &timer->work, morse_timer_worker, timer,
                 ms2tick(timer->period));
    }
  else
    {
      timer->active = false;
    }

  timer->callback(timer);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Memory management ********************************************************/

FAR void *mmosal_malloc_(size_t size)
{
  return kmm_malloc(size);
}

FAR void *mmosal_calloc_(size_t nitems, size_t size)
{
  return kmm_calloc(nitems, size);
}

FAR void *mmosal_realloc_(FAR void *ptr, size_t size)
{
  return kmm_realloc(ptr, size);
}

void mmosal_free(FAR void *p)
{
  kmm_free(p);
}

FAR void *mmosal_malloc_dbg(size_t size, FAR const char *name,
                            unsigned line_number)
{
  return kmm_malloc(size);
}

FAR void *mmosal_calloc_dbg(size_t nitems, size_t size,
                            FAR const char *name, unsigned line_number)
{
  return kmm_calloc(nitems, size);
}

FAR void *mmosal_realloc_dbg(FAR void *ptr, size_t size,
                             FAR const char *name, unsigned line_number)
{
  return kmm_realloc(ptr, size);
}

/* Tasks ********************************************************************/

FAR struct mmosal_task *
mmosal_task_create(mmosal_task_fn_t task_fn, FAR void *argument,
                   enum mmosal_task_priority priority,
                   unsigned stack_size_u32, FAR const char *name)
{
  FAR struct mmosal_task *task;
  FAR char *argv[2];
  char arg[2 + 2 * sizeof(uintptr_t) + 1];
  int prio;
  int pid;

  task = kmm_malloc(sizeof(*task));
  if (task == NULL)
    {
      return NULL;
    }

  task->fn  = task_fn;
  task->arg = argument;

  /* Map the mmosal priorities around the configured base priority */

  prio = CONFIG_IEEE80211_MORSE_TASK_PRIORITY +
         ((int)priority - (int)MMOSAL_TASK_PRI_NORM) * MORSE_TASK_PRIO_STEP;
  if (prio < SCHED_PRIORITY_MIN + 1)
    {
      prio = SCHED_PRIORITY_MIN + 1;
    }
  else if (prio > SCHED_PRIORITY_MAX)
    {
      prio = SCHED_PRIORITY_MAX;
    }

  snprintf(arg, sizeof(arg), "%" PRIxPTR, (uintptr_t)task);
  argv[0] = arg;
  argv[1] = NULL;

  pid = kthread_create(name, prio,
                       stack_size_u32 * sizeof(uint32_t) +
                       MORSE_TASK_STACK_EXTRA,
                       morse_task_main, argv);
  if (pid < 0)
    {
      wlerr("ERROR: failed to create task %s: %d\n", name, pid);
      kmm_free(task);
      return NULL;
    }

  /* The task handle is the PID of the kernel thread */

  return (FAR struct mmosal_task *)(uintptr_t)pid;
}

void mmosal_task_delete(FAR struct mmosal_task *task)
{
  kthread_delete((pid_t)(uintptr_t)task);
}

FAR struct mmosal_task *mmosal_task_get_active(void)
{
  return (FAR struct mmosal_task *)(uintptr_t)nxsched_gettid();
}

void mmosal_task_yield(void)
{
  sched_yield();
}

void mmosal_task_sleep(uint32_t duration_ms)
{
  nxsched_usleep((useconds_t)duration_ms * USEC_PER_MSEC);
}

void mmosal_task_enter_critical(void)
{
  irqstate_t flags = enter_critical_section();

  if (g_critical_nest++ == 0)
    {
      g_critical_flags = flags;
    }
}

void mmosal_task_exit_critical(void)
{
  DEBUGASSERT(g_critical_nest > 0);

  if (--g_critical_nest == 0)
    {
      leave_critical_section(g_critical_flags);
    }
}

void mmosal_disable_interrupts(void)
{
  g_irq_flags = up_irq_save();
}

void mmosal_enable_interrupts(void)
{
  up_irq_restore(g_irq_flags);
}

FAR const char *mmosal_task_name(void)
{
#if CONFIG_TASK_NAME_SIZE > 0
  return nxsched_self()->name;
#else
  return "";
#endif
}

/* Mutexes ******************************************************************/

FAR struct mmosal_mutex *mmosal_mutex_create(FAR const char *name)
{
  FAR struct mmosal_mutex *mutex = kmm_malloc(sizeof(*mutex));

  if (mutex != NULL)
    {
      nxmutex_init(&mutex->lock);
    }

  return mutex;
}

void mmosal_mutex_delete(FAR struct mmosal_mutex *mutex)
{
  if (mutex != NULL)
    {
      nxmutex_destroy(&mutex->lock);
      kmm_free(mutex);
    }
}

bool mmosal_mutex_get(FAR struct mmosal_mutex *mutex, uint32_t timeout_ms)
{
  if (timeout_ms == UINT32_MAX)
    {
      return nxmutex_lock(&mutex->lock) >= 0;
    }
  else if (timeout_ms == 0)
    {
      return nxmutex_trylock(&mutex->lock) >= 0;
    }

  return nxmutex_timedlock(&mutex->lock, timeout_ms) >= 0;
}

bool mmosal_mutex_release(FAR struct mmosal_mutex *mutex)
{
  return nxmutex_unlock(&mutex->lock) >= 0;
}

bool mmosal_mutex_is_held_by_active_task(FAR struct mmosal_mutex *mutex)
{
  return nxmutex_is_hold(&mutex->lock);
}

/* Counting semaphores ******************************************************/

FAR struct mmosal_sem *mmosal_sem_create(unsigned max_count,
                                         unsigned initial_count,
                                         FAR const char *name)
{
  FAR struct mmosal_sem *sem = kmm_malloc(sizeof(*sem));

  if (sem != NULL)
    {
      nxsem_init(&sem->sem, 0, initial_count);
      sem->max = max_count;
    }

  return sem;
}

void mmosal_sem_delete(FAR struct mmosal_sem *sem)
{
  if (sem != NULL)
    {
      nxsem_destroy(&sem->sem);
      kmm_free(sem);
    }
}

bool mmosal_sem_give(FAR struct mmosal_sem *sem)
{
  return sem_post_max(&sem->sem, sem->max);
}

bool mmosal_sem_give_from_isr(FAR struct mmosal_sem *sem)
{
  return sem_post_max(&sem->sem, sem->max);
}

bool mmosal_sem_wait(FAR struct mmosal_sem *sem, uint32_t timeout_ms)
{
  return sem_wait_ms(&sem->sem, timeout_ms);
}

uint32_t mmosal_sem_get_count(FAR struct mmosal_sem *sem)
{
  int value = 0;

  nxsem_get_value(&sem->sem, &value);
  return value > 0 ? value : 0;
}

/* Binary semaphores ********************************************************/

FAR struct mmosal_semb *mmosal_semb_create(FAR const char *name)
{
  FAR struct mmosal_semb *semb = kmm_malloc(sizeof(*semb));

  if (semb != NULL)
    {
      nxsem_init(&semb->sem, 0, 0);
    }

  return semb;
}

void mmosal_semb_delete(FAR struct mmosal_semb *semb)
{
  if (semb != NULL)
    {
      nxsem_destroy(&semb->sem);
      kmm_free(semb);
    }
}

bool mmosal_semb_give(FAR struct mmosal_semb *semb)
{
  return sem_post_max(&semb->sem, 1);
}

bool mmosal_semb_give_from_isr(FAR struct mmosal_semb *semb)
{
  return sem_post_max(&semb->sem, 1);
}

bool mmosal_semb_wait(FAR struct mmosal_semb *semb, uint32_t timeout_ms)
{
  return sem_wait_ms(&semb->sem, timeout_ms);
}

/* Queues *******************************************************************/

FAR struct mmosal_queue *mmosal_queue_create(size_t num_items,
                                             size_t item_size,
                                             FAR const char *name)
{
  FAR struct mmosal_queue *queue;

  queue = kmm_zalloc(sizeof(*queue) + num_items * item_size);
  if (queue != NULL)
    {
      nxsem_init(&queue->items, 0, 0);
      nxsem_init(&queue->spaces, 0, num_items);
      queue->itemsize = item_size;
      queue->nitems   = num_items;
    }

  return queue;
}

void mmosal_queue_delete(FAR struct mmosal_queue *queue)
{
  if (queue != NULL)
    {
      nxsem_destroy(&queue->items);
      nxsem_destroy(&queue->spaces);
      kmm_free(queue);
    }
}

static void queue_put(FAR struct mmosal_queue *queue, FAR const void *item)
{
  irqstate_t flags = enter_critical_section();

  memcpy(&queue->buf[queue->head * queue->itemsize], item,
         queue->itemsize);
  queue->head = (queue->head + 1) % queue->nitems;
  leave_critical_section(flags);
  nxsem_post(&queue->items);
}

static void queue_get(FAR struct mmosal_queue *queue, FAR void *item)
{
  irqstate_t flags = enter_critical_section();

  memcpy(item, &queue->buf[queue->tail * queue->itemsize],
         queue->itemsize);
  queue->tail = (queue->tail + 1) % queue->nitems;
  leave_critical_section(flags);
  nxsem_post(&queue->spaces);
}

bool mmosal_queue_push(FAR struct mmosal_queue *queue, FAR const void *item,
                       uint32_t timeout_ms)
{
  if (!sem_wait_ms(&queue->spaces, timeout_ms))
    {
      return false;
    }

  queue_put(queue, item);
  return true;
}

bool mmosal_queue_pop(FAR struct mmosal_queue *queue, FAR void *item,
                      uint32_t timeout_ms)
{
  if (!sem_wait_ms(&queue->items, timeout_ms))
    {
      return false;
    }

  queue_get(queue, item);
  return true;
}

bool mmosal_queue_push_from_isr(FAR struct mmosal_queue *queue,
                                FAR const void *item)
{
  return mmosal_queue_push(queue, item, 0);
}

bool mmosal_queue_pop_from_isr(FAR struct mmosal_queue *queue,
                               FAR void *item)
{
  return mmosal_queue_pop(queue, item, 0);
}

/* Time *********************************************************************/

uint32_t mmosal_get_time_ms(void)
{
  return (uint32_t)TICK2MSEC(clock_systime_ticks());
}

uint32_t mmosal_get_time_ticks(void)
{
  return (uint32_t)clock_systime_ticks();
}

uint32_t mmosal_ticks_per_second(void)
{
  return TICK_PER_SEC;
}

/* Timers *******************************************************************/

FAR struct mmosal_timer *mmosal_timer_create(FAR const char *name,
                                             uint32_t timer_period_ms,
                                             bool auto_reload,
                                             FAR void *arg,
                                             timer_callback_t callback)
{
  FAR struct mmosal_timer *timer = kmm_zalloc(sizeof(*timer));

  if (timer != NULL)
    {
      timer->callback = callback;
      timer->arg      = arg;
      timer->period   = timer_period_ms;
      timer->reload   = auto_reload;
    }

  return timer;
}

void mmosal_timer_delete(FAR struct mmosal_timer *timer)
{
  if (timer != NULL)
    {
      mmosal_timer_stop(timer);
      kmm_free(timer);
    }
}

bool mmosal_timer_start(FAR struct mmosal_timer *timer)
{
  timer->active = true;
  return work_queue(MORSE_TIMER_WORK, &timer->work, morse_timer_worker,
                    timer, ms2tick(timer->period)) >= 0;
}

bool mmosal_timer_stop(FAR struct mmosal_timer *timer)
{
  timer->active = false;
  work_cancel(MORSE_TIMER_WORK, &timer->work);
  return true;
}

bool mmosal_timer_change_period(FAR struct mmosal_timer *timer,
                                uint32_t new_period)
{
  /* Like FreeRTOS xTimerChangePeriod(), this also (re)starts the timer */

  timer->period = new_period;
  return mmosal_timer_start(timer);
}

FAR void *mmosal_timer_get_arg(FAR struct mmosal_timer *timer)
{
  return timer->arg;
}

bool mmosal_is_timer_active(FAR struct mmosal_timer *timer)
{
  return timer->active;
}

/* Debug and assertions *****************************************************/

int mmosal_printf(FAR const char *format, ...)
{
#ifdef CONFIG_IEEE80211_MORSE_LIBLOG
  va_list ap;

  va_start(ap, format);
  vsyslog(LOG_INFO, format, ap);
  va_end(ap);
#endif

  return 0;
}

void mmosal_log_failure_info(FAR const struct mmosal_failure_info *info)
{
  syslog(LOG_EMERG, "morse: failure at pc 0x%08" PRIx32 " lr 0x%08" PRIx32
         " file 0x%08" PRIx32 " line %" PRIu32 " (0x%08" PRIx32 ")\n",
         info->pc, info->lr, info->fileid, info->line,
         info->platform_info[0]);
}

void mmosal_impl_assert(void)
{
  PANIC();
}
