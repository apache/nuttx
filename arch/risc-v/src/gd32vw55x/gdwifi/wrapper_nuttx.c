/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gdwifi/wrapper_nuttx.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * NuttX backend for the GigaDevice GD32VW55x SDK OS wrapper (the sys_*
 * facade declared in rtos/rtos_wrapper/wrapper_os.h).
 *
 * Validated HAL: SDK V1.0.3g (2026-04-23, commit 945c6e2).  ilp32f ABI.
 *
 * The prebuilt Wi-Fi and BLE libraries link exclusively against these
 * symbols; semantics follow wrapper_freertos.c, including its quirks:
 *
 *  - task stack sizes are in 32-bit WORDS (converted to bytes here)
 *  - sys_sema_down()/sys_task_wait()/blocking sys_queue_fetch() treat
 *    timeout 0 as "wait forever", while the int-timeout APIs
 *    (sys_mutex_try_get, sys_queue_write/read, sys_task_wait_notification)
 *    use -1 = forever and 0 = no wait
 *  - sys_queue_write/read return 0 on success, 1 on failure;
 *    sys_timer_stop returns 1 on success
 *  - sys_task_create returns the task handle; the per-task mailbox
 *    wrapper is looked up internally (pid table here, TLS in FreeRTOS)
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#ifdef CONFIG_GD32VW55X_WIFI_TRACE
#  include <syslog.h>
#endif

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mm/mm.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/mutex.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/sched.h>

#include "wrapper_os.h"

#include "hardware/gd32vw55x_eclic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SDK priorities are 0..31 (higher = higher priority, idle = 0, app base
 * 16).  Map linearly into the NuttX range, keeping direction.
 */

#define PRIO_SDK2NX(p)   ((int)(64 + ((int)(p)) * 4) > 250 ? 250 : \
                          (int)(64 + ((int)(p)) * 4))
#define PRIO_NX2SDK(p)   (((p) - 64) / 4)

#define MAX_TASKS        16    /* Concurrent sys_task_create() tasks */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Copy-based message queue (ISR-safe) */

struct nx_queue_s
{
  uint8_t  *buf;
  uint32_t  item_size;
  uint32_t  depth;
  uint32_t  head;      /* Next slot to read */
  uint32_t  count;     /* Items currently stored */
  sem_t     sem_avail; /* Counts stored items */
  sem_t     sem_space; /* Counts free slots */
};

/* Per-task bookkeeping: mailbox + notification semaphore */

struct nx_task_s
{
  pid_t              pid;
  struct nx_queue_s *mbox;      /* NULL if queue_size == 0 */
  sem_t              notif;     /* Counting notification */
  task_func_t        func;      /* Entry handoff */
  void              *ctx;
  sem_t              started;
  char               name[16];
};

/* Timer: wdog (ISR) kicks a LP work item that runs the user callback in
 * task context, as the FreeRTOS timer daemon does.
 */

struct nx_timer_s
{
  struct wdog_s     wdog;
  struct work_s     work;
  uint32_t          delay_ms;
  uint8_t           periodic;
  volatile uint8_t  active;
  timer_func_t      func;
  void             *arg;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct nx_task_s *g_tasks[MAX_TASKS];

/* Nestable global critical section */

static volatile uint32_t g_crit_nest;

uint8_t sys_ps_mode = SYS_PS_OFF;

/* Lightweight bring-up trace (first N mailbox ops) */

#ifdef CONFIG_GD32VW55X_WIFI_TRACE
#  define WTRACE(fmt, ...) \
     do { static int n_; if (n_ < 40) { n_++; \
          syslog(LOG_INFO, fmt, ##__VA_ARGS__); } } while (0)
#else
#  define WTRACE(fmt, ...)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static struct nx_task_s *task_slot_find(pid_t pid)
{
  int i;

  for (i = 0; i < MAX_TASKS; i++)
    {
      if (g_tasks[i] != NULL && g_tasks[i]->pid == pid)
        {
          return g_tasks[i];
        }
    }

  return NULL;
}

static struct nx_task_s *task_slot_from_handle(void *task)
{
  pid_t pid = (task == NULL) ? nxsched_gettid() :
              (pid_t)(uintptr_t)task - 1;
  return task_slot_find(pid);
}

/* Convert the wrapper task handle convention: handle = pid + 1 so that a
 * valid handle is never NULL.
 */

static inline void *pid_to_handle(pid_t pid)
{
  return (void *)((uintptr_t)pid + 1);
}

/* Queue primitives */

static int nx_queue_create(struct nx_queue_s **qp, uint32_t depth,
                           uint32_t item_size)
{
  struct nx_queue_s *q;

  q = kmm_zalloc(sizeof(*q));
  if (q == NULL)
    {
      return -ENOMEM;
    }

  q->buf = kmm_malloc(depth * item_size);
  if (q->buf == NULL)
    {
      kmm_free(q);
      return -ENOMEM;
    }

  q->item_size = item_size;
  q->depth     = depth;
  nxsem_init(&q->sem_avail, 0, 0);
  nxsem_init(&q->sem_space, 0, depth);

  *qp = q;
  return OK;
}

static void nx_queue_destroy(struct nx_queue_s *q)
{
  nxsem_destroy(&q->sem_avail);
  nxsem_destroy(&q->sem_space);
  kmm_free(q->buf);
  kmm_free(q);
}

/* timeout_ms: <0 forever, 0 = no wait, >0 wait ms.  Returns OK/-ETIMEDOUT */

static int sem_take_timeout(sem_t *sem, int timeout_ms)
{
  if (timeout_ms < 0)
    {
      return nxsem_wait_uninterruptible(sem);
    }
  else if (timeout_ms == 0)
    {
      return nxsem_trywait(sem);
    }
  else
    {
      return nxsem_tickwait_uninterruptible(sem, MSEC2TICK(timeout_ms));
    }
}

static int nx_queue_send(struct nx_queue_s *q, const void *msg,
                         int timeout_ms)
{
  irqstate_t flags;
  uint32_t slot;
  int ret;

  ret = sem_take_timeout(&q->sem_space, up_interrupt_context() ?
                         0 : timeout_ms);
  if (ret < 0)
    {
      return ret;
    }

  flags = enter_critical_section();
  slot = (q->head + q->count) % q->depth;
  memcpy(q->buf + slot * q->item_size, msg, q->item_size);
  q->count++;
  leave_critical_section(flags);

  nxsem_post(&q->sem_avail);
  return OK;
}

static int nx_queue_recv(struct nx_queue_s *q, void *msg, int timeout_ms)
{
  irqstate_t flags;
  int ret;

  ret = sem_take_timeout(&q->sem_avail, up_interrupt_context() ?
                         0 : timeout_ms);
  if (ret < 0)
    {
      return ret;
    }

  flags = enter_critical_section();
  memcpy(msg, q->buf + q->head * q->item_size, q->item_size);
  q->head = (q->head + 1) % q->depth;
  q->count--;
  leave_critical_section(flags);

  nxsem_post(&q->sem_space);
  return OK;
}

/* Task entry trampoline */

static int task_trampoline(int argc, char *argv[])
{
  struct nx_task_s *t;
  int i;

  t = (struct nx_task_s *)(uintptr_t)strtoul(argv[1], NULL, 16);
  t->pid = nxsched_gettid();

  /* Register ourselves BEFORE signalling the creator: this task may have
   * higher priority and call sys_task_wait/wait_notification immediately.
   */

  sched_lock();
  for (i = 0; i < MAX_TASKS; i++)
    {
      if (g_tasks[i] == NULL)
        {
          g_tasks[i] = t;
          break;
        }
    }

  sched_unlock();
  DEBUGASSERT(i < MAX_TASKS);

  nxsem_post(&t->started);

  t->func(t->ctx);

  /* Entry returned: clean up as if sys_task_delete(NULL) */

  sys_task_delete(NULL);
  return 0;
}

/* Timer plumbing: wdog handler (ISR) -> LP work -> user callback */

static void timer_worker(void *arg)
{
  struct nx_timer_s *t = arg;

  if (!t->active)
    {
      return;
    }

  if (t->periodic)
    {
      wd_start(&t->wdog, MSEC2TICK(t->delay_ms),
               (wdentry_t)(uintptr_t)0, 0); /* patched below */
    }

  t->func(t, t->arg);
}

static void timer_wdog_handler(wdparm_t arg)
{
  struct nx_timer_s *t = (struct nx_timer_s *)(uintptr_t)arg;

  if (t->active)
    {
      work_queue(LPWORK, &t->work, timer_worker, t, 0);
    }
}

/****************************************************************************
 * Public Functions - heap
 ****************************************************************************/

void *sys_malloc(size_t size)
{
  return kmm_malloc(size);
}

void *sys_calloc(size_t count, size_t size)
{
  return kmm_zalloc(count * size);
}

void *sys_realloc(void *mem, size_t size)
{
  return kmm_realloc(mem, size);
}

void sys_mfree(void *ptr)
{
  kmm_free(ptr);
}

int32_t sys_free_heap_size(void)
{
  struct mallinfo info = kmm_mallinfo();
  return info.fordblks;
}

int32_t sys_min_free_heap_size(void)
{
  return sys_free_heap_size();
}

uint16_t sys_heap_block_size(void)
{
  return 16;
}

void sys_heap_info(int *total_size, int *free_size, int *min_free_size)
{
  struct mallinfo info = kmm_mallinfo();

  if (total_size)
    {
      *total_size = info.arena;
    }

  if (free_size)
    {
      *free_size = info.fordblks;
    }

  if (min_free_size)
    {
      *min_free_size = info.fordblks;
    }
}

void sys_add_heap_region(uint32_t start, uint32_t size)
{
  kmm_addregion((void *)(uintptr_t)start, (size_t)size);
}

void sys_remove_heap_region(uint32_t start, uint32_t size)
{
}

/****************************************************************************
 * Public Functions - mem manipulation
 ****************************************************************************/

void sys_memset(void *s, uint8_t c, uint32_t count)
{
  memset(s, c, count);
}

void sys_memcpy(void *des, const void *src, uint32_t n)
{
  memcpy(des, src, n);
}

void sys_memmove(void *des, const void *src, uint32_t n)
{
  memmove(des, src, n);
}

int32_t sys_memcmp(const void *buf1, const void *buf2, uint32_t count)
{
  return memcmp(buf1, buf2, count);
}

/****************************************************************************
 * Public Functions - tasks
 ****************************************************************************/

void *sys_task_create(void *static_tcb, const uint8_t *name,
                      uint32_t *stack_base, uint32_t stack_size,
                      uint32_t queue_size, uint32_t queue_item_size,
                      uint32_t priority, task_func_t func, void *ctx)
{
  struct nx_task_s *t;
  char arg1[16];
  char *argv[2];
  int pid;

  t = kmm_zalloc(sizeof(*t));
  if (t == NULL)
    {
      return NULL;
    }

  if (queue_size > 0)
    {
      if (nx_queue_create(&t->mbox, queue_size, queue_item_size) < 0)
        {
#ifdef CONFIG_GD32VW55X_WIFI_TRACE
          syslog(LOG_ERR, "gdwifi: mbox_create('%s' %lux%lu) failed\n",
                 (const char *)name, (unsigned long)queue_size,
                 (unsigned long)queue_item_size);
#endif
          kmm_free(t);
          return NULL;
        }
    }

  nxsem_init(&t->notif, 0, 0);
  nxsem_init(&t->started, 0, 0);
  strlcpy(t->name, (const char *)name, sizeof(t->name));
  t->func = func;
  t->ctx  = ctx;

  snprintf(arg1, sizeof(arg1), "%lx", (unsigned long)(uintptr_t)t);
  argv[0] = arg1;
  argv[1] = NULL;

  /* stack_size arrives in 32-bit words (FreeRTOS convention) */

  /* task_create (not kthread_create): the supplicant library touches
   * stdio (setvbuf) which needs the group FILE streams that kernel
   * threads do not initialize.
   */

  pid = kthread_create((const char *)name, PRIO_SDK2NX(priority),
                       stack_size * sizeof(uint32_t),
                       task_trampoline, argv);
  if (pid < 0)
    {
#ifdef CONFIG_GD32VW55X_WIFI_TRACE
      syslog(LOG_ERR, "gdwifi: task_create('%s' stack=%lu prio=%d) -> %d\n",
             t->name, (unsigned long)(stack_size * 4),
             PRIO_SDK2NX(priority), pid);
#endif
      if (t->mbox)
        {
          nx_queue_destroy(t->mbox);
        }

      kmm_free(t);
      return NULL;
    }

  /* Wait until the trampoline has registered the task and its pid */

  nxsem_wait_uninterruptible(&t->started);

#ifdef CONFIG_GD32VW55X_WIFI_TRACE
  syslog(LOG_INFO, "gdwifi: task '%s' pid=%d prio=%d stack=%luB q=%lux%lu\n",
         t->name, (int)t->pid, PRIO_SDK2NX(priority),
         (unsigned long)(stack_size * 4),
         (unsigned long)queue_size, (unsigned long)queue_item_size);
#endif

  return pid_to_handle(t->pid);
}

void sys_task_delete(void *task)
{
  struct nx_task_s *t = task_slot_from_handle(task);
  bool self = (task == NULL) ||
              ((pid_t)(uintptr_t)task - 1 == nxsched_gettid());
  int i;

  if (t != NULL)
    {
      sched_lock();
      for (i = 0; i < MAX_TASKS; i++)
        {
          if (g_tasks[i] == t)
            {
              g_tasks[i] = NULL;
              break;
            }
        }

      sched_unlock();

      if (!self)
        {
          kthread_delete(t->pid);
        }

      if (t->mbox)
        {
          nx_queue_destroy(t->mbox);
        }

      nxsem_destroy(&t->notif);
      nxsem_destroy(&t->started);
      kmm_free(t);
    }

  if (self)
    {
      exit(0);
    }
}

char *sys_task_name_get(void *task)
{
#if CONFIG_TASK_NAME_SIZE > 0
  struct tcb_s *tcb = nxsched_get_tcb(task == NULL ? nxsched_gettid() :
                                      (pid_t)(uintptr_t)task - 1);
  return tcb ? tcb->name : NULL;
#else
  return NULL;
#endif
}

void sys_task_list(char *pwrite_buf)
{
  if (pwrite_buf)
    {
      *pwrite_buf = '\0';
    }
}

int32_t sys_task_wait(uint32_t timeout_ms, void *msg_ptr)
{
  struct nx_task_s *t = task_slot_find(nxsched_gettid());
  int ret;

  if (t == NULL || t->mbox == NULL)
    {
      WTRACE("gdwifi: wait pid=%d NO MBOX\n", (int)nxsched_gettid());
      return OS_ERROR;
    }

  /* timeout 0 = wait forever (FreeRTOS wrapper convention here) */

  ret = nx_queue_recv(t->mbox, msg_ptr,
                      timeout_ms == 0 ? -1 : (int)timeout_ms);
  return (ret < 0) ? OS_TIMEOUT : OS_OK;
}

int32_t sys_task_post(void *receiver_task, void *msg_ptr, uint8_t from_isr)
{
  struct nx_task_s *t = task_slot_from_handle(receiver_task);
  int ret;

  if (t == NULL || t->mbox == NULL)
    {
      WTRACE("gdwifi: post to %p NO MBOX\n", receiver_task);
      return OS_ERROR;
    }

  ret = nx_queue_send(t->mbox, msg_ptr, 0);
  WTRACE("gdwifi: post '%s' ret=%d cnt=%lu\n", t->name, ret,
         (unsigned long)t->mbox->count);
  return (ret < 0) ? OS_ERROR : OS_OK;
}

void sys_task_msg_flush(void *task)
{
  struct nx_task_s *t = task_slot_from_handle(task);
  uint8_t scratch[32];

  if (t != NULL && t->mbox != NULL)
    {
      while (t->mbox->count > 0 && t->mbox->item_size <= sizeof(scratch))
        {
          if (nx_queue_recv(t->mbox, scratch, 0) < 0)
            {
              break;
            }
        }
    }
}

int32_t sys_task_msg_num(void *task, uint8_t from_isr)
{
  struct nx_task_s *t = task_slot_from_handle(task);

  if (t == NULL || t->mbox == NULL)
    {
      return OS_ERROR;
    }

  return t->mbox->count;
}

os_task_t sys_current_task_handle_get(void)
{
  return pid_to_handle(nxsched_gettid());
}

int32_t sys_current_task_stack_depth(unsigned long cur_sp)
{
  return 1024;
}

uint32_t sys_stack_free_get(void *task)
{
  return 1024;
}

int sys_task_init_notification(void *task)
{
  return 0;
}

int sys_task_wait_notification(int timeout)
{
  struct nx_task_s *t = task_slot_find(nxsched_gettid());
  int count = 0;
  int sval;

  if (t == NULL)
    {
      return 0;
    }

  if (sem_take_timeout(&t->notif, timeout) < 0)
    {
      return 0;
    }

  /* Drain remaining posts to emulate the counting "take all" behavior */

  count = 1;
  while (nxsem_get_value(&t->notif, &sval) == 0 && sval > 0 &&
         nxsem_trywait(&t->notif) == 0)
    {
      count++;
    }

  return count;
}

void sys_task_notify(void *task, bool isr)
{
  struct nx_task_s *t = task_slot_from_handle(task);

  if (t != NULL)
    {
      nxsem_post(&t->notif);
    }
}

uint8_t sys_task_exist(const uint8_t *name)
{
  int i;

  for (i = 0; i < MAX_TASKS; i++)
    {
      if (g_tasks[i] != NULL &&
          strncmp(g_tasks[i]->name, (const char *)name,
                  sizeof(g_tasks[i]->name)) == 0)
        {
          return 1;
        }
    }

  return 0;
}

void sys_priority_set(void *task, os_prio_t priority)
{
  pid_t pid = (task == NULL) ? nxsched_gettid() :
              (pid_t)(uintptr_t)task - 1;
  struct sched_param param;

  param.sched_priority = PRIO_SDK2NX(priority);
  nxsched_set_param(pid, &param);
}

os_prio_t sys_priority_get(void *task)
{
  pid_t pid = (task == NULL) ? nxsched_gettid() :
              (pid_t)(uintptr_t)task - 1;
  struct sched_param param;

  if (nxsched_get_param(pid, &param) < 0)
    {
      return 0;
    }

  return PRIO_NX2SDK(param.sched_priority);
}

/****************************************************************************
 * Public Functions - semaphores
 ****************************************************************************/

int32_t sys_sema_init_ext(os_sema_t *sema, int max_count, int init_count)
{
  sem_t *s = kmm_malloc(sizeof(sem_t));

  if (s == NULL)
    {
      return OS_ERROR;
    }

  nxsem_init(s, 0, init_count);
  *sema = s;
  return OS_OK;
}

int32_t sys_sema_init(os_sema_t *sema, int32_t init_val)
{
  return sys_sema_init_ext(sema, INT32_MAX, init_val);
}

void sys_sema_free(os_sema_t *sema)
{
  if (sema != NULL && *sema != NULL)
    {
      nxsem_destroy((sem_t *)*sema);
      kmm_free(*sema);
      *sema = NULL;
    }
}

void sys_sema_up(os_sema_t *sema)
{
  nxsem_post((sem_t *)*sema);
}

void sys_sema_up_from_isr(os_sema_t *sema)
{
  nxsem_post((sem_t *)*sema);
}

int32_t sys_sema_down(os_sema_t *sema, uint32_t timeout_ms)
{
  int ret;

  /* timeout 0 = wait forever here */

  ret = sem_take_timeout((sem_t *)*sema,
                         timeout_ms == 0 ? -1 : (int)timeout_ms);
  return (ret < 0) ? OS_TIMEOUT : OS_OK;
}

int sys_sema_get_count(os_sema_t *sema)
{
  int sval = 0;

  nxsem_get_value((sem_t *)*sema, &sval);
  return sval;
}

/****************************************************************************
 * Public Functions - mutexes
 ****************************************************************************/

int sys_mutex_init(os_mutex_t *mutex)
{
  rmutex_t *m = kmm_malloc(sizeof(rmutex_t));

  if (m == NULL)
    {
      return OS_ERROR;
    }

  nxrmutex_init(m);
  *mutex = m;
  return OS_OK;
}

void sys_mutex_free(os_mutex_t *mutex)
{
  if (mutex != NULL && *mutex != NULL)
    {
      nxrmutex_destroy((rmutex_t *)*mutex);
      kmm_free(*mutex);
      *mutex = NULL;
    }
}

int32_t sys_mutex_get(os_mutex_t *mutex)
{
  nxrmutex_lock((rmutex_t *)*mutex);
  return OS_OK;
}

int32_t sys_mutex_try_get(os_mutex_t *mutex, int timeout)
{
  int ret;

  if (timeout < 0)
    {
      ret = nxrmutex_lock((rmutex_t *)*mutex);
    }
  else if (timeout == 0)
    {
      ret = nxrmutex_trylock((rmutex_t *)*mutex);
    }
  else
    {
      /* No timed recursive lock; poll in 1-tick steps */

      int remaining = timeout;

      while ((ret = nxrmutex_trylock((rmutex_t *)*mutex)) < 0 &&
             remaining > 0)
        {
          nxsig_usleep(USEC_PER_TICK);
          remaining -= TICK2MSEC(1) > 0 ? TICK2MSEC(1) : 1;
        }
    }

  return (ret < 0) ? OS_ERROR : OS_OK;
}

void sys_mutex_put(os_mutex_t *mutex)
{
  nxrmutex_unlock((rmutex_t *)*mutex);
}

/****************************************************************************
 * Public Functions - queues
 ****************************************************************************/

int32_t sys_queue_init(os_queue_t *queue, int32_t queue_size,
                       uint32_t item_size)
{
  struct nx_queue_s *q;

  if (nx_queue_create(&q, queue_size, item_size) < 0)
    {
      return OS_ERROR;
    }

  *queue = q;
  return OS_OK;
}

void sys_queue_free(os_queue_t *queue)
{
  if (queue != NULL && *queue != NULL)
    {
      nx_queue_destroy((struct nx_queue_s *)*queue);
      *queue = NULL;
    }
}

int32_t sys_queue_post(os_queue_t *queue, void *msg)
{
  return (nx_queue_send((struct nx_queue_s *)*queue, msg, 0) < 0) ?
         OS_ERROR : OS_OK;
}

int32_t sys_queue_post_with_timeout(os_queue_t *queue, void *msg,
                                    int32_t timeout_ms)
{
  return (nx_queue_send((struct nx_queue_s *)*queue, msg, timeout_ms) < 0) ?
         OS_ERROR : OS_OK;
}

int32_t sys_queue_fetch(os_queue_t *queue, void *msg, uint32_t timeout_ms,
                        uint8_t is_blocking)
{
  int timeout;

  if (!is_blocking)
    {
      timeout = 0;
    }
  else if (timeout_ms == 0)
    {
      timeout = -1;
    }
  else
    {
      timeout = (int)timeout_ms;
    }

  return (nx_queue_recv((struct nx_queue_s *)*queue, msg, timeout) < 0) ?
         OS_TIMEOUT : OS_OK;
}

bool sys_queue_is_empty(os_queue_t *queue)
{
  return ((struct nx_queue_s *)*queue)->count == 0;
}

int sys_queue_cnt(os_queue_t *queue)
{
  return ((struct nx_queue_s *)*queue)->count;
}

/* Note inverted polarity: 0 = success, non-zero = failure */

int sys_queue_write(os_queue_t *queue, void *msg, int timeout, bool isr)
{
  return (nx_queue_send((struct nx_queue_s *)*queue, msg,
                        isr ? 0 : timeout) < 0) ? 1 : 0;
}

int sys_queue_read(os_queue_t *queue, void *msg, int timeout, bool isr)
{
  return (nx_queue_recv((struct nx_queue_s *)*queue, msg,
                        isr ? 0 : timeout) < 0) ? 1 : 0;
}

/****************************************************************************
 * Public Functions - time
 ****************************************************************************/

uint32_t sys_current_time_get(void)
{
  return (uint32_t)TICK2MSEC(clock_systime_ticks());
}

uint32_t sys_time_get(void *p)
{
  return sys_current_time_get();
}

uint32_t sys_os_now(bool isr)
{
  /* OS_MS_PER_TICK is 1 in wrapper_os_config.h, so "ticks" == ms */

  return sys_current_time_get();
}

void sys_ms_sleep(int ms)
{
  if (ms <= 0)
    {
      return;
    }

  nxsig_usleep(ms * 1000);
}

void sys_us_delay(uint32_t nus)
{
  sched_lock();
  up_udelay(nus);
  sched_unlock();
}

void sys_yield(void)
{
  sched_yield();
}

void sys_sched_lock(void)
{
  sched_lock();
}

void sys_sched_unlock(void)
{
  sched_unlock();
}

/****************************************************************************
 * Public Functions - timers
 ****************************************************************************/

static void timer_worker_fixed(void *arg)
{
  struct nx_timer_s *t = arg;

  if (!t->active)
    {
      return;
    }

  if (t->periodic)
    {
      wd_start(&t->wdog, MSEC2TICK(t->delay_ms), timer_wdog_handler,
               (wdparm_t)(uintptr_t)t);
    }
  else
    {
      t->active = 0;
    }

  t->func(t, t->arg);
}

void sys_timer_init(os_timer_t *timer, const uint8_t *name, uint32_t delay,
                    uint8_t periodic, timer_func_t func, void *arg)
{
  struct nx_timer_s *t = kmm_zalloc(sizeof(*t));

  if (t == NULL)
    {
      *timer = NULL;
      return;
    }

  t->delay_ms = delay;
  t->periodic = periodic;
  t->func     = func;
  t->arg      = arg;
  *timer      = t;
}

void sys_timer_delete(os_timer_t *timer)
{
  struct nx_timer_s *t;

  if (timer == NULL || *timer == NULL)
    {
      return;
    }

  t = (struct nx_timer_s *)*timer;
  *timer = NULL;

  t->active = 0;
  wd_cancel(&t->wdog);
  work_cancel(LPWORK, &t->work);
  kmm_free(t);
}

void sys_timer_start(os_timer_t *timer, uint8_t from_isr)
{
  struct nx_timer_s *t = (struct nx_timer_s *)*timer;

  t->active = 1;
  wd_start(&t->wdog, MSEC2TICK(t->delay_ms), timer_wdog_handler,
           (wdparm_t)(uintptr_t)t);
}

void sys_timer_start_ext(os_timer_t *timer, uint32_t delay,
                         uint8_t from_isr)
{
  struct nx_timer_s *t = (struct nx_timer_s *)*timer;

  t->delay_ms = delay;
  t->active   = 1;
  wd_start(&t->wdog, MSEC2TICK(delay), timer_wdog_handler,
           (wdparm_t)(uintptr_t)t);
}

uint8_t sys_timer_stop(os_timer_t *timer, uint8_t from_isr)
{
  struct nx_timer_s *t = (struct nx_timer_s *)*timer;

  t->active = 0;
  wd_cancel(&t->wdog);
  return 1;
}

uint8_t sys_timer_pending(os_timer_t *timer)
{
  struct nx_timer_s *t = (struct nx_timer_s *)*timer;

  return (t->active && WDOG_ISACTIVE(&t->wdog)) ? 1 : 0;
}

/****************************************************************************
 * Public Functions - misc
 ****************************************************************************/

int32_t sys_random_bytes_get(void *dst, uint32_t size)
{
  /* Hardware TRNG via the SDK driver (self-initializing) */

  extern int random_get(unsigned char *dst, unsigned int size);
  return random_get(dst, size);
}

/* The Wi-Fi/BLE firmware polls MAC status inside sys_enter_critical()
 * sections while its own interrupts (programmed at a higher ECLIC level
 * by the SDK) must keep running - the FreeRTOS port implements this with
 * configMAX_SYSCALL_INTERRUPT_PRIORITY.  Model it with the ECLIC MTH
 * threshold: kernel-level sources (level 1) are masked, the radio
 * sources (level 8) still fire.  mstatus.MIE is left alone, so NuttX's
 * own enter_critical_section() remains the stronger global gate.
 */

#define GDWIFI_ECLIC_MTH  (GD32VW55X_ECLIC_BASE + 0x000b)

uint32_t sys_in_critical(void)
{
  return (up_interrupt_context() || g_crit_nest > 0) ? 1 : 0;
}

void sys_enter_critical(void)
{
  irqstate_t flags = enter_critical_section();

  if (g_crit_nest == 0)
    {
      *(volatile uint8_t *)GDWIFI_ECLIC_MTH = ECLIC_INTCTL_LEVEL(1);
    }

  g_crit_nest++;
  leave_critical_section(flags);
}

void sys_exit_critical(void)
{
  irqstate_t flags = enter_critical_section();

  DEBUGASSERT(g_crit_nest > 0);

  if (--g_crit_nest == 0)
    {
      *(volatile uint8_t *)GDWIFI_ECLIC_MTH = 0;
    }

  leave_critical_section(flags);
}

void sys_int_enter(void)
{
}

void sys_int_exit(void)
{
}

void sys_ps_set(uint8_t mode)
{
  sys_ps_mode = mode;
}

uint8_t sys_ps_get(void)
{
  return sys_ps_mode;
}

void sys_cpu_sleep_time_get(uint32_t *stats_ms, uint32_t *sleep_ms)
{
  if (stats_ms)
    {
      *stats_ms = sys_current_time_get();
    }

  if (sleep_ms)
    {
      *sleep_ms = 0;
    }
}

void sys_cpu_stats(void)
{
}

void sys_os_init(void)
{
  memset(g_tasks, 0, sizeof(g_tasks));
}

void sys_os_misc_init(void)
{
}

void sys_os_start(void)
{
  /* NuttX scheduler is already running */
}
