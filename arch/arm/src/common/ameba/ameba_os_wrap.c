/****************************************************************************
 * arch/arm/src/common/ameba/ameba_os_wrap.c
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
 * Description
 *
 * NuttX-backed implementation of the Realtek "os_wrapper" (rtos_*) API.
 *
 * The Realtek Ameba chip libraries (fwlib, hal, swlib, soc) are
 * written against the SDK's OS-abstraction layer (os_wrapper.h, the rtos_*
 * functions).  In the upstream SDK these resolve to a FreeRTOS backend.  In
 * the NuttX port NuttX owns the image and provides the scheduler, so this
 * file re-implements the same rtos_* symbols on top of NuttX primitives
 * (nxsem / nxmutex / kmm / kthread / clock).  The SDK's FreeRTOS backend
 * (lib_freertos_os_wrapper.a) is therefore NOT linked.
 *
 * Only the subset of the os_wrapper API that the *kept* chip libraries
 * reference (verified with nm) is implemented here.  Unused entry points are
 * intentionally omitted so the linker never pulls in dead code.
 *
 * Signatures and return conventions are taken verbatim from the SDK headers
 * in component/os/os_wrapper/include/ (RTK_SUCCESS == 0, RTK_FAIL == -1).
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <malloc.h>

#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/mutex.h>
#include <nuttx/kthread.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/signal.h>
#include <nuttx/wqueue.h>

#include "arm_internal.h"
#include "ameba_os_wrap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RTK_SUCCESS              (0)
#define RTK_FAIL                 (-1)

/* "Block forever" sentinel used by the SDK timeout arguments. */

#define RTOS_MAX_DELAY           (0xffffffffu)

/* The SDK task API uses "higher value == higher priority" with a maximum of
 * RTOS_TASK_MAX_PRIORITIES (11).  NuttX uses the same direction but a 1..255
 * range.  Map the SDK band onto a NuttX band that sits comfortably below the
 * high-priority kernel threads but above the idle thread.
 */

#define RTOS_TASK_MAX_PRIORITIES (11)
#define AMEBA_TASK_PRIO_BASE     (100)

/* Maximum nesting depth tracked for rtos_critical_enter/exit. */

#define AMEBA_CRITICAL_NEST_MAX  (16)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* These mirror the opaque handle and callback typedefs from the SDK headers
 * (os_wrapper_semaphore.h / _mutex.h / _task.h).  They are replicated here
 * rather than including the SDK headers to avoid dragging in the rest of the
 * SDK build configuration.  The underlying types MUST stay (void *) so the
 * ABI matches the chip libraries that were compiled against the SDK headers.
 */

typedef void *rtos_sema_t;
typedef void *rtos_mutex_t;
typedef void *rtos_task_t;
typedef void *rtos_timer_t;
typedef void (*rtos_task_function_t)(void *);

/* Trampoline context: NuttX kernel threads have a main_t entry
 * (int (*)(int, char **)) while the SDK passes void (*)(void *).  We bridge
 * the two by allocating this context, encoding its address into argv[1] and
 * recovering it in ameba_task_trampoline().
 */

struct ameba_task_ctx_s
{
  void (*routine)(void *);
  void *param;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ameba_task_trampoline(int argc, char *argv[])
{
  struct ameba_task_ctx_s *ctx;
  void (*routine)(void *);
  void *param;

  /* argv[1] holds the context pointer encoded as a hex string. */

  ctx = (struct ameba_task_ctx_s *)(uintptr_t)strtoul(argv[1], NULL, 16);
  routine = ctx->routine;
  param   = ctx->param;
  kmm_free(ctx);

  routine(param);

  /* SDK tasks are expected never to return; if one does, self-delete. */

  return 0;
}

/****************************************************************************
 * Public Functions: memory (os_wrapper_memory.h)
 ****************************************************************************/

void rtos_mem_init(void)
{
  /* NuttX initialises its heap during nx_start(); nothing to do here. */
}

bool os_heap_add(uint8_t *start_addr, size_t heap_size)
{
  /* The NuttX heap is fixed at nx_start() time (see ameba_allocateheap.c).
   * Dynamic SDK heap-region donation (used by the SDK only for PSRAM, which
   * this board lacks) is therefore a no-op.
   */

  UNUSED(start_addr);
  UNUSED(heap_size);
  return false;
}

/* The WiFi WHC skb pool (and other DMA buffers shared with the NP) must be
 * cache-line aligned: the driver does DCache_Clean/Invalidate on them and
 * the NP DMAs them, so an unaligned base would clobber neighbouring data on
 * cache maintenance.  whc_ipc_host_init_skb() outright rejects an unaligned
 * skb_data_buf ("skb_data_buf malloc fail!"), leaving the skb pool empty and
 * the TX path handing the NP a garbage buffer pointer.  This is guaranteed
 * by CONFIG_MM_DEFAULT_ALIGNMENT=32 (>= SKB_CACHE_SZ) -- every NuttX heap
 * block is then cache-line aligned, so plain kmm_* suffices here.
 */

void *rtos_mem_malloc(uint32_t size)
{
  return kmm_malloc((size_t)size);
}

void *rtos_mem_zmalloc(uint32_t size)
{
  return kmm_zalloc((size_t)size);
}

void *rtos_mem_calloc(uint32_t element_num, uint32_t element_size)
{
  return kmm_calloc((size_t)element_num, (size_t)element_size);
}

void *rtos_mem_realloc(void *pbuf, uint32_t size)
{
  return kmm_realloc(pbuf, (size_t)size);
}

void rtos_mem_free(void *pbuf)
{
  if (pbuf != NULL)
    {
      kmm_free(pbuf);
    }
}

/****************************************************************************
 * Public Functions: unified object helpers (shared with FreeRTOS shim)
 ****************************************************************************/

struct ameba_qobj_s *ameba_qobj_alloc(uint8_t tag)
{
  struct ameba_qobj_s *obj = kmm_zalloc(sizeof(struct ameba_qobj_s));
  int ret;

  if (obj == NULL)
    {
      return NULL;
    }

  obj->tag = tag;

  switch (tag)
    {
      case AMEBA_QOBJ_MUTEX:
        ret = nxmutex_init(&obj->u.mutex);
        break;

      case AMEBA_QOBJ_RMUTEX:
        ret = nxrmutex_init(&obj->u.rmutex);
        break;

      case AMEBA_QOBJ_SEM:
      default:

        /* Caller initialises the semaphore counts via nxsem_init below. */

        ret = OK;
        break;
    }

  if (ret < 0)
    {
      kmm_free(obj);
      return NULL;
    }

  return obj;
}

void ameba_qobj_free(struct ameba_qobj_s *obj)
{
  if (obj == NULL)
    {
      return;
    }

  switch (obj->tag)
    {
      case AMEBA_QOBJ_SEM:
        nxsem_destroy(&obj->u.sem);
        break;

      case AMEBA_QOBJ_MUTEX:
        nxmutex_destroy(&obj->u.mutex);
        break;

      case AMEBA_QOBJ_RMUTEX:
        nxrmutex_destroy(&obj->u.rmutex);
        break;

      default:
        break;
    }

  kmm_free(obj);
}

int ameba_qobj_take(struct ameba_qobj_s *obj, uint32_t ms)
{
  int ret;

  if (obj == NULL)
    {
      return RTK_FAIL;
    }

  switch (obj->tag)
    {
      case AMEBA_QOBJ_MUTEX:
        if (up_interrupt_context() || ms == 0)
          {
            ret = nxmutex_trylock(&obj->u.mutex);
          }
        else
          {
            ret = nxmutex_lock(&obj->u.mutex);
          }
        break;

      case AMEBA_QOBJ_RMUTEX:
        if (up_interrupt_context() || ms == 0)
          {
            ret = nxrmutex_trylock(&obj->u.rmutex);
          }
        else
          {
            ret = nxrmutex_lock(&obj->u.rmutex);
          }
        break;

      case AMEBA_QOBJ_SEM:
      default:
        if (up_interrupt_context() || ms == 0)
          {
            ret = nxsem_trywait(&obj->u.sem);
          }
        else if (ms == RTOS_MAX_DELAY)
          {
            ret = nxsem_wait_uninterruptible(&obj->u.sem);
          }
        else
          {
            ret = nxsem_tickwait_uninterruptible(&obj->u.sem,
                                                 MSEC2TICK(ms));
          }
        break;
    }

  return (ret == OK) ? RTK_SUCCESS : RTK_FAIL;
}

int ameba_qobj_give(struct ameba_qobj_s *obj)
{
  int ret;

  if (obj == NULL)
    {
      return RTK_FAIL;
    }

  switch (obj->tag)
    {
      case AMEBA_QOBJ_MUTEX:
        ret = nxmutex_unlock(&obj->u.mutex);
        break;

      case AMEBA_QOBJ_RMUTEX:
        ret = nxrmutex_unlock(&obj->u.rmutex);
        break;

      case AMEBA_QOBJ_SEM:
      default:
        ret = nxsem_post(&obj->u.sem);
        break;
    }

  return (ret == OK) ? RTK_SUCCESS : RTK_FAIL;
}

/****************************************************************************
 * Public Functions: semaphore (os_wrapper_semaphore.h)
 ****************************************************************************/

int rtos_sema_create(rtos_sema_t *pp_handle, uint32_t init_count,
                     uint32_t max_count)
{
  struct ameba_qobj_s *obj;

  UNUSED(max_count);

  if (pp_handle == NULL)
    {
      return RTK_FAIL;
    }

  obj = ameba_qobj_alloc(AMEBA_QOBJ_SEM);
  if (obj == NULL)
    {
      return RTK_FAIL;
    }

  if (nxsem_init(&obj->u.sem, 0, init_count) < 0)
    {
      kmm_free(obj);
      return RTK_FAIL;
    }

  *pp_handle = (rtos_sema_t)obj;
  return RTK_SUCCESS;
}

int rtos_sema_create_binary(rtos_sema_t *pp_handle)
{
  return rtos_sema_create(pp_handle, 0, 1);
}

int rtos_sema_delete(rtos_sema_t p_handle)
{
  if (p_handle == NULL)
    {
      return RTK_FAIL;
    }

  ameba_qobj_free((struct ameba_qobj_s *)p_handle);
  return RTK_SUCCESS;
}

int rtos_sema_take(rtos_sema_t p_handle, uint32_t timeout_ms)
{
  return ameba_qobj_take((struct ameba_qobj_s *)p_handle, timeout_ms);
}

int rtos_sema_give(rtos_sema_t p_handle)
{
  return ameba_qobj_give((struct ameba_qobj_s *)p_handle);
}

uint32_t rtos_sema_get_count(rtos_sema_t p_handle)
{
  struct ameba_qobj_s *obj = (struct ameba_qobj_s *)p_handle;
  int count = 0;

  if (obj != NULL && obj->tag == AMEBA_QOBJ_SEM)
    {
      nxsem_get_value(&obj->u.sem, &count);
    }

  return (uint32_t)count;
}

/****************************************************************************
 * Public Functions: mutex (os_wrapper_mutex.h)
 ****************************************************************************/

int rtos_mutex_create(rtos_mutex_t *pp_handle)
{
  struct ameba_qobj_s *obj;

  if (pp_handle == NULL)
    {
      return RTK_FAIL;
    }

  obj = ameba_qobj_alloc(AMEBA_QOBJ_MUTEX);
  if (obj == NULL)
    {
      return RTK_FAIL;
    }

  *pp_handle = (rtos_mutex_t)obj;
  return RTK_SUCCESS;
}

int rtos_mutex_delete(rtos_mutex_t p_handle)
{
  if (p_handle == NULL)
    {
      return RTK_FAIL;
    }

  ameba_qobj_free((struct ameba_qobj_s *)p_handle);
  return RTK_SUCCESS;
}

int rtos_mutex_take(rtos_mutex_t p_handle, uint32_t wait_ms)
{
  return ameba_qobj_take((struct ameba_qobj_s *)p_handle, wait_ms);
}

int rtos_mutex_give(rtos_mutex_t p_handle)
{
  return ameba_qobj_give((struct ameba_qobj_s *)p_handle);
}

/****************************************************************************
 * Public Functions: task / scheduler (os_wrapper_task.h)
 ****************************************************************************/

int rtos_task_create(rtos_task_t *pp_handle, const char *p_name,
                     rtos_task_function_t p_routine, void *p_param,
                     size_t stack_size_in_byte, uint16_t priority)
{
  struct ameba_task_ctx_s *ctx;
  char argbuf[2 + sizeof(uintptr_t) * 2 + 1];
  char *argv[2];
  int prio;
  int pid;

  if (p_routine == NULL)
    {
      return RTK_FAIL;
    }

  ctx = kmm_malloc(sizeof(struct ameba_task_ctx_s));
  if (ctx == NULL)
    {
      return RTK_FAIL;
    }

  ctx->routine = p_routine;
  ctx->param   = p_param;

  snprintf(argbuf, sizeof(argbuf), "%p", (void *)ctx);
  argv[0] = argbuf;
  argv[1] = NULL;

  /* Map the SDK priority band (1..RTOS_TASK_MAX_PRIORITIES) onto a NuttX
   * band centred on AMEBA_TASK_PRIO_BASE, clamped to the legal range.
   */

  prio = AMEBA_TASK_PRIO_BASE + (int)priority;
  if (prio >= SCHED_PRIORITY_MAX)
    {
      prio = SCHED_PRIORITY_MAX - 1;
    }
  else if (prio <= SCHED_PRIORITY_MIN)
    {
      prio = SCHED_PRIORITY_MIN + 1;
    }

  pid = kthread_create(p_name ? p_name : "ameba",
                       prio, (int)stack_size_in_byte,
                       ameba_task_trampoline, argv);
  if (pid < 0)
    {
      kmm_free(ctx);
      return RTK_FAIL;
    }

  if (pp_handle != NULL)
    {
      *pp_handle = (rtos_task_t)(uintptr_t)pid;
    }

  return RTK_SUCCESS;
}

int rtos_task_delete(rtos_task_t p_handle)
{
  pid_t pid = (pid_t)(uintptr_t)p_handle;

  if (p_handle == NULL)
    {
      /* Delete the calling task. */

      pid = nxsched_gettid();
    }

  kthread_delete(pid);
  return RTK_SUCCESS;
}

uint32_t rtos_task_priority_get(rtos_task_t p_handle)
{
  struct sched_param param;
  pid_t pid = (pid_t)(uintptr_t)p_handle;

  if (pid == 0)
    {
      pid = nxsched_gettid();
    }

  if (nxsched_get_param(pid, &param) < 0)
    {
      return 0;
    }

  /* Map the NuttX priority back onto the SDK band (see rtos_task_create). */

  return (uint32_t)(param.sched_priority - AMEBA_TASK_PRIO_BASE);
}

int rtos_task_priority_set(rtos_task_t p_handle, uint16_t priority)
{
  struct sched_param param;
  pid_t pid = (pid_t)(uintptr_t)p_handle;
  int prio = AMEBA_TASK_PRIO_BASE + (int)priority;

  if (prio >= SCHED_PRIORITY_MAX)
    {
      prio = SCHED_PRIORITY_MAX - 1;
    }
  else if (prio <= SCHED_PRIORITY_MIN)
    {
      prio = SCHED_PRIORITY_MIN + 1;
    }

  if (pid == 0)
    {
      pid = nxsched_gettid();
    }

  param.sched_priority = prio;
  return (nxsched_set_param(pid, &param) < 0) ? RTK_FAIL : RTK_SUCCESS;
}

/* Run p_func(arg1, arg2) from the LP work queue after wait_ms (deferred
 * call, used by the WPA supplicant).  Mirrors FreeRTOS
 * xTimerPendFunctionCall.
 */

struct ameba_pend_call_s
{
  struct work_s work;
  void (*func)(void *, uint32_t);
  void *arg1;
  uint32_t arg2;
};

static void ameba_pend_call_trampoline(void *arg)
{
  struct ameba_pend_call_s *p = (struct ameba_pend_call_s *)arg;

  p->func(p->arg1, p->arg2);
  kmm_free(p);
}

int rtos_timer_pend_function_call(void (*p_func)(void *, uint32_t),
                                  void *pv_parameter1,
                                  uint32_t ul_parameter2,
                                  uint32_t wait_ms)
{
  struct ameba_pend_call_s *p;

  if (p_func == NULL)
    {
      return RTK_FAIL;
    }

  p = kmm_malloc(sizeof(struct ameba_pend_call_s));
  if (p == NULL)
    {
      return RTK_FAIL;
    }

  memset(&p->work, 0, sizeof(p->work));
  p->func = p_func;
  p->arg1 = pv_parameter1;
  p->arg2 = ul_parameter2;
  work_queue(LPWORK, &p->work, ameba_pend_call_trampoline, p,
             MSEC2TICK(wait_ms));
  return RTK_SUCCESS;
}

int rtos_sched_suspend(void)
{
  /* NuttX has no global "suspend all" that is safe to expose to the chip
   * libraries; the only callers are coarse critical sections.  Use a
   * lightweight scheduler lock instead.
   */

  sched_lock();
  return RTK_SUCCESS;
}

int rtos_sched_resume(void)
{
  sched_unlock();
  return RTK_SUCCESS;
}

/****************************************************************************
 * Public Functions: critical (os_wrapper_critical.h)
 ****************************************************************************/

int rtos_critical_is_in_interrupt(void)
{
  return up_interrupt_context() ? 1 : 0;
}

/* Nested critical section support.  The SDK API has no "flags" output
 * parameter; it relies on the backend to stack the saved interrupt state
 * internally.  We keep a small saved-state stack so nested enter/exit pairs
 * restore the correct PRIMASK on the way out.
 */

static irqstate_t g_critical_flags[AMEBA_CRITICAL_NEST_MAX];
static volatile uint32_t g_critical_nest;

void rtos_critical_enter(uint32_t component_id)
{
  irqstate_t flags;

  UNUSED(component_id);

  flags = enter_critical_section();

  if (g_critical_nest < AMEBA_CRITICAL_NEST_MAX)
    {
      g_critical_flags[g_critical_nest] = flags;
    }

  g_critical_nest++;
}

void rtos_critical_exit(uint32_t component_id)
{
  irqstate_t flags = 0;

  UNUSED(component_id);

  if (g_critical_nest > 0)
    {
      g_critical_nest--;
      if (g_critical_nest < AMEBA_CRITICAL_NEST_MAX)
        {
          flags = g_critical_flags[g_critical_nest];
        }
    }

  leave_critical_section(flags);
}

void __rtos_critical_enter_os(void)
{
  rtos_critical_enter(0);
}

void __rtos_critical_exit_os(void)
{
  rtos_critical_exit(0);
}

uint32_t rtos_get_critical_state(void)
{
  return g_critical_nest;
}

/****************************************************************************
 * Public Functions: time (os_wrapper_time.h)
 ****************************************************************************/

uint32_t rtos_time_get_current_system_time_ms(void)
{
  return (uint32_t)TICK2MSEC(clock_systime_ticks());
}

/****************************************************************************
 * Public Functions: static mutex / sema (os_wrapper_*.h)
 *
 * The SDK's "_static" variants exist because FreeRTOS lets the caller
 * supply the object storage.  Here the handle is already a pointer to our
 * own heap-allocated wrapper, so the static and dynamic variants are
 * identical -- just delegate.
 ****************************************************************************/

int rtos_mutex_create_static(rtos_mutex_t *pp_handle)
{
  return rtos_mutex_create(pp_handle);
}

int rtos_mutex_delete_static(rtos_mutex_t p_handle)
{
  return rtos_mutex_delete(p_handle);
}

int rtos_sema_create_static(rtos_sema_t *pp_handle, uint32_t init_count,
                            uint32_t max_count)
{
  return rtos_sema_create(pp_handle, init_count, max_count);
}

int rtos_sema_create_binary_static(rtos_sema_t *pp_handle)
{
  return rtos_sema_create(pp_handle, 0, 1);
}

int rtos_sema_delete_static(rtos_sema_t p_handle)
{
  return rtos_sema_delete(p_handle);
}

/****************************************************************************
 * Public Functions: memory / time extras (os_wrapper_memory.h, _time.h)
 ****************************************************************************/

uint32_t rtos_mem_get_free_heap_size(void)
{
  struct mallinfo info = mallinfo();
  return (uint32_t)info.fordblks;
}

void rtos_time_delay_ms(uint32_t ms)
{
  if (ms != 0)
    {
      nxsig_usleep(ms * 1000);
    }
}

/****************************************************************************
 * Public Functions: software timers (os_wrapper_timer.h)
 *
 * FreeRTOS-style software timers, backed by the NuttX low-priority work
 * queue so the timer callback runs in task context (the SDK's WiFi timer
 * callbacks may call back into the rtos_ / wifi APIs, so an ISR/wdog context
 * is unsafe).  reload == auto-reload (periodic); otherwise one-shot.
 ****************************************************************************/

struct ameba_timer_s
{
  struct work_s     work;
  void            (*cb)(void *);
  uint32_t          id;
  uint32_t          period_ticks;
  bool              reload;
  volatile bool     active;
};

static void ameba_timer_dispatch(void *arg)
{
  struct ameba_timer_s *t = (struct ameba_timer_s *)arg;

  if (!t->active)
    {
      return;
    }

  /* FreeRTOS passes the timer handle to the callback. */

  t->cb((void *)t);

  if (t->active && t->reload)
    {
      work_queue(LPWORK, &t->work, ameba_timer_dispatch, t, t->period_ticks);
    }
  else
    {
      t->active = false;
    }
}

int rtos_timer_create(rtos_timer_t *pp_handle, const char *p_timer_name,
                      uint32_t timer_id, uint32_t interval_ms,
                      uint8_t reload, void (*p_timer_callback)(void *))
{
  struct ameba_timer_s *t;

  UNUSED(p_timer_name);

  if (pp_handle == NULL || p_timer_callback == NULL)
    {
      return RTK_FAIL;
    }

  t = (struct ameba_timer_s *)kmm_zalloc(sizeof(*t));
  if (t == NULL)
    {
      return RTK_FAIL;
    }

  t->cb           = p_timer_callback;
  t->id           = timer_id;
  t->reload       = (reload != 0);
  t->period_ticks = MSEC2TICK(interval_ms);
  if (t->period_ticks == 0)
    {
      t->period_ticks = 1;
    }

  *pp_handle = (rtos_timer_t)t;
  return RTK_SUCCESS;
}

int rtos_timer_create_static(rtos_timer_t *pp_handle,
                             const char *p_timer_name,
                             uint32_t timer_id, uint32_t interval_ms,
                             uint8_t reload,
                             void (*p_timer_callback)(void *))
{
  return rtos_timer_create(pp_handle, p_timer_name, timer_id, interval_ms,
                           reload, p_timer_callback);
}

int rtos_timer_start(rtos_timer_t p_handle, uint32_t wait_ms)
{
  struct ameba_timer_s *t = (struct ameba_timer_s *)p_handle;

  UNUSED(wait_ms);

  if (t == NULL)
    {
      return RTK_FAIL;
    }

  t->active = true;
  work_queue(LPWORK, &t->work, ameba_timer_dispatch, t, t->period_ticks);
  return RTK_SUCCESS;
}

int rtos_timer_stop(rtos_timer_t p_handle, uint32_t wait_ms)
{
  struct ameba_timer_s *t = (struct ameba_timer_s *)p_handle;

  UNUSED(wait_ms);

  if (t == NULL)
    {
      return RTK_FAIL;
    }

  t->active = false;
  work_cancel(LPWORK, &t->work);
  return RTK_SUCCESS;
}

int rtos_timer_change_period(rtos_timer_t p_handle, uint32_t interval_ms,
                             uint32_t wait_ms)
{
  struct ameba_timer_s *t = (struct ameba_timer_s *)p_handle;

  UNUSED(wait_ms);

  if (t == NULL)
    {
      return RTK_FAIL;
    }

  t->period_ticks = MSEC2TICK(interval_ms);
  if (t->period_ticks == 0)
    {
      t->period_ticks = 1;
    }

  t->active = true;

  /* work_queue on an already-queued work re-schedules it. */

  work_queue(LPWORK, &t->work, ameba_timer_dispatch, t, t->period_ticks);
  return RTK_SUCCESS;
}

uint32_t rtos_timer_is_timer_active(rtos_timer_t p_handle)
{
  struct ameba_timer_s *t = (struct ameba_timer_s *)p_handle;
  return (t != NULL && t->active) ? 1 : 0;
}

int rtos_timer_delete(rtos_timer_t p_handle, uint32_t wait_ms)
{
  struct ameba_timer_s *t = (struct ameba_timer_s *)p_handle;

  UNUSED(wait_ms);

  if (t == NULL)
    {
      return RTK_FAIL;
    }

  t->active = false;
  work_cancel(LPWORK, &t->work);
  kmm_free(t);
  return RTK_SUCCESS;
}

int rtos_timer_delete_static(rtos_timer_t p_handle, uint32_t wait_ms)
{
  return rtos_timer_delete(p_handle, wait_ms);
}

uint32_t rtos_timer_get_id(rtos_timer_t p_handle)
{
  struct ameba_timer_s *t = (struct ameba_timer_s *)p_handle;
  return (t != NULL) ? t->id : 0;
}

/* rtos_queue_send/receive: referenced by the SDK event source (rtw_event.c)
 * from its wifi_cast / promisc paths, which the NuttX STA port does not use.
 * The symbols must resolve (rtw_event.o is pulled in for wifi_event_handle),
 * but the functions are never exercised -- provide failing no-ops.  Queues
 * are otherwise not part of the os_wrapper subset NuttX backs.
 */

int rtos_queue_send(void *p_handle, void *p_msg, uint32_t wait_ms)
{
  (void)p_handle;
  (void)p_msg;
  (void)wait_ms;
  return RTK_FAIL;
}

int rtos_queue_receive(void *p_handle, void *p_msg, uint32_t wait_ms)
{
  (void)p_handle;
  (void)p_msg;
  (void)wait_ms;
  return RTK_FAIL;
}
