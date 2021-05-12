/****************************************************************************
 * drivers/wireless/ieee80211/amebaz/amebaz_depend.h
 *
 *   Copyright (C) 2019 Xiaomi Inc. All rights reserved.
 *   Author: Chao An <anchao@xiaomi.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __DRIVERS_WIRELESS_IEEE80211_AMEBAZ_AMEBAZ_DEPEND_H
#define __DRIVERS_WIRELESS_IEEE80211_AMEBAZ_AMEBAZ_DEPEND_H

#include <nuttx/config.h>

#include <malloc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <inttypes.h>

#include <arpa/inet.h>
#include <nuttx/wqueue.h>
#include <nuttx/kthread.h>

typedef struct  __queue         _queue;
typedef void (*thread_func_t)   (void *context);
typedef struct { volatile int counter; } atomic_t;

struct list_head
{
  struct list_head    *next;
  struct list_head    *prev;
};

struct  __queue
{
  struct  list_head   queue;
  void                *lock;
};

struct task_struct
{
  const char          *name;
  void                *priv;
  void                *wsema;
  void                *tsema;
  uint32_t            blocked;
  uint32_t            running;
};

struct nthread_wrapper
{
  int                 pid;
  thread_func_t       func;
  void                *thctx;
};

struct ntimer_wrapper
{
  struct work_s       work;
  void (*callback)    (void *context);
};

struct osdep_service_ops
{
  uint8_t   *(*rtw_vmalloc)                 (uint32_t n);
  uint8_t   *(*rtw_zvmalloc)                (uint32_t n);
  void      (*rtw_vmfree)                   (uint8_t *pbuf, uint32_t n);
  uint8_t   *(*rtw_malloc)                  (uint32_t n);
  uint8_t   *(*rtw_zmalloc)                 (uint32_t n);
  void      (*rtw_mfree)                    (uint8_t *pbuf, uint32_t n);
  void      (*rtw_memcpy)                   (void *dst, void *src, uint32_t n);
  int       (*rtw_memcmp)                   (void *dst, void *src, uint32_t n);
  void      (*rtw_memset)                   (void *pbuf, int c, uint32_t n);
  void      (*rtw_init_sema)                (void **sema, int init_val);
  void      (*rtw_free_sema)                (void **sema);
  void      (*rtw_up_sema)                  (void **sema);
  void      (*rtw_up_sema_from_isr)         (void **sema);
  uint32_t  (*rtw_down_timeout_sema)        (void **sema, uint32_t timeout);
  void      (*rtw_mutex_init)               (void **pmutex);
  void      (*rtw_mutex_free)               (void **pmutex);
  void      (*rtw_mutex_get)                (void **pmutex);
  int       (*rtw_mutex_get_timeout)        (void **pmutex, uint32_t timeout_ms);
  void      (*rtw_mutex_put)                (void **pmutex);
  void      (*rtw_enter_critical)           (void **plock, unsigned long *pirqL);
  void      (*rtw_exit_critical)            (void **plock, unsigned long *pirqL);
  void      (*rtw_enter_critical_from_isr)  (void **plock, unsigned long *pirqL);
  void      (*rtw_exit_critical_from_isr)   (void **plock, unsigned long *pirqL);
  void      (*rtw_enter_critical_bh)        (void **plock, unsigned long *pirqL);
  void      (*rtw_exit_critical_bh)         (void **plock, unsigned long *pirqL);
  int       (*rtw_enter_critical_mutex)     (void **pmutex, unsigned long *pirqL);
  void      (*rtw_exit_critical_mutex)      (void **pmutex, unsigned long *pirqL);
  void      (*rtw_cpu_lock)                 (void);
  void      (*rtw_cpu_unlock)               (void);
  void      (*rtw_spinlock_init)            (void **plock);
  void      (*rtw_spinlock_free)            (void **plock);
  void      (*rtw_spin_lock)                (void **plock);
  void      (*rtw_spin_unlock)              (void **plock);
  void      (*rtw_spinlock_irqsave)         (void **plock, unsigned long *irqL);
  void      (*rtw_spinunlock_irqsave)       (void **plock, unsigned long *irqL);
  int       (*rtw_init_xqueue)              (void **queue, const char *name, uint32_t size, uint32_t len);
  int       (*rtw_push_to_xqueue)           (void **queue, void *message, uint32_t timeout_ms);
  int       (*rtw_pop_from_xqueue)          (void **queue, void *message, uint32_t timeout_ms);
  int       (*rtw_deinit_xqueue)            (void **queue);
  uint32_t  (*rtw_get_current_time)         (void);
  uint32_t  (*rtw_systime_to_ms)            (uint32_t systime);
  uint32_t  (*rtw_systime_to_sec)           (uint32_t systime);
  uint32_t  (*rtw_ms_to_systime)            (uint32_t ms);
  uint32_t  (*rtw_sec_to_systime)           (uint32_t sec);
  void      (*rtw_msleep_os)                (int ms);
  void      (*rtw_usleep_os)                (int us);
  void      (*rtw_mdelay_os)                (int ms);
  void      (*rtw_udelay_os)                (int us);
  void      (*rtw_yield_os)                 (void);
  void      (*ATOMIC_SET)                   (atomic_t *v, int i);
  int       (*ATOMIC_READ)                  (atomic_t *v);
  void      (*ATOMIC_ADD)                   (atomic_t *v, int i);
  void      (*ATOMIC_SUB)                   (atomic_t *v, int i);
  void      (*ATOMIC_INC)                   (atomic_t *v);
  void      (*ATOMIC_DEC)                   (atomic_t *v);
  int       (*ATOMIC_ADD_RETURN)            (atomic_t *v, int i);
  int       (*ATOMIC_SUB_RETURN)            (atomic_t *v, int i);
  int       (*ATOMIC_INC_RETURN)            (atomic_t *v);
  int       (*ATOMIC_DEC_RETURN)            (atomic_t *v);
  uint64_t  (*rtw_modular64)                (uint64_t x, uint64_t y);
  int       (*rtw_get_random_bytes)         (void* dst, uint32_t size);
  uint32_t  (*rtw_getFreeHeapSize)          (void);
  int       (*rtw_create_task)              (struct task_struct *task, const char *name, uint32_t stack_size,
                                             uint32_t priority, thread_func_t func, void *thctx);
  void      (*rtw_delete_task)              (struct task_struct *task);
  void      (*rtw_wakeup_task)              (struct task_struct *task);
  void      (*rtw_thread_enter)             (char *name);
  void      (*rtw_thread_exit)              (void);
  void      *(*rtw_timerCreate)             (const signed char *pcTimerName, unsigned long xTimerPeriodInTicks,
                                             uint32_t uxAutoReload, void *pvTimerID, thread_func_t pxCallbackFunction);
  uint32_t  (*rtw_timerDelete)              (void *xTimer, unsigned long xBlockTime);
  uint32_t  (*rtw_timerIsTimerActive)       (void *xTimer);
  uint32_t  (*rtw_timerStop)                (void *xTimer, unsigned long xBlockTime);
  uint32_t  (*rtw_timerChangePeriod)        (void *xTimer, unsigned long xNewPeriod, unsigned long xBlockTime);
  void      *(*rtw_timerGetID)              (void *xTimer);
  uint32_t  (*rtw_timerStart)               (void *xTimer, unsigned long xBlockTime);
  uint32_t  (*rtw_timerStartFromISR)        (void *xTimer, long *pxHigherPriorityTaskWoken);
  uint32_t  (*rtw_timerStopFromISR)         (void *xTimer, long *pxHigherPriorityTaskWoken);
  uint32_t  (*rtw_timerResetFromISR)        (void *xTimer, long *pxHigherPriorityTaskWoken);
  uint32_t  (*rtw_timerChangePeriodFromISR) (void *xTimer, unsigned long xNewPeriod, long *pxHigherPriorityTaskWoken);
  uint32_t  (*rtw_timerReset)               (void *xTimer, unsigned long xBlockTime);
  void      (*rtw_acquire_wakelock)         (void);
  void      (*rtw_release_wakelock)         (void);
  void      (*rtw_wakelock_timeout)         (uint32_t ms);
  uint8_t   (*rtw_get_scheduler_state)      (void);
  void      (*rtw_create_secure_context)    (uint32_t n);
};

#endif
