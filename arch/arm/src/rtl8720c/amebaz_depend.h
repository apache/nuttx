/****************************************************************************
 * arch/arm/src/rtl8720c/amebaz_depend.h
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

#ifndef __DRIVERS_WIRELESS_IEEE80211_AMEBAZ_AMEBAZ_DEPEND_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define __DRIVERS_WIRELESS_IEEE80211_AMEBAZ_AMEBAZ_DEPEND_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

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
typedef void (*thread_func_t)(void *context);
typedef struct
{
  volatile int counter;
} atomic_t;
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
  uint32_t blocked;
  uint32_t running;
};
struct nthread_wrapper
{
  int pid;
  thread_func_t       func;
  void                *thctx;
};
struct ntimer_wrapper
{
  struct work_s       work;
  void (*callback)(void *context);
};
struct osdep_service_ops
{
  uint8_t   *(*rtw_vmalloc)(uint32_t n);
  uint8_t   *(*rtw_zvmalloc)(uint32_t n);
  void (*rtw_vmfree)(uint8_t *pbuf, uint32_t n);
  uint8_t   *(*rtw_malloc)(uint32_t n);
  uint8_t   *(*rtw_zmalloc)(uint32_t n);
  void (*rtw_mfree)(uint8_t *pbuf, uint32_t n);
  void (*rtw_memcpy)(void *dst, void *src, uint32_t n);
  int (*rtw_memcmp)(void *dst, void *src, uint32_t n);
  void (*rtw_memset)(void *pbuf, int c, uint32_t n);
  void (*rtw_init_sema)(void **sema, int init_val);
  void (*rtw_free_sema)(void **sema);
  void (*rtw_up_sema)(void **sema);
  void (*rtw_up_sema_from_isr)(void **sema);
  uint32_t (*rtw_down_timeout_sema)(void **sema, uint32_t timeout);
  void (*rtw_mutex_init)(void **pmutex);
  void (*rtw_mutex_free)(void **pmutex);
  void (*rtw_mutex_get)(void **pmutex);
  int (*rtw_mutex_get_timeout)(void **pmutex, uint32_t timeout_ms);
  void (*rtw_mutex_put)(void **pmutex);
  void (*rtw_enter_critical)(void **plock, unsigned long *pirql);
  void (*rtw_exit_critical)(void **plock, unsigned long *pirql);
  void (*rtw_enter_critical_from_isr)(void **plock, unsigned long *pirql);
  void (*rtw_exit_critical_from_isr)(void **plock, unsigned long *pirql);
  void (*rtw_enter_critical_bh)(void **plock, unsigned long *pirql);
  void (*rtw_exit_critical_bh)(void **plock, unsigned long *pirql);
  int (*rtw_enter_critical_mutex)(void **pmutex, unsigned long *pirql);
  void (*rtw_exit_critical_mutex)(void **pmutex, unsigned long *pirql);
  void (*rtw_cpu_lock)(void);
  void (*rtw_cpu_unlock)(void);
  void (*rtw_spinlock_init)(void **plock);
  void (*rtw_spinlock_free)(void **plock);
  void (*rtw_spin_lock)(void **plock);
  void (*rtw_spin_unlock)(void **plock);
  void (*rtw_spinlock_irqsave)(void **plock, unsigned long *pirql);
  void (*rtw_spinunlock_irqsave)(void **plock, unsigned long *pirql);
  int (*rtw_init_xqueue)(void **queue, const char *name, uint32_t size,
                         uint32_t len);
  int (*rtw_push_to_xqueue)(void **queue, void *message,
                            uint32_t timeout_ms);
  int (*rtw_pop_from_xqueue)(void **queue, void *message,
                             uint32_t timeout_ms);
  int (*rtw_deinit_xqueue)(void **queue);
  uint32_t (*rtw_get_current_time)(void);
  uint32_t (*rtw_systime_to_ms)(uint32_t systime);
  uint32_t (*rtw_systime_to_sec)(uint32_t systime);
  uint32_t (*rtw_ms_to_systime)(uint32_t ms);
  uint32_t (*rtw_sec_to_systime)(uint32_t sec);
  void (*rtw_msleep_os)(int ms);
  void (*rtw_usleep_os)(int us);
  void (*rtw_mdelay_os)(int ms);
  void (*rtw_udelay_os)(int us);
  void (*rtw_yield_os)(void);
  void (*ATOMIC_SET)(atomic_t *v, int i);
  int (*ATOMIC_READ)(atomic_t *v);
  void (*ATOMIC_ADD)(atomic_t *v, int i);
  void (*ATOMIC_SUB)(atomic_t *v, int i);
  void (*ATOMIC_INC)(atomic_t *v);
  void (*ATOMIC_DEC)(atomic_t *v);
  int (*ATOMIC_ADD_RETURN)(atomic_t *v, int i);
  int (*ATOMIC_SUB_RETURN)(atomic_t *v, int i);
  int (*ATOMIC_INC_RETURN)(atomic_t *v);
  int (*ATOMIC_DEC_RETURN)(atomic_t *v);
  uint64_t (*rtw_modular64)(uint64_t x, uint64_t y);
  int (*rtw_get_random_bytes)(void *dst, uint32_t size);
  uint32_t (*rtw_get_free_heap_size)(void);
  int (*rtw_create_task)(struct task_struct *task, const char *name,
                         uint32_t stack_size,
                         uint32_t priority, thread_func_t func, void *thctx);
  void (*rtw_delete_task)(struct task_struct *task);
  void (*rtw_wakeup_task)(struct task_struct *task);
  void (*rtw_thread_enter)(char *name);
  void (*rtw_thread_exit)(void);
  void      *(*rtw_timer_create)(const signed char *pctimername,
                                unsigned long xtimerperiodinticks,
                                uint32_t uxautoreload,
                                void *pvtimerid,
                                thread_func_t pxcallbackfunction);
  uint32_t (*rtw_timer_delete)(void *xtimer, unsigned long xblocktime);
  uint32_t (*rtw_timer_is_timer_active)(void *xtimer);
  uint32_t (*rtw_timer_stop)(void *xtimer, unsigned long xblocktime);
  uint32_t (*rtw_timer_change_period)(void *xtimer, unsigned long xnewperiod,
                                    unsigned long xblocktime);
  void      *(*rtw_timer_get_id)(void *xtimer);
  uint32_t (*rtw_timer_start)(void *xtimer, unsigned long xblocktime);
  uint32_t (*rtw_timer_start_from_isr)(void *xtimer,
                                    long *pxhigherprioritytaskwoken);
  uint32_t (*rtw_timer_stop_from_isr)(void *xtimer,
                                   long *pxhigherprioritytaskwoken);
  uint32_t (*rtw_timer_reset_from_isr) (void *xtimer,
                                      long *pxhigherprioritytaskwoken);
  uint32_t (*rtw_timer_change_period_from_isr) (void *xtimer,
                                            unsigned long xnewperiod,
                                            long *pxhigherprioritytaskwoken);
  uint32_t (*rtw_timer_reset)(void *xtimer, unsigned long xblocktime);
  void (*rtw_acquire_wakelock)(void);
  void (*rtw_release_wakelock)(void);
  void (*rtw_wakelock_timeout)(uint32_t ms);
  uint8_t (*rtw_get_scheduler_state)(void);
  void (*rtw_create_secure_context)(uint32_t n);
};
#endif
