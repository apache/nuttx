/****************************************************************************
 * net/lwip/sys_arch_nuttx.c
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
#ifdef CONFIG_NET_LWIP

#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <nuttx/irq.h>

#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/debug.h"
#include "lwip/err.h"
#include "lwip/mem.h"
#include "lwip/stats.h"
#include "lwip/sys.h"

#include "arch/sys_arch.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sys_mbox_s
{
  pthread_mutex_t lock;
  sem_t slots;
  sem_t items;
  int size;
  int head;
  int tail;
  void **msgs;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t ts_to_ms(const struct timespec *ts)
{
  return (uint32_t)(ts->tv_sec * 1000u + ts->tv_nsec / 1000000u);
}

static void make_abs_timespec(struct timespec *abstime, uint32_t timeout_ms)
{
  clock_gettime(CLOCK_REALTIME, abstime);
  abstime->tv_sec  += timeout_ms / 1000u;
  abstime->tv_nsec += (long)(timeout_ms % 1000u) * 1000000L;
  if (abstime->tv_nsec >= 1000000000L)
    {
      abstime->tv_sec += 1;
      abstime->tv_nsec -= 1000000000L;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void sys_init(void)
{
}

err_t sys_sem_new(sys_sem_t *sem, u8_t count)
{
  return sem_init(sem, 0, count) == 0 ? ERR_OK : ERR_MEM;
}

void sys_sem_free(sys_sem_t *sem)
{
  sem_destroy(sem);
}

void sys_sem_signal(sys_sem_t *sem)
{
  sem_post(sem);
}

u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
  struct timespec start;
  struct timespec end;
  struct timespec abstime;
  int ret;

  clock_gettime(CLOCK_MONOTONIC, &start);

  if (timeout == 0)
    {
      do
        {
          ret = sem_wait(sem);
        }
      while (ret < 0 && errno == EINTR);
    }
  else
    {
      make_abs_timespec(&abstime, timeout);
      do
        {
          ret = sem_timedwait(sem, &abstime);
        }
      while (ret < 0 && errno == EINTR);
    }

  if (ret < 0)
    {
      return SYS_ARCH_TIMEOUT;
    }

  clock_gettime(CLOCK_MONOTONIC, &end);
  return LWIP_MAX(1u, ts_to_ms(&end) - ts_to_ms(&start));
}

int sys_sem_valid(sys_sem_t *sem)
{
  return sem != NULL;
}

void sys_sem_set_invalid(sys_sem_t *sem)
{
  memset(sem, 0, sizeof(*sem));
}

err_t sys_mutex_new(sys_mutex_t *mutex)
{
  return pthread_mutex_init(mutex, NULL) == 0 ? ERR_OK : ERR_MEM;
}

void sys_mutex_lock(sys_mutex_t *mutex)
{
  pthread_mutex_lock(mutex);
}

void sys_mutex_unlock(sys_mutex_t *mutex)
{
  pthread_mutex_unlock(mutex);
}

void sys_mutex_free(sys_mutex_t *mutex)
{
  pthread_mutex_destroy(mutex);
}

int sys_mutex_valid(sys_mutex_t *mutex)
{
  return mutex != NULL;
}

void sys_mutex_set_invalid(sys_mutex_t *mutex)
{
  memset(mutex, 0, sizeof(*mutex));
}

err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
  struct sys_mbox_s *m;

  if (size <= 0)
    {
      size = SYS_MBOX_SIZE;
    }

  m = calloc(1, sizeof(*m));
  if (m == NULL)
    {
      return ERR_MEM;
    }

  m->msgs = calloc((size_t)size, sizeof(void *));
  if (m->msgs == NULL)
    {
      free(m);
      return ERR_MEM;
    }

  m->size = size;
  pthread_mutex_init(&m->lock, NULL);
  sem_init(&m->slots, 0, (unsigned int)size);
  sem_init(&m->items, 0, 0);
  *mbox = m;
  return ERR_OK;
}

void sys_mbox_free(sys_mbox_t *mbox)
{
  struct sys_mbox_s *m = *mbox;

  if (m == NULL)
    {
      return;
    }

  sem_destroy(&m->items);
  sem_destroy(&m->slots);
  pthread_mutex_destroy(&m->lock);
  free(m->msgs);
  free(m);
  *mbox = NULL;
}

void sys_mbox_post(sys_mbox_t *mbox, void *msg)
{
  struct sys_mbox_s *m = *mbox;

  while (sem_wait(&m->slots) < 0 && errno == EINTR)
    {
    }

  pthread_mutex_lock(&m->lock);
  m->msgs[m->tail] = msg;
  m->tail = (m->tail + 1) % m->size;
  pthread_mutex_unlock(&m->lock);

  sem_post(&m->items);
}

err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
  struct sys_mbox_s *m = *mbox;

  if (sem_trywait(&m->slots) < 0)
    {
      return ERR_MEM;
    }

  pthread_mutex_lock(&m->lock);
  m->msgs[m->tail] = msg;
  m->tail = (m->tail + 1) % m->size;
  pthread_mutex_unlock(&m->lock);

  sem_post(&m->items);
  return ERR_OK;
}

err_t sys_mbox_trypost_fromisr(sys_mbox_t *mbox, void *msg)
{
  return sys_mbox_trypost(mbox, msg);
}

u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout)
{
  struct sys_mbox_s *m = *mbox;
  struct timespec start;
  struct timespec end;
  struct timespec abstime;
  int ret;

  clock_gettime(CLOCK_MONOTONIC, &start);

  if (timeout == 0)
    {
      do
        {
          ret = sem_wait(&m->items);
        }
      while (ret < 0 && errno == EINTR);
    }
  else
    {
      make_abs_timespec(&abstime, timeout);
      do
        {
          ret = sem_timedwait(&m->items, &abstime);
        }
      while (ret < 0 && errno == EINTR);
    }

  if (ret < 0)
    {
      if (msg != NULL)
        {
          *msg = NULL;
        }

      return SYS_ARCH_TIMEOUT;
    }

  pthread_mutex_lock(&m->lock);
  if (msg != NULL)
    {
      *msg = m->msgs[m->head];
    }

  m->msgs[m->head] = NULL;
  m->head = (m->head + 1) % m->size;
  pthread_mutex_unlock(&m->lock);

  sem_post(&m->slots);

  clock_gettime(CLOCK_MONOTONIC, &end);
  return LWIP_MAX(1u, ts_to_ms(&end) - ts_to_ms(&start));
}

u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
  struct sys_mbox_s *m = *mbox;

  if (sem_trywait(&m->items) < 0)
    {
      if (msg != NULL)
        {
          *msg = NULL;
        }

      return SYS_MBOX_EMPTY;
    }

  pthread_mutex_lock(&m->lock);
  if (msg != NULL)
    {
      *msg = m->msgs[m->head];
    }

  m->msgs[m->head] = NULL;
  m->head = (m->head + 1) % m->size;
  pthread_mutex_unlock(&m->lock);

  sem_post(&m->slots);
  return 0;
}

int sys_mbox_valid(sys_mbox_t *mbox)
{
  return mbox != NULL && *mbox != NULL;
}

void sys_mbox_set_invalid(sys_mbox_t *mbox)
{
  *mbox = NULL;
}

sys_thread_t sys_thread_new(const char *name, lwip_thread_fn thread,
                            void *arg, int stacksize, int prio)
{
  pthread_t tid;
  pthread_attr_t attr;

  (void)name;
  (void)prio;

  pthread_attr_init(&attr);
  pthread_attr_setstacksize(&attr, (size_t)stacksize);

  if (pthread_create(&tid, &attr, (void *(*)(void *))thread, arg) != 0)
    {
      pthread_attr_destroy(&attr);
      return (sys_thread_t)0;
    }

  pthread_attr_destroy(&attr);
  return tid;
}

sys_prot_t sys_arch_protect(void)
{
  return enter_critical_section();
}

void sys_arch_unprotect(sys_prot_t pval)
{
  leave_critical_section(pval);
}

u32_t sys_now(void)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts_to_ms(&ts);
}

u32_t sys_jiffies(void)
{
  return sys_now();
}

/* Do NOT provide mem_malloc()/mem_free() here.
 *
 * These are lwIP core allocator symbols implemented by src/core/mem.c.
 * If we want libc-backed allocation, enable MEM_LIBC_MALLOC in lwipopts.h
 * and let lwIP's own mem.c dispatch to malloc/calloc/free.
 */

#endif /* CONFIG_NET_LWIP */
