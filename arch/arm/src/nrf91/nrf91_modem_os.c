/****************************************************************************
 * arch/arm/src/nrf91/nrf91_modem_os.c
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

#include <arch/irq.h>

#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>

#include <nuttx/mm/mm.h>

#include <debug.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <semaphore.h>

#include "nrf_modem.h"
#include "nrf_modem_os.h"

#include "nrf91_modem.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NRF_ESHUTDOWN 110

#define NRF91_MODEM_WAITING_PREALLOC (32)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf91_modem_os_waiting_s
{
  sem_t           sem;
  uint32_t        context;
  bool            waiting;
  struct timespec ts_start;
};

struct nrf91_modem_os_s
{
  sem_t   sem[NRF_MODEM_OS_NUM_SEM_REQUIRED];
  uint8_t sem_cntr;
  mutex_t waiting_lock;
  struct nrf91_modem_os_waiting_s waiting[NRF91_MODEM_WAITING_PREALLOC];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct nrf91_modem_os_s g_nrf91_modem_os;
static struct mm_heap_s *g_shmtxheap = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf_waiting_get
 ****************************************************************************/

struct nrf91_modem_os_waiting_s *nrf_waiting_get(void)
{
  struct nrf91_modem_os_waiting_s *ret = NULL;
  int                              i   = 0;

  nxmutex_lock(&g_nrf91_modem_os.waiting_lock);

  for (i = 0; i < NRF91_MODEM_WAITING_PREALLOC; i++)
    {
      if (g_nrf91_modem_os.waiting[i].waiting == false)
        {
          ret = &g_nrf91_modem_os.waiting[i];
          nxsem_init(&ret->sem, 0, 0);
          break;
        }
    }

  nxmutex_unlock(&g_nrf91_modem_os.waiting_lock);

  return ret;
}

/****************************************************************************
 * Name: nrf_waiting_free
 ****************************************************************************/

void nrf_waiting_free(struct nrf91_modem_os_waiting_s *w)
{
  nxmutex_lock(&g_nrf91_modem_os.waiting_lock);

  nxsem_destroy(&w->sem);
  w->context = 0;
  w->waiting = false;

  nxmutex_unlock(&g_nrf91_modem_os.waiting_lock);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf_modem_os_init
 *
 * Description:
 *   Initialize the glue layer.
 *
 ****************************************************************************/

void nrf_modem_os_init(void)
{
  memset(&g_nrf91_modem_os, 0, sizeof(struct nrf91_modem_os_s));

  /* Initialize shared memory TX heap */

  g_shmtxheap = mm_initialize("shmtx",
                              (void *) NRF91_SHMEM_TX_BASE,
                              NRF91_SHMEM_TX_SIZE);

  nxmutex_init(&g_nrf91_modem_os.waiting_lock);
}

/****************************************************************************
 * Name: nrf_modem_os_shutdown
 *
 * Description:
 *   Deinitialize the glue layer.
 *
 *   When shutdown is called, all pending calls to @c nrf_modem_os_timedwait
 *   shall exit and return -NRF_ESHUTDOWN.
 *
 ****************************************************************************/

void nrf_modem_os_shutdown(void)
{
  struct nrf91_modem_os_waiting_s *waiting = NULL;
  int                              i       = 0;

  /* TODO: send AT+CFUN=0 */

  /* Wake up all waiting semaphores */

  for (i = 0; i < NRF91_MODEM_WAITING_PREALLOC; i++)
    {
      waiting = &g_nrf91_modem_os.waiting[i];
      if (waiting->waiting == true)
        {
          nxsem_post(&waiting->sem);
        }
    }

  /* Uninitialize shared memory TX heap */

  mm_uninitialize(g_shmtxheap);
}

/****************************************************************************
 * Name: nrf_modem_os_shm_tx_alloc
 *
 * Description:
 *   Allocate a buffer on the TX area of shared memory.
 *
 ****************************************************************************/

void *nrf_modem_os_shm_tx_alloc(size_t bytes)
{
  return mm_malloc(g_shmtxheap, bytes);
}

/****************************************************************************
 * Name: nrf_modem_os_shm_tx_free
 *
 * Description:
 *   Free a shared memory buffer in the TX area.
 *
 ****************************************************************************/

void nrf_modem_os_shm_tx_free(void *mem)
{
  mm_free(g_shmtxheap, mem);
}

/****************************************************************************
 * Name: nrf_modem_os_alloc
 *
 * Description:
 *   Allocate a buffer on the library heap.
 *
 ****************************************************************************/

void *nrf_modem_os_alloc(size_t bytes)
{
  return malloc(bytes);
}

/****************************************************************************
 * Name: nrf_modem_os_free
 *
 * Description:
 *   Free a memory buffer in the library heap.
 *
 ****************************************************************************/

void nrf_modem_os_free(void *mem)
{
  free(mem);
}

/****************************************************************************
 * Name: nrf_modem_os_busywait
 *
 * Description:
 *   Busy wait.
 *
 ****************************************************************************/

void nrf_modem_os_busywait(int32_t usec)
{
  up_udelay(usec);
}

/****************************************************************************
 * Name: nrf_modem_os_timedwait
 *
 * Description:
 *   Put a thread to sleep for a specific time or until an event occurs.
 *
 ****************************************************************************/

int32_t nrf_modem_os_timedwait(uint32_t context, int32_t *timeout)
{
  struct nrf91_modem_os_waiting_s *waiting   = NULL;
  struct timespec                  ts_now;
  struct timespec                  abstime;
  int32_t                          remaining = 0;
  int32_t                          diff      = 0;
  int                              ret       = -EAGAIN;

  /* Modem is not initialized or was shut down */

  if (!nrf_modem_is_initialized())
    {
      ret = -NRF_ESHUTDOWN;
      goto errout;
    }

  if (*timeout == 0)
    {
      ret = -EAGAIN;
      goto errout;
    }

  /* Get free waiting slot */

  waiting = nrf_waiting_get();
  if (waiting == NULL)
    {
      ret = -ENOMEM;
      goto errout;
    }

  waiting->context = context;
  waiting->waiting = true;
  clock_systime_timespec(&waiting->ts_start);

  if (*timeout < 0)
    {
      /* Wait for event */

      nxsem_wait(&waiting->sem);
      ret = OK;
    }
  else
    {
      /* Wait for event or timeout */

      abstime.tv_sec  = *timeout / 1000;
      abstime.tv_nsec = (*timeout % 1000) * 1000000;

      nxsem_timedwait(&waiting->sem, &abstime);
    }

  /* Free a waiting slot */

  nrf_waiting_free(waiting);

  /* Modem is not initialized or was shut down */

  if (!nrf_modem_is_initialized())
    {
      ret = -NRF_ESHUTDOWN;
      goto errout;
    }

  /* Handle timeout */

  if (ret < 0)
    {
      clock_systime_timespec(&ts_now);

      diff = ((ts_now.tv_sec - waiting->ts_start.tv_sec) * 1000 +
              (ts_now.tv_nsec - waiting->ts_start.tv_nsec) / 1000000);

      remaining = *timeout - diff;

      /* Return remaining timeout */

      *timeout = (remaining > 0) ? remaining : 0;

      if (*timeout == 0)
        {
          ret = -EAGAIN;
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf_modem_os_event_notify
 *
 * Description:
 *   Notify the application that an event has occurred.
 *
 ****************************************************************************/

void nrf_modem_os_event_notify(uint32_t context)
{
  struct nrf91_modem_os_waiting_s *waiting = NULL;
  int                              i       = 0;

  for (i = 0; i < NRF91_MODEM_WAITING_PREALLOC; i++)
    {
      waiting = &g_nrf91_modem_os.waiting[i];
      if (waiting->waiting == true)
        {
          if (waiting->context == context || context == 0)
            {
              nxsem_post(&waiting->sem);
            }
        }
    }
}

/****************************************************************************
 * Name: nrf_modem_os_sleep
 *
 * Description:
 *   Put a thread to sleep for a specific amount of time.
 *
 ****************************************************************************/

int nrf_modem_os_sleep(uint32_t timeout)
{
  /* Timeout in ms */

  nxsig_usleep(timeout * 1000);
  return OK;
}

/****************************************************************************
 * Name: nrf_modem_os_errno_set
 *
 * Description:
 *   Set errno.
 *
 ****************************************************************************/

void nrf_modem_os_errno_set(int errno_val)
{
  set_errno(errno_val);
}

/****************************************************************************
 * Name: nrf_modem_os_is_in_isr
 *
 * Description:
 *   Check if executing in interrupt context.
 *
 ****************************************************************************/

bool nrf_modem_os_is_in_isr(void)
{
  return up_interrupt_context();
}

/****************************************************************************
 * Name: nrf_modem_os_sem_init
 *
 * Description:
 *   Initialize a semaphore.
 *
 ****************************************************************************/

int nrf_modem_os_sem_init(void **sem, unsigned int initial_count,
                          unsigned int limit)
{
  sem_t *modemsem = &g_nrf91_modem_os.sem[g_nrf91_modem_os.sem_cntr];
  int    ret      = OK;

  UNUSED(limit);

  DEBUGASSERT(g_nrf91_modem_os.sem_cntr < NRF_MODEM_OS_NUM_SEM_REQUIRED);
  ret = nxsem_init(modemsem, 0, initial_count);
  g_nrf91_modem_os.sem_cntr++;

  *sem = (void *)modemsem;

  return ret;
}

/****************************************************************************
 * Name: nrf_modem_os_sem_give
 *
 * Description:
 *   Give a semaphore.
 *
 ****************************************************************************/

void nrf_modem_os_sem_give(void *sem)
{
  nxsem_post((sem_t *)sem);
}

/****************************************************************************
 * Name: nrf_modem_os_sem_take
 *
 * Description:
 *   Take a semaphore.
 *
 ****************************************************************************/

int nrf_modem_os_sem_take(void *sem, int timeout)
{
  int    ret = OK;
  sem_t *s   = sem;

  if (timeout == -1)
    {
      ret = nxsem_wait(s);
    }
  else
    {
      struct timespec abstime;

      abstime.tv_sec  = timeout / 1000;
      abstime.tv_nsec = (timeout % 1000) * 1000000;

      ret = nxsem_timedwait(s, &abstime);
    }

  return ret;
}

/****************************************************************************
 * Name: nrf_modem_os_sem_count_get
 *
 * Description:
 *   Get a semaphore's count.
 *
 ****************************************************************************/

unsigned int nrf_modem_os_sem_count_get(void *sem)
{
  int sval = 0;

  nxsem_get_value((sem_t *)sem, &sval);
  return sval;
}

/****************************************************************************
 * Name: nrf_modem_os_log
 *
 * Description:
 *   Generic logging procedure.
 *
 ****************************************************************************/

void nrf_modem_os_log(int level, const char *fmt, ...)
{
  va_list ap;

  va_start(ap, fmt);
  vsyslog(level, fmt, ap);
  va_end(ap);
}

/****************************************************************************
 * Name: nrf_modem_os_logdump
 *
 * Description:
 *   Logging procedure for dumping hex representation of object.
 *
 ****************************************************************************/

void nrf_modem_os_logdump(int level, const char *str, const void *data,
                          size_t len)
{
  syslog(level, "nrf_modem_os_logdump");
  lib_dumpbuffer(str, data, len);
}
