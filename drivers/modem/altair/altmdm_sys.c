/****************************************************************************
 * drivers/modem/altair/altmdm_sys.c
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

#include <errno.h>
#include <debug.h>
#include <nuttx/irq.h>
#include <signal.h>

#include <nuttx/signal.h>

#include "altmdm_dev.h"
#include "altmdm_sys.h"

#if defined(CONFIG_MODEM_ALTMDM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MY_TIMER_SIGNAL SIGUSR1

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altmdm_sys_initlock
 *
 * Description:
 *   Initialize lock resource.
 *
 ****************************************************************************/

int altmdm_sys_initlock(FAR struct altmdm_sys_lock_s *handle)
{
  int ret;

  /* Check argument. */

  if (handle == NULL)
    {
      return ERROR;
    }

  ret = nxmutex_init(&handle->lock);

#ifdef CONFIG_MODEM_ALTMDM_DEBUG
  if (ret < 0)
    {
      m_err("nxsem_init() failed:%d\n", ret);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: altmdm_sys_deletelock
 *
 * Description:
 *   Delete lock resource
 *
 ****************************************************************************/

int altmdm_sys_deletelock(FAR struct altmdm_sys_lock_s *handle)
{
  int ret;

  /* Check argument. */

  if (handle == NULL)
    {
      return ERROR;
    }

  ret = nxmutex_destroy(&handle->lock);

#ifdef CONFIG_MODEM_ALTMDM_DEBUG
  if (ret < 0)
    {
      m_err("nxsem_destroy() failed:%d\n", ret);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: altmdm_sys_lock
 *
 * Description:
 *   Acquire lock.
 *
 ****************************************************************************/

int altmdm_sys_lock(FAR struct altmdm_sys_lock_s *handle)
{
  int ret;

  /* Check argument. */

  if (handle == NULL)
    {
      return ERROR;
    }

  ret = nxmutex_lock(&handle->lock);
  if (ret < 0)
    {
      m_err("nxsem_wait_uninterruptible() failed:%d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: altmdm_sys_unlock
 *
 * Description:
 *   Release lock.
 *
 ****************************************************************************/

int altmdm_sys_unlock(FAR struct altmdm_sys_lock_s *handle)
{
  int ret;

  /* Check argument. */

  if (handle == NULL)
    {
      return ERROR;
    }

  ret = nxmutex_unlock(&handle->lock);

#ifdef CONFIG_MODEM_ALTMDM_DEBUG
  if (ret < 0)
    {
      m_err("nxsem_post() failed:%d\n", ret);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: altmdm_sys_initcsem
 *
 * Description:
 *   Initialize counting semaphore.
 *
 ****************************************************************************/

int altmdm_sys_initcsem(FAR struct altmdm_sys_csem_s *handle)
{
  int ret;

  /* Check argument. */

  if (handle == NULL)
    {
      return ERROR;
    }

  ret = nxsem_init(&handle->sem, 0, 0);

#ifdef CONFIG_MODEM_ALTMDM_DEBUG
  if (ret < 0)
    {
      m_err("nxsem_init() failed:%d\n", ret);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: altmdm_sys_deletecsem
 *
 * Description:
 *   Delete counting semaphore.
 *
 ****************************************************************************/

int altmdm_sys_deletecsem(FAR struct altmdm_sys_csem_s *handle)
{
  int ret;

  /* Check argument. */

  if (handle == NULL)
    {
      return ERROR;
    }

  ret = nxsem_destroy(&handle->sem);

#ifdef CONFIG_MODEM_ALTMDM_DEBUG
  if (ret < 0)
    {
      m_err("nxsem_destroy() failed:%d\n", ret);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: altmdm_sys_waitcsem
 *
 * Description:
 *   Wait counting semaphore.
 *
 ****************************************************************************/

int altmdm_sys_waitcsem(FAR struct altmdm_sys_csem_s *handle)
{
  int ret;

  /* Check argument. */

  if (handle == NULL)
    {
      return ERROR;
    }

  ret = nxsem_wait_uninterruptible(&handle->sem);
  if (ret < 0)
    {
      m_err("nxsem_wait_uninterruptible() failed:%d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: altmdm_sys_postcsem
 *
 * Description:
 *   Post counting semaphore.
 *
 ****************************************************************************/

int altmdm_sys_postcsem(FAR struct altmdm_sys_csem_s *handle)
{
  int ret;

  /* Check argument. */

  if (handle == NULL)
    {
      return ERROR;
    }

  ret = nxsem_post(&handle->sem);

#ifdef CONFIG_MODEM_ALTMDM_DEBUG
  if (ret < 0)
    {
      m_err("nxsem_post() failed:%d\n", ret);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: altmdm_sys_getcsemvalue
 *
 * Description:
 *   Get value of counting semaphore.
 *
 ****************************************************************************/

int altmdm_sys_getcsemvalue(FAR struct altmdm_sys_csem_s *handle,
                            FAR int *value)
{
  int ret;

  /* Check argument. */

  if ((handle == NULL) || (value == NULL))
    {
      return ERROR;
    }

  ret = nxsem_get_value(&handle->sem, value);

#ifdef CONFIG_MODEM_ALTMDM_DEBUG
  if (ret < 0)
    {
      m_err("nxsem_get_value() failed:%d\n", ret);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: altmdm_sys_initflag
 *
 * Description:
 *   Initialize event flag resource.
 *
 ****************************************************************************/

int altmdm_sys_initflag(FAR struct altmdm_sys_flag_s *handle)
{
  int ret;

  /* Check argument. */

  if (handle == NULL)
    {
      return ERROR;
    }

  handle->flag = 0;
  ret = nxsem_init(&handle->sem, 0, 0);

#ifdef CONFIG_MODEM_ALTMDM_DEBUG
  if (ret < 0)
    {
      m_err("nxsem_init() failed:%d\n", ret);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: altmdm_sys_deleteflag
 *
 * Description:
 *   Delete event flag resource.
 *
 ****************************************************************************/

int altmdm_sys_deleteflag(FAR struct altmdm_sys_flag_s *handle)
{
  int ret;

  /* Check argument. */

  if (handle == NULL)
    {
      return ERROR;
    }

  ret = nxsem_destroy(&handle->sem);

#ifdef CONFIG_MODEM_ALTMDM_DEBUG
  if (ret < 0)
    {
      m_err("nxsem_destroy() failed:%d\n", ret);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: altmdm_sys_waitflag
 *
 * Description:
 *   Wait event flag.
 *
 ****************************************************************************/

int altmdm_sys_waitflag(FAR struct altmdm_sys_flag_s *handle,
                        uint32_t wait_pattern, uint32_t wait_mode,
                        FAR uint32_t * pattern, uint32_t timeout_ms)
{
  int ret = OK;
  irqstate_t flags;
  uint32_t ptn;

  /* Check argument. */

  if ((handle == NULL) || (pattern == NULL))
    {
      m_err("invalid parameter\n");

      return ERROR;
    }

  switch (wait_mode)
    {
    case ALTMDM_SYS_FLAG_WMODEOR:
    case ALTMDM_SYS_FLAG_WMODEAND:
      break;

    default:
      m_err("invalid wait mode:%d\n", wait_mode);
      return ERROR;
    }

  *pattern = 0;

  while (1)
    {
      if (wait_mode == ALTMDM_SYS_FLAG_WMODEOR)
        {
          flags = enter_critical_section();

          ptn = (handle->flag & wait_pattern);
          if (ptn != 0)
            {
              /* Wait pattern matched. */

              *pattern = ptn;
              handle->flag = (handle->flag & ~ptn);

              /* Clear the semaphore posted by altmdm_sys_setflag. */

              while (1)
                {
                  if (nxsem_trywait(&handle->sem) < 0)
                    {
                      break;
                    }
                }

              leave_critical_section(flags);

              ret = OK;
              break;
            }

          leave_critical_section(flags);
        }
      else
        {
          flags = enter_critical_section();

          ptn = (handle->flag & wait_pattern);
          if (ptn == wait_pattern)
            {
              /* Wait pattern matched. */

              *pattern = ptn;
              handle->flag = (handle->flag & ~ptn);

              /* Clear the semaphore posted by altmdm_sys_setflag. */

              while (1)
                {
                  if (nxsem_trywait(&handle->sem) < 0)
                    {
                      break;
                    }
                }

              leave_critical_section(flags);

              ret = OK;
              break;
            }

          leave_critical_section(flags);
        }

      if (timeout_ms != ALTMDM_SYS_FLAG_TMOFEVR)
        {
          /* Wait for the semaphore to be posted until timeout occurs. */

          ret = nxsem_tickwait_uninterruptible(&handle->sem,
                                               MSEC2TICK(timeout_ms));
          if (ret < 0)
            {
              m_err("nxsem_tickwait_uninterruptible() failed:%d\n", ret);
              break;
            }
        }
      else
        {
          /* Wait for the semaphore to be posted forever. */

          ret = nxsem_wait_uninterruptible(&handle->sem);
          if (ret < 0)
            {
              m_err("nxsem_wait_uninterruptible() failed:%d\n", ret);
              break;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: altmdm_sys_setflag
 *
 * Description:
 *   Set event flag.
 *
 ****************************************************************************/

int altmdm_sys_setflag(FAR struct altmdm_sys_flag_s *handle,
                       uint32_t pattern)
{
  int ret;
  irqstate_t flags;

  /* Check argument. */

  if (handle == NULL)
    {
      return ERROR;
    }

  flags = enter_critical_section();

  handle->flag = (handle->flag | pattern);

  leave_critical_section(flags);

  ret = nxsem_post(&handle->sem);

  return ret;
}

/****************************************************************************
 * Name: altmdm_sys_clearflag
 *
 * Description:
 *   Clear event flag.
 *
 ****************************************************************************/

int altmdm_sys_clearflag(FAR struct altmdm_sys_flag_s *handle,
                         uint32_t pattern)
{
  irqstate_t flags;

  /* Check argument. */

  if (handle == NULL)
    {
      return ERROR;
    }

  flags = enter_critical_section();

  handle->flag = (handle->flag & ~pattern);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: altmdm_sys_referflag
 *
 * Description:
 *   Refer event flag.
 *
 ****************************************************************************/

int altmdm_sys_referflag(FAR struct altmdm_sys_flag_s *handle,
                         FAR struct altmdm_sys_flagstate_s *status)
{
  irqstate_t flags;

  /* Check argument. */

  if ((handle == NULL) || (status == NULL))
    {
      return ERROR;
    }

  flags = enter_critical_section();

  status->flag_pattern = handle->flag;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: altmdm_sys_starttimer
 *
 * Description:
 *   Start timer.
 *
 ****************************************************************************/

timer_t altmdm_sys_starttimer(int first_ms, int interval_ms,
                              FAR void *handler, int int_param,
                              FAR void *ptr_param)
{
  int ret;
  sigset_t mask;
  struct sigaction sa;
  struct sigevent sev;
  struct itimerspec timer;
  timer_t timerid;

  /* Check argument. */

  if (handler == NULL)
    {
      return NULL;
    }

  sigemptyset(&mask);
  nxsig_addset(&mask, MY_TIMER_SIGNAL);

  ret = nxsig_procmask(SIG_UNBLOCK, &mask, NULL);
  if (ret != OK)
    {
      m_err("nxsig_procmask() failed:%d\n", ret);
      return NULL;
    }

  sa.sa_sigaction = handler;
  sa.sa_flags = SA_SIGINFO;
  sigfillset(&sa.sa_mask);
  nxsig_delset(&sa.sa_mask, MY_TIMER_SIGNAL);

  ret = nxsig_action(MY_TIMER_SIGNAL, &sa, NULL, false);
  if (ret != OK)
    {
      m_err("nxsig_action() failed:%d\n", ret);
      return NULL;
    }

  sev.sigev_notify = SIGEV_SIGNAL;
  sev.sigev_signo = MY_TIMER_SIGNAL;
  sev.sigev_value.sival_int = int_param;
  sev.sigev_value.sival_ptr = ptr_param;

  ret = timer_create(CLOCK_REALTIME, &sev, &timerid);
  if (ret != OK)
    {
      m_err("timer_create() failed:%d\n", ret);
      return NULL;
    }

  timer.it_value.tv_sec = first_ms / 1000;
  timer.it_value.tv_nsec = (first_ms % 1000) * 1000 * 1000;
  timer.it_interval.tv_sec = interval_ms / 1000;
  timer.it_interval.tv_nsec = (interval_ms % 1000) * 1000 * 1000;

  ret = timer_settime(timerid, 0, &timer, NULL);
  if (ret != OK)
    {
      m_err("timer_settime() failed:%d\n", ret);
      return NULL;
    }

  return timerid;
}

/****************************************************************************
 * Name: altmdm_sys_restarttimer
 *
 * Description:
 *   Restart timer.
 *
 ****************************************************************************/

int altmdm_sys_restarttimer(timer_t timerid, int first_ms, int interval_ms)
{
  int ret;
  struct itimerspec timer;

  timer.it_value.tv_sec = first_ms / 1000;
  timer.it_value.tv_nsec = (first_ms % 1000) * 1000 * 1000;
  timer.it_interval.tv_sec = interval_ms / 1000;
  timer.it_interval.tv_nsec = (interval_ms % 1000) * 1000 * 1000;

  ret = timer_settime(timerid, 0, &timer, NULL);
  if (ret != OK)
    {
      m_err("timer_settime() failed:%d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: altmdm_sys_stoptimer
 *
 * Description:
 *   Stop timer.
 *
 ****************************************************************************/

void altmdm_sys_stoptimer(timer_t timerid)
{
  sigset_t mask;

  timer_delete(timerid);

  sigfillset(&mask);
  nxsig_procmask(SIG_SETMASK, &mask, NULL);
}

#endif
