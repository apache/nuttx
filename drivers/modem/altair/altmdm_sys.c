/****************************************************************************
 * drivers/modem/altmdm/altmdm_sys.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

  ret = nxsem_init(&handle->sem, 0, 1);

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

  ret = nxsem_wait_uninterruptible(&handle->sem);
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
  struct timespec abs_time;
  struct timespec curr_time;
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

  if (timeout_ms != ALTMDM_SYS_FLAG_TMOFEVR)
    {
      /* Get current time. */

      ret = clock_gettime(CLOCK_REALTIME, &curr_time);
      if (ret != OK)
        {
          return ret;
        }

      abs_time.tv_sec = timeout_ms / 1000;
      abs_time.tv_nsec = (timeout_ms - (abs_time.tv_sec * 1000)) *
                          1000 * 1000;

      abs_time.tv_sec += curr_time.tv_sec;
      abs_time.tv_nsec += curr_time.tv_nsec;

      /* Check more than 1 sec. */

      if (abs_time.tv_nsec >= (1000 * 1000 * 1000))
        {
          abs_time.tv_sec += 1;
          abs_time.tv_nsec -= (1000 * 1000 * 1000);
        }
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

          ret = nxsem_timedwait_uninterruptible(&handle->sem, &abs_time);
          if (ret < 0)
            {
              m_err("nxsem_timedwait_uninterruptible() failed:%d\n", ret);
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

  ret = sigprocmask(SIG_UNBLOCK, &mask, NULL);
  if (ret != OK)
    {
      m_err("sigprocmask() failed:%d\n", ret);
      return NULL;
    }

  sa.sa_sigaction = handler;
  sa.sa_flags = SA_SIGINFO;
  sigfillset(&sa.sa_mask);
  nxsig_delset(&sa.sa_mask, MY_TIMER_SIGNAL);

  ret = sigaction(MY_TIMER_SIGNAL, &sa, NULL);
  if (ret != OK)
    {
      m_err("sigaction() failed:%d\n", ret);
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
  sigprocmask(SIG_SETMASK, &mask, NULL);
}

#endif
