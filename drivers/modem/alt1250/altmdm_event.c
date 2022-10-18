/****************************************************************************
 * drivers/modem/alt1250/altmdm_event.c
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

#include <time.h>
#include <errno.h>
#include <nuttx/irq.h>
#include <nuttx/semaphore.h>

#include "altmdm_event.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int set_expiretime(int expire_time, FAR struct timespec *set_time)
{
  struct timespec curr_time;

  /* Get current time. */

  if (clock_gettime(CLOCK_REALTIME, &curr_time) != OK)
    {
      return ERROR;
    }

  set_time->tv_sec = expire_time / 1000;
  set_time->tv_nsec =
    (expire_time - (set_time->tv_sec * 1000)) * 1000 * 1000;

  set_time->tv_sec += curr_time.tv_sec;
  set_time->tv_nsec += curr_time.tv_nsec;

  /* Check more than 1 sec. */

  if (set_time->tv_nsec >= (1000 * 1000 * 1000))
    {
      set_time->tv_sec += 1;
      set_time->tv_nsec -= (1000 * 1000 * 1000);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int altmdm_event_init(FAR struct altmdm_event_s *evt)
{
  int ret;

  evt->event = 0;
  ret = nxsem_init(&evt->sem, 0, 0);

  return ret;
}

int altmdm_event_destroy(FAR struct altmdm_event_s *evt)
{
  int ret;

  ret = nxsem_destroy(&evt->sem);

  return ret;
}

uint32_t altmdm_event_wait(FAR struct altmdm_event_s *evt,
  uint32_t event, bool with_clear, int timeout_ms)
{
  int ret;
  struct timespec abs_time;
  irqstate_t flags;
  uint32_t ptn;

  /* Check argument. */

  if (evt == NULL)
    {
      return 0;
    }

  if (timeout_ms > 0)
    {
      if (set_expiretime(timeout_ms, &abs_time) != OK)
        {
          return 0;
        }
    }

  while (1)
    {
      flags = enter_critical_section();

      ptn = (evt->event & event);
      if (ptn != 0)
        {
          /* Clear read event. */

          if (with_clear)
            {
              evt->event = (evt->event & ~ptn);
            }

          /* Clear the semaphore posted by altmdm_sys_setflag. */

          while (1)
            {
              if (nxsem_trywait(&evt->sem) < 0)
                {
                  break;
                }
            }

          leave_critical_section(flags);

          /* Go out while(1) as expected event is happened */

          break;
        }

      leave_critical_section(flags);

      /* Wait for any event is occured related on the semaphore.. */

      if (timeout_ms > 0)
        {
          /* Wait with timeout. */

          ret = nxsem_timedwait_uninterruptible(&evt->sem, &abs_time);
        }
      else
        {
          /* Wait it forever. */

          ret = nxsem_wait_uninterruptible(&evt->sem);
        }

      /* Error check */

      if (ret < 0)
        {
          ptn = 0;
          break;
        }
    }

  /* end of while(1) */

  return ptn;
}

int altmdm_event_set(FAR struct altmdm_event_s *evt, uint32_t event)
{
  int ret;
  irqstate_t flags;

  flags = enter_critical_section();
  evt->event = (evt->event | event);
  leave_critical_section(flags);
  ret = nxsem_post(&evt->sem);

  return ret;
}

int altmdm_event_clear(FAR struct altmdm_event_s *evt, uint32_t event)
{
  irqstate_t flags;

  flags = enter_critical_section();
  evt->event = (evt->event & ~event);
  leave_critical_section(flags);

  return OK;
}

uint32_t altmdm_event_refer(FAR struct altmdm_event_s *evt)
{
  uint32_t event;
  irqstate_t flags;

  flags = enter_critical_section();
  event = evt->event;
  leave_critical_section(flags);

  return event;
}
