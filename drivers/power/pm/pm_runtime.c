/****************************************************************************
 * drivers/power/pm/pm_runtime.c
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

#include <debug.h>
#include <assert.h>
#include <errno.h>
#include <sched.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/power/pm_runtime.h>
#include <sched/sched.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rpm_changestate(FAR struct pm_runtime_s *rpm, rpm_state_e state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static irqstate_t pm_runtime_lock(FAR rmutex_t *lock)
{
  if (!up_interrupt_context() && !sched_idletask())
    {
      nxrmutex_lock(lock);
    }

  return enter_critical_section();
}

static void pm_runtime_unlock(FAR rmutex_t *lock, irqstate_t flags)
{
  leave_critical_section(flags);

  if (!up_interrupt_context() && !sched_idletask())
    {
      nxrmutex_unlock(lock);
    }
}

static int rpm_suspend(FAR struct pm_runtime_s *rpm)
{
  int ret = 0;

  if (rpm->ops && rpm->ops->runtime_suspend)
    {
      ret = rpm->ops->runtime_suspend(rpm);
    }

  return ret;
}

static int rpm_resume(FAR struct pm_runtime_s *rpm)
{
  int ret = 0;

  if (rpm->ops && rpm->ops->runtime_resume)
    {
      ret = rpm->ops->runtime_resume(rpm);
    }

  return ret;
}

static void rpm_autosuspend_cb(FAR void *arg)
{
  FAR struct pm_runtime_s *rpm = arg;
  irqstate_t flags;

  flags = pm_runtime_lock(&rpm->lock);

  if (rpm->state != RPM_SUSPENDING || !work_available(&rpm->suspend_work))
    {
      goto out;
    }

  if (rpm_changestate(rpm, RPM_SUSPENDED) < 0)
    {
      pwrerr("%p runtime suspend failed\n", rpm);
      rpm->state = RPM_ACTIVE;
    }
  else
    {
      rpm->state = RPM_SUSPENDED;
    }

out:
  pm_runtime_unlock(&rpm->lock, flags);
}

static int rpm_changestate(FAR struct pm_runtime_s *rpm, rpm_state_e state)
{
  int ret = 0;

  switch (rpm->state)
  {
  case RPM_ACTIVE:
    if (state == RPM_SUSPENDED)
      {
        ret = rpm_suspend(rpm);
      }
    else if (state == RPM_SUSPENDING)
      {
         ret = work_queue(HPWORK, &rpm->suspend_work,
                          rpm_autosuspend_cb, rpm,
                          MSEC2TICK(rpm->suspend_delay));
      }
    break;

  case RPM_SUSPENDED:
    if (state == RPM_ACTIVE)
      {
        ret = rpm_resume(rpm);
      }
    break;

  case RPM_SUSPENDING:
    if (state == RPM_ACTIVE)
      {
        work_cancel(HPWORK, &rpm->suspend_work);
      }
    else if (state == RPM_SUSPENDED)
      {
        ret = rpm_suspend(rpm);
      }
    break;

  default:
    break;
  }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pm_runtime_init
 *
 * Description:
 *   init struct pm_runtime_s members
 *
 * Input Parameters:
 *   rpm - the struct pm_runtime_s addr
 *   state - the init state of the rpm
 *   ops - the struct pm_runtime_ops_s  addr
 *
 * Returned Value:
 *    None
 ****************************************************************************/

void pm_runtime_init(FAR struct pm_runtime_s *rpm, rpm_state_e state,
                     FAR struct pm_runtime_ops_s *ops)
{
  DEBUGASSERT(rpm != NULL && ops != NULL);
  DEBUGASSERT(state == RPM_ACTIVE || state == RPM_SUSPENDED);
  nxrmutex_init(&rpm->lock);
  rpm->use_count = 0;
  rpm->suspend_delay = 0;
  rpm->state = state;
  rpm->ops = ops;
}

/****************************************************************************
 * Name: pm_runtime_get
 *
 * Description:
 *   add the rpm use_count, if the first time resume
 *
 * Input Parameters:
 *   rpm - the struct pm_runtime_s addr
 *
 * Returned Value:
 *   Zero on success or a negated errno value on failure
 ****************************************************************************/

int pm_runtime_get(FAR struct pm_runtime_s *rpm)
{
  irqstate_t flags;
  int ret = 0;

  DEBUGASSERT(rpm != NULL);
  flags = pm_runtime_lock(&rpm->lock);

  if (rpm->use_count++ > 0)
    {
      DEBUGASSERT(rpm->state == RPM_ACTIVE);
      goto out;
    }

  ret = rpm_changestate(rpm, RPM_ACTIVE);
  if (ret < 0)
    {
      rpm->use_count--;
      goto out;
    }

  rpm->state = RPM_ACTIVE;
out:
  pm_runtime_unlock(&rpm->lock, flags);
  return ret;
}

/****************************************************************************
 * Name: pm_runtime_put
 *
 * Description:
 *   drop the rpm use_count, if refcnt is zero suspend
 *
 * Input Parameters:
 *   rpm - the struct pm_runtime_s addr
 *
 * Returned Value:
 *   Zero on success or a negated errno value on failure
 ****************************************************************************/

int pm_runtime_put(FAR struct pm_runtime_s *rpm)
{
  irqstate_t flags;
  int ret = 0;

  DEBUGASSERT(rpm != NULL);
  flags = pm_runtime_lock(&rpm->lock);
  if (rpm->use_count == 0)
    {
      ret = -EPERM;
      goto out;
    }

  DEBUGASSERT(rpm->state == RPM_ACTIVE);
  if (--rpm->use_count > 0)
    {
      goto out;
    }

  ret = rpm_changestate(rpm, RPM_SUSPENDED);
  if (ret < 0)
    {
      rpm->use_count++;
      goto out;
    }

  rpm->state = RPM_SUSPENDED;
out:
  pm_runtime_unlock(&rpm->lock, flags);
  return ret;
}

/****************************************************************************
 * Name: pm_runtime_put_autosuspend
 *
 * Description:
 *   drop the rpm use_count, if refcnt is zero suspend or suspend
 *   (depends suspend_delay) after a delay duration
 *
 * Input Parameters:
 *   rpm - the struct pm_runtime_s addr
 *
 * Returned Value:
 *   Zero on success or a negated errno value on failure
 ****************************************************************************/

int pm_runtime_put_autosuspend(FAR struct pm_runtime_s *rpm)
{
  irqstate_t flags;
  int ret = 0;

  DEBUGASSERT(rpm != NULL);
  flags = pm_runtime_lock(&rpm->lock);
  if (rpm->use_count == 0)
    {
      ret = -EPERM;
      goto out;
    }

  DEBUGASSERT(rpm->state == RPM_ACTIVE);
  if (--rpm->use_count > 0)
    {
      goto out;
    }

  ret = rpm_changestate(rpm, RPM_SUSPENDING);
  if (ret < 0)
    {
      rpm->use_count++;
      goto out;
    }

  rpm->state = RPM_SUSPENDING;
out:
  pm_runtime_unlock(&rpm->lock, flags);
  return ret;
}

/****************************************************************************
 * Name: pm_runtime_set_autosuspend_delay
 *
 * Description:
 *   set rpm autosuspend_delay
 *
 * Input Parameters:
 *   rpm - the struct pm_runtime_s addr
 *   delay - the delay in milliseconds
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void pm_runtime_set_autosuspend_delay(FAR struct pm_runtime_s *rpm,
                                      unsigned int delay)
{
  irqstate_t flags;

  DEBUGASSERT(rpm != NULL);
  flags = pm_runtime_lock(&rpm->lock);
  rpm->suspend_delay = delay;
  pm_runtime_unlock(&rpm->lock, flags);
}
