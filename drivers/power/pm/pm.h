/****************************************************************************
 * drivers/power/pm/pm.h
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

#ifndef __DRIVERS_POWER_PM_PM_H
#define __DRIVERS_POWER_PM_PM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/queue.h>
#include <nuttx/mutex.h>
#include <nuttx/clock.h>
#include <nuttx/power/pm.h>
#include <nuttx/wqueue.h>

#ifdef CONFIG_PM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This describes the activity and state for one domain */

struct pm_domain_s
{
  /* The current state for this PM domain (as determined by an
   * explicit call to pm_changestate())
   */

  uint8_t state;

  /* The power state lock count */

  struct dq_queue_s wakelock[PM_COUNT];

#ifdef CONFIG_PM_PROCFS
  struct dq_queue_s wakelockall;
  struct timespec start;
  struct timespec wake[PM_COUNT];
  struct timespec sleep[PM_COUNT];
#endif

  /* Auto update or not */

  bool auto_update;

#if defined(CONFIG_SCHED_WORKQUEUE)
  /* The worker of update callback */

  struct work_s update_work;
#endif

  /* A pointer to the PM governor instance */

  FAR const struct pm_governor_s *governor;

  /* Recursive lock for race condition */

  rmutex_t lock;
};

/* This structure encapsulates all of the global data used by the PM system */

struct pm_global_s
{
  /* This rmutex manages mutually exclusive access to the power management
   * registry.  It must be initialized to the value 1.
   */

  rmutex_t reglock;

  /* registry is a doubly-linked list of registered power management
   * callback structures.  To ensure mutually exclusive access, this list
   * must be locked by calling pm_lock() before it is accessed.
   */

  dq_queue_t registry;

  /* The state information for each PM domain */

  struct pm_domain_s domain[CONFIG_PM_NDOMAINS];
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/* All PM global data: */

EXTERN struct pm_global_s g_pmglobals;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pm_lock
 *
 * Description:
 *   Lock the power management operation.
 *
 * Input Parameters:
 *   lock - The lock subjuct
 *
 * Returned Value:
 *   Return current state
 *
 ****************************************************************************/

irqstate_t pm_lock(FAR rmutex_t *lock);

/****************************************************************************
 * Name: pm_unlock
 *
 * Description:
 *   Unlock the power management operation.
 *
 * Input Parameters:
 *   lock - The lock subjuct
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pm_unlock(FAR rmutex_t *lock, irqstate_t flags);

/****************************************************************************
 * Name: pm_domain_lock
 *
 * Description:
 *   Lock the power management operation.
 *
 * Input Parameters:
 *   domain - The PM domain to lock
 *
 * Returned Value:
 *   Return current state
 *
 ****************************************************************************/

irqstate_t pm_domain_lock(int domain);

/****************************************************************************
 * Name: pm_domain_unlock
 *
 * Description:
 *   Unlock the power management operation.
 *
 * Input Parameters:
 *   domain - The PM domain to unlock
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pm_domain_unlock(int domain, irqstate_t flags);

/****************************************************************************
 * Name: pm_wakelock_global_init
 *
 * Description:
 *   This function is called to setup global wakelock when system init
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pm_wakelock_global_init(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_PM */
#endif /* #define __DRIVERS_POWER_PM_PM_H */
