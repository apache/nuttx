/****************************************************************************
 * drivers/power/pm.h
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

#ifndef __DRIVERS_POWER_PM_H
#define __DRIVERS_POWER_PM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <queue.h>

#include <nuttx/semaphore.h>
#include <nuttx/clock.h>
#include <nuttx/power/pm.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>

#ifdef CONFIG_PM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: pm_lock
 *
 * Description:
 *   Lock the power management registry.  NOTE: This function may return
 *   an error if a signal is received while what (errno == EINTR).
 *
 ****************************************************************************/

#define pm_lock() nxsem_wait(&g_pmglobals.regsem);

/****************************************************************************
 * Name: pm_unlock
 *
 * Description:
 *   Unlock the power management registry.
 *
 ****************************************************************************/

#define pm_unlock() nxsem_post(&g_pmglobals.regsem);

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

  uint16_t stay[PM_COUNT];

  /* Auto update or not */

  bool auto_update;

#if defined(CONFIG_SCHED_WORKQUEUE)
  /* The worker of update callback */

  struct work_s update_work;
#endif

  /* A pointer to the PM governor instance */

  FAR const struct pm_governor_s *governor;
};

/* This structure encapsulates all of the global data used by the PM system */

struct pm_global_s
{
  /* This semaphore manages mutually exclusive access to the power management
   * registry.  It must be initialized to the value 1.
   */

  sem_t regsem;

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
 * Name: pm_auto_updatestate
 *
 * Description:
 *   This function update the domain state and notify the power system.
 *
 * Input Parameters:
 *   domain - The PM domain to check
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void pm_auto_updatestate(int domain);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_PM */
#endif /* #define __DRIVERS_POWER_PM_H */
