/****************************************************************************
 * drivers/power/pm
 *
 *   Copyright (C) 2011-2012, 2016, 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Author: Matias Nitsche <mnitsche@dc.uba.ar>
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

  /* A pointer to the PM governor instance */

  FAR const struct pm_governor_s *governor;
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

/*****************************************************************************
 * Public Function Prototypes
 *****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_PM */
#endif /* #define __DRIVERS_POWER_PM_H */
