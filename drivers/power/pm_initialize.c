/****************************************************************************
 * drivers/power/pm_initialize.c
 *
 *   Copyright (C) 2011-2012, 2017 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/power/pm.h>

#include "pm.h"

#if defined(CONFIG_PM_GOVERNOR_ACTIVITY)
#  include "activity_governor.h"
#elif defined(CONFIG_PM_GOVERNOR_GREEDY)
#  include "greedy_governor.h"
#endif

#ifdef CONFIG_PM

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* All PM global data: */

/* Initialize the registry and the PM global data structures.  The PM
 * global data structure resides in .data which is zeroed at boot time.  So
 * it is only required to initialize non-zero elements of the PM global
 * data structure here.
 */

struct pm_global_s g_pmglobals =
{
  .regsem = SEM_INITIALIZER(1)
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pm_initialize
 *
 * Description:
 *   This function is called by MCU-specific one-time at power on reset in
 *   order to initialize the power management capabilities.  This function
 *   must be called *very* early in the initialization sequence *before* any
 *   other device drivers are initialize (since they may attempt to register
 *   with the power management subsystem).
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *    None.
 *
 ****************************************************************************/

void pm_initialize(void)
{
  /* Select governor */

#if defined(CONFIG_PM_GOVERNOR_ACTIVITY)
  g_pmglobals.governor = pm_activity_governor_initialize();
#elif defined(CONFIG_PM_GOVERNOR_GREEDY)
  g_pmglobals.governor = pm_greedy_governor_initialize();
#elif defined(CONFIG_PM_GOVERNOR_CUSTOM)
  /* TODO: call to board function to retrieve custom governor,
   * such as board_pm_governor_initialize()
   */

#  error "Not supported yet"
#endif

  /* Initialize selected governor */

  g_pmglobals.governor->initialize();
}

#endif /* CONFIG_PM */
