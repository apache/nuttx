/****************************************************************************
 * drivers/power/pm_initialize.c
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
