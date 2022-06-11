/****************************************************************************
 * boards/arm/stm32/stm3210e-eval/src/stm32_pm.c
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

#include "arm_internal.h"
#include "stm32_pm.h"
#include "stm3210e-eval.h"

#ifdef CONFIG_PM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_pminitialize
 *
 * Description:
 *   This function is called by MCU-specific logic at power-on reset in
 *   order to provide one-time initialization the power management subsystem.
 *   This function must be called *very* early in the initialization sequence
 *   *before* any other device drivers are initialized (since they may
 *   attempt to register with the power management subsystem).
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void arm_pminitialize(void)
{
  /* Initialize the NuttX power management subsystem proper */

  pm_initialize();

#if defined(CONFIG_ARCH_IDLE_CUSTOM) && defined(CONFIG_PM_BUTTONS)
  /* Initialize the buttons to wake up the system from low power modes */

  stm32_pmbuttons();
#endif

  /* Initialize the LED PM */

  stm32_ledpminitialize();
}

#endif /* CONFIG_PM */
