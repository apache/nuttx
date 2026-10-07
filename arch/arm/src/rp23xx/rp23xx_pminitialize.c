/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_pminitialize.c
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

#include <nuttx/arch.h>
#include <nuttx/power/pm.h>

#include "arm_internal.h"
#include "rp23xx_pm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_RP23XX_PM_WAKEUP_GPIO_HIGH
#  define RP23XX_PM_WAKEUP_ACTIVE_HIGH true
#else
#  define RP23XX_PM_WAKEUP_ACTIVE_HIGH false
#endif

#ifdef CONFIG_RP23XX_PM_WAKEUP_GPIO_EDGE
#  define RP23XX_PM_WAKEUP_ON_EDGE true
#else
#  define RP23XX_PM_WAKEUP_ON_EDGE false
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_pminitialize
 *
 * Description:
 *   This function is called by MCU-specific logic at power-on reset in
 *   order to provide one-time initialization the power management
 *   subsystem.  This function must be called *very* early in the
 *   initialization sequence *before* any other device drivers are
 *   initialized (since they may attempt to register with the power
 *   management subsystem).
 *
 ****************************************************************************/

void arm_pminitialize(void)
{
  /* Initialize the NuttX power management subsystem proper */

  pm_initialize();

#if defined(CONFIG_RP23XX_PM) && CONFIG_RP23XX_PM_WAKEUP_GPIO >= 0
  /* Arm the wake pin, without which PM_SLEEP is never dormant */

  rp23xx_pm_gpio_wakeup(CONFIG_RP23XX_PM_WAKEUP_GPIO,
                        RP23XX_PM_WAKEUP_ON_EDGE,
                        RP23XX_PM_WAKEUP_ACTIVE_HIGH);
#endif

#ifdef CONFIG_RP23XX_PM_QUIESCE_PADS
  rp23xx_pm_pads_quiesce();
#endif
}
