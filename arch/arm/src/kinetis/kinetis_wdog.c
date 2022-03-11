/****************************************************************************
 * arch/arm/src/kinetis/kinetis_wdog.c
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

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "kinetis.h"
#include "hardware/kinetis_wdog.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_wdunlock
 *
 * Description:
 *   Watchdog timer unlock routine. Writing 0xc520 followed by 0xd928 will
 *   unlock the write once registers in the WDOG so they are writable
 *   within the WCT period.
 *
 ****************************************************************************/

static void kinetis_wdunlock(void)
{
  irqstate_t flags;

  /* This sequence must execute within 20 clock cycles.  Disable interrupts
   * to assure that the following steps are atomic.
   */

  flags = enter_critical_section();

  /* Write 0xC520 followed by 0xD928 to the unlock register */

  putreg16(0xc520, KINETIS_WDOG_UNLOCK);
  putreg16(0xd928, KINETIS_WDOG_UNLOCK);
  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_wddisable
 *
 * Description:
 *   Disable the watchdog timer
 *
 ****************************************************************************/

void kinetis_wddisable(void)
{
  uint16_t regval;

  /* Unlock the watchdog so that we can write to registers */

  kinetis_wdunlock();

  /* Clear the WDOGEN bit to disable the watchdog */

  regval  = getreg16(KINETIS_WDOG_STCTRLH);
  regval &= ~WDOG_STCTRLH_WDOGEN;
  putreg16(regval, KINETIS_WDOG_STCTRLH);
}
