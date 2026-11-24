/****************************************************************************
 * arch/arm/src/pp502x/pp502x_lowsetup.c
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

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/arch.h>

#include "chip.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PP502X_RESET_PULSE_US   1000

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_PP502X_PLL_BOOST

/****************************************************************************
 * Name: pp502x_clockswitch
 *
 * Description:
 *   Write the bus timing and the clock source in the given order with the
 *   CPU stalled around the switch.  Interrupts are masked and the COP is
 *   parked, so the CPU is the only master.
 *
 ****************************************************************************/

static void pp502x_clockswitch(uint32_t source, uint32_t timing,
                               bool timingfirst)
{
  putreg32(PP502X_PROCCTL_STALL(3), PP502X_CPU_CTL);

  if (timingfirst)
    {
      putreg32(timing, PP502X_DEV_TIMING1);
      putreg32(source, PP502X_CLOCK_SOURCE);
    }
  else
    {
      putreg32(source, PP502X_CLOCK_SOURCE);
      putreg32(timing, PP502X_DEV_TIMING1);
    }

  putreg32(PP502X_PROCCTL_STALL(31), PP502X_CPU_CTL);
}

/****************************************************************************
 * Name: pp502x_pllwait
 *
 * Description:
 *   Wait for the PLL lock bit, for at most PP502X_PLL_LOCK_TIMEOUT_US.
 *
 ****************************************************************************/

static bool pp502x_pllwait(void)
{
  uint32_t start = getreg32(PP502X_USEC_TIMER);
  unsigned int polls;

  for (polls = 0; polls < PP502X_PLL_LOCK_MAX_POLLS; polls++)
    {
      if ((getreg32(PP502X_PLL_STATUS) & PP502X_PLL_STATUS_LOCKED) != 0)
        {
          return true;
        }

      if (getreg32(PP502X_USEC_TIMER) - start >=
          PP502X_PLL_LOCK_TIMEOUT_US)
        {
          break;
        }
    }

  return false;
}

#endif /* CONFIG_PP502X_PLL_BOOST */

/****************************************************************************
 * Name: pp502x_clockconfig
 *
 * Description:
 *   Move the core from the 24 MHz crystal to the 80 MHz PLL.  If the PLL
 *   does not lock in time, power it down and stay on the crystal; the bus
 *   is never routed onto an unlocked PLL.  The timers and the UART count
 *   from the crystal and do not depend on this choice.  Without
 *   CONFIG_PP502X_PLL_BOOST the clock stays as the boot loader left it.
 *
 ****************************************************************************/

static void pp502x_clockconfig(void)
{
#ifdef CONFIG_PP502X_PLL_BOOST
  modifyreg32(PP502X_DEV_INIT2, 0, PP502X_DEV_INIT2_PLL_POWER);
  modifyreg32(PP502X_PLL_CONTROL, 0, PP502X_PLL_CONTROL_ENABLE);

  pp502x_clockswitch(PP502X_CLOCK_SOURCE_CRYSTAL,
                     PP502X_DEV_TIMING1_CRYSTAL, false);

  putreg32(PP502X_PLL_CONTROL_80MHZ, PP502X_PLL_CONTROL);

  if (pp502x_pllwait())
    {
      pp502x_clockswitch(PP502X_CLOCK_SOURCE_PLL,
                         PP502X_DEV_TIMING1_80MHZ, true);
    }
  else
    {
      modifyreg32(PP502X_PLL_CONTROL, PP502X_PLL_CONTROL_RUN, 0);
      modifyreg32(PP502X_DEV_INIT2, PP502X_DEV_INIT2_PLL_POWER, 0);
    }
#endif
}

/****************************************************************************
 * Name: pp502x_uart0setup
 *
 * Description:
 *   Route UART0 to the dock connector pins, clock it and pulse its reset.
 *   The line settings are left to the 16550 driver.
 *
 ****************************************************************************/

static void pp502x_uart0setup(void)
{
  modifyreg32(PP502X_SER_PINMUX, PP502X_SER0_PINS_MASK, 0);
  modifyreg32(PP502X_GPO32_ENABLE, PP502X_SER0_PINS_MASK, 0);
  modifyreg32(PP502X_DEV_EN, 0, PP502X_DEV_SER0);

  modifyreg32(PP502X_DEV_RS, 0, PP502X_DEV_SER0);
  up_udelay(PP502X_RESET_PULSE_US);
  modifyreg32(PP502X_DEV_RS, PP502X_DEV_SER0, 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pp502x_lowsetup
 ****************************************************************************/

void pp502x_lowsetup(void)
{
  pp502x_clockconfig();
  pp502x_uart0setup();
}
