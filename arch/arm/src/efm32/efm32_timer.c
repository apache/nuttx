/****************************************************************************
 * arch/arm/src/efm32/efm32_timer.c
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

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/efm32_timer.h"
#include "efm32_config.h"
#include "efm32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_TIMER_INFO
#  define efm32_timer_dumpgpio(p,m) efm32_dumpgpio(p,m)
#else
#  define efm32_timer_dumpgpio(p,m)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efm32_timer_dumpregs
 *
 * Description:
 *   Dump all timer registers.
 *
 * Input Parameters:
 *   base - A base address of timer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void efm32_timer_dumpregs(uintptr_t base, const char *msg)
{
#ifdef CONFIG_DEBUG_TIMER_INFO
  int i;

  tmrinfo("%s:\n", msg);
  tmrinfo("  CTRL: %04x STATUS: %04x   IEN: %04x     IF: %04x\n",
          getreg32(base + EFM32_TIMER_CTRL_OFFSET),
          getreg32(base + EFM32_TIMER_STATUS_OFFSET),
          getreg32(base + EFM32_TIMER_IEN_OFFSET),
          getreg32(base + EFM32_TIMER_IF_OFFSET));
  tmrinfo("   TOP: %04x   TOPB: %04x   CNT: %04x  ROUTE: %04x\n",
          getreg32(base + EFM32_TIMER_TOP_OFFSET),
          getreg32(base + EFM32_TIMER_TOPB_OFFSET),
          getreg32(base + EFM32_TIMER_CNT_OFFSET),
          getreg32(base + EFM32_TIMER_ROUTE_OFFSET));

  for (i = 0; i < EFM32_TIMER_NCC; i++)
    {
      uintptr_t base_cc = base + EFM32_TIMER_CC_OFFSET(i);

      tmrinfo("CC%d => CTRL: %04x    CCV:  %04x  CCVP: %04x CCVB: %04x\n",
              i
              getreg32(base_cc + EFM32_TIMER_CC_CTRL_OFFSET),
              getreg32(base_cc + EFM32_TIMER_CC_CCV_OFFSET),
              getreg32(base_cc + EFM32_TIMER_CC_CCVP_OFFSET),
              getreg32(base_cc + EFM32_TIMER_CC_CCVB_OFFSET));
    }

  tmrinfo("DTCTRL: %04x DTTIME: %04x  DTFC: %04x DTOGEN:  %04x\n",
          getreg32(base + EFM32_TIMER_CTRL_OFFSET),
          getreg32(base + EFM32_TIMER_STATUS_OFFSET),
          getreg32(base + EFM32_TIMER_IEN_OFFSET),
          getreg32(base + EFM32_TIMER_IF_OFFSET));
  tmrinfo("DTFAULT: %04x DTFAULTC: %04x  DTLOCK: %04x\n",
          getreg32(base + EFM32_TIMER_CTRL_OFFSET),
          getreg32(base + EFM32_TIMER_STATUS_OFFSET),
          getreg32(base + EFM32_TIMER_IEN_OFFSET));
#endif
}

/****************************************************************************
 * Name: efm32_timer_reset
 *
 * Description:
 *   reset timer into reset state
 *
 * Input Parameters:
 *   base - A base address of timer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void efm32_timer_reset(uintptr_t base)
{
  int i;

  /* Make sure disabled first, before resetting other registers */

  putreg32(TIMER_CMD_STOP, base + EFM32_TIMER_CMD_OFFSET);

  /* Reset timer register */

  putreg32(_TIMER_CTRL_RESETVALUE,  base + EFM32_TIMER_CTRL_OFFSET);
  putreg32(_TIMER_IEN_RESETVALUE,   base + EFM32_TIMER_STATUS_OFFSET);
  putreg32(_TIMER_IFC_MASK,         base + EFM32_TIMER_IEN_OFFSET);
  putreg32(_TIMER_TOP_RESETVALUE,   base + EFM32_TIMER_IF_OFFSET);
  putreg32(_TIMER_TOPB_RESETVALUE,  base + EFM32_TIMER_CTRL_OFFSET);
  putreg32(_TIMER_CNT_RESETVALUE,   base + EFM32_TIMER_CMD_OFFSET);

  /* Do not reset route register, setting should be done independently
   * (Note: ROUTE register may be locked by DTLOCK register.)
   */

  /* putreg32(_TIMER_ROUTE_RESETVALUE, base + EFM32_TIMER_ROUTE_OFFSET); */

  for (i = 0; i < EFM32_TIMER_NCC; i++)
    {
      uintptr_t base_cc = base + EFM32_TIMER_CC_OFFSET(i);
      putreg32(_TIMER_CC_CTRL_RESETVALUE,
               base_cc + EFM32_TIMER_CC_CTRL_OFFSET);
      putreg32(_TIMER_CC_CCV_RESETVALUE,
               base_cc + EFM32_TIMER_CC_CCV_OFFSET);
      putreg32(_TIMER_CC_CCVB_RESETVALUE,
               base_cc + EFM32_TIMER_CC_CCVB_OFFSET);
    }

  /* Reset dead time insertion module, no effect on timers without DTI */

#ifdef TIMER_DTLOCK_LOCKKEY_UNLOCK
  /* Unlock DTI registers first in case locked */

  putreg32(TIMER_DTLOCK_LOCKKEY_UNLOCK, base + EFM32_TIMER_DTLOCK_OFFSET);

  putreg32(_TIMER_DTCTRL_RESETVALUE, base + EFM32_TIMER_DTCTRL_OFFSET);
  putreg32(_TIMER_DTTIME_RESETVALUE, base + EFM32_TIMER_DTTIME_OFFSET);
  putreg32(_TIMER_DTFC_RESETVALUE, base + EFM32_TIMER_DTFC_OFFSET);
  putreg32(_TIMER_DTOGEN_RESETVALUE, base + EFM32_TIMER_DTOGEN_OFFSET);
  putreg32(_TIMER_DTFAULTC_MASK, base + EFM32_TIMER_DTFAULTC_OFFSET);
#endif
}

/****************************************************************************
 * Name: efm32_timer_set_freq
 *
 * Description:
 *   set prescaler and top timer with best value to have "freq"
 *
 * Input Parameters:
 *   base       - A base address of timer
 *   clk_freq   - Clock source of timer.
 *   freq       - Wanted frequency.
 *
 * Returned Value:
 *   prescaler set, -1 in case of error.
 *
 ****************************************************************************/

int efm32_timer_set_freq(uintptr_t base, uint32_t clk_freq, uint32_t freq)
{
  int prescaler = 0;
  int cnt_freq = clk_freq >> 16;
  int reload;

  while (cnt_freq > freq)
    {
      prescaler++;
      cnt_freq >>= 1;
      if (prescaler > (_TIMER_CTRL_PRESC_MASK >> _TIMER_CTRL_PRESC_SHIFT))
        {
          return -1;
        }
    }

  modifyreg32(base + EFM32_TIMER_CTRL_OFFSET,
              _TIMER_CTRL_PRESC_MASK,
              prescaler << _TIMER_CTRL_PRESC_SHIFT);

  prescaler = 1 << prescaler;

  reload = (clk_freq / prescaler / freq);

  tmrinfo("Source: %4" PRIx32 "Hz Div: %4x Reload: %4x\n",
          clk_freq, prescaler, reload);

  putreg32(reload, base + EFM32_TIMER_TOP_OFFSET);

  return prescaler;
}
