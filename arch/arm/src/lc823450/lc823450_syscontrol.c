/****************************************************************************
 * arch/arm/src/lc823450/lc823450_syscontrol.c
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
#include <nuttx/spinlock.h>
#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "lc823450_gpio.h"
#include "lc823450_syscontrol.h"
#include <arch/chip/clk.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct clk_st lc823450_clocks[] = LC823450_CLOCKS;

/****************************************************************************
 * Public Data
 ****************************************************************************/

uint8_t cpu_ver;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_cpu_ver
 ****************************************************************************/

uint32_t get_cpu_ver(void)
{
  uint32_t ret = 0;
  uint32_t reg = getreg32(MODEM) & MODEM_MAV_MASK;

  switch (reg)
    {
      case MODEM_MAV_ES1:
        sinfo("ES1\n");
        break;

      case MODEM_MAV_ES2:
        sinfo("ES2\n");
        ret = 1;
        break;

      default:
        sinfo("???\n");
        break;
    }

  cpu_ver = ret;
  return ret;
}

/****************************************************************************
 * Name: mod_stby_regs
 *
 * Input Parameters:
 *   enabits : specify regions to be enabled
 *   disbits : specify regions to be disabled
 *
 ****************************************************************************/

#ifdef CONFIG_LC823450_LSISTBY
void mod_stby_regs(uint32_t enabits, uint32_t disbits)
{
  /* TODO : need to lock */

  /* isolate first if needed */

  modifyreg32(ISOCNT, disbits, 0);

  /* then modify LSTSTBY register */

  modifyreg32(LSISTBY, enabits, disbits);

  if (enabits)
    {
      /* stand-by to active case
       * assumption: the specified blocks are already isolated
       */

      up_udelay(100); /* need to wait 100us */

      /* then disable isolation for the region */

      modifyreg32(ISOCNT, 0, enabits);
    }

  /* sinfo("ISOCNT=0x%x, LSISTBY=0x%x\n",
   * getreg32(ISOCNT), getreg32(LSISTBY));
   */
}
#endif /* CONFIG_LC823450_LSISTBY */

/****************************************************************************
 * Name: up_enable_clk
 ****************************************************************************/

void up_enable_clk(enum clock_e clk)
{
  irqstate_t flags;
  flags = spin_lock_irqsave(NULL);

  DEBUGASSERT(clk < LC823450_CLOCK_NUM);

  if (lc823450_clocks[clk].count++ == 0)
    {
      modifyreg32(lc823450_clocks[clk].regaddr,
                  0, lc823450_clocks[clk].regmask);
    }

  spin_unlock_irqrestore(NULL, flags);
}

/****************************************************************************
 * Name: up_disable_clk
 ****************************************************************************/

void up_disable_clk(enum clock_e clk)
{
  irqstate_t flags;
  flags = spin_lock_irqsave(NULL);

  DEBUGASSERT(clk < LC823450_CLOCK_NUM);

  if (--lc823450_clocks[clk].count == 0)
    {
      modifyreg32(lc823450_clocks[clk].regaddr,
                  lc823450_clocks[clk].regmask, 0);
    }

  /*  DEBUGASSERT(lc823450_clocks[clk].count >= 0); */

  if (lc823450_clocks[clk].count < 0)
    {
      lc823450_clocks[clk].count = 0;
    }

  spin_unlock_irqrestore(NULL, flags);
}

/****************************************************************************
 * Name: lc823450_clock_dump
 ****************************************************************************/

void lc823450_clock_dump(void)
{
  int i;

  for (i = 0; i < LC823450_CLOCK_NUM; i++)
    {
      sinfo("%s:%d\n", lc823450_clocks[i].name,
            lc823450_clocks[i].count);
    }
}
