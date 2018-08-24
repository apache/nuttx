/****************************************************************************
 * arch/arm/src/lc823450/lc823450_syscontrol.c
 *
 *   Copyright 2014,2015,2016,2017,2018 Sony Video & Sound Products Inc.
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
 *   Author: Nobutaka Toyoshima <Nobutaka.Toyoshima@jp.sony.com>
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
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <stdint.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

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
        sinfo("ES1 \n");
        break;

      case MODEM_MAV_ES2:
        sinfo("ES2 \n");
        ret = 1;
        break;

      default:
        sinfo("??? \n");
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
      /* stand-by to active case */
      /* assumption: the specified blocks are already isolated */

      up_udelay(100); /* need to wait 100us */

      /* then disable isolation for the region */

      modifyreg32(ISOCNT, 0, enabits);
    }

  /* sinfo("ISOCNT=0x%x, LSISTBY=0x%x \n", getreg32(ISOCNT), getreg32(LSISTBY)); */
}
#endif /* CONFIG_LC823450_LSISTBY */

/****************************************************************************
 * Name: up_enable_clk
 ****************************************************************************/

void up_enable_clk(enum clock_e clk)
{
  irqstate_t flags;
  flags = spin_lock_irqsave();

  DEBUGASSERT(clk < LC823450_CLOCK_NUM);

  if (lc823450_clocks[clk].count++ == 0)
    {
      modifyreg32(lc823450_clocks[clk].regaddr,
                  0, lc823450_clocks[clk].regmask);
    }

  spin_unlock_irqrestore(flags);
}

/****************************************************************************
 * Name: up_disable_clk
 ****************************************************************************/

void up_disable_clk(enum clock_e clk)
{
  irqstate_t flags;
  flags = spin_lock_irqsave();

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

  spin_unlock_irqrestore(flags);
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
