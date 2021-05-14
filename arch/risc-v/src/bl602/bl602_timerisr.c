/****************************************************************************
 * arch/risc-v/src/bl602/bl602_timerisr.c
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
#include <time.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/spinlock.h>
#include <arch/board/board.h>
#include "hardware/bl602_clic.h"
#include "riscv_arch.h"

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Private definetions: mtimer frequency */
#define TICK_COUNT (10 * 1000 * 1000 / TICK_PER_SEC)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_b_tick_started = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* bl602 mmio registers are a bit odd, by default they are byte-wide
 * registers that are on 32-bit word boundaries. So a "32-bit" registers
 * is actually broken into four bytes spanning a total address space of
 * 16 bytes.
 */

static inline uint64_t bl602_clint_time_read(void)
{
  uint64_t r = getreg32(BL602_CLIC_MTIME + 4);
  r <<= 32;
  r |= getreg32(BL602_CLIC_MTIME);

  return r;
}

static inline uint64_t bl602_clint_time_cmp_read(void)
{
  return getreg64(BL602_CLIC_MTIMECMP);
}

static inline void bl602_clint_time_cmp_write(uint64_t v)
{
  putreg64(v, BL602_CLIC_MTIMECMP);
}

/****************************************************************************
 * Name:  bl602_reload_mtimecmp
 ****************************************************************************/

static void bl602_reload_mtimecmp(void)
{
  irqstate_t flags = spin_lock_irqsave(NULL);

  uint64_t current;
  uint64_t next;

  if (!g_b_tick_started)
    {
      g_b_tick_started = true;
      current          = bl602_clint_time_read();
    }
  else
    {
      current = bl602_clint_time_cmp_read();
    }

  next = current + TICK_COUNT;

  bl602_clint_time_cmp_write(next);

  spin_unlock_irqrestore(NULL, flags);
}

/****************************************************************************
 * Name:  bl602_timerisr
 ****************************************************************************/

static int bl602_timerisr(int irq, void *context, FAR void *arg)
{
  bl602_reload_mtimecmp();

  /* Process timer interrupt */

  nxsched_process_timer();
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  /* Attach timer interrupt handler */

  irq_attach(BL602_IRQ_MTIMER, bl602_timerisr, NULL);

  /* Reload CLINT mtimecmp */

  bl602_reload_mtimecmp();

  /* And enable the timer interrupt */

  up_enable_irq(BL602_IRQ_MTIMER);
}
