/****************************************************************************
 * arch/risc-v/src/k210/k210_timerisr.c
 *
 *   Copyright (C) 2019 Masayuki Ishikawa. All rights reserved.
 *   Author: Masayuki Ishikawa <masayuki.ishikawa@gmail.com>
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

#include <stdint.h>
#include <time.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "riscv_arch.h"

#include "k210.h"
#include "k210_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define getreg64(a)   (*(volatile uint64_t *)(a))
#define putreg64(v,a) (*(volatile uint64_t *)(a) = (v))

#ifdef CONFIG_K210_WITH_QEMU
#define TICK_COUNT (10000000 / TICK_PER_SEC)
#else
#define TICK_COUNT ((k210_get_cpuclk() / 50) / TICK_PER_SEC)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool _b_tick_started = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  k210_reload_mtimecmp
 ****************************************************************************/

static void k210_reload_mtimecmp(void)
{
  irqstate_t flags = spin_lock_irqsave();

  uint64_t current;
  uint64_t next;

  if (!_b_tick_started)
    {
      _b_tick_started = true;
      current = getreg64(K210_CLINT_MTIME);
    }
  else
    {
      current = getreg64(K210_CLINT_MTIMECMP);
    }

  uint64_t tick = TICK_COUNT;
  next = current + tick;

  putreg64(next, K210_CLINT_MTIMECMP);

  spin_unlock_irqrestore(flags);
}

/****************************************************************************
 * Name:  k210_timerisr
 ****************************************************************************/

static int k210_timerisr(int irq, void *context, FAR void *arg)
{
  k210_reload_mtimecmp();

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
#if 1
  /* Attach timer interrupt handler */

  irq_attach(K210_IRQ_MTIMER, k210_timerisr, NULL);

  /* Reload CLINT mtimecmp */

  k210_reload_mtimecmp();

  /* And enable the timer interrupt */

  up_enable_irq(K210_IRQ_MTIMER);
#endif
}
