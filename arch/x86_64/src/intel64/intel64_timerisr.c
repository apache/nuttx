/****************************************************************************
 * arch/x86/src/intel64/intel64_timerisr.c
 *
 *   Copyright (C) 2011, 2017 Gregory Nutt.
 *                 2020 Chung-Fan Yang
 *
 *   All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Chung-Fan Yang <sonic.tw.tp@gmail.com>
 *
 *   Based on Bran's kernel development tutorials. Rewritten for JamesM's
 *   kernel development tutorials. Reworked based on x86-64 TSC deadline timer
 *   by Chung-Fan Yang, referencing Jailhouse hypervisor's inmate libaray.
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
#include <arch/irq.h>
#include <arch/io.h>
#include <arch/board/board.h>

#include "clock/clock.h"
#include "up_internal.h"
#include "up_arch.h"

#include <stdio.h>

#include "chip.h"
#include "intel64.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define NS_PER_USEC    1000UL
#define NS_PER_MSEC    1000000UL
#define NS_PER_SEC     1000000000UL

/****************************************************************************
 * Private Data
 ****************************************************************************/

unsigned long x86_64_timer_freq;

static unsigned long tsc_overflow;
static unsigned long tsc_last;
static unsigned long tsc_overflows;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  apic_timer_set
 *
 * Description:
 *   Set a time for APIC timer to fire
 *
 ****************************************************************************/

void apic_timer_set(unsigned long timeout_ns)
{
  unsigned long long ticks =
    (unsigned long long)timeout_ns * x86_64_timer_freq / NS_PER_SEC;
#ifdef CONFIG_ARCH_INTEL64_HAVE_TSC_DEADLINE
    write_msr(MSR_IA32_TSC_DEADLINE, rdtsc() + ticks);
#else
    write_msr(MSR_X2APIC_TMICT, ticks);
#endif
}

/****************************************************************************
 * Function: intel64_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int intel64_timerisr(int irq, uint32_t *regs, void *arg)
{
  /* Process timer interrupt */
  nxsched_process_timer();
  apic_timer_set(CONFIG_USEC_PER_TICK * NS_PER_USEC);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  unsigned long ecx;
  uint32_t vector = IRQ0;

  (void)irq_attach(IRQ0, (xcpt_t)intel64_timerisr, NULL);

#ifdef CONFIG_ARCH_INTEL64_HAVE_TSC_DEADLINE
  vector |= MSR_X2APIC_LVTT_TSC_DEADLINE;
#endif

  write_msr(MSR_X2APIC_LVTT, vector);

  asm volatile("mfence" : : : "memory");

  apic_timer_set(NS_PER_MSEC);

  return;
}
