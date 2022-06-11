/****************************************************************************
 * arch/x86_64/src/intel64/intel64_timerisr.c
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
#include <arch/irq.h>
#include <arch/io.h>
#include <arch/board/board.h>

#include "clock/clock.h"
#include "up_internal.h"

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

  irq_attach(IRQ0, (xcpt_t)intel64_timerisr, NULL);

#ifdef CONFIG_ARCH_INTEL64_HAVE_TSC_DEADLINE
  vector |= MSR_X2APIC_LVTT_TSC_DEADLINE;
#endif

  write_msr(MSR_X2APIC_LVTT, vector);

  asm volatile("mfence" : : : "memory");

  apic_timer_set(NS_PER_MSEC);

  return;
}
