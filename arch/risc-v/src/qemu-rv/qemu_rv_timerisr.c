/****************************************************************************
 * arch/risc-v/src/qemu-rv/qemu_rv_timerisr.c
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

#include <assert.h>
#include <stdint.h>
#include <time.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/init.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/arch_alarm.h>
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "riscv_mtimer.h"
#include "riscv_percpu.h"
#include "hardware/qemu_rv_memorymap.h"
#include "hardware/qemu_rv_clint.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MTIMER_FREQ 10000000
#define TICK_COUNT (10000000 / TICK_PER_SEC)

#ifdef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_mtimer_cnt = 0;
static uint32_t g_stimer_pending = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qemu_rv_ssoft_interrupt
 *
 * Description:
 *   This function is S-mode software interrupt handler to proceed
 *   the OS timer
 *
 ****************************************************************************/

static int qemu_rv_ssoft_interrupt(int irq, void *context, void *arg)
{
  /* Cleaer Supervisor Software Interrupt */

  CLEAR_CSR(sip, SIP_SSIP);

  if (g_stimer_pending)
    {
      g_stimer_pending = false;

      /* Proceed the OS timer */

      nxsched_process_timer();
    }
#ifdef CONFIG_SMP
  else
    {
      /* We assume IPI has been issued */

      riscv_pause_handler(irq, context, arg);
    }
#endif

  return 0;
}

/****************************************************************************
 * Name: qemu_rv_reload_mtimecmp
 *
 * Description:
 *   This function is called during start-up to initialize mtimecmp
 *   for CONFIG_BUILD_KERNEL=y
 *
 ****************************************************************************/

static void qemu_rv_reload_mtimecmp(void)
{
  uint64_t current;
  uint64_t next;

  current = READ_CSR(time);
  next = current + TICK_COUNT;
  putreg64(next, QEMU_RV_CLINT_MTIMECMP);
}

#endif /* CONFIG_BUILD_KERNEL */

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
#ifndef CONFIG_BUILD_KERNEL
  struct oneshot_lowerhalf_s *lower = riscv_mtimer_initialize(
    QEMU_RV_CLINT_MTIME, QEMU_RV_CLINT_MTIMECMP,
    RISCV_IRQ_MTIMER, MTIMER_FREQ);

  DEBUGASSERT(lower);

  up_alarm_set_lowerhalf(lower);
#else
  /* NOTE: This function is called in S-mode */

  irq_attach(RISCV_IRQ_SSOFT, qemu_rv_ssoft_interrupt, NULL);
  up_enable_irq(RISCV_IRQ_SSOFT);
#endif
}

#ifdef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Name: up_mtimer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize the M-mode timer
 *
 ****************************************************************************/

void up_mtimer_initialize(void)
{
  uintptr_t irqstacktop = riscv_percpu_get_irqstack();

  /* Set the irq stack base to mscratch */

  WRITE_CSR(mscratch,
            irqstacktop - STACK_ALIGN_DOWN(CONFIG_ARCH_INTERRUPTSTACK));

  /* NOTE: we do not attach a handler for mtimer,
   * because it is handled in the exception_m directly
   */

  up_enable_irq(RISCV_IRQ_MTIMER);
  qemu_rv_reload_mtimecmp();
}

/****************************************************************************
 * Name: qemu_rv_mtimer_interrupt
 *
 * Description:
 *   In RISC-V with S-mode, M-mode timer must be handled in M-mode
 *   This function is called from exception_m in M-mode directly
 *
 ****************************************************************************/

void qemu_rv_mtimer_interrupt(void)
{
  uint64_t current;
  uint64_t next;

  /* Update mtimercmp */

  current = getreg64(QEMU_RV_CLINT_MTIMECMP);
  next = current + TICK_COUNT;
  putreg64(next, QEMU_RV_CLINT_MTIMECMP);

  g_mtimer_cnt++;
  g_stimer_pending = true;

  if (OSINIT_HW_READY())
    {
      /* Post Supervisor Software Interrupt */

      SET_CSR(sip, SIP_SSIP);
    }
}

#endif /* CONFIG_BUILD_KERNEL */
