/****************************************************************************
 * arch/risc-v/src/mindgrove/mindgrove_timerisr.c
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

// #include "riscv_arch.h"

#include "mindgrove.h"
#include "mindgrove_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TICK_COUNT 350// (mindgrove_get_hfclk() / TICK_PER_SEC)
#define MSIP       0x02000000

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool _b_tick_started = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* mindgrove mmio registers are a bit odd, by default they are byte-wide
 * registers that are on 32-bit word boundaries. So a "32-bit" registers
 * is actually broken into four bytes spanning a total address space of
 * 16 bytes.
 */

static inline uint64_t mindgrove_clint_time_read(void)
{
  uint64_t r = getreg32(MINDGROVE_CLINT_MTIME+ 0x04 );
  r <<= 32;
  r |= getreg32(MINDGROVE_CLINT_MTIME );
  return r;
}

static inline uint64_t mindgrove_clint_time_cmp_read(void)
{
  uint64_t r = getreg32(MINDGROVE_CLINT_MTIMECMP+ 0x04 );
  r <<= 32;
  r |= getreg32(MINDGROVE_CLINT_MTIMECMP);
  return r;
}

static inline void mindgrove_clint_time_cmp_write(uint64_t v)
{
  putreg64(v, MINDGROVE_CLINT_MTIMECMP  );
}

/* helper function to set/clear csr */

#define csr_clear(csr, val)					\
({								\
  unsigned long __v = (unsigned long)(val);		\
  __asm__ __volatile__ ("csrc " #csr ", %0"		\
            : : "rK" (__v));			\
})

#define csr_set(csr, val)					\
({								\
  unsigned long __v = (unsigned long)(val);		\
  __asm__ __volatile__ ("csrs " #csr ", %0"		\
            : : "rK" (__v));			\
})

/****************************************************************************
 * Name:  mindgrove_reload_mtimecmp
 ****************************************************************************/

static void mindgrove_reload_mtimecmp(void)
{
  irqstate_t flags = spin_lock_irqsave(NULL);

  uint64_t current;
  uint64_t next;

  if (!_b_tick_started)
    {
      _b_tick_started = true;
      current = mindgrove_clint_time_read();
    }
  else
    {
      current = mindgrove_clint_time_cmp_read();
    }

  next = current + TICK_COUNT;
  next = -1;
  // _alert("current : %llu, next: %llu",current,next);
  mindgrove_clint_time_cmp_write(next);
  putreg32(1, MSIP);

  csr_set(mie, MIE_MTIE);
  csr_clear(mip, MIP_MTIP);

  spin_unlock_irqrestore(NULL, flags);
}

/****************************************************************************
 * Name:  mindgrove_timerisr
 ****************************************************************************/

static int mindgrove_timerisr(int irq, void *context, void *arg)
{
  mindgrove_reload_mtimecmp();

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

  irq_attach(RISCV_IRQ_MTIMER, mindgrove_timerisr, NULL);

  /* Reload CLINT mtimecmp */

  mindgrove_reload_mtimecmp();

  /* And enable the timer interrupt */

  up_enable_irq(RISCV_IRQ_MTIMER);
}
