/****************************************************************************
 * arch/risc-v/src/litex/litex_timerisr.c
 *
 *   Copyright (C) 2020 Gregory Nutt. All rights reserved.
 *   Author: hctang <aenrbesaen@126.com>
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
#include <nuttx/clock.h>
#include <arch/board/board.h>

#include "up_arch.h"

#include "litex.h"
#include "litex_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TICK_COUNT (litex_get_hfclk() / TICK_PER_SEC)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool _b_tick_started = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*
 * litex mmio registers are a bit odd, by default they are byte-wide registers 
 * that are on 32-bit word boundaries. So a "32-bit" registers is actually broken 
 * into four bytes spanning a total address space of 16 bytes.
 */
static inline uint64_t litex_clint_time_read(void) {
	uint64_t r = getreg8(LITEX_CLINT_MTIME);
	r <<= 8;
	r |= getreg8(LITEX_CLINT_MTIME + 0x04UL);
	r <<= 8;
	r |= getreg8(LITEX_CLINT_MTIME + 0x08UL);
	r <<= 8;
	r |= getreg8(LITEX_CLINT_MTIME + 0x0cUL);
	r <<= 8;
	r |= getreg8(LITEX_CLINT_MTIME + 0x10UL);
	r <<= 8;
	r |= getreg8(LITEX_CLINT_MTIME + 0x14UL);
	r <<= 8;
	r |= getreg8(LITEX_CLINT_MTIME + 0x18UL);
	r <<= 8;
	r |= getreg8(LITEX_CLINT_MTIME + 0x1cUL);
	return r;
}

static inline uint64_t litex_clint_time_cmp_read(void) {
	uint64_t r = getreg8(LITEX_CLINT_MTIMECMP);
	r <<= 8;
	r |= getreg8(LITEX_CLINT_MTIMECMP + 0x04UL);
	r <<= 8;
	r |= getreg8(LITEX_CLINT_MTIMECMP + 0x08UL);
	r <<= 8;
	r |= getreg8(LITEX_CLINT_MTIMECMP + 0x0cUL);
	r <<= 8;
	r |= getreg8(LITEX_CLINT_MTIMECMP + 0x10UL);
	r <<= 8;
	r |= getreg8(LITEX_CLINT_MTIMECMP + 0x14UL);
	r <<= 8;
	r |= getreg8(LITEX_CLINT_MTIMECMP + 0x18UL);
	r <<= 8;
	r |= getreg8(LITEX_CLINT_MTIMECMP + 0x1cUL);
	return r;
}

static inline void litex_clint_time_cmp_write(uint64_t v) {
	putreg8(v >> 56, LITEX_CLINT_MTIMECMP);
	putreg8(v >> 48, LITEX_CLINT_MTIMECMP + 0x04UL);
	putreg8(v >> 40, LITEX_CLINT_MTIMECMP + 0x08UL);
	putreg8(v >> 32, LITEX_CLINT_MTIMECMP + 0x0cUL);
	putreg8(v >> 24, LITEX_CLINT_MTIMECMP + 0x10UL);
	putreg8(v >> 16, LITEX_CLINT_MTIMECMP + 0x14UL);
	putreg8(v >> 8, LITEX_CLINT_MTIMECMP + 0x18UL);
	putreg8(v, LITEX_CLINT_MTIMECMP + 0x1cUL);
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
 * Name:  litex_reload_mtimecmp
 ****************************************************************************/

static void litex_reload_mtimecmp(void)
{
  irqstate_t flags = spin_lock_irqsave();

  uint64_t current;
  uint64_t next;

  if (!_b_tick_started)
    {
      _b_tick_started = true;
      putreg8(1, LITEX_CLINT_LATCH);
      current = litex_clint_time_read();
    }
  else
    {
      current = litex_clint_time_cmp_read();
    }

  next = current + TICK_COUNT;
  
  litex_clint_time_cmp_write(next);
  putreg8(1, LITEX_CLINT_LATCH);
  csr_set(mie, MIE_MTIE);
  csr_clear(mip, MIP_MTIP);

  spin_unlock_irqrestore(flags);
}

/****************************************************************************
 * Name:  litex_timerisr
 ****************************************************************************/

static int litex_timerisr(int irq, void *context, FAR void *arg)
{
  litex_reload_mtimecmp();

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

  irq_attach(LITEX_IRQ_MTIMER, litex_timerisr, NULL);

  /* Reload CLINT mtimecmp */

  litex_reload_mtimecmp();
  up_lowputc('Z');
  up_lowputc('0');
  up_lowputc('\n');
  /* And enable the timer interrupt */

  up_enable_irq(LITEX_IRQ_MTIMER);
}
