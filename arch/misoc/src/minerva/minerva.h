/****************************************************************************
 * arch/misoc/src/minerva/minerva.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Ramtin Amin <keytwo@gmail.com>
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

#ifndef __ARCH_MISOC_SRC_MINERVA_MINERVA_H
#define __ARCH_MISOC_SRC_MINERVA_MINERVA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/compiler.h>
#  include <sys/types.h>
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is the value used to mark the stack for subsequent stack monitoring
 * logic.
 */

#define STACK_COLOR             0xdeadbeef
#define INTSTACK_COLOR          0xdeadbeef
#define HEAP_COLOR              'h'

/* In the MINERVA model, the state is copied from the stack to the TCB, but
 * only a referenced is passed to get the state from the TCB.
 */

#define up_savestate(regs)      minerva_copystate(regs, (uint32_t*)g_current_regs)
#define up_copystate(rega,regb) minerva_copystate(rega, regb)
#define up_restorestate(regs)   (g_current_regs = regs)

/* Determine which (if any) console driver to use.  If a console is enabled
 * and no other console device is specified, then a serial console is
 * assumed.
 */

#ifndef CONFIG_DEV_CONSOLE
#  undef  USE_SERIALDRIVER
#  undef  USE_EARLYSERIALINIT
#else
#  if defined(CONFIG_CONSOLE_SYSLOG)
#    undef  USE_SERIALDRIVER
#    undef  USE_EARLYSERIALINIT
#  else
#    define USE_SERIALDRIVER 1
#    define USE_EARLYSERIALINIT 1
#  endif
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

extern volatile uint32_t *g_current_regs;
extern uint32_t g_idle_topstack;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Low level initialization provided by board-level logic ******************/

void minerva_board_initialize(void);

/* Memory allocation ********************************************************/

#if CONFIG_MM_REGIONS > 1
void minerva_add_region(void);
#endif

/* Context switching ********************************************************/

void minerva_copystate(uint32_t * dest, uint32_t * src);

/* Interrupt decode *********************************************************/

uint32_t *minerva_decodeirq(uint32_t intstat, uint32_t * regs);
uint32_t *minerva_doirq(int irq, uint32_t * regs);

/* Software interrupts ******************************************************/

int minerva_swint(int irq, FAR void *context, FAR void *arg);

/* Rpmsg serial *************************************************************/

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void);
#else
#  define rpmsg_serialinit()
#endif

/* System timer *************************************************************/

void minerva_timer_initialize(void);

/* Signal handling **********************************************************/

void minerva_sigdeliver(void);

/* Cache flushing ***********************************************************/

void minerva_flush_dcache(void);
void minerva_flush_icache(void);

/* Debug ********************************************************************/

void minerva_dumpstate(void);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MISOC_SRC_MINERVA_MINERVA_H */
