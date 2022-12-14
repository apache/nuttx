/****************************************************************************
 * arch/misoc/src/minerva/minerva.h
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

/* In the MINERVA model, the state is copied from the stack to the TCB, but
 * only a referenced is passed to get the state from the TCB.
 */

#define misoc_savestate(regs)      minerva_copystate(regs, (uint32_t*)g_current_regs)
#define up_copystate(rega,regb) minerva_copystate(rega, regb)
#define misoc_restorestate(regs)   (g_current_regs = regs)

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

/* MINERVA requires at least a 4-byte stack alignment.  For floating point
 * use, however, the stack must be aligned to 8-byte addresses.
 */

#ifdef CONFIG_LIBC_FLOATINGPOINT
#  define STACK_ALIGNMENT   8
#else
#  define STACK_ALIGNMENT   4
#endif

/* Stack alignment macros */

#define STACK_ALIGN_MASK    (STACK_ALIGNMENT - 1)
#define STACK_ALIGN_DOWN(a) ((a) & ~STACK_ALIGN_MASK)
#define STACK_ALIGN_UP(a)   (((a) + STACK_ALIGN_MASK) & ~STACK_ALIGN_MASK)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

extern uint32_t g_idle_topstack;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/* Low level initialization provided by board-level logic *******************/

void minerva_board_initialize(void);

/* Context switching ********************************************************/

void minerva_copystate(uint32_t * dest, uint32_t * src);

/* Interrupt decode *********************************************************/

uint32_t *minerva_decodeirq(uint32_t intstat, uint32_t * regs);
uint32_t *minerva_doirq(int irq, uint32_t * regs);

/* Software interrupts ******************************************************/

int minerva_swint(int irq, void *context, void *arg);

/* Signal handling **********************************************************/

void minerva_sigdeliver(void);

/* Cache flushing ***********************************************************/

void minerva_flush_dcache(void);
void minerva_flush_icache(void);

/* Debug ********************************************************************/

void mineva_registerdump(volatile uint32_t *regs);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MISOC_SRC_MINERVA_MINERVA_H */
