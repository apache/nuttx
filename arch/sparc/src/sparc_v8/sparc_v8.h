/****************************************************************************
 * arch/sparc/src/sparc_v8/sparc_v8.h
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

#ifndef __ARCH_SPARC_SRC_SPARC_V8_SPARC_V8_H
#define __ARCH_SPARC_SRC_SPARC_V8_SPARC_V8_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/compiler.h>
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Macros to handle saving and restore interrupt state.  The state is copied
 * from the stack to the TCB, but only a referenced is passed to get the
 * state from the TCB.
 */

#define up_restorestate(regs) (g_current_regs = regs)

#define up_savestate(regs)   trap_flush_task(regs, (uint32_t*)g_current_regs)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
/* This is the beginning of heap as provided from up_head.S.This is the first
 * address in DRAM after the loaded program+bss+idle stack.  The end of the
 * heap is CONFIG_RAM_END
 */

extern uint32_t g_idle_topstack;

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name:  up_copystate
 *
 * Description:
 *  Copy the contents of a register state save structure from one location to
 *  another.
 *
 ****************************************************************************/

void up_copystate(uint32_t *dest, uint32_t *src);

void task_flush_trap(uint32_t *trap, uint32_t *task);
void trap_flush_task(uint32_t *task, uint32_t *trap);

/****************************************************************************
 * Name:  up_doirq
 *
 * Description:
 *   Dispatch an interrupt.
 *
 ****************************************************************************/

uint32_t *up_doirq(int irq, uint32_t *regs);

#endif /* __ASSEMBLY__ */
#endif  /* __ARCH_SPARC_SRC_SPARC_V8_SPARC_V8_H */

