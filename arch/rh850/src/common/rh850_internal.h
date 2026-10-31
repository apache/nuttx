/****************************************************************************
 * arch/rh850/src/common/rh850_internal.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_RH850_SRC_COMMON_RH850_INTERNAL_H
#define __ARCH_RH850_SRC_COMMON_RH850_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdbool.h>
#  include <stddef.h>
#  include <stdint.h>
#  include <syscall.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Console driver: the serial driver unless the console is the syslog */

#ifndef CONFIG_DEV_CONSOLE
#  undef  USE_SERIALDRIVER
#else
#  if defined(CONFIG_CONSOLE_SYSLOG)
#    undef  USE_SERIALDRIVER
#  else
#    define USE_SERIALDRIVER 1
#  endif
#endif

#define STACK_COLOR    0xdeadbeef

/* Context switch and restore through TRAP 0 */

#define rh850_switchcontext()       sys_call0(SYS_switch_context)
#define rh850_fullcontextrestore()  sys_call0(SYS_restore_context)

#define getreg8(a)          (*(volatile uint8_t *)(a))
#define putreg8(v,a)        (*(volatile uint8_t *)(a) = (v))
#define getreg16(a)         (*(volatile uint16_t *)(a))
#define putreg16(v,a)       (*(volatile uint16_t *)(a) = (v))
#define getreg32(a)         (*(volatile uint32_t *)(a))
#define putreg32(v,a)       (*(volatile uint32_t *)(a) = (v))

/* Exception level passed to rh850_exception() */

#define RH850_EXC_LEVEL_EI  0
#define RH850_EXC_LEVEL_FE  1

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Top of the idle thread stack (linker script), the heap starts there */

extern uint8_t g_idle_topstack[];

extern uint8_t g_intstackalloc[];
extern uint8_t g_intstacktop[];

/* EIINT nesting depth */

extern volatile uint8_t g_rh850_irqnest;

void rh850_start(void);

/* Software system reset, does not return */

void up_systemreset(void) noreturn_function;

/* Chip: clock and standby setup, console */

void rh850_clockconfig(void);
void rh850_lowsetup(void);
void rh850_lowputc(int ch);
void rh850_serialinit(void);
void u2c_rlin3_setup(uintptr_t base, uint32_t baud);
void u2c_rlin3_putc(uintptr_t base, int ch);
bool u2c_irq_pending(int irq);
void u2c_irq_raise(int irq);
int u2c_irq_setpriority(int irq, int priority);

/* Exception entry, signals, stacks */

uintptr_t *rh850_doirq(int irq, uintptr_t *regs);
uintptr_t *rh850_doswi(uintptr_t *regs);
void rh850_exception(uint32_t cause, uint32_t level, uintptr_t *regs);
void rh850_sigdeliver(void);
void rh850_stack_color(void *stackbase, size_t nbytes);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RH850_SRC_COMMON_RH850_INTERNAL_H */
