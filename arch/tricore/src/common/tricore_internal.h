/****************************************************************************
 * arch/tricore/src/common/tricore_internal.h
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

#ifndef __ARCH_TRICORE_SRC_COMMON_TRICORE_INTERNAL_H
#define __ARCH_TRICORE_SRC_COMMON_TRICORE_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/compiler.h>
/* #  include <nuttx/arch.h> */
#  include <stdbool.h>
#  include <sys/types.h>
#  include <stdint.h>
#  include <syscall.h>

#endif

#ifndef CONFIG_DEV_CONSOLE
#  undef  USE_SERIALDRIVER
#  undef  USE_EARLYSERIALINIT
#else
#  if defined(CONFIG_LWL_CONSOLE)
#    undef  USE_SERIALDRIVER
#    undef  USE_EARLYSERIALINIT
#  elif defined(CONFIG_CONSOLE_SYSLOG)
#    undef  USE_SERIALDRIVER
#    undef  USE_EARLYSERIALINIT
#  elif defined(CONFIG_SERIAL_RTT_CONSOLE)
#    undef  USE_SERIALDRIVER
#    undef  USE_EARLYSERIALINIT
#  elif defined(CONFIG_RPMSG_UART_CONSOLE)
#    undef  USE_SERIALDRIVER
#    undef  USE_EARLYSERIALINIT
#  else
#    define USE_SERIALDRIVER 1
#    define USE_EARLYSERIALINIT 1
#  endif
#endif

#if !defined(USE_SERIALDRIVER) && defined(CONFIG_STANDARD_SERIAL)
#  define USE_SERIALDRIVER 1
#endif

/* STACK_ALIGNMENT, STACK_ALIGN_MASK, STACK_ALIGN_UP/DOWN come from
 * <nuttx/irq.h> which derives them from STACKFRAME_ALIGN.
 */

/* Check if an interrupt stack size is configured */

#ifndef CONFIG_ARCH_INTERRUPTSTACK
#  define CONFIG_ARCH_INTERRUPTSTACK 0
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define INTSTACK_SIZE (CONFIG_ARCH_INTERRUPTSTACK & ~STACK_ALIGN_MASK)

#define STACK_COLOR    0xdeadbeef
#define INTSTACK_COLOR 0xdeadbeef
#define HEAP_COLOR     'h'

#define getreg8(a)     (*(volatile uint8_t *)(a))
#define putreg8(v,a)   (*(volatile uint8_t *)(a) = (v))
#define getreg16(a)    (*(volatile uint16_t *)(a))
#define putreg16(v,a)  (*(volatile uint16_t *)(a) = (v))
#define getreg32(a)    (*(volatile uint32_t *)(a))
#define putreg32(v,a)  (*(volatile uint32_t *)(a) = (v))
#define getreg64(a)    (*(volatile uint64_t *)(a))
#define putreg64(v,a)  (*(volatile uint64_t *)(a) = (v))

/* Non-atomic, but more effective modification of registers */

#define modreg8(v,m,a)  putreg8((getreg8(a) & ~(m)) | ((v) & (m)), (a))
#define modreg16(v,m,a) putreg16((getreg16(a) & ~(m)) | ((v) & (m)), (a))
#define modreg32(v,m,a) putreg32((getreg32(a) & ~(m)) | ((v) & (m)), (a))
#define modreg64(v,m,a) putreg64((getreg64(a) & ~(m)) | ((v) & (m)), (a))

/* Context switching */

#ifndef tricore_fullcontextrestore
#  define tricore_fullcontextrestore(restoreregs) \
    sys_call1(SYS_restore_context, (uintptr_t)restoreregs);
#else
extern void tricore_fullcontextrestore(uintptr_t *restoreregs);
#endif

#ifndef tricore_switchcontext
#  define tricore_switchcontext(saveregs, restoreregs) \
    sys_call2(SYS_switch_context, (uintptr_t)saveregs, (uintptr_t)restoreregs);
#else
extern void tricore_switchcontext(uintptr_t **saveregs,
                                  uintptr_t *restoreregs);
#endif

#ifndef __ASSEMBLY__
typedef void (*up_vector_t)(void);
#endif

extern uintptr_t        _sheap[];
extern uintptr_t        _eheap[];

extern uintptr_t __USTACK[];
#if defined(CONFIG_ARCH_INTERRUPTSTACK)
extern uintptr_t        __istack[];
#endif

/* Macros to handle saving and restoring interrupt state. */

#define tricore_savestate(regs)    (regs = up_current_regs())
#define tricore_restorestate(regs) (up_set_current_regs(regs))

/* Signal handling **********************************************************/

void tricore_sigdeliver(void);

/* Exception Handler ********************************************************/

void tricore_svcall(volatile void *trap);
void tricore_trapcall(volatile void *trap);

/* Context Save Areas *******************************************************/

uintptr_t *tricore_alloc_csa(uintptr_t pc, uintptr_t sp,
                             uintptr_t psw, bool irqsave);
void tricore_reclaim_csa(uintptr_t pcxi);

/* Debug ********************************************************************/

#ifdef CONFIG_STACK_COLORATION
size_t tricore_stack_check(uintptr_t alloc, size_t size);
void tricore_stack_color(void *stackbase, size_t nbytes);
#endif

void tricore_endinit_disable(void);
void tricore_endinit_enable(void);
void tricore_safety_endinit_enable(void);
void tricore_wdt_disable(void);
void up_clockconfig(void);

#if defined(CONFIG_ARCH_CHIP_FAMILY_TC3X)
void aurix_cpu_endinit_enable(bool enable);
void aurix_safety_endinit_enable(bool enable);
#endif

void aurix_earlyserialinit(void);
void aurix_serialinit(void);

void board_aurix_setup_serial_pin(int idx);

#ifdef CONFIG_TRICORE_FLASH_MTD
struct mtd_dev_s;
struct mtd_dev_s *tricore_flash_initialize(void);
void tricore_flash_get_last_dmu_state(uint32_t *err, uint32_t *status,
                                      uint32_t *phase);
#endif

#endif /* __ARCH_TRICORE_SRC_COMMON_TRICORE_INTERNAL_H */
