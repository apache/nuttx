/****************************************************************************
 * arch/tricore/src/common/tricore_internal.h
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
#  include <nuttx/arch.h>
#  include <sys/types.h>
#  include <stdint.h>
#  include <syscall.h>

#  include <IfxCpu_reg.h>
#  include <Ifx_Ssw_Compilers.h>
#  include <Tricore/Compilers/Compilers.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Determine which (if any) console driver to use.  If a console is enabled
 * and no other console device is specified, then a serial console is
 * assumed.
 */

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

/* If some other device is used as the console, then the serial driver may
 * still be needed.  Let's assume that if the upper half serial driver is
 * built, then the lower half will also be needed.  There is no need for
 * the early serial initialization in this case.
 */

#if !defined(USE_SERIALDRIVER) && defined(CONFIG_STANDARD_SERIAL)
#  define USE_SERIALDRIVER 1
#endif

/* For use with EABI and floating point, the stack must be aligned to 8-byte
 * addresses.
 */

#define STACK_ALIGNMENT     8

/* Stack alignment macros */

#define STACK_ALIGN_MASK    (STACK_ALIGNMENT - 1)
#define STACK_ALIGN_DOWN(a) ((a) & ~STACK_ALIGN_MASK)
#define STACK_ALIGN_UP(a)   (((a) + STACK_ALIGN_MASK) & ~STACK_ALIGN_MASK)

/* Check if an interrupt stack size is configured */

#ifndef CONFIG_ARCH_INTERRUPTSTACK
#  define CONFIG_ARCH_INTERRUPTSTACK 0
#endif

#define INTSTACK_SIZE (CONFIG_ARCH_INTERRUPTSTACK & ~STACK_ALIGN_MASK)

/* This is the value used to mark the stack for subsequent stack monitoring
 * logic.
 */

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

/* Address <--> Context Save Areas */

#define tricore_csa2addr(csa) ((uintptr_t *)((((csa) & 0x000F0000) << 12) \
                                             | (((csa) & 0x0000FFFF) << 6)))
#define tricore_addr2csa(addr) ((uintptr_t)(((((uintptr_t)(addr)) & 0xF0000000) >> 12) \
                                            | (((uintptr_t)(addr) & 0x003FFFC0) >> 6)))

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
typedef void (*up_vector_t)(void);
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* This is the beginning of heap as provided from up_head.S. This is the
 * first address in DRAM after the loaded program+bss+idle stack.  The
 * end of the heap is CONFIG_RAM_END
 */

extern uintptr_t        __USTACK0_END[];
extern uintptr_t        __USTACK0[];
#define g_idle_topstack __USTACK0

/* Address of the saved user stack pointer */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
extern uintptr_t        __ISTACK0_END[];
extern uintptr_t        __ISTACK0[];
#define g_intstackalloc __ISTACK0_END
#define g_intstacktop   __ISTACK0
#endif

/* These symbols are setup by the linker script. */

extern uintptr_t        _lc_gb_data[]; /* Start of .data */
extern uintptr_t        _lc_ge_data[]; /* End+1 of .data */
#define _sdata          _lc_gb_data
#define _edata          _lc_ge_data
#define _eheap          __USTACK0_END
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#define tricore_savecontext(regs)    (regs = (uintptr_t *)CURRENT_REGS)
#define tricore_restorecontext(regs) (CURRENT_REGS = regs)

/* Macros to handle saving and restoring interrupt state. */

#define tricore_savestate(regs)    (regs = (uintptr_t *)CURRENT_REGS)
#define tricore_restorestate(regs) (CURRENT_REGS = regs)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Signal handling **********************************************************/

void tricore_sigdeliver(void);

/* Exception Handler ********************************************************/

void tricore_svcall(volatile void *trap);
void tricore_trapcall(volatile void *trap);

/* Context Save Areas *******************************************************/

uintptr_t *tricore_alloc_csa(uintptr_t pc, uintptr_t sp,
                             uintptr_t psw, bool irqsave);
void tricore_reclaim_csa(uintptr_t pcxi);

/* Low level serial output **************************************************/

void tricore_lowputc(char ch);
void tricore_lowputs(const char *str);

#ifdef USE_SERIALDRIVER
void tricore_serialinit(void);
#endif

#ifdef USE_EARLYSERIALINIT
void tricore_earlyserialinit(void);
#endif

/* System Timer *************************************************************/

struct oneshot_lowerhalf_s *
tricore_systimer_initialize(void *tbase, int irq, uint64_t freq);

/* Debug ********************************************************************/

#ifdef CONFIG_STACK_COLORATION
size_t tricore_stack_check(uintptr_t alloc, size_t size);
void tricore_stack_color(void *stackbase, size_t nbytes);
#endif

#endif /* __ARCH_TRICORE_SRC_COMMON_TRICORE_INTERNAL_H */
