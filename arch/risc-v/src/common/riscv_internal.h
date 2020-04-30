/****************************************************************************
 * arch/risc-v/src/common/riscv_internal.h
 *
 *   Copyright (C) 2007-2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Modified for RISC-V:
 *
 *   Copyright (C) 2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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

#ifndef __ARCH_RISCV_SRC_COMMON_UP_INTERNAL_H
#define __ARCH_RISCV_SRC_COMMON_UP_INTERNAL_H

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

#define STACK_COLOR    0xdeadbeef
#define INTSTACK_COLOR 0xdeadbeef
#define HEAP_COLOR     'h'

/* In the RISC_V model, the state is copied from the stack to the TCB, but
 * only a referenced is passed to get the state from the TCB.
 */

#ifdef CONFIG_ARCH_RV64GC
#define up_savestate(regs)    up_copystate(regs, (uint64_t*)CURRENT_REGS)
#define up_restorestate(regs) (CURRENT_REGS = regs)
#else
#define up_savestate(regs)    up_copystate(regs, (uint32_t*)g_current_regs)
#define up_restorestate(regs) (g_current_regs = regs)
#endif

#define _START_TEXT  &_stext
#define _END_TEXT    &_etext
#define _START_BSS   &_sbss
#define _END_BSS     &_ebss
#define _DATA_INIT   &_eronly
#define _START_DATA  &_sdata
#define _END_DATA    &_edata

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
 * Public Variables
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifndef __ASSEMBLY__
#ifdef CONFIG_ARCH_RV64GC
#ifdef CONFIG_SMP
EXTERN volatile uint64_t *g_current_regs[CONFIG_SMP_NCPUS];
#  define CURRENT_REGS (g_current_regs[up_cpu_index()])
#else
EXTERN volatile uint64_t *g_current_regs[1];
#  define CURRENT_REGS (g_current_regs[0])
#endif
EXTERN uintptr_t g_idle_topstack;
#else
EXTERN volatile uint32_t *g_current_regs;
EXTERN uint32_t g_idle_topstack;
#endif

/* Address of the saved user stack pointer */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
EXTERN uint32_t g_intstackalloc; /* Allocated stack base */
EXTERN uint32_t g_intstackbase;  /* Initial top of interrupt stack */
#endif

/* These 'addresses' of these values are setup by the linker script.  They
 * are not actual uint32_t storage locations! They are only used meaningfully
 * in the following way:
 *
 *  - The linker script defines, for example, the symbol_sdata.
 *  - The declareion extern uint32_t _sdata; makes C happy.  C will believe
 *    that the value _sdata is the address of a uint32_t variable _data (it
 *    is not!).
 *  - We can recoved the linker value then by simply taking the address of
 *    of _data.  like:  uint32_t *pdata = &_sdata;
 */

EXTERN uint32_t _stext;           /* Start of .text */
EXTERN uint32_t _etext;           /* End_1 of .text + .rodata */
EXTERN const uint32_t _eronly;    /* End+1 of read only section (.text + .rodata) */
EXTERN uint32_t _sdata;           /* Start of .data */
EXTERN uint32_t _edata;           /* End+1 of .data */
EXTERN uint32_t _sbss;            /* Start of .bss */
EXTERN uint32_t _ebss;            /* End+1 of .bss */

#endif /* __ASSEMBLY__ */

/****************************************************************************
* Public Function Prototypes
  ***************************************************************************/

#ifndef __ASSEMBLY__

/* Low level initialization provided by board-level logic *******************/

void up_boot(void);

/* Memory allocation ********************************************************/

void up_addregion(void);
void up_allocate_heap(FAR void **heap_start, size_t *heap_size);

/* IRQ initialization *******************************************************/

void up_ack_irq(int irq);

#ifdef CONFIG_ARCH_RV64GC
void up_copystate(uint64_t *dest, uint64_t *src);
#else
void up_copystate(uint32_t *dest, uint32_t *src);
#endif

void up_sigdeliver(void);
int up_swint(int irq, FAR void *context, FAR void *arg);
uint32_t up_get_newintctx(void);

#ifdef CONFIG_ARCH_FPU
void up_savefpu(uint32_t *regs);
void up_restorefpu(const uint32_t *regs);
#else
#  define up_savefpu(regs)
#  define up_restorefpu(regs)
#endif

/* Power management *********************************************************/

#ifdef CONFIG_PM
void up_pminitialize(void);
#else
#  define up_pminitialize()
#endif

/* Low level serial output **************************************************/

void up_lowputc(char ch);
void up_puts(const char *str);
void up_lowputs(const char *str);

#ifdef USE_SERIALDRIVER
void up_serialinit(void);
#endif

#ifdef USE_EARLYSERIALINIT
void up_earlyserialinit(void);
#endif

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void);
#endif

/* The OS start routine    **************************************************/

void nx_start(void);

/* Debug ********************************************************************/

#ifdef CONFIG_STACK_COLORATION
void up_stack_color(FAR void *stackbase, size_t nbytes);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_RISCV_SRC_COMMON_UP_INTERNAL_H */
