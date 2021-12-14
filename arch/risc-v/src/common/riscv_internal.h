/****************************************************************************
 * arch/risc-v/src/common/riscv_internal.h
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
#define riscv_savestate(regs)    riscv_copystate(regs, (uint64_t*)CURRENT_REGS)
#define riscv_restorestate(regs) (CURRENT_REGS = regs)
#else
#define riscv_savestate(regs)    riscv_copystate(regs, (uint32_t*)g_current_regs)
#define riscv_restorestate(regs) (g_current_regs = regs)
#endif

#define _START_TEXT  &_stext
#define _END_TEXT    &_etext
#define _START_BSS   &_sbss
#define _END_BSS     &_ebss
#define _DATA_INIT   &_eronly
#define _START_DATA  &_sdata
#define _END_DATA    &_edata
#define _START_TDATA &_stdata
#define _END_TDATA   &_etdata
#define _START_TBSS  &_stbss
#define _END_TBSS    &_etbss

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
#  define CURRENT_REGS (g_current_regs)
EXTERN uint32_t g_idle_topstack;
#endif

/* Address of the saved user stack pointer */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
EXTERN uint32_t g_intstackalloc; /* Allocated stack base */
EXTERN uint32_t g_intstacktop;   /* Initial top of interrupt stack */
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
EXTERN uint32_t _stdata;          /* Start of .tdata */
EXTERN uint32_t _etdata;          /* End+1 of .tdata */
EXTERN uint32_t _stbss;           /* Start of .tbss */
EXTERN uint32_t _etbss;           /* End+1 of .tbss */

#endif /* __ASSEMBLY__ */

/****************************************************************************
* Public Function Prototypes
  ***************************************************************************/

#ifndef __ASSEMBLY__

/* Memory allocation ********************************************************/

#if CONFIG_MM_REGIONS > 1
void riscv_addregion(void);
#else
# define riscv_addregion()
#endif

/* IRQ initialization *******************************************************/

void riscv_ack_irq(int irq);

#ifdef CONFIG_ARCH_RV64GC
void riscv_copystate(uint64_t *dest, uint64_t *src);
void riscv_copyfullstate(uint64_t *dest, uint64_t *src);
#else
void riscv_copystate(uint32_t *dest, uint32_t *src);
void riscv_copyfullstate(uint32_t *dest, uint32_t *src);
#endif

void riscv_sigdeliver(void);
int riscv_swint(int irq, void *context, void *arg);
uint32_t riscv_get_newintctx(void);

#ifdef CONFIG_ARCH_FPU
#ifdef CONFIG_ARCH_RV64GC
void riscv_savefpu(uint64_t *regs);
void riscv_restorefpu(const uint64_t *regs);
#else /* !CONFIG_ARCH_RV64GC */
void riscv_savefpu(uint32_t *regs);
void riscv_restorefpu(const uint32_t *regs);
#endif /* CONFIG_ARCH_RV64GC */
#else
#  define riscv_savefpu(regs)
#  define riscv_restorefpu(regs)
#endif

/* RISC-V PMP Config ********************************************************/

void riscv_config_pmp_region(uintptr_t region, uintptr_t attr,
                             uintptr_t base, uintptr_t size);

/* Power management *********************************************************/

#ifdef CONFIG_PM
void riscv_pminitialize(void);
#else
#  define riscv_pminitialize()
#endif

/* Low level serial output **************************************************/

void riscv_lowputc(char ch);
void riscv_lowputs(const char *str);

#ifdef USE_SERIALDRIVER
void riscv_serialinit(void);
#endif

#ifdef USE_EARLYSERIALINIT
void riscv_earlyserialinit(void);
#endif

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void);
#endif

/* Exception Handler ********************************************************/

void riscv_exception(uint32_t mcause, uint32_t *regs);

/* Debug ********************************************************************/

#ifdef CONFIG_STACK_COLORATION
void riscv_stack_color(void *stackbase, size_t nbytes);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_RISCV_SRC_COMMON_UP_INTERNAL_H */
