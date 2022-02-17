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

#ifndef __ARCH_RISCV_SRC_COMMON_RISCV_INTERNAL_H
#define __ARCH_RISCV_SRC_COMMON_RISCV_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/compiler.h>
#  include <nuttx/arch.h>
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

/* Format output with register width and hex */

#ifdef CONFIG_ARCH_RV32
#  define PRIxREG "08" PRIxPTR
#else
#  define PRIxREG "016" PRIxPTR
#endif

/* In the RISC_V model, the state is copied from the stack to the TCB, but
 * only a referenced is passed to get the state from the TCB.
 */

#define riscv_savestate(regs)    riscv_copystate(regs, (uintptr_t*)CURRENT_REGS)
#define riscv_restorestate(regs) (CURRENT_REGS = regs)

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

/* Return values from riscv_check_pmp_access */

#define PMP_ACCESS_OFF      (0)     /* Access for area not set */
#define PMP_ACCESS_DENIED   (-1)    /* Access set and denied */
#define PMP_ACCESS_FULL     (1)     /* Access set and allowed */

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
EXTERN volatile uintptr_t *g_current_regs[CONFIG_SMP_NCPUS];
#define CURRENT_REGS (g_current_regs[up_cpu_index()])
EXTERN uintptr_t g_idle_topstack;

/* Address of per-cpu idle stack base */

EXTERN const uint8_t * const g_cpu_basestack[CONFIG_SMP_NCPUS];

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
 *  - The declaration extern uint32_t _sdata; makes C happy.  C will believe
 *    that the value _sdata is the address of a uint32_t variable _data (it
 *    is not!).
 *  - We can recover the linker value then by simply taking the address of
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

void riscv_copystate(uintptr_t *dest, uintptr_t *src);
void riscv_copyfullstate(uintptr_t *dest, uintptr_t *src);

void riscv_sigdeliver(void);
int riscv_swint(int irq, void *context, void *arg);
uint32_t riscv_get_newintctx(void);

#ifdef CONFIG_ARCH_FPU
void riscv_savefpu(uintptr_t *regs);
void riscv_restorefpu(const uintptr_t *regs);
#else
#  define riscv_savefpu(regs)
#  define riscv_restorefpu(regs)
#endif

/* RISC-V PMP Config ********************************************************/

int riscv_config_pmp_region(uintptr_t region, uintptr_t attr,
                            uintptr_t base, uintptr_t size);

int riscv_check_pmp_access(uintptr_t attr, uintptr_t base, uintptr_t size);
int riscv_configured_pmp_regions(void);
int riscv_next_free_pmp_region(void);

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

void riscv_fault(int irq, uintptr_t *regs);
void riscv_exception(uintptr_t mcause, uintptr_t *regs);

/* Debug ********************************************************************/

#ifdef CONFIG_STACK_COLORATION
void riscv_stack_color(void *stackbase, size_t nbytes);
#endif

#ifdef CONFIG_SMP
void riscv_cpu_boot(int cpu);
int riscv_pause_handler(int irq, void *c, void *arg);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_RISCV_SRC_COMMON_RISCV_INTERNAL_H */
