/****************************************************************************
 * arch/arm64/src/common/arm64_internal.h
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

#ifndef __ARCH_ARM64_SRC_COMMON_ARM64_INTERNAL_H
#define __ARCH_ARM64_SRC_COMMON_ARM64_INTERNAL_H

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

#include "arm64_arch.h"

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

/* Check if an interrupt stack size is configured */

#ifndef CONFIG_ARCH_INTERRUPTSTACK
#  define CONFIG_ARCH_INTERRUPTSTACK 0
#endif

/* If the floating point unit is present and enabled, then save the
 * floating point registers as well as normal ARM registers.
 */

#define arm64_savestate(regs) (regs = (uint64_t *)CURRENT_REGS)
#define arm64_restorestate(regs) (CURRENT_REGS = regs)

/* This is the value used to mark the stack for subsequent stack monitoring
 * logic.
 */

#define STACK_COLOR    0xdeaddead
#define HEAP_COLOR     'h'

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
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* AArch64 the stack-pointer must be 128-bit aligned */

#define STACK_ALIGNMENT     16

/* Stack alignment macros */

#define STACK_ALIGN_MASK    (STACK_ALIGNMENT - 1)
#define STACK_ALIGN_DOWN(a) ((a) & ~STACK_ALIGN_MASK)
#define STACK_ALIGN_UP(a)   (((a) + STACK_ALIGN_MASK) & ~STACK_ALIGN_MASK)

#define INIT_STACK_DEFINE(sym, size) \
    char locate_data(".initstack") \
     aligned_data(STACK_ALIGNMENT) sym[size]

#define INIT_STACK_ARRAY_DEFINE(sym, n, size) \
    char locate_data(".initstack") \
     aligned_data(STACK_ALIGNMENT) sym[n][size]

#define INIT_STACK_DEFINE_EXTERN(sym, size) \
    EXTERN char sym[size]

#define INIT_STACK_ARRAY_DEFINE_EXTERN(sym, n, size) \
    EXTERN char sym[n][size]

#define STACK_PTR_TO_FRAME(type, ptr) \
    (type *)((uintptr_t)(ptr) - sizeof(type))

#define INTSTACK_SIZE        (CONFIG_ARCH_INTERRUPTSTACK & ~STACK_ALIGN_MASK)

#ifdef CONFIG_SMP

/* The size of interrupt and idle stack.  This is the configured
 * value aligned the 8-bytes as required by the ARM EABI.
 */

#define SMP_STACK_SIZE       STACK_ALIGN_UP(CONFIG_IDLETHREAD_STACKSIZE)

INIT_STACK_ARRAY_DEFINE_EXTERN(g_cpu_idlestackalloc, CONFIG_SMP_NCPUS,
                          SMP_STACK_SIZE);
INIT_STACK_ARRAY_DEFINE_EXTERN(g_interrupt_stacks, CONFIG_SMP_NCPUS,
                          INTSTACK_SIZE);
uintptr_t arm64_intstack_alloc(void);
uintptr_t arm64_intstack_top(void);
#else
/* idle thread stack for primary core */

INIT_STACK_DEFINE_EXTERN(g_idle_stack, CONFIG_IDLETHREAD_STACKSIZE);
INIT_STACK_DEFINE_EXTERN(g_interrupt_stack, INTSTACK_SIZE);
#endif

/* This is the beginning of heap as provided from arm64_head.S.
 * This is the first address in DRAM after the loaded
 * program + bss + idle stack.  The end of the heap is
 * CONFIG_RAM_END
 */

/* Address of the saved user stack pointer */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
EXTERN uint64_t g_intstackalloc; /* Allocated stack base */
EXTERN uint64_t g_intstacktop;   /* Initial top of interrupt stack */
#else
#  error CONFIG_ARCH_INTERRUPTSTACK must be defined (4096 at least) at arm64
#endif

/* These 'addresses' of these values are setup by the linker script.  They
 * are not actual uint64_t storage locations! They are only used
 * meaningfully in the following way:
 *
 *  - The linker script defines, for example, the symbol_sdata.
 *  - The declaration extern uint64_t _sdata; makes C happy.  C will believe
 *    that the value _sdata is the address of a uint64_t variable _data
 *    (it is not!).
 *  - We can recover the linker value then by simply taking the address of
 *    of _data.  like:  uint64_t *pdata = &_sdata;
 *
 * Memory layout for Nuttx at arm64 for FLAT Build
 *
 *    +-------------------------+ <- RAM BASE
 *    |                         |
 *    |<<<<<<<<<<<<<<<<<<<<<<<<<| <- LOAD_BASE
 *    |   text(code) section    |
 *    |   vector table          |
 *    +-------------------------+-- page align(4096)
 *    |   rodata section        |
 *    +-------------------------+-- page align(4096)
 *    |   data/bss section      |
 *    +-------------------------+-- page align(4096)
 *    |   initstack section     |
 *    +-------------------------+-- page align(4096)
 *    |   heap alloc area       |
 *    |                         |
 *    |                         |
 *    |                         |
 *    +-------------------------+
 *
 * Note:
 *  1. initstack is for idle_thread and interrupt stack,
 *    it has dedicated stack for IRQ handler in arm64
 *  2. Every section with different memory attribute,
 *    please check mmu_nxrt_regions at arm64_mmu.c
 *
 * please check dramboot.ld at specified platform for more detail
 */

EXTERN char _stext[];            /* Start of .text */
EXTERN char _etext[];            /* End of .text */
EXTERN char _sztext[];           /* Size of .text */
EXTERN char _srodata[];          /* Start of .rodata */
EXTERN char _erodata[];          /* End+1 of .rodata */
EXTERN char _szrodata[];         /* Size of .rodata */
EXTERN const char _eronly[];     /* End+1 of read only section (.text + .rodata) */
EXTERN char _sdata[];            /* Start of .data */
EXTERN char _edata[];            /* End+1 of .data */
EXTERN char _sbss[];             /* Start of .bss */
EXTERN char _ebss[];             /* End+1 of .bss */
EXTERN char _szdata[];           /* Size of data(.data + .bss) */
EXTERN char _e_initstack[];      /* End+1 of .initstack */
EXTERN char g_idle_topstack[];   /* End+1 of heap */

#  define _START_TEXT  _stext
#  define _END_TEXT    _etext
#  define _START_BSS   _sbss
#  define _END_BSS     _ebss
#  define _DATA_INIT   _eronly
#  define _START_DATA  _sdata
#  define _END_DATA    _edata

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void arm64_new_task(struct tcb_s *tak_new);

/* Low level initialization provided by chip logic */

void arm64_chip_boot(void);

int arm64_psci_init(const char *method);

void __start(void);
void arm64_secondary_start(void);

/* Context switching */

void arm64_fullcontextrestore(uint64_t *restoreregs) noreturn_function;
void arm64_switchcontext(uint64_t **saveregs, uint64_t *restoreregs);
void arm64_context_snapshot(void *savereg);

/* Signal handling **********************************************************/

void arm64_sigdeliver(void);

/* Power management *********************************************************/

#ifdef CONFIG_PM
void arm64_pminitialize(void);
#else
#  define arm64_pminitialize()
#endif

/* Interrupt handling */

/* Exception handling logic unique to the Cortex-A and Cortex-R families
 * (but should be back-ported to the ARM7 and ARM9 families).
 */

/* Interrupt acknowledge and dispatch */

uint64_t *arm64_doirq(int irq, uint64_t *regs);

/* Paging support */

#ifdef CONFIG_PAGING
void arm64_pginitialize(void);
#else /* CONFIG_PAGING */
# define arm64_pginitialize()
#endif /* CONFIG_PAGING */

uint64_t * arm64_syscall_switch(uint64_t *regs);
int arm64_syscall(uint64_t *regs);

#ifdef USE_SERIALDRIVER
void arm64_serialinit(void);
#endif

#ifdef USE_EARLYSERIALINIT
void arm64_earlyserialinit(void);
#endif

/* DMA */

#ifdef CONFIG_ARCH_DMA
void weak_function arm64_dma_initialize(void);
#endif

/* Memory management */

#if CONFIG_MM_REGIONS > 1
void arm64_addregion(void);
#else
# define arm64_addregion()
#endif

/* Networking */

/* Defined in board/xyz_network.c for board-specific Ethernet
 * implementations, or chip/xyx_ethernet.c for chip-specific Ethernet
 * implementations, or common/arm_etherstub.c for a corner case where the
 * network is enabled yet there is no Ethernet driver to be initialized.
 *
 * Use of common/arm_etherstub.c is deprecated.  The preferred mechanism is
 * to use CONFIG_NETDEV_LATEINIT=y to suppress the call to
 * arm_netinitialize() in up_initialize().  Then this stub would not be
 * needed.
 */

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
void arm64_netinitialize(void);
#else
# define arm64_netinitialize()
#endif

/* USB */

#ifdef CONFIG_USBDEV
void arm64_usbinitialize(void);
void arm64_usbuninitialize(void);
#else
# define arm64_usbinitialize()
# define arm64_usbuninitialize()
#endif

/* Debug */

#ifdef CONFIG_STACK_COLORATION
size_t arm64_stack_check(void *stackbase, size_t nbytes);
void arm64_stack_color(void *stackbase, size_t nbytes);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_SRC_COMMON_ARM64_INTERNAL_H */
