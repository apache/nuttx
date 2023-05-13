/****************************************************************************
 * arch/sparc/src/common/sparc_internal.h
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

#ifndef __ARCH_SPARC_SRC_COMMON_UP_INTERNAL_H
#define __ARCH_SPARC_SRC_COMMON_UP_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/arch.h>
#  include <sys/types.h>
#  include <stdint.h>
#  include <stdbool.h>
#endif

#  include "sparc_v8.h"

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
#  undef  CONFIG_DEV_LOWCONSOLE
#  undef  CONFIG_RAMLOG_CONSOLE
#else
#  if defined(CONFIG_RAMLOG_CONSOLE)
#    undef  USE_SERIALDRIVER
#    undef  USE_EARLYSERIALINIT
#    undef  CONFIG_DEV_LOWCONSOLE
#  elif defined(CONFIG_DEV_LOWCONSOLE)
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
# define CONFIG_ARCH_INTERRUPTSTACK 0
#endif

#define INTSTACK_SIZE (CONFIG_ARCH_INTERRUPTSTACK & ~STACK_ALIGN_MASK)

/* sparc requires at least a 4-byte stack alignment.  For floating point use,
 * however, the stack must be aligned to 8-byte addresses.
 */

#define STACK_ALIGNMENT     8

/* Stack alignment macros */

#define STACK_ALIGN_MASK    (STACK_ALIGNMENT - 1)
#define STACK_ALIGN_DOWN(a) ((a) & ~STACK_ALIGN_MASK)
#define STACK_ALIGN_UP(a)   (((a) + STACK_ALIGN_MASK) & ~STACK_ALIGN_MASK)

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
 * first address in DRAM after the loaded program+bss+idle stack.  The end
 * of the heap is CONFIG_RAM_END
 */

extern uint32_t g_idle_topstack;

/* Address of the saved user stack pointer */

#if CONFIG_ARCH_INTERRUPTSTACK > 7
extern uint8_t g_intstackalloc[]; /* Allocated stack base */
extern uint8_t g_intstacktop[];   /* Initial top of interrupt stack */
#endif

/* These symbols are setup by the linker script. */

extern char _text_start[];         /* Start of .text */
extern char _etext[];              /* End+1 of .text + .rodata */
extern const char _rodata_start[]; /* Start of .data in FLASH */
extern char _sdata[];              /* Start of .data */
extern char _edata[];              /* End+1 of .data */
extern char _bss_start[];          /* Start of .bss */
extern char _end[];                /* End+1 of .bss */
#ifdef CONFIG_ARCH_RAMFUNCS
extern char _sramfunc[];           /* Start of ramfuncs */
extern char _eramfunc[];           /* End+1 of ramfuncs */
extern char _ramfunc_loadaddr[];   /* Start of ramfuncs in FLASH */
extern char _ramfunc_sizeof[];     /* Size of ramfuncs */
extern char _bmxdkpba_address[];   /* BMX register setting */
extern char _bmxdudba_address[];   /* BMX register setting */
extern char _bmxdupba_address[];   /* BMX register setting */
#endif /* CONFIG_ARCH_RAMFUNCS */
#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Common Functions *********************************************************/

/* Common functions define in arch/sparc/src/common.  These may be replaced
 * with chip-specific functions of the same name if needed.  See also
 * functions prototyped in include/nuttx/arch.h.
 */

/* Atomic modification of registers */

void modifyreg8(unsigned int addr, uint8_t clearbits, uint8_t setbits);
void modifyreg16(unsigned int addr, uint16_t clearbits, uint16_t setbits);
void modifyreg32(unsigned int addr, uint32_t clearbits, uint32_t setbits);

/* Context switching */

void sparc_copystate(uint32_t *dest, uint32_t *src);

/* Serial output */

void sparc_lowputs(const char *str);

/* Software interrupt 0 handler */

int sparc_swint0(int irq, void *context, void *arg);

/* Software interrupt 1 handler */

int sparc_swint1(int irq, void *context, void *arg);

/* Signals */

void sparc_sigdeliver(void);

/* Interrupt handling *******************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 7
uintptr_t sparc_intstack_alloc(void);
uintptr_t sparc_intstack_top(void);
#endif

/* Chip-specific functions **************************************************/

/* Chip specific functions defined in arch/sparc/src/<chip> */

/* IRQs */

bool sparc_pending_irq(int irq);
void sparc_clrpend_irq(int irq);

/* DMA */

#ifdef CONFIG_ARCH_DMA
void weak_function sparc_dma_initialize(void);
#endif

/* Memory management */

#if CONFIG_MM_REGIONS > 1
void sparc_addregion(void);
#else
# define sparc_addregion()
#endif

/* Serial output */

void sparc_lowputc(char ch);
void sparc_earlyserialinit(void);
void sparc_serialinit(void);

/* Network */

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
void sparc_netinitialize(void);
#else
# define sparc_netinitialize()
#endif

/* USB */

#ifdef CONFIG_USBDEV
void sparc_usbinitialize(void);
void sparc_usbuninitialize(void);
#else
# define sparc_usbinitialize()
# define sparc_usbuninitialize()
#endif

/* Debug ********************************************************************/
#ifdef CONFIG_STACK_COLORATION
size_t sparc_stack_check(void *stackbase, size_t nbytes);
void sparc_stack_color(void *stackbase, size_t nbytes);
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __ARCH_SPARC_SRC_COMMON_UP_INTERNAL_H */
