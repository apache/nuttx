/****************************************************************************
 * arch/ceva/src/common/ceva_internal.h
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

#ifndef __ARCH_CEVA_SRC_COMMON_UP_INTERNAL_H
#define __ARCH_CEVA_SRC_COMMON_UP_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/arch.h>
#  include <sys/types.h>
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Determine which (if any) console driver to use.  If a console is enabled
 * and no other console device is specified, then a serial console is
 * assumed.
 */

#if !defined(CONFIG_DEV_CONSOLE) || CONFIG_NFILE_DESCRIPTORS == 0
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

/* Stack alignment macros */

#define STACK_ALIGN_MASK    (sizeof(uint32_t) - 1)
#define STACK_ALIGN_DOWN(a) ((a) & ~STACK_ALIGN_MASK)
#define STACK_ALIGN_UP(a)   (((a) + STACK_ALIGN_MASK) & ~STACK_ALIGN_MASK)

/* Linker defined section addresses */

#define _START_TEXT    _stext
#define _END_TEXT      _etext
#define _START_BSS     _sbss
#define _END_BSS       _ebss
#define _DATA_INIT     _eronly
#define _START_DATA    _sdata
#define _END_DATA      _edata
#define _START_HEAP    (_ebss + CONFIG_IDLETHREAD_STACKSIZE)
#define _END_HEAP      _eheap
#define _END_MEM       ((char *)~0)

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

/* This is the beginning of heap as provided from up_head.S.
 * This is the first address in DRAM after the loaded
 * program+bss+idle stack.  The end of the heap is
 * CONFIG_RAM_END
 */

EXTERN void *g_idle_basestack;
EXTERN void *g_idle_topstack;

/* Address of the interrupt stack pointer */

EXTERN uint8_t g_intstackalloc[]; /* Allocated stack base */
EXTERN uint8_t g_intstackbase[];  /* Initial top of interrupt stack */

/* These symbols are setup by the linker script. */

/* Start of .text */

EXTERN const uint8_t _stext[];
EXTERN const uint8_t _stext2[];
EXTERN const uint8_t _stext3[];
EXTERN const uint8_t _stext4[];

/* End+1 of .text */

EXTERN const uint8_t _etext[];
EXTERN const uint8_t _etext2[];
EXTERN const uint8_t _etext3[];
EXTERN const uint8_t _etext4[];

/* End+1 of read only section (.text + .rodata) */

EXTERN const uint8_t _eronly[];
EXTERN const uint8_t _eronly2[];
EXTERN const uint8_t _eronly3[];
EXTERN const uint8_t _eronly4[];

/* Start of .data */

EXTERN uint8_t _sdata[];
EXTERN uint8_t _sdata2[];
EXTERN uint8_t _sdata3[];
EXTERN uint8_t _sdata4[];

/* End+1 of .data */

EXTERN uint8_t _edata[];
EXTERN uint8_t _edata2[];
EXTERN uint8_t _edata3[];
EXTERN uint8_t _edata4[];

/* Start of .bss */

EXTERN uint8_t _sbss[];
EXTERN uint8_t _sbss2[];
EXTERN uint8_t _sbss3[];
EXTERN uint8_t _sbss4[];

/* End+1 of .bss */

EXTERN uint8_t _ebss[];
EXTERN uint8_t _ebss2[];
EXTERN uint8_t _ebss3[];
EXTERN uint8_t _ebss4[];

/* End+1 of the memory */

EXTERN uint8_t _eheap[];
EXTERN uint8_t _eheap2[];
EXTERN uint8_t _eheap3[];
EXTERN uint8_t _eheap4[];

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
/* Atomic modification of registers */

void modifyreg8(unsigned int addr, uint8_t clearbits, uint8_t setbits);
void modifyreg16(unsigned int addr, uint16_t clearbits, uint16_t setbits);
void modifyreg32(unsigned int addr, uint32_t clearbits, uint32_t setbits);

/* Context switching */

void ceva_fullcontextrestore(uint32_t *restoreregs) noreturn_function;
void ceva_switchcontext(uint32_t **saveregs, uint32_t *restoreregs);

/* Signal handling **********************************************************/

void ceva_sigdeliver(void);

/* Arch specific ************************************************************/

void ceva_earlyinitialize(void);
void ceva_lateinitialize(void);
void ceva_finalinitialize(void);

/* Power management *********************************************************/

#ifdef CONFIG_PM
void ceva_pminitialize(void);
#else
#  define ceva_pminitialize()
#endif

void ceva_reset(void);

void ceva_cpu_doze(void);
void ceva_cpu_idle(void);
void ceva_cpu_standby(void);
void ceva_cpu_sleep(void);
void ceva_cpu_normal(void);

/* Interrupt handling *******************************************************/

/* Interrupt acknowledge and dispatch */

uint32_t *ceva_doirq(int irq, uint32_t *regs);

/* Exception Handlers */

int  ceva_svcall(int irq, void *context, void *arg);
int  ceva_hardfault(int irq, void *context, void *arg);

void ceva_svcall_handler(void);

/* Low level serial output **************************************************/

#ifdef USE_SERIALDRIVER
void ceva_serialinit(void);
#else
#  define ceva_serialinit()
#endif

#ifdef USE_EARLYSERIALINIT
void ceva_earlyserialinit(void);
#else
#  define ceva_earlyserialinit()
#endif

/* DMA **********************************************************************/

#ifdef CONFIG_ARCH_DMA
void ceva_dma_initialize(void);
#endif

/* Memory management ********************************************************/

#if CONFIG_MM_REGIONS > 1
void ceva_addregion(void);
#else
# define ceva_addregion()
#endif

/* Networking ***************************************************************/

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
void ceva_netinitialize(void);
#else
# define ceva_netinitialize()
#endif

/* USB **********************************************************************/

#ifdef CONFIG_USBDEV
void ceva_usbinitialize(void);
void ceva_usbuninitialize(void);
#else
# define ceva_usbinitialize()
# define ceva_usbuninitialize()
#endif

/* Debug ********************************************************************/
#ifdef CONFIG_HEAP_COLORATION
#  define ceva_heap_color(start, size) memset(start, HEAP_COLOR, size)
#else
#  define ceva_heap_color(start, size)
#endif

#ifdef CONFIG_STACK_COLORATION
size_t ceva_stack_check(uintptr_t alloc, size_t size);
void ceva_stack_color(void *stackbase, size_t nbytes);
#endif

void ceva_registerdump(volatile uint32_t *regs);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_CEVA_SRC_COMMON_UP_INTERNAL_H */
