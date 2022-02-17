/****************************************************************************
 * arch/ceva/src/common/up_internal.h
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

/* Linker defined section addresses */

#define _START_TEXT    ((const void *)&_stext)
#define _END_TEXT      ((const void *)&_etext)
#define _START_BSS     ((void *)&_sbss)
#define _END_BSS       ((void *)&_ebss)
#define _DATA_INIT     ((const void *)&_eronly)
#define _START_DATA    ((void *)&_sdata)
#define _END_DATA      ((void *)&_edata)
#define _START_HEAP    ((void *)&_ebss + CONFIG_IDLETHREAD_STACKSIZE)
#define _END_HEAP      ((void *)&_eheap)
#define _END_MEM       ((void *)~0)

/* This is the value used to mark the stack for subsequent stack monitoring
 * logic.
 */

#define STACK_COLOR    0xdeadbeef
#define INTSTACK_COLOR 0xdeadbeef
#define HEAP_COLOR     'h'

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

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

/* For the case of architectures with multiple CPUs, then there must be one
 * such value for each processor that can receive an interrupt.
 */

EXTERN uint32_t *volatile g_current_regs[CONFIG_SMP_NCPUS];
#define CURRENT_REGS (g_current_regs[up_cpu_index()])

/* This is the beginning of heap as provided from up_head.S.
 * This is the first address in DRAM after the loaded
 * program+bss+idle stack.  The end of the heap is
 * CONFIG_RAM_END
 */

EXTERN void *g_idle_basestack;
EXTERN void *g_idle_topstack;

/* Address of the interrupt stack pointer */

EXTERN char g_intstackalloc; /* Allocated stack base */
EXTERN char g_intstackbase;  /* Initial top of interrupt stack */

/* These 'addresses' of these values are setup by the linker script.
 * They are not actual char storage locations! They are only used
 * meaningfully in the following way:
 *
 *  - The linker script defines, for example, the symbol _sdata.
 *  - The declareion extern char _sdata; makes C happy.  C will believe
 *    that the value _sdata is the address of a char variable _data (it is
 *    not!).
 *  - We can recover the linker value then by simply taking the address of
 *    of _data.  like:  char *pdata = &_sdata;
 */

/* Start of .text */

EXTERN const char _stext;
EXTERN const char _stext2;
EXTERN const char _stext3;
EXTERN const char _stext4;

/* End+1 of .text */

EXTERN const char _etext;
EXTERN const char _etext2;
EXTERN const char _etext3;
EXTERN const char _etext4;

/* End+1 of read only section (.text + .rodata) */

EXTERN const char _eronly;
EXTERN const char _eronly2;
EXTERN const char _eronly3;
EXTERN const char _eronly4;

/* Start of .data */

EXTERN char _sdata;
EXTERN char _sdata2;
EXTERN char _sdata3;
EXTERN char _sdata4;

/* End+1 of .data */

EXTERN char _edata;
EXTERN char _edata2;
EXTERN char _edata3;
EXTERN char _edata4;

/* Start of .bss */

EXTERN char _sbss;
EXTERN char _sbss2;
EXTERN char _sbss3;
EXTERN char _sbss4;

/* End+1 of .bss */

EXTERN char _ebss;
EXTERN char _ebss2;
EXTERN char _ebss3;
EXTERN char _ebss4;

/* End+1 of the memory */

EXTERN char _eheap;
EXTERN char _eheap2;
EXTERN char _eheap3;
EXTERN char _eheap4;

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Context switching */

int  up_saveusercontext(uint32_t *saveregs);
void up_fullcontextrestore(uint32_t *restoreregs) noreturn_function;
void up_switchcontext(uint32_t **saveregs, uint32_t *restoreregs);

/* Signal handling **********************************************************/

void up_sigdeliver(void);

/* Arch specific ************************************************************/

void up_earlyinitialize(void);
void up_lateinitialize(void);
void up_finalinitialize(void);

/* Power management *********************************************************/

#ifdef CONFIG_PM
void up_pminitialize(void);
#else
#  define up_pminitialize()
#endif

void up_reset(void);

void up_cpu_doze(void);
void up_cpu_idle(void);
void up_cpu_standby(void);
void up_cpu_sleep(void);
void up_cpu_normal(void);

/* Interrupt handling *******************************************************/

void up_irqinitialize(void);

/* Interrupt acknowledge and dispatch */

uint32_t *up_doirq(int irq, uint32_t *regs);

/* Exception Handlers */

int  up_svcall(int irq, FAR void *context, FAR void *arg);
int  up_hardfault(int irq, FAR void *context, FAR void *arg);

void up_svcall_handler(void);

/* System timer *************************************************************/

void up_timer_initialize(void);

/* Low level serial output **************************************************/

#ifdef USE_SERIALDRIVER
void up_serialinit(void);
#else
#  define up_serialinit()
#endif

#ifdef USE_EARLYSERIALINIT
void up_earlyserialinit(void);
#else
#  define up_earlyserialinit()
#endif

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void);
#else
#  define rpmsg_serialinit()
#endif

/* Defined in drivers/lowconsole.c */

#ifdef CONFIG_DEV_LOWCONSOLE
void lowconsole_init(void);
#else
#  define lowconsole_init()
#endif

/* DMA **********************************************************************/

#ifdef CONFIG_ARCH_DMA
void up_dma_initialize(void);
#endif

/* Memory management ********************************************************/

#if CONFIG_MM_REGIONS > 1
void up_addregion(void);
#else
# define up_addregion()
#endif

/* Watchdog timer ***********************************************************/

void up_wdtinit(void);

/* Networking ***************************************************************/

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
void up_netinitialize(void);
#else
# define up_netinitialize()
#endif

/* USB **********************************************************************/

#ifdef CONFIG_USBDEV
void up_usbinitialize(void);
void up_usbuninitialize(void);
#else
# define up_usbinitialize()
# define up_usbuninitialize()
#endif

/* Debug ********************************************************************/
#ifdef CONFIG_HEAP_COLORATION
#  define up_heap_color(start, size) memset(start, HEAP_COLOR, size)
#else
#  define up_heap_color(start, size)
#endif

#ifdef CONFIG_STACK_COLORATION
void up_stack_color(FAR void *stackbase, size_t nbytes);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_CEVA_SRC_COMMON_UP_INTERNAL_H */
