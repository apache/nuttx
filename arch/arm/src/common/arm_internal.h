/****************************************************************************
 * arch/arm/src/common/arm_internal.h
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

#ifndef __ARCH_ARM_SRC_COMMON_ARM_INTERNAL_H
#define __ARCH_ARM_SRC_COMMON_ARM_INTERNAL_H

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

/* Macros to handle saving and restoring interrupt state. */

#define arm_savestate(regs)    (regs = (uint32_t *)CURRENT_REGS)
#define arm_restorestate(regs) (CURRENT_REGS = regs)

/* Toolchain dependent, linker defined section addresses */

#if defined(__ICCARM__)
#  define _START_TEXT  __sfb(".text")
#  define _END_TEXT    __sfe(".text")
#  define _START_BSS   __sfb(".bss")
#  define _END_BSS     __sfe(".bss")
#  define _DATA_INIT   __sfb(".data_init")
#  define _START_DATA  __sfb(".data")
#  define _END_DATA    __sfe(".data")
#else
#  define _START_TEXT  &_stext
#  define _END_TEXT    &_etext
#  define _START_BSS   &_sbss
#  define _END_BSS     &_ebss
#  define _DATA_INIT   &_eronly
#  define _START_DATA  &_sdata
#  define _END_DATA    &_edata
#  define _START_TDATA &_stdata
#  define _END_TDATA   &_etdata
#  define _START_TBSS  &_stbss
#  define _END_TBSS    &_etbss
#endif

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

/* Non-atomic, but more effective modification of registers */

#define modreg8(v,m,a)  putreg8((getreg8(a) & ~(m)) | ((v) & (m)), (a))
#define modreg16(v,m,a) putreg16((getreg16(a) & ~(m)) | ((v) & (m)), (a))
#define modreg32(v,m,a) putreg32((getreg32(a) & ~(m)) | ((v) & (m)), (a))

/* Context switching */

#define arm_fullcontextrestore(restoreregs) \
  sys_call1(SYS_restore_context, (uintptr_t)restoreregs);

#define arm_switchcontext(saveregs, restoreregs) \
  sys_call2(SYS_switch_context, (uintptr_t)saveregs, (uintptr_t)restoreregs);

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

/* This is the beginning of heap as provided from arm_head.S.
 * This is the first address in DRAM after the loaded
 * program+bss+idle stack.  The end of the heap is
 * CONFIG_RAM_END
 */

EXTERN const uintptr_t g_idle_topstack;

/* Address of the saved user stack pointer */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
EXTERN uint32_t g_intstackalloc; /* Allocated stack base */
EXTERN uint32_t g_intstacktop;   /* Initial top of interrupt stack */
#endif

/* These 'addresses' of these values are setup by the linker script.  They
 * are not actual uint32_t storage locations! They are only used
 * meaningfully in the following way:
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

/* Sometimes, functions must be executed from RAM.  In this case, the
 * following macro may be used (with GCC!) to specify a function that will
 * execute from RAM.  For example,
 *
 *   int __ramfunc__ foo (void);
 *   int __ramfunc__ foo (void) { return bar; }
 *
 * will create a function named foo that will execute from RAM.
 */

#ifdef CONFIG_ARCH_RAMFUNCS

#  define __ramfunc__ locate_code(".ramfunc") farcall_function noinline_function

/* Functions declared in the .ramfunc section will be packaged together
 * by the linker script and stored in FLASH.  During boot-up, the start
 * logic must include logic to copy the RAM functions from their storage
 * location in FLASH to their correct destination in SRAM.  The following
 * following linker-defined values provide the information to copy the
 * functions from flash to RAM.
 */

EXTERN const uint32_t _framfuncs; /* Copy source address in FLASH */
EXTERN uint32_t _sramfuncs;       /* Copy destination start address in RAM */
EXTERN uint32_t _eramfuncs;       /* Copy destination end address in RAM */

#else /* CONFIG_ARCH_RAMFUNCS */

/* Otherwise, a null definition is provided so that condition compilation is
 * not necessary in code that may operate with or without RAM functions.
 */

#  define __ramfunc__

#endif /* CONFIG_ARCH_RAMFUNCS */
#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
/* Atomic modification of registers */

void modifyreg8(unsigned int addr, uint8_t clearbits, uint8_t setbits);
void modifyreg16(unsigned int addr, uint16_t clearbits, uint16_t setbits);
void modifyreg32(unsigned int addr, uint32_t clearbits, uint32_t setbits);

/* Low level initialization provided by board-level logic *******************/

void arm_boot(void);

/* Context switching */

uint32_t *arm_decodeirq(uint32_t *regs);

/* Signal handling **********************************************************/

void arm_sigdeliver(void);

/* Power management *********************************************************/

#ifdef CONFIG_PM
void arm_pminitialize(void);
#else
#  define arm_pminitialize()
#endif

/* Interrupt handling *******************************************************/

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
uintptr_t arm_intstack_alloc(void);
uintptr_t arm_intstack_top(void);
#endif

/* Exception handling logic unique to the Cortex-M family */

#if defined(CONFIG_ARCH_ARMV6M) || defined(CONFIG_ARCH_ARMV7M) || \
    defined(CONFIG_ARCH_ARMV8M)

/* Interrupt acknowledge and dispatch */

void arm_ack_irq(int irq);
uint32_t *arm_doirq(int irq, uint32_t *regs);

/* Exception Handlers */

int  arm_svcall(int irq, void *context, void *arg);
int  arm_hardfault(int irq, void *context, void *arg);

#  if defined(CONFIG_ARCH_ARMV7M) || defined(CONFIG_ARCH_ARMV8M)

int  arm_memfault(int irq, void *context, void *arg);
int  arm_busfault(int irq, void *context, void *arg);
int  arm_usagefault(int irq, void *context, void *arg);
int  arm_securefault(int irq, void *context, void *arg);

#  endif /* CONFIG_ARCH_CORTEXM3,4,7 */

/* Exception handling logic unique to the Cortex-A and Cortex-R families
* (but should be back-ported to the ARM7 and ARM9 families).
 */

#elif defined(CONFIG_ARCH_ARMV7A) || defined(CONFIG_ARCH_ARMV7R)

/* Interrupt acknowledge and dispatch */

uint32_t *arm_doirq(int irq, uint32_t *regs);

/* Paging support */

#ifdef CONFIG_PAGING
void arm_pginitialize(void);
uint32_t *arm_va2pte(uintptr_t vaddr);
#else /* CONFIG_PAGING */
# define arm_pginitialize()
#endif /* CONFIG_PAGING */

/* Exception Handlers */

uint32_t *arm_dataabort(uint32_t *regs, uint32_t dfar, uint32_t dfsr);
uint32_t *arm_prefetchabort(uint32_t *regs, uint32_t ifar, uint32_t ifsr);
uint32_t *arm_syscall(uint32_t *regs);
uint32_t *arm_undefinedinsn(uint32_t *regs);

/* Exception handling logic common to other ARM7 and ARM9 family. */

#else /* ARM7 | ARM9 */

/* Interrupt acknowledge and dispatch */

void arm_ack_irq(int irq);
void arm_doirq(int irq, uint32_t *regs);

/* Paging support (and exception handlers) */

#ifdef CONFIG_PAGING
void arm_pginitialize(void);
uint32_t *arm_va2pte(uintptr_t vaddr);
void arm_dataabort(uint32_t *regs, uint32_t far, uint32_t fsr);
#else /* CONFIG_PAGING */
# define arm_pginitialize()
void arm_dataabort(uint32_t *regs);
#endif /* CONFIG_PAGING */

/* Exception handlers */

void arm_prefetchabort(uint32_t *regs);
uint32_t *arm_syscall(uint32_t *regs);
void arm_undefinedinsn(uint32_t *regs);

#endif /* CONFIG_ARCH_ARMV[6-8]M */

void arm_vectorundefinsn(void);
void arm_vectorsvc(void);
void arm_vectorprefetch(void);
void arm_vectordata(void);
void arm_vectoraddrexcptn(void);
void arm_vectorirq(void);
void arm_vectorfiq(void);

/* Floating point unit ******************************************************/

#ifdef CONFIG_ARCH_FPU
void arm_fpuconfig(void);
#else
#  define arm_fpuconfig()
#endif

/* Low level serial output **************************************************/

void arm_lowputc(char ch);
void arm_lowputs(const char *str);

#ifdef USE_SERIALDRIVER
void arm_serialinit(void);
#endif

#ifdef USE_EARLYSERIALINIT
void arm_earlyserialinit(void);
#endif

/* DMA **********************************************************************/

#ifdef CONFIG_ARCH_DMA
void weak_function arm_dma_initialize(void);
#endif

/* Cache control ************************************************************/

#ifdef CONFIG_ARCH_L2CACHE
void arm_l2ccinitialize(void);
#else
#  define arm_l2ccinitialize()
#endif

/* Memory management ********************************************************/

#if CONFIG_MM_REGIONS > 1
void arm_addregion(void);
#else
# define arm_addregion()
#endif

/* Networking ***************************************************************/

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
void arm_netinitialize(void);
#else
# define arm_netinitialize()
#endif

/* USB **********************************************************************/

#ifdef CONFIG_USBDEV
void arm_usbinitialize(void);
void arm_usbuninitialize(void);
#else
# define arm_usbinitialize()
# define arm_usbuninitialize()
#endif

/* Debug ********************************************************************/
#ifdef CONFIG_STACK_COLORATION
size_t arm_stack_check(void *stackbase, size_t nbytes);
void arm_stack_color(void *stackbase, size_t nbytes);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_COMMON_ARM_INTERNAL_H */
