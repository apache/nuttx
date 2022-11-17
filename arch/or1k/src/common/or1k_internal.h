/****************************************************************************
 * arch/or1k/src/common/or1k_internal.h
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

#ifndef __ARCH_OR1K_SRC_COMMON_UP_INTERNAL_H
#define __ARCH_OR1K_SRC_COMMON_UP_INTERNAL_H

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

/* For use with EABI and floating point, the stack must be aligned to 8-byte
 * addresses.
 */

#define STACK_ALIGNMENT     8

/* Stack alignment macros */

#define STACK_ALIGN_MASK    (STACK_ALIGNMENT - 1)
#define STACK_ALIGN_DOWN(a) ((a) & ~STACK_ALIGN_MASK)
#define STACK_ALIGN_UP(a)   (((a) + STACK_ALIGN_MASK) & ~STACK_ALIGN_MASK)

#define or1k_savestate(regs)  or1k_copyfullstate(regs, (uint32_t*)CURRENT_REGS)
#define or1k_restorestate(regs) or1k_copyfullstate((uint32_t*)CURRENT_REGS, regs)

#define _START_TEXT  _stext
#define _END_TEXT    _etext
#define _START_BSS   _sbss
#define _END_BSS     _ebss
#define _DATA_INIT   _eronly
#define _START_DATA  _sdata
#define _END_DATA    _edata

/* This is the value used to mark the stack for subsequent stack monitoring
 * logic.
 */

#define STACK_COLOR    0x1bad1dea
#define INTSTACK_COLOR 0x1bad1dea
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

EXTERN const uint32_t g_idle_topstack;

/* Address of the saved user stack pointer */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
EXTERN uint8_t g_intstackalloc[]; /* Allocated stack base */
EXTERN uint8_t g_intstacktop[];   /* Initial top of interrupt stack */
#endif

/* These symbols are setup by the linker script. */

EXTERN uint8_t _stext[];           /* Start of .text */
EXTERN uint8_t _etext[];           /* End_1 of .text + .rodata */
EXTERN const uint8_t _eronly[];    /* End+1 of read only section (.text + .rodata) */
EXTERN uint8_t _sdata[];           /* Start of .data */
EXTERN uint8_t _edata[];           /* End+1 of .data */
EXTERN uint8_t _sbss[];            /* Start of .bss */
EXTERN uint8_t _ebss[];            /* End+1 of .bss */

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
 * Public Functions Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
/* Atomic modification of registers */

void modifyreg8(unsigned int addr, uint8_t clearbits, uint8_t setbits);
void modifyreg16(unsigned int addr, uint16_t clearbits, uint16_t setbits);
void modifyreg32(unsigned int addr, uint32_t clearbits, uint32_t setbits);

/* Low level initialization provided by board-level logic *******************/

void or1k_boot(void);
int  or1k_print_cpuinfo(void);

/* Context switching */

void or1k_copyfullstate(uint32_t *dest, uint32_t *src);
void or1k_decodeirq(uint32_t *regs);
void or1k_fullcontextrestore(uint32_t *restoreregs) noreturn_function;
void or1k_switchcontext(uint32_t *saveregs, uint32_t *restoreregs);

/* Signal handling **********************************************************/

void or1k_sigdeliver(void);

/* Power management *********************************************************/

#ifdef CONFIG_PM
void or1k_pminitialize(void);
#else
#  define or1k_pminitialize()
#endif

/* Interrupt handling *******************************************************/

/* Exception handling logic unique to the Cortex-M family */

/* Interrupt acknowledge and dispatch */

void or1k_ack_irq(int irq);
uint32_t *or1k_doirq(int irq, uint32_t *regs);

/* Exception Handlers */

int  or1k_hardfault(int irq, void *context, void *arg);
int  or1k_memfault(int irq, void *context, void *arg);

/* Interrupt acknowledge and dispatch */

uint32_t *or1k_doirq(int irq, uint32_t *regs);

/* Exception Handlers */

uint32_t *or1k_syscall(uint32_t *regs);

/* Low level serial output **************************************************/

void or1k_lowputc(char ch);
void or1k_lowputs(const char *str);

#ifdef USE_SERIALDRIVER
void or1k_serialinit(void);
#endif

#ifdef USE_EARLYSERIALINIT
void or1k_earlyserialinit(void);
#endif

/* DMA **********************************************************************/

#ifdef CONFIG_ARCH_DMA
void weak_function or1k_dma_initialize(void);
#endif

/* Cache control ************************************************************/

#ifdef CONFIG_ARCH_L2CACHE
void or1k_l2ccinitialize(void);
#else
#  define or1k_l2ccinitialize()
#endif

/* Memory management ********************************************************/

#if CONFIG_MM_REGIONS > 1
void or1k_addregion(void);
#else
# define or1k_addregion()
#endif

/* Networking ***************************************************************/

/* Defined in board/xyz_network.c for board-specific Ethernet
 * implementations, or chip/xyx_ethernet.c for chip-specific Ethernet
 * implementations, or common/or1k_etherstub.c for a corner case where the
 * network is enabled yet there is no Ethernet driver to be initialized.
 *
 * Use of common/or1k_etherstub.c is deprecated. The preferred mechanism is
 * use CONFIG_NETDEV_LATEINIT=y to suppress the call to or1k_etinitialize()
 * in up_initialize().  Then this stub would not be needed.
 */

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
void or1k_etinitialize(void);
#else
# define or1k_etinitialize()
#endif

/* USB **********************************************************************/

#ifdef CONFIG_USBDEV
void or1k_usbinitialize(void);
void or1k_usbuninitialize(void);
#else
# define or1k_usbinitialize()
# define or1k_usbuninitialize()
#endif

/* Debug ********************************************************************/
#ifdef CONFIG_STACK_COLORATION
size_t or1k_stack_check(uintptr_t alloc, size_t size);
void or1k_stack_color(void *stackbase, size_t nbytes);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_OR1K_SRC_COMMON_UP_INTERNAL_H */
