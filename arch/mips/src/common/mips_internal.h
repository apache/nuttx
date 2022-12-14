/****************************************************************************
 * arch/mips/src/common/mips_internal.h
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

#ifndef __ARCH_MIPS_SRC_COMMON_MIPS_INTERNAL_H
#define __ARCH_MIPS_SRC_COMMON_MIPS_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <nuttx/arch.h>
#  include <stdint.h>
#  include <stdbool.h>
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

/* MIPS requires at least a 4-byte stack alignment.  For floating point use,
 * however, the stack must be aligned to 8-byte addresses.
 */

#ifdef CONFIG_LIBC_FLOATINGPOINT
#  define STACK_ALIGNMENT   8
#else
#  define STACK_ALIGNMENT   4
#endif

/* Stack alignment macros */

#define STACK_ALIGN_MASK    (STACK_ALIGNMENT - 1)
#define STACK_ALIGN_DOWN(a) ((a) & ~STACK_ALIGN_MASK)
#define STACK_ALIGN_UP(a)   (((a) + STACK_ALIGN_MASK) & ~STACK_ALIGN_MASK)

#define getreg8(a)          (*(volatile uint8_t *)(a))
#define putreg8(v,a)        (*(volatile uint8_t *)(a) = (v))
#define getreg16(a)         (*(volatile uint16_t *)(a))
#define putreg16(v,a)       (*(volatile uint16_t *)(a) = (v))
#define getreg32(a)         (*(volatile uint32_t *)(a))
#define putreg32(v,a)       (*(volatile uint32_t *)(a) = (v))

/* In the MIPS model, the state is copied from the stack to the TCB, but
 * only a referenced is passed to get the state from the TCB.
 */

#define mips_savestate(regs)    mips_copystate(regs, (uint32_t*)CURRENT_REGS)
#define mips_restorestate(regs) (CURRENT_REGS = regs)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
typedef void (*mips_vector_t)(void);
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

#if CONFIG_ARCH_INTERRUPTSTACK > 3
extern uint8_t g_intstackalloc[];
extern uint8_t g_intstacktop[];
#endif

/* These symbols are setup by the linker script. */

extern uint8_t _stext[];             /* Start of .text */
extern uint8_t _etext[];             /* End+1 of .text + .rodata */
extern const uint8_t _data_loaddr[]; /* Start of .data in FLASH */
extern uint8_t _sdata[];             /* Start of .data */
extern uint8_t _edata[];             /* End+1 of .data */
extern uint8_t _sbss[];              /* Start of .bss */
extern uint8_t _ebss[];              /* End+1 of .bss */
#ifdef CONFIG_ARCH_RAMFUNCS
extern uint8_t _sramfunc[];          /* Start of ramfuncs */
extern uint8_t _eramfunc[];          /* End+1 of ramfuncs */
extern uint8_t _ramfunc_loadaddr[];  /* Start of ramfuncs in FLASH */
extern uint8_t _ramfunc_sizeof[];    /* Size of ramfuncs */
extern uint8_t _bmxdkpba_address[];  /* BMX register setting */
extern uint8_t _bmxdudba_address[];  /* BMX register setting */
extern uint8_t _bmxdupba_address[];  /* BMX register setting */
#endif /* CONFIG_ARCH_RAMFUNCS */
#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
/* Common Functions *********************************************************/

/* Common functions define in arch/mips/src/common.  These may be replaced
 * with chip-specific functions of the same name if needed.  See also
 * functions prototyped in include/nuttx/arch.h.
 */

/* Atomic modification of registers */

void modifyreg8(unsigned int addr, uint8_t clearbits, uint8_t setbits);
void modifyreg16(unsigned int addr, uint16_t clearbits, uint16_t setbits);
void modifyreg32(unsigned int addr, uint32_t clearbits, uint32_t setbits);

/* Context switching */

void mips_copystate(uint32_t *dest, uint32_t *src);

/* Serial output */

void mips_lowputs(const char *str);

/* Debug */

void mips_registerdump(volatile uint32_t *regs);

/* Common MIPS32 functions defined in arch/mips/src/MIPS32 */

/* IRQs */

uint32_t *mips_doirq(int irq, uint32_t *regs);

/* Software interrupt 0 handler */

int mips_swint0(int irq, void *context, void *arg);

/* Signals */

void mips_sigdeliver(void);

/* Chip-specific functions **************************************************/

/* Chip specific functions defined in arch/mips/src/<chip> */

/* IRQs */

bool mips_pending_irq(int irq);
void mips_clrpend_irq(int irq);
void mips_clrpend_sw0(void);

/* DMA */

#ifdef CONFIG_ARCH_DMA
void weak_function mips_dma_initialize(void);
#endif

/* Memory management */

#if CONFIG_MM_REGIONS > 1
void mips_addregion(void);
#else
# define mips_addregion()
#endif

/* Serial output */

void mips_lowputc(char ch);

#ifdef USE_EARLYSERIALINIT
void mips_earlyserialinit(void);
#endif

#ifdef USE_SERIALDRIVER
void mips_serialinit(void);
#endif

/* Network */

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
void mips_netinitialize(void);
#else
# define mips_netinitialize()
#endif

/* USB */

#ifdef CONFIG_USBDEV
void mips_usbinitialize(void);
void mips_usbuninitialize(void);
#else
# define mips_usbinitialize()
# define mips_usbuninitialize()
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_COMMON_MIPS_INTERNAL_H */
