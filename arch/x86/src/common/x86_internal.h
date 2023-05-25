/****************************************************************************
 * arch/x86/src/common/x86_internal.h
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

#ifndef __ARCH_X86_SRC_COMMON_UP_INTERNAL_H
#define __ARCH_X86_SRC_COMMON_UP_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/compiler.h>
#  include <stdint.h>
#  include <arch/io.h>
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

/* The initial stack point is aligned at word (4 byte) boundaries. If
 * necessary frame_size must be rounded up to the next boundary to retain
 * this alignment.
 */

#define STACK_ALIGNMENT     4

/* Stack alignment macros */

#define STACK_ALIGN_MASK    (STACK_ALIGNMENT - 1)
#define STACK_ALIGN_DOWN(a) ((a) & ~STACK_ALIGN_MASK)
#define STACK_ALIGN_UP(a)   (((a) + STACK_ALIGN_MASK) & ~STACK_ALIGN_MASK)

#define getreg8(p)          inb(p)
#define putreg8(v,p)        outb(v,p)
#define getreg16(p)         inw(p)
#define putreg16(v,p)       outw(v,p)
#define getreg32(p)         inl(p)
#define putreg32(v,p)       outl(v,p)

/* Macros to handle saving and restore interrupt state.  In the current
 * model, the state is copied from the stack to the TCB, but only a
 * referenced is passed to get the state from the TCB.
 */

#define x86_restorestate(regs) (g_current_regs = regs)

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

#if CONFIG_ARCH_INTERRUPTSTACK > 3
extern uint8_t g_intstackalloc[];
extern uint8_t g_intstacktop[];
#endif

/* These symbols are setup by the linker script. */

extern uint8_t _stext[];           /* Start of .text */
extern uint8_t _etext[];           /* End_1 of .text + .rodata */
extern const uint8_t _eronly[];    /* End+1 of read only section (.text + .rodata) */
extern uint8_t _sdata[];           /* Start of .data */
extern uint8_t _edata[];           /* End+1 of .data */
extern uint8_t _sbss[];            /* Start of .bss */
extern uint8_t _ebss[];            /* End+1 of .bss */
#endif

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

/****************************************************************************
 * Name: x86_boardinitialize
 *
 * Description:
 *   This function must be provided by the board-specific logic in the
 *   directory boards/x86/<chip>/<board>/src/.
 *
 ****************************************************************************/

void x86_boardinitialize(void);

/* Defined in files with the same name as the function */

void x86_copystate(uint32_t *dest, uint32_t *src);
void x86_savestate(uint32_t *regs);
void x86_decodeirq(uint32_t *regs);
#ifdef CONFIG_ARCH_DMA
void weak_function x86_dma_initialize(void);
#endif
void x86_fullcontextrestore(uint32_t *restoreregs) noreturn_function;
void x86_switchcontext(uint32_t *saveregs, uint32_t *restoreregs);
void x86_sigdeliver(void);
void x86_lowputc(char ch);
void x86_lowputs(const char *str);

void x86_syscall(uint32_t *regs);

/* Defined in up_allocateheap.c */

#if CONFIG_MM_REGIONS > 1
void x86_addregion(void);
#else
#  define x86_addregion()
#endif

/* Defined in xyz_serial.c */
#ifdef USE_EARLYSERIALINIT
void x86_earlyserialinit(void);
#endif

#ifdef USE_SERIALDRIVER
void x86_serialinit(void);
#endif

/* Defined in board/x86_network.c */

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
void x86_netinitialize(void);
#else
#  define x86_netinitialize()
#endif

#ifdef CONFIG_USBDEV
void x86_usbinitialize(void);
void x86_usbuninitialize(void);
#else
#  define x86_usbinitialize()
#  define x86_usbuninitialize()
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_X86_SRC_COMMON_UP_INTERNAL_H */
