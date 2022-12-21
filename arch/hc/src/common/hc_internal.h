/****************************************************************************
 * arch/hc/src/common/hc_internal.h
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

#ifndef __ARCH_HC_SRC_COMMON_UP_INTERNAL_H
#define __ARCH_HC_SRC_COMMON_UP_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/compiler.h>
#  include <nuttx/irq.h>
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

/* The CPU12 stack should be aligned at half-word (2 byte) boundaries. If
 * necessary frame_size must be rounded up to the next boundary
 */

#define STACK_ALIGNMENT     2

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

/* Macros to handle saving and restore interrupt state.  In the current CPU12
 * model, the state is copied from the stack to the TCB, but only
 * a referenced is passed to get the state from the TCB.
 */

#define hc_savestate(regs)    hc_copystate(regs, (uint8_t*)g_current_regs)
#define hc_restorestate(regs) (g_current_regs = regs)

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
/* This is the beginning of heap as provided from processor-specific logic.
 * This is the first address in RAM after the loaded program+bss+idle stack.
 * The end of the heap is CONFIG_RAM_END
 */

extern uint16_t g_idle_topstack;

/* Address of the saved user stack pointer */

#if CONFIG_ARCH_INTERRUPTSTACK > 1
extern uint8_t g_intstackalloc[];
extern uint8_t g_intstacktop[];
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/* Atomic modification of registers */

void modifyreg8(unsigned int addr, uint8_t clearbits, uint8_t setbits);
void modifyreg16(unsigned int addr, uint16_t clearbits, uint16_t setbits);
void modifyreg32(unsigned int addr, uint32_t clearbits, uint32_t setbits);

/* Context switching functions */

void hc_copystate(uint8_t *dest, uint8_t *src);
void hc_decodeirq(uint8_t *regs);
void hc_fullcontextrestore(uint8_t *restoreregs) noreturn_function;
void hc_switchcontext(uint8_t *saveregs, uint8_t *restoreregs);

/* Interrupt handling */

uint8_t *hc_doirq(int irq, uint8_t *regs);

/* Signal handling */

void hc_sigdeliver(void);

/* Debug output */

#ifdef USE_EARLYSERIALINIT
void hc_earlyserialinit(void);
#endif

#ifdef USE_SERIALDRIVER
void hc_serialinit(void);
#endif

void hc_lowputc(char ch);
void hc_lowputs(const char *str);

/* Memory configuration */

#if CONFIG_MM_REGIONS > 1
void hc_addregion(void);
#else
# define hc_addregion()
#endif

/* Sub-system/driver initialization */

#ifdef CONFIG_ARCH_DMA
void weak_function hc_dma_initialize(void);
#endif

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
void hc_netinitialize(void);
#else
# define hc_netinitialize()
#endif

#ifdef CONFIG_USBDEV
void hc_usbinitialize(void);
void hc_usbuninitialize(void);
#else
# define hc_usbinitialize()
# define hc_usbuninitialize()
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_HC_SRC_COMMON_UP_INTERNAL_H */
