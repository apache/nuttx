/****************************************************************************
 * arch/mips/common/mips_internal.h
 *
 *   Copyright (C) 2011, 2012, 2015, 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_MIPS_SRC_COMMON_UP_INTERNAL_H
#define __ARCH_MIPS_SRC_COMMON_UP_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
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

/* In the MIPS model, the state is copied from the stack to the TCB, but
 * only a referenced is passed to get the state from the TCB.
 */

#define up_savestate(regs)    up_copystate(regs, (uint32_t*)CURRENT_REGS)
#define up_restorestate(regs) (CURRENT_REGS = regs)

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
/* g_current_regs holds a references to the current interrupt level
 * register storage structure.  It is non-NULL only during interrupt
 * processing.  Access to g_current_regs must be through the macro
 * CURRENT_REGS for portability.
 */

#ifdef CONFIG_SMP
/* For the case of architectures with multiple CPUs, then there must be one
 * such value for each processor that can receive an interrupt.
 */

int up_cpu_index(void); /* See include/nuttx/arch.h */
extern volatile uint32_t *g_current_regs[CONFIG_SMP_NCPUS];
#  define CURRENT_REGS (g_current_regs[up_cpu_index()])

#else

extern volatile uint32_t *g_current_regs[1];
#  define CURRENT_REGS (g_current_regs[0])

#endif

/* This is the beginning of heap as provided from up_head.S. This is the
 * first address in DRAM after the loaded program+bss+idle stack.  The end
 * of the heap is CONFIG_RAM_END
 */

extern uint32_t g_idle_topstack;

/* Address of the saved user stack pointer */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
extern void g_intstackbase;
#endif

/* These 'addresses' of these values are setup by the linker script. They are
 * not actual uint32_t storage locations! They are only used meaningfully in
 * the following way:
 *
 *  - The linker script defines, for example, the symbol_sdata.
 *  - The declaration extern uint32_t _sdata; makes C happy.  C will believe
 *    that the value _sdata is the address of a uint32_t variable _data (it
 *    is not!).
 *  - We can recoved the linker value then by simply taking the address of
 *    of _data.  like:  uint32_t *pdata = &_sdata;
 */

extern uint32_t _stext;             /* Start of .text */
extern uint32_t _etext;             /* End+1 of .text + .rodata */
extern const uint32_t _data_loaddr; /* Start of .data in FLASH */
extern uint32_t _sdata;             /* Start of .data */
extern uint32_t _edata;             /* End+1 of .data */
extern uint32_t _sbss;              /* Start of .bss */
extern uint32_t _ebss;              /* End+1 of .bss */
#ifdef CONFIG_ARCH_RAMFUNCS
extern uint32_t _sramfunc;          /* Start of ramfuncs */
extern uint32_t _eramfunc;          /* End+1 of ramfuncs */
extern uint32_t _ramfunc_loadaddr;  /* Start of ramfuncs in FLASH */
extern uint32_t _ramfunc_sizeof;    /* Size of ramfuncs */
extern uint32_t _bmxdkpba_address;  /* BMX register setting */
extern uint32_t _bmxdudba_address;  /* BMX register setting */
extern uint32_t _bmxdupba_address;  /* BMX register setting */
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

/* Context switching */

void up_copystate(uint32_t *dest, uint32_t *src);

/* Serial output */

void up_puts(const char *str);
void up_lowputs(const char *str);

/* Debug */

#ifdef CONFIG_ARCH_STACKDUMP
void up_dumpstate(void);
#else
#  define up_dumpstate()
#endif

/* Common MIPS32 functions defined in arch/mips/src/MIPS32 */

/* IRQs */

uint32_t *up_doirq(int irq, uint32_t *regs);

/* Software interrupt 0 handler */

int up_swint0(int irq, FAR void *context, FAR void *arg);

/* Signals */

void up_sigdeliver(void);

/* Chip-specific functions **************************************************/

/* Chip specific functions defined in arch/mips/src/<chip> */

/* IRQs */

bool up_pending_irq(int irq);
void up_clrpend_irq(int irq);
void up_clrpend_sw0(void);

/* DMA */

#ifdef CONFIG_ARCH_DMA
void weak_function up_dma_initialize(void);
#endif

/* Memory management */

#if CONFIG_MM_REGIONS > 1
void up_addregion(void);
#else
# define up_addregion()
#endif

/* Serial output */

void up_lowputc(char ch);

#ifdef USE_EARLYSERIALINIT
void up_earlyserialinit(void);
#endif

#ifdef USE_SERIALDRIVER
void up_serialinit(void);
#endif

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void);
#endif

/* Network */

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
void up_netinitialize(void);
#else
# define up_netinitialize()
#endif

/* USB */

#ifdef CONFIG_USBDEV
void up_usbinitialize(void);
void up_usbuninitialize(void);
#else
# define up_usbinitialize()
# define up_usbuninitialize()
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_COMMON_UP_INTERNAL_H */
