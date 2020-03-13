/****************************************************************************
 * arch/x86_64/src/common/up_internal.h
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

#ifndef __ARCH_X86_64_SRC_COMMON_UP_INTERNAL_H
#define __ARCH_X86_64_SRC_COMMON_UP_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/compiler.h>
#  include <nuttx/sched.h>
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bring-up debug configurations.  These are here (vs defconfig) because
 * these should only be controlled during low level board bring-up and not
 * part of normal platform configuration.
 */

#undef  CONFIG_SUPPRESS_INTERRUPTS    /* DEFINED: Do not enable interrupts */
#undef  CONFIG_SUPPRESS_TIMER_INTS    /* DEFINED: No timer */
#undef  CONFIG_SUPPRESS_SERIAL_INTS   /* DEFINED: Console will poll */
#undef  CONFIG_SUPPRESS_UART_CONFIG   /* DEFINED: Do not reconfig UART */
#undef  CONFIG_DUMP_ON_EXIT           /* DEFINED: Dump task state on exit */

#ifndef CONFIG_DEBUG_SCHED_INFO
#  undef CONFIG_DUMP_ON_EXIT          /* Needs CONFIG_DEBUG_SCHED_INFO */
#endif

/* Determine which (if any) console driver to use.  If a console is enabled
 * and no other console device is specified, then a serial console is
 * assumed.
 */

#if !defined(CONFIG_DEV_CONSOLE) || CONFIG_NFILE_DESCRIPTORS <= 0
#  undef  USE_SERIALDRIVER
#  undef  USE_EARLYSERIALINIT
#  undef  CONFIG_DEV_LOWCONSOLE
#  undef  CONFIG_RAMLOG_CONSOLE
#else
#  if defined(CONFIG_RAMLOG_CONSOLE)
#    undef  USE_SERIALDRIVER
#    undef  USE_EARLYSERIALINIT
#    undef  CONFIG_DEV_LOWCONSOLE
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

/* Macros to handle saving and restore interrupt state.  In the current
 * model, the state is copied from the stack to the TCB, but only a
 * referenced is passed to get the state from the TCB.
 */

#define up_restorestate(regs) (g_current_regs = regs)

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
/* This holds a references to the current interrupt level register storage
 * structure.  If is non-NULL only during interrupt processing.
 */

extern volatile uint64_t *g_current_regs;

/* This is the beginning of heap as provided from up_head.S. This is the first
 * address in DRAM after the loaded program+bss+idle stack.  The end of the
 * heap is CONFIG_RAM_END
 */

extern uint64_t g_idle_topstack;

/* Address of the saved user stack pointer */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
extern uint64_t g_intstackbase;
#endif

/* These 'addresses' of these values are setup by the linker script.  They are
 * not actual uint32_t storage locations! They are only used meaningfully in the
 * following way:
 *
 *  - The linker script defines, for example, the symbol_sdata.
 *  - The declareion extern uint32_t _sdata; makes C happy.  C will believe
 *    that the value _sdata is the address of a uint32_t variable _data (it is
 *    not!).
 *  - We can recoved the linker value then by simply taking the address of
 *    of _data.  like:  uint32_t *pdata = &_sdata;
 */

extern uint64_t _stext;           /* Start of .text */
extern uint64_t _etext;           /* End_1 of .text + .rodata */
extern const uint64_t _eronly;    /* End+1 of read only section (.text + .rodata) */
extern uint64_t _sdata;           /* Start of .data */
extern uint64_t _edata;           /* End+1 of .data */
extern uint64_t _sbss;            /* Start of .bss */
extern uint64_t _ebss;            /* End+1 of .bss */
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: x86_64_boardinitialize
 *
 * Description:
 *   This function must be provided by the board-specific logic in the
 *   directory configs/<board-name>/src/.
 *
 ****************************************************************************/

void x86_64_boardinitialize(void);

/* Defined in files with the same name as the function */

void up_copystate(uint64_t *dest, uint64_t *src);
void up_savestate(uint64_t *regs);
void up_decodeirq(uint64_t *regs);
void up_irqinitialize(void);
#ifdef CONFIG_ARCH_DMA
void weak_function up_dmainitialize(void);
#endif
int  up_saveusercontext(uint64_t *saveregs);
void up_fullcontextrestore(uint64_t *restoreregs) noreturn_function;
void up_switchcontext(uint64_t *saveregs, uint64_t *restoreregs);
void up_sigdeliver(void);
void up_lowputc(char ch);
void up_puts(const char *str);
void up_lowputs(const char *str);
void up_restore_auxstate(struct tcb_s *rtcb);
void up_checktasks(void);

void up_syscall(uint64_t *regs);
void up_registerdump(uint64_t *regs);

/* Defined in up_allocateheap.c */

#if CONFIG_MM_REGIONS > 1
void up_addregion(void);
#else
# define up_addregion()
#endif

/* Defined in xyz_serial.c */

#if CONFIG_NFILE_DESCRIPTORS > 0
void up_earlyserialinit(void);
void up_serialinit(void);
#else
# define up_earlyserialinit()
# define up_serialinit()
#endif

/* Defined in xyz_watchdog.c */

void up_wdtinit(void);

/* Defined in xyz_timerisr.c */

void x86_64_timer_calibrate_freq(void);
void x86_64_timer_initialize(void);

/* Defined in board/up_network.c */

#ifdef CONFIG_NET
void up_netinitialize(void);
#else
# define up_netinitialize()
#endif

#ifdef CONFIG_USBDEV
void up_usbinitialize(void);
void up_usbuninitialize(void);
#else
# define up_usbinitialize()
# define up_usbuninitialize()
#endif

#endif /* __ASSEMBLY__ */

#endif  /* __ARCH_X86_64_SRC_COMMON_UP_INTERNAL_H */
