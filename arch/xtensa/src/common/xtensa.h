/****************************************************************************
 * arch/xtensa/src/common/xtensa.h
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

#ifndef __ARCH_XTENSA_SRC_COMMON_XTENSA_H
#define __ARCH_XTENSA_SRC_COMMON_XTENSA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/arch.h>
#  include <stdint.h>
#  include <sys/types.h>
#  include <stdbool.h>
#  include <syscall.h>
#endif

#include <arch/chip/core-isa.h>
#include <arch/chip/tie.h>

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
#else
#  define INTSTACK_ALIGNMENT    16
#  define INTSTACK_ALIGN_MASK   (INTSTACK_ALIGNMENT - 1)
#  define INTSTACK_ALIGNDOWN(s) ((s) & ~INTSTACK_ALIGN_MASK)
#  define INTSTACK_ALIGNUP(s)   (((s) + INTSTACK_ALIGN_MASK) & ~INTSTACK_ALIGN_MASK)
#  define INTSTACK_SIZE         INTSTACK_ALIGNUP(CONFIG_ARCH_INTERRUPTSTACK)
#endif

/* XTENSA requires at least a 16-byte stack alignment. */

#define STACK_ALIGNMENT     16

/* Stack alignment macros */

#define STACK_ALIGN_MASK    (STACK_ALIGNMENT - 1)
#define STACK_ALIGN_DOWN(a) ((a) & ~STACK_ALIGN_MASK)
#define STACK_ALIGN_UP(a)   (((a) + STACK_ALIGN_MASK) & ~STACK_ALIGN_MASK)

/* An IDLE thread stack size for CPU0 must be defined */

#if !defined(CONFIG_IDLETHREAD_STACKSIZE)
#  error CONFIG_IDLETHREAD_STACKSIZE is not defined
#elif CONFIG_IDLETHREAD_STACKSIZE < 16
#  error CONFIG_IDLETHREAD_STACKSIZE is to small
#endif

#define IDLETHREAD_STACKSIZE  ((CONFIG_IDLETHREAD_STACKSIZE + 15) & ~15)
#define IDLETHREAD_STACKWORDS (IDLETHREAD_STACKSIZE >> 2)

/* In the Xtensa model, the state is saved in stack,
 * only a reference stored in TCB.
 */

#define xtensa_savestate(regs)    ((regs) = (uint32_t *)CURRENT_REGS)
#define xtensa_restorestate(regs) (CURRENT_REGS = (regs))

/* Context switching via system calls ***************************************/

#define xtensa_context_restore(regs)\
  sys_call1(SYS_restore_context, (uintptr_t)regs)

#define xtensa_switchcontext(saveregs, restoreregs)\
  sys_call2(SYS_switch_context, (uintptr_t)saveregs, (uintptr_t)restoreregs)

/* Interrupt codes from other CPUs: */

#define CPU_INTCODE_NONE  0
#define CPU_INTCODE_PAUSE 1

/* Exception Codes that may be received by xtensa_panic(). */

#define XTENSA_NMI_EXCEPTION     0
#define XTENSA_DEBUG_EXCEPTION   1
#define XTENSA_DOUBLE_EXCEPTION  2
#define XTENSA_KERNEL_EXCEPTION  3
#define XTENSA_COPROC_EXCEPTION  4
#define XTENSA_LEVEL2_EXCEPTION  5
#define XTENSA_LEVEL3_EXCEPTION  6
#define XTENSA_LEVEL4_EXCEPTION  7
#define XTENSA_LEVEL5_EXCEPTION  8
#define XTENSA_LEVEL6_EXCEPTION  9

/* Register access macros */

#define getreg8(a)        (*(volatile uint8_t *)(a))
#define putreg8(v,a)      (*(volatile uint8_t *)(a) = (v))
#define getreg16(a)       (*(volatile uint16_t *)(a))
#define putreg16(v,a)     (*(volatile uint16_t *)(a) = (v))
#define getreg32(a)       (*(volatile uint32_t *)(a))
#define putreg32(v,a)     (*(volatile uint32_t *)(a) = (v))

/* This is the value used to mark the stack for subsequent stack monitoring
 * logic.
 */

#define STACK_COLOR    0xdeadbeef
#define INTSTACK_COLOR 0xdeadbeef
#define HEAP_COLOR     'h'

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#if !defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 15
/* The (optional) interrupt stack */

extern uint8_t g_intstackalloc[]; /* Allocated interrupt stack */
extern uint8_t g_intstacktop[];   /* Initial top of interrupt stack */
#endif

/* Address of the CPU0 IDLE thread */

extern uint32_t g_idlestack[IDLETHREAD_STACKWORDS];

/* These symbols are setup by the linker script. */

extern uint8_t _init_start[];        /* Start of initialization logic */
extern uint8_t _stext[];             /* Start of .text */
extern uint8_t _etext[];             /* End+1 of .text + .rodata */
extern uint8_t _sdata[];             /* Start of .data */
extern uint8_t _edata[];             /* End+1 of .data */
extern uint8_t _srodata[];           /* Start of .rodata */
extern uint8_t _erodata[];           /* End+1 of .rodata */
extern uint8_t _sbss[];              /* Start of .bss */
extern uint8_t _ebss[];              /* End+1 of .bss */
extern uint8_t _sheap[];             /* Start of heap */
extern uint8_t _eheap[];             /* End+1 of heap */
extern uint8_t _sbss_extmem[];       /* start of external memory bss */
extern uint8_t _ebss_extmem[];       /* End+1 of external memory bss */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Common Functions *********************************************************/

/* Common functions defined in arch/xtensa/src/common.  These may be replaced
 * with chip-specific functions of the same name if needed.  See also
 * functions prototyped in include/nuttx/arch.h.
 */

/* Atomic modification of registers */

void modifyreg8(unsigned int addr, uint8_t clearbits, uint8_t setbits);
void modifyreg16(unsigned int addr, uint16_t clearbits, uint16_t setbits);
void modifyreg32(unsigned int addr, uint32_t clearbits, uint32_t setbits);

/* Serial output */

void xtensa_lowputs(const char *str);

/* Common XTENSA functions */

/* Initialization */

#if XCHAL_CP_NUM > 0
void xtensa_coproc_enable(int cpset);
void xtensa_coproc_disable(int cpset);
#endif

/* Window Spill */

void xtensa_window_spill(void);

/* IRQs */

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 15
uintptr_t xtensa_intstack_alloc(void);
uintptr_t xtensa_intstack_top(void);
#endif

uint32_t *xtensa_int_decode(uint32_t cpuints, uint32_t *regs);
uint32_t *xtensa_irq_dispatch(int irq, uint32_t *regs);
uint32_t xtensa_enable_cpuint(uint32_t *shadow, uint32_t intmask);
uint32_t xtensa_disable_cpuint(uint32_t *shadow, uint32_t intmask);
void xtensa_panic(int xptcode, uint32_t *regs) noreturn_function;
void xtensa_user_panic(int exccause, uint32_t *regs) noreturn_function;
uint32_t *xtensa_user(int exccause, uint32_t *regs);

/* Software interrupt handler */

#ifdef CONFIG_SMP
int xtensa_intercpu_interrupt(int tocpu, int intcode);
void xtensa_pause_handler(void);
#endif

/* Signals */

void xtensa_sig_deliver(void);

#ifdef CONFIG_LIB_SYSCALL
void xtensa_dispatch_syscall(unsigned int nbr, uintptr_t parm1,
                             uintptr_t parm2, uintptr_t parm3,
                             uintptr_t parm4, uintptr_t parm5);
#endif

/* Chip-specific functions **************************************************/

/* Chip specific functions defined in arch/xtensa/src/<chip> */

/* IRQs */

bool xtensa_pending_irq(int irq);
void xtensa_clrpend_irq(int irq);

/* DMA */

#ifdef CONFIG_ARCH_DMA
void weak_function xtensa_dma_initialize(void);
#endif

/* Memory management */

#if CONFIG_MM_REGIONS > 1
void xtensa_add_region(void);
#else
# define xtensa_add_region()
#endif

/* Watchdog timer ***********************************************************/

struct oneshot_lowerhalf_s *
xtensa_oneshot_initialize(uint32_t irq, uint32_t freq);

/* Serial output */

void xtensa_lowputc(char ch);
void xtensa_earlyserialinit(void);
void xtensa_serialinit(void);

/* Network */

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
void xtensa_netinitialize(void);
#else
# define xtensa_netinitialize()
#endif

/* USB */

#ifdef CONFIG_USBDEV
void xtensa_usbinitialize(void);
void xtensa_usbuninitialize(void);
#else
# define xtensa_usbinitialize()
# define xtensa_usbuninitialize()
#endif

/* Power management *********************************************************/

#ifdef CONFIG_PM
void xtensa_pminitialize(void);
#else
#  define xtensa_pminitialize()
#endif

/* Interrupt handling *******************************************************/

/* Exception Handlers */

int xtensa_swint(int irq, void *context, void *arg);

/* Debug ********************************************************************/

#ifdef CONFIG_STACK_COLORATION
size_t xtensa_stack_check(uintptr_t alloc, size_t size);
void xtensa_stack_color(void *stackbase, size_t nbytes);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_COMMON_XTENSA_H */
