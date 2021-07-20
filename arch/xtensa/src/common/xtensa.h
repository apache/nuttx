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
#  include <stdint.h>
#  include <sys/types.h>
#  include <stdbool.h>
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

/* An IDLE thread stack size for CPU0 must be defined */

#if !defined(CONFIG_IDLETHREAD_STACKSIZE)
#  error CONFIG_IDLETHREAD_STACKSIZE is not defined
#elif CONFIG_IDLETHREAD_STACKSIZE < 16
#  error CONFIG_IDLETHREAD_STACKSIZE is to small
#endif

#define IDLETHREAD_STACKSIZE  ((CONFIG_IDLETHREAD_STACKSIZE + 15) & ~15)
#define IDLETHREAD_STACKWORDS (IDLETHREAD_STACKSIZE >> 2)

/* In the XTENSA model, the state is copied from the stack to the TCB, but
 * only a referenced is passed to get the state from the TCB.
 *
 * REVISIT: It would not be too difficult to save only a pointer to the
 * state save area in the TCB and thus avoid the copy.
 */

#define xtensa_savestate(regs)    xtensa_copystate(regs, (uint32_t*)CURRENT_REGS)
#define xtensa_restorestate(regs) do { CURRENT_REGS = regs; } while (0)

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
/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
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

#if !defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 15
/* The (optional) interrupt stack */

extern uint32_t g_intstackalloc; /* Allocated interrupt stack */
extern uint32_t g_intstacktop;   /* Initial top of interrupt stack */
#endif

/* Address of the CPU0 IDLE thread */

extern uint32_t g_idlestack[IDLETHREAD_STACKWORDS];

/* These 'addresses' of these values are setup by the linker script.  They
 * are not actual uint32_t storage locations! They are only used meaningfully
 * in the following way:
 *
 *  - The linker script defines, for example, the symbol_sdata.
 *  - The declaration extern uint32_t _sdata; makes C happy.  C will believe
 *    that the value _sdata is the address of a uint32_t variable _data (it
 *    is not!).
 *  - We can recoved the linker value then by simply taking the address of
 *    of _data.  like:  uint32_t *pdata = &_sdata;
 */

extern uint32_t _init_start;        /* Start of initialization logic */
extern uint32_t _stext;             /* Start of .text */
extern uint32_t _etext;             /* End+1 of .text + .rodata */
extern uint32_t _sdata;             /* Start of .data */
extern uint32_t _edata;             /* End+1 of .data */
extern uint32_t _srodata;           /* Start of .rodata */
extern uint32_t _erodata;           /* End+1 of .rodata */
extern uint32_t _sbss;              /* Start of .bss */
extern uint32_t _ebss;              /* End+1 of .bss */
extern uint32_t _sheap;             /* Start of heap */
extern uint32_t _eheap;             /* End+1 of heap */
extern uint32_t _sbss_extmem;       /* start of external memory bss */
extern uint32_t _ebss_extmem;       /* End+1 of external memory bss */

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

/* Context switching */

void xtensa_copystate(uint32_t *dest, uint32_t *src);

/* Serial output */

void up_puts(const char *str);
void up_lowputs(const char *str);

/* Debug */

#ifdef CONFIG_ARCH_STACKDUMP
void xtensa_dumpstate(void);
#else
#  define xtensa_dumpstate()
#endif

/* Common XTENSA functions */

/* Initialization */

#if XCHAL_CP_NUM > 0
struct xtensa_cpstate_s;
void xtensa_coproc_enable(struct xtensa_cpstate_s *cpstate, int cpset);
void xtensa_coproc_disable(struct xtensa_cpstate_s *cpstate, int cpset);
#endif

/* IRQs */

uint32_t *xtensa_int_decode(uint32_t cpuints, uint32_t *regs);
uint32_t *xtensa_irq_dispatch(int irq, uint32_t *regs);
uint32_t xtensa_enable_cpuint(uint32_t *shadow, uint32_t intmask);
uint32_t xtensa_disable_cpuint(uint32_t *shadow, uint32_t intmask);
void xtensa_panic(int xptcode, uint32_t *regs) noreturn_function;
void xtensa_user_panic(int exccause, uint32_t *regs) noreturn_function;
uint32_t *xtensa_user(int exccause, uint32_t *regs);

/* Software interrupt handler */

#ifdef CONFIG_SMP
void __cpu1_start(void) noreturn_function;
int xtensa_intercpu_interrupt(int tocpu, int intcode);
void xtensa_pause_handler(void);
#endif

/* Synchronous context switching */

int xtensa_context_save(uint32_t *regs);
void xtensa_context_restore(uint32_t *regs) noreturn_function;

#if XCHAL_CP_NUM > 0
void xtensa_coproc_savestate(struct xtensa_cpstate_s *cpstate);
void xtensa_coproc_restorestate(struct xtensa_cpstate_s *cpstate);
#endif

/* Signals */

void _xtensa_sig_trampoline(void);
void xtensa_sig_deliver(void);

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

/* Serial output */

void up_lowputc(char ch);
void xtensa_earlyserialinit(void);
void xtensa_serialinit(void);

void rpmsg_serialinit(void);

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

/* Power management *********************************************************/

#ifdef CONFIG_PM
void xtensa_pminitialize(void);
#else
#  define xtensa_pminitialize()
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_STACK_COLORATION
void up_stack_color(FAR void *stackbase, size_t nbytes);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_COMMON_XTENSA_H */
