/****************************************************************************
 * arch/risc-v/src/common/riscv_internal.h
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

#ifndef __ARCH_RISCV_SRC_COMMON_RISCV_INTERNAL_H
#define __ARCH_RISCV_SRC_COMMON_RISCV_INTERNAL_H

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

/* This is the value used to mark the stack for subsequent stack monitoring
 * logic.
 */

#define STACK_COLOR    0xdeadbeef
#define INTSTACK_COLOR 0xdeadbeef
#define HEAP_COLOR     'h'

/* RISC-V requires a 16-byte stack alignment. */

#define STACK_ALIGNMENT     16
#define STACK_FRAME_SIZE    __XSTR(STACK_ALIGNMENT)

/* Stack alignment macros */

#define STACK_ALIGN_MASK    (STACK_ALIGNMENT - 1)
#define STACK_ALIGN_DOWN(a) ((a) & ~STACK_ALIGN_MASK)
#define STACK_ALIGN_UP(a)   (((a) + STACK_ALIGN_MASK) & ~STACK_ALIGN_MASK)

/* Format output with register width and hex */

#ifdef CONFIG_ARCH_RV32
#  define PRIxREG "08" PRIxPTR
#else
#  define PRIxREG "016" PRIxPTR
#endif

/* In the RISC-V model, the state is saved in stack,
 * only a reference stored in TCB.
 */

#define riscv_savestate(regs) (regs = (uintptr_t *)CURRENT_REGS)
#define riscv_restorestate(regs) (CURRENT_REGS = regs)

#define _START_TEXT  &_stext
#define _END_TEXT    &_etext
#define _START_BSS   &_sbss
#define _END_BSS     &_ebss
#define _DATA_INIT   &_eronly
#define _START_DATA  &_sdata
#define _END_DATA    &_edata
#define _START_TDATA &_stdata
#define _END_TDATA   &_etdata
#define _START_TBSS  &_stbss
#define _END_TBSS    &_etbss

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

/* Return values from riscv_check_pmp_access */

#define PMP_ACCESS_OFF      (0)     /* Access for area not set */
#define PMP_ACCESS_DENIED   (-1)    /* Access set and denied */
#define PMP_ACCESS_FULL     (1)     /* Access set and allowed */

#ifndef __ASSEMBLY__

#define getreg8(a)          (*(volatile uint8_t *)(a))
#define putreg8(v,a)        (*(volatile uint8_t *)(a) = (v))
#define getreg16(a)         (*(volatile uint16_t *)(a))
#define putreg16(v,a)       (*(volatile uint16_t *)(a) = (v))
#define getreg32(a)         (*(volatile uint32_t *)(a))
#define putreg32(v,a)       (*(volatile uint32_t *)(a) = (v))
#define getreg64(a)         (*(volatile uint64_t *)(a))
#define putreg64(v,a)       (*(volatile uint64_t *)(a) = (v))

#define READ_CSR(reg) \
  ({ \
     uintptr_t reg##_val; \
     __asm__ __volatile__("csrr %0, " __STR(reg) : "=r"(reg##_val)); \
     reg##_val; \
  })

#define READ_AND_SET_CSR(reg, bits) \
  ({ \
     uintptr_t reg##_val; \
     __asm__ __volatile__("csrrs %0, " __STR(reg) ", %1": "=r"(reg##_val) : "rK"(bits)); \
     reg##_val; \
  })

#define WRITE_CSR(reg, val) \
  ({ \
     __asm__ __volatile__("csrw " __STR(reg) ", %0" :: "rK"(val)); \
  })

#define SET_CSR(reg, bits) \
  ({ \
     __asm__ __volatile__("csrs " __STR(reg) ", %0" :: "rK"(bits)); \
  })

#define CLEAR_CSR(reg, bits) \
  ({ \
     __asm__ __volatile__("csrc " __STR(reg) ", %0" :: "rK"(bits)); \
  })

#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifndef __ASSEMBLY__
EXTERN uintptr_t g_idle_topstack;

/* Address of per-cpu idle stack base */

EXTERN const uint8_t * const g_cpu_basestack[CONFIG_SMP_NCPUS];

/* Address of the saved user stack pointer */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
EXTERN uint32_t g_intstackalloc; /* Allocated stack base */
EXTERN uint32_t g_intstacktop;   /* Initial top of interrupt stack */
#endif

/* These 'addresses' of these values are setup by the linker script.  They
 * are not actual uint32_t storage locations! They are only used meaningfully
 * in the following way:
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

#endif /* __ASSEMBLY__ */

/****************************************************************************
* Public Function Prototypes
  ***************************************************************************/

#ifndef __ASSEMBLY__
/* Atomic modification of registers */

void modifyreg32(uintptr_t addr, uint32_t clearbits, uint32_t setbits);

/* Memory allocation ********************************************************/

#if CONFIG_MM_REGIONS > 1
void riscv_addregion(void);
#else
# define riscv_addregion()
#endif

/* IRQ initialization *******************************************************/

void riscv_ack_irq(int irq);

void riscv_copystate(uintptr_t *dest, uintptr_t *src);

void riscv_sigdeliver(void);
int riscv_swint(int irq, void *context, void *arg);
uintptr_t riscv_get_newintctx(void);
void riscv_exception_attach(void);

#ifdef CONFIG_ARCH_FPU
void riscv_fpuconfig(void);
#else
#  define riscv_fpuconfig()
#endif

/* RISC-V PMP Config ********************************************************/

int riscv_config_pmp_region(uintptr_t region, uintptr_t attr,
                            uintptr_t base, uintptr_t size);

int riscv_check_pmp_access(uintptr_t attr, uintptr_t base, uintptr_t size);
int riscv_configured_pmp_regions(void);
int riscv_next_free_pmp_region(void);

/* RISC-V SBI wrappers ******************************************************/

#ifdef CONFIG_ARCH_USE_S_MODE
void riscv_sbi_set_timer(uint64_t stime_value);
uint64_t riscv_sbi_get_time(void);
#endif

/* Power management *********************************************************/

#ifdef CONFIG_PM
void riscv_pminitialize(void);
#else
#  define riscv_pminitialize()
#endif

/* Low level serial output **************************************************/

void riscv_lowputc(char ch);
void riscv_lowputs(const char *str);

#ifdef USE_SERIALDRIVER
void riscv_serialinit(void);
#endif

#ifdef USE_EARLYSERIALINIT
void riscv_earlyserialinit(void);
#endif

/* Networking ***************************************************************/

/* Defined in board/xyz_network.c for board-specific Ethernet
 * implementations, or chip/xyx_ethernet.c for chip-specific Ethernet
 * implementations.
 */

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
void riscv_netinitialize(void);
#else
# define riscv_netinitialize()
#endif

/* Exception Handler ********************************************************/

uintptr_t *riscv_doirq(int irq, uintptr_t *regs);
int riscv_exception(int mcause, void *regs, void *args);
int riscv_misaligned(int irq, void *context, void *arg);

/* Debug ********************************************************************/

#ifdef CONFIG_STACK_COLORATION
void riscv_stack_color(void *stackbase, size_t nbytes);
#endif

#ifdef CONFIG_SMP
void riscv_cpu_boot(int cpu);
int riscv_pause_handler(int irq, void *c, void *arg);
#endif

/****************************************************************************
 * Name: riscv_mhartid
 *
 * Description:
 *   Context aware way to query hart id
 *
 * Returned Value:
 *   Hart id
 *
 ****************************************************************************/

uintptr_t riscv_mhartid(void);

/* If kernel runs in Supervisor mode, a system call trampoline is needed */

#ifdef CONFIG_ARCH_USE_S_MODE
void *riscv_perform_syscall(uintptr_t *regs);
#endif

/* Context switching via system calls ***************************************/

/* SYS call 1:
 *
 * void riscv_fullcontextrestore(uintptr_t *restoreregs) noreturn_function;
 */

#define riscv_fullcontextrestore(restoreregs) \
  sys_call1(SYS_restore_context, (uintptr_t)restoreregs)

/* SYS call 2:
 *
 * void riscv_switchcontext(uintptr_t *saveregs, uintptr_t *restoreregs);
 */

#define riscv_switchcontext(saveregs, restoreregs) \
  sys_call2(SYS_switch_context, (uintptr_t)saveregs, (uintptr_t)restoreregs)

#ifdef CONFIG_BUILD_KERNEL
/* SYS call 3:
 *
 * void riscv_syscall_return(void);
 */

#define riscv_syscall_return() sys_call0(SYS_syscall_return)

#endif /* CONFIG_BUILD_KERNEL */

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_RISCV_SRC_COMMON_RISCV_INTERNAL_H */
