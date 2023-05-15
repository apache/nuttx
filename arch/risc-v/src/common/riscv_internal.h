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
#  include <sys/types.h>
#  include <stdint.h>
#  include <syscall.h>
#endif

#include <nuttx/irq.h>

#include "riscv_common_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_ARCH_QPFPU)
#  define FLOAD     __STR(flq)
#  define FSTORE    __STR(fsq)
#elif defined(CONFIG_ARCH_DPFPU)
#  define FLOAD     __STR(fld)
#  define FSTORE    __STR(fsd)
#else
#  define FLOAD     __STR(flw)
#  define FSTORE    __STR(fsw)
#endif

#ifdef CONFIG_ARCH_RV32
#  define REGLOAD   __STR(lw)
#  define REGSTORE  __STR(sw)
#else
#  define REGLOAD   __STR(ld)
#  define REGSTORE  __STR(sd)
#endif

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

void riscv_sigdeliver(void);
int riscv_swint(int irq, void *context, void *arg);
uintptr_t riscv_get_newintctx(void);
void riscv_set_idleintctx(void);
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

/* DMA **********************************************************************/

#ifdef CONFIG_ARCH_DMA
void weak_function riscv_dma_initialize(void);
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
size_t riscv_stack_check(uintptr_t alloc, size_t size);
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
