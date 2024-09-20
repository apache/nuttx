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
#  include <nuttx/sched.h>
#  include <sys/types.h>
#  include <stdint.h>
#  include <syscall.h>
#endif

#include <nuttx/irq.h>

#include "riscv_sbi.h"
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

/* Interrupt Stack macros */
#define INT_STACK_SIZE  (STACK_ALIGN_DOWN(CONFIG_ARCH_INTERRUPTSTACK))

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

/* Use ASM as rv64ilp32 compiler generated address is limited */

static inline uint8_t getreg8(const volatile uintreg_t a)
{
  uint8_t v;
  __asm__ __volatile__("lb %0, 0(%1)" : "=r" (v) : "r" (a));
  return v;
}

static inline void putreg8(uint8_t v, const volatile uintreg_t a)
{
  __asm__ __volatile__("sb %0, 0(%1)" : : "r" (v), "r" (a));
}

static inline uint16_t getreg16(const volatile uintreg_t a)
{
  uint16_t v;
  __asm__ __volatile__("lh %0, 0(%1)" : "=r" (v) : "r" (a));
  return v;
}

static inline void putreg16(uint16_t v, const volatile uintreg_t a)
{
  __asm__ __volatile__("sh %0, 0(%1)" : : "r" (v), "r" (a));
}

static inline uint32_t getreg32(const volatile uintreg_t a)
{
  uint32_t v;
  __asm__ __volatile__("lw %0, 0(%1)" : "=r" (v) : "r" (a));
  return v;
}

static inline void putreg32(uint32_t v, const volatile uintreg_t a)
{
  __asm__ __volatile__("sw %0, 0(%1)" : : "r" (v), "r" (a));
}

static inline uint64_t getreg64(const volatile uintreg_t a)
{
  uint64_t v;
  __asm__ __volatile__("ld %0, 0(%1)" : "=r" (v) : "r" (a));
  return v;
}

static inline void putreg64(uint64_t v, const volatile uintreg_t a)
{
  __asm__ __volatile__("sd %0, 0(%1)" : : "r" (v), "r" (a));
}

#define READ_CSR(reg) \
  ({ \
     uintreg_t __regval; \
     __asm__ __volatile__("csrr %0, " __STR(reg) : "=r"(__regval)); \
     __regval; \
  })

#define READ_AND_SET_CSR(reg, bits) \
  ({ \
     uintreg_t __regval; \
     __asm__ __volatile__("csrrs %0, " __STR(reg) ", %1": "=r"(__regval) : "rK"(bits)); \
     __regval; \
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

#define SWAP_CSR(reg, val) \
  ({ \
     uintptr_t regval; \
     __asm__ __volatile__("csrrw %0, " __STR(reg) ", %1" : "=r"(regval) \
                                                         : "rK"(val)); \
     regval; \
  })

#define WRITE_INDIRECT_CSR_REG0(reg, val) \
  ({ \
     WRITE_CSR(CSR_ISELECT, reg); \
     WRITE_CSR(CSR_IREG, val); \
  })

#define READ_INDIRECT_CSR_REG0(reg, val) \
  ({ \
     WRITE_CSR(CSR_ISELECT, reg); \
     READ_CSR(CSR_IREG, val); \
  })

#define SET_INDIRECT_CSR_REG0(reg, val) \
  ({ \
     WRITE_CSR(CSR_ISELECT, reg); \
     SET_CSR(CSR_IREG, val); \
  })

#define CLEAR_INDIRECT_CSR_REG0(reg, val) \
  ({ \
     WRITE_CSR(CSR_ISELECT, reg); \
     CLEAR_CSR(CSR_IREG, val); \
  })

#define riscv_append_pmp_region(a, b, s) \
  riscv_config_pmp_region(riscv_next_free_pmp_region(), a, b, s)

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

void modifyreg32(uintreg_t addr, uint32_t clearbits, uint32_t setbits);

/* Memory allocation ********************************************************/

#if CONFIG_MM_REGIONS > 1
void riscv_addregion(void);
#else
#  define riscv_addregion()
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
void riscv_savefpu(uintreg_t *regs, uintreg_t *fregs);
void riscv_restorefpu(uintreg_t *regs, uintreg_t *fregs);

/* Get FPU register save area */

static inline uintreg_t *riscv_fpuregs(struct tcb_s *tcb)
{
#ifdef CONFIG_ARCH_LAZYFPU
  /* With lazy FPU the registers are simply in tcb */

  return tcb->xcp.fregs;
#else
  /* Otherwise they are after the integer registers */

  return (uintreg_t *)((uintptr_t)tcb->xcp.regs + INT_XCPT_SIZE);
#endif
}
#else
#  define riscv_fpuconfig()
#  define riscv_savefpu(regs, fregs)
#  define riscv_restorefpu(regs, fregs)
#  define riscv_fpuregs(tcb)
#endif

#ifdef CONFIG_ARCH_RV_ISA_V
void riscv_vpuconfig(void);
void riscv_savevpu(uintptr_t *regs, uintptr_t *vregs);
void riscv_restorevpu(uintptr_t *regs, uintptr_t *vregs);

/* Get VPU register save area */

static inline uintptr_t *riscv_vpuregs(struct tcb_s *tcb)
{
  return tcb->xcp.vregs;
}
#else
#  define riscv_vpuconfig()
#  define riscv_savevpu(regs, vregs)
#  define riscv_restorevpu(regs, vregs)
#  define riscv_vpuregs(tcb)
#endif

/* Save / restore context of task */

static inline void riscv_savecontext(struct tcb_s *tcb)
{
#ifdef CONFIG_ARCH_FPU
  /* Save current process FPU state to TCB */

  riscv_savefpu(tcb->xcp.regs, riscv_fpuregs(tcb));
#endif

#ifdef CONFIG_ARCH_RV_ISA_V
  /* Save current process VPU state to TCB */

  riscv_savevpu(tcb->xcp.regs, riscv_vpuregs(tcb));
#endif
}

static inline void riscv_restorecontext(struct tcb_s *tcb)
{
#ifdef CONFIG_ARCH_FPU
  /* Restore FPU state for next process */

  riscv_restorefpu(tcb->xcp.regs, riscv_fpuregs(tcb));
#endif

#ifdef CONFIG_ARCH_RV_ISA_V
  /* Restore VPU state for next process */

  riscv_restorevpu(tcb->xcp.regs, riscv_vpuregs(tcb));
#endif
}

#ifdef CONFIG_ARCH_RISCV_INTXCPT_EXTENSIONS
void riscv_initial_extctx_state(struct tcb_s *tcb);
#endif

/* RISC-V PMP Config ********************************************************/

int riscv_config_pmp_region(uintptr_t region, uintptr_t attr,
                            uintptr_t base, uintptr_t size);

int riscv_check_pmp_access(uintptr_t attr, uintptr_t base, uintptr_t size);
int riscv_configured_pmp_regions(void);
int riscv_next_free_pmp_region(void);

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
#  define riscv_netinitialize()
#endif

/* Exception Handler ********************************************************/

uintreg_t *riscv_doirq(int irq, uintreg_t *regs);
int riscv_exception(int mcause, void *regs, void *args);
int riscv_fillpage(int mcause, void *regs, void *args);
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
void *riscv_perform_syscall(uintreg_t *regs);
#endif

/* Context switching via system calls ***************************************/

/* SYS call 1:
 *
 * void riscv_fullcontextrestore(struct tcb_s *next) noreturn_function;
 */

#define riscv_fullcontextrestore(next) \
  sys_call1(SYS_restore_context, (uintptr_t)next)

/* SYS call 2:
 *
 * riscv_switchcontext(struct tcb_s *prev, struct tcb_s *next);
 */

#define riscv_switchcontext(prev, next) \
  sys_call2(SYS_switch_context, (uintptr_t)prev, (uintptr_t)next)

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_RISCV_SRC_COMMON_RISCV_INTERNAL_H */
