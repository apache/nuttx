/****************************************************************************
 * arch/arm64/src/common/arm64_arch.h
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

#ifndef ___ARCH_ARM64_SRC_COMMON_ARM64_ARCH_H
#define ___ARCH_ARM64_SRC_COMMON_ARM64_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Unsigned integer with bit position n set (signed in
 * assembly language).
 */
#ifndef __ASSEMBLY__
  #include <stdint.h>
#endif

#include <sys/param.h>

#include "barriers.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BIT(n)          ((1UL) << (n))
#define BIT64(n)        ((1ULL) << (n))

/* Bit mask with bits 0 through n-1 (inclusive) set,
 * or 0 if n is 0.
 */
#define BIT_MASK(n)     (BIT(n) - 1)
#define BIT64_MASK(n)   (BIT64(n) - 1ULL)

#define DAIFSET_FIQ_BIT     BIT(0)
#define DAIFSET_IRQ_BIT     BIT(1)
#define DAIFSET_ABT_BIT     BIT(2)
#define DAIFSET_DBG_BIT     BIT(3)

#define DAIFCLR_FIQ_BIT     BIT(0)
#define DAIFCLR_IRQ_BIT     BIT(1)
#define DAIFCLR_ABT_BIT     BIT(2)
#define DAIFCLR_DBG_BIT     BIT(3)

#define DAIF_FIQ_BIT        BIT(6)
#define DAIF_IRQ_BIT        BIT(7)
#define DAIF_ABT_BIT        BIT(8)
#define DAIF_DBG_BIT        BIT(9)

#define DAIF_MASK           (0xf << 6)

/* Arm® Architecture Registers Armv8, for Armv8-A architecture profile
 * ( DDI 0595, ID121321 ), defined:
 *
 * SCTLR_EL1: System Control Register (EL1)
 * SCTLR_EL2: System Control Register (EL2)
 * SCTLR_EL3: System Control Register (EL3)
 *
 */

#define SCTLR_EL3_RES1      (BIT(29) | BIT(28) | BIT(23) | \
                             BIT(22) | BIT(18) | BIT(16) | \
                             BIT(11) | BIT(5)  | BIT(4))

#define SCTLR_EL2_RES1      (BIT(29) | BIT(28) | BIT(23) | \
                             BIT(22) | BIT(18) | BIT(16) | \
                             BIT(11) | BIT(5)  | BIT(4))

#define SCTLR_EL1_RES1      (BIT(29) | BIT(28) | BIT(23) | \
                             BIT(22) | BIT(20) | BIT(11))

#define SCTLR_M_BIT         BIT(0)
#define SCTLR_A_BIT         BIT(1)
#define SCTLR_C_BIT         BIT(2)
#define SCTLR_SA_BIT        BIT(3)
#define SCTLR_I_BIT         BIT(12)

/* SPSR M[3:0] define
 *
 * Arm® Architecture Registers Armv8, for Armv8-A architecture profile
 * ( DDI 0595, ID121321 ), defined:
 * SPSR_EL1: Saved Program Status Register (EL1)
 * SPSR_EL2: Saved Program Status Register (EL2)
 * SPSR_EL3: Saved Program Status Register (EL3)
 *
 * reference to Programmer’s Guide for ARMv8-A
 * (ARM DEN0024A, ID050815 ), 4.1.2 Stack pointer
 *
 * The T suffix, indicates use of the SP_EL0 stack pointer.
 * The H suffix, indicates use of the SP_ELx stack pointer.
 *
 */

#define SPSR_DAIF_SHIFT     (6)
#define SPSR_DAIF_MASK      (0xf << SPSR_DAIF_SHIFT)

#define SPSR_MODE_EL0T      (0x0)
#define SPSR_MODE_EL1T      (0x4)
#define SPSR_MODE_EL1H      (0x5)
#define SPSR_MODE_EL2T      (0x8)
#define SPSR_MODE_EL2H      (0x9)
#define SPSR_MODE_MASK      (0xf)

/* CurrentEL: Current Exception Level */

#define MODE_EL_SHIFT       (0x2)
#define MODE_EL_MASK        (0x3)

#define MODE_EL3            (0x3)
#define MODE_EL2            (0x2)
#define MODE_EL1            (0x1)
#define MODE_EL0            (0x0)

/* struct arm64_boot_params member offset for assembly code
 * struct is defined at arm64_cpustart.c
 */

#define BOOT_PARAM_MPID     0
#define BOOT_PARAM_SP       8

#ifndef __ASSEMBLY__

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STRINGIFY(x)    #x

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x)   (sizeof(x) / sizeof((x)[0]))
#endif

#define GET_EL(mode)  (((mode) >> MODE_EL_SHIFT) & MODE_EL_MASK)

/* MPIDR_EL1, Multiprocessor Affinity Register */

#define MPIDR_AFFLVL_MASK   (0xff)

#define MPIDR_AFF0_SHIFT    (0)
#define MPIDR_AFF1_SHIFT    (8)
#define MPIDR_AFF2_SHIFT    (16)
#define MPIDR_AFF3_SHIFT    (32)

#define MPIDR_AFFLVL(mpidr, aff_level) \
  (((mpidr) >> MPIDR_AFF ## aff_level ## _SHIFT) & MPIDR_AFFLVL_MASK)

#define GET_MPIDR()             read_sysreg(mpidr_el1)
#define MPIDR_TO_CORE(mpidr)    MPIDR_AFFLVL((mpidr), 0)
#define IS_PRIMARY_CORE()       (!MPIDR_TO_CORE(GET_MPIDR()))

/* System register interface to GICv3 */

#define ICC_IGRPEN1_EL1             S3_0_C12_C12_7
#define ICC_SGI1R                   S3_0_C12_C11_5
#define ICC_SRE_EL1                 S3_0_C12_C12_5
#define ICC_SRE_EL2                 S3_4_C12_C9_5
#define ICC_SRE_EL3                 S3_6_C12_C12_5
#define ICC_CTLR_EL1                S3_0_C12_C12_4
#define ICC_CTLR_EL3                S3_6_C12_C12_4
#define ICC_PMR_EL1                 S3_0_C4_C6_0
#define ICC_RPR_EL1                 S3_0_C12_C11_3
#define ICC_IGRPEN1_EL3             S3_6_C12_C12_7
#define ICC_IGRPEN0_EL1             S3_0_C12_C12_6
#define ICC_HPPIR0_EL1              S3_0_C12_C8_2
#define ICC_HPPIR1_EL1              S3_0_C12_C12_2
#define ICC_IAR0_EL1                S3_0_C12_C8_0
#define ICC_IAR1_EL1                S3_0_C12_C12_0
#define ICC_EOIR0_EL1               S3_0_C12_C8_1
#define ICC_EOIR1_EL1               S3_0_C12_C12_1
#define ICC_SGI0R_EL1               S3_0_C12_C11_7

/* register constants */
#define ICC_SRE_ELX_SRE_BIT         BIT(0)
#define ICC_SRE_ELX_DFB_BIT         BIT(1)
#define ICC_SRE_ELX_DIB_BIT         BIT(2)
#define ICC_SRE_EL3_EN_BIT          BIT(3)

/* ICC SGI macros */
#define SGIR_TGT_MASK               (0xffff)
#define SGIR_AFF1_SHIFT             (16)
#define SGIR_AFF2_SHIFT             (32)
#define SGIR_AFF3_SHIFT             (48)
#define SGIR_AFF_MASK               (0xf)
#define SGIR_INTID_SHIFT            (24)
#define SGIR_INTID_MASK             (0xf)
#define SGIR_IRM_SHIFT              (40)
#define SGIR_IRM_MASK               (0x1)
#define SGIR_IRM_TO_AFF             (0)
#define SGIR_IRM_TO_ALL             (1)

#define GICV3_SGIR_VALUE(_aff3, _aff2, _aff1, _intid, _irm, _tgt) \
  ((((uint64_t)(_aff3) & SGIR_AFF_MASK) << SGIR_AFF3_SHIFT) |     \
   (((uint64_t)(_irm) & SGIR_IRM_MASK) << SGIR_IRM_SHIFT) |       \
   (((uint64_t)(_aff2) & SGIR_AFF_MASK) << SGIR_AFF2_SHIFT) |     \
   (((_intid) & SGIR_INTID_MASK) << SGIR_INTID_SHIFT) |           \
   (((_aff1) & SGIR_AFF_MASK) << SGIR_AFF1_SHIFT) |               \
   ((_tgt) & SGIR_TGT_MASK))

/* CPTR_EL2, Architectural Feature Trap Register (EL2) */

#define CPTR_EZ_BIT                 BIT(8)
#define CPTR_TFP_BIT                BIT(10)
#define CPTR_TTA_BIT                BIT(20)
#define CPTR_TCPAC_BIT              BIT(31)
#define CPTR_EL2_RES1               BIT(13) | BIT(12) | BIT(9) | (0xff)

/* CPACR_EL1, Architectural Feature Access Control Register */
#define CPACR_EL1_FPEN_NOTRAP       (0x3 << 20)

/* SCR_EL3, Secure Configuration Register */
#define SCR_NS_BIT                  BIT(0)
#define SCR_IRQ_BIT                 BIT(1)
#define SCR_FIQ_BIT                 BIT(2)
#define SCR_EA_BIT                  BIT(3)
#define SCR_SMD_BIT                 BIT(7)
#define SCR_HCE_BIT                 BIT(8)
#define SCR_RW_BIT                  BIT(10)
#define SCR_ST_BIT                  BIT(11)
#define SCR_RES1                    (BIT(4) | BIT(5))

/* HCR_EL2, Hypervisor Configuration Register */

#define HCR_FMO_BIT                 BIT(3)
#define HCR_IMO_BIT                 BIT(4)
#define HCR_AMO_BIT                 BIT(5)
#define HCR_RW_BIT                  BIT(31)

/* CNTHCTL_EL2 bits definitions */

#define CNTHCTL_EL2_EL1PCEN_EN      BIT(1)
#define CNTHCTL_EL2_EL1PCTEN_EN     BIT(0)

/* CNTV_CVAL, Counter-timer Virtual Timer CompareValue register
 * CNTV_CTL, Counter-timer Virtual Timer Control register
 */

#define CNTV_CTL_ENABLE_BIT         BIT(0)
#define CNTV_CTL_IMASK_BIT          BIT(1)

/*  Maximum numbers of translation tables
 *      This option specifies the maximum numbers of translation tables
 *  excluding the base translation table. Based on this, translation
 *  tables are allocated at compile time and used at runtime as needed.
 *  If the runtime need exceeds preallocated numbers of translation
 *  tables, it will result in assert. Number of translation tables
 *  required is decided based on how many discrete memory regions
 *  (both normal and device memory) are present on given platform and
 *  how much granularity is required while assigning attributes
 *  to these memory regions.
 */

#define CONFIG_MAX_XLAT_TABLES      7

/* Virtual address space size
 * Allows choosing one of multiple possible virtual address
 * space sizes. The level of translation table is determined by
 * a combination of page size and virtual address space size.
 *
 * The choice could be: 32, 36, 42, 48
 */

#define CONFIG_ARM64_VA_BITS        36

/* Physical address space size
 * Choose the maximum physical address range that the kernel will support.
 *
 * The choice could be: 32, 36, 42, 48
 */

#define CONFIG_ARM64_PA_BITS        36

#define L1_CACHE_SHIFT              (6)
#define L1_CACHE_BYTES              BIT(L1_CACHE_SHIFT)

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

#ifdef CONFIG_ARCH_FPU

/****************************************************************************
 * armv8 fpu registers and context
 ****************************************************************************/

struct fpu_reg
{
  __int128 q[32];
  uint32_t fpsr;
  uint32_t fpcr;
  uint64_t fpu_trap;
};

#endif

/****************************************************************************
 * Registers and exception context
 ****************************************************************************/

struct regs_context
{
  uint64_t  regs[31];  /* x0~x30 */
  uint64_t  sp_elx;
  uint64_t  elr;
  uint64_t  spsr;
  uint64_t  sp_el0;
  uint64_t  exe_depth;
  uint64_t  tpidr_el0;
  uint64_t  tpidr_el1;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:
 *   getreg8/16/32/64
 *   putreg8/16/32/64
 *
 * Description:
 *   We need to use explicit assembler instruction there, because with
 *   classic "volatile pointer" approach compiler might generate
 *   instruction with immediate value like
 *
 *   str     w4, [x1], #4
 *
 *   Such instructions produce invalid syndrome in HSR register,
 *   so hypervisor can't emulate MMIO  when it traps memory access.
 *
 ****************************************************************************/

static inline uint8_t getreg8(unsigned long addr)
{
  uint8_t val;

  __asm__ volatile ("ldrb %w0, [%1]" : "=r" (val) : "r" (addr));

  ARM64_DMB();
  return val;
}

static inline void putreg8(uint8_t data, unsigned long addr)
{
  ARM64_DMB();
  __asm__ volatile ("strb %w0, [%1]" : : "r" (data), "r" (addr));
}

static inline uint16_t getreg16(unsigned long addr)
{
  uint16_t val;

  __asm__ volatile ("ldrh %w0, [%1]" : "=r" (val) : "r" (addr));

  ARM64_DMB();
  return val;
}

static inline void putreg16(uint16_t data, unsigned long addr)
{
  ARM64_DMB();
  __asm__ volatile ("strh %w0, [%1]" : : "r" (data), "r" (addr));
}

static inline uint32_t getreg32(unsigned long addr)
{
  uint32_t val;

  __asm__ volatile ("ldr %w0, [%1]" : "=r" (val) : "r" (addr));

  ARM64_DMB();
  return val;
}

static inline void putreg32(uint32_t data, unsigned long addr)
{
  ARM64_DMB();
  __asm__ volatile ("str %w0, [%1]" : : "r" (data), "r" (addr));
}

static inline uint64_t getreg64(unsigned long addr)
{
  uint64_t val;

  __asm__ volatile ("ldr %x0, [%1]" : "=r" (val) : "r" (addr));

  ARM64_DMB();
  return val;
}

static inline void putreg64(uint64_t data, unsigned long addr)
{
  ARM64_DMB();
  __asm__ volatile ("str %x0, [%1]" : : "r" (data), "r" (addr));
}

static inline void arch_nop(void)
{
  __asm__ volatile ("nop");
}

/****************************************************************************
 * Name:
 *   read_/write_/zero_ sysreg
 *
 * Description:
 *
 *   ARMv8 Architecture Registers access method
 *   All the macros need a memory clobber
 *
 ****************************************************************************/

#define read_sysreg(reg)                         \
  ({                                             \
    uint64_t __val;                              \
    __asm__ volatile ("mrs %0, " STRINGIFY(reg)  \
                    : "=r" (__val) :: "memory"); \
    __val;                                       \
  })

#define read_sysreg_dump(reg)                    \
  ({                                             \
    uint64_t __val;                              \
    __asm__ volatile ("mrs %0, " STRINGIFY(reg)  \
                    : "=r" (__val) :: "memory"); \
    sinfo("%s, regval=0x%llx\n",                 \
          STRINGIFY(reg), __val);                \
    __val;                                       \
  })

#define write_sysreg(__val, reg)                   \
  ({                                               \
    __asm__ volatile ("msr " STRINGIFY(reg) ", %0" \
                      : : "r" (__val) : "memory"); \
  })

#define zero_sysreg(reg)                            \
  ({                                                \
    __asm__ volatile ("msr " STRINGIFY(reg) ", xzr" \
                      ::: "memory");                \
  })

#define modreg8(v,m,a)  putreg8((getreg8(a) & ~(m)) | ((v) & (m)), (a))
#define modreg16(v,m,a) putreg16((getreg16(a) & ~(m)) | ((v) & (m)), (a))
#define modreg32(v,m,a) putreg32((getreg32(a) & ~(m)) | ((v) & (m)), (a))

/****************************************************************************
 * Name:
 *   arch_get_exception_depth
 *   arch_get_current_tcb
 *
 * Description:
 *   tpidrro_el0 is used to record exception depth, it's used for fpu trap
 * happened at exception context (like IRQ).
 *   tpidr_el1 is used to record TCB at present, it's used for fpu and task
 * switch propose
 *
 ****************************************************************************/

static inline int arch_get_exception_depth(void)
{
  return read_sysreg(tpidrro_el0);
}

static inline uint64_t arch_get_current_tcb(void)
{
  return read_sysreg(tpidr_el1);
}

void arch_cpu_idle(void);

/****************************************************************************
 * Name: arm64_cpu_disable
 *
 * Description:
 *   Called from CPU0 to make sure that all other CPUs are in the disabled
 *   state.  This is a formality because the other CPUs are actually running
 *   then we have probably already crashed.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
void arm64_cpu_disable(void);

#else
#  define arm64_cpu_disable()
#endif

/****************************************************************************
 * Name: arm64_cpu_enable
 *
 * Description:
 *   Called from CPU0 to enable all other CPUs.  The enabled CPUs will start
 *   execution at __cpuN_start and, after very low-level CPU initialization
 *   has been performed, will branch to arm_cpu_boot()
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
void arm64_cpu_enable(void);

#else
#  define arm64_cpu_enable()
#endif

#endif /* __ASSEMBLY__ */

#endif /* ___ARCH_ARM64_SRC_COMMON_ARM64_ARCH_H */
