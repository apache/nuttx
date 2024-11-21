/****************************************************************************
 * arch/arm64/src/common/arm64_pmuv3.h
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

#ifndef __ARCH_ARM64_SRC_COMMON_ARM64_PMUV3_H
#define __ARCH_ARM64_SRC_COMMON_ARM64_PMUV3_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <debug.h>
#include "arm64_arch.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ID_AA64DFR0_EL1.PMUVer */

#define ID_AA64DFR0_EL1_PMUVER_SHIFT    0x8
#define ID_AA64DFR0_EL1_PMUVER_NI       0x0
#define ID_AA64DFR0_EL1_PMUVER_V3P4     0x5
#define ID_AA64DFR0_EL1_PMUVER_V3P5     0x6
#define ID_AA64DFR0_EL1_PMUVER_IMP_DEF  0xf

#define PMU_MAX_COUNTERS                32
#define PMU_COUNTER_MASK                (PMU_MAX_COUNTERS - 1)

#define PMU_IDX_CYCLE_COUNTER           0
#define PMU_IDX_COUNTER0                1
#define PMU_IDX_CYCLE_COUNTER_USER      32

#define PMU_IDX_TO_COUNTER(x) \
        (((x) - PMU_IDX_COUNTER0) & PMU_COUNTER_MASK)

/* Common architectural and microarchitectural events */

#define PMUV3_PERFCTR_SW_INCR                       0x0000
#define PMUV3_PERFCTR_L1I_CACHE_REFILL              0x0001
#define PMUV3_PERFCTR_L1I_TLB_REFILL                0x0002
#define PMUV3_PERFCTR_L1D_CACHE_REFILL              0x0003
#define PMUV3_PERFCTR_L1D_CACHE                     0x0004
#define PMUV3_PERFCTR_L1D_TLB_REFILL                0x0005
#define PMUV3_PERFCTR_LD_RETIRED                    0x0006
#define PMUV3_PERFCTR_ST_RETIRED                    0x0007
#define PMUV3_PERFCTR_INST_RETIRED                  0x0008
#define PMUV3_PERFCTR_EXC_TAKEN                     0x0009
#define PMUV3_PERFCTR_EXC_RETURN                    0x000A
#define PMUV3_PERFCTR_CID_WRITE_RETIRED             0x000B
#define PMUV3_PERFCTR_PC_WRITE_RETIRED              0x000C
#define PMUV3_PERFCTR_BR_IMMED_RETIRED              0x000D
#define PMUV3_PERFCTR_BR_RETURN_RETIRED             0x000E
#define PMUV3_PERFCTR_UNALIGNED_LDST_RETIRED        0x000F
#define PMUV3_PERFCTR_BR_MIS_PRED                   0x0010
#define PMUV3_PERFCTR_CPU_CYCLES                    0x0011
#define PMUV3_PERFCTR_BR_PRED                       0x0012
#define PMUV3_PERFCTR_MEM_ACCESS                    0x0013
#define PMUV3_PERFCTR_L1I_CACHE                     0x0014
#define PMUV3_PERFCTR_L1D_CACHE_WB                  0x0015
#define PMUV3_PERFCTR_L2D_CACHE                     0x0016
#define PMUV3_PERFCTR_L2D_CACHE_REFILL              0x0017
#define PMUV3_PERFCTR_L2D_CACHE_WB                  0x0018
#define PMUV3_PERFCTR_BUS_ACCESS                    0x0019
#define PMUV3_PERFCTR_MEMORY_ERROR                  0x001A
#define PMUV3_PERFCTR_INST_SPEC                     0x001B
#define PMUV3_PERFCTR_TTBR_WRITE_RETIRED            0x001C
#define PMUV3_PERFCTR_BUS_CYCLES                    0x001D
#define PMUV3_PERFCTR_CHAIN                         0x001E
#define PMUV3_PERFCTR_L1D_CACHE_ALLOCATE            0x001F
#define PMUV3_PERFCTR_L2D_CACHE_ALLOCATE            0x0020
#define PMUV3_PERFCTR_BR_RETIRED                    0x0021
#define PMUV3_PERFCTR_BR_MIS_PRED_RETIRED           0x0022
#define PMUV3_PERFCTR_STALL_FRONTEND                0x0023
#define PMUV3_PERFCTR_STALL_BACKEND                 0x0024
#define PMUV3_PERFCTR_L1D_TLB                       0x0025
#define PMUV3_PERFCTR_L1I_TLB                       0x0026
#define PMUV3_PERFCTR_L2I_CACHE                     0x0027
#define PMUV3_PERFCTR_L2I_CACHE_REFILL              0x0028
#define PMUV3_PERFCTR_L3D_CACHE_ALLOCATE            0x0029
#define PMUV3_PERFCTR_L3D_CACHE_REFILL              0x002A
#define PMUV3_PERFCTR_L3D_CACHE                     0x002B
#define PMUV3_PERFCTR_L3D_CACHE_WB                  0x002C
#define PMUV3_PERFCTR_L2D_TLB_REFILL                0x002D
#define PMUV3_PERFCTR_L2I_TLB_REFILL                0x002E
#define PMUV3_PERFCTR_L2D_TLB                       0x002F
#define PMUV3_PERFCTR_L2I_TLB                       0x0030
#define PMUV3_PERFCTR_REMOTE_ACCESS                 0x0031
#define PMUV3_PERFCTR_LL_CACHE                      0x0032
#define PMUV3_PERFCTR_LL_CACHE_MISS                 0x0033
#define PMUV3_PERFCTR_DTLB_WALK                     0x0034
#define PMUV3_PERFCTR_ITLB_WALK                     0x0035
#define PMUV3_PERFCTR_LL_CACHE_RD                   0x0036
#define PMUV3_PERFCTR_LL_CACHE_MISS_RD              0x0037
#define PMUV3_PERFCTR_REMOTE_ACCESS_RD              0x0038
#define PMUV3_PERFCTR_L1D_CACHE_LMISS_RD            0x0039
#define PMUV3_PERFCTR_OP_RETIRED                    0x003A
#define PMUV3_PERFCTR_OP_SPEC                       0x003B
#define PMUV3_PERFCTR_STALL                         0x003C
#define PMUV3_PERFCTR_STALL_SLOT_BACKEND            0x003D
#define PMUV3_PERFCTR_STALL_SLOT_FRONTEND           0x003E
#define PMUV3_PERFCTR_STALL_SLOT                    0x003F

/* IMPLEMENTATION DEFINED events */

/* ARM-recommended common architectural and microarchitectural events */

#define PMUV3_IMPDEF_PERFCTR_L1D_CACHE_RD           0x0040
#define PMUV3_IMPDEF_PERFCTR_L1D_CACHE_WR           0x0041
#define PMUV3_IMPDEF_PERFCTR_L1D_CACHE_REFILL_RD    0x0042
#define PMUV3_IMPDEF_PERFCTR_L1D_CACHE_REFILL_WR    0x0043
#define PMUV3_IMPDEF_PERFCTR_L1D_CACHE_REFILL_INNER 0x0044
#define PMUV3_IMPDEF_PERFCTR_L1D_CACHE_REFILL_OUTER 0x0045
#define PMUV3_IMPDEF_PERFCTR_L1D_CACHE_WB_VICTIM    0x0046
#define PMUV3_IMPDEF_PERFCTR_L1D_CACHE_WB_CLEAN     0x0047
#define PMUV3_IMPDEF_PERFCTR_L1D_CACHE_INVAL        0x0048

#define PMUV3_IMPDEF_PERFCTR_L1D_TLB_REFILL_RD      0x004C
#define PMUV3_IMPDEF_PERFCTR_L1D_TLB_REFILL_WR      0x004D
#define PMUV3_IMPDEF_PERFCTR_L1D_TLB_RD             0x004E
#define PMUV3_IMPDEF_PERFCTR_L1D_TLB_WR             0x004F
#define PMUV3_IMPDEF_PERFCTR_L2D_CACHE_RD           0x0050
#define PMUV3_IMPDEF_PERFCTR_L2D_CACHE_WR           0x0051
#define PMUV3_IMPDEF_PERFCTR_L2D_CACHE_REFILL_RD    0x0052
#define PMUV3_IMPDEF_PERFCTR_L2D_CACHE_REFILL_WR    0x0053

#define PMUV3_IMPDEF_PERFCTR_L2D_CACHE_WB_VICTIM    0x0056
#define PMUV3_IMPDEF_PERFCTR_L2D_CACHE_WB_CLEAN     0x0057
#define PMUV3_IMPDEF_PERFCTR_L2D_CACHE_INVAL        0x0058

#define PMUV3_IMPDEF_PERFCTR_L2D_TLB_REFILL_RD      0x005C
#define PMUV3_IMPDEF_PERFCTR_L2D_TLB_REFILL_WR      0x005D
#define PMUV3_IMPDEF_PERFCTR_L2D_TLB_RD             0x005E
#define PMUV3_IMPDEF_PERFCTR_L2D_TLB_WR             0x005F
#define PMUV3_IMPDEF_PERFCTR_BUS_ACCESS_RD          0x0060
#define PMUV3_IMPDEF_PERFCTR_BUS_ACCESS_WR          0x0061
#define PMUV3_IMPDEF_PERFCTR_BUS_ACCESS_SHARED      0x0062
#define PMUV3_IMPDEF_PERFCTR_BUS_ACCESS_NOT_SHARED  0x0063
#define PMUV3_IMPDEF_PERFCTR_BUS_ACCESS_NORMAL      0x0064
#define PMUV3_IMPDEF_PERFCTR_BUS_ACCESS_PERIPH      0x0065
#define PMUV3_IMPDEF_PERFCTR_MEM_ACCESS_RD          0x0066
#define PMUV3_IMPDEF_PERFCTR_MEM_ACCESS_WR          0x0067
#define PMUV3_IMPDEF_PERFCTR_UNALIGNED_LD_SPEC      0x0068
#define PMUV3_IMPDEF_PERFCTR_UNALIGNED_ST_SPEC      0x0069
#define PMUV3_IMPDEF_PERFCTR_UNALIGNED_LDST_SPEC    0x006A

#define PMUV3_IMPDEF_PERFCTR_LDREX_SPEC             0x006C
#define PMUV3_IMPDEF_PERFCTR_STREX_PASS_SPEC        0x006D
#define PMUV3_IMPDEF_PERFCTR_STREX_FAIL_SPEC        0x006E
#define PMUV3_IMPDEF_PERFCTR_STREX_SPEC             0x006F
#define PMUV3_IMPDEF_PERFCTR_LD_SPEC                0x0070
#define PMUV3_IMPDEF_PERFCTR_ST_SPEC                0x0071
#define PMUV3_IMPDEF_PERFCTR_LDST_SPEC              0x0072
#define PMUV3_IMPDEF_PERFCTR_DP_SPEC                0x0073
#define PMUV3_IMPDEF_PERFCTR_ASE_SPEC               0x0074
#define PMUV3_IMPDEF_PERFCTR_VFP_SPEC               0x0075
#define PMUV3_IMPDEF_PERFCTR_PC_WRITE_SPEC          0x0076
#define PMUV3_IMPDEF_PERFCTR_CRYPTO_SPEC            0x0077
#define PMUV3_IMPDEF_PERFCTR_BR_IMMED_SPEC          0x0078
#define PMUV3_IMPDEF_PERFCTR_BR_RETURN_SPEC         0x0079
#define PMUV3_IMPDEF_PERFCTR_BR_INDIRECT_SPEC       0x007A

#define PMUV3_IMPDEF_PERFCTR_ISB_SPEC               0x007C
#define PMUV3_IMPDEF_PERFCTR_DSB_SPEC               0x007D
#define PMUV3_IMPDEF_PERFCTR_DMB_SPEC               0x007E

#define PMUV3_IMPDEF_PERFCTR_EXC_UNDEF              0x0081
#define PMUV3_IMPDEF_PERFCTR_EXC_SVC                0x0082
#define PMUV3_IMPDEF_PERFCTR_EXC_PABORT             0x0083
#define PMUV3_IMPDEF_PERFCTR_EXC_DABORT             0x0084

#define PMUV3_IMPDEF_PERFCTR_EXC_IRQ                0x0086
#define PMUV3_IMPDEF_PERFCTR_EXC_FIQ                0x0087
#define PMUV3_IMPDEF_PERFCTR_EXC_SMC                0x0088

#define PMUV3_IMPDEF_PERFCTR_EXC_HVC                0x008A
#define PMUV3_IMPDEF_PERFCTR_EXC_TRAP_PABORT        0x008B
#define PMUV3_IMPDEF_PERFCTR_EXC_TRAP_DABORT        0x008C
#define PMUV3_IMPDEF_PERFCTR_EXC_TRAP_OTHER         0x008D
#define PMUV3_IMPDEF_PERFCTR_EXC_TRAP_IRQ           0x008E
#define PMUV3_IMPDEF_PERFCTR_EXC_TRAP_FIQ           0x008F
#define PMUV3_IMPDEF_PERFCTR_RC_LD_SPEC             0x0090
#define PMUV3_IMPDEF_PERFCTR_RC_ST_SPEC             0x0091

#define PMUV3_IMPDEF_PERFCTR_L3D_CACHE_RD           0x00A0
#define PMUV3_IMPDEF_PERFCTR_L3D_CACHE_WR           0x00A1
#define PMUV3_IMPDEF_PERFCTR_L3D_CACHE_REFILL_RD    0x00A2
#define PMUV3_IMPDEF_PERFCTR_L3D_CACHE_REFILL_WR    0x00A3

#define PMUV3_IMPDEF_PERFCTR_L3D_CACHE_WB_VICTIM    0x00A6
#define PMUV3_IMPDEF_PERFCTR_L3D_CACHE_WB_CLEAN     0x00A7
#define PMUV3_IMPDEF_PERFCTR_L3D_CACHE_INVAL        0x00A8

/* ARMv8 Cortex-A53 specific event types. */

#define ARMA53_IMPDEF_PERFCTR_PREF_LINEFILL         0x00C2

/* ARMv8 Cortex-R82 specific event types. */

#define ARMR82_IMPDEF_L2D_CACHE_REFILL_PREFETCH     0x00C1
#define ARMR82_IMPDEF_L1D_CACHE_REFILL_PREFETCH     0x00C2
#define ARMR82_IMPDEF_L2D_WS_MODE                   0x00C3
#define ARMR82_IMPDEF_L1D_WS_MODE_ENTRY             0x00C4
#define ARMR82_IMPDEF_L1D_WS_MODE                   0x00C5

/* Extension microarchitectural events */

#define PMUV3_PERFCTR_SAMPLE_POP                    0x4000
#define PMUV3_PERFCTR_SAMPLE_FEED                   0x4001
#define PMUV3_PERFCTR_SAMPLE_FILTRATE               0x4002
#define PMUV3_PERFCTR_SAMPLE_COLLISION              0x4003

/* PMCR: Config reg */

#define PMU_PMCR_E                (1 << 0)
#define PMU_PMCR_P                (1 << 1)
#define PMU_PMCR_C                (1 << 2)
#define PMU_PMCR_D                (1 << 3)
#define PMU_PMCR_X                (1 << 4)
#define PMU_PMCR_DP               (1 << 5)
#define PMU_PMCR_LC               (1 << 6)
#define PMU_PMCR_LP               (1 << 7)
#define PMU_PMCR_N_SHIFT          11
#define PMU_PMCR_N_MASK           0x1f
#define PMU_PMCR_MASK             0xff

/* PMXEVTYPER: Event selection reg */

#define PMU_EVTYPE_MASK           0xc800ffff
#define PMU_EVTYPE_EVENT          0xffff

/* PMEVTYPER<n>_EL0 or PMCCFILTR_EL0: Event filters. */

#define PMU_EXCLUDE_EL1           (1U << 31)
#define PMU_EXCLUDE_EL0           (1U << 30)
#define PMU_INCLUDE_EL2           (1U << 27)

/* PMUSERENR: User enable reg */

#define PMU_USERENR_MASK          0xf
#define PMU_USERENR_EN            (1 << 0)
#define PMU_USERENR_SW            (1 << 1)
#define PMU_USERENR_CR            (1 << 2)
#define PMU_USERENR_ER            (1 << 3)

#define PMEVN_CASE(n, case_macro)                     \
                   case n: case_macro(n); break

#define PMEVN_SWITCH(x, case_macro)                   \
  do                                                  \
    {                                                 \
      switch (x)                                      \
        {                                             \
          PMEVN_CASE(0,  case_macro);                 \
          PMEVN_CASE(1,  case_macro);                 \
          PMEVN_CASE(2,  case_macro);                 \
          PMEVN_CASE(3,  case_macro);                 \
          PMEVN_CASE(4,  case_macro);                 \
          PMEVN_CASE(5,  case_macro);                 \
          PMEVN_CASE(6,  case_macro);                 \
          PMEVN_CASE(7,  case_macro);                 \
          PMEVN_CASE(8,  case_macro);                 \
          PMEVN_CASE(9,  case_macro);                 \
          PMEVN_CASE(10, case_macro);                 \
          PMEVN_CASE(11, case_macro);                 \
          PMEVN_CASE(12, case_macro);                 \
          PMEVN_CASE(13, case_macro);                 \
          PMEVN_CASE(14, case_macro);                 \
          PMEVN_CASE(15, case_macro);                 \
          PMEVN_CASE(16, case_macro);                 \
          PMEVN_CASE(17, case_macro);                 \
          PMEVN_CASE(18, case_macro);                 \
          PMEVN_CASE(19, case_macro);                 \
          PMEVN_CASE(20, case_macro);                 \
          PMEVN_CASE(21, case_macro);                 \
          PMEVN_CASE(22, case_macro);                 \
          PMEVN_CASE(23, case_macro);                 \
          PMEVN_CASE(24, case_macro);                 \
          PMEVN_CASE(25, case_macro);                 \
          PMEVN_CASE(26, case_macro);                 \
          PMEVN_CASE(27, case_macro);                 \
          PMEVN_CASE(28, case_macro);                 \
          PMEVN_CASE(29, case_macro);                 \
          PMEVN_CASE(30, case_macro);                 \
          default: _warn("Invalid PMEV* index\n");    \
        }                                             \
    }                                                 \
  while (0)

#define RETURN_READ_PMEVCNTRN(n) \
        return read_sysreg(pmevcntr##n##_el0)

#define WRITE_PMEVCNTRN(n) \
        write_sysreg(val, pmevcntr##n##_el0)

#define WRITE_PMEVTYPERN(n) \
        write_sysreg(val, pmevtyper##n##_el0)

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline uint64_t read_pmevcntrn(int n)
{
  PMEVN_SWITCH(n, RETURN_READ_PMEVCNTRN);
  return 0;
}

static inline void write_pmevcntrn(int n, unsigned long val)
{
  PMEVN_SWITCH(n, WRITE_PMEVCNTRN);
}

static inline void write_pmevtypern(int n, unsigned long val)
{
  PMEVN_SWITCH(n, WRITE_PMEVTYPERN);
}

static inline uint32_t read_pmuver(void)
{
  uint64_t ver = read_sysreg(id_aa64dfr0_el1);

  ver = (uint64_t)(ver << (64 - 4 - ID_AA64DFR0_EL1_PMUVER_SHIFT))
                          >> (64 - 4);

  return (uint32_t)ver;
}

static inline void write_pmcr(uint32_t val)
{
  write_sysreg(val, pmcr_el0);
}

static inline uint32_t read_pmcr(void)
{
  return read_sysreg(pmcr_el0);
}

static inline void write_pmselr(uint32_t val)
{
  write_sysreg(val, pmselr_el0);
}

static inline uint32_t read_pmselr(void)
{
  return read_sysreg(pmselr_el0);
}

static inline void write_pmccntr(uint64_t val)
{
  write_sysreg(val, pmccntr_el0);
}

static inline uint64_t read_pmccntr(void)
{
  return read_sysreg(pmccntr_el0);
}

static inline void write_pmxevcntr(uint32_t val)
{
  write_sysreg(val, pmxevcntr_el0);
}

static inline uint32_t read_pmxevcntr(void)
{
  return read_sysreg(pmxevcntr_el0);
}

static inline void write_pmxevtyper(uint32_t val)
{
  write_sysreg(val, pmxevtyper_el0);
}

static inline void write_pmcntenset(uint32_t val)
{
  write_sysreg(val, pmcntenset_el0);
}

static inline void write_pmcntenclr(uint32_t val)
{
  write_sysreg(val, pmcntenclr_el0);
}

static inline void write_pmintenset(uint32_t val)
{
  write_sysreg(val, pmintenset_el1);
}

static inline void write_pmintenclr(uint32_t val)
{
  write_sysreg(val, pmintenclr_el1);
}

static inline void write_pmccfiltr(uint32_t val)
{
  write_sysreg(val, pmccfiltr_el0);
}

static inline void write_pmovsclr(uint32_t val)
{
  write_sysreg(val, pmovsclr_el0);
}

static inline uint32_t read_pmovsclr(void)
{
  return read_sysreg(pmovsclr_el0);
}

static inline void write_pmuserenr(uint32_t val)
{
  write_sysreg(val, pmuserenr_el0);
}

static inline uint32_t read_pmceid0(void)
{
  return read_sysreg(pmceid0_el0);
}

static inline uint32_t read_pmceid1(void)
{
  return read_sysreg(pmceid1_el0);
}

static inline bool pmuv3_implemented(int pmuver)
{
  return !(pmuver == ID_AA64DFR0_EL1_PMUVER_IMP_DEF ||
           pmuver == ID_AA64DFR0_EL1_PMUVER_NI);
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int arm64_pmu_initialize(void);

#endif /* __ARCH_ARM64_SRC_COMMON_ARM64_PMUV3_H */
