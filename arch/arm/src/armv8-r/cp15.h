/****************************************************************************
 * arch/arm/src/armv8-r/cp15.h
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

/* References:
 *  "ARM Architecture Reference Manual, ARMv7-A and ARMv7-R edition",
 *   Copyright 1996-1998, 2000, 2004-2012 ARM.
 * All rights reserved. ARM DDI 0406C.c (ID051414)
 */

#ifndef __ARCH_ARM_SRC_ARMV8_R_CP15_H
#define __ARCH_ARM_SRC_ARMV8_R_CP15_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* System control register descriptions.
 *
 * CP15 registers are accessed with MRC and MCR instructions as follows:
 *
 *  MRC p15, <Op1>, <Rd>, <CRn>, <CRm>, <Op2> ; Read CP15 Register
 *  MCR p15, <Op1>, <Rd>, <CRn>, <CRm>, <Op2> ; Write CP15 Register
 *
 * Where
 *
 *   <Op1> is the Opcode_1 value for the register
 *   <Rd>  is a general purpose register
 *   <CRn> is the register number within CP15
 *   <CRm> is the operational register
 *   <Op2> is the Opcode_2 value for the register.
 *
 */

#ifdef __ASSEMBLY__
#  define _CP15(op1,rd,crn,crm,op2) p15, op1, rd, crn, crm, op2
#  define _CP15_64(op1,lo,hi,op2)   p15, op1, lo, hi, op2
#else
#  define _CP15(op1,rd,crn,crm,op2) "p15, " #op1 ", %0, " #crn ", " #crm ", " #op2
#  define _CP15_64(op1,lo,hi,op2)   "p15, " #op1 ", %Q0, %R0, " #op2
#endif

#define CP15_MIDR(r)       _CP15(0, r, c0, c0, 0)   /* Main ID Register */
#define CP15_CTR(r)        _CP15(0, r, c0, c0, 1)   /* Cache Type Register */
#define CP15_TCMTR(r)      _CP15(0, r, c0, c0, 2)   /* TCM Type Register */
#define CP15_MPUIR(r)      _CP15(0, r, c0, c0, 4)   /* MPU Type Register */
#define CP15_MPIDR(r)      _CP15(0, r, c0, c0, 5)   /* Multiprocessor Affinity Register */
#define CP15_REVIDR(r)     _CP15(0, r, c0, c0, 6)   /* Revision ID register (Cortex-A9) */
#define CP15_MID_PFR0(r)   _CP15(0, r, c0, c1, 0)   /* Processor Feature Register 0 */
#define CP15_MID_PFR1(r)   _CP15(0, r, c0, c1, 1)   /* Processor Feature Register 1 */
#define CP15_MID_DFR0(r)   _CP15(0, r, c0, c1, 2)   /* Debug Feature Register 0 */
#define CP15_MID_AFR0(r)   _CP15(0, r, c0, c1, 3)   /* Auxiliary Feature Register 0 (Cortex-A9) */
#define CP15_MID_MMFR0(r)  _CP15(0, r, c0, c1, 4)   /* Memory Model Features Register 0 */
#define CP15_MID_MMFR1(r)  _CP15(0, r, c0, c1, 5)   /* Memory Model Features Register 1 */
#define CP15_MID_MMFR2(r)  _CP15(0, r, c0, c1, 6)   /* Memory Model Features Register 2 */
#define CP15_MID_MMFR3(r)  _CP15(0, r, c0, c1, 7)   /* Memory Model Features Register 3 */
#define CP15_ID_ISAR0(r)   _CP15(0, r, c0, c2, 0)   /* Instruction Set Attributes Register 0 */
#define CP15_ID_ISAR1(r)   _CP15(0, r, c0, c2, 1)   /* Instruction Set Attributes Register 1 */
#define CP15_ID_ISAR2(r)   _CP15(0, r, c0, c2, 2)   /* Instruction Set Attributes Register 2 */
#define CP15_ID_ISAR3(r)   _CP15(0, r, c0, c2, 3)   /* Instruction Set Attributes Register 3 */
#define CP15_ID_ISAR4(r)   _CP15(0, r, c0, c2, 4)   /* Instruction Set Attributes Register 4 */
#define CP15_ID_ISAR5(r)   _CP15(0, r, c0, c2, 5)   /* Instruction Set Attributes Register 5 (Cortex-A5) */
#define CP15_CCSIDR(r)     _CP15(1, r, c0, c0, 0)   /* Cache Size Identification Register */
#define CP15_CLIDR(r)      _CP15(1, r, c0, c0, 1)   /* Cache Level ID Register */
#define CP15_AIDR(r)       _CP15(1, r, c0, c0, 7)   /* Auxiliary ID Register */
#define CP15_CSSELR(r)     _CP15(2, r, c0, c0, 0)   /* Cache Size Selection Register */

#define CP15_SCTLR(r)      _CP15(0, r, c1, c0, 0)   /* System Control Register */
#define CP15_ACTLR(r)      _CP15(0, r, c1, c0, 1)   /* Auxiliary Control Register */
#define CP15_CPACR(r)      _CP15(0, r, c1, c0, 2)   /* Coprocessor Access Control Register */

#define CP15_DFSR(r)       _CP15(0, r, c5, c0, 0)   /* Data Fault Status Register */
#define CP15_IFSR(r)       _CP15(0, r, c5, c0, 1)   /* Instruction Fault Status Register */
#define CP15_ADFSR(r)      _CP15(0, r, c5, c1, 0)   /* Auxiliary Data Fault Status Register */
#define CP15_AIFSR(r)      _CP15(0, r, c5, c1, 1)   /* Auxiliary Instruction Fault Status Register */

#define CP15_DFAR(r)       _CP15(0, r, c6, c0, 0)   /* Data Fault Address Register */
#define CP15_IFAR(r)       _CP15(0, r, c6, c0, 2)   /* Instruction Fault Address Register */
#define CP15_DRBAR(r)      _CP15(0, r, c6, c1, 0)   /* Data Region Base Address Register */
#define CP15_DRSR(r)       _CP15(0, r, c6, c1, 2)   /* Data Region Size and Enable Register */
#define CP15_DRACR(r)      _CP15(0, r, c6, c1, 4)   /* Data Region Access Control Register */
#ifndef CONFIG_ARM_HAVE_MPU_UNIFIED
#  define CP15_IRBAR(r)    _CP15(0, r, c6, c1, 1)   /* Instruction Region Base Address Register */
#  define CP15_IRSR(r)     _CP15(0, r, c6, c1, 3)   /* Instruction Region Size and Enable Register */
#  define CP15_IRACR(r)    _CP15(0, r, c6, c1, 5)   /* Instruction Region Access Control Register */
#endif
#define CP15_RGNR(r)       _CP15(0, r, c6, c2, 0)   /* MPU Region Number Register */

#define CP15_ICIALLUIS(r)  _CP15(0, r, c7, c1, 0)   /* Cache Operations Registers */
#define CP15_BPIALLIS(r)   _CP15(0, r, c7, c1, 6)

#define CP15_ICIALLU(r)    _CP15(0, r, c7, c5, 0)   /* Cache Operations Registers */
#define CP15_ICIMVAU(r)    _CP15(0, r, c7, c5, 1)
#define CP15_CP15ISB(r)    _CP15(0, r, c7, c5, 4)   /* CP15 Instruction Synchronization Barrier operation */
#define CP15_BPIALL(r)     _CP15(0, r, c7, c5, 6)   /* Cache Operations Registers */
#define CP15_BPIMVA(r)     _CP15(0, r, c7, c5, 7)   /* Cortex-A5 */
#define CP15_DCIMVAC(r)    _CP15(0, r, c7, c6, 1)
#define CP15_DCISW(r)      _CP15(0, r, c7, c6, 2)
#define CP15_DCCMVAC(r)    _CP15(0, r, c7, c10, 1)  /* Data Cache Clean by MVA to PoC */
#define CP15_DCCSW(r)      _CP15(0, r, c7, c10, 2)  /* Data Cache Clean by Set/Way */
#define CP15_CP15DSB(r)    _CP15(0, r, c7, c10, 4)  /* CP15 Data Synchronization Barrier operation */
#define CP15_CP15DMB(r)    _CP15(0, r, c7, c10, 5)  /* CP15 Instruction Synchronization Barrier operation */
#define CP15_DCCMVAU(r)    _CP15(0, r, c7, c11, 1)  /* Cache Operations Registers */
#define CP15_DCCIMVAC(r)   _CP15(0, r, c7, c14, 1)
#define CP15_DCCISW(r)     _CP15(0, r, c7, c14, 2)

#define CP15_PMCR(r)       _CP15(0, r, c9, c12, 0)  /* Performance Monitor Control Register */
#define CP15_PMCNTENSET(r) _CP15(0, r, c9, c12, 1)  /* Count Enable Set Register */
#define CP15_PMCNTENCLR(r) _CP15(0, r, c9, c12, 2)  /* Count Enable Clear Register */
#define CP15_PMOVSR(r)     _CP15(0, r, c9, c12, 3)  /* Overflow Flag Status Register */
#define CP15_PMSWINC(r)    _CP15(0, r, c9, c12, 4)  /* Software Increment Register */
#define CP15_PMSELR(r)     _CP15(0, r, c9, c12, 5)  /* Event Counter Selection Register */
#define CP15_PMCEID0(r)    _CP15(0, r, c9, c12, 6)  /* Common Event Identification Registers (Cortex-A5) */
#define CP15_PMCEID1(r)    _CP15(0, r, c9, c12, 7)
#define CP15_PMCCNTR(r)    _CP15(0, r, c9, c13, 0)  /* Cycle Count Register */
#define CP15_PMXEVTYPER(r) _CP15(0, r, c9, c13, 1)  /* Event Type Select Register */
#define CP15_PMXEVCNTR(r)  _CP15(0, r, c9, c13, 2)  /* Event Count Registers (Cortex-A5) */
#define CP15_PMUSERENR(r)  _CP15(0, r, c9, c14, 0)  /* User Enable Register */
#define CP15_PMINTENSET(r) _CP15(0, r, c9, c14, 1)  /* Interrupt Enable Set Register */
#define CP15_PMINTENCLR(r) _CP15(0, r, c9, c14, 2)  /* Interrupt Enable Clear Register */

#define CP15_VBAR(r)       _CP15(0, r, c12, c0, 0)  /* Vector Base Address Register */
#define CP15_HVBAR(r)      _CP15(4, r, c12, c0, 0)  /* Hypervisor Vector Base Address Register */

#define CP15_CONTEXTIDR(r) _CP15(0, r, c13, c0, 1)  /* Context ID Register */
#define CP15_TPIDRURW(r)   _CP15(0, r, c13, c0, 2)  /* Software Thread ID Registers */
#define CP15_TPIDRURO(r)   _CP15(0, r, c13, c0, 3)
#define CP15_TPIDRPRW(r)   _CP15(0, r, c13, c0, 4)

#define CP15_CNTFRQ(r)        _CP15(0, r, c14, c0, 0)     /* CNTFRQ */
#define CP15_CNTVCT(lo,hi)    _CP15_64(1, lo, hi, c14)    /* CNTVCT */
#define CP15_CNTV_CTL(r)      _CP15(0, r, c14, c3, 1)     /* CNTV_CTL */
#define CP15_CNTV_CVAL(lo,hi) _CP15_64(3, lo, hi, c14)    /* CNTV_CVAL */

#define CP15_CNTPCT(lo,hi)    _CP15_64(0, lo, hi, c14)    /* Physical Count register */

#define CP15_DCIALLU(r)       _CP15(0, r, c15, c5, 0)     /* Invalidate data cache */

#define CP15_ICC_PMR(r)       _CP15(0, r, c4,  c6,  0)    /* ICC_PMR */
#define CP15_ICC_IAR1(r)      _CP15(0, r, c12, c12, 0)    /* ICC_IAR1 */
#define CP15_ICC_EOIR1(r)     _CP15(0, r, c12, c12, 1)    /* ICC_EOIR1 */
#define CP15_ICC_SRE(r)       _CP15(0, r, c12, c12, 5)    /* ICC_SRE */
#define CP15_ICC_HSRE(r)      _CP15(4, r, c12,  c9, 5)    /* ICC_HSRE */
#define CP15_ICC_IGRPEN1(r)   _CP15(0, r, c12, c12, 7)    /* ICC_IGRPEN1 */
#define CP15_ICC_SGI1R(lo,hi) _CP15_64(2, lo, hi, c12)    /* ICC_SGI1R */

#define CP15_SET(reg, value)            \
  do                                    \
    {                                   \
      __asm__ __volatile__              \
      (                                 \
        "mcr " CP15_ ## reg(0) "\n"     \
        :: "r"(value): "memory"         \
      );                                \
    }                                   \
  while(0)                              \

#define CP15_SET2(reg, op, value)       \
  do                                    \
    {                                   \
      __asm__ __volatile__              \
      (                                 \
        "mcr " CP15_ ## reg(0, op) "\n" \
        :: "r"(value): "memory"         \
      );                                \
    }                                   \
  while(0)                              \

#define CP15_GET(reg)                   \
  ({                                    \
     uint32_t _value;                    \
     __asm__ __volatile__               \
     (                                  \
       "mrc " CP15_ ## reg(0) "\n"      \
       : "=r"(_value) :: "memory"        \
     );                                 \
     _value;                             \
  })                                    \

#define CP15_SET64(reg, value)          \
  do                                    \
    {                                   \
      __asm__ __volatile__              \
      (                                 \
        "mcrr " CP15_ ## reg(0,0) "\n"  \
        :: "r"(value): "memory"         \
      );                                \
    }                                   \
  while(0)                              \

#define CP15_GET64(reg)                 \
  ({                                    \
     uint64_t _value;                    \
     __asm__ __volatile__               \
     (                                  \
       "mrrc " CP15_ ## reg(0,0) "\n"   \
       : "=r"(_value) :: "memory"        \
     );                                 \
     _value;                             \
  })                                    \

#endif /* __ARCH_ARM_SRC_ARMV8_R_CP15_H */
