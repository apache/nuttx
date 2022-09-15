/****************************************************************************
 * arch/arm/src/armv7-a/cp15.h
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
 *
 *  "Cortex-A5™ MPCore, Technical Reference Manual", Revision: r0p1,
 *   Copyright © 2010 ARM. All rights reserved. ARM DDI 0434B (ID101810)
 *  "ARM® Architecture Reference Manual, ARMv7-A and ARMv7-R edition",
 *   Copyright © 1996-1998, 2000, 2004-2012 ARM. All rights reserved.
 * ARM DDI 0406C.b (ID072512)
 */

#ifndef __ARCH_ARM_SRC_ARMV7_A_CP15_H
#define __ARCH_ARM_SRC_ARMV7_A_CP15_H

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
 * Reference: Cortex-A5™ MPCore, Technical Reference Manual, Paragraph 4.2.
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
#define CP15_TLBTR(r)      _CP15(0, r, c0, c0, 3)   /* TLB Type Register */
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
#define CP15_SCR(r)        _CP15(0, r, c1, c1, 0)   /* Secure Configuration Register */
#define CP15_SDER(r)       _CP15(0, r, c1, c1, 1)   /* Secure Debug Enable Register */
#define CP15_NSACR(r)      _CP15(0, r, c1, c1, 2)   /* Non-secure Access Control Register */
#define CP15_VCR(r)        _CP15(0, r, c1, c1, 3)   /* Virtualization Control Register */

#define CP15_TTBR0(r)      _CP15(0, r, c2, c0, 0)   /* Translation Table Base Register 0 */
#define CP15_TTBR1(r)      _CP15(0, r, c2, c0, 1)   /* Translation Table Base Register 1 */
#define CP15_TTBCR(r)      _CP15(0, r, c2, c0, 2)   /* Translation Table Base Control Register */

#define CP15_DACR(r)       _CP15(0, r, c3, c0, 0)   /* Domain Access Control Register */

#define CP15_DFSR(r)       _CP15(0, r, c5, c0, 0)   /* Data Fault Status Register */
#define CP15_IFSR(r)       _CP15(0, r, c5, c0, 1)   /* Instruction Fault Status Register */
#define CP15_ADFSR(r)      _CP15(0, r, c5, c1, 0)   /* Auxiliary Data Fault Status Register */
#define CP15_AIFSR(r)      _CP15(0, r, c5, c1, 1)   /* Auxiliary Instruction Fault Status Register */

#define CP15_DFAR(r)       _CP15(0, r, c6, c0, 0)   /* Data Fault Address Register */
#define CP15_IFAR(r)       _CP15(0, r, c6, c0, 2)   /* Instruction Fault Address Register */

#define CP15_NOP(r)        _CP15(0, r, c7, c0, 4)
#define CP15_ICIALLUIS(r)  _CP15(0, r, c7, c1, 0)   /* Cache Operations Registers */
#define CP15_BPIALLIS(r)   _CP15(0, r, c7, c1, 6)
#define CP15_PAR(r)        _CP15(0, r, c7, c4, 0)   /* Physical Address Register */
#define CP15_ICIALLU(r)    _CP15(0, r, c7, c5, 0)   /* Cache Operations Registers */
#define CP15_ICIMVAU(r)    _CP15(0, r, c7, c5, 1)
#define CP15_ISB(r)        _CP15(0, r, c7, c5, 4)
#define CP15_BPIALL(r)     _CP15(0, r, c7, c5, 6)   /* Cache Operations Registers */
#define CP15_BPIMVA(r)     _CP15(0, r, c7, c5, 7)   /* Cortex-A5 */
#define CP15_DCIMVAC(r)    _CP15(0, r, c7, c6, 1)
#define CP15_DCISW(r)      _CP15(0, r, c7, c6, 2)
#define CP15_V2PCWPR(r,n)  _CP15(0, r, c7, c8, (n)) /* VA to PA operations, n=0-3 */
#  define CP15_V2PCWPR0(r) _CP15(0, r, c7, c8, 0)
#  define CP15_V2PCWPR1(r) _CP15(0, r, c7, c8, 1)
#  define CP15_V2PCWPR2(r) _CP15(0, r, c7, c8, 2)
#  define CP15_V2PCWPR3(r) _CP15(0, r, c7, c8, 3)
#define CP15_V2POWPR(r,n)  _CP15(0, r, c7, c8, ((n)+4)) /* n=0-3 */
#  define CP15_V2POWPR0(r) _CP15(0, r, c7, c8, 4)
#  define CP15_V2POWPR1(r) _CP15(0, r, c7, c8, 5)
#  define CP15_V2POWPR2(r) _CP15(0, r, c7, c8, 6)
#  define CP15_V2POWPR3(r) _CP15(0, r, c7, c8, 7)
#define CP15_DCCMVAC(r)    _CP15(0, r, c7, c10, 1)  /* Cache Operations Registers (aka DCCVAC) */
#define CP15_DCCSW(r)      _CP15(0, r, c7, c10, 2)
#define CP15_DSB(r)        _CP15(0, r, c7, c10, 4)
#define CP15_DMB(r)        _CP15(0, r, c7, c10, 5)
#define CP15_DCCMVAU(r)    _CP15(0, r, c7, c11, 1)  /* Cache Operations Registers */
#define CP15_DCCIMVAC(r)   _CP15(0, r, c7, c14, 1)
#define CP15_DCCISW(r)     _CP15(0, r, c7, c14, 2)

#define CP15_TLBIALLIS(r)  _CP15(0, r, c8, c3, 0)   /* Invalidate entire unified TLB Inner Shareable */
#define CP15_TLBIMVAIS(r)  _CP15(0, r, c8, c3, 1)   /* Invalidate unified TLB entry by MVA and ASID, Inner Shareable */
#define CP15_TLBIASIDIS(r) _CP15(0, r, c8, c3, 2)   /* Invalidate unified TLB by ASID match Inner Shareable */
#define CP15_TLBIMVAAIS(r) _CP15(0, r, c8, c3, 3)   /* Invalidate unified TLB entry by MVA all ASID Inner Shareable */
#define CP15_TLBIALL(r,c)  _CP15(0, r, c8, c, 0)    /* Invalidate entire instruction TLB. CRm = c5, c6, or c7 */
#define CP15_TLBIMVA(r,c)  _CP15(0, r, c8, c, 1)    /* Invalidate instruction TLB entry by MVA and ASID. CRm = c5, c6, or c7 */
#define CP15_TLBIASID(r,c) _CP15(0, r, c8, c, 2)    /* Invalidate data TLB by ASID match. CRm = c5, c6, or c7 */
#define CP15_TLBIMVAA(r,c) _CP15(0, r, c8, c, 3)    /* Invalidate unified TLB entry by MVA and ASID. CRm = c5, c6, or c7 */

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
#define CP15_PMCCFILTR(r)  _CP15(0, r, c9, c13, 1)  /* Cycle Count Filter Control Register */
#define CP15_PMXEVCNTR(r)  _CP15(0, r, c9, c13, 2)  /* Event Count Registers (Cortex-A5) */
#define CP15_PMUSERENR(r)  _CP15(0, r, c9, c14, 0)  /* User Enable Register */
#define CP15_PMINTENSET(r) _CP15(0, r, c9, c14, 1)  /* Interrupt Enable Set Register */
#define CP15_PMINTENCLR(r) _CP15(0, r, c9, c14, 2)  /* Interrupt Enable Clear Register */

#define CP15_TLBLCKDOWN(r) _CP15(0, r, c10, c0, 0)  /* TLB Lockdown register (Cortex-A9) */
#define CP15_PPRRR(r)      _CP15(0, r, c10, c2, 0)  /* Primary Region Remap Register */
#define CP15_NMRR(r)       _CP15(0, r, c10, c2, 1)  /* Normal Memory Remap Register */

#define CP15_PLEIDR(r)     _CP15(0, r, c11, c0, 0)  /* PLE ID Register (Cortex-A9) */
#define CP15_PLEASR(r)     _CP15(0, r, c11, c0, 2)  /* PLE Activity Status Register (Cortex-A9) */
#define CP15_PLEFSR(r)     _CP15(0, r, c11, c0, 4)  /* PLE FIFO Status Register (Cortex-A9) */
#define CP15_PLEUAR(r)     _CP15(0, r, c11, c1, 0)  /* Preload Engine User Accessibility Register (Cortex-A9) */
#define CP15_PLEPCR(r)     _CP15(0, r, c11, c1, 1)  /* Preload Engine Parameters Control Register (Cortex-A9) */

#define CP15_VBAR(r)       _CP15(0, r, c12, c0, 0)  /* Vector Base Address Register */
#define CP15_MVBAR(r)      _CP15(0, r, c12, c0, 1)  /* Monitor Vector Base Address Register */
#define CP15_ISR(r)        _CP15(0, r, c12, c1, 0)  /* Interrupt Status Register */
#define CP15_VIR(r)        _CP15(0, r, c12, c1, 1)  /* Virtualization Interrupt Register */

#define CP15_FCSEIDR(r)    _CP15(0, r, c13, c0, 0)  /* Fast Context Switch Extension (FCSE) not implemented */
#define CP15_CONTEXTIDR(r) _CP15(0, r, c13, c0, 1)  /* Context ID Register */
#define CP15_TPIDRURW(r)   _CP15(0, r, c13, c0, 2)  /* Software Thread ID Registers */
#define CP15_TPIDRURO(r)   _CP15(0, r, c13, c0, 3)
#define CP15_TPIDRPRW(r)   _CP15(0, r, c13, c0, 4)

#define CP15_CNTFRQ(r)     _CP15(0, r, c14, c0, 0)  /* Counter Frequency register */
#define CP15_CNTKCTL(r)    _CP15(0, r, c14, c1, 0)  /* Timer PL1 Control register */
#define CP15_CNTP_TVAL(r)  _CP15(0, r, c14, c2, 0)  /* PL1 Physical TimerValue register */
#define CP15_CNTP_CTL(r)   _CP15(0, r, c14, c2, 1)  /* PL1 Physical Timer Control register */
#define CP15_CNTV_TVAL(r)  _CP15(0, r, c14, c3, 0)  /* Virtual TimerValue register */
#define CP15_CNTV_CTL(r)   _CP15(0, r, c14, c3, 0)  /* Virtual Timer Control register */

#define CP15_CNTPCT(lo,hi) _CP15_64(0, lo, hi, c14)   /* Physical Count register */

#define CP15_PWRCTRL(r)    _CP15(0, r, c15, c0, 0)  /* Power Control Register (Cortex-A9) */
#define CP15_NEONBUSY(r)   _CP15(0, r, c15, c1, 1)  /* NEON Busy Register (Cortex-A9) */
#define CP15_DR0(r)        _CP15(3, r, c15, c0, 0)  /* Data Register (Cortex-A5) */
#define CP15_DR1(r)        _CP15(3, r, c15, c0, 1)  /* Data Register (Cortex-A5) */
#define CP15_DTAGR(r)      _CP15(3, r, c15, c2, 0)  /* Data Cache Tag Read Operation Register (Cortex-A5) */
#define CP15_ITAGR(r)      _CP15(3, r, c15, c2, 1)  /* Instruction Cache Tag Read Operation Register (Cortex-A5) */
#define CP15_DDATAR(r)     _CP15(3, r, c15, c4, 0)  /* Data Cache Data Read Operation Register (Cortex-A5) */
#define CP15_IDATAR(r)     _CP15(3, r, c15, c4, 1)  /* Instruction Cache Data Read Operation Register (Cortex-A5) */
#define CP15_TLBR(r)       _CP15(3, r, c15, c4, 2)  /* TLB Data Read Operation Register (Cortex-A5) */
#define CP15_CBADDR(r)     _CP15(4, r, c15, c0, 0)  /* Configuration Base Address Register */
#define CP15_TLBHITMAP(r)  _CP15(5, r, c15, c0, 0)  /* TLB access and attributes (Cortex-A5) */
#define CP15_RTLBLCKDWN(r) _CP15(5, r, c15, c4, 2)  /* Select Lockdown TLB Entry for read (Cortex-A9) */
#define CP15_WTLBLCKDWN(r) _CP15(5, r, c15, c4, 4)  /* Select Lockdown TLB Entry for write (Cortex-A9) */
#define CP15_MAINTLBVA(r)  _CP15(5, r, c15, c5, 2)  /* Main TLB VA register (Cortex-A9) */
#define CP15_MAINTLBPA(r)  _CP15(5, r, c15, c6, 2)  /* Main TLB PA register (Cortex-A9) */
#define CP15_MAINTLBAT(r)  _CP15(5, r, c15, c7, 2)  /* Main TLB Attribute register (Cortex-A9) */

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
     uint32_t value;                    \
     __asm__ __volatile__               \
     (                                  \
       "mrc " CP15_ ## reg(0) "\n"      \
       : "=r"(value) :: "memory"        \
     );                                 \
     value;                             \
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
     uint64_t value;                    \
     __asm__ __volatile__               \
     (                                  \
       "mrrc " CP15_ ## reg(0,0) "\n"   \
       : "=r"(value) :: "memory"        \
     );                                 \
     value;                             \
  })                                    \

#endif /* __ARCH_ARM_SRC_ARMV7_A_CP15_H */
