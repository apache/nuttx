/****************************************************************************
 * arch/or1k/include/spr.h
 *
 *   Copyright (C) 2018 Extent3D. All rights reserved.
 *   Author: Matt Thompson <matt@extent3d.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_OR1K_INCLUDE_SPR_H
#define __ARCH_OR1K_INCLUDE_SPR_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Move a register to an SPR */

#define mtspr(_spr, _val) __asm__ __volatile__ ( \
  "l.mtspr r0,%1,%0"          \
  : : "K" (_spr), "r" (_val))

/* Move an SPR into a register */

#define mfspr(_spr, _val) __asm__ __volatile__ ( \
  "l.mfspr %0,r0,%1" \
  : "=r" (_val) : "K" (_spr))

/* Special Purpose Registers
 *
 * A 16-bit SPR address is comprised of a 5-bit group index and an
 * 11-bit register index
 *
 */

#define SPR_GROUP_SHIFT     (11)
#define SPR_GROUP_MASK      (0x1f << SPR_GROUP_SHIFT)

/* Group index definitions.
 * OpenRISC 1000 Architecture Manual Table 4-1
 */

#define SPR_GROUP_SYS       (0 << SPR_GROUP_SHIFT)
#define SPR_GROUP_DMMU      (1 << SPR_GROUP_SHIFT)
#define SPR_GROUP_IMMU      (2 << SPR_GROUP_SHIFT)
#define SPR_GROUP_DCACHE    (3 << SPR_GROUP_SHIFT)
#define SPR_GROUP_ICACHE    (4 << SPR_GROUP_SHIFT)
#define SPR_GROUP_MAC       (5 << SPR_GROUP_SHIFT)
#define SPR_GROUP_DEBUG     (6 << SPR_GROUP_SHIFT)
#define SPR_GROUP_PERF      (7 << SPR_GROUP_SHIFT)
#define SPR_GROUP_PM        (8 << SPR_GROUP_SHIFT)
#define SPR_GROUP_PIC       (9 << SPR_GROUP_SHIFT)
#define SPR_GROUP_TICK      (10 << SPR_GROUP_SHIFT)
#define SPR_GROUP_FPU       (11 << SPR_GROUP_SHIFT)

/* System Group Registers */

#define SPR_SYS_VR       (SPR_GROUP_SYS | 0)          /* Version Register */
#define SPR_SYS_UPR      (SPR_GROUP_SYS | 1)          /* Unit Present Register */
#define SPR_SYS_CPUCFGR  (SPR_GROUP_SYS | 2)          /* CPU Configuration */
#define SPR_SYS_DMMUCFGR (SPR_GROUP_SYS | 3)          /* Data MMU Configuration */
#define SPR_SYS_IMMUCFGR (SPR_GROUP_SYS | 4)          /* Instruction MMU Config */
#define SPR_SYS_DCCFGR   (SPR_GROUP_SYS | 5)          /* Data Cache Configuration */
#define SPR_SYS_ICCFGR   (SPR_GROUP_SYS | 6)          /* Instruction Cache Config */
#define SPR_SYS_DCFGR    (SPR_GROUP_SYS | 7)          /* Debug Configuration */
#define SPR_SYS_PCCFGR   (SPR_GROUP_SYS | 8)          /* Performance Counter Config */
#define SPR_SYS_VR2      (SPR_GROUP_SYS | 9)          /* Version Register 2 */
#define SPR_SYS_AVR      (SPR_GROUP_SYS | 10)         /* Architecture Version */
#define SPR_SYS_EVBAR    (SPR_GROUP_SYS | 11)         /* Exception vector base */
#define SPR_SYS_AECR     (SPR_GROUP_SYS | 12)         /* Arithmetic exception control */
#define SPR_SYS_AESR     (SPR_GROUP_SYS | 13)         /* Arithmetic exception status */
#define SPR_SYS_NPC      (SPR_GROUP_SYS | 16)         /* PC mapped to SPR - next PC */
#define SPR_SYS_SR       (SPR_GROUP_SYS | 17)         /* Supervision register */
#define SPR_SYS_PPC      (SPR_GROUP_SYS | 18)         /* PC mapped to SPR space */
#define SPR_SYS_FPCSR    (SPR_GROUP_SYS | 20)         /* FP Control Status */
#define SPR_SYS_ISR0     (SPR_GROUP_SYS | 21)         /* Implementation specific */
#define SPR_SYS_ISR1     (SPR_GROUP_SYS | 22)         /* Implementation specific */
#define SPR_SYS_ISR2     (SPR_GROUP_SYS | 23)         /* Implementation specific */
#define SPR_SYS_ISR3     (SPR_GROUP_SYS | 24)         /* Implementation specific */
#define SPR_SYS_ISR4     (SPR_GROUP_SYS | 25)         /* Implementation specific */
#define SPR_SYS_ISR5     (SPR_GROUP_SYS | 26)         /* Implementation specific */
#define SPR_SYS_ISR6     (SPR_GROUP_SYS | 27)         /* Implementation specific */
#define SPR_SYS_ISR7     (SPR_GROUP_SYS | 28)         /* Implementation specific */
#define SPR_SYS_EPCR0    (SPR_GROUP_SYS | 32)         /* Exception PC register */
#define SPR_SYS_EPCR1    (SPR_GROUP_SYS | 33)         /* Exception PC register */
#define SPR_SYS_EPCR2    (SPR_GROUP_SYS | 34)         /* Exception PC register */
#define SPR_SYS_EPCR3    (SPR_GROUP_SYS | 35)         /* Exception PC register */
#define SPR_SYS_EPCR4    (SPR_GROUP_SYS | 36)         /* Exception PC register */
#define SPR_SYS_EPCR5    (SPR_GROUP_SYS | 37)         /* Exception PC register */
#define SPR_SYS_EPCR6    (SPR_GROUP_SYS | 38)         /* Exception PC register */
#define SPR_SYS_EPCR7    (SPR_GROUP_SYS | 39)         /* Exception PC register */
#define SPR_SYS_EPCR8    (SPR_GROUP_SYS | 40)         /* Exception PC register */
#define SPR_SYS_EPCR9    (SPR_GROUP_SYS | 41)         /* Exception PC register */
#define SPR_SYS_EPCR10   (SPR_GROUP_SYS | 42)         /* Exception PC register */
#define SPR_SYS_EPCR11   (SPR_GROUP_SYS | 43)         /* Exception PC register */
#define SPR_SYS_EPCR12   (SPR_GROUP_SYS | 44)         /* Exception PC register */
#define SPR_SYS_EPCR13   (SPR_GROUP_SYS | 45)         /* Exception PC register */
#define SPR_SYS_EPCR14   (SPR_GROUP_SYS | 46)         /* Exception PC register */
#define SPR_SYS_EPCR15   (SPR_GROUP_SYS | 47)         /* Exception PC register */
#define SPR_SYS_EEAR0    (SPR_GROUP_SYS | 48)         /* Exception EA register */
#define SPR_SYS_EEAR1    (SPR_GROUP_SYS | 49)         /* Exception EA register */
#define SPR_SYS_EEAR2    (SPR_GROUP_SYS | 50)         /* Exception EA register */
#define SPR_SYS_EEAR3    (SPR_GROUP_SYS | 51)         /* Exception EA register */
#define SPR_SYS_EEAR4    (SPR_GROUP_SYS | 52)         /* Exception EA register */
#define SPR_SYS_EEAR5    (SPR_GROUP_SYS | 53)         /* Exception EA register */
#define SPR_SYS_EEAR6    (SPR_GROUP_SYS | 54)         /* Exception EA register */
#define SPR_SYS_EEAR7    (SPR_GROUP_SYS | 55)         /* Exception EA register */
#define SPR_SYS_EEAR8    (SPR_GROUP_SYS | 56)         /* Exception EA register */
#define SPR_SYS_EEAR9    (SPR_GROUP_SYS | 57)         /* Exception EA register */
#define SPR_SYS_EEAR10   (SPR_GROUP_SYS | 58)         /* Exception EA register */
#define SPR_SYS_EEAR11   (SPR_GROUP_SYS | 59)         /* Exception EA register */
#define SPR_SYS_EEAR12   (SPR_GROUP_SYS | 60)         /* Exception EA register */
#define SPR_SYS_EEAR13   (SPR_GROUP_SYS | 61)         /* Exception EA register */
#define SPR_SYS_EEAR14   (SPR_GROUP_SYS | 62)         /* Exception EA register */
#define SPR_SYS_EEAR15   (SPR_GROUP_SYS | 63)         /* Exception EA register */
#define SPR_SYS_ESR0     (SPR_GROUP_SYS | 64)         /* Exception SR register */
#define SPR_SYS_ESR1     (SPR_GROUP_SYS | 65)         /* Exception SR register */
#define SPR_SYS_ESR2     (SPR_GROUP_SYS | 66)         /* Exception SR register */
#define SPR_SYS_ESR3     (SPR_GROUP_SYS | 67)         /* Exception SR register */
#define SPR_SYS_ESR4     (SPR_GROUP_SYS | 68)         /* Exception SR register */
#define SPR_SYS_ESR5     (SPR_GROUP_SYS | 69)         /* Exception SR register */
#define SPR_SYS_ESR6     (SPR_GROUP_SYS | 70)         /* Exception SR register */
#define SPR_SYS_ESR7     (SPR_GROUP_SYS | 71)         /* Exception SR register */
#define SPR_SYS_ESR8     (SPR_GROUP_SYS | 72)         /* Exception SR register */
#define SPR_SYS_ESR9     (SPR_GROUP_SYS | 73)         /* Exception SR register */
#define SPR_SYS_ESR10    (SPR_GROUP_SYS | 74)         /* Exception SR register */
#define SPR_SYS_ESR11    (SPR_GROUP_SYS | 75)         /* Exception SR register */
#define SPR_SYS_ESR12    (SPR_GROUP_SYS | 76)         /* Exception SR register */
#define SPR_SYS_ESR13    (SPR_GROUP_SYS | 77)         /* Exception SR register */
#define SPR_SYS_ESR14    (SPR_GROUP_SYS | 78)         /* Exception SR register */
#define SPR_SYS_ESR15    (SPR_GROUP_SYS | 79)         /* Exception SR register */
#define SPR_SYS_COREID   (SPR_GROUP_SYS | 128)        /* Core Identifier */
#define SPR_SYS_NUMCORES (SPR_GROUP_SYS | 129)        /* Number of cores */
#define SPR_SYS_GPR(n)   (SPR_GROUP_SYS | 1024+(n))   /* GPRs mapped to SPR */

/* Data MMU Group Registers */

#define SPR_DMMU_CR      (SPR_GROUP_DMMU | 0)         /* DMMU Control Register */
#define SPR_DMMU_PR      (SPR_GROUP_DMMU | 1)         /* DMMU Protection Register */
#define SPR_DMMU_DTLBEIR (SPR_GROUP_DMMU | 2)         /* TLB invalidate register */
#define SPR_DMMU_DATBMR0 (SPR_GROUP_DMMU | 4)         /* Data ATB match register */
#define SPR_DMMU_DATBMR1 (SPR_GROUP_DMMU | 5)         /* Data ATB match register */
#define SPR_DMMU_DATBMR2 (SPR_GROUP_DMMU | 6)         /* Data ATB match register */
#define SPR_DMMU_DATBMR3 (SPR_GROUP_DMMU | 7)         /* Data ATB match register */

/* DCACHE Group Registers */

#define SPR_DCACHE_CR    (SPR_GROUP_DCACHE | 0)
#define SPR_DCACHE_BPR   (SPR_GROUP_DCACHE | 1)
#define SPR_DCACHE_BFR   (SPR_GROUP_DCACHE | 2)
#define SPR_DCACHE_BIR   (SPR_GROUP_DCACHE | 3)
#define SPR_DCACHE_BWR   (SPR_GROUP_DCACHE | 4)
#define SPR_DCACHE_BLR   (SPR_GROUP_DCACHE | 5)

/* ICACHE Group Registers */

#define SPR_ICACHE_CR    (SPR_GROUP_DCACHE | 0)
#define SPR_ICACHE_BPR   (SPR_GROUP_DCACHE | 1)
#define SPR_ICACHE_BIR   (SPR_GROUP_DCACHE | 2)
#define SPR_ICACHE_BLR   (SPR_GROUP_DCACHE | 3)

/* Programmable Interrupt Controller */

#define SPR_PIC_MR       (SPR_GROUP_PIC | 0)          /* PIC Mask Register */
#define SPR_PIC_SR       (SPR_GROUP_PIC | 2)          /* PIC Status Register */

/* Tick Timer Group Registers */

#define SPR_TICK_TTMR    (SPR_GROUP_TICK | 0)         /* Tick Timer Mode Register */
#define SPR_TICK_TTCR    (SPR_GROUP_TICK | 1)         /* Tick Timer Count Register */

/* Version Register Bits */

#define SPR_VR_REV_SHIFT  (0)
#define SPR_VR_REV_MASK   (0x3f << SPR_VR_REV_SHIFT)  /* CPU Revision */
#define SPR_VR_UVRP       (1 << 6)                    /* VR2 Register Present */
#define SPR_VR_CFG_SHIFT  (16)
#define SPR_VR_CFG_MASK   (0xff << SPR_VR_CFG_SHIFT)  /* Configuration Template */
#define SPR_VR_VER_SHIFT  (24)
#define SPR_VR_VER_MASK   (0xff << SPR_VR_VER_SHIFT)  /* CPU Version */

/* VR2 Register Bits */

#define SPR_VR2_VER_SHIFT   (0)
#define SPR_VR2_VER_MASK    (0xfffff << SPR_VR2_VER_SHIFT)
#define SPR_VR2_CPUID_SHIFT (24)
#define SPR_VR2_CPUID_MASK  (0xff << SPR_VR2_CPUID_SHIFT)

/* Unit Present Register Bits */

#define SPR_UPR_UP          (1 << 0)                    /* UPR present */
#define SPR_UPR_DCP         (1 << 1)                    /* Data Cache present */
#define SPR_UPR_ICP         (1 << 2)                    /* Instruction Cache present */
#define SPR_UPR_DMP         (1 << 3)                    /* Data MMU present */
#define SPR_UPR_IMP         (1 << 4)                    /* Instruction MMU present */
#define SPR_UPR_MP          (1 << 5)                    /* MAC present */
#define SPR_UPR_DUP         (1 << 6)                    /* Debug unit present */
#define SPR_UPR_PCUP        (1 << 7)                    /* Performance counters present */
#define SPR_UPR_PICP        (1 << 8)                    /* Interrupt controller present */
#define SPR_UPR_PMP         (1 << 9)                    /* Power Management present */
#define SPR_UPR_TTP         (1 << 10)                   /* Tick timer present */
#define SPR_UPR_CUP_SHIFT   (24)
#define SPR_UPR_CUP_MASK    (0xff << SPR_UPR_CUP_SHIFT) /* Context units present mask */

/* CPU Configuration Register Bits */

#define SPR_CPUCFGR_NSGF_SHIFT  (0)
#define SPR_CPUCFGR_NSGF_MASK   (0xf << SPR_CPUCFGR_NSGF_SHIFT)
#define SPR_CPUCFGR_CGF         (1 << 4)
#define SPR_CPUCFGR_OB32S       (1 << 5)                /* ORBIS32 Supported */
#define SPR_CPUCFGR_OB64S       (1 << 6)                /* ORBIS64 Supported */
#define SPR_CPUCFGR_OF32S       (1 << 7)                /* ORFPX32 Supported */
#define SPR_CPUCFGR_OF64S       (1 << 8)                /* ORFPX64 Supported */
#define SPR_CPUCFGR_OV64S       (1 << 9)                /* ORVDX64 Supported */
#define SPR_CPUCFGR_ND          (1 << 10)               /* No delay slot */
#define SPR_CPUCFGR_AVRP        (1 << 11)               /* Arch Version Register present */
#define SPR_CPUCFGR_EVBARP      (1 << 12)               /* Exception Vector Base Addr */
#define SPR_CPUCFGR_ISRP        (1 << 13)               /* Implementation specific present */
#define SPR_CPUCFGR_AECSRP      (1 << 14)               /* Arithmetic Exception Status present */

/* DMMU Register Bits */

/* IMMU Register Bits */

/* Data Cache Configurtion Register Bits */

#define SPR_DCCFGR_NCW_SHIFT    (0)
#define SPR_DCCFGR_NCW_MASK     (0x7 << SPR_DCCFGR_NCW_SHIFT)
#define SPR_DCCFGR_NCS_SHIFT    (3)
#define SPR_DCCFGR_NCS_MASK     (0xf << SPR_DCCFGR_NCS_SHIFT)
#define SPR_DCCFGR_CBS          (1 << 7)
#define SPR_DCCFGR_CWS          (1 << 8)
#define SPR_DCCFGR_CCRI         (1 << 9)
#define SPR_DCCFGR_CBIRI        (1 << 10)
#define SPR_DCCFGR_CBPRI        (1 << 11)
#define SPR_DCCFGR_CBLRI        (1 << 12)
#define SPR_DCCFGR_CBFRI        (1 << 13)
#define SPR_DCCFGR_CBWBRI       (1 << 14)

/* Instruction Cache Configuration Register Bits */

#define SPR_ICCFGR_NCW_SHIFT    (0)
#define SPR_ICCFGR_NCW_MASK     (0x7 << SPR_ICCFGR_NCW_SHIFT)
#define SPR_ICCFGR_NCS_SHIFT    (3)
#define SPR_ICCFGR_NCS_MASK     (0xf << SPR_ICCFGR_NCS_SHIFT)
#define SPR_ICCFGR_CBS          (1 << 7)
#define SPR_ICCFGR_CCRI         (1 << 9)
#define SPR_ICCFGR_CBIRI        (1 << 10)
#define SPR_ICCFGR_CBPRI        (1 << 11)
#define SPR_ICCFGR_CBLRI        (1 << 12)

/* Supervision Register Bits */

#define SPR_SR_SM          0x00000001  /* Supervisor Mode */
#define SPR_SR_TEE         0x00000002  /* Tick timer Exception Enable */
#define SPR_SR_IEE         0x00000004  /* Interrupt Exception Enable */
#define SPR_SR_DCE         0x00000008  /* Data Cache Enable */
#define SPR_SR_ICE         0x00000010  /* Instruction Cache Enable */
#define SPR_SR_DME         0x00000020  /* Data MMU Enable */
#define SPR_SR_IME         0x00000040  /* Instruction MMU Enable */
#define SPR_SR_LEE         0x00000080  /* Little Endian Enable */
#define SPR_SR_CE          0x00000100  /* CID Enable */
#define SPR_SR_F           0x00000200  /* Condition Flag */
#define SPR_SR_CY          0x00000400  /* Carry flag */
#define SPR_SR_OV          0x00000800  /* Overflow flag */
#define SPR_SR_OVE         0x00001000  /* Overflow flag Exception */
#define SPR_SR_DSX         0x00002000  /* Delay Slot Exception */
#define SPR_SR_EPH         0x00004000  /* Exception Prefix High */
#define SPR_SR_FO          0x00008000  /* Fixed one */
#define SPR_SR_SUMRA       0x00010000  /* Supervisor SPR read access */
#define SPR_SR_RES         0x0ffe0000  /* Reserved */
#define SPR_SR_CID         0xf0000000  /* Context ID */

#define SPR_TTMR_TP_MASK   (0xFFFFFFF)
#define SPR_TTMR_IP        (1<<28)
#define SPR_TTMR_IE        (1<<29)
#define SPR_TTMR_M         (1<<30)

#endif /* __ARCH_OR1K_INCLUDE_SPR_H */
