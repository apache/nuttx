/****************************************************************************
 * arch/risc-v/include/csr.h
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_RISCV_INCLUDE_CSR_H
#define __ARCH_RISCV_INCLUDE_CSR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* User Trap Registers */

#define CSR_USTATUS         0x000
#define CSR_UIE             0x004
#define CSR_UTVEC           0x005

/* User Trap Handling Registers */

#define CSR_USCRATCH        0x040
#define CSR_UEPC            0x041
#define CSR_UCAUSE          0x042
#define CSR_UTVAL           0x043
#define CSR_UIP             0x044

/* User Floating-Point Registers */

#define CSR_FFLAGS          0x001
#define CSR_FRM             0x002
#define CSR_FCSR            0x003

/* User Counter/Times Registers */

#define CSR_CYCLE           0xc00
#define CSR_TIME            0xc01
#define CSR_INSTRET         0xc02
#define CSR_HPCOUNTER3      0xc03
#define CSR_HPCOUNTER4      0xc04
#define CSR_HPCOUNTER5      0xc05
#define CSR_HPCOUNTER6      0xc06
#define CSR_HPCOUNTER7      0xc07
#define CSR_HPCOUNTER8      0xc08
#define CSR_HPCOUNTER9      0xc09
#define CSR_HPCOUNTER10     0xc0a
#define CSR_HPCOUNTER11     0xc0b
#define CSR_HPCOUNTER12     0xc0c
#define CSR_HPCOUNTER13     0xc0d
#define CSR_HPCOUNTER14     0xc0e
#define CSR_HPCOUNTER15     0xc0f
#define CSR_HPCOUNTER16     0xc10
#define CSR_HPCOUNTER17     0xc11
#define CSR_HPCOUNTER18     0xc12
#define CSR_HPCOUNTER19     0xc13
#define CSR_HPCOUNTER20     0xc14
#define CSR_HPCOUNTER21     0xc15
#define CSR_HPCOUNTER22     0xc16
#define CSR_HPCOUNTER24     0xc17
#define CSR_HPCOUNTER25     0xc18
#define CSR_HPCOUNTER26     0xc19
#define CSR_HPCOUNTER27     0xc1a
#define CSR_HPCOUNTER28     0xc1b
#define CSR_HPCOUNTER29     0xc1c
#define CSR_HPCOUNTER30     0xc1d
#define CSR_HPCOUNTER31     0xc1f
#define CSR_CYCLEH          0xc80
#define CSR_TIMEH           0xc81
#define CSR_INSTRETH        0xc82
#define CSR_HPCOUNTER3H     0xc83
#define CSR_HPCOUNTER4H     0xc84
#define CSR_HPCOUNTER5H     0xc85
#define CSR_HPCOUNTER6H     0xc86
#define CSR_HPCOUNTER7H     0xc87
#define CSR_HPCOUNTER8H     0xc88
#define CSR_HPCOUNTER9H     0xc89
#define CSR_HPCOUNTER10H    0xc8a
#define CSR_HPCOUNTER11H    0xc8b
#define CSR_HPCOUNTER12H    0xc8c
#define CSR_HPCOUNTER13H    0xc8d
#define CSR_HPCOUNTER14H    0xc8e
#define CSR_HPCOUNTER15H    0xc8f
#define CSR_HPCOUNTER16H    0xc90
#define CSR_HPCOUNTER17H    0xc91
#define CSR_HPCOUNTER18H    0xc92
#define CSR_HPCOUNTER19H    0xc93
#define CSR_HPCOUNTER20H    0xc94
#define CSR_HPCOUNTER21H    0xc95
#define CSR_HPCOUNTER22H    0xc96
#define CSR_HPCOUNTER24H    0xc97
#define CSR_HPCOUNTER25H    0xc98
#define CSR_HPCOUNTER26H    0xc99
#define CSR_HPCOUNTER27H    0xc9a
#define CSR_HPCOUNTER28H    0xc9b
#define CSR_HPCOUNTER29H    0xc9c
#define CSR_HPCOUNTER30H    0xc9d
#define CSR_HPCOUNTER31H    0xc9f

/* Supervisor Trap Setup Registers */

#define CSR_SSTATUS         0x100
#define CSR_SEDELEG         0x102
#define CSR_SIDELEG         0x103
#define CSR_SIE             0x104
#define CSR_STVEC           0x105
#define CSR_SCOUNTEREN      0x106

/* Supervisor Trap Handling Registers */

#define CSR_SSCRATCH        0x140
#define CSR_SEPC            0x141
#define CSR_SCAUSE          0x142
#define CSR_STVAL           0x143
#define CSR_SIP             0x144

/* Supervisor Environment Configuration Registers */

#define CSR_SENVCFG         0x10a

/* Supervisor Protection and Translation Registers */

#define CSR_SATP            0x180

/* Supervisor Time Registers */

#define CSR_STIMECMP        0x14d
#define CSR_STIMECMPH       0x15d

/* Machine Information Registers */

#define CSR_MVENDORID       0xf11
#define CSR_MARCHID         0xf12
#define CSR_MIMPID          0xf13
#define CSR_MHARTID         0xf14

/* Machine Trap Registers */

#define CSR_MSTATUS         0x300
#define CSR_MISA            0x301
#define CSR_MEDELEG         0x302
#define CSR_MIDELEG         0x303
#define CSR_MIE             0x304
#define CSR_MTVEC           0x305
#define CSR_MCOUNTEREN      0x306

/* Machine Trap Handling */

#define CSR_MSCRATCH        0x340
#define CSR_MEPC            0x341
#define CSR_MCAUSE          0x342
#define CSR_MTVAL           0x343
#define CSR_MIP             0x344

/* Machine Environment Configuration Registers */

#define CSR_MENVCFG         0x30a
#define CSR_MENVCFGH        0x31a

/* Machine Protection and Translation */

#define CSR_PMPCFG0         0x3a0
#define CSR_PMPCFG1         0x3a1
#define CSR_PMPCFG2         0x3a2
#define CSR_PMPCFG3         0x3a3
#define CSR_PMPADDR0        0x3b0
#define CSR_PMPADDR1        0x3b1
#define CSR_PMPADDR2        0x3b2
#define CSR_PMPADDR3        0x3b3
#define CSR_PMPADDR4        0x3b4
#define CSR_PMPADDR5        0x3b5
#define CSR_PMPADDR6        0x3b6
#define CSR_PMPADDR7        0x3b7
#define CSR_PMPADDR8        0x3b8
#define CSR_PMPADDR9        0x3b9
#define CSR_PMPADDR10       0x3ba
#define CSR_PMPADDR11       0x3bb
#define CSR_PMPADDR12       0x3bc
#define CSR_PMPADDR13       0x3bd
#define CSR_PMPADDR14       0x3be
#define CSR_PMPADDR15       0x3bf

/* Machine Timers and Counters */

#define CSR_MCYCLE          0xb00
#define CSR_MINSTRET        0xb02
#define CSR_MHPMCOUNTER3    0xb03
#define CSR_MHPMCOUNTER4    0xb04
#define CSR_MHPMCOUNTER5    0xb05
#define CSR_MHPMCOUNTER6    0xb06
#define CSR_MHPMCOUNTER7    0xb07
#define CSR_MHPMCOUNTER8    0xb08
#define CSR_MHPMCOUNTER9    0xb09
#define CSR_MHPMCOUNTER10   0xb0a
#define CSR_MHPMCOUNTER11   0xb0b
#define CSR_MHPMCOUNTER12   0xb0c
#define CSR_MHPMCOUNTER13   0xb0d
#define CSR_MHPMCOUNTER14   0xb0e
#define CSR_MHPMCOUNTER15   0xb0f
#define CSR_MHPMCOUNTER16   0xb10
#define CSR_MHPMCOUNTER17   0xb11
#define CSR_MHPMCOUNTER18   0xb12
#define CSR_MHPMCOUNTER19   0xb13
#define CSR_MHPMCOUNTER20   0xb14
#define CSR_MHPMCOUNTER21   0xb15
#define CSR_MHPMCOUNTER22   0xb16
#define CSR_MHPMCOUNTER23   0xb17
#define CSR_MHPMCOUNTER24   0xb18
#define CSR_MHPMCOUNTER25   0xb19
#define CSR_MHPMCOUNTER26   0xb1a
#define CSR_MHPMCOUNTER27   0xb1b
#define CSR_MHPMCOUNTER28   0xb1c
#define CSR_MHPMCOUNTER29   0xb1d
#define CSR_MHPMCOUNTER30   0xb1e
#define CSR_MHPMCOUNTER31   0xb1f
#define CSR_MCYCLEH         0xb80
#define CSR_MINSTRETH       0xb82
#define CSR_MHPMCOUNTER3H   0xb83
#define CSR_MHPMCOUNTER4H   0xb84
#define CSR_MHPMCOUNTER5H   0xb85
#define CSR_MHPMCOUNTER6H   0xb86
#define CSR_MHPMCOUNTER7H   0xb87
#define CSR_MHPMCOUNTER8H   0xb88
#define CSR_MHPMCOUNTER9H   0xb89
#define CSR_MHPMCOUNTER10H  0xb8a
#define CSR_MHPMCOUNTER11H  0xb8b
#define CSR_MHPMCOUNTER12H  0xb8c
#define CSR_MHPMCOUNTER13H  0xb8d
#define CSR_MHPMCOUNTER14H  0xb8e
#define CSR_MHPMCOUNTER15H  0xb8f
#define CSR_MHPMCOUNTER16H  0xb90
#define CSR_MHPMCOUNTER17H  0xb91
#define CSR_MHPMCOUNTER18H  0xb92
#define CSR_MHPMCOUNTER19H  0xb93
#define CSR_MHPMCOUNTER20H  0xb94
#define CSR_MHPMCOUNTER21H  0xb95
#define CSR_MHPMCOUNTER22H  0xb96
#define CSR_MHPMCOUNTER23H  0xb97
#define CSR_MHPMCOUNTER24H  0xb98
#define CSR_MHPMCOUNTER25H  0xb99
#define CSR_MHPMCOUNTER26H  0xb9a
#define CSR_MHPMCOUNTER27H  0xb9b
#define CSR_MHPMCOUNTER28H  0xb9c
#define CSR_MHPMCOUNTER29H  0xb9d
#define CSR_MHPMCOUNTER30H  0xb9e
#define CSR_MHPMCOUNTER31H  0xb9f

/* Machine Counter Setup */

#define CSR_MPHEVENT3       0x323
#define CSR_MPHEVENT4       0x324
#define CSR_MPHEVENT5       0x325
#define CSR_MPHEVENT6       0x326
#define CSR_MPHEVENT7       0x327
#define CSR_MPHEVENT8       0x328
#define CSR_MPHEVENT9       0x329
#define CSR_MPHEVENT10      0x32a
#define CSR_MPHEVENT11      0x32b
#define CSR_MPHEVENT12      0x32c
#define CSR_MPHEVENT13      0x32d
#define CSR_MPHEVENT14      0x32e
#define CSR_MPHEVENT15      0x32f
#define CSR_MPHEVENT16      0x330
#define CSR_MPHEVENT17      0x331
#define CSR_MPHEVENT18      0x332
#define CSR_MPHEVENT19      0x333
#define CSR_MPHEVENT20      0x334
#define CSR_MPHEVENT21      0x335
#define CSR_MPHEVENT22      0x336
#define CSR_MPHEVENT23      0x337
#define CSR_MPHEVENT24      0x338
#define CSR_MPHEVENT25      0x339
#define CSR_MPHEVENT26      0x33a
#define CSR_MPHEVENT27      0x33b
#define CSR_MPHEVENT28      0x33c
#define CSR_MPHEVENT29      0x33d
#define CSR_MPHEVENT30      0x33e
#define CSR_MPHEVENT31      0x33f

/* Debug/Trace Registers */

#define CSR_TSELECT         0x7a0 /* Trigger Select */
#define CSR_TDATA1          0x7a1 /* Trigger Data 1 */
#define CSR_TDATA2          0x7a2 /* Trigger Data 2 */
#define CSR_TDATA3          0x7a3 /* Trigger Data 3 */
#define CSR_TINFO           0x7a4 /* Trigger Info */
#define CSR_TCONTROL        0x7a5 /* Trigger Control */
#define CSR_MCONTEXT        0x7a8 /* Machine Context */
#define CSR_MSCONTEXT       0x7aa /* Machine Supervisor Context */
#define CSR_SCONTEXT        0x5a8 /* Supervisor Context */
#define CSR_HCONTEXT        0x5aa /* Hypervisor Context */

/* Debug interface CSRs */

#define CSR_DCSR            0x7b0 /* Debug Control and Status */
#define CSR_DPC             0x7b1 /* Debug PC */
#define CSR_DSCRATCH0       0x7b2 /* Debug Scratch 0 */
#define CSR_DSCRATCH1       0x7b3 /* Debug Scratch 1 */

/* Vector CSRs */

#define CSR_VSTART          0x008 /* Vector Start Position */
#define CSR_VXSAT           0x009 /* Fixed-Point Saturate Flag */
#define CSR_VXRM            0x00a /* Fixed-Point Rounding Mode */
#define CSR_VCSR            0x00f /* Vector Control and Status */
#define CSR_VL              0xc20 /* Vector Length */
#define CSR_VTYPE           0xc21 /* Vector Data Type */
#define CSR_VLENB           0xc22 /* Vector Length in Bytes (VLEN/8) */

/* In mstatus register */

#define MSTATUS_UIE         (0x1 << 0)  /* User Interrupt Enable */
#define MSTATUS_SIE         (0x1 << 1)  /* Supervisor Interrupt Enable */
#define MSTATUS_MIE         (0x1 << 3)  /* Machine Interrupt Enable */
#define MSTATUS_SPIE        (0x1 << 5)  /* Supervisor Previous Interrupt Enable */
#define MSTATUS_MPIE        (0x1 << 7)  /* Machine Previous Interrupt Enable */
#define MSTATUS_SPPU        (0x0 << 8)  /* Supervisor Previous Privilege (u-mode) */
#define MSTATUS_SPPS        (0x1 << 8)  /* Supervisor Previous Privilege (s-mode) */
#define MSTATUS_VS          (0x3 << 9)  /* Machine Vector-extension Status */
#define MSTATUS_VS_INIT     (0x1 << 9)
#define MSTATUS_VS_CLEAN    (0x2 << 9)
#define MSTATUS_VS_DIRTY    (0x3 << 9)
#define MSTATUS_MPPU        (0x0 << 11) /* Machine Previous Privilege (u-mode) */
#define MSTATUS_MPPS        (0x1 << 11) /* Machine Previous Privilege (s-mode) */
#define MSTATUS_MPPM        (0x3 << 11) /* Machine Previous Privilege (m-mode) */
#define MSTATUS_MPP_MASK    (0x3 << 11)
#define MSTATUS_FS          (0x3 << 13) /* Machine Floating-point Status */
#define MSTATUS_FS_INIT     (0x1 << 13)
#define MSTATUS_FS_CLEAN    (0x2 << 13)
#define MSTATUS_FS_DIRTY    (0x3 << 13)
#define MSTATUS_XS          (0x3 << 15) /* Machine additional-extension Status */
#define MSTATUS_XS_INIT     (0x1 << 15)
#define MSTATUS_XS_CLEAN    (0x2 << 15)
#define MSTATUS_XS_DIRTY    (0x3 << 15)
#define MSTATUS_MPRV        (0x1 << 17) /* Modify Privilege */
#define MSTATUS_SUM         (0x1 << 18) /* S mode access to U mode memory */
#define MSTATUS_MXR         (0x1 << 19) /* Make executable / readable */
#define MSTATUS_TVM         (0x1 << 20) /* Trap access to satp from S mode */
#define MSTATUS_TW          (0x1 << 21) /* Trap WFI instruction from S mode */
#define MSTATUS_TSR         (0x1 << 22) /* Trap supervisor return (sret) */

/* Mask of preserved bits for mstatus */

#ifdef CONFIG_ARCH_RV32
#define MSTATUS_WPRI        (0xff << 23 | 0x15)
#else
#define MSTATUS_WPRI        (UINT64_C(0x1ffffff) << 38 | UINT64_C(0x1ff) << 23 | 0x15)
#endif

/* In menvcfg register */
#define MENVCFG_FIOM        (0x1 << 0)
#define MENVCFG_CBIE        (0x3 << 4)
#define MENVCFG_CBIE_ILL    (0x0 << 4)
#define MENVCFG_CBIE_FLUSH  (0x1 << 4)
#define MENVCFG_CBIE_INV    (0x3 << 4)
#define MENVCFG_CBCFE       (0x1 << 6)
#define MENVCFG_CBZE        (0x1 << 7)

#ifdef CONFIG_ARCH_RV32
#define MENVCFG_PBMTE       (0x1 << 30)
#define MENVCFG_STCE        (0x1 << 31)
#else
#define MENVCFG_PBMTE       (UINT64_C(0x1) << 62)
#define MENVCFG_STCE        (UINT64_C(0x1) << 63)
#endif

/* In mie (machine interrupt enable) register */

#define MIE_SSIE            (0x1 << 1)  /* Supervisor Software Interrupt Enable */
#define MIE_MSIE            (0x1 << 3)  /* Machine Software Interrupt Enable */
#define MIE_STIE            (0x1 << 5)  /* Supervisor Timer Interrupt Enable */
#define MIE_MTIE            (0x1 << 7)  /* Machine Timer Interrupt Enable */
#define MIE_SEIE            (0x1 << 9)  /* Supervisor External Interrupt Enable */
#define MIE_MEIE            (0x1 << 11) /* Machine External Interrupt Enable */

/* In mip (machine interrupt pending) register */

#define MIP_SSIP            (0x1 << 1)
#define MIP_MSIP            (0x1 << 3)
#define MIP_STIP            (0x1 << 5)
#define MIP_MTIP            (0x1 << 7)
#define MIP_SEIP            (0x1 << 9)
#define MIP_MEIP            (0x1 << 11)

/* In sstatus register (which is a view of mstatus) */

#define SSTATUS_SIE         MSTATUS_SIE
#define SSTATUS_SPIE        MSTATUS_SPIE
#define SSTATUS_SPPU        MSTATUS_SPPU
#define SSTATUS_SPPS        MSTATUS_SPPS
#define SSTATUS_VS          MSTATUS_VS
#define SSTATUS_VS_INIT     MSTATUS_VS_INIT
#define SSTATUS_VS_CLEAN    MSTATUS_VS_CLEAN
#define SSTATUS_VS_DIRTY    MSTATUS_VS_DIRTY
#define SSTATUS_FS          MSTATUS_FS
#define SSTATUS_FS_INIT     MSTATUS_FS_INIT
#define SSTATUS_FS_CLEAN    MSTATUS_FS_CLEAN
#define SSTATUS_FS_DIRTY    MSTATUS_FS_DIRTY
#define SSTATUS_XS          MSTATUS_XS
#define SSTATUS_XS_INIT     MSTATUS_XS_INIT
#define SSTATUS_XS_CLEAN    MSTATUS_XS_CLEAN
#define SSTATUS_XS_DIRTY    MSTATUS_XS_DIRTY
#define SSTATUS_SUM         MSTATUS_SUM
#define SSTATUS_MXR         MSTATUS_MXR

/* In sie register (which is a view of mie) */

#define SIE_SSIE            MIE_SSIE
#define SIE_STIE            MIE_STIE
#define SIE_SEIE            MIE_SEIE

/* In sip register (which is a view of mip) */

#define SIP_SSIP            MIP_SSIP
#define SIP_STIP            MIP_STIP
#define SIP_SEIP            MIP_SEIP

/* In senvcfg register */
#define SENVCFG_FIOM        MENVCFG_FIOM
#define SENVCFG_CBIE        MENVCFG_CBIE
#define SENVCFG_CBIE_ILL    MENVCFG_CBIE_ILL
#define SENVCFG_CBIE_FLUSH  MENVCFG_CBIE_FLUSH
#define SENVCFG_CBIE_INV    MENVCFG_CBIE_INV
#define SENVCFG_CBCFE       MENVCFG_CBCFE
#define SENVCFG_CBZE        MENVCFG_CBZE

/* In pmpcfg (PMP configuration) register */

#define PMPCFG_R            (1 << 0)  /* readable ? */
#define PMPCFG_W            (1 << 1)  /* writable ? */
#define PMPCFG_X            (1 << 2)  /* executable ? */
#define PMPCFG_RWX_MASK     (7 << 0)  /* access rights mask */
#define PMPCFG_A_OFF        (0 << 3)  /* null region (disabled) */
#define PMPCFG_A_TOR        (1 << 3)  /* top of range */
#define PMPCFG_A_NA4        (2 << 3)  /* naturally aligned four-byte region */
#define PMPCFG_A_NAPOT      (3 << 3)  /* naturally aligned power-of-two region */
#define PMPCFG_A_MASK       (3 << 3)  /* address-matching mode mask */
#define PMPCFG_L            (1 << 7)  /* locked ? */

/* In mcounteren/scounteren register */
#define COUNTEREN_CY       (0x1 << 0)
#define COUNTEREN_TM       (0x1 << 1)
#define COUNTEREN_IR       (0x1 << 2)
#define COUNTEREN_HPM3     (0x1 << 3)
#define COUNTEREN_HPM4     (0x1 << 4)
#define COUNTEREN_HPM5     (0x1 << 5)
#define COUNTEREN_HPM6     (0x1 << 6)
#define COUNTEREN_HPM7     (0x1 << 7)
#define COUNTEREN_HPM8     (0x1 << 8)
#define COUNTEREN_HPM9     (0x1 << 9)
#define COUNTEREN_HPM10    (0x1 << 10)
#define COUNTEREN_HPM11    (0x1 << 11)
#define COUNTEREN_HPM12    (0x1 << 12)
#define COUNTEREN_HPM13    (0x1 << 13)
#define COUNTEREN_HPM14    (0x1 << 14)
#define COUNTEREN_HPM15    (0x1 << 15)
#define COUNTEREN_HPM16    (0x1 << 16)
#define COUNTEREN_HPM17    (0x1 << 17)
#define COUNTEREN_HPM18    (0x1 << 18)
#define COUNTEREN_HPM19    (0x1 << 19)
#define COUNTEREN_HPM20    (0x1 << 20)
#define COUNTEREN_HPM21    (0x1 << 21)
#define COUNTEREN_HPM22    (0x1 << 22)
#define COUNTEREN_HPM23    (0x1 << 23)
#define COUNTEREN_HPM24    (0x1 << 24)
#define COUNTEREN_HPM25    (0x1 << 25)
#define COUNTEREN_HPM26    (0x1 << 26)
#define COUNTEREN_HPM27    (0x1 << 27)
#define COUNTEREN_HPM28    (0x1 << 28)
#define COUNTEREN_HPM29    (0x1 << 29)
#define COUNTEREN_HPM30    (0x1 << 30)
#define COUNTEREN_HPM31    (0x1 << 31)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_RISCV_INCLUDE_CSR_H */
