/****************************************************************************
 * arch/risc-v/include/thead/c9xx_csr.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_RISCV_INCLUDE_THEAD_C9XX_CSR_H
#define __ARCH_RISCV_INCLUDE_THEAD_C9XX_CSR_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* T-HEAD C9xx Machine Control and Status extension Registers  */
#define THEAD_CSR_MXSTATUS          0x7c0
#define THEAD_CSR_MHCR              0x7c1
#define THEAD_CSR_MCOR              0x7c2
#define THEAD_CSR_MCCR2             0x7c3
#define THEAD_CSR_MCER2             0x7c4
#define THEAD_CSR_MHINT             0x7c5
#define THEAD_CSR_MRMR              0x7c6
#define THEAD_CSR_MRVBR             0x7c7
#define THEAD_CSR_MCER              0x7c8
#define THEAD_CSR_MCOUNTERWEN       0x7c9
#define THEAD_CSR_MCOUNTERINTEN     0x7ca
#define THEAD_CSR_MCOUNTEROF        0x7cb
#define THEAD_CSR_MHINT2            0x7cc
#define THEAD_CSR_MHINT3            0x7cd
#define THEAD_CSR_MRADDR            0x7e0
#define THEAD_CSR_MEXSTATUS         0x7e1
#define THEAD_CSR_MNMICAUSE         0x7e2
#define THEAD_CSR_MNMIPC            0x7e3
#define THEAD_CSR_MHPMCR            0x7f0
#define THEAD_CSR_MHPMSR            0x7f1
#define THEAD_CSR_MHPMER            0x7f2
#define THEAD_CSR_MTEECFG           0x7f4
#define THEAD_CSR_MZONEID           0x7f5
#define THEAD_CSR_ML2CPID           0x7f6
#define THEAD_CSR_ML2WP             0x7f7
#define THEAD_CSR_MDTCMCR           0x7f8
#define THEAD_CSR_USP               0x7d1
#define THEAD_CSR_MEICR             0x7d6
#define THEAD_CSR_MEICR2            0x7d7
#define THEAD_CSR_MBEADDR           0x7d8
#define THEAD_CSR_MWMSR             0xfc2

/* T-HEAD C9xx Machine Cache Access extension Registers  */
#define THEAD_CSR_MCINS             0x7d2
#define THEAD_CSR_MCINDEX           0x7d3
#define THEAD_CSR_MCDATA0           0x7d4
#define THEAD_CSR_MCDATA1           0x7d5

/* T-HEAD C9xx Machine CPU model extension Registers  */
#define THEAD_CSR_MCPUID            0xfc0
#define THEAD_CSR_MAPBADDR          0xfc1

/* T-HEAD C9xx Machine Multi-core extension Registers  */
#define THEAD_CSR_MSMPR             0x7f3

/* T-HEAD C9xx Machine Debug Registers.  */
#define THEAD_CSR_MHALTCAUSE        0xfe0
#define THEAD_CSR_MDBGINFO          0xfe1
#define THEAD_CSR_MPCFIFO           0xfe2

/* T-HEAD C9xx Supervisor Control and Status extension Registers  */
#define THEAD_CSR_SXSTATUS          0x5c0
#define THEAD_CSR_SHCR              0x5c1
#define THEAD_CSR_SCER2             0x5c2
#define THEAD_CSR_SCER              0x5c3
#define THEAD_CSR_SCOUNTERINTEN     0x5c4
#define THEAD_CSR_SCOUNTEROF        0x5c5
#define THEAD_CSR_SHINT             0x5c6
#define THEAD_CSR_SHINT2            0x5c7
#define THEAD_CSR_SHPMINHIBIT       0x5c8
#define THEAD_CSR_SHPMCR            0x5c9
#define THEAD_CSR_SHPMSR            0x5ca
#define THEAD_CSR_SHPMER            0x5cb
#define THEAD_CSR_SL2CPID           0x5cc
#define THEAD_CSR_SL2WP             0x5cd
#define THEAD_CSR_SBEADDR           0x5d0
#define THEAD_CSR_SCYCLE            0x5e0
#define THEAD_CSR_SINSTRET          0x5e2
#define THEAD_CSR_SHPMCOUNTER1      0x5e1
#define THEAD_CSR_SHPMCOUNTER2      0x5e2
#define THEAD_CSR_SHPMCOUNTER3      0x5e3
#define THEAD_CSR_SHPMCOUNTER4      0x5e4
#define THEAD_CSR_SHPMCOUNTER5      0x5e5
#define THEAD_CSR_SHPMCOUNTER6      0x5e6
#define THEAD_CSR_SHPMCOUNTER7      0x5e7
#define THEAD_CSR_SHPMCOUNTER8      0x5e8
#define THEAD_CSR_SHPMCOUNTER9      0x5e9
#define THEAD_CSR_SHPMCOUNTER10     0x5ea
#define THEAD_CSR_SHPMCOUNTER11     0x5eb
#define THEAD_CSR_SHPMCOUNTER12     0x5ec
#define THEAD_CSR_SHPMCOUNTER13     0x5ed
#define THEAD_CSR_SHPMCOUNTER14     0x5ee
#define THEAD_CSR_SHPMCOUNTER15     0x5ef
#define THEAD_CSR_SHPMCOUNTER16     0x5f0
#define THEAD_CSR_SHPMCOUNTER17     0x5f1
#define THEAD_CSR_SHPMCOUNTER18     0x5f2
#define THEAD_CSR_SHPMCOUNTER19     0x5f3
#define THEAD_CSR_SHPMCOUNTER20     0x5f4
#define THEAD_CSR_SHPMCOUNTER21     0x5f5
#define THEAD_CSR_SHPMCOUNTER22     0x5f6
#define THEAD_CSR_SHPMCOUNTER23     0x5f7
#define THEAD_CSR_SHPMCOUNTER24     0x5f8
#define THEAD_CSR_SHPMCOUNTER25     0x5f9
#define THEAD_CSR_SHPMCOUNTER26     0x5fa
#define THEAD_CSR_SHPMCOUNTER27     0x5fb
#define THEAD_CSR_SHPMCOUNTER28     0x5fc
#define THEAD_CSR_SHPMCOUNTER29     0x5fd
#define THEAD_CSR_SHPMCOUNTER30     0x5fe
#define THEAD_CSR_SHPMCOUNTER31     0x5ff

/* T-HEAD C9xx Supervisor MMU extension Registers  */
#define THEAD_CSR_SMIR              0x9c0
#define THEAD_CSR_SMEL              0x9c1
#define THEAD_CSR_SMEH              0x9c2
#define THEAD_CSR_SMCIR             0x9c3

/* T-HEAD C9xx User Floating-point control Registers.  */
#define THEAD_CSR_FXCR              0x800

/* In mxstatus register */

/* U-mode performance monitoring count enable */
#define THEAD_MXSTATUS_PMDU         (1 << 10)
/* S-mode performance monitoring count enable */
#define THEAD_MXSTATUS_PMDS         (1 << 11)
/* M-mode performance monitoring count enable */
#define THEAD_MXSTATUS_PMDM         (1 << 13)
/* PMP minimum granularity control */
#define THEAD_MXSTATUS_PMP4K        (1 << 14)
/* Misaligned access enable */
#define THEAD_MXSTATUS_MM           (1 << 15)
/* Execute extended cache instructions in U-mode */
#define THEAD_MXSTATUS_UCME         (1 << 16)
/* Clint timer/software interrupt supervisor extension enable */
#define THEAD_MXSTATUS_CLINTEE      (1 << 17)
/* Disable hardware writeback */
#define THEAD_MXSTATUS_MHRD         (1 << 18)
/* Disable Icache snoop D-Cache */
#define THEAD_MXSTATUS_INSDE        (1 << 19)
/* Extend MMU address attribute */
#define THEAD_MXSTATUS_MAEE         (1 << 21)
/* Enables extended instruction sets */
#define THEAD_MXSTATUS_ISAEE        (1 << 22)

#endif /* __ARCH_RISCV_INCLUDE_THEAD_C9XX_CSR_H */
