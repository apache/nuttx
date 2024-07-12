/****************************************************************************
 * arch/tricore/include/tc3xx/irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_TRICORE_INCLUDE_TC3XX_IRQ_H
#define __ARCH_TRICORE_INCLUDE_TC3XX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Upper CSA */

#define REG_UPCXI        0
#define REG_PSW          1
#define REG_A10          2
#define REG_UA11         3
#define REG_D8           4
#define REG_D9           5
#define REG_D10          6
#define REG_D11          7
#define REG_A12          8
#define REG_A13          9
#define REG_A14          10
#define REG_A15          11
#define REG_D12          12
#define REG_D13          13
#define REG_D14          14
#define REG_D15          15

/* Lower CSA */

#define REG_LPCXI        0
#define REG_LA11         1
#define REG_A2           2
#define REG_A3           3
#define REG_D0           4
#define REG_D1           5
#define REG_D2           6
#define REG_D3           7
#define REG_A4           8
#define REG_A5           9
#define REG_A6           10
#define REG_A7           11
#define REG_D4           12
#define REG_D5           13
#define REG_D6           14
#define REG_D7           15

#define REG_RA           REG_UA11
#define REG_SP           REG_A10
#define REG_UPC          REG_UA11

#define REG_LPC          REG_LA11

#define TC_CONTEXT_REGS  (16)

#define XCPTCONTEXT_REGS (TC_CONTEXT_REGS)
#define XCPTCONTEXT_SIZE (sizeof(void *) * TC_CONTEXT_REGS)

#define NR_IRQS          (255)

/* PSW: Program Status Word Register */

#define PSW_CDE         (1 << 7) /* Bits 7: Call Depth Count Enable */
#define PSW_IS          (1 << 9) /* Bits 9: Interrupt Stack Control */
#define PSW_IO          (10)     /* Bits 10-11: Access Privilege Level Control (I/O Privilege) */
#  define PSW_IO_USER0      (0 << PSW_IO)
#  define PSW_IO_USER1      (1 << PSW_IO)
#  define PSW_IO_SUPERVISOR (2 << PSW_IO)

/* PCXI: Previous Context Information and Pointer Register */

#define PCXI_UL         (1 << 20) /* Bits 20: Upper or Lower Context Tag */
#define PCXI_PIE        (1 << 21) /* Bits 21: Previous Interrupt Enable */

/* FCX: Free CSA List Head Pointer Register */

#define FCX_FCXO        (0)       /* Bits 0-15: FCX Offset Address */
#define FCX_FCXS        (16)      /* Bits 16-19: FCX Segment Address */
#define FCX_FCXO_MASK   (0xffff << FCX_FCXO)
#define FCX_FCXS_MASK   (0xf    << FCX_FCXS)
#define FCX_FREE        (FCX_FCXS_MASK | FCX_FCXO_MASK) /* Free CSA manipulation */

/** Ifx CPUs register */

#ifndef IFXCPU_REG_H

/** \brief 1030, CPUx SRI Error Generation Register */
#define CPU_SEGEN 0x1030

/** \brief 8004, CPUx Task Address Space Identifier Register */
#define CPU_TASK_ASI 0x8004

/** \brief 8100, CPUx Data Access CacheabilityRegister */
#define CPU_PMA0 0x8100

/** \brief 8104, CPUx Code Access CacheabilityRegister */
#define CPU_PMA1 0x8104

/** \brief 8108, CPUx  Peripheral Space Identifier register */
#define CPU_PMA2 0x8108

/** \brief 9000, CPUx Data Control Register 2 */
#define CPU_DCON2 0x9000

/** \brief 900C, CPUx SIST Mode Access Control Register */
#define CPU_SMACON 0x900C

/** \brief 9010, CPUx Data Synchronous Trap Register */
#define CPU_DSTR 0x9010

/** \brief 9018, CPUx Data Asynchronous Trap Register */
#define CPU_DATR 0x9018

/** \brief 901C, CPUx Data Error Address Register */
#define CPU_DEADD 0x901C

/** \brief 9020, CPUx Data Integrity Error Address Register */
#define CPU_DIEAR 0x9020

/** \brief 9024, CPUx Data Integrity Error Trap Register */
#define CPU_DIETR 0x9024

/** \brief 9040, CPUx Data Memory Control Register */
#define CPU_DCON0 0x9040

/** \brief 9200, CPUx Program Synchronous Trap Register */
#define CPU_PSTR 0x9200

/** \brief 9204, CPUx Program Control 1 */
#define CPU_PCON1 0x9204

/** \brief 9208, CPUx Program Control 2 */
#define CPU_PCON2 0x9208

/** \brief 920C, CPUx Program Control 0 */
#define CPU_PCON0 0x920C

/** \brief 9210, CPUx Program Integrity Error Address Register */
#define CPU_PIEAR 0x9210

/** \brief 9214, CPUx Program Integrity Error Trap Register */
#define CPU_PIETR 0x9214

/** \brief 9400, CPUx Compatibility Control Register */
#define CPU_COMPAT 0x9400

/** \brief A000, CPUx Trap Control Register */
#define CPU_FPU_TRAP_CON 0xA000

/** \brief A004, CPUx Trapping Instruction Program Counter Register */
#define CPU_FPU_TRAP_PC 0xA004

/** \brief A008, CPUx Trapping Instruction Opcode Register */
#define CPU_FPU_TRAP_OPC 0xA008

/** \brief A010, CPUx Trapping Instruction Operand Register */
#define CPU_FPU_TRAP_SRC1 0xA010

/** \brief A014, CPUx Trapping Instruction Operand Register */
#define CPU_FPU_TRAP_SRC2 0xA014

/** \brief A018, CPUx Trapping Instruction Operand Register */
#define CPU_FPU_TRAP_SRC3 0xA018

/** \brief C000, CPUx Data Protection Range 0, Lower Bound Register */
#define CPU_DPR0_L 0xC000

/** \brief C004, CPUx Data Protection Range 0, Upper Bound Register */
#define CPU_DPR0_U 0xC004

/** \brief C008, CPUx Data Protection Range 1, Lower Bound Register */
#define CPU_DPR1_L 0xC008

/** \brief C00C, CPUx Data Protection Range 1, Upper Bound Register */
#define CPU_DPR1_U 0xC00C

/** \brief C010, CPUx Data Protection Range 2, Lower Bound Register */
#define CPU_DPR2_L 0xC010

/** \brief C014, CPUx Data Protection Range 2, Upper Bound Register */
#define CPU_DPR2_U 0xC014

/** \brief C018, CPUx Data Protection Range 3, Lower Bound Register */
#define CPU_DPR3_L 0xC018

/** \brief C01C, CPUx Data Protection Range 3, Upper Bound Register */
#define CPU_DPR3_U 0xC01C

/** \brief C020, CPUx Data Protection Range 4, Lower Bound Register */
#define CPU_DPR4_L 0xC020

/** \brief C024, CPUx Data Protection Range 4, Upper Bound Register */
#define CPU_DPR4_U 0xC024

/** \brief C028, CPUx Data Protection Range 5, Lower Bound Register */
#define CPU_DPR5_L 0xC028

/** \brief C02C, CPUx Data Protection Range 5, Upper Bound Register */
#define CPU_DPR5_U 0xC02C

/** \brief C030, CPUx Data Protection Range 6, Lower Bound Register */
#define CPU_DPR6_L 0xC030

/** \brief C034, CPUx Data Protection Range 6, Upper Bound Register */
#define CPU_DPR6_U 0xC034

/** \brief C038, CPUx Data Protection Range 7, Lower Bound Register */
#define CPU_DPR7_L 0xC038

/** \brief C03C, CPUx Data Protection Range 7, Upper Bound Register */
#define CPU_DPR7_U 0xC03C

/** \brief C040, CPUx Data Protection Range 8, Lower Bound Register */
#define CPU_DPR8_L 0xC040

/** \brief C044, CPUx Data Protection Range 8, Upper Bound Register */
#define CPU_DPR8_U 0xC044

/** \brief C048, CPUx Data Protection Range 9, Lower Bound Register */
#define CPU_DPR9_L 0xC048

/** \brief C04C, CPUx Data Protection Range 9, Upper Bound Register */
#define CPU_DPR9_U 0xC04C

/** \brief C050, CPUx Data Protection Range 10, Lower Bound Register */
#define CPU_DPR10_L 0xC050

/** \brief C054, CPUx Data Protection Range 10, Upper Bound Register */
#define CPU_DPR10_U 0xC054

/** \brief C058, CPUx Data Protection Range 11, Lower Bound Register */
#define CPU_DPR11_L 0xC058

/** \brief C05C, CPUx Data Protection Range 11, Upper Bound Register */
#define CPU_DPR11_U 0xC05C

/** \brief C060, CPUx Data Protection Range 12, Lower Bound Register */
#define CPU_DPR12_L 0xC060

/** \brief C064, CPUx Data Protection Range 12, Upper Bound Register */
#define CPU_DPR12_U 0xC064

/** \brief C068, CPUx Data Protection Range 13, Lower Bound Register */
#define CPU_DPR13_L 0xC068

/** \brief C06C, CPUx Data Protection Range 13, Upper Bound Register */
#define CPU_DPR13_U 0xC06C

/** \brief C070, CPUx Data Protection Range 14, Lower Bound Register */
#define CPU_DPR14_L 0xC070

/** \brief C074, CPUx Data Protection Range 14, Upper Bound Register */
#define CPU_DPR14_U 0xC074

/** \brief C078, CPUx Data Protection Range 15, Lower Bound Register */
#define CPU_DPR15_L 0xC078

/** \brief C07C, CPUx Data Protection Range 15, Upper Bound Register */
#define CPU_DPR15_U 0xC07C

/** \brief C080, CPUx Data Protection Range 16, Lower Bound Register */
#define CPU_DPR16_L 0xC080

/** \brief C084, CPUx Data Protection Range 16, Upper Bound Register */
#define CPU_DPR16_U 0xC084

/** \brief C088, CPUx Data Protection Range 17, Lower Bound Register */
#define CPU_DPR17_L 0xC088

/** \brief C08C, CPUx Data Protection Range 17, Upper Bound Register */
#define CPU_DPR17_U 0xC08C

/** \brief D000, CPUx Code Protection Range 0 Lower Bound Register */
#define CPU_CPR0_L 0xD000

/** \brief D004, CPUx Code Protection Range 0 Upper Bound Register */
#define CPU_CPR0_U 0xD004

/** \brief D008, CPUx Code Protection Range 1 Lower Bound Register */
#define CPU_CPR1_L 0xD008

/** \brief D00C, CPUx Code Protection Range 1 Upper Bound Register */
#define CPU_CPR1_U 0xD00C

/** \brief D010, CPUx Code Protection Range 2 Lower Bound Register */
#define CPU_CPR2_L 0xD010

/** \brief D014, CPUx Code Protection Range 2 Upper Bound Register */
#define CPU_CPR2_U 0xD014

/** \brief D018, CPUx Code Protection Range 3 Lower Bound Register */
#define CPU_CPR3_L 0xD018

/** \brief D01C, CPUx Code Protection Range 3 Upper Bound Register */
#define CPU_CPR3_U 0xD01C

/** \brief D020, CPUx Code Protection Range 4 Lower Bound Register */
#define CPU_CPR4_L 0xD020

/** \brief D024, CPUx Code Protection Range 4 Upper Bound Register */
#define CPU_CPR4_U 0xD024

/** \brief D028, CPUx Code Protection Range 5 Lower Bound Register */
#define CPU_CPR5_L 0xD028

/** \brief D02C, CPUx Code Protection Range 5 Upper Bound Register */
#define CPU_CPR5_U 0xD02C

/** \brief D030, CPUx Code Protection Range 6 Lower Bound Register */
#define CPU_CPR6_L 0xD030

/** \brief D034, CPUx Code Protection Range 6 Upper Bound Register */
#define CPU_CPR6_U 0xD034

/** \brief D038, CPUx Code Protection Range 7 Lower Bound Register */
#define CPU_CPR7_L 0xD038

/** \brief D03C, CPUx Code Protection Range 7 Upper Bound Register */
#define CPU_CPR7_U 0xD03C

/** \brief D040, CPUx Code Protection Range 8 Lower Bound Register */
#define CPU_CPR8_L 0xD040

/** \brief D044, CPUx Code Protection Range 8 Upper Bound Register */
#define CPU_CPR8_U 0xD044

/** \brief D048, CPUx Code Protection Range 9 Lower Bound Register */
#define CPU_CPR9_L 0xD048

/** \brief D04C, CPUx Code Protection Range 9 Upper Bound Register */
#define CPU_CPR9_U 0xD04C

/** \brief E000, CPUx Code Protection Execute Enable Register Set 0 */
#define CPU_CPXE_0 0xE000

/** \brief E004, CPUx Code Protection Execute Enable Register Set 1 */
#define CPU_CPXE_1 0xE004

/** \brief E008, CPUx Code Protection Execute Enable Register Set 2 */
#define CPU_CPXE_2 0xE008

/** \brief E00C, CPUx Code Protection Execute Enable Register Set 3 */
#define CPU_CPXE_3 0xE00C

/** \brief E010, CPUx Data Protection Read Enable Register Set 0 */
#define CPU_DPRE_0 0xE010

/** \brief E014, CPUx Data Protection Read Enable Register Set 1 */
#define CPU_DPRE_1 0xE014

/** \brief E018, CPUx Data Protection Read Enable Register Set 2 */
#define CPU_DPRE_2 0xE018

/** \brief E01C, CPUx Data Protection Read Enable Register Set 3 */
#define CPU_DPRE_3 0xE01C

/** \brief E020, CPUx Data Protection Write Enable Register Set 0 */
#define CPU_DPWE_0 0xE020

/** \brief E024, CPUx Data Protection Write Enable Register Set 1 */
#define CPU_DPWE_1 0xE024

/** \brief E028, CPUx Data Protection Write Enable Register Set 2 */
#define CPU_DPWE_2 0xE028

/** \brief E02C, CPUx Data Protection Write Enable Register Set 3 */
#define CPU_DPWE_3 0xE02C

/** \brief E040, CPUx Code Protection Execute Enable Register Set 4 */
#define CPU_CPXE_4 0xE040

/** \brief E044, CPUx Code Protection Execute Enable Register Set 5 */
#define CPU_CPXE_5 0xE044

/** \brief E050, CPUx Data Protection Read Enable Register Set 4 */
#define CPU_DPRE_4 0xE050

/** \brief E054, CPUx Data Protection Read Enable Register Set 5 */
#define CPU_DPRE_5 0xE054

/** \brief E060, CPUx Data Protection Write Enable Register Set 4 */
#define CPU_DPWE_4 0xE060

/** \brief E064, CPUx Data Protection Write Enable Register Set 5 */
#define CPU_DPWE_5 0xE064

/** \brief E400, CPUx Temporal Protection System Control Register */
#define CPU_TPS_CON 0xE400

/** \brief E404, CPUx Temporal Protection System Timer Register 0 */
#define CPU_TPS_TIMER0 0xE404

/** \brief E408, CPUx Temporal Protection System Timer Register 1 */
#define CPU_TPS_TIMER1 0xE408

/** \brief E40C, CPUx Temporal Protection System Timer Register 2 */
#define CPU_TPS_TIMER2 0xE40C

/** \brief E440, CPUx Exception Entry Timer Load Value */
#define CPU_TPS_EXTIM_ENTRY_LVAL 0xE440

/** \brief E444, CPUx Exception Entry Timer Current Value */
#define CPU_TPS_EXTIM_ENTRY_CVAL 0xE444

/** \brief E448, CPUx Exception Exit  Timer Load Value */
#define CPU_TPS_EXTIM_EXIT_LVAL 0xE448

/** \brief E44C, CPUx Exception Exit Timer Current Value */
#define CPU_TPS_EXTIM_EXIT_CVAL 0xE44C

/** \brief E450, CPUx Exception Timer Class Enable Register */
#define CPU_TPS_EXTIM_CLASS_EN 0xE450

/** \brief E454, CPUx Exception Timer Status Register */
#define CPU_TPS_EXTIM_STAT 0xE454

/** \brief E458, CPUx Exception Timer FCX Register */
#define CPU_TPS_EXTIM_FCX 0xE458

/** \brief F000, CPUx Trigger Event 0 */
#define CPU_TR0_EVT 0xF000
/* Alias (User Manual Name) for CPU_TR0_EVT.
 * To use register names with standard convension, please use CPU_TR0_EVT.
 */
#define CPU_TR0EVT (CPU_TR0_EVT)

/** \brief F004, CPUx Trigger Address 0 */
#define CPU_TR0_ADR 0xF004
/** Alias (User Manual Name) for CPU_TR0_ADR.
 * To use register names with standard convension, please use CPU_TR0_ADR.
 */
#define CPU_TR0ADR (CPU_TR0_ADR)

/** \brief F008, CPUx Trigger Event 1 */
#define CPU_TR1_EVT 0xF008
/** Alias (User Manual Name) for CPU_TR1_EVT.
 * To use register names with standard convension, please use CPU_TR1_EVT.
 */
#define CPU_TR1EVT (CPU_TR1_EVT)

/** \brief F00C, CPUx Trigger Address 1 */
#define CPU_TR1_ADR 0xF00C
/** Alias (User Manual Name) for CPU_TR1_ADR.
 * To use register names with standard convension, please use CPU_TR1_ADR.
 */
#define CPU_TR1ADR (CPU_TR1_ADR)

/** \brief F010, CPUx Trigger Event 2 */
#define CPU_TR2_EVT 0xF010
/** Alias (User Manual Name) for CPU_TR2_EVT.
 * To use register names with standard convension, please use CPU_TR2_EVT.
 */
#define CPU_TR2EVT (CPU_TR2_EVT)

/** \brief F014, CPUx Trigger Address 2 */
#define CPU_TR2_ADR 0xF014
/** Alias (User Manual Name) for CPU_TR2_ADR.
 * To use register names with standard convension, please use CPU_TR2_ADR.
 */
#define CPU_TR2ADR (CPU_TR2_ADR)

/** \brief F018, CPUx Trigger Event 3 */
#define CPU_TR3_EVT 0xF018
/** Alias (User Manual Name) for CPU_TR3_EVT.
 * To use register names with standard convension, please use CPU_TR3_EVT.
 */
#define CPU_TR3EVT (CPU_TR3_EVT)

/** \brief F01C, CPUx Trigger Address 3 */
#define CPU_TR3_ADR 0xF01C
/** Alias (User Manual Name) for CPU_TR3_ADR.
 * To use register names with standard convension, please use CPU_TR3_ADR.
 */
#define CPU_TR3ADR (CPU_TR3_ADR)

/** \brief F020, CPUx Trigger Event 4 */
#define CPU_TR4_EVT 0xF020
/** Alias (User Manual Name) for CPU_TR4_EVT.
 * To use register names with standard convension, please use CPU_TR4_EVT.
 */
#define CPU_TR4EVT (CPU_TR4_EVT)

/** \brief F024, CPUx Trigger Address 4 */
#define CPU_TR4_ADR 0xF024
/** Alias (User Manual Name) for CPU_TR4_ADR.
 * To use register names with standard convension, please use CPU_TR4_ADR.
 */
#define CPU_TR4ADR (CPU_TR4_ADR)

/** \brief F028, CPUx Trigger Event 5 */
#define CPU_TR5_EVT 0xF028
/** Alias (User Manual Name) for CPU_TR5_EVT.
 * To use register names with standard convension, please use CPU_TR5_EVT.
 */
#define CPU_TR5EVT (CPU_TR5_EVT)

/** \brief F02C, CPUx Trigger Address 5 */
#define CPU_TR5_ADR 0xF02C
/** Alias (User Manual Name) for CPU_TR5_ADR.
 * To use register names with standard convension, please use CPU_TR5_ADR.
 */
#define CPU_TR5ADR (CPU_TR5_ADR)

/** \brief F030, CPUx Trigger Event 6 */
#define CPU_TR6_EVT 0xF030
/** Alias (User Manual Name) for CPU_TR6_EVT.
 * To use register names with standard convension, please use CPU_TR6_EVT.
 */
#define CPU_TR6EVT (CPU_TR6_EVT)

/** \brief F034, CPUx Trigger Address 6 */
#define CPU_TR6_ADR 0xF034
/** Alias (User Manual Name) for CPU_TR6_ADR.
 * To use register names with standard convension, please use CPU_TR6_ADR.
 */
#define CPU_TR6ADR (CPU_TR6_ADR)

/** \brief F038, CPUx Trigger Event 7 */
#define CPU_TR7_EVT 0xF038
/** Alias (User Manual Name) for CPU_TR7_EVT.
 * To use register names with standard convension, please use CPU_TR7_EVT.
 */
#define CPU_TR7EVT (CPU_TR7_EVT)

/** \brief F03C, CPUx Trigger Address 7 */
#define CPU_TR7_ADR 0xF03C
/** Alias (User Manual Name) for CPU_TR7_ADR.
 * To use register names with standard convension, please use CPU_TR7_ADR.
 */
#define CPU_TR7ADR (CPU_TR7_ADR)

/** \brief FC00, CPUx Counter Control */
#define CPU_CCTRL 0xFC00

/** \brief FC04, CPUx CPU Clock Cycle Count */
#define CPU_CCNT 0xFC04

/** \brief FC08, CPUx Instruction Count */
#define CPU_ICNT 0xFC08

/** \brief FC0C, CPUx Multi-Count Register 1 */
#define CPU_M1CNT 0xFC0C

/** \brief FC10, CPUx Multi-Count Register 2 */
#define CPU_M2CNT 0xFC10

/** \brief FC14, CPUx Multi-Count Register 3 */
#define CPU_M3CNT 0xFC14

/** \brief FD00, CPUx Debug Status Register */
#define CPU_DBGSR 0xFD00

/** \brief FD08, CPUx External Event Register */
#define CPU_EXEVT 0xFD08

/** \brief FD0C, CPUx Core Register Access Event */
#define CPU_CREVT 0xFD0C

/** \brief FD10, CPUx Software Debug Event */
#define CPU_SWEVT 0xFD10

/** \brief FD30, CPUx TriggerAddressx */
#define CPU_TRIG_ACC 0xFD30

/** \brief FD40, CPUx Debug Monitor Start Address */
#define CPU_DMS 0xFD40

/** \brief FD44, CPUx Debug Context Save Area Pointer */
#define CPU_DCX 0xFD44

/** \brief FD48, CPUx Debug Trap Control Register */
#define CPU_DBGTCR 0xFD48

/** \brief FE00, CPUx Previous Context Information Register */
#define CPU_PCXI 0xFE00

/** \brief FE04, CPUx Program Status Word */
#define CPU_PSW 0xFE04

/** \brief FE08, CPUx Program Counter */
#define CPU_PC 0xFE08

/** \brief FE14, CPUx System Configuration Register */
#define CPU_SYSCON 0xFE14

/** \brief FE18, CPUx Identification Register TC1.6.2P */
#define CPU_CPU_ID 0xFE18

/** \brief FE1C, CPUx Core Identification Register */
#define CPU_CORE_ID 0xFE1C

/** \brief FE20, CPUx Base Interrupt Vector Table Pointer */
#define CPU_BIV 0xFE20

/** \brief FE24, CPUx Base Trap Vector Table Pointer */
#define CPU_BTV 0xFE24

/** \brief FE28, CPUx Interrupt Stack Pointer */
#define CPU_ISP 0xFE28

/** \brief FE2C, CPUx Interrupt Control Register */
#define CPU_ICR 0xFE2C

/** \brief FE38, CPUx Free CSA List Head Pointer */
#define CPU_FCX 0xFE38

/** \brief FE3C, CPUx Free CSA List Limit Pointer */
#define CPU_LCX 0xFE3C

/** \brief FE50, CPUx Customer ID register */
#define CPU_CUS_ID 0xFE50

/** \brief FF00, CPUx Data General Purpose Register 0 */
#define CPU_D0 0xFF00

/** \brief FF04, CPUx Data General Purpose Register 1 */
#define CPU_D1 0xFF04

/** \brief FF08, CPUx Data General Purpose Register 2 */
#define CPU_D2 0xFF08

/** \brief FF0C, CPUx Data General Purpose Register 3 */
#define CPU_D3 0xFF0C

/** \brief FF10, CPUx Data General Purpose Register 4 */
#define CPU_D4 0xFF10

/** \brief FF14, CPUx Data General Purpose Register 5 */
#define CPU_D5 0xFF14

/** \brief FF18, CPUx Data General Purpose Register 6 */
#define CPU_D6 0xFF18

/** \brief FF1C, CPUx Data General Purpose Register 7 */
#define CPU_D7 0xFF1C

/** \brief FF20, CPUx Data General Purpose Register 8 */
#define CPU_D8 0xFF20

/** \brief FF24, CPUx Data General Purpose Register 9 */
#define CPU_D9 0xFF24

/** \brief FF28, CPUx Data General Purpose Register 10 */
#define CPU_D10 0xFF28

/** \brief FF2C, CPUx Data General Purpose Register 11 */
#define CPU_D11 0xFF2C

/** \brief FF30, CPUx Data General Purpose Register 12 */
#define CPU_D12 0xFF30

/** \brief FF34, CPUx Data General Purpose Register 13 */
#define CPU_D13 0xFF34

/** \brief FF38, CPUx Data General Purpose Register 14 */
#define CPU_D14 0xFF38

/** \brief FF3C, CPUx Data General Purpose Register 15 */
#define CPU_D15 0xFF3C

/** \brief FF80, CPUx Address General Purpose Register 0 */
#define CPU_A0 0xFF80

/** \brief FF84, CPUx Address General Purpose Register 1 */
#define CPU_A1 0xFF84

/** \brief FF88, CPUx Address General Purpose Register 2 */
#define CPU_A2 0xFF88

/** \brief FF8C, CPUx Address General Purpose Register 3 */
#define CPU_A3 0xFF8C

/** \brief FF90, CPUx Address General Purpose Register 4 */
#define CPU_A4 0xFF90

/** \brief FF94, CPUx Address General Purpose Register 5 */
#define CPU_A5 0xFF94

/** \brief FF98, CPUx Address General Purpose Register 6 */
#define CPU_A6 0xFF98

/** \brief FF9C, CPUx Address General Purpose Register 7 */
#define CPU_A7 0xFF9C

/** \brief FFA0, CPUx Address General Purpose Register 8 */
#define CPU_A8 0xFFA0

/** \brief FFA4, CPUx Address General Purpose Register 9 */
#define CPU_A9 0xFFA4

/** \brief FFA8, CPUx Address General Purpose Register 10 */
#define CPU_A10 0xFFA8

/** \brief FFAC, CPUx Address General Purpose Register 11 */
#define CPU_A11 0xFFAC

/** \brief FFB0, CPUx Address General Purpose Register 12 */
#define CPU_A12 0xFFB0

/** \brief FFB4, CPUx Address General Purpose Register 13 */
#define CPU_A13 0xFFB4

/** \brief FFB8, CPUx Address General Purpose Register 14 */
#define CPU_A14 0xFFB8

/** \brief FFBC, CPUx Address General Purpose Register 15 */
#define CPU_A15 0xFFBC

#endif /* IFXCPU_REG_H */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
struct xcptcontext
{
  /* The following function pointer is non-zero if there are pending signals
   * to be processed.
   */

  void *sigdeliver; /* Actual type is sig_deliver_t */

  /* These are saved copies of the context used during
   * signal processing.
   */

  uintptr_t *saved_regs;

  /* Register save area with XCPTCONTEXT_SIZE, only valid when:
   * 1.The task isn't running or
   * 2.The task is interrupted
   * otherwise task is running, and regs contain the stale value.
   */

  uintptr_t *regs;
};
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_TRICORE_INCLUDE_TC3XX_IRQ_H */
