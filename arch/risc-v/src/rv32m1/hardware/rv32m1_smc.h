/****************************************************************************
 * arch/risc-v/src/rv32m1/hardware/rv32m1_smc.h
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

#ifndef __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_SMC_H
#define __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_SMC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RV32M1_SMC_VERID_OFFSET       0x0000 /* Version ID */
#define RV32M1_SMC_PARAM_OFFSET       0x0004 /* Parameter */
#define RV32M1_SMC_PMPROT_OFFSET      0x0008 /* Power Mode Protection */
#define RV32M1_SMC_PMCTRL_OFFSET      0x0010 /* Power Mode Control */
#define RV32M1_SMC_PMSTAT_OFFSET      0x0018 /* Power Mode Status */
#define RV32M1_SMC_SRS_OFFSET         0x0020 /* System Reset Status */
#define RV32M1_SMC_RPC_OFFSET         0x0024 /* Reset Pin Control */
#define RV32M1_SMC_SSRS_OFFSET        0x0028 /* Sticky System Reset */
#define RV32M1_SMC_SRIE_OFFSET        0x002c /* System Reset Interrupt Enable */
#define RV32M1_SMC_SRIF_OFFSET        0x0030 /* System Reset Interrupt Flag */
#define RV32M1_SMC_MR_OFFSET          0x0040 /* Mode Register */
#define RV32M1_SMC_FM_OFFSET          0x0050 /* Force Mode Register */
#define RV32M1_SMC_SRAMLPR_OFFSET     0x0060 /* SRAM Low Power */
#define RV32M1_SMC_SRAMDSR_OFFSET     0x0064 /* SRAM Deep Sleep */

/* Register Address *********************************************************/

#if defined(CONFIG_ARCH_CHIP_RV32M1_RI5CY)
#  define RV32M1_SMC_BASE     RV32M1_SMC0_BASE
#else
#  define RV32M1_SMC_BASE     RV32M1_SMC1_BASE
#endif

#define RV32M1_SMC_VERID     (RV32M1_SMC_BASE + RV32M1_SMC_VERID_OFFSET)
#define RV32M1_SMC_PARAM     (RV32M1_SMC_BASE + RV32M1_SMC_PARAM_OFFSET)
#define RV32M1_SMC_PMPROT    (RV32M1_SMC_BASE + RV32M1_SMC_PMPROT_OFFSET)
#define RV32M1_SMC_PMCTRL    (RV32M1_SMC_BASE + RV32M1_SMC_PMCTRL_OFFSET)
#define RV32M1_SMC_PMSTAT    (RV32M1_SMC_BASE + RV32M1_SMC_PMSTAT_OFFSET)
#define RV32M1_SMC_SRS       (RV32M1_SMC_BASE + RV32M1_SMC_SRS_OFFSET)
#define RV32M1_SMC_RPC       (RV32M1_SMC_BASE + RV32M1_SMC_RPC_OFFSET)
#define RV32M1_SMC_SSRS      (RV32M1_SMC_BASE + RV32M1_SMC_SSRS_OFFSET)
#define RV32M1_SMC_SRIE      (RV32M1_SMC_BASE + RV32M1_SMC_SRIE_OFFSET)
#define RV32M1_SMC_SRIF      (RV32M1_SMC_BASE + RV32M1_SMC_SRIF_OFFSET)
#define RV32M1_SMC_MR        (RV32M1_SMC_BASE + RV32M1_SMC_MR_OFFSET)
#define RV32M1_SMC_FM        (RV32M1_SMC_BASE + RV32M1_SMC_FM_OFFSET)
#define RV32M1_SMC_SRAMLPR   (RV32M1_SMC_BASE + RV32M1_SMC_SRAMLPR_OFFSET)
#define RV32M1_SMC_SRAMDSR   (RV32M1_SMC_BASE + RV32M1_SMC_SRAMDSR_OFFSET)

/* Register Bitfield Definitions ********************************************/

#define SMC_PARAM_PWRD_INDPT          (1 << 1) /* Bit1: Power Domains Independent */

#define SMC_PMPROT_AHSRUN             (1 << 7) /* Bit7: Allow High Speed Run mode */
#define SMC_PMPROT_AVLP               (1 << 5) /* Bit5: Allow Very-Low-Power Mode */
#define SMC_PMPROT_ALLS               (1 << 3) /* Bit3: Allow Low-Leakage Stop Mode */
#define SMC_PMPROT_AVLLS_SHIFT        (0)      /* Bit[1:0]: Allow Very-Low-Leakage Stop Mode */
#define SMC_PMPROT_AVLLS_MASK         (3 << SMC_PMPROT_AVLLS_SHIFT)
#define SMC_PMPROT_AVLLS_NOT_ALLOWED  (0 << SMC_PMPROT_AVLLS_SHIFT) /* VLLS mode is dinied */
#define SMC_PMPROT_AVLLS_0_1_ALLOWED  (1 << SMC_PMPROT_AVLLS_SHIFT) /* VLLS0/1 mode is Allowded */
#define SMC_PMPROT_AVLLS_2_3_ALLOWED  (2 << SMC_PMPROT_AVLLS_SHIFT) /* VLLS2/3 mode is Allowed */
#define SMC_PMPROT_AVLLS_ALL_ALLOWED  (3 << SMC_PMPROT_AVLLS_SHIFT) /* VLSS0/1/2/3 mode is Allowed */

#  define SMC_PMPROT_PM_ALL_ALLOWED  \
  (SMC_PMPROT_AHSRUN | SMC_PMPROT_AVLP | \
   SMC_PMPROT_ALLS   | SMC_PMPROT_AVLLS_ALL_ALLOWED)

#define SMC_PMCTRL_PSTOPO_SHIFT       (16) /* Bit[17:16]: Partial Stop Option */
#define SMC_PMCTRL_PSTOPO_MASK        (3 << SMC_PMCTRL_PSTOPO_SHIFT)
#define SMC_PMCTRL_PSTOPO_STOP        (0 << SMC_PMCTRL_PSTOPO_SHIFT)
#define SMC_PMCTRL_PSTOPO_PSTOP1      (1 << SMC_PMCTRL_PSTOPO_SHIFT)
#define SMC_PMCTRL_PSTOPO_PSTOP2      (2 << SMC_PMCTRL_PSTOPO_SHIFT)
#define SMC_PMCTRL_PSTOPO_PSTOP3      (3 << SMC_PMCTRL_PSTOPO_SHIFT)

#define SMC_PMCTRL_RUNM_SHIFT         (8) /* Bit[9:8]: Run Mode Control */
#define SMC_PMCTRL_RUNM_MASK          (3 << SMC_PMCTRL_RUNM_SHIFT)
#define SMC_PMCTRL_RUNM_RUN           (0 << SMC_PMCTRL_RUNM_SHIFT)
#define SMC_PMCTRL_RUNM_VLPR          (2 << SMC_PMCTRL_RUNM_SHIFT)
#define SMC_PMCTRL_RUNM_HSRUN         (3 << SMC_PMCTRL_RUNM_SHIFT)

#define SMC_PMCTRL_STOPM_SHIFT        (0) /* Bit[2:0]: Stop Mode Control */
#define SMC_PMCTRL_STOPM_MASK         (7 << SMC_PMCTRL_STOPM_SHIFT)
#define SMC_PMCTRL_STOPM_STOP         (0 << SMC_PMCTRL_STOPM_SHIFT)
#define SMC_PMCTRL_STOPM_VLPS         (2 << SMC_PMCTRL_STOPM_SHIFT)
#define SMC_PMCTRL_STOPM_LLS          (3 << SMC_PMCTRL_STOPM_SHIFT)
#define SMC_PMCTRL_STOPM_VLLS_2_3     (4 << SMC_PMCTRL_STOPM_SHIFT)
#define SMC_PMCTRL_STOPM_VLLS_0_1     (6 << SMC_PMCTRL_STOPM_SHIFT)

#define SMC_PMSTAT_PMSTAT_SHIFT       (0) /* Bit[7:0]: Power Mode Status */
#define SMC_PMSTAT_PMSTAT_MASK        (0xff << SMC_PMSTAT_PMSTAT_SHIFT)
#define SMC_PMSTAT_RUN                (0x01 << SMC_PMSTAT_PMSTAT_SHIFT)
#define SMC_PMSTAT_STOP               (0x02 << SMC_PMSTAT_PMSTAT_SHIFT)
#define SMC_PMSTAT_VLPR               (0x04 << SMC_PMSTAT_PMSTAT_SHIFT)
#define SMC_PMSTAT_HSRUN              (0x80 << SMC_PMSTAT_PMSTAT_SHIFT)

#endif /* __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_SMC_H */
