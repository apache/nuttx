/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32_fdcan.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_FDCAN_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_FDCAN_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
#define FDCAN_RAM_MAX_WORDS 2560

#define FDCAN_MAX_NFILTERS_STD   128
#define FDCAN_MAX_NFILTERS_EXT   64

/* Register Offsets *****************************************************************/

/* FDCAN core registers */

#define STM32_FDCAN_CREL_OFFSET    0x0000  /* Core release register */
#define STM32_FDCAN_ENDN_OFFSET    0x0004  /* Endian register */
#define STM32_FDCAN_DBTP_OFFSET    0x000c  /* Data bit timing and prescaler register*/
#define STM32_FDCAN_TEST_OFFSET    0x0010  /* Test register */
#define STM32_FDCAN_RWD_OFFSET     0x0014  /* RAM watchdog register */
#define STM32_FDCAN_CCCR_OFFSET    0x0018  /* CC control register */
#define STM32_FDCAN_NBTP_OFFSET    0x001c  /* Nominal bit timing and prescaler register */
#define STM32_FDCAN_TSCC_OFFSET    0x0020  /* Timestamp counter configuration register */
#define STM32_FDCAN_TSCV_OFFSET    0x0024  /* Timestamp counter value register */
#define STM32_FDCAN_TOCC_OFFSET    0x0028  /* Timeout counter configuration register */
#define STM32_FDCAN_TOCV_OFFSET    0x002c  /* Timeout counter valie register */
#define STM32_FDCAN_ECR_OFFSET     0x0040  /* Error counter register */
#define STM32_FDCAN_PSR_OFFSET     0x0044  /* Protocol status register  */
#define STM32_FDCAN_TDCR_OFFSET    0x0048  /* Transmitter delay compensation register */
#define STM32_FDCAN_IR_OFFSET      0x0050  /* Interrupt register */
#define STM32_FDCAN_IE_OFFSET      0x0054  /* Interrupt enable register */
#define STM32_FDCAN_ILS_OFFSET     0x0058  /* Interrupt line select register */
#define STM32_FDCAN_ILE_OFFSET     0x005c  /* Interrupt line enable register */
#define STM32_FDCAN_GFC_OFFSET     0x0080  /* Global filter configuration register*/
#define STM32_FDCAN_SIDFC_OFFSET   0x0084  /* Standard ID filter configuration register */
#define STM32_FDCAN_XIDFC_OFFSET   0x0088  /* Extended ID filter configuration register */
#define STM32_FDCAN_XIDAM_OFFSET   0x0090  /* Extended ID and mask register register */
#define STM32_FDCAN_HPMS_OFFSET    0x0094  /* High priority message status register */
#define STM32_FDCAN_NDAT1_OFFSET   0x0098  /* New data 1 register */
#define STM32_FDCAN_NDAT2_OFFSET   0x009c  /* New data 2 register */

/* Rx buffer registers */

#define STM32_FDCAN_RXFC_OFFSET(f) (0x00a0+((f)<<4)) 
#define STM32_FDCAN_RXF0C_OFFSET   0x00a0  /* Rx FIFO 0 configuration register */
#define STM32_FDCAN_RXF1C_OFFSET   0x00b0  /* Rx FIFO 1 configuration register */

#define STM32_FDCAN_RXFS_OFFSET(f) (0x00a4+((f)<<4)) 
#define STM32_FDCAN_RXF0S_OFFSET   0x00a4  /* Rx FIFO 0 status register */
#define STM32_FDCAN_RXF1S_OFFSET   0x00b4  /* Rx FIFO 1 status register */

#define STM32_FDCAN_RXFA_OFFSET(f) (0x00a8+((f)<<4))
#define STM32_FDCAN_RXF0A_OFFSET   0x00a8  /* Rx FIFO 0 acknowledge register */
#define STM32_FDCAN_RXF1A_OFFSET   0x00b8  /* Rx FIFO 1 acknowledge register */

#define STM32_FDCAN_RXBC_OFFSET    0x00ac  /* Rx buffer configuration register */
#define STM32_FDCAN_RXESC_OFFSET   0x00bc  /* Rx buffer element size configuration register */

/* Tx buffer registers */

#define STM32_FDCAN_TXBC_OFFSET    0x00c0  /* Tx buffer configuration register */
#define STM32_FDCAN_TXFQS_OFFSET   0x00c4  /* Tx FIFO/queue status register */
#define STM32_FDCAN_TXESC_OFFSET   0x00c8  /* Tx buffer element size configuration register */
#define STM32_FDCAN_TXBRP_OFFSET   0x00cc  /* Tx buffer request pending register register */
#define STM32_FDCAN_TXBAR_OFFSET   0x00d0  /* Tx buffer add request register */
#define STM32_FDCAN_TXBCR_OFFSET   0x00d4  /* Tx buffer cancellation request register */
#define STM32_FDCAN_TXBTO_OFFSET   0x00d8  /* Tx buffer transmission occurred register */
#define STM32_FDCAN_TXBCF_OFFSET   0x00dc  /* Tx buffer cancellation finished register */
#define STM32_FDCAN_TXBTIE_OFFSET  0x00e0  /* Tx buffer transmission interrupt enable register */
#define STM32_FDCAN_TXBCIE_OFFSET  0x00e4  /* Tx buffer cancellation finished interrupt enable register */

/* Tx event FIFO registers */

#define STM32_FDCAN_TXEFC_OFFSET   0x00f0  /* Tx event FIFO configuration register */
#define STM32_FDCAN_TXEFS_OFFSET   0x00f4  /* Tx event FIFO status register */
#define STM32_FDCAN_TXEFA_OFFSET   0x00f8  /* Tx event FIFO acknowledge register */

/* Timer trigger registers (FDCAN1 only) */

#define STM32_FDCAN_TTTMC_OFFSET   0x0100  /* TT trigger memory configuration trregister */
#define STM32_FDCAN_TTRMC_OFFSET   0x0104  /* TT reference message configuration register */
#define STM32_FDCAN_TTOCF_OFFSET   0x0108  /* TT operation configration register */
#define STM32_FDCAN_TTMLM_OFFSET   0x010c  /* TT matrix limits register */
#define STM32_FDCAN_TURCF_OFFSET   0x0110  /* TUR configuration register */
#define STM32_FDCAN_TTOCN_OFFSET   0x0114  /* TT operation control register */
#define STM32_FDCAN_TTGTP_OFFSET   0x0118  /* TT global time preset register */
#define STM32_FDCAN_TTTMK_OFFSET   0x011c  /* TT time mark register */
#define STM32_FDCAN_TTIR_OFFSET    0x0120  /* TT interrupt register */
#define STM32_FDCAN_TTIE_OFFSET    0x0124  /* TT interrupt enable register */
#define STM32_FDCAN_TTILS_OFFSET   0x0128  /* TT interrupt line select register */
#define STM32_FDCAN_TTOST_OFFSET   0x012c  /* TT operation status register */
#define STM32_FDCAN_TURNA_OFFSET   0x0130  /* TUR numerator actual register */
#define STM32_FDCAN_TTLGT_OFFSET   0x0134  /* TT local and global time register */
#define STM32_FDCAN_TTCTC_OFFSET   0x0138  /* TT cycle time and count register */
#define STM32_FDCAN_TTCPT_OFFSET   0x013c  /* TT capture time register */
#define STM32_FDCAN_TTCSM_OFFSET   0x0140  /* TT cycle sync mark register */
#define STM32_FDCAN_TTTS_OFFSET    0x0300  /* TT trigger select register */

/* Clock Calibration Unit (CCU) registers */

#define STM32_FDCAN_CCU_CREL_OFFSET   0x0000  /* Core release register */
#define STM32_FDCAN_CCU_CCFG_OFFSET   0x0004  /* Calibration configuration register */
#define STM32_FDCAN_CCU_CSTAT_OFFSET  0x0008  /* Calibration status register */
#define STM32_FDCAN_CCU_CWD_OFFSET    0x000c  /* Calibration watchdog register */
#define STM32_FDCAN_CCU_IR_OFFSET     0x0010  /* CCU interrupt register */
#define STM32_FDCAN_CCU_IE_OFFSET     0x0014  /* CCU interrupt enable register*/

/* Register Addresses ***************************************************************/

#ifdef CONFIG_STM32H7_FDCAN1

#define STM32_FDCAN1_CREL   (STM32_FDCAN1_BASE + STM32_FDCAN_CREL_OFFSET)
#define STM32_FDCAN1_ENDN   (STM32_FDCAN1_BASE + STM32_FDCAN_ENDN_OFFSET)
#define STM32_FDCAN1_DBTP   (STM32_FDCAN1_BASE + STM32_FDCAN_DBTP_OFFSET)
#define STM32_FDCAN1_TEST   (STM32_FDCAN1_BASE + STM32_FDCAN_TEST_OFFSET)
#define STM32_FDCAN1_RWD    (STM32_FDCAN1_BASE + STM32_FDCAN_RWD_OFFSET)
#define STM32_FDCAN1_CCCR   (STM32_FDCAN1_BASE + STM32_FDCAN_CCCR_OFFSET)
#define STM32_FDCAN1_NBTP   (STM32_FDCAN1_BASE + STM32_FDCAN_NBTP_OFFSET)
#define STM32_FDCAN1_TSCC   (STM32_FDCAN1_BASE + STM32_FDCAN_TSCC_OFFSET)
#define STM32_FDCAN1_TSCV   (STM32_FDCAN1_BASE + STM32_FDCAN_TSCV_OFFSET)
#define STM32_FDCAN1_TOCC   (STM32_FDCAN1_BASE + STM32_FDCAN_TOCC_OFFSET)
#define STM32_FDCAN1_TOCV   (STM32_FDCAN1_BASE + STM32_FDCAN_TOCV_OFFSET)
#define STM32_FDCAN1_ECR    (STM32_FDCAN1_BASE + STM32_FDCAN_ECR_OFFSET)
#define STM32_FDCAN1_PSR    (STM32_FDCAN1_BASE + STM32_FDCAN_PSR_OFFSET)
#define STM32_FDCAN1_TDCR   (STM32_FDCAN1_BASE + STM32_FDCAN_TDCR_OFFSET)
#define STM32_FDCAN1_IR     (STM32_FDCAN1_BASE + STM32_FDCAN_IR_OFFSET)
#define STM32_FDCAN1_IE     (STM32_FDCAN1_BASE + STM32_FDCAN_IE_OFFSET)
#define STM32_FDCAN1_ILS    (STM32_FDCAN1_BASE + STM32_FDCAN_ILS_OFFSET)
#define STM32_FDCAN1_ILE    (STM32_FDCAN1_BASE + STM32_FDCAN_ILE_OFFSET)
#define STM32_FDCAN1_GFC    (STM32_FDCAN1_BASE + STM32_FDCAN_GFC_OFFSET)
#define STM32_FDCAN1_SIDFC  (STM32_FDCAN1_BASE + STM32_FDCAN_SIDFC_OFFSET)
#define STM32_FDCAN1_XIDFC  (STM32_FDCAN1_BASE + STM32_FDCAN_XIDFC_OFFSET)
#define STM32_FDCAN1_XIDAM  (STM32_FDCAN1_BASE + STM32_FDCAN_XIDAM_OFFSET)
#define STM32_FDCAN1_HPMS   (STM32_FDCAN1_BASE + STM32_FDCAN_HPMS_OFFSET)
#define STM32_FDCAN1_NDAT1  (STM32_FDCAN1_BASE + STM32_FDCAN_NDAT1_OFFSET)
#define STM32_FDCAN1_NDAT2  (STM32_FDCAN1_BASE + STM32_FDCAN_NDAT2_OFFSET)
#define STM32_FDCAN1_RXF0C  (STM32_FDCAN1_BASE + STM32_FDCAN_RXF0C_OFFSET)
#define STM32_FDCAN1_RXF0S  (STM32_FDCAN1_BASE + STM32_FDCAN_RXF0S_OFFSET)
#define STM32_FDCAN1_RXF0A  (STM32_FDCAN1_BASE + STM32_FDCAN_RXF0A_OFFSET)
#define STM32_FDCAN1_RXBC   (STM32_FDCAN1_BASE + STM32_FDCAN_RXBC_OFFSET)
#define STM32_FDCAN1_RXF1C  (STM32_FDCAN1_BASE + STM32_FDCAN_RXF1C_OFFSET)
#define STM32_FDCAN1_RXF1S  (STM32_FDCAN1_BASE + STM32_FDCAN_RXF1S_OFFSET)
#define STM32_FDCAN1_RXF1A  (STM32_FDCAN1_BASE + STM32_FDCAN_RXF1A_OFFSET)
#define STM32_FDCAN1_RXESC  (STM32_FDCAN1_BASE + STM32_FDCAN_RXESC_OFFSET)
#define STM32_FDCAN1_TXBC   (STM32_FDCAN1_BASE + STM32_FDCAN_TXBC_OFFSET)
#define STM32_FDCAN1_TXFQS  (STM32_FDCAN1_BASE + STM32_FDCAN_TXFQS_OFFSET)
#define STM32_FDCAN1_TXESC  (STM32_FDCAN1_BASE + STM32_FDCAN_TXESC_OFFSET)
#define STM32_FDCAN1_TXBRP  (STM32_FDCAN1_BASE + STM32_FDCAN_TXBRP_OFFSET)
#define STM32_FDCAN1_TXBAR  (STM32_FDCAN1_BASE + STM32_FDCAN_TXBAR_OFFSET)
#define STM32_FDCAN1_TXBCR  (STM32_FDCAN1_BASE + STM32_FDCAN_TXBCR_OFFSET)
#define STM32_FDCAN1_TXBTO  (STM32_FDCAN1_BASE + STM32_FDCAN_TXBTO_OFFSET)
#define STM32_FDCAN1_TXBCF  (STM32_FDCAN1_BASE + STM32_FDCAN_TXBCF_OFFSET)
#define STM32_FDCAN1_TXBTIE (STM32_FDCAN1_BASE + STM32_FDCAN_TXBTIE_OFFSET)
#define STM32_FDCAN1_TXBCIE (STM32_FDCAN1_BASE + STM32_FDCAN_TXBCIE_OFFSET)
#define STM32_FDCAN1_TXEFC  (STM32_FDCAN1_BASE + STM32_FDCAN_TXEFC_OFFSET)
#define STM32_FDCAN1_TXEFS  (STM32_FDCAN1_BASE + STM32_FDCAN_TXEFS_OFFSET)
#define STM32_FDCAN1_TXEFA  (STM32_FDCAN1_BASE + STM32_FDCAN_TXEFA_OFFSET)
#define STM32_FDCAN1_TTTMC  (STM32_FDCAN1_BASE + STM32_FDCAN_TTTMC_OFFSET)
#define STM32_FDCAN1_TTRMC  (STM32_FDCAN1_BASE + STM32_FDCAN_TTRMC_OFFSET)
#define STM32_FDCAN1_TTOCF  (STM32_FDCAN1_BASE + STM32_FDCAN_TTOCF_OFFSET)
#define STM32_FDCAN1_TTMLM  (STM32_FDCAN1_BASE + STM32_FDCAN_TTMLM_OFFSET)
#define STM32_FDCAN1_TURCF  (STM32_FDCAN1_BASE + STM32_FDCAN_TURCF_OFFSET)
#define STM32_FDCAN1_TTOCN  (STM32_FDCAN1_BASE + STM32_FDCAN_TTOCN_OFFSET)
#define STM32_FDCAN1_TTGTP  (STM32_FDCAN1_BASE + STM32_FDCAN_TTGTP_OFFSET)
#define STM32_FDCAN1_TTTMK  (STM32_FDCAN1_BASE + STM32_FDCAN_TTTMK_OFFSET)
#define STM32_FDCAN1_TTIR   (STM32_FDCAN1_BASE + STM32_FDCAN_TTIR_OFFSET)
#define STM32_FDCAN1_TTIE   (STM32_FDCAN1_BASE + STM32_FDCAN_TTIE_OFFSET)
#define STM32_FDCAN1_TTILS  (STM32_FDCAN1_BASE + STM32_FDCAN_TTILS_OFFSET)
#define STM32_FDCAN1_TTOST  (STM32_FDCAN1_BASE + STM32_FDCAN_TTOST_OFFSET)
#define STM32_FDCAN1_TURNA  (STM32_FDCAN1_BASE + STM32_FDCAN_TURNA_OFFSET)
#define STM32_FDCAN1_TTLGT  (STM32_FDCAN1_BASE + STM32_FDCAN_TTLGT_OFFSET)
#define STM32_FDCAN1_TTCTC  (STM32_FDCAN1_BASE + STM32_FDCAN_TTCTC_OFFSET)
#define STM32_FDCAN1_TTCPT  (STM32_FDCAN1_BASE + STM32_FDCAN_TTCPT_OFFSET)
#define STM32_FDCAN1_TTCSM  (STM32_FDCAN1_BASE + STM32_FDCAN_TTCSM_OFFSET)
#define STM32_FDCAN1_TTTS   (STM32_FDCAN1_BASE + STM32_FDCAN_TTTS_OFFSET)

#endif

#ifdef CONFIG_STM32H7_FDCAN2

#define STM32_FDCAN2_CREL   (STM32_FDCAN2_BASE + STM32_FDCAN_CREL_OFFSET)
#define STM32_FDCAN2_ENDN   (STM32_FDCAN2_BASE + STM32_FDCAN_ENDN_OFFSET)
#define STM32_FDCAN2_DBTP   (STM32_FDCAN2_BASE + STM32_FDCAN_DBTP_OFFSET)
#define STM32_FDCAN2_TEST   (STM32_FDCAN2_BASE + STM32_FDCAN_TEST_OFFSET)
#define STM32_FDCAN2_RWD    (STM32_FDCAN2_BASE + STM32_FDCAN_RWD_OFFSET)
#define STM32_FDCAN2_CCCR   (STM32_FDCAN2_BASE + STM32_FDCAN_CCCR_OFFSET)
#define STM32_FDCAN2_NBTP   (STM32_FDCAN2_BASE + STM32_FDCAN_NBTP_OFFSET)
#define STM32_FDCAN2_TSCC   (STM32_FDCAN2_BASE + STM32_FDCAN_TSCC_OFFSET)
#define STM32_FDCAN2_TSCV   (STM32_FDCAN2_BASE + STM32_FDCAN_TSCV_OFFSET)
#define STM32_FDCAN2_TOCC   (STM32_FDCAN2_BASE + STM32_FDCAN_TOCC_OFFSET)
#define STM32_FDCAN2_TOCV   (STM32_FDCAN2_BASE + STM32_FDCAN_TOCV_OFFSET)
#define STM32_FDCAN2_ECR    (STM32_FDCAN2_BASE + STM32_FDCAN_ECR_OFFSET)
#define STM32_FDCAN2_PSR    (STM32_FDCAN2_BASE + STM32_FDCAN_PSR_OFFSET)
#define STM32_FDCAN2_TDCR   (STM32_FDCAN2_BASE + STM32_FDCAN_TDCR_OFFSET)
#define STM32_FDCAN2_IR     (STM32_FDCAN2_BASE + STM32_FDCAN_IR_OFFSET)
#define STM32_FDCAN2_IE     (STM32_FDCAN2_BASE + STM32_FDCAN_IE_OFFSET)
#define STM32_FDCAN2_ILS    (STM32_FDCAN2_BASE + STM32_FDCAN_ILS_OFFSET)
#define STM32_FDCAN2_ILE    (STM32_FDCAN2_BASE + STM32_FDCAN_ILE_OFFSET)
#define STM32_FDCAN2_GFC    (STM32_FDCAN2_BASE + STM32_FDCAN_GFC_OFFSET)
#define STM32_FDCAN2_SIDFC  (STM32_FDCAN2_BASE + STM32_FDCAN_SIDFC_OFFSET)
#define STM32_FDCAN2_XIDFC  (STM32_FDCAN2_BASE + STM32_FDCAN_XIDFC_OFFSET)
#define STM32_FDCAN2_XIDAM  (STM32_FDCAN2_BASE + STM32_FDCAN_XIDAM_OFFSET)
#define STM32_FDCAN2_HPMS   (STM32_FDCAN2_BASE + STM32_FDCAN_HPMS_OFFSET)
#define STM32_FDCAN2_NDAT1  (STM32_FDCAN2_BASE + STM32_FDCAN_NDAT1_OFFSET)
#define STM32_FDCAN2_NDAT2  (STM32_FDCAN2_BASE + STM32_FDCAN_NDAT2_OFFSET)
#define STM32_FDCAN2_RXF0C  (STM32_FDCAN2_BASE + STM32_FDCAN_RXF0C_OFFSET)
#define STM32_FDCAN2_RXF0S  (STM32_FDCAN2_BASE + STM32_FDCAN_RXF0S_OFFSET)
#define STM32_FDCAN2_RXF0A  (STM32_FDCAN2_BASE + STM32_FDCAN_RXF0A_OFFSET)
#define STM32_FDCAN2_RXBC   (STM32_FDCAN2_BASE + STM32_FDCAN_RXBC_OFFSET)
#define STM32_FDCAN2_RXF1C  (STM32_FDCAN2_BASE + STM32_FDCAN_RXF1C_OFFSET)
#define STM32_FDCAN2_RXF1S  (STM32_FDCAN2_BASE + STM32_FDCAN_RXF1S_OFFSET)
#define STM32_FDCAN2_RXF1A  (STM32_FDCAN2_BASE + STM32_FDCAN_RXF1A_OFFSET)
#define STM32_FDCAN2_RXESC  (STM32_FDCAN2_BASE + STM32_FDCAN_RXESC_OFFSET)
#define STM32_FDCAN2_TXBC   (STM32_FDCAN2_BASE + STM32_FDCAN_TXBC_OFFSET)
#define STM32_FDCAN2_TXFQS  (STM32_FDCAN2_BASE + STM32_FDCAN_TXFQS_OFFSET)
#define STM32_FDCAN2_TXESC  (STM32_FDCAN2_BASE + STM32_FDCAN_TXESC_OFFSET)
#define STM32_FDCAN2_TXBRP  (STM32_FDCAN2_BASE + STM32_FDCAN_TXBRP_OFFSET)
#define STM32_FDCAN2_TXBAR  (STM32_FDCAN2_BASE + STM32_FDCAN_TXBAR_OFFSET)
#define STM32_FDCAN2_TXBCR  (STM32_FDCAN2_BASE + STM32_FDCAN_TXBCR_OFFSET)
#define STM32_FDCAN2_TXBTO  (STM32_FDCAN2_BASE + STM32_FDCAN_TXBTO_OFFSET)
#define STM32_FDCAN2_TXBCF  (STM32_FDCAN2_BASE + STM32_FDCAN_TXBCF_OFFSET)
#define STM32_FDCAN2_TXBTIE (STM32_FDCAN2_BASE + STM32_FDCAN_TXBTIE_OFFSET)
#define STM32_FDCAN2_TXBCIE (STM32_FDCAN2_BASE + STM32_FDCAN_TXBCIE_OFFSET)
#define STM32_FDCAN2_TXEFC  (STM32_FDCAN2_BASE + STM32_FDCAN_TXEFC_OFFSET)
#define STM32_FDCAN2_TXEFS  (STM32_FDCAN2_BASE + STM32_FDCAN_TXEFS_OFFSET)
#define STM32_FDCAN2_TXEFA  (STM32_FDCAN2_BASE + STM32_FDCAN_TXEFA_OFFSET)

#endif

#define STM32_FDCAN_CCU_CREL   (STM32_CANCCU_BASE + STM32_FDCAN_CCU_CREL_OFFSET)
#define STM32_FDCAN_CCU_CCFG   (STM32_CANCCU_BASE + STM32_FDCAN_CCU_CCFG_OFFSET)
#define STM32_FDCAN_CCU_CSTAT  (STM32_CANCCU_BASE + STM32_FDCAN_CCU_CSTAT_OFFSET)
#define STM32_FDCAN_CCU_CWD    (STM32_CANCCU_BASE + STM32_FDCAN_CCU_CWD_OFFSET)
#define STM32_FDCAN_CCU_IR     (STM32_CANCCU_BASE + STM32_FDCAN_CCU_IR_OFFSET)
#define STM32_FDCAN_CCU_IE     (STM32_CANCCU_BASE + STM32_FDCAN_CCU_IE_OFFSET)

/* Register Bitfield Definitions ****************************************************/

/* FDCAN data bit timing and prescaler register */

#define FDCAN_DBTP_DSJW_SHIFT       (0)       /* Bits 3-0: Synchronization jump width */
#define FDCAN_DBTP_DSJW_MASK        (15 << FDCAN_DBTP_DSJW_SHIFT)
#  define FDCAN_DBTP_DSJW(value)    ((value) << FDCAN_DBTP_DSJW_SHIFT) /* value = 0..15 */
#  define FDCAN_DBTP_DSJW_MAX       (15)
#define FDCAN_DBTP_DTSEG2_SHIFT     (4)       /* Bits 7-4: Data time segment after sample point*/
#define FDCAN_DBTP_DTSEG2_MASK      (15 << FDCAN_DBTP_DTSEG2_SHIFT)
#  define FDCAN_DBTP_DTSEG2(value)  ((value) << FDCAN_DBTP_DTSEG2_SHIFT) /* value = 0..15 */
#  define FDCAN_DBTP_DTSEG2_MAX     (15)
#define FDCAN_DBTP_DTSEG1_SHIFT     (8)       /* Bits 12-8: Data time segment before sample point*/
#define FDCAN_DBTP_DTSEG1_MASK      (31 << FDCAN_DBTP_DTSEG1_SHIFT)
#  define FDCAN_DBTP_DTSEG1(value)  ((value) << FDCAN_DBTP_DTSEG1_SHIFT) /* value = 0..31 */
#  define FDCAN_DBTP_DTSEG1_MAX     (31)
#define FDCAN_DBTP_DBRP_SHIFT       (16)      /* Bits 20-16: Data bitrate prescaler */
#define FDCAN_DBTP_DBRP_MASK        (31 << FDCAN_DBTP_DBRP_SHIFT)
#  define FDCAN_DBTP_DBRP(value)    ((value) << FDCAN_DBTP_DBRP_SHIFT) /* value = 0..31 */
#  define FDCAN_DBTP_DBRP_MAX       (31)
#define FDCAN_DBTP_TDC_EN           (1 << 23) /* Bit 23: Transceiver delay compensation enable */ 

/* FDCAN test register */

#define FDCAN_TEST_LBCK             (1 << 4) /* Bit 4: Loop back mode enable */
#define FDCAN_TEST_TX_SHIFT         (5)      /* Bits 6-5: Control of transmit pin*/
#define FDCAN_TEST_TX_MASK          (3 << FDCAN_TEST_TX_SHIFT)
#  define FDCAN_TEST_TX_RST         (0)                        /* Controlled by CAN core */
#  define FDCAN_TEST_TX_SMPL        (1 << FDCAN_TEST_TX_SHIFT) /* Sample point monitoring */
#  define FDCAN_TEST_TX_DOM         (2 << FDCAN_TEST_TX_SHIFT) /* Set dominant */
#  define FDCAN_TEST_TX_REC         (3 << FDCAN_TEST_TX_SHIFT) /* Set recessive */
#define FDCAN_TEST_RX               (1 << 7) /* Bit 7: Receive pin value */

/* FDCAN RAM watchdog register */

#define FDCAN_RWD_WDC_SHIFT         (0)      /* Bits 7-0: RAM watchdog counter start value */
#define FDCAN_RWD_WDC_MASK          (255 << FDCAN_RWD_WDC_SHIFT)
#  define FDCAN_RWD_WDC_DIS         (0 << FDCAN_RWD_WDC_SHIFT)       /* Counter disabled */
#  define FDCAN_RWD_WDC(value)      ((value) << FDCAN_RWD_WDC_SHIFT) /* Value = 1..255 */
#define FDCAN_RWD_WDV_SHIFT         (8)      /* Bits 15-8: RAM watchdog counter value */
#define FDCAN_RWD_WDV_MASK          (255 << FDCAN_RWD_WDV_SHIFT)

/* FDCAN CC control register */

#define FDCAN_CCCR_INIT             (1 << 0)   /* Bit 0: Initialization */
#define FDCAN_CCCR_CCE              (1 << 1)   /* Bit 1: Configuration change enable */
#define FDCAN_CCCR_ASM              (1 << 2)   /* Bit 2: ASM restricted operation mode */
#define FDCAN_CCCR_CSA              (1 << 3)   /* Bit 3: Clock stop acknowledge */
#define FDCAN_CCCR_CSR              (1 << 4)   /* Bit 4: Clock stop request */
#define FDCAN_CCCR_MON              (1 << 5)   /* Bit 5: Bus monitoring mode */
#define FDCAN_CCCR_DAR              (1 << 6)   /* Bit 6: Disable automatic retransmission */
#define FDCAN_CCCR_TEST             (1 << 7)   /* Bit 7: Test mode enable */
#define FDCAN_CCCR_FDOE             (1 << 8)   /* Bit 8: FD operation enable */
#define FDCAN_CCCR_BRSE             (1 << 9)   /* Bit 9: FDCAN Bitrate switching */
#define FDCAN_CCCR_PXHD             (1 << 12)  /* Bit 12: Protocol exception handling disable */
#define FDCAN_CCCR_EFBI             (1 << 13)  /* Bit 13: Edge filtering during bus integration */
#define FDCAN_CCCR_TXP              (1 << 14)  /* Bit 14: Tx pause */
#define FDCAN_CCCR_NISO             (1 << 15)  /* Bit 15: Non ISO operation */

/* FDCAN nominal bit timing and prescaler register */

#define FDCAN_NBTP_NTSEG2_SHIFT     (0)  /* Bits 6-0: Nominal time segment after sample point */
#define FDCAN_NBTP_NTSEG2_MASK      (127 << FDCAN_NBTP_NTSEG2_SHIFT)
#  define FDCAN_NBTP_NTSEG2(value)  ((value) << FDCAN_NBTP_NTSEG2_SHIFT) /* Value = 0..127 */
#  define FDCAN_NBTP_NTSEG2_MAX     (127)
#define FDCAN_NBTP_NTSEG1_SHIFT     (8)  /* Bits 15-8: Nominal time segment before sample point */
#define FDCAN_NBTP_NTSEG1_MASK      (0Xff << FDCAN_NBTP_NTSEG1_SHIFT)
#  define FDCAN_NBTP_NTSEG1(value)  ((value) << FDCAN_NBTP_NTSEG1_SHIFT) /* Value = 0..255 */
#  define FDCAN_NBTP_NTSEG1_MAX     (255)
#define FDCAN_NBTP_NBRP_SHIFT       (16) /* Bits 24-16: Bitrate prescaler */
#define FDCAN_NBTP_NBRP_MASK        (0X1ff << FDCAN_NBTP_NBRP_SHIFT)
#  define FDCAN_NBTP_NBRP(value)    ((value) << FDCAN_NBTP_NBRP_SHIFT) /* Value = 0..511 */
#  define FDCAN_NBTP_NBRP_MAX       (511)
#define FDCAN_NBTP_NSJW_SHIFT       (25) /* Bits 31-25: Nominal (re)synchronization jump width */
#define FDCAN_NBTP_NSJW_MASK        (127 << FDCAN_NBTP_NSJW_SHIFT)
#  define FDCAN_NBTP_NSJW(value)    ((value) << FDCAN_NBTP_NSJW_SHIFT) /* Value = 0..127 */
#  define FDCAN_NBTP_NSJW_MAX       (127)


/* FDCAN timestamp counter configuration register */

#define FDCAN_TSCC_TSS_SHIFT        (0)  /* Bits 1-0: Timestamp counter select */
#define FDCAN_TSCC_TSS_MASK         (3 << FDCAN_TSCC_TSS_SHIFT)
#  define FDCAN_TSCC_TSS_ZERO       (0 << FDCAN_TSCC_TSS_SHIFT) /* 00: Always 0 */
#  define FDCAN_TSCC_TSS_TCP        (1 << FDCAN_TSCC_TSS_SHIFT) /* 01: Incremented based on TCP */
#  define FDCAN_TSCC_TSS_TIM3       (2 << FDCAN_TSCC_TSS_SHIFT) /* 10: Value from TIM3 used */
#define FDCAN_TSCC_TCP_SHIFT        (16) /* Bits 19-16: Timestamp counter prescaler */
#define FDCAN_TSCC_TCP_MASK         (15 << FDCAN_TSCC_TCP_SHIFT)
#  define FDCAN_TSCC_TCP(value)     ((value) << FDCAN_TSCC_TCP_SHIFT) /* Value = 0..15 */

/* FDCAN timeout counter configuration register */

#define FDCAN_TOCC_ETOC             (1 << 0) /* Bit 0: Enable timeout counter */
#define FDCAN_TOCC_TOS_SHIFT        (1)      /* Bits 2-1: Timeout select */
#define FDCAN_TOCC_TOS_MASK         (3 << FDCAN_TOCC_TOS_SHIFT)
#  define FDCAN_TOCC_TOS_CONT       (0 << FDCAN_TOCC_TOS_SHIFT) /* 00: Continuous operation */
#  define FDCAN_TOCC_TOS_TXFIFO     (1 << FDCAN_TOCC_TOS_SHIFT) /* 01: Tx event FIFO */
#  define FDCAN_TOCC_TOS_RX_FIFO0   (2 << FDCAN_TOCC_TOS_SHIFT) /* 10: Rx FIFO 0 */
#  define FDCAN_TOCC_TOS_RX_FIFO1   (3 << FDCAN_TOCC_TOS_SHIFT) /* 11: Rx FIFO 1 */
#define FDCAN_TOCC_TOP_SHIFT        (16)     /* Bits 31-16: Timeout period counter start value */
#define FDCAN_TOCC_TOP_MASK         (0xffff << FDCAN_TOCC_TOP_SHIFT)
#  define FDCAN_TOCC_TOP(value)     ((value) << FDCAN_TOCC_TOP_SHIFT) /* Value = 0..65,535 */

/* FDCAN error counter register */

#define FDCAN_ECR_TEC_SHIFT         (0)       /* Bits 7-0: Transmit error counter */
#define FDCAN_CR_TEC_MASK           (255 << FDCAN_ECR_TEC_SHIFT)
#define FDCAN_ECR_REC_SHIFT         (8)       /* Bits 14-8: Receive error counter */
#define FDCAN_ECR_REC_MASK          (127 << FDCAN_ECR_REC_SHIFT)
#define FDCAN_ECR_RP                (1 << 15) /* Bit 15: Receive error passive */
#define FDCAN_ECR_CEL_SHIFT         (16)      /* Bits 23-16: CAN error logging */
#define FDCAN_ECR_CEL_MASK          (255 << FDCAN_ECR_CEL_SHIFT)

/* FDCAN protocol status register */

/* Error codes */

#define FDCAN_PSR_EC_NO_ERROR       (0) /* No error occurred since LEC has been reset */
#define FDCAN_PSR_EC_STUFF_ERROR    (1) /* More than 5 equal bits in a sequence */
#define FDCAN_PSR_EC_FORM_ERROR     (2) /* Part of a received frame has wrong format */
#define FDCAN_PSR_EC_ACK_ERROR      (3) /* Message not acknowledged by another node */
#define FDCAN_PSR_EC_BIT1_ERROR     (4) /* Send with recessive level, but bus value was dominant */
#define FDCAN_PSR_EC_BIT0_ERROR     (5) /* Send with dominant level, but bus value was recessive */
#define FDCAN_PSR_EC_CRC_ERROR      (6) /* CRC received message incorrect */
#define FDCAN_PSR_EC_NO_CHANGE      (7) /* No CAN bus event was detected since last read */

#define FDCAN_PSR_LEC_SHIFT         (0) /* Bits 2-0: Last error code */
#define FDCAN_PSR_LEC_MASK          (7 << FDCAN_PSR_LEC_SHIFT)
#  define FDCAN_PSR_LEC(n)         ((uint32_t)(n) << FDCAN_PSR_LEC_SHIFT) /* See error codes above */
#define FDCAN_PSR_ACT_SHIFT         (3) /* Bits 4-3: Activity */
#define FDCAN_PSR_ACT_MASK          (3 << FDCAN_PSR_ACT_SHIFT)
#  define FDCAN_PSR_ACT_SYNC        (0 << FDCAN_PSR_ACT_SHIFT) /* 00: Synchronizing */
#  define FDCAN_PSR_ACT_IDLE        (1 << FDCAN_PSR_ACT_SHIFT) /* 01: Idle */
#  define FDCAN_PSR_ACT_RECV        (2 << FDCAN_PSR_ACT_SHIFT) /* 10: Receiver */
#  define FDCAN_PSR_ACT_TRANS       (3 << FDCAN_PSR_ACT_SHIFT) /* 11: Transmitter */
#define FDCAN_PSR_EP                (1 << 5) /* Bit 5: Error passive */
#define FDCAN_PSR_EW                (1 << 6) /* Bit 6: Warning status */
#define FDCAN_PSR_BO                (1 << 7) /* Bit 7: Bus_off status */
#define FDCAN_PSR_DLEC_SHIFT        (8)      /* Bits 10-8: Data last error code */
#define FDCAN_PSR_DLEC_MASK         (7 << FDCAN_PSR_DLEC_SHIFT)
#  define FDCAN_PSR_DLEC(n)        ((uint32_t)(n) << FDCAN_PSR_DLEC_SHIFT) /* See error codes above */
#define FDCAN_PSR_RESI              (1 << 11) /* Bit 11: ESI flag of last message */
#define FDCAN_PSR_RBRS              (1 << 12) /* Bit 12: BRS flag of last message */
#define FDCAN_PSR_REDL              (1 << 13) /* Bit 13: Recieved message */
#define FDCAN_PSR_PXE               (1 << 14) /* Bit 14: Protocol exception event */
#define FDCAN_PSR_TDCV_SHIFT        (16) /* Bits 22-16: Transmitter delay compensation */
#define FDCAN_PSR_TDCV_MASK         (127 << FDCAN_PSR_TDCV_SHIFT)

/* FDCAN transmitter delay compensation register */

#define FDCAN_TDCR_TDCF_SHIFT       (0) /* Bits 6-0: Transmitter delay compensation filter window length */
#define FDCAN_TDCR_TDCF_MASK        (127 << FDCAN_TDCR_TDCF_SHIFT)
#  define FDCAN_TDCR_TDCF(value)    ((value) << FDCAN_TDCR_TDCF_SHIFT) /* Value = 0..127 */
#define FDCAN_TDCR_TDCO_SHIFT       (8) /* Bits 14-8: Transmiiter delay compensation offset */
#define FDCAN_TDCR_TDCO_MASK        (127 << FDCAN_TDCR_TDCO_SHIFT)
#  define FDCAN_TDCR_TDCO(value)    ((value) << FDCAN_TDCR_TDCO_SHIFT) /* Value = 0..127 */

/* FDCAN interrupt register */

#define FDCAN_INT_RF0N               (1 << 0) /* Bit 0: Rx FIFO 0 new message */
#define FDCAN_INT_RF0W               (1 << 1) /* Bit 1: Rx FIFO 0 watermark reached */
#define FDCAN_INT_RF0F               (1 << 2) /* Bit 2: Rx FIFO 0 full */
#define FDCAN_INT_RF0L               (1 << 3) /* Bit 3: Rx FIFO 0 message lost */
#define FDCAN_INT_RF1N               (1 << 4) /* Bit 4: Rx FIFO 1 new message */
#define FDCAN_INT_RF1W               (1 << 5) /* Bit 5: Rx FIFO 1 watermark reached */
#define FDCAN_INT_RF1F               (1 << 6) /* Bit 6: Rx FIFO 1 full */
#define FDCAN_INT_RF1L               (1 << 7) /* Bit 7: Rx FIFO 1 message lost */
#define FDCAN_INT_HPM                (1 << 8) /* Bit 8: High priority message */
#define FDCAN_INT_TC                 (1 << 9) /* Bit 9: Transmission completed */
#define FDCAN_INT_TCF                (1 << 10) /* Bit 10: Transmission cancellation finished */
#define FDCAN_INT_TFE                (1 << 11) /* Bit 11: Tx FIFO empty */
#define FDCAN_INT_TEFN               (1 << 12) /* Bit 12: Tx event FIFO new entry */
#define FDCAN_INT_TEFW               (1 << 13) /* Bit 13: Tx event FIFO watermark reached */
#define FDCAN_INT_TEFF               (1 << 14) /* Bit 14: Tx event FIFO full */
#define FDCAN_INT_TEFL               (1 << 15) /* Bit 15: Tx event FIFO element lost */
#define FDCAN_INT_TSW                (1 << 16) /* Bit 16: Timestamp wraparound */
#define FDCAN_INT_MRAF               (1 << 17) /* Bit 17: Message RAM access failure */
#define FDCAN_INT_TOO                (1 << 18) /* Bit 18: Timeout occurred */
#define FDCAN_INT_DRX                (1 << 19) /* Bit 19: Messsage stored in Rx buffer */
#define FDCAN_INT_ELO                (1 << 22) /* Bit 22: Error logging overflow*/
#define FDCAN_INT_EP                 (1 << 23) /* Bit 23: Error_passive status*/
#define FDCAN_INT_EW                 (1 << 24) /* Bit 24: Error_warning status */
#define FDCAN_INT_BO                 (1 << 25) /* Bit 25: Buss_off status */
#define FDCAN_INT_WDI                (1 << 26) /* Bit 26: Watchdog interrupt */
#define FDCAN_INT_PEA                (1 << 27) /* Bit 27: Protocol error arbitration phase*/
#define FDCAN_INT_PED                (1 << 28) /* Bit 28: Protocol error data phase*/
#define FDCAN_INT_ARA                (1 << 29) /* Bit 29: Access to reserved address*/

/* FDCAN interrupt line enable register */

#define FDCAN_ILE_EINT0             (1 << 0) /* Bit 0: Enable interrupt line 0 */
#define FDCAN_ILE_EINT1             (1 << 1) /* Bit 1: Enable interrupt line 1 */

/*FDCAN global filter configuration register */

#define FDCAN_GFC_RRFE              (1 << 0) /* Bit 0: Reject remote frames ext */
#define FDCAN_GFC_RRFS              (1 << 1) /* Bit 1: Reject remote frames std */
#define FDCAN_GFC_ANFE_SHIFT        (2)      /* Bits 3-2: Accept non-matching frames ext */
#define FDCAN_GFC_ANFE_MASK         (3 << FDCAN_GFC_ANFE_SHIFT)
#  define FDCAN_GFC_ANFE_RX_FIFO0   (0 << FDCAN_GFC_ANFE_SHIFT) /* 00: Accept in Rx FIFO 0 */
#  define FDCAN_GFC_ANFE_RX_FIFO1   (1 << FDCAN_GFC_ANFE_SHIFT) /* 01: Accept in Rx FIFO 1 */
#  define FDCAN_GFC_ANFE_REJECTED   (2 << FDCAN_GFC_ANFE_SHIFT) /* 10: Reject */
#define FDCAN_GFC_ANFS_SHIFT        (4)      /* Bits 5-4: Accept non-matching frames std */
#define FDCAN_GFC_ANFS_MASK         (3 << FDCAN_GFC_ANFS_SHIFT)
#  define FDCAN_GFC_ANFS_RX_FIFO0   (0 << FDCAN_GFC_ANFS_SHIFT) /* 00: Accept in Rx FIFO 0 */
#  define FDCAN_GFC_ANFS_RX_FIFO1   (1 << FDCAN_GFC_ANFS_SHIFT) /* 01: Accept in Rx FIFO 1 */
#  define FDCAN_GFC_ANFS_REJECTED   (2 << FDCAN_GFC_ANFS_SHIFT) /* 10: Reject */

/* FDCAN standard ID filter configuration register */

#define FDCAN_SIDFC_FLSSA_SHIFT     (2)  /* Bits 15-2: Filter list standard start address */
#define FDCAN_SIDFC_FLSSA_MASK      (0x3fff << FDCAN_SIDFC_FLSSA_SHIFT)
#  define FDCAN_SIDFC_FLSSA(value)  ((value) << FDCAN_SIDFC_FLSSA_SHIFT)
#define FDCAN_SIDFC_LSS_SHIFT       (16) /* Bits 23-16: List size standard */
#define FDCAN_SIDFC_LSS_MASK        (255 << FDCAN_SIDFC_LSS_SHIFT)
#  define FDCAN_SIDFC_LSS(value)    ((value) << FDCAN_SIDFC_LSS_SHIFT)

/* FDCAN extended ID filter configuration register */

#define FDCAN_XIDFC_FLESA_SHIFT     (2)  /* Bits 15-2: Filter list extended start address */
#define FDCAN_XIDFC_FLESA_MASK      (0x3fff << FDCAN_XIDFC_FLESA_SHIFT)
#  define FDCAN_XIDFC_FLESA(value)  ((value) << FDCAN_XIDFC_FLESA_SHIFT)
#define FDCAN_XIDFC_LSE_SHIFT       (16) /* Bits 23-16: List size extended */
#define FDCAN_XIDFC_LSE_MASK        (255 << FDCAN_XIDFC_LSE_SHIFT)
#  define FDCAN_XIDFC_LSE(value)    ((value) << FDCAN_XIDFC_LSE_SHIFT)

/* FDCAN extended ID and mask register */

#define FDCAN_XIDAM_EIDM_SHIFT      (0) /* Bits 0-28: Extended ID mask */
#define FDCAN_XIDAM_EIDM_MASK       (0x1FFFFFFF << FDCAN_XIDAM_EIDM_SHIFT)

/* FDCAN high priority message status register */

#define FDCAN_HPMS_BIDX_SHIFT       (0)       /* Bits 5-0: Buffer index */
#define FDCAN_HPMS_BIDX_MASK        (63 << FDCAN_HPMS_BIDX_SHIFT)
#  define FDCAN_HPMS_BIDX(value)    ((value) << FDCAN_HPMS_BIDX_SHIFT)
#define FDCAN_HPMS_MSI_SHIFT        (6)       /* Bits 7-6: Message storage indicator */
#define FDCAN_HPMS_MSI_MASK         (3 << FDCAN_HPMS_MSI_SHIFT)
#  define FDCAN_HPMS_MSI(value)     ((value) << FDCAN_HPMS_MSI_SHIFT)
#define FDCAN_HPMS_FIDX_SHIFT       (8)       /* Bits 14-8: Filter index */
#define FDCAN_HPMS_FIDX_MASK        (127 << FDCAN_HPMS_FIDX_SHIFT)
#  define FDCAN_HPMS_FIDX(value)    ((value) << FDCAN_HPMS_FIDX_SHIFT)
#define FDCAN_HPMS_FLST             (1 << 15) /* Bit 15: Filter list */

/* FDCAN New Data 1 Register */

#define FDCAN_NDAT1(n)              (1 << (n)) /* New data for buffer n, n=0-31 */

/* FDCAN New Data 2 Register */

#define FDCAN_NDAT2(n)              (1 << ((n)-32)) /* New data for buffer n, n=32-63 */

/* FDCAN Rx FIFO x configuration register */

#define FDCAN_RXFC_FSA_SHIFT      (2)       /* Bits 15-2: FIFO start address */
#define FDCAN_RXFC_FSA_MASK       (0x3fff << FDCAN_RXFC_FSA_SHIFT)
#  define FDCAN_RXFC_FSA(value)   ((value) << FDCAN_RXFC_FSA_SHIFT)
#define FDCAN_RXFC_FS_SHIFT       (16)      /* Bits 22-16: FIFO size */
#define FDCAN_RXFC_FS_MASK        (127 << FDCAN_RXFC_FS_SHIFT)
#  define FDCAN_RXFC_FS(value)    ((value) << FDCAN_RXFC_FS_SHIFT) /* Value = 0..64 */
#define FDCAN_RXFC_FWM_SHIFT      (24)      /* Bits 30-24: FIFO watermark */
#define FDCAN_RXFC_FWM_MASK       (127 << FDCAN_RXFC_FWM_SHIFT)
#  define FDCAN_RXFC_FWM(value)   ((value) << FDCAN_RXFC_FWM_SHIFT) /* Value = 0..64 */
#define FDCAN_RXFC_FOM            (1 << 31) /* Bit 31: FIFO operation mode */

/* FDCAN Rx FIFO x status register */

#define FDCAN_RXFS_FFL_SHIFT      (0)       /* Bits 6-0: FIFO fill level */
#define FDCAN_RXFS_FFL_MASK       (127 << FDCAN_RXFS_FFL_SHIFT)
#  define FDCAN_RXFS_FFL(value)   ((value) << FDCAN_RXFS_FFL_SHIFT) /* Value = 0..64 */
#define FDCAN_RXFS_FGI_SHIFT      (8)       /* Bits 13-8: FIFO get index */
#define FDCAN_RXFS_FGI_MASK       (63 << FDCAN_RXFS_FGI_SHIFT)
#  define FDCAN_RXFS_FGI(value)   ((value) << FDCAN_RXFS_FGI_SHIFT) /* Value = 0..63 */
#define FDCAN_RXFS_FPI_SHIFT      (16)      /* Bits 21-16: FIFO put index */
#define FDCAN_RXFS_FPI_MASK       (63 << FDCAN_RXFS_FPI_SHIFT)
#  define FDCAN_RXFS_FPI(value)   ((value) << FDCAN_RXFS_FPI_SHIFT) /* Value = 0..63 */
#define FDCAN_RXFS_FF             (1 << 24) /* Bit 24: FIFO full */
#define FDCAN_RXFS_RFL            (1 << 25) /* Bit 25: FIFO message lost */

/* FDCAN Rx FIFO x acknowledge register */

#define FDCAN_RXFA_FAI_SHIFT      (0) /* Bits 5-0: FIFO 0 acknowledge index */
#define FDCAN_RXFA_FAI_MASK       (63 << FDCAN_RXFA_FAI_SHIFT)

/* FDCAN Rx FIFO 1 status register */

#define FDCAN_RXF1S_DMS_SHIFT       (30)      /* Bits 31-30: Debug message status */
#define FDCAN_RXF1S_DMS_MASK        (3 << FDCAN_RXF1S_DMS_SHIFT)
#  define FDCAN_RXF1S_DMS_IDLE      (0 << FDCAN_RXF1S_DMS_SHIFT) /* 00: Idle state */
#  define FDCAN_RXF1S_DMS_A         (1 << FDCAN_RXF1S_DMS_SHIFT) /* 01: Message A */
#  define FDCAN_RXF1S_DMS_AB        (2 << FDCAN_RXF1S_DMS_SHIFT) /* 10: Message A,B */
#  define FDCAN_RXF1S_DMS_ABC       (3 << FDCAN_RXF1S_DMS_SHIFT) /* 11: Message A,B,C*/

/* FDCAN Rx buffer configuration register */

#define FDCAN_RXBC_RBSA_SHIFT      (2) /* Bits 15-2: Buffer start address */
#define FDCAN_RXBC_RBSA_MASK       (63 << FDCAN_RXBC_RBSA_SHIFT)
#  define FDCAN_RXBC_RBSA(value)   ((value) << FDCAN_RXBC_RBSA_SHIFT)

/* FDCAN Rx buffer element size configuration register */

#define FDCAN_RXESC_F0DS_SHIFT      (0) /* Bits 2-0: FIFO 0 data field size */
#define FDCAN_RXESC_F0DS_MASK       (7 << FDCAN_RXESC_F0DS_SHIFT)
#  define FDCAN_RXESC_F0DS_8BYTE    (0 << FDCAN_RXESC_F0DS_SHIFT) /* 000: 8 byte field */
#  define FDCAN_RXESC_F0DS_12BYTE   (1 << FDCAN_RXESC_F0DS_SHIFT) /* 001: 12 byte field */
#  define FDCAN_RXESC_F0DS_16BYTE   (2 << FDCAN_RXESC_F0DS_SHIFT) /* 010: 16 byte field */
#  define FDCAN_RXESC_F0DS_20BYTE   (3 << FDCAN_RXESC_F0DS_SHIFT) /* 011: 20 byte field */
#  define FDCAN_RXESC_F0DS_24BYTE   (4 << FDCAN_RXESC_F0DS_SHIFT) /* 100: 24 byte field */
#  define FDCAN_RXESC_F0DS_32BYTE   (5 << FDCAN_RXESC_F0DS_SHIFT) /* 101: 32 byte field */
#  define FDCAN_RXESC_F0DS_48BYTE   (6 << FDCAN_RXESC_F0DS_SHIFT) /* 110: 48 byte field */
#  define FDCAN_RXESC_F0DS_64BYTE   (7 << FDCAN_RXESC_F0DS_SHIFT) /* 111: 64 byte field */
#  define FDCAN_RXESC_F0DS(value)   ((value) << FDCAN_RXESC_F0DS_SHIFT)
#define FDCAN_RXESC_F1DS_SHIFT      (4) /* Bits 6-4: FIFO 1 data field size */
#define FDCAN_RXESC_F1DS_MASK       (7 << FDCAN_RXESC_F1DS_SHIFT)
#  define FDCAN_RXESC_F1DS_8BYTE    (0 << FDCAN_RXESC_F1DS_SHIFT) /* 000: 8 byte field */
#  define FDCAN_RXESC_F1DS_12BYTE   (1 << FDCAN_RXESC_F1DS_SHIFT) /* 001: 12 byte field */
#  define FDCAN_RXESC_F1DS_16BYTE   (2 << FDCAN_RXESC_F1DS_SHIFT) /* 010: 16 byte field */
#  define FDCAN_RXESC_F1DS_20BYTE   (3 << FDCAN_RXESC_F1DS_SHIFT) /* 011: 20 byte field */
#  define FDCAN_RXESC_F1DS_24BYTE   (4 << FDCAN_RXESC_F1DS_SHIFT) /* 100: 24 byte field */
#  define FDCAN_RXESC_F1DS_32BYTE   (5 << FDCAN_RXESC_F1DS_SHIFT) /* 101: 32 byte field */
#  define FDCAN_RXESC_F1DS_48BYTE   (6 << FDCAN_RXESC_F1DS_SHIFT) /* 110: 48 byte field */
#  define FDCAN_RXESC_F1DS_64BYTE   (7 << FDCAN_RXESC_F1DS_SHIFT) /* 111: 64 byte field */
#  define FDCAN_RXESC_F1DS(value)   ((value) << FDCAN_RXESC_F1DS_SHIFT)
#define FDCAN_RXESC_RBDS_SHIFT      (8) /* Bits 10-8: Rx buffer data field size */
#define FDCAN_RXESC_RBDS_MASK       (7 << FDCAN_RXESC_RBDS_SHIFT)
#  define FDCAN_RXESC_RBDS_8BYTE    (0 << FDCAN_RXESC_RBDS_SHIFT) /* 000: 8 byte field */
#  define FDCAN_RXESC_RBDS_12BYTE   (1 << FDCAN_RXESC_RBDS_SHIFT) /* 001: 12 byte field */
#  define FDCAN_RXESC_RBDS_16BYTE   (2 << FDCAN_RXESC_RBDS_SHIFT) /* 010: 16 byte field */
#  define FDCAN_RXESC_RBDS_20BYTE   (3 << FDCAN_RXESC_RBDS_SHIFT) /* 011: 20 byte field */
#  define FDCAN_RXESC_RBDS_24BYTE   (4 << FDCAN_RXESC_RBDS_SHIFT) /* 100: 24 byte field */
#  define FDCAN_RXESC_RBDS_32BYTE   (5 << FDCAN_RXESC_RBDS_SHIFT) /* 101: 32 byte field */
#  define FDCAN_RXESC_RBDS_48BYTE   (6 << FDCAN_RXESC_RBDS_SHIFT) /* 110: 48 byte field */
#  define FDCAN_RXESC_RBDS_64BYTE   (7 << FDCAN_RXESC_RBDS_SHIFT) /* 111: 64 byte field */
#  define FDCAN_RXESC_RBDS(value)   ((value) << FDCAN_RXESC_RBDS_SHIFT)

/* Tx buffer configuration register */

#define FDCAN_TXBC_TBSA_SHIFT       (2)       /* Bits 15-2: Buffer start address */
#define FDCAN_TXBC_TBSA_MASK        (0x3fff << FDCAN_TXBC_TBSA_SHIFT)
#  define FDCAN_TXBC_TBSA(value)    ((value) << FDCAN_TXBC_TBSA_SHIFT)
#define FDCAN_TXBC_NDTB_SHIFT       (16)      /* Bits 21-16: Number of dedicated buffers */
#define FDCAN_TXBC_NDTB_MASK        (63 << FDCAN_TXBC_NDTB_SHIFT)
#  define FDCAN_TXBC_NDTB(value)    ((value) << FDCAN_TXBC_NDTB_SHIFT) /* Value = 0..32 */
#define FDCAN_TXBC_TFQS_SHIFT       (24)      /* Bits 29-24: FIFO/queue size */
#define FDCAN_TXBC_TFQS_MASK        (63 << FDCAN_TXBC_TFQS_SHIFT)
#  define FDCAN_TXBC_TFQS(value)    ((value) << FDCAN_TXBC_TFQS_SHIFT) /* Value = 0..32 */
#define FDCAN_TXBC_TFQM             (1 << 30) /* Bit 30: FIFO/queue mode */

/* Tx FIFO/queue status register */

#define FDCAN_TXFQS_TFFL_SHIFT      (0)       /* Bits 5-0: FIFO free level */
#define FDCAN_TXFQS_TFFL_MASK       (63 << FDCAN_TXFQS_TFFL_SHIFT)
#define FDCAN_TXFQS_TFGI_SHIFT      (8)       /* Bits 12-8: FIFO get index */
#define FDCAN_TXFQS_TFGI_MASK       (31 << FDCAN_TXFQS_TFGI_SHIFT)
#define FDCAN_TXFQS_TFQPI_SHIFT     (16)      /* Bits 20-16: FIFO/queue put index */
#define FDCAN_TXFQS_TFQPI_MASK      (31 << FDCAN_TXFQS_TFQPI_SHIFT)
#define FDCAN_TXFQS_TFQF            (1 << 21) /*Bit 21: FIFO/queue full */

/* Tx buffer element size configuration register */

#define FDCAN_TXESC_TBDS_SHIFT      (0) /* Bits 2-0: Buffer data field size */
#define FDCAN_TXESC_TBDS_MASK       (7 << FDCAN_TXESC_TBDS_SHIFT)
#  define FDCAN_TXESC_TBDS_8BYTE    (0 << FDCAN_TXESC_TBDS_SHIFT) /* 000: 8 byte field */
#  define FDCAN_TXESC_TBDS_12BYTE   (1 << FDCAN_TXESC_TBDS_SHIFT) /* 001: 12 byte field */
#  define FDCAN_TXESC_TBDS_16BYTE   (2 << FDCAN_TXESC_TBDS_SHIFT) /* 010: 16 byte field */
#  define FDCAN_TXESC_TBDS_20BYTE   (3 << FDCAN_TXESC_TBDS_SHIFT) /* 011: 20 byte field */
#  define FDCAN_TXESC_TBDS_24BYTE   (4 << FDCAN_TXESC_TBDS_SHIFT) /* 100: 24 byte field */
#  define FDCAN_TXESC_TBDS_32BYTE   (5 << FDCAN_TXESC_TBDS_SHIFT) /* 101: 32 byte field */
#  define FDCAN_TXESC_TBDS_48BYTE   (6 << FDCAN_TXESC_TBDS_SHIFT) /* 110: 48 byte field */
#  define FDCAN_TXESC_TBDS_64BYTE   (7 << FDCAN_TXESC_TBDS_SHIFT) /* 111: 64 byte field */
#  define FDCAN_TXESC_TBDS(value)   ((value) << FDCAN_TXESC_TBDS_SHIFT)

/* Transmit buffer request pending register */

#define FDCAN_TXBRP(n)              (1 << (n)) /* Transmission request pending for buffer n, n=0-31 */

/* Transmit buffer add request register */

#define FDCAN_TXBAR(n)              (1 << (n)) /* Add request for transmit buffer n, n=0-31 */

/* Transmit buffer cancellation request register */

#define FDCAN_TXBCR(n)              (1 << (n)) /* Cancellation request for transmit buffer n, n=0-31 */

/* Transmit buffer transmission occurred register */

#define FDCAN_TXBTO(n)              (1 << (n)) /* Transmission occurred for buffer n, n=0-31 */

/* Transmit buffer cancellation finished register */

#define FDCAN_TXBCF(n)              (1 << (n)) /* Cancellation finished for transmit buffer n, n=0-31 */

/* Transmit buffer transmission interrupt enable register */

#define FDCAN_TXBTIE(n)             (1 << (n)) /* Transmission interrupt enable for buffer n, n=0-31 */

/* Transmit buffer cancellation finished interrupt enable register */

#define FDCAN_TXBCIE(n)             (1 << (n)) /* Cancellation finished interrupt enable for transmit buffer n, n=0-31 */


/* FDCAN Tx event FIFO configuration register */

#define FDCAN_TXEFC_EFSA_SHIFT      (2)  /* Bits 15-2: Event FIFO start address */
#define FDCAN_TXEFC_EFSA_MASK       (0x3fff << FDCAN_TXEFC_EFSA_SHIFT)
#  define FDCAN_TXEFC_EFSA(value)   ((value) << FDCAN_TXEFC_EFSA_SHIFT)
#define FDCAN_TXEFC_EFS_SHIFT       (16) /* Bits 21-16: Event FIFO size */
#define FDCAN_TXEFC_EFS_MASK        (63 << FDCAN_TXEFC_EFS_SHIFT)
#  define FDCAN_TXEFC_EFS(value)    ((value) << FDCAN_TXEFC_EFS_SHIFT)  /* Value = 0..32 */
#define FDCAN_TXEFC_EFWM_SHIFT      (24) /* Bits 29-24: Event FIFO watermark */
#define FDCAN_TXEFC_EFWM_MASK       (63 << FDCAN_TXEFC_EFWM_SHIFT)
#  define FDCAN_TXEFC_EFWM(value)   ((value) << FDCAN_TXEFC_EFWM_SHIFT) /* Value = 0..32 */

/* FDCAN Tx event FIFO status register */

#define FDCAN_TXEFS_EFFL_SHIFT      (0) /* Bits 5-0: Event FIFO fill level */
#define FDCAN_TXEFS_EFFL_MASK       (63 << FDCAN_TXEFS_EFFL_SHIFT)
#  define FDCAN_TXEFS_EFFL(value)   ((value) << FDCAN_TXEFS_EFFL_SHIFT) /* Value = 0..31 */
#define FDCAN_TXEFS_EFGI_SHIFT      (8) /* Bits 12-8: Event FIFO get index */
#define FDCAN_TXEFS_EFGI_MASK       (31 << FDCAN_TXEFS_EFGI_SHIFT)
#  define FDCAN_TXEFS_EFGI(value)   ((value) << FDCAN_TXEFS_EFGI_SHIFT) /* Value = 0..31 */
#define FDCAN_TXEFS_EFPI_SHIFT      (16)/* Bits 20-16: Event FIFO put index */
#define FDCAN_TXEFS_EFPI_MASK       (31 << FDCAN_TXEFS_EFPI_SHIFT)
#  define FDCAN_TXEFS_EFPI(value)   ((value) << FDCAN_TXEFS_EFPI_SHIFT) /* Value = 0..31 */
#define FDCAN_TXEFS_EFF             (1 << 24) /* Bit 24: Event FIFO full */
#define FDCAN_TXEFS_TEFL            (1 << 25) /* Bit 25: Tx Event FIFO element lost */

/* FDCAN Tx event FIFO acknowledge register */

#define FDCAN_TXEFA_EFAI_SHIFT      (0) /* Bits 4-0: Event FIFO acknowledge index */
#define FDCAN_TXEFA_EFAI_MASK       (31 << FDCAN_TXEFA_EFAI_SHIFT)

/* FDCAN TT operation configuration register */

#define FDCAN_TTOCF_OM_SHIFT        (0) /* Bits 1-0: Operation mode */
#define FDCAN_TTOCF_OM_MASK         (3 << FDCAN_TTOCF_OM_SHIFT)
#  define FDCAN_TTOCF_OM_EVENT      (0 << FDCAN_TTOCF_OM_SHIFT) /* 00: Event-driven CAN */
#  define FDCAN_TTOCF_OM_LVL1       (1 << FDCAN_TTOCF_OM_SHIFT) /* 01: TTCAN level 1 */
#  define FDCAN_TTOCF_OM_LVL2       (2 << FDCAN_TTOCF_OM_SHIFT) /* 10: TTCAN level 2 */
#  define FDCAN_TTOCF_OM_LVL0       (3 << FDCAN_TTOCF_OM_SHIFT) /* 11: TTCAN level 0 */

/* CCU configuration register */

#define FDCAN_CCU_CCFG_TQBT_SHIFT    (0)      /* Bits 4-0: Time quanta per bit time */
#define FDCAN_CCU_CCFG_TQBT_MASK     (31 << FDCAN_CCU_CCFG_TQBT_SHIFT)
#define FDCAN_CCU_CCFG_BCC           (1 << 6) /* Bit 6: Bypass clock calibration */
#define FDCAN_CCU_CCFG_CFL           (1 << 7) /* Bit 7: Calibration field length */
#define FDCAN_CCU_CCFG_OCPM_SHIFT    (8)      /* Bits 15-8: Osc. clock periods min */
#define FDCAN_CCU_CCFG_OCPM_MASK     (255 << FDCAN_CCU_CCFG_OCPM_SHIFT)
#  define FDCAN_CCU_CCFG_OCPM(value) ((value) << FDCAN_CCU_CCFG_OCPM_SHIFT)
#define FDCAN_CCU_CCFG_CDIV_SHIFT    (16)     /* Bits 19-16: Clock divider */
#define FDCAN_CCU_CCFG_CDIV_MASK     (15 << FDCAN_CCU_CCFG_CDIV_SHIFT)
#  define FDCAN_CCU_CCFG_CDIV1       (0 << FDCAN_CCU_CCFG_CDIV_SHIFT) /* 0000: Divide by 1 */
#  define FDCAN_CCU_CCFG_CDIV2       (1 << FDCAN_CCU_CCFG_CDIV_SHIFT) /* 0001: Divide by 2 */
#  define FDCAN_CCU_CCFG_CDIV4       (2 << FDCAN_CCU_CCFG_CDIV_SHIFT) /* 0010: Divide by 4 */
#  define FDCAN_CCU_CCFG_CDIV6       (3 << FDCAN_CCU_CCFG_CDIV_SHIFT) /* 0011: Divide by 6 */
#  define FDCAN_CCU_CCFG_CDIV8       (4 << FDCAN_CCU_CCFG_CDIV_SHIFT) /* 0100: Divide by 8 */
#  define FDCAN_CCU_CCFG_CDIV10      (5 << FDCAN_CCU_CCFG_CDIV_SHIFT) /* 0101: Divide by 10 */
#  define FDCAN_CCU_CCFG_CDIV12      (5 << FDCAN_CCU_CCFG_CDIV_SHIFT) /* 0110: Divide by 12 */
#  define FDCAN_CCU_CCFG_CDIV14      (5 << FDCAN_CCU_CCFG_CDIV_SHIFT) /* 0111: Divide by 14 */
#  define FDCAN_CCU_CCFG_CDIV16      (5 << FDCAN_CCU_CCFG_CDIV_SHIFT) /* 1000: Divide by 16 */
#  define FDCAN_CCU_CCFG_CDIV18      (5 << FDCAN_CCU_CCFG_CDIV_SHIFT) /* 1001: Divide by 18 */
#  define FDCAN_CCU_CCFG_CDIV20      (5 << FDCAN_CCU_CCFG_CDIV_SHIFT) /* 1010: Divide by 20 */
#  define FDCAN_CCU_CCFG_CDIV22      (5 << FDCAN_CCU_CCFG_CDIV_SHIFT) /* 1011: Divide by 22 */
#  define FDCAN_CCU_CCFG_CDIV24      (5 << FDCAN_CCU_CCFG_CDIV_SHIFT) /* 1100: Divide by 24 */
#  define FDCAN_CCU_CCFG_CDIV26      (5 << FDCAN_CCU_CCFG_CDIV_SHIFT) /* 1101: Divide by 26 */
#  define FDCAN_CCU_CCFG_CDIV28      (5 << FDCAN_CCU_CCFG_CDIV_SHIFT) /* 1110: Divide by 28 */
#  define FDCAN_CCU_CCFG_CDIV30      (5 << FDCAN_CCU_CCFG_CDIV_SHIFT) /* 1111: Divide by 30 */
#define FDCAN_CCU_CCFG_SWR           (1 << 31) /* Bit 31: Software reset */

/* CCU Calibration watchdog register */

#define FDCAN_CCU_CWD_WDC_SHIFT      (0)      /* Bits 15-0: Watchdog counter start value */
#define FDCAN_CCU_CWD_WDC_MASK       (0xffff << FDCAN_CCU_CWD_WDC_SHIFT)
#  define FDCAN_CCU_CWD_WDC_DIS      (0 << FDCAN_CCU_CWD_WDC_SHIFT)       /* Counter disabled */
#  define FDCAN_CCU_CWD_WDC(value)   ((value) << FDCAN_CCU_CWD_WDC_SHIFT) /* Value = 1..255 */
#define FDCAN_CCU_CWD_WDV_SHIFT      (16)      /* Bits 31-16: Watchdog counter value */
#define FDCAN_CCU_CWD_WDV_MASK       (0xffff << FDCAN_CCU_CWD_WDV_SHIFT)

/* CCU interrupt register */

#define FDCAN_CCU_IR_CWE             (1 << 0) /* Bit 0: Calibration watchdog event */
#define FDCAN_CCU_IR_CSC             (1 << 1) /* Bit 1: Calibration state changed */

/* CCU interrupt enable register */

#define FDCAN_CCU_IE_CWEE            (1 << 0) /* Bit 0: Calibration watchdog event enable */
#define FDCAN_CCU_IE_CSCE            (1 << 1) /* Bit 1: Calibration state changed enable */

/* Message RAM Definitions **************************************************************/
/* Common Buffer and FIFO element bit definitions:
 *
 *   --------------- ------------------- --------------------------------
 *   RESOURCE               R0                        R1
 *   --------------- ------------------- --------------------------------
 *   RX Buffer:      ESI, XTD, RTR, ID,  ANMF, FIDX, EDL, BRS, DLC, RXTS
 *   RX FIFO:        ESI, XTD, RTR, ID,  ANMF, FIDX, EDL, BRS, DLC, RXTS
 *   TX buffer:           XTD, RTR, ID,  MM,   EFC,            DLC
 *   TX Event FIFO:  ESI, XTD, RTR, ID,  MM,   ET,   EDL, BRS, DLC, TXTS
 *   --------------- ------------------- --------------------------------
 */

/* Common */

#define BUFFER_R0_EXTID_SHIFT      (0)       /* Bits 0-28: Extended identifier */
#define BUFFER_R0_EXTID_MASK       (0x1fffffff << BUFFER_R0_EXTID_SHIFT)
#  define BUFFER_R0_EXTID(n)       ((uint32_t)(n) << BUFFER_R0_EXTID_SHIFT)
#define BUFFER_R0_STDID_SHIFT      (18)      /* Bits 18-28: Standard identifier */
#define BUFFER_R0_STDID_MASK       (0x7ff << BUFFER_R0_STDID_SHIFT)
#  define BUFFER_R0_STDID(n)       ((uint32_t)(n) << BUFFER_R0_STDID_SHIFT)
#define BUFFER_R0_RTR              (1 << 29) /* Bit 29: Remote Transmission Request */
#define BUFFER_R0_XTD              (1 << 30) /* Bit 30: Extended Identifier */
#define BUFFER_R0_ESI              (1 << 31) /* Bit 31: Error State Indicator */

/* Common */

#define BUFFER_R1_DLC_SHIFT        (16)      /* Bits 16-19: Date length code */
#define BUFFER_R1_DLC_MASK         (15 << BUFFER_R1_DLC_SHIFT)
#  define BUFFER_R1_DLC(n)         ((uint32_t)(n) << BUFFER_R1_DLC_SHIFT)
#define BUFFER_R1_BRS              (1 << 20) /* Bit 20: Bit Rate Switch */
#define BUFFER_R1_FDF              (1 << 21) /* Bit 21: FD Format */

/* RX buffer/RX FIFOs */

#define BUFFER_R1_RXTS_SHIFT       (0)       /* Bits 0-15: RX Timestamp */
#define BUFFER_R1_RXTS_MASK        (0xffff << BUFFER_R1_RXTS_SHIFT)
#  define BUFFER_R1_RXTS(n)        ((uint32_t)(n) << BUFFER_R1_RXTS_SHIFT)
#define BUFFER_R1_FIDX_SHIFT       (24)      /* Bits 24-30: Filter index */
#define BUFFER_R1_FIDX_MASK        (0x7f << BUFFER_R1_FIDX_SHIFT)
#  define BUFFER_R1_FIDX(n)        ((uint32_t)(n) << BUFFER_R1_FIDX_SHIFT)
#define BUFFER_R1_ANMF             (1 << 31) /* Bit 31: Accepted Non-matching Frame */

/* TX buffer/TX Event FIFO */

#define BUFFER_R1_MM_SHIFT         (24)      /* Bits 24-31: Message Marker */
#define BUFFER_R1_MM_MASK          (0xff << BUFFER_R1_MM_SHIFT)
#  define BUFFER_R1_MM(n)          ((uint32_t)(n) << BUFFER_R1_MM_SHIFT)

/* TX buffer */

#define BUFFER_R1_EFC              (1 << 23) /* Bit 23: Event FIFO Control */

/* TX Event FIFO */

#define BUFFER_R1_TXTS_SHIFT       (0)       /* Bits 0-15: TX Timestamp */
#define BUFFER_R1_TXTS_MASK        (0xffff << BUFFER_R1_TXTS_SHIFT)
#  define BUFFER_R1_TXTS(n)        ((uint32_t)(n) << BUFFER_R1_TXTS_SHIFT)
#define BUFFER_R1_EDL              (1 << 21) /* Bit 21: Extended Data Length */
#define BUFFER_R1_ET_SHIFT         (22)      /* Bits 22-23: Event Type */
#define BUFFER_R1_ET_MASK          (3 << BUFFER_R1_ET_SHIFT)
#  define BUFFER_R1_ET_TXEVENT     (1 << BUFFER_R1_ET_SHIFT) /* Tx event */
#  define BUFFER_R1_ET_TXCANCEL    (2 << BUFFER_R1_ET_SHIFT) /* Transmission despite cancellation */

/* Standard Message ID Filter Element */

#define STDFILTER_S0_SFID2_SHIFT   (0)       /* Bits 0-10: Standard Filter ID 2 */
#define STDFILTER_S0_SFID2_MASK    (0x7ff << STDFILTER_S0_SFID2_SHIFT)
#  define STDFILTER_S0_SFID2(n)    ((uint32_t)(n) << STDFILTER_S0_SFID2_SHIFT)
#define STDFILTER_S0_BUFFER_SHIFT  (0)       /* Bits 0-5: RX buffer start address */
#define STDFILTER_S0_BUFFER_MASK   (63 << STDFILTER_S0_BUFFER_SHIFT)
#  define STDFILTER_S0_BUFFER(n)   ((uint32_t)(n) << STDFILTER_S0_BUFFER_SHIFT)
#define STDFILTER_S0_ACTION_SHIFT  (9)       /* Bits 9-10: Action taken */
#define STDFILTER_S0_ACTION_MASK   (3 << STDFILTER_S0_ACTION_SHIFT)
#  define STDFILTER_S0_RXBUFFER    (0 << STDFILTER_S0_ACTION_SHIFT) /* Store message in a Rx buffer */
#  define STDFILTER_S0_DEBUGA      (1 << STDFILTER_S0_ACTION_SHIFT) /* Debug Message A */
#  define STDFILTER_S0_DEBUGB      (2 << STDFILTER_S0_ACTION_SHIFT) /* Debug Message B */
#  define STDFILTER_S0_DEBUGC      (3 << STDFILTER_S0_ACTION_SHIFT) /* Debug Message C */
#define STDFILTER_S0_SFID1_SHIFT   (16)      /* Bits 16-26: Standard Filter ID 2 */
#define STDFILTER_S0_SFID1_MASK    (0x7ff << STDFILTER_S0_SFID1_SHIFT)
#  define STDFILTER_S0_SFID1(n)    ((uint32_t)(n) << STDFILTER_S0_SFID1_SHIFT)
#define STDFILTER_S0_SFEC_SHIFT    (27)      /* Bits 27-29: Standard Filter Element Configuration */
#define STDFILTER_S0_SFEC_MASK     (7 << STDFILTER_S0_SFEC_SHIFT)
#  define STDFILTER_S0_SFEC_DISABLE   (0 << STDFILTER_S0_SFEC_SHIFT) /* Disable filter element */
#  define STDFILTER_S0_SFEC_FIFO0     (1 << STDFILTER_S0_SFEC_SHIFT) /* Store in Rx FIFO 0 on match */
#  define STDFILTER_S0_SFEC_FIFO1     (2 << STDFILTER_S0_SFEC_SHIFT) /* Store in Rx FIFO 1 on match */
#  define STDFILTER_S0_SFEC_REJECT    (3 << STDFILTER_S0_SFEC_SHIFT) /* Reject ID on match */
#  define STDFILTER_S0_SFEC_PRIORITY  (4 << STDFILTER_S0_SFEC_SHIFT) /* Set priority ion match */
#  define STDFILTER_S0_SFEC_PRIOFIFO0 (5 << STDFILTER_S0_SFEC_SHIFT) /* Set priority and store in FIFO 0 on match */
#  define STDFILTER_S0_SFEC_PRIOFIFO1 (6 << STDFILTER_S0_SFEC_SHIFT) /* Set priority and store in FIFO 1 on match */
#  define STDFILTER_S0_SFEC_BUFFER    (7 << STDFILTER_S0_SFEC_SHIFT) /* Store into Rx Buffer or as debug message */
#define STDFILTER_S0_SFT_SHIFT     (30)      /* Bits 30-31: Standard Filter Type */
#define STDFILTER_S0_SFT_MASK      (3 << STDFILTER_S0_SFT_SHIFT)
#  define STDFILTER_S0_SFT_RANGE   (0 << STDFILTER_S0_SFT_SHIFT) /* Range filter from SF1ID to SF2ID */
#  define STDFILTER_S0_SFT_DUAL    (1 << STDFILTER_S0_SFT_SHIFT) /* Dual ID filter for SF1ID or SF2ID */
#  define STDFILTER_S0_SFT_CLASSIC (2 << STDFILTER_S0_SFT_SHIFT) /* Classic filter: SF1ID=filter SF2ID=mask */

/* Extended Message ID Filter Element */

#define EXTFILTER_F0_EFID1_SHIFT   (0)    /* Bits 0-28: Extended Filter ID 1 */
#define EXTFILTER_F0_EFID1_MASK    (0x1fffffff << EXTFILTER_F0_EFID1_SHIFT)
#  define EXTFILTER_F0_EFID1(n)    ((uint32_t)(n) << EXTFILTER_F0_EFID1_SHIFT)
#define EXTFILTER_F0_EFEC_SHIFT    (29)  /* Bits 29-31: Extended Filter Element Configuration */
#define EXTFILTER_F0_EFEC_MASK     (7 << EXTFILTER_F0_EFEC_SHIFT)
#  define EXTFILTER_F0_EFEC_DISABLE    (0 << EXTFILTER_F0_EFEC_SHIFT) /* Disable filter element */
#  define EXTFILTER_F0_EFEC_FIFO0      (1 << EXTFILTER_F0_EFEC_SHIFT) /* Store in Rx FIFO 0 on match */
#  define EXTFILTER_F0_EFEC_FIFO1      (2 << EXTFILTER_F0_EFEC_SHIFT) /* Store in Rx FIFO 1 on match */
#  define EXTFILTER_F0_EFEC_REJECT     (3 << EXTFILTER_F0_EFEC_SHIFT) /* Reject ID on match */
#  define EXTFILTER_F0_EFEC_PRIORITY   (4 << EXTFILTER_F0_EFEC_SHIFT) /* Set priority on match */
#  define EXTFILTER_F0_EFEC_PRIOFIFO0  (5 << EXTFILTER_F0_EFEC_SHIFT) /* Set priority and store in FIFO 0 on match */
#  define EXTFILTER_F0_EFEC_PRIOFIFO1  (6 << EXTFILTER_F0_EFEC_SHIFT) /* Set priority and store in FIFO 1 on match */
#  define EXTFILTER_F0_EFEC_BUFFER     (7 << EXTFILTER_F0_EFEC_SHIFT) /* Store into Rx Buffer or as debug message */

#define EXTFILTER_F1_EFID2_SHIFT   (0)       /* Bits 0-28: Extended Filter ID 2 */
#define EXTFILTER_F1_EFID2_MASK    (0x1fffffff << EXTFILTER_F1_EFID2_SHIFT)
#  define EXTFILTER_F1_EFID2(n)    ((uint32_t)(n) << EXTFILTER_F1_EFID2_SHIFT)
#define EXTFILTER_F1_BUFFER_SHIFT  (0)       /* Bits 0-5: RX buffer start address */
#define EXTFILTER_F1_BUFFER_MASK   (63 << EXTFILTER_F1_BUFFER_SHIFT)
#  define EXTFILTER_F1_BUFFER(n)   ((uint32_t)(n) << EXTFILTER_F1_BUFFER_SHIFT)
#define EXTFILTER_F1_ACTION_SHIFT  (9)       /* Bits 9-10: Action taken */
#define EXTFILTER_F1_ACTION_MASK   (3 << EXTFILTER_F1_ACTION_SHIFT)
#  define EXTFILTER_F1_RXBUFFER    (0 << EXTFILTER_F1_ACTION_SHIFT) /* Store message in a Rx buffer */
#  define EXTFILTER_F1_DEBUGA      (1 << EXTFILTER_F1_ACTION_SHIFT) /* Debug Message A */
#  define EXTFILTER_F1_DEBUGB      (2 << EXTFILTER_F1_ACTION_SHIFT) /* Debug Message B */
#  define EXTFILTER_F1_DEBUGC      (3 << EXTFILTER_F1_ACTION_SHIFT) /* Debug Message C */
#define EXTFILTER_F1_EFT_SHIFT     (30)      /* Bits 30-31: Extended Filter Type */
#define EXTFILTER_F1_EFT_MASK      (3 << EXTFILTER_F1_EFT_SHIFT)
#  define EXTFILTER_F1_EFT_RANGE   (0 << EXTFILTER_F1_EFT_SHIFT) /* Range filter from SF1ID to SF2ID */
#  define EXTFILTER_F1_EFT_DUAL    (1 << EXTFILTER_F1_EFT_SHIFT) /* Dual ID filter for SF1ID or SF2ID */
#  define EXTFILTER_F1_EFT_CLASSIC (2 << EXTFILTER_F1_EFT_SHIFT) /* Classic filter: SF1ID=filter SF2ID=mask */
#  define EXTFILTER_F1_EFT_NOXIDAM (3 << EXTFILTER_F1_EFT_SHIFT) /* Range filter from EF1ID to EF2ID, no XIDAM */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_FDCAN_H */
