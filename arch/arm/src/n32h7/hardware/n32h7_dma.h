/****************************************************************************
 * arch/arm/src/n32h7/hardware/n32h7_dma.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_N32H7_HARDWARE_N32H7_DMA_H
#define __ARCH_ARM_SRC_N32H7_HARDWARE_N32H7_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

#include "n32h7_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 3 DMA controllers + 1 MDMA */

#define MDMA                      (0)
#define DMA1                      (1)
#define DMA2                      (2)
#define DMA3                      (3)

/* 8 DMA chanels for standard DMA */

#define DMA_CH0                   (0)
#define DMA_CH1                   (1)
#define DMA_CH2                   (2)
#define DMA_CH3                   (3)
#define DMA_CH4                   (4)
#define DMA_CH5                   (5)
#define DMA_CH6                   (6)
#define DMA_CH7                   (7)

/* ==========================================================================
 * Channel Register Offset（Each channel 0x58 bytes per channel）
 * ==========================================================================
 */
#define N32_DMA_CH_OFFSET(n)     (0x58U * (n))

#define N32_DMA_CH_SA_OFFSET     0x0000U /* Source Address */
#define N32_DMA_CH_DA_OFFSET     0x0008U /* Destination Address */
#define N32_DMA_CH_LLP_OFFSET    0x0010U /* Linked List Pointer */
#define N32_DMA_CH_CTRL_OFFSET   0x0018U /* Control Register */
#define N32_DMA_CH_CFG_OFFSET    0x0040U /* Configuration Register */
#define N32_DMA_CH_SG_OFFSET     0x0048U /* Source Gather */
#define N32_DMA_CH_DS_OFFSET     0x0050U /* Destination Scatter */

/* ==========================================================================
 * Global Interrupt Register Offset
 * ==========================================================================
 */
#define N32_DMA_RAWTCINTSTS_OFFSET   0x02C0U
#define N32_DMA_RAWBTCINTSTS_OFFSET  0x02C8U
#define N32_DMA_RAWSTCINTSTS_OFFSET  0x02D0U
#define N32_DMA_RAWDTCINTSTS_OFFSET  0x02D8U
#define N32_DMA_RAWERRINTSTS_OFFSET  0x02E0U

#define N32_DMA_TCINTSTS_OFFSET      0x02E8U
#define N32_DMA_BTCINTSTS_OFFSET     0x02F0U
#define N32_DMA_STCINTSTS_OFFSET     0x02F8U
#define N32_DMA_DTCINTSTS_OFFSET     0x0300U
#define N32_DMA_ERRINTSTS_OFFSET     0x0308U

#define N32_DMA_TCINTMSK_OFFSET      0x0310U
#define N32_DMA_BTCINTMSK_OFFSET     0x0318U
#define N32_DMA_STCINTMSK_OFFSET     0x0320U
#define N32_DMA_DTCINTMSK_OFFSET     0x0328U
#define N32_DMA_ERRINTMSK_OFFSET     0x0330U

#define N32_DMA_TCINTCLR_OFFSET      0x0338U
#define N32_DMA_BTCINTCLR_OFFSET     0x0340U
#define N32_DMA_STCINTCLR_OFFSET     0x0348U
#define N32_DMA_DTCINTCLR_OFFSET     0x0350U
#define N32_DMA_ERRINTCLR_OFFSET     0x0358U

#define N32_DMA_INTCBESTS_OFFSET     0x0360U

/* ==========================================================================
 * Software Handshake Register Offset
 * ==========================================================================
 */
#define N32_DMA_SRCSWTREQ_OFFSET     0x0368U
#define N32_DMA_DSTSWTREQ_OFFSET     0x0370U
#define N32_DMA_SRCSGTREQ_OFFSET     0x0378U
#define N32_DMA_DSTSGTREQ_OFFSET     0x0380U
#define N32_DMA_SRCLTREQ_OFFSET      0x0388U
#define N32_DMA_DSTLTREQ_OFFSET      0x0390U

/* ==========================================================================
 * DMA Miscellaneous Register Offset
 * ==========================================================================
 */
#define N32_DMA_CFG_OFFSET          0x0398U /* DMA Configuration Register */
#define N32_DMA_CHEN_OFFSET         0x03A0U /* DMA Channel Enable Register */
#define N32_DMA_ID_OFFSET           0x03A8U /* DMA ID Register */
#define N32_DMA_TEST_OFFSET         0x03B0U /* DMA Test Register */
#define N32_DMA_LPTIMEOUT_OFFSET    0x03B8U /* DMA Low Power Timeout Register */
#define N32_DMA_COMPID_OFFSET       0x03F8U /* DMA Component ID Register */

/* ==========================================================================
 * Channel Register Address Macro Definitions
 * ==========================================================================
 */
#define N32_DMA1_CH_SA(n)           (N32_DMA1_BASE + N32_DMA_CH_OFFSET(n) + N32_DMA_CH_SA_OFFSET)
#define N32_DMA1_CH_DA(n)           (N32_DMA1_BASE + N32_DMA_CH_OFFSET(n) + N32_DMA_CH_DA_OFFSET)
#define N32_DMA1_CH_LLP(n)          (N32_DMA1_BASE + N32_DMA_CH_OFFSET(n) + N32_DMA_CH_LLP_OFFSET)
#define N32_DMA1_CH_CTRL(n)         (N32_DMA1_BASE + N32_DMA_CH_OFFSET(n) + N32_DMA_CH_CTRL_OFFSET)
#define N32_DMA1_CH_CFG(n)          (N32_DMA1_BASE + N32_DMA_CH_OFFSET(n) + N32_DMA_CH_CFG_OFFSET)
#define N32_DMA1_CH_SG(n)           (N32_DMA1_BASE + N32_DMA_CH_OFFSET(n) + N32_DMA_CH_SG_OFFSET)
#define N32_DMA1_CH_DS(n)           (N32_DMA1_BASE + N32_DMA_CH_OFFSET(n) + N32_DMA_CH_DS_OFFSET)

#define N32_DMA2_CH_SA(n)           (N32_DMA2_BASE + N32_DMA_CH_OFFSET(n) + N32_DMA_CH_SA_OFFSET)
#define N32_DMA2_CH_DA(n)           (N32_DMA2_BASE + N32_DMA_CH_OFFSET(n) + N32_DMA_CH_DA_OFFSET)
#define N32_DMA2_CH_LLP(n)          (N32_DMA2_BASE + N32_DMA_CH_OFFSET(n) + N32_DMA_CH_LLP_OFFSET)
#define N32_DMA2_CH_CTRL(n)         (N32_DMA2_BASE + N32_DMA_CH_OFFSET(n) + N32_DMA_CH_CTRL_OFFSET)
#define N32_DMA2_CH_CFG(n)          (N32_DMA2_BASE + N32_DMA_CH_OFFSET(n) + N32_DMA_CH_CFG_OFFSET)
#define N32_DMA2_CH_SG(n)           (N32_DMA2_BASE + N32_DMA_CH_OFFSET(n) + N32_DMA_CH_SG_OFFSET)
#define N32_DMA2_CH_DS(n)           (N32_DMA2_BASE + N32_DMA_CH_OFFSET(n) + N32_DMA_CH_DS_OFFSET)

#define N32_DMA3_CH_SA(n)           (N32_DMA3_BASE + N32_DMA_CH_OFFSET(n) + N32_DMA_CH_SA_OFFSET)
#define N32_DMA3_CH_DA(n)           (N32_DMA3_BASE + N32_DMA_CH_OFFSET(n) + N32_DMA_CH_DA_OFFSET)
#define N32_DMA3_CH_LLP(n)          (N32_DMA3_BASE + N32_DMA_CH_OFFSET(n) + N32_DMA_CH_LLP_OFFSET)
#define N32_DMA3_CH_CTRL(n)         (N32_DMA3_BASE + N32_DMA_CH_OFFSET(n) + N32_DMA_CH_CTRL_OFFSET)
#define N32_DMA3_CH_CFG(n)          (N32_DMA3_BASE + N32_DMA_CH_OFFSET(n) + N32_DMA_CH_CFG_OFFSET)
#define N32_DMA3_CH_SG(n)           (N32_DMA3_BASE + N32_DMA_CH_OFFSET(n) + N32_DMA_CH_SG_OFFSET)
#define N32_DMA3_CH_DS(n)           (N32_DMA3_BASE + N32_DMA_CH_OFFSET(n) + N32_DMA_CH_DS_OFFSET)

/* ==========================================================================
 * Global Interrupt Register Offset
 * ==========================================================================
 */
#define N32_DMA1_RAWTCINTSTS        (N32_DMA1_BASE + N32_DMA_RAWTCINTSTS_OFFSET)
#define N32_DMA1_TCINTSTS           (N32_DMA1_BASE + N32_DMA_TCINTSTS_OFFSET)
#define N32_DMA1_TCINTMSK           (N32_DMA1_BASE + N32_DMA_TCINTMSK_OFFSET)
#define N32_DMA1_TCINTCLR           (N32_DMA1_BASE + N32_DMA_TCINTCLR_OFFSET)

#define N32_DMA2_RAWTCINTSTS        (N32_DMA2_BASE + N32_DMA_RAWTCINTSTS_OFFSET)
#define N32_DMA2_TCINTSTS           (N32_DMA2_BASE + N32_DMA_TCINTSTS_OFFSET)
#define N32_DMA2_TCINTMSK           (N32_DMA2_BASE + N32_DMA_TCINTMSK_OFFSET)
#define N32_DMA2_TCINTCLR           (N32_DMA2_BASE + N32_DMA_TCINTCLR_OFFSET)

#define N32_DMA3_RAWTCINTSTS        (N32_DMA3_BASE + N32_DMA_RAWTCINTSTS_OFFSET)
#define N32_DMA3_TCINTSTS           (N32_DMA3_BASE + N32_DMA_TCINTSTS_OFFSET)
#define N32_DMA3_TCINTMSK           (N32_DMA3_BASE + N32_DMA_TCINTMSK_OFFSET)
#define N32_DMA3_TCINTCLR           (N32_DMA3_BASE + N32_DMA_TCINTCLR_OFFSET)

#define N32_DMA1_RAWBTCINTSTS       (N32_DMA1_BASE + N32_DMA_RAWBTCINTSTS_OFFSET)
#define N32_DMA1_BTCINTSTS          (N32_DMA1_BASE + N32_DMA_BTCINTSTS_OFFSET)
#define N32_DMA1_BTCINTMSK          (N32_DMA1_BASE + N32_DMA_BTCINTMSK_OFFSET)
#define N32_DMA1_BTCINTCLR          (N32_DMA1_BASE + N32_DMA_BTCINTCLR_OFFSET)

#define N32_DMA2_RAWBTCINTSTS       (N32_DMA2_BASE + N32_DMA_RAWBTCINTSTS_OFFSET)
#define N32_DMA2_BTCINTSTS          (N32_DMA2_BASE + N32_DMA_BTCINTSTS_OFFSET)
#define N32_DMA2_BTCINTMSK          (N32_DMA2_BASE + N32_DMA_BTCINTMSK_OFFSET)
#define N32_DMA2_BTCINTCLR          (N32_DMA2_BASE + N32_DMA_BTCINTCLR_OFFSET)

#define N32_DMA3_RAWBTCINTSTS       (N32_DMA3_BASE + N32_DMA_RAWBTCINTSTS_OFFSET)
#define N32_DMA3_BTCINTSTS          (N32_DMA3_BASE + N32_DMA_BTCINTSTS_OFFSET)
#define N32_DMA3_BTCINTMSK          (N32_DMA3_BASE + N32_DMA_BTCINTMSK_OFFSET)
#define N32_DMA3_BTCINTCLR          (N32_DMA3_BASE + N32_DMA_BTCINTCLR_OFFSET)

#define N32_DMA1_RAWSTCINTSTS       (N32_DMA1_BASE + N32_DMA_RAWSTCINTSTS_OFFSET)
#define N32_DMA1_STCINTSTS          (N32_DMA1_BASE + N32_DMA_STCINTSTS_OFFSET)
#define N32_DMA1_STCINTMSK          (N32_DMA1_BASE + N32_DMA_STCINTMSK_OFFSET)
#define N32_DMA1_STCINTCLR          (N32_DMA1_BASE + N32_DMA_STCINTCLR_OFFSET)

#define N32_DMA2_RAWSTCINTSTS       (N32_DMA2_BASE + N32_DMA_RAWSTCINTSTS_OFFSET)
#define N32_DMA2_STCINTSTS          (N32_DMA2_BASE + N32_DMA_STCINTSTS_OFFSET)
#define N32_DMA2_STCINTMSK          (N32_DMA2_BASE + N32_DMA_STCINTMSK_OFFSET)
#define N32_DMA2_STCINTCLR          (N32_DMA2_BASE + N32_DMA_STCINTCLR_OFFSET)

#define N32_DMA3_RAWSTCINTSTS       (N32_DMA3_BASE + N32_DMA_RAWSTCINTSTS_OFFSET)
#define N32_DMA3_STCINTSTS          (N32_DMA3_BASE + N32_DMA_STCINTSTS_OFFSET)
#define N32_DMA3_STCINTMSK          (N32_DMA3_BASE + N32_DMA_STCINTMSK_OFFSET)
#define N32_DMA3_STCINTCLR          (N32_DMA3_BASE + N32_DMA_STCINTCLR_OFFSET)

#define N32_DMA1_RAWDTCINTSTS       (N32_DMA1_BASE + N32_DMA_RAWDTCINTSTS_OFFSET)
#define N32_DMA1_DTCINTSTS          (N32_DMA1_BASE + N32_DMA_DTCINTSTS_OFFSET)
#define N32_DMA1_DTCINTMSK          (N32_DMA1_BASE + N32_DMA_DTCINTMSK_OFFSET)
#define N32_DMA1_DTCINTCLR          (N32_DMA1_BASE + N32_DMA_DTCINTCLR_OFFSET)

#define N32_DMA2_RAWDTCINTSTS       (N32_DMA2_BASE + N32_DMA_RAWDTCINTSTS_OFFSET)
#define N32_DMA2_DTCINTSTS          (N32_DMA2_BASE + N32_DMA_DTCINTSTS_OFFSET)
#define N32_DMA2_DTCINTMSK          (N32_DMA2_BASE + N32_DMA_DTCINTMSK_OFFSET)
#define N32_DMA2_DTCINTCLR          (N32_DMA2_BASE + N32_DMA_DTCINTCLR_OFFSET)

#define N32_DMA3_RAWDTCINTSTS       (N32_DMA3_BASE + N32_DMA_RAWDTCINTSTS_OFFSET)
#define N32_DMA3_DTCINTSTS          (N32_DMA3_BASE + N32_DMA_DTCINTSTS_OFFSET)
#define N32_DMA3_DTCINTMSK          (N32_DMA3_BASE + N32_DMA_DTCINTMSK_OFFSET)
#define N32_DMA3_DTCINTCLR          (N32_DMA3_BASE + N32_DMA_DTCINTCLR_OFFSET)

#define N32_DMA1_RAWERRINTSTS       (N32_DMA1_BASE + N32_DMA_RAWERRINTSTS_OFFSET)
#define N32_DMA1_ERRINTSTS          (N32_DMA1_BASE + N32_DMA_ERRINTSTS_OFFSET)
#define N32_DMA1_ERRINTMSK          (N32_DMA1_BASE + N32_DMA_ERRINTMSK_OFFSET)
#define N32_DMA1_ERRINTCLR          (N32_DMA1_BASE + N32_DMA_ERRINTCLR_OFFSET)

#define N32_DMA2_RAWERRINTSTS       (N32_DMA2_BASE + N32_DMA_RAWERRINTSTS_OFFSET)
#define N32_DMA2_ERRINTSTS          (N32_DMA2_BASE + N32_DMA_ERRINTSTS_OFFSET)
#define N32_DMA2_ERRINTMSK          (N32_DMA2_BASE + N32_DMA_ERRINTMSK_OFFSET)
#define N32_DMA2_ERRINTCLR          (N32_DMA2_BASE + N32_DMA_ERRINTCLR_OFFSET)

#define N32_DMA3_RAWERRINTSTS       (N32_DMA3_BASE + N32_DMA_RAWERRINTSTS_OFFSET)
#define N32_DMA3_ERRINTSTS          (N32_DMA3_BASE + N32_DMA_ERRINTSTS_OFFSET)
#define N32_DMA3_ERRINTMSK          (N32_DMA3_BASE + N32_DMA_ERRINTMSK_OFFSET)
#define N32_DMA3_ERRINTCLR          (N32_DMA3_BASE + N32_DMA_ERRINTCLR_OFFSET)

#define N32_DMA1_INTCBESTS          (N32_DMA1_BASE + N32_DMA_INTCBESTS_OFFSET)

#define N32_DMA2_INTCBESTS          (N32_DMA2_BASE + N32_DMA_INTCBESTS_OFFSET)

#define N32_DMA3_INTCBESTS          (N32_DMA3_BASE + N32_DMA_INTCBESTS_OFFSET)

/* ==========================================================================
 * Software Handshake Register Offset
 * ==========================================================================
 */
#define N32_DMA1_SRCSWTREQ          (N32_DMA1_BASE + N32_DMA_SRCSWTREQ_OFFSET)
#define N32_DMA1_DSTSWTREQ          (N32_DMA1_BASE + N32_DMA_DSTSWTREQ_OFFSET)
#define N32_DMA1_SRCSGTREQ          (N32_DMA1_BASE + N32_DMA_SRCSGTREQ_OFFSET)
#define N32_DMA1_DSTSGTREQ          (N32_DMA1_BASE + N32_DMA_DSTSGTREQ_OFFSET)
#define N32_DMA1_SRCLTREQ           (N32_DMA1_BASE + N32_DMA_SRCLTREQ_OFFSET)
#define N32_DMA1_DSTLTREQ           (N32_DMA1_BASE + N32_DMA_DSTLTREQ_OFFSET)

#define N32_DMA2_SRCSWTREQ          (N32_DMA2_BASE + N32_DMA_SRCSWTREQ_OFFSET)
#define N32_DMA2_DSTSWTREQ          (N32_DMA2_BASE + N32_DMA_DSTSWTREQ_OFFSET)
#define N32_DMA2_SRCSGTREQ          (N32_DMA2_BASE + N32_DMA_SRCSGTREQ_OFFSET)
#define N32_DMA2_DSTSGTREQ          (N32_DMA2_BASE + N32_DMA_DSTSGTREQ_OFFSET)
#define N32_DMA2_SRCLTREQ           (N32_DMA2_BASE + N32_DMA_SRCLTREQ_OFFSET)
#define N32_DMA2_DSTLTREQ           (N32_DMA2_BASE + N32_DMA_DSTLTREQ_OFFSET)

#define N32_DMA3_SRCSWTREQ          (N32_DMA3_BASE + N32_DMA_SRCSWTREQ_OFFSET)
#define N32_DMA3_DSTSWTREQ          (N32_DMA3_BASE + N32_DMA_DSTSWTREQ_OFFSET)
#define N32_DMA3_SRCSGTREQ          (N32_DMA3_BASE + N32_DMA_SRCSGTREQ_OFFSET)
#define N32_DMA3_DSTSGTREQ          (N32_DMA3_BASE + N32_DMA_DSTSGTREQ_OFFSET)
#define N32_DMA3_SRCLTREQ           (N32_DMA3_BASE + N32_DMA_SRCLTREQ_OFFSET)
#define N32_DMA3_DSTLTREQ           (N32_DMA3_BASE + N32_DMA_DSTLTREQ_OFFSET)

/* ==========================================================================
 * DMA Miscellaneous Register Offset
 * ==========================================================================
 */
#define N32_DMA1_CFG                (N32_DMA1_BASE + N32_DMA_CFG_OFFSET)
#define N32_DMA1_CHEN               (N32_DMA1_BASE + N32_DMA_CHEN_OFFSET)
#define N32_DMA1_ID                 (N32_DMA1_BASE + N32_DMA_ID_OFFSET)
#define N32_DMA1_TEST               (N32_DMA1_BASE + N32_DMA_TEST_OFFSET)
#define N32_DMA1_LPTIMEOUT          (N32_DMA1_BASE + N32_DMA_LPTIMEOUT_OFFSET)
#define N32_DMA1_COMPID             (N32_DMA1_BASE + N32_DMA_COMPID_OFFSET)

#define N32_DMA2_CFG                (N32_DMA2_BASE + N32_DMA_CFG_OFFSET)
#define N32_DMA2_CHEN               (N32_DMA2_BASE + N32_DMA_CHEN_OFFSET)
#define N32_DMA2_ID                 (N32_DMA2_BASE + N32_DMA_ID_OFFSET)
#define N32_DMA2_TEST               (N32_DMA2_BASE + N32_DMA_TEST_OFFSET)
#define N32_DMA2_LPTIMEOUT          (N32_DMA2_BASE + N32_DMA_LPTIMEOUT_OFFSET)
#define N32_DMA2_COMPID             (N32_DMA2_BASE + N32_DMA_COMPID_OFFSET)

#define N32_DMA3_CFG                (N32_DMA3_BASE + N32_DMA_CFG_OFFSET)
#define N32_DMA3_CHEN               (N32_DMA3_BASE + N32_DMA_CHEN_OFFSET)
#define N32_DMA3_ID                 (N32_DMA3_BASE + N32_DMA_ID_OFFSET)
#define N32_DMA3_TEST               (N32_DMA3_BASE + N32_DMA_TEST_OFFSET)
#define N32_DMA3_LPTIMEOUT          (N32_DMA3_BASE + N32_DMA_LPTIMEOUT_OFFSET)
#define N32_DMA3_COMPID             (N32_DMA3_BASE + N32_DMA_COMPID_OFFSET)

/* ==========================================================================
 * Interrupt Bit Definitions (Channel Number n = 0~7)
 * ==========================================================================
 */
#define DMA_INT_CH(n)               (1U << (n))
#define DMA_INT_CH_MASK             0xFFU

/****************************************************************************
 * 64-bit CH_CTRL / DMA_CHnCTRL Bit Definitions
 * Address：channel_base + 0x18
 * Reset Value：0x00000002_00304801
 ****************************************************************************/

/* ===== High 32 Bits (Bits 63:32) ===== */
#define DMA_CHCTRL_DONE             (1ULL << 44)            /* 44:  Done bit (status write-back) */

#define DMA_CHCTRL_BTS_SHIFT        32                      /* 43:32: Block Transfer Size (12-bit) */
#define DMA_CHCTRL_BTS_MASK         (0xFFFULL << DMA_CHCTRL_BTS_SHIFT)
#define DMA_CHCTRL_BTS(n)           ((n) << DMA_CHCTRL_BTS_SHIFT)

/* ===== Low 32 Bits (Bits 31:0) ===== */

/* LLP Control Bits */
#define DMA_CHCTRL_LLPSRCEN         (1ULL << 28)            /* 28: Source List Enable */
#define DMA_CHCTRL_LLPDSTEN         (1ULL << 27)            /* 27: Destination List Enable */

/* AHB Master Port Select */
#define DMA_CHCTRL_SMS_SHIFT        25                      /* 26:25: Source Master Port */
#define DMA_CHCTRL_SMS_MASK         (0x3ULL << DMA_CHCTRL_SMS_SHIFT)
#define DMA_CHCTRL_SMS(n)           ((n) << DMA_CHCTRL_SMS_SHIFT)

#define DMA_CHCTRL_DMS_SHIFT        23                      /* 24:23: Destination Master Port */
#define DMA_CHCTRL_DMS_MASK         (0x3ULL << DMA_CHCTRL_DMS_SHIFT)
#define DMA_CHCTRL_DMS(n)           ((n) << DMA_CHCTRL_DMS_SHIFT)

/* Transfer Type & Flow Control */
#define DMA_CHCTRL_TTFC_SHIFT       20                      /* 22:20: Transfer Type & Flow Control */
#define DMA_CHCTRL_TTFC_MASK        (0x7ULL << DMA_CHCTRL_TTFC_SHIFT)
#define DMA_CHCTRL_TTFC(n)          ((n) << DMA_CHCTRL_TTFC_SHIFT)

/* Function Enable Bits */
#define DMA_CHCTRL_DSTSCAEN         (1ULL << 18)            /* 18: Destination Scatter Enable */
#define DMA_CHCTRL_SRCGATEN         (1ULL << 17)            /* 17: Source Gather Enable */

/* Burst Length */
#define DMA_CHCTRL_SRCMSIZE_SHIFT   14                      /* 16:14: Source Burst Length (Encoded Value) */
#define DMA_CHCTRL_SRCMSIZE_MASK    (0x7ULL << DMA_CHCTRL_SRCMSIZE_SHIFT)
#define DMA_CHCTRL_SRCMSIZE(n)      ((n) << DMA_CHCTRL_SRCMSIZE_SHIFT)

#define DMA_CHCTRL_DSTMSIZE_SHIFT   11                      /* 13:11: Destination Burst Length (Encoded Value) */
#define DMA_CHCTRL_DSTMSIZE_MASK    (0x7ULL << DMA_CHCTRL_DSTMSIZE_SHIFT)
#define DMA_CHCTRL_DSTMSIZE(n)      ((n) << DMA_CHCTRL_DSTMSIZE_SHIFT)

/* Address Increment Direction */
#define DMA_CHCTRL_SINC_SHIFT        9                       /* 10:9: Source Address Control */
#define DMA_CHCTRL_SINC_MASK        (0x3ULL << DMA_CHCTRL_SINC_SHIFT)
#define DMA_CHCTRL_SINC(n)          ((n) << DMA_CHCTRL_SINC_SHIFT)

#define DMA_CHCTRL_DINC_SHIFT        7                       /* 8:7: Destination Address Control */
#define DMA_CHCTRL_DINC_MASK        (0x3ULL << DMA_CHCTRL_DINC_SHIFT)
#define DMA_CHCTRL_DINC(n)          ((n) << DMA_CHCTRL_DINC_SHIFT)

/* Transfer Width */
#define DMA_CHCTRL_STW_SHIFT         4                       /* 6:4: Source Width */
#define DMA_CHCTRL_STW_MASK         (0x7ULL << DMA_CHCTRL_STW_SHIFT)
#define DMA_CHCTRL_STW(n)           ((n) << DMA_CHCTRL_STW_SHIFT)

#define DMA_CHCTRL_DTW_SHIFT         1                       /* 3:1: Destination Width */
#define DMA_CHCTRL_DTW_MASK         (0x7ULL << DMA_CHCTRL_DTW_SHIFT)
#define DMA_CHCTRL_DTW(n)           ((n) << DMA_CHCTRL_DTW_SHIFT)

/* Global Interrupt Enable Bit */
#define DMA_CHCTRL_INTEN             (1ULL << 0)             /* 0: Interrupt Enable */

/* TTFC Enumeration */
#define DMA_TTFC_MEM_TO_MEM_DMA     (0ULL)
#define DMA_TTFC_MEM_TO_PER_DMA     (1ULL)
#define DMA_TTFC_PER_TO_MEM_DMA     (2ULL)
#define DMA_TTFC_PER_TO_PER_DMA     (3ULL)
#define DMA_TTFC_PER_TO_MEM_PER     (4ULL)
#define DMA_TTFC_PER_TO_PER_SRC     (5ULL)
#define DMA_TTFC_MEM_TO_PER_PER     (6ULL)
#define DMA_TTFC_PER_TO_PER_DST     (7ULL)

/* Address Control Enumeration */
#define DMA_ADDR_INC                (0ULL)
#define DMA_ADDR_DEC                (1ULL)
#define DMA_ADDR_NOCHANGE           (2ULL)

/* Width Enumeration */
#define DMA_WIDTH_8BITS             (0ULL)
#define DMA_WIDTH_16BITS            (1ULL)
#define DMA_WIDTH_32BITS            (2ULL)
#define DMA_WIDTH_64BITS            (3ULL)
#define DMA_WIDTH_128BITS           (4ULL)
#define DMA_WIDTH_256BITS           (5ULL)

/* Burst Length Enumeration */
#define DMA_BURST_1                 (0ULL)
#define DMA_BURST_4                 (1ULL)
#define DMA_BURST_8                 (2ULL)
#define DMA_BURST_16                (3ULL)
#define DMA_BURST_32                (4ULL)
#define DMA_BURST_64                (5ULL)
#define DMA_BURST_128               (6ULL)
#define DMA_BURST_256               (7ULL)

/* AHB Master Port Enumeration */
#define DMA_MS_1             (0ULL)
#define DMA_MS_2             (1ULL)

/****************************************************************************
 * 64-bit DMA_CHnCFG Register Bit Fields
 * Address Offset: 0x40 + 0x58*n （n=0..7）
 * Reset Value：0x0000000400000E00
 ****************************************************************************/

/* High 32 Bits (bits 63:32) ************************************************/
#define DMA_CHCFG_DONE              (1ULL << 44)            /* 44: Block Completion Flag (Write Back) */
#define DMA_CHCFG_BTS_SHIFT         32                      /* 43:32: Block Transfer Size */
#define DMA_CHCFG_BTS_MASK          (0xFFFULL << DMA_CHCFG_BTS_SHIFT)
#define DMA_CHCFG_BTS(n)            ((n) << DMA_CHCFG_BTS_SHIFT)

#define DMA_CHCFG_DSTPER_SHIFT      43                      /* 45:43: Destination Peripheral Interface Number */
#define DMA_CHCFG_DSTPER_MASK       (0x7ULL << DMA_CHCFG_DSTPER_SHIFT)
#define DMA_CHCFG_DSTPER(n)         ((n) << DMA_CHCFG_DSTPER_SHIFT)

#define DMA_CHCFG_SRCPER_SHIFT      39                      /* 41:39: Source Peripheral Interface Number */
#define DMA_CHCFG_SRCPER_MASK       (0x7ULL << DMA_CHCFG_SRCPER_SHIFT)
#define DMA_CHCFG_SRCPER(n)         ((n) << DMA_CHCFG_SRCPER_SHIFT)

#define DMA_CHCFG_PROTCTL_SHIFT     34                      /* 36:34: AHB Protection Control Register */
#define DMA_CHCFG_PROTCTL_MASK      (0x7ULL << DMA_CHCFG_PROTCTL_SHIFT)
#define DMA_CHCFG_PROTCTL(n)        ((n) << DMA_CHCFG_PROTCTL_SHIFT)

#define DMA_CHCFG_FIFOMS            (1ULL << 33)            /* 33: FIFO Mode Select */
#define DMA_CHCFG_FCM               (1ULL << 32)            /* 32: Flow Control Mode */

/* Low 32 Bits (bits 31:0) **************************************************/
#define DMA_CHCFG_ADR               (1ULL << 31)            /* 31: Auto Destination Reload */
#define DMA_CHCFG_ASR               (1ULL << 30)            /* 30: Auto Source Reload */

#define DMA_CHCFG_MAMBABL_SHIFT     20                      /* 29:20: AMBA Burst Length (Encoded Value) */
#define DMA_CHCFG_MAMBABL_MASK      (0x3FFULL << DMA_CHCFG_MAMBABL_SHIFT)
#define DMA_CHCFG_MAMBABL(n)        ((n) << DMA_CHCFG_MAMBABL_SHIFT)

#define DMA_CHCFG_SRCHSPO_SHIFT     19                      /* 19: Source Handshake Polarity */
#define DMA_CHCFG_SRCHSPO           (1ULL << 19)

#define DMA_CHCFG_DSTHSPO_SHIFT     18                      /* 18: Destination Handshake Polarity */
#define DMA_CHCFG_DSTHSPO           (1ULL << 18)

#define DMA_CHCFG_LOCKB             (1ULL << 17)            /* 17: Bus Lock Level */
#define DMA_CHCFG_LOCKCH            (1ULL << 16)            /* 16: Channel Lock Level */

#define DMA_CHCFG_LOCKBL_SHIFT      14                      /* 15:14: Bus Lock Level */
#define DMA_CHCFG_LOCKBL_MASK       (0x3ULL << DMA_CHCFG_LOCKBL_SHIFT)
#define DMA_CHCFG_LOCKBL(n)         ((n) << DMA_CHCFG_LOCKBL_SHIFT)

#define DMA_CHCFG_LOCKCHL_SHIFT     12                      /* 13:12: Channel Lock Level */
#define DMA_CHCFG_LOCKCHL_MASK      (0x3ULL << DMA_CHCFG_LOCKCHL_SHIFT)
#define DMA_CHCFG_LOCKCHL(n)        ((n) << DMA_CHCFG_LOCKCHL_SHIFT)

#define DMA_CHCFG_HSSELSRC          (1ULL << 11)            /* 11: Source Handshake Select */
#define DMA_CHCFG_HSSELDST          (1ULL << 10)            /* 10: Destination Handshake Select */

#define DMA_CHCFG_FIFOEMPTY         (1ULL << 9)             /* 9: FIFO Empty Flag (Read Only) */
#define DMA_CHCFG_CHSUSP            (1ULL << 8)             /* 8: Channel Suspend */

#define DMA_CHCFG_CHPRIOR_SHIFT     5                       /* 7:5: Channel Priority */
#define DMA_CHCFG_CHPRIOR_MASK      (0x7ULL << DMA_CHCFG_CHPRIOR_SHIFT)
#define DMA_CHCFG_CHPRIOR(n)        ((n) << DMA_CHCFG_CHPRIOR_SHIFT)

/* 4:0 Reserved */

/* ==========================================================================
 * CH_SG Register Bit Fields
 * ==========================================================================
 */
#define DMA_SG_SGC_SHIFT            20
#define DMA_SG_SGC_MASK             (0xFFFULL << DMA_SG_SGC_SHIFT)
#define DMA_SG_SGI_SHIFT            0
#define DMA_SG_SGI_MASK             (0xFFFFFULL << DMA_SG_SGI_SHIFT)

/* CH_SG Register Bit Fields */
#define DMA_SG_SGC(n)               ((n) << DMA_SG_SGC_SHIFT)
#define DMA_SG_SGI(n)               ((n) << DMA_SG_SGI_SHIFT)

/* ==========================================================================
 * CH_DS Register Bit Fields
 * ==========================================================================
 */
#define DMA_DS_DSC_SHIFT            20
#define DMA_DS_DSC_MASK             (0xFFFULL << DMA_DS_DSC_SHIFT)
#define DMA_DS_DSI_SHIFT            0
#define DMA_DS_DSI_MASK             (0xFFFFFULL << DMA_DS_DSI_SHIFT)

/* CH_DS Register Bit Fields */
#define DMA_DS_DSC(n)               ((n) << DMA_DS_DSC_SHIFT)
#define DMA_DS_DSI(n)               ((n) << DMA_DS_DSI_SHIFT)

/* ==========================================================================
 * DMA Miscellaneous Registers Bit Fields
 * ==========================================================================
 */

/* DMA Configuration Register (DMA_CFG) */
#define DMA_CFG_EN                  (1U << 0)            /* Bit 0: DMA Enable bit */

/* DMA Channel Enable Register (DMA_CHEN) */
#define DMA_CHEN_CH(n)              (1U << (n))          /* Bit n: Channel n Enable */
#define DMA_CHEN_CHWEN(n)           (1U << ((n) + 8))    /* Bit n+8: Channel n Enable bit write enable */

/* DMA ID Register (DMA_ID) */
#define DMA_ID_VALUE                (0xFFFFFFFFU)        /* Bits 31:0: Hardcoded DMA peripheral ID */

/* DMA Test Register (DMA_TEST) */
#define DMA_TEST_TMS                (1U << 0)            /* Bit 0: Test Mode Select */

/* DMA Low Power Timeout Register (DMA_LPTIMEOUT) */
#define DMA_LPTIMEOUT_VALUE         (0xFFU)              /* Bits 7:0: Timeout value of low power counter register */

/* DMA Component ID Register (DMA_COMPID) */
#define DMA_COMPID_VERSION          (0xFFFFFFFFU << 32)  /* Bits 63:32: DMA Component Version */
#define DMA_COMPID_TYPE             (0xFFFFFFFFU)        /* Bits 31:0: DMA Component Type */

#endif /* __ARCH_ARM_SRC_N32H7_HARDWARE_N32H7_DMA_H */
