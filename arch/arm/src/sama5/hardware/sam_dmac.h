/****************************************************************************************
 * arch/arm/src/sama5/hardware/sam_dmac.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_DMAC_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_DMAC_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* DMAC register offsets ****************************************************************/

/* Global Registers */

#define SAM_DMAC_GCFG_OFFSET           0x0000 /* DMAC Global Configuration Register */
#define SAM_DMAC_EN_OFFSET             0x0004 /* DMAC Enable Register */
#define SAM_DMAC_SREQ_OFFSET           0x0008 /* DMAC Software Single Request Register */
#define SAM_DMAC_CREQ_OFFSET           0x000c /* DMAC Software Chunk Transfer Request Register */
#define SAM_DMAC_LAST_OFFSET           0x0010 /* DMAC Software Last Transfer Flag Register */
                                              /* 0x014: Reserved */
#define SAM_DMAC_EBCIER_OFFSET         0x0018 /* DMAC Error Interrupt Enable */
#define SAM_DMAC_EBCIDR_OFFSET         0x001c /* DMAC Error Interrupt Disable */
#define SAM_DMAC_EBCIMR_OFFSET         0x0020 /* DMAC Error Interrupt Mask */
#define SAM_DMAC_EBCISR_OFFSET         0x0024 /* DMAC Error Status */
#define SAM_DMAC_CHER_OFFSET           0x0028 /* DMAC Channel Handler Enable Register */
#define SAM_DMAC_CHDR_OFFSET           0x002c /* DMAC Channel Handler Disable Register */
#define SAM_DMAC_CHSR_OFFSET           0x0030 /* DMAC Channel Handler Status Register */
                                              /* 0x034-0x38: Reserved */
/* DMA channel registers */

#define SAM_DMAC_CH_OFFSET(n)          (0x003c+((n)*0x0028))
#define SAM_DMAC_CH0_OFFSET            0x003c /* 0x3c-0x60: Channel 0 */
#define SAM_DMAC_CH1_OFFSET            0x0064 /* 0x64-0x88: Channel 1 */
#define SAM_DMAC_CH2_OFFSET            0x008c /* 0x8c-0xb0: Channel 2 */
#define SAM_DMAC_CH3_OFFSET            0x00b4 /* 0xb4-0xd8: Channel 3 */
#define SAM_DMAC_CH4_OFFSET            0x00dc /* 0xb4-0xd8: Channel 4 */
#define SAM_DMAC_CH5_OFFSET            0x0104 /* 0xb4-0xd8: Channel 5 */
#define SAM_DMAC_CH6_OFFSET            0x012c /* 0xb4-0xd8: Channel 6 */
#define SAM_DMAC_CH7_OFFSET            0x0154 /* 0xb4-0xd8: Channel 7 */

#define SAM_DMAC_CH_SADDR_OFFSET       0x0000 /* DMAC Channel Source Address Register */
#define SAM_DMAC_CH_DADDR_OFFSET       0x0004 /* DMAC Channel Destination Address Register */
#define SAM_DMAC_CH_DSCR_OFFSET        0x0008 /* DMAC Channel Descriptor Address Register */
#define SAM_DMAC_CH_CTRLA_OFFSET       0x000c /* DMAC Channel Control A Register */
#define SAM_DMAC_CH_CTRLB_OFFSET       0x0010 /* DMAC Channel Control B Register */
#define SAM_DMAC_CH_CFG_OFFSET         0x0014 /* DMAC Channel Configuration Register */
#define SAM_DMAC_CH_SPIP_OFFSET        0x0018 /* DMAC Channel Source PinP Configuration Register */
#define SAM_DMAC_CH_DPIP_OFFSET        0x001c /* DMAC Channel Destination PinP Configuration Register */
                                              /* 0x20-0x24: Reserved */
#define SAM_DMAC_WPMR_OFFSET           0x01e4 /* DMAC Write Protect Mode Register */
#define SAM_DMAC_WPSR_OFFSET           0x01e8 /* DMAC Write Protect Status Register */
                                              /* 0x01ec-0x1fc: Reserved */

/* DMAC0 register addresses *************************************************************/
/* DMAC0 Global Registers */

#define SAM_DMAC0_GCFG                 (SAM_DMAC0_VBASE+SAM_DMAC_GCFG_OFFSET)
#define SAM_DMAC0_EN                   (SAM_DMAC0_VBASE+SAM_DMAC_EN_OFFSET)
#define SAM_DMAC0_SREQ                 (SAM_DMAC0_VBASE+SAM_DMAC_SREQ_OFFSET)
#define SAM_DMAC0_CREQ                 (SAM_DMAC0_VBASE+SAM_DMAC_CREQ_OFFSET)
#define SAM_DMAC0_LAST                 (SAM_DMAC0_VBASE+SAM_DMAC_LAST_OFFSET)
#define SAM_DMAC0_EBCIER               (SAM_DMAC0_VBASE+SAM_DMAC_EBCIER_OFFSET)
#define SAM_DMAC0_EBCIDR               (SAM_DMAC0_VBASE+SAM_DMAC_EBCIDR_OFFSET)
#define SAM_DMAC0_EBCIMR               (SAM_DMAC0_VBASE+SAM_DMAC_EBCIMR_OFFSET)
#define SAM_DMAC0_EBCISR               (SAM_DMAC0_VBASE+SAM_DMAC_EBCISR_OFFSET)
#define SAM_DMAC0_CHER                 (SAM_DMAC0_VBASE+SAM_DMAC_CHER_OFFSET)
#define SAM_DMAC0_CHDR                 (SAM_DMAC0_VBASE+SAM_DMAC_CHDR_OFFSET)
#define SAM_DMAC0_CHSR                 (SAM_DMAC0_VBASE+SAM_DMAC_CHSR_OFFSET)

#define SAM_DMAC0_WPMR                 (SAM_DMAC0_VBASE+SAM_DMAC_WPMR_OFFSET)
#define SAM_DMAC0_WPSR                 (SAM_DMAC0_VBASE+SAM_DMAC_WPSR_OFFSET)

/* DMAC0 channel registers */

#define SAM_DMAC0_CH_BASE(n)           (SAM_DMAC0_VBASE+SAM_DMAC_CH_OFFSET(n))
#define SAM_DMAC0_CH0_BASE             (SAM_DMAC0_VBASE+SAM_DMAC_CH0_OFFSET)
#define SAM_DMAC0_CH1_BASE             (SAM_DMAC0_VBASE+SAM_DMAC_CH1_OFFSET)
#define SAM_DMAC0_CH2_BASE             (SAM_DMAC0_VBASE+SAM_DMAC_CH2_OFFSET)
#define SAM_DMAC0_CH3_BASE             (SAM_DMAC0_VBASE+SAM_DMAC_CH3_OFFSET)
#define SAM_DMAC0_CH4_BASE             (SAM_DMAC0_VBASE+SAM_DMAC_CH4_OFFSET)
#define SAM_DMAC0_CH5_BASE             (SAM_DMAC0_VBASE+SAM_DMAC_CH5_OFFSET)
#define SAM_DMAC0_CH6_BASE             (SAM_DMAC0_VBASE+SAM_DMAC_CH6_OFFSET)
#define SAM_DMAC0_CH7_BASE             (SAM_DMAC0_VBASE+SAM_DMAC_CH7_OFFSET)

#define SAM_DMAC0_CH_SADDR(n)          (SAM_DMAC0_CH_BASE(n)+SAM_DMAC_CH_SADDR_OFFSET)
#define SAM_DMAC0_CH_DADDR(n)          (SAM_DMAC0_CH_BASE(n)+SAM_DMAC_CH_DADDR_OFFSET)
#define SAM_DMAC0_CH_DSCR(n)           (SAM_DMAC0_CH_BASE(n)+SAM_DMAC_CH_DSCR_OFFSET)
#define SAM_DMAC0_CH_CTRLA(n)          (SAM_DMAC0_CH_BASE(n)+SAM_DMAC_CH_CTRLA_OFFSET)
#define SAM_DMAC0_CH_CTRLB(n)          (SAM_DMAC0_CH_BASE(n)+SAM_DMAC_CH_CTRLB_OFFSET)
#define SAM_DMAC0_CH_CFG(n)            (SAM_DMAC0_CH_BASE(n)+SAM_DMAC_CH_CFG_OFFSET)
#define SAM_DMAC0_CH_SPIP(n)           (SAM_DMAC0_CH_BASE(n)+SAM_DMAC_CH_SPIP_OFFSET)
#define SAM_DMAC0_CH_DPIP(n)           (SAM_DMAC0_CH_BASE(n)+SAM_DMAC_CH_DPIP_OFFSET)

#define SAM_DMAC0_CH0_SADDR            (SAM_DMAC0_CH0_BASE+SAM_DMAC_CH_SADDR_OFFSET)
#define SAM_DMAC0_CH0_DADDR            (SAM_DMAC0_CH0_BASE+SAM_DMAC_CH_DADDR_OFFSET)
#define SAM_DMAC0_CH0_DSCR             (SAM_DMAC0_CH0_BASE+SAM_DMAC_CH_DSCR_OFFSET)
#define SAM_DMAC0_CH0_CTRLA            (SAM_DMAC0_CH0_BASE+SAM_DMAC_CH_CTRLA_OFFSET)
#define SAM_DMAC0_CH0_CTRLB            (SAM_DMAC0_CH0_BASE+SAM_DMAC_CH_CTRLB_OFFSET)
#define SAM_DMAC0_CH0_CFG              (SAM_DMAC0_CH0_BASE+SAM_DMAC_CH_CFG_OFFSET)
#define SAM_DMAC0_CH0_SPIP             (SAM_DMAC0_CH0_BASE+SAM_DMAC_CH_SPIP_OFFSET)
#define SAM_DMAC0_CH0_DPIP             (SAM_DMAC0_CH0_BASE+SAM_DMAC_CH_DPIP_OFFSET)

#define SAM_DMAC0_CH1_SADDR            (SAM_DMAC0_CH1_BASE+SAM_DMAC_CH_SADDR_OFFSET)
#define SAM_DMAC0_CH1_DADDR            (SAM_DMAC0_CH1_BASE+SAM_DMAC_CH_DADDR_OFFSET)
#define SAM_DMAC0_CH1_DSCR             (SAM_DMAC0_CH1_BASE+SAM_DMAC_CH_DSCR_OFFSET)
#define SAM_DMAC0_CH1_CTRLA            (SAM_DMAC0_CH1_BASE+SAM_DMAC_CH_CTRLA_OFFSET)
#define SAM_DMAC0_CH1_CTRLB            (SAM_DMAC0_CH1_BASE+SAM_DMAC_CH_CTRLB_OFFSET)
#define SAM_DMAC0_CH1_CFG              (SAM_DMAC0_CH1_BASE+SAM_DMAC_CH_CFG_OFFSET)
#define SAM_DMAC0_CH1_SPIP             (SAM_DMAC0_CH1_BASE+SAM_DMAC_CH_SPIP_OFFSET)
#define SAM_DMAC0_CH1_DPIP             (SAM_DMAC0_CH1_BASE+SAM_DMAC_CH_DPIP_OFFSET)

#define SAM_DMAC0_CH2_SADDR            (SAM_DMAC0_CH2_BASE+SAM_DMAC_CH_SADDR_OFFSET)
#define SAM_DMAC0_CH2_DADDR            (SAM_DMAC0_CH2_BASE+SAM_DMAC_CH_DADDR_OFFSET)
#define SAM_DMAC0_CH2_DSCR             (SAM_DMAC0_CH2_BASE+SAM_DMAC_CH_DSCR_OFFSET)
#define SAM_DMAC0_CH2_CTRLA            (SAM_DMAC0_CH2_BASE+SAM_DMAC_CH_CTRLA_OFFSET)
#define SAM_DMAC0_CH2_CTRLB            (SAM_DMAC0_CH2_BASE+SAM_DMAC_CH_CTRLB_OFFSET)
#define SAM_DMAC0_CH2_CFG              (SAM_DMAC0_CH2_BASE+SAM_DMAC_CH_CFG_OFFSET)
#define SAM_DMAC0_CH2_SPIP             (SAM_DMAC0_CH2_BASE+SAM_DMAC_CH_SPIP_OFFSET)
#define SAM_DMAC0_CH2_DPIP             (SAM_DMAC0_CH2_BASE+SAM_DMAC_CH_DPIP_OFFSET)

#define SAM_DMAC0_CH3_SADDR            (SAM_DMAC0_CH3_BASE+SAM_DMAC_CH_SADDR_OFFSET)
#define SAM_DMAC0_CH3_DADDR            (SAM_DMAC0_CH3_BASE+SAM_DMAC_CH_DADDR_OFFSET)
#define SAM_DMAC0_CH3_DSCR             (SAM_DMAC0_CH3_BASE+SAM_DMAC_CH_DSCR_OFFSET)
#define SAM_DMAC0_CH3_CTRLA            (SAM_DMAC0_CH3_BASE+SAM_DMAC_CH_CTRLA_OFFSET)
#define SAM_DMAC0_CH3_CTRLB            (SAM_DMAC0_CH3_BASE+SAM_DMAC_CH_CTRLB_OFFSET)
#define SAM_DMAC0_CH3_CFG              (SAM_DMAC0_CH3_BASE+SAM_DMAC_CH_CFG_OFFSET)
#define SAM_DMAC0_CH3_SPIP             (SAM_DMAC0_CH3_BASE+SAM_DMAC_CH_SPIP_OFFSET)
#define SAM_DMAC0_CH3_DPIP             (SAM_DMAC0_CH3_BASE+SAM_DMAC_CH_DPIP_OFFSET)

#define SAM_DMAC0_CH4_SADDR            (SAM_DMAC0_CH4_BASE+SAM_DMAC_CH_SADDR_OFFSET)
#define SAM_DMAC0_CH4_DADDR            (SAM_DMAC0_CH4_BASE+SAM_DMAC_CH_DADDR_OFFSET)
#define SAM_DMAC0_CH4_DSCR             (SAM_DMAC0_CH4_BASE+SAM_DMAC_CH_DSCR_OFFSET)
#define SAM_DMAC0_CH4_CTRLA            (SAM_DMAC0_CH4_BASE+SAM_DMAC_CH_CTRLA_OFFSET)
#define SAM_DMAC0_CH4_CTRLB            (SAM_DMAC0_CH4_BASE+SAM_DMAC_CH_CTRLB_OFFSET)
#define SAM_DMAC0_CH4_CFG              (SAM_DMAC0_CH4_BASE+SAM_DMAC_CH_CFG_OFFSET)
#define SAM_DMAC0_CH4_SPIP             (SAM_DMAC0_CH4_BASE+SAM_DMAC_CH_SPIP_OFFSET)
#define SAM_DMAC0_CH4_DPIP             (SAM_DMAC0_CH4_BASE+SAM_DMAC_CH_DPIP_OFFSET)

#define SAM_DMAC0_CH5_SADDR            (SAM_DMAC0_CH5_BASE+SAM_DMAC_CH_SADDR_OFFSET)
#define SAM_DMAC0_CH5_DADDR            (SAM_DMAC0_CH5_BASE+SAM_DMAC_CH_DADDR_OFFSET)
#define SAM_DMAC0_CH5_DSCR             (SAM_DMAC0_CH5_BASE+SAM_DMAC_CH_DSCR_OFFSET)
#define SAM_DMAC0_CH5_CTRLA            (SAM_DMAC0_CH5_BASE+SAM_DMAC_CH_CTRLA_OFFSET)
#define SAM_DMAC0_CH5_CTRLB            (SAM_DMAC0_CH5_BASE+SAM_DMAC_CH_CTRLB_OFFSET)
#define SAM_DMAC0_CH5_CFG              (SAM_DMAC0_CH5_BASE+SAM_DMAC_CH_CFG_OFFSET)
#define SAM_DMAC0_CH5_SPIP             (SAM_DMAC0_CH5_BASE+SAM_DMAC_CH_SPIP_OFFSET)
#define SAM_DMAC0_CH5_DPIP             (SAM_DMAC0_CH5_BASE+SAM_DMAC_CH_DPIP_OFFSET)

#define SAM_DMAC0_CH6_SADDR            (SAM_DMAC0_CH6_BASE+SAM_DMAC_CH_SADDR_OFFSET)
#define SAM_DMAC0_CH6_DADDR            (SAM_DMAC0_CH6_BASE+SAM_DMAC_CH_DADDR_OFFSET)
#define SAM_DMAC0_CH6_DSCR             (SAM_DMAC0_CH6_BASE+SAM_DMAC_CH_DSCR_OFFSET)
#define SAM_DMAC0_CH6_CTRLA            (SAM_DMAC0_CH6_BASE+SAM_DMAC_CH_CTRLA_OFFSET)
#define SAM_DMAC0_CH6_CTRLB            (SAM_DMAC0_CH6_BASE+SAM_DMAC_CH_CTRLB_OFFSET)
#define SAM_DMAC0_CH6_CFG              (SAM_DMAC0_CH6_BASE+SAM_DMAC_CH_CFG_OFFSET)
#define SAM_DMAC0_CH6_SPIP             (SAM_DMAC0_CH6_BASE+SAM_DMAC_CH_SPIP_OFFSET)
#define SAM_DMAC0_CH6_DPIP             (SAM_DMAC0_CH6_BASE+SAM_DMAC_CH_DPIP_OFFSET)

#define SAM_DMAC0_CH7_SADDR            (SAM_DMAC0_CH7_BASE+SAM_DMAC_CH_SADDR_OFFSET)
#define SAM_DMAC0_CH7_DADDR            (SAM_DMAC0_CH7_BASE+SAM_DMAC_CH_DADDR_OFFSET)
#define SAM_DMAC0_CH7_DSCR             (SAM_DMAC0_CH7_BASE+SAM_DMAC_CH_DSCR_OFFSET)
#define SAM_DMAC0_CH7_CTRLA            (SAM_DMAC0_CH7_BASE+SAM_DMAC_CH_CTRLA_OFFSET)
#define SAM_DMAC0_CH7_CTRLB            (SAM_DMAC0_CH7_BASE+SAM_DMAC_CH_CTRLB_OFFSET)
#define SAM_DMAC0_CH7_CFG              (SAM_DMAC0_CH7_BASE+SAM_DMAC_CH_CFG_OFFSET)
#define SAM_DMAC0_CH7_SPIP             (SAM_DMAC0_CH7_BASE+SAM_DMAC_CH_SPIP_OFFSET)
#define SAM_DMAC0_CH7_DPIP             (SAM_DMAC0_CH7_BASE+SAM_DMAC_CH_DPIP_OFFSET)

/* DMAC1 register addresses *************************************************************/
/* DMAC1 Global Registers */

#define SAM_DMAC1_GCFG                 (SAM_DMAC1_VBASE+SAM_DMAC_GCFG_OFFSET)
#define SAM_DMAC1_EN                   (SAM_DMAC1_VBASE+SAM_DMAC_EN_OFFSET)
#define SAM_DMAC1_SREQ                 (SAM_DMAC1_VBASE+SAM_DMAC_SREQ_OFFSET)
#define SAM_DMAC1_CREQ                 (SAM_DMAC1_VBASE+SAM_DMAC_CREQ_OFFSET)
#define SAM_DMAC1_LAST                 (SAM_DMAC1_VBASE+SAM_DMAC_LAST_OFFSET)
#define SAM_DMAC1_EBCIER               (SAM_DMAC1_VBASE+SAM_DMAC_EBCIER_OFFSET)
#define SAM_DMAC1_EBCIDR               (SAM_DMAC1_VBASE+SAM_DMAC_EBCIDR_OFFSET)
#define SAM_DMAC1_EBCIMR               (SAM_DMAC1_VBASE+SAM_DMAC_EBCIMR_OFFSET)
#define SAM_DMAC1_EBCISR               (SAM_DMAC1_VBASE+SAM_DMAC_EBCISR_OFFSET)
#define SAM_DMAC1_CHER                 (SAM_DMAC1_VBASE+SAM_DMAC_CHER_OFFSET)
#define SAM_DMAC1_CHDR                 (SAM_DMAC1_VBASE+SAM_DMAC_CHDR_OFFSET)
#define SAM_DMAC1_CHSR                 (SAM_DMAC1_VBASE+SAM_DMAC_CHSR_OFFSET)

#define SAM_DMAC1_WPMR                 (SAM_DMAC1_VBASE+SAM_DMAC_WPMR_OFFSET)
#define SAM_DMAC1_WPSR                 (SAM_DMAC1_VBASE+SAM_DMAC_WPSR_OFFSET)

/* DMAC1 channel registers */

#define SAM_DMAC1_CH_BASE(n)           (SAM_DMAC1_VBASE+SAM_DMAC_CH_OFFSET(n))
#define SAM_DMAC1_CH0_BASE             (SAM_DMAC1_VBASE+SAM_DMAC_CH0_OFFSET)
#define SAM_DMAC1_CH1_BASE             (SAM_DMAC1_VBASE+SAM_DMAC_CH1_OFFSET)
#define SAM_DMAC1_CH2_BASE             (SAM_DMAC1_VBASE+SAM_DMAC_CH2_OFFSET)
#define SAM_DMAC1_CH3_BASE             (SAM_DMAC1_VBASE+SAM_DMAC_CH3_OFFSET)
#define SAM_DMAC1_CH4_BASE             (SAM_DMAC1_VBASE+SAM_DMAC_CH4_OFFSET)
#define SAM_DMAC1_CH5_BASE             (SAM_DMAC1_VBASE+SAM_DMAC_CH5_OFFSET)
#define SAM_DMAC1_CH6_BASE             (SAM_DMAC1_VBASE+SAM_DMAC_CH6_OFFSET)
#define SAM_DMAC1_CH7_BASE             (SAM_DMAC1_VBASE+SAM_DMAC_CH7_OFFSET)

#define SAM_DMAC1_CH_SADDR(n)          (SAM_DMAC1_CH_BASE(n)+SAM_DMAC_CH_SADDR_OFFSET)
#define SAM_DMAC1_CH_DADDR(n)          (SAM_DMAC1_CH_BASE(n)+SAM_DMAC_CH_DADDR_OFFSET)
#define SAM_DMAC1_CH_DSCR(n)           (SAM_DMAC1_CH_BASE(n)+SAM_DMAC_CH_DSCR_OFFSET)
#define SAM_DMAC1_CH_CTRLA(n)          (SAM_DMAC1_CH_BASE(n)+SAM_DMAC_CH_CTRLA_OFFSET)
#define SAM_DMAC1_CH_CTRLB(n)          (SAM_DMAC1_CH_BASE(n)+SAM_DMAC_CH_CTRLB_OFFSET)
#define SAM_DMAC1_CH_CFG(n)            (SAM_DMAC1_CH_BASE(n)+SAM_DMAC_CH_CFG_OFFSET)
#define SAM_DMAC1_CH_SPIP(n)           (SAM_DMAC1_CH_BASE(n)+SAM_DMAC_CH_SPIP_OFFSET)
#define SAM_DMAC1_CH_DPIP(n)           (SAM_DMAC1_CH_BASE(n)+SAM_DMAC_CH_DPIP_OFFSET)

#define SAM_DMAC1_CH0_SADDR            (SAM_DMAC1_CH0_BASE+SAM_DMAC_CH_SADDR_OFFSET)
#define SAM_DMAC1_CH0_DADDR            (SAM_DMAC1_CH0_BASE+SAM_DMAC_CH_DADDR_OFFSET)
#define SAM_DMAC1_CH0_DSCR             (SAM_DMAC1_CH0_BASE+SAM_DMAC_CH_DSCR_OFFSET)
#define SAM_DMAC1_CH0_CTRLA            (SAM_DMAC1_CH0_BASE+SAM_DMAC_CH_CTRLA_OFFSET)
#define SAM_DMAC1_CH0_CTRLB            (SAM_DMAC1_CH0_BASE+SAM_DMAC_CH_CTRLB_OFFSET)
#define SAM_DMAC1_CH0_CFG              (SAM_DMAC1_CH0_BASE+SAM_DMAC_CH_CFG_OFFSET)
#define SAM_DMAC1_CH0_SPIP             (SAM_DMAC1_CH0_BASE+SAM_DMAC_CH_SPIP_OFFSET)
#define SAM_DMAC1_CH0_DPIP             (SAM_DMAC1_CH0_BASE+SAM_DMAC_CH_DPIP_OFFSET)

#define SAM_DMAC1_CH1_SADDR            (SAM_DMAC1_CH1_BASE+SAM_DMAC_CH_SADDR_OFFSET)
#define SAM_DMAC1_CH1_DADDR            (SAM_DMAC1_CH1_BASE+SAM_DMAC_CH_DADDR_OFFSET)
#define SAM_DMAC1_CH1_DSCR             (SAM_DMAC1_CH1_BASE+SAM_DMAC_CH_DSCR_OFFSET)
#define SAM_DMAC1_CH1_CTRLA            (SAM_DMAC1_CH1_BASE+SAM_DMAC_CH_CTRLA_OFFSET)
#define SAM_DMAC1_CH1_CTRLB            (SAM_DMAC1_CH1_BASE+SAM_DMAC_CH_CTRLB_OFFSET)
#define SAM_DMAC1_CH1_CFG              (SAM_DMAC1_CH1_BASE+SAM_DMAC_CH_CFG_OFFSET)
#define SAM_DMAC1_CH1_SPIP             (SAM_DMAC1_CH1_BASE+SAM_DMAC_CH_SPIP_OFFSET)
#define SAM_DMAC1_CH1_DPIP             (SAM_DMAC1_CH1_BASE+SAM_DMAC_CH_DPIP_OFFSET)

#define SAM_DMAC1_CH2_SADDR            (SAM_DMAC1_CH2_BASE+SAM_DMAC_CH_SADDR_OFFSET)
#define SAM_DMAC1_CH2_DADDR            (SAM_DMAC1_CH2_BASE+SAM_DMAC_CH_DADDR_OFFSET)
#define SAM_DMAC1_CH2_DSCR             (SAM_DMAC1_CH2_BASE+SAM_DMAC_CH_DSCR_OFFSET)
#define SAM_DMAC1_CH2_CTRLA            (SAM_DMAC1_CH2_BASE+SAM_DMAC_CH_CTRLA_OFFSET)
#define SAM_DMAC1_CH2_CTRLB            (SAM_DMAC1_CH2_BASE+SAM_DMAC_CH_CTRLB_OFFSET)
#define SAM_DMAC1_CH2_CFG              (SAM_DMAC1_CH2_BASE+SAM_DMAC_CH_CFG_OFFSET)
#define SAM_DMAC1_CH2_SPIP             (SAM_DMAC1_CH2_BASE+SAM_DMAC_CH_SPIP_OFFSET)
#define SAM_DMAC1_CH2_DPIP             (SAM_DMAC1_CH2_BASE+SAM_DMAC_CH_DPIP_OFFSET)

#define SAM_DMAC1_CH3_SADDR            (SAM_DMAC1_CH3_BASE+SAM_DMAC_CH_SADDR_OFFSET)
#define SAM_DMAC1_CH3_DADDR            (SAM_DMAC1_CH3_BASE+SAM_DMAC_CH_DADDR_OFFSET)
#define SAM_DMAC1_CH3_DSCR             (SAM_DMAC1_CH3_BASE+SAM_DMAC_CH_DSCR_OFFSET)
#define SAM_DMAC1_CH3_CTRLA            (SAM_DMAC1_CH3_BASE+SAM_DMAC_CH_CTRLA_OFFSET)
#define SAM_DMAC1_CH3_CTRLB            (SAM_DMAC1_CH3_BASE+SAM_DMAC_CH_CTRLB_OFFSET)
#define SAM_DMAC1_CH3_CFG              (SAM_DMAC1_CH3_BASE+SAM_DMAC_CH_CFG_OFFSET)
#define SAM_DMAC1_CH3_SPIP             (SAM_DMAC1_CH3_BASE+SAM_DMAC_CH_SPIP_OFFSET)
#define SAM_DMAC1_CH3_DPIP             (SAM_DMAC1_CH3_BASE+SAM_DMAC_CH_DPIP_OFFSET)

#define SAM_DMAC1_CH4_SADDR            (SAM_DMAC1_CH4_BASE+SAM_DMAC_CH_SADDR_OFFSET)
#define SAM_DMAC1_CH4_DADDR            (SAM_DMAC1_CH4_BASE+SAM_DMAC_CH_DADDR_OFFSET)
#define SAM_DMAC1_CH4_DSCR             (SAM_DMAC1_CH4_BASE+SAM_DMAC_CH_DSCR_OFFSET)
#define SAM_DMAC1_CH4_CTRLA            (SAM_DMAC1_CH4_BASE+SAM_DMAC_CH_CTRLA_OFFSET)
#define SAM_DMAC1_CH4_CTRLB            (SAM_DMAC1_CH4_BASE+SAM_DMAC_CH_CTRLB_OFFSET)
#define SAM_DMAC1_CH4_CFG              (SAM_DMAC1_CH4_BASE+SAM_DMAC_CH_CFG_OFFSET)
#define SAM_DMAC1_CH4_SPIP             (SAM_DMAC1_CH4_BASE+SAM_DMAC_CH_SPIP_OFFSET)
#define SAM_DMAC1_CH4_DPIP             (SAM_DMAC1_CH4_BASE+SAM_DMAC_CH_DPIP_OFFSET)

#define SAM_DMAC1_CH5_SADDR            (SAM_DMAC1_CH5_BASE+SAM_DMAC_CH_SADDR_OFFSET)
#define SAM_DMAC1_CH5_DADDR            (SAM_DMAC1_CH5_BASE+SAM_DMAC_CH_DADDR_OFFSET)
#define SAM_DMAC1_CH5_DSCR             (SAM_DMAC1_CH5_BASE+SAM_DMAC_CH_DSCR_OFFSET)
#define SAM_DMAC1_CH5_CTRLA            (SAM_DMAC1_CH5_BASE+SAM_DMAC_CH_CTRLA_OFFSET)
#define SAM_DMAC1_CH5_CTRLB            (SAM_DMAC1_CH5_BASE+SAM_DMAC_CH_CTRLB_OFFSET)
#define SAM_DMAC1_CH5_CFG              (SAM_DMAC1_CH5_BASE+SAM_DMAC_CH_CFG_OFFSET)
#define SAM_DMAC1_CH5_SPIP             (SAM_DMAC1_CH5_BASE+SAM_DMAC_CH_SPIP_OFFSET)
#define SAM_DMAC1_CH5_DPIP             (SAM_DMAC1_CH5_BASE+SAM_DMAC_CH_DPIP_OFFSET)

#define SAM_DMAC1_CH6_SADDR            (SAM_DMAC1_CH6_BASE+SAM_DMAC_CH_SADDR_OFFSET)
#define SAM_DMAC1_CH6_DADDR            (SAM_DMAC1_CH6_BASE+SAM_DMAC_CH_DADDR_OFFSET)
#define SAM_DMAC1_CH6_DSCR             (SAM_DMAC1_CH6_BASE+SAM_DMAC_CH_DSCR_OFFSET)
#define SAM_DMAC1_CH6_CTRLA            (SAM_DMAC1_CH6_BASE+SAM_DMAC_CH_CTRLA_OFFSET)
#define SAM_DMAC1_CH6_CTRLB            (SAM_DMAC1_CH6_BASE+SAM_DMAC_CH_CTRLB_OFFSET)
#define SAM_DMAC1_CH6_CFG              (SAM_DMAC1_CH6_BASE+SAM_DMAC_CH_CFG_OFFSET)
#define SAM_DMAC1_CH6_SPIP             (SAM_DMAC1_CH6_BASE+SAM_DMAC_CH_SPIP_OFFSET)
#define SAM_DMAC1_CH6_DPIP             (SAM_DMAC1_CH6_BASE+SAM_DMAC_CH_DPIP_OFFSET)

#define SAM_DMAC1_CH7_SADDR            (SAM_DMAC1_CH7_BASE+SAM_DMAC_CH_SADDR_OFFSET)
#define SAM_DMAC1_CH7_DADDR            (SAM_DMAC1_CH7_BASE+SAM_DMAC_CH_DADDR_OFFSET)
#define SAM_DMAC1_CH7_DSCR             (SAM_DMAC1_CH7_BASE+SAM_DMAC_CH_DSCR_OFFSET)
#define SAM_DMAC1_CH7_CTRLA            (SAM_DMAC1_CH7_BASE+SAM_DMAC_CH_CTRLA_OFFSET)
#define SAM_DMAC1_CH7_CTRLB            (SAM_DMAC1_CH7_BASE+SAM_DMAC_CH_CTRLB_OFFSET)
#define SAM_DMAC1_CH7_CFG              (SAM_DMAC1_CH7_BASE+SAM_DMAC_CH_CFG_OFFSET)
#define SAM_DMAC1_CH7_SPIP             (SAM_DMAC1_CH7_BASE+SAM_DMAC_CH_SPIP_OFFSET)
#define SAM_DMAC1_CH7_DPIP             (SAM_DMAC1_CH7_BASE+SAM_DMAC_CH_DPIP_OFFSET)

/* DMAC register bit definitions ********************************************************/

/* Global Registers */

/* DMAC Global Configuration Register */

#define DMAC_GCFG_ARB_CFG              (1 << 4)  /* Bit 4:  Arbiter Configuration */
#  define DMAC_GCFG_ARB_FIXED          (0)       /* Bit 4=0: Fixed priority arbiter */
#  define DMAC_GCFG_ARB_ROUNDROBIN     (1 << 4)  /* Bit 4=1: Round robin arbiter */
#define DMAC_DICEN                     (1 << 8)  /* Bit 8:  Descriptor Integrity Check */

/* DMAC Enable Register */

#define DMAC_EN_ENABLE                 (1 << 0)  /* Bit 0:  DMA controller enable */

/* DMAC Software Single Request Register */

#define DMAC_SREQ_SHIFT(n)             ((n)<<1)
#define DMAC_SREQ_MASK(n)              (3 << DMAC_SREQ_SHIFT(n))
#define DMAC_SREQ0_SHIFT               (0)      /* Bits 0-1: Channel 0 */
#define DMAC_SREQ0_MASK                (3 << DMAC_SREQ0_SHIFT)
#define DMAC_SREQ1_SHIFT               (2)      /* Bits 2-3: Channel 1 */
#define DMAC_SREQ1_MASK                (3 << DMAC_SREQ1_SHIFT)
#define DMAC_SREQ2_SHIFT               (4)      /* Bits 4-5: Channel 2 */
#define DMAC_SREQ2_MASK                (3 << DMAC_SREQ2_SHIFT)
#define DMAC_SREQ3_SHIFT               (6)      /* Bits 6-7: Channel 3 */
#define DMAC_SREQ3_MASK                (3 << DMAC_SREQ3_SHIFT)
#define DMAC_SREQ4_SHIFT               (8)      /* Bits 8-9: Channel 4 */
#define DMAC_SREQ4_MASK                (3 << DMAC_SREQ4_SHIFT)
#define DMAC_SREQ5_SHIFT               (10)     /* Bits 10-11: Channel 5 */
#define DMAC_SREQ5_MASK                (3 << DMAC_SREQ5_SHIFT)
#define DMAC_SREQ6_SHIFT               (12)     /* Bits 12-13: Channel 6 */
#define DMAC_SREQ6_MASK                (3 << DMAC_SREQ6_SHIFT)
#define DMAC_SREQ7_SHIFT               (14)     /* Bits 14-15: Channel 7 */
#define DMAC_SREQ7_MASK                (3 << DMAC_SREQ7_SHIFT)

#define DMAC_SREQ_SSREQ_SHIFT          (0)      /* Even bits: Request a source single transfer */
#  define DMAC_SREQ_SSREQ(n)           (1 << (DMAC_SREQ_SSREQ_SHIFT+DMAC_SREQ_SHIFT(n)))
#  define DMAC_SREQ_SSREQ0             (1 << (DMAC_SREQ_SSREQ_SHIFT+DMAC_SREQ0_SHIFT))
#  define DMAC_SREQ_SSREQ1             (1 << (DMAC_SREQ_SSREQ_SHIFT+DMAC_SREQ1_SHIFT))
#  define DMAC_SREQ_SSREQ2             (1 << (DMAC_SREQ_SSREQ_SHIFT+DMAC_SREQ2_SHIFT))
#  define DMAC_SREQ_SSREQ3             (1 << (DMAC_SREQ_SSREQ_SHIFT+DMAC_SREQ3_SHIFT))
#  define DMAC_SREQ_SSREQ4             (1 << (DMAC_SREQ_SSREQ_SHIFT+DMAC_SREQ4_SHIFT))
#  define DMAC_SREQ_SSREQ5             (1 << (DMAC_SREQ_SSREQ_SHIFT+DMAC_SREQ5_SHIFT))
#  define DMAC_SREQ_SSREQ6             (1 << (DMAC_SREQ_SSREQ_SHIFT+DMAC_SREQ6_SHIFT))
#  define DMAC_SREQ_SSREQ7             (1 << (DMAC_SREQ_SSREQ_SHIFT+DMAC_SREQ7_SHIFT))
#define DMAC_SREQ_DSREQ_SHIFT          (1)      /* Odd bits: Request a destination single transfer */
#  define DMAC_SREQ_DSREQ(n)           (1 << (DMAC_SREQ_DSREQ_SHIFT+DMAC_SREQ_SHIFT(n))))
#  define DMAC_SREQ_DSREQ0             (1 << (DMAC_SREQ_DSREQ_SHIFT+DMAC_SREQ0_SHIFT))
#  define DMAC_SREQ_DSREQ1             (1 << (DMAC_SREQ_DSREQ_SHIFT+DMAC_SREQ1_SHIFT))
#  define DMAC_SREQ_DSREQ2             (1 << (DMAC_SREQ_DSREQ_SHIFT+DMAC_SREQ2_SHIFT))
#  define DMAC_SREQ_DSREQ3             (1 << (DMAC_SREQ_DSREQ_SHIFT+DMAC_SREQ3_SHIFT))
#  define DMAC_SREQ_DSREQ4             (1 << (DMAC_SREQ_DSREQ_SHIFT+DMAC_SREQ4_SHIFT))
#  define DMAC_SREQ_DSREQ5             (1 << (DMAC_SREQ_DSREQ_SHIFT+DMAC_SREQ5_SHIFT))
#  define DMAC_SREQ_DSREQ6             (1 << (DMAC_SREQ_DSREQ_SHIFT+DMAC_SREQ6_SHIFT))
#  define DMAC_SREQ_DSREQ7             (1 << (DMAC_SREQ_DSREQ_SHIFT+DMAC_SREQ7_SHIFT))

/* DMAC Software Chunk Transfer Request Register */

#define DMAC_CREQ_SHIFT(n)             ((n)<<1)
#define DMAC_CREQ_MASK(n)              (3 << DMAC_CREQ_SHIFT(n))
#define DMAC_CREQ0_SHIFT               (0)      /* Bits 0-1:  Channel 0 */
#define DMAC_CREQ0_MASK                (3 << DMAC_CREQ0_SHIFT)
#define DMAC_CREQ1_SHIFT               (2)      /* Bits 2-3:  Channel 1 */
#define DMAC_CREQ1_MASK                (3 << DMAC_CREQ1_SHIFT)
#define DMAC_CREQ2_SHIFT               (4)      /* Bits 4-5:  Channel 2 */
#define DMAC_CREQ2_MASK                (3 << DMAC_CREQ2_SHIFT)
#define DMAC_CREQ3_SHIFT               (6)      /* Bits 6-7:  Channel 3 */
#define DMAC_CREQ3_MASK                (3 << DMAC_CREQ3_SHIFT)
#define DMAC_CREQ4_SHIFT               (8)      /* Bits 8-9: Channel 4 */
#define DMAC_CREQ4_MASK                (3 << DMAC_CREQ4_SHIFT)
#define DMAC_CREQ5_SHIFT               (10)     /* Bits 10-11: Channel 5 */
#define DMAC_CREQ5_MASK                (3 << DMAC_CREQ5_SHIFT)
#define DMAC_CREQ6_SHIFT               (12)     /* Bits 12-13: Channel 6 */
#define DMAC_CREQ6_MASK                (3 << DMAC_CREQ6_SHIFT)
#define DMAC_CREQ7_SHIFT               (14)     /* Bits 14-15: Channel 7 */
#define DMAC_CREQ7_MASK                (3 << DMAC_CREQ7_SHIFT)

#define DMAC_CREQ_SCREQ_SHIFT          (0)      /* Even bits: Request a source chunk transfer */
#  define DMAC_CREQ_SCREQ(n)           (1 << (DMAC_CREQ_SCREQ_SHIFT+DMAC_CREQ_SHIFT(n)))
#  define DMAC_CREQ_SCREQ0             (1 << (DMAC_CREQ_SCREQ_SHIFT+DMAC_CREQ0_SHIFT))
#  define DMAC_CREQ_SCREQ1             (1 << (DMAC_CREQ_SCREQ_SHIFT+DMAC_CREQ1_SHIFT))
#  define DMAC_CREQ_SCREQ2             (1 << (DMAC_CREQ_SCREQ_SHIFT+DMAC_CREQ2_SHIFT))
#  define DMAC_CREQ_SCREQ3             (1 << (DMAC_CREQ_SCREQ_SHIFT+DMAC_CREQ3_SHIFT))
#  define DMAC_CREQ_SCREQ4             (1 << (DMAC_CREQ_SCREQ_SHIFT+DMAC_CREQ4_SHIFT))
#  define DMAC_CREQ_SCREQ5             (1 << (DMAC_CREQ_SCREQ_SHIFT+DMAC_CREQ5_SHIFT))
#  define DMAC_CREQ_SCREQ6             (1 << (DMAC_CREQ_SCREQ_SHIFT+DMAC_CREQ6_SHIFT))
#  define DMAC_CREQ_SCREQ7             (1 << (DMAC_CREQ_SCREQ_SHIFT+DMAC_CREQ7_SHIFT))
#define DMAC_CREQ_DCREQ_SHIFT          (1)      /* Odd bits: Request a destination chunk transfer */
#  define DMAC_CREQ_DCREQ(n)           (1 << (DMAC_CREQ_DCREQ_SHIFT+DMAC_CREQ_SHIFT(n)))
#  define DMAC_CREQ_DCREQ0             (1 << (DMAC_CREQ_DCREQ_SHIFT+DMAC_CREQ0_SHIFT))
#  define DMAC_CREQ_DCREQ1             (1 << (DMAC_CREQ_DCREQ_SHIFT+DMAC_CREQ1_SHIFT))
#  define DMAC_CREQ_DCREQ2             (1 << (DMAC_CREQ_DCREQ_SHIFT+DMAC_CREQ2_SHIFT))
#  define DMAC_CREQ_DCREQ3             (1 << (DMAC_CREQ_DCREQ_SHIFT+DMAC_CREQ3_SHIFT))
#  define DMAC_CREQ_DCREQ4             (1 << (DMAC_CREQ_DCREQ_SHIFT+DMAC_CREQ4_SHIFT))
#  define DMAC_CREQ_DCREQ5             (1 << (DMAC_CREQ_DCREQ_SHIFT+DMAC_CREQ5_SHIFT))
#  define DMAC_CREQ_DCREQ6             (1 << (DMAC_CREQ_DCREQ_SHIFT+DMAC_CREQ6_SHIFT))
#  define DMAC_CREQ_DCREQ7             (1 << (DMAC_CREQ_DCREQ_SHIFT+DMAC_CREQ7_SHIFT))

/* DMAC Software Last Transfer Flag Register */

#define DMAC_LAST_SHIFT(n)             ((n)<<1)
#define DMAC_LAST_MASK(n)              (3 << DMAC_LAST_SHIFT(n))
#define DMAC_LAST0_SHIFT               (0)      /* Bits 0-1:  Channel 0 */
#define DMAC_LAST0_MASK                (3 << DMAC_LAST0_SHIFT)
#define DMAC_LAST1_SHIFT               (2)      /* Bits 2-3:  Channel 1 */
#define DMAC_LAST1_MASK                (3 << DMAC_LAST1_SHIFT)
#define DMAC_LAST2_SHIFT               (4)      /* Bits 4-5:  Channel 2 */
#define DMAC_LAST2_MASK                (3 << DMAC_LAST2_SHIFT)
#define DMAC_LAST3_SHIFT               (6)      /* Bits 6-7:  Channel 3 */
#define DMAC_LAST3_MASK                (3 << DMAC_LAST3_SHIFT)
#define DMAC_LAST4_SHIFT               (8)      /* Bits 8-9: Channel 4 */
#define DMAC_LAST4_MASK                (3 << DMAC_LAST4_SHIFT)
#define DMAC_LAST5_SHIFT               (10)     /* Bits 10-11: Channel 5 */
#define DMAC_LAST5_MASK                (3 << DMAC_LAST5_SHIFT)
#define DMAC_LAST6_SHIFT               (12)     /* Bits 12-13: Channel 6 */
#define DMAC_LAST6_MASK                (3 << DMAC_LAST6_SHIFT)
#define DMAC_LAST7_SHIFT               (14)     /* Bits 14-15: Channel 7 */
#define DMAC_LAST7_MASK                (3 << DMAC_LAST7_SHIFT)

#define DMAC_LAST_SLAST_SHIFT          (0)      /* Bits 0, 2, 4, 6:   Indicates the last transfer */
#  define DMAC_LAST_SLAST(n)           (1 << (DMAC_LAST_SLAST_SHIFT+DMAC_LAST_SHIFT(n)))
#  define DMAC_LAST_SLAST0             (1 << (DMAC_LAST_SLAST_SHIFT+DMAC_LAST0_SHIFT))
#  define DMAC_LAST_SLAST1             (1 << (DMAC_LAST_SLAST_SHIFT+DMAC_LAST1_SHIFT))
#  define DMAC_LAST_SLAST2             (1 << (DMAC_LAST_SLAST_SHIFT+DMAC_LAST2_SHIFT))
#  define DMAC_LAST_SLAST3             (1 << (DMAC_LAST_SLAST_SHIFT+DMAC_LAST3_SHIFT))
#  define DMAC_LAST_SLAST4             (1 << (DMAC_LAST_SLAST_SHIFT+DMAC_LAST4_SHIFT))
#  define DMAC_LAST_SLAST5             (1 << (DMAC_LAST_SLAST_SHIFT+DMAC_LAST5_SHIFT))
#  define DMAC_LAST_SLAST6             (1 << (DMAC_LAST_SLAST_SHIFT+DMAC_LAST6_SHIFT))
#  define DMAC_LAST_SLAST7             (1 << (DMAC_LAST_SLAST_SHIFT+DMAC_LAST7_SHIFT))
#define DMAC_LAST_DLAST_SHIFT          (1)      /* Bits 1, 3, 5, 7:   Indicates the last transfer */
#  define DMAC_LAST_DLAST(n)           (1 << (DMAC_LAST_DLAST_SHIFT+DMAC_LAST_SHIFT(n))))
#  define DMAC_LAST_DLAST0             (1 << (DMAC_LAST_DLAST_SHIFT+DMAC_LAST0_SHIFT))
#  define DMAC_LAST_DLAST1             (1 << (DMAC_LAST_DLAST_SHIFT+DMAC_LAST1_SHIFT))
#  define DMAC_LAST_DLAST2             (1 << (DMAC_LAST_DLAST_SHIFT+DMAC_LAST2_SHIFT))
#  define DMAC_LAST_DLAST3             (1 << (DMAC_LAST_DLAST_SHIFT+DMAC_LAST3_SHIFT))
#  define DMAC_LAST_DLAST4             (1 << (DMAC_LAST_DLAST_SHIFT+DMAC_LAST4_SHIFT))
#  define DMAC_LAST_DLAST5             (1 << (DMAC_LAST_DLAST_SHIFT+DMAC_LAST5_SHIFT))
#  define DMAC_LAST_DLAST6             (1 << (DMAC_LAST_DLAST_SHIFT+DMAC_LAST6_SHIFT))
#  define DMAC_LAST_DLAST7             (1 << (DMAC_LAST_DLAST_SHIFT+DMAC_LAST7_SHIFT))

/* DMAC Error, Buffer Transfer and Chained Buffer Transfer Interrupt Enable Register,
 * DMAC Error, Buffer Transfer and Chained Buffer Transfer Interrupt Disable Register,
 * DMAC Error, Buffer Transfer and Chained Buffer Transfer Interrupt Mask Register, and
 * DMAC Error, Buffer Transfer and Chained Buffer Transfer Status Register common
 * bit field definitions
 */

#define DMAC_EBC_BTC_SHIFT             (0)       /* Bits 0-7: Buffer Transfer Completed */
#define DMAC_EBC_BTC_MASK              (0xff << DMAC_EBC_BTC_SHIFT)
#  define DMAC_EBC_BTC(n)              (1 << (DMAC_EBC_BTC_SHIFT+(n)))
#  define DMAC_EBC_BTC0                (1 << (DMAC_EBC_BTC_SHIFT+0))
#  define DMAC_EBC_BTC1                (1 << (DMAC_EBC_BTC_SHIFT+1))
#  define DMAC_EBC_BTC2                (1 << (DMAC_EBC_BTC_SHIFT+2))
#  define DMAC_EBC_BTC3                (1 << (DMAC_EBC_BTC_SHIFT+3))
#  define DMAC_EBC_BTC4                (1 << (DMAC_EBC_BTC_SHIFT+4))
#  define DMAC_EBC_BTC5                (1 << (DMAC_EBC_BTC_SHIFT+5))
#  define DMAC_EBC_BTC6                (1 << (DMAC_EBC_BTC_SHIFT+6))
#  define DMAC_EBC_BTC7                (1 << (DMAC_EBC_BTC_SHIFT+7))
#define DMAC_EBC_CBTC_SHIFT            (8)       /* Bits 8-15: Chained Buffer Transfer Completed  */
#define DMAC_EBC_CBTC_MASK             (0xff << DMAC_EBC_CBTC_SHIFT)
#  define DMAC_EBC_CBTC(n)             (1 << (DMAC_EBC_CBTC_SHIFT+(n)))
#  define DMAC_EBC_CBTC0               (1 << (DMAC_EBC_CBTC_SHIFT+0))
#  define DMAC_EBC_CBTC1               (1 << (DMAC_EBC_CBTC_SHIFT+1))
#  define DMAC_EBC_CBTC2               (1 << (DMAC_EBC_CBTC_SHIFT+2))
#  define DMAC_EBC_CBTC3               (1 << (DMAC_EBC_CBTC_SHIFT+3))
#  define DMAC_EBC_CBTC4               (1 << (DMAC_EBC_CBTC_SHIFT+4))
#  define DMAC_EBC_CBTC5               (1 << (DMAC_EBC_CBTC_SHIFT+5))
#  define DMAC_EBC_CBTC6               (1 << (DMAC_EBC_CBTC_SHIFT+6))
#  define DMAC_EBC_CBTC7               (1 << (DMAC_EBC_CBTC_SHIFT+7))
#define DMAC_EBC_ERR_SHIFT             (16)      /* Bits 16-23: Access Error */
#define DMAC_EBC_ERR_MASK              (0xff << DMAC_EBC_ERR_SHIFT)
#  define DMAC_EBC_ERR(n)              (1 << (DMAC_EBC_ERR_SHIFT+(n)))
#  define DMAC_EBC_ERR0                (1 << (DMAC_EBC_ERR_SHIFT+0))
#  define DMAC_EBC_ERR1                (1 << (DMAC_EBC_ERR_SHIFT+1))
#  define DMAC_EBC_ERR2                (1 << (DMAC_EBC_ERR_SHIFT+2))
#  define DMAC_EBC_ERR3                (1 << (DMAC_EBC_ERR_SHIFT+3))
#  define DMAC_EBC_ERR4                (1 << (DMAC_EBC_ERR_SHIFT+4))
#  define DMAC_EBC_ERR5                (1 << (DMAC_EBC_ERR_SHIFT+5))
#  define DMAC_EBC_ERR6                (1 << (DMAC_EBC_ERR_SHIFT+6))
#  define DMAC_EBC_ERR7                (1 << (DMAC_EBC_ERR_SHIFT+7))
#define DMAC_EBC_DICERR_SHIFT          (24)      /* Bits 24-31: Descriptor Integrity Check Error */
#define DMAC_EBC_DICERR_MASK           (0xff << DMAC_EBC_DICERR_SHIFT)
#  define DMAC_EBC_DICERR(n)           (1 << (DMAC_EBC_DICERR_SHIFT+(n)))
#  define DMAC_EBC_DICERR0             (1 << (DMAC_EBC_DICERR_SHIFT+0))
#  define DMAC_EBC_DICERR1             (1 << (DMAC_EBC_DICERR_SHIFT+1))
#  define DMAC_EBC_DICERR2             (1 << (DMAC_EBC_DICERR_SHIFT+2))
#  define DMAC_EBC_DICERR3             (1 << (DMAC_EBC_DICERR_SHIFT+3))
#  define DMAC_EBC_DICERR4             (1 << (DMAC_EBC_DICERR_SHIFT+4))
#  define DMAC_EBC_DICERR5             (1 << (DMAC_EBC_DICERR_SHIFT+5))
#  define DMAC_EBC_DICERR6             (1 << (DMAC_EBC_DICERR_SHIFT+6))
#  define DMAC_EBC_DICERR7             (1 << (DMAC_EBC_DICERR_SHIFT+7))

#define DMAC_EBC_CBTCINTS(n)           (0x00010100 << (n))  /* CBT+ERR interrupts */
#define DMAC_EBC_CHANINTS(n)           (0x00010101 << (n))  /* BTC+CBT+ERR interrupts */
#define DMAC_EBC_ALLCHANINTS           (0x00ffffff)         /* All BTC+CBT+ERR interrupts */
#define DMAC_EBC_ALLINTS               (0xffffffff)         /* All channel interrupts */

/* DMAC Channel Handler Enable Register */

#define DMAC_CHER_ENA_SHIFT            (0)       /* Bits 0-7:  Enable channel  */
#define DMAC_CHER_ENA_MASK             (0xff << DMAC_CHER_ENA_SHIFT)
#  define DMAC_CHER_ENA(n)             (1 << (DMAC_CHER_ENA_SHIFT+(n)))
#  define DMAC_CHER_ENA0               (1 << (DMAC_CHER_ENA_SHIFT+0))
#  define DMAC_CHER_ENA1               (1 << (DMAC_CHER_ENA_SHIFT+1))
#  define DMAC_CHER_ENA2               (1 << (DMAC_CHER_ENA_SHIFT+2))
#  define DMAC_CHER_ENA3               (1 << (DMAC_CHER_ENA_SHIFT+3))
#  define DMAC_CHER_ENA4               (1 << (DMAC_CHER_ENA_SHIFT+4))
#  define DMAC_CHER_ENA5               (1 << (DMAC_CHER_ENA_SHIFT+5))
#  define DMAC_CHER_ENA6               (1 << (DMAC_CHER_ENA_SHIFT+6))
#  define DMAC_CHER_ENA7               (1 << (DMAC_CHER_ENA_SHIFT+7))
#define DMAC_CHER_SUSP_SHIFT           (8)       /* Bits 8-15: Freeze channel and its context */
#define DMAC_CHER_SUSP_MASK            (0xff << DMAC_CHER_SUSP_SHIFT)
#  define DMAC_CHER_SUSP(n)            (1 << (DMAC_CHER_SUSP_SHIFT+(n)))
#  define DMAC_CHER_SUSP0              (1 << (DMAC_CHER_SUSP_SHIFT+0))
#  define DMAC_CHER_SUSP1              (1 << (DMAC_CHER_SUSP_SHIFT+1))
#  define DMAC_CHER_SUSP2              (1 << (DMAC_CHER_SUSP_SHIFT+2))
#  define DMAC_CHER_SUSP3              (1 << (DMAC_CHER_SUSP_SHIFT+3))
#  define DMAC_CHER_SUSP4              (1 << (DMAC_CHER_SUSP_SHIFT+4))
#  define DMAC_CHER_SUSP5              (1 << (DMAC_CHER_SUSP_SHIFT+5))
#  define DMAC_CHER_SUSP6              (1 << (DMAC_CHER_SUSP_SHIFT+6))
#  define DMAC_CHER_SUSP7              (1 << (DMAC_CHER_SUSP_SHIFT+7))
#define DMAC_CHER_KEEP_SHIFT           (24)      /* Bits 24-31:  Resume channel from automatic stall */
#define DMAC_CHER_KEEP_MASK            (0xff << DMAC_CHER_KEEP_SHIFT)
#  define DMAC_CHER_KEEP(n)            (1 << (DMAC_CHER_KEEP_SHIFT+(n)))
#  define DMAC_CHER_KEEP0              (1 << (DMAC_CHER_KEEP_SHIFT+0))
#  define DMAC_CHER_KEEP1              (1 << (DMAC_CHER_KEEP_SHIFT+1))
#  define DMAC_CHER_KEEP2              (1 << (DMAC_CHER_KEEP_SHIFT+2))
#  define DMAC_CHER_KEEP3              (1 << (DMAC_CHER_KEEP_SHIFT+3))
#  define DMAC_CHER_KEEP4              (1 << (DMAC_CHER_KEEP_SHIFT+4))
#  define DMAC_CHER_KEEP5              (1 << (DMAC_CHER_KEEP_SHIFT+5))
#  define DMAC_CHER_KEEP6              (1 << (DMAC_CHER_KEEP_SHIFT+6))
#  define DMAC_CHER_KEEP7              (1 << (DMAC_CHER_KEEP_SHIFT+7))

/* DMAC Channel Handler Disable Register */

#define DMAC_CHDR_DIS_SHIFT            (0)       /* Bits 0-7:  Disable DMAC channel  */
#define DMAC_CHDR_DIS_MASK             (0xff << DMAC_CHDR_DIS_SHIFT)
#  define DMAC_CHDR_DIS(n)             (1 << (DMAC_CHDR_DIS_SHIFT+(n)))
#  define DMAC_CHDR_DIS0               (1 << (DMAC_CHDR_DIS_SHIFT+0))
#  define DMAC_CHDR_DIS1               (1 << (DMAC_CHDR_DIS_SHIFT+1))
#  define DMAC_CHDR_DIS2               (1 << (DMAC_CHDR_DIS_SHIFT+2))
#  define DMAC_CHDR_DIS3               (1 << (DMAC_CHDR_DIS_SHIFT+3))
#  define DMAC_CHDR_DIS4               (1 << (DMAC_CHDR_DIS_SHIFT+4))
#  define DMAC_CHDR_DIS5               (1 << (DMAC_CHDR_DIS_SHIFT+5))
#  define DMAC_CHDR_DIS6               (1 << (DMAC_CHDR_DIS_SHIFT+6))
#  define DMAC_CHDR_DIS7               (1 << (DMAC_CHDR_DIS_SHIFT+7))
#  define DMAC_CHDR_DIS_ALL            DMAC_CHDR_DIS_MASK
#define DMAC_CHDR_RES_SHIFT            (8)       /* Bits 8-15:  Resume trasnfer, restoring context */
#define DMAC_CHDR_RES_MASK             (0xff << DMAC_CHDR_RES_SHIFT)
#  define DMAC_CHDR_RES(n)             (1 << (DMAC_CHDR_RES_SHIFT+(n)))
#  define DMAC_CHDR_RES0               (1 << (DMAC_CHDR_RES_SHIFT+0))
#  define DMAC_CHDR_RES1               (1 << (DMAC_CHDR_RES_SHIFT+1))
#  define DMAC_CHDR_RES2               (1 << (DMAC_CHDR_RES_SHIFT+2))
#  define DMAC_CHDR_RES3               (1 << (DMAC_CHDR_RES_SHIFT+3))
#  define DMAC_CHDR_RES4               (1 << (DMAC_CHDR_RES_SHIFT+4))
#  define DMAC_CHDR_RES5               (1 << (DMAC_CHDR_RES_SHIFT+5))
#  define DMAC_CHDR_RES6               (1 << (DMAC_CHDR_RES_SHIFT+6))
#  define DMAC_CHDR_RES7               (1 << (DMAC_CHDR_RES_SHIFT+7))

/* DMAC Channel Handler Status Register */

#define DMAC_CHSR_ENA_SHIFT            (0)       /* Bits 0-7:  Indicates that the channel is stalling  */
#define DMAC_CHSR_ENA_MASK             (0xff << DMAC_CHSR_ENA_SHIFT)
#  define DMAC_CHSR_ENA(n)             (1 << (DMAC_CHSR_ENA_SHIFT+(n)))
#  define DMAC_CHSR_ENA0               (1 << (DMAC_CHSR_ENA_SHIFT+0))
#  define DMAC_CHSR_ENA1               (1 << (DMAC_CHSR_ENA_SHIFT+1))
#  define DMAC_CHSR_ENA2               (1 << (DMAC_CHSR_ENA_SHIFT+2))
#  define DMAC_CHSR_ENA3               (1 << (DMAC_CHSR_ENA_SHIFT+3))
#  define DMAC_CHSR_ENA4               (1 << (DMAC_CHSR_ENA_SHIFT+4))
#  define DMAC_CHSR_ENA5               (1 << (DMAC_CHSR_ENA_SHIFT+5))
#  define DMAC_CHSR_ENA6               (1 << (DMAC_CHSR_ENA_SHIFT+6))
#  define DMAC_CHSR_ENA7               (1 << (DMAC_CHSR_ENA_SHIFT+7))
#define DMAC_CHSR_SUSP_SHIFT           (8)       /* Bits 8-15:  Indicates that the channel is empty */
#define DMAC_CHSR_SUSP_MASK            (0xff << DMAC_CHSR_SUSP_SHIFT)
#  define DMAC_CHSR_SUSP(n)            (1 << (DMAC_CHSR_SUSP_SHIFT+(n)))
#  define DMAC_CHSR_SUSP0              (1 << (DMAC_CHSR_SUSP_SHIFT+0))
#  define DMAC_CHSR_SUSP1              (1 << (DMAC_CHSR_SUSP_SHIFT+1))
#  define DMAC_CHSR_SUSP2              (1 << (DMAC_CHSR_SUSP_SHIFT+2))
#  define DMAC_CHSR_SUSP3              (1 << (DMAC_CHSR_SUSP_SHIFT+3))
#  define DMAC_CHSR_SUSP4              (1 << (DMAC_CHSR_SUSP_SHIFT+4))
#  define DMAC_CHSR_SUSP5              (1 << (DMAC_CHSR_SUSP_SHIFT+5))
#  define DMAC_CHSR_SUSP6              (1 << (DMAC_CHSR_SUSP_SHIFT+6))
#  define DMAC_CHSR_SUSP7              (1 << (DMAC_CHSR_SUSP_SHIFT+7))
#define DMAC_CHSR_EMPT_SHIFT           (16)      /* Bits 16-23:  Access Error Interrupt Enable */
#define DMAC_CHSR_EMPT_MASK            (0xff << DMAC_CHSR_EMPT_SHIFT)
#  define DMAC_CHSR_EMPT(n)            (1 << (DMAC_CHSR_EMPT_SHIFT+(n)))
#  define DMAC_CHSR_EMPT0              (1 << (DMAC_CHSR_EMPT_SHIFT+0))
#  define DMAC_CHSR_EMPT1              (1 << (DMAC_CHSR_EMPT_SHIFT+1))
#  define DMAC_CHSR_EMPT2              (1 << (DMAC_CHSR_EMPT_SHIFT+2))
#  define DMAC_CHSR_EMPT3              (1 << (DMAC_CHSR_EMPT_SHIFT+3))
#  define DMAC_CHSR_EMPT4              (1 << (DMAC_CHSR_EMPT_SHIFT+4))
#  define DMAC_CHSR_EMPT5              (1 << (DMAC_CHSR_EMPT_SHIFT+5))
#  define DMAC_CHSR_EMPT6              (1 << (DMAC_CHSR_EMPT_SHIFT+6))
#  define DMAC_CHSR_EMPT7              (1 << (DMAC_CHSR_EMPT_SHIFT+7))
#define DMAC_CHSR_STAL_SHIFT           (24)      /* Bits 24-31:  Access Error Interrupt Enable */
#define DMAC_CHSR_STAL_MASK            (0xff << DMAC_CHSR_STAL_SHIFT)
#  define DMAC_CHSR_STAL(n)            (1 << (DMAC_CHSR_STAL_SHIFT+(n)))
#  define DMAC_CHSR_STAL0              (1 << (DMAC_CHSR_STAL_SHIFT+0))
#  define DMAC_CHSR_STAL1              (1 << (DMAC_CHSR_STAL_SHIFT+1))
#  define DMAC_CHSR_STAL2              (1 << (DMAC_CHSR_STAL_SHIFT+2))
#  define DMAC_CHSR_STAL3              (1 << (DMAC_CHSR_STAL_SHIFT+3))
#  define DMAC_CHSR_STAL4              (1 << (DMAC_CHSR_STAL_SHIFT+4))
#  define DMAC_CHSR_STAL5              (1 << (DMAC_CHSR_STAL_SHIFT+5))
#  define DMAC_CHSR_STAL6              (1 << (DMAC_CHSR_STAL_SHIFT+6))
#  define DMAC_CHSR_STAL7              (1 << (DMAC_CHSR_STAL_SHIFT+7))

/* DMA channel registers */
/* DMAC Channel x [x = 0..7] Source Address Register (32-bit address) */
/* DMAC Channel x [x = 0..7] Destination Address Register (32-bit address) */

/* DMAC Channel x [x = 0..7] Descriptor Address Register */

#define DMAC_CH_DSCR_IF_SHIFT          (0)       /* Bits 0-1: Descriptor Interface Selection */
#define DMAC_CH_DSCR_IF_MASK           (3 << DMAC_CH_DSCR_IF_SHIFT)
#  define DMAC_CH_DSCR_AHB_IF0         (0 << DMAC_CH_DSCR_IF_SHIFT) /* Fetched via AHB-Lite Interface 0 */
#  define DMAC_CH_DSCR_AHB_IF1         (1 << DMAC_CH_DSCR_IF_SHIFT) /* Fetched via AHB-Lite Interface 1 */
#  define DMAC_CH_DSCR_AHB_IF2         (2 << DMAC_CH_DSCR_IF_SHIFT) /* Fetched via AHB-Lite Interface 2 */
#define DMAC_CH_DSCR_MASK              (0xfffffffc) /* Bits 2-31: Buffer Transfer Descriptor Address */

/* DMAC Channel n [n = 0..7] Control A Register */

#define DMAC_CH_CTRLA_BTSIZE_MAX       (0xffff)
#define DMAC_CH_CTRLA_BTSIZE_SHIFT     (0)       /* Bits 0-15: Buffer Transfer Size */
#define DMAC_CH_CTRLA_BTSIZE_MASK      (DMAC_CH_CTRLA_BTSIZE_MAX << DMAC_CH_CTRLA_BTSIZE_SHIFT)
#define DMAC_CH_CTRLA_SCSIZE_SHIFT     (16)      /* Bits 16-18: Source Chunk Transfer Size */
#define DMAC_CH_CTRLA_SCSIZE_MASK      (7 << DMAC_CH_CTRLA_SCSIZE_SHIFT)
#  define DMAC_CH_CTRLA_SCSIZE_1       (0 << DMAC_CH_CTRLA_SCSIZE_SHIFT)
#  define DMAC_CH_CTRLA_SCSIZE_4       (1 << DMAC_CH_CTRLA_SCSIZE_SHIFT)
#  define DMAC_CH_CTRLA_SCSIZE_8       (2 << DMAC_CH_CTRLA_SCSIZE_SHIFT)
#  define DMAC_CH_CTRLA_SCSIZE_16      (3 << DMAC_CH_CTRLA_SCSIZE_SHIFT)
#define DMAC_CH_CTRLA_DCSIZE_SHIFT     (20)     /* Bits 20-21:  Destination Chunk Transfer size */
#define DMAC_CH_CTRLA_DCSIZE_MASK      (7 << DMAC_CH_CTRLA_DCSIZE_SHIFT)
#  define DMAC_CH_CTRLA_DCSIZE_1       (0 << DMAC_CH_CTRLA_DCSIZE_SHIFT)
#  define DMAC_CH_CTRLA_DCSIZE_4       (1 << DMAC_CH_CTRLA_DCSIZE_SHIFT)
#  define DMAC_CH_CTRLA_DCSIZE_8       (2 << DMAC_CH_CTRLA_DCSIZE_SHIFT)
#  define DMAC_CH_CTRLA_DCSIZE_16      (3 << DMAC_CH_CTRLA_DCSIZE_SHIFT)
#define DMAC_CH_CTRLA_SRCWIDTH_SHIFT   (24)      /* Bits 24-25 */
#define DMAC_CH_CTRLA_SRCWIDTH_MASK    (3 << DMAC_CH_CTRLA_SRCWIDTH_SHIFT)
#  define DMAC_CH_CTRLA_SRCWIDTH_BYTE  (0 << DMAC_CH_CTRLA_SRCWIDTH_SHIFT)
#  define DMAC_CH_CTRLA_SRCWIDTH_HWORD (1 << DMAC_CH_CTRLA_SRCWIDTH_SHIFT)
#  define DMAC_CH_CTRLA_SRCWIDTH_WORD  (2 << DMAC_CH_CTRLA_SRCWIDTH_SHIFT)
#  define DMAC_CH_CTRLA_SRCWIDTH_DWORD (3 << DMAC_CH_CTRLA_SRCWIDTH_SHIFT)
#define DMAC_CH_CTRLA_DSTWIDTH_SHIFT   (28)      /* Bits 28-29 */
#define DMAC_CH_CTRLA_DSTWIDTH_MASK    (3 << DMAC_CH_CTRLA_DSTWIDTH_SHIFT)
#  define DMAC_CH_CTRLA_DSTWIDTH_BYTE  (0 << DMAC_CH_CTRLA_DSTWIDTH_SHIFT)
#  define DMAC_CH_CTRLA_DSTWIDTH_HWORD (1 << DMAC_CH_CTRLA_DSTWIDTH_SHIFT)
#  define DMAC_CH_CTRLA_DSTWIDTH_WORD  (2 << DMAC_CH_CTRLA_DSTWIDTH_SHIFT)
#  define DMAC_CH_CTRLA_DSTWIDTH_DWORD (3 << DMAC_CH_CTRLA_DSTWIDTH_SHIFT)
#define DMAC_CH_CTRLA_DONE             (1 << 31) /* Bit 31: Auto disable DMAC */

/* DMAC Channel n [n = 0..7] Control B Register */

#define DMAC_CH_CTRLB_SIF_SHIFT        (0)        /* Bits 0-1: Source Interface Selection Field */
#define DMAC_CH_CTRLB_SIF_MASK         (3 << DMAC_CH_CTRLB_SIF_SHIFT)
#  define DMAC_CH_CTRLB_SIF_IF0        (0 << DMAC_CH_CTRLB_SIF_SHIFT) /* Via AHB-Lite Interface 0 */
#  define DMAC_CH_CTRLB_SIF_IF1        (1 << DMAC_CH_CTRLB_SIF_SHIFT) /* Via AHB-Lite Interface 1 */
#  define DMAC_CH_CTRLB_SIF_IF2        (2 << DMAC_CH_CTRLB_SIF_SHIFT) /* Via AHB-Lite Interface 2 */
#define DMAC_CH_CTRLB_DIF_SHIFT        (4)        /* Bits 4-5: Destination Interface Selection Field */
#define DMAC_CH_CTRLB_DIF_MASK         (3 << DMAC_CH_CTRLB_DIF_SHIFT)
#  define DMAC_CH_CTRLB_DIF_IF0        (0 << DMAC_CH_CTRLB_DIF_SHIFT) /* Via AHB-Lite Interface 0 */
#  define DMAC_CH_CTRLB_DIF_IF1        (1 << DMAC_CH_CTRLB_DIF_SHIFT) /* Via AHB-Lite Interface 1 */
#  define DMAC_CH_CTRLB_DIF_IF2        (2 << DMAC_CH_CTRLB_DIF_SHIFT) /* Via AHB-Lite Interface 2 */
#define DMAC_CH_CTRLB_SRC_PIP          (1 << 8)  /* Bit 8:  Source Picture-in-Picture Mode */
#define DMAC_CH_CTRLB_DST_PIP          (1 << 12) /* Bit 12: Destination Picture-in-Picture Mode */
#define DMAC_CH_CTRLB_SRCDSCR          (1 << 16) /* Bit 16: Source buffer descriptor fetch operation disabled */
#define DMAC_CH_CTRLB_DSTDSCR          (1 << 20) /* Bit 20: Dest buffer descriptor fetch operation disabled */
#define DMAC_CH_CTRLB_FC_SHIFT         (21)      /* Bits 21-22:  Flow controller  */
#define DMAC_CH_CTRLB_FC_MASK          (3 << DMAC_CH_CTRLB_FC_SHIFT)
#  define DMAC_CH_CTRLB_FC_M2M         (0 << DMAC_CH_CTRLB_FC_SHIFT) /* Memory-to-Memory  */
#  define DMAC_CH_CTRLB_FC_M2P         (1 << DMAC_CH_CTRLB_FC_SHIFT) /* Memory-to-Peripheral */
#  define DMAC_CH_CTRLB_FC_P2M         (2 << DMAC_CH_CTRLB_FC_SHIFT) /* Peripheral-to-Memory  */
#  define DMAC_CH_CTRLB_FC_P2P         (3 << DMAC_CH_CTRLB_FC_SHIFT) /* Peripheral-to-Peripheral */
#define DMAC_CH_CTRLB_SRCINCR_SHIFT    (24)      /* Bits 24-25 */
#define DMAC_CH_CTRLB_SRCINCR_MASK     (3 << DMAC_CH_CTRLB_SRCINCR_SHIFT)
#  define DMAC_CH_CTRLB_SRCINCR_INCR   (0 << DMAC_CH_CTRLB_SRCINCR_SHIFT) /* Incrementing address */
#  define DMAC_CH_CTRLB_SRCINCR_DECR   (1 << DMAC_CH_CTRLB_SRCINCR_SHIFT) /* Decrementing address */
#  define DMAC_CH_CTRLB_SRCINCR_FIXED  (2 << DMAC_CH_CTRLB_SRCINCR_SHIFT) /* Fixed address */
#define DMAC_CH_CTRLB_DSTINCR_SHIFT    (28)      /* Bits 28-29 */
#define DMAC_CH_CTRLB_DSTINCR_MASK     (3 << DMAC_CH_CTRLB_DSTINCR_SHIFT)
#  define DMAC_CH_CTRLB_DSTINCR_INCR   (0 << DMAC_CH_CTRLB_DSTINCR_SHIFT) /* Incrementing address */
#  define DMAC_CH_CTRLB_DSTINCR_DECR   (1 << DMAC_CH_CTRLB_DSTINCR_SHIFT) /* Decrementing address */
#  define DMAC_CH_CTRLB_DSTINCR_FIXED  (2 << DMAC_CH_CTRLB_DSTINCR_SHIFT) /* Fixed address */
#define DMAC_CH_CTRLB_IEN              (1 << 30)  /* Bit 30: Interrupt Enable Not */
#define DMAC_CH_CTRLB_AUTO             (1 << 31)  /* Bit 31: Automatic Multiple Buffer Transfer*/

/* DMAC Channel n [n = 0..7] Configuration Register */

#define DMAC_CH_CFG_SRCPER_SHIFT       (0)       /* Bits 0-3: Source with Peripheral identifier */
#define DMAC_CH_CFG_SRCPER_MASK        (0xff << DMAC_CH_CFG_SRCPER_SHIFT)
#define DMAC_CH_CFG_DSTPER_SHIFT       (4)       /* Bits 4-7: Destination with Peripheral identifier */
#define DMAC_CH_CFG_DSTPER_MASK        (0xff << DMAC_CH_CFG_DSTPER_SHIFT)
#define DMAC_CH_CFG_SRCREP             (1 << 8)  /* Bit 8:  Source Reloaded from Previous */
#define DMAC_CH_CFG_SRCH2SEL           (1 << 9)  /* Bit 9:  HW handshake triggers transfer */
#define DMAC_CH_CFG_SRCPERMSB_SHIFT    (10)      /* Bits 10-11: SRC_PER Most Significant Bits */
#define DMAC_CH_CFG_SRCPERMSB_MASK     (3 << DMAC_CH_CFG_SRCPERMSB_SHIFT)
#define DMAC_CH_CFG_DSTREP             (1 << 8)  /* Bit 12: Destination Reloaded from Previous */
#define DMAC_CH_CFG_DSTH2SEL           (1 << 13) /* Bit 13: HW handshake trigger transfer */
#define DMAC_CH_CFG_DSTPERMSB_SHIFT    (14)      /* Bits 14-15: SRC_PER Most Significant Bits */
#define DMAC_CH_CFG_DSTPERMSB_MASK     (3 << DMAC_CH_CFG_DSTPERMSB_SHIFT)
#define DMAC_CH_CFG_SOD                (1 << 16) /* Bit 16: Stop on done */
#define DMAC_CH_CFG_LOCKIF             (1 << 20) /* Bit 20: Enable lock interface capability */
#define DMAC_CH_CFG_LOCKB              (1 << 21) /* Bit 21: Enable AHB Bus Locking capability */
#define DMAC_CH_CFG_LOCKIFL            (1 << 22) /* Bit 22: Lock Master Interface Arbiter */
#define DMAC_CH_CFG_AHBPROT_SHIFT      (24)      /* Bits 24-26: AHB access privilege */
#define DMAC_CH_CFG_AHBPROT_MASK       (7 << DMAC_CH_CFG_AHBPROT_SHIFT)
#  define DMAC_CH_CFG_AHBPROT_PRIV     (1 << DMAC_CH_CFG_AHBPROT_SHIFT) /* Privileged Access */
#  define DMAC_CH_CFG_AHBPROT_BUFF     (2 << DMAC_CH_CFG_AHBPROT_SHIFT) /* Bufferable */
#  define DMAC_CH_CFG_AHBPROT_CACHE    (4 << DMAC_CH_CFG_AHBPROT_SHIFT) /* Cacheable  */
#define DMAC_CH_CFG_FIFOCFG_SHIFT      (28)      /* Bits 28-29: FIFO Configuration */
#define DMAC_CH_CFG_FIFOCFG_MASK       (3 << DMAC_CH_CFG_FIFOCFG_SHIFT)
#  define DMAC_CH_CFG_FIFOCFG_ALAP     (0 << DMAC_CH_CFG_FIFOCFG_SHIFT) /* Largest length AHB burst */
#  define DMAC_CH_CFG_FIFOCFG_HALF     (1 << DMAC_CH_CFG_FIFOCFG_SHIFT) /* Half FIFO size */
#  define DMAC_CH_CFG_FIFOCFG_ASAP     (2 << DMAC_CH_CFG_FIFOCFG_SHIFT) /* Single AHB access ASAP */

/* DMAC Channel n [n = 0..7] Source Picture-in-Picture Configuration Register */

#define DMAC_CH_SPIP_HOLE_SHIFT        (0)       /* Bits 0-15: Source Picture-in-Picture Hole */
#define DMAC_CH_SPIP_HOLE_MASK         (0xffff << DMAC_CH_SPIP_HOLE_SHIFT)
#define DMAC_CH_SPIP_BOUNDARY_SHIFT    (16)      /* Bits 16-25: Source Picture-in-Picture Boundary */
#define DMAC_CH_SPIP_BOUNDARY_MASK     (0x3ff << DMAC_CH_SPIP_BOUNDARY_SHIFT)

/* DMAC Channel n [n = 0..7] Destination Picture-in-Picture Configuration Register */

#define DMAC_CH_DPIP_HOLE_SHIFT        (0)       /* Bits 0-15: Destination Picture-in-Picture Hole */
#define DMAC_CH_DPIP_HOLE_MASK         (0xffff << DMAC_CH_DPIP_HOLE_SHIFT)
#define DMAC_CH_DPIP_BOUNDARY_SHIFT    (16)      /* Bits 16-25: Destination Picture-in-Picture Boundary */
#define DMAC_CH_DPIP_BOUNDARY_MASK     (0x3ff << DMAC_CH_DPIP_BOUNDARY_SHIFT)

/* DMAC Write Protect Mode Register */

#define DMAC_WPMR_WPEN                 (1 << 0)  /* Bit 0:  Write Protect Enable */
#define DMAC_WPMR_WPKEY_SHIFT          (8)       /* Bits 8-31:  Write Protect KEY */
#define DMAC_WPMR_WPKEY_MASK           (0xffffff << DMAC_WPMR_WPKEY_SHIFT)
#define DMAC_WPMR_WPKEY                (0x444d41 << DMAC_WPMR_WPKEY_SHIFT)

/* DMAC Write Protect Status Register */

#define DMAC_WPSR_WPVS                 (1 << 0)  /* Bit 0:  Write Protect Violation Status */
#define DMAC_WPSR_WPVSRC_SHIFT         (8)       /* Bits 8-23:  Write Protect Violation Source */
#define DMAC_WPSR_WPVSRC_MASK          (0xffff << DMAC_WPSR_WPVSRC_SHIFT)

/* DMA Channel Definitions **************************************************************/
/* DMA Controller 0 Channel Definitions */

#define DMAC0_CH_HSMCI0               (0)        /* HSMCI0 Receive/transmit */
#define DMAC0_CH_SPI0_TX              (1)        /* SPI0 Transmit */
#define DMAC0_CH_SPI0_RX              (2)        /* SPI0 Receive */
#define DMAC0_CH_USART0_TX            (3)        /* USART0 Transmit */
#define DMAC0_CH_USART0_RX            (4)        /* USART0 Receive */
#define DMAC0_CH_USART1_TX            (5)        /* USART1 Transmit */
#define DMAC0_CH_USART1_RX            (6)        /* USART1 Receive */
#define DMAC0_CH_TWI0_TX              (7)        /* TWI0 Transmit */
#define DMAC0_CH_TWI0_RX              (8)        /* TWI0 Receive */
#define DMAC0_CH_TWI1_TX              (9)        /* TWI1 Transmit */
#define DMAC0_CH_TWI1_RX              (10)       /* TWI1 Receive */
#define DMAC0_CH_UART0_TX             (11)       /* UART0 Transmit */
#define DMAC0_CH_UART0_RX             (12)       /* UART0 Receive */
#define DMAC0_CH_SSC0_TX              (13)       /* SSC0 Transmit */
#define DMAC0_CH_SSC0_RX              (14)       /* SSC0 Receive */
#define DMAC0_CH_SMD_TX               (15)       /* SMD Transmit */
#define DMAC0_CH_SMD_RX               (16)       /* SMD Receive */

/* DMA Controller 1 Channel Definitions */

#define DMAC1_CH_HSMCI1               (0)        /* HSMCI1 Receive/transmit */
#define DMAC1_CH_HSMCI2               (1)        /* HSMCI2 Receive/transmit */
#define DMAC1_CH_ADC_RX               (2)        /* ADC Receive */
#define DMAC1_CH_SSC1_TX              (3)        /* SSC1 Transmit */
#define DMAC1_CH_SSC1_RX              (4)        /* SSC1 Receive */
#define DMAC1_CH_UART1_TX             (5)        /* UART1 Transmit */
#define DMAC1_CH_UART1_RX             (6)        /* UART1 Receive */
#define DMAC1_CH_USART2_TX            (7)        /* USART2 Transmit */
#define DMAC1_CH_USART2_RX            (8)        /* USART2 Receive */
#define DMAC1_CH_USART3_TX            (9)        /* USART3 Transmit */
#define DMAC1_CH_USART3_RX            (10)       /* USART3 Receive */
#define DMAC1_CH_TWI2_TX              (11)       /* TWI2 Transmit */
#define DMAC1_CH_TWI2_RX              (12)       /* TWI2 Receive */
#define DMAC1_CH_DBGU_TX              (13)       /* DBGU Transmit */
#define DMAC1_CH_DBGU_RX              (14)       /* DBGU Receive */
#define DMAC1_CH_SPI1_TX              (15)       /* SPI1 Transmit */
#define DMAC1_CH_SPI1_RX              (16)       /* SPI1 Receive */
#define DMAC1_CH_SHA_TX               (17)       /* SHA Transmit */
#define DMAC1_CH_AES_TX               (18)       /* AES Transmit */
#define DMAC1_CH_AES_RX               (19)       /* AES Receive */
#define DMAC1_CH_TDES_TX              (20)       /* TDES Transmit */
#define DMAC1_CH_TDES_RX              (21)       /* TDES Receive */

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/* DMA multi buffer transfer link list entry structure */

struct dma_linklist_s
{
  uint32_t saddr;   /*  0 Source address */
  uint32_t daddr;   /*  4 Destination address */
  uint32_t ctrla;   /*  8 Control A value */
  uint32_t ctrlb;   /* 12 Control B value */
  uint32_t dscr;    /* 16 Next descriptor address */
};

/* Linked List with CRC16 Attached */

struct dma_crc16_linklist_s
{
  uint32_t saddr;   /*  0 Source address */
  uint32_t daddr;   /*  4 Destination address */
  uint32_t ctrla;   /*  8 Control A value */
  uint32_t ctrlb;   /* 12 Control B value */
  uint32_t dscr;    /* 16 Next descriptor address */
  uint32_t crc16;   /* 20 CRC */
};

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_DMAC_H */
