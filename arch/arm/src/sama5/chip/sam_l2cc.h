/************************************************************************************
 * arch/arm/src/sama5/chip/sam_l2cc.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMA5_CHIP_SAM_L2CC_H
#define __ARCH_ARM_SRC_SAMA5_CHIP_SAM_L2CC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip/sam_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* L2CC Register Offsets ************************************************************/

#define SAM_L2CC_IDR_OFFSET        0x0000 /* Cache ID Register */
#define SAM_L2CC_TYPR_OFFSET       0x0004 /* Cache Type Register */
#define SAM_L2CC_CR_OFFSET         0x0100 /* Control Register */
#define SAM_L2CC_ACR_OFFSET        0x0104 /* Auxiliary Control Register */
#define SAM_L2CC_TRCR_OFFSET       0x0108 /* Tag RAM Control Register */
#define SAM_L2CC_DRCR_OFFSET       0x010c /* Data RAM Control Register */
                                          /* 0x0110-0x01fc Reserved */
#define SAM_L2CC_ECR_OFFSET        0x0200 /* Event Counter Control Register */
#define SAM_L2CC_ECFGR1_OFFSET     0x0204 /* Event Counter 1 Configuration Register */
#define SAM_L2CC_ECFGR0_OFFSET     0x0208 /* Event Counter 0 Configuration Register */
#define SAM_L2CC_EVR1_OFFSET       0x020c /* Event Counter 1 Value Register */
#define SAM_L2CC_EVR0_OFFSET       0x0210 /* Event Counter 0 Value Register */
#define SAM_L2CC_IMR_OFFSET        0x0214 /* Interrupt Mask Register */
#define SAM_L2CC_MISR_OFFSET       0x0218 /* Masked Interrupt Status Register */
#define SAM_L2CC_RISR_OFFSET       0x021c /* Raw Interrupt Status Register */
#define SAM_L2CC_ICR_OFFSET        0x0220 /* Interrupt Clear Register */
                                          /* 0x0224-0x072c Reserved */
#define SAM_L2CC_CSR_OFFSET        0x0730 /* Cache Synchronization Register */
                                          /* 0x0734-0x076c Reserved */
#define SAM_L2CC_IPALR_OFFSET      0x0770 /* Invalidate Physical Address Line Register */
                                          /* 0x0774-0x0778 Reserved */
#define SAM_L2CC_IWR_OFFSET        0x077c /* Invalidate Way Register */
                                          /* 0x0780-0x07af Reserved */
#define SAM_L2CC_CPALR_OFFSET      0x07b0 /* Clean Physical Address Line Register */
                                          /* 0x07b4 Reserved */
#define SAM_L2CC_CIR_OFFSET        0x07b8 /* Clean Index Register */
#define SAM_L2CC_CWR_OFFSET        0x07bc /* Clean Way Register */
                                          /* 0x07c0-0x07ec Reserved */
#define SAM_L2CC_CIPALR_OFFSET     0x07f0 /* Clean Invalidate Physical Address Line Register */
                                          /* 0x07f4 Reserved */
#define SAM_L2CC_CIIR_OFFSET       0x07f8 /* Clean Invalidate Index Register */
#define SAM_L2CC_CIWR_OFFSET       0x07fc /* Clean Invalidate Way Register */
                                          /* 0x0800-0x08fc Reserved */
#define SAM_L2CC_DLKR_OFFSET       0x0900 /* Data Lockdown Register */
#define SAM_L2CC_ILKR_OFFSET       0x0904 /* Instruction Lockdown Register */
                                          /* 0x0908-0x0f3c Reserved */
#define SAM_L2CC_DCR_OFFSET        0x0f40 /* Debug Control Register */
                                          /* 0x0f44-0x0f5c Reserved */
#define SAM_L2CC_PCR_OFFSET        0x0f60 /* Prefetch Control Register */
                                          /* 0x0f64-0x0f7c Reserved */
#define SAM_L2CC_POWCR_OFFSET      0x0f80 /* Power Control Register */

/* L2CC Register Addresses **********************************************************/

#define SAM_L2CC_IDR               (SAM_L2CC_VSECTION+SAM_L2CC_IDR_OFFSET)
#define SAM_L2CC_TYPR              (SAM_L2CC_VSECTION+SAM_L2CC_TYPR_OFFSET)
#define SAM_L2CC_CR                (SAM_L2CC_VSECTION+SAM_L2CC_CR_OFFSET)
#define SAM_L2CC_ACR               (SAM_L2CC_VSECTION+SAM_L2CC_ACR_OFFSET)
#define SAM_L2CC_TRCR              (SAM_L2CC_VSECTION+SAM_L2CC_TRCR_OFFSET)
#define SAM_L2CC_DRCR              (SAM_L2CC_VSECTION+SAM_L2CC_DRCR_OFFSET)
#define SAM_L2CC_ECR               (SAM_L2CC_VSECTION+SAM_L2CC_ECR_OFFSET)
#define SAM_L2CC_ECFGR1            (SAM_L2CC_VSECTION+SAM_L2CC_ECFGR1_OFFSET)
#define SAM_L2CC_ECFGR0            (SAM_L2CC_VSECTION+SAM_L2CC_ECFGR0_OFFSET)
#define SAM_L2CC_EVR1              (SAM_L2CC_VSECTION+SAM_L2CC_EVR1_OFFSET)
#define SAM_L2CC_EVR0              (SAM_L2CC_VSECTION+SAM_L2CC_EVR0_OFFSET)
#define SAM_L2CC_IMR               (SAM_L2CC_VSECTION+SAM_L2CC_IMR_OFFSET)
#define SAM_L2CC_MISR              (SAM_L2CC_VSECTION+SAM_L2CC_MISR_OFFSET)
#define SAM_L2CC_RISR              (SAM_L2CC_VSECTION+SAM_L2CC_RISR_OFFSET)
#define SAM_L2CC_ICR               (SAM_L2CC_VSECTION+SAM_L2CC_ICR_OFFSET)
#define SAM_L2CC_CSR               (SAM_L2CC_VSECTION+SAM_L2CC_CSR_OFFSET)
#define SAM_L2CC_IPALR             (SAM_L2CC_VSECTION+SAM_L2CC_IPALR_OFFSET)
#define SAM_L2CC_IWR               (SAM_L2CC_VSECTION+SAM_L2CC_IWR_OFFSET)
#define SAM_L2CC_CPALR             (SAM_L2CC_VSECTION+SAM_L2CC_CPALR_OFFSET)
#define SAM_L2CC_CIR               (SAM_L2CC_VSECTION+SAM_L2CC_CIR_OFFSET)
#define SAM_L2CC_CWR               (SAM_L2CC_VSECTION+SAM_L2CC_CWR_OFFSET)
#define SAM_L2CC_CIPALR            (SAM_L2CC_VSECTION+SAM_L2CC_CIPALR_OFFSET)
#define SAM_L2CC_CIIR              (SAM_L2CC_VSECTION+SAM_L2CC_CIIR_OFFSET)
#define SAM_L2CC_CIWR              (SAM_L2CC_VSECTION+SAM_L2CC_CIWR_OFFSET)
#define SAM_L2CC_DLKR              (SAM_L2CC_VSECTION+SAM_L2CC_DLKR_OFFSET)
#define SAM_L2CC_ILKR              (SAM_L2CC_VSECTION+SAM_L2CC_ILKR_OFFSET)
#define SAM_L2CC_DCR               (SAM_L2CC_VSECTION+SAM_L2CC_DCR_OFFSET)
#define SAM_L2CC_PCR               (SAM_L2CC_VSECTION+SAM_L2CC_PCR_OFFSET)
#define SAM_L2CC_POWCR             (SAM_L2CC_VSECTION+SAM_L2CC_POWCR_OFFSET)

/* L2CC Register Bit Definitions ****************************************************/

/* Cache ID Register */
#define L2CC_IDR_
/* Cache Type Register */
#define L2CC_TYPR_
/* Control Register */
#define L2CC_CR_
/* Auxiliary Control Register */
#define L2CC_ACR_
/* Tag RAM Control Register */
#define L2CC_TRCR_
/* Data RAM Control Register */
#define L2CC_DRCR_
/* Event Counter Control Register */
#define L2CC_ECR_
/* Event Counter 1 Configuration Register */
#define L2CC_ECFGR1_
/* Event Counter 0 Configuration Register */
#define L2CC_ECFGR0_
/* Event Counter 1 Value Register */
#define L2CC_EVR1_
/* Event Counter 0 Value Register */
#define L2CC_EVR0_
/* Interrupt Mask Register */
#define L2CC_IMR_
/* Masked Interrupt Status Register */
#define L2CC_MISR_
/* Raw Interrupt Status Register */
#define L2CC_RISR_
/* Interrupt Clear Register */
#define L2CC_ICR_
/* Cache Synchronization Register */
#define L2CC_CSR_
/* Invalidate Physical Address Line Register */
#define L2CC_IPALR_
/* Invalidate Way Register */
#define L2CC_IWR_
/* Clean Physical Address Line Register */
#define L2CC_CPALR_
/* Clean Index Register */
#define L2CC_CIR_
/* Clean Way Register */
#define L2CC_CWR_
/* Clean Invalidate Physical Address Line Register */
#define L2CC_CIPALR_
/* Clean Invalidate Index Register */
#define L2CC_CIIR_
/* Clean Invalidate Way Register */
#define L2CC_CIWR_
/* Data Lockdown Register */
#define L2CC_DLKR_
/* Instruction Lockdown Register */
#define L2CC_ILKR_
/* Debug Control Register */
#define L2CC_DCR_
/* Prefetch Control Register */
#define L2CC_PCR_
/* Power Control Register */
#define L2CC_POWCR_

#endif /* __ARCH_ARM_SRC_SAMA5_CHIP_SAM_L2CC_H */
