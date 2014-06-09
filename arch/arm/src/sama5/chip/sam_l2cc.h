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

/* Cache ID Register (32-bit ID) */

/* Cache Type Register */

#define L2CC_TYPR_IL2ASS           (1 << 6)  /* Bit 6:  Instruction L2 Cache Associativity */
#define L2CC_TYPR_IL2WSIZE_SHIFT   (8)       /* Bits 8-10: Instruction L2 Cache Way Size */
#define L2CC_TYPR_IL2WSIZE_MASK    (7 << L2CC_TYPR_IL2WSIZE_SHIFT)
#  define L2CC_TYPR_IL2WSIZE(n)    ((uint32_t)(n) << L2CC_TYPR_IL2WSIZE_SHIFT)
#define L2CC_TYPR_DL2ASS           (1 << 18) /* Bit 18: Data L2 Cache Associativity */
#define L2CC_TYPR_DL2WSIZE_SHIFT   (20)      /* Bits 20-22: Data L2 Cache Way Size */
#define L2CC_TYPR_DL2WSIZE_MASK    (7 << L2CC_TYPR_DL2WSIZE_SHIFT)
#  define L2CC_TYPR_DL2WSIZE(n)    ((uint32_t)(n) << L2CC_TYPR_DL2WSIZE_SHIFT)

/* Control Register */

#define L2CC_CR_L2CEN              (1 << 0)  /* Bit 0:  L2 Cache Enable */

/* Auxiliary Control Register */

#define L2CC_ACR_HPSO              (1 << 10) /* Bit 10:  High Priority for SO and Dev Reads Enable */
#define L2CC_ACR_SBDLE             (1 << 11) /* Bit 11:  Store Buffer Device Limitation Enable */
#define L2CC_ACR_EXCC              (1 << 12) /* Bit 12:  Exclusive Cache Configuration */
#define L2CC_ACR_SAIE              (1 << 13) /* Bit 13:  Shared Attribute Invalidate Enable */
#define L2CC_ACR_ASS               (1 << 16) /* Bit 16:  Associativity */
#define L2CC_ACR_WAYSIZE_SHIFT     (17)      /* Bits 17-19: Way Size */
#define L2CC_ACR_WAYSIZE_MASK      (7 << L2CC_ACR_WAYSIZE_SHIFT)
#  define L2CC_ACR_WAYSIZE_16KB    (1 << L2CC_ACR_WAYSIZE_SHIFT)
#define L2CC_ACR_EMBEN             (1 << 20) /* Bit 20:  Event Monitor Bus Enable */
#define L2CC_ACR_PEN               (1 << 21) /* Bit 21:  Parity Enable */
#define L2CC_ACR_SAOEN             (1 << 22) /* Bit 22:  Shared Attribute Override Enable */
#define L2CC_ACR_FWA_SHIFT         (23)      /* Bits 23-24:  Force Write Allocate */
#define L2CC_ACR_FWA_MASK          (3 << L2CC_ACR_FWA_SHIFT)
#  define L2CC_ACR_FWA_AWCACHE     (0 << L2CC_ACR_FWA_SHIFT) /* Use AWCACHE attributes for WA */
#  define L2CC_ACR_FWA_NOALLOC     (1 << L2CC_ACR_FWA_SHIFT) /* No allocate */
#  define L2CC_ACR_FWA_OVERRIDE    (2 << L2CC_ACR_FWA_SHIFT) /* Override AWCACHE attributes */
#  define L2CC_ACR_FWA_MAPPED      (3 << L2CC_ACR_FWA_SHIFT) /* Internally mapped to 00 */
#define L2CC_ACR_CRPOL             (1 << 25) /* Bit 25:  Cache Replacement Policy */
#define L2CC_ACR_NSLEN             (1 << 26) /* Bit 26:  Non-Secure Lockdown Enable */
#define L2CC_ACR_NSIAC             (1 << 27) /* Bit 27:  Non-Secure Interrupt Access Control */
#define L2CC_ACR_DPEN              (1 << 28) /* Bit 28:  Data Prefetch Enable */
#define L2CC_ACR_IPEN              (1 << 29) /* Bit 29:  Instruction Prefetch Enable */

/* Tag RAM Control Register */

#define L2CC_TRCR_TSETLAT_SHIFT    (0)       /* Bits 0-2: Setup Latency */
#define L2CC_TRCR_TSETLAT_MASK     (7 << L2CC_TRCR_TSETLAT_SHIFT)
#  define L2CC_TRCR_TSETLAT(n)     ((uint32_t)(n) << L2CC_TRCR_TSETLAT_SHIFT)
#define L2CC_TRCR_TRDLAT_SHIFT     (4)       /* Bits 4-6: Read Access Latency */
#define L2CC_TRCR_TRDLAT_MASK      (7 << L2CC_TRCR_TRDLAT_SHIFT)
#  define L2CC_TRCR_TRDLAT(n)      ((uint32_t)(n) << L2CC_TRCR_TRDLAT_SHIFT)
#define L2CC_TRCR_TWRLAT_SHIFT     (8)       /* Bits 8-10: Write Access Latency */
#define L2CC_TRCR_TWRLAT_MASK      (7 << L2CC_TRCR_TWRLAT_SHIFT)
#  define L2CC_TRCR_TWRLAT(n)      ((uint32_t)(n) << L2CC_TRCR_TWRLAT_SHIFT)

/* Data RAM Control Register */

#define L2CC_DRCR_DSETLAT_SHIFT    (0)       /* Bits 0-2: Setup Latency */
#define L2CC_DRCR_DSETLAT_MASK     (7 << L2CC_DRCR_DSETLAT_SHIFT)
#  define L2CC_DRCR_DSETLAT(n)     ((uint32_t)(n) << L2CC_DRCR_DSETLAT_SHIFT)
#define L2CC_DRCR_DRDLAT_SHIFT     (4)       /* Bits 4-6: Read Access Latency */
#define L2CC_DRCR_DRDLAT_MASK      (7 << L2CC_DRCR_DRDLAT_SHIFT)
#  define L2CC_DRCR_DRDLAT(n)      ((uint32_t)(n) << L2CC_DRCR_DRDLAT_SHIFT)
#define L2CC_DRCR_DWRLAT_SHIFT     (8)       /* Bits 8-10: Write Access Latency */
#define L2CC_DRCR_DWRLAT_MASK      (7 << L2CC_DRCR_DWRLAT_SHIFT)
#  define L2CC_DRCR_DWRLAT(n)      ((uint32_t)(n) << L2CC_DRCR_DWRLAT_SHIFT)

/* Event Counter Control Register */

#define L2CC_ECR_EVCEN             (1 << 0)  /* Bit 0:  Event Counter Enable */
#define L2CC_ECR_EVC0RST           (1 << 1)  /* Bit 1:  Event Counter 0 Reset */
#define L2CC_ECR_EVC1RST           (1 << 2)  /* Bit 2:  Event Counter 1 Reset */

/* Event Counter 1 Configuration Register */


#define L2CC_ECFGR1_EIGEN_SHIFT    (0)       /* Bits 0-1: Event Counter Interrupt Generation */
#define L2CC_ECFGR1_EIGEN_MASK     (3 << L2CC_ECFGR1_EIGEN_SHIFT)
#  define L2CC_ECFGR1_EIGEN_INTDIS    (0 << L2CC_ECFGR1_EIGEN_SHIFT) /* Disables (default) */
#  define L2CC_ECFGR1_EIGEN_INTENINCR (1 << L2CC_ECFGR1_EIGEN_SHIFT) /* Enables with Increment condition */
#  define L2CC_ECFGR1_EIGEN_INTENOVER (2 << L2CC_ECFGR1_EIGEN_SHIFT) /* Enables with Overflow condition */
#  define L2CC_ECFGR1_EIGEN_INTGENDIS (3 << L2CC_ECFGR1_EIGEN_SHIFT) /* Disables Interrupt generation */
#define L2CC_ECFGR1_ESRC_SHIFT     (2)       /* Bits 2-5: Event Counter Source */
#define L2CC_ECFGR1_ESRC_MASK      (15 << L2CC_ECFGR1_ESRC_SHIFT)
#  define L2CC_ECFGR1_ESRC_CNTDIS     (0 << L2CC_ECFGR1_ESRC_SHIFT)  /* Counter Disabled */
#  define L2CC_ECFGR1_ESRC_CO         (1 << L2CC_ECFGR1_ESRC_SHIFT)  /* Source is CO */
#  define L2CC_ECFGR1_ESRC_DRHIT      (2 << L2CC_ECFGR1_ESRC_SHIFT)  /* Source is DRHIT */
#  define L2CC_ECFGR1_ESRC_DRREQ      (3 << L2CC_ECFGR1_ESRC_SHIFT)  /* Source is DRREQ */
#  define L2CC_ECFGR1_ESRC_DWHIT      (4 << L2CC_ECFGR1_ESRC_SHIFT)  /* Source is DWHIT */
#  define L2CC_ECFGR1_ESRC_DWREQ      (5 << L2CC_ECFGR1_ESRC_SHIFT)  /* Source is DWREQ */
#  define L2CC_ECFGR1_ESRC_DWTREQ     (6 << L2CC_ECFGR1_ESRC_SHIFT)  /* Source is DWTREQ */
#  define L2CC_ECFGR1_ESRC_IRHIT      (7 << L2CC_ECFGR1_ESRC_SHIFT)  /* Source is IRHIT */
#  define L2CC_ECFGR1_ESRC_IRREQ      (8 << L2CC_ECFGR1_ESRC_SHIFT)  /* Source is IRREQ */
#  define L2CC_ECFGR1_ESRC_WA         (9 << L2CC_ECFGR1_ESRC_SHIFT)  /* Source is WA */
#  define L2CC_ECFGR1_ESRC_IPFALLOC   (10 << L2CC_ECFGR1_ESRC_SHIFT) /* Source is IPFALLOC */
#  define L2CC_ECFGR1_ESRC_EPFHIT     (11 << L2CC_ECFGR1_ESRC_SHIFT) /* Source is EPFHIT */
#  define L2CC_ECFGR1_ESRC_EPFALLOC   (12 << L2CC_ECFGR1_ESRC_SHIFT) /* Source is EPFALLOC */
#  define L2CC_ECFGR1_ESRC_SRRCVD     (13 << L2CC_ECFGR1_ESRC_SHIFT) /* Source is SRRCVD */
#  define L2CC_ECFGR1_ESRC_SRCONF     (14 << L2CC_ECFGR1_ESRC_SHIFT) /* Source is SRCONF */
#  define L2CC_ECFGR1_ESRC_EPFRCVD    (15 << L2CC_ECFGR1_ESRC_SHIFT) /* Source is EPFRCVD */

/* Event Counter 0 Configuration Register */

#define L2CC_ECFGR0_EIGEN_SHIFT    (0)       /* Bits 0-1: Event Counter Interrupt Generation */
#define L2CC_ECFGR0_EIGEN_MASK     (3 << L2CC_ECFGR0_EIGEN_SHIFT)
#  define L2CC_ECFGR0_EIGEN_INTDIS    (0 << L2CC_ECFGR0_EIGEN_SHIFT) /* Disables (default) */
#  define L2CC_ECFGR0_EIGEN_INTENINCR (1 << L2CC_ECFGR0_EIGEN_SHIFT) /* Enables with Increment condition */
#  define L2CC_ECFGR0_EIGEN_INTENOVER (2 << L2CC_ECFGR0_EIGEN_SHIFT) /* Enables with Overflow condition */
#  define L2CC_ECFGR0_EIGEN_INTGENDIS (3 << L2CC_ECFGR0_EIGEN_SHIFT) /* Disables Interrupt generation */
#define L2CC_ECFGR0_ESRC_SHIFT     (2)       /* Bits 2-5: Event Counter Source */
#define L2CC_ECFGR0_ESRC_MASK      (15 << L2CC_ECFGR0_ESRC_SHIFT)
#  define L2CC_ECFGR0_ESRC_CNTDIS     (0 << L2CC_ECFGR0_ESRC_SHIFT)  /* Counter Disabled */
#  define L2CC_ECFGR0_ESRC_CO         (1 << L2CC_ECFGR0_ESRC_SHIFT)  /* Source is CO */
#  define L2CC_ECFGR0_ESRC_DRHIT      (2 << L2CC_ECFGR0_ESRC_SHIFT)  /* Source is DRHIT */
#  define L2CC_ECFGR0_ESRC_DRREQ      (3 << L2CC_ECFGR0_ESRC_SHIFT)  /* Source is DRREQ */
#  define L2CC_ECFGR0_ESRC_DWHIT      (4 << L2CC_ECFGR0_ESRC_SHIFT)  /* Source is DWHIT */
#  define L2CC_ECFGR0_ESRC_DWREQ      (5 << L2CC_ECFGR0_ESRC_SHIFT)  /* Source is DWREQ */
#  define L2CC_ECFGR0_ESRC_DWTREQ     (6 << L2CC_ECFGR0_ESRC_SHIFT)  /* Source is DWTREQ */
#  define L2CC_ECFGR0_ESRC_IRHIT      (7 << L2CC_ECFGR0_ESRC_SHIFT)  /* Source is IRHIT */
#  define L2CC_ECFGR0_ESRC_IRREQ      (8 << L2CC_ECFGR0_ESRC_SHIFT)  /* Source is IRREQ */
#  define L2CC_ECFGR0_ESRC_WA         (9 << L2CC_ECFGR0_ESRC_SHIFT)  /* Source is WA */
#  define L2CC_ECFGR0_ESRC_IPFALLOC   (10 << L2CC_ECFGR0_ESRC_SHIFT) /* Source is IPFALLOC */
#  define L2CC_ECFGR0_ESRC_EPFHIT     (11 << L2CC_ECFGR0_ESRC_SHIFT) /* Source is EPFHIT */
#  define L2CC_ECFGR0_ESRC_EPFALLOC   (12 << L2CC_ECFGR0_ESRC_SHIFT) /* Source is EPFALLOC */
#  define L2CC_ECFGR0_ESRC_SRRCVD     (13 << L2CC_ECFGR0_ESRC_SHIFT) /* Source is SRRCVD */
#  define L2CC_ECFGR0_ESRC_SRCONF     (14 << L2CC_ECFGR0_ESRC_SHIFT) /* Source is SRCONF */
#  define L2CC_ECFGR0_ESRC_EPFRCVD    (15 << L2CC_ECFGR0_ESRC_SHIFT) /* Source is EPFRCVD */

/* Event Counter 1 Value Register (32-bit value) */
/* Event Counter 0 Value Register (32-bit value) */

/* Interrupt Mask Register, Masked Interrupt Status Register, Raw Interrupt Status
 * Register, and Interrupt Clear Register.
 */

#define L2CC_INT_ECNTR             (1 << 0)  /* Bit 0:  Event Counter 1/0 Overflow Increment */
#define L2CC_INT_PARRT             (1 << 1)  /* Bit 1:  Parity Error on L2 Tag RAM, Read */
#define L2CC_INT_PARRD             (1 << 2)  /* Bit 2:  Parity Error on L2 Data RAM, Read */
#define L2CC_INT_ERRWT             (1 << 3)  /* Bit 3:  Error on L2 Tag RAM, Write */
#define L2CC_INT_ERRWD             (1 << 4)  /* Bit 4:  Error on L2 Data RAM, Write */
#define L2CC_INT_ERRRT             (1 << 5)  /* Bit 5:  Error on L2 Tag RAM, Read */
#define L2CC_INT_ERRRD             (1 << 6)  /* Bit 6:  Error on L2 Data RAM, Read */
#define L2CC_INT_SLVERR            (1 << 7)  /* Bit 7:  SLVERR from L3 Memory */
#define L2CC_INT_DECERR            (1 << 8)  /* Bit 8:  DECERR from L3 Memory */

/* Cache Synchronization Register */

#define L2CC_CSR_C                 (1 << 0)  /* Bit 0:  Cache Synchronization Status */

/* Invalidate Physical Address Line Register */

#define L2CC_IPALR_C               (1 << 0)  /* Bit 0:  Cache Synchronization Status */
#define L2CC_IPALR_IDX_SHIFT       (5)       /* Bits 5-13: Index Number */
#define L2CC_IPALR_IDX_MASK        (0x1ff << L2CC_IPALR_IDX_SHIFT)
#  define L2CC_IPALR_IDX(n)        ((uint32_t)(n) << L2CC_IPALR_IDX_SHIFT)
#define L2CC_IPALR_TAG_SHIFT       (14)      /* Bits 14-31: Tag Number */
#define L2CC_IPALR_TAG_MASK        (0x3ffff << L2CC_IPALR_TAG_SHIFT)
#  define L2CC_IPALR_TAG(n)        ((uint32_t)(n) << L2CC_IPALR_TAG_SHIFT)

/* Invalidate Way Register */

#define L2CC_IWR_WAY(n)            (1 << (n)) /* Bist 0-7:  Invalidate Way Number n, n=0..7 */
#  define L2CC_IWR_WAY0            (1 << 0)   /* Bit 0:  Invalidate Way Number 0 */
#  define L2CC_IWR_WAY1            (1 << 1)   /* Bit 1:  Invalidate Way Number 1 */
#  define L2CC_IWR_WAY2            (1 << 2)   /* Bit 2:  Invalidate Way Number 2 */
#  define L2CC_IWR_WAY3            (1 << 3)   /* Bit 3:  Invalidate Way Number 3 */
#  define L2CC_IWR_WAY4            (1 << 4)   /* Bit 4:  Invalidate Way Number 4 */
#  define L2CC_IWR_WAY5            (1 << 5)   /* Bit 5:  Invalidate Way Number 5 */
#  define L2CC_IWR_WAY6            (1 << 6)   /* Bit 6:  Invalidate Way Number 6 */
#  define L2CC_IWR_WAY7            (1 << 7)   /* Bit 7:  Invalidate Way Number 7 */

/* Clean Physical Address Line Register */

#define L2CC_CPALR_C               (1 << 0)  /* Bit 0:  Cache Synchronization Status */
#define L2CC_CPALR_IDX_SHIFT       (5)       /* Bits 5-13: Index number */
#define L2CC_CPALR_IDX_MASK        (0x1ff << L2CC_CPALR_IDX_SHIFT)
#  define L2CC_CPALR_IDX(n)        ((uint32_t)(n) << L2CC_CPALR_IDX_SHIFT)
#define L2CC_CPALR_TAG_SHIFT       (14)      /* Bits 14-31: Tag number */
#define L2CC_CPALR_TAG_MASK        (0x3ffff << L2CC_CPALR_TAG_SHIFT)
#  define L2CC_CPALR_TAG(n)        ((uint32_t)(n) << L2CC_CPALR_TAG_SHIFT)

/* Clean Index Register */

#define L2CC_CIR_C                 (1 << 0)  /* Bit 0:  Cache Synchronization Status */
#define L2CC_CIR_IDX_SHIFT         (5)       /* Bits 5-13: Index number */
#define L2CC_CIR_IDX_MASK          (0x1ff << L2CC_CIR_IDX_SHIFT)
#  define L2CC_CIR_IDX(n)          ((uint32_t)(n) << L2CC_CIR_IDX_SHIFT)
#define L2CC_CIR_WAY_SHIFT         (28)      /* Bits 28-30: Way number */
#define L2CC_CIR_WAY_MASK          (7 << L2CC_CIR_WAY_SHIFT)
#  define L2CC_CIR_WAY(n)          ((uint32_t)(n) << L2CC_CIR_WAY_SHIFT)

/* Clean Way Register */

#define L2CC_CWR_WAY(n)            (1 << (n)) /* Bits 0-7:  Clean Way Number n, n=0..7 */
#  define L2CC_CWR_WAY0            (1 << 0)   /* Bit 0:  Clean Way Number 0 */
#  define L2CC_CWR_WAY1            (1 << 1)   /* Bit 1:  Clean Way Number 1 */
#  define L2CC_CWR_WAY2            (1 << 2)   /* Bit 2:  Clean Way Number 2 */
#  define L2CC_CWR_WAY3            (1 << 3)   /* Bit 3:  Clean Way Number 3 */
#  define L2CC_CWR_WAY4            (1 << 4)   /* Bit 4:  Clean Way Number 4 */
#  define L2CC_CWR_WAY5            (1 << 5)   /* Bit 5:  Clean Way Number 5 */
#  define L2CC_CWR_WAY6            (1 << 6)   /* Bit 6:  Clean Way Number 6 */
#  define L2CC_CWR_WAY7            (1 << 7)   /* Bit 7:  Clean Way Number 7 */

/* Clean Invalidate Physical Address Line Register */

#define L2CC_CIPALR_C              (1 << 0)  /* Bit 0:  Cache Synchronization Status */
#define L2CC_CIPALR_IDX_SHIFT      (5)       /* Bits 5-13: Index Number */
#define L2CC_CIPALR_IDX_MASK       (0x1ff << L2CC_CIPALR_IDX_SHIFT)
#  define L2CC_CIPALR_IDX(n)       ((uint32_t)(n) << L2CC_CIPALR_IDX_SHIFT)
#define L2CC_CIPALR_TAG_SHIFT      (14)      /* Bits 14-31: Tag Number */
#define L2CC_CIPALR_TAG_MASK       (0x3ffff << L2CC_CIPALR_TAG_SHIFT)
#  define L2CC_CIPALR_TAG(n)       ((uint32_t)(n) << L2CC_CIPALR_TAG_SHIFT)

/* Clean Invalidate Index Register */

#define L2CC_CIIR_C                (1 << 0)  /* Bit 0:  Cache Synchronization Status */
#define L2CC_CIIR_IDX_SHIFT        (5)       /* Bits 5-13: Index Number */
#define L2CC_CIIR_IDX_MASK         (0x1ff << L2CC_CIIR_IDX_SHIFT)
#  define L2CC_CIIR_IDX(n)         ((uint32_t)(n) << L2CC_CIIR_IDX_SHIFT)
#define L2CC_CIIR_WAY_SHIFT        (28)      /* Bits 28-30: Way Number */
#define L2CC_CIIR_WAY_MASK         (7 << L2CC_CIIR_WAY_SHIFT)
#  define L2CC_CIIR_WAY(n)         ((uint32_t)(n) << L2CC_CIIR_WAY_SHIFT)

/* Clean Invalidate Way Register */

#define L2CC_CIWR_WAY(n)           (1 << (n)) /* Bits 0-7:  Clean Invalidate Way Number n, n=1..7 */
#  define L2CC_CIWR_WAY0           (1 << 0)   /* Bit 0:  Clean Invalidate Way Number 0 */
#  define L2CC_CIWR_WAY1           (1 << 1)   /* Bit 1:  Clean Invalidate Way Number 1 */
#  define L2CC_CIWR_WAY2           (1 << 2)   /* Bit 2:  Clean Invalidate Way Number 2 */
#  define L2CC_CIWR_WAY3           (1 << 3)   /* Bit 3:  Clean Invalidate Way Number 3 */
#  define L2CC_CIWR_WAY4           (1 << 4)   /* Bit 4:  Clean Invalidate Way Number 4 */
#  define L2CC_CIWR_WAY5           (1 << 5)   /* Bit 5:  Clean Invalidate Way Number 5 */
#  define L2CC_CIWR_WAY6           (1 << 6)   /* Bit 6:  Clean Invalidate Way Number 6 */
#  define L2CC_CIWR_WAY7           (1 << 7)   /* Bit 7:  Clean Invalidate Way Number 7 */

/* Data Lockdown Register */

#define L2CC_DLKR_DLK(n)           (1 << (n)) /* Bits 0-7:  Data Lockdown in Way Number n, n=0..7 */
#  define L2CC_DLKR_DLK0           (1 << 0)   /* Bit 0:  Data Lockdown in Way Number 0 */
#  define L2CC_DLKR_DLK1           (1 << 1)   /* Bit 1:  Data Lockdown in Way Number 1 */
#  define L2CC_DLKR_DLK2           (1 << 2)   /* Bit 2:  Data Lockdown in Way Number 2 */
#  define L2CC_DLKR_DLK3           (1 << 3)   /* Bit 3:  Data Lockdown in Way Number 3 */
#  define L2CC_DLKR_DLK4           (1 << 4)   /* Bit 4:  Data Lockdown in Way Number 4 */
#  define L2CC_DLKR_DLK5           (1 << 5)   /* Bit 5:  Data Lockdown in Way Number 5 */
#  define L2CC_DLKR_DLK6           (1 << 6)   /* Bit 6:  Data Lockdown in Way Number 6 */
#  define L2CC_DLKR_DLK7           (1 << 7)   /* Bit 7:  Data Lockdown in Way Number 7 */

/* Instruction Lockdown Register */

#define L2CC_ILKR_ILK(n)           (1 << (n)) /* Bits 0-7:  Instruction Lockdown in Way Number n, n=0..7 */
#  define L2CC_ILKR_ILK0           (1 << 0)   /* Bit 0:  Instruction Lockdown in Way Number 0 */
#  define L2CC_ILKR_ILK1           (1 << 1)   /* Bit 1:  Instruction Lockdown in Way Number 1 */
#  define L2CC_ILKR_ILK2           (1 << 2)   /* Bit 2:  Instruction Lockdown in Way Number 2 */
#  define L2CC_ILKR_ILK3           (1 << 3)   /* Bit 3:  Instruction Lockdown in Way Number 3 */
#  define L2CC_ILKR_ILK4           (1 << 4)   /* Bit 4:  Instruction Lockdown in Way Number 4 */
#  define L2CC_ILKR_ILK5           (1 << 5)   /* Bit 5:  Instruction Lockdown in Way Number 5 */
#  define L2CC_ILKR_ILK6           (1 << 6)   /* Bit 6:  Instruction Lockdown in Way Number 6 */
#  define L2CC_ILKR_ILK7           (1 << 7)   /* Bit 7:  Instruction Lockdown in Way Number 7 */

/* Debug Control Register */

#define L2CC_DCR_DCL               (1 << 0)  /* Bit 0:  Disable Cache Linefill */
#define L2CC_DCR_DWB               (1 << 1)  /* Bit 1:  Disable Write-back, Force Write-through */
#define L2CC_DCR_SPNIDEN           (1 << 2)  /* Bit 2:  SPNIDEN Value */

/* Prefetch Control Register */

#define L2CC_PCR_OFFSET_SHIFT      (0)       /* Bits 0-4: Prefetch Offset */
#define L2CC_PCR_OFFSET_MASK       (31 << L2CC_PCR_OFFSET_SHIFT)
#  define L2CC_PCR_OFFSET(n)       ((uint32_t)(n) << L2CC_PCR_OFFSET_SHIFT)
#define L2CC_PCR_NSIDEN            (1 << 21) /* Bit 21: Not Same ID on Exclusive Sequence Enable */
#define L2CC_PCR_IDLEN             (1 << 23) /* Bit 23: INCR Double Linefill Enable */
#define L2CC_PCR_PDEN              (1 << 24) /* Bit 24: Prefetch Drop Enable */
#define L2CC_PCR_DLFWRDIS          (1 << 27) /* Bit 27: Double Linefill on WRAP Read Disable */
#define L2CC_PCR_DATPEN            (1 << 28) /* Bit 28: Data Prefetch Enable */
#define L2CC_PCR_INSPEN            (1 << 29) /* Bit 29: Instruction Prefetch Enable */
#define L2CC_PCR_DLEN              (1 << 30) /* Bit 30: Double Linefill Enable */

/* Power Control Register */

#define L2CC_POWCR_STBYEN          (1 << 0)  /* Bit 0:  Standby Mode Enable */
#define L2CC_POWCR_DCKGATEN        (1 << 1)  /* Bit 1:  Dynamic Clock Gating Enable */

#endif /* __ARCH_ARM_SRC_SAMA5_CHIP_SAM_L2CC_H */
