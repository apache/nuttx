/************************************************************************************
 * arch/arm/src/sama5/chip/sam_xdmach
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

#ifndef __ARCH_ARM_SRC_SAMA5_CHIP_SAM_XDMAC_H
#define __ARCH_ARM_SRC_SAMA5_CHIP_SAM_XDMAC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/sama5/chip.h>

/************************************************************************************
 * Included Files
 ************************************************************************************/
/* XDMAC Register Offsets ***********************************************************/

#define SAM_XDMAC_GTYPE_OFFSET      0x0000 /* Global Type Register */
#define SAM_XDMAC_GCFG_OFFSET       0x0004 /* Global Configuration Register */
#define SAM_XDMAC_GWAC_OFFSET       0x0008 /* Global Weighted Arbiter Configuration Register */
#define SAM_XDMAC_GIE_OFFSET        0x000c /* Global Interrupt Enable Register */
#define SAM_XDMAC_GID_OFFSET        0x0010 /* Global Interrupt Disable Register */
#define SAM_XDMAC_GIM_OFFSET        0x0014 /* Global Interrupt Mask Register */
#define SAM_XDMAC_GIS_OFFSET        0x0018 /* Global Interrupt Status Register */
#define SAM_XDMAC_GE_OFFSET         0x001c /* Global Channel Enable Register */
#define SAM_XDMAC_GD_OFFSET         0x0020 /* Global Channel Disable Register */
#define SAM_XDMAC_GS_OFFSET         0x0024 /* Global Channel Status Register */
#define SAM_XDMAC_GRS_OFFSET        0x0028 /* Global Channel Read Suspend Register */
#define SAM_XDMAC_GWS_OFFSET        0x002c /* Global Channel Write Suspend Register */
#define SAM_XDMAC_GRWS_OFFSET       0x0030 /* Global Channel Read Write Suspend Register */
#define SAM_XDMAC_GRWR_OFFSET       0x0034 /* Global Channel Read Write Resume Register */
#define SAM_XDMAC_GSWR_OFFSET       0x0038 /* Global Channel Software Request Register */
#define SAM_XDMAC_GSWS_OFFSET       0x003c /* Global Channel Software Request Status Register */
#define SAM_XDMAC_GSWF_OFFSET       0x0040 /* Global Channel Software Flush Request Register */
                                           /* 0x0044–0x004c Reserved */
#define SAM_XDMACH_OFFSET(n)        (0x0050 + ((n) << 6))
#define SAM_XDMACH_CIE_OFFSET       0x0000 /* Channel Interrupt Enable Register */
#define SAM_XDMACH_CID_OFFSET       0x0004 /* Channel Interrupt Disable Register */
#define SAM_XDMACH_CIM_OFFSET       0x0008 /* Channel Interrupt Mask Register */
#define SAM_XDMACH_CIS_OFFSET       0x000C /* Channel Interrupt Status Register */
#define SAM_XDMACH_CSA_OFFSET       0x0010 /* Channel Source Address Register */
#define SAM_XDMACH_CDA_OFFSET       0x0014 /* Channel Destination Address Register */
#define SAM_XDMACH_CNDA_OFFSET      0x0018 /* Channel Next Descriptor Address Register */
#define SAM_XDMACH_CNDC_OFFSET      0x001c /* Channel Next Descriptor Control Register */
#define SAM_XDMACH_CUBC_OFFSET      0x0020 /* Channel Microblock Control Register */
#define SAM_XDMACH_CBC_OFFSET       0x0024 /* Channel Block Control Register */
#define SAM_XDMACH_CC_OFFSET        0x0028 /* Channel Configuration Register */
#define SAM_XDMACH_CDSMSP_OFFSET    0x002c /* Channel Data Stride Memory Set Pattern */
#define SAM_XDMACH_CSUS_OFFSET      0x0030 /* Channel Source Microblock Stride */
#define SAM_XDMACH_CDUS_OFFSET      0x0034 /* Channel Destination Microblock Stride */
                                           /* 0x0038-0x003c Reserved */
                                           /* 0x0fec–0x0ffc Reserved */

/* XDMAC Register Addresses *********************************************************/

#define SAM_XDMAC0_GTYPE            (SAM_XDMAC0_VBASE+SAM_XDMAC_GTYPE_OFFSET)
#define SAM_XDMAC0_GCFG             (SAM_XDMAC0_VBASE+SAM_XDMAC_GCFG_OFFSET)
#define SAM_XDMAC0_GWAC             (SAM_XDMAC0_VBASE+SAM_XDMAC_GWAC_OFFSET)
#define SAM_XDMAC0_GIE              (SAM_XDMAC0_VBASE+SAM_XDMAC_GIE_OFFSET)
#define SAM_XDMAC0_GID              (SAM_XDMAC0_VBASE+SAM_XDMAC_GID_OFFSET)
#define SAM_XDMAC0_GIM              (SAM_XDMAC0_VBASE+SAM_XDMAC_GIM_OFFSET)
#define SAM_XDMAC0_GIS              (SAM_XDMAC0_VBASE+SAM_XDMAC_GIS_OFFSET)
#define SAM_XDMAC0_GE               (SAM_XDMAC0_VBASE+SAM_XDMAC_GE_OFFSET)
#define SAM_XDMAC0_GD               (SAM_XDMAC0_VBASE+SAM_XDMAC_GD_OFFSET)
#define SAM_XDMAC0_GS               (SAM_XDMAC0_VBASE+SAM_XDMAC_GS_OFFSET)
#define SAM_XDMAC0_GRS              (SAM_XDMAC0_VBASE+SAM_XDMAC_GRS_OFFSET)
#define SAM_XDMAC0_GWS              (SAM_XDMAC0_VBASE+SAM_XDMAC_GWS_OFFSET)
#define SAM_XDMAC0_GRWS             (SAM_XDMAC0_VBASE+SAM_XDMAC_GRWS_OFFSET)
#define SAM_XDMAC0_GRWR             (SAM_XDMAC0_VBASE+SAM_XDMAC_GRWR_OFFSET)
#define SAM_XDMAC0_GSWR             (SAM_XDMAC0_VBASE+SAM_XDMAC_GSWR_OFFSET)
#define SAM_XDMAC0_GSWS             (SAM_XDMAC0_VBASE+SAM_XDMAC_GSWS_OFFSET)
#define SAM_XDMAC0_GSWF             (SAM_XDMAC0_VBASE+SAM_XDMAC_GSWF_OFFSET)

#define SAM_XDMACH0_BASE(n)         (SAM_XDMAC0_VBASE+SAM_XDMACH_OFFSET(n))
#define SAM_XDMACH0_CIE(n)          (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CIE_OFFSET)
#define SAM_XDMACH0_CID(n)          (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CID_OFFSET)
#define SAM_XDMACH0_CIM(n)          (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CIM_OFFSET)
#define SAM_XDMACH0_CIS(n)          (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CIS_OFFSET)
#define SAM_XDMACH0_CSA(n)          (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CSA_OFFSET)
#define SAM_XDMACH0_CDA(n)          (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CDA_OFFSET)
#define SAM_XDMACH0_CNDA(n)         (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CNDA_OFFSET)
#define SAM_XDMACH0_CNDC(n)         (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CNDC_OFFSET)
#define SAM_XDMACH0_CUBC(n)         (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CUBC_OFFSET)
#define SAM_XDMACH0_CBC(n)          (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CBC_OFFSET)
#define SAM_XDMACH0_CC(n)           (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CC_OFFSET)
#define SAM_XDMACH0_CDSMSP(n)       (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CDSMSP_OFFSET)
#define SAM_XDMACH0_CSUS(n)         (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CSUS_OFFSET)
#define SAM_XDMACH0_CDUS(n)         (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CDUS_OFFSET)

#define SAM_XDMAC1_GTYPE            (SAM_XDMAC1_VBASE+SAM_XDMAC_GTYPE_OFFSET)
#define SAM_XDMAC1_GCFG             (SAM_XDMAC1_VBASE+SAM_XDMAC_GCFG_OFFSET)
#define SAM_XDMAC1_GWAC             (SAM_XDMAC1_VBASE+SAM_XDMAC_GWAC_OFFSET)
#define SAM_XDMAC1_GIE              (SAM_XDMAC1_VBASE+SAM_XDMAC_GIE_OFFSET)
#define SAM_XDMAC1_GID              (SAM_XDMAC1_VBASE+SAM_XDMAC_GID_OFFSET)
#define SAM_XDMAC1_GIM              (SAM_XDMAC1_VBASE+SAM_XDMAC_GIM_OFFSET)
#define SAM_XDMAC1_GIS              (SAM_XDMAC1_VBASE+SAM_XDMAC_GIS_OFFSET)
#define SAM_XDMAC1_GE               (SAM_XDMAC1_VBASE+SAM_XDMAC_GE_OFFSET)
#define SAM_XDMAC1_GD               (SAM_XDMAC1_VBASE+SAM_XDMAC_GD_OFFSET)
#define SAM_XDMAC1_GS               (SAM_XDMAC1_VBASE+SAM_XDMAC_GS_OFFSET)
#define SAM_XDMAC1_GRS              (SAM_XDMAC1_VBASE+SAM_XDMAC_GRS_OFFSET)
#define SAM_XDMAC1_GWS              (SAM_XDMAC1_VBASE+SAM_XDMAC_GWS_OFFSET)
#define SAM_XDMAC1_GRWS             (SAM_XDMAC1_VBASE+SAM_XDMAC_GRWS_OFFSET)
#define SAM_XDMAC1_GRWR             (SAM_XDMAC1_VBASE+SAM_XDMAC_GRWR_OFFSET)
#define SAM_XDMAC1_GSWR             (SAM_XDMAC1_VBASE+SAM_XDMAC_GSWR_OFFSET)
#define SAM_XDMAC1_GSWS             (SAM_XDMAC1_VBASE+SAM_XDMAC_GSWS_OFFSET)
#define SAM_XDMAC1_GSWF             (SAM_XDMAC1_VBASE+SAM_XDMAC_GSWF_OFFSET)

#define SAM_XDMACH1_BASE(n)         (SAM_XDMAC1_VBASE+SAM_XDMACH_OFFSET(n))
#define SAM_XDMACH1_CIE(n)          (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CIE_OFFSET)
#define SAM_XDMACH1_CID(n)          (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CID_OFFSET)
#define SAM_XDMACH1_CIM(n)          (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CIM_OFFSET)
#define SAM_XDMACH1_CIS(n)          (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CIS_OFFSET)
#define SAM_XDMACH1_CSA(n)          (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CSA_OFFSET)
#define SAM_XDMACH1_CDA(n)          (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CDA_OFFSET)
#define SAM_XDMACH1_CNDA(n)         (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CNDA_OFFSET)
#define SAM_XDMACH1_CNDC(n)         (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CNDC_OFFSET)
#define SAM_XDMACH1_CUBC(n)         (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CUBC_OFFSET)
#define SAM_XDMACH1_CBC(n)          (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CBC_OFFSET)
#define SAM_XDMACH1_CC(n)           (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CC_OFFSET)
#define SAM_XDMACH1_CDSMSP(n)       (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CDSMSP_OFFSET)
#define SAM_XDMACH1_CSUS(n)         (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CSUS_OFFSET)
#define SAM_XDMACH1_CDUS(n)         (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CDUS_OFFSET)

/* XDMAC Register Bit Definitions ***************************************************/

/* Global Type Register */

#define XDMAC_GTYPE_NB_CH_SHIFT     (0)       /* Bits 0-4: Number of Channels Minus One */
#define XDMAC_GTYPE_NB_CH_MASK      (31 << XDMAC_GTYPE_NB_CH_SHIFT)
  #define XDMAC_GTYPE_NB_CH(n)      ((uint32_t)(n) << XDMAC_GTYPE_NB_CH_SHIFT)
#define XDMAC_GTYPE_FIFO_SZ_SHIFT   (5)       /* Bits 5-15: Number of Bytes */
#define XDMAC_GTYPE_FIFO_SZ_MASK    (0x7ff << XDMAC_GTYPE_FIFO_SZ_SHIFT)
#  define XDMAC_GTYPE_FIFO_SZ(n)    ((uint32_t)(n) << XDMAC_GTYPE_FIFO_SZ_SHIFT)
#define XDMAC_GTYPE_NB_REQ_SHIFT    (16)      /* Bits 16-22: Number of Peripheral Requests Minus One */
#define XDMAC_GTYPE_NB_REQ_MASK     (0x7f << XDMAC_GTYPE_NB_REQ_SHIFT)
#  define XDMAC_GTYPE_NB_REQ(n)     ((uint32_t)(n) << XDMAC_GTYPE_NB_REQ_SHIFT)

/* Global Configuration Register */

#define XDMAC_GCFG_CGDISREG         (1 << 0)  /* Bit 0:  Configuration Registers Clock Gating Disable */
#define XDMAC_GCFG_CGDISPIPE        (1 << 1)  /* Bit 1:  Pipeline Clock Gating Disable */
#define XDMAC_GCFG_CGDISFIFO        (1 << 2)  /* Bit 2:  FIFO Clock Gating Disable */
#define XDMAC_GCFG_CGDISIF          (1 << 3)  /* Bit 3:  Bus Interface Clock Gating Disable */
#define XDMAC_GCFG_BXKBEN           (1 << 8)  /* Bit 8:  Boundary X Kilo byte Enable */

/* Global Weighted Arbiter Configuration Register */

#define XDMAC_GWAC_PW0_SHIFT        (0)       /* Bits 0-3: Pool Weight 0 */
#define XDMAC_GWAC_PW0_MASK         (15 << XDMAC_GWAC_PW0_SHIFT)
#  define XDMAC_GWAC_PW0(n)         ((uint32_t)(n) << XDMAC_GWAC_PW0_SHIFT)
#define XDMAC_GWAC_PW1_SHIFT        (4)       /* Bits 4-7: Pool Weight 1 */
#define XDMAC_GWAC_PW1_MASK         (15 << XDMAC_GWAC_PW1_SHIFT)
#  define XDMAC_GWAC_PW1(n)         ((uint32_t)(n) << XDMAC_GWAC_PW1_SHIFT)
#define XDMAC_GWAC_PW2_SHIFT        (8)       /* Bits 8-11: Pool Weight 2 */
#define XDMAC_GWAC_PW2_MASK         (15 << XDMAC_GWAC_PW2_SHIFT)
#  define XDMAC_GWAC_PW2(n)         ((uint32_t)(n) << XDMAC_GWAC_PW2_SHIFT)
#define XDMAC_GWAC_PW3_SHIFT        (12)      /* Bits 12-15: Pool Weight 3 */
#define XDMAC_GWAC_PW3_MASK         (15 << XDMAC_GWAC_PW3_SHIFT)
#  define XDMAC_GWAC_PW3(n)         ((uint32_t)(n) << XDMAC_GWAC_PW3_SHIFT)

/* All of these registers have the same layout:
 *
 * - Global Interrupt Enable Register, Global Interrupt Disable Register, Interrupt
 *   Mask Register, and Global Interrupt Status Register.
 *
 * - Global Channel Enable Register, Global Channel Disable Register, and Global
 *   Channel Status Register
 *
 * - Global Channel Read Suspend Register, Global Channel Write Suspend Register,
 *   Channel Read Write Suspend Register, and Global Channel Read Write Resume
 *   Register
 *
 * - Global Channel Software Request Register, Global Channel Software Request
 *   Status Register, and Global Channel Software Flush Request Register
 */

#define XDMAC_CHAN(n)               (1 << (n))

/* Channel Interrupt Enable Register, Channel Interrupt Disable Register, Channel
 * Interrupt Mask Register, and Channel Interrupt Status Register.
 */

#define XDMAC_CHINT_BI              (1 << 0)  /* Bit 0:  End of Block Interrupt  */
#define XDMAC_CHINT_LI              (1 << 1)  /* Bit 1:  End of Linked List Interrupt  */
#define XDMAC_CHINT_DI              (1 << 2)  /* Bit 2:  End of Disable Interrupt  */
#define XDMAC_CHINT_FI              (1 << 3)  /* Bit 3:  End of Flush Interrupt  */
#define XDMAC_CHINT_RBI             (1 << 4)  /* Bit 4:  Read Bus Error Interrupt  */
#define XDMAC_CHINT_WBI             (1 << 5)  /* Bit 5:  Write Bus Error Interrupt */ 
#define XDMAC_CHINT_ROI             (1 << 6)  /* Bit 6:  Request Overflow Error Interrupt Disable Bit */

/* Channel Source Address Register (32-bit address) */
/* Channel Destination Address Register (32-bit address) */
/* Channel Next Descriptor Address Register (32-bit address) */

/* Channel Next Descriptor Control Register */

#define XDMACH_CNDC_NDE             (1 << 0)  /* Bit 0:  Channel x Next Descriptor Enable */
#define XDMACH_CNDC_NDSUP           (1 << 1)  /* Bit 1:  Channel x Next Descriptor Source Update */
#define XDMACH_CNDC_NDDUP           (1 << 2)  /* Bit 2:  Channel x Next Descriptor Destination Update */
#define XDMACH_CNDC_NDVIEW_SHIFT    (3)       /* Bits 3-4: Channel x Next Descriptor View */
#define XDMACH_CNDC_NDVIEW_MASK     (3 << XDMACH_CNDC_NDVIEW_SHIFT)
#  define XDMACH_CNDC_NDVIEW_NDV0   (0 << XDMACH_CNDC_NDVIEW_SHIFT) /* Next Descriptor View 0 */
#  define XDMACH_CNDC_NDVIEW_NDV1   (1 << XDMACH_CNDC_NDVIEW_SHIFT) /* Next Descriptor View 1 */
#  define XDMACH_CNDC_NDVIEW_NDV2   (2 << XDMACH_CNDC_NDVIEW_SHIFT) /* Next Descriptor View 2 */
#  define XDMACH_CNDC_NDVIEW_NDV3   (3 << XDMACH_CNDC_NDVIEW_SHIFT) /* Next Descriptor View 3 */

/* Channel Microblock Control Register */

#define XDMACH_CUBC_UBLEN_MASK      (0x00ffffff)       /* Bits 0-23: Channel x Microblock Length */

/* Channel Block Control Register */

#define XDMACH_CBC_BLEN_MASK        (0x000000fff)       /* Bits 0-11: Channel x Block Length */

/* Channel Configuration Register */

#define XDMACH_CC_TYPE              (1 << 0)  /* Bit 0:  Channel x Transfer Type */
#define XDMACH_CC_MBSIZE_SHIFT      (1)       /* Bits 1-2: Channel x Memory Burst Size */
#define XDMACH_CC_MBSIZE_MASK       (3 << XDMACH_CC_MBSIZE_SHIFT)
#  define XDMACH_CC_MBSIZE_1        (0 << XDMACH_CC_MBSIZE_SHIFT) /* The memory burst size is set to one */
#  define XDMACH_CC_MBSIZE_4        (1 << XDMACH_CC_MBSIZE_SHIFT) /* The memory burst size is set to four */
#  define XDMACH_CC_MBSIZE_8        (2 << XDMACH_CC_MBSIZE_SHIFT) /* The memory burst size is set to eight */
#  define XDMACH_CC_MBSIZE_16       (3 << XDMACH_CC_MBSIZE_SHIFT) /* The memory burst size is set to sixteen */
#define XDMACH_CC_DSYNC             (1 << 4)  /* Bit 4:  Channel x Synchronization */
#define XDMACH_CC_PROT              (1 << 5)  /* Bit 5:  Channel x Protection */
#define XDMACH_CC_SWREQ             (1 << 6)  /* Bit 6:  Channel x Software Request Trigger */
#define XDMACH_CC_MEMSET            (1 << 7)  /* Bit 7:  Channel x Fill Block of memory */
#define XDMACH_CC_CSIZE_SHIFT       (8)       /* Bits 8-10: Channel x Chunk Size */
#define XDMACH_CC_CSIZE_MASK        (7 << XDMACH_CC_CSIZE_SHIFT)
#  define XDMACH_CC_CSIZE_1         (0 << XDMACH_CC_CSIZE_SHIFT) /* 1 data transferred */
#  define XDMACH_CC_CSIZE_2         (1 << XDMACH_CC_CSIZE_SHIFT) /* 2 data transferred */
#  define XDMACH_CC_CSIZE_4         (2 << XDMACH_CC_CSIZE_SHIFT) /* 4 data transferred */
#  define XDMACH_CC_CSIZE_8         (3 << XDMACH_CC_CSIZE_SHIFT) /* 8 data transferred */
#  define XDMACH_CC_CSIZE_16        (4 << XDMACH_CC_CSIZE_SHIFT) /* 16 data transferred */
#define XDMACH_CC_DWIDTH_SHIFT      (11)      /* Bits 11-12: Channel x Data Width */
#define XDMACH_CC_DWIDTH_MASK       (3 << XDMACH_CC_DWIDTH_SHIFT)
#  define XDMACH_CC_DWIDTH_BYTE     (0 << XDMACH_CC_DWIDTH_SHIFT) /* The data size is set to 8 bits */
#  define XDMACH_CC_DWIDTH_HWORD    (1 << XDMACH_CC_DWIDTH_SHIFT) /* The data size is set to 16 bits */
#  define XDMACH_CC_DWIDTH_WORD     (2 << XDMACH_CC_DWIDTH_SHIFT) /* The data size is set to 32 bits */
#  define XDMACH_CC_DWIDTH_DWORD    (3 << XDMACH_CC_DWIDTH_SHIFT) /* The data size is set to 64 bits */
#define XDMACH_CC_SIF               (1 << 13) /* Bit 13: Channel x Source Interface Identifier */
#define XDMACH_CC_DIF               (1 << 14) /* Bit 14: Channel x Destination Interface Identifier */
#define XDMACH_CC_SAM_SHIFT         (16)      /* Bits 16-17: Channel x Source Addressing Mode */
#define XDMACH_CC_SAM_MASK          (3 << XDMACH_CC_SAM_SHIFT)
#  define XDMACH_CC_SAM_FIXED       (0 << XDMACH_CC_SAM_SHIFT) /* The address remains unchanged */
#  define XDMACH_CC_SAM_INCR        (1 << XDMACH_CC_SAM_SHIFT) /* Address is incremented */
#  define XDMACH_CC_SAM_UBS         (2 << XDMACH_CC_SAM_SHIFT) /* Microblock stride is added */
#  define XDMACH_CC_SAM_UBSDS       (3 << XDMACH_CC_SAM_SHIFT) /* Microblock stride and data stride is added */
#define XDMACH_CC_DAM_SHIFT         (18)      /* Bits 18-19: Channel x Destination Addressing Mode */
#define XDMACH_CC_DAM_MASK          (3 << XDMACH_CC_DAM_SHIFT)
#  define XDMACH_CC_DAM_FIXED       (0 << XDMACH_CC_DAM_SHIFT) /* The address remains unchanged */
#  define XDMACH_CC_DAM_INCR        (1 << XDMACH_CC_DAM_SHIFT) /* Address is incremented */
#  define XDMACH_CC_DAM_UBS         (2 << XDMACH_CC_DAM_SHIFT) /* Microblock stride is added */
#  define XDMACH_CC_DAM_UBSDS       (3 << XDMACH_CC_DAM_SHIFT) /* Microblock stride and data stride is added */
#define XDMACH_CC_INITD             (1 << 21) /* Bit 21: Channel Initialization Terminated */
#define XDMACH_CC_RDIP              (1 << 22) /* Bit 22: Read in Progress */
#define XDMACH_CC_WRIP              (1 << 23) /* Bit 23: Write in Progress */
#define XDMACH_CC_PERID_SHIFT       (24)      /* Bits 24-30: Channel x Peripheral Identifier */
#define XDMACH_CC_PERID_MASK        (0x7f << XDMACH_CC_PERID_SHIFT)
#  define XDMACH_CC_PERID(n)        ((uint32_t)(n) << XDMACH_CC_PERID_SHIFT)

/* Channel Data Stride Memory Set Pattern */

#define XDMACH_CDSMSP_SDS_MSP_SHIFT (0)     /* Bits 0-15: Channel x Source Data stride or Memory Set Pattern */
#define XDMACH_CDSMSP_SDS_MSP_MASK  (0xffff << XDMACH_CDSMSP_SDS_MSP_SHIFT)
#  define XDMACH_CDSMSP_SDS_MSP(n)  ((uint32_t)(n) << XDMACH_CDSMSP_SDS_MSP_SHIFT)
#define XDMACH_CDSMSP_DDS_MSP_SHIFT (16)    /* Bits 16-31: Channel x Destination Data Stride or Memory Set Pattern */
#define XDMACH_CDSMSP_DDS_MSP_MASK  (0xffff << XDMACH_CDSMSP_DDS_MSP_SHIFT)
#  define XDMACH_CDSMSP_DDS_MSP(n)  ((uint32_t)(n) << XDMACH_CDSMSP_DDS_MSP_SHIFT)

/* Channel Source Microblock Stride */

#define XDMACH_CSUS_SUBS_MASK       (0x00ffffff)       /* Bits 0-23: Channel x Source Microblock Stride */

/* Channel Destination Microblock Stride */

#define XDMACH_CDUS_DUBS_MASK       (0x00ffffff)       /* Bits 0-23: Channel x Destination Microblock Stride */

#endif /* __ARCH_ARM_SRC_SAMA5_CHIP_SAM_XDMAC_H */
