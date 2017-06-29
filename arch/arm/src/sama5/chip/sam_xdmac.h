/************************************************************************************
 * arch/arm/src/sama5/chip/sam_xdmac.h
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

/* Offsets to the base of the DMA channel registers */

#define SAM_XDMAC_CH_OFFSET(n)     (0x0050 + ((n) << 6))
#  define SAM_XDMAC_CH0_OFFSET     0x0050
#  define SAM_XDMAC_CH1_OFFSET     0x0090
#  define SAM_XDMAC_CH2_OFFSET     0x00d0
#  define SAM_XDMAC_CH3_OFFSET     0x0110
#  define SAM_XDMAC_CH4_OFFSET     0x0150
#  define SAM_XDMAC_CH5_OFFSET     0x0190
#  define SAM_XDMAC_CH6_OFFSET     0x01d0
#  define SAM_XDMAC_CH7_OFFSET     0x0210
#  define SAM_XDMAC_CH8_OFFSET     0x0250
#  define SAM_XDMAC_CH9_OFFSET     0x0290
#  define SAM_XDMAC_CH10_OFFSET    0x02d0
#  define SAM_XDMAC_CH11_OFFSET    0x0310
#  define SAM_XDMAC_CH12_OFFSET    0x0350
#  define SAM_XDMAC_CH13_OFFSET    0x0390
#  define SAM_XDMAC_CH14_OFFSET    0x03d0
#  define SAM_XDMAC_CH15_OFFSET    0x0410

/* Offsets to channel registers relative to the base of the DMA channel registers */

#define SAM_XDMACH_CIE_OFFSET       0x0000 /* Channel Interrupt Enable Register */
#define SAM_XDMACH_CID_OFFSET       0x0004 /* Channel Interrupt Disable Register */
#define SAM_XDMACH_CIM_OFFSET       0x0008 /* Channel Interrupt Mask Register */
#define SAM_XDMACH_CIS_OFFSET       0x000c /* Channel Interrupt Status Register */
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

/* Base addresses of XDMAC0 channel registers */

#define SAM_XDMAC0_CH_BASE(n)       (SAM_XDMAC0_VBASE+SAM_XDMAC_CH_OFFSET(n))
#  define SAM_XDMAC0_CH0_BASE       (SAM_XDMAC0_VBASE+SAM_XDMAC_CH0_OFFSET)
#  define SAM_XDMAC0_CH1_BASE       (SAM_XDMAC0_VBASE+SAM_XDMAC_CH1_OFFSET)
#  define SAM_XDMAC0_CH2_BASE       (SAM_XDMAC0_VBASE+SAM_XDMAC_CH2_OFFSET)
#  define SAM_XDMAC0_CH3_BASE       (SAM_XDMAC0_VBASE+SAM_XDMAC_CH3_OFFSET)
#  define SAM_XDMAC0_CH4_BASE       (SAM_XDMAC0_VBASE+SAM_XDMAC_CH4_OFFSET)
#  define SAM_XDMAC0_CH5_BASE       (SAM_XDMAC0_VBASE+SAM_XDMAC_CH5_OFFSET)
#  define SAM_XDMAC0_CH6_BASE       (SAM_XDMAC0_VBASE+SAM_XDMAC_CH6_OFFSET)
#  define SAM_XDMAC0_CH7_BASE       (SAM_XDMAC0_VBASE+SAM_XDMAC_CH7_OFFSET)
#  define SAM_XDMAC0_CH8_BASE       (SAM_XDMAC0_VBASE+SAM_XDMAC_CH8_OFFSET)
#  define SAM_XDMAC0_CH9_BASE       (SAM_XDMAC0_VBASE+SAM_XDMAC_CH9_OFFSET)
#  define SAM_XDMAC0_CH10_BASE      (SAM_XDMAC0_VBASE+SAM_XDMAC_CH10_OFFSET)
#  define SAM_XDMAC0_CH11_BASE      (SAM_XDMAC0_VBASE+SAM_XDMAC_CH11_OFFSET)
#  define SAM_XDMAC0_CH12_BASE      (SAM_XDMAC0_VBASE+SAM_XDMAC_CH12_OFFSET)
#  define SAM_XDMAC0_CH13_BASE      (SAM_XDMAC0_VBASE+SAM_XDMAC_CH13_OFFSET)
#  define SAM_XDMAC0_CH14_BASE      (SAM_XDMAC0_VBASE+SAM_XDMAC_CH14_OFFSET)
#  define SAM_XDMAC0_CH15_BASE      (SAM_XDMAC0_VBASE+SAM_XDMAC_CH15_OFFSET)

/* Addresses of XDMAC0 channel registers */

#define SAM_XDMACH0_CIE(n)          (SAM_XDMACH0_CH_BASE(n)+SAM_XDMACH_CIE_OFFSET)
#define SAM_XDMACH0_CID(n)          (SAM_XDMACH0_CH_BASE(n)+SAM_XDMACH_CID_OFFSET)
#define SAM_XDMACH0_CIM(n)          (SAM_XDMACH0_CH_BASE(n)+SAM_XDMACH_CIM_OFFSET)
#define SAM_XDMACH0_CIS(n)          (SAM_XDMACH0_CH_BASE(n)+SAM_XDMACH_CIS_OFFSET)
#define SAM_XDMACH0_CSA(n)          (SAM_XDMACH0_CH_BASE(n)+SAM_XDMACH_CSA_OFFSET)
#define SAM_XDMACH0_CDA(n)          (SAM_XDMACH0_CH_BASE(n)+SAM_XDMACH_CDA_OFFSET)
#define SAM_XDMACH0_CNDA(n)         (SAM_XDMACH0_CH_BASE(n)+SAM_XDMACH_CNDA_OFFSET)
#define SAM_XDMACH0_CNDC(n)         (SAM_XDMACH0_CH_BASE(n)+SAM_XDMACH_CNDC_OFFSET)
#define SAM_XDMACH0_CUBC(n)         (SAM_XDMACH0_CH_BASE(n)+SAM_XDMACH_CUBC_OFFSET)
#define SAM_XDMACH0_CBC(n)          (SAM_XDMACH0_CH_BASE(n)+SAM_XDMACH_CBC_OFFSET)
#define SAM_XDMACH0_CC(n)           (SAM_XDMACH0_CH_BASE(n)+SAM_XDMACH_CC_OFFSET)
#define SAM_XDMACH0_CDSMSP(n)       (SAM_XDMACH0_CH_BASE(n)+SAM_XDMACH_CDSMSP_OFFSET)
#define SAM_XDMACH0_CSUS(n)         (SAM_XDMACH0_CH_BASE(n)+SAM_XDMACH_CSUS_OFFSET)
#define SAM_XDMACH0_CDUS(n)         (SAM_XDMACH0_CH_BASE(n)+SAM_XDMACH_CDUS_OFFSET)

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

/* Base addresses of XDMAC1 channel registers */

#define SAM_XDMAC1_CH_BASE(n)       (SAM_XDMAC1_VBASE+SAM_XDMAC_CH_OFFSET(n))
#  define SAM_XDMAC1_CH0_BASE       (SAM_XDMAC1_VBASE+SAM_XDMAC_CH0_OFFSET)
#  define SAM_XDMAC1_CH1_BASE       (SAM_XDMAC1_VBASE+SAM_XDMAC_CH1_OFFSET)
#  define SAM_XDMAC1_CH2_BASE       (SAM_XDMAC1_VBASE+SAM_XDMAC_CH2_OFFSET)
#  define SAM_XDMAC1_CH3_BASE       (SAM_XDMAC1_VBASE+SAM_XDMAC_CH3_OFFSET)
#  define SAM_XDMAC1_CH4_BASE       (SAM_XDMAC1_VBASE+SAM_XDMAC_CH4_OFFSET)
#  define SAM_XDMAC1_CH5_BASE       (SAM_XDMAC1_VBASE+SAM_XDMAC_CH5_OFFSET)
#  define SAM_XDMAC1_CH6_BASE       (SAM_XDMAC1_VBASE+SAM_XDMAC_CH6_OFFSET)
#  define SAM_XDMAC1_CH7_BASE       (SAM_XDMAC1_VBASE+SAM_XDMAC_CH7_OFFSET)
#  define SAM_XDMAC1_CH8_BASE       (SAM_XDMAC1_VBASE+SAM_XDMAC_CH8_OFFSET)
#  define SAM_XDMAC1_CH9_BASE       (SAM_XDMAC1_VBASE+SAM_XDMAC_CH9_OFFSET)
#  define SAM_XDMAC1_CH10_BASE      (SAM_XDMAC1_VBASE+SAM_XDMAC_CH10_OFFSET)
#  define SAM_XDMAC1_CH11_BASE      (SAM_XDMAC1_VBASE+SAM_XDMAC_CH11_OFFSET)
#  define SAM_XDMAC1_CH12_BASE      (SAM_XDMAC1_VBASE+SAM_XDMAC_CH12_OFFSET)
#  define SAM_XDMAC1_CH13_BASE      (SAM_XDMAC1_VBASE+SAM_XDMAC_CH13_OFFSET)
#  define SAM_XDMAC1_CH14_BASE      (SAM_XDMAC1_VBASE+SAM_XDMAC_CH14_OFFSET)
#  define SAM_XDMAC1_CH15_BASE      (SAM_XDMAC1_VBASE+SAM_XDMAC_CH15_OFFSET)

/* Addresses of XDMAC0 channel registers */

#define SAM_XDMACH1_CIE(n)          (SAM_XDMACH1_CH_BASE(n)+SAM_XDMACH_CIE_OFFSET)
#define SAM_XDMACH1_CID(n)          (SAM_XDMACH1_CH_BASE(n)+SAM_XDMACH_CID_OFFSET)
#define SAM_XDMACH1_CIM(n)          (SAM_XDMACH1_CH_BASE(n)+SAM_XDMACH_CIM_OFFSET)
#define SAM_XDMACH1_CIS(n)          (SAM_XDMACH1_CH_BASE(n)+SAM_XDMACH_CIS_OFFSET)
#define SAM_XDMACH1_CSA(n)          (SAM_XDMACH1_CH_BASE(n)+SAM_XDMACH_CSA_OFFSET)
#define SAM_XDMACH1_CDA(n)          (SAM_XDMACH1_CH_BASE(n)+SAM_XDMACH_CDA_OFFSET)
#define SAM_XDMACH1_CNDA(n)         (SAM_XDMACH1_CH_BASE(n)+SAM_XDMACH_CNDA_OFFSET)
#define SAM_XDMACH1_CNDC(n)         (SAM_XDMACH1_CH_BASE(n)+SAM_XDMACH_CNDC_OFFSET)
#define SAM_XDMACH1_CUBC(n)         (SAM_XDMACH1_CH_BASE(n)+SAM_XDMACH_CUBC_OFFSET)
#define SAM_XDMACH1_CBC(n)          (SAM_XDMACH1_CH_BASE(n)+SAM_XDMACH_CBC_OFFSET)
#define SAM_XDMACH1_CC(n)           (SAM_XDMACH1_CH_BASE(n)+SAM_XDMACH_CC_OFFSET)
#define SAM_XDMACH1_CDSMSP(n)       (SAM_XDMACH1_CH_BASE(n)+SAM_XDMACH_CDSMSP_OFFSET)
#define SAM_XDMACH1_CSUS(n)         (SAM_XDMACH1_CH_BASE(n)+SAM_XDMACH_CSUS_OFFSET)
#define SAM_XDMACH1_CDUS(n)         (SAM_XDMACH1_CH_BASE(n)+SAM_XDMACH_CDUS_OFFSET)

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
#define XDMAC_CHAN_ALL              (0x0000ffff)

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

#define XDMAC_CHINT_ERRORS          (0x00000070)
#define XDMAC_CHINT_ALL             (0x0000007f)

/* Channel Source Address (SA) Register (aligned 32-bit address) */
/* Channel Destination Address (DA) Register (aligned 32-bit address) */

/* Channel Next Descriptor Address (CNDA) Register (aligned 32-bit address) */

#define XDMACH_CNDA_NDAIF           (1 << 0)  /* Bit 0:  Channel Next Descriptor Interface */

/* Channel Next Descriptor Control Register */

#define XDMACH_CNDC_NDE             (1 << 0)  /* Bit 0:  Channel Next Descriptor Enable */
#define XDMACH_CNDC_NDSUP           (1 << 1)  /* Bit 1:  Channel Next Descriptor Source Update */
#define XDMACH_CNDC_NDDUP           (1 << 2)  /* Bit 2:  Channel Next Descriptor Destination Update */
#define XDMACH_CNDC_NDVIEW_SHIFT    (3)       /* Bits 3-4: Channel Next Descriptor View */
#define XDMACH_CNDC_NDVIEW_MASK     (3 << XDMACH_CNDC_NDVIEW_SHIFT)
#  define XDMACH_CNDC_NDVIEW_NDV0   (0 << XDMACH_CNDC_NDVIEW_SHIFT) /* Next Descriptor View 0 */
#  define XDMACH_CNDC_NDVIEW_NDV1   (1 << XDMACH_CNDC_NDVIEW_SHIFT) /* Next Descriptor View 1 */
#  define XDMACH_CNDC_NDVIEW_NDV2   (2 << XDMACH_CNDC_NDVIEW_SHIFT) /* Next Descriptor View 2 */
#  define XDMACH_CNDC_NDVIEW_NDV3   (3 << XDMACH_CNDC_NDVIEW_SHIFT) /* Next Descriptor View 3 */

/* Channel Microblock Control Register */

#define XDMACH_CUBC_UBLEN_SHIFT     (0)       /* Bits 0-23: Channel Microblock Length */
#define XDMACH_CUBC_UBLEN_MASK      (0x00ffffff << XDMACH_CUBC_UBLEN_SHIFT)
#define XDMACH_CUBC_UBLEN_MAX       (0x00ffffff)

/* Channel Block Control Register */

#define XDMACH_CBC_BLEN_MASK        (0x000000fff)       /* Bits 0-11: Channel Block Length */

/* Channel Configuration Register */

#define XDMACH_CC_TYPE              (1 << 0)  /* Bit 0:  Channel Transfer Type */
#define XDMACH_CC_MBSIZE_SHIFT      (1)       /* Bits 1-2: Channel Memory Burst Size */
#define XDMACH_CC_MBSIZE_MASK       (3 << XDMACH_CC_MBSIZE_SHIFT)
#  define XDMACH_CC_MBSIZE(n)       ((uint32_t)(n) << XDMACH_CC_MBSIZE_SHIFT) /* n=0-3 */
#  define XDMACH_CC_MBSIZE_1        (0 << XDMACH_CC_MBSIZE_SHIFT) /* The memory burst size is set to one */
#  define XDMACH_CC_MBSIZE_4        (1 << XDMACH_CC_MBSIZE_SHIFT) /* The memory burst size is set to four */
#  define XDMACH_CC_MBSIZE_8        (2 << XDMACH_CC_MBSIZE_SHIFT) /* The memory burst size is set to eight */
#  define XDMACH_CC_MBSIZE_16       (3 << XDMACH_CC_MBSIZE_SHIFT) /* The memory burst size is set to sixteen */
#define XDMACH_CC_DSYNC             (1 << 4)  /* Bit 4:  Channel Synchronization */
#define XDMACH_CC_PROT              (1 << 5)  /* Bit 5:  Channel Protection */
#define XDMACH_CC_SWREQ             (1 << 6)  /* Bit 6:  Channel Software Request Trigger */
#define XDMACH_CC_MEMSET            (1 << 7)  /* Bit 7:  Channel Fill Block of memory */
#define XDMACH_CC_CSIZE_SHIFT       (8)       /* Bits 8-10: Channel Chunk Size */
#define XDMACH_CC_CSIZE_MASK        (7 << XDMACH_CC_CSIZE_SHIFT)
#  define XDMACH_CC_CSIZE_1         (0 << XDMACH_CC_CSIZE_SHIFT) /* 1 data transferred */
#  define XDMACH_CC_CSIZE_2         (1 << XDMACH_CC_CSIZE_SHIFT) /* 2 data transferred */
#  define XDMACH_CC_CSIZE_4         (2 << XDMACH_CC_CSIZE_SHIFT) /* 4 data transferred */
#  define XDMACH_CC_CSIZE_8         (3 << XDMACH_CC_CSIZE_SHIFT) /* 8 data transferred */
#  define XDMACH_CC_CSIZE_16        (4 << XDMACH_CC_CSIZE_SHIFT) /* 16 data transferred */
#define XDMACH_CC_DWIDTH_SHIFT      (11)      /* Bits 11-12: Channel Data Width */
#define XDMACH_CC_DWIDTH_MASK       (3 << XDMACH_CC_DWIDTH_SHIFT)
#  define XDMACH_CC_DWIDTH_BYTE     (0 << XDMACH_CC_DWIDTH_SHIFT) /* The data size is set to 8 bits */
#  define XDMACH_CC_DWIDTH_HWORD    (1 << XDMACH_CC_DWIDTH_SHIFT) /* The data size is set to 16 bits */
#  define XDMACH_CC_DWIDTH_WORD     (2 << XDMACH_CC_DWIDTH_SHIFT) /* The data size is set to 32 bits */
#  define XDMACH_CC_DWIDTH_DWORD    (3 << XDMACH_CC_DWIDTH_SHIFT) /* The data size is set to 64 bits */
#define XDMACH_CC_SIF               (1 << 13) /* Bit 13: Channel Source Interface Identifier */
#define XDMACH_CC_DIF               (1 << 14) /* Bit 14: Channel Destination Interface Identifier */
#define XDMACH_CC_SAM_SHIFT         (16)      /* Bits 16-17: Channel Source Addressing Mode */
#define XDMACH_CC_SAM_MASK          (3 << XDMACH_CC_SAM_SHIFT)
#  define XDMACH_CC_SAM_FIXED       (0 << XDMACH_CC_SAM_SHIFT) /* The address remains unchanged */
#  define XDMACH_CC_SAM_INCR        (1 << XDMACH_CC_SAM_SHIFT) /* Address is incremented */
#  define XDMACH_CC_SAM_UBS         (2 << XDMACH_CC_SAM_SHIFT) /* Microblock stride is added */
#  define XDMACH_CC_SAM_UBSDS       (3 << XDMACH_CC_SAM_SHIFT) /* Microblock stride and data stride is added */
#define XDMACH_CC_DAM_SHIFT         (18)      /* Bits 18-19: Channel Destination Addressing Mode */
#define XDMACH_CC_DAM_MASK          (3 << XDMACH_CC_DAM_SHIFT)
#  define XDMACH_CC_DAM_FIXED       (0 << XDMACH_CC_DAM_SHIFT) /* The address remains unchanged */
#  define XDMACH_CC_DAM_INCR        (1 << XDMACH_CC_DAM_SHIFT) /* Address is incremented */
#  define XDMACH_CC_DAM_UBS         (2 << XDMACH_CC_DAM_SHIFT) /* Microblock stride is added */
#  define XDMACH_CC_DAM_UBSDS       (3 << XDMACH_CC_DAM_SHIFT) /* Microblock stride and data stride is added */
#define XDMACH_CC_INITD             (1 << 21) /* Bit 21: Channel Initialization Terminated */
#define XDMACH_CC_RDIP              (1 << 22) /* Bit 22: Read in Progress */
#define XDMACH_CC_WRIP              (1 << 23) /* Bit 23: Write in Progress */
#define XDMACH_CC_PERID_SHIFT       (24)      /* Bits 24-30: Channel Peripheral Identifier */
#define XDMACH_CC_PERID_MASK        (0x7f << XDMACH_CC_PERID_SHIFT)
#  define XDMACH_CC_PERID(n)        ((uint32_t)(n) << XDMACH_CC_PERID_SHIFT)

/* Channel Data Stride Memory Set Pattern */

#define XDMACH_CDSMSP_SDS_MSP_SHIFT (0)     /* Bits 0-15: Channel Source Data stride or Memory Set Pattern */
#define XDMACH_CDSMSP_SDS_MSP_MASK  (0xffff << XDMACH_CDSMSP_SDS_MSP_SHIFT)
#  define XDMACH_CDSMSP_SDS_MSP(n)  ((uint32_t)(n) << XDMACH_CDSMSP_SDS_MSP_SHIFT)
#define XDMACH_CDSMSP_DDS_MSP_SHIFT (16)    /* Bits 16-31: Channel Destination Data Stride or Memory Set Pattern */
#define XDMACH_CDSMSP_DDS_MSP_MASK  (0xffff << XDMACH_CDSMSP_DDS_MSP_SHIFT)
#  define XDMACH_CDSMSP_DDS_MSP(n)  ((uint32_t)(n) << XDMACH_CDSMSP_DDS_MSP_SHIFT)

/* Channel Source Microblock Stride */

#define XDMACH_CSUS_SUBS_MASK       (0x00ffffff)       /* Bits 0-23: Channel Source Microblock Stride */

/* Channel Destination Microblock Stride */

#define XDMACH_CDUS_DUBS_MASK       (0x00ffffff)       /* Bits 0-23: Channel Destination Microblock Stride */

/* XDMA Channel Definitions *************************************************************/
/* XDMA Controller 0 Channel Definitions (always secure) */

#define XDMAC0_CH_HSMCI0            0  /* HSMCI0 Receive/Transmit */
#define XDMAC0_CH_HSMCI1            1  /* HSMCI1 Receive/Transmit */
#define XDMAC0_CH_TWI0_TX           2  /* TWI0 Transmit */
#define XDMAC0_CH_TWI0_RX           3  /* TWI0 Receive */
#define XDMAC0_CH_TWI1_TX           4  /* TWI1 Transmit */
#define XDMAC0_CH_TWI1_RX           5  /* TWI1 Receive */
#define XDMAC0_CH_TWI2_TX           6  /* TWI2 Transmit */
#define XDMAC0_CH_TWI2_RX           7  /* TWI2 Receive */
#define XDMAC0_CH_TWI3_TX           8  /* TWI3 Transmit */
#define XDMAC0_CH_TWI3_RX           9  /* TWI3 Receive */
#define XDMAC0_CH_SPI0_TX           10 /* SPI0 Transmit */
#define XDMAC0_CH_SPI0_RX           11 /* SPI0 Receive */
#define XDMAC0_CH_SPI1_TX           12 /* SPI1 Transmit */
#define XDMAC0_CH_SPI1_RX           13 /* SPI1 Receive */
#define XDMAC0_CH_SPI2_TX           14 /* SPI2 Transmit */
#define XDMAC0_CH_SPI2_RX           15 /* SPI2 Receive */
#define XDMAC0_CH_USART2_TX         16 /* USART2 Transmit */
#define XDMAC0_CH_USART2_RX         17 /* USART2 Receive */
#define XDMAC0_CH_USART3_TX         18 /* USART3 Transmit */
#define XDMAC0_CH_USART3_RX         19 /* USART3 Receive */
#define XDMAC0_CH_USART4_TX         20 /* USART4 Transmit */
#define XDMAC0_CH_USART4_RX         21 /* USART4 Receive */
#define XDMAC0_CH_UART0_TX          22 /* UART0 Transmit */
#define XDMAC0_CH_UART0_RX          23 /* UART0 Receive */
#define XDMAC0_CH_UART1_TX          24 /* UART1 Transmit */
#define XDMAC0_CH_UART1_RX          25 /* UART1 Receive */
#define XDMAC0_CH_SSC0_TX           26 /* SSC0 Transmit */
#define XDMAC0_CH_SSC0_RX           27 /* SSC0 Receive */
#define XDMAC0_CH_SSC1_TX           28 /* SSC1 Transmit */
#define XDMAC0_CH_SSC1_RX           29 /* SSC1 Receive */
#define XDMAC0_CH_DBGU_TX           30 /* DBGU Transmit */
#define XDMAC0_CH_DBGU_RX           31 /* DBGU Receive */
#define XDMAC0_CH_ADC_RX            32 /* ADC Receive */
#define XDMAC0_CH_SMD_TX            33 /* SMD Transmit */
#define XDMAC0_CH_SMD_RX            34 /* SMD Receive */
#define XDMAC0_CH_USART0_TX         36 /* USART0 Transmit */
#define XDMAC0_CH_USART0_RX         37 /* USART0 Receive */
#define XDMAC0_CH_USART1_TX         38 /* USART1 Transmit */
#define XDMAC0_CH_USART1_RX         39 /* USART1 Receive */
#define XDMAC0_CH_AES_RX            40 /* AES Receive */
#define XDMAC0_CH_AES_TX            41 /* AES Transmit */
#define XDMAC0_CH_TDES_TX           42 /* TDES Transmit */
#define XDMAC0_CH_TDES_RX           43 /* TDES Receive */
#define XDMAC0_CH_SHA_TX            44 /* SHA Transmit */
#define XDMAC0_CH_CATB_TX           46 /* CATB Transmit */
#define XDMAC0_CH_CATB_RX           47 /* CATB Receive */

/* XDMA Controller 1 Channel Definitions (never secure) */

#define XDMAC1_CH_HSMCI0            0  /* HSMCI0 Receive/Transmit */
#define XDMAC1_CH_HSMCI1            1  /* HSMCI1 Receive/Transmit */
#define XDMAC1_CH_TWI0_TX           2  /* TWI0 Transmit */
#define XDMAC1_CH_TWI0_RX           3  /* TWI0 Receive */
#define XDMAC1_CH_TWI1_TX           4  /* TWI1 Transmit */
#define XDMAC1_CH_TWI1_RX           5  /* TWI1 Receive */
#define XDMAC1_CH_TWI2_TX           6  /* TWI2 Transmit */
#define XDMAC1_CH_TWI2_RX           7  /* TWI2 Receive */
#define XDMAC1_CH_TWI3_TX           8  /* TWI3 Transmit */
#define XDMAC1_CH_TWI3_RX           9  /* TWI3 Receive */
#define XDMAC1_CH_SPI0_TX           10 /* SPI0 Transmit */
#define XDMAC1_CH_SPI0_RX           11 /* SPI0 Receive */
#define XDMAC1_CH_SPI1_TX           12 /* SPI1 Transmit */
#define XDMAC1_CH_SPI1_RX           13 /* SPI1 Receive */
#define XDMAC1_CH_SPI2_TX           14 /* SPI2 Transmit */
#define XDMAC1_CH_SPI2_RX           15 /* SPI2 Receive */
#define XDMAC1_CH_USART2_TX         16 /* USART2 Transmit */
#define XDMAC1_CH_USART2_RX         17 /* USART2 Receive */
#define XDMAC1_CH_USART3_TX         18 /* USART3 Transmit */
#define XDMAC1_CH_USART3_RX         19 /* USART3 Receive */
#define XDMAC1_CH_USART4_TX         20 /* USART4 Transmit */
#define XDMAC1_CH_USART4_RX         21 /* USART4 Receive */
#define XDMAC1_CH_UART0_TX          22 /* UART0 Transmit */
#define XDMAC1_CH_UART0_RX          23 /* UART0 Receive */
#define XDMAC1_CH_UART1_TX          24 /* UART1 Transmit */
#define XDMAC1_CH_UART1_RX          25 /* UART1 Receive */
#define XDMAC1_CH_SSC0_TX           26 /* SSC0 Transmit */
#define XDMAC1_CH_SSC0_RX           27 /* SSC0 Receive */
#define XDMAC1_CH_SSC1_TX           28 /* SSC1 Transmit */
#define XDMAC1_CH_SSC1_RX           29 /* SSC1 Receive */
#define XDMAC1_CH_DBGU_TX           30 /* DBGU Transmit */
#define XDMAC1_CH_DBGU_RX           31 /* DBGU Receive */
#define XDMAC1_CH_ADC_RX            32 /* ADC Receive */
#define XDMAC1_CH_SMD_TX            33 /* SMD Transmit */
#define XDMAC1_CH_SMD_RX            34 /* SMD Receive */

/* Descriptor structure member definitions **********************************************/

/* Next Descriptor Address (32-bit address) */

/* Microblock Control */

#define CHNEXT_UBC_UBLEN_SHIFT      (0)       /* Bits 0-23: Microblock Length */
#define CHNEXT_UBC_UBLEN_MASK       (0x00ffffff << CHNEXT_UBC_UBLEN_SHIFT)
#  define CHNEXT_UBC_UBLEN(n)       ((uint32_t)(n) << CHNEXT_UBC_UBLEN_SHIFT)
#define CHNEXT_UBC_NDE              (1 << 24) /* Bit 24: Next Descriptor Enable */
#define CHNEXT_UBC_NSEN             (1 << 25) /* Bit 25: Next Descriptor Source Update */
#define CHNEXT_UBC_NDEN             (1 << 26) /* Bit 26: Next Descriptor Destination Update */
#define CHNEXT_UBC_NVIEW_SHIFT      (27)      /* Bits 27-29: Next Descriptor View */
#define CHNEXT_UBC_NVIEW_MASK       (3 << CHNEXT_UBC_NVIEW_SHIFT)
#  define CHNEXT_UBC_NVIEW_0        (0 << CHNEXT_UBC_NVIEW_SHIFT) /* Next Descriptor View 0 */
#  define CHNEXT_UBC_NVIEW_1        (1 << CHNEXT_UBC_NVIEW_SHIFT) /* Next Descriptor View 1 */
#  define CHNEXT_UBC_NVIEW_2        (2 << CHNEXT_UBC_NVIEW_SHIFT) /* Next Descriptor View 2 */
#  define CHNEXT_UBC_NVIEW_3        (3 << CHNEXT_UBC_NVIEW_SHIFT) /* Next Descriptor View 3 */

/* Source Address (32-bit address) */
/* Destination Address (32-bit address) */

/* Configuration Register */
/* Block Control */

/* Data Stride (32-bit value) */
/* Source Microblock Stride (32-bit value) */
/* Destination Microblock Stride (32-bit value) */

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

struct chnext_view0_s
{
  uint32_t cnda; /* Next Descriptor Address */
  uint32_t cubc; /* Microblock Control */
  uint32_t cta;  /* Transfer Address */
};

struct chnext_view1_s
{
  uint32_t cnda; /* Next Descriptor Address */
  uint32_t cubc; /* Microblock Control */
  uint32_t csa;  /* Source Address */
  uint32_t cda;  /* Destination Address */
};

struct chnext_view2_s
{
  uint32_t cnda; /* Next Descriptor Address */
  uint32_t cubc; /* Microblock Control */
  uint32_t csa;  /* Source Address */
  uint32_t cda;  /* Destination Address */
  uint32_t cc;   /* Configuration Register */
};

struct chnext_view3_s
{
  uint32_t cnda; /* Next Descriptor Address */
  uint32_t cubc; /* Microblock Control */
  uint32_t csa;  /* Source Address */
  uint32_t cda;  /* Destination Address */
  uint32_t cc;   /* Configuration Register */
  uint32_t cbc;  /* Block Control */
  uint32_t cds;  /* Data Stride */
  uint32_t csus; /* Source Microblock Stride */
  uint32_t cdus; /* Destination Microblock Stride */
};

#endif /* __ARCH_ARM_SRC_SAMA5_CHIP_SAM_XDMAC_H */
