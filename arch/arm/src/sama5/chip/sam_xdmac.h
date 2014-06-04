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

#define SAM_XDMAC_GTYPE_OFFSET    0x0000 /* Global Type Register */
#define SAM_XDMAC_GCFG_OFFSET     0x0004 /* Global Configuration Register */
#define SAM_XDMAC_GWAC_OFFSET     0x0008 /* Global Weighted Arbiter Configuration Register */
#define SAM_XDMAC_GIE_OFFSET      0x000c /* Global Interrupt Enable Register */
#define SAM_XDMAC_GID_OFFSET      0x0010 /* Global Interrupt Disable Register */
#define SAM_XDMAC_GIM_OFFSET      0x0014 /* Global Interrupt Mask Register */
#define SAM_XDMAC_GIS_OFFSET      0x0018 /* Global Interrupt Status Register */
#define SAM_XDMAC_GE_OFFSET       0x001c /* Global Channel Enable Register */
#define SAM_XDMAC_GD_OFFSET       0x0020 /* Global Channel Disable Register */
#define SAM_XDMAC_GS_OFFSET       0x0024 /* Global Channel Status Register */
#define SAM_XDMAC_GRS_OFFSET      0x0028 /* Global Channel Read Suspend Register */
#define SAM_XDMAC_GWS_OFFSET      0x002c /* Global Channel Write Suspend Register */
#define SAM_XDMAC_GRWS_OFFSET     0x0030 /* Global Channel Read Write Suspend Register */
#define SAM_XDMAC_GRWR_OFFSET     0x0034 /* Global Channel Read Write Resume Register */
#define SAM_XDMAC_GSWR_OFFSET     0x0038 /* Global Channel Software Request Register */
#define SAM_XDMAC_GSWS_OFFSET     0x003c /* Global Channel Software Request Status Register */
#define SAM_XDMAC_GSWF_OFFSET     0x0040 /* Global Channel Software Flush Request Register */
                                         /* 0x0044–0x004c Reserved */
#define SAM_XDMACH_OFFSET(n)      (0x0050 + ((n) << 6))
#define SAM_XDMACH_CIE_OFFSET     0x0000 /* Channel Interrupt Enable Register */
#define SAM_XDMACH_CID_OFFSET     0x0004 /* Channel Interrupt Disable Register */
#define SAM_XDMACH_CIM_OFFSET     0x0008 /* Channel Interrupt Mask Register */
#define SAM_XDMACH_CIS_OFFSET     0x000C /* Channel Interrupt Status Register */
#define SAM_XDMACH_CSA_OFFSET     0x0010 /* Channel Source Address Register */
#define SAM_XDMACH_CDA_OFFSET     0x0014 /* Channel Destination Address Register */
#define SAM_XDMACH_CNDA_OFFSET    0x0018 /* Channel Next Descriptor Address Register */
#define SAM_XDMACH_CNDC_OFFSET    0x001c /* Channel Next Descriptor Control Register */
#define SAM_XDMACH_CUBC_OFFSET    0x0020 /* Channel Microblock Control Register */
#define SAM_XDMACH_CBC_OFFSET     0x0024 /* Channel Block Control Register */
#define SAM_XDMACH_CC_OFFSET      0x0028 /* Channel Configuration Register */
#define SAM_XDMACH_CDSMSP_OFFSET  0x002c /* Channel Data Stride Memory Set Pattern */
#define SAM_XDMACH_CSUS_OFFSET    0x0030 /* Channel Source Microblock Stride */
#define SAM_XDMACH_CDUS_OFFSET    0x0034 /* Channel Destination Microblock Stride */
                                         /* 0x0038-0x003c Reserved */
                                         /* 0x0fec–0x0ffc Reserved */

/* XDMAC Register Addresses *********************************************************/

#define SAM_XDMAC0_GTYPE          (SAM_XDMAC0_VBASE+SAM_XDMAC_GTYPE_OFFSET)
#define SAM_XDMAC0_GCFG           (SAM_XDMAC0_VBASE+SAM_XDMAC_GCFG_OFFSET)
#define SAM_XDMAC0_GWAC           (SAM_XDMAC0_VBASE+SAM_XDMAC_GWAC_OFFSET)
#define SAM_XDMAC0_GIE            (SAM_XDMAC0_VBASE+SAM_XDMAC_GIE_OFFSET)
#define SAM_XDMAC0_GID            (SAM_XDMAC0_VBASE+SAM_XDMAC_GID_OFFSET)
#define SAM_XDMAC0_GIM            (SAM_XDMAC0_VBASE+SAM_XDMAC_GIM_OFFSET)
#define SAM_XDMAC0_GIS            (SAM_XDMAC0_VBASE+SAM_XDMAC_GIS_OFFSET)
#define SAM_XDMAC0_GE             (SAM_XDMAC0_VBASE+SAM_XDMAC_GE_OFFSET)
#define SAM_XDMAC0_GD             (SAM_XDMAC0_VBASE+SAM_XDMAC_GD_OFFSET)
#define SAM_XDMAC0_GS             (SAM_XDMAC0_VBASE+SAM_XDMAC_GS_OFFSET)
#define SAM_XDMAC0_GRS            (SAM_XDMAC0_VBASE+SAM_XDMAC_GRS_OFFSET)
#define SAM_XDMAC0_GWS            (SAM_XDMAC0_VBASE+SAM_XDMAC_GWS_OFFSET)
#define SAM_XDMAC0_GRWS           (SAM_XDMAC0_VBASE+SAM_XDMAC_GRWS_OFFSET)
#define SAM_XDMAC0_GRWR           (SAM_XDMAC0_VBASE+SAM_XDMAC_GRWR_OFFSET)
#define SAM_XDMAC0_GSWR           (SAM_XDMAC0_VBASE+SAM_XDMAC_GSWR_OFFSET)
#define SAM_XDMAC0_GSWS           (SAM_XDMAC0_VBASE+SAM_XDMAC_GSWS_OFFSET)
#define SAM_XDMAC0_GSWF           (SAM_XDMAC0_VBASE+SAM_XDMAC_GSWF_OFFSET)

#define SAM_XDMACH0_BASE(n)       (SAM_XDMAC0_VBASE+SAM_XDMACH_OFFSET(n))
#define SAM_XDMACH0_CIE(n)        (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CIE_OFFSET)
#define SAM_XDMACH0_CID(n)        (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CID_OFFSET)
#define SAM_XDMACH0_CIM(n)        (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CIM_OFFSET)
#define SAM_XDMACH0_CIS(n)        (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CIS_OFFSET)
#define SAM_XDMACH0_CSA(n)        (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CSA_OFFSET)
#define SAM_XDMACH0_CDA(n)        (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CDA_OFFSET)
#define SAM_XDMACH0_CNDA(n)       (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CNDA_OFFSET)
#define SAM_XDMACH0_CNDC(n)       (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CNDC_OFFSET)
#define SAM_XDMACH0_CUBC(n)       (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CUBC_OFFSET)
#define SAM_XDMACH0_CBC(n)        (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CBC_OFFSET)
#define SAM_XDMACH0_CC(n)         (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CC_OFFSET)
#define SAM_XDMACH0_CDSMSP(n)     (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CDSMSP_OFFSET)
#define SAM_XDMACH0_CSUS(n)       (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CSUS_OFFSET)
#define SAM_XDMACH0_CDUS(n)       (SAM_XDMACH0_BASE(n)+SAM_XDMACH_CDUS_OFFSET)

#define SAM_XDMAC1_GTYPE          (SAM_XDMAC1_VBASE+SAM_XDMAC_GTYPE_OFFSET)
#define SAM_XDMAC1_GCFG           (SAM_XDMAC1_VBASE+SAM_XDMAC_GCFG_OFFSET)
#define SAM_XDMAC1_GWAC           (SAM_XDMAC1_VBASE+SAM_XDMAC_GWAC_OFFSET)
#define SAM_XDMAC1_GIE            (SAM_XDMAC1_VBASE+SAM_XDMAC_GIE_OFFSET)
#define SAM_XDMAC1_GID            (SAM_XDMAC1_VBASE+SAM_XDMAC_GID_OFFSET)
#define SAM_XDMAC1_GIM            (SAM_XDMAC1_VBASE+SAM_XDMAC_GIM_OFFSET)
#define SAM_XDMAC1_GIS            (SAM_XDMAC1_VBASE+SAM_XDMAC_GIS_OFFSET)
#define SAM_XDMAC1_GE             (SAM_XDMAC1_VBASE+SAM_XDMAC_GE_OFFSET)
#define SAM_XDMAC1_GD             (SAM_XDMAC1_VBASE+SAM_XDMAC_GD_OFFSET)
#define SAM_XDMAC1_GS             (SAM_XDMAC1_VBASE+SAM_XDMAC_GS_OFFSET)
#define SAM_XDMAC1_GRS            (SAM_XDMAC1_VBASE+SAM_XDMAC_GRS_OFFSET)
#define SAM_XDMAC1_GWS            (SAM_XDMAC1_VBASE+SAM_XDMAC_GWS_OFFSET)
#define SAM_XDMAC1_GRWS           (SAM_XDMAC1_VBASE+SAM_XDMAC_GRWS_OFFSET)
#define SAM_XDMAC1_GRWR           (SAM_XDMAC1_VBASE+SAM_XDMAC_GRWR_OFFSET)
#define SAM_XDMAC1_GSWR           (SAM_XDMAC1_VBASE+SAM_XDMAC_GSWR_OFFSET)
#define SAM_XDMAC1_GSWS           (SAM_XDMAC1_VBASE+SAM_XDMAC_GSWS_OFFSET)
#define SAM_XDMAC1_GSWF           (SAM_XDMAC1_VBASE+SAM_XDMAC_GSWF_OFFSET)

#define SAM_XDMACH1_BASE(n)       (SAM_XDMAC1_VBASE+SAM_XDMACH_OFFSET(n))
#define SAM_XDMACH1_CIE(n)        (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CIE_OFFSET)
#define SAM_XDMACH1_CID(n)        (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CID_OFFSET)
#define SAM_XDMACH1_CIM(n)        (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CIM_OFFSET)
#define SAM_XDMACH1_CIS(n)        (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CIS_OFFSET)
#define SAM_XDMACH1_CSA(n)        (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CSA_OFFSET)
#define SAM_XDMACH1_CDA(n)        (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CDA_OFFSET)
#define SAM_XDMACH1_CNDA(n)       (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CNDA_OFFSET)
#define SAM_XDMACH1_CNDC(n)       (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CNDC_OFFSET)
#define SAM_XDMACH1_CUBC(n)       (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CUBC_OFFSET)
#define SAM_XDMACH1_CBC(n)        (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CBC_OFFSET)
#define SAM_XDMACH1_CC(n)         (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CC_OFFSET)
#define SAM_XDMACH1_CDSMSP(n)     (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CDSMSP_OFFSET)
#define SAM_XDMACH1_CSUS(n)       (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CSUS_OFFSET)
#define SAM_XDMACH1_CDUS(n)       (SAM_XDMACH1_BASE(n)+SAM_XDMACH_CDUS_OFFSET)

/* XDMAC Register Bit Definitions ***************************************************/

/* Global Type Register */
#define XDMAC_GTYPE_
/* Global Configuration Register */
#define XDMAC_GCFG_
/* Global Weighted Arbiter Configuration Register */
#define XDMAC_GWAC_
/* Global Interrupt Enable Register */
#define XDMAC_GIE_
/* Global Interrupt Disable Register */
#define XDMAC_GID_
/* Global Interrupt Mask Register */
#define XDMAC_GIM_
/* Global Interrupt Status Register */
#define XDMAC_GIS_
/* Global Channel Enable Register */
#define XDMAC_GE_
/* Global Channel Disable Register */
#define XDMAC_GD_
/* Global Channel Status Register */
#define XDMAC_GS_
/* Global Channel Read Suspend Register */
#define XDMAC_GRS_
/* Global Channel Write Suspend Register */
#define XDMAC_GWS_
/* Global Channel Read Write Suspend Register */
#define XDMAC_GRWS_
/* Global Channel Read Write Resume Register */
#define XDMAC_GRWR_
/* Global Channel Software Request Register */
#define XDMAC_GSWR_
/* Global Channel Software Request Status Register */
#define XDMAC_GSWS_
/* Global Channel Software Flush Request Register */
#define XDMAC_GSWF_

/* Channel Interrupt Enable Register */
#define XDMACH_CIE_
/* Channel Interrupt Disable Register */
#define XDMACH_CID_
/* Channel Interrupt Mask Register */
#define XDMACH_CIM_
/* Channel Interrupt Status Register */
#define XDMACH_CIS_
/* Channel Source Address Register */
#define XDMACH_CSA_
/* Channel Destination Address Register */
#define XDMACH_CDA_
/* Channel Next Descriptor Address Register */
#define XDMACH_CNDA_
/* Channel Next Descriptor Control Register */
#define XDMACH_CNDC_
/* Channel Microblock Control Register */
#define XDMACH_CUBC_
/* Channel Block Control Register */
#define XDMACH_CBC_
/* Channel Configuration Register */
#define XDMACH_CC_
/* Channel Data Stride Memory Set Pattern */
#define XDMACH_CDSMSP_
/* Channel Source Microblock Stride */
#define XDMACH_CSUS_
/* Channel Destination Microblock Stride */
#define XDMACH_CDUS_

#endif /* __ARCH_ARM_SRC_SAMA5_CHIP_SAM_XDMAC_H */
