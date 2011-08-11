/************************************************************************************
 * arch/arm/src/kinetis/kinetis_sdhc.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_SDHC_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_SDHC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KINETIS_SDHC_DSADDR_OFFSET     0x0000 /* DMA System Address Register */
#define KINETIS_SDHC_BLKATTR_OFFSET    0x0004 /* Block Attributes Register */
#define KINETIS_SDHC_CMDARG_OFFSET     0x0008 /* Command Argument Register */
#define KINETIS_SDHC_XFERTYP_OFFSET    0x000c /* Transfer Type Register */
#define KINETIS_SDHC_CMDRSP0_OFFSET    0x0010 /* Command Response 0 */
#define KINETIS_SDHC_CMDRSP1_OFFSET    0x0014 /* Command Response 1 */
#define KINETIS_SDHC_CMDRSP2_OFFSET    0x0018 /* Command Response 2 */
#define KINETIS_SDHC_CMDRSP3_OFFSET    0x001c /* Command Response 3 */
#define KINETIS_SDHC_DATPORT_OFFSET    0x0020 /* Buffer Data Port Register */
#define KINETIS_SDHC_PRSSTAT_OFFSET    0x0024 /* Present State Register */
#define KINETIS_SDHC_PROCTL_OFFSET     0x0028 /* Protocol Control Register */
#define KINETIS_SDHC_SYSCTL_OFFSET     0x002c /* System Control Register */
#define KINETIS_SDHC_IRQSTAT_OFFSET    0x0030 /* Interrupt Status Register */
#define KINETIS_SDHC_IRQSTATEN_OFFSET  0x0034 /* Interrupt Status Enable Register */
#define KINETIS_SDHC_IRQSIGEN_OFFSET   0x0038 /* Interrupt Signal Enable Register */
#define KINETIS_SDHC_AC12ERR_OFFSET    0x003c /* Auto CMD12 Error Status Register */
#define KINETIS_SDHC_HTCAPBLT_OFFSET   0x0040 /* Host Controller Capabilities */
#define KINETIS_SDHC_WML_OFFSET        0x0044 /* Watermark Level Register */
#define KINETIS_SDHC_FEVT_OFFSET       0x0050 /* Force Event Register */
#define KINETIS_SDHC_ADMAES_OFFSET     0x0054 /* ADMA Error Status Register */
#define KINETIS_SDHC_ADSADDR_OFFSET    0x0058 /* ADMA System Address Register */
#define KINETIS_SDHC_VENDOR_OFFSET     0x00c0 /* Vendor Specific Register */
#define KINETIS_SDHC_MMCBOOT_OFFSET    0x00c4 /* MMC Boot Register */
#define KINETIS_SDHC_HOSTVER_OFFSET    0x00fc /* Host Controller Version */

/* Register Addresses ***************************************************************/

#define KINETIS_SDHC_DSADDR            (KINETIS_SDHC_BASE+KINETIS_SDHC_DSADDR_OFFSET)
#define KINETIS_SDHC_BLKATTR           (KINETIS_SDHC_BASE+KINETIS_SDHC_BLKATTR_OFFSET)
#define KINETIS_SDHC_CMDARG            (KINETIS_SDHC_BASE+KINETIS_SDHC_CMDARG_OFFSET)
#define KINETIS_SDHC_XFERTYP           (KINETIS_SDHC_BASE+KINETIS_SDHC_XFERTYP_OFFSET)
#define KINETIS_SDHC_CMDRSP0           (KINETIS_SDHC_BASE+KINETIS_SDHC_CMDRSP0_OFFSET)
#define KINETIS_SDHC_CMDRSP1           (KINETIS_SDHC_BASE+KINETIS_SDHC_CMDRSP1_OFFSET)
#define KINETIS_SDHC_CMDRSP2           (KINETIS_SDHC_BASE+KINETIS_SDHC_CMDRSP2_OFFSET)
#define KINETIS_SDHC_CMDRSP3           (KINETIS_SDHC_BASE+KINETIS_SDHC_CMDRSP3_OFFSET)
#define KINETIS_SDHC_DATPORT           (KINETIS_SDHC_BASE+KINETIS_SDHC_DATPORT_OFFSET)
#define KINETIS_SDHC_PRSSTAT           (KINETIS_SDHC_BASE+KINETIS_SDHC_PRSSTAT_OFFSET)
#define KINETIS_SDHC_PROCTL            (KINETIS_SDHC_BASE+KINETIS_SDHC_PROCTL_OFFSET)
#define KINETIS_SDHC_SYSCTL            (KINETIS_SDHC_BASE+KINETIS_SDHC_SYSCTL_OFFSET)
#define KINETIS_SDHC_IRQSTAT           (KINETIS_SDHC_BASE+KINETIS_SDHC_IRQSTAT_OFFSET)
#define KINETIS_SDHC_IRQSTATEN         (KINETIS_SDHC_BASE+KINETIS_SDHC_IRQSTATEN_OFFSET)
#define KINETIS_SDHC_IRQSIGEN          (KINETIS_SDHC_BASE+KINETIS_SDHC_IRQSIGEN_OFFSET)
#define KINETIS_SDHC_AC12ERR           (KINETIS_SDHC_BASE+KINETIS_SDHC_AC12ERR_OFFSET)
#define KINETIS_SDHC_HTCAPBLT          (KINETIS_SDHC_BASE+KINETIS_SDHC_HTCAPBLT_OFFSET)
#define KINETIS_SDHC_WML               (KINETIS_SDHC_BASE+KINETIS_SDHC_WML_OFFSET)
#define KINETIS_SDHC_FEVT              (KINETIS_SDHC_BASE+KINETIS_SDHC_FEVT_OFFSET)
#define KINETIS_SDHC_ADMAES            (KINETIS_SDHC_BASE+KINETIS_SDHC_ADMAES_OFFSET)
#define KINETIS_SDHC_ADSADDR           (KINETIS_SDHC_BASE+KINETIS_SDHC_ADSADDR_OFFSET)
#define KINETIS_SDHC_VENDOR            (KINETIS_SDHC_BASE+KINETIS_SDHC_VENDOR_OFFSET)
#define KINETIS_SDHC_MMCBOOT           (KINETIS_SDHC_BASE+KINETIS_SDHC_MMCBOOT_OFFSET)
#define KINETIS_SDHC_HOSTVER           (KINETIS_SDHC_BASE+KINETIS_SDHC_HOSTVER_OFFSET)

/* Register Bit Definitions *********************************************************/

/* DMA System Address Register */
#define SDHC_DSADDR_
/* Block Attributes Register */
#define SDHC_BLKATTR_
/* Command Argument Register */
#define SDHC_CMDARG_
/* Transfer Type Register */
#define SDHC_XFERTYP_
/* Command Response 0 */
#define SDHC_CMDRSP0_
/* Command Response 1 */
#define SDHC_CMDRSP1_
/* Command Response 2 */
#define SDHC_CMDRSP2_
/* Command Response 3 */
#define SDHC_CMDRSP3_
/* Buffer Data Port Register */
#define SDHC_DATPORT_
/* Present State Register */
#define SDHC_PRSSTAT_
/* Protocol Control Register */
#define SDHC_PROCTL_
/* System Control Register */
#define SDHC_SYSCTL_
/* Interrupt Status Register */
#define SDHC_IRQSTAT_
/* Interrupt Status Enable Register */
#define SDHC_IRQSTATEN_
/* Interrupt Signal Enable Register */
#define SDHC_IRQSIGEN_
/* Auto CMD12 Error Status Register */
#define SDHC_AC12ERR_
/* Host Controller Capabilities */
#define SDHC_HTCAPBLT_
/* Watermark Level Register */
#define SDHC_WML_
/* Force Event Register */
#define SDHC_FEVT_
/* ADMA Error Status Register */
#define SDHC_ADMAES_
/* ADMA System Address Register */
#define SDHC_ADSADDR_
/* Vendor Specific Register */
#define SDHC_VENDOR_
/* MMC Boot Register */
#define SDHC_MMCBOOT_
/* Host Controller Version */
#define SDHC_HOSTVER_

                (1 << nn)  /* Bit nn:  
_SHIFT          (nn)       /* Bits nn-nn: 
_MASK           (nn << nn)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_SDHC_H */
