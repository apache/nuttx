/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_sdhci.h
 *
 *   Copyright (C) 2008-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_SDHCI_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_SDHCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define CXD56_SDHCI_DSADDR_OFFSET      (0x0000) /* DMA System Address Register */
#define CXD56_SDHCI_BLKATTR_OFFSET     (0x0004) /* Block Attributes Register */
#define CXD56_SDHCI_CMDARG_OFFSET      (0x0008) /* Command Argument Register */
#define CXD56_SDHCI_XFERTYP_OFFSET     (0x000c) /* Transfer Type Register */
#define CXD56_SDHCI_CMDRSP0_OFFSET     (0x0010) /* Command Response 0 */
#define CXD56_SDHCI_CMDRSP1_OFFSET     (0x0014) /* Command Response 1 */
#define CXD56_SDHCI_CMDRSP2_OFFSET     (0x0018) /* Command Response 2 */
#define CXD56_SDHCI_CMDRSP3_OFFSET     (0x001c) /* Command Response 3 */
#define CXD56_SDHCI_DATPORT_OFFSET     (0x0020) /* Buffer Data Port Register */
#define CXD56_SDHCI_PRSSTAT_OFFSET     (0x0024) /* Present State Register */
#define CXD56_SDHCI_PROCTL_OFFSET      (0x0028) /* Protocol Control Register */
#define CXD56_SDHCI_SYSCTL_OFFSET      (0x002c) /* System Control Register */
#define CXD56_SDHCI_IRQSTAT_OFFSET     (0x0030) /* Interrupt Status Register */
#define CXD56_SDHCI_IRQSTATEN_OFFSET   (0x0034) /* Interrupt Status Enable Register */
#define CXD56_SDHCI_IRQSIGEN_OFFSET    (0x0038) /* Interrupt Signal Enable Register */
#define CXD56_SDHCI_AC12ERR_OFFSET     (0x003c) /* Auto CMD12 Error Status Register */
#define CXD56_SDHCI_HTCAPBLT_OFFSET    (0x0040) /* Host Controller Capabilities */
#define CXD56_SDHCI_FEVT_OFFSET        (0x0050) /* Force Event Register */
#define CXD56_SDHCI_ADMAES_OFFSET      (0x0054) /* ADMA Error Status Register */
#define CXD56_SDHCI_ADSADDR_OFFSET     (0x0058) /* ADMA System Address Register */
#define CXD56_SDHCI_VENDSPEC_OFFSET    (0x0110) /* Vendor Specific Control */
#define CXD56_SDHCI_OTHERIOLL_OFFSET   (0x021C) /* IO Pin Control       */
#define CXD56_SDHCI_USERDEF1CTL_OFFSET (0x0270) /* User Define1 Control Register */
#define CXD56_SDHCI_USERDEF2CTL_OFFSET (0x0274) /* User Define2 Control Register */

/* Register Addresses *******************************************************/

#define CXD56_SDHCI_BASE              CXD56_SDIO_BASE

#define CXD56_SDHCI_DSADDR            (CXD56_SDHCI_BASE+CXD56_SDHCI_DSADDR_OFFSET)
#define CXD56_SDHCI_BLKATTR           (CXD56_SDHCI_BASE+CXD56_SDHCI_BLKATTR_OFFSET)
#define CXD56_SDHCI_CMDARG            (CXD56_SDHCI_BASE+CXD56_SDHCI_CMDARG_OFFSET)
#define CXD56_SDHCI_XFERTYP           (CXD56_SDHCI_BASE+CXD56_SDHCI_XFERTYP_OFFSET)
#define CXD56_SDHCI_CMDRSP0           (CXD56_SDHCI_BASE+CXD56_SDHCI_CMDRSP0_OFFSET)
#define CXD56_SDHCI_CMDRSP1           (CXD56_SDHCI_BASE+CXD56_SDHCI_CMDRSP1_OFFSET)
#define CXD56_SDHCI_CMDRSP2           (CXD56_SDHCI_BASE+CXD56_SDHCI_CMDRSP2_OFFSET)
#define CXD56_SDHCI_CMDRSP3           (CXD56_SDHCI_BASE+CXD56_SDHCI_CMDRSP3_OFFSET)
#define CXD56_SDHCI_DATPORT           (CXD56_SDHCI_BASE+CXD56_SDHCI_DATPORT_OFFSET)
#define CXD56_SDHCI_PRSSTAT           (CXD56_SDHCI_BASE+CXD56_SDHCI_PRSSTAT_OFFSET)
#define CXD56_SDHCI_PROCTL            (CXD56_SDHCI_BASE+CXD56_SDHCI_PROCTL_OFFSET)
#define CXD56_SDHCI_SYSCTL            (CXD56_SDHCI_BASE+CXD56_SDHCI_SYSCTL_OFFSET)
#define CXD56_SDHCI_IRQSTAT           (CXD56_SDHCI_BASE+CXD56_SDHCI_IRQSTAT_OFFSET)
#define CXD56_SDHCI_IRQSTATEN         (CXD56_SDHCI_BASE+CXD56_SDHCI_IRQSTATEN_OFFSET)
#define CXD56_SDHCI_IRQSIGEN          (CXD56_SDHCI_BASE+CXD56_SDHCI_IRQSIGEN_OFFSET)
#define CXD56_SDHCI_AC12ERR           (CXD56_SDHCI_BASE+CXD56_SDHCI_AC12ERR_OFFSET)
#define CXD56_SDHCI_HTCAPBLT          (CXD56_SDHCI_BASE+CXD56_SDHCI_HTCAPBLT_OFFSET)
#define CXD56_SDHCI_FEVT              (CXD56_SDHCI_BASE+CXD56_SDHCI_FEVT_OFFSET)
#define CXD56_SDHCI_ADMAES            (CXD56_SDHCI_BASE+CXD56_SDHCI_ADMAES_OFFSET)
#define CXD56_SDHCI_ADSADDR           (CXD56_SDHCI_BASE+CXD56_SDHCI_ADSADDR_OFFSET)
#define CXD56_SDHCI_VENDSPEC          (CXD56_SDHCI_BASE+CXD56_SDHCI_VENDSPEC_OFFSET)
#define CXD56_SDHCI_OTHERIOLL         (CXD56_SDHCI_BASE+CXD56_SDHCI_OTHERIOLL_OFFSET)
#define CXD56_SDHCI_USERDEF1CTL       (CXD56_SDHCI_BASE+CXD56_SDHCI_USERDEF1CTL_OFFSET)
#define CXD56_SDHCI_USERDEF2CTL       (CXD56_SDHCI_BASE+CXD56_SDHCI_USERDEF2CTL_OFFSET)

/* Register Bit Definitions *************************************************/

/* DMA System Address Register */

#define SDHCI_DSADDR_SHIFT               (1)          /* Bits 1-31: DMA System Address */
#define SDHCI_DSADDR_MASK                (0xfffffffe) /* Bits 0-1: Reserved */

/* Block Attributes Register */

#define SDHCI_BLKATTR_SIZE_SHIFT         (0)       /* Bits 0-12: Transfer Block Size */
#define SDHCI_BLKATTR_SIZE_MASK          (0x1fff << SDHCI_BLKATTR_SIZE_SHIFT)
                                                  /* Bits 13-15: Reserved */

#define SDHCI_BLKATTR_CNT_SHIFT          (16)      /* Bits 16-31: Blocks Count For Current Transfer */
#define SDHCI_BLKATTR_CNT_MASK           (0xffff << SDHCI_BLKATTR_CNT_SHIFT)

/* Command Argument Register (32-bit cmd/arg data) */

/* Transfer Type Register */

#define SDHCI_XFERTYP_DMAEN              (1 << 0)  /* Bit 0:  DMA Enable */
#define SDHCI_XFERTYP_BCEN               (1 << 1)  /* Bit 1:  Block Count Enable */
#define SDHCI_XFERTYP_AC12EN             (1 << 2)  /* Bit 2:  Auto CMD12 Enable */
                                                   /* Bit 3: Reserved */

#define SDHCI_XFERTYP_DTDSEL             (1 << 4)  /* Bit 4:  Data Transfer Direction Select */
#define SDHCI_XFERTYP_MSBSEL             (1 << 5)  /* Bit 5:  Multi/Single Block Select */
                                                   /* Bits 6-15: Reserved */

#define SDHCI_XFERTYP_RSPTYP_SHIFT       (16)      /* Bits 16-17: Response Type Select */
#define SDHCI_XFERTYP_RSPTYP_MASK        (3 << SDHCI_XFERTYP_RSPTYP_SHIFT)
#define SDHCI_XFERTYP_RSPTYP_NONE        (0 << SDHCI_XFERTYP_RSPTYP_SHIFT) /* No response */
#define SDHCI_XFERTYP_RSPTYP_LEN136      (1 << SDHCI_XFERTYP_RSPTYP_SHIFT) /* Response length 136 */
#define SDHCI_XFERTYP_RSPTYP_LEN48       (2 << SDHCI_XFERTYP_RSPTYP_SHIFT) /* Response length 48 */
#define SDHCI_XFERTYP_RSPTYP_LEN48BSY    (3 << SDHCI_XFERTYP_RSPTYP_SHIFT) /* Response length 48, check busy */

                                                  /* Bit 18: Reserved */

#define SDHCI_XFERTYP_CCCEN              (1 << 19) /* Bit 19: Command CRC Check Enable */
#define SDHCI_XFERTYP_CICEN              (1 << 20) /* Bit 20: Command Index Check Enable */
#define SDHCI_XFERTYP_DPSEL              (1 << 21) /* Bit 21: Data Present Select */
#define SDHCI_XFERTYP_CMDTYP_SHIFT       (22)      /* Bits 22-23: Command Type */

#define SDHCI_XFERTYP_CMDTYP_MASK        (3 << SDHCI_XFERTYP_CMDTYP_SHIFT)
#define SDHCI_XFERTYP_CMDTYP_NORMAL      (0 << SDHCI_XFERTYP_CMDTYP_SHIFT) /* Normal other commands */
#define SDHCI_XFERTYP_CMDTYP_SUSPEND     (1 << SDHCI_XFERTYP_CMDTYP_SHIFT) /* Suspend CMD52 for writing bus suspend in CCCR */
#define SDHCI_XFERTYP_CMDTYP_RESUME      (2 << SDHCI_XFERTYP_CMDTYP_SHIFT) /* Resume CMD52 for writing function select in CCCR */
#define SDHCI_XFERTYP_CMDTYP_ABORT       (3 << SDHCI_XFERTYP_CMDTYP_SHIFT) /* Abort CMD12, CMD52 for writing I/O abort in CCCR */

#define SDHCI_XFERTYP_CMDINX_SHIFT       (24)      /* Bits 24-29: Command Index */
#define SDHCI_XFERTYP_CMDINX_MASK        (63 << SDHCI_XFERTYP_CMDINX_SHIFT)
                                                  /* Bits 30-31: Reserved */

/* Command Response 0-3 (32-bit response data) */

/* Buffer Data Port Register (32-bit data content) */

/* Present State Register */

#define SDHCI_PRSSTAT_CIHB               (1 << 0)  /* Bit 0:  Command Inhibit (CMD) */
#define SDHCI_PRSSTAT_CDIHB              (1 << 1)  /* Bit 1:  Command Inhibit (DAT) */
#define SDHCI_PRSSTAT_DLA                (1 << 2)  /* Bit 2:  Data Line Active */
#define SDHCI_PRSSTAT_SDSTB              (1 << 3)  /* Bit 3:  SD Clock Stable */
#define SDHCI_PRSSTAT_IPGOFF             (1 << 4)  /* Bit 4:  Bus Clock */
#define SDHCI_PRSSTAT_HCKOFF             (1 << 5)  /* Bit 5:  System Clock */
#define SDHCI_PRSSTAT_PEROFF             (1 << 6)  /* Bit 6:  SDHC clock */
#define SDHCI_PRSSTAT_SDOFF              (1 << 7)  /* Bit 7:  SD Clock Gated Off Internally */
#define SDHCI_PRSSTAT_WTA                (1 << 8)  /* Bit 8:  Write Transfer Active */
#define SDHCI_PRSSTAT_RTA                (1 << 9)  /* Bit 9:  Read Transfer Active */
#define SDHCI_PRSSTAT_BWEN               (1 << 10) /* Bit 10: Buffer Write Enable */
#define SDHCI_PRSSTAT_BREN               (1 << 11) /* Bit 11: Buffer Read Enable */
                                                   /* Bits 12-15: Reserved */

#define SDHCI_PRSSTAT_CINS               (1 << 16) /* Bit 16: Card Inserted */
#define SDHCI_PRSSTAT_CSTS               (1 << 17) /* Bit 17: Card State Stable */
#define SDHCI_PRSSTAT_SDCD               (1 << 18) /* Bit 18: Card Detect Pin Level */
#define SDHCI_PRSSTAT_SDWPN              (1 << 19) /* Bit 19: Write Protect Switch Pin Level*/
#define SDHCI_PRSSTAT_DLSL_SHIFT         (20)      /* Bits 20-23: DAT Line Signal Level */
#define SDHCI_PRSSTAT_DLSL_MASK          (0xf << SDHCI_PRSSTAT_DLSL_SHIFT)
#define SDHCI_PRSSTAT_DLSL_DAT0          (0x1 << SDHCI_PRSSTAT_DLSL_SHIFT)
#define SDHCI_PRSSTAT_DLSL_DAT1          (0x2 << SDHCI_PRSSTAT_DLSL_SHIFT)
#define SDHCI_PRSSTAT_DLSL_DAT2          (0x4 << SDHCI_PRSSTAT_DLSL_SHIFT)
#define SDHCI_PRSSTAT_DLSL_DAT3          (0x8 << SDHCI_PRSSTAT_DLSL_SHIFT)
#define SDHCI_PRSSTAT_CLSL               (1 << 24) /* Bit 23: CMD Line Signal Level */

/* Protocol Control Register */

#define SDHCI_PROCTL_LCTL                (1 << 0)  /* Bit 0:  LED Control */
#define SDHCI_PROCTL_DTW_SHIFT           (1)       /* Bits 1-2: Data Transfer Width */

#define SDHCI_PROCTL_DTW_MASK            (1 << SDHCI_PROCTL_DTW_SHIFT)
#define SDHCI_PROCTL_DTW_1BIT            (0 << SDHCI_PROCTL_DTW_SHIFT) /* 1-bit mode */
#define SDHCI_PROCTL_DTW_4BIT            (1 << SDHCI_PROCTL_DTW_SHIFT) /* 4-bit mode */

#define SDHCI_PROCTL_DMAS_SHIFT          (3)       /* Bits 8-9: DMA Select */

#define SDHCI_PROCTL_DMAS_MASK           (3 << SDHCI_PROCTL_DMAS_SHIFT)
#define SDHCI_PROCTL_DMAS_NODMA          (0 << SDHCI_PROCTL_DMAS_SHIFT) /* No DMA or simple DMA is selected */
#define SDHCI_PROCTL_DMAS_ADMA2          (2 << SDHCI_PROCTL_DMAS_SHIFT) /* ADMA2 is selected */

#define SDHCI_PROCTL_CDTL                (1 << 6)  /* Bit 6:  Card Detect Test Level */
#define SDHCI_PROCTL_CDSS                (1 << 7)  /* Bit 7:  Card Detect Signal Selection */
                                                   /* Bits 10-15: Reserved */

#define SDHCI_PROCTL_SABGREQ             (1 << 16) /* Bit 16: Stop At Block Gap Request */
#define SDHCI_PROCTL_CREQ                (1 << 17) /* Bit 17: Continue Request */
#define SDHCI_PROCTL_RWCTL               (1 << 18) /* Bit 18: Read Wait Control */
#define SDHCI_PROCTL_IABG                (1 << 19) /* Bit 19: Interrupt At Block Gap */
                                                   /* Bits 20-23: Reserved */

#define SDHCI_PROCTL_WECINT              (1 << 24) /* Bit 24: Wakeup Event Enable On Card Interrupt */
#define SDHCI_PROCTL_WECINS              (1 << 25) /* Bit 25: Wakeup Event Enable On SD Card Insertion */
#define SDHCI_PROCTL_WECRM               (1 << 26) /* Bit 26: Wakeup Event Enable On SD Card Removal */
                                                   /* Bits 27-31: Reserved */

/* System Control Register */

#define SDHCI_SYSCTL_ICLKEN              (1 << 0)  /* Bit 0:  Internal Clock Enable */
#define SDHCI_SYSCTL_ICLKSTA             (1 << 1)  /* Bit 1:  Internal Clock Stable */
#define SDHCI_SYSCTL_SDCLKEN             (1 << 2)  /* Bit 2:  SD Clock Enable */
#define SDHCI_SYSCTL_GENSEL              (1 << 5)  /* Bit 5:  Clock Generetor Select */
#define SDHCI_SYSCTL_SDCLKFSUP_SHIFT     (6)       /* Bits 6-7: Divisor */
#define SDHCI_SYSCTL_SDCLKFSUP_MASK      (3 << SDHCI_SYSCTL_SDCLKFSUP_SHIFT)
#define SDHCI_SYSCTL_SDCLKFS_SHIFT       (8)       /* Bits 8-15: SDCLK Frequency Select */
#define SDHCI_SYSCTL_SDCLKFS_MASK        (0xff << SDHCI_SYSCTL_SDCLKFS_SHIFT)
#define SDHCI_SYSCTL_DTOCV_SHIFT         (16)      /* Bits 16-19: Data Timeout Counter Value */
#define SDHCI_SYSCTL_DTOCV_MASK          (0xf << SDHCI_SYSCTL_DTOCV_SHIFT)
#define SDHCI_SYSCTL_DTOCV_MUL(n)        (((n)-213) << SDHCI_SYSCTL_DTOCV_SHIFT) /* SDCLK x n, n=213..227 */

                                                  /* Bits 20-23: Reserved */

#define SDHCI_SYSCTL_RSTA                (1 << 24) /* Bit 24: Software Reset For ALL */
#define SDHCI_SYSCTL_RSTC                (1 << 25) /* Bit 25: Software Reset For CMD Line */
#define SDHCI_SYSCTL_RSTD                (1 << 26) /* Bit 26: Software Reset For DAT Line */
#define SDHCI_SYSCTL_INITA               (1 << 27) /* Bit 27: Initialization Active */
                                                   /* Bits 28-31: Reserved */

/* Interrupt Status Register, Interrupt Status Enable Register,
 * and Interrupt Signal Enable Register
 * Common interrupt bit definitions
 */

#define SDHCI_INT_CC                     (1 << 0)  /* Bit 0:  Command Complete */
#define SDHCI_INT_TC                     (1 << 1)  /* Bit 1:  Transfer Complete */
#define SDHCI_INT_BGE                    (1 << 2)  /* Bit 2:  Block Gap Event */
#define SDHCI_INT_DINT                   (1 << 3)  /* Bit 3:  DMA Interrupt */
#define SDHCI_INT_BWR                    (1 << 4)  /* Bit 4:  Buffer Write Ready */
#define SDHCI_INT_BRR                    (1 << 5)  /* Bit 5:  Buffer Read Ready */
#define SDHCI_INT_CINS                   (1 << 6)  /* Bit 6:  Card Insertion */
#define SDHCI_INT_CRM                    (1 << 7)  /* Bit 7:  Card Removal */
#define SDHCI_INT_CINT                   (1 << 8)  /* Bit 8:  Card Interrupt */
                                                   /* Bits 9-14: Reserved */

#define SDHCI_INT_EINT                   (1 << 15) /* Bit 15: Error Interrupt */
#define SDHCI_INT_CTOE                   (1 << 16) /* Bit 16: Command Timeout Error */
#define SDHCI_INT_CCE                    (1 << 17) /* Bit 17: Command CRC Error */
#define SDHCI_INT_CEBE                   (1 << 18) /* Bit 18: Command End Bit Error */
#define SDHCI_INT_CIE                    (1 << 19) /* Bit 19: Command Index Error */
#define SDHCI_INT_DTOE                   (1 << 20) /* Bit 20: Data Timeout Error */
#define SDHCI_INT_DCE                    (1 << 21) /* Bit 21: Data CRC Error */
#define SDHCI_INT_DEBE                   (1 << 22) /* Bit 22: Data End Bit Error */
                                                   /* Bit 23: Reserved */

#define SDHCI_INT_AC12E                  (1 << 24) /* Bit 24: Auto CMD12 Error */
                                                   /* Bits 25-27: Reserved */

#define SDHCI_INT_DMAE                   (1 << 28) /* Bit 28: DMA Error */
                                                   /* Bits 29-31: Reserved */

#define SDHCI_EINT_MASK                  0xffff0000
#define SDHCI_INT_ALL                    0x117f01ff

/* Auto CMD12 Error Status Register */

#define SDHCI_AC12ERR_NE                 (1 << 0)  /* Bit 0:  Auto CMD12 Not Executed */
#define SDHCI_AC12ERR_TOE                (1 << 1)  /* Bit 1:  Auto CMD12 Timeout Error */
#define SDHCI_AC12ERR_EBE                (1 << 2)  /* Bit 2:  Auto CMD12 End Bit Error */
#define SDHCI_AC12ERR_CE                 (1 << 3)  /* Bit 3:  Auto CMD12 CRC Error */
#define SDHCI_AC12ERR_IE                 (1 << 4)  /* Bit 4:  Auto CMD12 Index Error */
                                                   /* Bits 5-6: Reserved */

#define SDHCI_AC12ERR_CNI                (1 << 7)  /* Bit 7: Command Not Issued By Auto CMD12 Error */
                                                   /* Bits 8-31: Reserved */

/* Host Controller Capabilities */

                                                   /* Bits 0-15: Reserved */
#define SDHCI_HTCAPBLT_MBL_SHIFT         (16)      /* Bits 16-18: Max Block Length */
#define SDHCI_HTCAPBLT_MBL_MASK          (7 << SDHCI_HTCAPBLT_MBL_SHIFT)
#define SDHCI_HTCAPBLT_MBL_512BYTES      (0 << SDHCI_HTCAPBLT_MBL_SHIFT)
#define SDHCI_HTCAPBLT_MBL_1KB           (1 << SDHCI_HTCAPBLT_MBL_SHIFT)
#define SDHCI_HTCAPBLT_MBL_2KB           (2 << SDHCI_HTCAPBLT_MBL_SHIFT)
#define SDHCI_HTCAPBLT_MBL_4KB           (3 << SDHCI_HTCAPBLT_MBL_SHIFT)
                                                   /* Bit 19: Reserved */

#define SDHCI_HTCAPBLT_ADMAS             (1 << 20) /* Bit 20: ADMA Support */
#define SDHCI_HTCAPBLT_HSS               (1 << 21) /* Bit 21: High Speed Support */
#define SDHCI_HTCAPBLT_DMAS              (1 << 22) /* Bit 22: DMA Support */
#define SDHCI_HTCAPBLT_SRS               (1 << 23) /* Bit 23: Suspend/Resume Support */
#define SDHCI_HTCAPBLT_VS33              (1 << 24) /* Bit 24: Voltage Support 3.3 V */
#define SDHCI_HTCAPBLT_VS30              (1 << 25) /* Bit 25: Voltage Support 3.0 V */
#define SDHCI_HTCAPBLT_VS18              (1 << 26) /* Bit 26: Voltage Support 1.8 */
                                                   /* Bits 27-31: Reserved */

/* Force Event Register */

#define SDHCI_FEVT_AC12NE                (1 << 0)  /* Bit 0:  Force Event Auto Command 12 Not Executed */
#define SDHCI_FEVT_AC12TOE               (1 << 1)  /* Bit 1:  Force Event Auto Command 12 Time Out Error */
#define SDHCI_FEVT_AC12CE                (1 << 2)  /* Bit 2:  Force Event Auto Command 12 CRC Error */
#define SDHCI_FEVT_AC12EBE               (1 << 3)  /* Bit 3:  Force Event Auto Command 12 End Bit Error */
#define SDHCI_FEVT_AC12IE                (1 << 4)  /* Bit 4:  Force Event Auto Command 12 Index Error */
                                                   /* Bits 5-6: Reserved */

#define SDHCI_FEVT_CNIBAC12E             (1 << 7)  /* Bit 7:  Force Event Command Not Executed By Auto Command 12 Error */
                                                   /* Bits 8-15: Reserved */

#define SDHCI_FEVT_CTOE                  (1 << 16) /* Bit 16: Force Event Command Time Out Error */
#define SDHCI_FEVT_CCE                   (1 << 17) /* Bit 17: Force Event Command CRC Error */
#define SDHCI_FEVT_CEBE                  (1 << 18) /* Bit 18: Force Event Command End Bit Error */
#define SDHCI_FEVT_CIE                   (1 << 19) /* Bit 19: Force Event Command Index Error */
#define SDHCI_FEVT_DTOE                  (1 << 20) /* Bit 20: Force Event Data Time Out Error */
#define SDHCI_FEVT_DCE                   (1 << 21) /* Bit 21: Force Event Data CRC Error */
#define SDHCI_FEVT_DEBE                  (1 << 22) /* Bit 22: Force Event Data End Bit Error */
                                                   /* Bit 23: Reserved */

#define SDHCI_FEVT_AC12E                 (1 << 24) /* Bit 24: Force Event Auto Command 12 Error */
                                                   /* Bits 25-27: Reserved */

#define SDHCI_FEVT_DMAE                  (1 << 28) /* Bit 28: Force Event DMA Error */
                                                   /* Bits 29-30: Reserved */

#define SDHCI_FEVT_CINT                  (1 << 31) /* Bit 31: Force Event Card Interrupt */

/* ADMA Error Status Register */

#define SDHCI_ADMAES_SHIFT               (0)       /* Bits 0-1: ADMA Error State (when ADMA Error is occurred) */
#define SDHCI_ADMAES_MASK                (3 << SDHCI_ADMAES_ADMAES_SHIFT)
#define SDHCI_ADMAES_STOP                (0 << SDHCI_ADMAES_ADMAES_SHIFT) /* Stop DMA */
#define SDHCI_ADMAES_FDS                 (1 << SDHCI_ADMAES_ADMAES_SHIFT) /* Fetch descriptor */
#define SDHCI_ADMAES_CADR                (2 << SDHCI_ADMAES_ADMAES_SHIFT) /* Change address */
#define SDHCI_ADMAES_TFR                 (3 << SDHCI_ADMAES_ADMAES_SHIFT) /* Transfer data */

#define SDHCI_ADMAES_LME                 (1 << 2)  /* Bit 2:  ADMA Length Mismatch Error */
#define SDHCI_ADMAES_DCE                 (1 << 3)  /* Bit 3:  ADMA Descriptor Error */
                                                   /* Bits 4-31: Reserved */

/* ADMA System Address Register */

#define SDHCI_ADSADDR_SHIFT              (1)       /* Bits 1-31: ADMA System Address */
#define SDHCI_ADSADDR_MASK               (0xfffffffe)
                                                   /* Bits 0-1: Reserved */

/* Vendor Specific Register */

#define SDHCI_VENDOR_EXTDMAEN            (1 << 0)  /* Bit 0:  External DMA Request Enable */
#define SDHCI_VENDOR_EXBLKNU             (1 << 1)  /* Bit 1:  Exact block number block read enable for SDIO CMD53 */
                                                   /* Bits 2-15: Reserved */

#define SDHCI_VENDOR_INTSTVAL_SHIFT      (16)      /* Bits 16-23: Internal State Value */
#define SDHCI_VENDOR_INTSTVAL_MASK       (0xff << SDHCI_VENDOR_INTSTVAL_SHIFT)
                                                   /* Bits 24-31: Reserved */

/* User Define1 Control Register */

#define SDHCI_UDEF1_SDCLKI_SEL           (1 << 0)
#define SDHCI_UDEF1_SDCLKI_SEL_EXT       (1 << 0)
#define SDHCI_UDEF1_SDCLKI_SEL_INT       (0 << 0)
#define SDHCI_UDEF1_SDCLK_SEL            (1 << 1)
#define SDHCI_UDEF1_SDCLK_SEL_EXT        (1 << 1)
#define SDHCI_UDEF1_SDCLK_SEL_INT        (0 << 1)
#define SDHCI_UDEF1_TAP_SEL_SHIFT        (4)
#define SDHCI_UDEF1_TAP_SEL_MASK         (0x1f << SDHCI_UDEF1_TAP_SEL_SHIFT)
#define SDHCI_UDEF1_TAP_SEL              (1 << 12)
#define SDHCI_UDEF1_TAP_SEL_SW           (1 << 12)
#define SDHCI_UDEF1_TAP_SEL_HW           (0 << 12)
#define SDHCI_UDEF1_DAT_DLY_BUF          (1 << 24)
#define SDHCI_UDEF1_DAT_DLY_BUF_ON       (1 << 24)
#define SDHCI_UDEF1_DAT_DLY_BUF_OFF      (0 << 24)

/* User Define2 Control Register */

#define SDHCI_UDEF2_CLK_DLY_SHIFT        (0)
#define SDHCI_UDEF2_CLK_DLY_MASK         (0x7 << SDHCI_UDEF2_CLK_DLY_SHIFT)
#define SDHCI_UDEF2_CMD_EDGE_DET_ON      (1 << 4)
#define SDHCI_UDEF2_CMD_EDGE_DET_OFF     (0 << 4)
#define SDHCI_UDEF2_DAT_DIR_ACT_HI       (0x0 << 8)
#define SDHCI_UDEF2_DAT_DIR_ACT_LOW      (0x7 << 8)
#define SDHCI_UDEF2_TTCLK_DIV1           (1 << 16)
#define SDHCI_UDEF2_TTCLK_DIV2           (0 << 16)
#define SDHCI_UDEF2_CLKI_SEL             (1 << 20)
#define SDHCI_UDEF2_CLKI_SEL_EXT         (1 << 20)
#define SDHCI_UDEF2_CLKI_SEL_INT         (0 << 20)
#define SDHCI_UDEF2_CMD_SEL              (1 << 24)
#define SDHCI_UDEF2_CMD_SEL_CLKI         (1 << 24)
#define SDHCI_UDEF2_CMD_SEL_INT          (0 << 24)
#define SDHCI_UDEF2_FORCE_1p8V_EN        (1 << 31)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct sdio_dev_s;
/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_sdhci_initialize
 *
 * Description:
 *   Initialize SDIO for operation.
 *
 * Input Parameters:
 *   slotno - Not used.
 *
 * Returned Values:
 *   A reference to an SDIO interface structure.
 *   NULL is returned on failures.
 *
 ****************************************************************************/

FAR struct sdio_dev_s *cxd56_sdhci_initialize(int slotno);

/****************************************************************************
 * Name: cxd56_sdhci_finalize
 *
 * Description:
 *   Finalize SDIO for operation.
 *
 * Input Parameters:
 *   slotno - Not used.
 *
 * Returned Values:
 *   A reference to an SDIO interface structure.
 *   NULL is returned on failures.
 *
 ****************************************************************************/

FAR struct sdio_dev_s *cxd56_sdhci_finalize(int slotno);

/****************************************************************************
 * Name: cxd56_sdhci_mediachange
 *
 * Description:
 *   Called by board-specific logic -- possibly from an interrupt handler --
 *   in order to signal to the driver that a card has been inserted or
 *   removed from the slot
 *
 * Input Parameters:
 *   dev        - An instance of the SDIO driver device state structure.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void cxd56_sdhci_mediachange(FAR struct sdio_dev_s *dev);

/****************************************************************************
 * Name: sdio_wrprotect
 *
 * Description:
 *   Called by board-specific logic to report if the card in the slot is
 *   mechanically write protected.
 *
 * Input Parameters:
 *   dev       - An instance of the SDIO driver device state structure.
 *   wrprotect - true is a card is writeprotected.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void cxd56_sdhci_wrprotect(FAR struct sdio_dev_s *dev, bool wrprotect);

/****************************************************************************
 * Name: cxd56_sdio_resetstatus
 *
 * Description:
 *   Reset SDIO status.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 ****************************************************************************/

void cxd56_sdio_resetstatus(FAR struct sdio_dev_s *dev);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_SDHCI_H */
