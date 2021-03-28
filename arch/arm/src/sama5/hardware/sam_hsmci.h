/****************************************************************************
 * arch/arm/src/sama5/hardware/sam_hsmci.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_HSMCI_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_HSMCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* HSMCI register offsets ***************************************************/

#define SAM_HSMCI_CR_OFFSET           0x0000 /* Control Register */
#define SAM_HSMCI_MR_OFFSET           0x0004 /* Mode Register */
#define SAM_HSMCI_DTOR_OFFSET         0x0008 /* Data Timeout Register */
#define SAM_HSMCI_SDCR_OFFSET         0x000c /* SD/SDIO Card Register */
#define SAM_HSMCI_ARGR_OFFSET         0x0010 /* Argument Register */
#define SAM_HSMCI_CMDR_OFFSET         0x0014 /* Command Register */
#define SAM_HSMCI_BLKR_OFFSET         0x0018 /* Block Register */
#define SAM_HSMCI_CSTOR_OFFSET        0x001c /* Completion Signal Timeout Register */
#define SAM_HSMCI_RSPR0_OFFSET        0x0020 /* Response Register 0 */
#define SAM_HSMCI_RSPR1_OFFSET        0x0024 /* Response Register 1 */
#define SAM_HSMCI_RSPR2_OFFSET        0x0028 /* Response Register 2 */
#define SAM_HSMCI_RSPR3_OFFSET        0x002c /* Response Register 3 */
#define SAM_HSMCI_RDR_OFFSET          0x0030 /* Receive Data Register */
#define SAM_HSMCI_TDR_OFFSET          0x0034 /* Transmit Data Register  */
                                             /* 0x0038-0x003c: Reserved */
#define SAM_HSMCI_SR_OFFSET           0x0040 /* Status Register */
#define SAM_HSMCI_IER_OFFSET          0x0044 /* Interrupt Enable Register */
#define SAM_HSMCI_IDR_OFFSET          0x0048 /* Interrupt Disable Register */
#define SAM_HSMCI_IMR_OFFSET          0x004c /* Interrupt Mask Register */
#define SAM_HSMCI_DMA_OFFSET          0x0050 /* DMA Configuration Register */
#define SAM_HSMCI_CFG_OFFSET          0x0054 /* Configuration Register */
                                             /* 0x0058-0x00e0: Reserved */
#define SAM_HSMCI_WPMR_OFFSET         0x00e4 /* Write Protection Mode Register */
#define SAM_HSMCI_WPSR_OFFSET         0x00e8 /* Write Protection Status Register */
                                             /* 0x00ec-0x00fc: Reserved */
                                             /* 0x0100-0x0124: Reserved for PCD registers */
#define SAM_HSMCI_FIFO_OFFSET         0x0200 /* 0x0200-0x05fc FIFO Memory Aperture */

/* HSMCI register addresses *************************************************/

#define SAM_HSMCI0_CR                 (SAM_HSMCI0_VBASE+SAM_HSMCI_CR_OFFSET)
#define SAM_HSMCI0_MR                 (SAM_HSMCI0_VBASE+SAM_HSMCI_MR_OFFSET)
#define SAM_HSMCI0_DTOR               (SAM_HSMCI0_VBASE+SAM_HSMCI_DTOR_OFFSET)
#define SAM_HSMCI0_SDCR               (SAM_HSMCI0_VBASE+SAM_HSMCI_SDCR_OFFSET)
#define SAM_HSMCI0_ARGR               (SAM_HSMCI0_VBASE+SAM_HSMCI_ARGR_OFFSET)
#define SAM_HSMCI0_CMDR               (SAM_HSMCI0_VBASE+SAM_HSMCI_CMDR_OFFSET)
#define SAM_HSMCI0_BLKR               (SAM_HSMCI0_VBASE+SAM_HSMCI_BLKR_OFFSET)
#define SAM_HSMCI0_CSTOR              (SAM_HSMCI0_VBASE+SAM_HSMCI_CSTOR_OFFSET)
#define SAM_HSMCI0_RSPR0              (SAM_HSMCI0_VBASE+SAM_HSMCI_RSPR0_OFFSET)
#define SAM_HSMCI0_RSPR1              (SAM_HSMCI0_VBASE+SAM_HSMCI_RSPR1_OFFSET)
#define SAM_HSMCI0_RSPR2              (SAM_HSMCI0_VBASE+SAM_HSMCI_RSPR2_OFFSET)
#define SAM_HSMCI0_RSPR3              (SAM_HSMCI0_VBASE+SAM_HSMCI_RSPR3_OFFSET)
#define SAM_HSMCI0_RDR                (SAM_HSMCI0_VBASE+SAM_HSMCI_RDR_OFFSET)
#define SAM_HSMCI0_TDR                (SAM_HSMCI0_VBASE+SAM_HSMCI_TDR_OFFSET)
#define SAM_HSMCI0_SR                 (SAM_HSMCI0_VBASE+SAM_HSMCI_SR_OFFSET)
#define SAM_HSMCI0_IER                (SAM_HSMCI0_VBASE+SAM_HSMCI_IER_OFFSET)
#define SAM_HSMCI0_IDR                (SAM_HSMCI0_VBASE+SAM_HSMCI_IDR_OFFSET)
#define SAM_HSMCI0_IMR                (SAM_HSMCI0_VBASE+SAM_HSMCI_IMR_OFFSET)
#define SAM_HSMCI0_DMA                (SAM_HSMCI0_VBASE+SAM_HSMCI_DMA_OFFSET)
#define SAM_HSMCI0_CFG                (SAM_HSMCI0_VBASE+SAM_HSMCI_CFG_OFFSET)
#define SAM_HSMCI0_WPMR               (SAM_HSMCI0_VBASE+SAM_HSMCI_WPMR_OFFSET)
#define SAM_HSMCI0_WPSR               (SAM_HSMCI0_VBASE+SAM_HSMCI_WPSR_OFFSET)
#define SAM_HSMCI0_FIFO               (SAM_HSMCI0_VBASE+SAM_HSMCI_FIFO_OFFSET)

#define SAM_HSMCI1_CR                 (SAM_HSMCI1_VBASE+SAM_HSMCI_CR_OFFSET)
#define SAM_HSMCI1_MR                 (SAM_HSMCI1_VBASE+SAM_HSMCI_MR_OFFSET)
#define SAM_HSMCI1_DTOR               (SAM_HSMCI1_VBASE+SAM_HSMCI_DTOR_OFFSET)
#define SAM_HSMCI1_SDCR               (SAM_HSMCI1_VBASE+SAM_HSMCI_SDCR_OFFSET)
#define SAM_HSMCI1_ARGR               (SAM_HSMCI1_VBASE+SAM_HSMCI_ARGR_OFFSET)
#define SAM_HSMCI1_CMDR               (SAM_HSMCI1_VBASE+SAM_HSMCI_CMDR_OFFSET)
#define SAM_HSMCI1_BLKR               (SAM_HSMCI1_VBASE+SAM_HSMCI_BLKR_OFFSET)
#define SAM_HSMCI1_CSTOR              (SAM_HSMCI1_VBASE+SAM_HSMCI_CSTOR_OFFSET)
#define SAM_HSMCI1_RSPR0              (SAM_HSMCI1_VBASE+SAM_HSMCI_RSPR0_OFFSET)
#define SAM_HSMCI1_RSPR1              (SAM_HSMCI1_VBASE+SAM_HSMCI_RSPR1_OFFSET)
#define SAM_HSMCI1_RSPR2              (SAM_HSMCI1_VBASE+SAM_HSMCI_RSPR2_OFFSET)
#define SAM_HSMCI1_RSPR3              (SAM_HSMCI1_VBASE+SAM_HSMCI_RSPR3_OFFSET)
#define SAM_HSMCI1_RDR                (SAM_HSMCI1_VBASE+SAM_HSMCI_RDR_OFFSET)
#define SAM_HSMCI1_TDR                (SAM_HSMCI1_VBASE+SAM_HSMCI_TDR_OFFSET)
#define SAM_HSMCI1_SR                 (SAM_HSMCI1_VBASE+SAM_HSMCI_SR_OFFSET)
#define SAM_HSMCI1_IER                (SAM_HSMCI1_VBASE+SAM_HSMCI_IER_OFFSET)
#define SAM_HSMCI1_IDR                (SAM_HSMCI1_VBASE+SAM_HSMCI_IDR_OFFSET)
#define SAM_HSMCI1_IMR                (SAM_HSMCI1_VBASE+SAM_HSMCI_IMR_OFFSET)
#define SAM_HSMCI1_DMA                (SAM_HSMCI1_VBASE+SAM_HSMCI_DMA_OFFSET)
#define SAM_HSMCI1_CFG                (SAM_HSMCI1_VBASE+SAM_HSMCI_CFG_OFFSET)
#define SAM_HSMCI1_WPMR               (SAM_HSMCI1_VBASE+SAM_HSMCI_WPMR_OFFSET)
#define SAM_HSMCI1_WPSR               (SAM_HSMCI1_VBASE+SAM_HSMCI_WPSR_OFFSET)
#define SAM_HSMCI1_FIFO               (SAM_HSMCI1_VBASE+SAM_HSMCI_FIFO_OFFSET)

#ifdef CONFIG_SAMA5_HAVE_HSMCI2
#  define SAM_HSMCI2_CR               (SAM_HSMCI2_VBASE+SAM_HSMCI_CR_OFFSET)
#  define SAM_HSMCI2_MR               (SAM_HSMCI2_VBASE+SAM_HSMCI_MR_OFFSET)
#  define SAM_HSMCI2_DTOR             (SAM_HSMCI2_VBASE+SAM_HSMCI_DTOR_OFFSET)
#  define SAM_HSMCI2_SDCR             (SAM_HSMCI2_VBASE+SAM_HSMCI_SDCR_OFFSET)
#  define SAM_HSMCI2_ARGR             (SAM_HSMCI2_VBASE+SAM_HSMCI_ARGR_OFFSET)
#  define SAM_HSMCI2_CMDR             (SAM_HSMCI2_VBASE+SAM_HSMCI_CMDR_OFFSET)
#  define SAM_HSMCI2_BLKR             (SAM_HSMCI2_VBASE+SAM_HSMCI_BLKR_OFFSET)
#  define SAM_HSMCI2_CSTOR            (SAM_HSMCI2_VBASE+SAM_HSMCI_CSTOR_OFFSET)
#  define SAM_HSMCI2_RSPR0            (SAM_HSMCI2_VBASE+SAM_HSMCI_RSPR0_OFFSET)
#  define SAM_HSMCI2_RSPR1            (SAM_HSMCI2_VBASE+SAM_HSMCI_RSPR1_OFFSET)
#  define SAM_HSMCI2_RSPR2            (SAM_HSMCI2_VBASE+SAM_HSMCI_RSPR2_OFFSET)
#  define SAM_HSMCI2_RSPR3            (SAM_HSMCI2_VBASE+SAM_HSMCI_RSPR3_OFFSET)
#  define SAM_HSMCI2_RDR              (SAM_HSMCI2_VBASE+SAM_HSMCI_RDR_OFFSET)
#  define SAM_HSMCI2_TDR              (SAM_HSMCI2_VBASE+SAM_HSMCI_TDR_OFFSET)
#  define SAM_HSMCI2_SR               (SAM_HSMCI2_VBASE+SAM_HSMCI_SR_OFFSET)
#  define SAM_HSMCI2_IER              (SAM_HSMCI2_VBASE+SAM_HSMCI_IER_OFFSET)
#  define SAM_HSMCI2_IDR              (SAM_HSMCI2_VBASE+SAM_HSMCI_IDR_OFFSET)
#  define SAM_HSMCI2_IMR              (SAM_HSMCI2_VBASE+SAM_HSMCI_IMR_OFFSET)
#  define SAM_HSMCI2_DMA              (SAM_HSMCI2_VBASE+SAM_HSMCI_DMA_OFFSET)
#  define SAM_HSMCI2_CFG              (SAM_HSMCI2_VBASE+SAM_HSMCI_CFG_OFFSET)
#  define SAM_HSMCI2_WPMR             (SAM_HSMCI2_VBASE+SAM_HSMCI_WPMR_OFFSET)
#  define SAM_HSMCI2_WPSR             (SAM_HSMCI2_VBASE+SAM_HSMCI_WPSR_OFFSET)
#  define SAM_HSMCI2_FIFO             (SAM_HSMCI2_VBASE+SAM_HSMCI_FIFO_OFFSET)
#endif

/* HSMCI register bit definitions *******************************************/

/* HSMCI Control Register */

#define HSMCI_CR_MCIEN                (1 << 0)  /* Bit 0:  Multi-Media Interface Enable */
#define HSMCI_CR_MCIDIS               (1 << 1)  /* Bit 1:  Multi-Media Interface Disable */
#define HSMCI_CR_PWSEN                (1 << 2)  /* Bit 2:  Power Save Mode Enable */
#define HSMCI_CR_PWSDIS               (1 << 3)  /* Bit 3:  Power Save Mode Disable */
#define HSMCI_CR_SWRST                (1 << 7)  /* Bit 7:  Software Reset */

/* HSMCI Mode Register */

#define HSMCI_MR_CLKDIV_SHIFT         (0)       /* Bits 0-7: Clock Divider */
#define HSMCI_MR_CLKDIV_MASK          (0xff << HSMCI_MR_CLKDIV_SHIFT)
#define HSMCI_MR_PWSDIV_SHIFT         (8)       /* Bits 8-10: Power Saving Divider */
#define HSMCI_MR_PWSDIV_MASK          (7 << HSMCI_MR_PWSDIV_SHIFT)
#  define HSMCI_MR_PWSDIV_MAX         (7 << HSMCI_MR_PWSDIV_SHIFT)
#define HSMCI_MR_RDPROOF              (1 << 11) /* Bit 11: Read Proof Enable */
#define HSMCI_MR_WRPROOF              (1 << 12) /* Bit 12: Write Proof Enable */
#define HSMCI_MR_FBYTE                (1 << 13) /* Bit 13: Force Byte Transfer */
#define HSMCI_MR_PADV                 (1 << 14) /* Bit 14: Padding Value */
#define HSMCI_MR_CLKODD               (1 << 16) /* Bit 15: Clock divider is odd */

/* HSMCI Data Timeout Register */

#define HSMCI_DTOR_DTOCYC_SHIFT       (0)       /* Bits 0-3: Data Timeout Cycle Number */
#define HSMCI_DTOR_DTOCYC_MASK        (15 << HSMCI_DTOR_DTOCYC_SHIFT)
#  define HSMCI_DTOR_DTOCYC_MAX       (15 << HSMCI_DTOR_DTOCYC_SHIFT)
#define HSMCI_DTOR_DTOMUL_SHIFT       (4)       /* Bits 4-6: Data Timeout Multiplier */
#define HSMCI_DTOR_DTOMUL_MASK        (7 << HSMCI_DTOR_DTOMUL_SHIFT)
#  define HSMCI_DTOR_DTOMUL_1         (0 << HSMCI_DTOR_DTOMUL_SHIFT)
#  define HSMCI_DTOR_DTOMUL_16        (1 << HSMCI_DTOR_DTOMUL_SHIFT)
#  define HSMCI_DTOR_DTOMUL_128       (2 << HSMCI_DTOR_DTOMUL_SHIFT)
#  define HSMCI_DTOR_DTOMUL_256       (3 << HSMCI_DTOR_DTOMUL_SHIFT)
#  define HSMCI_DTOR_DTOMUL_1024      (4 << HSMCI_DTOR_DTOMUL_SHIFT)
#  define HSMCI_DTOR_DTOMUL_4096      (5 << HSMCI_DTOR_DTOMUL_SHIFT)
#  define HSMCI_DTOR_DTOMUL_65536     (6 << HSMCI_DTOR_DTOMUL_SHIFT)
#  define HSMCI_DTOR_DTOMUL_1048576   (7 << HSMCI_DTOR_DTOMUL_SHIFT)
#  define HSMCI_DTOR_DTOMUL_MAX       (7 << HSMCI_DTOR_DTOMUL_SHIFT)

/* HSMCI SDCard/SDIO Register */

#define HSMCI_SDCR_SDCSEL_SHIFT       (0)       /* Bits 0-1: SDCard/SDIO Slot */

#define HSMCI_SDCR_SDCSEL_MASK        (3 << HSMCI_SDCR_SDCSEL_SHIFT) /* Slot A is selected */
#  define HSMCI_SDCR_SDCSEL_SLOTA     (0 << HSMCI_SDCR_SDCSEL_SHIFT) /* Reserved */
#  define HSMCI_SDCR_SDCSEL_SLOTB     (1 << HSMCI_SDCR_SDCSEL_SHIFT) /* Reserved */
#  define HSMCI_SDCR_SDCSEL_SLOTC     (2 << HSMCI_SDCR_SDCSEL_SHIFT) /* Reserved */
#  define HSMCI_SDCR_SDCSEL_SLOTD     (3 << HSMCI_SDCR_SDCSEL_SHIFT) /* Reserved */

#define HSMCI_SDCR_SDCBUS_SHIFT       (6)       /* Bits 6-7: SDCard/SDIO Bus Width */
#define HSMCI_SDCR_SDCBUS_MASK        (3 << HSMCI_SDCR_SDCBUS_SHIFT)
#  define HSMCI_SDCR_SDCBUS_1BIT      (0 << HSMCI_SDCR_SDCBUS_SHIFT)
#  define HSMCI_SDCR_SDCBUS_4BIT      (2 << HSMCI_SDCR_SDCBUS_SHIFT)
#  define HSMCI_SDCR_SDCBUS_8BIT      (3 << HSMCI_SDCR_SDCBUS_SHIFT)

/* HSMCI Argument Register (32-bit value) */

/* HSMCI Command Register */

#define HSMCI_CMDR_CMDNB_SHIFT        (0)       /* Bits 0-5: Command Number */
#define HSMCI_CMDR_CMDNB_MASK         (63 << HSMCI_CMDR_CMDNB_SHIFT)
#define HSMCI_CMDR_RSPTYP_SHIFT       (6)       /* Bits 6-7: Response Type */
#define HSMCI_CMDR_RSPTYP_MASK        (3 << HSMCI_CMDR_RSPTYP_SHIFT)
#  define HSMCI_CMDR_RSPTYP_NONE      (0 << HSMCI_CMDR_RSPTYP_SHIFT) /* No response */
#  define HSMCI_CMDR_RSPTYP_48BIT     (1 << HSMCI_CMDR_RSPTYP_SHIFT) /* 48-bit response */
#  define HSMCI_CMDR_RSPTYP_136BIT    (2 << HSMCI_CMDR_RSPTYP_SHIFT) /* 136-bit response */
#  define HSMCI_CMDR_RSPTYP_R1B       (3 << HSMCI_CMDR_RSPTYP_SHIFT) /* R1b response type */

#define HSMCI_CMDR_SPCMD_SHIFT        (8)       /* Bits 8-10: Special Command */
#define HSMCI_CMDR_SPCMD_MASK         (7 << HSMCI_CMDR_SPCMD_SHIFT)
#  define HSMCI_CMDR_SPCMD_STD        (0 << HSMCI_CMDR_SPCMD_SHIFT) /* Not a special CMD */
#  define HSMCI_CMDR_SPCMD_INIT       (1 << HSMCI_CMDR_SPCMD_SHIFT) /* Initialization CMD */
#  define HSMCI_CMDR_SPCMD_SYNC       (2 << HSMCI_CMDR_SPCMD_SHIFT) /* Synchronized CMD */
#  define HSMCI_CMDR_SPCMD_CEATAC     (3 << HSMCI_CMDR_SPCMD_SHIFT) /* CE-ATA Completion Signal disable CMD */
#  define HSMCI_CMDR_SPCMD_ITCMD      (4 << HSMCI_CMDR_SPCMD_SHIFT) /* Interrupt command */
#  define HSMCI_CMDR_SPCMD_ITRESP     (5 << HSMCI_CMDR_SPCMD_SHIFT) /* Interrupt response */
#  define HSMCI_CMDR_SPCMD_BOR        (6 << HSMCI_CMDR_SPCMD_SHIFT) /* Boot Operation Request */
#  define HSMCI_CMDR_SPCMD_EBO        (7 << HSMCI_CMDR_SPCMD_SHIFT) /* End Boot Operation */

#define HSMCI_CMDR_OPDCMD             (1 << 11) /* Bit 11: Open Drain Command */
#define HSMCI_CMDR_MAXLAT             (1 << 12) /* Bit 12: Max Latency for Command to Response */
#define HSMCI_CMDR_TRCMD_SHIFT        (16)      /* Bits 16-17: Transfer Command */
#define HSMCI_CMDR_TRCMD_MASK         (3 << HSMCI_CMDR_TRCMD_SHIFT)
#  define HSMCI_CMDR_TRCMD_NONE       (0 << HSMCI_CMDR_TRCMD_SHIFT) /* No data transfer */
#  define HSMCI_CMDR_TRCMD_START      (1 << HSMCI_CMDR_TRCMD_SHIFT) /* Start data transfer */
#  define HSMCI_CMDR_TRCMD_STOP       (2 << HSMCI_CMDR_TRCMD_SHIFT) /* Stop data transfer */

#define HSMCI_CMDR_TRDIR              (1 << 18) /* Bit 18: Transfer Direction */
#  define HSMCI_CMDR_TRDIR_WRITE      (0 << 18)
#  define HSMCI_CMDR_TRDIR_READ       (1 << 18)
#define HSMCI_CMDR_TRTYP_SHIFT        (19)      /* Bits 19-21: Transfer Type */
#define HSMCI_CMDR_TRTYP_MASK         (7 << HSMCI_CMDR_TRTYP_SHIFT)
#  define HSMCI_CMDR_TRTYP_SINGLE     (0 << HSMCI_CMDR_TRTYP_SHIFT) /* MMC/SDCard Single Block */
#  define HSMCI_CMDR_TRTYP_MULTIPLE   (1 << HSMCI_CMDR_TRTYP_SHIFT) /* MMC/SDCard Multiple Block */
#  define HSMCI_CMDR_TRTYP_STREAM     (2 << HSMCI_CMDR_TRTYP_SHIFT) /* MMC Stream */
#  define HSMCI_CMDR_TRTYP_SDIOBYTE   (4 << HSMCI_CMDR_TRTYP_SHIFT) /* SDIO Byte */
#  define HSMCI_CMDR_TRTYP_SDIOBLK    (5 << HSMCI_CMDR_TRTYP_SHIFT) /* SDIO Block */

#define HSMCI_CMDR_IOSPCMD_SHIFT      (24)      /* Bits 24-25: SDIO Special Command */
#define HSMCI_CMDR_IOSPCMD_MASK       (3 << HSMCI_CMDR_IOSPCMD_SHIFT)
#  define HSMCI_CMDR_IOSPCMD_STD      (0 << HSMCI_CMDR_IOSPCMD_SHIFT) /* Not an SDIO Special Command */
#  define HSMCI_CMDR_IOSPCMD_SUSPEND  (1 << HSMCI_CMDR_IOSPCMD_SHIFT) /* SDIO Suspend Command */
#  define HSMCI_CMDR_IOSPCMD_RESUME   (2 << HSMCI_CMDR_IOSPCMD_SHIFT) /* SDIO Resume Command */

#define HSMCI_CMDR_ATACS              (1 << 26) /* Bit 26: ATA with Command Completion Signal */
#define HSMCI_CMDR_BOOTACK            (1 << 27) /* Bit 27: Boot Operation Acknowledge */

/* HSMCI Block Register */

#define HSMCI_BLKR_BCNT_SHIFT         (0)       /* Bits 0-15: MMC/SDIO Block Count - SDIO Byte Count */
#define HSMCI_BLKR_BCNT_MASK          (0xffff << HSMCI_BLKR_BCNT_SHIFT)
#  define HSMCI_BLKR_BCNT(n)          ((uint32_t)(n) << HSMCI_BLKR_BCNT_SHIFT)
#define HSMCI_BLKR_BLKLEN_SHIFT       (16)      /* Bits 16-31: Data Block Length */
#define HSMCI_BLKR_BLKLEN_MASK        (0xffff << HSMCI_BLKR_BLKLEN_SHIFT)
#  define HSMCI_BLKR_BLKLEN(n)        ((uint32_t)(n) << HSMCI_BLKR_BLKLEN_SHIFT)

/* HSMCI Completion Signal Timeout Register */

#define HSMCI_CSTOR_CSTOCYC_SHIFT     (0)       /* Bits 0-3: Completion Signal Timeout Cycle Number */
#define HSMCI_CSTOR_CSTOCYC_MASK      (15 << HSMCI_CSTOR_CSTOCYC_SHIFT)
#  define HSMCI_CSTOR_CSTOCYC(n)      ((uint32_t)(n) << HSMCI_CSTOR_CSTOCYC_SHIFT)
#define HSMCI_CSTOR_CSTOMUL_SHIFT     (4)       /* Bits 4-6: Completion Signal Timeout Multiplier */
#define HSMCI_CSTOR_CSTOMUL_MASK      (7 << HSMCI_CSTOR_CSTOMUL_SHIFT)
#  define HSMCI_CSTOR_CSTOMUL_1       (0 << HSMCI_CSTOR_CSTOMUL_SHIFT)
#  define HSMCI_CSTOR_CSTOMUL_16      (1 << HSMCI_CSTOR_CSTOMUL_SHIFT)
#  define HSMCI_CSTOR_CSTOMUL_128     (2 << HSMCI_CSTOR_CSTOMUL_SHIFT)
#  define HSMCI_CSTOR_CSTOMUL_256     (3 << HSMCI_CSTOR_CSTOMUL_SHIFT)
#  define HSMCI_CSTOR_CSTOMUL_1024    (4 << HSMCI_CSTOR_CSTOMUL_SHIFT)
#  define HSMCI_CSTOR_CSTOMUL_4096    (5 << HSMCI_CSTOR_CSTOMUL_SHIFT)
#  define HSMCI_CSTOR_CSTOMUL_65536   (6 << HSMCI_CSTOR_CSTOMUL_SHIFT)
#  define HSMCI_CSTOR_CSTOMUL_1048576 (7 << HSMCI_CSTOR_CSTOMUL_SHIFT)

/* HSMCI Response Registers (32-bit data) */

/* HSMCI Receive Data Registers (32-bit data) */

/* HSMCI Transmit Data Registers (32-bit data) */

/* HSMCI Status Register,
 * HSMCI Interrupt Enable Register, HSMCI Interrupt Disable
 * Register, and HSMCI Interrupt Mask Register common bit-field definitions
 */

#define HSMCI_INT_CMDRDY              (1 << 0)  /* Bit 0:  Command Ready */
#define HSMCI_INT_RXRDY               (1 << 1)  /* Bit 1:  Receiver Ready */
#define HSMCI_INT_TXRDY               (1 << 2)  /* Bit 2:  Transmit Ready */
#define HSMCI_INT_BLKE                (1 << 3)  /* Bit 3:  Data Block Ended */
#define HSMCI_INT_DTIP                (1 << 4)  /* Bit 4:  Data Transfer in Progress */
#define HSMCI_INT_NOTBUSY             (1 << 5)  /* Bit 6:  HSMCI Not Busy */
#define HSMCI_INT_SDIOIRQA            (1 << 8)  /* Bit 8:  SDIO Interrupt for Slot A */

#ifdef ATSAMA5D4
#  define HSMCI_INT_SDIOIRQB          (1 << 9)  /* Bit 9:  SDIO Interrupt for Slot B */
#endif

#define HSMCI_INT_SDIOWAIT            (1 << 12) /* Bit 12: SDIO Read Wait Operation Status */
#define HSMCI_INT_CSRCV               (1 << 13) /* Bit 13: CE-ATA Completion Signal Received */
#define HSMCI_INT_RINDE               (1 << 16) /* Bit 16: Response Index Error */
#define HSMCI_INT_RDIRE               (1 << 17) /* Bit 17: Response Direction Error */
#define HSMCI_INT_RCRCE               (1 << 18) /* Bit 18: Response CRC Error */
#define HSMCI_INT_RENDE               (1 << 19) /* Bit 19: Response End Bit Error */
#define HSMCI_INT_RTOE                (1 << 20) /* Bit 20: Response Time-out */
#define HSMCI_INT_DCRCE               (1 << 21) /* Bit 21: Data CRC Error */
#define HSMCI_INT_DTOE                (1 << 22) /* Bit 22: Data Time-out Error */
#define HSMCI_INT_CSTOE               (1 << 23) /* Bit 23: Completion Signal Time-out Error */
#define HSMCI_INT_BLKOVRE             (1 << 24) /* Bit 24: DMA Block Overrun Error */
#define HSMCI_INT_DMADONE             (1 << 25) /* Bit 25: DMA Transfer done */
#define HSMCI_INT_FIFOEMPTY           (1 << 26) /* Bit 26: FIFO empty flag */
#define HSMCI_INT_XFRDONE             (1 << 27) /* Bit 27: Transfer Done flag */
#define HSMCI_INT_ACKRCV              (1 << 28) /* Bit 28: Boot Operation Acknowledge Received */
#define HSMCI_INT_ACKRCVE             (1 << 29) /* Bit 29: Boot Operation Acknowledge Error */
#define HSMCI_INT_OVRE                (1 << 30) /* Bit 30: Overrun */
#define HSMCI_INT_UNRE                (1 << 31) /* Bit 31: Underrun */

/* HSMCI DMA Configuration Register */

#ifdef ATSAMA5D3
#  define HSMCI_DMA_OFFSET_SHIFT      (0)       /* Bits 0-1: DMA Write Buffer Offset */
#  define HSMCI_DMA_OFFSET_MASK       (3 << HSMCI_DMA_OFFSET_SHIFT)
#    define HSMCI_DMA_OFFSET(n)       ((uint32_t)(n) << HSMCI_DMA_OFFSET_SHIFT)
#endif

#define HSMCI_DMA_CHKSIZE_SHIFT       (4)       /* Bits 4-6:  DMA Channel Read and Write Chunk Size */
#define HSMCI_DMA_CHKSIZE_MASK        (7 << HSMCI_DMA_CHKSIZE_SHIFT)
#  define HSMCI_DMA_CHKSIZE_1         (0 << HSMCI_DMA_CHKSIZE_SHIFT) /* 1 data available */
#  define HSMCI_DMA_CHKSIZE_4         (1 << HSMCI_DMA_CHKSIZE_SHIFT) /* 4 data available */
#  define HSMCI_DMA_CHKSIZE_8         (2 << HSMCI_DMA_CHKSIZE_SHIFT) /* 8 data available */
#  define HSMCI_DMA_CHKSIZE_16        (3 << HSMCI_DMA_CHKSIZE_SHIFT) /* 16 data available */
#  define HSMCI_DMA_CHKSIZE_32        (4 << HSMCI_DMA_CHKSIZE_SHIFT) /* 32 data available */

#define HSMCI_DMA_DMAEN               (1 << 8)  /* Bit 8:  DMA Hardware Handshaking Enable */

#ifdef ATSAMA5D3
#  define HSMCI_DMA_ROPT              (1 << 12) /* Bit 12: Read Optimization with padding */
#endif

/* HSMCI Configuration Register */

#define HSMCI_CFG_FIFOMODE            (1 << 0)  /* Bit 0:  HSMCI Internal FIFO control mode */
#define HSMCI_CFG_FERRCTRL            (1 << 4)  /* Bit 4:  Flow Error flag reset control mode */
#define HSMCI_CFG_HSMODE              (1 << 8)  /* Bit 8:  High Speed Mode */
#define HSMCI_CFG_LSYNC               (1 << 12) /* Bit 12: Synchronize on the last block */

/* HSMCI Write Protect Mode Register */

#define HSMCI_WPMR_WPEN               (1 << 0)  /* Bit 0:  Write Protection Enable */
#define HSMCI_WPMR_WPKEY_SHIFT        (8)       /* Bits 8-31: Write Protection Key password */
#define HSMCI_WPMR_WPKEY_MASK         (0x00ffffff << HSMCI_WPMR_WPKEY_SHIFT)
#  define HSMCI_WPMR_WPKEY            (0x004d4349 << HSMCI_WPMR_WPKEY_SHIFT)

/* HSMCI Write Protect Status Register */

#define HSMCI_WPSR_WPVS               (1 << 0)  /* Bit 0: Write Protection Violation Status */
#define HSMCI_WPSR_WPVSRC_SHIFT       (8)       /* Bits 8-23: Write Protection Violation Source */
#define HSMCI_WPSR_WPVSRC_MASK        (0xffff << HSMCI_WPSR_WPVSRC_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_HSMCI_H */
