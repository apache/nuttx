/****************************************************************************
 * arch/arm/src/sama5/hardware/sam_sdmmc.h
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAMA5_SDMMC_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAMA5_SDMMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define SAMA5_SDMMC_DSADDR_OFFSET        0x0000 /* DMA System Address Register */
#define SAMA5_SDMMC_BLKATTR_OFFSET       0x0004 /* Block Size Register */
#define SAMA5_SDMMC_BSR_OFFSET           0x0004 /* Block Size Register */
#define SAMA5_SDMMC_BCR_OFFSET           0x0006 /* Block Count Register */
#define SAMA5_SDMMC_CMDARG_OFFSET        0x0008 /* Command Argument Register */
#define SAMA5_SDMMC_XFERTYP_OFFSET       0x000c /* Transfer Type Register SAMA5: Transfer Mode Register */
#define SAMA5_SDMMC_CMDRSP0_OFFSET       0x0010 /* Command Response 0 */
#define SAMA5_SDMMC_CMDRSP1_OFFSET       0x0014 /* Command Response 1 */
#define SAMA5_SDMMC_CMDRSP2_OFFSET       0x0018 /* Command Response 2 */
#define SAMA5_SDMMC_CMDRSP3_OFFSET       0x001c /* Command Response 3 */
#define SAMA5_SDMMC_DATAPORT_OFFSET      0x0020 /* Buffer Data Port Register */
#define SAMA5_SDMMC_PRSSTAT_OFFSET       0x0024 /* Present State Register */
#define SAMA5_SDMMC_PROCTL_OFFSET        0x0028 /* Protocol Control Register */
#define SAMA5_SDMMC_PWRCTL_OFFSET        0x0029 /* Power Control Control Register */
#define SAMA5_SDMMC_SYSCTL_OFFSET        0x002c /* System Control Register, or Clock Control Register/Timout Control Register */
#define SAMA5_SDMMC_TCR_OFFSET           0x002e /* Timout Control Register (8 bit) */
#define SAMA5_SDMMC_SRR_OFFSET           0x002f /* Software Reset Register (8 bit) */
#define SAMA5_SDMMC_IRQSTAT_OFFSET       0x0030 /* Interrupt Status Register */
#define SAMA5_SDMMC_IRQSTATEN_OFFSET     0x0034 /* Interrupt Status Enable Register */
#define SAMA5_SDMMC_EISTER_OFFSET        0x0036 /* SAMA5: Error Interrupt Status Register */
#define SAMA5_SDMMC_IRQSIGEN_OFFSET      0x0038 /* Interrupt Signal Enable Register */
#define SAMA5_SDMMC_AC12ERR_OFFSET       0x003c /* Auto CMD12 Error Status Register */
#define SAMA5_SDMMC_H2CR_OFFSET          0x003e /* Host Control 2 Register */
#define SAMA5_SDMMC_HTCAPBLT0_OFFSET     0x0040 /* Host Controller Capabilities 0 Register */
#define SAMA5_SDMMC_HTCAPBLT1_OFFSET     0x0044 /* Host Controller Capabilities 1 Register */
#define SAMA5_SDMMC_MIX_OFFSET           0x0048 /* Mixer Control Register SAMA5: Maximum Current Capabililities Register */
#define SAMA5_SDMMC_FEVT_OFFSET          0x0050 /* Force Event Register */
#define SAMA5_SDMMC_ADMAES_OFFSET        0x0054 /* ADMA Error Status Register */
#define SAMA5_SDMMC_ADSADDR_OFFSET       0x0058 /* ADMA System Address Register */
#define SAMA5_SDMMC_DLL_CONTROL_OFFSET   0x0060 /* DLL Control Register */
#define SAMA5_SDMMC_DLL_STATUS_OFFSET    0x0064 /* DLL Status Register */
#define SAMA5_SDMMC_CLK_TUNE_CTRL_OFFSET 0x0068 /* Clock tuning control Register */
                                                /* 0x0070 – 0x00f8: reserved */
#define SAMA5_SDMMC_HOST_VERSION_OFFSET  0x00fe /* Host Controller Version Register */
#define SAMA5_SDMMC_APSR_OFFSET          0x0200 /* Additional Present State Register */
#define SAMA5_SDMMC_MC1R_OFFSET          0x0204 /* e.MMC Control 1 Register */
#define SAMA5_SDMMC_MC2R_OFFSET          0x0205 /* e.MMC Control 2 Register */
#define SAMA5_SDMMC_ACR_OFFSET           0x0208 /* AHB Control Register */
#define SAMA5_SDMMC_CC2R_OFFSET          0x020C /* Clock Control 2 Register */
#define SAMA5_SDMMC_RTC1R_OFFSET         0x0210 /* Retuning Timer Control 1 Register */
#define SAMA5_SDMMC_RTC2R_OFFSET         0x0211 /* Retuning Timer Control 2 Register */
#define SAMA5_SDMMC_RTCVR_OFFSET         0x0214 /* Retuning Timer Counter Value Register */
#define SAMA5_SDMMC_RTISTER_OFFSET       0x0218 /* Retuning Timer Interrupt Status Enable Register */
#define SAMA5_SDMMC_RTISIER_OFFSET       0x0219 /* Retuning Timer Interrupt Signal Enable Register */
#define SAMA5_SDMMC_RTISTR_OFFSET        0x021C /* Retuning Timer Interrupt Status Register */
#define SAMA5_SDMMC_RTSSR_OFFSET         0x021D /* Retuning Timer Status Slots Register */
#define SAMA5_SDMMC_TUNCR_OFFSET         0x0220 /* Tuning Control Register */
                                                /* 0x0224 – 0x022C Reserved */
#define SAMA5_SDMMC_CACR_OFFSET          0x0230 /* Capabilities Control Register */
                                                /* 0x0234 – 0x023C Reserved */
#define SAMA5_SDMMC_CALCR_OFFSET         0x0240 /* Calibration Control Register */

/* Register Addresses *******************************************************/

/* For SDMMC0... */

#define SAMA5_SDMMC0_DSADDR              (SAM_SDMMC0_VBASE + SAMA5_SDMMC_DSADDR_OFFSET)
#define SAMA5_SDMMC0_BLKATTR             (SAM_SDMMC0_VBASE + SAMA5_SDMMC_BLKATTR_OFFSET)
#define SAMA5_SDMMC0_CMDARG              (SAM_SDMMC0_VBASE + SAMA5_SDMMC_CMDARG_OFFSET)
#define SAMA5_SDMMC0_XFERTYP             (SAM_SDMMC0_VBASE + SAMA5_SDMMC_XFERTYP_OFFSET)
#define SAMA5_SDMMC0_CMDRSP0             (SAM_SDMMC0_VBASE + SAMA5_SDMMC_CMDRSP0_OFFSET)
#define SAMA5_SDMMC0_CMDRSP1             (SAM_SDMMC0_VBASE + SAMA5_SDMMC_CMDRSP1_OFFSET)
#define SAMA5_SDMMC0_CMDRSP2             (SAM_SDMMC0_VBASE + SAMA5_SDMMC_CMDRSP2_OFFSET)
#define SAMA5_SDMMC0_CMDRSP3             (SAM_SDMMC0_VBASE + SAMA5_SDMMC_CMDRSP3_OFFSET)
#define SAMA5_SDMMC0_DATAPORT            (SAM_SDMMC0_VBASE + SAMA5_SDMMC_DATAPORT_OFFSET)
#define SAMA5_SDMMC0_PRSSTAT             (SAM_SDMMC0_VBASE + SAMA5_SDMMC_PRSSTAT_OFFSET)
#define SAMA5_SDMMC0_PROCTL              (SAM_SDMMC0_VBASE + SAMA5_SDMMC_PROCTL_OFFSET)
#define SAMA5_SDMMC0_PWRCTL              (SAM_SDMMC0_VBASE + SAMA5_SDMMC_PWRCTL_OFFSET)
#define SAMA5_SDMMC0_SYSCTL              (SAM_SDMMC0_VBASE + SAMA5_SDMMC_SYSCTL_OFFSET)
#define SAMA5_SDMMC0_IRQSTAT             (SAM_SDMMC0_VBASE + SAMA5_SDMMC_IRQSTAT_OFFSET)
#define SAMA5_SDMMC0_IRQSTATEN           (SAM_SDMMC0_VBASE + SAMA5_SDMMC_IRQSTATEN_OFFSET)
#define SAMA5_SDMMC0_IRQSIGEN            (SAM_SDMMC0_VBASE + SAMA5_SDMMC_IRQSIGEN_OFFSET)
#define SAMA5_SDMMC0_AC12ERR             (SAM_SDMMC0_VBASE + SAMA5_SDMMC_AC12ERR_OFFSET)
#define SAMA5_SDMMC0_HTCAPBLT0           (SAM_SDMMC0_VBASE + SAMA5_SDMMC_HTCAPBLT0_OFFSET)
#define SAMA5_SDMMC0_HTCAPBLT1           (SAM_SDMMC0_VBASE + SAMA5_SDMMC_HTCAPBLT1_OFFSET)
#define SAMA5_SDMMC0_MIX                 (SAM_SDMMC0_VBASE + SAMA5_SDMMC_MIX_OFFSET)
#define SAMA5_SDMMC0_FEVT                (SAM_SDMMC0_VBASE + SAMA5_SDMMC_FEVT_OFFSET)
#define SAMA5_SDMMC0_ADMAES              (SAM_SDMMC0_VBASE + SAMA5_SDMMC_ADMAES_OFFSET)
#define SAMA5_SDMMC0_ADSADDR             (SAM_SDMMC0_VBASE + SAMA5_SDMMC_ADSADDR_OFFSET)
#define SAMA5_SDMMC_DLL_CONTROL          (SAM_SDMMC0_VBASE + SAMA5_SDMMC_DLL_CONTROL_OFFSET)
#define SAMA5_SDMMC_DLL_STATUS           (SAM_SDMMC0_VBASE + SAMA5_SDMMC_DLL_STATUS)
#define SAMA5_SDMMC_CLK_TUNE_CTRL        (SAM_SDMMC0_VBASE + SAMA5_SDMMC_CLK_TUNE_CTRL)
#define SAMA5_SDMMC0_VENDOR              (SAM_SDMMC0_VBASE + SAMA5_SDMMC_VENDOR_OFFSET)
#define SAMA5_SDMMC0_MMCBOOT             (SAM_SDMMC0_VBASE + SAMA5_SDMMC_MMCBOOT_OFFSET)
#define SAMA5_SDMMC0_VENDOR2             (SAM_SDMMC0_VBASE + SAMA5_SDMMC_VENDOR2_OFFSET)
#define SAMA5_SDMMC0_TC                  (SAM_SDMMC0_VBASE + SAMA5_SDMMC_TC_OFFSET)

/* For SDMMC1... */

#define SAMA5_SDMMC1_DSADDR              (SAM_SDMMC1_VBASE + SAMA5_SDMMC_DSADDR_OFFSET)
#define SAMA5_SDMMC1_BLKATTR             (SAM_SDMMC1_VBASE + SAMA5_SDMMC_BLKATTR_OFFSET)
#define SAMA5_SDMMC1_CMDARG              (SAM_SDMMC1_VBASE + SAMA5_SDMMC_CMDARG_OFFSET)
#define SAMA5_SDMMC1_XFERTYP             (SAM_SDMMC1_VBASE + SAMA5_SDMMC_XFERTYP_OFFSET)
#define SAMA5_SDMMC1_CMDRSP0             (SAM_SDMMC1_VBASE + SAMA5_SDMMC_CMDRSP0_OFFSET)
#define SAMA5_SDMMC1_CMDRSP1             (SAM_SDMMC1_VBASE + SAMA5_SDMMC_CMDRSP1_OFFSET)
#define SAMA5_SDMMC1_CMDRSP2             (SAM_SDMMC1_VBASE + SAMA5_SDMMC_CMDRSP2_OFFSET)
#define SAMA5_SDMMC1_CMDRSP3             (SAM_SDMMC1_VBASE + SAMA5_SDMMC_CMDRSP3_OFFSET)
#define SAMA5_SDMMC1_DATAPORT            (SAM_SDMMC1_VBASE + SAMA5_SDMMC_DATPORT_OFFSET)
#define SAMA5_SDMMC1_PRSSTAT             (SAM_SDMMC1_VBASE + SAMA5_SDMMC_PRSSTAT_OFFSET)
#define SAMA5_SDMMC1_PROCTL              (SAM_SDMMC1_VBASE + SAMA5_SDMMC_PROCTL_OFFSET)
#define SAMA5_SDMMC1_PWRCTL              (SAM_SDMMC1_VBASE + SAMA5_SDMMC_PWRCTL_OFFSET)
#define SAMA5_SDMMC1_SYSCTL              (SAM_SDMMC1_VBASE + SAMA5_SDMMC_SYSCTL_OFFSET)
#define SAMA5_SDMMC1_IRQSTAT             (SAM_SDMMC1_VBASE + SAMA5_SDMMC_IRQSTAT_OFFSET)
#define SAMA5_SDMMC1_IRQSTATEN           (SAM_SDMMC1_VBASE + SAMA5_SDMMC_IRQSTATEN_OFFSET)
#define SAMA5_SDMMC1_IRQSIGEN            (SAM_SDMMC1_VBASE + SAMA5_SDMMC_IRQSIGEN_OFFSET)
#define SAMA5_SDMMC1_AC12ERR             (SAM_SDMMC1_VBASE + SAMA5_SDMMC_AC12ERR_OFFSET)
#define SAMA5_SDMMC1_HTCAPBLT0           (SAM_SDMMC1_VBASE + SAMA5_SDMMC_HTCAPBLT0_OFFSET)
#define SAMA5_SDMMC1_HTCAPBLT1           (SAM_SDMMC1_VBASE + SAMA5_SDMMC_HTCAPBLT1_OFFSET)
#define SAMA5_SDMMC1_FEVT                (SAM_SDMMC1_VBASE + SAMA5_SDMMC_FEVT_OFFSET)
#define SAMA5_SDMMC1_ADMAES              (SAM_SDMMC1_VBASE + SAMA5_SDMMC_ADMAES_OFFSET)
#define SAMA5_SDMMC1_ADSADDR             (SAM_SDMMC1_VBASE + SAMA5_SDMMC_ADSADDR_OFFSET)
#define SAMA5_SDMMC1_DLL_CONTROL         (SAM_SDMMC1_VBASE + SAMA5_SDMMC_DLL_CONTROL_OFFSET)
#define SAMA5_SDMMC1_DLL_STATUS          (SAM_SDMMC1_VBASE + SAMA5_SDMMC_DLL_STATUS)
#define SAMA5_USSDHC_CLK_TUNE_CTRL       (SAM_SDMMC1_VBASE + SAMA5_SDMMC_CLK_TUNE_CTRL)
#define SAMA5_SDMMC1_VENDOR              (SAM_SDMMC1_VBASE + SAMA5_SDMMC_VENDOR_OFFSET)
#define SAMA5_SDMMC1_MMCBOOT             (SAM_SDMMC1_VBASE + SAMA5_SDMMC_MMCBOOT_OFFSET)
#define SAMA5_SDMMC1_VENDOR2             (SAM_SDMMC1_VBASE + SAMA5_SDMMC_VENDOR2_OFFSET)
#define SAMA5_SDMMC1_TC                  (SAM_SDMMC1_VBASE + SAMA5_SDMMC_TC_OFFSET)

/* Register Bit Definitions *************************************************/

/* DMA System Address Register */

#define SDMMC_DSADDR_SHIFT               (0)           /* Bits 2-31: DMA System Address */
#define SDMMC_DSADDR_MASK                (0xfffffffc)  /* Bits 0-1: 32 bit aligned, low bits Reserved */

/* Block Attributes Register */

#define SDMMC_BSR_BLKSIZE_SHIFT         (0)            /* Bits 0-9: Transfer Block Size */
                                                       /* Bits 10-11: Reserved */
#define SDMMC_BSR_BOUNDARY_SHIFT        (12)           /* Bits 12-14: Boundary */
                                                       /* Bit 15: Reserved */

/* Block Count Register (16-bit block count) */

/* Command Argument Register (32-bit cmd/arg data) */

/* Transfer Type Register */

#define SDMMC_XFERTYP_DMAEN              (1 << 0)      /* Bit 0:  DMA Enable */
#define SDMMC_XFERTYP_BCEN               (1 << 1)      /* Bit 1:  Block Count Enable */
#define SDMMC_XFERTYP_AC12EN             (1 << 2)      /* Bit 2:  Auto CMD12 Enable */
                                                       /* Bit 3: Reserved */

#define SDMMC_XFERTYP_DTDSEL             (1 << 4)      /* Bit 4:  Data Transfer Direction Select */
#define SDMMC_XFERTYP_MSBSEL             (1 << 5)      /* Bit 5:  Multi/Single Block Select */
                                                       /* Bits 6-15: Reserved */

#define SDMMC_XFERTYP_RSPTYP_SHIFT       (16)          /* Bits 16-17: Response Type Select */
#define SDMMC_XFERTYP_RSPTYP_MASK        (3 << SDMMC_XFERTYP_RSPTYP_SHIFT)
#  define SDMMC_XFERTYP_RSPTYP_NONE      (0 << SDMMC_XFERTYP_RSPTYP_SHIFT) /* No response */
#  define SDMMC_XFERTYP_RSPTYP_LEN136    (1 << SDMMC_XFERTYP_RSPTYP_SHIFT) /* Response length 136 */
#  define SDMMC_XFERTYP_RSPTYP_LEN48     (2 << SDMMC_XFERTYP_RSPTYP_SHIFT) /* Response length 48 */
#  define SDMMC_XFERTYP_RSPTYP_LEN48BSY  (3 << SDMMC_XFERTYP_RSPTYP_SHIFT) /* Response length 48, check busy */
                                                       /* Bit 18: Reserved */
#define SDMMC_XFERTYP_CCCEN              (1 << 19)     /* Bit 19: Command CRC Check Enable */
#define SDMMC_XFERTYP_CICEN              (1 << 20)     /* Bit 20: Command Index Check Enable */
#define SDMMC_XFERTYP_DPSEL              (1 << 21)     /* Bit 21: Data Present Select */
#define SDMMC_XFERTYP_CMDTYP_SHIFT       (22)          /* Bits 22-23: Command Type */
#define SDMMC_XFERTYP_CMDTYP_MASK        (3 << SDMMC_XFERTYP_CMDTYP_SHIFT)
#  define SDMMC_XFERTYP_CMDTYP_NORMAL    (0 << SDMMC_XFERTYP_CMDTYP_SHIFT) /* Normal other commands */
#  define SDMMC_XFERTYP_CMDTYP_SUSPEND   (1 << SDMMC_XFERTYP_CMDTYP_SHIFT) /* Suspend CMD52 for writing bus suspend in CCCR */
#  define SDMMC_XFERTYP_CMDTYP_RESUME    (2 << SDMMC_XFERTYP_CMDTYP_SHIFT) /* Resume CMD52 for writing function select in CCCR */
#  define SDMMC_XFERTYP_CMDTYP_ABORT     (3 << SDMMC_XFERTYP_CMDTYP_SHIFT) /* Abort CMD12, CMD52 for writing I/O abort in CCCR */
#define SDMMC_XFERTYP_CMDINX_SHIFT       (24)          /* Bits 24-29: Command Index */
#define SDMMC_XFERTYP_CMDINX_MASK        (0x3f << SDMMC_XFERTYP_CMDINX_SHIFT)
                                                       /* Bits 30-31: Reserved */

/* Command Response 0-3 (32-bit response data) */

/* Buffer Data Port Register (32-bit data content) */

/* Present State Register */

#define SDMMC_PRSSTAT_CIHB               (1 << 0)     /* Bit 0:  Command Inhibit (CMD) */
#define SDMMC_PRSSTAT_CDIHB              (1 << 1)     /* Bit 1:  Command Inhibit (DAT) */
#define SDMMC_PRSSTAT_DLA                (1 << 2)     /* Bit 2:  Data Line Active */
#define SDMMC_PRSSTAT_SDSTB              (1 << 3)     /* Bit 3:  Unused in SAMA5 */
#define SDMMC_PRSSTAT_IPGOFF             (1 << 4)     /* Bit 4:  Unused */
#define SDMMC_PRSSTAT_HCKOFF             (1 << 5)     /* Bit 5:  Unused */
#define SDMMC_PRSSTAT_PEROFF             (1 << 6)     /* Bit 6:  Unused */
#define SDMMC_PRSSTAT_SDOFF              (1 << 7)     /* Bit 7:  Unused */
#define SDMMC_PRSSTAT_WTA                (1 << 8)     /* Bit 8:  Write Transfer Active */
#define SDMMC_PRSSTAT_RTA                (1 << 9)     /* Bit 9:  Read Transfer Active */
#define SDMMC_PRSSTAT_BWEN               (1 << 10)    /* Bit 10: Buffer Write Enable */
#define SDMMC_PRSSTAT_BREN               (1 << 11)    /* Bit 11: Buffer Read Enable */
#define SDMMC_PRSSTAT_RTR                (1 << 12)    /* Bit 12: Retuning request */
                                                      /* Bits 13-14: Reserved */
#define SDMMC_PRSSTAT_TSCD               (1 << 15)    /* Bit 15: Tape Select Change Done */
#define SDMMC_PRSSTAT_CINS               (1 << 16)    /* Bit 16: Card Inserted */
                                                      /* Bit 17: Reserved */
#define SDMMC_PRSSTAT_CDPL               (1 << 18)    /* Bit 18: Card Detect Pin Level */
#define SDMMC_PRSSTAT_WPSPL              (1 << 19)    /* Bit 19: Write Protect Switch Pin Level */
#define SDMMC_PRSSTAT_CLSL               (1 << 24)    /* Bit 24: CMD Line Signal Level */
                                                      /* Bits 25-31: Reserved */
#define SDMMC_PRSSTAT_DLSL_SHIFT         (20)         /* Bits 20-23: DAT Line Signal Level */
#define SDMMC_PRSSTAT_DLSL_MASK          (0x0f << SDMMC_PRSSTAT_DLSL_SHIFT)
#  define SDMMC_PRSSTAT_DLSL_DAT0        (0x01 << SDMMC_PRSSTAT_DLSL_SHIFT)
#  define SDMMC_PRSSTAT_DLSL_DAT1        (0x02 << SDMMC_PRSSTAT_DLSL_SHIFT)
#  define SDMMC_PRSSTAT_DLSL_DAT2        (0x04 << SDMMC_PRSSTAT_DLSL_SHIFT)
#  define SDMMC_PRSSTAT_DLSL_DAT3        (0x08 << SDMMC_PRSSTAT_DLSL_SHIFT)

/* Protocol Control Register */

#define SDMMC_PROCTL_LCTL                (1 << 0)     /* Bit 0:  LED Control */
#define SDMMC_PROCTL_DTW_SHIFT           (1)          /* Bit 1: Data Transfer Width */
#define SDMMC_PROCTL_DTW_MASK            (1 << SDMMC_PROCTL_DTW_SHIFT)
#  define SDMMC_PROCTL_DTW_1BIT          (0 << SDMMC_PROCTL_DTW_SHIFT) /* 1-bit mode */
#  define SDMMC_PROCTL_DTW_4BIT          (1 << SDMMC_PROCTL_DTW_SHIFT) /* 4-bit mode */
#define SDMMC_PROCTL_HSEN                (1 << 2)     /* Bit 2: High Speed Enable */
#define SDMMC_PROCTL_DMAS_SHIFT          (3)          /* Bits 3-4: DMA Select */
#define SDMMC_PROCTL_DMAS_MASK           (3 << SDMMC_PROCTL_DMAS_SHIFT)
#  define SDMMC_PROCTL_DMAS_SDMA         (0 << SDMMC_PROCTL_DMAS_SHIFT) /* No DMA or simple DMA is selected */
#  define SDMMC_PROCTL_DMAS_RES1         (1 << SDMMC_PROCTL_DMAS_SHIFT) /* Reserved */
#  define SDMMC_PROCTL_DMAS_ADMA         (2 << SDMMC_PROCTL_DMAS_SHIFT) /* ADMA is selected */
#  define SDMMC_PROCTL_DMAS_RES2         (1 << SDMMC_PROCTL_DMAS_SHIFT) /* Reserved */
#define SDMMC_PROCTL_EXTDW_SHIFT         (5)          /* Bits 3-4: DMA Select */
#define SDMMC_PROCTL_EXTDW_MASK          (1 << SDMMC_PROCTL_EXTDW_SHIFT)
                                                      /* Bits 6-7: Reserved */
#define SDMMC_PROCTL_SDBPWR_SHIFT         (9)         /* Bits 3-4: SD Bus Power */
#define SDMMC_PROCTL_SDBPWR_MASK          (1 << SDMMC_PROCTL_SDBPWR_SHIFT)
                                                      /* Bits 10-15: Reserved */
#define SDMMC_PROCTL_SABGREQ             (1 << 16)    /* Bit 16: Stop At Block Gap Request */
#define SDMMC_PROCTL_CREQ                (1 << 17)    /* Bit 17: Continue Request */
#define SDMMC_PROCTL_RWCTL               (1 << 18)    /* Bit 18: Read Wait Control */
#define SDMMC_PROCTL_IABG                (1 << 19)    /* Bit 19: Interrupt At Block Gap */
#define SDMMC_PROCTL_RDDONENO8CLK        (1 << 20)    /* Bit 20: Read done to 8 clock */
#define SDMMC_PROCTL_RESV2023            (4 << 21)    /* Bits 21-23: Reserved, write as 0x100 */
#define SDMMC_PROCTL_WECINT              (1 << 24)    /* Bit 24: Wakeup Event Enable On Card Interrupt */
#define SDMMC_PROCTL_WECINS              (1 << 25)    /* Bit 25: Wakeup Event Enable On SD Card Insertion */
#define SDMMC_PROCTL_WECRM               (1 << 26)    /* Bit 26: Wakeup Event Enable On SD Card Removal */
#define SDMMC_PROCTL_BURST_SHIFT         (27)         /* Bits 27-29: Burst Length */
#define SDMMC_PROCTL_BURST_MASK          (7 << SDMMC_PROCTL_BUSRT_SHIFT)
#  define SDMMC_PROCTL_BURST_INCR        (1 << SDMMC_PROCTL_BURST_SHIFT) /* Burst for Incr */
#  define SDMMC_PROCTL_BURST_4816        (2 << SDMMC_PROCTL_BURST_SHIFT) /* Burst for 4/8/16 */
#  define SDMMC_PROCTL_BURST_4W8W16W     (4 << SDMMC_PROCTL_BURST_SHIFT) /* Burst for 4w/8w/16w */
#define SDMMC_PROTCTL_NEBLKRD            (1 << 30)    /* Bit 30: Non-exect block read */
                                                      /* Bit 31: Reserved */

/* Power Control Register */

#define  SDMMC_POWER_ON                 (0x01)
#define  SDMMC_POWER_180                (0x0a)
#define  SDMMC_POWER_300                (0x0c)
#define  SDMMC_POWER_330                (0x0e)

/* System Control Register */

#define SDMMC_SYSCTL_INTCLKEN            (1 << 0)     /* Bit 0: Internal Clock Enable */
#define SDMMC_SYSCTL_INTCLKS             (1 << 1)     /* Bit 1: Internal Clock Stable */
#define SDMMC_SYSCTL_SDCLKEN             (1 << 2)     /* Bit 2: SD Clock Enable */
#define SDMMC_SYSCTL_CLKGSEL             (1 << 5)     /* Bit 5: Clock Generator Select */
#define SDMMC_SYSCTL_USDCLKFSEL          (3 << 6)     /* Bit 0: Upper bits of SDCLK Frequency Select */
#define SDMMC_SYSCTL_CLKFSEL             (0xf0)       /* Bit 0: SDCLK Frequency Select */
#define SDMMC_CLOCK_MUL_MASK	         (0x00ff0000)
#define SDMMC_CLOCK_MUL_SHIFT	         (16)

#define SDMMC_SYSCTL_DVS_SHIFT           (4)          /* Bits 4-7: Divisor */
#define SDMMC_SYSCTL_DVS_MASK            (0x0f << SDMMC_SYSCTL_DVS_SHIFT)
#  define SDMMC_SYSCTL_DVS_DIV(n)        (((n) - 1) << SDMMC_SYSCTL_DVS_SHIFT) /* Divide by n, n=1..16 */
#define SDMMC_SYSCTL_SDCLKFS_SHIFT       (8)          /* Bits 8-15: SDCLK Frequency Select */
#define SDMMC_SYSCTL_SDCLKFS_MASK        (0xff << SDMMC_SYSCTL_SDCLKFS_SHIFT)
#  define SDMMC_SYSCTL_SDCLKFS_BYPASS    (0x00 << SDMMC_SYSCTL_SDCLKFS_SHIFT) /* Bypass the prescaler */
#  define SDMMC_SYSCTL_SDCLKFS_DIV2      (0x01 << SDMMC_SYSCTL_SDCLKFS_SHIFT) /* Base clock / 2 */
#  define SDMMC_SYSCTL_SDCLKFS_DIV4      (0x02 << SDMMC_SYSCTL_SDCLKFS_SHIFT) /* Base clock / 4 */
#  define SDMMC_SYSCTL_SDCLKFS_DIV8      (0x04 << SDMMC_SYSCTL_SDCLKFS_SHIFT) /* Base clock / 8 */
#  define SDMMC_SYSCTL_SDCLKFS_DIV16     (0x08 << SDMMC_SYSCTL_SDCLKFS_SHIFT) /* Base clock / 16 */
#  define SDMMC_SYSCTL_SDCLKFS_DIV32     (0x10 << SDMMC_SYSCTL_SDCLKFS_SHIFT) /* Base clock / 32 */
#  define SDMMC_SYSCTL_SDCLKFS_DIV64     (0x20 << SDMMC_SYSCTL_SDCLKFS_SHIFT) /* Base clock / 64 */
#  define SDMMC_SYSCTL_SDCLKFS_DIV128    (0x40 << SDMMC_SYSCTL_SDCLKFS_SHIFT) /* Base clock / 128 */
#  define SDMMC_SYSCTL_SDCLKFS_DIV256    (0x80 << SDMMC_SYSCTL_SDCLKFS_SHIFT) /* Base clock / 256 */
#define SDMMC_SYSCTL_DTOCV_SHIFT         (16)         /* Bits 16-19: Data Timeout Counter Value */
#define SDMMC_SYSCTL_DTOCV_MASK          (0x0f << SDMMC_SYSCTL_DTOCV_SHIFT)
#  define SDMMC_SYSCTL_DTOCV_MUL(n)      (((n) - 213) << SDMMC_SYSCTL_DTOCV_SHIFT) /* SDCLK x n, n=213..227 */
                                                      /* Bits 20-22: Reserved */
#define SDMMC_SYSCTL_IPPRSTN             (1 << 23)    /* Bit 23: Card /reset (default 1) */
#define SDMMC_SYSCTL_RSTA                (1 << 24)    /* Bit 24: Software Reset For ALL */
#define SDMMC_SYSCTL_RSTC                (1 << 25)    /* Bit 25: Software Reset For CMD Line */
#define SDMMC_SYSCTL_RSTD                (1 << 26)    /* Bit 26: Software Reset For DAT Line */
#define SDMMC_SYSCTL_INITA               (1 << 27)    /* Bit 27: Initialization Active (SAMA5: unused) */
#define SDMMC_SYSCTL_RSTT                (1 << 28)    /* Bit 28: Reset tuning (SAMA5: unused) */
                                                      /* Bits 29-31: Reserved */
#define SDMMC_DIVIDER_SHIFT               (8)
#define SDMMC_DIV_MASK                    (0xff)
#define SDMMC_DIVIDER_HI_SHIFT            (6)
#define SDMMC_DIV_MASK_LEN                (8)
#define SDMMC_DIV_HI_MASK                 (0x300)
#define SDMMC_CLOCK_BASE_MASK             (0x00003f00)
#define SDMMC_CLOCK_V3_BASE_MASK          (0x0000ff00)
#define SDMMC_CLOCK_BASE_SHIFT            (8)

/* Timeout Control Register */

#define SDMMC_TCR_MASK                  (0x0f)        /* Bits 0-3 */

/* Interrupt Status Register, Interrupt Status Enable Register and Interrupt
 * Signal Enable Register Common interrupt bit definitions
 */

#define SDMMC_INT_CC                     (1 << 0)     /* Bit 0:  Command Complete */
#define SDMMC_INT_TC                     (1 << 1)     /* Bit 1:  Transfer Complete */
#define SDMMC_INT_BGE                    (1 << 2)     /* Bit 2:  Block Gap Event */
#define SDMMC_INT_DINT                   (1 << 3)     /* Bit 3:  DMA Interrupt - indicates SDMA Boundary Pause */
#define SDMMC_INT_BWR                    (1 << 4)     /* Bit 4:  Buffer Write Ready */
#define SDMMC_INT_BRR                    (1 << 5)     /* Bit 5:  Buffer Read Ready */
#define SDMMC_INT_CINS                   (1 << 6)     /* Bit 6:  Card Insertion */
#define SDMMC_INT_CRM                    (1 << 7)     /* Bit 7:  Card Removal */
#define SDMMC_INT_CINT                   (1 << 8)     /* Bit 8:  Card Interrupt */
                                                      /* Bits 9-11: Reserved */
#define SDMMC_INT_RTR                    (1 << 12)    /* Bit 12: Re-tuning event */
                                                      /* Bit 13: Reserved */
#define SDMMC_INT_TP                     (1 << 14)    /* Bit 14: Tuning pass */
                                                      /* Bit 15: Reserved */
#define SDMMC_INT_CTOE                   (1 << 16)    /* Bit 16: Command Timeout Error */
#define SDMMC_INT_CCE                    (1 << 17)    /* Bit 17: Command CRC Error */
#define SDMMC_INT_CEBE                   (1 << 18)    /* Bit 18: Command End Bit Error */
#define SDMMC_INT_CIE                    (1 << 19)    /* Bit 19: Command Index Error */
#define SDMMC_INT_DTOE                   (1 << 20)    /* Bit 20: Data Timeout Error */
#define SDMMC_INT_DCE                    (1 << 21)    /* Bit 21: Data CRC Error */
#define SDMMC_INT_DEBE                   (1 << 22)    /* Bit 22: Data End Bit Error */
#define SDMMC_INT_CURLIM                 (1 << 23)    /* Bit 23: Current Limit Error */
#define SDMMC_INT_AC12E                  (1 << 24)    /* Bit 24: Auto CMD12 Error */
#define SDMMC_INT_ADMAE                  (1 << 25)    /* Bit 25: ADMA error */
                                                      /* Bits 26-31: Reserved */
#define SDMMC_INT_ALL                    0x117f01ff

/* Auto CMD12 Error Status Register */

#define SDMMC_AC12ERR_NE                 (1 << 0)     /* Bit 0:  Auto CMD12 Not Executed */
#define SDMMC_AC12ERR_TOE                (1 << 1)     /* Bit 1:  Auto CMD12 Timeout Error */
#define SDMMC_AC12ERR_EBE                (1 << 2)     /* Bit 2:  Auto CMD12 End Bit Error */
#define SDMMC_AC12ERR_CE                 (1 << 3)     /* Bit 3:  Auto CMD12 CRC Error */
#define SDMMC_AC12ERR_IE                 (1 << 4)     /* Bit 4:  Auto CMD12 Index Error */
                                                      /* Bits 5-6: Reserved */
#define SDMMC_AC12ERR_CNI                (1 << 7)     /* Bit 7: Command Not Issued By Auto CMD12 Error */
                                                      /* Bits 8-21: Reserved */
#define SDMMC_AC12ERR_EXECUTE_TUNING     (1 << 22)    /* Bit 22: Execute Tuning */
#define SDMMC_AC12ERR_SMP_CLK_SEL        (1 << 23)    /* Bit 23: Sample clock sel */
                                                      /* Bits 24-31: Reserved */

/* Host Controller Capabilities */

#define SDMMC_HTCAPBLT_SDR50             (1 << 0)     /* Bit 0: SDR50 support indication */
#define SDMMC_HTCAPBLT_SDR104            (1 << 1)     /* Bit 1: SDR104 support indication */
#define SDMMC_HTCAPBLT_DDR50             (1 << 2)     /* Bit 2: DDR50 support indication */
                                                      /* Bits 3-7: Reserved */
#define SDMMC_HTCAPBLT_TCR_SHIFT         (8)          /* Bits 8-11: Time count retuning */
#define SDMMC_HTCAPBLT_TCR_MASK          (0xF << SDMMC_HTCAPBLT_TCR_SHIFT)
#  define SDMMC_HTCAPBLT_TCR(n)          ((n) << SDMMC_HTCAPBLT_TCR_SHIFT)
#define SDMMC_HTCAPBLT_USE_TUNING_SDR50  (1 << 13)    /* Bit 13: Use tuning for SDR50 */
#define SDMMC_HTCAPBLT_RET_MODE_SHIFT    (14)         /* bit 14-15: Retuning mode */
#define SDMMC_HTCAPBLT_RET_MODE_MASK     (3<<SDHC_HTCAPBLT_RET_MODE_SHIFT)
#define SDMMC_HTCAPBLT_RET_MODE_1        (0<<SDHC_HTCAPBLT_RET_MODE_SHIFT)
#define SDMMC_HTCAPBLT_RET_MODE_2        (1<<SDHC_HTCAPBLT_RET_MODE_SHIFT)
#define SDMMC_HTCAPBLT_RET_MODE_3        (2<<SDHC_HTCAPBLT_RET_MODE_SHIFT)
#define SDMMC_HTCAPBLT_MBL_SHIFT         (16)         /* Bits 16-18: Max Block Length */
#define SDMMC_HTCAPBLT_MBL_MASK          (7 << SDMMC_HTCAPBLT_MBL_SHIFT)
#  define SDMMC_HTCAPBLT_MBL_512BYTES    (0 << SDMMC_HTCAPBLT_MBL_SHIFT)
#  define SDMMC_HTCAPBLT_MBL_1KB         (1 << SDMMC_HTCAPBLT_MBL_SHIFT)
#  define SDMMC_HTCAPBLT_MBL_2KB         (2 << SDMMC_HTCAPBLT_MBL_SHIFT)
#  define SDMMC_HTCAPBLT_MBL_4KB         (3 << SDMMC_HTCAPBLT_MBL_SHIFT)
                                                      /* Bit 19: Reserved */
#define SDMMC_HTCAPBLT_ADMAS             (1 << 20)    /* Bit 20: ADMA Support */
#define SDMMC_HTCAPBLT_HSS               (1 << 21)    /* Bit 21: High Speed Support */
#define SDMMC_HTCAPBLT_DMAS              (1 << 22)    /* Bit 22: DMA Support */
#define SDMMC_HTCAPBLT_SRS               (1 << 23)    /* Bit 23: Suspend/Resume Support */
#define SDMMC_HTCAPBLT_VS33              (1 << 24)    /* Bit 24: Voltage Support 3.3 V */
#define SDMMC_HTCAPBLT_VS30              (1 << 25)    /* Bit 25: Voltage Support 3.0 V */
#define SDMMC_HTCAPBLT_VS18              (1 << 26)    /* Bit 26: Voltage Support 1.8 */
                                                      /* Bits 27-31: Reserved */

/* Capabilities 0 Register */

/* Capabilities 1 Register */

/* Force Event Register */

#define SDMMC_FEVT_AC12NE                (1 << 0)     /* Bit 0:  Force Event Auto Command 12 Not Executed */
#define SDMMC_FEVT_AC12TOE               (1 << 1)     /* Bit 1:  Force Event Auto Command 12 Time Out Error */
#define SDMMC_FEVT_AC12CE                (1 << 2)     /* Bit 2:  Force Event Auto Command 12 CRC Error */
#define SDMMC_FEVT_AC12EBE               (1 << 3)     /* Bit 3:  Force Event Auto Command 12 End Bit Error */
#define SDMMC_FEVT_AC12IE                (1 << 4)     /* Bit 4:  Force Event Auto Command 12 Index Error */
                                                      /* Bits 5-6: Reserved */
#define SDMMC_FEVT_CNIBAC12E             (1 << 7)     /* Bit 7:  Force Event Command Not Executed By Auto Command 12 Error */
                                                      /* Bits 8-15: Reserved */
#define SDMMC_FEVT_CTOE                  (1 << 16)    /* Bit 16: Force Event Command Time Out Error */
#define SDMMC_FEVT_CCE                   (1 << 17)    /* Bit 17: Force Event Command CRC Error */
#define SDMMC_FEVT_CEBE                  (1 << 18)    /* Bit 18: Force Event Command End Bit Error */
#define SDMMC_FEVT_CIE                   (1 << 19)    /* Bit 19: Force Event Command Index Error */
#define SDMMC_FEVT_DTOE                  (1 << 20)    /* Bit 20: Force Event Data Time Out Error */
#define SDMMC_FEVT_DCE                   (1 << 21)    /* Bit 21: Force Event Data CRC Error */
#define SDMMC_FEVT_DEBE                  (1 << 22)    /* Bit 22: Force Event Data End Bit Error */
                                                      /* Bit 23: Reserved */
#define SDMMC_FEVT_AC12E                 (1 << 24)    /* Bit 24: Force Event Auto Command 12 Error */
                                                      /* Bit 25: Reserved */
#define SDMMC_FEVT_TTNE                  (1 << 26)    /* Bit 26: Force tuning error */
                                                      /* Bit 27: reserved */
#define SDMMC_FEVT_DMAE                  (1 << 28)    /* Bit 28: Force Event DMA Error */
                                                      /* Bits 29-30: Reserved */
#define SDMMC_FEVT_CINT                  (1 << 31)    /* Bit 31: Force Event Card Interrupt */

/* ADMA Error Status Register */

#define SDMMC_ADMAES_SHIFT               (0)          /* Bits 0-1: ADMA Error State (when ADMA Error is occurred) */
#define SDMMC_ADMAES_MASK                (3 << SDMMC_ADMAES_ADMAES_SHIFT)
#  define SDMMC_ADMAES_STOP              (0 << SDMMC_ADMAES_ADMAES_SHIFT) /* Stop DMA */
#  define SDMMC_ADMAES_FDS               (1 << SDMMC_ADMAES_ADMAES_SHIFT) /* Fetch descriptor */
#  define SDMMC_ADMAES_CADR              (2 << SDMMC_ADMAES_ADMAES_SHIFT) /* Change address */
#  define SDMMC_ADMAES_TFR               (3 << SDMMC_ADMAES_ADMAES_SHIFT) /* Transfer data */
#define SDMMC_ADMAES_LME                 (1 << 2)     /* Bit 2:  ADMA Length Mismatch Error */
#define SDMMC_ADMAES_DCE                 (1 << 3)     /* Bit 3:  ADMA Descriptor Error */
                                                      /* Bits 4-31: Reserved */
/* ADMA System Address Register */

#define SDMMC_ADSADDR_SHIFT              (0)          /* Bits 1-31: ADMA System Address */
#define SDMMC_ADSADDR_MASK               (0xfffffffc) /* Bits 0-1: Reserved */

/* Delay Line Control */

#define SDMMC_DL_CTRL_EN                 (1 << 0)     /* Bit 0: Delay Line enable */
#define SDMMC_DL_CTRL_RST                (1 << 1)     /* Bit 1: Delay line reset */
#define SDMMC_DL_CTRL_SLV_FORCE_UP       (1 << 2)     /* Bit 2: SLV Force update */
#define SDMMC_DL_SLV_DLY_TGT0_SHIFT      (3)          /* Bits 3-6: Delay Target 0 */
#define SDMMC_DL_SLV_DLY_TGT0_MASK       (0xf << SDMMC_DL_SLV_DLY_TGT0_SHIFT)
#  define SDMMC_DL_SLV_DLY_TGT0(n)       ((n) << SDMMC_DL_SLV_DLY_TGT0_SHIFT)
#define SDMMC_DL_CTRL_SLV_UPD            (1 << 7)     /* Bit 7: Delay Control Gate update */
#define SDMMC_DL_CTRL_SLV_OVR            (1 << 8)     /* Bit 8: Delay Control Gate override */
#define SDMMC_DL_CTRL_OVR_VAL_SHIFT      (9)          /* Bits 9-15: Override Value */
#define SDMMC_DL_CTRL_OVR_VAL_MASK       (0x7f << SDMMC_DL_CTRL_OVR_VAL_SHIFT)
#  define SDMMC_DL_CTRL_OVR_VAL(n)       ((n) << SDMMC_DL_CTRL_OVR_VAL_SHIFT)
#define SDMMC_DL_SLV_DLY_TGT1_SHIFT      (16)         /* Bits 16-18: Delay Target 1 */
#define SDMMC_DL_SLV_DLY_TGT1_MASK       (0x7 << SDMMC_DL_SLV_DLY_TGT1_SHIFT)
#  define SDMMC_DL_SLV_DLY_TGT1(n)       ((n) << SDMMC_DL_SLV_DLY_TGT1_SHIFT)
                                                      /* Bit 19: Reserved */
#define SDMMC_DL_CTRL_SLV_UPDINT_SHIFT   (20)         /* Bits 20-27: DLL Control SLV Update Interval */
#define SDMMC_DL_CTRL_SLV_UPDINT_MASK    (0xff << SDMMC_DL_CTRL_SLV_UPDINT_SHIFT)
#  define SDMMC_DL_CTRL_SLV_UPDINT(n)    ((n) << SDMMC_DL_CTRL_SLV_UPDINT_SHIFT)
#define SDMMC_DL_CTRL_REF_UPDINT_SHIFT   (28)         /* Bits 28-31: DLL Control Reference Update Interval */
#define SDMMC_DL_CTRL_REF_UPDINT_MASK    (0xf << SDMMC_DL_CTRL_REF_UPDINT_SHIFT)
#  define SDMMC_DL_CTRL_REF_UPDINT(n)    ((n)<< SDMMC_DL_CTRL_REF_UPDINT_SHIFT)

/* Delay Line Status */

#define SDMMC_DL_STAT_SLV_LOCK           (1 << 0)     /* Bit 0: Slave delay-line lock status */
#define SDMMC_DL_STAT_REF_LOCK           (1 << 1)     /* Bit 1: Reference delay-line lock status */
#define SDMMC_DL_STAT_SLV_SEL_SHIFT      (2)          /* Bits 2-8: Slave delay line select status */
#define SDMMC_DL_STAT_SLV_SEL_MASK       (0x7f << SDMMC_DL_STAT_SLV_SEL_SHIFT)
#  define SDMMC_DL_STAT_SLV_SEL(n)       ((n) << SDMMC_DL_STAT_SLV_SEL_SHIFT)
#define SDMMC_DL_STAT_REF_SEL_SHIFT      (9)           /* Bits 9-15: Reference delay line select taps */
#define SDMMC_DL_STAT_REF_SEL_MASK       (0x7f << SDMMC_DL_STAT_REF_SEL_SHIFT)
#  define SDMMC_DL_STAT_REF_SEL(n)       ((n) << SDMMC_DL_STAT_REF_SEL_SHIFT)
                                                      /* Bits 16-31: Reserved */

/* Tuning Control Register */

#define SDMMC_TC_SMPLPT_MASK             (1 << 0)    /* Bit 0: Sampling Point */
#define SDMMC_TC_SMPLPT_50_PCT           (0 << 0)    /* Sampling Point at 50% of window */
#define SDMMC_TC_SMPLPT_75_PCT           (1 << 0)    /* Sampling Point at 75% of window */

/* Host Version Register */

#define SDMMC_SPEC_1    0
#define SDMMC_SPEC_2    1
#define SDMMC_SPEC_3    2
#define SDMMC_MAX_DIV_SPEC_2    256
#define SDMMC_MAX_DIV_SPEC_3    2046

/* Software Reset Register */
#define  SDMMC_RESET_ALL	0x01
#define  SDMMC_RESET_CMD	0x02
#define  SDMMC_RESET_DATA	0x04

/* Host Control 2 Register */
#define SDMMC_UHSMS_MASK                    (7)         /* Bits 0-2: UHS Mode Select */
#define SDMMC_VS18EN                        (1 << 3)    /* Bit 3: 1.8V Signaling Enable */
#define SDMMC_DRVSEL_MASK                   (0x3 << 4)  /* Driver Strength Select */
#define SDMMC_EXTUN                         (1 << 6)    /* Bit 6: Execute Tuning */
#define SDMMC_SCLKSEL                       (1 << 7)    /* Bit 7: Sampling Clock Select */
#define SDMMC_ASINTEN                       (1 << 14)   /* Bit 14: Asynchronous Interrupt Enable*/
#define SDMMC_PVALEN                        (1 << 15)   /* Bit 15: Preset Value Enable */

/* Host Control 2 Register */
#define  SDMMC_H2CR_UHS_MASK                (0x0007)
#define  SDMMC_H2CR_UHS_SDR12               (0x0000)
#define  SDMMC_H2CR_UHS_SDR25               (0x0001)
#define  SDMMC_H2CR_UHS_SDR50               (0x0002)
#define  SDMMC_H2CR_UHS_SDR104              (0x0003)
#define  SDMMC_H2CR_UHS_DDR50               (0x0004)
#define  SDMMC_H2CR_HS400                   (0x0005) /* Non-standard */
#define  SDMMC_H2CR_VDD_180                 (0x0008)
#define  SDMMC_H2CR_DRV_TYPE_MASK           (0x0030)
#define  SDMMC_H2CR_DRV_TYPE_B              (0x0000)
#define  SDMMC_H2CR_DRV_TYPE_A              (0x0010)
#define  SDMMC_H2CR_DRV_TYPE_C              (0x0020)
#define  SDMMC_H2CR_DRV_TYPE_D              (0x0030)
#define  SDMMC_H2CR_EXEC_TUNING             (0x0040)
#define  SDMMC_H2CR_TUNED_CLK               (0x0080)
#define  SDMMC_H2CR_PRESET_VAL_ENABLE       (0x8000)

/* SDMMC bus modes */

enum bus_mode
  {
     MMC_LEGACY,
     SD_LEGACY,
     MMC_HS,
     SD_HS,
     MMC_HS_52,
     MMC_DDR_52,
     UHS_SDR12,              /* Single data rate, 12Mhz - default */
     UHS_SDR25,              /* Single data rate, 25Mhz */
     UHS_SDR50,              /* Single data rate, 50Mhz */
     UHS_DDR50,              /* Double data rate, 50Mhz */
     UHS_SDR104,             /* Single data rate, 100Mhz */
     MMC_HS_200,
     MMC_HS_400,
     MMC_HS_400_ES,
     MMC_MODES_END
  };

/* MMC Power */

#define MMCSD_VDD_19_20             ((uint32_t)1 << 7)     /* VDD voltage 1.9 - 2.0 */
#define MMCSD_VDD_29_30             ((uint32_t)1 << 17)    /* VDD voltage 2.9-3.0 */
#define MMCSD_VDD_30_31             ((uint32_t)1 << 18)    /* VDD voltage 3.0-3.1 */
#define MMCSD_VDD_32_33             ((uint32_t)1 << 20)    /* VDD voltage 3.2-3.3 */
#define MMCSD_VDD_33_34             ((uint32_t)1 << 21)    /* VDD voltage 3.3-3.4 */

/* SDMMC bus speed for IDMODE transfers */

#define SAMA5_SDMMC_BUS_SPEED_IDMODE 400000

/* SDMMC bus speed threshold for high-speed mode transfers-
 * above this we need to set SDMMC_PROCTL_HSEN bit
 */

#define SAMA5_SDMMC_BUS_HIGH_SPEED_THRESHOLD 26000000

/* SDMMC maximum bus speed - only used if SD Card supports this speed;
 * DDR (double data rate) signaling on rising and falling edges of the clock
 * will also be used if card supports it.
 *
 * DDR50 in widebus mode = 400 megabits/s
 */

#ifdef CONFIG_SAMA5_SDMMC_50MHZ
# define SAMA5_SDMMC_BUS_SPEED 50000000
#else
# define SAMA5_SDMMC_BUS_SPEED 25000000
#endif

/* SDMA default buffer size - needed to reset the start address after an SDMA
 * Buffer Boundary pause event.
 */

#define SDMMC_DEFAULT_BOUNDARY_SIZE	(512 * 1024)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAMA5_SDMMC_H */
