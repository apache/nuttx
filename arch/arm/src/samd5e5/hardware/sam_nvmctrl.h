/****************************************************************************
 * arch/arm/src/samd5e5/hardware/sam_nvmctrl.h
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

#ifndef __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_NVMCTRL_H
#define __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_NVMCTRL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* NVMCTRL register offsets *************************************************/

#define SAM_NVMCTRL_CTRLA_OFFSET     0x0000  /* Control A register */
#define SAM_NVMCTRL_CTRLB_OFFSET     0x0004  /* Control B register */
#define SAM_NVMCTRL_PARAM_OFFSET     0x0008  /* NVM parameter register */
#define SAM_NVMCTRL_INTENCLR_OFFSET  0x000c  /* Interrupt clear register */
#define SAM_NVMCTRL_INTENSET_OFFSET  0x000e  /* Interrupt set register */
#define SAM_NVMCTRL_INTFLAG_OFFSET   0x0010  /* Interface flags status and clear register */
#define SAM_NVMCTRL_STATUS_OFFSET    0x0012  /* Status register */
#define SAM_NVMCTRL_ADDR_OFFSET      0x0014  /* Address register */
#define SAM_NVMCTRL_RUNLOCK_OFFSET   0x0018  /* Lock section register */
#define SAM_NVMCTRL_PBLDATAn0_OFFSET 0x001c  /* Page buffer load data n 0 */
#define SAM_NVMCTRL_PBLDATAn1_OFFSET 0x0020  /* Page buffer load data n 1 */
#define SAM_NVMCTRL_ECCERR_OFFSET    0x0024  /* ECC error status register */
#define SAM_NVMCTRL_DBGCTRL_OFFSET   0x0028  /* Debug control register */
#define SAM_NVMCTRL_SEECFG_OFFSET    0x002a  /* SmartEEPROM configuration register */
#define SAM_NVMCTRL_SEESTAT_OFFSET   0x002c  /* SmartEEPROM status register */

/* NVMCTRL register addresses ***********************************************/

#define SAM_NVMCTRL_CTRLA            (SAM_NVMCTRL_BASE + SAM_NVMCTRL_CTRLA_OFFSET)
#define SAM_NVMCTRL_CTRLB            (SAM_NVMCTRL_BASE + SAM_NVMCTRL_CTRLB_OFFSET)
#define SAM_NVMCTRL_PARAM            (SAM_NVMCTRL_BASE + SAM_NVMCTRL_PARAM_OFFSET)
#define SAM_NVMCTRL_INTENCLR         (SAM_NVMCTRL_BASE + SAM_NVMCTRL_INTENCLR_OFFSET)
#define SAM_NVMCTRL_INTENSET         (SAM_NVMCTRL_BASE + SAM_NVMCTRL_INTENSET_OFFSET)
#define SAM_NVMCTRL_INTFLAG          (SAM_NVMCTRL_BASE + SAM_NVMCTRL_INTFLAG_OFFSET)
#define SAM_NVMCTRL_STATUS           (SAM_NVMCTRL_BASE + SAM_NVMCTRL_STATUS_OFFSET)
#define SAM_NVMCTRL_ADDR             (SAM_NVMCTRL_BASE + SAM_NVMCTRL_ADDR_OFFSET)
#define SAM_NVMCTRL_RUNLOCK          (SAM_NVMCTRL_BASE + SAM_NVMCTRL_RUNLOCK_OFFSET)
#define SAM_NVMCTRL_PBLDATAn0        (SAM_NVMCTRL_BASE + SAM_NVMCTRL_PBLDATAn0_OFFSET)
#define SAM_NVMCTRL_PBLDATAn1        (SAM_NVMCTRL_BASE + SAM_NVMCTRL_PBLDATAn1_OFFSET)
#define SAM_NVMCTRL_ECCERR           (SAM_NVMCTRL_BASE + SAM_NVMCTRL_ECCERR_OFFSET)
#define SAM_NVMCTRL_DBGCTRL          (SAM_NVMCTRL_BASE + SAM_NVMCTRL_DBGCTRL_OFFSET)
#define SAM_NVMCTRL_SEECFG           (SAM_NVMCTRL_BASE + SAM_NVMCTRL_SEECFG_OFFSET)
#define SAM_NVMCTRL_SEESTAT          (SAM_NVMCTRL_BASE + SAM_NVMCTRL_SEESTAT_OFFSET)

/* NVMCTRL register bit definitions *****************************************/

/* Control A register */

#define NVMCTRL_CTRLA_AUTOWS         (1 << 2)  /* Bit 2: Auto Wait State Enable */
#define NVMCTRL_CTRLA_SUSPEN         (1 << 3)  /* Bit 3: Suspend Enable */
#define NVMCTRL_CTRLA_WMODE_SHIFT    (4)       /* Bits 4-5: NVMCTRL Write Mode */
#define NVMCTRL_CTRLA_WMODE_MASK     (3 << NVMCTRL_CTRLA_WMODE_SHIFT)
#  define NVMCTRL_CTRLA_WMODE_MAN    (0 << NVMCTRL_CTRLA_WMODE_SHIFT) /* Manual Write */
#  define NVMCTRL_CTRLA_WMODE_ADW    (1 << NVMCTRL_CTRLA_WMODE_SHIFT) /* Automatic Double Word Write */
#  define NVMCTRL_CTRLA_WMODE_AQW    (2 << NVMCTRL_CTRLA_WMODE_SHIFT) /* Automatic Quad Word */
#  define NVMCTRL_CTRLA_WMODE_AP     (3 << NVMCTRL_CTRLA_WMODE_SHIFT) /* Automatic Page Write */
#define NVMCTRL_CTRLA_PRM_SHIFT      (6)                              /* Bits 8-9: Power Reduction Mode during Sleep */
#define NVMCTRL_CTRLA_PRM_MASK       (3 << NVMCTRL_CTRLA_PRM_SHIFT)
#  define NVMCTRL_CTRLA_PRM_SEMIAUTO (0 << NVMCTRL_CTRLA_PRM_SHIFT) /* Enter low power on STANDBY/SPRM cmd;
                                                                     * Exit on first access */
#  define NVMCTRL_CTRLA_PRM_FULLAUTO (1 << NVMCTRL_CTRLA_PRM_SHIFT) /* Enter/Exit low power on STANDBY */
#  define NVMCTRL_CTRLA_PRM_MANUAL   (3 << NVMCTRL_CTRLA_PRM_SHIFT) /* Enter low power only on SPRM cmd;
                                                                     * Exit on first access */
#define NVMCTRL_CTRLA_RWS_SHIFT      (8)                            /* Bits 8-11: NVM Read Wait States */
#define NVMCTRL_CTRLA_RWS_MASK       (15 << NVMCTRL_CTRLA_RWS_SHIFT)
#  define NVMCTRL_CTRLA_RWS(n)       ((uint32_t)(n) << NVMCTRL_CTRLA_RWS_SHIFT)
#define NVMCTRL_CTRLA_AHBNS0         (1 << 12) /* Bit 12: Force AHB0 access to Non-Sequential */
#define NVMCTRL_CTRLA_AHBNS1         (1 << 13) /* Bit 13: Force AHB1 access to Non-Sequential */
#define NVMCTRL_CTRLA_CACHEDIS0      (1 << 14) /* Bit 14: AHB0 Cache Disable */
#define NVMCTRL_CTRLA_CACHEDIS1      (1 << 15) /* Bit 15: AHB1 Cache Disable */

/* Control B register */

#define NVMCTRL_CTRLB_CMD_SHIFT      (0)       /* Bits 0-6: Command */
#define NVMCTRL_CTRLB_CMD_MASK       (0x7f << NVMCTRL_CTRLB_CMD_SHIFT)
#  define NVMCTRL_CTRLB_CMD_EP       (0x00 << NVMCTRL_CTRLB_CMD_SHIFT) /* Erase Page */
#  define NVMCTRL_CTRLB_CMD_EB       (0x01 << NVMCTRL_CTRLB_CMD_SHIFT) /* Erase Block */
#  define NVMCTRL_CTRLB_CMD_WP       (0x03 << NVMCTRL_CTRLB_CMD_SHIFT) /* Write Page */
#  define NVMCTRL_CTRLB_CMD_WQW      (0x04 << NVMCTRL_CTRLB_CMD_SHIFT) /* Write Quad Word */
#  define NVMCTRL_CTRLB_CMD_SRST     (0x10 << NVMCTRL_CTRLB_CMD_SHIFT) /* Software reset */
#  define NVMCTRL_CTRLB_CMD_LR       (0x11 << NVMCTRL_CTRLB_CMD_SHIFT) /* Lock Region */
#  define NVMCTRL_CTRLB_CMD_UR       (0x12 << NVMCTRL_CTRLB_CMD_SHIFT) /* Unlock Region */
#  define NVMCTRL_CTRLB_CMD_SPRM     (0x13 << NVMCTRL_CTRLB_CMD_SHIFT) /* Set power reduction mode */
#  define NVMCTRL_CTRLB_CMD_CPRM     (0x14 << NVMCTRL_CTRLB_CMD_SHIFT) /* Clear power reduction mode */
#  define NVMCTRL_CTRLB_CMD_PBC      (0x15 << NVMCTRL_CTRLB_CMD_SHIFT) /* Page Buffer Clear */
#  define NVMCTRL_CTRLB_CMD_SSB      (0x16 << NVMCTRL_CTRLB_CMD_SHIFT) /* Set Security Bit */
#  define NVMCTRL_CTRLB_CMD_BKSWRST  (0x17 << NVMCTRL_CTRLB_CMD_SHIFT) /* Bank swap and system reset */
#  define NVMCTRL_CTRLB_CMD_CELCK    (0x18 << NVMCTRL_CTRLB_CMD_SHIFT) /* Chip Erase Lock */
#  define NVMCTRL_CTRLB_CMD_CEULCK   (0x19 << NVMCTRL_CTRLB_CMD_SHIFT) /* Chip Erase Lock */
#  define NVMCTRL_CTRLB_CMD_SBPDIS   (0x1a << NVMCTRL_CTRLB_CMD_SHIFT) /* Sets STATUS */
#  define NVMCTRL_CTRLB_CMD_CBPDIS   (0x1b << NVMCTRL_CTRLB_CMD_SHIFT) /* Clears STATUS */
#  define NVMCTRL_CTRLB_CMD_ASEES0   (0x30 << NVMCTRL_CTRLB_CMD_SHIFT) /* Configure SmartEEPROM Sector 0 */
#  define NVMCTRL_CTRLB_CMD_ASEES1   (0x31 << NVMCTRL_CTRLB_CMD_SHIFT) /* Configure SmartEEPROM Sector 1 */
#  define NVMCTRL_CTRLB_CMD_SEERALOC (0x32 << NVMCTRL_CTRLB_CMD_SHIFT) /* SmartEEPROM sector reallocation */
#  define NVMCTRL_CTRLB_CMD_SEEFLSUH (0x33 << NVMCTRL_CTRLB_CMD_SHIFT) /* Flush SmartEEPROM data1 */
#  define NVMCTRL_CTRLB_CMD_LSEE     (0x34 << NVMCTRL_CTRLB_CMD_SHIFT) /* Lock SmartEEPROM data */
#  define NVMCTRL_CTRLB_CMD_USEE     (0x35 << NVMCTRL_CTRLB_CMD_SHIFT) /* Unlock SmartEEPROM data*/
#  define NVMCTRL_CTRLB_CMD_LSEER    (0x36 << NVMCTRL_CTRLB_CMD_SHIFT) /* Lock SmartEEPROM registers */
#  define NVMCTRL_CTRLB_CMD_USEER    (0x37 << NVMCTRL_CTRLB_CMD_SHIFT) /* Unlock SmartEEPROM registers */
#define NVMCTRL_CTRLB_CMDEX_SHIFT    (8)                               /* Bits 8-15: Command Execution */
#define NVMCTRL_CTRLB_CMDEX_MASK     (0xff << NVMCTRL_CTRLB_CMDEX_SHIFT)
#  define NVMCTRL_CTRLB_CMDEX_KEY    (0xa5 << NVMCTRL_CTRLB_CMDEX_SHIFT)

/* NVM parameter register */

#define NVMCTRL_PARAM_NVMP_SHIFT     (0)      /* Bits 0-15: NVM Pages */
#define NVMCTRL_PARAM_NVMP_MASK      (0xffff << NVMCTRL_PARAM_NVMP_SHIFT)
#  define NVMCTRL_PARAM_NVMP(n)      ((uint32_t)(n) << NVMCTRL_PARAM_NVMP_SHIFT)
#define NVMCTRL_PARAM_PSZ_SHIFT      (16)      /* Bits 16-18: Page Size */
#define NVMCTRL_PARAM_PSZ_MASK       (7 << NVMCTRL_PARAM_PSZ_SHIFT)
#  define NVMCTRL_PARAM_PSZ_8B       (0 << NVMCTRL_PARAM_PSZ_SHIFT) /* 8 bytes */
#  define NVMCTRL_PARAM_PSZ_16B      (1 << NVMCTRL_PARAM_PSZ_SHIFT) /* 16 bytes */
#  define NVMCTRL_PARAM_PSZ_32B      (2 << NVMCTRL_PARAM_PSZ_SHIFT) /* 32 bytes */
#  define NVMCTRL_PARAM_PSZ_64B      (3 << NVMCTRL_PARAM_PSZ_SHIFT) /* 64 bytes */
#  define NVMCTRL_PARAM_PSZ_128B     (4 << NVMCTRL_PARAM_PSZ_SHIFT) /* 128 bytes */
#  define NVMCTRL_PARAM_PSZ_256B     (5 << NVMCTRL_PARAM_PSZ_SHIFT) /* 256 bytes */
#  define NVMCTRL_PARAM_PSZ_512B     (6 << NVMCTRL_PARAM_PSZ_SHIFT) /* 512 bytes */
#  define NVMCTRL_PARAM_PSZ_1KB      (7 << NVMCTRL_PARAM_PSZ_SHIFT) /* 1024 bytes */
#define NVMCTRL_PARAM_SEE_           (1 << 31)                      /* Bit 31: SmartEEPROM Supported */

/* Interrupt clear register
 * Interrupt set register
 * Interface flags status and clear register
 */

#define NVMCTRL_INT_DONE             (1 << 0)  /* Bit 0:  Command done interrupt */
#define NVMCTRL_INT_ADDRE            (1 << 1)  /* Bit 1:  Address error interrupt */
#define NVMCTRL_INT_PROGE            (1 << 2)  /* Bit 2:  Programming Error Status */
#define NVMCTRL_INT_LOCKE            (1 << 3)  /* Bit 3:  Lock Error Status */
#define NVMCTRL_INT_ECCSE            (1 << 4)  /* Bit 4:  ECC single error interrupt */
#define NVMCTRL_INT_ECCDE            (1 << 5)  /* Bit 5:  ECC dual error interrupt */
#define NVMCTRL_INT_NVME             (1 << 6)  /* Bit 6:  NVM error interrupt */
#define NVMCTRL_INT_SUSP             (1 << 7)  /* Bit 7:  Suspended write or erase interrupt */
#define NVMCTRL_INT_SEESFULL         (1 << 8)  /* Bit 8:  Active SEES full interrupt */
#define NVMCTRL_INT_SEESOVF          (1 << 9)  /* Bit 9:  Active SEES overflow interrupt */
#define NVMCTRL_INT_SEEWRC           (1 << 10) /* Bit 10: SEE Write completed interrupt */

/* Status register */

#define NVMCTRL_STATUS_READY         (1 << 0)  /* Bit 0: Ready to accept a command */
#define NVMCTRL_STATUS_PRM           (1 << 1)  /* Bit 1: Power reduction mode */
#define NVMCTRL_STATUS_LOAD          (1 << 2)  /* Bit 2: NVM Page Buffer active loading */
#define NVMCTRL_STATUS_SUSP          (1 << 3)  /* Bit 3: NVM Write or erase operation suspended */
#define NVMCTRL_STATUS_AFIRST        (1 << 4)  /* Bit 2: BANKA first */
#define NVMCTRL_STATUS_BPDIS         (1 << 5)  /* Bit 5: Boot loader protection disable */
#define NVMCTRL_STATUS_BOOTPROT_SHIFT  (8)     /* Bits 8-11: Boot loader protection size */
#define NVMCTRL_STATUS_BOOTPROT_MASK   (15 << NVMCTRL_STATUS_BOOTPROT_SHIFT)
#  define NVMCTRL_STATUS_BOOTPROT(n)   ((uint16_t)(n) << NVMCTRL_STATUS_BOOTPROT_SHIFT)

/* Address register */

#define NVMCTRL_ADDR_MASK            (0x00ffffff) /* Bits 0-23: NVM Address */

/* Lock section register */

#define NVMCTRL_RUNLOCK_REGION(n)     (1 << (n)) /* Region n is locked */

/* Page buffer load data n 0/1 (32-bit data) */

/* ECC error status register */

#define NVMCTRL_ECCERR_ADDR_SHIFT     (0)      /* Bits 0-23: Error address */
#define NVMCTRL_ECCERR_ADDR_MASK      (0xffffff << NVMCTRL_ECCERR_ADDR_SHIFT)
#  define NVMCTRL_ECCERR_ADDR(n)      ((uint32_t)(n) << NVMCTRL_ECCERR_ADDR_SHIFT)
#define NVMCTRL_ECCERR_TYPEL_SHIFT    (28)     /* Bits 28-29: Low double-word error type */
#define NVMCTRL_ECCERR_TYPEL_MASK     (3 << NVMCTRL_ECCERR_TYPEL_SHIFT)
#  define NVMCTRL_ECCERR_TYPEL_NONE   (0 << NVMCTRL_ECCERR_TYPEL_SHIFT) /* No error detected */
#  define NVMCTRL_ECCERR_TYPEL_SINGLE (1 << NVMCTRL_ECCERR_TYPEL_SHIFT) /* Single error(s) detected */
#  define NVMCTRL_ECCERR_TYPEL_DUAL   (2 << NVMCTRL_ECCERR_TYPEL_SHIFT) /* Dual error(s) detected */
#define NVMCTRL_ECCERR_TYPEH_SHIFT    (30)                              /* Bits 30-31: High double-word error type */
#define NVMCTRL_ECCERR_TYPEH_MASK     (3 << NVMCTRL_ECCERR_TYPEH_SHIFT)
#  define NVMCTRL_ECCERR_TYPEH_NONE   (0 << NVMCTRL_ECCERR_TYPEH_SHIFT) /* No error detected */
#  define NVMCTRL_ECCERR_TYPEH_SINGLE (1 << NVMCTRL_ECCERR_TYPEH_SHIFT) /* Single error(s) detected */
#  define NVMCTRL_ECCERR_TYPEH_DUAL   (2 << NVMCTRL_ECCERR_TYPEH_SHIFT) /* Dual error(s) detected */

/* Debug control register */

#define NVMCTRL_DBGCTRL_ECCDIS        (1 << 0)  /* Bit 0:  Debugger ECC read disable */
#define NVMCTRL_DBGCTRL_ECCELOG       (1 << 1)  /* Bit 1:  Debugger ECC error tracking mode */

/* SmartEEPROM configuration register */

#define NVMCTRL_SEECFG_WMODE          (1 << 0)  /* Bit 0:  Write mode */
#define NVMCTRL_SEECFG_APRDIS         (1 << 1)  /* Bit 1:  Automatic page reallocation disable */

/* SmartEEPROM status register */

#define NVMCTRL_SEESTAT_ASEES         (1 << 0)  /* Bit 0:  Active SmartEEPROM Sector */
#define NVMCTRL_SEESTAT_LOAD          (1 << 1)  /* Bit 1:  Page buffer loaded */
#define NVMCTRL_SEESTAT_BUSY          (1 << 2)  /* Bit 2:  Busy */
#define NVMCTRL_SEESTAT_LOCK          (1 << 3)  /* Bit 3:  SmartEEPROM section locked */
#define NVMCTRL_SEESTAT_RLOCK         (1 << 4)  /* Bit 4:  RLOCK */
#define NVMCTRL_SEESTAT_SBLK_SHIFT    (8)       /* Bits 8-11: Blocks number in a sector */
#define NVMCTRL_SEESTAT_SBLK_MASK     (15 << NVMCTRL_SEESTAT_SBLK_SHIFT)
#  define NVMCTRL_SEESTAT_SBLK(n)     ((uint32_t)(n) << NVMCTRL_SEESTAT_SBLK_SHIFT)
#define NVMCTRL_SEESTAT_PSZ_SHIFT     (16)      /* Bits 16-18: SmartEEPROM page size */
#define NVMCTRL_SEESTAT_PSZ_MASK      (7 << NVMCTRL_SEESTAT_PSZ_SHIFT)
#  define NVMCTRL_SEESTAT_PSZ(n)      ((uint32_t)(n) << NVMCTRL_SEESTAT_PSZ_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_NVMCTRL_H */
