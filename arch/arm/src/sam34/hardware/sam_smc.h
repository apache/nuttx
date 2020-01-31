/****************************************************************************************
 * arch/arm/src/sam34/hardware/sam_smc.h
 * Static Memory Controller (SMC) definitions for the SAM3U, SAM4E, and SAM4S
 *
 *   Copyright (C) 2009, 2013-2014 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_SMC_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_SMC_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* SMC register offsets *****************************************************************/

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SAM_SMC_CFG_OFFSET           0x0000 /* SMC NFC Configuration Register */
#  define SAM_SMC_CTRL_OFFSET          0x0004 /* SMC NFC Control Register */
#  define SAM_SMC_SR_OFFSET            0x0008 /* SMC NFC Status Register */
#  define SAM_SMC_IER_OFFSET           0x000c /* SMC NFC Interrupt Enable Register */
#  define SAM_SMC_IDR_OFFSET           0x0010 /* SMC NFC Interrupt Disable Register */
#  define SAM_SMC_IMR_OFFSET           0x0014 /* SMC NFC Interrupt Mask Register */
#  define SAM_SMC_ADDR_OFFSET          0x0018 /* SMC NFC Address Cycle Zero Register */
#  define SAM_SMC_BANK_OFFSET          0x001c /* SMC Bank Address Register */
#  define SAM_SMC_ECCCTRL_OFFSET       0x0020 /* SMC ECC Control Register */
#  define SAM_SMC_ECCMD_OFFSET         0x0024 /* SMC ECC Mode Register */
#  define SAM_SMC_ECCSR1_OFFSET        0x0028 /* SMC ECC Status 1 Register */
#  define SAM_SMC_ECCPR0_OFFSET        0x002c /* SMC ECC parity 0 Register */
#  define SAM_SMC_ECCPR1_OFFSET        0x0030 /* SMC ECC parity 1 Register */
#  define SAM_SMC_ECCSR2_OFFSET        0x0034 /* SMC ECC status 2 Register */
#  define SAM_SMC_ECCPR2_OFFSET        0x0038 /* SMC ECC parity 2 Register */
#  define SAM_SMC_ECCPR3_OFFSET        0x003c /* SMC ECC parity 3 Register */
#  define SAM_SMC_ECCPR4_OFFSET        0x0040 /* SMC ECC parity 4 Register */
#  define SAM_SMC_ECCPR5_OFFSET        0x0044 /* SMC ECC parity 5 Register */
#  define SAM_SMC_ECCPR6_OFFSET        0x0048 /* SMC ECC parity 6 Register */
#  define SAM_SMC_ECCPR7_OFFSET        0x004c /* SMC ECC parity 7 Register */
#  define SAM_SMC_ECCPR8_OFFSET        0x0050 /* SMC ECC parity 8 Register */
#  define SAM_SMC_ECCPR9_OFFSET        0x0054 /* SMC ECC parity 9 Register */
#  define SAM_SMC_ECCPR10_OFFSET       0x0058 /* SMC ECC parity 10 Register */
#  define SAM_SMC_ECCPR11_OFFSET       0x005c /* SMC ECC parity 11 Register */
#  define SAM_SMC_ECCPR12_OFFSET       0x0060 /* SMC ECC parity 12 Register */
#  define SAM_SMC_ECCPR13_OFFSET       0x0064 /* SMC ECC parity 13 Register */
#  define SAM_SMC_ECCPR14_OFFSET       0x0068 /* SMC ECC parity 14 Register */
#  define SAM_SMC_ECCPR15_OFFSET       0x006c /* SMC ECC parity 15 Register */

#  define SAM_SMCCS_OFFSET(n)          (0x0070+((n)*0x014))
#    define SAM_SMCCS0_OFFSET          0x0070 /* SMC CS0 offset */
#    define SAM_SMCCS1_OFFSET          0x0084 /* SMC CS1 offset */
#    define SAM_SMCCS2_OFFSET          0x0098 /* SMC CS2 offset */
#    define SAM_SMCCS3_OFFSET          0x00ac /* SMC CS3 offset */

#  define SAM_SMCCS_SETUP_OFFSET       0x0000 /* SMC Setup register */
#  define SAM_SMCCS_PULSE_OFFSET       0x0004 /* SMC Pulse Register */
#  define SAM_SMCCS_CYCLE_OFFSET       0x0008 /* SMC Cycle Register */
#  define SAM_SMCCS_TIMINGS_OFFSET     0x000c /* SMC Timings Register */
#  define SAM_SMCCS_MODE_OFFSET        0x0010 /* SMC Mode Register */

#  define SAM_SMC_OCMS_OFFSET          0x0110 /* SMC OCMS Mode Register */
#  define SAM_SMC_KEY1_OFFSET          0x0114 /* SMC KEY1 Register */
#  define SAM_SMC_KEY2_OFFSET          0x0118 /* SMC KEY2 Register */
#  define SAM_SMC_WPCR_OFFSET          0x01e4 /* Write Protection Control Register */
#  define SAM_SMC_WPSR_OFFSET          0x01e8 /* Write Protection Status Register */

#elif defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_SMCCS_OFFSET(n)          ((n) << 4)
#    define SAM_SMCCS0_OFFSET          0x0000 /* SMC CS0 offset */
#    define SAM_SMCCS1_OFFSET          0x0010 /* SMC CS1 offset */
#    define SAM_SMCCS2_OFFSET          0x0020 /* SMC CS2 offset */
#    define SAM_SMCCS3_OFFSET          0x0030 /* SMC CS3 offset */

#  define SAM_SMCCS_SETUP_OFFSET       0x0000 /* SMC Setup Register */
#  define SAM_SMCCS_PULSE_OFFSET       0x0004 /* SMC Pulse Register */
#  define SAM_SMCCS_CYCLE_OFFSET       0x0008 /* SMC Cycle Register */
#  define SAM_SMCCS_MODE_OFFSET        0x000c /* SMC Mode Register */

#  define SAM_SMC_OCMS_OFFSET          0x0080 /* SMC OCMS Mode Register */
#  define SAM_SMC_KEY1_OFFSET          0x0084 /* SMC KEY1 Register */
#  define SAM_SMC_KEY2_OFFSET          0x0088 /* SMC KEY2 Register */
#  define SAM_SMC_WPCR_OFFSET          0x00e4 /* Write Protection Control Register */
#  define SAM_SMC_WPSR_OFFSET          0x00e8 /* Write Protection Status Register */

#else
#  error Unrecognized SAM architecture
#endif

/* SMC register addresses ***************************************************************/

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SAM_SMC_CFG                  (SAM_SMC_BASE+SAM_SMC_CFG_OFFSET)
#  define SAM_SMC_CTRL                 (SAM_SMC_BASE+SAM_SMC_CTRL_OFFSET)
#  define SAM_SMC_SR                   (SAM_SMC_BASE+SAM_SMC_SR_OFFSET)
#  define SAM_SMC_IER                  (SAM_SMC_BASE+SAM_SMC_IER_OFFSET)
#  define SAM_SMC_IDR                  (SAM_SMC_BASE+SAM_SMC_IDR_OFFSET)
#  define SAM_SMC_IMR                  (SAM_SMC_BASE+SAM_SMC_IMR_OFFSET)
#  define SAM_SMC_ADDR                 (SAM_SMC_BASE+SAM_SMC_ADDR_OFFSET)
#  define SAM_SMC_BANK                 (SAM_SMC_BASE+SAM_SMC_BANK_OFFSET)
#  define SAM_SMC_ECCCTRL              (SAM_SMC_BASE+SAM_SMC_ECCCTRL_OFFSET)
#  define SAM_SMC_ECCMD                (SAM_SMC_BASE+SAM_SMC_ECCMD_OFFSET)
#  define SAM_SMC_ECCSR1               (SAM_SMC_BASE+SAM_SMC_ECCSR1_OFFSET)
#  define SAM_SMC_ECCPR0               (SAM_SMC_BASE+SAM_SMC_ECCPR0_OFFSET)
#  define SAM_SMC_ECCPR1               (SAM_SMC_BASE+SAM_SMC_ECCPR1_OFFSET)
#  define SAM_SMC_ECCSR2               (SAM_SMC_BASE+SAM_SMC_ECCSR2_OFFSET)
#  define SAM_SMC_ECCPR2               (SAM_SMC_BASE+SAM_SMC_ECCPR2_OFFSET)
#  define SAM_SMC_ECCPR3               (SAM_SMC_BASE+SAM_SMC_ECCPR3_OFFSET)
#  define SAM_SMC_ECCPR4               (SAM_SMC_BASE+SAM_SMC_ECCPR4_OFFSET)
#  define SAM_SMC_ECCPR5               (SAM_SMC_BASE+SAM_SMC_ECCPR5_OFFSET)
#  define SAM_SMC_ECCPR6               (SAM_SMC_BASE+SAM_SMC_ECCPR6_OFFSET)
#  define SAM_SMC_ECCPR7               (SAM_SMC_BASE+SAM_SMC_ECCPR7_OFFSET)
#  define SAM_SMC_ECCPR8               (SAM_SMC_BASE+SAM_SMC_ECCPR8_OFFSET)
#  define SAM_SMC_ECCPR9               (SAM_SMC_BASE+SAM_SMC_ECCPR9_OFFSET)
#  define SAM_SMC_ECCPR10              (SAM_SMC_BASE+SAM_SMC_ECCPR10_OFFSET)
#  define SAM_SMC_ECCPR11              (SAM_SMC_BASE+SAM_SMC_ECCPR11_OFFSET)
#  define SAM_SMC_ECCPR12              (SAM_SMC_BASE+SAM_SMC_ECCPR12_OFFSET)
#  define SAM_SMC_ECCPR13              (SAM_SMC_BASE+SAM_SMC_ECCPR13_OFFSET)
#  define SAM_SMC_ECCPR14              (SAM_SMC_BASE+SAM_SMC_ECCPR14_OFFSET)
#  define SAM_SMC_ECCPR15              (SAM_SMC_BASE+SAM_SMC_ECCPR15_OFFSET)
#endif

#define SAM_SMCCS_BASE(n)              (SAM_SMC_BASE+SAM_SMCCS_OFFSET(n))
#  define SAM_SMC_CS0_BASE             (SAM_SMC_BASE+SAM_SMCCS0_OFFSET)
#  define SAM_SMC_CS1_BASE             (SAM_SMC_BASE+SAM_SMCCS1_OFFSET)
#  define SAM_SMC_CS2_BASE             (SAM_SMC_BASE+SAM_SMCCS2_OFFSET)
#  define SAM_SMC_CS3_BASE             (SAM_SMC_BASE+SAM_SMCCS3_OFFSET)

#define SAM_SMCCS_SETUP(n)             (SAM_SMCCS_BASE(n)+SAM_SMCCS_SETUP_OFFSET)
#define SAM_SMCCS_PULSE(n)             (SAM_SMCCS_BASE(n)+SAM_SMCCS_PULSE_OFFSET)
#define SAM_SMCCS_CYCLE(n)             (SAM_SMCCS_BASE(n)+SAM_SMCCS_CYCLE_OFFSET)

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SAM_SMCCS_TIMINGS(n)         (SAM_SMCCS_BASE(n)+SAM_SMCCS_TIMINGS_OFFSET)
#endif

#define SAM_SMCCS_MODE(n)              (SAM_SMCCS_BASE(n)+SAM_SMCCS_MODE_OFFSET)

#  define SAM_SMCCS0_SETUP             (SAM_SMC_CS0_BASE+SAM_SMCCS_SETUP_OFFSET)
#  define SAM_SMCCS0_PULSE             (SAM_SMC_CS0_BASE+SAM_SMCCS_PULSE_OFFSET)
#  define SAM_SMCCS0_CYCLE             (SAM_SMC_CS0_BASE+SAM_SMCCS_CYCLE_OFFSET)

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#    define SAM_SMCCS0_TIMINGS         (SAM_SMC_CS0_BASE+SAM_SMCCS_TIMINGS_OFFSET)
#  endif

#  define SAM_SMCCS0_MODE              (SAM_SMC_CS0_BASE+SAM_SMCCS_MODE_OFFSET)

#  define SAM_SMCCS1_SETUP             (SAM_SMC_CS1_BASE+SAM_SMCCS_SETUP_OFFSET)
#  define SAM_SMCCS1_PULSE             (SAM_SMC_CS1_BASE+SAM_SMCCS_PULSE_OFFSET)
#  define SAM_SMCCS1_CYCLE             (SAM_SMC_CS1_BASE+SAM_SMCCS_CYCLE_OFFSET)

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#    define SAM_SMCCS1_TIMINGS         (SAM_SMC_CS1_BASE+SAM_SMCCS_TIMINGS_OFFSET)
#  endif

#  define SAM_SMCCS1_MODE              (SAM_SMC_CS1_BASE+SAM_SMCCS_MODE_OFFSET)

#  define SAM_SMCCS2_SETUP             (SAM_SMC_CS2_BASE+SAM_SMCCS_SETUP_OFFSET)
#  define SAM_SMCCS2_PULSE             (SAM_SMC_CS2_BASE+SAM_SMCCS_PULSE_OFFSET)
#  define SAM_SMCCS2_CYCLE             (SAM_SMC_CS2_BASE+SAM_SMCCS_CYCLE_OFFSET)

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#    define SAM_SMCCS2_TIMINGS         (SAM_SMC_CS2_BASE+SAM_SMCCS_TIMINGS_OFFSET)
#  endif

#  define SAM_SMCCS2_MODE              (SAM_SMC_CS2_BASE+SAM_SMCCS_MODE_OFFSET)

#  define SAM_SMCCS3_SETUP             (SAM_SMC_CS3_BASE+SAM_SMCCS_SETUP_OFFSET)
#  define SAM_SMCCS3_PULSE             (SAM_SMC_CS3_BASE+SAM_SMCCS_PULSE_OFFSET)
#  define SAM_SMCCS3_CYCLE             (SAM_SMC_CS3_BASE+SAM_SMCCS_CYCLE_OFFSET)

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#    define SAM_SMCCS3_TIMINGS         (SAM_SMC_CS3_BASE+SAM_SMCCS_TIMINGS_OFFSET)
#  endif

#  define SAM_SMCCS3_MODE              (SAM_SMC_CS3_BASE+SAM_SMCCS_MODE_OFFSET)

#define SAM_SMC_OCMS                   (SAM_SMC_BASE+SAM_SMC_OCMS_OFFSET)
#define SAM_SMC_KEY1                   (SAM_SMC_BASE+SAM_SMC_KEY1_OFFSET)
#define SAM_SMC_KEY2                   (SAM_SMC_BASE+SAM_SMC_KEY2_OFFSET)
#define SAM_SMC_WPCR                   (SAM_SMC_BASE+SAM_SMC_WPCR_OFFSET)
#define SAM_SMC_WPSR                   (SAM_SMC_BASE+SAM_SMC_WPSR_OFFSET)

/* SMC register bit definitions *********************************************************/

/* SMC NFC Configuration Register */

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SMC_CFG_PAGESIZE_SHIFT       (0)       /* Bits 0-1: Page size of NAND Flash device */
#  define SMC_CFG_PAGESIZE_MASK        (3 << SMC_CFG_PAGESIZE_SHIFT)
#    define SMC_CFG_PAGESIZE_528       (0 << SMC_CFG_PAGESIZE_SHIFT) /* 512 bytes + 16 byte spare */
#    define SMC_CFG_PAGESIZE_1056      (1 << SMC_CFG_PAGESIZE_SHIFT) /* 1024 Bytes + 32 bytes spare */
#    define SMC_CFG_PAGESIZE_2122      (2 << SMC_CFG_PAGESIZE_SHIFT) /* 2048 Bytes + 64 bytes spare */
#    define SMC_CFG_PAGESIZE_4224      (3 << SMC_CFG_PAGESIZE_SHIFT) /* 4096 Bytes + 128 bytes spare */
#  define SMC_CFG_WSPARE               (1 << 8)  /* Bit 8:  Write Spare Area */
#  define SMC_CFG_RSPARE               (1 << 9)  /* Bit 9:  Read Spare Area */
#  define SMC_CFG_EDGECTRL             (1 << 12) /* Bit 12: Rising/Falling Edge Detection Control */
#  define SMC_CFG_RBEDGE               (1 << 13) /* Bit 13: Ready/Busy Signal Edge Detection */
#  define SMC_CFG_DTOCYC_SHIFT         (16)      /* Bits 16-19: Data Timeout Cycle Number */
#  define SMC_CFG_DTOCYC_MASK          (15 << SMC_CFG_DTOCYC_SHIFT)
#  define SMC_CFG_DTOMUL_SHIFT         (20)      /* Bits 20-22: Data Timeout Multiplier */
#  define SMC_CFG_DTOMUL_MASK          (7 << SMC_CFG_DTOMUL_SHIFT)
#    define SMC_CFG_DTOMUL_1           (0 << SMC_CFG_DTOMUL_SHIFT)
#    define SMC_CFG_DTOMUL_16          (1 << SMC_CFG_DTOMUL_SHIFT)
#    define SMC_CFG_DTOMUL_128         (2 << SMC_CFG_DTOMUL_SHIFT)
#    define SMC_CFG_DTOMUL_256         (3 << SMC_CFG_DTOMUL_SHIFT)
#    define SMC_CFG_DTOMUL_1024        (4 << SMC_CFG_DTOMUL_SHIFT)
#    define SMC_CFG_DTOMUL_4096        (5 << SMC_CFG_DTOMUL_SHIFT)
#    define SMC_CFG_DTOMUL_65536       (6 << SMC_CFG_DTOMUL_SHIFT)
#    define SMC_CFG_DTOMUL_1048576     (7 << SMC_CFG_DTOMUL_SHIFT)
#endif

/* SMC NFC Control Register */

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SMC_CTRL_NFCEN               (1 << 0)  /* Bit 0:  NAND Flash Controller Enable */
#  define SMC_CTRL_NFCDIS              (1 << 1)  /* Bit 1:  NAND Flash Controller Disable */
#endif

/* SMC NFC Status Register, SMC NFC Interrupt Enable Register, SMC NFC Interrupt
 * Disable Register, and SMC NFC Interrupt Mask Register common bit-field definitions
 */

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SMC_SR_SMCSTS                (1 << 0)  /* Bit 0:  NAND Flash Controller status (SR only) */
#  define SMC_INT_RBRISE               (1 << 4)  /* Bit 4:  Ready Busy Rising Edge Detection Interrupt */
#  define SMC_INT_RBFALL               (1 << 5)  /* Bit 5:  Ready Busy Falling Edge Detection Interrupt */
#  define SMC_SR_NFCBUSY               (1 << 8)  /* Bit 8:  NFC Busy (SR only) */
#  define SMC_SR_NFCWR                 (1 << 11) /* Bit 11: NFC Write/Read Operation (SR only) */
#  define SMC_SR_NFCSID_SHIFT          (12)      /* Bits 12-14: NFC Chip Select ID (SR only) */
#  define SMC_SR_NFCSID_MASK           (7 << SMC_SR_NFCSID_SHIFT)
#  define SMC_INT_XFRDONE              (1 << 16) /* Bit 16: Transfer Done Interrupt */
#  define SMC_INT_CMDDONE              (1 << 17) /* Bit 17: Command Done Interrupt */
#  define SMC_INT_DTOE                 (1 << 20) /* Bit 20: Data Timeout Error Interrupt */
#  define SMC_INT_UNDEF                (1 << 21) /* Bit 21: Undefined Area Access Interrupt */
#  define SMC_INT_AWB                  (1 << 22) /* Bit 22: Accessing While Busy Interrupt */
#  define SMC_INT_NFCASE               (1 << 23) /* Bit 23: NFC Access Size Error Interrupt */
#ifdef CONFIG_ARCH_CHIP_SAM3U
#  define SMC_INT_RBEDGE(n)            (1<<((n)+24))
#  define SMC_INT_RB_EDGE0             (1 << 24) /* Bit 24: Ready/Busy Line 0 Interrupt */
#  define SMC_INT_RB_EDGE1             (1 << 25) /* Bit 25: Ready/Busy Line 1 Interrupt */
#  define SMC_INT_RB_EDGE2             (1 << 26) /* Bit 26: Ready/Busy Line 2 Interrupt */
#  define SMC_INT_RB_EDGE3             (1 << 27) /* Bit 27: Ready/Busy Line 3 Interrupt */
#  define SMC_INT_RB_EDGE4             (1 << 28) /* Bit 28: Ready/Busy Line 4 Interrupt */
#  define SMC_INT_RB_EDGE5             (1 << 29) /* Bit 29: Ready/Busy Line 5 Interrupt */
#  define SMC_INT_RB_EDGE6             (1 << 30) /* Bit 30: Ready/Busy Line 6 Interrupt */
#  define SMC_INT_RB_EDGE7             (1 << 31) /* Bit 31: Ready/Busy Line 7 Interrupt */
#else
#  define SMC_INT_RB_EDGE0             (1 << 24) /* Bit 24: Ready/Busy Line 0 Interrupt */
#endif
#endif

/* SMC NFC Address Cycle Zero Register */

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SMC_ADDR_CYCLE0_SHIFT       (3)       /* Bits 0-7: NAND Flash Array Address cycle 0 */
#  define SMC_ADDR_CYCLE0_MASK        (0xff << SMC_ADDR_CYCLE0_SHIFT)
#endif

/* SMC NFC Bank Register */

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SMC_BANK_SHIFT               (0)       /* Bits 0-2: Bank identifier */
#  define SMC_BANK_MASK                (7 << SMC_BANK_SHIFT)
#endif

/* SMC ECC Control Register */

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SMC_ECCCTRL_RST              (1 << 0)  /* Bit 0:  Reset ECC */
#  define SMC_ECCCTRL_SWRST            (1 << 1)  /* Bit 1:  Software Reset */
#endif

/* SMC ECC MODE Register */

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SMC_ECCMD_ECC_PAGESIZE_SHIFT  (0)       /* Bits 0-1 */
#  define SMC_ECCMD_ECC_PAGESIZE_MASK   (3 << SMC_ECCMD_ECC_PAGESIZE_SHIFT)
#    define SMC_ECCMD_ECC_PAGESIZE_528  (0 << SMC_ECCMD_ECC_PAGESIZE_SHIFT) /* 512 bytes + 16 byte spare */
#    define SMC_ECCMD_ECC_PAGESIZE_1056 (1 << SMC_ECCMD_ECC_PAGESIZE_SHIFT) /* 1024 Bytes + 32 bytes spare */
#    define SMC_ECCMD_ECC_PAGESIZE_2112 (2 << SMC_ECCMD_ECC_PAGESIZE_SHIFT) /* 2048 Bytes + 64 bytes spare */
#    define SMC_ECCMD_ECC_PAGESIZE_4224 (3 << SMC_ECCMD_ECC_PAGESIZE_SHIFT) /* 4096 Bytes + 128 bytes spare */
#  define SMC_ECCMD_TYPCORREC_SHIFT     (4)      /* Bits 4-5: type of correction */
#  define SMC_ECCMD_TYPCORREC_MASK      (3 << SMC_ECCMD_TYPCORREC_SHIFT)
#    define SMC_ECCMD_TYPCORREC_PAGE    (0 << SMC_ECCMD_TYPCORREC_SHIFT) /* 1 bit correction for a page */
#    define SMC_ECCMD_TYPCORREC_256     (1 << SMC_ECCMD_TYPCORREC_SHIFT) /* 1 bit correction for 256 bytes */
#    define SMC_ECCMD_TYPCORREC_512     (2 << SMC_ECCMD_TYPCORREC_SHIFT) /* 1 bit correction for 512 bytes */
#endif

/* SMC ECC Status Register 1 */

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define _RECERR                      (0) /* Recoverable Error */
#  define _ECCERR                      (1) /* ECC Error */
#  define _MULERR                      (2) /* Multiple Error */

#  define SMC_ECCSR1_RECERR(n)         (1 << (((n)<<4)+_RECERR))
#  define SMC_ECCSR1_ECCERR(n)         (1 << (((n)<<4)+_ECCERR))
#  define SMC_ECCSR1_MULERR(n)         (1 << (((n)<<4)+_MULERR))

#  define SMC_ECCSR1_RECERR0           SMC_ECCSR1_RECERR(0)
#  define SMC_ECCSR1_ECCERR0           SMC_ECCSR1_ECCERR(0)
#  define SMC_ECCSR1_MULERR0           SMC_ECCSR1_MULERR(0)
#  define SMC_ECCSR1_RECERR1           SMC_ECCSR1_RECERR(1)
#  define SMC_ECCSR1_ECCERR1           SMC_ECCSR1_ECCERR(1)
#  define SMC_ECCSR1_MULERR1           SMC_ECCSR1_MULERR(1)
#  define SMC_ECCSR1_RECERR2           SMC_ECCSR1_RECERR(2)
#  define SMC_ECCSR1_ECCERR2           SMC_ECCSR1_ECCERR(2)
#  define SMC_ECCSR1_MULERR2           SMC_ECCSR1_MULERR(2)
#  define SMC_ECCSR1_RECERR3           SMC_ECCSR1_RECERR(3)
#  define SMC_ECCSR1_ECCERR3           SMC_ECCSR1_ECCERR(3)
#  define SMC_ECCSR1_MULERR3           SMC_ECCSR1_MULERR(3)
#  define SMC_ECCSR1_RECERR4           SMC_ECCSR1_RECERR(4)
#  define SMC_ECCSR1_ECCERR4           SMC_ECCSR1_ECCERR(4)
#  define SMC_ECCSR1_MULERR4           SMC_ECCSR1_MULERR(4)
#  define SMC_ECCSR1_RECERR5           SMC_ECCSR1_RECERR(5)
#  define SMC_ECCSR1_ECCERR5           SMC_ECCSR1_ECCERR(5)
#  define SMC_ECCSR1_MULERR5           SMC_ECCSR1_MULERR(5)
#  define SMC_ECCSR1_RECERR6           SMC_ECCSR1_RECERR(6)
#  define SMC_ECCSR1_ECCERR6           SMC_ECCSR1_ECCERR(6)
#  define SMC_ECCSR1_MULERR6           SMC_ECCSR1_MULERR(6)
#  define SMC_ECCSR1_RECERR7           SMC_ECCSR1_RECERR(7)
#  define SMC_ECCSR1_ECCERR7           SMC_ECCSR1_ECCERR(7)
#  define SMC_ECCSR1_MULERR7           SMC_ECCSR1_MULERR(7)
#endif

/* SMC ECC Status Register 2 */

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SMC_ECCSR2_RECERR(n)         (1 << ((((n)-8)<<4)+_RECERR))
#  define SMC_ECCSR2_ECCERR(n)         (1 << ((((n)-8)<<4)+_ECCERR))
#  define SMC_ECCSR2_MULERR(n)         (1 << ((((n)-8)<<4)+_MULERR))

#  define SMC_ECCSR2_RECERR8           SMC_ECCSR2_RECERR(8)
#  define SMC_ECCSR2_ECCERR8           SMC_ECCSR2_ECCERR(8)
#  define SMC_ECCSR2_MULERR8           SMC_ECCSR2_MULERR(8)
#  define SMC_ECCSR2_RECERR9           SMC_ECCSR2_RECERR(9)
#  define SMC_ECCSR2_ECCERR9           SMC_ECCSR2_ECCERR(9)
#  define SMC_ECCSR2_MULERR9           SMC_ECCSR2_MULERR(9)
#  define SMC_ECCSR2_RECERR10          SMC_ECCSR2_RECERR(10)
#  define SMC_ECCSR2_ECCERR10          SMC_ECCSR2_ECCERR(10)
#  define SMC_ECCSR2_MULERR10          SMC_ECCSR2_MULERR(10)
#  define SMC_ECCSR2_RECERR11          SMC_ECCSR2_RECERR(11)
#  define SMC_ECCSR2_ECCERR11          SMC_ECCSR2_ECCERR(11)
#  define SMC_ECCSR2_MULERR11          SMC_ECCSR2_MULERR(11)
#  define SMC_ECCSR2_RECERR12          SMC_ECCSR2_RECERR(12)
#  define SMC_ECCSR2_ECCERR12          SMC_ECCSR2_ECCERR(12)
#  define SMC_ECCSR2_MULERR12          SMC_ECCSR2_MULERR(12)
#  define SMC_ECCSR2_RECERR13          SMC_ECCSR2_RECERR(13)
#  define SMC_ECCSR2_ECCERR13          SMC_ECCSR2_ECCERR(13)
#  define SMC_ECCSR2_MULERR13          SMC_ECCSR2_MULERR(13)
#  define SMC_ECCSR1_RECERR14          SMC_ECCSR2_RECERR(14)
#  define SMC_ECCSR1_ECCERR14          SMC_ECCSR2_ECCERR(14)
#  define SMC_ECCSR1_MULERR14          SMC_ECCSR2_MULERR(14)
#  define SMC_ECCSR1_RECERR15          SMC_ECCSR2_RECERR(15)
#  define SMC_ECCSR1_ECCERR15          SMC_ECCSR2_ECCERR(15)
#  define SMC_ECCSR1_MULERR15          SMC_ECCSR2_MULERR(15)
#endif

/* Registers for 1 ECC for a page of 512/1024/2048/4096 bytes */
/* SMC_ECC_PR0 */

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SMC_ECCPR0_BITADDR_SHIFT     (0)       /* Bits 0-3: Bit Address */
#  define SMC_ECCPR0_BITADDR_MASK      (15 << SMC_ECCPR0_BITADDR_SHIFT)
#  define SMC_ECCPR0_WORDADDR_SHIFT    (4)       /* Bits 4-15: Word Address */
#  define SMC_ECCPR0_WORDADDR_MASK     (0xfff << SMC_ECCPR0_WORDADDR_SHIFT)

#ifdef CONFIG_ARCH_CHIP_SAM3U
#  define SMC_ECCPR1_NPARITY_SHIFT     (0)       /* Bits 0-15 */
#  define SMC_ECCPR1_NPARITY_MASK      (0xffff << SMC_ECCPR1_NPARITY_SHIFT)
#endif
#endif

/* Registers for 1 ECC per 512 bytes for a page of 512/2048/4096 bytes, 8-bit word */

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SMC_ECCPR512_BITADDR_SHIFT   (0)       /* Bits 0-3: Bit Address */
#  define SMC_ECCPR512_BITADDR_MASK    (15 << SMC_ECCPR512_BITADDR_SHIFT)
#  define SMC_ECCPR512_WORDADDR_SHIFT  (4)       /* Bits 4-15: Word Address */
#  define SMC_ECCPR512_WORDADDR_MASK   (0xfff << SMC_ECCPR512_WORDADDR_SHIFT)
#  define SMC_ECCPR512_NPARITY_SHIFT   (12)      /* Bits 12-23 (or is it 31?) */
#  define SMC_ECCPR512_NPARITY_MASK    (0xfff << SMC_ECCPR512_NPARITY_SHIFT)
#endif

/* Registers for 1 ECC per 256 bytes for a page of 512/2048/4096 bytes, 8-bit word */

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SMC_ECCPR256_BITADDR_SHIFT   (0)       /* Bits 0-2: Bit Address */
#  define SMC_ECCPR256_BITADDR_MASK    (7 << SMC_ECCPR256_BITADDR_SHIFT)
#  define SMC_ECCPR256_WORDADDR_SHIFT  (4)       /* Bits 4-10: Word Address */
#  define SMC_ECCPR256_WORDADDR_MASK   (0x7f << SMC_ECCPR256_WORDADDR_SHIFT)
#  define SMC_ECCPR256_NPARITY_SHIFT   (12)      /* Bits 12-22 */
#  define SMC_ECCPR256_NPARITY_MASK    (0x7ff << SMC_ECCPR256_NPARITY_SHIFT)
#endif

/* SMC Setup Register */

#define SMCCS_SETUP_NWESETUP_SHIFT     (0)       /* Bits 0-5: NWE Setup length */
#define SMCCS_SETUP_NWESETUP_MASK      (63 << SMCCS_SETUP_NWESETUP_SHIFT)
#  define SMCCS_SETUP_NWESETUP(n)      ((n) << SMCCS_SETUP_NWESETUP_SHIFT)
#define SMCCS_SETUP_NCSWRSETUP_SHIFT   (8)       /* Bits 8-13: NCS Setup length in Write access */
#define SMCCS_SETUP_NCSWRSETUP_MASK    (63 << SMCCS_SETUP_NCSWRSETUP_SHIFT)
#  define SMCCS_SETUP_NCSWRSETUP(n)    ((n) << SMCCS_SETUP_NCSWRSETUP_SHIFT)
#define SMCCS_SETUP_NRDSETUP_SHIFT     (16)      /* Bits 16-21: NRD Setup length */
#define SMCCS_SETUP_NRDSETUP_MASK      (63 << SMCCS_SETUP_NRDSETUP_SHIFT)
#  define SMCCS_SETUP_NRDSETUP(n)      ((n) << SMCCS_SETUP_NRDSETUP_SHIFT)
#define SMCCS_SETUP_NCSRDSETUP_SHIFT   (24)      /* Bits 24-29: NCS Setup length in Read access */
#define SMCCS_SETUP_NCSRDSETUP_MASK    (63 << SMCCS_SETUP_NCSRDSETUP_SHIFT)
#  define SMCCS_SETUP_NCSRDSETUP(n)    ((n) << SMCCS_SETUP_NCSRDSETUP_SHIFT)

/* SMC Pulse Register */

#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SMCCS_PULSE_NWEPULSE_SHIFT   (0)       /* Bits 0-6: NWE Pulse Length */
#  define SMCCS_PULSE_NWEPULSE_MASK    (127 << SMCCS_PULSE_NWEPULSE_SHIFT)
#    define SMCCS_PULSE_NWEPULSE(n)    ((n) << SMCCS_PULSE_NWEPULSE_SHIFT)
#  define SMCCS_PULSE_NCSWRPULSE_SHIFT (8)       /* Bits 8-14: NCS Pulse Length in WRITE Access */
#  define SMCCS_PULSE_NCSWRPULSE_MASK  (127 << SMCCS_PULSE_NCSWRPULSE_SHIFT)
#    define SMCCS_PULSE_NCSWRPULSE(n)  ((n) << SMCCS_PULSE_NCSWRPULSE_SHIFT)
#  define SMCCS_PULSE_NRDPULSE_SHIFT   (16)      /* Bits 16-22: NRD Pulse Length */
#  define SMCCS_PULSE_NRDPULSE_MASK    (127 << SMCCS_PULSE_NRDPULSE_SHIFT)
#    define SMCCS_PULSE_NRDPULSE(n)    ((n) << SMCCS_PULSE_NRDPULSE_SHIFT)
#  define SMCCS_PULSE_NCSRDPULSE_SHIFT (24)      /* Bits 24-30: NCS Pulse Length in READ Access */
#  define SMCCS_PULSE_NCSRDPULSE_MASK  (127 << SMCCS_PULSE_NCSRDPULSE_SHIFT)
#    define SMCCS_PULSE_NCSRDPULSE(n)  ((n) << SMCCS_PULSE_NCSRDPULSE_SHIFT)
#else
#  define SMCCS_PULSE_NWEPULSE_SHIFT   (0)       /* Bits 0-5: NWE Pulse Length */
#  define SMCCS_PULSE_NWEPULSE_MASK    (63 << SMCCS_PULSE_NWEPULSE_SHIFT)
#    define SMCCS_PULSE_NWEPULSE(n)    ((n) << SMCCS_PULSE_NWEPULSE_SHIFT)
#  define SMCCS_PULSE_NCSWRPULSE_SHIFT (8)       /* Bits 8-13: NCS Pulse Length in WRITE Access */
#  define SMCCS_PULSE_NCSWRPULSE_MASK  (63 << SMCCS_PULSE_NCSWRPULSE_SHIFT)
#    define SMCCS_PULSE_NCSWRPULSE(n)  ((n) << SMCCS_PULSE_NCSWRPULSE_SHIFT)
#  define SMCCS_PULSE_NRDPULSE_SHIFT   (16)      /* Bits 16-21: NRD Pulse Length */
#  define SMCCS_PULSE_NRDPULSE_MASK    (63 << SMCCS_PULSE_NRDPULSE_SHIFT)
#    define SMCCS_PULSE_NRDPULSE(n)    ((n) << SMCCS_PULSE_NRDPULSE_SHIFT)
#  define SMCCS_PULSE_NCSRDPULSE_SHIFT (24)      /* Bits 24-29: NCS Pulse Length in READ Access */
#  define SMCCS_PULSE_NCSRDPULSE_MASK  (63 << SMCCS_PULSE_NCSRDPULSE_SHIFT)
#    define SMCCS_PULSE_NCSRDPULSE(n)  ((n) << SMCCS_PULSE_NCSRDPULSE_SHIFT)
#endif

/* SMC Cycle Register */

#define SMCCS_CYCLE_NWECYCLE_SHIFT     (0)       /* Bits 0-8: Total Write Cycle Length */
#define SMCCS_CYCLE_NWECYCLE_MASK      (0x1ff << SMCCS_CYCLE_NWECYCLE_SHIFT)
#  define SMCCS_CYCLE_NWECYCLE(n)      ((n) << SMCCS_CYCLE_NWECYCLE_SHIFT)
#define SMCCS_CYCLE_NRDCYCLE_SHIFT     (16)      /* Bits 16-24: Total Read Cycle Length */
#define SMCCS_CYCLE_NRDCYCLE_MASK      (0x1ff << SMCCS_CYCLE_NRDCYCLE_SHIFT)
#  define SMCCS_CYCLE_NRDCYCLE(n)      ((n) << SMCCS_CYCLE_NRDCYCLE_SHIFT)

/* SMC Timings Register */

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SMCCS_TIMINGS_TCLR_SHIFT     (0)       /* Bits 0-3: CLE to REN Low Delay */
#  define SMCCS_TIMINGS_TCLR_MASK      (15 << SMCCS_TIMINGS_TCLR_SHIFT)
#  define SMCCS_TIMINGS_TADL_SHIFT     (4)       /* Bits 4-7: ALE to Data Start */
#  define SMCCS_TIMINGS_TADL_MASK      (15 << SMCCS_TIMINGS_TADL_SHIFT)
#  define SMCCS_TIMINGS_TAR_SHIFT      (8)       /* Bits 8-11: ALE to REN Low Delay */
#  define SMCCS_TIMINGS_TAR_MASK       (15 << SMCCS_TIMINGS_TAR_SHIFT)
#  define SMCCS_TIMINGS_OCMS           (1 << 12) /* Bit 12: Off Chip Memory Scrambling Enable */
#  define SMCCS_TIMINGS_TRR_SHIFT      (16)      /* Bits 16-19: Ready to REN Low Delay */
#  define SMCCS_TIMINGS_TRR_MASK       (15 << SMCCS_TIMINGS_TRR_SHIFT)
#  define SMCCS_TIMINGS_TWB_SHIFT      (24)      /* Bits 24-27: WEN High to REN to Busy */
#  define SMCCS_TIMINGS_TWB_MASK       (15 << SMCCS_TIMINGS_TWB_SHIFT)
#  define SMCCS_TIMINGS_RBNSEL_SHIFT   (28)      /* Bits 28-30: Ready/Busy Line Selection */
#  define SMCCS_TIMINGS_RBNSEL_MASK    (7 << SMCCS_TIMINGS_RBNSEL_SHIFT)
#  define SMCCS_TIMINGS_NFSEL          (1 << 31) /* Bit 31: NAND Flash Selection */
#endif

/* SMC Mode Register */

#define SMCCS_MODE_READMODE            (1 << 0)  /* Bit 0: Read mode */
#define SMCCS_MODE_WRITEMODE           (1 << 1)  /* Bit 1: Write mode */
#define SMCCS_MODE_EXNWMODE_SHIFT      (4)       /* Bits 4-5: NWAIT Mode */
#define SMCCS_MODE_EXNWMODE_MASK       (3 << SMCCS_MODE_EXNWMODE_SHIFT)
#  define SMCCS_EXNWMODE_DISABLED      (0 << SMCCS_MODE_EXNWMODE_SHIFT)
#  define SMCCS_EXNWMODE_FROZEN        (2 << SMCCS_MODE_EXNWMODE_SHIFT)
#  define SMCCS_EXNWMODE_READY         (3 << SMCCS_MODE_EXNWMODE_SHIFT)

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SMCCS_MODE_BAT               (1 << 8)  /* Bit 8:  Byte Access Type */
#  define SMCCS_MODE_DBW_SHIFT         (12)      /* Bits 12-13: Data Bus Width */
#  define SMCCS_MODE_DBW_MASK          (3 << SMCCS_MODE_DBW_SHIFT)
#    define SMCCS_MODE_DBW_8BITS       (0 << 12) /* 8 bits */
#    define SMCCS_MODE_DBW_16BITS      (1 << 12) /* 16 bits */
#    define SMCCS_MODE_DBW_32BITS      (2 << 12) /* 32 bits */
#endif

#define SMCCS_MODE_TDFCYCLES_SHIFT     (16)      /* Bits 16-19: Data Float Time */
#define SMCCS_MODE_TDFCYCLES_MASK      (15 << SMCCS_MODE_TDFCYCLES_SHIFT)
#  define SMCCS_MODE_TDFCYCLES(n)      ((uint32_t)(n) << SMCCS_MODE_TDFCYCLES_SHIFT)
#define SMCCS_MODE_TDFMODE             (1 << 20) /* Bit 20: TDF Optimization */

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM4S) || \
    defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SMCCS_MODE_PMEN              (1 << 24) /* Bit 24: Page Mode Enabled */
#  define SMCCS_MODE_PS_SHIFT          (28) /* Bits 28-29: Page Size */
#  define SMCCS_MODE_PS_MASK           (3 << SMCCS_MODE_PS_SHIFT)
#    define SMCCS_MODE_PS_SIZE_4BYTES  (0 << SMCCS_MODE_PS_SHIFT) /* 4 bytes */
#    define SMCCS_MODE_PS_SIZE_8BYTES  (1 << SMCCS_MODE_PS_SHIFT) /* 8 bytes */
#    define SMCCS_MODE_PS_SIZE_16BYTES (2 << SMCCS_MODE_PS_SHIFT) /* 16 bytes */
#    define SMCCS_MODE_PS_SIZE_32BYTES (3 << SMCCS_MODE_PS_SHIFT) /* 32 bytes */
#endif

/* SMC OCMS Mode Register */

#define SMC_OCMS_SMSE                  (1 << 0)  /* Bit 0:  Static Memory Controller Scrambling Enable */

#if !defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SMC_OCMS_SRSE                (1 << 1)  /* Bit 1:  SRAM Scrambling Enable */
#endif

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SMC_OCMS_CSSE(n)             (1 << ((n)+16)) /* Chip Select (n=0-3) Scrambling Enable */
#  define SMC_OCMS_CS0SE               (1 << 16) /* Bit 16: Chip Select 0 Scrambling Enable */
#  define SMC_OCMS_CS1SE               (1 << 17) /* Bit 17: Chip Select 1 Scrambling Enable */
#  define SMC_OCMS_CS2SE               (1 << 18) /* Bit 18: Chip Select 2 Scrambling Enable */
#  define SMC_OCMS_CS3SE               (1 << 19) /* Bit 19: Chip Select 3 Scrambling Enable */
#endif

/* SMC KEY1/2 Registers (32-bit data) */

/* SMC Write Protect Mode Register */

#define SMC_WPCR_WPPEN                 (1 << 0)  /* Bit 0:  Write Protection Enable */
#define SMC_WPCR_WPKEY_SHIFT           (8)       /* Bits 8-31: Write Protection KEY password */
#define SMC_WPCR_WPKEY_MASK            (0x00ffffff << SMC_WPCR_WPKEY_SHIFT)
#  define SMC_WPCR_WPKEY               (0x00534d43 << SMC_WPCR_WPKEY_SHIFT)

/* SMC Write Protection Status */

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SMC_WPSR_PVS_SHIFT           (0)       /* Bits 0-3: Write Protection Violation Status */
#  define SMC_WPSR_PVS_MASK            (15 << SMC_WPSR_PVS_SHIFT)
#    define SMC_WPSR_PVS_NONE          (0 << SMC_WPSR_PVS_SHIFT) /* No Write Protection Violation */
#    define SMC_WPSR_PVS_ RCREG        (1 << SMC_WPSR_PVS_SHIFT) /* Attempt to write a control reg */
#    define SMC_WPSR_PVS_RESET         (2 << SMC_WPSR_PVS_SHIFT) /* Software reset */
#    define SMC_WPSR_PVS_BOTH          (3 << SMC_WPSR_PVS_SHIFT) /* Write + reset */
#elif defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#    define SMC_WPSR_WPVS              (1 << 0)  /* Bit 0:  Write Protect Violation Source */
#endif

#define SMC_WPSR_WPVSRC_SHIFT          (8)       /* Bits 8-23: Write Protection Violation Source */
#define SMC_WPSR_WPVSRC_MASK           (0xffff << SMC_WPSR_WPVSRC_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_SMC_H */
