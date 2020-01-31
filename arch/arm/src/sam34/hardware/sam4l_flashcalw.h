/************************************************************************************
 * arch/arm/src/sam34/hardware/sam4l_flashcalw.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This file is derived from nuttx/arch/avr/src/at32uc3/at32uc3_flashc.h.
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

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4L_FLASHCALW_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4L_FLASHCALW_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/
/* Relative to SAM_FLASHCALW_BASE */

#define SAM_FLASHCALW_FCR_OFFSET     0x0000 /* Flash Control Register */
#define SAM_FLASHCALW_FCMD_OFFSET    0x0004 /* Flash Command Register */
#define SAM_FLASHCALW_FSR_OFFSET     0x0008 /* Flash Status Register */
#define SAM_FLASHCALW_FPR_OFFSET     0x000c /* Flash Parameter Register */
#define SAM_FLASHCALW_FVR_OFFSET     0x0010 /* Flash Version Register */
#define SAM_FLASHCALW_FGPFRHI_OFFSET 0x0014 /* Flash General Purpose Fuse Register Hi */
#define SAM_FLASHCALW_FGPFRLO_OFFSET 0x0018 /* Flash General Purpose Fuse Register Lo */

/* Relative to SAM_PICOCACHE_BASE */

#define SAM_PICOCACHE_CTRL_OFFSET   0x0008 /* PicoCache Control Register */
#define SAM_PICOCACHE_SR_OFFSET     0x000c /* PicoCache Status Register */
#define SAM_PICOCACHE_MAINT0_OFFSET 0x0020 /* PicoCache Maintenance Register 0 */
#define SAM_PICOCACHE_MAINT1_OFFSET 0x0024 /* PicoCache Maintenance Register 1 */
#define SAM_PICOCACHE_MCFG_OFFSET   0x0028 /* PicoCache Monitor Configuration Register */
#define SAM_PICOCACHE_MEN_OFFSET    0x002c /* PicoCache Monitor Enable Register */
#define SAM_PICOCACHE_MCTRL_OFFSET  0x0030 /* PicoCache Monitor Control Register */
#define SAM_PICOCACHE_MSR_OFFSET    0x0034 /* PicoCache Monitor Status Register */
#define SAM_PICOCACHE_PVR_OFFSET    0x00fc /* Version Register */

/* Register Addresses ***************************************************************/

#define SAM_FLASHCALW_FCR           (SAM_FLASHCALW_BASE+SAM_FLASHCALW_FCR_OFFSET)
#define SAM_FLASHCALW_FCMD          (SAM_FLASHCALW_BASE+SAM_FLASHCALW_FCMD_OFFSET)
#define SAM_FLASHCALW_FSR           (SAM_FLASHCALW_BASE+SAM_FLASHCALW_FSR_OFFSET)
#define SAM_FLASHCALW_FPR           (SAM_FLASHCALW_BASE+SAM_FLASHCALW_FPR_OFFSET)
#define SAM_FLASHCALW_FVR           (SAM_FLASHCALW_BASE+SAM_FLASHCALW_FVR_OFFSET)
#define SAM_FLASHCALW_FGPFRHI       (SAM_FLASHCALW_BASE+SAM_FLASHCALW_FGPFRHI_OFFSET)
#define SAM_FLASHCALW_FGPFRLO       (SAM_FLASHCALW_BASE+SAM_FLASHCALW_FGPFRLO_OFFSET)

#define SAM_PICOCACHE_CTRL          (SAM_PICOCACHE_BASE+SAM_PICOCACHE_CTRL_OFFSET)
#define SAM_PICOCACHE_SR            (SAM_PICOCACHE_BASE+SAM_PICOCACHE_SR_OFFSET)
#define SAM_PICOCACHE_MAINT0        (SAM_PICOCACHE_BASE+SAM_PICOCACHE_MAINT0_OFFSET)
#define SAM_PICOCACHE_MAINT1        (SAM_PICOCACHE_BASE+SAM_PICOCACHE_MAINT1_OFFSET)
#define SAM_PICOCACHE_MCFG          (SAM_PICOCACHE_BASE+SAM_PICOCACHE_MCFG_OFFSET)
#define SAM_PICOCACHE_MEN           (SAM_PICOCACHE_BASE+SAM_PICOCACHE_MEN_OFFSET)
#define SAM_PICOCACHE_MCTRL         (SAM_PICOCACHE_BASE+SAM_PICOCACHE_MCTRL_OFFSET)
#define SAM_PICOCACHE_MSR           (SAM_PICOCACHE_BASE+SAM_PICOCACHE_MSR_OFFSET)
#define SAM_PICOCACHE_PVR           (SAM_PICOCACHE_BASE+SAM_PICOCACHE_PVR_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/* Flash Control Register */

#define FLASHCALW_FCR_FRDY             (1 << 0)  /* Bit 0:  Flash Ready Interrupt Enable */
#define FLASHCALW_FCR_LOCKE            (1 << 2)  /* Bit 2:  Lock Error Interrupt Enable */
#define FLASHCALW_FCR_PROGE            (1 << 3)  /* Bit 3:  Programming Error Interrupt Enable */
#define FLASHCALW_FCR_ECCE             (1 << 4)  /* Bit 4:  ECC Error Interrupt Enable */
#define FLASHCALW_FCR_FWS              (1 << 6)  /* Bit 6:  Flash Wait State */
#define FLASHCALW_FCR_WS1OPT           (1 << 7)  /* Bit 7:  Wait State 1 Optimization */

/* Flash Command Register */

#define FLASHCALW_FCMD_CMD_SHIFT       (0)       /* Bits 0-5: Command */
#define FLASHCALW_FCMD_CMD_MASK        (0x3f << FLASHCALW_FCMD_CMD_SHIFT)
#  define FLASHCALW_FCMD_CMD_NOP       (0 << FLASHCALW_FCMD_CMD_SHIFT)  /* No operation */
#  define FLASHCALW_FCMD_CMD_WP        (1 << FLASHCALW_FCMD_CMD_SHIFT)  /* Write Page */
#  define FLASHCALW_FCMD_CMD_EP        (2 << FLASHCALW_FCMD_CMD_SHIFT)  /* Erase Page */
#  define FLASHCALW_FCMD_CMD_CPB       (3 << FLASHCALW_FCMD_CMD_SHIFT)  /* Clear Page Buffer */
#  define FLASHCALW_FCMD_CMD_LP        (4 << FLASHCALW_FCMD_CMD_SHIFT)  /* Lock region containing given Page */
#  define FLASHCALW_FCMD_CMD_UP        (5 << FLASHCALW_FCMD_CMD_SHIFT)  /* Unlock region containing given Page */
#  define FLASHCALW_FCMD_CMD_EA        (6 << FLASHCALW_FCMD_CMD_SHIFT)  /* Erase All */
#  define FLASHCALW_FCMD_CMD_WGPB      (7 << FLASHCALW_FCMD_CMD_SHIFT)  /* Write General-Purpose Fuse Bit */
#  define FLASHCALW_FCMD_CMD_EGPB      (8 << FLASHCALW_FCMD_CMD_SHIFT)  /* Erase General-Purpose Fuse Bit */
#  define FLASHCALW_FCMD_CMD_SSB       (9 << FLASHCALW_FCMD_CMD_SHIFT)  /* Set Security Fuses */
#  define FLASHCALW_FCMD_CMD_PGPFB     (10 << FLASHCALW_FCMD_CMD_SHIFT) /* Program GP Fuse Byte */
#  define FLASHCALW_FCMD_CMD_EAGPF     (11 << FLASHCALW_FCMD_CMD_SHIFT) /* Erase All GPFuses */
#  define FLASHCALW_FCMD_CMD_QPR       (12 << FLASHCALW_FCMD_CMD_SHIFT) /* Quick Page Read */
#  define FLASHCALW_FCMD_CMD_WUP       (13 << FLASHCALW_FCMD_CMD_SHIFT) /* Write User Page */
#  define FLASHCALW_FCMD_CMD_EUP       (14 << FLASHCALW_FCMD_CMD_SHIFT) /* Erase User Page */
#  define FLASHCALW_FCMD_CMD_QPRUP     (15 << FLASHCALW_FCMD_CMD_SHIFT) /* Quick Page Read User Page */
#  define FLASHCALW_FCMD_CMD_HSEN      (16 << FLASHCALW_FCMD_CMD_SHIFT) /* High Speed Mode Enable */
#  define FLASHCALW_FCMD_CMD_HSDIS     (17 << FLASHCALW_FCMD_CMD_SHIFT) /* High Speed Mode Disable */
#define FLASHCALW_FCMD_PAGEN_SHIFT     (8)       /* Bits 8-23: Page number */
#define FLASHCALW_FCMD_PAGEN_MASK      (0xffff << FLASHCALW_FCMD_PAGEN_SHIFT)
#define FLASHCALW_FCMD_KEY_SHIFT       (14)      /* Bits 24-31: Write protection key */
#define FLASHCALW_FCMD_KEY_MASK        (0xff << FLASHCALW_FCMD_KEY_SHIFT)
#  define FLASHCALW_FCMD_KEY           (0xa5 << FLASHCALW_FCMD_KEY_SHIFT)

/* Flash Status Register */

#define FLASHCALW_FSR_FRDY             (1 << 0)  /* Bit 0:  Flash Ready Status */
#define FLASHCALW_FSR_LOCKE            (1 << 2)  /* Bit 2:  Lock Error Status */
#define FLASHCALW_FSR_PROGE            (1 << 3)  /* Bit 3:  Programming Error Status */
#define FLASHCALW_FSR_SECURITY         (1 << 4)  /* Bit 4:  Security Bit Status */
#define FLASHCALW_FSR_QPRR             (1 << 5)  /* Bit 5:  Quick Page Read Result */
#define FLASHCALW_FSR_HSMODE           (1 << 6)  /* Bit 6:  High-Speed Mode */
#define FLASHCALW_FSR_ECCERR_SHIFT     (8)       /* Bits 8-0: ECC Error Status */
#define FLASHCALW_FSR_ECCERR_MASK      (3 << FLASHCALW_FSR_ECCERR_SHIFT)
#define FLASHCALW_FSR_LOCK(n)          (1 << ((n)+16)
#define FLASHCALW_FSR_LOCK0            (1 << 16) /* Bit 16: Lock Region 0 Lock Status */
#define FLASHCALW_FSR_LOCK1            (1 << 17) /* Bit 17: Lock Region 1 Lock Status */
#define FLASHCALW_FSR_LOCK2            (1 << 18) /* Bit 18: Lock Region 2 Lock Status */
#define FLASHCALW_FSR_LOCK3            (1 << 19) /* Bit 19: Lock Region 3 Lock Status */
#define FLASHCALW_FSR_LOCK4            (1 << 20) /* Bit 20: Lock Region 4 Lock Status */
#define FLASHCALW_FSR_LOCK5            (1 << 21) /* Bit 21: Lock Region 5 Lock Status */
#define FLASHCALW_FSR_LOCK6            (1 << 22) /* Bit 22: Lock Region 6 Lock Status */
#define FLASHCALW_FSR_LOCK7            (1 << 23) /* Bit 23: Lock Region 7 Lock Status */
#define FLASHCALW_FSR_LOCK8            (1 << 24) /* Bit 24: Lock Region 8 Lock Status */
#define FLASHCALW_FSR_LOCK9            (1 << 25) /* Bit 25: Lock Region 9 Lock Status */
#define FLASHCALW_FSR_LOCK10           (1 << 26) /* Bit 26: Lock Region 10 Lock Status */
#define FLASHCALW_FSR_LOCK11           (1 << 27) /* Bit 27: Lock Region 11 Lock Status */
#define FLASHCALW_FSR_LOCK12           (1 << 28) /* Bit 28: Lock Region 12 Lock Status */
#define FLASHCALW_FSR_LOCK13           (1 << 29) /* Bit 29: Lock Region 13 Lock Status */
#define FLASHCALW_FSR_LOCK14           (1 << 30) /* Bit 30: Lock Region 14 Lock Status */
#define FLASHCALW_FSR_LOCK15           (1 << 31) /* Bit 31: Lock Region 15 Lock Status */

/* Flash Parameter Register */

#define FLASHCALW_FPR_FSZ_SHIFT        (0)       /* Bits 0-3: Flash Size */
#define FLASHCALW_FPR_FSZ_MASK         (15 << FLASHCALW_FPR_FSZ_SHIFT)
#  define FLASHCALW_FPR_FSZ_4KB        (0 << FLASHCALW_FPR_FSZ_SHIFT)  /* 4 Kbytes */
#  define FLASHCALW_FPR_FSZ_8KB        (1 << FLASHCALW_FPR_FSZ_SHIFT)  /* 8 Kbytes */
#  define FLASHCALW_FPR_FSZ_16KB       (2 << FLASHCALW_FPR_FSZ_SHIFT)  /* 16 Kbytes */
#  define FLASHCALW_FPR_FSZ_32KB       (3 << FLASHCALW_FPR_FSZ_SHIFT)  /* 32 Kbytes */
#  define FLASHCALW_FPR_FSZ_48KB       (4 << FLASHCALW_FPR_FSZ_SHIFT)  /* 48 Kbytes */
#  define FLASHCALW_FPR_FSZ_64KB       (5 << FLASHCALW_FPR_FSZ_SHIFT)  /* 64 Kbytes */
#  define FLASHCALW_FPR_FSZ_96KB       (6 << FLASHCALW_FPR_FSZ_SHIFT)  /* 96 Kbytes */
#  define FLASHCALW_FPR_FSZ_128KB      (7 << FLASHCALW_FPR_FSZ_SHIFT)  /* 128 Kbytes */
#  define FLASHCALW_FPR_FSZ_192KB      (8 << FLASHCALW_FPR_FSZ_SHIFT)  /* 192 Kbytes */
#  define FLASHCALW_FPR_FSZ_256KB      (9 << FLASHCALW_FPR_FSZ_SHIFT)  /* 256 Kbytes */
#  define FLASHCALW_FPR_FSZ_384KB      (10 << FLASHCALW_FPR_FSZ_SHIFT) /* 384 Kbytes */
#  define FLASHCALW_FPR_FSZ_512KB      (11 << FLASHCALW_FPR_FSZ_SHIFT) /* 512 Kbytes */
#  define FLASHCALW_FPR_FSZ_768KB      (12 << FLASHCALW_FPR_FSZ_SHIFT) /* 768 Kbytes */
#  define FLASHCALW_FPR_FSZ_1MB        (13 << FLASHCALW_FPR_FSZ_SHIFT) /* 1024 Kbytes */
#  define FLASHCALW_FPR_FSZ_2MB        (14 << FLASHCALW_FPR_FSZ_SHIFT) /* 2048 Kbytes */
#define FLASHCALW_FPR_PSZ_SHIFT        (8)       /* Bits 8-9: Page Size */
#define FLASHCALW_FPR_PSZ_MASK         (7 << FLASHCALW_FPR_PSZ_SHIFT)
#  define FLASHCALW_FPR_PSZ_32KB       (0 << FLASHCALW_FPR_PSZ_SHIFT)  /* 32 Kbytes */
#  define FLASHCALW_FPR_PSZ_64KB       (1 << FLASHCALW_FPR_PSZ_SHIFT)  /* 64 Kbytes */
#  define FLASHCALW_FPR_PSZ_128KB      (2 << FLASHCALW_FPR_PSZ_SHIFT)  /* 128 Kbytes */
#  define FLASHCALW_FPR_PSZ_256KB      (3 << FLASHCALW_FPR_PSZ_SHIFT)  /* 256 Kbytes */
#  define FLASHCALW_FPR_PSZ_512KGB     (4 << FLASHCALW_FPR_PSZ_SHIFT)  /* 512 Kbytes */
#  define FLASHCALW_FPR_PSZ_1MB        (5 << FLASHCALW_FPR_PSZ_SHIFT)  /* 1024 Kbytes */
#  define FLASHCALW_FPR_PSZ_2MB        (6 << FLASHCALW_FPR_PSZ_SHIFT)  /* 2048 Kbytes */
#  define FLASHCALW_FPR_PSZ_4MB        (7 << FLASHCALW_FPR_PSZ_SHIFT)  /* 4096 Kbytes */

/* Flash Version Register */

#define FLASHCALW_FVR_VERSION_SHIFT   (0)        /* Bits 0-11: Version Number */
#define FLASHCALW_FVR_VERSION_MASK    (0xfff << FLASHCALW_FVR_VERSION_SHIFT)
#define FLASHCALW_FVR_VARIANT_SHIFT   (16)       /* Bits 16-19: Variant Number */
#define FLASHCALW_FVR_VARIANT_MASK    (15 << FLASHCALW_FVR_VARIANT_SHIFT)

/* Flash General Purpose Fuse Register Hi */

#define FLASHCALW_FGPFRHI(n)           (1 << ((n)-32))
#define FLASHCALW_FGPFRHI32            (1 << 0)  /* Bit 0:  General Purpose Fuse 32 */
#define FLASHCALW_FGPFRHI33            (1 << 1)  /* Bit 1:  General Purpose Fuse 33 */
#define FLASHCALW_FGPFRHI34            (1 << 2)  /* Bit 2:  General Purpose Fuse 34 */
#define FLASHCALW_FGPFRHI35            (1 << 3)  /* Bit 3:  General Purpose Fuse 35 */
#define FLASHCALW_FGPFRHI36            (1 << 4)  /* Bit 4:  General Purpose Fuse 36 */
#define FLASHCALW_FGPFRHI37            (1 << 5)  /* Bit 5:  General Purpose Fuse 37 */
#define FLASHCALW_FGPFRHI38            (1 << 6)  /* Bit 6:  General Purpose Fuse 38 */
#define FLASHCALW_FGPFRHI39            (1 << 7)  /* Bit 7:  General Purpose Fuse 39 */
#define FLASHCALW_FGPFRHI40            (1 << 8)  /* Bit 8:  General Purpose Fuse 40 */
#define FLASHCALW_FGPFRHI41            (1 << 9)  /* Bit 9:  General Purpose Fuse 41 */
#define FLASHCALW_FGPFRHI42            (1 << 10) /* Bit 10: General Purpose Fuse 42 */
#define FLASHCALW_FGPFRHI43            (1 << 11) /* Bit 11: General Purpose Fuse 43 */
#define FLASHCALW_FGPFRHI44            (1 << 12) /* Bit 12: General Purpose Fuse 44 */
#define FLASHCALW_FGPFRHI45            (1 << 13) /* Bit 13: General Purpose Fuse 45 */
#define FLASHCALW_FGPFRHI46            (1 << 14) /* Bit 14: General Purpose Fuse 46 */
#define FLASHCALW_FGPFRHI47            (1 << 15) /* Bit 15: General Purpose Fuse 47 */
#define FLASHCALW_FGPFRHI48            (1 << 16) /* Bit 16: General Purpose Fuse 48 */
#define FLASHCALW_FGPFRHI49            (1 << 17) /* Bit 17: General Purpose Fuse 49 */
#define FLASHCALW_FGPFRHI50            (1 << 18) /* Bit 18: General Purpose Fuse 50 */
#define FLASHCALW_FGPFRHI51            (1 << 19) /* Bit 19: General Purpose Fuse 51 */
#define FLASHCALW_FGPFRHI52            (1 << 20) /* Bit 20: General Purpose Fuse 52 */
#define FLASHCALW_FGPFRHI53            (1 << 21) /* Bit 21: General Purpose Fuse 53 */
#define FLASHCALW_FGPFRHI54            (1 << 22) /* Bit 22: General Purpose Fuse 54 */
#define FLASHCALW_FGPFRHI55            (1 << 23) /* Bit 23: General Purpose Fuse 55 */
#define FLASHCALW_FGPFRHI56            (1 << 24) /* Bit 24: General Purpose Fuse 56 */
#define FLASHCALW_FGPFRHI57            (1 << 25) /* Bit 25: General Purpose Fuse 57 */
#define FLASHCALW_FGPFRHI58            (1 << 26) /* Bit 26: General Purpose Fuse 58 */
#define FLASHCALW_FGPFRHI59            (1 << 27) /* Bit 27: General Purpose Fuse 59 */
#define FLASHCALW_FGPFRHI60            (1 << 28) /* Bit 28: General Purpose Fuse 60 */
#define FLASHCALW_FGPFRHI61            (1 << 29) /* Bit 29: General Purpose Fuse 61 */
#define FLASHCALW_FGPFRHI62            (1 << 30) /* Bit 30: General Purpose Fuse 62 */
#define FLASHCALW_FGPFRHI63            (1 << 31) /* Bit 31: General Purpose Fuse 63 */

/* Flash General Purpose Fuse Register Lo */

#define FLASHCALW_FGPFRLO(n)           (1 << (n))
#define FLASHCALW_FGPFRLO00            (1 << 0)  /* Bit 0:  General Purpose Fuse 00 */
#define FLASHCALW_FGPFRLO01            (1 << 1)  /* Bit 1:  General Purpose Fuse 01 */
#define FLASHCALW_FGPFRLO02            (1 << 2)  /* Bit 2:  General Purpose Fuse 02 */
#define FLASHCALW_FGPFRLO03            (1 << 3)  /* Bit 3:  General Purpose Fuse 03 */
#define FLASHCALW_FGPFRLO04            (1 << 4)  /* Bit 4:  General Purpose Fuse 04 */
#define FLASHCALW_FGPFRLO05            (1 << 5)  /* Bit 5:  General Purpose Fuse 05 */
#define FLASHCALW_FGPFRLO06            (1 << 6)  /* Bit 6:  General Purpose Fuse 06 */
#define FLASHCALW_FGPFRLO07            (1 << 7)  /* Bit 7:  General Purpose Fuse 07 */
#define FLASHCALW_FGPFRLO08            (1 << 8)  /* Bit 8:  General Purpose Fuse 08 */
#define FLASHCALW_FGPFRLO09            (1 << 9)  /* Bit 9:  General Purpose Fuse 09 */
#define FLASHCALW_FGPFRLO10            (1 << 10) /* Bit 10: General Purpose Fuse 10 */
#define FLASHCALW_FGPFRLO11            (1 << 11) /* Bit 11: General Purpose Fuse 11 */
#define FLASHCALW_FGPFRLO12            (1 << 12) /* Bit 12: General Purpose Fuse 12 */
#define FLASHCALW_FGPFRLO13            (1 << 13) /* Bit 13: General Purpose Fuse 13 */
#define FLASHCALW_FGPFRLO14            (1 << 14) /* Bit 14: General Purpose Fuse 14 */
#define FLASHCALW_FGPFRLO15            (1 << 15) /* Bit 15: General Purpose Fuse 15 */
#define FLASHCALW_FGPFRLO16            (1 << 16) /* Bit 16: General Purpose Fuse 16 */
#define FLASHCALW_FGPFRLO17            (1 << 17) /* Bit 17: General Purpose Fuse 17 */
#define FLASHCALW_FGPFRLO18            (1 << 18) /* Bit 18: General Purpose Fuse 18 */
#define FLASHCALW_FGPFRLO19            (1 << 19) /* Bit 19: General Purpose Fuse 19 */
#define FLASHCALW_FGPFRLO20            (1 << 20) /* Bit 20: General Purpose Fuse 20 */
#define FLASHCALW_FGPFRLO21            (1 << 21) /* Bit 21: General Purpose Fuse 21 */
#define FLASHCALW_FGPFRLO22            (1 << 22) /* Bit 22: General Purpose Fuse 22 */
#define FLASHCALW_FGPFRLO23            (1 << 23) /* Bit 23: General Purpose Fuse 23 */
#define FLASHCALW_FGPFRLO24            (1 << 24) /* Bit 24: General Purpose Fuse 24 */
#define FLASHCALW_FGPFRLO25            (1 << 25) /* Bit 25: General Purpose Fuse 25 */
#define FLASHCALW_FGPFRLO26            (1 << 26) /* Bit 26: General Purpose Fuse 26 */
#define FLASHCALW_FGPFRLO27            (1 << 27) /* Bit 27: General Purpose Fuse 27 */
#define FLASHCALW_FGPFRLO28            (1 << 28) /* Bit 28: General Purpose Fuse 28 */
#define FLASHCALW_FGPFRLO29            (1 << 29) /* Bit 29: General Purpose Fuse 29 */
#define FLASHCALW_FGPFRLO30            (1 << 30) /* Bit 30: General Purpose Fuse 30 */
#define FLASHCALW_FGPFRLO31            (1 << 31) /* Bit 31: General Purpose Fuse 31 */

/* PicoCache Control Register */

#define PICOCACHE_CTRL_CEN             (1 << 0)  /* Bit 0:  Cache Enable */

/* PicoCache Status Register */

#define PICOCACHE_SR_CSTS              (1 << 0)  /* Bit 0:  Cache Controller Status */

/* PicoCache Maintenance Register 0 */

#define PICOCACHE_MAINT0_INVALL        (1 << 0)  /* Bit 0:  Cache Controller Invalidate All */

/* PicoCache Maintenance Register 1 */

#define PICOCACHE_MAINT1_INDEX_SHIFT   (4)       /* Bits 4-7: Invalidate Index */
#define PICOCACHE_MAINT1_INDEX_MASK    (15 << PICOCACHE_MAINT1_INDEX_SHIFT)

/* PicoCache Monitor Configuration Register */

#define PICOCACHE_MCFG_MODE_SHIFT      (0)      /* Bits 0-1: Cache Controller Monitor Counter Mode */
#define PICOCACHE_MCFG_MODE_MASK       (3 << PICOCACHE_MCFG_MODE_SHIFT)
#  define PICOCACHE_MCFG_MODE_CYCLE    (0 << PICOCACHE_MCFG_MODE_SHIFT) /* CYCLE_COUNT cycle counter */
#  define PICOCACHE_MCFG_MODE_IHIT     (1 << PICOCACHE_MCFG_MODE_SHIFT) /* IHIT_COUNT instruction hit counter */
#  define PICOCACHE_MCFG_MODE_DHIT     (2 << PICOCACHE_MCFG_MODE_SHIFT) /* DHIT_COUNT data hit counter */

/* PicoCache Monitor Enable Register */

#define PICOCACHE_MEN_MENABLE        (1 << 0)  /* Bit 0:  Monitor Enable */

/* PicoCache Monitor Control Register */

#define PICOCACHE_MCTRL_SWRST        (1 << 0)  /* Bit 0:  Monitor Software Reset */

/* PicoCache Monitor Status Register (32-bit event count) */

/* Version Register */

#define PICOCACHE_PVR_VERSION_SHIFT   (0)        /* Bits 0-11: Version Number */
#define PICOCACHE_PVR_VERSION_MASK    (0xfff << PICOCACHE_PVR_FVR_VERSION_SHIFT)
#define PICOCACHE_PVR_MFN_SHIFT       (16)       /* Bits 16-19: MFN */
#define PICOCACHE_PVR_MFN_MASK        (15 << PICOCACHE_PVR_FVR_MFN_SHIFT)

/* Flash Command Set ****************************************************************/

#define FLASH_CMD_NOP                0 /* No operation */
#define FLASH_CMD_WP                 1 /* Write Page */
#define FLASH_CMD_EP                 2 /* Erase Page */
#define FLASH_CMD_CPB                3 /* Clear Page Buffer */
#define FLASH_CMD_LP                 4 /* Lock region containing given Page */
#define FLASH_CMD_UP                 5 /* Unlock region containing given Page */
#define FLASH_CMD_EA                 6 /* Erase All */
#define FLASH_CMD_WGPB               7 /* Write General-Purpose Fuse Bit */
#define FLASH_CMD_EGPB               8 /* Erase General-Purpose Fuse Bit */
#define FLASH_CMD_SSB                9 /* Set Security Fuses */
#define FLASH_CMD_PGPFB             10 /* Program GP Fuse Byte */
#define FLASH_CMD_EAGPF             11 /* Erase All GPFuses */
#define FLASH_CMD_QPR               12 /* Quick Page Read */
#define FLASH_CMD_WUP               13 /* Write User Page */
#define FLASH_CMD_EUP               14 /* Erase User Page */
#define FLASH_CMD_QPRUP             15 /* Quick Page Read User Page */
#define FLASH_CMD_HSEN              16 /* High Speed Mode Enable */
#define FLASH_CMD_HSDIS             17 /* High Speed Mode Disable */

/* Maximum CPU frequency for 0 and 1 FLASH wait states (FWS) in various modes
 * (Table 42-30 in the big data sheet).
 *
 *   ------- ------------------- ---------- ----------
 *   Power     Flash Read Mode     Flash     Maximum
 *   Sclaing                        Wait    Operating
 *   Mode    HSEN HSDIS FASTWKUP   States   Frequency
 *   ------- ---- ----- -------- ---------- ----------
 *     PS0          X       X        1        12MHz
 *     " "          X                0        18MHz
 *     " "          X                1        36MHz
 *     PS1          X       X        1        12MHz
 *     " "          X                0         8MHz
 *     " "          X                1        12MHz
 *     PS2     X                     0        24Mhz
 *     " "     X                     1        48MHz
 *   ------- ---- ----- -------- ---------- ----------
 */

#define FLASH_MAXFREQ_PS0_HSDIS_FASTWKUP_FWS1 (12000000ul)
#define FLASH_MAXFREQ_PS0_HSDIS_FWS0          (18000000ul)
#define FLASH_MAXFREQ_PS0_HSDIS_FWS1          (36000000ul)

#define FLASH_MAXFREQ_PS1_HSDIS_FASTWKUP_FWS1 (12000000ul)
#define FLASH_MAXFREQ_PS1_HSDIS_FWS0          (8000000ul)
#define FLASH_MAXFREQ_PS1_HSDIS_FWS1          (12000000ul)

#define FLASH_MAXFREQ_PS2_HSEN_FWS0           (24000000ul)
#define FLASH_MAXFREQ_PS2_HSEN_FWS1           (48000000ul)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4L_FLASHCALW_H */
