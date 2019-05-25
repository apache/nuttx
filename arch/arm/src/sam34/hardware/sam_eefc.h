/****************************************************************************************
 * arch/arm/src/sam34/hardware/sam_eefc.h
 * Enhanced Embedded Flash Controller (EEFC) definitions for the SAM3U, SAM3X, SAM3A,
 * SAM4E, and SAM4S
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

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_EEFC_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_EEFC_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* EEFC register offsets ****************************************************************/

#define SAM_EEFC_FMR_OFFSET          0x00 /* EEFC Flash Mode Register */
#define SAM_EEFC_FCR_OFFSET          0x04 /* EEFC Flash Command Register */
#define SAM_EEFC_FSR_OFFSET          0x08 /* EEFC Flash Status Register */
#define SAM_EEFC_FRR_OFFSET          0x0c /* EEFC Flash Result Register */

/* EEFC register addresses **************************************************************/

#define SAM_EEFC_FMR(n)              (SAM_EEFCN_BASE(n)+SAM_EEFC_FMR_OFFSET)
#define SAM_EEFC_FCR(n)              (SAM_EEFCN_BASE(n)+SAM_EEFC_FCR_OFFSET)
#define SAM_EEFC_FSR(n)              (SAM_EEFCN_BASE(n)+SAM_EEFC_FSR_OFFSET)
#define SAM_EEFC_FRR(n)              (SAM_EEFCN_BASE(n)+SAM_EEFC_FRR_OFFSET)

#define SAM_EEFC0_FMR                (SAM_EEFC0_BASE+SAM_EEFC_FMR_OFFSET)
#define SAM_EEFC0_FCR                (SAM_EEFC0_BASE+SAM_EEFC_FCR_OFFSET)
#define SAM_EEFC0_FSR                (SAM_EEFC0_BASE+SAM_EEFC_FSR_OFFSET)
#define SAM_EEFC0_FRR                (SAM_EEFC0_BASE+SAM_EEFC_FRR_OFFSET)

#if !defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_EEFC1_FMR              (SAM_EEFC1_BASE+SAM_EEFC_FMR_OFFSET)
#  define SAM_EEFC1_FCR              (SAM_EEFC1_BASE+SAM_EEFC_FCR_OFFSET)
#  define SAM_EEFC1_FSR              (SAM_EEFC1_BASE+SAM_EEFC_FSR_OFFSET)
#  define SAM_EEFC1_FRR              (SAM_EEFC1_BASE+SAM_EEFC_FRR_OFFSET)
#endif

/* EEFC register bit definitions ********************************************************/
/* EEFC Flash Mode Register */

#define EEFC_FMR_FRDY                (1 << 0)  /* Bit 0:  Ready Interrupt Enable */
#define EEFC_FMR_FWS_SHIFT           (8)       /* Bits 8-11:  Flash Wait State */
#define EEFC_FMR_FWS_MASK            (15 << EEFC_FMR_FWS_SHIFT)
#  define EEFC_FMR_FWS(n)            ((n) << EEFC_FMR_FWS_SHIFT)

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define EEFC_FMR_SCOD              (1 << 16) /* Bit 16: Sequential Code Optimization Disable */
#endif

#define EEFC_FMR_FAM                 (1 << 24) /* Bit 24: Flash Access Mode */

#if defined(CONFIG_ARCH_CHIP_SAM4S) ||  defined(CONFIG_ARCH_CHIP_SAM4E)
#  define EEFC_FMR_CLOE              (1 << 26) /* Bit 26: Code Loops Optimization Enable */
#endif

/* EEFC Flash Command Register */

#define EEFC_FCR_FCMD_SHIFT          (0)       /* Bits 0-7:  Flash Command */
#define EEFC_FCR_FCMD_MASK           (0xff << EEFC_FCR_FCMD_SHIFT)

#  define EEFC_FCR_FCMD_GETD         (0  << EEFC_FCR_FCMD_SHIFT) /* Get Flash Descriptor */
#  define EEFC_FCR_FCMD_WP           (1  << EEFC_FCR_FCMD_SHIFT) /* Write page */
#  define EEFC_FCR_FCMD_WPL          (2  << EEFC_FCR_FCMD_SHIFT) /* Write page and lock */
#  define EEFC_FCR_FCMD_EWP          (3  << EEFC_FCR_FCMD_SHIFT) /* Erase page and write page */
#  define EEFC_FCR_FCMD_EWPL         (4  << EEFC_FCR_FCMD_SHIFT) /* Erase page and write page then lock */
#  define EEFC_FCR_FCMD_EA           (5  << EEFC_FCR_FCMD_SHIFT) /* Erase all */

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define EEFC_FCR_FCMD_EPA          (7  << EEFC_FCR_FCMD_SHIFT) /* Erase Pages */
#endif

#  define EEFC_FCR_FCMD_SLB          (8  << EEFC_FCR_FCMD_SHIFT) /* Set Lock Bit */
#  define EEFC_FCR_FCMD_CLB          (9  << EEFC_FCR_FCMD_SHIFT) /* Clear Lock Bit */
#  define EEFC_FCR_FCMD_GLB          (10 << EEFC_FCR_FCMD_SHIFT) /* Get Lock Bit */
#  define EEFC_FCR_FCMD_SGPB         (11 << EEFC_FCR_FCMD_SHIFT) /* Set GPNVM Bit */
#  define EEFC_FCR_FCMD_CGPB         (12 << EEFC_FCR_FCMD_SHIFT) /* Clear GPNVM Bit */
#  define EEFC_FCR_FCMD_GGPB         (13 << EEFC_FCR_FCMD_SHIFT) /* Get GPNVM Bit */
#  define EEFC_FCR_FCMD_STUI         (14 << EEFC_FCR_FCMD_SHIFT) /* Start Read Unique Identifier */
#  define EEFC_FCR_FCMD_SPUI         (15 << EEFC_FCR_FCMD_SHIFT) /* Stop Read Unique Identifier */

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A) || \
    defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define EEFC_FCR_FCMD_GCALB        (16 << EEFC_FCR_FCMD_SHIFT) /* Get CALIB Bit */
#endif

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define EEFC_FCR_FCMD_ES           (17 << EEFC_FCR_FCMD_SHIFT) /* Erase Sector */
#  define EEFC_FCR_FCMD_WUS          (18 << EEFC_FCR_FCMD_SHIFT) /* Write User Signature */
#  define EEFC_FCR_FCMD_EUS          (19 << EEFC_FCR_FCMD_SHIFT) /* Erase User Signature */
#  define EEFC_FCR_FCMD_STUS         (20 << EEFC_FCR_FCMD_SHIFT) /* Start Read User Signature */
#  define EEFC_FCR_FCMD_SPUS         (21 << EEFC_FCR_FCMD_SHIFT) /* Stop Read User Signature */
#endif

#define EEFC_FCR_FARG_SHIFT          (8)       /* Bits 8-23:  Flash Command Argument */
#define EEFC_FCR_FARG_MASK           (0xffff << EEFC_FCR_FARG_SHIFT)
#  define EEFC_FCR_FARG(n)           ((uint32_t)(n) << EEFC_FCR_FARG_SHIFT)
#define EEFC_FCR_FKEY_SHIFT          (24)      /* Bits 24-31:  Flash Writing Protection Key */
#define EEFC_FCR_FKEY_MASK           (0xff << EEFC_FCR_FKEY_SHIFT)
#  define EEFC_FCR_FKEY_PASSWD       (0x5a << EEFC_FCR_FKEY_SHIFT)

/* EEFC Flash Status Register */

#define EEFC_FSR_FRDY                (1 << 0)  /* Bit 0:  Flash Ready Status */
#define EEFC_FSR_FCMDE               (1 << 1)  /* Bit 1:  Flash Command Error Status */
#define EEFC_FSR_FLOCKE              (1 << 2)  /* Bit 2:  Flash Lock Error Status */

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define EEFC_FSR_FLERR             (1 << 3)  /* Bit 3:  Flash Error Status */
#endif

/* EEFC Flash Result Register -- 32-bit value */

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_EEFC_H */
