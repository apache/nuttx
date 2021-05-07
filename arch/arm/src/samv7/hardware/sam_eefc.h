/****************************************************************************
 * arch/arm/src/samv7/hardware/sam_eefc.h
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

#ifndef __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_EEFC_H
#define __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_EEFC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* EEFC register offsets ****************************************************/

#define SAM_EEFC_FMR_OFFSET          0x00 /* EEFC Flash Mode Register */
#define SAM_EEFC_FCR_OFFSET          0x04 /* EEFC Flash Command Register */
#define SAM_EEFC_FSR_OFFSET          0x08 /* EEFC Flash Status Register */
#define SAM_EEFC_FRR_OFFSET          0x0c /* EEFC Flash Result Register */
#define SAM_EEFC_WPMR_OFFSET         0xec /* EEFC Write Protection Mode Register */

/* EEFC register addresses **************************************************/

#define SAM_EEFC_FMR                (SAM_EEFC_BASE+SAM_EEFC_FMR_OFFSET)
#define SAM_EEFC_FCR                (SAM_EEFC_BASE+SAM_EEFC_FCR_OFFSET)
#define SAM_EEFC_FSR                (SAM_EEFC_BASE+SAM_EEFC_FSR_OFFSET)
#define SAM_EEFC_FRR                (SAM_EEFC_BASE+SAM_EEFC_FRR_OFFSET)
#define SAM_EEFC_WPMR               (SAM_EEFC_BASE+SAM_EEFC_WPMR_OFFSET)

/* EEFC register bit definitions ********************************************/

/* EEFC Flash Mode Register */

#define EEFC_FMR_FRDY                (1 << 0)  /* Bit 0:  Ready Interrupt Enable */
#define EEFC_FMR_FWS_SHIFT           (8)       /* Bits 8-11:  Flash Wait State */
#define EEFC_FMR_FWS_MASK            (15 << EEFC_FMR_FWS_SHIFT)
#  define EEFC_FMR_FWS(n)            ((n) << EEFC_FMR_FWS_SHIFT)
#define EEFC_FMR_SCOD                (1 << 16) /* Bit 16: Sequential Code Optimization Disable */
#define EEFC_FMR_CLOE                (1 << 26) /* Bit 26: Code Loops Optimization Enable */

/* EEFC Flash Command Register */

#define FCMD_GETD                    (0)       /* Get Flash Descriptor */
#define FCMD_WP                      (1)       /* Write page */
#define FCMD_WPL                     (2)       /* Write page and lock */
#define FCMD_EWP                     (3)       /* Erase page and write page */
#define FCMD_EWPL                    (4)       /* Erase page and write page then lock */
#define FCMD_EA                      (5)       /* Erase all */
#define FCMD_EPA                     (7)       /* Erase pages */
#define FCMD_SLB                     (8)       /* Set Lock Bit */
#define FCMD_CLB                     (9)       /* Clear Lock Bit */
#define FCMD_GLB                     (10)      /* Get Lock Bit */
#define FCMD_SGPB                    (11)      /* Set GPNVM Bit */
#define FCMD_CGPB                    (12)      /* Clear GPNVM Bit */
#define FCMD_GGPB                    (13)      /* Get GPNVM Bit */
#define FCMD_STUI                    (14)      /* Start Read Unique Identifier */
#define FCMD_SPUI                    (15)      /* Stop Read Unique Identifier */
#define FCMD_GCALB                   (16)      /* Get CALIB Bit */
#define FCMD_ES                      (17)      /* Erase Sector */
#define FCMD_WUS                     (18)      /* Write User Signature */
#define FCMD_EUS                     (19)      /* Erase User Signature */
#define FCMD_STUS                    (20)      /* Start Read User Signature */
#define FCMD_SPUS                    (21)      /* Stop Read User Signature */

#define EEFC_FCR_FCMD_SHIFT          (0)       /* Bits 0-7:  Flash Command */
#define EEFC_FCR_FCMD_MASK           (0xff << EEFC_FCR_FCMD_SHIFT)
#  define EEFC_FCR_FCMD(cmd)         ((uint32_t)(cmd) << EEFC_FCR_FCMD_SHIFT)
#  define EEFC_FCR_FCMD_GETD         (0  << EEFC_FCR_FCMD_SHIFT) /* Get Flash Descriptor */
#  define EEFC_FCR_FCMD_WP           (1  << EEFC_FCR_FCMD_SHIFT) /* Write page */
#  define EEFC_FCR_FCMD_WPL          (2  << EEFC_FCR_FCMD_SHIFT) /* Write page and lock */
#  define EEFC_FCR_FCMD_EWP          (3  << EEFC_FCR_FCMD_SHIFT) /* Erase page and write page */
#  define EEFC_FCR_FCMD_EWPL         (4  << EEFC_FCR_FCMD_SHIFT) /* Erase page and write page then lock */
#  define EEFC_FCR_FCMD_EA           (5  << EEFC_FCR_FCMD_SHIFT) /* Erase all */
#  define EEFC_FCR_FCMD_EPA          (7  << EEFC_FCR_FCMD_SHIFT) /* Erase pages */
#  define EEFC_FCR_FCMD_SLB          (8  << EEFC_FCR_FCMD_SHIFT) /* Set Lock Bit */
#  define EEFC_FCR_FCMD_CLB          (9  << EEFC_FCR_FCMD_SHIFT) /* Clear Lock Bit */
#  define EEFC_FCR_FCMD_GLB          (10 << EEFC_FCR_FCMD_SHIFT) /* Get Lock Bit */
#  define EEFC_FCR_FCMD_SGPB         (11 << EEFC_FCR_FCMD_SHIFT) /* Set GPNVM Bit */
#  define EEFC_FCR_FCMD_CGPB         (12 << EEFC_FCR_FCMD_SHIFT) /* Clear GPNVM Bit */
#  define EEFC_FCR_FCMD_GGPB         (13 << EEFC_FCR_FCMD_SHIFT) /* Get GPNVM Bit */
#  define EEFC_FCR_FCMD_STUI         (14 << EEFC_FCR_FCMD_SHIFT) /* Start Read Unique Identifier */
#  define EEFC_FCR_FCMD_SPUI         (15 << EEFC_FCR_FCMD_SHIFT) /* Stop Read Unique Identifier */
#  define EEFC_FCR_FCMD_GCALB        (16 << EEFC_FCR_FCMD_SHIFT) /* Get CALIB Bit */
#  define EEFC_FCR_FCMD_ES           (17 << EEFC_FCR_FCMD_SHIFT) /* Erase Sector */
#  define EEFC_FCR_FCMD_WUS          (18 << EEFC_FCR_FCMD_SHIFT) /* Write User Signature */
#  define EEFC_FCR_FCMD_EUS          (19 << EEFC_FCR_FCMD_SHIFT) /* Erase User Signature */
#  define EEFC_FCR_FCMD_STUS         (20 << EEFC_FCR_FCMD_SHIFT) /* Start Read User Signature */
#  define EEFC_FCR_FCMD_SPUS         (21 << EEFC_FCR_FCMD_SHIFT) /* Stop Read User Signature */

#define EEFC_FCR_FARG_SHIFT          (8)       /* Bits 8-23:  Flash Command Argument */
#define EEFC_FCR_FARG_MASK           (0xffff << EEFC_FCR_FARG_SHIFT)
#  define EEFC_FCR_FARG(arg)         ((uint32_t)(arg) << EEFC_FCR_FARG_SHIFT)
#define EEFC_FCR_FKEY_SHIFT          (24)      /* Bits 24-31:  Flash Writing Protection Key */
#define EEFC_FCR_FKEY_MASK           (0xff << EEFC_FCR_FKEY_SHIFT)
#  define EEFC_FCR_FKEY_PASSWD       (0x5a << EEFC_FCR_FKEY_SHIFT)

/* EEFC Flash Status Register */

#define EEFC_FSR_FRDY                (1 << 0)  /* Bit 0:  Flash Ready Status */
#define EEFC_FSR_FCMDE               (1 << 1)  /* Bit 1:  Flash Command Error Status */
#define EEFC_FSR_FLOCKE              (1 << 2)  /* Bit 2:  Flash Lock Error Status */
#define EEFC_FSR_FLERR               (1 << 3)  /* Bit 3:  Flash Error Status */
#define EEFC_FSR_UECCELSB            (1 << 16) /* Bit 16: Unique ECC Error on LSB Part */
#define EEFC_FSR_MECCELSB            (1 << 17) /* Bit 17: Multiple ECC Error on LSB Part */
#define EEFC_FSR_UECCEMSB            (1 << 18) /* Bit 18: Unique ECC Error on MSB Part */
#define EEFC_FSR_MECCEMSB            (1 << 19) /* Bit 19: Multiple ECC Error on MSB Part */

/* EEFC Flash Result Register -- 32-bit value */

/* EEFC Write Protect Mode Register */

#define EEFC_WPMR_WPEN               (1 << 0)  /* Bit 0:  Write Protect Enable */
#define EEFC_WPMR_WPKEY_SHIFT        (8)       /* Bits 8-31: Write Protect KEY */
#define EEFC_WPMR_WPKEY_MASK         (0x00ffffff << EEFC_WPMR_WPKEY_SHIFT)
#  define EEFC_WPMR_WPKEY            (0x00454643 << EEFC_WPMR_WPKEY_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_EEFC_H */
