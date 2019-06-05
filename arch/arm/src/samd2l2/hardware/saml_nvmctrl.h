/********************************************************************************************
 * arch/arm/src/samd2l2/hardware/saml_nvmctrl.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   "Atmel SAM L21E / SAM L21G / SAM L21J Smart ARM-Based Microcontroller
 *   Datasheet", Atmel-42385C-SAML21_Datasheet_Preliminary-03/20/15
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_NVMCTRL_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_NVMCTRL_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAML21

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* NVMCTRL register offsets *****************************************************************/

#define SAM_NVMCTRL_CTRLA_OFFSET     0x0000 /* Control A register */
#define SAM_NVMCTRL_CTRLB_OFFSET     0x0004 /* Control B register */
#define SAM_NVMCTRL_PARAM_OFFSET     0x0008 /* NVM parameter register */
#define SAM_NVMCTRL_INTENCLR_OFFSET  0x000c /* Interrupt clear register */
#define SAM_NVMCTRL_INTENSET_OFFSET  0x0010 /* Interrupt set register */
#define SAM_NVMCTRL_INTFLAG_OFFSET   0x0014 /* Interface flags status and clear register */
#define SAM_NVMCTRL_STATUS_OFFSET    0x0018 /* Status register */
#define SAM_NVMCTRL_ADDR_OFFSET      0x001c /* Address register */
#define SAM_NVMCTRL_LOCK_OFFSET      0x0020 /* Lock section register */

/* NVMCTRL register addresses ***************************************************************/

#define SAM_NVMCTRL_CTRLA            (SAM_NVMCTRL_BASE+SAM_NVMCTRL_CTRLA_OFFSET)
#define SAM_NVMCTRL_CTRLB            (SAM_NVMCTRL_BASE+SAM_NVMCTRL_CTRLB_OFFSET)
#define SAM_NVMCTRL_INTENCLR         (SAM_NVMCTRL_BASE+SAM_NVMCTRL_INTENCLR_OFFSET)
#define SAM_NVMCTRL_INTENSET         (SAM_NVMCTRL_BASE+SAM_NVMCTRL_INTENSET_OFFSET)
#define SAM_NVMCTRL_INTFLAG          (SAM_NVMCTRL_BASE+SAM_NVMCTRL_INTFLAG_OFFSET)
#define SAM_NVMCTRL_STATUS           (SAM_NVMCTRL_BASE+SAM_NVMCTRL_STATUS_OFFSET)
#define SAM_NVMCTRL_ADDR             (SAM_NVMCTRL_BASE+SAM_NVMCTRL_ADDR_OFFSET)
#define SAM_NVMCTRL_LOCK             (SAM_NVMCTRL_BASE+SAM_NVMCTRL_LOCK_OFFSET)

/* NVMCTRL register bit definitions *********************************************************/

/* Control A register */

#define NVMCTRL_CTRLA_CMD_SHIFT      (0)       /* Bits 0-6: Command */
#define NVMCTRL_CTRLA_CMD_MASK       (0x7f << NVMCTRL_CTRLA_CMD_SHIFT)
#  define NVMCTRL_CTRLA_CMD_ER       (0x02 << NVMCTRL_CTRLA_CMD_SHIFT) /* Erase Row */
#  define NVMCTRL_CTRLA_CMD_WP       (0x04 << NVMCTRL_CTRLA_CMD_SHIFT) /* Write Page */
#  define NVMCTRL_CTRLA_CMD_EAR      (0x05 << NVMCTRL_CTRLA_CMD_SHIFT) /* Erase Auxiliary Row */
#  define NVMCTRL_CTRLA_CMD_WAP      (0x06 << NVMCTRL_CTRLA_CMD_SHIFT) /* Write Auxiliary Page */
#  define NVMCTRL_CTRLA_CMD_RWWEEER  (0x1a << NVMCTRL_CTRLA_CMD_SHIFT) /* RWWEE Erase Row */
#  define NVMCTRL_CTRLA_CMD_RWWEEWP  (0x1a << NVMCTRL_CTRLA_CMD_SHIFT) /* RWWEE Write page */
#  define NVMCTRL_CTRLA_CMD_LR       (0x40 << NVMCTRL_CTRLA_CMD_SHIFT) /* Lock Region */
#  define NVMCTRL_CTRLA_CMD_UR       (0x41 << NVMCTRL_CTRLA_CMD_SHIFT) /* Unlock Region */
#  define NVMCTRL_CTRLA_CMD_SPRM     (0x42 << NVMCTRL_CTRLA_CMD_SHIFT) /* Set power reduction mode */
#  define NVMCTRL_CTRLA_CMD_CPRM     (0x43 << NVMCTRL_CTRLA_CMD_SHIFT) /* Clear power reduction mode */
#  define NVMCTRL_CTRLA_CMD_PBC      (0x44 << NVMCTRL_CTRLA_CMD_SHIFT) /* Page Buffer Clear */
#  define NVMCTRL_CTRLA_CMD_SSB      (0x45 << NVMCTRL_CTRLA_CMD_SHIFT) /* Set Security Bit */
#  define NVMCTRL_CTRLA_CMD_INVALL   (0x46 << NVMCTRL_CTRLA_CMD_SHIFT) /* Invalidate all cache lines */
#define NVMCTRL_CTRLA_CMDEX_SHIFT    (8)       /* Bits 8-15: Command Execution */
#define NVMCTRL_CTRLA_CMDEX_MASK     (0xff << NVMCTRL_CTRLA_CMDEX_SHIFT)
#  define NVMCTRL_CTRLA_CMDEX        (0xa5 << NVMCTRL_CTRLA_CMDEX_SHIFT)

/* Control B register */

#define NVMCTRL_CTRLB_RWS_SHIFT      (1)       /* Bits 1-4: NVM Read Wait States */
#define NVMCTRL_CTRLB_RWS_MASK       (15 << NVMCTRL_CTRLB_RWS_SHIFT)
#  define NVMCTRL_CTRLB_RWS(n)       ((uint32_t)(n) << NVMCTRL_CTRLB_RWS_SHIFT)
#define NVMCTRL_CTRLB_MANW           (1 << 7)  /* Bit 7: Manual Write */
#define NVMCTRL_CTRLB_SLEEPPRM_SHIFT (8)       /* Bits 8-9: Power Reduction Mode during Sleep */
#define NVMCTRL_CTRLB_SLEEPPRM_MASK  (3 << NVMCTRL_CTRLB_SLEEPPRM_SHIFT)
#  define NVMCTRL_CTRLB_SLEEPPRM_WAKEONACCESS    (0 << NVMCTRL_CTRLB_SLEEPPRM_SHIFT) /* Exit low power on first access */
#  define NVMCTRL_CTRLB_SLEEPPRM_WAKEUPINSTANT   (1 << NVMCTRL_CTRLB_SLEEPPRM_SHIFT) /* Exit low power when exit sleep */
#  define NVMCTRL_CTRLB_SLEEPPRM_DISABLED        (3 << NVMCTRL_CTRLB_SLEEPPRM_SHIFT) /* Auto power reduction disabled */
#define NVMCTRL_CTRLB_READMODE_SHIFT (16)      /* Bits 16-17: NVMCTRL Read Mode */
#define NVMCTRL_CTRLB_READMODE_MASK  (3 << NVMCTRL_CTRLB_READMODE_SHIFT)
#  define NVMCTRL_CTRLB_READMODE_NO_MISS_PENALTY (0 << NVMCTRL_CTRLB_READMODE_SHIFT) /* No extra wait states on miss */
#  define NVMCTRL_CTRLB_READMODE_LOW_POWER       (1 << NVMCTRL_CTRLB_READMODE_SHIFT) /* Insert wait/reduce power */
#  define NVMCTRL_CTRLB_READMODE_DETERMINISTIC   (2 << NVMCTRL_CTRLB_READMODE_SHIFT) /* Same wait on all access */
#define NVMCTRL_CTRLB_CACHEDIS       (1 << 18)  /* Bit 18: Cache Disable */

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
#define NVMCTRL_PARAM_RWWEEP_SHIFT   (20)     /* Bits 20-31: Read while write EEPROM emulation area pages */
#define NVMCTRL_PARAM_RWWEEP_MASK    (0xfff << NVMCTRL_PARAM_RWWEEP_SHIFT)
#  define NVMCTRL_PARAM_RWWEEP(n)    ((uint32_t)(n) << NVMCTRL_PARAM_RWWEEP_SHIFT)

/* Interrupt clear register */
/* Interrupt set register */
/* Interface flags status and clear register */

#define NVMCTRL_INT_READY            (1 << 0)  /* Bit 0: NVM Ready Interrupt */
#define NVMCTRL_INT_ERROR            (1 << 1)  /* Bit 1: Error Interrupt */

/* Status register */

#define NVMCTRL_STATUS_PRM           (1 << 0)  /* Bit 0: Power Reduction Mode */
#define NVMCTRL_STATUS_LOAD          (1 << 1)  /* Bit 1: NVM Page Buffer Active Loading */
#define NVMCTRL_STATUS_PROGE         (1 << 2)  /* Bit 2: Programming Error Status */
#define NVMCTRL_STATUS_LOCKE         (1 << 3)  /* Bit 3: Lock Error Status */
#define NVMCTRL_STATUS_NVME          (1 << 4)  /* Bit 4: NVM Error */
#define NVMCTRL_STATUS_SB            (1 << 8)  /* Bit 8: Security Bit Status */

/* Address register */

#define NVMCTRL_ADDR_MASK            (0x001fffff) /* Bits 0-20: NVM Address */

/* Lock section register */

#define NVMCTRL_LOCK_REGION(n)       (1 << (n)) /* Region n is locked */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAML21 */
#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_NVMCTRL_H */
