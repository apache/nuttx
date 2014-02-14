/********************************************************************************************
 * arch/arm/src/samd/chip/sam_nvmctrl.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMD_CHIP_SAM_NVMCTRL_H
#define __ARCH_ARM_SRC_SAMD_CHIP_SAM_NVMCTRL_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* NVMCTRL register offsets ********************************************************************/

#define SAM_NVMCTRL_CTRLA_OFFSET     0x0000 /* Control A register */
#define SAM_NVMCTRL_CTRLB_OFFSET     0x0004 /* Control B register */
#define SAM_NVMCTRL_PARAM_OFFSET     0x0008 /* NVM parameter register */
#define SAM_NVMCTRL_INTENCLR_OFFSET  0x000c /* Interrupt clear register */
#define SAM_NVMCTRL_INTENSET_OFFSET  0x0010 /* Interrupt set register */
#define SAM_NVMCTRL_INTFLAG_OFFSET   0x0014 /* Interface flags status and clear register */
#define SAM_NVMCTRL_STATUS_OFFSET    0x0018 /* Status register */
#define SAM_NVMCTRL_ADDR_OFFSET      0x001c /* Address register */
#define SAM_NVMCTRL_LOCK_OFFSET      0x0020 /* Lock section register */

/* NVMCTRL register addresses ******************************************************************/

#define SAM_NVMCTRL_CTRLA            (SAM_NVMCTRL_BASE+SAM_NVMCTRL_CTRLA_OFFSET)
#define SAM_NVMCTRL_CTRLB            (SAM_NVMCTRL_BASE+SAM_NVMCTRL_CTRLB_OFFSET)
#define SAM_NVMCTRL_PARAM            (SAM_NVMCTRL_BASE+SAM_NVMCTRL_PARAM_OFFSET)
#define SAM_NVMCTRL_INTENCLR         (SAM_NVMCTRL_BASE+SAM_NVMCTRL_INTENCLR_OFFSET)
#define SAM_NVMCTRL_INTENSET         (SAM_NVMCTRL_BASE+SAM_NVMCTRL_INTENSET_OFFSET)
#define SAM_NVMCTRL_INTFLAG          (SAM_NVMCTRL_BASE+SAM_NVMCTRL_INTFLAG_OFFSET)
#define SAM_NVMCTRL_STATUS           (SAM_NVMCTRL_BASE+SAM_NVMCTRL_STATUS_OFFSET)
#define SAM_NVMCTRL_ADDR             (SAM_NVMCTRL_BASE+SAM_NVMCTRL_ADDR_OFFSET)
#define SAM_NVMCTRL_LOCK             (SAM_NVMCTRL_BASE+SAM_NVMCTRL_LOCK_OFFSET)

/* NVMCTRL register bit definitions ************************************************************/

/* Control A register */

#define NVMCTRL_CTRLA_CMD_SHIFT      (0)       /* Bits 0-6: Command */
#define NVMCTRL_CTRLA_CMD_MASK       (0x7f << NVMCTRL_CTRLA_CMD_SHIFT)
#  define NVMCTRL_CTRLA_CMD_ER       (0x02 << NVMCTRL_CTRLA_CMD_SHIFT) /* Erase Row */
#  define NVMCTRL_CTRLA_CMD_WP       (0x04 << NVMCTRL_CTRLA_CMD_SHIFT) /* Write Page */
#  define NVMCTRL_CTRLA_CMD_EAR      (0x05 << NVMCTRL_CTRLA_CMD_SHIFT) /* Erase Auxiliary Row */
#  define NVMCTRL_CTRLA_CMD_WAP      (0x06 << NVMCTRL_CTRLA_CMD_SHIFT) /* Write Auxiliary Page */
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
#  define NVMCTRL_CTRLB_RWS(n)       ((n) << NVMCTRL_CTRLB_RWS_SHIFT)
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
#  define NVMCTRL_CTRLB_READMODE_DETERMINISTIC       (2 << NVMCTRL_CTRLB_READMODE_SHIFT) /* Same wait on all access */
#define NVMCTRL_CTRLB_CACHEDIS       (1 << 18)  /* Bit 18: Cache Disable */

/* NVM parameter register */

#define NVMCTRL_PARAM_NVMP_SHIFT     (0)      /* Bits 0-15: NVM Pages */
#define NVMCTRL_PARAM_NVMP_MASK      (0xffff << NVMCTRL_PARAM_NVMP_SHIFT)
#  define NVMCTRL_PARAM_NVMP(n)      ((n) << NVMCTRL_PARAM_NVMP_SHIFT)
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

#define NVMCTRL_ADDR_MASK            (0x003fffff) /* Bits 0-21: NVM Address */

/* Lock section register */

#define NVMCTRL_LOCK_REGION(n)       (1 << (n)) /* Region n is locked */

/* Fuse definitions *************************************************************************/

#define ADC_FUSES_BIASCAL_ADDR       (SAM_AUX1_AREA4 + 4)
#define ADC_FUSES_BIASCAL_SHIFT      (3)        /* ADC Bias Calibration */
#define ADC_FUSES_BIASCAL_MASK       (7 << ADC_FUSES_BIASCAL_SHIFT)
#  define ADC_FUSES_BIASCAL(n)       ((n) << ADC_FUSES_BIASCAL_SHIFT)

#define ADC_FUSES_BIAS_OPA_ADDR      (SAM_AUX1_AREA2 + 4)
#define ADC_FUSES_BIAS_OPA_SHIFT     (19)       /* ADC OPA Bias */
#define ADC_FUSES_BIAS_OPA_MASK      (1 << ADC_FUSES_BIAS_OPA_SHIFT)

#define ADC_FUSES_BOOSTEN_ADDR       (SAM_AUX1_AREA2 + 4)
#define ADC_FUSES_BOOSTEN_SHIFT      (17)       /* ADC Boost Enable */
#define ADC_FUSES_BOOSTEN_MASK       (1 << ADC_FUSES_BOOSTEN_SHIFT)

#define ADC_FUSES_CMPDELAY_ADDR      (SAM_AUX1_AREA2 + 4)
#define ADC_FUSES_CMPDELAY_SHIFT     (16)      /* ADC Comparator Delay */
#define ADC_FUSES_CMPDELAY_MASK      (1 << ADC_FUSES_CMPDELAY_SHIFT)

#define ADC_FUSES_DCFG_ADDR          (SAM_AUX1_AREA2 + 4)
#define ADC_FUSES_DCFG_SHIFT         (16)      /* ADC Device Configuration */
#define ADC_FUSES_DCFG_MASK          (15 << ADC_FUSES_DCFG_SHIFT)
#  define ADC_FUSES_DCFG(n)          ((n) << ADC_FUSES_DCFG_SHIFT)

#define ADC_FUSES_GAINCORR_ADDR      (SAM_AUX1_AREA4 + 0)
#define ADC_FUSES_GAINCORR_SHIFT     (3)       /* ADC Gain Correction */
#define ADC_FUSES_GAINCORR_MASK      (0xfff << ADC_FUSES_GAINCORR_SHIFT)
#  define ADC_FUSES_GAINCORR(n)      ((n) << ADC_FUSES_GAINCORR_SHIFT)

#define ADC_FUSES_LINEARITY_0_ADDR   (SAM_AUX1_AREA4 + 0)
#define ADC_FUSES_LINEARITY_0_SHIFT  (27)      /* ADC Linearity bits 4:0 */
#define ADC_FUSES_LINEARITY_0_MASK   (0x1f << ADC_FUSES_LINEARITY_0_SHIFT)
#  define ADC_FUSES_LINEARITY_0(n)   ((n) << ADC_FUSES_LINEARITY_0_SHIFT)

#define ADC_FUSES_LINEARITY_1_ADDR   (SAM_AUX1_AREA4 + 4)
#define ADC_FUSES_LINEARITY_1_SHIFT  (0)       /* ADC Linearity bits 7:5 */
#define ADC_FUSES_LINEARITY_1_MASK   (7 << ADC_FUSES_LINEARITY_1_SHIFT)
#  define ADC_FUSES_LINEARITY_1(n)   ((n) << ADC_FUSES_LINEARITY_1_SHIFT)

#define ADC_FUSES_OFFSETCORR_ADDR    (SAM_AUX1_AREA4 + 0)
#define ADC_FUSES_OFFSETCORR_SHIFT   (15)      /* ADC Offset Correction */
#define ADC_FUSES_OFFSETCORR_MASK    (0xfff << ADC_FUSES_OFFSETCORR_SHIFT)
#  define ADC_FUSES_OFFSETCORR(n)    ((n) << ADC_FUSES_OFFSETCORR_SHIFT)

#define ADC_FUSES_VCMPULSE_ADDR      (SAM_AUX1_AREA2 + 4)
#define ADC_FUSES_VCMPULSE_SHIFT     (18)      /* ADC VCM Pulse */
#define ADC_FUSES_VCMPULSE_MASK      (1 << ADC_FUSES_VCMPULSE_SHIFT)

#define DSU_FUSES_DCFG0_ADDR         (SAM_AUX1_AREA2 + 0)
#define DSU_FUSES_DCFG0_SHIFT        (0)       /* Device Configuration 0 */
#define DSU_FUSES_DCFG0_MASK         (0xffffffff << DSU_FUSES_DCFG0_SHIFT)
#  define DSU_FUSES_DCFG0(n)         ((n) << DSU_FUSES_DCFG0_SHIFT)

#define DSU_FUSES_DCFG1_ADDR         (SAM_AUX1_AREA2 + 4)
#define DSU_FUSES_DCFG1_SHIFT        (0)       /* Device Configuration 1 */
#define DSU_FUSES_DCFG1_MASK         (0xffffffff << DSU_FUSES_DCFG1_SHIFT)
#  define DSU_FUSES_DCFG1(n)         ((n) << DSU_FUSES_DCFG1_SHIFT)

#define DSU_FUSES_DEV_FAMILY_CFG_0_ADDR  (SAM_AUX1_AREA2 + 0)
#define DSU_FUSES_DEV_FAMILY_CFG_0_SHIFT (5)   /* Device Family Configuration bits 26:0 */
#define DSU_FUSES_DEV_FAMILY_CFG_0_MASK  (0x7ffffff << DSU_FUSES_DEV_FAMILY_CFG_0_SHIFT)
#  define DSU_FUSES_DEV_FAMILY_CFG_0(n)  ((n) << DSU_FUSES_DEV_FAMILY_CFG_0_SHIFT)

#define DSU_FUSES_DEV_FAMILY_CFG_1_ADDR  (SAM_AUX1_AREA2 + 4)
#define DSU_FUSES_DEV_FAMILY_CFG_1_SHIFT (0)   /* Device Family Configuration bits 42:27 */
#define DSU_FUSES_DEV_FAMILY_CFG_1_MASK  (0xffff << DSU_FUSES_DEV_FAMILY_CFG_1_SHIFT)
#  define DSU_FUSES_DEV_FAMILY_CFG_1(n)  ((n) << DSU_FUSES_DEV_FAMILY_CFG_1_SHIFT)

#define DSU_FUSES_DID_DEVSEL_ADDR    (SAM_AUX1_AREA2 + 0)
#define DSU_FUSES_DID_DEVSEL_SHIFT   (0)       /* Device Number */
#define DSU_FUSES_DID_DEVSEL_MASK    (0x1f << DSU_FUSES_DID_DEVSEL_SHIFT)
#  define DSU_FUSES_DID_DEVSEL(n)    ((n) << DSU_FUSES_DID_DEVSEL_SHIFT)

#define DSU_FUSES_RAM_BIAS_ADDR      (SAM_AUX1_AREA2 + 4)
#define DSU_FUSES_RAM_BIAS_SHIFT     (20)      /* RAM Bias */
#define DSU_FUSES_RAM_BIAS_MASK      (3 << DSU_FUSES_RAM_BIAS_SHIFT)
#  define DSU_FUSES_RAM_BIAS(n)      ((n) << DSU_FUSES_RAM_BIAS_SHIFT)

#define DSU_FUSES_RAM_READ_MARGIN_ADDR  (SAM_AUX1_AREA2 + 4)
#define DSU_FUSES_RAM_READ_MARGIN_SHIFT (22)   /* RAM Read Margin */
#define DSU_FUSES_RAM_READ_MARGIN_MASK  (15 << DSU_FUSES_RAM_READ_MARGIN_SHIFT)
#  define DSU_FUSES_RAM_READ_MARGIN(n)  ((n) << DSU_FUSES_RAM_READ_MARGIN_SHIFT)

#define NVMCTRL_FUSES_BOOTPROT_ADDR  (SAM_AUX0_BASE + 0)
#define NVMCTRL_FUSES_BOOTPROT_SHIFT (0)       /* Bootloader Size */
#define NVMCTRL_FUSES_BOOTPROT_MASK  (7 << NVMCTRL_FUSES_BOOTPROT_SHIFT)
#  define NVMCTRL_FUSES_BOOTPROT(n)  ((n) << NVMCTRL_FUSES_BOOTPROT_SHIFT)

#define NVMCTRL_FUSES_EEPROM_SIZE_ADDR  (SAM_AUX0_BASE + 0)
#define NVMCTRL_FUSES_EEPROM_SIZE_SHIFT (4)    /* EEPROM Size */
#define NVMCTRL_FUSES_EEPROM_SIZE_MASK  (7 << NVMCTRL_FUSES_EEPROM_SIZE_SHIFT)
#  define NVMCTRL_FUSES_EEPROM_SIZE(n)  ((n) << NVMCTRL_FUSES_EEPROM_SIZE_SHIFT)

#define NVMCTRL_FUSES_LOCKFIELD_ADDR  (SAM_LOCKBIT_BASE + 0)
#define NVMCTRL_FUSES_LOCKFIELD_SHIFT (0)      /* LOCK Region */
#define NVMCTRL_FUSES_LOCKFIELD_MASK  (0xff << NVMCTRL_FUSES_LOCKFIELD_SHIFT)
#  define NVMCTRL_FUSES_LOCKFIELD(n)  ((n) << NVMCTRL_FUSES_LOCKFIELD_SHIFT)

#define NVMCTRL_FUSES_NVMP_ADDR      (SAM_AUX1_AREA1 + 0)
#define NVMCTRL_FUSES_NVMP_SHIFT     (16       /* Number of NVM Pages */
#define NVMCTRL_FUSES_NVMP_MASK      (0xffff << NVMCTRL_FUSES_NVMP_SHIFT)
#  define NVMCTRL_FUSES_NVMP(n)      ((n) << NVMCTRL_FUSES_NVMP_SHIFT)

#define NVMCTRL_FUSES_NVM_LOCK_ADDR  (SAM_AUX1_AREA1 + 0)
#define NVMCTRL_FUSES_NVM_LOCK_SHIFT (0)       /* NVM Lock */
#define NVMCTRL_FUSES_NVM_LOCK_MASK  (0xff << NVMCTRL_FUSES_NVM_LOCK_SHIFT)
#  define NVMCTRL_FUSES_NVM_LOCK(n)  ((n) << NVMCTRL_FUSES_NVM_LOCK_SHIFT)

#define NVMCTRL_FUSES_PSZ_ADDR       (SAM_AUX1_AREA1 + 0)
#define NVMCTRL_FUSES_PSZ_SHIFT      (8)       /* NVM Page Size */
#define NVMCTRL_FUSES_PSZ_MASK       (15 << NVMCTRL_FUSES_PSZ_SHIFT)
#  define NVMCTRL_FUSES_PSZ(n)       ((n) << NVMCTRL_FUSES_PSZ_SHIFT)

#define NVMCTRL_FUSES_REGION_LOCKS_ADDR  (SAM_AUX0_BASE + 4)
#define NVMCTRL_FUSES_REGION_LOCKS_SHIFT (16)  /* NVM Region Locks */
#define NVMCTRL_FUSES_REGION_LOCKS_MASK  (0xffff << NVMCTRL_FUSES_REGION_LOCKS_SHIFT)
#  define NVMCTRL_FUSES_REGION_LOCKS(n)  ((n) << NVMCTRL_FUSES_REGION_LOCKS_SHIFT)

#define SYSCTRL_FUSES_OSC32KCAL_ADDR  (SAM_AUX1_AREA4 + 4)
#define SYSCTRL_FUSES_OSC32KCAL_SHIFT (6)      /* OSC32K Calibration */
#define SYSCTRL_FUSES_OSC32KCAL_MASK  (0x7f << SYSCTRL_FUSES_OSC32KCAL_SHIFT)
#  define SYSCTRL_FUSES_OSC32KCAL(n)  ((n) << SYSCTRL_FUSES_OSC32KCAL_SHIFT)

#define SYSCTRL_FUSES_BOD12USERLEVEL_ADDR  (SAM_AUX0_BASE + 0)
#define SYSCTRL_FUSES_BOD12USERLEVEL_SHIFT (17) /* BOD12 User Level */
#define SYSCTRL_FUSES_BOD12USERLEVEL_MASK  (0x1f << SYSCTRL_FUSES_BOD12USERLEVEL_SHIFT)
#  define SYSCTRL_FUSES_BOD12USERLEVEL(n)  ((n) << SYSCTRL_FUSES_BOD12USERLEVEL_SHIFT)

#define SYSCTRL_FUSES_BOD12_ACTION_ADDR  (SAM_AUX0_BASE + 0)
#define SYSCTRL_FUSES_BOD12_ACTION_SHIFT (23)  /* BOD12 Action */
#define SYSCTRL_FUSES_BOD12_ACTION_MASK  (3 << SYSCTRL_FUSES_BOD12_ACTION_SHIFT)
#  define SYSCTRL_FUSES_BOD12_ACTION(n)  ((n) << SYSCTRL_FUSES_BOD12_ACTION_SHIFT)

#define SYSCTRL_FUSES_BOD12_EN_ADDR  (SAM_AUX0_BASE + 0)
#define SYSCTRL_FUSES_BOD12_EN_SHIFT (22)      /* BOD12 Enable */
#define SYSCTRL_FUSES_BOD12_EN_MASK  (1 << SYSCTRL_FUSES_BOD12_EN_SHIFT)

#define SYSCTRL_FUSES_BOD33USERLEVEL_ADDR  (SAM_AUX0_BASE + 8)
#define SYSCTRL_FUSES_BOD33USERLEVEL_SHIFT (8) /* BOD33 User Level */
#define SYSCTRL_FUSES_BOD33USERLEVEL_MASK  (0x3f << SYSCTRL_FUSES_BOD33USERLEVEL_SHIFT)
#  define SYSCTRL_FUSES_BOD33USERLEVEL(n)  ((n) << SYSCTRL_FUSES_BOD33USERLEVEL_SHIFT)

#define SYSCTRL_FUSES_BOD33_ACTION_ADDR  (SAM_AUX0_BASE + 0)
#define SYSCTRL_FUSES_BOD33_ACTION_SHIFT (15)    /* BOD33 Action */
#define SYSCTRL_FUSES_BOD33_ACTION_MASK  (3 << SYSCTRL_FUSES_BOD33_ACTION_SHIFT)
#  define SYSCTRL_FUSES_BOD33_ACTION(n)  ((n) << SYSCTRL_FUSES_BOD33_ACTION_SHIFT)

#define SYSCTRL_FUSES_BOD33_EN_ADDR  (SAM_AUX0_BASE + 0)
#define SYSCTRL_FUSES_BOD33_EN_SHIFT (14)      /* BOD33 Enable */
#define SYSCTRL_FUSES_BOD33_EN_MASK  (1 << SYSCTRL_FUSES_BOD33_EN_SHIFT)

#define SYSCTRL_FUSES_ULPVREG_ADDR   (SAM_AUX1_AREA4 + 0)
#define SYSCTRL_FUSES_ULPVREG_SHIFT  (0)       /* ULP Regulator Fallback Mode */
#define SYSCTRL_FUSES_ULPVREG_MASK   (7 << SYSCTRL_FUSES_ULPVREG_SHIFT)
#  define SYSCTRL_FUSES_ULPVREG(n)   ((n) << SYSCTRL_FUSES_ULPVREG_SHIFT)

#define WDT_FUSES_ALWAYSON_ADDR      (SAM_AUX0_BASE
#define WDT_FUSES_ALWAYSON_SHIFT     (26)      /* WDT Always On */
#define WDT_FUSES_ALWAYSON_MASK      (1 << WDT_FUSES_ALWAYSON_SHIFT)

#define WDT_FUSES_ENABLE_ADDR        (SAM_AUX0_BASE + 0)
#define WDT_FUSES_ENABLE_SHIFT       (25)      /* WDT Enable */
#define WDT_FUSES_ENABLE_MASK        (1 << WDT_FUSES_ENABLE_SHIFT)

#define WDT_FUSES_EWOFFSET_ADDR      (SAM_AUX0_BASE + 4)
#define WDT_FUSES_EWOFFSET_SHIFT     (3)       /* WDT Early Warning Offset */
#define WDT_FUSES_EWOFFSET_MASK      (15 << WDT_FUSES_EWOFFSET_SHIFT)
#  define WDT_FUSES_EWOFFSET(n)      ((n) << WDT_FUSES_EWOFFSET_SHIFT)

#define WDT_FUSES_PER_ADDR           (SAM_AUX0_BASE + 0)
#define WDT_FUSES_PER_SHIFT          (27)      /* WDT Period */
#define WDT_FUSES_PER_MASK           (15 << WDT_FUSES_PER_SHIFT)
#  define WDT_FUSES_PER(n)           ((n) << WDT_FUSES_PER_SHIFT)

#define WDT_FUSES_WEN_ADDR           (SAM_AUX0_BASE + 4)
#define WDT_FUSES_WEN_SHIFT          (7)       /* WDT Window Mode Enable */
#define WDT_FUSES_WEN_MASK           (1 << WDT_FUSES_WEN_SHIFT)

#define WDT_FUSES_WINDOW_0_ADDR      (SAM_AUX0_BASE + 0)
#define WDT_FUSES_WINDOW_0_SHIFT     (31)      /* WDT Window bit 0 */
#define WDT_FUSES_WINDOW_0_MASK      (1 << WDT_FUSES_WINDOW_0_SHIFT)

#define WDT_FUSES_WINDOW_1_ADDR      (SAM_AUX0_BASE + 4)
#define WDT_FUSES_WINDOW_1_SHIFT     (0)       /* WDT Window bits 3:1 */
#define WDT_FUSES_WINDOW_1_MASK      (7 << WDT_FUSES_WINDOW_1_SHIFT)
#  define WDT_FUSES_WINDOW_1(n)      ((n) << WDT_FUSES_WINDOW_1_SHIFT)

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD_CHIP_SAM_NVMCTRL_H */
