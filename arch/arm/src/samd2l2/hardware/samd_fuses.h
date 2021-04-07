/****************************************************************************
 * arch/arm/src/samd2l2/hardware/samd_fuses.h
 *
 *   Copyright (C) 2014-2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Some fuse-related definitions derive from Atmel sample code:
 *
 *   Copyright (c) 2013 Atmel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with
 *    an Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* References:
 *   "Atmel SAM D20J / SAM D20G / SAM D20E ARM-Based Microcontroller
 *   Datasheet", 42129J–SAM–12/2013
 *   "Atmel SAM D21E / SAM D21G / SAM D21J SMART ARM-Based Microcontroller
 *   Datasheet", Atmel-42181E–SAM-D21_Datasheet–02/2015
 */

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_FUSES_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_FUSES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#if defined(CONFIG_ARCH_FAMILY_SAMD20) || defined(CONFIG_ARCH_FAMILY_SAMD21)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Fuse definitions *********************************************************/

#ifdef CONFIG_ARCH_FAMILY_SAMD20
#  define NVMCTRL_FUSES_LOCKFIELD_ADDR       (SAM_LOCKBIT_BASE + 0)
#  define NVMCTRL_FUSES_LOCKFIELD_SHIFT      (0)      /* LOCK Region */
#  define NVMCTRL_FUSES_LOCKFIELD_MASK       (0xff << NVMCTRL_FUSES_LOCKFIELD_SHIFT)
#    define NVMCTRL_FUSES_LOCKFIELD(n)       ((n) << NVMCTRL_FUSES_LOCKFIELD_SHIFT)
#endif

#define NVMCTRL_FUSES_BOOTPROT_ADDR          (SAM_AUX0_BASE + 0)
#define NVMCTRL_FUSES_BOOTPROT_SHIFT         (0)     /* Bits 0-2: Bootloader Size */
#define NVMCTRL_FUSES_BOOTPROT_MASK          (7 << NVMCTRL_FUSES_BOOTPROT_SHIFT)
#  define NVMCTRL_FUSES_BOOTPROT(n)          ((n) << NVMCTRL_FUSES_BOOTPROT_SHIFT)

#define NVMCTRL_FUSES_EEPROM_SIZE_ADDR       (SAM_AUX0_BASE + 0)
#define NVMCTRL_FUSES_EEPROM_SIZE_SHIFT      (4)     /* Bits 4-6: EEPROM Size */
#define NVMCTRL_FUSES_EEPROM_SIZE_MASK       (7 << NVMCTRL_FUSES_EEPROM_SIZE_SHIFT)
#  define NVMCTRL_FUSES_EEPROM_SIZE(n)       ((n) << NVMCTRL_FUSES_EEPROM_SIZE_SHIFT)

#define SYSCTRL_FUSES_BOD33USERLEVEL_ADDR    (SAM_AUX0_BASE + 8)
#define SYSCTRL_FUSES_BOD33USERLEVEL_SHIFT   (8)     /* Bits 8-13: BOD33 User Level */
#define SYSCTRL_FUSES_BOD33USERLEVEL_MASK    (0x3f << SYSCTRL_FUSES_BOD33USERLEVEL_SHIFT)
#  define SYSCTRL_FUSES_BOD33USERLEVEL(n)    ((n) << SYSCTRL_FUSES_BOD33USERLEVEL_SHIFT)

#define SYSCTRL_FUSES_BOD33_EN_ADDR          (SAM_AUX0_BASE + 0)
#define SYSCTRL_FUSES_BOD33_EN_SHIFT         (14)    /* Bit 14: BOD33 Enable */
#define SYSCTRL_FUSES_BOD33_EN_MASK          (1 << SYSCTRL_FUSES_BOD33_EN_SHIFT)

#define SYSCTRL_FUSES_BOD33_ACTION_ADDR      (SAM_AUX0_BASE + 0)
#define SYSCTRL_FUSES_BOD33_ACTION_SHIFT     (15)    /* Bits 15-16: BOD33 Action */
#define SYSCTRL_FUSES_BOD33_ACTION_MASK      (3 << SYSCTRL_FUSES_BOD33_ACTION_SHIFT)
#  define SYSCTRL_FUSES_BOD33_ACTION(n)      ((n) << SYSCTRL_FUSES_BOD33_ACTION_SHIFT)

#ifdef CONFIG_ARCH_FAMILY_SAMD20
#  define SYSCTRL_FUSES_BOD12USERLEVEL_ADDR  (SAM_AUX0_BASE + 0)
#  define SYSCTRL_FUSES_BOD12USERLEVEL_SHIFT (17)    /* Bit 17: BOD12 User Level */
#  define SYSCTRL_FUSES_BOD12USERLEVEL_MASK  (0x1f << SYSCTRL_FUSES_BOD12USERLEVEL_SHIFT)
#    define SYSCTRL_FUSES_BOD12USERLEVEL(n)  ((n) << SYSCTRL_FUSES_BOD12USERLEVEL_SHIFT)
#  define SYSCTRL_FUSES_BOD12_ACTION_ADDR    (SAM_AUX0_BASE + 0)
#  define SYSCTRL_FUSES_BOD12_EN_ADDR        (SAM_AUX0_BASE + 0)
#  define SYSCTRL_FUSES_BOD12_EN_SHIFT       (22)    /* Bit 22: BOD12 Enable */
#  define SYSCTRL_FUSES_BOD12_EN_MASK        (1 << SYSCTRL_FUSES_BOD12_EN_SHIFT)
#  define SYSCTRL_FUSES_BOD12_ACTION_SHIFT   (23)    /* Bits 23-24: BOD12 Action */
#  define SYSCTRL_FUSES_BOD12_ACTION_MASK    (3 << SYSCTRL_FUSES_BOD12_ACTION_SHIFT)
#    define SYSCTRL_FUSES_BOD12_ACTION(n)    ((n) << SYSCTRL_FUSES_BOD12_ACTION_SHIFT)
#endif

#define WDT_FUSES_ENABLE_ADDR                (SAM_AUX0_BASE + 0)
#define WDT_FUSES_ENABLE_SHIFT               (25)    /* Bit 25: WDT Enable */
#define WDT_FUSES_ENABLE_MASK                (1 << WDT_FUSES_ENABLE_SHIFT)

#define WDT_FUSES_ALWAYSON_ADDR              (SAM_AUX0_BASE + 0)
#define WDT_FUSES_ALWAYSON_SHIFT             (26)    /* Bit 26: WDT Always On */
#define WDT_FUSES_ALWAYSON_MASK              (1 << WDT_FUSES_ALWAYSON_SHIFT)

#define WDT_FUSES_PER_ADDR                   (SAM_AUX0_BASE + 0)
#define WDT_FUSES_PER_SHIFT                  (27)    /* Bits 27-30: WDT Period */
#define WDT_FUSES_PER_MASK                   (15 << WDT_FUSES_PER_SHIFT)
#  define WDT_FUSES_PER(n)                   ((n) << WDT_FUSES_PER_SHIFT)

#define WDT_FUSES_WINDOW_0_ADDR              (SAM_AUX0_BASE + 0)
#define WDT_FUSES_WINDOW_0_SHIFT             (31)    /* Bit 31: WDT Window bit 0 */
#define WDT_FUSES_WINDOW_0_MASK              (1 << WDT_FUSES_WINDOW_0_SHIFT)

#define WDT_FUSES_WINDOW_1_ADDR              (SAM_AUX0_BASE + 4)
#define WDT_FUSES_WINDOW_1_SHIFT             (0)     /* Bits 32-34: WDT Window bits 3:1 */
#define WDT_FUSES_WINDOW_1_MASK              (7 << WDT_FUSES_WINDOW_1_SHIFT)
#  define WDT_FUSES_WINDOW_1(n)              ((n) << WDT_FUSES_WINDOW_1_SHIFT)

#define WDT_FUSES_EWOFFSET_ADDR              (SAM_AUX0_BASE + 4)
#define WDT_FUSES_EWOFFSET_SHIFT             (3)     /* Bits 35-38: WDT Early Warning Offset */
#define WDT_FUSES_EWOFFSET_MASK              (15 << WDT_FUSES_EWOFFSET_SHIFT)
#  define WDT_FUSES_EWOFFSET(n)              ((n) << WDT_FUSES_EWOFFSET_SHIFT)

#define WDT_FUSES_WEN_ADDR                   (SAM_AUX0_BASE + 4)
#define WDT_FUSES_WEN_SHIFT                  (7)     /* Bit 39: WDT Window Mode Enable */
#define WDT_FUSES_WEN_MASK                   (1 << WDT_FUSES_WEN_SHIFT)

#define NVMCTRL_FUSES_REGION_LOCKS_ADDR      (SAM_AUX0_BASE + 4)
#define NVMCTRL_FUSES_REGION_LOCKS_SHIFT     (16)    /* Bits 48-63: NVM Region Locks */
#define NVMCTRL_FUSES_REGION_LOCKS_MASK      (0xffff << NVMCTRL_FUSES_REGION_LOCKS_SHIFT)
#  define NVMCTRL_FUSES_REGION_LOCKS(n)      ((n) << NVMCTRL_FUSES_REGION_LOCKS_SHIFT)

#ifdef CONFIG_ARCH_FAMILY_SAMD20
#  define NVMCTRL_FUSES_NVM_LOCK_ADDR        (SAM_AUX1_AREA1 + 0)
#  define NVMCTRL_FUSES_NVM_LOCK_SHIFT       (0)     /* Bits 0-7: NVM Lock */
#  define NVMCTRL_FUSES_NVM_LOCK_MASK        (0xff << NVMCTRL_FUSES_NVM_LOCK_SHIFT)
#  define NVMCTRL_FUSES_NVM_LOCK(n)          ((n) << NVMCTRL_FUSES_NVM_LOCK_SHIFT)

#  define NVMCTRL_FUSES_PSZ_ADDR             (SAM_AUX1_AREA1 + 0)
#  define NVMCTRL_FUSES_PSZ_SHIFT            (8)     /* Bits 8-11: NVM Page Size */
#  define NVMCTRL_FUSES_PSZ_MASK             (15 << NVMCTRL_FUSES_PSZ_SHIFT)
#    define NVMCTRL_FUSES_PSZ(n)             ((n) << NVMCTRL_FUSES_PSZ_SHIFT)

#  define NVMCTRL_FUSES_NVMP_ADDR            (SAM_AUX1_AREA1 + 0)
#  define NVMCTRL_FUSES_NVMP_SHIFT           (16)    /* Bits 16-31: Number of NVM Pages */
#  define NVMCTRL_FUSES_NVMP_MASK            (0xffff << NVMCTRL_FUSES_NVMP_SHIFT)
#    define NVMCTRL_FUSES_NVMP(n)            ((n) << NVMCTRL_FUSES_NVMP_SHIFT)
#endif

#ifdef CONFIG_ARCH_FAMILY_SAMD20
#  define DSU_FUSES_DCFG0_ADDR               (SAM_AUX1_AREA2 + 0)
#  define DSU_FUSES_DCFG0_SHIFT              (0)     /* Bits 0-31: Device Configuration 0 */
#  define DSU_FUSES_DCFG0_MASK               (0xffffffff << DSU_FUSES_DCFG0_SHIFT)
#    define DSU_FUSES_DCFG0(n)               ((n) << DSU_FUSES_DCFG0_SHIFT)

#  define DSU_FUSES_DID_DEVSEL_ADDR          (SAM_AUX1_AREA2 + 0)
#  define DSU_FUSES_DID_DEVSEL_SHIFT         (0)     /* Bits 0-4: Device Number */
#  define DSU_FUSES_DID_DEVSEL_MASK          (0x1f << DSU_FUSES_DID_DEVSEL_SHIFT)
#    define DSU_FUSES_DID_DEVSEL(n)          ((n) << DSU_FUSES_DID_DEVSEL_SHIFT)

#  define DSU_FUSES_DEV_FAMILY_CFG_0_ADDR    (SAM_AUX1_AREA2 + 0)
#  define DSU_FUSES_DEV_FAMILY_CFG_0_SHIFT   (5)     /* Bits 5-31: Device Family Configuration bits 26:0 */
#  define DSU_FUSES_DEV_FAMILY_CFG_0_MASK    (0x7ffffff << DSU_FUSES_DEV_FAMILY_CFG_0_SHIFT)
#    define DSU_FUSES_DEV_FAMILY_CFG_0(n)    ((n) << DSU_FUSES_DEV_FAMILY_CFG_0_SHIFT)

#  define DSU_FUSES_DCFG1_ADDR               (SAM_AUX1_AREA2 + 4)
#  define DSU_FUSES_DCFG1_SHIFT              (0)     /* Bits 0-31: Device Configuration 1 */
#  define DSU_FUSES_DCFG1_MASK               (0xffffffff << DSU_FUSES_DCFG1_SHIFT)
#    define DSU_FUSES_DCFG1(n)               ((n) << DSU_FUSES_DCFG1_SHIFT)

#  define DSU_FUSES_DEV_FAMILY_CFG_1_ADDR    (SAM_AUX1_AREA2 + 4)
#  define DSU_FUSES_DEV_FAMILY_CFG_1_SHIFT   (0)     /* Bits 0-15: Device Family Configuration bits 42:27 */
#  define DSU_FUSES_DEV_FAMILY_CFG_1_MASK    (0xffff << DSU_FUSES_DEV_FAMILY_CFG_1_SHIFT)
#    define DSU_FUSES_DEV_FAMILY_CFG_1(n)    ((n) << DSU_FUSES_DEV_FAMILY_CFG_1_SHIFT)

#  define ADC_FUSES_DCFG_ADDR                (SAM_AUX1_AREA2 + 4)
#  define ADC_FUSES_DCFG_SHIFT               (16)    /* Bits 16-19: ADC Device Configuration */
#  define ADC_FUSES_DCFG_MASK                (15 << ADC_FUSES_DCFG_SHIFT)
#    define ADC_FUSES_DCFG(n)                ((n) << ADC_FUSES_DCFG_SHIFT)

#  define ADC_FUSES_CMPDELAY_ADDR            (SAM_AUX1_AREA2 + 4)
#  define ADC_FUSES_CMPDELAY_SHIFT           (16)    /* Bit 16: ADC Comparator Delay */
#  define ADC_FUSES_CMPDELAY_MASK            (1 << ADC_FUSES_CMPDELAY_SHIFT)

#  define ADC_FUSES_BOOSTEN_ADDR             (SAM_AUX1_AREA2 + 4)
#  define ADC_FUSES_BOOSTEN_SHIFT            (17)    /* Bit 17: ADC Boost Enable */
#  define ADC_FUSES_BOOSTEN_MASK             (1 << ADC_FUSES_BOOSTEN_SHIFT)

#  define ADC_FUSES_VCMPULSE_ADDR            (SAM_AUX1_AREA2 + 4)
#  define ADC_FUSES_VCMPULSE_SHIFT           (18)    /* Bit 18: ADC VCM Pulse */
#  define ADC_FUSES_VCMPULSE_MASK            (1 << ADC_FUSES_VCMPULSE_SHIFT)

#  define ADC_FUSES_BIAS_OPA_ADDR            (SAM_AUX1_AREA2 + 4)
#  define ADC_FUSES_BIAS_OPA_SHIFT           (19)    /* Bit 19: ADC OPA Bias */
#  define ADC_FUSES_BIAS_OPA_MASK            (1 << ADC_FUSES_BIAS_OPA_SHIFT)

#  define DSU_FUSES_RAM_BIAS_ADDR            (SAM_AUX1_AREA2 + 4)
#  define DSU_FUSES_RAM_BIAS_SHIFT           (20)    /* Bits 20-21: RAM Bias */
#  define DSU_FUSES_RAM_BIAS_MASK            (3 << DSU_FUSES_RAM_BIAS_SHIFT)
#    define DSU_FUSES_RAM_BIAS(n)            ((n) << DSU_FUSES_RAM_BIAS_SHIFT)

#  define DSU_FUSES_RAM_READ_MARGIN_ADDR     (SAM_AUX1_AREA2 + 4)
#  define DSU_FUSES_RAM_READ_MARGIN_SHIFT    (22)   /* Bits 22-25: RAM Read Margin */
#  define DSU_FUSES_RAM_READ_MARGIN_MASK     (15 << DSU_FUSES_RAM_READ_MARGIN_SHIFT)
#    define DSU_FUSES_RAM_READ_MARGIN(n)     ((n) << DSU_FUSES_RAM_READ_MARGIN_SHIFT)
#endif

#ifdef CONFIG_ARCH_FAMILY_SAMD20
#  define SYSCTRL_FUSES_ULPVREG_ADDR         (SAM_AUX1_AREA4 + 0)
#  define SYSCTRL_FUSES_ULPVREG_SHIFT        (0)    /* Bits 0-2: ULP Regulator Fallback Mode */
#  define SYSCTRL_FUSES_ULPVREG_MASK         (7 << SYSCTRL_FUSES_ULPVREG_SHIFT)
#    define SYSCTRL_FUSES_ULPVREG(n)         ((n) << SYSCTRL_FUSES_ULPVREG_SHIFT)

#  define ADC_FUSES_GAINCORR_ADDR            (SAM_AUX1_AREA4 + 0)
#  define ADC_FUSES_GAINCORR_SHIFT           (3)    /* Bits 3-14: ADC Gain Correction */
#  define ADC_FUSES_GAINCORR_MASK            (0xfff << ADC_FUSES_GAINCORR_SHIFT)
#    define ADC_FUSES_GAINCORR(n)            ((n) << ADC_FUSES_GAINCORR_SHIFT)

#  define ADC_FUSES_OFFSETCORR_ADDR          (SAM_AUX1_AREA4 + 0)
#  define ADC_FUSES_OFFSETCORR_SHIFT         (15)   /* Bits 15-26: ADC Offset Correction */
#  define ADC_FUSES_OFFSETCORR_MASK          (0xfff << ADC_FUSES_OFFSETCORR_SHIFT)
#    define ADC_FUSES_OFFSETCORR(n)          ((n) << ADC_FUSES_OFFSETCORR_SHIFT)
#endif

#define ADC_FUSES_LINEARITY_0_ADDR           (SAM_AUX1_AREA4 + 0)
#define ADC_FUSES_LINEARITY_0_SHIFT          (27)   /* Bits 27-31: ADC Linearity bits 4:0 */
#define ADC_FUSES_LINEARITY_0_MASK           (0x1f << ADC_FUSES_LINEARITY_0_SHIFT)
#  define ADC_FUSES_LINEARITY_0(n)           ((n) << ADC_FUSES_LINEARITY_0_SHIFT)

#define ADC_FUSES_LINEARITY_1_ADDR           (SAM_AUX1_AREA4 + 4)
#define ADC_FUSES_LINEARITY_1_SHIFT          (0)    /* Bits 32-34: ADC Linearity bits 7:5 */
#define ADC_FUSES_LINEARITY_1_MASK           (7 << ADC_FUSES_LINEARITY_1_SHIFT)
#  define ADC_FUSES_LINEARITY_1(n)           ((n) << ADC_FUSES_LINEARITY_1_SHIFT)

#define ADC_FUSES_BIASCAL_ADDR               (SAM_AUX1_AREA4 + 4)
#define ADC_FUSES_BIASCAL_SHIFT              (3)    /* Bits 35-27: ADC Bias Calibration */
#define ADC_FUSES_BIASCAL_MASK               (7 << ADC_FUSES_BIASCAL_SHIFT)
#  define ADC_FUSES_BIASCAL(n)               ((n) << ADC_FUSES_BIASCAL_SHIFT)

#define SYSCTRL_FUSES_OSC32KCAL_ADDR         (SAM_AUX1_AREA4 + 4)
#define SYSCTRL_FUSES_OSC32KCAL_SHIFT        (6)    /* Bits 38-44: OSC32K Calibration */
#define SYSCTRL_FUSES_OSC32KCAL_MASK         (0x7f << SYSCTRL_FUSES_OSC32KCAL_SHIFT)
#  define SYSCTRL_FUSES_OSC32KCAL(n)         ((n) << SYSCTRL_FUSES_OSC32KCAL_SHIFT)

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SYSCTRL_FUSES_USBTRANSN_ADDR       (SAM_AUX1_AREA4 + 4)
#  define SYSCTRL_FUSES_USBTRANSN_SHIFT      (13)   /* Bits 45-49: USB TRANSN calibration value. */
#  define SYSCTRL_FUSES_USBTRANSN_MASK       (0x1f << SYSCTRL_FUSES_USBTRANSN_SHIFT)
#    define SYSCTRL_FUSES_USBTRANSN(n)       ((n) << SYSCTRL_FUSES_USBTRANSN_SHIFT)

#  define SYSCTRL_FUSES_USBTRANSP_ADDR       (SAM_AUX1_AREA4 + 4)
#  define SYSCTRL_FUSES_USBTRANSP_SHIFT      (18)   /* Bits 50-54: USB TRANSP calibration value. */
#  define SYSCTRL_FUSES_USBTRANSP_MASK       (0x1f << SYSCTRL_FUSES_USBTRANSP_SHIFT)
#    define SYSCTRL_FUSES_USBTRANSP(n)       ((n) << SYSCTRL_FUSES_USBTRANSP_SHIFT)

#  define SYSCTRL_FUSES_USBTRIM_ADDR         (SAM_AUX1_AREA4 + 4)
#  define SYSCTRL_FUSES_USBTRIM_SHIFT        (23)   /* Bits 55-57: USB TRIM calibration value. */
#  define SYSCTRL_FUSES_USBTRIM_MASK         (7 << SYSCTRL_FUSES_USBTRIM_SHIFT)
#    define SYSCTRL_FUSES_USBTRIM(n)         ((n) << SYSCTRL_FUSES_USBTRIM_SHIFT)

#  define SYSCTRL_FUSES_DFLL48MCOARSE_ADDR   (SAM_AUX1_AREA4 + 4)
#  define SYSCTRL_FUSES_DFLL48MCOARSE_SHIFT  (26)   /* Bits 58-63: DFLL48M Coarse calibration value. */
#  define SYSCTRL_FUSES_DFLL48MCOARSE_MASK   (0x3f << SYSCTRL_FUSES_DFLL48MCOARSE_SHIFT)
#    define SYSCTRL_FUSES_DFLL48MCOARSE(n)   ((n) << SYSCTRL_FUSES_DFLL48MCOARSE_SHIFT)

#  define SYSCTRL_FUSES_DFLL48MFINE_ADDR     (SAM_AUX1_AREA4 + 8)
#  define SYSCTRL_FUSES_DFLL48MFINE_SHIFT    (0)    /* Bits 64-74: DFLL48M Fine calibration value. */
#  define SYSCTRL_FUSES_DFLL48MFINE_MASK     (0x7ff << SYSCTRL_FUSES_DFLL48MFINE_SHIFT)
#    define SYSCTRL_FUSES_DFLL48MFINE(n)     ((n) << SYSCTRL_FUSES_DFLL48MFINE_SHIFT)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAMD20 || CONFIG_ARCH_FAMILY_SAMD21 */
#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_FUSES_H */
