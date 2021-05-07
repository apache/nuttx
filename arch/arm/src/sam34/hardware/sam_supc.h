/****************************************************************************
 * arch/arm/src/sam34/hardware/sam_supc.h
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

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_SUPC_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_SUPC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SUPC register offsets ****************************************************/

#define SAM_SUPC_CR_OFFSET              0x00 /* Supply Controller Control Register */
#define SAM_SUPC_SMMR_OFFSET            0x04 /* Supply Controller Supply Monitor Mode Register */
#define SAM_SUPC_MR_OFFSET              0x08 /* Supply Controller Mode Register */
#define SAM_SUPC_WUMR_OFFSET            0x0c /* Supply Controller Wake Up Mode Register */
#define SAM_SUPC_WUIR_OFFSET            0x10 /* Supply Controller Wake Up Inputs Register */
#define SAM_SUPC_SR_OFFSET              0x14 /* Supply Controller Status Register */

/* SUPC register addresses **************************************************/

#define SAM_SUPC_CR                     (SAM_SUPC_BASE+SAM_SUPC_CR_OFFSET)
#define SAM_SUPC_SMMR                   (SAM_SUPC_BASE+SAM_SUPC_SMMR_OFFSET)
#define SAM_SUPC_MR                     (SAM_SUPC_BASE+SAM_SUPC_MR_OFFSET)
#define SAM_SUPC_WUMR                   (SAM_SUPC_BASE+SAM_SUPC_WUMR_OFFSET)
#define SAM_SUPC_WUIR                   (SAM_SUPC_BASE+SAM_SUPC_WUIR_OFFSET)
#define SAM_SUPC_SR                     (SAM_SUPC_BASE+SAM_SUPC_SR_OFFSET)

/* SUPC register bit definitions ********************************************/

/* Supply Controller Control Register */

#define SUPC_CR_VROFF                   (1 << 2)  /* Bit 2:  Voltage Regulator Off */
#define SUPC_CR_XTALSEL                 (1 << 3)  /* Bit 3:  Crystal Oscillator Select */
#define SUPC_CR_KEY_SHIFT               (24)      /* Bits 24-31:  Password */
#define SUPC_CR_KEY_MASK                (0xff << SUPC_CR_KEY_SHIFT)
#  define SUPR_CR_KEY                   (0xa5 << SUPC_CR_KEY_SHIFT)

/* Supply Controller Supply Monitor Mode Register */

#define SUPC_SMMR_SMTH_SHIFT            (0)       /* Bits 0-3:  Supply Monitor Threshold */
#define SUPC_SMMR_SMTH_MASK             (15 << SUPC_SMMR_SMTH_SHIFT)
#  define SUPC_SMMR_SMTH(n)             ((uint32_t)(n) << SUPC_SMMR_SMTH_SHIFT)

#if defined(CONFIG_ARCH_CHIP_SAM4CM) || defined(CONFIG_ARCH_CHIP_SAM4S) || \
    defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SUPC_SMMR_SMTH_1p6V           (0  << SUPC_SMMR_SMTH_SHIFT) /* 1.56 < 1.6 < 1.64 */
#  define SUPC_SMMR_SMTH_1p7V           (1  << SUPC_SMMR_SMTH_SHIFT) /* 1.68 < 1.72 < 1.76 */
#  define SUPC_SMMR_SMTH_1p8V           (2  << SUPC_SMMR_SMTH_SHIFT) /* 1.79 < 1.84 < 1.89 */
#  define SUPC_SMMR_SMTH_2p0V           (3  << SUPC_SMMR_SMTH_SHIFT) /* 1.91 < 1.96 < 2.01 */
#  define SUPC_SMMR_SMTH_2p1V           (4  << SUPC_SMMR_SMTH_SHIFT) /* 2.03 < 2.08 < 2.13 */
#  define SUPC_SMMR_SMTH_2p2V           (5  << SUPC_SMMR_SMTH_SHIFT) /* 2.15 < 2.2  < 2.23 */
#  define SUPC_SMMR_SMTH_2p3V           (6  << SUPC_SMMR_SMTH_SHIFT) /* 2.26 < 2.32 < 2.38 */
#  define SUPC_SMMR_SMTH_2p4V           (7  << SUPC_SMMR_SMTH_SHIFT) /* 2.38 < 2.44 < 2.50 */
#  define SUPC_SMMR_SMTH_2p6V           (8  << SUPC_SMMR_SMTH_SHIFT) /* 2.50 < 2.56 < 2.62 */
#  define SUPC_SMMR_SMTH_2p7V           (9  << SUPC_SMMR_SMTH_SHIFT) /* 2.61 < 2.68 < 2.75 */
#  define SUPC_SMMR_SMTH_2p8V           (10 << SUPC_SMMR_SMTH_SHIFT) /* 2.73 < 2.8  < 2.87 */
#  define SUPC_SMMR_SMTH_2p9V           (11 << SUPC_SMMR_SMTH_SHIFT) /* 2.85 < 2.92 < 2.99 */
#  define SUPC_SMMR_SMTH_3p0V           (12 << SUPC_SMMR_SMTH_SHIFT) /* 2.96 < 3.04 < 3.12 */
#  define SUPC_SMMR_SMTH_3p2V           (13 << SUPC_SMMR_SMTH_SHIFT) /* 3.08 < 3.16 < 3.24 */
#  define SUPC_SMMR_SMTH_3p3V           (14 << SUPC_SMMR_SMTH_SHIFT) /* 3.20 < 3.28 < 3.36 */
#  define SUPC_SMMR_SMTH_3p4V           (15 << SUPC_SMMR_SMTH_SHIFT) /* 3.32 < 3.4  < 3.49 */
#elif defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
      defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SUPC_SMMR_SMTH_1p9V           (0  << SUPC_SMMR_SMTH_SHIFT) /* 1.9V */
#  define SUPC_SMMR_SMTH_2p0V           (1  << SUPC_SMMR_SMTH_SHIFT) /* 2.0V */
#  define SUPC_SMMR_SMTH_2p1V           (2  << SUPC_SMMR_SMTH_SHIFT) /* 2.1V */
#  define SUPC_SMMR_SMTH_2p2V           (3  << SUPC_SMMR_SMTH_SHIFT) /* 2.2V */
#  define SUPC_SMMR_SMTH_2p3V           (4  << SUPC_SMMR_SMTH_SHIFT) /* 2.3V */
#  define SUPC_SMMR_SMTH_2p4V           (5  << SUPC_SMMR_SMTH_SHIFT) /* 2.4V */
#  define SUPC_SMMR_SMTH_2p5V           (6  << SUPC_SMMR_SMTH_SHIFT) /* 2.5V */
#  define SUPC_SMMR_SMTH_2p6V           (7  << SUPC_SMMR_SMTH_SHIFT) /* 2.6V */
#  define SUPC_SMMR_SMTH_2p7V           (8  << SUPC_SMMR_SMTH_SHIFT) /* 2.7V */
#  define SUPC_SMMR_SMTH_2p8V           (9  << SUPC_SMMR_SMTH_SHIFT) /* 2.8V */
#  define SUPC_SMMR_SMTH_2p9V           (10 << SUPC_SMMR_SMTH_SHIFT) /* 2.9V */
#  define SUPC_SMMR_SMTH_3p0V           (11 << SUPC_SMMR_SMTH_SHIFT) /* 3.0V */
#  define SUPC_SMMR_SMTH_3p1V           (12 << SUPC_SMMR_SMTH_SHIFT) /* 3.1V */
#  define SUPC_SMMR_SMTH_3p2V           (13 << SUPC_SMMR_SMTH_SHIFT) /* 3.2V */
#  define SUPC_SMMR_SMTH_3p3V           (14 << SUPC_SMMR_SMTH_SHIFT) /* 3.3V */
#  define SUPC_SMMR_SMTH_3p4V           (15 << SUPC_SMMR_SMTH_SHIFT) /* 3.4V */
#endif

#define SUPC_SMMR_SMSMPL_SHIFT          (8)       /* Bits 8-10:  Supply Monitor Sampling Period */
#define SUPC_SMMR_SMSMPL_MASK           (7 << SUPC_SMMR_SMSMPL_SHIFT)
#  define SUPC_SMMR_SMSMPL_SMD          (0 << SUPC_SMMR_SMSMPL_SHIFT) /* Supply Monitor disabled */
#  define SUPC_SMMR_SMSMPL_CSM          (1 << SUPC_SMMR_SMSMPL_SHIFT) /* Continuous Supply Monitor */
#  define SUPC_SMMR_SMSMPL_32SLCK       (2 << SUPC_SMMR_SMSMPL_SHIFT) /* Eevery 32 SLCK periods */
#  define SUPC_SMMR_SMSMPL_256SLCK      (3 << SUPC_SMMR_SMSMPL_SHIFT) /* Every 256 SLCK periods */
#  define SUPC_SMMR_SMSMPL_2048SLCK     (4 << SUPC_SMMR_SMSMPL_SHIFT) /* Every 2,048 SLCK periods */

#define SUPC_SMMR_SMRSTEN               (1 << 12) /* Bit 12: Supply Monitor Reset Enable */
#define SUPC_SMMR_SMIEN                 (1 << 13) /* Bit 13: Supply Monitor Interrupt Enable */

/* Supply Controller Mode Register */

#if defined(CONFIG_ARCH_CHIP_SAM4CM)

#define SUPC_MR_LCDVROUT_SHIFT          (0)
#define SUPC_MR_LCDVROUT_MASK           (15 << SUPC_MR_LCDVROUT_SHIFT)
#  define SUPC_MR_LCDVROUT_2p92V        (0 << SUPC_MR_LCDVROUT_SHIFT)
#  define SUPC_MR_LCDVROUT_2p85V        (1 << SUPC_MR_LCDVROUT_SHIFT)
#  define SUPC_MR_LCDVROUT_2p77V        (2 << SUPC_MR_LCDVROUT_SHIFT)
#  define SUPC_MR_LCDVROUT_2p70V        (3 << SUPC_MR_LCDVROUT_SHIFT)
#  define SUPC_MR_LCDVROUT_2p63V        (4 << SUPC_MR_LCDVROUT_SHIFT)
#  define SUPC_MR_LCDVROUT_2p55V        (5 << SUPC_MR_LCDVROUT_SHIFT)
#  define SUPC_MR_LCDVROUT_2p48V        (6 << SUPC_MR_LCDVROUT_SHIFT)
#  define SUPC_MR_LCDVROUT_2p41V        (7 << SUPC_MR_LCDVROUT_SHIFT)
#  define SUPC_MR_LCDVROUT_3p51V        (8 << SUPC_MR_LCDVROUT_SHIFT)
#  define SUPC_MR_LCDVROUT_3p44V        (9 << SUPC_MR_LCDVROUT_SHIFT)
#  define SUPC_MR_LCDVROUT_3p36V        (10 << SUPC_MR_LCDVROUT_SHIFT)
#  define SUPC_MR_LCDVROUT_3p29V        (11 << SUPC_MR_LCDVROUT_SHIFT)
#  define SUPC_MR_LCDVROUT_3p22V        (12 << SUPC_MR_LCDVROUT_SHIFT)
#  define SUPC_MR_LCDVROUT_3p14V        (13 << SUPC_MR_LCDVROUT_SHIFT)
#  define SUPC_MR_LCDVROUT_3p07V        (14 << SUPC_MR_LCDVROUT_SHIFT)
#  define SUPC_MR_LCDVROUT_3p00V        (15 << SUPC_MR_LCDVROUT_SHIFT)

#define SUPC_MR_LCDMODE_SHIFT           (4)
#define SUPC_MR_LCDMODE_MASK            (3 << SUPC_MR_LCDMODE_SHIFT)
#  define SUPC_MR_LCDMODE_LCDOFF        (0 << SUPC_MR_LCDMODE_SHIFT) /* The internal supply source and the external supply source are both deselected */
#  define SUPC_MR_LCDMODE_LCDON_EXTVR   (2 << SUPC_MR_LCDMODE_SHIFT) /* The external supply source for LCD (VDDLCD) is selected (the LCD voltage regulator is in Hi-Z Mode) */
#  define SUPC_MR_LCDMODE_LCDON_INVR    (3 << SUPC_MR_LCDMODE_SHIFT) /* The internal supply source for LCD (the LCD Voltage Regulator) is selected (Active Mode) */

#endif

#define SUPC_MR_BODRSTEN                (1 << 12) /* Bit 12: Brownout Detector Reset Enable */
#define SUPC_MR_BODDIS                  (1 << 13) /* Bit 13: Brownout Detector Disable */

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A) || \
    defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SUPC_MR_ONREG                 (1 << 14) /* Bit 14: Voltage Regulator enable */
#endif

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SUPC_MR_VDDIORDY              (1 << 14) /* Bit 14: VDDIO Ready */
#endif

#define SUPC_MR_OSCBYPASS               (1 << 20) /* Bit 20: Oscillator Bypass */
#define SUPC_MR_KEY_SHIFT               (24)      /* Bits 24-31:  Password Key */
#define SUPC_MR_KEY_MASK                (0xff << SUPC_MR_KEY_SHIFT)
#  define SUPC_MR_KEY                   (0xa5 << SUPC_MR_KEY_SHIFT)

/* Supply Controller Wake Up Mode Register */

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SUPC_WUMR_FWUPEN              (1 << 0)  /* Bit 0:  Force Wake Up Enable */
#endif

#define SUPC_WUMR_SMEN                  (1 << 1)  /* Bit 1:  Supply Monitor Wake Up Enable */
#define SUPC_WUMR_RTTEN                 (1 << 2)  /* Bit 2:  Real Time Timer Wake Up Enable */
#define SUPC_WUMR_RTCEN                 (1 << 3)  /* Bit 3:  Real Time Clock Wake Up Enable */

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SUPC_WUMR_LPDBCEN0            (1 << 5)  /* Bit 5:  Low power Debouncer ENable WKUP0 */
#  define SUPC_WUMR_LPDBCEN1            (1 << 6)  /* Bit 6:  Low power Debouncer ENable WKUP1 */
#  define SUPC_WUMR_LPDBCCLR            (1 << 7)  /* Bit 7:  Low power Debouncer Clear */
#endif

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SUPC_WUMR_FWUPDBC_SHIFT       (8)       /* Bits 8-10:  Force Wake Up Debouncer */
#  define SUPC_WUMR_FWUPDBC_MASK        (7 << SUPC_WUMR_FWUPDBC_SHIFT)
#    define SUPC_WUMR_FWUPDBC_1SCLK     (0 << SUPC_WUMR_FWUPDBC_SHIFT) /* Immediate, no debouncing */
#    define SUPC_WUMR_FWUPDBC_3SCLK     (1 << SUPC_WUMR_FWUPDBC_SHIFT) /* FWUP at least 3 SLCK periods */
#    define SUPC_WUMR_FWUPDBC_32SCLK    (2 << SUPC_WUMR_FWUPDBC_SHIFT) /* FWUP at least 32 SLCK periods */
#    define SUPC_WUMR_FWUPDBC_512SCLK   (3 << SUPC_WUMR_FWUPDBC_SHIFT) /* FWUP at least 512 SLCK periods */
#    define SUPC_WUMR_FWUPDBC_4096SCLK  (4 << SUPC_WUMR_FWUPDBC_SHIFT) /* FWUP at least 4096 SLCK periods */
#    define SUPC_WUMR_FWUPDBC_32768SCLK (5 << SUPC_WUMR_FWUPDBC_SHIFT) /* FWUP at least 32768 SLCK periods */
#endif

#define SUPC_WUMR_WKUPDBC_SHIFT         (12)      /* Bits 12-14:  Wake Up Inputs Debouncer */
#define SUPC_WUMR_WKUPDBC_MASK          (7 << SUPC_WUMR_WKUPDBC_SHIFT)
#  define SUPC_WUMR_WKUPDBC_1SCLK       (0 << SUPC_WUMR_WKUPDBC_SHIFT) /* Immediate, no debouncing */
#  define SUPC_WUMR_WKUPDBC_3SCLK       (1 << SUPC_WUMR_WKUPDBC_SHIFT) /* Input active at least 3 SLCK periods */
#  define SUPC_WUMR_WKUPDBC_32SCLK      (2 << SUPC_WUMR_WKUPDBC_SHIFT) /* Input active at least 32 SLCK periods */
#  define SUPC_WUMR_WKUPDBC_512SCLK     (3 << SUPC_WUMR_WKUPDBC_SHIFT) /* Input active at least 512 SLCK periods */
#  define SUPC_WUMR_WKUPDBC_4096SCLK    (4 << SUPC_WUMR_WKUPDBC_SHIFT) /* Input active at least 4096 SLCK periods */
#  define SUPC_WUMR_WKUPDBC_32768SCLK   (5 << SUPC_WUMR_WKUPDBC_SHIFT) /* Input active at least 32768 SLCK periods */

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SUPC_WUMR_LPDBC_SHIFT         (16)      /* Bits 16-18: Low Power Debouncer Period */
#  define SUPC_WUMR_LPDBC_MASK          (7 << SUPC_WUMR_LPDBC_SHIFT)
#    define SUPC_WUMR_LPDBC_DISABLE     (0 << SUPC_WUMR_LPDBC_SHIFT) /* Disable low power debouncer */
#    define SUPC_WUMR_LPDBC_2_RTCOUT0   (1 << SUPC_WUMR_LPDBC_SHIFT) /* WKUP0/1 for 2 RTCOUT0 */
#    define SUPC_WUMR_LPDBC_3_RTCOUT0   (2 << SUPC_WUMR_LPDBC_SHIFT) /* WKUP0/1 for 3 RTCOUT0 */
#    define SUPC_WUMR_LPDBC_4_RTCOUT0   (3 << SUPC_WUMR_LPDBC_SHIFT) /* WKUP0/1 for 4 RTCOUT0 */
#    define SUPC_WUMR_LPDBC_5_RTCOUT0   (4 << SUPC_WUMR_LPDBC_SHIFT) /* WKUP0/1 for 5 RTCOUT0 */
#    define SUPC_WUMR_LPDBC_6_RTCOUT0   (5 << SUPC_WUMR_LPDBC_SHIFT) /* WKUP0/1 for 6 RTCOUT0 */
#    define SUPC_WUMR_LPDBC_7_RTCOUT0   (6 << SUPC_WUMR_LPDBC_SHIFT) /* WKUP0/1 for 7 RTCOUT0 */
#    define SUPC_WUMR_LPDBC_8_RTCOUT0   (7 << SUPC_WUMR_LPDBC_SHIFT) /* WKUP0/1 for 8 RTCOUT0 */
#endif

/* System Controller Wake Up Inputs Register */

#define SUPC_WUIR_WKUPEN_SHIFT          (0)       /* Bits 0-15:  Wake Up Input Enable 0 to 15 */
#define SUPC_WUIR_WKUPEN_MASK           (0xffff << SUPC_WUIR_WKUPEN_SHIFT)
#  define SUPC_WUIR_WKUPEN(n)           ((1 << (n)) << SUPC_WUIR_WKUPEN_SHIFT)
#define SUPC_WUIR_WKUPT_SHIFT           (16)      /* Bits 16-31  Wake Up Input Transition 0 to 15 */
#define SUPC_WUIR_WKUPT_MASK            (0xffff << SUPC_WUIR_WKUPT_SHIFT)
#  define SUPC_WUIR_WKUPT(n)            ((1 << (n)) << SUPC_WUIR_WKUPT_SHIFT)

/* Supply Controller Status Register */

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SUPC_SR_FWUPS                (1 << 0)  /* Bit 0:  FWUP Wake Up Status */
#endif

#define SUPC_SR_WKUPS                  (1 << 1)  /* Bit 1:  WKUP Wake Up Status */
#define SUPC_SR_SMWS                   (1 << 2)  /* Bit 2:  Supply Monitor Detection Wake Up Status */
#define SUPC_SR_BODRSTS                (1 << 3)  /* Bit 3:  Brownout Detector Reset Status */
#define SUPC_SR_SMRSTS                 (1 << 4)  /* Bit 4:  Supply Monitor Reset Status */
#define SUPC_SR_SMS                    (1 << 5)  /* Bit 5:  Supply Monitor Status */
#define SUPC_SR_SMOS                   (1 << 6)  /* Bit 6:  Supply Monitor Output Status */
#define SUPC_SR_OSCSEL                 (1 << 7)  /* Bit 7:  32-kHz Oscillator Selection Status */

#if defined(CONFIG_ARCH_CHIP_SAM4CM)
#  define SUPC_SR_LCDS                 (1 << 8)  /* Bit 8: LCD Status */
#endif

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SUPC_SR_FWUPIS               (1 << 12) /* Bit 12: FWUP Input Status */
#endif

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SUPC_SR_LPDBCS0              (1 << 13) /* Bit 13: Low Power Debouncer Wake Up Status on WKUP0 */
#  define SUPC_SR_LPDBCS1              (1 << 14) /* Bit 14: Low Power Debouncer Wake Up Status on WKUP1 */
#endif

#define SUPC_SR_WKUPIS_SHIFT           (16)      /* Bits 16-31:  WKUP Input Status 0 to 15 */
#define SUPC_SR_WKUPIS_MASK            (0xffff << SUPC_SR_WKUPIS_SHIFT)
#  define SUPC_SR_WKUPIS(n)            (1 << (SUPC_SR_WKUPIS_SHIFT+(n)))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_SUPC_H */
