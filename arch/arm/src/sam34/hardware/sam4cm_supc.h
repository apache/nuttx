/****************************************************************************************
 * arch/arm/src/sam34/hardware/sam4cm_supc.h
 * Supply Controller (SUPC) definitions for the SAM4CM
 *
 *   Copyright (C) 2014-2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4CM_SUPC_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4CM_SUPC_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* SUPC register offsets ****************************************************************/

#define SAM_SUPC_CR_OFFSET              0x00 /* Supply Controller Control Register */
#define SAM_SUPC_SMMR_OFFSET            0x04 /* Supply Controller Supply Monitor Mode Register */
#define SAM_SUPC_MR_OFFSET              0x08 /* Supply Controller Mode Register */
#define SAM_SUPC_WUMR_OFFSET            0x0c /* Supply Controller Wake Up Mode Register */
#define SAM_SUPC_WUIR_OFFSET            0x10 /* Supply Controller Wake Up Inputs Register */
#define SAM_SUPC_SR_OFFSET              0x14 /* Supply Controller Status Register */

/* SUPC register addresses **************************************************************/

#define SAM_SUPC_CR                     (SAM_SUPC_BASE+SAM_SUPC_CR_OFFSET)
#define SAM_SUPC_SMMR                   (SAM_SUPC_BASE+SAM_SUPC_SMMR_OFFSET)
#define SAM_SUPC_MR                     (SAM_SUPC_BASE+SAM_SUPC_MR_OFFSET)
#define SAM_SUPC_WUMR                   (SAM_SUPC_BASE+SAM_SUPC_WUMR_OFFSET)
#define SAM_SUPC_WUIR                   (SAM_SUPC_BASE+SAM_SUPC_WUIR_OFFSET)
#define SAM_SUPC_SR                     (SAM_SUPC_BASE+SAM_SUPC_SR_OFFSET)

/* SUPC register bit definitions ********************************************************/
/* Supply Controller Control Register */

#define SUPC_CR_VROFF                   (1 << 2)  /* Bit 2:  Voltage Regulator Off */
#define SUPC_CR_XTALSEL                 (1 << 3)  /* Bit 3:  Crystal Oscillator Select */
#define SUPC_CR_KEY_SHIFT               (24)      /* Bits 24-31:  Password */
#define SUPC_CR_KEY_MASK                (0xff << SUPC_CR_KEY_SHIFT)
#  define SUPR_CR_KEY                   (0xa5 << SUPC_CR_KEY_SHIFT)

/* Supply Controller Supply Monitor Mode Register */

#define SUPC_SMMR_SMTH_SHIFT            (0)       /* Bits 0-3:  Supply Monitor Threshold */
#define SUPC_SMMR_SMTH_MASK             (15 << SUPC_SMMR_SMTH_SHIFT)
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

#define SUPC_MR_
#define SUPC_MR_BODRSTEN                (1 << 12) /* Bit 12: Brownout Detector Reset Enable */
#define SUPC_MR_BODDIS                  (1 << 13) /* Bit 13: Brownout Detector Disable */
#define SUPC_MR_OSCBYPASS               (1 << 20) /* Bit 20: Oscillator Bypass */
#define SUPC_MR_KEY_SHIFT               (24)      /* Bits 24-31:  Password Key */
#define SUPC_MR_KEY_MASK                (0xff << SUPC_MR_KEY_SHIFT)
#  define SUPR_MR_KEY                   (0xa5 << SUPC_MR_KEY_SHIFT)

/* Supply Controller Wake Up Mode Register */

#define SUPC_WUMR_FWUPEN                (1 << 0)  /* Bit 0:  Force Wake Up Enable */
#define SUPC_WUMR_SMEN                  (1 << 1)  /* Bit 1:  Supply Monitor Wake Up Enable */
#define SUPC_WUMR_RTTEN                 (1 << 2)  /* Bit 2:  Real Time Timer Wake Up Enable */
#define SUPC_WUMR_RTCEN                 (1 << 3)  /* Bit 3:  Real Time Clock Wake Up Enable */
#define SUPC_WUMR_FWUPDBC_SHIFT         (8)       /* Bits 8-10:  Force Wake Up Debouncer */
#define SUPC_WUMR_FWUPDBC_MASK          (7 << SUPC_WUMR_FWUPDBC_SHIFT)
#  define SUPC_WUMR_FWUPDBC_1SCLK       (0 << SUPC_WUMR_FWUPDBC_SHIFT) /* Immediate, no debouncing */
#  define SUPC_WUMR_FWUPDBC_3SCLK       (1 << SUPC_WUMR_FWUPDBC_SHIFT) /* FWUP at least 3 SLCK periods */
#  define SUPC_WUMR_FWUPDBC_32SCLK      (2 << SUPC_WUMR_FWUPDBC_SHIFT) /* FWUP at least 32 SLCK periods */
#  define SUPC_WUMR_FWUPDBC_512SCLK     (3 << SUPC_WUMR_FWUPDBC_SHIFT) /* FWUP at least 512 SLCK periods */
#  define SUPC_WUMR_FWUPDBC_4096SCLK    (4 << SUPC_WUMR_FWUPDBC_SHIFT) /* FWUP at least 4096 SLCK periods */
#  define SUPC_WUMR_FWUPDBC_32768SCLK   (5 << SUPC_WUMR_FWUPDBC_SHIFT) /* FWUP at least 32768 SLCK periods */
#define SUPC_WUMR_WKUPDBC_SHIFT         (12)      /* Bits 12-14:  Wake Up Inputs Debouncer */
#define SUPC_WUMR_WKUPDBC_MASK          (7 << SUPC_WUMR_WKUPDBC_SHIFT)
#  define SUPC_WUMR_WKUPDBC_1SCLK       (0 << SUPC_WUMR_WKUPDBC_SHIFT) /* Immediate, no debouncing */
#  define SUPC_WUMR_WKUPDBC_3SCLK       (1 << SUPC_WUMR_WKUPDBC_SHIFT) /* Input active at least 3 SLCK periods */
#  define SUPC_WUMR_WKUPDBC_32SCLK      (2 << SUPC_WUMR_WKUPDBC_SHIFT) /* Input active at least 32 SLCK periods */
#  define SUPC_WUMR_WKUPDBC_512SCLK     (3 << SUPC_WUMR_WKUPDBC_SHIFT) /* Input active at least 512 SLCK periods */
#  define SUPC_WUMR_WKUPDBC_4096SCLK    (4 << SUPC_WUMR_WKUPDBC_SHIFT) /* Input active at least 4096 SLCK periods */
#  define SUPC_WUMR_WKUPDBC_32768SCLK   (5 << SUPC_WUMR_WKUPDBC_SHIFT) /* Input active at least 32768 SLCK periods */

/* System Controller Wake Up Inputs Register */

#define SUPC_WUIR_WKUPEN_SHIFT          (0)       /* Bits 0-15:  Wake Up Input Enable 0 to 15 */
#define SUPC_WUIR_WKUPEN_MASK           (0xffff << SUPC_WUIR_WKUPEN_SHIFT)
#  define SUPC_WUIR_WKUPEN(n)           ((1 << (n)) << SUPC_WUIR_WKUPEN_SHIFT)
#define SUPC_WUIR_WKUPT_SHIFT           (16)      /* Bits 16-31  Wake Up Input Transition 0 to 15 */
#define SUPC_WUIR_WKUPT_MASK            (0xffff << SUPC_WUIR_WKUPT_SHIFT)
#  define SUPC_WUIR_WKUPT(n)            ((1 << (n)) << SUPC_WUIR_WKUPT_SHIFT)

/* Supply Controller Status Register */

#define SUPC_SR_WKUPS                   (1 << 1)  /* Bit 1:  WKUP Wake Up Status */
#define SUPC_SR_SMWS                    (1 << 2)  /* Bit 2:  Supply Monitor Detection Wake Up Status */
#define SUPC_SR_BODRSTS                 (1 << 3)  /* Bit 3:  Brownout Detector Reset Status */
#define SUPC_SR_SMRSTS                  (1 << 4)  /* Bit 4:  Supply Monitor Reset Status */
#define SUPC_SR_SMS                     (1 << 5)  /* Bit 5:  Supply Monitor Status */
#define SUPC_SR_SMOS                    (1 << 6)  /* Bit 6:  Supply Monitor Output Status */
#define SUPC_SR_OSCSEL                  (1 << 7)  /* Bit 7:  32-kHz Oscillator Selection Status */
#define SUPC_SR_WKUPIS_SHIFT            (16)      /* Bits 16-31:  WKUP Input Status 0 to 15 */
#define SUPC_SR_WKUPIS_MASK             (0xffff << SUPC_SR_WKUPIS_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4CM_SUPC_H */
