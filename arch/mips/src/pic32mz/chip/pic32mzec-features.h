/************************************************************************************
 * arch/mips/src/pic32mz/chip/pic32mzec-features.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_MIPS_SRC_PIC32MZ_CHIP_PIC32MZEC_FEATURES_H
#define __ARCH_MIPS_SRC_PIC32MZ_CHIP_PIC32MZEC_FEATURES_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

mips32-memorymap.h:#define KSEG1_BASE 

#define PIC32MZ_LOWERBOOT_K1BASE  (0xbfc00000)
#define  PIC32MZ_BOOTCFG_K1BASE    (0xbfc0ff40)
#define PIC32MZ_UPPERBOOT_K1BASE  (0xbfc20000)
#define PIC32MZ_BOOT1_K1BASE      (0xbfc40000)
#define PIC32MZ_SEQCFG1_K1BASE    (0xbfc4ff40)
#define PIC32MZ_ADCCALIB_K1BASE   (0xbfc54000)
#define PIC32MZ_DEVSN_K1BASE      (0xbfc54020)
#define PIC32MZ_BOOT2_K1BASE      (0xbfc60000)
#define PIC32MZ_SEQCFG2_K1BASE    (0xbfc6ff40)


/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Register/Flash Offsets ***********************************************************/

/* Device ID, Revision, and Configuration (SFR PIC32MZ_CONFIG_K1BASE) */

#define PIC32MZ_CFGCON_OFFSET    0x0000 /* Configuration control register */
#define PIC32MZ_DEVID_OFFSET     0x0020 /* Device ID and revision register */
#define PIC32MZ_SYSKEY_OFFSET    0x0030 /* System key register */
#define PIC32MZ_CFGEBIA_OFFSET   0x00c0 /* External bus interface address pin configuration register */
#define PIC32MZ_CFGEBIC_OFFSET   0x00d0 /* External bus interface address pin control register */
#define PIC32MZ_CFGPG_OFFSET     0x00e0 /* Permission group configuration register */

/* Alternate Device Configuration (Boot Flash PIC32MZ_BOOTCFG_K1BASE) */

#define PIC32MZ_ADEVCFG3_OFFSET  0x0000 /* Alternate device configuration word 3 */
#define PIC32MZ_ADEVCFG2_OFFSET  0x0004 /* Alternate device configuration word 2 */
#define PIC32MZ_ADEVCFG1_OFFSET  0x0008 /* Alternate device configuration word 1 */
#define PIC32MZ_ADEVCFG0_OFFSET  0x000c /* Alternate device configuration word 0 */
#define PIC32MZ_ADEVCP3_OFFSET   0x0010 /* Alternate device code protect word 3 */
#define PIC32MZ_ADEVCP2_OFFSET   0x0014 /* Alternate device code protect word 2 */
#define PIC32MZ_ADEVCP1_OFFSET   0x0018 /* Alternate device code protect word 1 */
#define PIC32MZ_ADEVCP0_OFFSET   0x001c /* Alternate device code protect word 0 */
#define PIC32MZ_ADEVSIGN3_OFFSET 0x0020 /* Alternate evice signature word 3 */
#define PIC32MZ_ADEVSIGN2_OFFSET 0x0024 /* Alternate evice signature word 2 */
#define PIC32MZ_ADEVSIGN1_OFFSET 0x0028 /* Alternate evice signature word 1 */
#define PIC32MZ_ADEVSIGN0_OFFSET 0x002c /* Alternate evice signature word 0 */

/* Device Configuration (Boot Flash PIC32MZ_BOOTCFG_K1BASE) */

#define PIC32MZ_DEVCFG3_OFFSET   0x0080 /* Device configuration word 3 */
#define PIC32MZ_DEVCFG2_OFFSET   0x0084 /* Device configuration word 2 */
#define PIC32MZ_DEVCFG1_OFFSET   0x0088 /* Device configuration word 1 */
#define PIC32MZ_DEVCFG0_OFFSET   0x008c /* Device configuration word 0 */
#define PIC32MZ_DEVCP3_OFFSET    0x0090 /* Device code protect word 3 */
#define PIC32MZ_DEVCP2_OFFSET    0x0094 /* Device code protect word 2 */
#define PIC32MZ_DEVCP1_OFFSET    0x0098 /* Device code protect word 1 */
#define PIC32MZ_DEVCP0_OFFSET    0x009c /* Device code protect word 0 */
#define PIC32MZ_DEVSIGN3_OFFSET  0x00a0 /* Device signature word 3 */
#define PIC32MZ_DEVSIGN2_OFFSET  0x00a4 /* Device signature word 2 */
#define PIC32MZ_DEVSIGN1_OFFSET  0x00a8 /* Device signature word 1 */
#define PIC32MZ_DEVSIGN0_OFFSET  0x00ac /* Device signature word 0 */

/* Device ADC Calibration (Boot Flash PIC32MZ_ADCCALIB_K1BASE) */

#define PIC32MZ_DEVADC1_OFFSET   0x0000 /* ADC1 Calibration */
#define PIC32MZ_DEVADC2_OFFSET   0x0004 /* ADC2 Calibration */
#define PIC32MZ_DEVADC3_OFFSET   0x0008 /* ADC3 Calibration */
#define PIC32MZ_DEVADC4_OFFSET   0x000c /* ADC4 Calibration */
#define PIC32MZ_DEVADC5_OFFSET   0x0010 /* ADC5 Calibration */

/* Device Serial Number (Boot Flash PIC32MZ_DEVSN_K1BASE) */

#define PIC32MZ_DEVSN0_OFFSET    0x0000 /* Device serial number 0 */
#define PIC32MZ_DEVSN1_OFFSET    0x0004 /* Device serial number 1 */

/* Register/Flash Addresses *********************************************************/

/* Device ID, Revision, and Configuration (SFR PIC32MZ_CONFIG_K1BASE) */

#define PIC32MZ_CFGCON           (PIC32MZ_CONFIG_K1BASE+PIC32MZ_CFGCON_OFFSET)
#define PIC32MZ_DEVID            (PIC32MZ_CONFIG_K1BASE+PIC32MZ_DEVID_OFFSET)
#define PIC32MZ_SYSKEY           (PIC32MZ_CONFIG_K1BASE+PIC32MZ_SYSKEY_OFFSET)
#define PIC32MZ_CFGEBIA          (PIC32MZ_CONFIG_K1BASE+PIC32MZ_CFGEBIA_OFFSET)
#define PIC32MZ_CFGEBIC          (PIC32MZ_CONFIG_K1BASE+PIC32MZ_CFGEBIC_OFFSET)
#define PIC32MZ_CFGPG            (PIC32MZ_CONFIG_K1BASE+PIC32MZ_CFGPG_OFFSET)

/* Alternate Device Configuration (Boot Flash PIC32MZ_BOOTCFG_K1BASE) */

#define PIC32MZ_ADEVCFG3         (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_ADEVCFG3_OFFSET)
#define PIC32MZ_ADEVCFG2         (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_ADEVCFG2_OFFSET)
#define PIC32MZ_ADEVCFG1         (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_ADEVCFG1_OFFSET)
#define PIC32MZ_ADEVCFG0         (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_ADEVCFG0_OFFSET)
#define PIC32MZ_ADEVCP3          (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_ADEVCP3_OFFSET)
#define PIC32MZ_ADEVCP2          (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_ADEVCP2_OFFSET)
#define PIC32MZ_ADEVCP1          (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_ADEVCP1_OFFSET)
#define PIC32MZ_ADEVCP0          (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_ADEVCP0_OFFSET)
#define PIC32MZ_ADEVSIGN3        (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_ADEVSIGN3_OFFSET)
#define PIC32MZ_ADEVSIGN2        (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_ADEVSIGN2_OFFSET)
#define PIC32MZ_ADEVSIGN1        (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_ADEVSIGN1_OFFSET)
#define PIC32MZ_ADEVSIGN0        (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_ADEVSIGN0_OFFSET)

/* Device Configuration (Boot Flash PIC32MZ_BOOTCFG_K1BASE) */

#define PIC32MZ_DEVCFG3          (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_DEVCFG3_OFFSET)
#define PIC32MZ_DEVCFG2          (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_DEVCFG2_OFFSET)
#define PIC32MZ_DEVCFG1          (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_DEVCFG1_OFFSET)
#define PIC32MZ_DEVCFG0          (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_DEVCFG0_OFFSET)
#define PIC32MZ_DEVCP3           (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_DEVCP3_OFFSET)
#define PIC32MZ_DEVCP2           (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_DEVCP2_OFFSET)
#define PIC32MZ_DEVCP1           (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_DEVCP1_OFFSET)
#define PIC32MZ_DEVCP0           (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_DEVCP0_OFFSET)
#define PIC32MZ_DEVSIGN3         (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_DEVSIGN3_OFFSET)
#define PIC32MZ_DEVSIGN2         (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_DEVSIGN2_OFFSET)
#define PIC32MZ_DEVSIGN1         (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_DEVSIGN1_OFFSET)
#define PIC32MZ_DEVSIGN0         (PIC32MZ_BOOTCFG_K1BASE+PIC32MZ_DEVSIGN0_OFFSET)

/* Device ADC Calibration (Boot Flash PIC32MZ_ADCCALIB_K1BASE) */

#define PIC32MZ_DEVADC1          (PIC32MZ_ADCCALIB_K1BASE+PIC32MZ_DEVADC1_OFFSET)
#define PIC32MZ_DEVADC2          (PIC32MZ_ADCCALIB_K1BASE+PIC32MZ_DEVADC2_OFFSET)
#define PIC32MZ_DEVADC3          (PIC32MZ_ADCCALIB_K1BASE+PIC32MZ_DEVADC3_OFFSET)
#define PIC32MZ_DEVADC4          (PIC32MZ_ADCCALIB_K1BASE+PIC32MZ_DEVADC4_OFFSET)
#define PIC32MZ_DEVADC5          (PIC32MZ_ADCCALIB_K1BASE+PIC32MZ_DEVADC5_OFFSET)

/* Device Serial Number (Boot Flash PIC32MZ_DEVSN_K1BASEPIC32MZ_DEVSN_K1BASE) */

#define PIC32MZ_DEVSN0           (PIC32MZ_ADCCALIB_K1BASE+PIC32MZ_DEVSN0_OFFSET)
#define PIC32MZ_DEVSN1           (PIC32MZ_ADCCALIB_K1BASE+PIC32MZ_DEVSN1_OFFSET)

/* Register/Flash Bit Field Definitions *********************************************/

/* Device ID, Revision, and Configuration (SFR PIC32MZ_CONFIG_K1BASE) */
/* Configuration control register */
#define CFGCON_
/* Device ID and revision register */
#define DEVID_
/* System key register */
#define SYSKEY_
/* External bus interface address pin configuration register */
#define CFGEBIA_
/* External bus interface address pin control register */
#define CFGEBIC_
/* Permission group configuration register */
#define CFGPG_

/* Alternate Device Configuration (Boot Flash PIC32MZ_BOOTCFG_K1BASE) */
/* Alternate device configuration word 3 */
#define ADEVCFG3_
/* Alternate device configuration word 2 */
#define ADEVCFG2_
/* Alternate device configuration word 1 */
#define ADEVCFG1_
/* Alternate device configuration word 0 */
#define ADEVCFG0_
/* Alternate device code protect word 3 */
#define ADEVCP3_
/* Alternate device code protect word 2 */
#define ADEVCP2_
/* Alternate device code protect word 1 */
#define ADEVCP1_
/* Alternate device code protect word 0 */
#define ADEVCP0_
/* Alternate evice signature word 3 */
#define ADEVSIGN3_
/* Alternate evice signature word 2 */
#define ADEVSIGN2_
/* Alternate evice signature word 1 */
#define ADEVSIGN1_
/* Alternate evice signature word 0 */
#define ADEVSIGN0_

/* Device Configuration (Boot Flash PIC32MZ_BOOTCFG_K1BASE) */
/* Device configuration word 3 */
#define DEVCFG3_
/* Device configuration word 2 */
#define DEVCFG2_
/* Device configuration word 1 */
#define DEVCFG1_
/* Device configuration word 0 */
#define DEVCFG0_
/* Device code protect word 3 */
#define DEVCP3_
/* Device code protect word 2 */
#define DEVCP2_
/* Device code protect word 1 */
#define DEVCP1_
/* Device code protect word 0 */
#define DEVCP0_
/* Device signature word 3 */
#define DEVSIGN3_
/* Device signature word 2 */
#define DEVSIGN2_
/* Device signature word 1 */
#define DEVSIGN1_
/* Device signature word 0 */
#define DEVSIGN0_

/* Device ADC Calibration (Boot Flash PIC32MZ_ADCCALIB_K1BASE) */
/* ADC1 Calibration */
#define DEVADC1_
/* ADC2 Calibration */
#define DEVADC2_
/* ADC3 Calibration */
#define DEVADC3_
/* ADC4 Calibration */
#define DEVADC4_
/* ADC5 Calibration */
#define DEVADC5_

/* Device Serial Number (Boot Flash PIC32MZ_DEVSN_K1BASE) */
/* Device serial number 0 */
#define DEVSN0_OFFSET    0x0000 /* Device serial number 0 */
/* Device serial number 1 */
#define DEVSN1_OFFSET    0x0004 /* Device serial number 1 */

#endif /* __ARCH_MIPS_SRC_PIC32MZ_CHIP_PIC32MZEC_FEATURES_H */
