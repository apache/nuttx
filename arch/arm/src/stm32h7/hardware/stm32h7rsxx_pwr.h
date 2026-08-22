/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32h7rsxx_pwr.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7RSXX_PWR_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7RSXX_PWR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "stm32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_PWR_CR1_OFFSET           0x0000 /* Power control 1 */
#define STM32_PWR_SR1_OFFSET           0x0004 /* Power status 1 */
#define STM32_PWR_CSR1_OFFSET          0x0008 /* Control/status 1 */
#define STM32_PWR_CSR2_OFFSET          0x000c /* Control/status 2 */
#define STM32_PWR_CSR3_OFFSET          0x0010 /* Control/status 3 */
#define STM32_PWR_CSR4_OFFSET          0x0014 /* Control/status 4 */
#define STM32_PWR_WKUPCR_OFFSET        0x0020 /* Wakeup clear */
#define STM32_PWR_WKUPFR_OFFSET        0x0024 /* Wakeup flags */
#define STM32_PWR_WKUPEPR_OFFSET       0x0028 /* Wakeup enable/polarity */
#define STM32_PWR_UCPDR_OFFSET         0x002c /* USB Type-C power delivery */
#define STM32_PWR_APCR_OFFSET          0x0030 /* Apply pull configuration */
#define STM32_PWR_PUCRN_OFFSET         0x0034 /* Port N pull-up control */
#define STM32_PWR_PDCRN_OFFSET         0x0038 /* Port N pull-down control */
#define STM32_PWR_PUCRO_OFFSET         0x003c /* Port O pull-up control */
#define STM32_PWR_PDCRO_OFFSET         0x0040 /* Port O pull-down control */
#define STM32_PWR_PDCRP_OFFSET         0x0044 /* Port P pull-down control */
#define STM32_PWR_PDR1_OFFSET          0x0050 /* Debug register 1 */

/* Register Addresses *******************************************************/

#define STM32_PWR_CR1                  (STM32_PWR_BASE + STM32_PWR_CR1_OFFSET)
#define STM32_PWR_SR1                  (STM32_PWR_BASE + STM32_PWR_SR1_OFFSET)
#define STM32_PWR_CSR1                 (STM32_PWR_BASE + STM32_PWR_CSR1_OFFSET)
#define STM32_PWR_CSR2                 (STM32_PWR_BASE + STM32_PWR_CSR2_OFFSET)
#define STM32_PWR_CSR3                 (STM32_PWR_BASE + STM32_PWR_CSR3_OFFSET)
#define STM32_PWR_CSR4                 (STM32_PWR_BASE + STM32_PWR_CSR4_OFFSET)
#define STM32_PWR_WKUPCR               (STM32_PWR_BASE + STM32_PWR_WKUPCR_OFFSET)
#define STM32_PWR_WKUPFR               (STM32_PWR_BASE + STM32_PWR_WKUPFR_OFFSET)
#define STM32_PWR_WKUPEPR              (STM32_PWR_BASE + STM32_PWR_WKUPEPR_OFFSET)
#define STM32_PWR_UCPDR                (STM32_PWR_BASE + STM32_PWR_UCPDR_OFFSET)
#define STM32_PWR_APCR                 (STM32_PWR_BASE + STM32_PWR_APCR_OFFSET)
#define STM32_PWR_PUCRN                (STM32_PWR_BASE + STM32_PWR_PUCRN_OFFSET)
#define STM32_PWR_PDCRN                (STM32_PWR_BASE + STM32_PWR_PDCRN_OFFSET)
#define STM32_PWR_PUCRO                (STM32_PWR_BASE + STM32_PWR_PUCRO_OFFSET)
#define STM32_PWR_PDCRO                (STM32_PWR_BASE + STM32_PWR_PDCRO_OFFSET)
#define STM32_PWR_PDCRP                (STM32_PWR_BASE + STM32_PWR_PDCRP_OFFSET)
#define STM32_PWR_PDR1                 (STM32_PWR_BASE + STM32_PWR_PDR1_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Power control register 1 */

#define PWR_CR1_SVOS                   (1 << 0)  /* Stop-mode voltage scaling */
#define PWR_CR1_PVDE                   (1 << 4)  /* Voltage detector enable */
#define PWR_CR1_PLS_SHIFT              (5)       /* PVD level selection */
#define PWR_CR1_PLS_MASK               (7 << PWR_CR1_PLS_SHIFT)
#  define PWR_CR1_PLS(n)               ((n) << PWR_CR1_PLS_SHIFT)
#define PWR_CR1_DBP                    (1 << 8)  /* Backup-domain access */
#define PWR_CR1_FLPS                   (1 << 9)  /* Flash low-power mode */
#define PWR_CR1_BOOSTE                 (1 << 11) /* Analog switch boost */
#define PWR_CR1_AVDREADY               (1 << 12) /* Analog voltage ready */
#define PWR_CR1_AVDEN                  (1 << 13) /* Analog detector enable */
#define PWR_CR1_ALS_SHIFT              (14)      /* AVD level selection */
#define PWR_CR1_ALS_MASK               (3 << PWR_CR1_ALS_SHIFT)
#  define PWR_CR1_ALS(n)               ((n) << PWR_CR1_ALS_SHIFT)

/* Power status register 1 */

#define PWR_SR1_ACTVOS                 (1 << 0)  /* Applied voltage scale */
#define PWR_SR1_ACTVOSRDY              (1 << 1)  /* Applied scale ready */
#define PWR_SR1_PVDO                   (1 << 4)  /* Voltage detector output */
#define PWR_SR1_AVDO                   (1 << 13) /* Analog detector output */

/* Power control/status register 1 */

#define PWR_CSR1_BREN                  (1 << 0)  /* Backup regulator enable */
#define PWR_CSR1_MONEN                 (1 << 4)  /* VBAT monitor enable */
#define PWR_CSR1_BRRDY                 (1 << 16) /* Backup regulator ready */
#define PWR_CSR1_VBATL                 (1 << 20) /* VBAT above low level */
#define PWR_CSR1_VBATH                 (1 << 21) /* VBAT above high level */
#define PWR_CSR1_TEMPL                 (1 << 22) /* Temperature above low level */
#define PWR_CSR1_TEMPH                 (1 << 23) /* Temperature above high level */

/* Power control/status register 2 */

#define PWR_CSR2_BYPASS                (1 << 0)  /* PMU bypass */
#define PWR_CSR2_LDOEN                 (1 << 1)  /* LDO enable */
#define PWR_CSR2_SDEN                  (1 << 2)  /* SMPS enable */
#define PWR_CSR2_SMPSEXTHP             (1 << 3)  /* SMPS external supply */
#define PWR_CSR2_SDHILEVEL             (1 << 4)  /* SMPS high level */
#define PWR_CSR2_VBE                   (1 << 8)  /* VBAT charging enable */
#define PWR_CSR2_VBRS                  (1 << 9)  /* VBAT charging resistor */
#define PWR_CSR2_XSPICAP1_SHIFT        (10)      /* XSPI1 capacitor value */
#define PWR_CSR2_XSPICAP1_MASK         (3 << PWR_CSR2_XSPICAP1_SHIFT)
#  define PWR_CSR2_XSPICAP1(n)         ((n) << PWR_CSR2_XSPICAP1_SHIFT)
#define PWR_CSR2_XSPICAP2_SHIFT        (12)      /* XSPI2 capacitor value */
#define PWR_CSR2_XSPICAP2_MASK         (3 << PWR_CSR2_XSPICAP2_SHIFT)
#  define PWR_CSR2_XSPICAP2(n)         ((n) << PWR_CSR2_XSPICAP2_SHIFT)
#define PWR_CSR2_EN_XSPIM1             (1 << 14) /* XSPIM1 supply enable */
#define PWR_CSR2_EN_XSPIM2             (1 << 15) /* XSPIM2 supply enable */
#define PWR_CSR2_SDEXTRDY              (1 << 16) /* SMPS supply ready */
#define PWR_CSR2_USB33DEN              (1 << 24) /* USB detector enable */
#define PWR_CSR2_USBREGEN              (1 << 25) /* USB regulator enable */
#define PWR_CSR2_USB33RDY              (1 << 26) /* USB supply ready */
#define PWR_CSR2_USBHSREGEN            (1 << 27) /* USB HS regulator enable */

/* Power control/status registers 3 and 4 */

#define PWR_CSR3_PDDS                  (1 << 0) /* Deep-sleep power down */
#define PWR_CSR3_CSSF                  (1 << 1) /* Clear stop/standby flags */
#define PWR_CSR3_STOPF                 (1 << 8) /* Stop flag */
#define PWR_CSR3_SBF                   (1 << 9) /* Standby flag */
#define PWR_CSR4_VOS                   (1 << 0) /* Voltage scaling selection */
#define PWR_CSR4_VOSRDY                (1 << 1) /* Voltage scaling ready */

/* Wakeup registers */

#define PWR_WKUP_PIN(n)                (1 << ((n) - 1))
#define PWR_WKUPCR_WKUPC(n)            PWR_WKUP_PIN(n)
#define PWR_WKUPFR_WKUPF(n)            PWR_WKUP_PIN(n)
#define PWR_WKUPEPR_WKUPEN(n)          PWR_WKUP_PIN(n)
#define PWR_WKUPEPR_WKUPP(n)           (1 << ((n) + 7))
#define PWR_WKUPEPR_WKUPPUPD_SHIFT(n)  (14 + ((n) << 1))
#define PWR_WKUPEPR_WKUPPUPD_MASK(n)   (3 << PWR_WKUPEPR_WKUPPUPD_SHIFT(n))
#define PWR_WKUPEPR_WKUPPUPD(n,v)      ((v) << PWR_WKUPEPR_WKUPPUPD_SHIFT(n))

#define PWR_WKUPCR_WKUPC1              PWR_WKUPCR_WKUPC(1)
#define PWR_WKUPCR_WKUPC2              PWR_WKUPCR_WKUPC(2)
#define PWR_WKUPCR_WKUPC3              PWR_WKUPCR_WKUPC(3)
#define PWR_WKUPCR_WKUPC4              PWR_WKUPCR_WKUPC(4)
#define PWR_WKUPFR_WKUPF1              PWR_WKUPFR_WKUPF(1)
#define PWR_WKUPFR_WKUPF2              PWR_WKUPFR_WKUPF(2)
#define PWR_WKUPFR_WKUPF3              PWR_WKUPFR_WKUPF(3)
#define PWR_WKUPFR_WKUPF4              PWR_WKUPFR_WKUPF(4)
#define PWR_WKUPEPR_WKUPEN1            PWR_WKUPEPR_WKUPEN(1)
#define PWR_WKUPEPR_WKUPEN2            PWR_WKUPEPR_WKUPEN(2)
#define PWR_WKUPEPR_WKUPEN3            PWR_WKUPEPR_WKUPEN(3)
#define PWR_WKUPEPR_WKUPEN4            PWR_WKUPEPR_WKUPEN(4)
#define PWR_WKUPEPR_WKUPP1             PWR_WKUPEPR_WKUPP(1)
#define PWR_WKUPEPR_WKUPP2             PWR_WKUPEPR_WKUPP(2)
#define PWR_WKUPEPR_WKUPP3             PWR_WKUPEPR_WKUPP(3)
#define PWR_WKUPEPR_WKUPP4             PWR_WKUPEPR_WKUPP(4)

/* USB Type-C and apply-pull registers */

#define PWR_UCPDR_UCPD_DBDIS           (1 << 0)  /* Dead-battery disable */
#define PWR_UCPDR_UCPD_STBY            (1 << 1)  /* UCPD standby mode */
#define PWR_APCR_APC                   (1 << 0)  /* Apply pull configuration */
#define PWR_APCR_PN7_PUPD              (1 << 16) /* PN7 pull configuration */
#define PWR_APCR_PO5_PUPD              (1 << 17) /* PO5 pull configuration */
#define PWR_APCR_I3CPB6_PU             (1 << 28) /* PB6 I3C pull-up */
#define PWR_APCR_I3CPB7_PU             (1 << 29) /* PB7 I3C pull-up */
#define PWR_APCR_I3CPB8_PU             (1 << 30) /* PB8 I3C pull-up */
#define PWR_APCR_I3CPB9_PU             (1 << 31) /* PB9 I3C pull-up */

/* Standby pull registers */

#define PWR_PUCRN_PUN1                 (1 << 1)  /* PN1 pull-up */
#define PWR_PUCRN_PUN6                 (1 << 6)  /* PN6 pull-up */
#define PWR_PUCRN_PUN12                (1 << 12) /* PN12 pull-up */
#define PWR_PDCRN_PDN0                 (1 << 0)  /* PN0 pull-down */
#define PWR_PDCRN_PDN1                 (1 << 1)  /* PN1 pull-down */
#define PWR_PDCRN_PDN2N5               (1 << 2)  /* PN2-PN5 pull-down */
#define PWR_PDCRN_PDN6                 (1 << 6)  /* PN6 pull-down */
#define PWR_PDCRN_PDN8N11              (1 << 8)  /* PN8-PN11 pull-down */
#define PWR_PDCRN_PDN12                (1 << 12) /* PN12 pull-down */
#define PWR_PUCRO_PUO0                 (1 << 0)  /* PO0 pull-up */
#define PWR_PUCRO_PUO1                 (1 << 1)  /* PO1 pull-up */
#define PWR_PUCRO_PUO4                 (1 << 4)  /* PO4 pull-up */
#define PWR_PDCRO_PDO0                 (1 << 0)  /* PO0 pull-down */
#define PWR_PDCRO_PDO1                 (1 << 1)  /* PO1 pull-down */
#define PWR_PDCRO_PDO2                 (1 << 2)  /* PO2 pull-down */
#define PWR_PDCRO_PDO3                 (1 << 3)  /* PO3 pull-down */
#define PWR_PDCRO_PDO4                 (1 << 4)  /* PO4 pull-down */
#define PWR_PDCRP_PDP0P3               (1 << 0)  /* PP0-PP3 pull-down */
#define PWR_PDCRP_PDP4P7               (1 << 4)  /* PP4-PP7 pull-down */
#define PWR_PDCRP_PDP8P11              (1 << 8)  /* PP8-PP11 pull-down */
#define PWR_PDCRP_PDP12P15             (1 << 12) /* PP12-PP15 pull-down */

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7RSXX_PWR_H */
