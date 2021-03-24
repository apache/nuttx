/****************************************************************************
 * arch/arm/src/samd5e5/hardware/sam_pac.h
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

#ifndef __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_PAC_H
#define __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_PAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PAC register offsets *****************************************************/

#define SAM_PAC_WRCTRL_OFFSET        0x0000  /* Write control */
#define SAM_PAC_EVCTRL_OFFSET        0x0004  /* Event control */
#define SAM_PAC_INTENCLR_OFFSET      0x0008  /* Interrupt enable clear */
#define SAM_PAC_INTENSET_OFFSET      0x0009  /* Interrupt enable set */
#define SAM_PAC_INTFLAGAHB_OFFSET    0x0010  /* Bridge interrupt flag status */
#define SAM_PAC_INTFLAGA_OFFSET      0x0014  /* Interrupt flag status bridge A */
#define SAM_PAC_INTFLAGB_OFFSET      0x0018  /* Interrupt flag status bridge B */
#define SAM_PAC_INTFLAGC_OFFSET      0x001c  /* Interrupt flag status bridge C */
#define SAM_PAC_INTFLAGD_OFFSET      0x0020  /* Interrupt flag status bridge D */
#define SAM_PAC_STATUSA_OFFSET       0x0034  /* Write protection status bridge A */
#define SAM_PAC_STATUSB_OFFSET       0x0038  /* Write protection status bridge B */
#define SAM_PAC_STATUSC_OFFSET       0x003c  /* Write protection status bridge C */
#define SAM_PAC_STATUSD_OFFSET       0x0040  /* Write protection status bridge D */

/* PAC register addresses ***************************************************/

#define SAM_PAC_WRCTRL               (SAM_PAC_BASE + SAM_PAC_WRCTRL_OFFSET)
#define SAM_PAC_EVCTRL               (SAM_PAC_BASE + SAM_PAC_EVCTRL_OFFSET)
#define SAM_PAC_INTENCLR             (SAM_PAC_BASE + SAM_PAC_INTENCLR_OFFSET)
#define SAM_PAC_INTENSET             (SAM_PAC_BASE + SAM_PAC_INTENSET_OFFSET)
#define SAM_PAC_INTFLAGAHB           (SAM_PAC_BASE + SAM_PAC_INTFLAGAHB_OFFSET)
#define SAM_PAC_INTFLAGA             (SAM_PAC_BASE + SAM_PAC_INTFLAGA_OFFSET)
#define SAM_PAC_INTFLAGB             (SAM_PAC_BASE + SAM_PAC_INTFLAGB_OFFSET)
#define SAM_PAC_INTFLAGC             (SAM_PAC_BASE + SAM_PAC_INTFLAGC_OFFSET)
#define SAM_PAC_INTFLAGD             (SAM_PAC_BASE + SAM_PAC_INTFLAGD_OFFSET)
#define SAM_PAC_STATUSA              (SAM_PAC_BASE + SAM_PAC_STATUSA_OFFSET)
#define SAM_PAC_STATUSB              (SAM_PAC_BASE + SAM_PAC_STATUSB_OFFSET)
#define SAM_PAC_STATUSC              (SAM_PAC_BASE + SAM_PAC_STATUSC_OFFSET)
#define SAM_PAC_STATUSD              (SAM_PAC_BASE + SAM_PAC_STATUSD_OFFSET)

/* PAC register bit definitions *********************************************/

/* Write control */

#define PAC_WRCTRL_PERID_SHIFT       (0)       /* Bits 0-15:  Peripheral Identifier */
#define PAC_WRCTRL_PERID_MASK        (0xffff << PAC_WRCTRL_PERID_SHIFT)
#  define PAC_WRCTRL_PERID(n)        ((uint32_t)(n) << PAC_WRCTRL_PERID_SHIFT)
#define PAC_WRCTRL_KEY_SHIFT         (16)      /* Bits 16-23:  Peripheral Access Control Key */
#define PAC_WRCTRL_KEY_MASK          (0xff << PAC_WRCTRL_KEY_SHIFT)
#  define PAC_WRCTRL_KEY_OFF         (0 << PAC_WRCTRL_KEY_SHIFT) /* No action */
#  define PAC_WRCTRL_KEY_CLEAR       (1 << PAC_WRCTRL_KEY_SHIFT) /* Clear the peripheral write
                                                                  * control */
#  define PAC_WRCTRL_KEY_SET         (2 << PAC_WRCTRL_KEY_SHIFT) /* Set the peripheral write
                                                                  * control */
#  define PAC_WRCTRL_KEY_LOCK        (3 << PAC_WRCTRL_KEY_SHIFT) /* Set and lock the peripheral
                                                                  * write control until the
                                                                  * next hardware reset */

/* Event control */

#define PAC_EVCTRL_ERREO             (1 << 0)  /* Bit 0: Peripheral access error event output */

/* Interrupt enable clear, Interrupt enable set */

#define PAC_INTEN_ERR                (1 << 0)  /* Bit 0:  Peripheral access error interrupt */

/* Bridge interrupt flag status */

#define PAC_INTFLAGAHB_NVMCTRL0      (1 << 0)  /* Bit 0:  Interrupt flag for NVMCTRL0 */
#define PAC_INTFLAGAHB_NVMCTRL1      (1 << 1)  /* Bit 1:  Interrupt flag for NVMCTRL1 */
#define PAC_INTFLAGAHB_NVMCTRL2      (1 << 2)  /* Bit 2:  Interrupt flag for NVMCTRL2 */
#define PAC_INTFLAGAHB_RAMCM4S       (1 << 3)  /* Bit 3:  Interrupt flag for RAMCM4S */
#define PAC_INTFLAGAHB_RAMPPPDSU     (1 << 4)  /* Bit 4:  Interrupt flag for RAMPPPDSU */
#define PAC_INTFLAGAHB_RAMDMAWR      (1 << 5)  /* Bit 5:  Interrupt flag for RAMDMAWR */
#define PAC_INTFLAGAHB_RAMDMACICM    (1 << 6)  /* Bit 6:  Interrupt flag for RAMDMACICM */
#define PAC_INTFLAGAHB_HPB0          (1 << 7)  /* Bit 7:  Interrupt flag for HPB0 */
#define PAC_INTFLAGAHB_HPB1          (1 << 8)  /* Bit 8:  Interrupt flag for HPB1 */
#define PAC_INTFLAGAHB_HPB2          (1 << 9)  /* Bit 9:  Interrupt flag for HPB2 */
#define PAC_INTFLAGAHB_HPB3          (1 << 0)  /* Bit 10: Interrupt flag for HPB3 */
#define PAC_INTFLAGAHB_PUKCC         (1 << 1)  /* Bit 11: Interrupt flag for PUKCC */
#define PAC_INTFLAGAHB_SDHC0         (1 << 2)  /* Bit 12: Interrupt flag for SDHC0 */
#define PAC_INTFLAGAHB_SDHC1         (1 << 3)  /* Bit 13: Interrupt flag for SDHC1 */
#define PAC_INTFLAGAHB_QSPI          (1 << 4)  /* Bit 14: Interrupt flag for QSPI */

/* Interrupt flag status bridge A and Write protection status bridge A */

#define PAC_INTA_PAC                 (1 << 0)  /* Bit 0:  Interrupt for PAC */
#define PAC_INTA_PM                  (1 << 1)  /* Bit 1:  Interrupt for PM */
#define PAC_INTA_MCLK                (1 << 2)  /* Bit 2:  Interrupt for MCLK */
#define PAC_INTA_RSTC                (1 << 3)  /* Bit 3:  Interrupt for RSTC */
#define PAC_INTA_OSCCTRL             (1 << 4)  /* Bit 4:  Interrupt for OSCCTRL */
#define PAC_INTA_OSC32KCTRL          (1 << 5)  /* Bit 5:  Interrupt for OSC32KCTRL */
#define PAC_INTA_SUPC                (1 << 6)  /* Bit 6:  Interrupt for SUPC */
#define PAC_INTA_GCLK                (1 << 7)  /* Bit 7:  Interrupt for GCLK */
#define PAC_INTA_WDT                 (1 << 8)  /* Bit 8:  Interrupt for WDT */
#define PAC_INTA_RTC                 (1 << 9)  /* Bit 9:  Interrupt for RTC */
#define PAC_INTA_EIC                 (1 << 10) /* Bit 10: Interrupt for EIC */
#define PAC_INTA_FREQM               (1 << 11) /* Bit 11: Interrupt for FREQM */
#define PAC_INTA_SERCOM0             (1 << 12) /* Bit 12: Interrupt for SERCOM0 */
#define PAC_INTA_SERCOM1             (1 << 13) /* Bit 13: Interrupt for SERCOM1 */
#define PAC_INTA_TC0                 (1 << 14) /* Bit 14: Interrupt for TC0 */
#define PAC_INTA_TC1                 (1 << 15) /* Bit 15: Interrupt for TC1 */

#define SAM_PAC_PERID                ((0 << 5) + 0)  /* PAC Peripheral IDs */
#define SAM_PM_PERID                 ((0 << 5) + 1)
#define SAM_MCLK_PERID               ((0 << 5) + 2)
#define SAM_RSTC_PERID               ((0 << 5) + 3)
#define SAM_OSCCTRL_PERID            ((0 << 5) + 4)
#define SAM_OSC32KCTRL_PERID         ((0 << 5) + 5)
#define SAM_SUPC_PERID               ((0 << 5) + 6)
#define SAM_GCLK_PERID               ((0 << 5) + 7)
#define SAM_WDT_PERID                ((0 << 5) + 8)
#define SAM_RTC_PERID                ((0 << 5) + 9)
#define SAM_EIC_PERID                ((0 << 5) + 10)
#define SAM_FREQM_PERID              ((0 << 5) + 11)
#define SAM_SERCOM0_PERID            ((0 << 5) + 12)
#define SAM_SERCOM1_PERID            ((0 << 5) + 13)
#define SAM_TC0_PERID                ((0 << 5) + 14)
#define SAM_TC1_PERID                ((0 << 5) + 15)

/* Interrupt flag status bridge B and Write protection status bridge B */

#define PAC_INTB_USB                 (1 << 0)  /* Bit 0:  Interrupt for USB */
#define PAC_INTB_DSU                 (1 << 1)  /* Bit 1:  Interrupt for DSU */
#define PAC_INTB_CMCC                (1 << 2)  /* Bit 3:  Interrupt for CMCC */
#define PAC_INTB_NVMCTRL             (1 << 3)  /* Bit 2:  Interrupt for NVMCTRL */
#define PAC_INTB_PORT                (1 << 4)  /* Bit 4:  Interrupt for PORT */
#define PAC_INTB_DMAC                (1 << 5)  /* Bit 5:  Interrupt for DMAC */
#define PAC_INTB_EVSYS               (1 << 7)  /* Bit 7:  Interrupt for EVSYS */
#define PAC_INTB_SERCOM2             (1 << 9)  /* Bit 9:  Interrupt for SERCOM2 */
#define PAC_INTB_SERCOM3             (1 << 10) /* Bit 10: Interrupt for SERCOM3 */
#define PAC_INTB_TCC0                (1 << 11) /* Bit 11: Interrupt for TCC0 */
#define PAC_INTB_TCC1                (1 << 12) /* Bit 12: Interrupt for TCC1 */
#define PAC_INTB_TC2                 (1 << 13) /* Bit 13: Interrupt for TC2 */
#define PAC_INTB_TC3                 (1 << 14) /* Bit 14: Interrupt for TC3 */
#define PAC_INTB_RAMECC              (1 << 16) /* Bit 16: Interrupt for RAMECC */

#define SAM_USB_PERID                ((1 << 5) + 0)  /* PAC Peripheral IDs */
#define SAM_DSU_PERID                ((1 << 5) + 1)
#define SAM_NVMCTRL_PERID            ((1 << 5) + 2)
#define SAM_CMCCC_PERID              ((1 << 5) + 3)
#define SAM_PORT_PERID               ((1 << 5) + 4)
#define SAM_DMAC_PERID               ((1 << 5) + 5)
#define SAM_EVSYS_PERID              ((1 << 5) + 7)
#define SAM_SERCOM2_PERID            ((1 << 5) + 9)
#define SAM_SERCOM3_PERID            ((1 << 5) + 10)
#define SAM_TCC0_PERID               ((1 << 5) + 11)
#define SAM_TCC1_PERID               ((1 << 5) + 12)
#define SAM_TC2_PERID                ((1 << 5) + 13)
#define SAM_TC3_PERID                ((1 << 5) + 14)
#define SAM_RAMECC_PERID             ((1 << 5) + 16)

/* Interrupt flag status bridge C and Write protection status bridge C */

#define PAC_INTC_CAN0                (1 << 0)  /* Bit 0:  Interrupt for CAN0 */
#define PAC_INTC_CAN1                (1 << 1)  /* Bit 1:  Interrupt for CAN1 */
#define PAC_INTC_GMAC                (1 << 2)  /* Bit 2:  Interrupt for GMAC */
#define PAC_INTC_TCC2                (1 << 3)  /* Bit 3:  Interrupt for TCC2 */
#define PAC_INTC_TCC3                (1 << 4)  /* Bit 4:  Interrupt for TCC3 */
#define PAC_INTC_TC4                 (1 << 5)  /* Bit 5:  Interrupt for TC4 */
#define PAC_INTC_TC5                 (1 << 6)  /* Bit 6:  Interrupt for TC5 */
#define PAC_INTC_PDEC                (1 << 7)  /* Bit 7:  Interrupt for PDEC */
#define PAC_INTC_AES                 (1 << 9)  /* Bit 9:  Interrupt for AES */
#define PAC_INTC_TRNG                (1 << 10) /* Bit 10: Interrupt for TRNG */
#define PAC_INTC_ICM                 (1 << 11) /* Bit 11: Interrupt for ICM */
#define PAC_INTC_PUKCC               (1 << 12) /* Bit 12: Interrupt for PUKCC */
#define PAC_INTC_QSPI                (1 << 13) /* Bit 13: Interrupt for QSPI */
#define PAC_INTC_CCL                 (1 << 14) /* Bit 14: Interrupt for CCL */

#define SAM_CAN0_PERID               ((2 << 5) + 0)  /* PAC Peripheral IDs */
#define SAM_CAN1_PERID               ((2 << 5) + 1)
#define SAM_GMAC_PERID               ((2 << 5) + 2)
#define SAM_TCC2_PERID               ((2 << 5) + 3)
#define SAM_TCC3_PERID               ((2 << 5) + 4)
#define SAM_TC4_PERID                ((2 << 5) + 5)
#define SAM_TC5_PERID                ((2 << 5) + 6)
#define SAM_PDEC_PERID               ((2 << 5) + 7)
#define SAM_AC_PERID                 ((2 << 5) + 8)
#define SAM_AES_PERID                ((2 << 5) + 9)
#define SAM_TRNG_PERID               ((2 << 5) + 10)
#define SAM_ICM_PERID                ((2 << 5) + 11)
#define SAM_PUKCC_PERID              ((2 << 5) + 12)
#define SAM_QSPIC_PERID              ((2 << 5) + 13)
#define SAM_CCL_PERID                ((2 << 5) + 14)

/* Interrupt flag status bridge D and Write protection status bridge D */

#define PAC_INTD_SERCOM4             (1 << 0)  /* Bit 0:  Interrupt for SERCOM4 */
#define PAC_INTD_SERCOM5             (1 << 1)  /* Bit 1:  Interrupt for SERCOM5 */
#define PAC_INTD_SERCOM6             (1 << 2)  /* Bit 2:  Interrupt for SERCOM6 */
#define PAC_INTD_SERCOM7             (1 << 3)  /* Bit 3:  Interrupt for SERCOM7 */
#define PAC_INTD_TCC4                (1 << 4)  /* Bit 4:  Interrupt for TCC4 */
#define PAC_INTD_TC6                 (1 << 5)  /* Bit 5:  Interrupt for TC6 */
#define PAC_INTD_TC7                 (1 << 6)  /* Bit 6:  Interrupt for TC7 */
#define PAC_INTD_ADC0                (1 << 7)  /* Bit 7:  Interrupt for ADC0 */
#define PAC_INTD_ADC1                (1 << 8)  /* Bit 8:  Interrupt for ADC1 */
#define PAC_INTD_DAC                 (1 << 9)  /* Bit 9:  Interrupt for DAC */
#define PAC_INTD_I2S                 (1 << 10) /* Bit 10: Interrupt for I2S */
#define PAC_INTD_PCC                 (1 << 11) /* Bit 11: Interrupt for PCC */

#define SAM_SERCOM4_PERID            ((3 << 5) + 0)  /* PAC Peripheral IDs */
#define SAM_SERCOM5_PERID            ((3 << 5) + 1)
#define SAM_SERCOM6_PERID            ((3 << 5) + 2)
#define SAM_SERCOM7_PERID            ((3 << 5) + 3)
#define SAM_TCC4_PERID               ((3 << 5) + 4)
#define SAM_TC6_PERID                ((3 << 5) + 5)
#define SAM_TC7_PERID                ((3 << 5) + 6)
#define SAM_ADC0_PERID               ((3 << 5) + 7)
#define SAM_ADC1_PERID               ((3 << 5) + 8)
#define SAM_DAC_PERID                ((3 << 5) + 9)
#define SAM_I2S_PERID                ((3 << 5) + 10)
#define SAM_PCC_PERID                ((3 << 5) + 11)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_PAC_H */
