/************************************************************************************
 * arch/arm/src/samd2l2/hardware/saml_mclk.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_MCLK_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_MCLK_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAML21

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* MCLK register offsets ************************************************************/

#define SAM_MCLK_CTRLA_OFFSET      0x0000  /* CTRLA register */
#define SAM_MCLK_INTENCLR_OFFSET   0x0001  /* Interrupt enable clear */
#define SAM_MCLK_INTENSET_OFFSET   0x0002  /* Interrupt enable set */
#define SAM_MCLK_INTFLAG_OFFSET    0x0003  /* Interrupt flag status and clear */
#define SAM_MCLK_CPUDIV_OFFSET     0x0004  /* CPU clock division */
#define SAM_MCLK_LPDIV_OFFSET      0x0005  /* Low-power clock division */
#define SAM_MCLK_BUPDIV_OFFSET     0x0006  /* Backup clock division */
                                           /* 0x0007-0x000f: Reserved */
#define SAM_MCLK_AHBMASK_OFFSET    0x0010  /* AHB mask */
#define SAM_MCLK_APBAMASK_OFFSET   0x0014  /* APBA mask */
#define SAM_MCLK_APBBMASK_OFFSET   0x0018  /* APBB mask */
#define SAM_MCLK_APBCMASK_OFFSET   0x001c  /* APBC mask */
#define SAM_MCLK_APBDMASK_OFFSET   0x0020  /* APBD mask */
#define SAM_MCLK_APBEMASK_OFFSET   0x0024  /* APBE mask */

/* MCLK register addresses **********************************************************/

#define SAM_MCLK_CTRLA             (SAM_MCLK_BASE+SAM_MCLK_CTRLA_OFFSET)
#define SAM_MCLK_INTENCLR          (SAM_MCLK_BASE+SAM_MCLK_INTENCLR_OFFSET)
#define SAM_MCLK_INTENSET          (SAM_MCLK_BASE+SAM_MCLK_INTENSET_OFFSET)
#define SAM_MCLK_INTFLAG           (SAM_MCLK_BASE+SAM_MCLK_INTFLAG_OFFSET)
#define SAM_MCLK_CPUDIV            (SAM_MCLK_BASE+SAM_MCLK_CPUDIV_OFFSET)
#define SAM_MCLK_LPDIV             (SAM_MCLK_BASE+SAM_MCLK_LPDIV_OFFSET)
#define SAM_MCLK_BUPDIV            (SAM_MCLK_BASE+SAM_MCLK_BUPDIV_OFFSET)

#define SAM_MCLK_AHBMASK           (SAM_MCLK_BASE+SAM_MCLK_AHBMASK_OFFSET)
#define SAM_MCLK_APBAMASK          (SAM_MCLK_BASE+SAM_MCLK_APBAMASK_OFFSET)
#define SAM_MCLK_APBBMASK          (SAM_MCLK_BASE+SAM_MCLK_APBBMASK_OFFSET)
#define SAM_MCLK_APBCMASK          (SAM_MCLK_BASE+SAM_MCLK_APBCMASK_OFFSET)
#define SAM_MCLK_APBDMASK          (SAM_MCLK_BASE+SAM_MCLK_APBDMASK_OFFSET)
#define SAM_MCLK_APBEMASK          (SAM_MCLK_BASE+SAM_MCLK_APBEMASK_OFFSET)

/* MCLK register bit definitions ****************************************************/

/* CTRLA register */

#define MCLK_CTRLA_CFDEN           (1 << 2)  /* Bit 2: Clock Failure Detector Enable */
#define MCLK_CTRLA_EMCLK           (1 << 4)  /* Bit 4: Emergency Clock Select */

/* Interrupt enable clear, Interrupt enable set, and  Interrupt flag status and
 * clear.
 */

#define MCLK_INT_CKRDY             (1 << 0)  /* Bit 0:  Clock ready */

/* CPU clock division (8-bit divider) */

#define MCLK_CPUDIV_DIV1           0x01
#define MCLK_CPUDIV_DIV2           0x02
#define MCLK_CPUDIV_DIV4           0x04
#define MCLK_CPUDIV_DIV8           0x08
#define MCLK_CPUDIV_DIV16          0x10
#define MCLK_CPUDIV_DIV32          0x20
#define MCLK_CPUDIV_DIV64          0x40
#define MCLK_CPUDIV_DIV128         0x80

/* Low-power clock division (8-bit divider) */

#define MCLK_LPDIV_DIV1            0x01
#define MCLK_LPDIV_DIV2            0x02
#define MCLK_LPDIV_DIV4            0x04
#define MCLK_LPDIV_DIV8            0x08
#define MCLK_LPDIV_DIV16           0x10
#define MCLK_LPDIV_DIV32           0x20
#define MCLK_LPDIV_DIV64           0x40
#define MCLK_LPDIV_DIV128          0x80

/* Backup clock division (8-bit divider) */

#define MCLK_BUPDIV_DIV1           0x01
#define MCLK_BUPDIV_DIV2           0x02
#define MCLK_BUPDIV_DIV4           0x04
#define MCLK_BUPDIV_DIV8           0x08
#define MCLK_BUPDIV_DIV16          0x10
#define MCLK_BUPDIV_DIV32          0x20
#define MCLK_BUPDIV_DIV64          0x40
#define MCLK_BUPDIV_DIV128         0x80

/* AHB mask */

#define MCLK_AHBMASK_APBA          (1 << 0)  /* Bit 0:  APBA AHB clock enable */
#define MCLK_AHBMASK_APBB          (1 << 1)  /* Bit 1:  APBB AHB clock enable */
#define MCLK_AHBMASK_APBC          (1 << 2)  /* Bit 2:  APBC AHB clock enable */
#define MCLK_AHBMASK_APBD          (1 << 3)  /* Bit 3:  APBD AHB clock enable */
#define MCLK_AHBMASK_APBE          (1 << 4)  /* Bit 4:  APBE AHB clock enable */
#define MCLK_AHBMASK_DSU           (1 << 5)  /* Bit 5:  DSU AHB clock enable */
#define MCLK_AHBMASK_NVMCTRL       (1 << 8)  /* Bit 8:  NVMCTRL AHB clock enable */
#define MCLK_AHBMASK_DMAC          (1 << 11) /* Bit 11: DMAC AHB clock enable */
#define MCLK_AHBMASK_USB           (1 << 12) /* Bit 12: USB AHB clock enable */
#define MCLK_AHBMASK_PAC           (1 << 14) /* Bit 14: PAC AHB clock enable */

/* APBA mask */

#define MCLK_APBAMASK_PM           (1 << 0)  /* Bit 0:  PM APBA clock enable */
#define MCLK_APBAMASK_MCLK         (1 << 1)  /* Bit 1:  MCLK APBA clock enable */
#define MCLK_APBAMASK_RSTC         (1 << 2)  /* Bit 2:  RSTC APBA clock enable */
#define MCLK_APBAMASK_OSCCTRL      (1 << 3)  /* Bit 3:  OSCCTRL APBA clock enable */
#define MCLK_APBAMASK_OSC32KCTRL   (1 << 4)  /* Bit 4:  OSC32KCTRL APBA clock enable */
#define MCLK_APBAMASK_SUPC         (1 << 5)  /* Bit 5:  SUPC APBA clock enable */
#define MCLK_APBAMASK_GCLK         (1 << 6)  /* Bit 6:  GCLK APBA clock enable */
#define MCLK_APBAMASK_WDT          (1 << 7)  /* Bit 7:  WDT APBA clock enable */
#define MCLK_APBAMASK_RTC          (1 << 8)  /* Bit 8:  RTC APBA clock enable */
#define MCLK_APBAMASK_EIC          (1 << 9)  /* Bit 9:  EIC APBA clock enable */
#define MCLK_APBAMASK_PORT         (1 << 10) /* Bit 10: PORT APBA clock enable */

/* APBB mask */

#define MCLK_APBBMASK_USB          (1 << 0)  /* Bit 0:  USB APBB clock enable */
#define MCLK_APBBMASK_DSU          (1 << 1)  /* Bit 1:  DSU APBB clock enable */
#define MCLK_APBBMASK_NVMCTRL      (1 << 2)  /* Bit 2:  NVMCTRL APBB clock enable */

/* APBC mask */

#define MCLK_APBCMASK_SERCOM(n)    (1 << (n))  /* Bit n:  SERCOMn APBC clock enable, n=0-4 */
#  define MCLK_APBCMASK_SERCOM0    (1 << 0)  /* Bit 0:  SERCOM0 APBC clock enable */
#  define MCLK_APBCMASK_SERCOM1    (1 << 1)  /* Bit 1:  SERCOM1 APBC clock enable */
#  define MCLK_APBCMASK_SERCOM2    (1 << 2)  /* Bit 2:  SERCOM2 APBC clock enable */
#  define MCLK_APBCMASK_SERCOM3    (1 << 3)  /* Bit 3:  SERCOM3 APBC clock enable */
#  define MCLK_APBCMASK_SERCOM4    (1 << 4)  /* Bit 4:  SERCOM4 APBC clock enable */
#define MCLK_APBCMASK_TCC0         (1 << 5)  /* Bit 5:  TCC0 APBC clock enable */
#define MCLK_APBCMASK_TCC1         (1 << 6)  /* Bit 6:  TCC1 APBC clock enable */
#define MCLK_APBCMASK_TCC2         (1 << 7)  /* Bit 7:  TCC2 APBC clock enable */
#define MCLK_APBCMASK_TC0          (1 << 8)  /* Bit 8:  TC0 APBC clock enable */
#define MCLK_APBCMASK_TC1          (1 << 9)  /* Bit 9:  TC1 APBC clock enable */
#define MCLK_APBCMASK_TC2          (1 << 10) /* Bit 10: TC2 APBC clock enable */
#define MCLK_APBCMASK_TC3          (1 << 11) /* Bit 11: TC3 APBC clock enable */
#define MCLK_APBCMASK_DAC          (1 << 12) /* Bit 12: DAC APBC clock enable */
#define MCLK_APBCMASK_AES          (1 << 13) /* Bit 13: AES APBC clock enable */
#define MCLK_APBCMASK_TRNG         (1 << 14) /* Bit 14: TRNG APBC clock enable */

/* APBD mask */

#define MCLK_APBDMASK_EVSYS        (1 << 0)  /* Bit 0:  EVSYS APBD clock enable */
#define MCLK_APBDMASK_SERCOM5      (1 << 1)  /* Bit 1:  SERCOM5 APBD clock enable */
#define MCLK_APBDMASK_TC4          (1 << 2)  /* Bit 2:  TC4 APBD clock enable */
#define MCLK_APBDMASK_ADC          (1 << 3)  /* Bit 3:  ADC APBD clock enable */
#define MCLK_APBDMASK_AC           (1 << 4)  /* Bit 4:  AC APBD clock enable */
#define MCLK_APBDMASK_PTC          (1 << 5)  /* Bit 5:  PTC APBD clock enable */
#define MCLK_APBDMASK_OPAMP        (1 << 6)  /* Bit 6:  OpAmp APBD clock enable */
#define MCLK_APBDMASK_CCL          (1 << 7)  /* Bit 7:  CCL APBD clock enable */

/* APBE mask */

#define MCLK_APBEMASK_PAC          (1 << 0)  /* Bit 0:  PAC APBE clock enable */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAML21 */
#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_MCLK_H */
