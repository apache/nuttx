/************************************************************************************
 * arch/arm/src/samd5e5/chip/sam_mclk.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAMD5E5_CHIP_SAM_MCLK_H
#define __ARCH_ARM_SRC_SAMD5E5_CHIP_SAM_MCLK_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip/sam_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* MCLK register offsets ************************************************************/

#define SAM_MCLK_CTRLA_OFFSET      0x0000  /* CTRLA register */
#define SAM_MCLK_INTENCLR_OFFSET   0x0001  /* Interrupt enable clear */
#define SAM_MCLK_INTENSET_OFFSET   0x0002  /* Interrupt enable set */
#define SAM_MCLK_INTFLAG_OFFSET    0x0003  /* Interrupt flag status and clear */
#define SAM_MCLK_HSDIV_OFFSET      0x0004  /* High-Speed Clock Division */
#define SAM_MCLK_CPUDIV_OFFSET     0x0005  /* CPU clock division */
                                           /* 0x0006-0x000f: Reserved */
#define SAM_MCLK_AHBMASK_OFFSET    0x0010  /* AHB mask */
#define SAM_MCLK_APBAMASK_OFFSET   0x0014  /* APBA mask */
#define SAM_MCLK_APBBMASK_OFFSET   0x0018  /* APBB mask */
#define SAM_MCLK_APBCMASK_OFFSET   0x001c  /* APBC mask */
#define SAM_MCLK_APBDMASK_OFFSET   0x0020  /* APBD mask */

/* MCLK register addresses **********************************************************/

#define SAM_MCLK_CTRLA             (SAM_MCLK_BASE + SAM_MCLK_CTRLA_OFFSET)
#define SAM_MCLK_INTENCLR          (SAM_MCLK_BASE + SAM_MCLK_INTENCLR_OFFSET)
#define SAM_MCLK_INTENSET          (SAM_MCLK_BASE + SAM_MCLK_INTENSET_OFFSET)
#define SAM_MCLK_INTFLAG           (SAM_MCLK_BASE + SAM_MCLK_INTFLAG_OFFSET)
#define SAM_MCLK_HSDIV             (SAM_MCLK_BASE + SAM_MCLK_HSDIV_OFFSET)
#define SAM_MCLK_CPUDIV            (SAM_MCLK_BASE + SAM_MCLK_CPUDIV_OFFSET)
#define SAM_MCLK_AHBMASK           (SAM_MCLK_BASE + SAM_MCLK_AHBMASK_OFFSET)
#define SAM_MCLK_APBAMASK          (SAM_MCLK_BASE + SAM_MCLK_APBAMASK_OFFSET)
#define SAM_MCLK_APBBMASK          (SAM_MCLK_BASE + SAM_MCLK_APBBMASK_OFFSET)
#define SAM_MCLK_APBCMASK          (SAM_MCLK_BASE + SAM_MCLK_APBCMASK_OFFSET)
#define SAM_MCLK_APBDMASK          (SAM_MCLK_BASE + SAM_MCLK_APBDMASK_OFFSET)

/* MCLK register bit definitions ****************************************************/

/* CTRLA register -- All bits are reserved (?) */

/* Interrupt enable clear, Interrupt enable set, and  Interrupt flag status and
 * clear.
 */

#define MCLK_INT_CKRDY             (1 << 0)  /* Bit 0:  Clock ready */

/* High-Speed Clock Division (8-bit value) */

/* CPU clock division (8-bit divider) */

#define MCLK_CPUDIV_DIV1           0x01
#define MCLK_CPUDIV_DIV2           0x02
#define MCLK_CPUDIV_DIV4           0x04
#define MCLK_CPUDIV_DIV8           0x08
#define MCLK_CPUDIV_DIV16          0x10
#define MCLK_CPUDIV_DIV32          0x20
#define MCLK_CPUDIV_DIV64          0x40
#define MCLK_CPUDIV_DIV128         0x80

/* AHB mask */

#define MCLK_AHBMASK_HPB0          (1 << 0)  /* Bit 0:  AHB HPB0 clock for enable */
#define MCLK_AHBMASK_HPB1          (1 << 1)  /* Bit 1:  AHB HPB1 clock enable */
#define MCLK_AHBMASK_HPB2          (1 << 2)  /* Bit 2:  AHB HPB2 clock enable */
#define MCLK_AHBMASK_HPB3          (1 << 3)  /* Bit 3:  AHB HPB3 clock enable */
#define MCLK_AHBMASK_DSU           (1 << 4)  /* Bit 4:  DSU AHB clock enable */
                                             /* Bit 5:  Reserved */
#define MCLK_AHBMASK_NVMCTRL       (1 << 6)  /* Bit 6:  NVMCTRL AHB clock enable */
                                             /* Bit 7:  Reserved */
#define MCLK_AHBMASK_CMCC          (1 << 8)  /* Bit 8:  CMCC AHB Clock Enable */
#define MCLK_AHBMASK_DMAC          (1 << 9)  /* Bit 9:  DMAC AHB clock enable */
#define MCLK_AHBMASK_USB           (1 << 10) /* Bit 10: USB AHB clock enable */
                                             /* Bit 11: Reserved */
#define MCLK_AHBMASK_PAC           (1 << 12) /* Bit 12: PAC AHB clock enable */
#define MCLK_AHBMASK_QSPI          (1 << 13) /* Bit 13: QSPI AHB Clock Enable */
#define MCLK_AHBMASK_GMAC          (1 << 14) /* Bit 14: GMAC AHB Clock Enable */
#define MCLK_AHBMASK_SDHC0         (1 << 15) /* Bit 15: SDHC0 HB Clock Enable */
#define MCLK_AHBMASK_SDHC1         (1 << 16) /* Bit 16: SDHC1 HB Clock Enable */
#define MCLK_AHBMASK_CAN0          (1 << 17) /* Bit 17: CAN0 AHB Clock Enable */
#define MCLK_AHBMASK_CAN1          (1 << 18) /* Bit 18: CAN1 AHB Clock Enable */
#define MCLK_AHBMASK_ICM           (1 << 19) /* Bit 19: ICM AHB Clock Enable */
#define MCLK_AHBMASK_PUKCC         (1 << 20) /* Bit 20: PUKCC AHB Clock Enable */
#define MCLK_AHBMASK_QSPI2X        (1 << 21) /* Bit 21: QSPI_2X AHB Clock Enable */
#define MCLK_AHBMASK_NVMCTRL_SMEEPROM (1 << 22) /* Bit 22: NVMCTRL_SMEEPROM AHB Clock Enable */
#define MCLK_AHBMASK_NVMCTRL_CACHE (1 << 23) /* Bit 23: NVMCTRL_CACHE AHB Clock Enable */

/* APBA mask */

#define MCLK_APBAMASK_PAC          (1 << 1)  /* Bit 0:  PAC APBA Clock Enable */
#define MCLK_APBAMASK_PM           (1 << 1)  /* Bit 1:  PM APBA clock enable */
#define MCLK_APBAMASK_MCLK         (1 << 2)  /* Bit 2:  MCLK APBA clock enable */
#define MCLK_APBAMASK_RSTC         (1 << 3)  /* Bit 3:  RSTC APBA clock enable */
#define MCLK_APBAMASK_OSCCTRL      (1 << 4)  /* Bit 4:  OSCCTRL APBA clock enable */
#define MCLK_APBAMASK_OSC32KCTRL   (1 << 5)  /* Bit 5:  OSC32KCTRL APBA clock enable */
#define MCLK_APBAMASK_SUPC         (1 << 6)  /* Bit 6:  SUPC APBA clock enable */
#define MCLK_APBAMASK_GCLK         (1 << 7)  /* Bit 7:  GCLK APBA clock enable */
#define MCLK_APBAMASK_WDT          (1 << 8)  /* Bit 8:  WDT APBA clock enable */
#define MCLK_APBAMASK_RTC          (1 << 9)  /* Bit 9:  RTC APBA clock enable */
#define MCLK_APBAMASK_EIC          (1 << 10) /* Bit 10: EIC APBA clock enable */
#define MCLK_APBAMASK_FREQM        (1 << 11) /* Bit 11: FREQM APBA clock enable */
#define MCLK_APBAMASK_SERCOM0      (1 << 12) /* Bit 12: SERCOM0 APBA Clock Enable */
#define MCLK_APBAMASK_SERCOM1      (1 << 13) /* Bit 13: SERCOM1 APBA Clock Enable */
#define MCLK_APBAMASK_TC0          (1 << 14) /* Bit 14: TC0 APBA clock enable */
#define MCLK_APBAMASK_TC1          (1 << 15) /* Bit 15: TC1 APBA clock enable */

/* APBB mask */

#define MCLK_APBBMASK_USB          (1 << 0)  /* Bit 0:  USB APBB clock enable */
#define MCLK_APBBMASK_DSU          (1 << 1)  /* Bit 1:  DSU APBB clock enable */
#define MCLK_APBBMASK_NVMCTRL      (1 << 2)  /* Bit 2:  NVMCTRL APBB clock enable */
#define MCLK_APBBMASK_PORT         (1 << 4)  /* Bit 4:  PORT APBB Clock Enable */
#define MCLK_APBBMASK_EVSYS        (1 << 7)  /* Bit 7:  EVSYS APBB Clock Enable */
#define MCLK_APBBMASK_SERCOM2      (1 << 9)  /* Bit 9:  SERCOM2 APBB Clock Enable */
#define MCLK_APBBMASK_SERCOM3      (1 << 10) /* Bit 10: SERCOM3 APBB Clock Enable */
#define MCLK_APBBMASK_TCC0         (1 << 11) /* Bit 11: TCC2 APBB Clock Enable */
#define MCLK_APBBMASK_TCC1         (1 << 12) /* Bit 12: TCC3 APBB Clock Enable */
#define MCLK_APBBMASK_TC2          (1 << 13) /* Bit 13: TC2 APBB Clock Enable */
#define MCLK_APBBMASK_TC3          (1 << 14) /* Bit 14: TC3 APBB Clock Enable */
#define MCLK_APBBMASK_RAMECC       (1 << 16) /* Bit 16: RAMECC APBB Clock Enable */

/* APBC mask */

#define MCLK_APBCMASK_GMAC         (1 << 2)  /* Bit 2:  GMAC APBC Mask Clock Enable */
#define MCLK_APBCMASK_TCC2         (1 << 3)  /* Bit 3:  TCC2 APBC Clock Enable */
#define MCLK_APBCMASK_TCC3         (1 << 4)  /* Bit 4:  TCC3 APBC Clock Enable */
#define MCLK_APBCMASK_TC4          (1 << 5)  /* Bit 5:  TC4 APBC Clock Enable */
#define MCLK_APBCMASK_TC5          (1 << 6)  /* Bit 6:  TC5 APBC Clock Enable */
#define MCLK_APBCMASK_PDEC         (1 << 7)  /* Bit 7:  PDEC APBC Mask Clock Enable */
#define MCLK_APBCMASK_AC           (1 << 8)  /* Bit 8:  AC APBC Mask Clock Enable */
#define MCLK_APBCMASK_AES          (1 << 9)  /* Bit 9:  AES APBC Mask Clock Enable */
#define MCLK_APBCMASK_TRNG         (1 << 10) /* Bit 10: TRNG APBC Mask Clock Enable */
#define MCLK_APBCMASK_ICM          (1 << 11) /* Bit 11: ICM APBC Mask Clock Enable */
#define MCLK_APBCMASK_QSPI         (1 << 13) /* Bit 13: QSPI APBC Mask Clock Enable */
#define MCLK_APBCMASK_CCL          (1 << 14) /* Bit 14: CCL APBC Mask Clock Enable */

/* APBD mask */

#define MCLK_APBDMASK_SERCOM4      (1 << 0)  /* Bit 0:  SERCOM4 APBD clock enable */
#define MCLK_APBDMASK_SERCOM5      (1 << 1)  /* Bit 1:  SERCOM5 APBD clock enable */
#define MCLK_APBDMASK_SERCOM6      (1 << 2)  /* Bit 2:  SERCOM6 APBD clock enable */
#define MCLK_APBDMASK_SERCOM7      (1 << 3)  /* Bit 3:  SERCOM7 APBD clock enable */
#define MCLK_APBDMASK_TCC4         (1 << 4)  /* Bit 4:  TCC5 APBD Clock Enable */
#define MCLK_APBDMASK_TC6          (1 << 5)  /* Bit 5:  TC6 APBD Clock Enable */
#define MCLK_APBDMASK_TC7          (1 << 6)  /* Bit 6:  TC7 APBD Clock Enable */
#define MCLK_APBDMASK_ADC0         (1 << 7)  /* Bit 7:  ADC0 APBD clock enable */
#define MCLK_APBDMASK_ADC1         (1 << 8)  /* Bit 8:  ADC1 APBD clock enable */
#define MCLK_APBDMASK_DAC          (1 << 9)  /* Bit 9:  DAC APBD clock enable */
#define MCLK_APBDMASK_I2C          (1 << 10) /* Bit 10: I2S APBD clock enable */
#define MCLK_APBDMASK_PCC          (1 << 11) /* Bit 11: PCC APBD clock enable */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD5E5_CHIP_SAM_MCLK_H */
