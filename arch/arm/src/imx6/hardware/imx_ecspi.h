/************************************************************************************
 * arch/arm/src/imx6/hardware/imx_ecspi.h
 *
 *   Copyright (C) 2009-2010, 2015-2016 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_IMX6_HARDWARE_ECSPI_H
#define __ARCH_ARM_SRC_IMX6_HARDWARE_ECSPI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/imx_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* ECSPI Register Offsets ***********************************************************/

#define ECSPI_RXDATA_OFFSET           0x0000 /* Receive Data Register */
#define ECSPI_TXDATA_OFFSET           0x0004 /* Transmit Data Register */
#define ECSPI_CONREG_OFFSET           0x0008 /* Control Register */
#define ECSPI_CONFIGREG_OFFSET        0x000c /* Configuration Register */
#define ECSPI_INTREG_OFFSET           0x0010 /* Interrupt Control Register */
#define ECSPI_DMAREG_OFFSET           0x0014 /* DMA Control Register */
#define ECSPI_STATREG_OFFSET          0x0018 /* Status Register */
#define ECSPI_PERIODREG_OFFSET        0x001c /* Sample Period Control Register */
#define ECSPI_TESTREG_OFFSET          0x0020 /* Test Control Register */
#define ECSPI_MSGDATA_OFFSET          0x0040 /* Message Data Register */

/* ECSPI Register Addresses *********************************************************/

/* ECSPI1 */

#define IMX_ECSPI1_RXDATA             (IMX_ECSPI1_VBASE + ECSPI_RXDATA_OFFSET)
#define IMX_ECSPI1_TXDATA             (IMX_ECSPI1_VBASE + ECSPI_TXDATA_OFFSET)
#define IMX_ECSPI1_CONREG             (IMX_ECSPI1_VBASE + ECSPI_CONREG_OFFSET)
#define IMX_ECSPI1_CONFIGREG          (IMX_ECSPI1_VBASE + ECSPI_CONFIGREG_OFFSET)
#define IMX_ECSPI1_INTREG             (IMX_ECSPI1_VBASE + ECSPI_INTREG_OFFSET)
#define IMX_ECSPI1_DMAREG             (IMX_ECSPI1_VBASE + ECSPI_DMAREG_OFFSET)
#define IMX_ECSPI1_STATREG            (IMX_ECSPI1_VBASE + ECSPI_STATREG_OFFSET)
#define IMX_ECSPI1_PERIODREG          (IMX_ECSPI1_VBASE + ECSPI_PERIODREG_OFFSET)
#define IMX_ECSPI1_TESTREG            (IMX_ECSPI1_VBASE + ECSPI_TESTREG_OFFSET)
#define IMX_ECSPI1_MSGDATA            (IMX_ECSPI1_VBASE + ECSPI_MSGDATA_OFFSET)

/* ECSPI2 */

#define IMX_ECSPI2_RXDATA             (IMX_ECSPI2_VBASE + ECSPI_RXDATA_OFFSET)
#define IMX_ECSPI2_TXDATA             (IMX_ECSPI2_VBASE + ECSPI_TXDATA_OFFSET)
#define IMX_ECSPI2_CONREG             (IMX_ECSPI2_VBASE + ECSPI_CONREG_OFFSET)
#define IMX_ECSPI2_CONFIGREG          (IMX_ECSPI2_VBASE + ECSPI_CONFIGREG_OFFSET)
#define IMX_ECSPI2_INTREG             (IMX_ECSPI2_VBASE + ECSPI_INTREG_OFFSET)
#define IMX_ECSPI2_DMAREG             (IMX_ECSPI2_VBASE + ECSPI_DMAREG_OFFSET)
#define IMX_ECSPI2_STATREG            (IMX_ECSPI2_VBASE + ECSPI_STATREG_OFFSET)
#define IMX_ECSPI2_PERIODREG          (IMX_ECSPI2_VBASE + ECSPI_PERIODREG_OFFSET)
#define IMX_ECSPI2_TESTREG            (IMX_ECSPI2_VBASE + ECSPI_TESTREG_OFFSET)
#define IMX_ECSPI2_MSGDATA            (IMX_ECSPI2_VBASE + ECSPI_MSGDATA_OFFSET)

/* ECSPI3 */

#define IMX_ECSPI3_RXDATA             (IMX_ECSPI3_VBASE + ECSPI_RXDATA_OFFSET)
#define IMX_ECSPI3_TXDATA             (IMX_ECSPI3_VBASE + ECSPI_TXDATA_OFFSET)
#define IMX_ECSPI3_CONREG             (IMX_ECSPI3_VBASE + ECSPI_CONREG_OFFSET)
#define IMX_ECSPI3_CONFIGREG          (IMX_ECSPI3_VBASE + ECSPI_CONFIGREG_OFFSET)
#define IMX_ECSPI3_INTREG             (IMX_ECSPI3_VBASE + ECSPI_INTREG_OFFSET)
#define IMX_ECSPI3_DMAREG             (IMX_ECSPI3_VBASE + ECSPI_DMAREG_OFFSET)
#define IMX_ECSPI3_STATREG            (IMX_ECSPI3_VBASE + ECSPI_STATREG_OFFSET)
#define IMX_ECSPI3_PERIODREG          (IMX_ECSPI3_VBASE + ECSPI_PERIODREG_OFFSET)
#define IMX_ECSPI3_TESTREG            (IMX_ECSPI3_VBASE + ECSPI_TESTREG_OFFSET)
#define IMX_ECSPI3_MSGDATA            (IMX_ECSPI3_VBASE + ECSPI_MSGDATA_OFFSET)

/* ECSPI4 */

#define IMX_ECSPI4_RXDATA             (IMX_ECSPI4_VBASE + ECSPI_RXDATA_OFFSET)
#define IMX_ECSPI4_TXDATA             (IMX_ECSPI4_VBASE + ECSPI_TXDATA_OFFSET)
#define IMX_ECSPI4_CONREG             (IMX_ECSPI4_VBASE + ECSPI_CONREG_OFFSET)
#define IMX_ECSPI4_CONFIGREG          (IMX_ECSPI4_VBASE + ECSPI_CONFIGREG_OFFSET)
#define IMX_ECSPI4_INTREG             (IMX_ECSPI4_VBASE + ECSPI_INTREG_OFFSET)
#define IMX_ECSPI4_DMAREG             (IMX_ECSPI4_VBASE + ECSPI_DMAREG_OFFSET)
#define IMX_ECSPI4_STATREG            (IMX_ECSPI4_VBASE + ECSPI_STATREG_OFFSET)
#define IMX_ECSPI4_PERIODREG          (IMX_ECSPI4_VBASE + ECSPI_PERIODREG_OFFSET)
#define IMX_ECSPI4_TESTREG            (IMX_ECSPI4_VBASE + ECSPI_TESTREG_OFFSET)
#define IMX_ECSPI4_MSGDATA            (IMX_ECSPI4_VBASE + ECSPI_MSGDATA_OFFSET)

/* ECSPI5 */

#define IMX_ECSPI5_RXDATA             (IMX_ECSPI5_VBASE + ECSPI_RXDATA_OFFSET)
#define IMX_ECSPI5_TXDATA             (IMX_ECSPI5_VBASE + ECSPI_TXDATA_OFFSET)
#define IMX_ECSPI5_CONREG             (IMX_ECSPI5_VBASE + ECSPI_CONREG_OFFSET)
#define IMX_ECSPI5_CONFIGREG          (IMX_ECSPI5_VBASE + ECSPI_CONFIGREG_OFFSET)
#define IMX_ECSPI5_INTREG             (IMX_ECSPI5_VBASE + ECSPI_INTREG_OFFSET)
#define IMX_ECSPI5_DMAREG             (IMX_ECSPI5_VBASE + ECSPI_DMAREG_OFFSET)
#define IMX_ECSPI5_STATREG            (IMX_ECSPI5_VBASE + ECSPI_STATREG_OFFSET)
#define IMX_ECSPI5_PERIODREG          (IMX_ECSPI5_VBASE + ECSPI_PERIODREG_OFFSET)
#define IMX_ECSPI5_TESTREG            (IMX_ECSPI5_VBASE + ECSPI_TESTREG_OFFSET)
#define IMX_ECSPI5_MSGDATA            (IMX_ECSPI5_VBASE + ECSPI_MSGDATA_OFFSET)

/* ECSPI Register Bit Definitions ***************************************************/

/* Control Register */

#define ECSPI_CONREG_EN               (1 << 0)  /* Bit 0:  SPI Block enable control */
#define ECSPI_CONREG_HT               (1 << 1)  /* Bit 1:  Hardware trigger enable */
#define ECSPI_CONREG_XCH              (1 << 2)  /* Bit 2:  SPI Exchange bit */
#define ECSPI_CONREG_SMC              (1 << 3)  /* Bit 3:  Start mode control */
#define ECSPI_CONREG_CHMODE_SHIFT     (4)      /* Bits 4-7: SPI Channel mode */
#define ECSPI_CONREG_CHMODE_MASK      (15 << ECSPI_CONREG_CHMODE_SHIFT)
#  define ECSPI_CONREG_CH0MASTER      (1 << ECSPI_CONREG_CHMODE_SHIFT) /* Channel 0 master mode */
#  define ECSPI_CONREG_CH1MASTER      (2 << ECSPI_CONREG_CHMODE_SHIFT) /* Channel 1 master mode */
#  define ECSPI_CONREG_CH2MASTER      (4 << ECSPI_CONREG_CHMODE_SHIFT) /* Channel 2 master mode */
#  define ECSPI_CONREG_CH3MASTER      (8 << ECSPI_CONREG_CHMODE_SHIFT) /* Channel 3 master mode */
#define ECSPI_CONREG_POSTDIV_SHIFT    (8)      /* Bits 8-11: SPI Post divider (exponent) */
#define ECSPI_CONREG_POSTDIV_MASK     (15 << ECSPI_CONREG_POSTDIV_SHIFT)
#  define ECSPI_CONREG_POSTDIV_EXP(n) ((uint32_t)(n) << ECSPI_CONREG_POSTDIV_SHIFT)
#  define ECSPI_CONREG_POSTDIV_1      (0  << ECSPI_CONREG_POSTDIV_SHIFT) /* Divide by 2*0 */
#  define ECSPI_CONREG_POSTDIV_2      (1  << ECSPI_CONREG_POSTDIV_SHIFT) /* Divide by 2*1 */
#  define ECSPI_CONREG_POSTDIV_4      (2  << ECSPI_CONREG_POSTDIV_SHIFT) /* Divide by 2*2 */
#  define ECSPI_CONREG_POSTDIV_8      (3  << ECSPI_CONREG_POSTDIV_SHIFT) /* Divide by 2*3 */
#  define ECSPI_CONREG_POSTDIV_16     (4  << ECSPI_CONREG_POSTDIV_SHIFT) /* Divide by 2*4 */
#  define ECSPI_CONREG_POSTDIV_32     (5  << ECSPI_CONREG_POSTDIV_SHIFT) /* Divide by 2*5 */
#  define ECSPI_CONREG_POSTDIV_64     (6  << ECSPI_CONREG_POSTDIV_SHIFT) /* Divide by 2*6 */
#  define ECSPI_CONREG_POSTDIV_128    (7  << ECSPI_CONREG_POSTDIV_SHIFT) /* Divide by 2*7 */
#  define ECSPI_CONREG_POSTDIV_256    (8  << ECSPI_CONREG_POSTDIV_SHIFT) /* Divide by 2*8 */
#  define ECSPI_CONREG_POSTDIV_512    (9  << ECSPI_CONREG_POSTDIV_SHIFT) /* Divide by 2*9 */
#  define ECSPI_CONREG_POSTDIV_1024   (10 << ECSPI_CONREG_POSTDIV_SHIFT) /* Divide by 2*10 */
#  define ECSPI_CONREG_POSTDIV_2048   (11 << ECSPI_CONREG_POSTDIV_SHIFT) /* Divide by 2*11 */
#  define ECSPI_CONREG_POSTDIV_4096   (12 << ECSPI_CONREG_POSTDIV_SHIFT) /* Divide by 2*12 */
#  define ECSPI_CONREG_POSTDIV_8192   (13 << ECSPI_CONREG_POSTDIV_SHIFT) /* Divide by 2*13 */
#  define ECSPI_CONREG_POSTDIV_16384  (14 << ECSPI_CONREG_POSTDIV_SHIFT) /* Divide by 2*14 */
#  define ECSPI_CONREG_POSTDIV_32768  (15 << ECSPI_CONREG_POSTDIV_SHIFT) /* Divide by 2*15 */
#define ECSPI_CONREG_PREDIV_SHIFT     (12)      /* Bits 12-15: SPI Pre divider (minus 1) */
#define ECSPI_CONREG_PREDIV_MASK      (15 << ECSPI_CONREG_PREDIV_SHIFT)
#  define ECSPI_CONREG_PREDIV(n)      ((uint32_t)(n) << ECSPI_CONREG_PREDIV_SHIFT)
#define ECSPI_CONREG_DRCTL_SHIFT      16  /* Bits 16-17: SPI Data ready control */
#define ECSPI_CONREG_DRCTL_MASK       (3 << ECSPI_CONREG_DRCTL_SHIFT)
#  define ECSPI_CONREG_DRCTL_IGNRDY   (0 << ECSPI_CONREG_DRCTL_SHIFT)
#  define ECSPI_CONREG_DRCTL_FALLING  (1 << ECSPI_CONREG_DRCTL_SHIFT)
#  define ECSPI_CONREG_DRCTL_ACTVLOW  (2 << ECSPI_CONREG_DRCTL_SHIFT)
#define ECSPI_CONREG_CHSEL_SHIFT      (18)      /* Bits 18-19: SPI Channel select bits */
#define ECSPI_CONREG_CHSEL_MASK       (3 << ECSPI_CONREG_CHSEL_SHIFT)
#  define ECSPI_CONREG_CHSEL_SS0      (0 << ECSPI_CONREG_CHSEL_SHIFT) /* Channel 0 select (SS0) */
#  define ECSPI_CONREG_CHSEL_SS1      (1 << ECSPI_CONREG_CHSEL_SHIFT) /* Channel 1 select (SS1) */
#  define ECSPI_CONREG_CHSEL_SS2      (2 << ECSPI_CONREG_CHSEL_SHIFT) /* Channel 2 select (SS2) */
#  define ECSPI_CONREG_CHSEL_SS3      (3 << ECSPI_CONREG_CHSEL_SHIFT) /* Channel 3 select (SS3) */
#define ECSPI_CONREG_BURSTLEN_SHIFT   (20)      /* Bits 20-31: Burst length */
#define ECSPI_CONREG_BURSTLEN_MASK    (0xfff << ECSPI_CONREG_BURSTLEN_SHIFT)
#  define ECSPI_CONREG_BURSTLEN(n)    ((uint32_t)(n) << ECSPI_CONREG_BURSTLEN_SHIFT)

/* Configuration Register */

#define ECSPI_CONFIGREG_SCLKPHA_SHIFT (0)       /* Bits 0-3: SPI Clock/Data Phase Control */
#define ECSPI_CONFIGREG_SCLKPHA_MASK  (15 << ECSPI_CONFIGREG_SCLKPHA_SHIFT)
#  define ECSPI_CONFIGREG_CH0PHA      (1 << ECSPI_CONFIGREG_SCLKPHA_SHIFT) /* Channel 0 SCLK Phase */
#  define ECSPI_CONFIGREG_CH1PHA      (2 << ECSPI_CONFIGREG_SCLKPHA_SHIFT) /* Channel 1 SCLK Phase */
#  define ECSPI_CONFIGREG_CH2PHA      (4 << ECSPI_CONFIGREG_SCLKPHA_SHIFT) /* Channel 2 SCLK Phase */
#  define ECSPI_CONFIGREG_CH3PHA      (8 << ECSPI_CONFIGREG_SCLKPHA_SHIFT) /* Channel 3 SCLK Phase */
#define ECSPI_CONFIGREG_SCLKPOL_SHIFT (4)       /* Bits 4-7: SPI Clock polarity control */
#define ECSPI_CONFIGREG_SCLKPOL_MASK  (15 << ECSPI_CONFIGREG_SCLKPOL_SHIFT)
#  define ECSPI_CONFIGREG_CH0POL      (1 << ECSPI_CONFIGREG_SCLKPOL_SHIFT) /* Channel 0 SCLK polarity */
#  define ECSPI_CONFIGREG_CH1POL      (2 << ECSPI_CONFIGREG_SCLKPOL_SHIFT) /* Channel 1 SCLK polarity */
#  define ECSPI_CONFIGREG_CH2POL      (4 << ECSPI_CONFIGREG_SCLKPOL_SHIFT) /* Channel 2 SCLK polarity */
#  define ECSPI_CONFIGREG_CH3POL      (8 << ECSPI_CONFIGREG_SCLKPOL_SHIFT) /* Channel 3 SCLK polarity */
#define ECSPI_CONFIGREG_SSCTL_SHIFT   (8)       /* Bits 8-11: SPI SS Wave form select */
#define ECSPI_CONFIGREG_SSCTL_MASK    (15 << ECSPI_CONFIGREG_SSCTL_SHIFT)
#  define ECSPI_CONFIGREG_CH0SSCTRL   (1 << ECSPI_CONFIGREG_SSCTL_SHIFT) /* Channel 0 SS control */
#  define ECSPI_CONFIGREG_CH1SSCTRL   (2 << ECSPI_CONFIGREG_SSCTL_SHIFT) /* Channel 1 SS control */
#  define ECSPI_CONFIGREG_CH2SSCTRL   (4 << ECSPI_CONFIGREG_SSCTL_SHIFT) /* Channel 2 SS control */
#  define ECSPI_CONFIGREG_CH3SSCTRL   (8 << ECSPI_CONFIGREG_SSCTL_SHIFT) /* Channel 3 SS control */
#define ECSPI_CONFIGREG_SSPOL_SHIFT   (12)      /* Bits 12-15: SPI SS Polarity select */
#define ECSPI_CONFIGREG_SSPOL_MASK    (15 << ECSPI_CONFIGREG_SSPOL_SHIFT)
#  define ECSPI_CONFIGREG_CH0SSPOL    (1 << ECSPI_CONFIGREG_CHMODE_SHIFT) /* Channel 0 SS polarity */
#  define ECSPI_CONFIGREG_CH1SSPOL    (2 << ECSPI_CONFIGREG_CHMODE_SHIFT) /* Channel 1 SS polarity */
#  define ECSPI_CONFIGREG_CH2SSPOL    (4 << ECSPI_CONFIGREG_CHMODE_SHIFT) /* Channel 2 SS polarity */
#  define ECSPI_CONFIGREG_CH3SSPOL    (8 << ECSPI_CONFIGREG_CHMODE_SHIFT) /* Channel 3 SS polarity */
#define ECSPI_CONFIGREG_DATCTL_SHIFT  (16)      /* Bits 16-19: Data control */
#define ECSPI_CONFIGREG_DATCTL_MASK   (15 << ECSPI_CONFIGREG_DATCTL_SHIFT)
#  define ECSPI_CONFIGREG_CH0DATLOW   (1 << ECSPI_CONFIGREG_DATCTL_SHIFT) /* Channel 0 SS low when inactive */
#  define ECSPI_CONFIGREG_CH1DATLOW   (2 << ECSPI_CONFIGREG_DATCTL_SHIFT) /* Channel 1 SS low when inactive */
#  define ECSPI_CONFIGREG_CH2DATLOW   (4 << ECSPI_CONFIGREG_DATCTL_SHIFT) /* Channel 2 SS low when inactive */
#  define ECSPI_CONFIGREG_CH3DATLOW   (8 << ECSPI_CONFIGREG_DATCTL_SHIFT) /* Channel 3 SS low when inactive */
#define ECSPI_CONFIGREG_SCLKCTL_SHIFT (20)      /* Bits 20-23: SCLK Control */
#define ECSPI_CONFIGREG_SCLKCTL_MASK  (15 << ECSPI_CONFIGREG_SCLKCTL_SHIFT)
#  define ECSPI_CONFIGREG_CH0SCLKLOW  (1 << ECSPI_CONFIGREG_SCLKCTL_SHIFT) /* Channel 0 SCLK low when inactive */
#  define ECSPI_CONFIGREG_CH1SCLKLOW  (2 << ECSPI_CONFIGREG_SCLKCTL_SHIFT) /* Channel 1 SCLK low when inactive */
#  define ECSPI_CONFIGREG_CH2SCLKLOW  (4 << ECSPI_CONFIGREG_SCLKCTL_SHIFT) /* Channel 2 SCLK low when inactive */
#  define ECSPI_CONFIGREG_CH3SCLKLOW  (8 << ECSPI_CONFIGREG_SCLKCTL_SHIFT) /* Channel 3 SCLK low when inactive */
#define ECSPI_CONFIGREG_HTLEN_SHIFT   (24)      /* Bits 24-28: HT Length */
#define ECSPI_CONFIGREG_HTLEN_MASK    (15 << ECSPI_CONFIGREG_HTLEN_SHIFT)
#  define ECSPI_CONFIGREG_HTLEN(n)    ((uint32_t)(n) << ECSPI_CONFIGREG_HTLEN_SHIFT)
                                                /* Bits 29-31: Reserved */

/* Common Interrupt Control Register and Status Register */

#define ECSPI_INT_TE                  (1 << 0)  /* Bit 0:  TXFIFO Empty Interrupt */
#define ECSPI_INT_TDR                 (1 << 1)  /* Bit 1:  TXFIFO Data Request Interrupt */
#define ECSPI_INT_TF                  (1 << 2)  /* Bit 2:  TXFIFO Full Interrupt */
#define ECSPI_INT_RR                  (1 << 3)  /* Bit 3:  RXFIFO Data Ready Interrupt */
#define ECSPI_INT_RDR                 (1 << 4)  /* Bit 4:  RXFIFO Data Request Interrupt enable */
#define ECSPI_INT_RF                  (1 << 5)  /* Bit 5:  RXFIFO Full Interrupt */
#define ECSPI_INT_RO                  (1 << 6)  /* Bit 6:  RXFIFO Overflow Interrupt */
#define ECSPI_INT_TC                  (1 << 7)  /* Bit 7:  Transfer Completed Interrupt enable */
                                                /* Bits 8-31: Reserved */

#define ECSPI_INT_ALL                 0x000000ff

/* ECSPI DMA Control Register */

#define ECSPI_DMAREG_TXTHRES_SHIFT    (0)       /* Bits 0-5: TX Threshold */
#define ECSPI_DMAREG_TXTHRES_MASK     (0x3f << ECSPI_DMAREG_TXTHRES_SHIFT)
#  define ECSPI_DMAREG_TXTHRES(n)     ((uint32_t)(n) << ECSPI_DMAREG_TXTHRES_SHIFT)
                                                /* Bit 6: Reserved */
#define ECSPI_DMAREG_TEDEN            (1 << 7)  /* Bit 7:  TXFIFO Empty DMA request enable */
                                                /* Bits 8-15: Reserved */
#define ECSPI_DMAREG_RXTHRES_SHIFT    (16)      /* Bits 16-21: RX Threshold */
#define ECSPI_DMAREG_RXTHRES_MASK     (0x3f << ECSPI_DMAREG_RXTHRES_SHIFT)
#  define ECSPI_DMAREG_RXTHRES(n)     ((uint32_t)(n) << ECSPI_DMAREG_RXTHRES_SHIFT)
                                                /* Bit 22: Reserved */
#define ECSPI_DMAREG_RXDEN            (1 << 23) /* Bit 23: RXFIFO DMA request enable */
#define ECSPI_DMAREG_RXLEN_SHIFT      (24)      /* Bits 24-29: RX DMA length */
#define ECSPI_DMAREG_RXLEN_MASK       (0x3f << ECSPI_DMAREG_RXLEN_SHIFT)
#  define ECSPI_DMAREG_RXLEN(n)       ((uint32_t)(n) << ECSPI_DMAREG_RXLEN_SHIFT)
                                                /* Bit 30: Reserved */
#define ECSPI_DMAREG_RXTDEN           (1 << 31) /* Bit 31: RXFIFO Tail DMA request enable */

/* ECSPI Sample Period Control Register */

#define ECSPI_PERIODREG_PERIOD_SHIFT  (0)        /* Bits 0-14: Sample Period Control */
#define ECSPI_PERIODREG_PERIOD_MASK   (0x7fff << ECSPI_PERIODREG_PERIOD_SHIFT)
#  define ECSPI_PERIODREG_PERIOD(n)   ((uint32_t)(n) << ECSPI_PERIODREG_PERIOD_SHIFT)
#define ECSPI_PERIODREG_CSRC          (1 << 15) /* Bit 15: Clock source control */
#define ECSPI_PERIODREG_CSDCTL_SHIFT  (16)      /* Bits 16-21: Chip Select delay control */
#define ECSPI_PERIODREG_CSDCTL_MASK   (0x3f << ECSPI_PERIODREG_CSDCTL_SHIFT)
#  define ECSPI_PERIODREG_CSDCTL(n)   ((uint32_t)(n) << ECSPI_PERIODREG_CSDCTL_SHIFT)
                                                /* Bits 22-31: Reserved */

/* Test Control Register */

#define ECSPI_TESTREG_TXCNT_SHIFT     (0)       /* Bits 0-6: TXFIFO Counter */
#define ECSPI_TESTREG_TXCNT_MASK      (0x7f << ECSPI_TESTREG_TXCNT_SHIFT)
#  define ECSPI_TESTREG_TXCNT(n)      ((uint32_t)(n) << ECSPI_TESTREG_TXCNT_SHIFT)
                                                /* Bit 7: Reserved */
#define ECSPI_TESTREG_RXCNT_SHIFT     (8)       /* Bits 8-14: RXFIFO Counter */
#define ECSPI_TESTREG_RXCNT_MASK      (0x7f << ECSPI_TESTREG_RXCNT_SHIFT)
#  define ECSPI_TESTREG_RXCNT(n)      ((uint32_t)(n) << ECSPI_TESTREG_RXCNT_SHIFT)
                                                /* Bits 15-30: Reserved */
#define ECSPI_TESTREG_LBC             (1 << 31) /* Bit 31: Loop Back Control */

/* Message Data Register (32-bit message data) */

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_IMX6_HARDWARE_ECSPI_H */
