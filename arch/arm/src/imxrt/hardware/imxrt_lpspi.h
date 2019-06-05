/********************************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_lpspi.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Pavlina Koleva <pavlinaikoleva19@gmail.com>
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_LPSPI_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_LPSPI_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Register offsets *************************************************************************/

#define IMXRT_LPSPI_VERID_OFFSET       0x0000  /* Version ID Register offset */
#define IMXRT_LPSPI_PARAM_OFFSET       0x0004  /* Parameter Register offset */
#define IMXRT_LPSPI_CR_OFFSET          0x0010  /* Control Register offset */
#define IMXRT_LPSPI_SR_OFFSET          0x0014  /* Status Register offset */
#define IMXRT_LPSPI_IER_OFFSET         0x0018  /* Interrupt Enable Register offset */
#define IMXRT_LPSPI_DER_OFFSET         0x001C  /* DMA Enable Register offset */
#define IMXRT_LPSPI_CFGR0_OFFSET       0x0020  /* Configuration Register 0 offset */
#define IMXRT_LPSPI_CFGR1_OFFSET       0x0024  /* Configuration Register 1 offset */
#define IMXRT_LPSPI_DMR0_OFFSET        0x0030  /* Data Match Register 0 offset */
#define IMXRT_LPSPI_DMR1_OFFSET        0x0034  /* Data Match Register 1 offset */
#define IMXRT_LPSPI_CCR_OFFSET         0x0040  /* Clock Configuration Register offset */
#define IMXRT_LPSPI_FCR_OFFSET         0x0058  /* FIFO Control Register offset */
#define IMXRT_LPSPI_FSR_OFFSET         0x005C  /* FIFO Status Register offset */
#define IMXRT_LPSPI_TCR_OFFSET         0x0060  /* Transmit Command Register offset */
#define IMXRT_LPSPI_TDR_OFFSET         0x0064  /* Transmit Data Register offset */
#define IMXRT_LPSPI_RSR_OFFSET         0x0070  /* Receive Status Register offset */
#define IMXRT_LPSPI_RDR_OFFSET         0x0074  /* Receive Data Register offset */

/* Register addresses ***********************************************************************/

#define IMXRT_LPSPI1_VERID             (IMXRT_LPSPI1_BASE + IMXRT_LPSPI_VERID_OFFSET)
#define IMXRT_LPSPI1_PARAM             (IMXRT_LPSPI1_BASE + IMXRT_LPSPI_PARAM_OFFSET)
#define IMXRT_LPSPI1_CR                (IMXRT_LPSPI1_BASE + IMXRT_LPSPI_CR_OFFSET)
#define IMXRT_LPSPI1_SR                (IMXRT_LPSPI1_BASE + IMXRT_LPSPI_SR_OFFSET)
#define IMXRT_LPSPI1_IER               (IMXRT_LPSPI1_BASE + IMXRT_LPSPI_IER_OFFSET)
#define IMXRT_LPSPI1_DER               (IMXRT_LPSPI1_BASE + IMXRT_LPSPI_DER_OFFSET)
#define IMXRT_LPSPI1_CFGR0             (IMXRT_LPSPI1_BASE + IMXRT_LPSPI_CFGR0_OFFSET)
#define IMXRT_LPSPI1_CFGR1             (IMXRT_LPSPI1_BASE + IMXRT_LPSPI_CFGR1_OFFSET)
#define IMXRT_LPSPI1_DMR0              (IMXRT_LPSPI1_BASE + IMXRT_LPSPI_DMR0_OFFSET)
#define IMXRT_LPSPI1_DMR1              (IMXRT_LPSPI1_BASE + IMXRT_LPSPI_DMR1_OFFSET)
#define IMXRT_LPSPI1_CCR               (IMXRT_LPSPI1_BASE + IMXRT_LPSPI_CCR_OFFSET)
#define IMXRT_LPSPI1_FCR               (IMXRT_LPSPI1_BASE + IMXRT_LPSPI_FCR_OFFSET)
#define IMXRT_LPSPI1_FSR               (IMXRT_LPSPI1_BASE + IMXRT_LPSPI_FSR_OFFSET)
#define IMXRT_LPSPI1_TCR               (IMXRT_LPSPI1_BASE + IMXRT_LPSPI_TCR_OFFSET)
#define IMXRT_LPSPI1_TDR               (IMXRT_LPSPI1_BASE + IMXRT_LPSPI_TDR_OFFSET)
#define IMXRT_LPSPI1_RSR               (IMXRT_LPSPI1_BASE + IMXRT_LPSPI_RSR_OFFSET)
#define IMXRT_LPSPI1_RDR               (IMXRT_LPSPI1_BASE + IMXRT_LPSPI_RDR_OFFSET)

#define IMXRT_LPSPI2_VERID             (IMXRT_LPSPI2_BASE + IMXRT_LPSPI_VERID_OFFSET)
#define IMXRT_LPSPI2_PARAM             (IMXRT_LPSPI2_BASE + IMXRT_LPSPI_PARAM_OFFSET)
#define IMXRT_LPSPI2_CR                (IMXRT_LPSPI2_BASE + IMXRT_LPSPI_CR_OFFSET)
#define IMXRT_LPSPI2_SR                (IMXRT_LPSPI2_BASE + IMXRT_LPSPI_SR_OFFSET)
#define IMXRT_LPSPI2_IER               (IMXRT_LPSPI2_BASE + IMXRT_LPSPI_IER_OFFSET)
#define IMXRT_LPSPI2_DER               (IMXRT_LPSPI2_BASE + IMXRT_LPSPI_DER_OFFSET)
#define IMXRT_LPSPI2_CFGR0             (IMXRT_LPSPI2_BASE + IMXRT_LPSPI_CFGR0_OFFSET)
#define IMXRT_LPSPI2_CFGR1             (IMXRT_LPSPI2_BASE + IMXRT_LPSPI_CFGR1_OFFSET)
#define IMXRT_LPSPI2_DMR0              (IMXRT_LPSPI2_BASE + IMXRT_LPSPI_DMR0_OFFSET)
#define IMXRT_LPSPI2_DMR1              (IMXRT_LPSPI2_BASE + IMXRT_LPSPI_DMR1_OFFSET)
#define IMXRT_LPSPI2_CCR               (IMXRT_LPSPI2_BASE + IMXRT_LPSPI_CCR_OFFSET)
#define IMXRT_LPSPI2_FCR               (IMXRT_LPSPI2_BASE + IMXRT_LPSPI_FCR_OFFSET)
#define IMXRT_LPSPI2_FSR               (IMXRT_LPSPI2_BASE + IMXRT_LPSPI_FSR_OFFSET)
#define IMXRT_LPSPI2_TCR               (IMXRT_LPSPI2_BASE + IMXRT_LPSPI_TCR_OFFSET)
#define IMXRT_LPSPI2_TDR               (IMXRT_LPSPI2_BASE + IMXRT_LPSPI_TDR_OFFSET)
#define IMXRT_LPSPI2_RSR               (IMXRT_LPSPI2_BASE + IMXRT_LPSPI_RSR_OFFSET)
#define IMXRT_LPSPI2_RDR               (IMXRT_LPSPI2_BASE + IMXRT_LPSPI_RDR_OFFSET)

#define IMXRT_LPSPI3_VERID             (IMXRT_LPSPI3_BASE + IMXRT_LPSPI_VERID_OFFSET)
#define IMXRT_LPSPI3_PARAM             (IMXRT_LPSPI3_BASE + IMXRT_LPSPI_PARAM_OFFSET)
#define IMXRT_LPSPI3_CR                (IMXRT_LPSPI3_BASE + IMXRT_LPSPI_CR_OFFSET)
#define IMXRT_LPSPI3_SR                (IMXRT_LPSPI3_BASE + IMXRT_LPSPI_SR_OFFSET)
#define IMXRT_LPSPI3_IER               (IMXRT_LPSPI3_BASE + IMXRT_LPSPI_IER_OFFSET)
#define IMXRT_LPSPI3_DER               (IMXRT_LPSPI3_BASE + IMXRT_LPSPI_DER_OFFSET)
#define IMXRT_LPSPI3_CFGR0             (IMXRT_LPSPI3_BASE + IMXRT_LPSPI_CFGR0_OFFSET)
#define IMXRT_LPSPI3_CFGR1             (IMXRT_LPSPI3_BASE + IMXRT_LPSPI_CFGR1_OFFSET)
#define IMXRT_LPSPI3_DMR0              (IMXRT_LPSPI3_BASE + IMXRT_LPSPI_DMR0_OFFSET)
#define IMXRT_LPSPI3_DMR1              (IMXRT_LPSPI3_BASE + IMXRT_LPSPI_DMR1_OFFSET)
#define IMXRT_LPSPI3_CCR               (IMXRT_LPSPI3_BASE + IMXRT_LPSPI_CCR_OFFSET)
#define IMXRT_LPSPI3_FCR               (IMXRT_LPSPI3_BASE + IMXRT_LPSPI_FCR_OFFSET)
#define IMXRT_LPSPI3_FSR               (IMXRT_LPSPI3_BASE + IMXRT_LPSPI_FSR_OFFSET)
#define IMXRT_LPSPI3_TCR               (IMXRT_LPSPI3_BASE + IMXRT_LPSPI_TCR_OFFSET)
#define IMXRT_LPSPI3_TDR               (IMXRT_LPSPI3_BASE + IMXRT_LPSPI_TDR_OFFSET)
#define IMXRT_LPSPI3_RSR               (IMXRT_LPSPI3_BASE + IMXRT_LPSPI_RSR_OFFSET)
#define IMXRT_LPSPI3_RDR               (IMXRT_LPSPI3_BASE + IMXRT_LPSPI_RDR_OFFSET)

#define IMXRT_LPSPI4_VERID             (IMXRT_LPSPI4_BASE + IMXRT_LPSPI_VERID_OFFSET)
#define IMXRT_LPSPI4_PARAM             (IMXRT_LPSPI4_BASE + IMXRT_LPSPI_PARAM_OFFSET)
#define IMXRT_LPSPI4_CR                (IMXRT_LPSPI4_BASE + IMXRT_LPSPI_CR_OFFSET)
#define IMXRT_LPSPI4_SR                (IMXRT_LPSPI4_BASE + IMXRT_LPSPI_SR_OFFSET)
#define IMXRT_LPSPI4_IER               (IMXRT_LPSPI4_BASE + IMXRT_LPSPI_IER_OFFSET)
#define IMXRT_LPSPI4_DER               (IMXRT_LPSPI4_BASE + IMXRT_LPSPI_DER_OFFSET)
#define IMXRT_LPSPI4_CFGR0             (IMXRT_LPSPI4_BASE + IMXRT_LPSPI_CFGR0_OFFSET)
#define IMXRT_LPSPI4_CFGR1             (IMXRT_LPSPI4_BASE + IMXRT_LPSPI_CFGR1_OFFSET)
#define IMXRT_LPSPI4_DMR0              (IMXRT_LPSPI4_BASE + IMXRT_LPSPI_DMR0_OFFSET)
#define IMXRT_LPSPI4_DMR1              (IMXRT_LPSPI4_BASE + IMXRT_LPSPI_DMR1_OFFSET)
#define IMXRT_LPSPI4_CCR               (IMXRT_LPSPI4_BASE + IMXRT_LPSPI_CCR_OFFSET)
#define IMXRT_LPSPI4_FCR               (IMXRT_LPSPI4_BASE + IMXRT_LPSPI_FCR_OFFSET)
#define IMXRT_LPSPI4_FSR               (IMXRT_LPSPI4_BASE + IMXRT_LPSPI_FSR_OFFSET)
#define IMXRT_LPSPI4_TCR               (IMXRT_LPSPI4_BASE + IMXRT_LPSPI_TCR_OFFSET)
#define IMXRT_LPSPI4_TDR               (IMXRT_LPSPI4_BASE + IMXRT_LPSPI_TDR_OFFSET)
#define IMXRT_LPSPI4_RSR               (IMXRT_LPSPI4_BASE + IMXRT_LPSPI_RSR_OFFSET)
#define IMXRT_LPSPI4_RDR               (IMXRT_LPSPI4_BASE + IMXRT_LPSPI_RDR_OFFSET)

/* Register bit definitions *****************************************************************/

/* Version ID Register */

#define LPSPI_VERID_FEATURE_SHIFT      (0)       /* Bits 0-15: Module Identification Number */
#define LPSPI_VERID_FEATURE_MASK       (0xffff << LPSPI_VERID_FEATURE_SHIFT)
#define LPSPI_VERID_MINOR_SHIFT        (16)      /* Bits 16-23: Minor Version Number */
#define LPSPI_VERID_MINOR_MASK         (0xff << LPSPI_VERID_MINOR_SHIFT)
#define LPSPI_VERID_MAJOR_SHIFT        (24)      /* Bits 24-31: Major Version Number */
#define LPSPI_VERID_MAJOR_MASK         (0xff << LPSPI_VERID_MAJOR_SHIFT)

/* Parameter Register */

#define LPSPI_PARAM_TXFIFO_SHIFT       (0)       /* Bits 0-7: Transmit FIFO Size */
#define LPSPI_PARAM_TXFIFO_MASK        (0xff << LPSPI_PARAM_TXFIFO_SHIFT)
#define LPSPI_PARAM_RXFIFO_SHIFT       (8)       /* Bits 8-15: Receive FIFO Size */
#define LPSPI_PARAM_RXFIFO_MASK        (0xff << LPSPI_PARAM_RXFIFO_SHIFT)
#define LPSPI_PARAM_PCSNUM_SHIFT       (16)      /* Bits 16-23: PCS Number */
#define LPSPI_PARAM_PCSNUM_MASK        (0xff << LPSPI_PARAM_PCSNUM_SHIFT)
                                                 /* Bits 24-31: Reserved */

/* Control Register */

#define LPSPI_CR_MEN                   (1 << 0)  /* Bit 0: Module Enable */
#define LPSPI_CR_RST                   (1 << 1)  /* Bit 1: Software Reset */
#define LPSPI_CR_DOZEN                 (1 << 2)  /* Bit 2: Doze mode enable */
#  define LPSPI_CR_DOZEN_EN            (0 << 2)  /*         Module is enabled in Doze mode */
#  define LPSPI_CR_DOZEN_DIS           (1 << 2)  /*         Module is disabled in Doze mode */
#define LPSPI_CR_DBGEN                 (1 << 3)  /* Bit 3: Debug Enable */
                                                 /* Bits 4-7:  Reserved */
#define LPSPI_CR_RTF                   (1 << 8)  /* Bit 8: Reset Transmit FIFO */
#define LPSPI_CR_RRF                   (1 << 9)  /* Bit 9: Reset Receive FIFO */
                                                 /* Bits 10-31:  Reserved */

/* Status Register */

#define LPSPI_SR_TDF                   (1 << 0)  /* Bit 0: Transmit Data Flag */
#define LPSPI_SR_RDF                   (1 << 1)  /* Bit 1: Receive Data Flag */
                                                 /* Bits 2-7:  Reserved */
#define LPSPI_SR_WCF                   (1 << 8)  /* Bit 8: Word Complete Flag */
#define LPSPI_SR_FCF                   (1 << 9)  /* Bit 9: Frame Complete Flag */
#define LPSPI_SR_TCF                   (1 << 10) /* Bit 10: Transfer Complete Flag */
#define LPSPI_SR_TEF                   (1 << 11) /* Bit 11: Transmit Error Flag */
#define LPSPI_SR_REF                   (1 << 12) /* Bit 12: Receive Error Flag */
#define LPSPI_SR_DMF                   (1 << 13) /* Bit 13: Data Match Flag */
                                                 /* Bits 14-23:  Reserved */
#define LPSPI_SR_MBF                   (1 << 24) /* Bit 24: Module Busy Flag */
                                                 /* Bits 25-31:  Reserved */

/* Interrupt Enable Register */

#define LPSPI_IER_TDIE                 (1 << 0)  /* Bit 0: Transmit Data Interrupt Enable */
#define LPSPI_IER_RDIE                 (1 << 1)  /* Bit 1: Receive Data Interrupt Enable */
                                                 /* Bits 2-7:  Reserved */
#define LPSPI_IER_WCIE                 (1 << 8)  /* Bit 8: Word Complete Interrupt Enable */
#define LPSPI_IER_FCIE                 (1 << 9)  /* Bit 9: Frame Complete Interrupt Enable */
#define LPSPI_IER_TCIE                 (1 << 10) /* Bit 10: Transfer Complete Interrupt Enable */
#define LPSPI_IER_TEIE                 (1 << 11) /* Bit 11: Transmit Error Interrupt Enable */
#define LPSPI_IER_REIE                 (1 << 12) /* Bit 12: Receive Error Interrupt Enable */
#define LPSPI_IER_DMIE                 (1 << 13) /* Bit 13: Data Match Interrupt Enable */
                                                 /* Bits 14-31:  Reserved */

/* DMA Enable Register */

#define LPSPI_DER_TDDE                 (1 << 0)  /* Bit 0: Transmit Data DMA Enable */
#define LPSPI_DER_RDDE                 (1 << 1)  /* Bit 1: Receive Data DMA Enable */
                                                 /* Bits 2-31:  Reserved */

/* Configuration Register 0 */

#define LPSPI_CFGR0_HREN               (1 << 0)  /* Bit 0: Host Request Enable */
#define LPSPI_CFGR0_HRPOL              (1 << 1)  /* Bit 1: Host Request Polarity */
#  define LPSPI_CFGR0_HRPOL_LOW        (0 << 1)  /* Active low */
#  define LPSPI_CFGR0_HRPOL_HIGH       (1 << 1)  /* Active high */
#define LPSPI_CFGR0_HRSEL              (1 << 2)  /* Bit 2: Host Request Select */
#  define LPSPI_CFGR0_HRSEL_HREQ       (0 << 2)  /* Host request input is the LPSPI_HREQ pin */
#  define LPSPI_CFGR0_HRSEL_INTR       (1 << 2)  /* Host request input is the input trigger */
                                                 /* Bits 3-7:  Reserved */
#define LPSPI_CFGR0_CIRFIFO            (1 << 8)  /* Bits 8: Circular FIFO Enable */
#define LPSPI_CFGR0_RDMO               (1 << 9)  /* Bits 9: Receive Data Match Only */
#    define LPSPI_CFGR0_RDMO_FIFO      (0 << 9)  /* RD stored in the receive FIFO as in normal operations */
#    define LPSPI_CFGR0_RDMO_DMF       (1 << 9)  /* RD discarded unless the Data Match Flag (DMF) is set */
                                                 /* Bits 10-31:  Reserved */

/* Configuration Register 1 */

#define LPSPI_CFGR1_MASTER             (1 << 0)  /* Bit 0: Master Mode */
#define LPSPI_CFGR1_SAMPLE             (1 << 1)  /* Bit 1: Sample Point */
#    define LPSPI_CFGR1_SAMPLE_SCK     (0 << 1)  /* Input data is sampled on SCK edge */
#    define LPSPI_CFGR1_SAMPLE_DELAY   (1 << 1)  /* Input data is sampled on delayed SCK edge */
#define LPSPI_CFGR1_AUTOPCS            (1 << 2)  /* Bit 2: Automatic PCS enabled */
#define LPSPI_CFGR1_NOSTALL            (1 << 3)  /* Bit 3: No Stall enabled */
                                                 /* Bits 4-7:  Reserved */
#define LPSPI_CFGR1_PCSPOL_SHIFT       (8)       /* Bits 8-11: Peripheral Chip Select Polarity */
#define LPSPI_CFGR1_PCSPOL_MASK        (0xf << LPSPI_CFGR1_PCSPOL_SHIFT)
#    define LPSPI_CFGR1_PCSPOL_LOW     (0 << LPSPI_CFGR1_PCSPOL_SHIFT) /* The Peripheral Chip Select pin PCSx is active low */
#    define LPSPI_CFGR1_PCSPOL_HIGH    (1 << LPSPI_CFGR1_PCSPOL_SHIFT) /* The Peripheral Chip Select pin PCSx is active high */
                                                 /* Bits 12-15:  Reserved */
#define LPSPI_CFGR1_MATCFG_SHIFT       (16)      /* Bits 16-18: Match Configuration */
#define LPSPI_CFGR1_MATCFG_MASK        (7 << LPSPI_CFGR1_MATCFG_SHIFT)
#  define LPSPI_CFGR1_MATCFG(n)        ((uint32_t)(n) << LPSPI_CFGR1_MATCFG_SHIFT)
                                                 /* Bits 19-23:  Reserved */
#define LPSPI_CFGR1_PINCFG_SHIFT       (24)      /* Bits 24-25: Pin Configuration */
#define LPSPI_CFGR1_PINCFG_MASK        (3 << LPSPI_CFGR1_PINCFG_SHIFT)
#  define LPSPI_CFGR1_PINCFG_SIN_SOUT  (0 << LPSPI_CFGR1_PINCFG_SHIFT)  /* SIN is used for input data and SOUT is used for output data */
#  define LPSPI_CFGR1_PINCFG_SIN_SIN   (1 << LPSPI_CFGR1_PINCFG_SHIFT)  /* SIN is used for both input and output data */
#  define LPSPI_CFGR1_PINCFG_SOUT_SOUT (2 << LPSPI_CFGR1_PINCFG_SHIFT)  /* SOUT is used for both input and output data */
#  define LPSPI_CFGR1_PINCFG_SOUT_SIN  (3 << LPSPI_CFGR1_PINCFG_SHIFT)  /* SOUT is used for input data and SIN is used for output data */
#  define LPSPI_CFGR1_PINCFG(n)        ((uint32_t)(n) << LPSPI_CFGR1_PINCFG_SHIFT)
#define LPSPI_CFGR1_OUTCFG             (1 << 26) /* Bit 26: Output Config */
#    define LPSPI_CFGR1_OUTCFG_RETAIN  (0 << 26) /* Output data retains last value when chip select is negated */
#    define LPSPI_CFGR1_OUTCFG_TRISTATE (1 << 26) /* Output data is tristated when chip select is negated */
#define LPSPI_CFGR1_PCSCFG             (1 << 27) /* Bit 27: Peripheral Chip Select Configuration */
#    define LPSPI_CFGR1_PCSCFG_EN      (0 << 27) /* PCS[3:2] are enabled */
#    define LPSPI_CFGR1_PCSCFG_DIS     (1 << 27) /* PCS[3:2] are disabled */
                                                 /* Bits 28-31:  Reserved */

/* Data Match Register 0 */

#define LPSPI_DMR0_MATCH0_SHIFT        (0)       /* Bits 0-31: Match 0 Value */
#define LPSPI_DMR0_MATCH0_MASK         (0xffffffff << LPSPI_DMR0_MATCH0_SHIFT)
#    define LPSPI_DMR0_MATCH0(n)       ((uint32_t)(n) << LPSPI_DMR0_MATCH0_SHIFT)

/* Data Match Register 0 */

#define LPSPI_DMR1_MATCH1_SHIFT        (0)       /* Bits 0-31: Match 1 Value */
#define LPSPI_DMR1_MATCH1_MASK         (0xffffffff << LPSPI_DMR1_MATCH1_SHIFT)
#    define LPSPI_DMR1_MATCH1(n)       ((uint32_t)(n) << LPSPI_DMR1_MATCH1_SHIFT)

/* Clock Configuration Register */

#define LPSPI_CCR_SCKDIV_SHIFT         (0)       /* Bits 0-7: SCK Divider */
#define LPSPI_CCR_SCKDIV_MASK          (0xff << LPSPI_CCR_SCKDIV_SHIFT)
#    define LPSPI_CCR_SCKDIV(n)        ((uint32_t)(n) << LPSPI_CCR_SCKDIV_SHIFT)
#define LPSPI_CCR_DBT_SHIFT            (8)       /* Bits 8-15: Delay Between Transfers */
#define LPSPI_CCR_DBT_MASK             (0xff << LPSPI_CCR_DBT_SHIFT)
#    define LPSPI_CCR_DBT(n)           ((uint32_t)(n) << LPSPI_CCR_DBT_SHIFT)
#define LPSPI_CCR_PCSSCK_SHIFT         (16)      /* Bits 16-23: PCS-to-SCK Delay */
#define LPSPI_CCR_PCSSCK_MASK          (0xff << LPSPI_CCR_PCSSCK_SHIFT)
#    define LPSPI_CCR_PCSSCK(n)        ((uint32_t)(n) << LPSPI_CCR_PCSSCK_SHIFT)
#define LPSPI_CCR_SCKPCS_SHIFT         (24)      /* Bits 24-31: SCK-to-PCS Delay */
#define LPSPI_CCR_SCKPCS_MASK          (0xff << LPSPI_CCR_SCKPCS_SHIFT)
#    define LPSPI_CCR_SCKPCS(n)        ((uint32_t)(n) << LPSPI_CCR_SCKPCS_SHIFT)

/* FIFO Control Register */

#define LPSPI_FCR_TXWATER_SHIFT        (0)       /* Bits 0-3: Transmit FIFO Watermark */
#define LPSPI_FCR_TXWATER_MASK         (0xf << LPSPI_FCR_TXWATER_SHIFT)
#    define LPSPI_FCR_TXWATER(n)       ((uint32_t)(n) << LPSPI_FCR_TXWATER_SHIFT)
                                                 /* Bits 4-7:  Reserved */
                                                 /* Bits 8-15:  Reserved */
#define LPSPI_FCR_RXWATER_SHIFT        (8)       /* Bits 16-19: Receive FIFO Watermark */
#define LPSPI_FCR_RXWATER_MASK         (0xf << LPSPI_FCR_RXWATER_SHIFT)
#    define LPSPI_FCR_RXWATER(n)       ((uint32_t)(n) << LPSPI_FCR_RXWATER_SHIFT)
                                                 /* Bits 20-23:  Reserved */
                                                 /* Bits 24-31:  Reserved */

/* FIFO Status Register */

#define LPSPI_FSR_TXCOUNT_SHIFT        (0)       /* Bits 0-4: Transmit FIFO Count */
#define LPSPI_FSR_TXCOUNT_MASK         (0x1f << LPSPI_FSR_TXCOUNT_SHIFT)
                                                 /* Bits 5-7:  Reserved */
                                                 /* Bits 8-15:  Reserved */
#define LPSPI_FSR_RXCOUNT_SHIFT        (16)      /* Bits 16-20: Receive FIFO Count */
#define LPSPI_FSR_RXCOUNT_MASK         (0x1f << LPSPI_FSR_RXCOUNT_SHIFT)
                                                 /* Bits 21-23:  Reserved */
                                                 /* Bits 24-31:  Reserved */

/* Transmit Command Register */

#define LPSPI_TCR_FRAMESZ_SHIFT        (0)       /* Bits 0-11: Frame Size */
#define LPSPI_TCR_FRAMESZ_MASK         (0xfff << LPSPI_TCR_FRAMESZ_SHIFT)
#    define LPSPI_TCR_FRAMESZ(n)       ((uint32_t)(n) << LPSPI_TCR_FRAMESZ_SHIFT)
                                                 /* Bits 12-15:  Reserved */
#define LPSPI_TCR_WIDTH_SHIFT          (16)      /* Bits 16-17: Transfer Width */
#define LPSPI_TCR_WIDTH_MASK           (3 << LPSPI_TCR_WIDTH_SHIFT)
#    define LPSPI_TCR_WIDTH_1BIT       (0 << LPSPI_TCR_WIDTH_SHIFT)  /* 1 bit transfer */
#    define LPSPI_TCR_WIDTH_2BIT       (1 << LPSPI_TCR_WIDTH_SHIFT)  /* 2 bit transfer */
#    define LPSPI_TCR_WIDTH_4BIT       (2 << LPSPI_TCR_WIDTH_SHIFT)  /* 4 bit transfer */
#define LPSPI_TCR_TXMSK                (1 << 18) /* Bit 18: Transmit Data Mask */
#define LPSPI_TCR_RXMSK                (1 << 19) /* Bit 19: Receive Data Mask */
#define LPSPI_TCR_CONTC                (1 << 20) /* Bit 20: Continuing Command */
#define LPSPI_TCR_CONT                 (1 << 21) /* Bit 21: Continuous Transfer */
#define LPSPI_TCR_BYSW                 (1 << 22) /* Bit 22: Byte Swap */
#define LPSPI_TCR_LSBF                 (1 << 23) /* Bit 23: LSB First */
#    define LPSPI_TCR_MSBF             (0 << 23) /* MSB First */
#define LPSPI_TCR_PCS_SHIFT            (24)      /* Bits 24-25: Peripheral Chip Select */
#define LPSPI_TCR_PCS_MASK             (3 << LPSPI_TCR_PCS_SHIFT)
#    define LPSPI_TCR_PCS_0            (0 << LPSPI_TCR_PCS_SHIFT)  /* Transfer using LPSPI_PCS[0] */
#    define LPSPI_TCR_PCS_1            (1 << LPSPI_TCR_PCS_SHIFT)  /* Transfer using LPSPI_PCS[1] */
#    define LPSPI_TCR_PCS_2            (2 << LPSPI_TCR_PCS_SHIFT)  /* Transfer using LPSPI_PCS[2] */
#    define LPSPI_TCR_PCS_3            (3 << LPSPI_TCR_PCS_SHIFT)  /* Transfer using LPSPI_PCS[3] */
                                                 /* Bit 26:  Reserved */
#define LPSPI_TCR_PRESCALE_SHIFT       (27)      /* Bits 27-29: Prescaler Value */
#define LPSPI_TCR_PRESCALE_MASK        (7 << LPSPI_TCR_PRESCALE_SHIFT)
#    define LPSPI_TCR_PRESCALE_1       (0 << LPSPI_TCR_PRESCALE_SHIFT) /* Divide by 1 */
#    define LPSPI_TCR_PRESCALE_2       (1 << LPSPI_TCR_PRESCALE_SHIFT) /* Divide by 2 */
#    define LPSPI_TCR_PRESCALE_4       (2 << LPSPI_TCR_PRESCALE_SHIFT) /* Divide by 4 */
#    define LPSPI_TCR_PRESCALE_8       (3 << LPSPI_TCR_PRESCALE_SHIFT) /* Divide by 8 */
#    define LPSPI_TCR_PRESCALE_16      (4 << LPSPI_TCR_PRESCALE_SHIFT) /* Divide by 16 */
#    define LPSPI_TCR_PRESCALE_32      (5 << LPSPI_TCR_PRESCALE_SHIFT) /* Divide by 32 */
#    define LPSPI_TCR_PRESCALE_64      (6 << LPSPI_TCR_PRESCALE_SHIFT) /* Divide by 64 */
#    define LPSPI_TCR_PRESCALE_128     (7 << LPSPI_TCR_PRESCALE_SHIFT) /* Divide by 128 */
#    define LPSPI_TCR_PRESCALE(n)      ((uint32_t)(n) << LPSPI_TCR_PRESCALE_SHIFT)
#define LPSPI_TCR_CPHA                 (1 << 30) /* Bit 30: Clock Phase */
#    define LPSPI_TCR_CPHA_CPT_LEAD    (0 << 30) /* Data captured - leading edge of SCK and changed - following edge of SCK */
#    define LPSPI_TCR_CPHA_CPT_FOLLOW  (1 << 30) /* Data changed - leading edge of SCK and captured - following edge of SCK */
#define LPSPI_TCR_CPOL                 (1 << 31) /* Bit 31: Clock Polarity */
#    define LPSPI_TCR_CPOL_INACT_LOW   (0 << 31) /* The inactive state value of SCK is low */
#    define LPSPI_TCR_CPOL_INACT_HIGH  (1 << 31) /* The inactive state value of SCK is high */

/* Transmit Data Register */

#define LPSPI_TDR_DATA_SHIFT           (0)       /* Bits 0-31: Transmit Data */
#    define LPSPI_TCR_DATA(n)          ((uint32_t)(n) << LPSPI_TDR_DATA_SHIFT)

/* Receive Status Register */

#define LPSPI_RSR_SOF                  (1 << 0)  /* Bit 0: Start Of Frame */
#define LPSPI_RSR_RXEMPTY              (1 << 1)  /* Bit 1: RX FIFO Empty */
                                                 /* Bits 2-31:  Reserved */

/* Receive Data Register */

#define LPSPI_RDR_DATA_SHIFT           (0)       /* Bits 0-31: Receive Data */
#define LPSPI_RDR_DATA_MASK            (0xffffffff << LPSPI_RDR_DATA_SHIFT)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_LPSPI_H */
