/****************************************************************************************************
 * arch/arm/src/tiva/hardware/cc13x2_cc26x2/cc13x2_cc26x2_prcm.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_PRCM_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_PRCM_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* PRCM Register Offsets ****************************************************************************/

#define TIVA_PRCM_INFRCLKDIVR_OFFSET     0x0000  /* Infrastructure Clock Division Factor For Run Mode */
#define TIVA_PRCM_INFRCLKDIVS_OFFSET     0x0004  /* Infrastructure Clock Division Factor For Sleep Mode */
#define TIVA_PRCM_INFRCLKDIVDS_OFFSET    0x0008  /* Infrastructure Clock Division Factor For DeepSleep Mode */
#define TIVA_PRCM_VDCTL_OFFSET           0x000c  /* MCU Voltage Domain Control */
#define TIVA_PRCM_CLKLOADCTL_OFFSET      0x0028  /* Load PRCM Settings To CLKCTRL Power Domain */
#define TIVA_PRCM_RFCCLKG_OFFSET         0x002c  /* RFC Clock Gate */
#define TIVA_PRCM_VIMSCLKG_OFFSET        0x0030  /* VIMS Clock Gate */
#define TIVA_PRCM_SECDMACLKGR_OFFSET     0x003c  /* SEC (PKA And TRNG And CRYPTO) And UDMA Clock Gate For Run And All Modes */
#define TIVA_PRCM_SECDMACLKGS_OFFSET     0x0040  /* SEC (PKA And TRNG And CRYPTO) And UDMA Clock Gate For Sleep Mode */
#define TIVA_PRCM_SECDMACLKGDS_OFFSET    0x0044  /* SEC (PKA And TRNG and CRYPTO) And UDMA Clock Gate For Deep Sleep Mode */
#define TIVA_PRCM_GPIOCLKGR_OFFSET       0x0048  /* GPIO Clock Gate For Run And All Modes */
#define TIVA_PRCM_GPIOCLKGS_OFFSET       0x004c  /* GPIO Clock Gate For Sleep Mode */
#define TIVA_PRCM_GPIOCLKGDS_OFFSET      0x0050  /* GPIO Clock Gate For Deep Sleep Mode */
#define TIVA_PRCM_GPTCLKGR_OFFSET        0x0054  /* GPT Clock Gate For Run And All Modes */
#define TIVA_PRCM_GPTCLKGS_OFFSET        0x0058  /* GPT Clock Gate For Sleep Mode */
#define TIVA_PRCM_GPTCLKGDS_OFFSET       0x005c  /* GPT Clock Gate For Deep Sleep Mode */
#define TIVA_PRCM_I2CCLKGR_OFFSET        0x0060  /* I2C Clock Gate For Run And All Modes */
#define TIVA_PRCM_I2CCLKGS_OFFSET        0x0064  /* I2C Clock Gate For Sleep Mode */
#define TIVA_PRCM_I2CCLKGDS_OFFSET       0x0068  /* I2C Clock Gate For Deep Sleep Mode */
#define TIVA_PRCM_UARTCLKGR_OFFSET       0x006c  /* UART Clock Gate For Run And All Modes */
#define TIVA_PRCM_UARTCLKGS_OFFSET       0x0070  /* UART Clock Gate For Sleep Mode */
#define TIVA_PRCM_UARTCLKGDS_OFFSET      0x0074  /* UART Clock Gate For Deep Sleep Mode */
#define TIVA_PRCM_SSICLKGR_OFFSET        0x0078  /* SSI Clock Gate For Run And All Modes */
#define TIVA_PRCM_SSICLKGS_OFFSET        0x007c  /* SSI Clock Gate For Sleep Mode */
#define TIVA_PRCM_SSICLKGDS_OFFSET       0x0080  /* SSI Clock Gate For Deep Sleep Mode */
#define TIVA_PRCM_I2SCLKGR_OFFSET        0x0084  /* I2S Clock Gate For Run And All Modes */
#define TIVA_PRCM_I2SCLKGS_OFFSET        0x0088  /* I2S Clock Gate For Sleep Mode */
#define TIVA_PRCM_I2SCLKGDS_OFFSET       0x008c  /* I2S Clock Gate For Deep Sleep Mode */
#define TIVA_PRCM_SYSBUSCLKDIV_OFFSET    0x00b4  /* Internal */
#define TIVA_PRCM_CPUCLKDIV_OFFSET       0x00b8  /* Internal */
#define TIVA_PRCM_PERBUSCPUCLKDIV_OFFSET 0x00bc  /* Internal */
#define TIVA_PRCM_PERDMACLKDIV_OFFSET    0x00c4  /* Internal */
#define TIVA_PRCM_I2SBCLKSEL_OFFSET      0x00c8  /* I2S Clock Control */
#define TIVA_PRCM_GPTCLKDIV_OFFSET       0x00cc  /* GPT Scalar */
#define TIVA_PRCM_I2SCLKCTL_OFFSET       0x00d0  /* I2S Clock Control */
#define TIVA_PRCM_I2SMCLKDIV_OFFSET      0x00d4  /* MCLK Division Ratio */
#define TIVA_PRCM_I2SBCLKDIV_OFFSET      0x00d8  /* BCLK Division Ratio */
#define TIVA_PRCM_I2SWCLKDIV_OFFSET      0x00dc  /* WCLK Division Ratio */
#define TIVA_PRCM_RESETSECDMA_OFFSET     0x00f0  /* RESET For SEC (PKA And TRNG And CRYPTO) And UDMA */
#define TIVA_PRCM_RESETGPIO_OFFSET       0x00f4  /* RESET For GPIO IPs */
#define TIVA_PRCM_RESETGPT_OFFSET        0x00f8  /* RESET For GPT Ips */
#define TIVA_PRCM_RESETI2C_OFFSET        0x00fc  /* RESET For I2C IPs */
#define TIVA_PRCM_RESETUART_OFFSET       0x0100  /* RESET For UART IPs */
#define TIVA_PRCM_RESETSSI_OFFSET        0x0104  /* RESET For SSI IPs */
#define TIVA_PRCM_RESETI2S_OFFSET        0x0108  /* RESET For I2S IP */
#define TIVA_PRCM_PDCTL0_OFFSET          0x012c  /* Power Domain Control */
#define TIVA_PRCM_PDCTL0RFC_OFFSET       0x0130  /* RFC Power Domain Control */
#define TIVA_PRCM_PDCTL0SERIAL_OFFSET    0x0134  /* SERIAL Power Domain Control */
#define TIVA_PRCM_PDCTL0PERIPH_OFFSET    0x0138  /* PERIPH Power Domain Control */
#define TIVA_PRCM_PDSTAT0_OFFSET         0x0140  /* Power Domain Status */
#define TIVA_PRCM_PDSTAT0RFC_OFFSET      0x0144  /* RFC Power Domain Status */
#define TIVA_PRCM_PDSTAT0SERIAL_OFFSET   0x0148  /* SERIAL Power Domain Status */
#define TIVA_PRCM_PDSTAT0PERIPH_OFFSET   0x014c  /* PERIPH Power Domain Status */
#define TIVA_PRCM_PDCTL1_OFFSET          0x017c  /* Power Domain Control */
#define TIVA_PRCM_PDCTL1CPU_OFFSET       0x0184  /* CPU Power Domain Direct Control */
#define TIVA_PRCM_PDCTL1RFC_OFFSET       0x0188  /* RFC Power Domain Direct Control */
#define TIVA_PRCM_PDCTL1VIMS_OFFSET      0x018c  /* VIMS Mode Direct Control */
#define TIVA_PRCM_PDSTAT1_OFFSET         0x0194  /* Power Manager Status */
#define TIVA_PRCM_PDSTAT1BUS_OFFSET      0x0198  /* BUS Power Domain Direct Read Status */
#define TIVA_PRCM_PDSTAT1RFC_OFFSET      0x019c  /* RFC Power Domain Direct Read Status */
#define TIVA_PRCM_PDSTAT1CPU_OFFSET      0x01a0  /* CPU Power Domain Direct Read Status */
#define TIVA_PRCM_PDSTAT1VIMS_OFFSET     0x01a4  /* VIMS Mode Direct Read Status */
#define TIVA_PRCM_RFCBITS_OFFSET         0x01cc  /* Control To RFC */
#define TIVA_PRCM_RFCMODESEL_OFFSET      0x01d0  /* Selected RFC Mode */
#define TIVA_PRCM_RFCMODEHWOPT_OFFSET    0x01d4  /* Allowed RFC Modes */
#define TIVA_PRCM_PWRPROFSTAT_OFFSET     0x01e0  /* Power Profiler Register */
#define TIVA_PRCM_MCUSRAMCFG_OFFSET      0x021c  /* MCU SRAM configuration */
#define TIVA_PRCM_RAMRETEN_OFFSET        0x0224  /* Memory Retention Control */
#define TIVA_PRCM_OSCIMSC_OFFSET         0x0290  /* Oscillator Interrupt Mask */
#define TIVA_PRCM_OSCRIS_OFFSET          0x0294  /* Oscillator Raw Interrupt Status */
#define TIVA_PRCM_OSCICR_OFFSET          0x0298  /* Oscillator Raw Interrupt Clear */

/* PRCM Register Addresses *************************************************************************/

#define TIVA_PRCM_INFRCLKDIVR            (TIVA_PRCM_BASE + TIVA_PRCM_INFRCLKDIVR_OFFSET)
#define TIVA_PRCM_INFRCLKDIVS            (TIVA_PRCM_BASE + TIVA_PRCM_INFRCLKDIVS_OFFSET)
#define TIVA_PRCM_INFRCLKDIVDS           (TIVA_PRCM_BASE + TIVA_PRCM_INFRCLKDIVDS_OFFSET)
#define TIVA_PRCM_VDCTL                  (TIVA_PRCM_BASE + TIVA_PRCM_VDCTL_OFFSET)
#define TIVA_PRCM_CLKLOADCTL             (TIVA_PRCM_BASE + TIVA_PRCM_CLKLOADCTL_OFFSET)
#define TIVA_PRCM_RFCCLKG                (TIVA_PRCM_BASE + TIVA_PRCM_RFCCLKG_OFFSET)
#define TIVA_PRCM_VIMSCLKG               (TIVA_PRCM_BASE + TIVA_PRCM_VIMSCLKG_OFFSET)
#define TIVA_PRCM_SECDMACLKGR            (TIVA_PRCM_BASE + TIVA_PRCM_SECDMACLKGR_OFFSET)
#define TIVA_PRCM_SECDMACLKGS            (TIVA_PRCM_BASE + TIVA_PRCM_SECDMACLKGS_OFFSET)
#define TIVA_PRCM_SECDMACLKGDS           (TIVA_PRCM_BASE + TIVA_PRCM_SECDMACLKGDS_OFFSET)
#define TIVA_PRCM_GPIOCLKGR              (TIVA_PRCM_BASE + TIVA_PRCM_GPIOCLKGR_OFFSET)
#define TIVA_PRCM_GPIOCLKGS              (TIVA_PRCM_BASE + TIVA_PRCM_GPIOCLKGS_OFFSET)
#define TIVA_PRCM_GPIOCLKGDS             (TIVA_PRCM_BASE + TIVA_PRCM_GPIOCLKGDS_OFFSET)
#define TIVA_PRCM_GPTCLKGR               (TIVA_PRCM_BASE + TIVA_PRCM_GPTCLKGR_OFFSET)
#define TIVA_PRCM_GPTCLKGS               (TIVA_PRCM_BASE + TIVA_PRCM_GPTCLKGS_OFFSET)
#define TIVA_PRCM_GPTCLKGDS              (TIVA_PRCM_BASE + TIVA_PRCM_GPTCLKGDS_OFFSET)
#define TIVA_PRCM_I2CCLKGR               (TIVA_PRCM_BASE + TIVA_PRCM_I2CCLKGR_OFFSET)
#define TIVA_PRCM_I2CCLKGS               (TIVA_PRCM_BASE + TIVA_PRCM_I2CCLKGS_OFFSET)
#define TIVA_PRCM_I2CCLKGDS              (TIVA_PRCM_BASE + TIVA_PRCM_I2CCLKGDS_OFFSET)
#define TIVA_PRCM_UARTCLKGR              (TIVA_PRCM_BASE + TIVA_PRCM_UARTCLKGR_OFFSET)
#define TIVA_PRCM_UARTCLKGS              (TIVA_PRCM_BASE + TIVA_PRCM_UARTCLKGS_OFFSET)
#define TIVA_PRCM_UARTCLKGDS             (TIVA_PRCM_BASE + TIVA_PRCM_UARTCLKGDS_OFFSET)
#define TIVA_PRCM_SSICLKGR               (TIVA_PRCM_BASE + TIVA_PRCM_SSICLKGR_OFFSET)
#define TIVA_PRCM_SSICLKGS               (TIVA_PRCM_BASE + TIVA_PRCM_SSICLKGS_OFFSET)
#define TIVA_PRCM_SSICLKGDS              (TIVA_PRCM_BASE + TIVA_PRCM_SSICLKGDS_OFFSET)
#define TIVA_PRCM_I2SCLKGR               (TIVA_PRCM_BASE + TIVA_PRCM_I2SCLKGR_OFFSET)
#define TIVA_PRCM_I2SCLKGS               (TIVA_PRCM_BASE + TIVA_PRCM_I2SCLKGS_OFFSET)
#define TIVA_PRCM_I2SCLKGDS              (TIVA_PRCM_BASE + TIVA_PRCM_I2SCLKGDS_OFFSET)
#define TIVA_PRCM_SYSBUSCLKDIV           (TIVA_PRCM_BASE + TIVA_PRCM_SYSBUSCLKDIV_OFFSET)
#define TIVA_PRCM_CPUCLKDIV              (TIVA_PRCM_BASE + TIVA_PRCM_CPUCLKDIV_OFFSET)
#define TIVA_PRCM_PERBUSCPUCLKDIV        (TIVA_PRCM_BASE + TIVA_PRCM_PERBUSCPUCLKDIV_OFFSET)
#define TIVA_PRCM_PERDMACLKDIV           (TIVA_PRCM_BASE + TIVA_PRCM_PERDMACLKDIV_OFFSET)
#define TIVA_PRCM_I2SBCLKSEL             (TIVA_PRCM_BASE + TIVA_PRCM_I2SBCLKSEL_OFFSET)
#define TIVA_PRCM_GPTCLKDIV              (TIVA_PRCM_BASE + TIVA_PRCM_GPTCLKDIV_OFFSET)
#define TIVA_PRCM_I2SCLKCTL              (TIVA_PRCM_BASE + TIVA_PRCM_I2SCLKCTL_OFFSET)
#define TIVA_PRCM_I2SMCLKDIV             (TIVA_PRCM_BASE + TIVA_PRCM_I2SMCLKDIV_OFFSET)
#define TIVA_PRCM_I2SBCLKDIV             (TIVA_PRCM_BASE + TIVA_PRCM_I2SBCLKDIV_OFFSET)
#define TIVA_PRCM_I2SWCLKDIV             (TIVA_PRCM_BASE + TIVA_PRCM_I2SWCLKDIV_OFFSET)
#define TIVA_PRCM_RESETSECDMA            (TIVA_PRCM_BASE + TIVA_PRCM_RESETSECDMA_OFFSET)
#define TIVA_PRCM_RESETGPIO              (TIVA_PRCM_BASE + TIVA_PRCM_RESETGPIO_OFFSET)
#define TIVA_PRCM_RESETGPT               (TIVA_PRCM_BASE + TIVA_PRCM_RESETGPT_OFFSET)
#define TIVA_PRCM_RESETI2C               (TIVA_PRCM_BASE + TIVA_PRCM_RESETI2C_OFFSET)
#define TIVA_PRCM_RESETUART              (TIVA_PRCM_BASE + TIVA_PRCM_RESETUART_OFFSET)
#define TIVA_PRCM_RESETSSI               (TIVA_PRCM_BASE + TIVA_PRCM_RESETSSI_OFFSET)
#define TIVA_PRCM_RESETI2S               (TIVA_PRCM_BASE + TIVA_PRCM_RESETI2S_OFFSET)
#define TIVA_PRCM_PDCTL0                 (TIVA_PRCM_BASE + TIVA_PRCM_PDCTL0_OFFSET)
#define TIVA_PRCM_PDCTL0RFC              (TIVA_PRCM_BASE + TIVA_PRCM_PDCTL0RFC_OFFSET)
#define TIVA_PRCM_PDCTL0SERIAL           (TIVA_PRCM_BASE + TIVA_PRCM_PDCTL0SERIAL_OFFSET)
#define TIVA_PRCM_PDCTL0PERIPH           (TIVA_PRCM_BASE + TIVA_PRCM_PDCTL0PERIPH_OFFSET)
#define TIVA_PRCM_PDSTAT0                (TIVA_PRCM_BASE + TIVA_PRCM_PDSTAT0_OFFSET)
#define TIVA_PRCM_PDSTAT0RFC             (TIVA_PRCM_BASE + TIVA_PRCM_PDSTAT0RFC_OFFSET)
#define TIVA_PRCM_PDSTAT0SERIAL          (TIVA_PRCM_BASE + TIVA_PRCM_PDSTAT0SERIAL_OFFSET)
#define TIVA_PRCM_PDSTAT0PERIPH          (TIVA_PRCM_BASE + TIVA_PRCM_PDSTAT0PERIPH_OFFSET)
#define TIVA_PRCM_PDCTL1                 (TIVA_PRCM_BASE + TIVA_PRCM_PDCTL1_OFFSET)
#define TIVA_PRCM_PDCTL1CPU              (TIVA_PRCM_BASE + TIVA_PRCM_PDCTL1CPU_OFFSET)
#define TIVA_PRCM_PDCTL1RFC              (TIVA_PRCM_BASE + TIVA_PRCM_PDCTL1RFC_OFFSET)
#define TIVA_PRCM_PDCTL1VIMS             (TIVA_PRCM_BASE + TIVA_PRCM_PDCTL1VIMS_OFFSET)
#define TIVA_PRCM_PDSTAT1                (TIVA_PRCM_BASE + TIVA_PRCM_PDSTAT1_OFFSET)
#define TIVA_PRCM_PDSTAT1BUS             (TIVA_PRCM_BASE + TIVA_PRCM_PDSTAT1BUS_OFFSET)
#define TIVA_PRCM_PDSTAT1RFC             (TIVA_PRCM_BASE + TIVA_PRCM_PDSTAT1RFC_OFFSET)
#define TIVA_PRCM_PDSTAT1CPU             (TIVA_PRCM_BASE + TIVA_PRCM_PDSTAT1CPU_OFFSET)
#define TIVA_PRCM_PDSTAT1VIMS            (TIVA_PRCM_BASE + TIVA_PRCM_PDSTAT1VIMS_OFFSET)
#define TIVA_PRCM_RFCBITS                (TIVA_PRCM_BASE + TIVA_PRCM_RFCBITS_OFFSET)
#define TIVA_PRCM_RFCMODESEL             (TIVA_PRCM_BASE + TIVA_PRCM_RFCMODESEL_OFFSET)
#define TIVA_PRCM_RFCMODEHWOPT           (TIVA_PRCM_BASE + TIVA_PRCM_RFCMODEHWOPT_OFFSET)
#define TIVA_PRCM_PWRPROFSTAT            (TIVA_PRCM_BASE + TIVA_PRCM_PWRPROFSTAT_OFFSET)
#define TIVA_PRCM_MCUSRAMCFG             (TIVA_PRCM_BASE + TIVA_PRCM_MCUSRAMCFG_OFFSET)
#define TIVA_PRCM_RAMRETEN               (TIVA_PRCM_BASE + TIVA_PRCM_RAMRETEN_OFFSET)
#define TIVA_PRCM_OSCIMSC                (TIVA_PRCM_BASE + TIVA_PRCM_OSCIMSC_OFFSET)
#define TIVA_PRCM_OSCRIS                 (TIVA_PRCM_BASE + TIVA_PRCM_OSCRIS_OFFSET)
#define TIVA_PRCM_OSCICR                 (TIVA_PRCM_BASE + TIVA_PRCM_OSCICR_OFFSET)

/* PRCM Register Bitfield Definitions **************************************************************/

/* Infrastructure Clock Division Factor For Run Mode */
#define PRCM_INFRCLKDIVR_
/* Infrastructure Clock Division Factor For Sleep Mode */
#define PRCM_INFRCLKDIVS_
/* Infrastructure Clock Division Factor For DeepSleep Mode */
#define PRCM_INFRCLKDIVDS_
/* MCU Voltage Domain Control */
#define PRCM_VDCTL_
/* Load PRCM Settings To CLKCTRL Power Domain */
#define PRCM_CLKLOADCTL_
/* RFC Clock Gate */
#define PRCM_RFCCLKG_
/* VIMS Clock Gate */
#define PRCM_VIMSCLKG_
/* SEC (PKA And TRNG And CRYPTO) And UDMA Clock Gate For Run And All Modes */
#define PRCM_SECDMACLKGR_
/* SEC (PKA And TRNG And CRYPTO) And UDMA Clock Gate For Sleep Mode */
#define PRCM_SECDMACLKGS_
/* SEC (PKA And TRNG and CRYPTO) And UDMA Clock Gate For Deep Sleep Mode */
#define PRCM_SECDMACLKGDS_
/* GPIO Clock Gate For Run And All Modes */
#define PRCM_GPIOCLKGR_
/* GPIO Clock Gate For Sleep Mode */
#define PRCM_GPIOCLKGS_
/* GPIO Clock Gate For Deep Sleep Mode */
#define PRCM_GPIOCLKGDS_
/* GPT Clock Gate For Run And All Modes */
#define PRCM_GPTCLKGR_
/* GPT Clock Gate For Sleep Mode */
#define PRCM_GPTCLKGS_
/* GPT Clock Gate For Deep Sleep Mode */
#define PRCM_GPTCLKGDS_
/* I2C Clock Gate For Run And All Modes */
#define PRCM_I2CCLKGR_
/* I2C Clock Gate For Sleep Mode */
#define PRCM_I2CCLKGS_
/* I2C Clock Gate For Deep Sleep Mode */
#define PRCM_I2CCLKGDS_
/* UART Clock Gate For Run And All Modes */
#define PRCM_UARTCLKGR_
/* UART Clock Gate For Sleep Mode */
#define PRCM_UARTCLKGS_
/* UART Clock Gate For Deep Sleep Mode */
#define PRCM_UARTCLKGDS_
/* SSI Clock Gate For Run And All Modes */
#define PRCM_SSICLKGR_
/* SSI Clock Gate For Sleep Mode */
#define PRCM_SSICLKGS_
/* SSI Clock Gate For Deep Sleep Mode */
#define PRCM_SSICLKGDS_
/* I2S Clock Gate For Run And All Modes */
#define PRCM_I2SCLKGR_
/* I2S Clock Gate For Sleep Mode */
#define PRCM_I2SCLKGS_
/* I2S Clock Gate For Deep Sleep Mode */
#define PRCM_I2SCLKGDS_
/* Internal */
#define PRCM_SYSBUSCLKDIV_
/* Internal */
#define PRCM_CPUCLKDIV_
/* Internal */
#define PRCM_PERBUSCPUCLKDIV_
/* Internal */
#define PRCM_PERDMACLKDIV_
/* I2S Clock Control */
#define PRCM_I2SBCLKSEL_
/* GPT Scalar */
#define PRCM_GPTCLKDIV_
/* I2S Clock Control */
#define PRCM_I2SCLKCTL_
/* MCLK Division Ratio */
#define PRCM_I2SMCLKDIV_
/* BCLK Division Ratio */
#define PRCM_I2SBCLKDIV_
/* WCLK Division Ratio */
#define PRCM_I2SWCLKDIV_
/* RESET For SEC (PKA And TRNG And CRYPTO) And UDMA */
#define PRCM_RESETSECDMA_
/* RESET For GPIO IPs */
#define PRCM_RESETGPIO_
/* RESET For GPT Ips */
#define PRCM_RESETGPT_
/* RESET For I2C IPs */
#define PRCM_RESETI2C_
/* RESET For UART IPs */
#define PRCM_RESETUART_
/* RESET For SSI IPs */
#define PRCM_RESETSSI_
/* RESET For I2S IP */
#define PRCM_RESETI2S_
/* Power Domain Control */
#define PRCM_PDCTL0_
/* RFC Power Domain Control */
#define PRCM_PDCTL0RFC_
/* SERIAL Power Domain Control */
#define PRCM_PDCTL0SERIAL_
/* PERIPH Power Domain Control */
#define PRCM_PDCTL0PERIPH_
/* Power Domain Status */
#define PRCM_PDSTAT0_
/* RFC Power Domain Status */
#define PRCM_PDSTAT0RFC_
/* SERIAL Power Domain Status */
#define PRCM_PDSTAT0SERIAL_
/* PERIPH Power Domain Status */
#define PRCM_PDSTAT0PERIPH_
/* Power Domain Control */
#define PRCM_PDCTL1_
/* CPU Power Domain Direct Control */
#define PRCM_PDCTL1CPU_
/* RFC Power Domain Direct Control */
#define PRCM_PDCTL1RFC_
/* VIMS Mode Direct Control */
#define PRCM_PDCTL1VIMS_
/* Power Manager Status */
#define PRCM_PDSTAT1_
/* BUS Power Domain Direct Read Status */
#define PRCM_PDSTAT1BUS_
/* RFC Power Domain Direct Read Status */
#define PRCM_PDSTAT1RFC_
/* CPU Power Domain Direct Read Status */
#define PRCM_PDSTAT1CPU_
/* VIMS Mode Direct Read Status */
#define PRCM_PDSTAT1VIMS_
/* Control To RFC */
#define PRCM_RFCBITS_
/* Selected RFC Mode */
#define PRCM_RFCMODESEL_
/* Allowed RFC Modes */
#define PRCM_RFCMODEHWOPT_
/* Power Profiler Register */
#define PRCM_PWRPROFSTAT_
/* MCU SRAM configuration */
#define PRCM_MCUSRAMCFG_
/* Memory Retention Control */
#define PRCM_RAMRETEN_
/* Oscillator Interrupt Mask */
#define PRCM_OSCIMSC_
/* Oscillator Raw Interrupt Status */
#define PRCM_OSCRIS_
/* Oscillator Raw Interrupt Clear */
#define PRCM_OSCICR_

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Function Prototypes
 ****************************************************************************************************/

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_PRCM_H */
