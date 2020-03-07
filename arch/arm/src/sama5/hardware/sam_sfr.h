/************************************************************************************
 * arch/arm/src/sama5/hardware/sam_sfr.h
 *
 *   Copyright (C) 2013-2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_SFR_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_SFR_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/sam_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* SFR Register Offsets *************************************************************/

                                           /* 0x0000: Reserved */
#define SAM_SFR_DDRCFG_OFFSET       0x0004 /* DDR Configuration register */
                                           /* 0x0008-0x000c: Reserved */
#define SAM_SFR_OHCIICR_OFFSET      0x0010 /* OHCI Interrupt Configuration Register */
#define SAM_SFR_OHCIISR_OFFSET      0x0014 /* OHCI Interrupt Status Register */
                                           /* 0x0018-0x001c: Reserved */
#define SAM_SFR_SECURE_OFFSET       0x0028 /* Security Configuration Register */
                                           /* 0x002c: Reserved */
#define SAM_SFR_UTMICKTRIM_OFFSET   0x0030 /* UTMI Clock Trimming Register */

#if defined(ATSAMA5D2) || defined(ATSAMA5D4)
#  define SAM_SFR_UTMIHSTRIM_OFFSET 0x0034 /* UTMI High Speed Trimming Register */
#  define SAM_SFR_UTMIFSTRIM_OFFSET 0x0038 /* UTMI Full Speed Trimming Register */
#  define SAM_SFR_UTMISWAP_OFFSET   0x003c /* UTMI DP/DM Pin Swapping Register */
#endif

#define SAM_SFR_EBICFG_OFFSET       0x0040 /* EBI Configuration Register */

#ifdef ATSAMA5D2
#  define SAM_SFR_ANACFG_OFFSET     0x0044 /* Analog Configuration Register */
#  define SAM_SFR_CAN_OFFSET        0x0048 /* CAN Memories Address-based Register */
#endif

#if defined(ATSAMA5D2) || defined(ATSAMA5D4)
#  define SAM_SFR_SN0_OFFSET        0x004c /* Serial Number 0 Register */
#  define SAM_SFR_SN1_OFFSET        0x0050 /* Serial Number 1 Register */
#  define SAM_SFR_AICREDIR_OFFSET   0x0054 /* AIC Redirection Register */
#endif

#ifdef ATSAMA5D2
#  define SAM_SFR_L2CCHRAMC_OFFSET  0x0044 /* L2CC HRAMC1 */
                                           /* 0x005c-0x008c: Reserved */
#  define SAM_SFR_I2SCLKSEL_OFFSET  0x0090 /* I2SC Register */
#  define SAM_SFR_QSPICLK_OFFSET    0x0094 /* QSPI Clock Pad Supply Select Register */
#endif
                                           /* 0x0098-0x3ffc: Reserved */

/* SFR Register Addresses ***********************************************************/

#define SAM_SFR_DDRCFG              (SAM_SFR_VBASE+SAM_SFR_DDRCFG_OFFSET) /* REVISIT */
#define SAM_SFR_OHCIICR             (SAM_SFR_VBASE+SAM_SFR_OHCIICR_OFFSET)
#define SAM_SFR_OHCIISR             (SAM_SFR_VBASE+SAM_SFR_OHCIISR_OFFSET)
#define SAM_SFR_SECURE              (SAM_SFR_VBASE+SAM_SFR_SECURE_OFFSET)
#define SAM_SFR_UTMICKTRIM          (SAM_SFR_VBASE+SAM_SFR_UTMICKTRIM_OFFSET)

#if defined(ATSAMA5D2) || defined(ATSAMA5D4)
#  define SAM_SFR_UTMIHSTRIM        (SAM_SFR_VBASE+SAM_SFR_UTMIHSTRIM_OFFSET)
#  define SAM_SFR_UTMIFSTRIM        (SAM_SFR_VBASE+SAM_SFR_UTMIFSTRIM_OFFSET)
#  define SAM_SFR_UTMISWAP          (SAM_SFR_VBASE+SAM_SFR_UTMISWAP_OFFSET)
#endif

#define SAM_SFR_EBICFG              (SAM_SFR_VBASE+SAM_SFR_EBICFG_OFFSET)

#ifdef ATSAMA5D2
#  define SAM_SFR_ANACFG            (SAM_SFR_VBASE+SAM_SFR_ANACFG_OFFSET)
#  define SAM_SFR_CAN               (SAM_SFR_VBASE+SAM_SFR_CAN_OFFSET)
#endif

#if defined(ATSAMA5D2) || defined(ATSAMA5D4)
#  define SAM_SFR_SN0               (SAM_SFR_VBASE+SAM_SFR_SN0_OFFSET)
#  define SAM_SFR_SN1               (SAM_SFR_VBASE+SAM_SFR_SN1_OFFSET)
#  define SAM_SFR_AICREDIR          (SAM_SFR_VBASE+SAM_SFR_AICREDIR_OFFSET)
#endif

#ifdef ATSAMA5D2
#  define SAM_SFR_L2CCHRAMC         (SAM_SFR_VBASE+SAM_SFR_L2CCHRAMC_OFFSET)
#  define SAM_SFR_I2SCLKSEL         (SAM_SFR_VBASE+SAM_SFR_I2SCLKSEL_OFFSET)
#  define SAM_SFR_QSPICLK           (SAM_SFR_VBASE+SAM_SFR_QSPICLK_OFFSET)
#endif

/* SFR Register Bit Definitions *****************************************************/

/* DDR Configuration register */

#define SFR_FDQIEN                  (1 << 16) /* Force DDR_DQ input buffer always on */
#define SFR_FDQSIEN                 (1 << 17) /* Force DDR_DQS input buffer always on */

/* OHCI Interrupt Configuration Register */

#define SFR_OHCIICR_RES(n)          (1 << (n)) /* Bit 0:  USB port n reset, n=0..2 */
#  define SFR_OHCIICR_RES0          (1 << 0)  /* Bit 0:  USB port 0 reset */
#  define SFR_OHCIICR_RES1          (1 << 1)  /* Bit 1:  USB port 1 reset */
#  define SFR_OHCIICR_RES2          (1 << 2)  /* Bit 2:  USB port 2 reset */
#define SFR_OHCIICR_ARIE            (1 << 4)  /* Bit 4:  OHCI asynchronous resume interrupt enable */
#define SFR_OHCIICR_APPSTART        (0)       /* Bit 5:  Reserved, must write 0 */

#ifdef ATSAMA5D2
#  define SFR_OHCIICR_SUSPEND(n)    (1 << ((n)+8))
#    define SFR_OHCIICR_SUSPENDA    (1 << 8)  /* Bit 8:  Suspend USB port A */
#    define SFR_OHCIICR_SUSPENDB    (1 << 9)  /* Bit 9:  Suspend USB port B */
#    define SFR_OHCIICR_SUSPENDC    (1 << 10) /* Bit 10: Suspend USB port C */
#endif

#define SFR_OHCIICR_UDPPUDIS        (1 << 23) /* Bit 23: USB device pull-up disable */

#ifdef ATSAMA5D2
#  define SFR_OHCIICR_HSIC_SEL      (0)       /* Bit 27: Reserved (must write 0) */
#endif

/* OHCI Interrupt Status Register */

#define SFR_OHCIISR_RIS0            (1 << 0)  /* Bit 0:  USB port 0 resume detected */
#define SFR_OHCIISR_RIS1            (1 << 1)  /* Bit 1:  USB port 1 resume detected */
#define SFR_OHCIISR_RIS2            (1 << 2)  /* Bit 2:  USB port 2 resume detected */

/* Security Configuration Register */

#define SFR_SECURE_ROM              (1 << 0)  /* Bit 0:  Disable Access to ROM Code */
#define SFR_SECURE_FUSE             (1 << 8)  /* Bit 8:  Disable Access to Fuse Controller */

/* UTMI Clock Trimming Register */

#define SFR_UTMICKTRIM_FREQ_SHIFT   (0)       /* Bits 0-1: UTMI Reference Clock Frequency */
#define SFR_UTMICKTRIM_FREQ_MASK    (3 << SFR_UTMICKTRIM_FREQ_SHIFT)
#  define SFR_UTMICKTRIM_FREQ_12MHZ (0 << SFR_UTMICKTRIM_FREQ_SHIFT) /* 12 MHz reference clock */
#  define SFR_UTMICKTRIM_FREQ_16MHZ (1 << SFR_UTMICKTRIM_FREQ_SHIFT) /* 16 MHz reference clock */
#  define SFR_UTMICKTRIM_FREQ_24MHZ (2 << SFR_UTMICKTRIM_FREQ_SHIFT) /* 24 MHz reference clock */
#ifndef ATSAMA5D2
#  define SFR_UTMICKTRIM_FREQ_48MHZ (3 << SFR_UTMICKTRIM_FREQ_SHIFT) /* 48 MHz reference clock */
#endif

#if defined(ATSAMA5D2) || defined(ATSAMA5D4)
#  define SFR_UTMICKTRIM_VBG_SHIFT  (16)     /* Bits 16-19: UTMI Band Gap Voltage Trimming */
#  define SFR_UTMICKTRIM_VBG_MASK   (15 << SFR_UTMICKTRIM_VBG_SHIFT)
#    define SFR_UTMICKTRIM_VBG(n)   ((uint32_t)(n) << SFR_UTMICKTRIM_VBG_SHIFT)
#endif

#if defined(ATSAMA5D2) || defined(ATSAMA5D4)
/* UTMI High Speed Trimming Register */

#  define SFR_UTMIHSTRIM_SQUELCH_SHIFT (0)   /* Bits 0-2: UTMI HS SQUELCH Voltage Trimming */
#  define SFR_UTMIHSTRIM_SQUELCH_MASK  (7 << SFR_UTMIHSTRIM_SQUELCH_SHIFT)
#    define SFR_UTMIHSTRIM_SQUELCH(n)  ((uint32_t)(n) << SFR_UTMIHSTRIM_SQUELCH_SHIFT)
#  define SFR_UTMIHSTRIM_DISC_SHIFT    (4)   /* Bits 4-6: UTMI Disconnect Voltage Trimming */
#  define SFR_UTMIHSTRIM_DISC_MASK     (7 << SFR_UTMIHSTRIM_DISC_SHIFT)
#    define SFR_UTMIHSTRIM_DISC(n)     ((uint32_t)(n) << SFR_UTMIHSTRIM_DISC_SHIFT)
#  define SFR_UTMIHSTRIM_SLOPE0_SHIFT  (8)   /* Bits 8-10: UTMI HS PORT0 Transceiver Slope Trimming */
#  define SFR_UTMIHSTRIM_SLOPE0_MASK   (7 << SFR_UTMIHSTRIM_SLOPE0_SHIFT)
#    define SFR_UTMIHSTRIM_SLOPE0(n)   ((uint32_t)(n) << SFR_UTMIHSTRIM_SLOPE0_SHIFT)
#  define SFR_UTMIHSTRIM_SLOPE1_SHIFT  (12)  /* Bits 12-14: UTMI HS PORT1 Transceiver Slope Trimming */
#  define SFR_UTMIHSTRIM_SLOPE1_MASK   (7 << SFR_UTMIHSTRIM_SLOPE1_SHIFT)
#    define SFR_UTMIHSTRIM_SLOPE1(n)   ((uint32_t)(n) << SFR_UTMIHSTRIM_SLOPE1_SHIFT)
#  define SFR_UTMIHSTRIM_SLOPE2_SHIFT  (16)  /* Bits 16-18: UTMI HS PORT2 Transceiver Slope Trimming */
#  define SFR_UTMIHSTRIM_SLOPE2_MASK   (7 << SFR_UTMIHSTRIM_SLOPE2_SHIFT)
#    define SFR_UTMIHSTRIM_SLOPE2(n)   ((uint32_t)(n) << SFR_UTMIHSTRIM_SLOPE2_SHIFT)
#endif

#if defined(ATSAMA5D2) || defined(ATSAMA5D4)
/* UTMI Full Speed Trimming Register */

#  define SFR_UTMIFSTRIM_RISE_SHIFT (0)      /* Bits 0-2: FS Transceiver Output Rising Slope Trimming */
#  define SFR_UTMIFSTRIM_RISE_MASK  (7 << SFR_UTMIHSTRIM_SLOPE2_SHIFT)
#    define SFR_UTMIFSTRIM_RISE(n)  ((uint32_t)(n) << SFR_UTMIHSTRIM_SLOPE2_SHIFT)
#  define SFR_UTMIFSTRIM_FALL_SHIFT (4)      /* Bits 4-6: FS Transceiver Output Falling Slope Trimming */
#  define SFR_UTMIFSTRIM_FALL_MASK  (7 << SFR_UTMIHSTRIM_SLOPE2_SHIFT)
#    define SFR_UTMIFSTRIM_FALL(n)  ((uint32_t)(n) << SFR_UTMIHSTRIM_SLOPE2_SHIFT)
#  define SFR_UTMIFSTRIM_XCVR_SHIFT (8)      /* Bits 8-9: FS Transceiver Crossover Voltage Trimming */
#  define SFR_UTMIFSTRIM_XCVR_MASK  (3 << SFR_UTMIHSTRIM_SLOPE2_SHIFT)
#    define SFR_UTMIFSTRIM_XCVR(n)  ((uint32_t)(n) << SFR_UTMIHSTRIM_SLOPE2_SHIFT)
#  define SFR_UTMIFSTRIM_ZN_SHIFT   (16)     /* Bits 16-18: FS Transceiver NMOS Impedance Trimming */
#  define SFR_UTMIFSTRIM_ZN_MASK    (7 << SFR_UTMIHSTRIM_SLOPE2_SHIFT)
#    define SFR_UTMIFSTRIM_ZN(n)    ((uint32_t)(n) << SFR_UTMIHSTRIM_SLOPE2_SHIFT)
#  define SFR_UTMIFSTRIM_ZP_SHIFT   (20)     /* Bits 20-22: FS Transceiver PMOS Impedance Trimming */
#  define SFR_UTMIFSTRIM_ZP_MASK    (7 << SFR_UTMIHSTRIM_SLOPE2_SHIFT)
#    define SFR_UTMIFSTRIM_ZP(n)    ((uint32_t)(n) << SFR_UTMIHSTRIM_SLOPE2_SHIFT)
#endif

#if defined(ATSAMA5D2) || defined(ATSAMA5D4)
/* UTMI DP/DM Pin Swapping Register */

#  define SFR_UTMISWAP_PORT(n)     (1 << (n)) /* Bit n:  PORT n DP/DM Pin Swapping */
#    define SFR_UTMISWAP_PORT0     (1 << 0)  /* Bit 0:  PORT 0 DP/DM Pin Swapping */
#    define SFR_UTMISWAP_PORT1     (1 << 1)  /* Bit 1:  PORT 1 DP/DM Pin Swapping */
#    define SFR_UTMISWAP_PORT2     (1 << 2)  /* Bit 2:  PORT 2 DP/DM Pin Swapping */
#endif

/* EBI Configuration Register */

#define SFR_EBICFG_DRIVE_LOW        (0)       /* LOW Low drive level */
#define SFR_EBICFG_DRIVE_MEDIUM     (2)       /* MEDIUM Medium drive level */
#define SFR_EBICFG_DRIVE_HIGH       (3)       /* HIGH High drive level */

#define SFR_EBICFG_PULL_UP          (0)       /* Pull-up */
#define SFR_EBICFG_PULL_NONE        (1)       /* No Pull */
#define SFR_EBICFG_PULL_DOWN        (3)       /* Pull-down */

#define SFR_EBICFG_DRIVE0_SHIFT     (0)       /* Bits 0-1: EBI Pins Drive Level */
#define SFR_EBICFG_DRIVE0_MASK      (3 << SFR_EBICFG_DRIVE0_SHIFT)
#  define SFR_EBICFG_DRIVE0(n)      ((n) << SFR_EBICFG_DRIVE0_SHIFT)
#define SFR_EBICFG_PULL0_SHIFT      (2)       /* Bits 2-3: EBI Pins Pull Value */
#define SFR_EBICFG_PULL0_MASK       (3 << SFR_EBICFG_PULL0_SHIFT)
#  define SFR_EBICFG_PULL0(n)       ((n) << SFR_EBICFG_PULL0_SHIFT)
#define SFR_EBICFG_SCH0             (1 << 4)  /* Bit 4:  EBI Pins Schmitt Trigger */
#define SFR_EBICFG_DRIVE1_SHIFT     (8)       /* Bits 8-9: EBI Pins Drive Level */
#define SFR_EBICFG_DRIVE1_MASK      (3 << SFR_EBICFG_DRIVE1_SHIFT)
#  define SFR_EBICFG_DRIVE1(n)      ((n) << SFR_EBICFG_DRIVE1_SHIFT)
#define SFR_EBICFG_PULL1_SHIFT      (10)      /* Bits 10-11: EBI Pins Pull Value */
#define SFR_EBICFG_PULL1_MASK       (3 << SFR_EBICFG_PULL1_SHIFT)
#  define SFR_EBICFG_PULL1(n)       ((n) << SFR_EBICFG_PULL1_SHIFT)
#define SFR_EBICFG_SCH1             (1 << 12) /* Bit 12: EBI Pins Schmitt Trigger */

#ifdef ATSAMA5D3
#  define SFR_EBICFG_BMS            (1 << 16) /* Bit 16:  BMS Sampled Value (Read Only) */
#endif

#ifdef ATSAMA5D2
/* Analog Configuration Register */

#  define SFR_ANACFG_SMDDREN        (1 << 0)  /* Bit 0: DDR Supply Monitor Enable */
#endif

#ifdef ATSAMA5D2
/* CAN Memories Address-based Register */

#  define SFR_CAN0_EXTMEMADDR_SHIFT (0)        /* Bits 0-15: MSB CAN0 Base Address */
#  define SFR_CAN0_EXTMEMADDR_MASK  (0xffff << SFR_CAN0_EXTMEMADDR_SHIFT)
#    define SFR_CAN0_EXTMEMADDR(n)  ((uint32_t)(n) << SFR_CAN0_EXTMEMADDR_SHIFT)
#  define SFR_CAN1_EXTMEMADDR_SHIFT (16)       /* Bits 16-31: MSB CAN0 Base Address */
#  define SFR_CAN1_EXTMEMADDR_MASK  (0xffff << SFR_CAN1_EXTMEMADDR_SHIFT)
#    define SFR_CAN1_EXTMEMADDR(n)  ((uint32_t)(n) << SFR_CAN1_EXTMEMADDR_SHIFT)
#endif

#if defined(ATSAMA5D2) || defined(ATSAMA5D4)
/* Serial Number 0 Register (32-bit value) */
/* Serial Number 1 Register (32-bit value) */

/* AIC Redirection Register */

#  define SFR_AICREDIR_NSAIC        (1 << 0)      /* Bit 0: Interrupt redirection to Non-Secure AIC */
#    define SFR_AICREDIR_ENABLE     (1 << 0)      /* Bit 0: 1=All interrupts to AIC */
#    define SFR_AICREDIR_DISABLE    (0)           /* Bit 0: 0=Secure interrupts to SAIC */
#    define SFR_AICREDIR_KEY        (0xb6d81c4d)  /* Bits 1-31: Access key */
#endif

#ifdef ATSAMA5D2
/* L2CC HRAMC1 */

#  define SFR_L2CCHRAMC_SRAMSEL     (1 << 0)  /* Bit 0: SRAM selector */
#    define SFR_L2CCHRAMC_SRAM      (0)       /* Bit 0: 0=Selects SRAM */
#    define SFR_L2CCHRAMC_L2CC      (1 << 0)  /* Bit 0: 1=Selects L2CC */
#endif

#ifdef ATSAMA5D2
/* I2SC Register */

#  define SFR_I2S_CLKSEL0           (1 << 0)  /* Bit 0: Clock selection 0 */
#    define SFR_I2S_CLKSEL0_GCLK    (0)       /* Bit 0: 0=Selects GCLK */
#    define SFR_I2S_CLKSEL0_PCLK    (1 << 0)  /* Bit 0: 1=Selects PCLK */
#  define SFR_I2S_CLKSEL1           (1 << 1)  /* Bit 1: Clock selection 1 */
#    define SFR_I2S_CLKSEL1_GCLK    (0)       /* Bit 0: 0=Selects GCLK */
#    define SFR_I2S_CLKSEL1_PCLK    (1 << 1)  /* Bit 0: 1=Selects PCLK */
#endif

#ifdef ATSAMA5D2
/* QSPI Clock Pad Supply Select Register */

#  define SFR_QSPICLK_SUPSEL        (1 << 0)  /* Bit 0: Supply selection */
#    define SFR_QSPICLK_1p8V        (0)       /* Bit 0: 0=1.8V */
#    define SFR_QSPICLK_3p3V        (1 << 0)  /* Bit 0: 1=3.3V */
#endif

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_SFR_H */
