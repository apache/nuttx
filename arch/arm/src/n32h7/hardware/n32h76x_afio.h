/****************************************************************************
 * arch/arm/src/n32h7/hardware/n32h76x_afio.h
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

#ifndef __ARCH_ARM_SRC_N32H7_HARDWARE_N32H76X_AFIO_H
#define __ARCH_ARM_SRC_N32H7_HARDWARE_N32H76X_AFIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

 #include <nuttx/config.h>
 #include "chip.h"
 #include "hardware/n32h7_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define N32_AFIO_XSPI1_NON_OFFSET(n)       (0x0008 + (n) * 4)
#define N32_AFIO_EXTI_CFG_OFFSET(n)        (0x0018 + (n) * 4)
#define N32_AFIO_TOL5V_CFG0_2_OFFSET(n)    (0x0028 + (n) * 4)
#define N32_AFIO_TOL5V_CFG3_6_OFFSET(n)    (0x003C + (n) * 4)
#define N32_AFIO_EFT_CFG_OFFSET(n)         (0x0054 + (n) * 4)
#define N32_AFIO_DIGEFT_CFG_OFFSET(n)      (0x006C + (n) * 4)
#define N32_AFIO_SHRT1_EXEV_CFG_OFFSET(n)  (0x0084 + (n) * 4)
#define N32_AFIO_SHRT2_EXEV_CFG_OFFSET(n)  (0x008C + (n) * 4)
#define N32_AFIO_SDRAM_HSMOD_CFG_OFFSET(n) (0x0098 + (n) * 4)
#define N32_AFIO_SDRAM_VREF_EN_OFFSET(n)   (0x00D8 + (n) * 4)
#define N32_AFIO_SDRAMDSN_CFG_OFFSET(n)    (0x00E4 + (n) * 4)
#define N32_AFIO_SDRAMDSP_CFG_OFFSET(n)    (0x00F0 + (n) * 4)

#define N32_AFIO_RMP_CFG_OFFSET           0x0000    /* Mapping Configuration Register */
#define N32_AFIO_FILTER_CFG_OFFSET        0x0004    /* IOM Filter Control Register */
#define N32_AFIO_XSPI1_NON0_OFFSET        0x0008    /* XSPI1 Nonce Register 0 */
#define N32_AFIO_XSPI1_NON1_OFFSET        0x000C    /* XSPI1 Nonce Register 1 */
#define N32_AFIO_XSPI1_NON2_OFFSET        0x0010    /* XSPI1 Nonce Register 2 */
#define N32_AFIO_ADCRMP_CFG_OFFSET        0x0014    /* ADC Remap Configure Register */
#define N32_AFIO_EXTI_CFG0_OFFSET         0x0018    /* External Interrupt Config Reg 0 */
#define N32_AFIO_EXTI_CFG1_OFFSET         0x001C    /* External Interrupt Config Reg 1 */
#define N32_AFIO_EXTI_CFG2_OFFSET         0x0020    /* External Interrupt Config Reg 2 */
#define N32_AFIO_EXTI_CFG3_OFFSET         0x0024    /* External Interrupt Config Reg 3 */
#define N32_AFIO_TOL5V_CFG0_OFFSET        0x0028    /* IO Port 5V Tolerance Config Reg 0 */
#define N32_AFIO_TOL5V_CFG1_OFFSET        0x002C    /* IO Port 5V Tolerance Config Reg 1 */
#define N32_AFIO_TOL5V_CFG2_OFFSET        0x0030    /* IO Port 5V Tolerance Config Reg 2 */
#define N32_AFIO_SHRT1_FALT_CFG_OFFSET    0x0034    /* SHRTIM1 Fat Config Register */
#define N32_AFIO_SHRT2_FALT_CFG_OFFSET    0x0038    /* SHRTIM2 Fat Config Register */
#define N32_AFIO_TOL5V_CFG3_OFFSET        0x003C    /* IO Port 5V Tolerance Config Reg 3 */
#define N32_AFIO_TOL5V_CFG4_OFFSET        0x0044    /* IO Port 5V Tolerance Config Reg 4 */
#define N32_AFIO_TOL5V_CFG5_OFFSET        0x0048    /* IO Port 5V Tolerance Config Reg 5 */
#define N32_AFIO_TOL5V_CFG6_OFFSET        0x004C    /* IO Port 5V Tolerance Config Reg 6 */
#define N32_AFIO_EFT_CFG0_OFFSET          0x0054    /* IO Port Analog Filtering Config Reg 0 */
#define N32_AFIO_EFT_CFG1_OFFSET          0x0058    /* IO Port Analog Filtering Config Reg 1 */
#define N32_AFIO_EFT_CFG2_OFFSET          0x005C    /* IO Port Analog Filtering Config Reg 2 */
#define N32_AFIO_EFT_CFG3_OFFSET          0x0060    /* IO Port Analog Filtering Config Reg 3 */
#define N32_AFIO_EFT_CFG4_OFFSET          0x0064    /* IO Port Analog Filtering Config Reg 4 */
#define N32_AFIO_EFT_CFG5_OFFSET          0x0068    /* IO Port Analog Filtering Config Reg 5 */
#define N32_AFIO_DIGEFT_CFG0_OFFSET       0x006C    /* IO Port Digital Filtering Config Reg 0 */
#define N32_AFIO_DIGEFT_CFG1_OFFSET       0x0070    /* IO Port Digital Filtering Config Reg 1 */
#define N32_AFIO_DIGEFT_CFG2_OFFSET       0x0074    /* IO Port Digital Filtering Config Reg 2 */
#define N32_AFIO_DIGEFT_CFG3_OFFSET       0x0078    /* IO Port Digital Filtering Config Reg 3 */
#define N32_AFIO_DIGEFT_CFG4_OFFSET       0x007C    /* IO Port Digital Filtering Config Reg 4 */
#define N32_AFIO_DIGEFT_CFG5_OFFSET       0x0080    /* IO Port Digital Filtering Config Reg 5 */
#define N32_AFIO_SHRT1_EXEV_CFG0_OFFSET   0x0084    /* SHRTIM1 External Event Config Reg 0 */
#define N32_AFIO_SHRT1_EXEV_CFG1_OFFSET   0x0088    /* SHRTIM1 External Event Config Reg 1 */
#define N32_AFIO_SHRT2_EXEV_CFG0_OFFSET   0x008C    /* SHRTIM2 External Event Config Reg 0 */
#define N32_AFIO_SHRT2_EXEV_CFG1_OFFSET   0x0090    /* SHRTIM2 External Event Config Reg 1 */
#define N32_AFIO_SIP_PUPD_OFFSET          0x0094    /* SIP Pl-up/Pl-down Config Register */
#define N32_AFIO_SDRAM_HSMOD_CFG0_OFFSET  0x0098    /* SDRAM HS MODE Config Register 0 */
#define N32_AFIO_SDRAM_HSMOD_CFG1_OFFSET  0x009C    /* SDRAM HS MODE Config Register 1 */
#define N32_AFIO_SDRAM_HSMOD_CFG2_OFFSET  0x00A0    /* SDRAM HS MODE Config Register 2 */
#define N32_AFIO_SDRAM_HSMOD_CFG3_OFFSET  0x00A4    /* SDRAM HS MODE Config Register 3 */
#define N32_AFIO_SDRAM_HSMOD_CFG4_OFFSET  0x00A8    /* SDRAM HS MODE Config Register 4 */
#define N32_AFIO_SIP_SR_OFFSET            0x00B0    /* SIP SR Configuration Register */
#define N32_AFIO_SIP_DS_OFFSET            0x00B4    /* SIP DS Configuration Register */
#define N32_AFIO_ADCSW_CFG_OFFSET         0x00D0    /* ADC Switch Configuration Register */
#define N32_AFIO_SDRAM_VREF_EN0_OFFSET    0x00D8    /* SDRAM VREF Enable 0 Config Register */
#define N32_AFIO_SDRAM_VREF_EN1_OFFSET    0x00DC    /* SDRAM VREF Enable 1 Config Register */
#define N32_AFIO_SDRAM_VREF_EN2_OFFSET    0x00E0    /* SDRAM VREF Enable 2 Config Register */
#define N32_AFIO_SDRAMDSN_CFG0_OFFSET     0x00E4    /* SDRAM DSN 0 Config Register */
#define N32_AFIO_SDRAMDSN_CFG1_OFFSET     0x00E8    /* SDRAM DSN 1 Config Register */
#define N32_AFIO_SDRAMDSN_CFG2_OFFSET     0x00EC    /* SDRAM DSN 2 Config Register */
#define N32_AFIO_SDRAMDSP_CFG0_OFFSET     0x00F0    /* SDRAM DSP 0 Config Register */
#define N32_AFIO_SDRAMDSP_CFG1_OFFSET     0x00F4    /* SDRAM DSP 1 Config Register */
#define N32_AFIO_SDRAMDSP_CFG2_OFFSET     0x00F8    /* SDRAM DSP 2 Config Register */

/* Register Addresses *******************************************************/

#define N32_AFIO_XSPI1_NON(n)             (N32_AFIO_BASE + N32_AFIO_XSPI1_NON_OFFSET(n))
#define N32_AFIO_EXTI_CFG(n)              (N32_AFIO_BASE + N32_AFIO_EXTI_CFG_OFFSET(n))
#define N32_AFIO_TOL5V_CFG0_2(n)          (N32_AFIO_BASE + N32_AFIO_TOL5V_CFG0_2_OFFSET(n))
#define N32_AFIO_TOL5V_CFG3_6(n)          (N32_AFIO_BASE + N32_AFIO_TOL5V_CFG3_6_OFFSET(n))
#define N32_AFIO_EFT_CFG(n)               (N32_AFIO_BASE + N32_AFIO_EFT_CFG_OFFSET(n))
#define N32_AFIO_DIGEFT_CFG(n)            (N32_AFIO_BASE + N32_AFIO_DIGEFT_CFG_OFFSET(n))
#define N32_AFIO_SHRT1_EXEV_CFG(n)        (N32_AFIO_BASE + N32_AFIO_SHRT1_EXEV_CFG_OFFSET(n))
#define N32_AFIO_SHRT2_EXEV_CFG(n)        (N32_AFIO_BASE + N32_AFIO_SHRT2_EXEV_CFG_OFFSET(n))
#define N32_AFIO_SDRAM_HSMOD_CFG(n)       (N32_AFIO_BASE + N32_AFIO_SDRAM_HSMOD_CFG_OFFSET(n))
#define N32_AFIO_SDRAM_VREF_EN(n)         (N32_AFIO_BASE + N32_AFIO_SDRAM_VREF_EN_OFFSET(n))
#define N32_AFIO_SDRAMDSN_CFG(n)          (N32_AFIO_BASE + N32_AFIO_SDRAMDSN_CFG_OFFSET(n))
#define N32_AFIO_SDRAMDSP_CFG(n)          (N32_AFIO_BASE + N32_AFIO_SDRAMDSP_CFG_OFFSET(n))

#define N32_AFIO_RMP_CFG                  (N32_AFIO_BASE + N32_AFIO_RMP_CFG_OFFSET)
#define N32_AFIO_FILTER_CFG               (N32_AFIO_BASE + N32_AFIO_FILTER_CFG_OFFSET)
#define N32_AFIO_XSPI1_NON0               (N32_AFIO_BASE + N32_AFIO_XSPI1_NON0_OFFSET)
#define N32_AFIO_XSPI1_NON1               (N32_AFIO_BASE + N32_AFIO_XSPI1_NON1_OFFSET)
#define N32_AFIO_XSPI1_NON2               (N32_AFIO_BASE + N32_AFIO_XSPI1_NON2_OFFSET)
#define N32_AFIO_ADCRMP_CFG               (N32_AFIO_BASE + N32_AFIO_ADCRMP_CFG_OFFSET)
#define N32_AFIO_EXTI_CFG0                (N32_AFIO_BASE + N32_AFIO_EXTI_CFG0_OFFSET)
#define N32_AFIO_EXTI_CFG1                (N32_AFIO_BASE + N32_AFIO_EXTI_CFG1_OFFSET)
#define N32_AFIO_EXTI_CFG2                (N32_AFIO_BASE + N32_AFIO_EXTI_CFG2_OFFSET)
#define N32_AFIO_EXTI_CFG3                (N32_AFIO_BASE + N32_AFIO_EXTI_CFG3_OFFSET)
#define N32_AFIO_TOL5V_CFG0               (N32_AFIO_BASE + N32_AFIO_TOL5V_CFG0_OFFSET)
#define N32_AFIO_TOL5V_CFG1               (N32_AFIO_BASE + N32_AFIO_TOL5V_CFG1_OFFSET)
#define N32_AFIO_TOL5V_CFG2               (N32_AFIO_BASE + N32_AFIO_TOL5V_CFG2_OFFSET)
#define N32_AFIO_SHRT1_FALT_CFG           (N32_AFIO_BASE + N32_AFIO_SHRT1_FALT_CFG_OFFSET)
#define N32_AFIO_SHRT2_FALT_CFG           (N32_AFIO_BASE + N32_AFIO_SHRT2_FALT_CFG_OFFSET)
#define N32_AFIO_TOL5V_CFG3               (N32_AFIO_BASE + N32_AFIO_TOL5V_CFG3_OFFSET)
#define N32_AFIO_TOL5V_CFG4               (N32_AFIO_BASE + N32_AFIO_TOL5V_CFG4_OFFSET)
#define N32_AFIO_TOL5V_CFG5               (N32_AFIO_BASE + N32_AFIO_TOL5V_CFG5_OFFSET)
#define N32_AFIO_TOL5V_CFG6               (N32_AFIO_BASE + N32_AFIO_TOL5V_CFG6_OFFSET)
#define N32_AFIO_EFT_CFG0                 (N32_AFIO_BASE + N32_AFIO_EFT_CFG0_OFFSET)
#define N32_AFIO_EFT_CFG1                 (N32_AFIO_BASE + N32_AFIO_EFT_CFG1_OFFSET)
#define N32_AFIO_EFT_CFG2                 (N32_AFIO_BASE + N32_AFIO_EFT_CFG2_OFFSET)
#define N32_AFIO_EFT_CFG3                 (N32_AFIO_BASE + N32_AFIO_EFT_CFG3_OFFSET)
#define N32_AFIO_EFT_CFG4                 (N32_AFIO_BASE + N32_AFIO_EFT_CFG4_OFFSET)
#define N32_AFIO_EFT_CFG5                 (N32_AFIO_BASE + N32_AFIO_EFT_CFG5_OFFSET)
#define N32_AFIO_DIGEFT_CFG0              (N32_AFIO_BASE + N32_AFIO_DIGEFT_CFG0_OFFSET)
#define N32_AFIO_DIGEFT_CFG1              (N32_AFIO_BASE + N32_AFIO_DIGEFT_CFG1_OFFSET)
#define N32_AFIO_DIGEFT_CFG2              (N32_AFIO_BASE + N32_AFIO_DIGEFT_CFG2_OFFSET)
#define N32_AFIO_DIGEFT_CFG3              (N32_AFIO_BASE + N32_AFIO_DIGEFT_CFG3_OFFSET)
#define N32_AFIO_DIGEFT_CFG4              (N32_AFIO_BASE + N32_AFIO_DIGEFT_CFG4_OFFSET)
#define N32_AFIO_DIGEFT_CFG5              (N32_AFIO_BASE + N32_AFIO_DIGEFT_CFG5_OFFSET)
#define N32_AFIO_SHRT1_EXEV_CFG0          (N32_AFIO_BASE + N32_AFIO_SHRT1_EXEV_CFG0_OFFSET)
#define N32_AFIO_SHRT1_EXEV_CFG1          (N32_AFIO_BASE + N32_AFIO_SHRT1_EXEV_CFG1_OFFSET)
#define N32_AFIO_SHRT2_EXEV_CFG0          (N32_AFIO_BASE + N32_AFIO_SHRT2_EXEV_CFG0_OFFSET)
#define N32_AFIO_SHRT2_EXEV_CFG1          (N32_AFIO_BASE + N32_AFIO_SHRT2_EXEV_CFG1_OFFSET)
#define N32_AFIO_SIP_PUPD                 (N32_AFIO_BASE + N32_AFIO_SIP_PUPD_OFFSET)
#define N32_AFIO_SDRAM_HSMOD_CFG0         (N32_AFIO_BASE + N32_AFIO_SDRAM_HSMOD_CFG0_OFFSET)
#define N32_AFIO_SDRAM_HSMOD_CFG1         (N32_AFIO_BASE + N32_AFIO_SDRAM_HSMOD_CFG1_OFFSET)
#define N32_AFIO_SDRAM_HSMOD_CFG2         (N32_AFIO_BASE + N32_AFIO_SDRAM_HSMOD_CFG2_OFFSET)
#define N32_AFIO_SDRAM_HSMOD_CFG3         (N32_AFIO_BASE + N32_AFIO_SDRAM_HSMOD_CFG3_OFFSET)
#define N32_AFIO_SDRAM_HSMOD_CFG4         (N32_AFIO_BASE + N32_AFIO_SDRAM_HSMOD_CFG4_OFFSET)
#define N32_AFIO_SIP_SR                   (N32_AFIO_BASE + N32_AFIO_SIP_SR_OFFSET)
#define N32_AFIO_SIP_DS                   (N32_AFIO_BASE + N32_AFIO_SIP_DS_OFFSET)
#define N32_AFIO_ADCSW_CFG                (N32_AFIO_BASE + N32_AFIO_ADCSW_CFG_OFFSET)
#define N32_AFIO_SDRAM_VREF_EN0           (N32_AFIO_BASE + N32_AFIO_SDRAM_VREF_EN0_OFFSET)
#define N32_AFIO_SDRAM_VREF_EN1           (N32_AFIO_BASE + N32_AFIO_SDRAM_VREF_EN1_OFFSET)
#define N32_AFIO_SDRAM_VREF_EN2           (N32_AFIO_BASE + N32_AFIO_SDRAM_VREF_EN2_OFFSET)
#define N32_AFIO_SDRAMDSN_CFG0            (N32_AFIO_BASE + N32_AFIO_SDRAMDSN_CFG0_OFFSET)
#define N32_AFIO_SDRAMDSN_CFG1            (N32_AFIO_BASE + N32_AFIO_SDRAMDSN_CFG1_OFFSET)
#define N32_AFIO_SDRAMDSN_CFG2            (N32_AFIO_BASE + N32_AFIO_SDRAMDSN_CFG2_OFFSET)
#define N32_AFIO_SDRAMDSP_CFG0            (N32_AFIO_BASE + N32_AFIO_SDRAMDSP_CFG0_OFFSET)
#define N32_AFIO_SDRAMDSP_CFG1            (N32_AFIO_BASE + N32_AFIO_SDRAMDSP_CFG1_OFFSET)
#define N32_AFIO_SDRAMDSP_CFG2            (N32_AFIO_BASE + N32_AFIO_SDRAMDSP_CFG2_OFFSET)

/* Register Bit Definitions *************************************************/

/* AFIO_RMP_CFG - AFIO Mapping Configuration Register (Offset: 0x00) */

/* Single-bit fields */
#define N32_AFIO_RMP_CFG_EXTL_AFLTBYPS   (1 << 30)  /* Bit 30: Bypass analog filter for EXTI */
#define N32_AFIO_RMP_CFG_SIP_SDRAM_SEL   (1 << 29)  /* Bit 29: Set SIP SDRAM PAD priority */
#define N32_AFIO_RMP_CFG_SPI4SEL         (1 << 28)  /* Bit 28: Select SPI4/I2S4 function */
#define N32_AFIO_RMP_CFG_SPI3SEL         (1 << 27)  /* Bit 27: Select SPI3/I2S3 function */
#define N32_AFIO_RMP_CFG_SPI2SEL         (1 << 26)  /* Bit 26: Select SPI2/I2S2 function */
#define N32_AFIO_RMP_CFG_SPI1SEL         (1 << 25)  /* Bit 25: Select SPI1/I2S1 function */
#define N32_AFIO_RMP_CFG_SDMMC1_CLKFB    (1 << 24)  /* Bit 24: SDMMC1 clock input selection */
#define N32_AFIO_RMP_CFG_SDMMC2_CLKFB    (1 << 23)  /* Bit 23: SDMMC2 clock input selection */
#define N32_AFIO_RMP_CFG_XSPI2_EDN       (1 << 19)  /* Bit 19: XSPI2 endian selection */
#define N32_AFIO_RMP_CFG_XSPI1_EDN       (1 << 18)  /* Bit 18: XSPI1 endian selection */
#define N32_AFIO_RMP_CFG_FEMCSEL         (1 << 17)  /* Bit 17: FEMC work mode selection */
#define N32_AFIO_RMP_CFG_FEMC_NOBYTE     (1 << 16)  /* Bit 16: FEMC byte strokes support */
#define N32_AFIO_RMP_CFG_ETH2_PHY        (1 << 15)  /* Bit 15: ETH2 PHY selection */
#define N32_AFIO_RMP_CFG_SPI7_NSS        (1 << 12)  /* Bit 12: SPI7 NSS idle mode */
#define N32_AFIO_RMP_CFG_SPI6_NSS        (1 << 11)  /* Bit 11: SPI6 NSS idle mode */
#define N32_AFIO_RMP_CFG_SPI5_NSS        (1 << 10)  /* Bit 10: SPI5 NSS idle mode */
#define N32_AFIO_RMP_CFG_SPI4_NSS        (1 << 9)   /* Bit 9:  SPI4 NSS idle mode */
#define N32_AFIO_RMP_CFG_SPI3_NSS        (1 << 8)   /* Bit 8:  SPI3 NSS idle mode */
#define N32_AFIO_RMP_CFG_SPI2_NSS        (1 << 7)   /* Bit 7:  SPI2 NSS idle mode */
#define N32_AFIO_RMP_CFG_SPI1_NSS        (1 << 6)   /* Bit 6:  SPI1 NSS idle mode */

/* Multi-bit fields */

/* I2S_FDUP[1:0] (Bits 22-21) */
#define N32_AFIO_RMP_CFG_I2S_FDUP_SHIFT  (21)       /* I2S full duplex mode selection shift */
#define N32_AFIO_RMP_CFG_I2S_FDUP_MASK   (0x3 << N32_AFIO_RMP_CFG_I2S_FDUP_SHIFT)
#  define N32_AFIO_RMP_CFG_I2S1_FDUP     (0x0 << N32_AFIO_RMP_CFG_I2S_FDUP_SHIFT) /* I2S1 full duplex */
#  define N32_AFIO_RMP_CFG_I2S2_FDUP     (0x1 << N32_AFIO_RMP_CFG_I2S_FDUP_SHIFT) /* I2S2 full duplex */
#  define N32_AFIO_RMP_CFG_I2S3_FDUP     (0x2 << N32_AFIO_RMP_CFG_I2S_FDUP_SHIFT) /* I2S3 full duplex */
#  define N32_AFIO_RMP_CFG_I2S4_FDUP     (0x3 << N32_AFIO_RMP_CFG_I2S_FDUP_SHIFT) /* I2S4 full duplex */

/* ETH1_PHY[1:0] (Bits 14-13) */
#define N32_AFIO_RMP_CFG_ETH1_PHY_SHIFT  (13)       /* ETH1 PHY selection shift */
#define N32_AFIO_RMP_CFG_ETH1_PHY_MASK   (0x3 << N32_AFIO_RMP_CFG_ETH1_PHY_SHIFT)
#  define N32_AFIO_RMP_CFG_ETH1_GMII     (0x0 << N32_AFIO_RMP_CFG_ETH1_PHY_SHIFT) /* GMII mode */
#  define N32_AFIO_RMP_CFG_ETH1_RGMII    (0x1 << N32_AFIO_RMP_CFG_ETH1_PHY_SHIFT) /* RGMII mode */
#  define N32_AFIO_RMP_CFG_ETH1_RMII     (0x2 << N32_AFIO_RMP_CFG_ETH1_PHY_SHIFT) /* RMII mode */
#  define N32_AFIO_RMP_CFG_ETH1_MII      (0x3 << N32_AFIO_RMP_CFG_ETH1_PHY_SHIFT) /* MII mode */

/* SIP_FLASHSEL[2:0] (Bits 2-0) */
#define N32_AFIO_RMP_CFG_SIP_FLASHSEL_SHIFT (0)     /* SIP flash model selection shift */
#define N32_AFIO_RMP_CFG_SIP_FLASHSEL_MASK  (0x7 << N32_AFIO_RMP_CFG_SIP_FLASHSEL_SHIFT)
#  define N32_AFIO_RMP_CFG_SIP_IS25WJ032F   (0x0 << N32_AFIO_RMP_CFG_SIP_FLASHSEL_SHIFT) /* IS25WJ032F */
#  define N32_AFIO_RMP_CFG_SIP_IS25LP016D   (0x1 << N32_AFIO_RMP_CFG_SIP_FLASHSEL_SHIFT) /* IS25LP016D/WP016D */
#  define N32_AFIO_RMP_CFG_SIP_XM25LU32CK   (0x2 << N32_AFIO_RMP_CFG_SIP_FLASHSEL_SHIFT) /* XM25LU32CK */
#  define N32_AFIO_RMP_CFG_SIP_GT25Q16A     (0x3 << N32_AFIO_RMP_CFG_SIP_FLASHSEL_SHIFT) /* GT25Q16A */
#  define N32_AFIO_RMP_CFG_SIP_GT25Q32A     (0x4 << N32_AFIO_RMP_CFG_SIP_FLASHSEL_SHIFT) /* GT25Q32A */
#  define N32_AFIO_RMP_CFG_SIP_DISABLE      (0x7 << N32_AFIO_RMP_CFG_SIP_FLASHSEL_SHIFT) /* Disable SIP */

/* AFIO_FILTER_CFG - AFIO IOM Filter Control Register (Offset: 0x04) */

/* Filter configuration field */
#define N32_AFIO_FILTER_CFG_IDFLITCFG_SHIFT (0)      /* Bits 6-0: Glitch Filter stage control shift */
#define N32_AFIO_FILTER_CFG_IDFLITCFG_MASK  (0x7F << N32_AFIO_FILTER_CFG_IDFLITCFG_SHIFT)
#  define N32_AFIO_FILTER_CFG_BYPASS        (0x00 << N32_AFIO_FILTER_CFG_IDFLITCFG_SHIFT) /* Filter bypass */

/* AFIO_XSPI1_NON0 - AFIO XSPI1 Nonce Register 0 (Offset: 0x08) */

/* 32-bit nonce value */
#define N32_AFIO_XSPI1_NON0_NONCE_SHIFT    (0)       /* Nonce [31:0] shift */
#define N32_AFIO_XSPI1_NON0_NONCE_MASK     (0xFFFFFFFF << N32_AFIO_XSPI1_NON0_NONCE_SHIFT)

/* AFIO_XSPI1_NON1 - AFIO XSPI1 Nonce Register 1 (Offset: 0x0C) */

/* 32-bit nonce value */
#define N32_AFIO_XSPI1_NON1_NONCE_SHIFT    (0)       /* Nonce [63:32] shift */
#define N32_AFIO_XSPI1_NON1_NONCE_MASK     (0xFFFFFFFF << N32_AFIO_XSPI1_NON1_NONCE_SHIFT)

/* AFIO_XSPI1_NON2 - AFIO XSPI1 Nonce Register 2 (Offset: 0x10) */

/* 32-bit nonce value */
#define N32_AFIO_XSPI1_NON2_NONCE_SHIFT    (0)       /* Nonce [95:64] shift */
#define N32_AFIO_XSPI1_NON2_NONCE_MASK     (0xFFFFFFFF << N32_AFIO_XSPI1_NON2_NONCE_SHIFT)

/* AFIO_ADCRMP_CFG - AFIO ADC Remap Configure Register (Offset: 0x14) */

/* ADC1 External Trigger Selections */
#define N32_AFIO_ADCRMP_CFG_ADC1_INJ_EXTI_SHIFT   (20)  /* Bits 23-20: ADC1 injection channel external trigger */
#define N32_AFIO_ADCRMP_CFG_ADC1_INJ_EXTI_MASK    (0xF << N32_AFIO_ADCRMP_CFG_ADC1_INJ_EXTI_SHIFT)
#define N32_AFIO_ADCRMP_CFG_ADC1_REG_EXTI_SHIFT   (16)  /* Bits 19-16: ADC1 regular channel external trigger */
#define N32_AFIO_ADCRMP_CFG_ADC1_REG_EXTI_MASK    (0xF << N32_AFIO_ADCRMP_CFG_ADC1_REG_EXTI_SHIFT)

/* ADC2 External Trigger Selections */
#define N32_AFIO_ADCRMP_CFG_ADC2_INJ_EXTI_SHIFT   (12)  /* Bits 15-12: ADC2 injection channel external trigger */
#define N32_AFIO_ADCRMP_CFG_ADC2_INJ_EXTI_MASK    (0xF << N32_AFIO_ADCRMP_CFG_ADC2_INJ_EXTI_SHIFT)
#define N32_AFIO_ADCRMP_CFG_ADC2_REG_EXTI_SHIFT   (8)   /* Bits 11-8: ADC2 regular channel external trigger */
#define N32_AFIO_ADCRMP_CFG_ADC2_REG_EXTI_MASK    (0xF << N32_AFIO_ADCRMP_CFG_ADC2_REG_EXTI_SHIFT)

/* ADC3 External Trigger Selections */
#define N32_AFIO_ADCRMP_CFG_ADC3_INJ_EXTI_SHIFT   (4)   /* Bits 7-4: ADC3 injection channel external trigger */
#define N32_AFIO_ADCRMP_CFG_ADC3_INJ_EXTI_MASK    (0xF << N32_AFIO_ADCRMP_CFG_ADC3_INJ_EXTI_SHIFT)
#define N32_AFIO_ADCRMP_CFG_ADC3_REG_EXTI_SHIFT   (0)   /* Bits 3-0: ADC3 regular channel external trigger */
#define N32_AFIO_ADCRMP_CFG_ADC3_REG_EXTI_MASK    (0xF << N32_AFIO_ADCRMP_CFG_ADC3_REG_EXTI_SHIFT)

/* EXTI Line Selection Values (applies to all ADC trigger fields) */
#define N32_AFIO_ADC_EXTI_LINE0     0x0
#define N32_AFIO_ADC_EXTI_LINE1     0x1
#define N32_AFIO_ADC_EXTI_LINE2     0x2
#define N32_AFIO_ADC_EXTI_LINE3     0x3
#define N32_AFIO_ADC_EXTI_LINE4     0x4
#define N32_AFIO_ADC_EXTI_LINE5     0x5
#define N32_AFIO_ADC_EXTI_LINE6     0x6
#define N32_AFIO_ADC_EXTI_LINE7     0x7
#define N32_AFIO_ADC_EXTI_LINE8     0x8
#define N32_AFIO_ADC_EXTI_LINE9     0x9
#define N32_AFIO_ADC_EXTI_LINE10    0xA
#define N32_AFIO_ADC_EXTI_LINE11    0xB
#define N32_AFIO_ADC_EXTI_LINE12    0xC
#define N32_AFIO_ADC_EXTI_LINE13    0xD
#define N32_AFIO_ADC_EXTI_LINE14    0xE
#define N32_AFIO_ADC_EXTI_LINE15    0xF

/* AFIO External Interrupt Configuration Registers */

/* EXTI port selection values */
#define N32_AFIO_EXTICR_PORTA           0x00  /* 0000: PA[x] pin */
#define N32_AFIO_EXTICR_PORTB           0x01  /* 0001: PB[x] pin */
#define N32_AFIO_EXTICR_PORTC           0x02  /* 0010: PC[x] pin */
#define N32_AFIO_EXTICR_PORTD           0x03  /* 0011: PD[x] pin */
#define N32_AFIO_EXTICR_PORTE           0x04  /* 0100: PE[x] pin */
#define N32_AFIO_EXTICR_PORTF           0x05  /* 0101: PF[x] pin */
#define N32_AFIO_EXTICR_PORTG           0x06  /* 0110: PG[x] pin */
#define N32_AFIO_EXTICR_PORTH           0x07  /* 0111: PH[x] pin */
#define N32_AFIO_EXTICR_PORTI           0x08  /* 1000: PI[x] pin */
#define N32_AFIO_EXTICR_PORTJ           0x09  /* 1001: PJ[x] pin */
#define N32_AFIO_EXTICR_PORTK           0x0A  /* 1010: PK[x] pin */

/* Common field definitions */
#define N32_AFIO_EXTICR_PORT_MASK       0xFF  /* 8-bit port selection mask */
#define N32_AFIO_EXTICR_EXTI_WIDTH      8     /* Bits per EXTI line config */
#define N32_AFIO_EXTICR_EXTI_SHIFT(g)   (((g) & 3) << 3)
#define N32_AFIO_EXTICR_EXTI_MASK(g)    (N32_AFIO_EXTICR_PORT_MASK << (N32_AFIO_EXTICR_EXTI_SHIFT(g)))

/* AFIO_EXTI_CFG0 (Offset: 0x18) */
#define N32_AFIO_EXTI_CFG0_EXTI0_SHIFT  0     /* Bits 7-0:   EXTI0 configuration */
#define N32_AFIO_EXTI_CFG0_EXTI0_MASK   (N32_AFIO_EXTICR_PORT_MASK << N32_AFIO_EXTI_CFG0_EXTI0_SHIFT)
#define N32_AFIO_EXTI_CFG0_EXTI1_SHIFT  8     /* Bits 15-8:  EXTI1 configuration */
#define N32_AFIO_EXTI_CFG0_EXTI1_MASK   (N32_AFIO_EXTICR_PORT_MASK << N32_AFIO_EXTI_CFG0_EXTI1_SHIFT)
#define N32_AFIO_EXTI_CFG0_EXTI2_SHIFT  16    /* Bits 23-16: EXTI2 configuration */
#define N32_AFIO_EXTI_CFG0_EXTI2_MASK   (N32_AFIO_EXTICR_PORT_MASK << N32_AFIO_EXTI_CFG0_EXTI2_SHIFT)
#define N32_AFIO_EXTI_CFG0_EXTI3_SHIFT  24    /* Bits 31-24: EXTI3 configuration */
#define N32_AFIO_EXTI_CFG0_EXTI3_MASK   (N32_AFIO_EXTICR_PORT_MASK << N32_AFIO_EXTI_CFG0_EXTI3_SHIFT)

/* AFIO_EXTI_CFG1 (Offset: 0x1C) */
#define N32_AFIO_EXTI_CFG1_EXTI4_SHIFT  0     /* Bits 7-0:   EXTI4 configuration */
#define N32_AFIO_EXTI_CFG1_EXTI4_MASK   (N32_AFIO_EXTICR_PORT_MASK << N32_AFIO_EXTI_CFG1_EXTI4_SHIFT)
#define N32_AFIO_EXTI_CFG1_EXTI5_SHIFT  8     /* Bits 15-8:  EXTI5 configuration */
#define N32_AFIO_EXTI_CFG1_EXTI5_MASK   (N32_AFIO_EXTICR_PORT_MASK << N32_AFIO_EXTI_CFG1_EXTI5_SHIFT)
#define N32_AFIO_EXTI_CFG1_EXTI6_SHIFT  16    /* Bits 23-16: EXTI6 configuration */
#define N32_AFIO_EXTI_CFG1_EXTI6_MASK   (N32_AFIO_EXTICR_PORT_MASK << N32_AFIO_EXTI_CFG1_EXTI6_SHIFT)
#define N32_AFIO_EXTI_CFG1_EXTI7_SHIFT  24    /* Bits 31-24: EXTI7 configuration */
#define N32_AFIO_EXTI_CFG1_EXTI7_MASK   (N32_AFIO_EXTICR_PORT_MASK << N32_AFIO_EXTI_CFG1_EXTI7_SHIFT)

/* AFIO_EXTI_CFG2 (Offset: 0x20) */
#define N32_AFIO_EXTI_CFG2_EXTI8_SHIFT  0     /* Bits 7-0:   EXTI8 configuration */
#define N32_AFIO_EXTI_CFG2_EXTI8_MASK   (N32_AFIO_EXTICR_PORT_MASK << N32_AFIO_EXTI_CFG2_EXTI8_SHIFT)
#define N32_AFIO_EXTI_CFG2_EXTI9_SHIFT  8     /* Bits 15-8:  EXTI9 configuration */
#define N32_AFIO_EXTI_CFG2_EXTI9_MASK   (N32_AFIO_EXTICR_PORT_MASK << N32_AFIO_EXTI_CFG2_EXTI9_SHIFT)
#define N32_AFIO_EXTI_CFG2_EXTI10_SHIFT 16    /* Bits 23-16: EXTI10 configuration */
#define N32_AFIO_EXTI_CFG2_EXTI10_MASK  (N32_AFIO_EXTICR_PORT_MASK << N32_AFIO_EXTI_CFG2_EXTI10_SHIFT)
#define N32_AFIO_EXTI_CFG2_EXTI11_SHIFT 24    /* Bits 31-24: EXTI11 configuration */
#define N32_AFIO_EXTI_CFG2_EXTI11_MASK  (N32_AFIO_EXTICR_PORT_MASK << N32_AFIO_EXTI_CFG2_EXTI11_SHIFT)

/* AFIO_EXTI_CFG3 (Offset: 0x24) */
#define N32_AFIO_EXTI_CFG3_EXTI12_SHIFT 0     /* Bits 7-0:   EXTI12 configuration */
#define N32_AFIO_EXTI_CFG3_EXTI12_MASK  (N32_AFIO_EXTICR_PORT_MASK << N32_AFIO_EXTI_CFG3_EXTI12_SHIFT)
#define N32_AFIO_EXTI_CFG3_EXTI13_SHIFT 8     /* Bits 15-8:  EXTI13 configuration */
#define N32_AFIO_EXTI_CFG3_EXTI13_MASK  (N32_AFIO_EXTICR_PORT_MASK << N32_AFIO_EXTI_CFG3_EXTI13_SHIFT)
#define N32_AFIO_EXTI_CFG3_EXTI14_SHIFT 16    /* Bits 23-16: EXTI14 configuration */
#define N32_AFIO_EXTI_CFG3_EXTI14_MASK  (N32_AFIO_EXTICR_PORT_MASK << N32_AFIO_EXTI_CFG3_EXTI14_SHIFT)
#define N32_AFIO_EXTI_CFG3_EXTI15_SHIFT 24    /* Bits 31-24: EXTI15 configuration */
#define N32_AFIO_EXTI_CFG3_EXTI15_MASK  (N32_AFIO_EXTICR_PORT_MASK << N32_AFIO_EXTI_CFG3_EXTI15_SHIFT)

/* AFIO_TOL5V_CFG0 (Offset: 0x28) */
#define N32_AFIO_TOL5V_CFG0_PA7TOLENN    (1 << 9)   /* Bit 9: PA7 5V tolerance */
#define N32_AFIO_TOL5V_CFG0_PA6TOLENN    (1 << 8)   /* Bit 8: PA6 5V tolerance */
#define N32_AFIO_TOL5V_CFG0_PA5TOLENN    (1 << 7)   /* Bit 7: PA5 5V tolerance */
#define N32_AFIO_TOL5V_CFG0_PA4TOLENN    (1 << 6)   /* Bit 6: PA4 5V tolerance */
#define N32_AFIO_TOL5V_CFG0_PA3TOLENN    (1 << 5)   /* Bit 5: PA3 5V tolerance */
#define N32_AFIO_TOL5V_CFG0_PA2TOLENN    (1 << 4)   /* Bit 4: PA2 5V tolerance */
#define N32_AFIO_TOL5V_CFG0_PA1_CTOLENN  (1 << 3)   /* Bit 3: PA1_C 5V tolerance */
#define N32_AFIO_TOL5V_CFG0_PA1TOLENN    (1 << 2)   /* Bit 2: PA1 5V tolerance */
#define N32_AFIO_TOL5V_CFG0_PA0_CTOLENN  (1 << 1)   /* Bit 1: PA0_C 5V tolerance */
#define N32_AFIO_TOL5V_CFG0_PA0TOLENN    (1 << 0)   /* Bit 0: PA0 5V tolerance */

/* AFIO_TOL5V_CFG1 (Offset: 0x2C) */
#define N32_AFIO_TOL5V_CFG1_PB1TOLENN    (1 << 1)   /* Bit 1: PB1 5V tolerance */
#define N32_AFIO_TOL5V_CFG1_PB0TOLENN    (1 << 0)   /* Bit 0: PB0 5V tolerance */

/* AFIO_TOL5V_CFG2 (Offset: 0x30) */
#define N32_AFIO_TOL5V_CFG2_PC13TOLENN   (1 << 10)  /* Bit 10: PC13 5V tolerance */
#define N32_AFIO_TOL5V_CFG2_PC8TOLENN    (1 << 9)   /* Bit 9: PC8 5V tolerance */
#define N32_AFIO_TOL5V_CFG2_PC6TOLENN    (1 << 8)   /* Bit 8: PC6 5V tolerance */
#define N32_AFIO_TOL5V_CFG2_PC5TOLENN    (1 << 7)   /* Bit 7: PC5 5V tolerance */
#define N32_AFIO_TOL5V_CFG2_PC4TOLENN    (1 << 6)   /* Bit 6: PC4 5V tolerance */
#define N32_AFIO_TOL5V_CFG2_PC3_CTOLENN  (1 << 5)   /* Bit 5: PC3_C 5V tolerance */
#define N32_AFIO_TOL5V_CFG2_PC3TOLENN    (1 << 4)   /* Bit 4: PC3 5V tolerance */
#define N32_AFIO_TOL5V_CFG2_PC2_CTOLENN  (1 << 3)   /* Bit 3: PC2_C 5V tolerance */
#define N32_AFIO_TOL5V_CFG2_PC2TOLENN    (1 << 2)   /* Bit 2: PC2 5V tolerance */
#define N32_AFIO_TOL5V_CFG2_PC1TOLENN    (1 << 1)   /* Bit 1: PC1 5V tolerance */
#define N32_AFIO_TOL5V_CFG2_PC0TOLENN    (1 << 0)   /* Bit 0: PC0 5V tolerance */

/* AFIO_TOL5V_CFG3 (Offset: 0x3C) */
#define N32_AFIO_TOL5V_CFG3_PF14TOLENN   (1 << 11)  /* Bit 11: PF14 5V tolerance */
#define N32_AFIO_TOL5V_CFG3_PF13TOLENN   (1 << 10)  /* Bit 10: PF13 5V tolerance */
#define N32_AFIO_TOL5V_CFG3_PF12TOLENN   (1 << 9)   /* Bit 9: PF12 5V tolerance */
#define N32_AFIO_TOL5V_CFG3_PF11TOLENN   (1 << 8)   /* Bit 8: PF11 5V tolerance */
#define N32_AFIO_TOL5V_CFG3_PF10TOLENN   (1 << 7)   /* Bit 7: PF10 5V tolerance */
#define N32_AFIO_TOL5V_CFG3_PF9TOLENN    (1 << 6)   /* Bit 6: PF9 5V tolerance */
#define N32_AFIO_TOL5V_CFG3_PF8TOLENN    (1 << 5)   /* Bit 5: PF8 5V tolerance */
#define N32_AFIO_TOL5V_CFG3_PF7TOLENN    (1 << 4)   /* Bit 4: PF7 5V tolerance */
#define N32_AFIO_TOL5V_CFG3_PF6TOLENN    (1 << 3)   /* Bit 3: PF6 5V tolerance */
#define N32_AFIO_TOL5V_CFG3_PF5TOLENN    (1 << 2)   /* Bit 2: PF5 5V tolerance */
#define N32_AFIO_TOL5V_CFG3_PF4TOLENN    (1 << 1)   /* Bit 1: PF4 5V tolerance */
#define N32_AFIO_TOL5V_CFG3_PF3TOLENN    (1 << 0)   /* Bit 0: PF3 5V tolerance */

/* AFIO_TOL5V_CFG4 (Offset: 0x44) */
#define N32_AFIO_TOL5V_CFG4_PH5TOLENN    (1 << 3)   /* Bit 3: PH5 5V tolerance */
#define N32_AFIO_TOL5V_CFG4_PH4TOLENN    (1 << 2)   /* Bit 2: PH4 5V tolerance */
#define N32_AFIO_TOL5V_CFG4_PH3TOLENN    (1 << 1)   /* Bit 1: PH3 5V tolerance */
#define N32_AFIO_TOL5V_CFG4_PH2TOLENN    (1 << 0)   /* Bit 0: PH2 5V tolerance */

/* AFIO_TOL5V_CFG6 (Offset: 0x4C) */
#define N32_AFIO_TOL5V_CFG6_PI7TOLENN    (1 << 5)   /* Bit 5: PI7 5V tolerance */
#define N32_AFIO_TOL5V_CFG6_PI6TOLENN    (1 << 4)   /* Bit 4: PI6 5V tolerance */
#define N32_AFIO_TOL5V_CFG6_PI5TOLENN    (1 << 3)   /* Bit 3: PI5 5V tolerance */
#define N32_AFIO_TOL5V_CFG6_PI4TOLENN    (1 << 2)   /* Bit 2: PI4 5V tolerance */
#define N32_AFIO_TOL5V_CFG6_PI3TOLENN    (1 << 1)   /* Bit 1: PI3 5V tolerance */
#define N32_AFIO_TOL5V_CFG6_PI0TOLENN    (1 << 0)   /* Bit 0: PI0 5V tolerance */

/* AFIO_SHRT1_FALT_CFG (Offset: 0x34) */

/* Fault channel selection values */
#define N32_AFIO_SHRT1_FAULT_PA15      0x1
#define N32_AFIO_SHRT1_FAULT_PB3       0x2
#define N32_AFIO_SHRT1_FAULT_PC11      0x3
#define N32_AFIO_SHRT1_FAULT_PD4       0x4
#define N32_AFIO_SHRT1_FAULT_PE4       0x5
#define N32_AFIO_SHRT1_FAULT_PG9       0x6
#define N32_AFIO_SHRT1_FAULT_PG10      0x7
#define N32_AFIO_SHRT1_FAULT_PI6       0x8
#define N32_AFIO_SHRT1_FAULT_PI15      0x9
#define N32_AFIO_SHRT1_FAULT_PK2       0xA

/* Fault channel configuration fields */
#define N32_AFIO_SHRT1_FAULT1_SHIFT    (0)   /* Bits 3-0: Fault channel 1 */
#define N32_AFIO_SHRT1_FAULT1_MASK     (0xF << N32_AFIO_SHRT1_FAULT1_SHIFT)
#define N32_AFIO_SHRT1_FAULT2_SHIFT    (4)   /* Bits 7-4: Fault channel 2 */
#define N32_AFIO_SHRT1_FAULT2_MASK     (0xF << N32_AFIO_SHRT1_FAULT2_SHIFT)
#define N32_AFIO_SHRT1_FAULT3_SHIFT    (8)   /* Bits 11-8: Fault channel 3 */
#define N32_AFIO_SHRT1_FAULT3_MASK     (0xF << N32_AFIO_SHRT1_FAULT3_SHIFT)
#define N32_AFIO_SHRT1_FAULT4_SHIFT    (12)  /* Bits 15-12: Fault channel 4 */
#define N32_AFIO_SHRT1_FAULT4_MASK     (0xF << N32_AFIO_SHRT1_FAULT4_SHIFT)
#define N32_AFIO_SHRT1_FAULT5_SHIFT    (16)  /* Bits 19-16: Fault channel 5 */
#define N32_AFIO_SHRT1_FAULT5_MASK     (0xF << N32_AFIO_SHRT1_FAULT5_SHIFT)
#define N32_AFIO_SHRT1_FAULT6_SHIFT    (20)  /* Bits 23-20: Fault channel 6 */
#define N32_AFIO_SHRT1_FAULT6_MASK     (0xF << N32_AFIO_SHRT1_FAULT6_SHIFT)

/* AFIO_SHRT2_FALT_CFG (Offset: 0x38) */

/* Fault channel selection values */
#define N32_AFIO_SHRT2_FAULT_PC5      0x1
#define N32_AFIO_SHRT2_FAULT_PD1      0x2
#define N32_AFIO_SHRT2_FAULT_PD15     0x3
#define N32_AFIO_SHRT2_FAULT_PF9      0x4
#define N32_AFIO_SHRT2_FAULT_PF13     0x5
#define N32_AFIO_SHRT2_FAULT_PG1      0x6
#define N32_AFIO_SHRT2_FAULT_PI0      0x7
#define N32_AFIO_SHRT2_FAULT_PJ9      0x8
#define N32_AFIO_SHRT2_FAULT_PI13     0x9
#define N32_AFIO_SHRT2_FAULT_PJ0      0xA
#define N32_AFIO_SHRT2_FAULT_PK1      0xB
#define N32_AFIO_SHRT2_FAULT_PK6      0xC

/* Fault channel configuration fields */
#define N32_AFIO_SHRT2_FAULT1_SHIFT    (0)   /* Bits 3-0: Fault channel 1 */
#define N32_AFIO_SHRT2_FAULT1_MASK     (0xF << N32_AFIO_SHRT2_FAULT1_SHIFT)
#define N32_AFIO_SHRT2_FAULT2_SHIFT    (4)   /* Bits 7-4: Fault channel 2 */
#define N32_AFIO_SHRT2_FAULT2_MASK     (0xF << N32_AFIO_SHRT2_FAULT2_SHIFT)
#define N32_AFIO_SHRT2_FAULT3_SHIFT    (8)   /* Bits 11-8: Fault channel 3 */
#define N32_AFIO_SHRT2_FAULT3_MASK     (0xF << N32_AFIO_SHRT2_FAULT3_SHIFT)
#define N32_AFIO_SHRT2_FAULT4_SHIFT    (12)  /* Bits 15-12: Fault channel 4 */
#define N32_AFIO_SHRT2_FAULT4_MASK     (0xF << N32_AFIO_SHRT2_FAULT4_SHIFT)
#define N32_AFIO_SHRT2_FAULT5_SHIFT    (16)  /* Bits 19-16: Fault channel 5 */
#define N32_AFIO_SHRT2_FAULT5_MASK     (0xF << N32_AFIO_SHRT2_FAULT5_SHIFT)
#define N32_AFIO_SHRT2_FAULT6_SHIFT    (20)  /* Bits 23-20: Fault channel 6 */
#define N32_AFIO_SHRT2_FAULT6_MASK     (0xF << N32_AFIO_SHRT2_FAULT6_SHIFT)

/* Analog Filtering Configuration Registers */

/* AFIO_EFT_CFG0 - IO Port Analog Filtering Config 0 (Offset: 0x54) */
#define N32_AFIO_EFT_CFG0_PA_EFTEN_MASK  0xFFFF       /* PAx EFT enable (x=0-15) */
#define N32_AFIO_EFT_CFG0_PA_EFTEN_SHIFT 0
#define N32_AFIO_EFT_CFG0_PB_EFTEN_MASK  0xFFFF0000   /* PBx EFT enable (x=0-15) */
#define N32_AFIO_EFT_CFG0_PB_EFTEN_SHIFT 16

/* AFIO_EFT_CFG1 - IO Port Analog Filtering Config 1 (Offset: 0x58) */
#define N32_AFIO_EFT_CFG1_PC_EFTEN_MASK  0xFFFF       /* PCx EFT enable (x=0-15) */
#define N32_AFIO_EFT_CFG1_PC_EFTEN_SHIFT 0
#define N32_AFIO_EFT_CFG1_PD_EFTEN_MASK  0xFFFF0000   /* PDx EFT enable (x=0-15) */
#define N32_AFIO_EFT_CFG1_PD_EFTEN_SHIFT 16

/* AFIO_EFT_CFG2 - IO Port Analog Filtering Config 2 (Offset: 0x5C) */

/* Note: High word has no supported pads (reserved) */
#define N32_AFIO_EFT_CFG2_PE_EFTEN_MASK  0xFFFF       /* PEx EFT enable (x=0-15) */
#define N32_AFIO_EFT_CFG2_PE_EFTEN_SHIFT 0

/* AFIO_EFT_CFG3 - IO Port Analog Filtering Config 3 (Offset: 0x60) */
#define N32_AFIO_EFT_CFG3_PG_EFTEN_MASK  0xFFFF       /* PGx EFT enable (x=0-15) */
#define N32_AFIO_EFT_CFG3_PG_EFTEN_SHIFT 0
#define N32_AFIO_EFT_CFG3_PH_EFTEN_MASK  0xFFFF0000   /* PHx EFT enable (x=0-15) */
#define N32_AFIO_EFT_CFG3_PH_EFTEN_SHIFT 16

/* AFIO_EFT_CFG4 - IO Port Analog Filtering Config 4 (Offset: 0x64) */
#define N32_AFIO_EFT_CFG4_PJ_EFTEN_MASK  0xFFFF       /* PJx EFT enable (x=0-15) */
#define N32_AFIO_EFT_CFG4_PJ_EFTEN_SHIFT 0
#define N32_AFIO_EFT_CFG4_PI_EFTEN_MASK  0xFFFF0000   /* PIx EFT enable (x=0-15) */
#define N32_AFIO_EFT_CFG4_PI_EFTEN_SHIFT 16

/* AFIO_EFT_CFG5 - IO Port Analog Filtering Config 5 (Offset: 0x68) */
#define N32_AFIO_EFT_CFG5_PK_EFTEN_MASK  0xFF         /* PKx EFT enable (x=0-7) */
#define N32_AFIO_EFT_CFG5_PK_EFTEN_SHIFT 0
#define N32_AFIO_EFT_CFG5_BOOT_EFTEN     (1 << 8)     /* Bit 8: BOOT pin EFT enable */
#define N32_AFIO_EFT_CFG5_JRST_EFTBYPS   (1 << 9)     /* Bit 9: JTAG RST bypass */

/* Digital Filtering Configuration Registers */

/* AFIO_DIGEFT_CFG0 - IO Port Digital Filtering Config 0 (Offset: 0x6C) */
#define N32_AFIO_DIGEFT_CFG0_PA_DIGEFTEN_MASK  0xFFFF       /* PAx digital filter enable (x=0-15) */
#define N32_AFIO_DIGEFT_CFG0_PA_DIGEFTEN_SHIFT 0
#define N32_AFIO_DIGEFT_CFG0_PB_DIGEFTEN_MASK  0xFFFF0000   /* PBx digital filter enable (x=0-15) */
#define N32_AFIO_DIGEFT_CFG0_PB_DIGEFTEN_SHIFT 16

/* AFIO_DIGEFT_CFG1 - IO Port Digital Filtering Config 1 (Offset: 0x70) */
#define N32_AFIO_DIGEFT_CFG1_PC_DIGEFTEN_MASK  0xFFFF       /* PCx digital filter enable (x=0-15) */
#define N32_AFIO_DIGEFT_CFG1_PC_DIGEFTEN_SHIFT 0
#define N32_AFIO_DIGEFT_CFG1_PD_DIGEFTEN_MASK  0xFFFF0000   /* PDx digital filter enable (x=0-15) */
#define N32_AFIO_DIGEFT_CFG1_PD_DIGEFTEN_SHIFT 16

/* AFIO_DIGEFT_CFG2 - IO Port Digital Filtering Config 2 (Offset: 0x74) */
#define N32_AFIO_DIGEFT_CFG2_PE_DIGEFTEN_MASK  0xFFFF       /* PEx digital filter enable (x=0-15) */
#define N32_AFIO_DIGEFT_CFG2_PE_DIGEFTEN_SHIFT 0
#define N32_AFIO_DIGEFT_CFG2_PF_DIGEFTEN_MASK  0xFFFF0000   /* PFx digital filter enable (x=0-15) */
#define N32_AFIO_DIGEFT_CFG2_PF_DIGEFTEN_SHIFT 16

/* AFIO_DIGEFT_CFG3 - IO Port Digital Filtering Config 3 (Offset: 0x78) */
#define N32_AFIO_DIGEFT_CFG3_PG_DIGEFTEN_MASK  0xFFFF       /* PGx digital filter enable (x=0-15) */
#define N32_AFIO_DIGEFT_CFG3_PG_DIGEFTEN_SHIFT 0
#define N32_AFIO_DIGEFT_CFG3_PH_DIGEFTEN_MASK  0xFFFF0000   /* PHx digital filter enable (x=0-15) */
#define N32_AFIO_DIGEFT_CFG3_PH_DIGEFTEN_SHIFT 16

/* AFIO_DIGEFT_CFG4 - IO Port Digital Filtering Config 4 (Offset: 0x7C) */
#define N32_AFIO_DIGEFT_CFG4_PJ_DIGEFTEN_MASK  0xFFFF       /* PJx digital filter enable (x=0-15) */
#define N32_AFIO_DIGEFT_CFG4_PJ_DIGEFTEN_SHIFT 0
#define N32_AFIO_DIGEFT_CFG4_PI_DIGEFTEN_MASK  0x7FFF0000   /* PIx digital filter enable (x=0-6, bit16-22) */
#define N32_AFIO_DIGEFT_CFG4_PI_DIGEFTEN_SHIFT 16

/* AFIO_DIGEFT_CFG5 - IO Port Digital Filtering Config 5 (Offset: 0x80) */
#define N32_AFIO_DIGEFT_CFG5_PK_DIGEFTEN_MASK  0xFF         /* PKx digital filter enable (x=0-7) */
#define N32_AFIO_DIGEFT_CFG5_PK_DIGEFTEN_SHIFT 0
#define N32_AFIO_DIGEFT_CFG5_RESERVED_MASK     0xFFFFFF00   /* Reserved bits (8-31) */

/* SHRTIM1 External Event Configuration Registers */

/* AFIO_SHRT1_EXEV_CFG0 - SHRTIM1 External Event Config 0 (Offset: 0x84) */
#define N32_AFIO_SHRT1_EXEV_CFG0_EV1_MASK    0x1F        /* EXEV1 event source (5-bit) */
#define N32_AFIO_SHRT1_EXEV_CFG0_EV1_SHIFT   0
#define N32_AFIO_SHRT1_EXEV_CFG0_EV2_MASK    0x1F        /* EXEV2 event source (5-bit) */
#define N32_AFIO_SHRT1_EXEV_CFG0_EV2_SHIFT   5
#define N32_AFIO_SHRT1_EXEV_CFG0_EV3_MASK    0x1F        /* EXEV3 event source (5-bit) */
#define N32_AFIO_SHRT1_EXEV_CFG0_EV3_SHIFT   10
#define N32_AFIO_SHRT1_EXEV_CFG0_EV4_MASK    0x1F        /* EXEV4 event source (5-bit) */
#define N32_AFIO_SHRT1_EXEV_CFG0_EV4_SHIFT   15
#define N32_AFIO_SHRT1_EXEV_CFG0_EV5_MASK    0x1F        /* EXEV5 event source (5-bit) */
#define N32_AFIO_SHRT1_EXEV_CFG0_EV5_SHIFT   20
#define N32_AFIO_SHRT1_EXEV_CFG0_RESERVED_MASK 0xFE000000 /* Reserved bits (25-31) */

/* AFIO_SHRT1_EXEV_CFG1 - SHRTIM1 External Event Config 1 (Offset: 0x88) */
#define N32_AFIO_SHRT1_EXEV_CFG1_EV6_MASK    0x1F        /* EXEV6 event source (5-bit) */
#define N32_AFIO_SHRT1_EXEV_CFG1_EV6_SHIFT   0
#define N32_AFIO_SHRT1_EXEV_CFG1_EV7_MASK    0x1F        /* EXEV7 event source (5-bit) */
#define N32_AFIO_SHRT1_EXEV_CFG1_EV7_SHIFT   5
#define N32_AFIO_SHRT1_EXEV_CFG1_EV8_MASK    0x1F        /* EXEV8 event source (5-bit) */
#define N32_AFIO_SHRT1_EXEV_CFG1_EV8_SHIFT   10
#define N32_AFIO_SHRT1_EXEV_CFG1_EV9_MASK    0x1F        /* EXEV9 event source (5-bit) */
#define N32_AFIO_SHRT1_EXEV_CFG1_EV9_SHIFT   15
#define N32_AFIO_SHRT1_EXEV_CFG1_EV10_MASK   0x1F        /* EXEV10 event source (5-bit) */
#define N32_AFIO_SHRT1_EXEV_CFG1_EV10_SHIFT  20
#define N32_AFIO_SHRT1_EXEV_CFG1_RESERVED_MASK 0xFE000000 /* Reserved bits (25-31) */

/* SHRTIM2 External Event Configuration Registers */

/* AFIO_SHRT2_EXEV_CFG0 - SHRTIM2 External Event Config 0 (Offset: 0x8C) */
#define N32_AFIO_SHRT2_EXEV_CFG0_EV1_MASK    0x1F        /* EXEV1 event source (5-bit) */
#define N32_AFIO_SHRT2_EXEV_CFG0_EV1_SHIFT   0
#define N32_AFIO_SHRT2_EXEV_CFG0_EV2_MASK    0x1F        /* EXEV2 event source (5-bit) */
#define N32_AFIO_SHRT2_EXEV_CFG0_EV2_SHIFT   5
#define N32_AFIO_SHRT2_EXEV_CFG0_EV3_MASK    0x1F        /* EXEV3 event source (5-bit) */
#define N32_AFIO_SHRT2_EXEV_CFG0_EV3_SHIFT   10
#define N32_AFIO_SHRT2_EXEV_CFG0_EV4_MASK    0x1F        /* EXEV4 event source (5-bit) */
#define N32_AFIO_SHRT2_EXEV_CFG0_EV4_SHIFT   15
#define N32_AFIO_SHRT2_EXEV_CFG0_EV5_MASK    0x1F        /* EXEV5 event source (5-bit) */
#define N32_AFIO_SHRT2_EXEV_CFG0_EV5_SHIFT   20
#define N32_AFIO_SHRT2_EXEV_CFG0_RESERVED_MASK 0xFE000000 /* Reserved bits (25-31) */

/* AFIO_SHRT2_EXEV_CFG1 - SHRTIM2 External Event Config 1 (Offset: 0x90) */
#define N32_AFIO_SHRT2_EXEV_CFG1_EV6_MASK    0x1F        /* EXEV6 event source (5-bit) */
#define N32_AFIO_SHRT2_EXEV_CFG1_EV6_SHIFT   0
#define N32_AFIO_SHRT2_EXEV_CFG1_EV7_MASK    0x1F        /* EXEV7 event source (5-bit) */
#define N32_AFIO_SHRT2_EXEV_CFG1_EV7_SHIFT   5
#define N32_AFIO_SHRT2_EXEV_CFG1_EV8_MASK    0x1F        /* EXEV8 event source (5-bit) */
#define N32_AFIO_SHRT2_EXEV_CFG1_EV8_SHIFT   10
#define N32_AFIO_SHRT2_EXEV_CFG1_EV9_MASK    0x1F        /* EXEV9 event source (5-bit) */
#define N32_AFIO_SHRT2_EXEV_CFG1_EV9_SHIFT   15
#define N32_AFIO_SHRT2_EXEV_CFG1_EV10_MASK   0x1F        /* EXEV10 event source (5-bit) */
#define N32_AFIO_SHRT2_EXEV_CFG1_EV10_SHIFT  20
#define N32_AFIO_SHRT2_EXEV_CFG1_RESERVED_MASK 0xFE000000 /* Reserved bits (25-31) */

/* Event source encoding (for all EXEVx fields) */
#define N32_EXEV_SOURCE_PB4      0x01
#define N32_EXEV_SOURCE_PB5      0x02
#define N32_EXEV_SOURCE_PB6      0x03
#define N32_EXEV_SOURCE_PB7      0x04
#define N32_EXEV_SOURCE_PC10     0x05
#define N32_EXEV_SOURCE_PC12     0x06
#define N32_EXEV_SOURCE_PD5      0x07
#define N32_EXEV_SOURCE_PD8      0x08
#define N32_EXEV_SOURCE_PD9      0x09
#define N32_EXEV_SOURCE_PE6      0x0A
#define N32_EXEV_SOURCE_PG0      0x0B
#define N32_EXEV_SOURCE_PG11     0x0C
#define N32_EXEV_SOURCE_PG12     0x0D
#define N32_EXEV_SOURCE_PG13     0x0E
#define N32_EXEV_SOURCE_PI14     0x0F
#define N32_EXEV_SOURCE_PJ5      0x10
#define N32_EXEV_SOURCE_PK3      0x11
#define N32_EXEV_SOURCE_PK4      0x12

/* SHRTIM2 specific sources */
#define N32_EXEV_SOURCE_PA2      0x01
#define N32_EXEV_SOURCE_PC4      0x02
#define N32_EXEV_SOURCE_PD0      0x03
#define N32_EXEV_SOURCE_PD11     0x04
#define N32_EXEV_SOURCE_PE3      0x05
#define N32_EXEV_SOURCE_PE14     0x06
#define N32_EXEV_SOURCE_PF10     0x07
#define N32_EXEV_SOURCE_PG8      0x08
#define N32_EXEV_SOURCE_PG15     0x09
#define N32_EXEV_SOURCE_PH7      0x0A
#define N32_EXEV_SOURCE_PH8      0x0B
#define N32_EXEV_SOURCE_PH10     0x0C
#define N32_EXEV_SOURCE_PH11     0x0D
#define N32_EXEV_SOURCE_PH12     0x0E
#define N32_EXEV_SOURCE_PI11     0x0F
#define N32_EXEV_SOURCE_PJ2      0x10
#define N32_EXEV_SOURCE_PJ14     0x11
#define N32_EXEV_SOURCE_PK0      0x12

/* SIP Pull-up/Pull-down Configuration Register (AFIO_SIP_PUPD) */
#define N32_AFIO_SIP_PUPD_PUPD0_MASK  0x03  /* SIP pad0 pull config */
#define N32_AFIO_SIP_PUPD_PUPD0_SHIFT 0
#define N32_AFIO_SIP_PUPD_PUPD1_MASK  0x0C  /* SIP pad1 pull config */
#define N32_AFIO_SIP_PUPD_PUPD1_SHIFT 2
#define N32_AFIO_SIP_PUPD_PUPD2_MASK  0x30  /* SIP pad2 pull config */
#define N32_AFIO_SIP_PUPD_PUPD2_SHIFT 4
#define N32_AFIO_SIP_PUPD_PUPD3_MASK  0xC0  /* SIP pad3 pull config */
#define N32_AFIO_SIP_PUPD_PUPD3_SHIFT 6
#define N32_AFIO_SIP_PUPD_PUPD4_MASK  0x300  /* SIP pad4 pull config */
#define N32_AFIO_SIP_PUPD_PUPD4_SHIFT 8
#define N32_AFIO_SIP_PUPD_PUPD5_MASK  0xC00  /* SIP pad5 pull config */
#define N32_AFIO_SIP_PUPD_PUPD5_SHIFT 10
#define N32_AFIO_SIP_PUPD_PUPD6_MASK  0x3000 /* SIP pad6 pull config */
#define N32_AFIO_SIP_PUPD_PUPD6_SHIFT 12
#define N32_AFIO_SIP_PUPD_PUPD7_MASK  0xC000 /* SIP pad7 pull config */
#define N32_AFIO_SIP_PUPD_PUPD7_SHIFT 14
#define N32_AFIO_SIP_PUPD_RESERVED_MASK 0xFFFF0000 /* Reserved bits (16-31) */

/* Pull configuration values (for each PUPDx field) */
#define N32_PUPD_NONE   0x0  /* No pull-up or pull-down */
#define N32_PUPD_UP     0x1  /* Pull-up enabled */
#define N32_PUPD_DOWN   0x2  /* Pull-down enabled */
#define N32_PUPD_RESV   0x3  /* Reserved (do not use) */

/* SDRAM High-Speed Mode Configuration Registers */

/* AFIO_SDRAM_HSMOD_CFG0 - SDRAM HS Mode Config 0 (Offset: 0x98) */
#define N32_AFIO_SDRAM_HSMOD_CFG0_HSA_MASK    0xFFFF       /* Port A HS enable */
#define N32_AFIO_SDRAM_HSMOD_CFG0_HSA_SHIFT   0
#define N32_AFIO_SDRAM_HSMOD_CFG0_HSB_MASK    0xFFFF0000   /* Port B HS enable */
#define N32_AFIO_SDRAM_HSMOD_CFG0_HSB_SHIFT   16

/* AFIO_SDRAM_HSMOD_CFG1 - SDRAM HS Mode Config 1 (Offset: 0x9C) */
#define N32_AFIO_SDRAM_HSMOD_CFG1_HSC_MASK    0xFFFF       /* Port C HS enable */
#define N32_AFIO_SDRAM_HSMOD_CFG1_HSC_SHIFT   0
#define N32_AFIO_SDRAM_HSMOD_CFG1_HSD_MASK    0xFFFF0000   /* Port D HS enable */
#define N32_AFIO_SDRAM_HSMOD_CFG1_HSD_SHIFT   16

/* AFIO_SDRAM_HSMOD_CFG2 - SDRAM HS Mode Config 2 (Offset: 0xA0) */
#define N32_AFIO_SDRAM_HSMOD_CFG2_HSE_MASK    0xFFFF       /* Port E HS enable */
#define N32_AFIO_SDRAM_HSMOD_CFG2_HSE_SHIFT   0
#define N32_AFIO_SDRAM_HSMOD_CFG2_HSF_MASK    0xFFFF0000   /* Port F HS enable */
#define N32_AFIO_SDRAM_HSMOD_CFG2_HSF_SHIFT   16

/* AFIO_SDRAM_HSMOD_CFG3 - SDRAM HS Mode Config 3 (Offset: 0xA4) */
#define N32_AFIO_SDRAM_HSMOD_CFG3_HSG_MASK    0xFFFF       /* Port G HS enable */
#define N32_AFIO_SDRAM_HSMOD_CFG3_HSG_SHIFT   0
#define N32_AFIO_SDRAM_HSMOD_CFG3_HSH_MASK    0xFFFF0000   /* Port H HS enable */
#define N32_AFIO_SDRAM_HSMOD_CFG3_HSH_SHIFT   16

/* AFIO_SDRAM_HSMOD_CFG4 - SDRAM HS Mode Config 4 (Offset: 0xA8) */
#define N32_AFIO_SDRAM_HSMOD_CFG4_HSI_MASK    0xFFFF       /* Port I HS enable */
#define N32_AFIO_SDRAM_HSMOD_CFG4_HSI_SHIFT   0
#define N32_AFIO_SDRAM_HSMOD_CFG4_RESERVED_MASK 0xFFFF0000 /* Reserved bits (16-31) */

/* HS_EN bit values */
#define N32_HS_DISABLE 0  /* Disable high-speed mode */
#define N32_HS_ENABLE  1  /* Enable high-speed mode */

/* SIP Slew Rate Configuration Register (AFIO_SIP_SR) */
#define N32_AFIO_SIP_SR_SR0_MASK    0x01  /* Pad0 slew rate control */
#define N32_AFIO_SIP_SR_SR0_SHIFT   0
#define N32_AFIO_SIP_SR_SR1_MASK    0x02  /* Pad1 slew rate control */
#define N32_AFIO_SIP_SR_SR1_SHIFT   1
#define N32_AFIO_SIP_SR_SR2_MASK    0x04  /* Pad2 slew rate control */
#define N32_AFIO_SIP_SR_SR2_SHIFT   2
#define N32_AFIO_SIP_SR_SR3_MASK    0x08  /* Pad3 slew rate control */
#define N32_AFIO_SIP_SR_SR3_SHIFT   3
#define N32_AFIO_SIP_SR_SR4_MASK    0x10  /* Pad4 slew rate control */
#define N32_AFIO_SIP_SR_SR4_SHIFT   4
#define N32_AFIO_SIP_SR_SR5_MASK    0x20  /* Pad5 slew rate control */
#define N32_AFIO_SIP_SR_SR5_SHIFT   5
#define N32_AFIO_SIP_SR_SR6_MASK    0x40  /* Pad6 slew rate control */
#define N32_AFIO_SIP_SR_SR6_SHIFT   6
#define N32_AFIO_SIP_SR_SR7_MASK    0x80  /* Pad7 slew rate control */
#define N32_AFIO_SIP_SR_SR7_SHIFT   7
#define N32_AFIO_SIP_SR_RESERVED_MASK 0xFFFFFF00 /* Reserved bits (8-31) */

/* Slew Rate values */
#define N32_SR_FAST 0  /* Fast slew rate */
#define N32_SR_SLOW 1  /* Slow slew rate */

/* SIP Driver Strength Configuration Register (AFIO_SIP_DS) */
#define N32_AFIO_SIP_DS_DS0_MASK    0x03  /* Pad0 driver strength */
#define N32_AFIO_SIP_DS_DS0_SHIFT   0
#define N32_AFIO_SIP_DS_DS1_MASK    0x0C  /* Pad1 driver strength */
#define N32_AFIO_SIP_DS_DS1_SHIFT   2
#define N32_AFIO_SIP_DS_DS2_MASK    0x30  /* Pad2 driver strength */
#define N32_AFIO_SIP_DS_DS2_SHIFT   4
#define N32_AFIO_SIP_DS_DS3_MASK    0xC0  /* Pad3 driver strength */
#define N32_AFIO_SIP_DS_DS3_SHIFT   6
#define N32_AFIO_SIP_DS_DS4_MASK    0x300  /* Pad4 driver strength */
#define N32_AFIO_SIP_DS_DS4_SHIFT   8
#define N32_AFIO_SIP_DS_DS5_MASK    0xC00  /* Pad5 driver strength */
#define N32_AFIO_SIP_DS_DS5_SHIFT   10
#define N32_AFIO_SIP_DS_DS6_MASK    0x3000 /* Pad6 driver strength */
#define N32_AFIO_SIP_DS_DS6_SHIFT   12
#define N32_AFIO_SIP_DS_DS7_MASK    0xC000 /* Pad7 driver strength */
#define N32_AFIO_SIP_DS_DS7_SHIFT   14
#define N32_AFIO_SIP_DS_RESERVED_MASK 0xFFFF0000 /* Reserved bits (16-31) */

/* Driver Strength values (for each DSy field) */
#define N32_DS_2MA   0x0  /* 2mA drive strength */
#define N32_DS_8MA   0x1  /* 8mA drive strength */
#define N32_DS_4MA   0x2  /* 4mA drive strength */
#define N32_DS_12MA  0x3  /* 12mA drive strength */

/* ADC Switch Configuration Register (AFIO_ADCSW_CFG) */
#define N32_AFIO_ADCSW_CFG_SWPA0_C_MASK   0x0F       /* PA0_C switch config */
#define N32_AFIO_ADCSW_CFG_SWPA0_C_SHIFT  0
#define N32_AFIO_ADCSW_CFG_SWPC2_C_MASK   0xF0       /* PC2_C switch config */
#define N32_AFIO_ADCSW_CFG_SWPC2_C_SHIFT  4
#define N32_AFIO_ADCSW_CFG_SWPC3_C_MASK   0x300      /* PC3_C switch config */
#define N32_AFIO_ADCSW_CFG_SWPC3_C_SHIFT  8
#define N32_AFIO_ADCSW_CFG_SWPA1_C_MASK   0xC00      /* PA1_C switch config */
#define N32_AFIO_ADCSW_CFG_SWPA1_C_SHIFT  10
#define N32_AFIO_ADCSW_CFG_SWPJ15_MASK    0x3000     /* PJ15 switch config */
#define N32_AFIO_ADCSW_CFG_SWPJ15_SHIFT   12
#define N32_AFIO_ADCSW_CFG_VBAT           (1 << 14)  /* VBAT monitor enable */
#define N32_AFIO_ADCSW_CFG_DAC_OUT2_MASK  0xE0000    /* DACx_OUT2 config */
#define N32_AFIO_ADCSW_CFG_DAC_OUT2_SHIFT 17
#define N32_AFIO_ADCSW_CFG_SWPJ0          (1 << 18)  /* PJ0 switch open */
#define N32_AFIO_ADCSW_CFG_SWPJ3          (1 << 19)  /* PJ3 switch open */
#define N32_AFIO_ADCSW_CFG_TEMP           (1 << 20)  /* Temp monitor enable */
#define N32_AFIO_ADCSW_CFG_SWPJ4          (1 << 21)  /* PJ4 switch open */
#define N32_AFIO_ADCSW_CFG_SWPJ5          (1 << 22)  /* PJ5 switch open */
#define N32_AFIO_ADCSW_CFG_DAC_OUT1_MASK  0xE000000  /* DACx_OUT1 config */
#define N32_AFIO_ADCSW_CFG_DAC_OUT1_SHIFT 23
#define N32_AFIO_ADCSW_CFG_SWPJ6          (1 << 26)  /* PJ6 switch open */
#define N32_AFIO_ADCSW_CFG_SWPJ7          (1 << 27)  /* PJ7 switch open */
#define N32_AFIO_ADCSW_CFG_VREF           (1 << 28)  /* VREF monitor enable */
#define N32_AFIO_ADCSW_CFG_RESERVED_MASK  0xE0000000 /* Reserved bits (29-31) */

/* DAC monitor values */
#define N32_DAC_MON_DISABLE 0x0  /* Disable monitor */
#define N32_DAC1_MON_EN     0x1  /* Enable DAC1 monitor */
#define N32_DAC2_MON_EN     0x2  /* Enable DAC2 monitor */
#define N32_DAC3_MON_EN     0x4  /* Enable DAC3 monitor */

/* SDRAM DSN Configuration Registers */
#define N32_AFIO_SDRAM_DSN_CFG0_OFFSET 0xE4
#define N32_AFIO_SDRAM_DSN_CFG1_OFFSET 0xE8
#define N32_AFIO_SDRAM_DSN_CFG2_OFFSET 0xEC

/* SDRAM DSP Configuration Registers */
#define N32_AFIO_SDRAM_DSP_CFG0_OFFSET 0xF0
#define N32_AFIO_SDRAM_DSP_CFG1_OFFSET 0xF4
#define N32_AFIO_SDRAM_DSP_CFG2_OFFSET 0xF8

/* SDRAM pad mapping (for VREF_EN, DSN, DSP) */
#define N32_SDRAM_PAD_PA4   0
#define N32_SDRAM_PAD_PA5   1
#define N32_SDRAM_PAD_PA7   2
#define N32_SDRAM_PAD_PB5   3
#define N32_SDRAM_PAD_PB6   4
#define N32_SDRAM_PAD_PB14  5
#define N32_SDRAM_PAD_PB15  6
#define N32_SDRAM_PAD_PC0   7
/* ... (complete mapping from Table 12-118) */
#define N32_SDRAM_PAD_PI10  72

#endif /* __ARCH_ARM_SRC_N32H7_HARDWARE_N32H76X_AFIO_H */
