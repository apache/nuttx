/****************************************************************************
 * arch/arm/src/imx1/imx_system.h
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

#ifndef __ARCH_ARM_SRC_IMX1_IMX_SYSTEM_H
#define __ARCH_ARM_SRC_IMX1_IMX_SYSTEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* AIPI Register Offsets ****************************************************/

#define AIPI_PSR0_OFFSET        0x0000 /* Peripheral Size Register 0 */
#define AIPI_PSR1_OFFSET        0x0004 /* Peripheral Size Register 1 */
#define AIPI_PAR_OFFSET         0x0008 /* Peripheral Access Register */

/* AIPI Register Addresses **************************************************/

#define IMX_AIPI1_PSR0          (IMX_AIPI1_VBASE + AIPI_PSR0_OFFSET)
#define IMX_AIPI1_PSR1          (IMX_AIPI1_VBASE + AIPI_PSR1_OFFSET)
#define IMX_AIPI1_PAR           (IMX_AIPI1_VBASE + AIPI_PAR_OFFSET)

#define IMX_AIPI2_PSR0          (IMX_AIP2_VBASE + AIPI_PSR0_OFFSET)
#define IMX_AIPI2_PSR1          (IMX_AIP2_VBASE + AIPI_PSR1_OFFSET)
#define IMX_AIPI2_PAR           (IMX_AIP2_VBASE + 0xAIPI_PAR_OFFSET)

/* AIPI Register Bit Definitions ********************************************/

/* PLL Register Offsets *****************************************************/

#define PLL_CSCR_OFFSET         0x0000 /* Clock Source Control Register */
#define PLL_MPCTL0_OFFSET       0x0004 /* MCU PLL Control Register 0 */
#define PLL_MPCTL1_OFFSET       0x0008 /* MCU PLL & System Clock Control Register 1 */
#define PLL_SPCTL0_OFFSET       0x000c /* System PLL Control Register 0 */
#define PLL_SPCTL1_OFFSET       0x0010 /* System PLL Control Register 1 */
#define PLL_PCDR_OFFSET         0x0020 /* Peripheral Clock Divider Register */

/* PLL Register Addresses ***************************************************/

#define IMX_PLL_CSCR            (IMX_PLL_VBASE + PLL_CSCR_OFFSET)
#define IMX_PLL_MPCTL0          (IMX_PLL_VBASE + PLL_MPCTL0_OFFSET)
#define IMX_PLL_MPCTL1          (IMX_PLL_VBASE + PLL_MPCTL1_OFFSET)
#define IMX_PLL_SPCTL0          (IMX_PLL_VBASE + PLL_SPCTL0_OFFSET)
#define IMX_PLL_SPCTL1          (IMX_PLL_VBASE + PLL_SPCTL1_OFFSET)
#define IMX_PLL_PCDR            (IMX_PLL_VBASE + PLL_PCDR_OFFSET)

/* PLL Register Bit Definitions *********************************************/

#define PLL_CSCR_MPEN           (1 << 0)  /* Bit 0: 1 = MCU PLL enabled */
#define PLL_CSCR_SPEN           (1 << 1)  /* Bit 1: System PLL Enable */
#define PLL_CSCR_BCLKDIV_SHIFT  10        /* Bits 13-10: BClock Divider */
#define PLL_CSCR_BCLKDIV_MASK   (15 << PLL_CSCR_BCLK_DIV_SHIFT)
#define PLL_CSCR_PRESC          (1 << 15) /* Bit 15: MPU PLL clock prescaler */
#define PLL_CSCR_SYSTEM_SEL     (1 << 16) /* Bit 16: System clock source select */
#define PLL_CSCR_OSCEN          (1 << 17) /* Bit 17: Ext. 16MHz oscillator enable */
#define PLL_CSCR_CLK16_SEL      (1 << 18) /* Bit 18: Select BT ref RFBTCLK16 */
#define PLL_CSCR_MPLLRESTART    (1 << 21) /* Bit 21: MPLL Restart */
#define PLL_CSCR_SPLLRESTART    (1 << 22) /* Bit 22: SPLL Restart */
#define PLL_CSCR_SDCNT_SHIFT    24        /* Bits 25-24: Shut-Down Control */
#define PLL_CSCR_SDCNT_MASK     (3 << PLL_CSCR_SDCNT_SHIFT)
#define   CSCR_SDCNT_2ndEDGE    (1 << PLL_CSCR_SDCNT_SHIFT)
#define   CSCR_SDCNT_3rdEDGE    (2 << PLL_CSCR_SDCNT_SHIFT)
#define   CSCR_SDCNT_4thEDGE    (3 << PLL_CSCR_SDCNT_SHIFT)
#define PLL_CSCR_USBDIV_SHIFT   28        /* Bits 28-26: USB Divider */
#define PLL_CSCR_USBDIV_MASK    (7 << PLL_CSCR_USB_DIV_SHIFT)
#define PLL_CSCR_CLKOSEL_SHIFT  29        /* Bits 31-29: CLKO Select */
#define PLL_CSCR_CLKOSEL_MASK   (7 << PLL_CSCR_CLKOSEL_SHIFT)
#define   CSCR_CLKOSEL_PERCLK1  (0 << PLL_CSCR_CLKOSEL_SHIFT)
#define   CSCR_CLKOSEL_HCLK     (1 << PLL_CSCR_CLKOSEL_SHIFT)
#define   CSCR_CLKOSEL_CLK48M   (2 << PLL_CSCR_CLKOSEL_SHIFT)
#define   CSCR_CLKOSEL_CLK16M   (3 << PLL_CSCR_CLKOSEL_SHIFT)
#define   CSCR_CLKOSEL_PREMCLK  (4 << PLL_CSCR_CLKOSEL_SHIFT)
#define   CSCR_CLKOSEL_FCLK     (5 << PLL_CSCR_CLKOSEL_SHIFT)

#define PLL_MPCTL0_MFN_SHIFT    0         /* Bits 9-0: Multiplication Factor (Numerator) */
#define PLL_MPCTL0_MFN_MASK     (0x03ff << PLL_MPCTL0_MFN_SHIFT)
#define PLL_MPCTL0_MFI_SHIFT    10        /* Bits 13-10: Multiplication Factor (Integer) */
#define PLL_MPCTL0_MFI_MASK     (0x0f << PLL_MPCTL0_MFI_SHIFT)
#define PLL_MPCTL0_MFD_SHIFT    16        /* Bits 25-16: Multiplication Factor (Denominator) */
#define PLL_MPCTL0_MFD_MASK     (0x03ff << PLL_MPCTL0_MFD_SHIFT)
#define PLL_MPCTL0_PD_SHIFT     26        /* Bits 29-26: Predivider Factor */
#define PLL_MPCTL0_PD_MASK      (0x0f << PLL_MPCTL0_PD_SHIFT

#define PLL_MPCTL1_BRMO         (1 << 6)  /* Bit 6: Controls the BRM order */

#define PLL_SPCTL0_MFN_SHIFT    0         /* Bits 9-0: Multiplication Factor (Numerator) */
#define PLL_SPCTL0_MFN_MASK     (0x03ff << PLL_SPCTL0_MFN_SHIFT)
#define PLL_SPCTL0_MFI_SHIFT    10        /* Bits 13-10: Multiplication Factor (Integer) */
#define PLL_SPCTL0_MFI_MASK     (0x0f << PLL_SPCTL0_MFI_SHIFT)
#define PLL_SPCTL0_MFD_SHIFT    16        /* Bits 25-16: Multiplication Factor (Denominator) */
#define PLL_SPCTL0_MFD_MASK     (0x03ff << PLL_SPCTL0_MFD_SHIFT)
#define PLL_SPCTL0_PD_SHIFT     26        /* Bits 29-26: Predivider Factor */
#define PLL_SPCTL0_PD_MASK      (0x0f << PLL_SPCTL0_PD_SHIFT)

#define PLL_SPCTL1_BRMO         (1 << 6)  /* Bit 6: Controls the BRM order */
#define PLL_SPCTL1_LF           (1 << 15) /* Bit 15: Indicates if System PLL is locked */

#define PLL_PCDR_PCLKDIV1_SHIFT 0         /* Bits 3-0: Peripheral Clock Divider 1 */
#define PLL_PCDR_PCLKDIV1_MASK  (0x0f << PLL_PCDR_PCLKDIV1_SHIFT)
#define PLL_PCDR_PCLKDIV2_SHIFT 4         /* Bits 7-4: Peripheral Clock Divider 2 */
#define PLL_PCDR_PCLKDIV2_MASK  (0x0f << PLL_PCDR_PCLKDIV2_SHIFT)
#define PLL_PCDR_PCLKDIV3_SHIFT 16        /* Bits 22-16: Peripheral Clock Divider 3 */
#define PLL_PCDR_PCLKDIV3_MASK  (0x7f << PLL_PCDR_PCLKDIV3_SHIFT)

/* PLL Helper Macros ********************************************************/

/* SC Register Offsets ******************************************************/

#define SC_RSR_OFFSET           0x0000 /* Reset Source Register */
#define SC_SIDR_OFFSET          0x0004 /* Silicon ID Register */
#define SC_FMCR_OFFSET          0x0008 /* Function Muxing Control Register */
#define SC_GPCR_OFFSET          0x000c /* Global Peripheral Control Register */

/* SC Register Addresses ****************************************************/

#define IMX_SC_RSR              (IMX_SC_VBASE + SC_RSR_OFFSET)
#define IMX_SC_SIDR             (IMX_SC_VBASE + SC_SIDR_OFFSET)
#define IMX_SC_FMCR             (IMX_SC_VBASE + SC_FMCR_OFFSET)
#define IMX_SC_GPCR             (IMX_SC_VBASE + SC_GPCR_OFFSET)

/* SC Register Bit Definitions **********************************************/

#define FMCR_SDCS_SEL           (1 << 0) /* Bit 0: 1:CSD0 selected */
#define FMCR_SDCS1_SEL          (1 << 1) /* Bit 1: 1:CSD1 selected */
#define FMCR_EXT_BREN           (1 << 2) /* Bit 2: 1:External bus request enabled */
#define FMCR_SSI_TXCLKSEL       (1 << 3) /* Bit 3: 1:Input from Port B[19] SIM_CLK pin */
#define FMCR_SSI_TXFSSEL        (1 << 4) /* Bit 4: 1:Input from Port B[18] SIM_RST pin */
#define FMCR_SSI_RXDATSEL       (1 << 5) /* Bit 5: 1:Input from Port B[16] SIM_TX pin */
#define FMCR_SSI_RXCLKSEL       (1 << 6) /* Bit 6: 1:Input from Port B[15] SIM_PD pin */
#define FMCR_SSI_RXFSSEL        (1 << 7) /* Bit 7: 1:Input from Port B[14] SIM_SVEN pin */
#define FMCR_SPI2_RXDSEL        (1 << 8) /* Bit 8: 1:Input from SPI2_RXD_1 pin
                                          * (AOUT of Port D[9]) */

/* SDRAMC Register Offsets **************************************************/

#define SDRAMC_SDCTL0_OFFSET    0x0000
#define SDRAMC_SDCTL1_OFFSET    0x0004

/* SDRAMC Register Addresses ************************************************/

#define IMX_SDRAMC_SDCTL0       (IMX_SDRAMC_VBASE + SDRAMC_SDCTL0_OFFSET)
#define IMX_SDRAMC_SDCTL1       (IMX_SDRAMC_VBASE + SDRAMC_SDCTL1_OFFSET))

/* SDRAMC Register Bit Definitions ******************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_IMX1_IMX_SYSTEM_H */
