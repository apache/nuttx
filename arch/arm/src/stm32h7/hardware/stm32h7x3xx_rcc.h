/***************************************************************************************************************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32h7x3xx_rcc.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *            Mateusz Szafoni <raiden00@railab.me>
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
 ***************************************************************************************************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7X3XX_RCC_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7X3XX_RCC_H

/***************************************************************************************************************************************************************************
 * Included Files
 ***************************************************************************************************************************************************************************/

#include <nuttx/config.h>

/***************************************************************************************************************************************************************************
 * Pre-processor Definitions
 ***************************************************************************************************************************************************************************/

/* Register Offsets ********************************************************************************************************************************************************/

/* TODO: Complete comments */

#define STM32_RCC_CR_OFFSET             0x0000  /* Clock control register */
#define STM32_RCC_ICSCR_OFFSET          0x0004
#define STM32_RCC_CRRCR_OFFSET          0x0008
#define STM32_RCC_CFGR_OFFSET           0x0010  /* Clock configuration register */
#define STM32_RCC_D1CFGR_OFFSET         0x0018
#define STM32_RCC_D2CFGR_OFFSET         0x001c
#define STM32_RCC_D3CFGR_OFFSET         0x0020
#define STM32_RCC_PLLCKSELR_OFFSET      0x0028
#define STM32_RCC_PLLCFGR_OFFSET        0x002c
#define STM32_RCC_PLL1DIVR_OFFSET       0x0030
#define STM32_RCC_PLL1FRACR_OFFSET      0x0034
#define STM32_RCC_PLL2DIVR_OFFSET       0x0038
#define STM32_RCC_PLL2FRACR_OFFSET      0x003c
#define STM32_RCC_PLL3DIVR_OFFSET       0x0040
#define STM32_RCC_PLL3FRACR_OFFSET      0x0044
#define STM32_RCC_D1CCIPR_OFFSET        0x004c  /* Domain 1 Kernel Clock configuration register */
#define STM32_RCC_D2CCIP1R_OFFSET       0x0050  /* Domain 2 Kernel Clock configuration register 1 */
#define STM32_RCC_D2CCIP2R_OFFSET       0x0054  /* Domain 2 Kernel Clock configuration register 2 */
#define STM32_RCC_D3CCIPR_OFFSET        0x0058  /* Domain 3 Kernel Clock configuration register */
#define STM32_RCC_CIER_OFFSET           0x0060  /* Clock Source Interrupt enable register */
#define STM32_RCC_CIFR_OFFSET           0x0064  /* Clock Source Interrupt Flag register */
#define STM32_RCC_CICR_OFFSET           0x0068  /* Clock Source Interrupt Clear register  */
#define STM32_RCC_BDCR_OFFSET           0x0070  /* Backup Domain Control register */
#define STM32_RCC_CSR_OFFSET            0x0074  /* Clock Control and Status register */
#define STM32_RCC_AHB1RSTR_OFFSET       0x0080  /* AHB1 peripheral reset register */
#define STM32_RCC_AHB2RSTR_OFFSET       0x0084  /* AHB2 peripheral reset register */
#define STM32_RCC_AHB3RSTR_OFFSET       0x007c  /* AHB3 peripheral reset register */
#define STM32_RCC_AHB4RSTR_OFFSET       0x0088  /* AHB4 peripheral reset register */
#define STM32_RCC_APB1LRSTR_OFFSET      0x0090  /* APB1 L Peripheral reset register */
#define STM32_RCC_APB1HRSTR_OFFSET      0x0094  /* APB1 H Peripheral reset register */
#define STM32_RCC_APB2RSTR_OFFSET       0x0098  /* APB2 Peripheral reset register */
#define STM32_RCC_APB3RSTR_OFFSET       0x008c  /* APB3 Peripheral reset register */
#define STM32_RCC_APB4RSTR_OFFSET       0x009c  /* APB4 Peripheral reset register */
#define STM32_RCC_GCR_OFFSET            0x00a0  /* RCC Global Control register */
#define STM32_RCC_D3AMR_OFFSET          0x00a8  /* D3 Autonomous mode register */
#define STM32_RCC_RSR_OFFSET            0x00d0  /* RCC Reset Status register */
#define STM32_RCC_AHB1ENR_OFFSET        0x00d8  /* AHB1 Peripheral Clock enable register */
#define STM32_RCC_AHB2ENR_OFFSET        0x00dc  /* AHB2 Peripheral Clock enable register */
#define STM32_RCC_AHB3ENR_OFFSET        0x00d4  /* AHB3 Peripheral Clock enable register */
#define STM32_RCC_AHB4ENR_OFFSET        0x00e0  /* AHB4 Peripheral Clock enable register */
#define STM32_RCC_APB1LENR_OFFSET       0x00e8  /* APB1 L Peripheral Clock enable register */
#define STM32_RCC_APB1HENR_OFFSET       0x00ec  /* APB1 H Peripheral Clock enable register */
#define STM32_RCC_APB2ENR_OFFSET        0x00f0  /* APB2 Peripheral Clock enable register */
#define STM32_RCC_APB3ENR_OFFSET        0x00e4  /* APB3 Peripheral Clock enable register */
#define STM32_RCC_APB4ENR_OFFSET        0x00f4  /* APB4 Peripheral Clock enable register */
#define STM32_RCC_AHB1LPENR_OFFSET      0x0100  /* RCC AHB1 low power mode peripheral clock enable register */
#define STM32_RCC_AHB2LPENR_OFFSET      0x0104  /* RCC AHB2 low power mode peripheral clock enable register */
#define STM32_RCC_AHB3LPENR_OFFSET      0x00fc  /* RCC AHB3 low power mode peripheral clock enable register */
#define STM32_RCC_AHB4LPENR_OFFSET      0x0108  /* RCC AHB4 low power mode peripheral clock enable register */
#define STM32_RCC_APB1LLPENR_OFFSET     0x0110  /* RCC APB1 L low power mode peripheral clock enable register */
#define STM32_RCC_APB1HLPENR_OFFSET     0x0114  /* RCC APB1 H low power mode peripheral clock enable register */
#define STM32_RCC_APB2LPENR_OFFSET      0x0118  /* RCC APB2 low power mode peripheral clock enable register */
#define STM32_RCC_APB3LPENR_OFFSET      0x010c  /* RCC APB3 low power mode peripheral clock enable register */
#define STM32_RCC_APB4LPENR_OFFSET      0x011c  /* RCC APB4 low power mode peripheral clock enable register */

/* Register Addresses ******************************************************************************************************************************************************/

#define STM32_RCC_CR                    (STM32_RCC_BASE + STM32_RCC_CR_OFFSET)
#define STM32_RCC_ICSCR                 (STM32_RCC_BASE + STM32_RCC_ICSCR_OFFSET)
#define STM32_RCC_CRRCR                 (STM32_RCC_BASE + STM32_RCC_CRRCR_OFFSET)
#define STM32_RCC_CFGR                  (STM32_RCC_BASE + STM32_RCC_CFGR_OFFSET)
#define STM32_RCC_D1CFGR                (STM32_RCC_BASE + STM32_RCC_D1CFGR_OFFSET)
#define STM32_RCC_D2CFGR                (STM32_RCC_BASE + STM32_RCC_D2CFGR_OFFSET)
#define STM32_RCC_D3CFGR                (STM32_RCC_BASE + STM32_RCC_D3CFGR_OFFSET)
#define STM32_RCC_PLLCKSELR             (STM32_RCC_BASE + STM32_RCC_PLLCKSELR_OFFSET)
#define STM32_RCC_PLLCFGR               (STM32_RCC_BASE + STM32_RCC_PLLCFGR_OFFSET)
#define STM32_RCC_PLL1DIVR              (STM32_RCC_BASE + STM32_RCC_PLL1DIVR_OFFSET)
#define STM32_RCC_PLL1FRACR             (STM32_RCC_BASE + STM32_RCC_PLL1FRACR_OFFSET)
#define STM32_RCC_PLL2DIVR              (STM32_RCC_BASE + STM32_RCC_PLL2DIVR_OFFSET)
#define STM32_RCC_PLL2FRACR             (STM32_RCC_BASE + STM32_RCC_PLL2FRACR_OFFSET)
#define STM32_RCC_PLL3DIVR              (STM32_RCC_BASE + STM32_RCC_PLL3DIVR_OFFSET)
#define STM32_RCC_PLL3FRACR             (STM32_RCC_BASE + STM32_RCC_PLL3FRACR_OFFSET)
#define STM32_RCC_D1CCIPR               (STM32_RCC_BASE + STM32_RCC_D1CCIPR_OFFSET)
#define STM32_RCC_D2CCIP1R              (STM32_RCC_BASE + STM32_RCC_D2CCIP1R_OFFSET)
#define STM32_RCC_D2CCIP2R              (STM32_RCC_BASE + STM32_RCC_D2CCIP2R_OFFSET)
#define STM32_RCC_D3CCIPR               (STM32_RCC_BASE + STM32_RCC_D3CCIPR_OFFSET)
#define STM32_RCC_CIER                  (STM32_RCC_BASE + STM32_RCC_CIER_OFFSET)
#define STM32_RCC_CIFR                  (STM32_RCC_BASE + STM32_RCC_CIFR_OFFSET)
#define STM32_RCC_CICR                  (STM32_RCC_BASE + STM32_RCC_CICR_OFFSET)
#define STM32_RCC_BDCR                  (STM32_RCC_BASE + STM32_RCC_BDCR_OFFSET)
#define STM32_RCC_CSR                   (STM32_RCC_BASE + STM32_RCC_CSR_OFFSET)
#define STM32_RCC_AHB1RSTR              (STM32_RCC_BASE + STM32_RCC_AHB1RSTR_OFFSET)
#define STM32_RCC_AHB2RSTR              (STM32_RCC_BASE + STM32_RCC_AHB2RSTR_OFFSET)
#define STM32_RCC_AHB3RSTR              (STM32_RCC_BASE + STM32_RCC_AHB3RSTR_OFFSET)
#define STM32_RCC_AHB4RSTR              (STM32_RCC_BASE + STM32_RCC_AHB4RSTR_OFFSET)
#define STM32_RCC_APB1LRSTR             (STM32_RCC_BASE + STM32_RCC_APB1LRSTR_OFFSET)
#define STM32_RCC_APB1HRSTR             (STM32_RCC_BASE + STM32_RCC_APB1HRSTR_OFFSET)
#define STM32_RCC_APB2RSTR              (STM32_RCC_BASE + STM32_RCC_APB2RSTR_OFFSET)
#define STM32_RCC_APB3RSTR              (STM32_RCC_BASE + STM32_RCC_APB3RSTR_OFFSET)
#define STM32_RCC_APB4RSTR              (STM32_RCC_BASE + STM32_RCC_APB4RSTR_OFFSET)
#define STM32_RCC_GCR                   (STM32_RCC_BASE + STM32_RCC_GCR_OFFSET)
#define STM32_RCC_D3AMR                 (STM32_RCC_BASE + STM32_RCC_D3AMR_OFFSET)
#define STM32_RCC_RSR                   (STM32_RCC_BASE + STM32_RCC_RSR_OFFSET)
#define STM32_RCC_AHB1ENR               (STM32_RCC_BASE + STM32_RCC_AHB1ENR_OFFSET)
#define STM32_RCC_AHB2ENR               (STM32_RCC_BASE + STM32_RCC_AHB2ENR_OFFSET)
#define STM32_RCC_AHB3ENR               (STM32_RCC_BASE + STM32_RCC_AHB3ENR_OFFSET)
#define STM32_RCC_AHB4ENR               (STM32_RCC_BASE + STM32_RCC_AHB4ENR_OFFSET)
#define STM32_RCC_APB1LENR              (STM32_RCC_BASE + STM32_RCC_APB1LENR_OFFSET)
#define STM32_RCC_APB1HENR              (STM32_RCC_BASE + STM32_RCC_APB1HENR_OFFSET)
#define STM32_RCC_APB2ENR               (STM32_RCC_BASE + STM32_RCC_APB2ENR_OFFSET)
#define STM32_RCC_APB3ENR               (STM32_RCC_BASE + STM32_RCC_APB3ENR_OFFSET)
#define STM32_RCC_APB4ENR               (STM32_RCC_BASE + STM32_RCC_APB4ENR_OFFSET)
#define STM32_RCC_AHB1LPENR             (STM32_RCC_BASE + STM32_RCC_AHB1LPENR_OFFSET)
#define STM32_RCC_AHB2LPENR             (STM32_RCC_BASE + STM32_RCC_AHB2LPENR_OFFSET)
#define STM32_RCC_AHB3LPENR             (STM32_RCC_BASE + STM32_RCC_AHB3LPENR_OFFSET)
#define STM32_RCC_AHB4LPENR             (STM32_RCC_BASE + STM32_RCC_AHB4LPENR_OFFSET)
#define STM32_RCC_APB1LLPENR            (STM32_RCC_BASE + STM32_RCC_APB1LLPENR_OFFSET)
#define STM32_RCC_APB1HLPENR            (STM32_RCC_BASE + STM32_RCC_APB1HLPENR_OFFSET)
#define STM32_RCC_APB2LPENR             (STM32_RCC_BASE + STM32_RCC_APB2LPENR_OFFSET)
#define STM32_RCC_APB3LPENR             (STM32_RCC_BASE + STM32_RCC_APB3LPENR_OFFSET)
#define STM32_RCC_APB4LPENR             (STM32_RCC_BASE + STM32_RCC_APB4LPENR_OFFSET)

/* Register Bitfield Definitions *******************************************************************************************************************************************/

/* Source Control Register */

#define RCC_CR_HSION                    (1 << 0)   /* Bit 0: Internal High Speed clock enable */
#define RCC_CR_HSIKERON                 (1 << 1)   /* Bit 1: Internal High Speed clock enable for some IPs Kernel ?? */
#define RCC_CR_HSIRDY                   (1 << 2)   /* Bit 2: Internal High Speed clock ready flag */
#define RCC_CR_HSIDIV_SHIFT             (3)        /* Bits 3-4: HSI clock divider */
#define RCC_CR_HSIDIV_MASK              (3 << RCC_CR_HSIDIV_SHIFT)
#  define RCC_CR_HSIDIV_1               (0 << RCC_CR_HSIDIV_SHIFT)
#  define RCC_CR_HSIDIV_2               (1 << RCC_CR_HSIDIV_SHIFT)
#  define RCC_CR_HSIDIV_4               (2 << RCC_CR_HSIDIV_SHIFT)
#  define RCC_CR_HSIDIV_8               (3 << RCC_CR_HSIDIV_SHIFT)
#define RCC_CR_HSIDIVF                  (1 << 5)   /* Bit 5: HSI Divider flag */
                                                   /* Bit 6: Reserved */
#define RCC_CR_CSION                    (1 << 7)   /* Bit 7: The Internal RC 4MHz oscillator clock enable */
#define RCC_CR_CSIRDY                   (1 << 8)   /* Bit 8: The Internal RC 4MHz oscillator clock ready */
#define RCC_CR_CSIKERON                 (1 << 9)   /* Bit 9: Internal RC 4MHz oscillator clock enable for some IPs Kernel */
                                                   /* Bits 10-11: Reserved */
#define RCC_CR_HSI48ON                  (1 << 12)  /* Bit 12: HSI48 clock enable clock enable */
#define RCC_CR_HSI48RDY                 (1 << 13)  /* Bit 13: HSI48 clock ready */
#define RCC_CR_D1CKRDY                  (1 << 14)  /* Bit 14: D1 domain clocks ready flag */
#define RCC_CR_D2CKRDY                  (1 << 15)  /* Bit 15: D2 domain clocks ready flag */
#define RCC_CR_HSEON                    (1 << 16)  /* Bit 16: External High Speed clock enable */
#define RCC_CR_HSERDY                   (1 << 17)  /* Bit 17: External High Speed clock ready */
#define RCC_CR_HSEBYP                   (1 << 18)  /* Bit 18: External High Speed clock Bypass */
#define RCC_CR_CSSHSEON                 (1 << 19)  /* Bit 19: HSE Clock security System enable */
                                                   /* Bits 20-23: Reserved */
#define RCC_CR_PLL1ON                   (1 << 24)  /* Bit 24: System PLL1 clock enable */
#define RCC_CR_PLL1RDY                  (1 << 25)  /* Bit 25: System PLL1 clock ready */
#define RCC_CR_PLL2ON                   (1 << 26)  /* Bit 26: System PLL2 clock enable */
#define RCC_CR_PLL2RDY                  (1 << 27)  /* Bit 27: System PLL2 clock ready */
#define RCC_CR_PLL3ON                   (1 << 28)  /* Bit 28: System PLL3 clock enable */
#define RCC_CR_PLL3RDY                  (1 << 29)  /* Bit 29: System PLL3 clock ready */
                                                   /* Bits 30-31: Reserved */

/* Internal Clock Source Calibration Register */

/* HSICAL configuration */

#define RCC_ICSCR_HSICAL_SHIFT          (0ul)
#define RCC_ICSCR_HSICAL_MASK           (0xFFFul << RCC_ICSCR_HSICAL_SHIFT)
#define RCC_ICSCR_HSICAL                RCC_ICSCR_HSICAL_MASK  /* HSICAL[11:0] bits */

/* HSITRIM configuration */

#define RCC_ICSCR_HSITRIM_SHIFT         (12ul)
#define RCC_ICSCR_HSITRIM_MASK          (0x3Ful << RCC_ICSCR_HSITRIM_SHIFT)
#define RCC_ICSCR_HSITRIM               RCC_ICSCR_HSITRIM_MASK /* HSITRIM[5:0] bits */

/* CSICAL configuration */

#define RCC_ICSCR_CSICAL_SHIFT          (18ul)
#define RCC_ICSCR_CSICAL_MASK           (0xFFul << RCC_ICSCR_CSICAL_SHIFT)
#define RCC_ICSCR_CSICAL                RCC_ICSCR_CSICAL_MASK  /* CSICAL[7:0] bits */

/* CSITRIM configuration */

#define RCC_ICSCR_CSITRIM_SHIFT         (26ul)
#define RCC_ICSCR_CSITRIM_MASK          (0x1Ful << RCC_ICSCR_CSITRIM_SHIFT)
#define RCC_ICSCR_CSITRIM               RCC_ICSCR_CSITRIM_MASK /* CSITRIM[4:0] bits */

/* Clock Recovery RC Register */

/* HSI48CAL configuration */

#define RCC_CRRCR_HSI48CAL_SHIFT        (0ul)
#define RCC_CRRCR_HSI48CAL_MASK         (0x3FFul << RCC_CRRCR_HSI48CAL_SHIFT)
#define RCC_CRRCR_HSI48CAL               RCC_CRRCR_HSI48CAL_MASK   /* HSI48CAL[9:0] bits */

/* Clock Configuration Register (CFGR) */

#define RCC_CFGR_SW_SHIFT               (0)                        /* Bits 0-2: System clock Switch */
#define RCC_CFGR_SW_MASK                (7 << RCC_CFGR_SW_SHIFT)
#  define RCC_CFGR_SW_HSI               (0 << RCC_CFGR_SW_SHIFT)   /* 000: HSI selection as system clock */
#  define RCC_CFGR_SW_CSI               (1 << RCC_CFGR_SW_SHIFT)   /* 001: CSI selection as system clock */
#  define RCC_CFGR_SW_HSE               (2 << RCC_CFGR_SW_SHIFT)   /* 010: HSE selection as system clock */
#  define RCC_CFGR_SW_PLL1              (3 << RCC_CFGR_SW_SHIFT)   /* 011: PLL1 selection as system clock */
#define RCC_CFGR_SWS_SHIFT              (3)                        /* Bits 3-5: System Clock Switch Status */
#define RCC_CFGR_SWS_MASK               (7 << RCC_CFGR_SWS_SHIFT)
#  define RCC_CFGR_SWS_HSI              (0 << RCC_CFGR_SWS_SHIFT)  /* 000: HSI used as system clock */
#  define RCC_CFGR_SWS_CSI              (1 << RCC_CFGR_SWS_SHIFT)  /* 001: CSI used as system clock */
#  define RCC_CFGR_SWS_HSE              (2 << RCC_CFGR_SWS_SHIFT)  /* 010: HSE used as system clock */
#  define RCC_CFGR_SWS_PLL1             (3 << RCC_CFGR_SWS_SHIFT)  /* 011: PLL1 used as system clock */
#define RCC_CFGR_STOPWUCK               (1 << 6)                   /* Bit 6: Wake Up from stop and CSS backup clock selection */
#define RCC_CFGR_STOPKERWUCK            (1 << 7)                   /* Bit 7: Kernel Clock Selection after a Wake Up from STOP */
#define RCC_CFGR_RTCPRE_SHIFT           (8)                        /* Bits 8-13: HSE division factor for RTC clock */
#define RCC_CFGR_RTCPRE_MASK            (0x3f << RCC_CFGR_RTCPRE_SHIFT)
#  define RCC_CFGR_RTCPRE(x)            (((uint32_t)(x)) << RCC_CFGR_RTCPRE_SHIFT)
#define RCC_CFGR_HRTIMSEL               (1 << 14)                  /* Bit 14: HRTIM TImer clock prescaler */
#define RCC_CFGR_TIMPRE                 (1 << 15)                  /* Timers clocks prescaler */
#define RCC_CFGR_MCO1PRE_SHIFT          (18)                       /* Bits 18-21: MCO1 prescaler */
#define RCC_CFGR_MCO1PRE_MASK           (0xf << RCC_CFGR_MCO1PRE_SHIFT)
#  define RCC_CFGR_MCO1PRE(x)           (((uint32_t)(x)) << 18)
#define RCC_CFGR_MCO1_SHIFT             (22)                       /* Bits 22-24: Microcontroller Clock Output 1 */
#define RCC_CFGR_MCO1_MASK              (7 << RCC_CFGR_MCO1_SHIFT)
#  define RCC_CFGR_MCO1_HSI             (0 << RCC_CFGR_MCO1_SHIFT) /* 000: HSI clock selected */
#  define RCC_CFGR_MCO1_LSE             (1 << RCC_CFGR_MCO1_SHIFT) /* 001: LSE oscillator selected */
#  define RCC_CFGR_MCO1_HSE             (2 << RCC_CFGR_MCO1_SHIFT) /* 010: HSE oscillator clock selected */
#  define RCC_CFGR_MCO1_PLL1Q           (3 << RCC_CFGR_MCO1_SHIFT) /* 011: PLL clock selected */
#  define RCC_CFGR_MCO1_HSI48           (4 << RCC_CFGR_MCO1_SHIFT) /* 100: HSI48 clock selected */
#define RCC_CFGR_MCO2PRE_SHIFT          (25)                       /* Bits 25-28: MCO2 prescaler */
#define RCC_CFGR_MCO2PRE_MASK           (0xf << RCC_CFGR_MCO2PRE_SHIFT)
#  define RCC_CFGR_MCO2PRE(x)           (((uint32_t)(x)) << RCC_CFGR_MCO2PRE_SHIFT)
#define RCC_CFGR_MCO2_SHIFT             (29)                       /* Bits 29-31: Microcontroller Clock Output 2 */
#define RCC_CFGR_MCO2_MASK              (7 << RCC_CFGR_MCO2_SHIFT)
#  define RCC_CFGR_MCO2_SYS             (0 << RCC_CFGR_MCO2_SHIFT) /* 000: HSI clock selected */
#  define RCC_CFGR_MCO2_PLL2P           (1 << RCC_CFGR_MCO2_SHIFT) /* 001: PLL2 peripheral clock selected */
#  define RCC_CFGR_MCO2_HSE             (2 << RCC_CFGR_MCO2_SHIFT) /* 010: HSE oscillator clock selected */
#  define RCC_CFGR_MCO2_PLL1P           (3 << RCC_CFGR_MCO2_SHIFT) /* 011: PLL1 peripheral clock selected */
#  define RCC_CFGR_MCO2_CSI             (4 << RCC_CFGR_MCO2_SHIFT) /* 100: CSI clock selected */
#  define RCC_CFGR_MCO2_LSI             (5 << RCC_CFGR_MCO2_SHIFT) /* 101: LSI clock selected */

/* Bit definitions for RCC_D1CFGR */

#define RCC_D1CFGR_HPRE_SHIFT           (0)  /* Bits 0-3: D1 domain AHB prescaler */
#define RCC_D1CFGR_HPRE_MASK            (15 << RCC_D1CFGR_HPRE_SHIFT)
#  define RCC_D1CFGR_HPRE_SYSCLK        (0 << RCC_D1CFGR_HPRE_SHIFT)
#  define RCC_D1CFGR_HPRE_SYSCLKd2      (8 << RCC_D1CFGR_HPRE_SHIFT)
#  define RCC_D1CFGR_HPRE_SYSCLKd4      (9 << RCC_D1CFGR_HPRE_SHIFT)
#  define RCC_D1CFGR_HPRE_SYSCLKd8      (10 << RCC_D1CFGR_HPRE_SHIFT)
#  define RCC_D1CFGR_HPRE_SYSCLKd16     (11 << RCC_D1CFGR_HPRE_SHIFT)
#  define RCC_D1CFGR_HPRE_SYSCLKd64     (12 << RCC_D1CFGR_HPRE_SHIFT)
#  define RCC_D1CFGR_HPRE_SYSCLKd128    (13 << RCC_D1CFGR_HPRE_SHIFT)
#  define RCC_D1CFGR_HPRE_SYSCLKd256    (14 << RCC_D1CFGR_HPRE_SHIFT)
#  define RCC_D1CFGR_HPRE_SYSCLKd512    (15 << RCC_D1CFGR_HPRE_SHIFT)

#define RCC_D1CFGR_D1PPRE_SHIFT         (4)  /* Bits 4-6: D1 domain APB3 prescaler */
#define RCC_D1CFGR_D1PPRE_MASK          (7 << RCC_D1CFGR_D1PPRE_SHIFT)
#  define RCC_D1CFGR_D1PPRE_HCLK        (0 << RCC_D1CFGR_D1PPRE_SHIFT)
#  define RCC_D1CFGR_D1PPRE_HCLKd2      (4 << RCC_D1CFGR_D1PPRE_SHIFT)
#  define RCC_D1CFGR_D1PPRE_HCLKd4      (5 << RCC_D1CFGR_D1PPRE_SHIFT)
#  define RCC_D1CFGR_D1PPRE_HCLKd8      (6 << RCC_D1CFGR_D1PPRE_SHIFT)
#  define RCC_D1CFGR_D1PPRE_HCLKd16     (7 << RCC_D1CFGR_D1PPRE_SHIFT)
                                             /* Bit 7: Reserved */
#define RCC_D1CFGR_D1CPRE_SHIFT         (8)  /* Bits 8-11: D1 domain Core prescaler */
#define RCC_D1CFGR_D1CPRE_MASK          (15 << RCC_D1CFGR_D1CPRE_SHIFT)
#  define RCC_D1CFGR_D1CPRE_SYSCLK      (0 << RCC_D1CFGR_D1CPRE_SHIFT)
#  define RCC_D1CFGR_D1CPRE_SYSCLKd2    (8 << RCC_D1CFGR_D1CPRE_SHIFT)
#  define RCC_D1CFGR_D1CPRE_SYSCLKd4    (9 << RCC_D1CFGR_D1CPRE_SHIFT)
#  define RCC_D1CFGR_D1CPRE_SYSCLKd8    (10 << RCC_D1CFGR_D1CPRE_SHIFT)
#  define RCC_D1CFGR_D1CPRE_SYSCLKd16   (11 << RCC_D1CFGR_D1CPRE_SHIFT)
#  define RCC_D1CFGR_D1CPRE_SYSCLKd64   (12 << RCC_D1CFGR_D1CPRE_SHIFT)
#  define RCC_D1CFGR_D1CPRE_SYSCLKd128  (13 << RCC_D1CFGR_D1CPRE_SHIFT)
#  define RCC_D1CFGR_D1CPRE_SYSCLKd256  (14 << RCC_D1CFGR_D1CPRE_SHIFT)
#  define RCC_D1CFGR_D1CPRE_SYSCLKd512  (15 << RCC_D1CFGR_D1CPRE_SHIFT)
                                             /* Bits 12-31: Reserved */

/* Bit definitions for RCC_D2CFGR */

                                          /* Bits 0-3: Reserved */
#define RCC_D2CFGR_D2PPRE1_SHIFT     (4)  /* Bits 4-6: D2 domain APB1 prescaler */
#define RCC_D2CFGR_D2PPRE1_MASK      (7 << RCC_D2CFGR_D2PPRE1_SHIFT)
#  define RCC_D2CFGR_D2PPRE1_HCLK      (0 << RCC_D2CFGR_D2PPRE1_SHIFT)
#  define RCC_D2CFGR_D2PPRE1_HCLKd2    (4 << RCC_D2CFGR_D2PPRE1_SHIFT)
#  define RCC_D2CFGR_D2PPRE1_HCLKd4    (5 << RCC_D2CFGR_D2PPRE1_SHIFT)
#  define RCC_D2CFGR_D2PPRE1_HCLKd8    (6 << RCC_D2CFGR_D2PPRE1_SHIFT)
#  define RCC_D2CFGR_D2PPRE1_HCLKd16   (7 << RCC_D2CFGR_D2PPRE1_SHIFT)
                                          /* Bit 7: Reserved */
#define RCC_D2CFGR_D2PPRE2_SHIFT     (8)  /* Bits 8-10: D2 domain APB2 prescaler */
#define RCC_D2CFGR_D2PPRE2_MASK      (7 << RCC_D2CFGR_D2PPRE2_SHIFT)
#  define RCC_D2CFGR_D2PPRE2_HCLK      (0 << RCC_D2CFGR_D2PPRE2_SHIFT)
#  define RCC_D2CFGR_D2PPRE2_HCLKd2    (4 << RCC_D2CFGR_D2PPRE2_SHIFT)
#  define RCC_D2CFGR_D2PPRE2_HCLKd4    (5 << RCC_D2CFGR_D2PPRE2_SHIFT)
#  define RCC_D2CFGR_D2PPRE2_HCLKd8    (6 << RCC_D2CFGR_D2PPRE2_SHIFT)
#  define RCC_D2CFGR_D2PPRE2_HCLKd16   (7 << RCC_D2CFGR_D2PPRE2_SHIFT)
                                          /* Bits 11-31: Reserved */

/* Bit definitions for RCC_D3CFGR */

                                          /* Bits 0-3: Reserved */
#define RCC_D3CFGR_D3PPRE_SHIFT     (4)   /* Bits 4-6: D3 domain APB4 prescaler */
#define RCC_D3CFGR_D3PPRE_MASK      (7 << RCC_D3CFGR_D3PPRE_SHIFT)
#  define RCC_D3CFGR_D3PPRE_HCLK      (0 << RCC_D3CFGR_D3PPRE_SHIFT)
#  define RCC_D3CFGR_D3PPRE_HCLKd2    (4 << RCC_D3CFGR_D3PPRE_SHIFT)
#  define RCC_D3CFGR_D3PPRE_HCLKd4    (5 << RCC_D3CFGR_D3PPRE_SHIFT)
#  define RCC_D3CFGR_D3PPRE_HCLKd8    (6 << RCC_D3CFGR_D3PPRE_SHIFT)
#  define RCC_D3CFGR_D3PPRE_HCLKd16   (7 << RCC_D3CFGR_D3PPRE_SHIFT)
                                          /* Bits 7-31: Reserved */

/* Bit definitions for RCC_PLLCKSELR register */

#define RCC_PLLCKSELR_PLLSRC_SHIFT      (0)  /* Bit 0: */
#define RCC_PLLCKSELR_PLLSRC_MASK       (0x3 << RCC_PLLCKSELR_PLLSRC_SHIFT)
#define RCC_PLLCKSELR_PLLSRC            RCC_PLLCKSELR_PLLSRC_MASK

#define RCC_PLLCKSELR_PLLSRC_HSI        ((uint32_t)0x00000000)  /* HSI source clock selected */
#define RCC_PLLCKSELR_PLLSRC_CSI        ((uint32_t)0x00000001)  /* CSI source clock selected */
#define RCC_PLLCKSELR_PLLSRC_HSE        ((uint32_t)0x00000002)  /* HSE source clock selected */
#define RCC_PLLCKSELR_PLLSRC_NONE       ((uint32_t)0x00000003)  /* No source clock selected */

#define RCC_PLLCKSELR_DIVM1_SHIFT       (4ul)
#define RCC_PLLCKSELR_DIVM1(x)          ((x) << RCC_PLLCKSELR_DIVM1_SHIFT)  /* Prescaler for PLL1: 1 - 63, 0 = disabled */
#define RCC_PLLCKSELR_DIVM2_SHIFT       (12ul)
#define RCC_PLLCKSELR_DIVM2(x)          ((x) << RCC_PLLCKSELR_DIVM2_SHIFT)  /* Prescaler for PLL2: 1 - 63, 0 = disabled */
#define RCC_PLLCKSELR_DIVM3_SHIFT       (20ul)
#define RCC_PLLCKSELR_DIVM3(x)          ((x) << RCC_PLLCKSELR_DIVM3_SHIFT)  /* Prescaler for PLL3: 1 - 63, 0 = disabled */

/*  Bit definition for RCC_PLLCFGR register */

#define RCC_PLLCFGR_RESET               ((uint32_t)0x01FF0000)

#define RCC_PLLCFGR_PLL1FRACEN_SHIFT    (0ul)
#define RCC_PLLCFGR_PLL1FRACEN_MASK     (0x1ul << RCC_PLLCFGR_PLL1FRACEN_SHIFT)
#define RCC_PLLCFGR_PLL1FRACEN          RCC_PLLCFGR_PLL1FRACEN_MASK           /* Fractional latch enable */
#define RCC_PLLCFGR_PLL1VCOSEL_SHIFT    (1ul)
#define RCC_PLLCFGR_PLL1VCOSEL_MASK     (0x1ul << RCC_PLLCFGR_PLL1VCOSEL_SHIFT)
#define RCC_PLLCFGR_PLL1VCOSEL          RCC_PLLCFGR_PLL1VCOSEL_MASK           /* VCO freq range: 1 = Medium VCO range: 150 to 420 MHz, 0 = Wide VCO range: 192 to 836 MHz */
#define RCC_PLLCFGR_PLL1VCOSEL_WIDE     (0ul)                                 /* VCO freq range: Wide VCO range: 192 to 836 MHz, input clock >= 2 MHz */
#define RCC_PLLCFGR_PLL1VCOSEL_MEDIUM   RCC_PLLCFGR_PLL3VCOSEL                /* VCO freq range: Medium VCO range: 150 to 420 MHz, input clock <= 2 MHz */
#define RCC_PLLCFGR_PLL1RGE_SHIFT       (2ul)
#define RCC_PLLCFGR_PLL1RGE_MASK        (0x3ul << RCC_PLLCFGR_PLL1RGE_SHIFT)
#define RCC_PLLCFGR_PLL1RGE             RCC_PLLCFGR_PLL1RGE_MASK
#define RCC_PLLCFGR_PLL1RGE_1_2_MHZ     (0x0ul << RCC_PLLCFGR_PLL1RGE_SHIFT)  /* The PLL input clock range frequency is between 1 and 2 MHz */
#define RCC_PLLCFGR_PLL1RGE_2_4_MHZ     (0x1ul << RCC_PLLCFGR_PLL1RGE_SHIFT)  /* The PLL input clock range frequency is between 2 and 4 MHz */
#define RCC_PLLCFGR_PLL1RGE_4_8_MHZ     (0x2ul << RCC_PLLCFGR_PLL1RGE_SHIFT)  /* The PLL input clock range frequency is between 4 and 8 MHz */
#define RCC_PLLCFGR_PLL1RGE_8_16_MHZ    (0x3ul << RCC_PLLCFGR_PLL1RGE_SHIFT)  /* The PLL input clock range frequency is between 8 and 16 MHz */

#define RCC_PLLCFGR_PLL2FRACEN_SHIFT    (4ul)
#define RCC_PLLCFGR_PLL2FRACEN_MASK     (0x1ul << RCC_PLLCFGR_PLL2FRACEN_SHIFT)
#define RCC_PLLCFGR_PLL2FRACEN          RCC_PLLCFGR_PLL2FRACEN_MASK           /* Fractional latch enable */
#define RCC_PLLCFGR_PLL2VCOSEL_SHIFT    (5ul)
#define RCC_PLLCFGR_PLL2VCOSEL_MASK     (0x1ul << RCC_PLLCFGR_PLL2VCOSEL_SHIFT)
#define RCC_PLLCFGR_PLL2VCOSEL          RCC_PLLCFGR_PLL2VCOSEL_MASK           /* VCO freq range: 1 = Medium VCO range: 150 to 420 MHz, 0 = Wide VCO range: 192 to 836 MHz */
#define RCC_PLLCFGR_PLL2VCOSEL_WIDE     (0ul)                                 /* VCO freq range: Wide VCO range: 192 to 836 MHz, input clock >= 2 MHz */
#define RCC_PLLCFGR_PLL2VCOSEL_MEDIUM   RCC_PLLCFGR_PLL3VCOSEL                /* VCO freq range: Medium VCO range: 150 to 420 MHz, input clock <= 2 MHz */
#define RCC_PLLCFGR_PLL2RGE_SHIFT       (6ul)
#define RCC_PLLCFGR_PLL2RGE_MASK        (0x3ul << RCC_PLLCFGR_PLL2RGE_SHIFT)
#define RCC_PLLCFGR_PLL2RGE             RCC_PLLCFGR_PLL2RGE_MASK
#define RCC_PLLCFGR_PLL2RGE_1_2_MHZ     (0x0ul << RCC_PLLCFGR_PLL2RGE_SHIFT)  /* The PLL input clock range frequency is between 1 and 2 MHz */
#define RCC_PLLCFGR_PLL2RGE_2_4_MHZ     (0x1ul << RCC_PLLCFGR_PLL2RGE_SHIFT)  /* The PLL input clock range frequency is between 2 and 4 MHz */
#define RCC_PLLCFGR_PLL2RGE_4_8_MHZ     (0x2ul << RCC_PLLCFGR_PLL2RGE_SHIFT)  /* The PLL input clock range frequency is between 4 and 8 MHz */
#define RCC_PLLCFGR_PLL2RGE_8_16_MHZ    (0x3ul << RCC_PLLCFGR_PLL2RGE_SHIFT)  /* The PLL input clock range frequency is between 8 and 16 MHz */

#define RCC_PLLCFGR_PLL3FRACEN_SHIFT    (8ul)
#define RCC_PLLCFGR_PLL3FRACEN_MASK     (0x1ul << RCC_PLLCFGR_PLL3FRACEN_SHIFT)
#define RCC_PLLCFGR_PLL3FRACEN          RCC_PLLCFGR_PLL3FRACEN_MASK           /* Fractional latch enable */
#define RCC_PLLCFGR_PLL3VCOSEL_SHIFT    (9ul)
#define RCC_PLLCFGR_PLL3VCOSEL_MASK     (0x1ul << RCC_PLLCFGR_PLL3VCOSEL_SHIFT)
#define RCC_PLLCFGR_PLL3VCOSEL          RCC_PLLCFGR_PLL3VCOSEL_MASK           /* VCO freq range: 1 = Medium VCO range: 150 to 420 MHz, 0 = Wide VCO range: 192 to 836 MHz */
#define RCC_PLLCFGR_PLL3VCOSEL_WIDE     (0ul)                                 /* VCO freq range: Wide VCO range: 192 to 836 MHz, input clock >= 2 MHz */
#define RCC_PLLCFGR_PLL3VCOSEL_MEDIUM   RCC_PLLCFGR_PLL3VCOSEL                /* VCO freq range: Medium VCO range: 150 to 420 MHz, input clock <= 2 MHz */
#define RCC_PLLCFGR_PLL3RGE_SHIFT       (10ul)
#define RCC_PLLCFGR_PLL3RGE_MASK        (0x3ul << RCC_PLLCFGR_PLL3RGE_SHIFT)
#define RCC_PLLCFGR_PLL3RGE             RCC_PLLCFGR_PLL3RGE_MASK
#define RCC_PLLCFGR_PLL3RGE_1_2_MHZ     (0x0ul << RCC_PLLCFGR_PLL3RGE_SHIFT)  /* The PLL input clock range frequency is between 1 and 2 MHz */
#define RCC_PLLCFGR_PLL3RGE_2_4_MHZ     (0x1ul << RCC_PLLCFGR_PLL3RGE_SHIFT)  /* The PLL input clock range frequency is between 2 and 4 MHz */
#define RCC_PLLCFGR_PLL3RGE_4_8_MHZ     (0x2ul << RCC_PLLCFGR_PLL3RGE_SHIFT)  /* The PLL input clock range frequency is between 4 and 8 MHz */
#define RCC_PLLCFGR_PLL3RGE_8_16_MHZ    (0x3ul << RCC_PLLCFGR_PLL3RGE_SHIFT)  /* The PLL input clock range frequency is between 8 and 16 MHz */

#define RCC_PLLCFGR_DIVP1EN_SHIFT       (16ul)
#define RCC_PLLCFGR_DIVP1EN_MASK        (0x1ul << RCC_PLLCFGR_DIVP1EN_SHIFT)
#define RCC_PLLCFGR_DIVP1EN              RCC_PLLCFGR_DIVP1EN_MASK
#define RCC_PLLCFGR_DIVQ1EN_SHIFT       (17ul)
#define RCC_PLLCFGR_DIVQ1EN_MASK        (0x1ul << RCC_PLLCFGR_DIVQ1EN_SHIFT)
#define RCC_PLLCFGR_DIVQ1EN              RCC_PLLCFGR_DIVQ1EN_MASK
#define RCC_PLLCFGR_DIVR1EN_SHIFT       (18ul)
#define RCC_PLLCFGR_DIVR1EN_MASK        (0x1ul << RCC_PLLCFGR_DIVR1EN_SHIFT)
#define RCC_PLLCFGR_DIVR1EN              RCC_PLLCFGR_DIVR1EN_MASK

#define RCC_PLLCFGR_DIVP2EN_SHIFT       (19ul)
#define RCC_PLLCFGR_DIVP2EN_MASK        (0x1ul << RCC_PLLCFGR_DIVP2EN_SHIFT)
#define RCC_PLLCFGR_DIVP2EN              RCC_PLLCFGR_DIVP2EN_MASK
#define RCC_PLLCFGR_DIVQ2EN_SHIFT       (20ul)
#define RCC_PLLCFGR_DIVQ2EN_MASK        (0x1ul << RCC_PLLCFGR_DIVQ2EN_SHIFT)
#define RCC_PLLCFGR_DIVQ2EN              RCC_PLLCFGR_DIVQ2EN_MASK
#define RCC_PLLCFGR_DIVR2EN_SHIFT       (21ul)
#define RCC_PLLCFGR_DIVR2EN_MASK        (0x1ul << RCC_PLLCFGR_DIVR2EN_SHIFT)
#define RCC_PLLCFGR_DIVR2EN              RCC_PLLCFGR_DIVR2EN_MASK

#define RCC_PLLCFGR_DIVP3EN_SHIFT       (22ul)
#define RCC_PLLCFGR_DIVP3EN_MASK        (0x1ul << RCC_PLLCFGR_DIVP3EN_SHIFT)
#define RCC_PLLCFGR_DIVP3EN              RCC_PLLCFGR_DIVP3EN_MASK
#define RCC_PLLCFGR_DIVQ3EN_SHIFT       (23ul)
#define RCC_PLLCFGR_DIVQ3EN_MASK        (0x1ul << RCC_PLLCFGR_DIVQ3EN_SHIFT)
#define RCC_PLLCFGR_DIVQ3EN              RCC_PLLCFGR_DIVQ3EN_MASK
#define RCC_PLLCFGR_DIVR3EN_SHIFT       (24ul)
#define RCC_PLLCFGR_DIVR3EN_MASK        (0x1ul << RCC_PLLCFGR_DIVR3EN_SHIFT)
#define RCC_PLLCFGR_DIVR3EN              RCC_PLLCFGR_DIVR3EN_MASK

/* Bit definitions for RCC_PLL1DIVR register */

#define RCC_PLL1DIVR_N1_SHIFT           (0ul)
#define RCC_PLL1DIVR_N1(x)              (((x) - 1) << RCC_PLL1DIVR_N1_SHIFT)  /* Multiplication factor for VCO: 4 - 512 */
#define RCC_PLL1DIVR_P1_SHIFT           (9ul)
#define RCC_PLL1DIVR_P1(x)              (((x) - 1) << RCC_PLL1DIVR_P1_SHIFT)  /* DIVP division factor: 2 - 128, must be even */
#define RCC_PLL1DIVR_Q1_SHIFT           (16ul)
#define RCC_PLL1DIVR_Q1(x)              (((x) - 1) << RCC_PLL1DIVR_Q1_SHIFT)  /* DIVQ division factor: 2 - 128 */
#define RCC_PLL1DIVR_R1_SHIFT           (24ul)
#define RCC_PLL1DIVR_R1(x)              (((x) - 1) << RCC_PLL1DIVR_R1_SHIFT)  /* DIVR division factor: 2 - 128 */

/* Bit definitions for RCC_PLL1FRACR register */

#define RCC_PLL1FRACR_FRACN1_SHIFT      (3ul)
#define RCC_PLL1FRACR_FRACN1_MASK       (0x1FFFul << RCC_PLL1FRACR_FRACN1_SHIFT)
#define RCC_PLL1FRACR_FRACN1            RCC_PLL1FRACR_FRACN1_MASK

/* Bit definitions for RCC_PLL2DIVR register */

#define RCC_PLL2DIVR_N2_SHIFT           (0ul)
#define RCC_PLL2DIVR_N2(x)              (((x) - 1) << RCC_PLL2DIVR_N2_SHIFT)  /* Multiplication factor for VCO: 4 - 512 */
#define RCC_PLL2DIVR_P2_SHIFT           (9ul)
#define RCC_PLL2DIVR_P2(x)              (((x) - 1) << RCC_PLL2DIVR_P2_SHIFT)  /* DIVP division factor: 2 - 128 */
#define RCC_PLL2DIVR_Q2_SHIFT           (16ul)
#define RCC_PLL2DIVR_Q2(x)              (((x) - 1) << RCC_PLL2DIVR_Q2_SHIFT)  /* DIVQ division factor: 2 - 128 */
#define RCC_PLL2DIVR_R2_SHIFT           (24ul)
#define RCC_PLL2DIVR_R2(x)              (((x) - 1) << RCC_PLL2DIVR_R2_SHIFT)  /* DIVR division factor: 2 - 128 */

/* Bit definitions for RCC_PLL2FRACR register */

#define RCC_PLL2FRACR_FRACN2_SHIFT      (3ul)
#define RCC_PLL2FRACR_FRACN2_MASK       (0x1FFFul << RCC_PLL2FRACR_FRACN2_SHIFT)
#define RCC_PLL2FRACR_FRACN2             RCC_PLL2FRACR_FRACN2_MASK

/* Bit definitions for RCC_PLL3DIVR register */

#define RCC_PLL3DIVR_N3_SHIFT           (0ul)
#define RCC_PLL3DIVR_N3(x)              (((x) - 1) << RCC_PLL3DIVR_N3_SHIFT)  /* Multiplication factor for VCO: 4 - 512 */
#define RCC_PLL3DIVR_P3_SHIFT           (9ul)
#define RCC_PLL3DIVR_P3(x)              (((x) - 1) << RCC_PLL3DIVR_P3_SHIFT)  /* DIVP division factor: 2 - 128 */
#define RCC_PLL3DIVR_Q3_SHIFT           (16ul)
#define RCC_PLL3DIVR_Q3(x)              (((x) - 1) << RCC_PLL3DIVR_Q3_SHIFT)  /* DIVQ division factor: 2 - 128 */
#define RCC_PLL3DIVR_R3_SHIFT           (24ul)
#define RCC_PLL3DIVR_R3(x)              (((x) - 1) << RCC_PLL3DIVR_R3_SHIFT)  /* DIVR division factor: 2 - 128 */

/* Bit definitions for RCC_PLL3FRACR register */

#define RCC_PLL3FRACR_FRACN3_SHIFT      (3ul)
#define RCC_PLL3FRACR_FRACN3_MASK       (0x1FFFul << RCC_PLL3FRACR_FRACN3_SHIFT)
#define RCC_PLL3FRACR_FRACN3            RCC_PLL3FRACR_FRACN3_MASK

/* Bit definitions for RCC_D1CCIPR register */

#define RCC_D1CCIPR_FMCSEL_SHIFT        (0)  /* Bits 0-1: */
#define RCC_D1CCIPR_FMCSEL_MASK         (3 << RCC_D1CCIPR_FMCSEL_SHIFT)
#  define RCC_D1CCIPR_FMCSEL_HCLK       (0 << RCC_D1CCIPR_FMCSEL_SHIFT)
#  define RCC_D1CCIPR_FMCSEL_PLL1       (1 << RCC_D1CCIPR_FMCSEL_SHIFT)
#  define RCC_D1CCIPR_FMCSEL_PLL2       (2 << RCC_D1CCIPR_FMCSEL_SHIFT)
#  define RCC_D1CCIPR_FMCSEL_PER        (3 << RCC_D1CCIPR_FMCSEL_SHIFT)
                                             /* Bits 2-3: Reserved */
#define RCC_D1CCIPR_QSPISEL_SHIFT       (4)  /* Bits 4-5: */
#define RCC_D1CCIPR_QSPISEL_MASK        (3 << RCC_D1CCIPR_QSPISEL_SHIFT)
#  define RCC_D1CCIPR_QSPISEL_HCLK      (0 << RCC_D1CCIPR_QSPISEL_SHIFT)
#  define RCC_D1CCIPR_QSPISEL_PLL1      (1 << RCC_D1CCIPR_QSPISEL_SHIFT)
#  define RCC_D1CCIPR_QSPISEL_PLL2      (2 << RCC_D1CCIPR_QSPISEL_SHIFT)
#  define RCC_D1CCIPR_QSPISEL_PER       (3 << RCC_D1CCIPR_QSPISEL_SHIFT)
                                             /* Bits 6-15: Reserved */
#define RCC_D1CCIPR_SDMMC_SHIFT         (16) /* Bit 16: */
#define RCC_D1CCIPR_SDMMC_MASK          (1 << RCC_D1CCIPR_SDMMC_SHIFT)
#  define RCC_D1CCIPR_SDMMC_PLL1        (0 << RCC_D1CCIPR_SDMMC_SHIFT)
#  define RCC_D1CCIPR_SDMMC_PLL2        (1 << RCC_D1CCIPR_SDMMC_SHIFT)
                                             /* Bits 17-27: Reserved */
#define RCC_D1CCIPR_CKPERSEL_SHIFT      (28) /* Bits 28-29: */
#define RCC_D1CCIPR_CKPERSEL_MASK       (3 << RCC_D1CCIPR_CKPERSEL_SHIFT)
#  define RCC_D1CCIPR_CKPERSEL_HSI      (0 << RCC_D1CCIPR_CKPERSEL_SHIFT)
#  define RCC_D1CCIPR_CKPERSEL_CSI      (1 << RCC_D1CCIPR_CKPERSEL_SHIFT)
#  define RCC_D1CCIPR_CKPERSEL_HSE      (2 << RCC_D1CCIPR_CKPERSEL_SHIFT)
                                             /* Bits 30-31: Reserved */

/* Bit definitions for RCC_D2CCIP1R register */

#define RCC_D2CCIP1R_SAI1SEL_SHIFT      (0)    /* Bits 0-2 */
#define RCC_D2CCIP1R_SAI1SEL_MASK       (7 << RCC_D2CCIP1R_SAI1SEL_MASK)
#  define RCC_D2CCIP1R_SAI1SEL_PLL1     (0 << RCC_D2CCIP1R_SAI1SEL_SHIFT)
#  define RCC_D2CCIP1R_SAI1SEL_PLL2     (1 << RCC_D2CCIP1R_SAI1SEL_SHIFT)
#  define RCC_D2CCIP1R_SAI1SEL_PLL3     (2 << RCC_D2CCIP1R_SAI1SEL_SHIFT)
#  define RCC_D2CCIP1R_SAI1SEL_I2SCKIN  (3 << RCC_D2CCIP1R_SAI1SEL_SHIFT)
#  define RCC_D2CCIP1R_SAI1SEL_PER      (4 << RCC_D2CCIP1R_SAI1SEL_SHIFT)
                                               /* Bits 3-5: Reserved */
#define RCC_D2CCIP1R_SAI23SEL_SHIFT     (6)    /* Bits 6-8 */
#define RCC_D2CCIP1R_SAI23SEL_MASK      (7 << RCC_D2CCIP1R_SAI23SEL_SHIFT)
#  define RCC_D2CCIP1R_SAI23SEL_PLL1    (0 << RCC_D2CCIP1R_SAI23SEL_SHIFT)
#  define RCC_D2CCIP1R_SAI23SEL_PLL2    (1 << RCC_D2CCIP1R_SAI23SEL_SHIFT)
#  define RCC_D2CCIP1R_SAI23SEL_PLL3    (2 << RCC_D2CCIP1R_SAI23SEL_SHIFT)
#  define RCC_D2CCIP1R_SAI23SEL_I2SCKIN (3 << RCC_D2CCIP1R_SAI23SEL_SHIFT)
#  define RCC_D2CCIP1R_SAI23SEL_PER     (4 << RCC_D2CCIP1R_SAI23SEL_SHIFT)
                                               /* Bits 9-11: Reserved */
#define RCC_D2CCIP1R_SPI123SEL_SHIFT    (12)   /* Bits 12-14 */
#define RCC_D2CCIP1R_SPI123SEL_MASK     (7 << RCC_D2CCIP1R_SPI123SEL_SHIFT)
#  define RCC_D2CCIP1R_SPI123SEL_PLL1   (0 << RCC_D2CCIP1R_SPI123SEL_SHIFT)
#  define RCC_D2CCIP1R_SPI123SEL_PLL2   (1 << RCC_D2CCIP1R_SPI123SEL_SHIFT)
#  define RCC_D2CCIP1R_SPI123SEL_PLL3   (2 << RCC_D2CCIP1R_SPI123SEL_SHIFT)
#  define RCC_D2CCIP1R_SPI123SEL_I2SCKIN (3 << RCC_D2CCIP1R_SPI123SEL_SHIFT)
#  define RCC_D2CCIP1R_SPI123SEL_PER    (4 << RCC_D2CCIP1R_SPI123SEL_SHIFT)
                                               /* Bit 15: Reserved */
#define RCC_D2CCIP1R_SPI45SEL_SHIFT     (16)   /* Bits 16-18 */
#define RCC_D2CCIP1R_SPI45SEL_MASK      (7 << RCC_D2CCIP1R_SPI45SEL_SHIFT)
#  define RCC_D2CCIP1R_SPI45SEL_APB     (0 << RCC_D2CCIP1R_SPI45SEL_SHIFT)
#  define RCC_D2CCIP1R_SPI45SEL_PLL2    (1 << RCC_D2CCIP1R_SPI45SEL_SHIFT)
#  define RCC_D2CCIP1R_SPI45SEL_PLL3    (2 << RCC_D2CCIP1R_SPI45SEL_SHIFT)
#  define RCC_D2CCIP1R_SPI45SEL_HSI     (3 << RCC_D2CCIP1R_SPI45SEL_SHIFT)
#  define RCC_D2CCIP1R_SPI45SEL_CSI     (4 << RCC_D2CCIP1R_SPI45SEL_SHIFT)
#  define RCC_D2CCIP1R_SPI45SEL_HSE     (5 << RCC_D2CCIP1R_SPI45SEL_SHIFT)
                                               /* Bit 19: Reserved */
#define RCC_D2CCIP1R_SPDIFSEL_SHIFT     (20)   /* Bits 20-21 */
#define RCC_D2CCIP1R_SPDIFSEL_MASK      (3 << RCC_D2CCIP1R_SPDIFSEL_SHIFT)
#  define RCC_D2CCIP1R_SPDIFSEL_PLL1    (0 << RCC_D2CCIP1R_SPDIFSEL_SHIFT)
#  define RCC_D2CCIP1R_SPDIFSEL_PLL2    (1 << RCC_D2CCIP1R_SPDIFSEL_SHIFT)
#  define RCC_D2CCIP1R_SPDIFSEL_PLL3    (2 << RCC_D2CCIP1R_SPDIFSEL_SHIFT)
#  define RCC_D2CCIP1R_SPDIFSEL_HSI     (3 << RCC_D2CCIP1R_SPDIFSEL_SHIFT)
                                               /* Bits 22-23: Reserved */
#define RCC_D2CCIP1R_DFSDM1SEL_SHIFT    (24)   /* Bit 24 */
#define RCC_D2CCIP1R_DFSDM1SEL_MASK     (1 << RCC_D2CCIP1R_DFSDM1SEL_SHIFT)
#  define RCC_D2CCIP1R_DFSDM1SEL_PCLK2  (0 << RCC_D2CCIP1R_DFSDM1SEL_SHIFT)
#  define RCC_D2CCIP1R_DFSDM1SEL_SYSCLK (1 << RCC_D2CCIP1R_DFSDM1SEL_SHIFT)
                                               /* Bits 25-27: Reserved */
#define RCC_D2CCIP1R_FDCANSEL_SHIFT     (28)   /* Bits 28-29 */
#define RCC_D2CCIP1R_FDCANSEL_MASK      (3 << RCC_D2CCIP1R_FDCANSEL_SHIFT)
#  define RCC_D2CCIP1R_FDCANSEL_HSE     (0 << RCC_D2CCIP1R_FDCANSEL_SHIFT)
#  define RCC_D2CCIP1R_FDCANSEL_PLL1    (1 << RCC_D2CCIP1R_FDCANSEL_SHIFT)
#  define RCC_D2CCIP1R_FDCANSEL_PLL2    (2 << RCC_D2CCIP1R_FDCANSEL_SHIFT)
                                               /* Bit 30: Reserved */
#define RCC_D2CCIP1R_SWPSEL_SHIFT       (31)   /* Bit 31 */
#define RCC_D2CCIP1R_SWPSEL_MASK        (1 << RCC_D2CCIP1R_SWPSEL_SHIFT)
#  define RCC_D2CCIP1R_SWPSEL_PCLK      (0 << RCC_D2CCIP1R_SWPSEL_SHIFT)
#  define RCC_D2CCIP1R_SWPSEL_HSI       (1 << RCC_D2CCIP1R_SWPSEL_SHIFT)

/* Bit definitions for RCC_D2CCIP2R register */

#define RCC_D2CCIP2R_USART234578SEL_SHIFT  (0)  /* Bits 0-2 */
#  define RCC_D2CCIP2R_USART234578SEL_MASK (7 << RCC_D2CCIP2R_USART234578SEL_SHIFT)
#define RCC_D2CCIP2R_USART16SEL_SHIFT      (3)  /* Bits 3-5 */
#  define RCC_D2CCIP2R_USART16SEL_MASK     (7 << RCC_D2CCIP2R_USART16SEL_SHIFT)
                                                /* Bits 6-7: Reserved */
#define RCC_D2CCIP2R_RNGSEL_SHIFT          (8)  /* Bits 8-9 */
#  define RCC_D2CCIP2R_RNGSEL_MASK         (3 << RCC_D2CCIP2R_RNGSEL_SHIFT)
                                                /* Bits 10-11: Reserved */
#define RCC_D2CCIP2R_I2C123SEL_SHIFT       (12) /* Bits 12-13 */
#define RCC_D2CCIP2R_I2C123SEL_MASK        (3 << RCC_D2CCIP2R_I2C123SEL_SHIFT)
#   define RCC_D2CCIP2R_I2C123SEL_PCLK1    (0 << RCC_D2CCIP2R_I2C123SEL_SHIFT)
#   define RCC_D2CCIP2R_I2C123SEL_PLL3     (1 << RCC_D2CCIP2R_I2C123SEL_SHIFT)
#   define RCC_D2CCIP2R_I2C123SEL_HSI      (2 << RCC_D2CCIP2R_I2C123SEL_SHIFT)
#   define RCC_D2CCIP2R_I2C123SEL_CSI      (3 << RCC_D2CCIP2R_I2C123SEL_SHIFT)
                                                /* Bits 14-19: Reserved */
#define RCC_D2CCIP2R_USBSEL_SHIFT          (20) /* Bits 20-21 */
#  define RCC_D2CCIP2R_USBSEL_MASK         (3 << RCC_D2CCIP2R_USBSEL_SHIFT)
#  define RCC_D2CCIP2R_USBSEL_DISABLE      (0 << RCC_D2CCIP2R_USBSEL_SHIFT)
#  define RCC_D2CCIP2R_USBSEL_PLL1         (1 << RCC_D2CCIP2R_USBSEL_SHIFT)
#  define RCC_D2CCIP2R_USBSEL_PLL3         (2 << RCC_D2CCIP2R_USBSEL_SHIFT)
#  define RCC_D2CCIP2R_USBSEL_HSI48        (3 << RCC_D2CCIP2R_USBSEL_SHIFT)
#define RCC_D2CCIP2R_CECSEL_SHIFT          (22) /* Bits 22-23 */
#  define RCC_D2CCIP2R_CECSEL_MASK         (3 << RCC_D2CCIP2R_CECSEL_SHIFT)
                                                /* Bits 24-27: Reserved */
#define RCC_D2CCIP2R_LPTIM1SEL_SHIFT       (28) /* Bits 28-30 */
#  define RCC_D2CCIP2R_LPTIM1SEL_MASK      (3 << RCC_D2CCIP2R_LPTIM1SEL_SHIFT)
                                                /* Bit 31: Reserved */

/* Bit definitions for RCC_D3CCIPR register */

#define RCC_D3CCIPR_LPUART1SEL_SHIFT     (0)  /* Bits 0-2: LPUART1 kernel clock source selection */
#define RCC_D3CCIPR_LPUART1SEL_MASK      (7 << RCC_D3CCIPR_LPUART1SEL_SHIFT)
#  define RCC_D3CCIPR_LPUART1SEL_PCLK    (0 << RCC_D3CCIPR_LPUART1SEL_SHIFT)
#  define RCC_D3CCIPR_LPUART1SEL_PLL2    (1 << RCC_D3CCIPR_LPUART1SEL_SHIFT)
#  define RCC_D3CCIPR_LPUART1SEL_PLL3    (2 << RCC_D3CCIPR_LPUART1SEL_SHIFT)
#  define RCC_D3CCIPR_LPUART1SEL_HSI     (3 << RCC_D3CCIPR_LPUART1SEL_SHIFT)
#  define RCC_D3CCIPR_LPUART1SEL_CSI     (4 << RCC_D3CCIPR_LPUART1SEL_SHIFT)
#  define RCC_D3CCIPR_LPUART1SEL_LSE     (5 << RCC_D3CCIPR_LPUART1SEL_SHIFT)
                                              /* Bits 3-7: Reserved */
#define RCC_D3CCIPR_I2C4SEL_SHIFT        (8)  /* Bits 8-9: I2C4 kernel clock source selection */
#define RCC_D3CCIPR_I2C4SEL_MASK         (3 << RCC_D3CCIPR_I2C4SEL_SHIFT)
#   define RCC_D3CCIPR_I2C4SEL_PCLK4     (0 << RCC_D3CCIPR_I2C4SEL_SHIFT)
#   define RCC_D3CCIPR_I2C4SEL_PLL3      (1 << RCC_D3CCIPR_I2C4SEL_SHIFT)
#   define RCC_D3CCIPR_I2C4SEL_HSI       (2 << RCC_D3CCIPR_I2C4SEL_SHIFT)
#   define RCC_D3CCIPR_I2C4SEL_CSI       (3 << RCC_D3CCIPR_I2C4SEL_SHIFT)
#define RCC_D3CCIPR_LPTIM2SEL_SHIFT      (10) /* Bits 10-12: LPTIM2 kernel clock source selection */
#define RCC_D3CCIPR_LPTIM2SEL_MASK       (7 << RCC_D3CCIPR_LPTIM2SEL_SHIFT)
#   define RCC_D3CCIPR_LPTIM2SEL_PCLK4   (0 << RCC_D3CCIPR_LPTIM2SEL_SHIFT)
#   define RCC_D3CCIPR_LPTIM2SEL_PLL2    (1 << RCC_D3CCIPR_LPTIM2SEL_SHIFT)
#   define RCC_D3CCIPR_LPTIM2SEL_PLL3    (2 << RCC_D3CCIPR_LPTIM2SEL_SHIFT)
#   define RCC_D3CCIPR_LPTIM2SEL_LSE     (3 << RCC_D3CCIPR_LPTIM2SEL_SHIFT)
#   define RCC_D3CCIPR_LPTIM2SEL_LSI     (4 << RCC_D3CCIPR_LPTIM2SEL_SHIFT)
#   define RCC_D3CCIPR_LPTIM2SEL_PER     (5 << RCC_D3CCIPR_LPTIM2SEL_SHIFT)
#define RCC_D3CCIPR_LPTIM345SEL_SHIFT    (13) /* Bits 13-15: LPTIM3,4,5 kernel clock source selection */
#define RCC_D3CCIPR_LPTIM345SEL_MASK     (7 << RCC_D3CCIPR_LPTIM345SEL_SHIFT)
#   define RCC_D3CCIPR_LPTIM345SEL_PCLK4 (0 << RCC_D3CCIPR_LPTIM345SEL_SHIFT)
#   define RCC_D3CCIPR_LPTIM345SEL_PLL2  (1 << RCC_D3CCIPR_LPTIM345SEL_SHIFT)
#   define RCC_D3CCIPR_LPTIM345SEL_PLL3  (2 << RCC_D3CCIPR_LPTIM345SEL_SHIFT)
#   define RCC_D3CCIPR_LPTIM345SEL_LSE   (3 << RCC_D3CCIPR_LPTIM345SEL_SHIFT)
#   define RCC_D3CCIPR_LPTIM345SEL_LSI   (4 << RCC_D3CCIPR_LPTIM345SEL_SHIFT)
#   define RCC_D3CCIPR_LPTIM345SEL_PER   (5 << RCC_D3CCIPR_LPTIM345SEL_SHIFT)
#define RCC_D3CCIPR_ADCSEL_SHIFT         (16) /* Bits 16-17: SAR ADC kernel clock source selection */
#define RCC_D3CCIPR_ADCSEL_MASK          (3 << RCC_D3CCIPR_ADCSEL_SHIFT)
#   define RCC_D3CCIPR_ADCSEL_PLL2       (0 << RCC_D3CCIPR_ADCSEL_SHIFT)
#   define RCC_D3CCIPR_ADCSEL_PLL3       (1 << RCC_D3CCIPR_ADCSEL_SHIFT)
#   define RCC_D3CCIPR_ADCSEL_PER        (2 << RCC_D3CCIPR_ADCSEL_SHIFT)
                                              /* Bits 18-20: Reserved */
#define RCC_D3CCIPR_SAI4ASEL_SHIFT       (21) /* Bits 21-23: Sub-Block A of SAI4 kernel clock source selection */
#define RCC_D3CCIPR_SAI4ASEL_MASK        (7 << RCC_D3CCIPR_SAI4ASEL_SHIFT)
#   define RCC_D3CCIPR_SAI4ASEL_PLL1     (0 << RCC_D3CCIPR_SAI4ASEL_SHIFT)
#   define RCC_D3CCIPR_SAI4ASEL_PLL2     (1 << RCC_D3CCIPR_SAI4ASEL_SHIFT)
#   define RCC_D3CCIPR_SAI4ASEL_PLL3     (2 << RCC_D3CCIPR_SAI4ASEL_SHIFT)
#   define RCC_D3CCIPR_SAI4ASEL_I2CCKIN  (3 << RCC_D3CCIPR_SAI4ASEL_SHIFT)
#   define RCC_D3CCIPR_SAI4ASEL_PER      (4 << RCC_D3CCIPR_SAI4ASEL_SHIFT)
#define RCC_D3CCIPR_SAI4BSEL_SHIFT       (24) /* Bits 24-26: Sub-Block B of SAI4 kernel clock source selection */
#define RCC_D3CCIPR_SAI4BSEL_MASK        (7 << RCC_D3CCIPR_SAI4BSEL_SHIFT)
#   define RCC_D3CCIPR_SAI4BSEL_PLL1     (0 << RCC_D3CCIPR_SAI4BSEL_SHIFT)
#   define RCC_D3CCIPR_SAI4BSEL_PLL2     (1 << RCC_D3CCIPR_SAI4BSEL_SHIFT)
#   define RCC_D3CCIPR_SAI4BSEL_PLL3     (2 << RCC_D3CCIPR_SAI4BSEL_SHIFT)
#   define RCC_D3CCIPR_SAI4BSEL_I2CCKIN  (3 << RCC_D3CCIPR_SAI4BSEL_SHIFT)
#   define RCC_D3CCIPR_SAI4BSEL_PER      (4 << RCC_D3CCIPR_SAI4BSEL_SHIFT)
#define RCC_D3CCIPR_SPI6SEL_SHIFT        (28) /* Bits 28-30: SPI6 kernel clock source selection */
#define RCC_D3CCIPR_SPI6SEL_MASK         (7 << RCC_D3CCIPR_SPI6SEL_SHIFT)
#   define RCC_D3CCIPR_SPI6SEL_PCLK4     (0 << RCC_D3CCIPR_SPI6SEL_SHIFT)
#   define RCC_D3CCIPR_SPI6SEL_PLL2      (1 << RCC_D3CCIPR_SPI6SEL_SHIFT)
#   define RCC_D3CCIPR_SPI6SEL_PLL3      (2 << RCC_D3CCIPR_SPI6SEL_SHIFT)
#   define RCC_D3CCIPR_SPI6SEL_HSI       (3 << RCC_D3CCIPR_SPI6SEL_SHIFT)
#   define RCC_D3CCIPR_SPI6SEL_CSI       (4 << RCC_D3CCIPR_SPI6SEL_SHIFT)
#   define RCC_D3CCIPR_SPI6SEL_HSE       (5 << RCC_D3CCIPR_SPI6SEL_SHIFT)
                                              /* Bit 31: Reserved */

/* TODO: CIER */

/* TODO: CIFR */

/* TODO: CICR */

/* TODO: BDCR */

/* Bit definitions for RCC_CSR register */

#define RCC_CSR_LSION                   (1 << 0)  /* RCC CSR: LSION */
#define RCC_CSR_LSIRDY                  (1 << 1)  /* RCC CSR: LSIRDY */

/* AHB3 peripheral reset register */

#define RCC_AHB3RSTR_MDMARST            (1 << 0)  /* RCC AHB3RSTR: MDMARST */
#define RCC_AHB3RSTR_DMA2DRST           (1 << 4)  /* RCC AHB3RSTR: DMA2DRST */
#define RCC_AHB3RSTR_JPGDECRST          (1 << 5)  /* RCC AHB3RSTR: JPGDECRST */
#define RCC_AHB3RSTR_FMCRST             (1 << 12) /* RCC AHB3RSTR: FMCRST */
#define RCC_AHB3RSTR_QSPIRST            (1 << 14) /* RCC AHB3RSTR: QSPIRST */
#define RCC_AHB3RSTR_SDMMC1RST          (1 << 16) /* RCC AHB3RSTR: SDMMC1RST */
#define RCC_AHB3RSTR_CPURST             (1 << 31) /* RCC AHB3RSTR: CPURST */

/* AHB1 peripheral reset register */

#define RCC_AHB1RSTR_DMA1RST            (1 << 0)  /* RCC AHB1RSTR: DMA1RST */
#define RCC_AHB1RSTR_DMA2RST            (1 << 1)  /* RCC AHB1RSTR: DMA2RST */
#define RCC_AHB1RSTR_ADC12RST           (1 << 5)  /* RCC AHB1RSTR: ADC12RST */
                                                  /* Bits 6-14: Reserved */
#define RCC_AHB1RSTR_ETH1MACRST         (1 << 15) /* RCC AHB1RSTR: ETH1MACRST */
                                                  /* Bits 16-24: Reserved */
#define RCC_AHB1RSTR_OTGHSRST           (1 << 25) /* RCC AHB1RSTR: OTGHSRST */
                                                  /* Bit 26: Reserved */
#define RCC_AHB1RSTR_OTGFSRST           (1 << 27) /* RCC AHB1RSTR: OTGFSRST */
                                                  /* Bits 28-31: Reserved */

/* AHB2 peripheral reset register */

#define RCC_AHB2RSTR_CAMITFRST          (1 << 0)  /* RCC AHB2RSTR: CAMITFRST */
                                                  /* Bits 1-3: Reserved */
#define RCC_AHB2RSTR_CRYPTRST           (1 << 4)  /* RCC AHB2RSTR: CRYPTRST */
#define RCC_AHB2RSTR_HASHRST            (1 << 5)  /* RCC AHB2RSTR: HASHRST */
#define RCC_AHB2RSTR_RNGRST             (1 << 6)  /* RCC AHB2RSTR: RNGRST */
                                                  /* Bits 7-8: Reserved */
#define RCC_AHB2RSTR_SDMMC2RST          (1 << 9)  /* RCC AHB2RSTR: SDMMC2RST */
                                                  /* Bits 10-31: Reserved */

/* AHB4 peripheral reset register */

#define RCC_AHB4RSTR_GPIOARST           (1 << 0)  /* RCC AHB4RSTR: GPIOARST */
#define RCC_AHB4RSTR_GPIOBRST           (1 << 1)  /* RCC AHB4RSTR: GPIOBRST */
#define RCC_AHB4RSTR_GPIOCRST           (1 << 2)  /* RCC AHB4RSTR: GPIOCRST */
#define RCC_AHB4RSTR_GPIODRST           (1 << 3)  /* RCC AHB4RSTR: GPIODRST */
#define RCC_AHB4RSTR_GPIOERST           (1 << 4)  /* RCC AHB4RSTR: GPIOERST */
#define RCC_AHB4RSTR_GPIOFRST           (1 << 5)  /* RCC AHB4RSTR: GPIOFRST */
#define RCC_AHB4RSTR_GPIOGRST           (1 << 6)  /* RCC AHB4RSTR: GPIOGRST */
#define RCC_AHB4RSTR_GPIOHRST           (1 << 7)  /* RCC AHB4RSTR: GPIOHRST */
#define RCC_AHB4RSTR_GPIOIRST           (1 << 8)  /* RCC AHB4RSTR: GPIOIRST */
#define RCC_AHB4RSTR_GPIOJRST           (1 << 9)  /* RCC AHB4RSTR: GPIOJRST */
#define RCC_AHB4RSTR_GPIOKRST           (1 << 10) /* RCC AHB4RSTR: GPIOKRST */
                                                  /* Bits 11-18: Reserved */
#define RCC_AHB4RSTR_CRCRST             (1 << 19) /* RCC AHB4RSTR: CRCRST */
                                                  /* Bit 20: Reserved */
#define RCC_AHB4RSTR_BDMARST            (1 << 21) /* RCC AHB4RSTR: BDMARST */
                                                  /* Bits 22-23: Reserved */
#define RCC_AHB4RSTR_ADC3RST            (1 << 24) /* RCC AHB4RSTR: ADC3RST */
#define RCC_AHB4RSTR_HSEMRST            (1 << 25) /* RCC AHB4RSTR: HSEMRST */
                                                  /* Bits 26-31: Reserved */

/* APB3 peripheral reset register */

                                                  /* Bits 0-2: Reserved */
#define RCC_APB3RSTR_LTDCRST            (1 << 3)  /* RCC APB3RSTR: LTDCRST */
                                                  /* Bits 4-31: Reserved */

/* APB3 L peripheral reset register */

#define RCC_APB1LRSTR_TIM2RST           (1 << 0)  /* RCC APB1LRSTR: TIM2RST */
#define RCC_APB1LRSTR_TIM3RST           (1 << 1)  /* RCC APB1LRSTR: TIM3RST */
#define RCC_APB1LRSTR_TIM4RST           (1 << 2)  /* RCC APB1LRSTR: TIM4RST */
#define RCC_APB1LRSTR_TIM5RST           (1 << 3)  /* RCC APB1LRSTR: TIM5RST */
#define RCC_APB1LRSTR_TIM6RST           (1 << 4)  /* RCC APB1LRSTR: TIM6RST */
#define RCC_APB1LRSTR_TIM7RST           (1 << 5)  /* RCC APB1LRSTR: TIM7RST */
#define RCC_APB1LRSTR_TIM12RST          (1 << 6)  /* RCC APB1LRSTR: TIM12RST */
#define RCC_APB1LRSTR_TIM13RST          (1 << 7)  /* RCC APB1LRSTR: TIM13RST */
#define RCC_APB1LRSTR_TIM14RST          (1 << 8)  /* RCC APB1LRSTR: TIM14RST */
#define RCC_APB1LRSTR_LPTIM1RST         (1 << 9)  /* RCC APB1LRSTR: LPTIM1RST */
                                                  /* Bits 10-13: Reserved */
#define RCC_APB1LRSTR_SPI2RST           (1 << 14) /* RCC APB1LRSTR: SPI2RST */
#define RCC_APB1LRSTR_SPI3RST           (1 << 15) /* RCC APB1LRSTR: SPI3RST */
#define RCC_APB1LRSTR_SPDIFRXRST        (1 << 16) /* RCC APB1LRSTR: SPDIFRXRST */
#define RCC_APB1LRSTR_USART2RST         (1 << 17) /* RCC APB1LRSTR: USART2RST */
#define RCC_APB1LRSTR_USART3RST         (1 << 18) /* RCC APB1LRSTR: USART3RST */
#define RCC_APB1LRSTR_UART4RST          (1 << 19) /* RCC APB1LRSTR: UART4RST */
#define RCC_APB1LRSTR_UART5RST          (1 << 20) /* RCC APB1LRSTR: UART5RST */
#define RCC_APB1LRSTR_I2C1RST           (1 << 21) /* RCC APB1LRSTR: I2C1RST */
#define RCC_APB1LRSTR_I2C2RST           (1 << 22) /* RCC APB1LRSTR: I2C2RST */
#define RCC_APB1LRSTR_I2C3RST           (1 << 23) /* RCC APB1LRSTR: I2C3RST */
                                                  /* Bits 24-26: Reserved */
#define RCC_APB1LRSTR_HDMICECRST        (1 << 27) /* RCC APB1LRSTR: HDMICECRST */
                                                  /* Bit 28: Reserved */
#define RCC_APB1LRSTR_DAC1RST           (1 << 29) /* RCC APB1LRSTR: DAC1RST */
#define RCC_APB1LRSTR_UART7RST          (1 << 30) /* RCC APB1LRSTR: UART7RST */
#define RCC_APB1LRSTR_UART8RST          (1 << 31) /* RCC APB1LRSTR: UART8RST */

/* APB1 H peripheral reset register */

                                                  /* Bit 0: Reserved */
#define RCC_APB1HRSTR_CRSRST            (1 << 1)  /* RCC APB1HRSTR: CRSRST */
#define RCC_APB1HRSTR_SWPRST            (1 << 2)  /* RCC APB1HRSTR: SWPRST */

/* Bit 3: Reserved */

#define RCC_APB1HRSTR_OPAMPRST          (1 << 4)  /* RCC APB1HRSTR: OPAMPRST*/
#define RCC_APB1HRSTR_MDIOSRST          (1 << 5)  /* RCC APB1HRSTR: MDIOSRST */
                                                  /* Bits 6-7: Reserved */
#define RCC_APB1HRSTR_FDCANRST          (1 << 8)  /* RCC APB1HRSTR: FDCANRST*/

/* Bits 9-31: Reserved */

/* APB2 peripheral reset register */

#define RCC_APB2RSTR_TIM1RST            (1 << 0)  /* RCC APB2RSTR: TIM1RST */
#define RCC_APB2RSTR_TIM8RST            (1 << 1)  /* RCC APB2RSTR: TIM8RST */
#define RCC_APB2RSTR_USART1RST          (1 << 4)  /* RCC APB2RSTR: USART1RST */
#define RCC_APB2RSTR_USART6RST          (1 << 5)  /* RCC APB2RSTR: USART6RST */
#define RCC_APB2RSTR_SPI1RST            (1 << 12) /* RCC APB2RSTR: SPI1RST */
#define RCC_APB2RSTR_SPI4RST            (1 << 13) /* RCC APB2RSTR: SPI4RST */
#define RCC_APB2RSTR_TIM15RST           (1 << 16) /* RCC APB2RSTR: TIM15RST */
#define RCC_APB2RSTR_TIM16RST           (1 << 17) /* RCC APB2RSTR: TIM16RST */
#define RCC_APB2RSTR_TIM17RST           (1 << 18) /* RCC APB2RSTR: TIM17RST */
#define RCC_APB2RSTR_SPI5RST            (1 << 20) /* RCC APB2RSTR: SPI5RST */
#define RCC_APB2RSTR_SAI1RST            (1 << 22) /* RCC APB2RSTR: SAI1RST */
#define RCC_APB2RSTR_SAI2RST            (1 << 23) /* RCC APB2RSTR: SAI2RST */
#define RCC_APB2RSTR_SAI3RST            (1 << 24) /* RCC APB2RSTR: SAI3RST */
#define RCC_APB2RSTR_DFSDM1RST          (1 << 28) /* RCC APB2RSTR: DFSDM1RST */
#define RCC_APB2RSTR_HRTIMRST           (1 << 29) /* RCC APB2RSTR: HRTIMRST */

/* APB4 peripheral reset register */

#define RCC_APB4RSTR_SYSCFGRST          (1 << 1)  /* RCC APB4RSTR: SYSCFGRST */
#define RCC_APB4RSTR_LPUART1RST         (1 << 3)  /* RCC APB4RSTR: LPUART1RST */
#define RCC_APB4RSTR_SPI6RST            (1 << 5)  /* RCC APB4RSTR: SPI6RST */
#define RCC_APB4RSTR_I2C4RST            (1 << 7)  /* RCC APB4RSTR: I2C4RST */
#define RCC_APB4RSTR_LPTIM2RST          (1 << 9)  /* RCC APB4RSTR: LPTIM2RST */
#define RCC_APB4RSTR_LPTIM3RST          (1 << 10) /* RCC APB4RSTR: LPTIM3RST */
#define RCC_APB4RSTR_LPTIM4RST          (1 << 11) /* RCC APB4RSTR: LPTIM4RST */
#define RCC_APB4RSTR_LPTIM5RST          (1 << 12) /* RCC APB4RSTR: LPTIM5RST */
#define RCC_APB4RSTR_COMP12RST          (1 << 14) /* RCC APB4RSTR: COMP12RST */
#define RCC_APB4RSTR_VREFRST            (1 << 15) /* RCC APB4RSTR: VREFRST */
#define RCC_APB4RSTR_SAI4RST            (1 << 21) /* RCC APB4RSTR: SAI4RST */

/* RCC Global Control register */

#define RCC_GCR_WW1RSC                  (1 << 0)  /* Bit 0: WWDG1 reset scope control */

/* TODO: D3 Autonomous mode register */

/* RCC Reset Status register */

                                                  /* Bits 0-15: Reserved */
#define RCC_RSR_RMVF                    (1 << 16) /* Bit 16: Remove reset flag */
#define RCC_RSR_CPURSTF                 (1 << 17) /* Bit 17: CPU reset flag */
                                                  /* Bit 18: Reserved */
#define RCC_RSR_D1RSTF                  (1 << 19) /* Bit 19: D1 domain power switch reset flag */
#define RCC_RSR_D2RSTF                  (1 << 20) /* Bit 20: D2 domain power switch reset flag */
#define RCC_RSR_BORRSTF                 (1 << 21) /* Bit 21: BOR reset flag */
#define RCC_RSR_PINRSTF                 (1 << 22) /* Bit 22: Pin reset flag */
#define RCC_RSR_PORRSTF                 (1 << 23) /* Bit 23: POR/PDR reset flag */
#define RCC_RSR_SFTRSTF                 (1 << 24) /* Bit 24: System reset from CPU flag */
                                                  /* Bit 25: Reserved */
#define RCC_RSR_IWDG1RSTF               (1 << 26) /* Bit 26: Independent watchdog reset flag */
                                                  /* Bit 27: Reserved */
#define RCC_RSR_WWDG1RSTF               (1 << 28) /* Bit 28: Window watchdog reset flag */
                                                  /* Bit 29: Reserved */
#define RCC_RSR_LPWRRSTF                (1 << 30) /* Bit 30: Reset due to illegal D1 DStandby or CPU Cstop flag */
                                                  /* Bit 31: Reserved */

/* AHB3 Peripheral Clock enable register */

#define RCC_AHB3ENR_MDMAEN              (1 << 0)  /* RCC AHB3ENR: MDMAEN */
#define RCC_AHB3ENR_DMA2DEN             (1 << 4)  /* RCC AHB3ENR: DMA2DEN */
#define RCC_AHB3ENR_JPGDECEN            (1 << 5)  /* RCC AHB3ENR: JPGDECEN */
#define RCC_AHB3ENR_FMCEN               (1 << 12) /* RCC AHB3ENR: FMCEN */
#define RCC_AHB3ENR_QSPIEN              (1 << 14) /* RCC AHB3ENR: QSPIEN */
#define RCC_AHB3ENR_SDMMC1EN            (1 << 16) /* RCC AHB3ENR: SDMMC1EN */

/* AHB1 Peripheral Clock enable register */

#define RCC_AHB1ENR_DMA1EN              (1 << 0)  /* RCC AHB1ENR: DMA1EN */
#define RCC_AHB1ENR_DMA2EN              (1 << 1)  /* RCC AHB1ENR: DMA2EN */
#define RCC_AHB1ENR_ADC12EN             (1 << 5)  /* RCC AHB1ENR: ADC12EN */
#define RCC_AHB1ENR_ETH1MACEN           (1 << 15) /* RCC AHB1ENR: ETH1MACEN */
#define RCC_AHB1ENR_ETH1TXEN            (1 << 16) /* RCC AHB1ENR: ETH1TXEN */
#define RCC_AHB1ENR_ETH1RXEN            (1 << 17) /* RCC AHB1ENR: ETH1RXEN */
#define RCC_AHB1ENR_OTGHSEN             (1 << 25) /* RCC AHB1ENR: OTGHSEN */
#define RCC_AHB1ENR_OTGHSULPIEN         (1 << 26) /* RCC AHB1ENR: OTGHSULPIEN */
#define RCC_AHB1ENR_OTGFSEN             (1 << 27) /* RCC AHB1ENR: OTGFSEN */

/* AHB2 Peripheral Clock enable register */

#define RCC_AHB2ENR_CAMITFEN            (1 << 0)  /* RCC AHB2ENR: CAMITFEN */
#define RCC_AHB2ENR_CRYPTEN             (1 << 4)  /* RCC AHB2ENR: CRYPTEN */
#define RCC_AHB2ENR_HASHEN              (1 << 5)  /* RCC AHB2ENR: HASHEN */
#define RCC_AHB2ENR_RNGEN               (1 << 6)  /* RCC AHB2ENR: RNGEN */
#define RCC_AHB2ENR_SDMMC2EN            (1 << 9)  /* RCC AHB2ENR: SDMMC2EN */
#define RCC_AHB2ENR_SRAM1EN             (1 << 29) /* RCC AHB2ENR: SRAM1EN */
#define RCC_AHB2ENR_SRAM2EN             (1 << 30) /* RCC AHB2ENR: SRAM2EN */
#define RCC_AHB2ENR_SRAM3EN             (1 << 31) /* RCC AHB2ENR: SRAM3EN */

/* AHB4 Peripheral Clock enable register */

#define RCC_AHB4ENR_GPIOAEN             (1 << 0)  /* RCC AHB4ENR: GPIOAEN */
#define RCC_AHB4ENR_GPIOBEN             (1 << 1)  /* RCC AHB4ENR: GPIOBEN */
#define RCC_AHB4ENR_GPIOCEN             (1 << 2)  /* RCC AHB4ENR: GPIOCEN */
#define RCC_AHB4ENR_GPIODEN             (1 << 3)  /* RCC AHB4ENR: GPIODEN */
#define RCC_AHB4ENR_GPIOEEN             (1 << 4)  /* RCC AHB4ENR: GPIOEEN */
#define RCC_AHB4ENR_GPIOFEN             (1 << 5)  /* RCC AHB4ENR: GPIOFEN */
#define RCC_AHB4ENR_GPIOGEN             (1 << 6)  /* RCC AHB4ENR: GPIOGEN */
#define RCC_AHB4ENR_GPIOHEN             (1 << 7)  /* RCC AHB4ENR: GPIOHEN */
#define RCC_AHB4ENR_GPIOIEN             (1 << 8)  /* RCC AHB4ENR: GPIOIEN */
#define RCC_AHB4ENR_GPIOJEN             (1 << 9)  /* RCC AHB4ENR: GPIOJEN */
#define RCC_AHB4ENR_GPIOKEN             (1 << 10) /* RCC AHB4ENR: GPIOKEN */
#define RCC_AHB4ENR_CRCEN               (1 << 19) /* RCC AHB4ENR: CRCEN */
#define RCC_AHB4ENR_BDMAEN              (1 << 21) /* RCC AHB4ENR: BDMAEN and DMAMUX2 */
#define RCC_AHB4ENR_ADC3EN              (1 << 24) /* RCC AHB4ENR: ADC3EN */
#define RCC_AHB4ENR_HSEMEN              (1 << 25) /* RCC AHB4ENR: HSEMEN */
#define RCC_AHB4ENR_BKPSRAMEN           (1 << 28) /* RCC AHB4ENR: BKPRAMEN */

/* APB3 Peripheral Clock enable register */

#define RCC_APB3ENR_LTDCEN              (1 << 3)  /* RCC APB3ENR: LTDCEN */
#define RCC_APB3ENR_WWDG1EN             (1 << 6)  /* RCC APB3ENR: WWDG1EN  */

/* APB1 L Peripheral Clock enable register */

#define RCC_APB1LENR_TIM2EN             (1 << 0)  /* RCC APB1LENR: TIM2EN */
#define RCC_APB1LENR_TIM3EN             (1 << 1)  /* RCC APB1LENR: TIM3EN */
#define RCC_APB1LENR_TIM4EN             (1 << 2)  /* RCC APB1LENR: TIM4EN */
#define RCC_APB1LENR_TIM5EN             (1 << 3)  /* RCC APB1LENR: TIM5EN */
#define RCC_APB1LENR_TIM6EN             (1 << 4)  /* RCC APB1LENR: TIM6EN */
#define RCC_APB1LENR_TIM7EN             (1 << 5)  /* RCC APB1LENR: TIM7EN */
#define RCC_APB1LENR_TIM12EN            (1 << 6)  /* RCC APB1LENR: TIM12EN */
#define RCC_APB1LENR_TIM13EN            (1 << 7)  /* RCC APB1LENR: TIM13EN */
#define RCC_APB1LENR_TIM14EN            (1 << 8)  /* RCC APB1LENR: TIM14EN */
#define RCC_APB1LENR_LPTIM1EN           (1 << 9)  /* RCC APB1LENR: LPTIM1EN */
                                                  /* Bits 10-13: Reserved */
#define RCC_APB1LENR_SPI2EN             (1 << 14) /* RCC APB1LENR: SPI2EN */
#define RCC_APB1LENR_SPI3EN             (1 << 15) /* RCC APB1LENR: SPI3EN */
#define RCC_APB1LENR_SPDIFRXEN          (1 << 16) /* RCC APB1LENR: SPDIFRXEN */
#define RCC_APB1LENR_USART2EN           (1 << 17) /* RCC APB1LENR: USART2EN */
#define RCC_APB1LENR_USART3EN           (1 << 18) /* RCC APB1LENR: USART3EN */
#define RCC_APB1LENR_UART4EN            (1 << 19) /* RCC APB1LENR: UART4EN */
#define RCC_APB1LENR_UART5EN            (1 << 20) /* RCC APB1LENR: UART5EN */
#define RCC_APB1LENR_I2C1EN             (1 << 21) /* RCC APB1LENR: I2C1EN */
#define RCC_APB1LENR_I2C2EN             (1 << 22) /* RCC APB1LENR: I2C2EN */
#define RCC_APB1LENR_I2C3EN             (1 << 23) /* RCC APB1LENR: I2C3EN */
                                                  /* Bits 24-25: Reserved */
#define RCC_APB1LENR_HDMICECEN          (1 << 27) /* RCC APB1LENR: HDMICECEN */
                                                  /* Bit 28: Reserved */
#define RCC_APB1LENR_DAC1EN             (1 << 29) /* RCC APB1LENR: DAC1EN */
#define RCC_APB1LENR_UART7EN            (1 << 30) /* RCC APB1LENR: UART7EN */
#define RCC_APB1LENR_UART8EN            (1 << 31) /* RCC APB1LENR: UART8EN */

/* APB1 H Peripheral Clock enable register */

                                                  /* Bit 0: Reserved */
#define RCC_APB1HENR_CRSEN              (1 << 1)  /* RCC APB1HENR: CRSEN */
#define RCC_APB1HENR_SWPEN              (1 << 2)  /* RCC APB1HENR: SWPEN */
                                                  /* Bit 3: Reserved */
#define RCC_APB1HENR_OPAMPEN            (1 << 4)  /* RCC APB1HENR: OPAMPEN */
#define RCC_APB1HENR_MDIOSEN            (1 << 5)  /* RCC APB1HENR: MDIOSEN */
                                                  /* Bits 6-7: Reserved */
#define RCC_APB1HENR_FDCANEN            (1 << 8)  /* RCC APB1HENR: FDCANEN */
                                                  /* Bits 9-31: Reserved */

/* APB2 Peripheral Clock enable register */

#define RCC_APB2ENR_TIM1EN              (1 << 0)  /* Bit 0: RCC APB2ENR: TIM1EN */
#define RCC_APB2ENR_TIM8EN              (1 << 1)  /* Bit 1: RCC APB2ENR: TIM8EN  */
                                                  /* Bits 2-3: Reserved */
#define RCC_APB2ENR_USART1EN            (1 << 4)  /* Bit 4: RCC APB2ENR: USART1EN */
#define RCC_APB2ENR_USART6EN            (1 << 5)  /* Bit 5: RCC APB2ENR: USART6EN */
                                                  /* Bits 6-11: Reserved */
#define RCC_APB2ENR_SPI1EN              (1 << 12) /* Bit 12: RCC APB2ENR: SPI1EN */
#define RCC_APB2ENR_SPI4EN              (1 << 13) /* Bit 13: RCC APB2ENR: SPI4EN  */
                                                  /* Bits 14-15: Reserved */
#define RCC_APB2ENR_TIM15EN             (1 << 16) /* Bit 16: RCC APB2ENR: TIM15EN */
#define RCC_APB2ENR_TIM16EN             (1 << 17) /* Bit 17: RCC APB2ENR: TIM16EN */
#define RCC_APB2ENR_TIM17EN             (1 << 18) /* Bit 18: RCC APB2ENR: TIM17EN  */
#define RCC_APB2ENR_SPI5EN              (1 << 20) /* Bit 20: RCC APB2ENR: SPI5EN */
                                                  /* Bit 21: Reserved */
#define RCC_APB2ENR_SAI1EN              (1 << 22) /* Bit 22: RCC APB2ENR: SAI1EN */
#define RCC_APB2ENR_SAI2EN              (1 << 23) /* Bit 23: RCC APB2ENR: SAI2EN */
#define RCC_APB2ENR_SAI3EN              (1 << 24) /* Bit 24: RCC APB2ENR: SAI3EN */
                                                  /* Bits 25-27: Reserved */
#define RCC_APB2ENR_DFSDM1EN            (1 << 28) /* Bit 28: RCC APB2ENR: DFSDM1EN */
#define RCC_APB2ENR_HRTIMEN             (1 << 29) /* Bit 29: RCC APB2ENR: HRTIMEN */
                                                  /* Bits 30-31: Reserved */

/* APB4 Peripheral Clock enable register */

                                                  /* Bit 0: Reserved */
#define RCC_APB4ENR_SYSCFGEN            (1 << 1)  /* Bit 1: RCC APB4ENR: SYSCFGEN */
                                                  /* Bit 2: Reserved */
#define RCC_APB4ENR_LPUART1EN           (1 << 3)  /* Bit 3: RCC APB4ENR: LPUART1EN */
                                                  /* Bit 4: Reserved */
#define RCC_APB4ENR_SPI6EN              (1 << 5)  /* Bit 5: RCC APB4ENR: SPI6EN */
                                                  /* Bit 6: Reserved */
#define RCC_APB4ENR_I2C4EN              (1 << 7)  /* Bit 7: RCC APB4ENR: I2C4EN */
                                                  /* Bit 8: Reserved */
#define RCC_APB4ENR_LPTIM2EN            (1 << 9)  /* Bit 9: RCC APB4ENR: LPTIM2EN */
#define RCC_APB4ENR_LPTIM3EN            (1 << 10) /* Bit 10: RCC APB4ENR: LPTIM3EN */
#define RCC_APB4ENR_LPTIM4EN            (1 << 11) /* Bit 11: RCC APB4ENR: LPTIM4EN */
#define RCC_APB4ENR_LPTIM5EN            (1 << 12) /* Bit 12: RCC APB4ENR: LPTIM5EN */
                                                  /* Bit 13: Reserved */
#define RCC_APB4ENR_COMP12EN            (1 << 14) /* Bit 14: RCC APB4ENR: COMP12EN */
#define RCC_APB4ENR_VREFEN              (1 << 15) /* Bit 15: RCC APB4ENR: VREFEN */
#define RCC_APB4ENR_RTCAPBEN            (1 << 16) /* Bit 16: RCC APB4ENR: RTCAPBEN */
                                                  /* Bits 17-20: Reserved */
#define RCC_APB4ENR_SAI4EN              (1 << 21) /* Bit 21: RCC APB4ENR: SAI4EN */
                                                  /* Bit2 22-31: Reserved */

/* AHB3 low power mode peripheral clock enable register */

#define RCC_AHB3LPENR_MDMALPEN          (1 << 0)  /* RCC AHB3LPENR: MDMALPEN */
#define RCC_AHB3LPENR_DMA2DLPEN         (1 << 4)  /* RCC AHB3LPENR: DMA2DLPEN */
#define RCC_AHB3LPENR_JPGDECLPEN        (1 << 5)  /* RCC AHB3LPENR: JPGDECLPEN */
#define RCC_AHB3LPENR_FLITFLPEN         (1 << 8)  /* RCC AHB3LPENR: FLITFLPEN */
#define RCC_AHB3LPENR_FMCLPEN           (1 << 12) /* RCC AHB3LPENR: FMCLPEN */
#define RCC_AHB3LPENR_QSPILPEN          (1 << 14) /* RCC AHB3LPENR: QSPILPEN */
#define RCC_AHB3LPENR_SDMMC1LPEN        (1 << 16) /* RCC AHB3LPENR: SDMMC1LPEN */
#define RCC_AHB3LPENR_D1DTCM1LPEN       (1 << 28) /* RCC AHB3LPENR: D1DTCM1LPEN */
#define RCC_AHB3LPENR_DTCM2LPEN         (1 << 29) /* RCC AHB3LPENR: DTCM2LPEN */
#define RCC_AHB3LPENR_ITCMLPEN          (1 << 30) /* RCC AHB3LPENR: ITCMLPEN */
#define RCC_AHB3LPENR_AXISRAMLPEN       (1 << 31) /* RCC AHB3LPENR: AXISRAMLPEN */

/* AHB1 low power mode peripheral clock enable register */

#define RCC_AHB1LPENR_DMA1LPEN          (1 << 0)  /* RCC AHB1LPENR: DMA1LPEN */
#define RCC_AHB1LPENR_DMA2LPEN          (1 << 1)  /* RCC AHB1LPENR: DMA2LPEN */
#define RCC_AHB1LPENR_ADC12LPEN         (1 << 5)  /* RCC AHB1LPENR: ADC12LPEN */
#define RCC_AHB1LPENR_ETH1MACLPEN       (1 << 15) /* RCC AHB1LPENR: ETH1MACLPEN */
#define RCC_AHB1LPENR_ETH1TXLPEN        (1 << 16) /* RCC AHB1LPENR: ETH1TXLPEN */
#define RCC_AHB1LPENR_ETH1RXLPEN        (1 << 17) /* RCC AHB1LPENR: ETH1RXLPEN */
#define RCC_AHB1LPENR_OTGHSLPEN         (1 << 25) /* RCC AHB1LPENR: OTGHSLPEN */
#define RCC_AHB1LPENR_OTGHSULPILPEN     (1 << 26) /* RCC AHB1LPENR: OTGHSULPILPEN */
#define RCC_AHB1LPENR_OTGFSLPEN         (1 << 27) /* RCC AHB1LPENR: OTGFSLPEN */

/* AHB2 low power mode peripheral clock enable register */

#define RCC_AHB2LPENR_CAMITFLPEN        (1 << 0)  /* RCC AHB2LPENR: CAMITFLPEN */
#define RCC_AHB2LPENR_CRYPTLPEN         (1 << 4)  /* RCC AHB2LPENR: CRYPTLPEN */
#define RCC_AHB2LPENR_HASHLPEN          (1 << 5)  /* RCC AHB2LPENR: HASHLPEN */
#define RCC_AHB2LPENR_SDMMC2LPEN        (1 << 6)  /* RCC AHB2LPENR: SDMMC2LPEN */
#define RCC_AHB2LPENR_RNGLPEN           (1 << 9)  /* RCC AHB2LPENR: RNGLPEN */
#define RCC_AHB2LPENR_SRAM1LPEN         (1 << 29) /* RCC AHB2LPENR: SRAM1LPEN */
#define RCC_AHB2LPENR_SRAM2LPEN         (1 << 30) /* RCC AHB2LPENR: SRAM2LPEN */
#define RCC_AHB2LPENR_SRAM3LPEN         (1 << 31) /* RCC AHB2LPENR: SRAM3LPEN */

/* AHB4 low power mode peripheral clock enable register */

#define RCC_AHB4LPENR_GPIOALPEN         (1 << 0)  /* RCC AHB4LPENR: GPIOALPEN */
#define RCC_AHB4LPENR_GPIOBLPEN         (1 << 1)  /* RCC AHB4LPENR: GPIOBLPEN */
#define RCC_AHB4LPENR_GPIOCLPEN         (1 << 2)  /* RCC AHB4LPENR: GPIOCLPEN */
#define RCC_AHB4LPENR_GPIODLPEN         (1 << 3)  /* RCC AHB4LPENR: GPIODLPEN */
#define RCC_AHB4LPENR_GPIOELPEN         (1 << 4)  /* RCC AHB4LPENR: GPIOELPEN */
#define RCC_AHB4LPENR_GPIOFLPEN         (1 << 5)  /* RCC AHB4LPENR: GPIOFLPEN */
#define RCC_AHB4LPENR_GPIOGLPEN         (1 << 6)  /* RCC AHB4LPENR: GPIOGLPEN */
#define RCC_AHB4LPENR_GPIOHLPEN         (1 << 7)  /* RCC AHB4LPENR: GPIOHLPEN */
#define RCC_AHB4LPENR_GPIOILPEN         (1 << 8)  /* RCC AHB4LPENR: GPIOILPEN */
#define RCC_AHB4LPENR_GPIOJLPEN         (1 << 9)  /* RCC AHB4LPENR: GPIOJLPEN */
#define RCC_AHB4LPENR_GPIOKLPEN         (1 << 10) /* RCC AHB4LPENR: GPIOKLPEN */
#define RCC_AHB4LPENR_CRCLPEN           (1 << 19) /* RCC AHB4LPENR: CRCLPEN */
#define RCC_AHB4LPENR_BDMALPEN          (1 << 21) /* RCC AHB4LPENR: BDMALPEN */
#define RCC_AHB4LPENR_ADC3LPEN          (1 << 24) /* RCC AHB4LPENR: ADC3LPEN */
#define RCC_AHB4LPENR_BKPRAMLPEN        (1 << 28) /* RCC AHB4LPENR: BKPRAMLPEN */
#define RCC_AHB4LPENR_SRAM4LPEN         (1 << 29) /* RCC AHB4LPENR: SRAM4LPEN */

/* APB3 low power mode peripheral clock enable register */

#define RCC_APB3LPENR_LTDCLPEN          (1 << 3)  /* RCC APB3LPENR: LTDCLPEN */
#define RCC_APB3LPENR_WWDG1LPEN         (1 << 6)  /* RCC APB3LPENR: WWDG1LPEN */

/* APB1 L low power mode peripheral clock enable register */

#define RCC_APB1LLPENR_TIM2LPEN         (1 << 0)  /* RCC APB1LLPENR: TIM2LPEN */
#define RCC_APB1LLPENR_TIM3LPEN         (1 << 1)  /* RCC APB1LLPENR: TIM3LPEN */
#define RCC_APB1LLPENR_TIM4LPEN         (1 << 2)  /* RCC APB1LLPENR: TIM4LPEN */
#define RCC_APB1LLPENR_TIM5LPEN         (1 << 3)  /* RCC APB1LLPENR: TIM5LPEN */
#define RCC_APB1LLPENR_TIM6LPEN         (1 << 4)  /* RCC APB1LLPENR: TIM6LPEN */
#define RCC_APB1LLPENR_TIM7LPEN         (1 << 5)  /* RCC APB1LLPENR: TIM7LPEN */
#define RCC_APB1LLPENR_TIM12LPEN        (1 << 6)  /* RCC APB1LLPENR: TIM12LPEN */
#define RCC_APB1LLPENR_TIM13LPEN        (1 << 7)  /* RCC APB1LLPENR: TIM13LPEN */
#define RCC_APB1LLPENR_TIM14LPEN        (1 << 8)  /* RCC APB1LLPENR: TIM14LPEN */
#define RCC_APB1LLPENR_LPTIM1LPEN       (1 << 9)  /* RCC APB1LLPENR: LPTIM1LPEN */
#define RCC_APB1LLPENR_SPI2LPEN         (1 << 14) /* RCC APB1LLPENR: SPI2LPEN */
#define RCC_APB1LLPENR_SPI3LPEN         (1 << 15) /* RCC APB1LLPENR: SPI3LPEN */
#define RCC_APB1LLPENR_SPDIFRXLPEN      (1 << 16) /* RCC APB1LLPENR: SPDIFRXLPEN */
#define RCC_APB1LLPENR_USART2LPEN       (1 << 17) /* RCC APB1LLPENR: USART2LPEN */
#define RCC_APB1LLPENR_USART3LPEN       (1 << 18) /* RCC APB1LLPENR: USART3LPEN */
#define RCC_APB1LLPENR_UART4LPEN        (1 << 19) /* RCC APB1LLPENR: UART4LPEN */
#define RCC_APB1LLPENR_UART5LPEN        (1 << 20) /* RCC APB1LLPENR: UART5LPEN */
#define RCC_APB1LLPENR_I2C1LPEN         (1 << 21) /* RCC APB1LLPENR: I2C1LPEN */
#define RCC_APB1LLPENR_I2C2LPEN         (1 << 22) /* RCC APB1LLPENR: I2C2LPEN */
#define RCC_APB1LLPENR_I2C3LPEN         (1 << 23) /* RCC APB1LLPENR: I2C3LPEN */
#define RCC_APB1LLPENR_HDMICECLPEN      (1 << 27) /* RCC APB1LLPENR: HDMICECLPEN */
#define RCC_APB1LLPENR_DAC1LPEN         (1 << 29) /* RCC APB1LLPENR: DAC1LPEN */
#define RCC_APB1LLPENR_UART7LPEN        (1 << 30) /* RCC APB1LLPENR: UART7LPEN */
#define RCC_APB1LLPENR_UART8LPEN        (1 << 31) /* RCC APB1LLPENR: UART8LPEN */

/* APB1 H low power mode peripheral clock enable register */

#define RCC_APB1HLPENR_CRSLPEN          (1 << 1)  /* RCC APB1HLPENR: CRSLPEN */
#define RCC_APB1HLPENR_SWPLPEN          (1 << 2)  /* RCC APB1HLPENR: SWPLPEN */
#define RCC_APB1HLPENR_OPAMPLPEN        (1 << 4)  /* RCC APB1HLPENR: OPAMPLPEN */
#define RCC_APB1HLPENR_MDIOSLPEN        (1 << 5)  /* RCC APB1HLPENR: MDIOSLPEN */
#define RCC_APB1HLPENR_FDCANLPEN        (1 << 8)  /* RCC APB1HLPENR: FDCANLPEN */

/* APB2 low power mode peripheral clock enable register */

#define RCC_APB2LPENR_TIM1LPEN          (1 << 0)  /* RCC APB2LPENR: TIM1LPEN */
#define RCC_APB2LPENR_TIM8LPEN          (1 << 1)  /* RCC APB2LPENR: TIM8LPEN */
#define RCC_APB2LPENR_USART1LPEN        (1 << 4)  /* RCC APB2LPENR: USART1LPEN */
#define RCC_APB2LPENR_USART6LPEN        (1 << 5)  /* RCC APB2LPENR: USART6LPEN */
#define RCC_APB2LPENR_SPI1LPEN          (1 << 12) /* RCC APB2LPENR: SPI1LPEN */
#define RCC_APB2LPENR_SPI4LPEN          (1 << 13) /* RCC APB2LPENR: SPI4LPEN */
#define RCC_APB2LPENR_TIM15LPEN         (1 << 16) /* RCC APB2LPENR: TIM15LPEN */
#define RCC_APB2LPENR_TIM16LPEN         (1 << 17) /* RCC APB2LPENR: TIM16LPEN */
#define RCC_APB2LPENR_TIM17LPEN         (1 << 18) /* RCC APB2LPENR: TIM17LPEN */
#define RCC_APB2LPENR_SPI5LPEN          (1 << 20) /* RCC APB2LPENR: SPI5LPEN */
#define RCC_APB2LPENR_SAI1LPEN          (1 << 22) /* RCC APB2LPENR: SAI1LPEN */
#define RCC_APB2LPENR_SAI2LPEN          (1 << 23) /* RCC APB2LPENR: SAI2LPEN */
#define RCC_APB2LPENR_SAI3LPEN          (1 << 24) /* RCC APB2LPENR: SAI3LPEN */
#define RCC_APB2LPENR_DFSDM1LPEN        (1 << 28) /* RCC APB2LPENR: DFSDM1LPEN */
#define RCC_APB2LPENR_HRTIMLPEN         (1 << 29) /* RCC APB2LPENR: HRTIMLPEN */

/* APB4 low power mode peripheral clock enable register */

#define RCC_APB4LPENR_SYSCFGLPEN        (1 << 1)  /* RCC APB4LPENR: SYSCFGLPEN */
#define RCC_APB4LPENR_LPUART1LPEN       (1 << 3)  /* RCC APB4LPENR: LPUART1LPEN */
#define RCC_APB4LPENR_SPI6LPEN          (1 << 5)  /* RCC APB4LPENR: SPI6LPEN */
#define RCC_APB4LPENR_I2C4LPEN          (1 << 7)  /* RCC APB4LPENR: I2C4LPEN */
#define RCC_APB4LPENR_LPTIM2LPEN        (1 << 9)  /* RCC APB4LPENR: LPTIM2LPEN */
#define RCC_APB4LPENR_LPTIM3LPEN        (1 << 10) /* RCC APB4LPENR: LPTIM3LPEN */
#define RCC_APB4LPENR_LPTIM4LPEN        (1 << 11) /* RCC APB4LPENR: LPTIM4LPEN */
#define RCC_APB4LPENR_LPTIM5LPEN        (1 << 12) /* RCC APB4LPENR: LPTIM5LPEN */
#define RCC_APB4LPENR_COMP12LPEN        (1 << 14) /* RCC APB4LPENR: COMP12LPEN */
#define RCC_APB4LPENR_VREFLPEN          (1 << 15) /* RCC APB4LPENR: VREFLPEN */
#define RCC_APB4LPENR_RTCAPBLPEN        (1 << 16) /* RCC APB4LPENR: RTCAPBLPEN */
#define RCC_APB4LPENR_SAI4LPEN          (1 << 21) /* RCC APB4LPENR: SAI4LPEN */

/* Backup domain control register */

#define RCC_BDCR_LSEON                  (1 << 0)                     /* Bit 0: External Low Speed oscillator enable */
#define RCC_BDCR_LSERDY                 (1 << 1)                     /* Bit 1: External Low Speed oscillator Ready */
#define RCC_BDCR_LSEBYP                 (1 << 2)                     /* Bit 2: External Low Speed oscillator Bypass */
#define RCC_BDCR_LSEDRV_SHIFT           (3)                          /* Bits 4:3: LSE oscillator Drive selection */
#define RCC_BDCR_LSEDRV_MASK            (3 << RCC_BDCR_LSEDRV_SHIFT) /* See errata ES0392 Rev 7. 2.2.14 */
#  define RCC_BDCR_LSEDRV_LOW           (0 << RCC_BDCR_LSEDRV_SHIFT) /* 00: Low driving capability */
#  define RCC_BDCR_LSEDRV_MEDHI_Y       (1 << RCC_BDCR_LSEDRV_SHIFT) /* 01: Medium high driving capability rev y */
#  define RCC_BDCR_LSEDRV_MEDHI         (2 << RCC_BDCR_LSEDRV_SHIFT) /* 10: Medium high driving capability */
#  define RCC_BDCR_LSEDRV_MEDLO_Y       (2 << RCC_BDCR_LSEDRV_SHIFT) /* 10: Medium low driving capability rev y */
#  define RCC_BDCR_LSEDRV_MEDLO         (1 << RCC_BDCR_LSEDRV_SHIFT) /* 01: Medium low driving capability */
#  define RCC_BDCR_LSEDRV_HIGH          (3 << RCC_BDCR_LSEDRV_SHIFT) /* 11: High driving capability */
#define RCC_BDCR_LSECSSON               (1 << 5)                     /* Bit 5: LSE clock security system enable */
#define RCC_BDCR_LSECSSD                (1 << 6)                     /* Bit 6: LSE clock security system failure detection */
                                                                     /* Bit 7: Reserved */
#define RCC_BDCR_RTCSEL_SHIFT           (8)                          /* Bits 9:8: RTC clock source selection */
#define RCC_BDCR_RTCSEL_MASK            (3 << RCC_BDCR_RTCSEL_SHIFT)
#  define RCC_BDCR_RTCSEL_NOCLK         (0 << RCC_BDCR_RTCSEL_SHIFT) /* 00: No clock */
#  define RCC_BDCR_RTCSEL_LSE           (1 << RCC_BDCR_RTCSEL_SHIFT) /* 01: LSE oscillator clock used as RTC clock */
#  define RCC_BDCR_RTCSEL_LSI           (2 << RCC_BDCR_RTCSEL_SHIFT) /* 10: LSI oscillator clock used as RTC clock */
#  define RCC_BDCR_RTCSEL_HSE           (3 << RCC_BDCR_RTCSEL_SHIFT) /* 11: HSE oscillator clock divided by 128 used as RTC clock */
                                                                     /* Bits 10-15: Reserved */
#define RCC_BDCR_RTCEN                  (1 << 15)                    /* Bit 15: RTC clock enable */
#define RCC_BDCR_BDRST                  (1 << 16)                    /* Bit 16: Backup domain software reset */
                                                                     /* Bits 17-31: Reserved */

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7X3XX_RCC_H */
