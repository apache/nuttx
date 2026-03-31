/****************************************************************************
 * arch/arm/src/ht32f491x3/hardware/ht32f491x3_crm.h
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

#ifndef __ARCH_ARM_SRC_HT32F491X3_HARDWARE_HT32F491X3_CRM_H
#define __ARCH_ARM_SRC_HT32F491X3_HARDWARE_HT32F491X3_CRM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define HT32_CRM_CTRL_OFFSET            0x000
#define HT32_CRM_PLLCFG_OFFSET          0x004
#define HT32_CRM_CFG_OFFSET             0x008
#define HT32_CRM_CLKINT_OFFSET          0x00c
#define HT32_CRM_AHBRST1_OFFSET         0x010
#define HT32_CRM_AHBRST2_OFFSET         0x014
#define HT32_CRM_AHBRST3_OFFSET         0x018
#define HT32_CRM_APB1RST_OFFSET         0x020
#define HT32_CRM_APB2RST_OFFSET         0x024
#define HT32_CRM_AHBEN1_OFFSET          0x030
#define HT32_CRM_AHBEN2_OFFSET          0x034
#define HT32_CRM_AHBEN3_OFFSET          0x038
#define HT32_CRM_APB1EN_OFFSET          0x040
#define HT32_CRM_APB2EN_OFFSET          0x044
#define HT32_CRM_APB1LPEN_OFFSET        0x060
#define HT32_CRM_APB2LPEN_OFFSET        0x064
#define HT32_CRM_PICLKS_OFFSET          0x068
#define HT32_CRM_MISC1_OFFSET           0x0a0
#define HT32_CRM_MISC2_OFFSET           0x0a4

/* Register Addresses *******************************************************/

#define HT32_CRM_CTRL                   (HT32_CRM_BASE + HT32_CRM_CTRL_OFFSET)
#define HT32_CRM_PLLCFG                 (HT32_CRM_BASE + HT32_CRM_PLLCFG_OFFSET)
#define HT32_CRM_CFG                    (HT32_CRM_BASE + HT32_CRM_CFG_OFFSET)
#define HT32_CRM_CLKINT                 (HT32_CRM_BASE + HT32_CRM_CLKINT_OFFSET)
#define HT32_CRM_AHBRST1                (HT32_CRM_BASE + HT32_CRM_AHBRST1_OFFSET)
#define HT32_CRM_AHBRST2                (HT32_CRM_BASE + HT32_CRM_AHBRST2_OFFSET)
#define HT32_CRM_AHBRST3                (HT32_CRM_BASE + HT32_CRM_AHBRST3_OFFSET)
#define HT32_CRM_APB1RST                (HT32_CRM_BASE + HT32_CRM_APB1RST_OFFSET)
#define HT32_CRM_APB2RST                (HT32_CRM_BASE + HT32_CRM_APB2RST_OFFSET)
#define HT32_CRM_AHBEN1                 (HT32_CRM_BASE + HT32_CRM_AHBEN1_OFFSET)
#define HT32_CRM_AHBEN2                 (HT32_CRM_BASE + HT32_CRM_AHBEN2_OFFSET)
#define HT32_CRM_AHBEN3                 (HT32_CRM_BASE + HT32_CRM_AHBEN3_OFFSET)
#define HT32_CRM_APB1EN                 (HT32_CRM_BASE + HT32_CRM_APB1EN_OFFSET)
#define HT32_CRM_APB2EN                 (HT32_CRM_BASE + HT32_CRM_APB2EN_OFFSET)
#define HT32_CRM_APB1LPEN               (HT32_CRM_BASE + HT32_CRM_APB1LPEN_OFFSET)
#define HT32_CRM_APB2LPEN               (HT32_CRM_BASE + HT32_CRM_APB2LPEN_OFFSET)
#define HT32_CRM_PICLKS                 (HT32_CRM_BASE + HT32_CRM_PICLKS_OFFSET)
#define HT32_CRM_MISC1                  (HT32_CRM_BASE + HT32_CRM_MISC1_OFFSET)
#define HT32_CRM_MISC2                  (HT32_CRM_BASE + HT32_CRM_MISC2_OFFSET)

/* Clock control register ***************************************************/

#define HT32_CRM_CTRL_HICKEN            (1 << 0)
#define HT32_CRM_CTRL_HICKSTBL          (1 << 1)
#define HT32_CRM_CTRL_HEXTEN            (1 << 16)
#define HT32_CRM_CTRL_HEXTSTBL          (1 << 17)
#define HT32_CRM_CTRL_HEXTBYPS          (1 << 18)
#define HT32_CRM_CTRL_PLLEN             (1 << 24)
#define HT32_CRM_CTRL_PLLSTBL           (1 << 25)

/* PLL configuration register ***********************************************/

#define HT32_CRM_PLLCFG_PLLMS_SHIFT     (0)
#define HT32_CRM_PLLCFG_PLLMS_MASK      (0x0f << HT32_CRM_PLLCFG_PLLMS_SHIFT)
#define HT32_CRM_PLLCFG_PLLNS_SHIFT     (6)
#define HT32_CRM_PLLCFG_PLLNS_MASK      (0x1ff << HT32_CRM_PLLCFG_PLLNS_SHIFT)
#define HT32_CRM_PLLCFG_PLLFR_SHIFT     (16)
#define HT32_CRM_PLLCFG_PLLFR_MASK      (7 << HT32_CRM_PLLCFG_PLLFR_SHIFT)
#define HT32_CRM_PLLCFG_PLLRCS          (1 << 22)

#define HT32_CRM_PLLCFG_SOURCE_HICK     0
#define HT32_CRM_PLLCFG_SOURCE_HEXT     HT32_CRM_PLLCFG_PLLRCS

#define HT32_CRM_PLLCFG_FR_1            (0 << HT32_CRM_PLLCFG_PLLFR_SHIFT)
#define HT32_CRM_PLLCFG_FR_2            (1 << HT32_CRM_PLLCFG_PLLFR_SHIFT)
#define HT32_CRM_PLLCFG_FR_4            (2 << HT32_CRM_PLLCFG_PLLFR_SHIFT)
#define HT32_CRM_PLLCFG_FR_8            (3 << HT32_CRM_PLLCFG_PLLFR_SHIFT)
#define HT32_CRM_PLLCFG_FR_16           (4 << HT32_CRM_PLLCFG_PLLFR_SHIFT)
#define HT32_CRM_PLLCFG_FR_32           (5 << HT32_CRM_PLLCFG_PLLFR_SHIFT)

/* Clock configuration register *********************************************/

#define HT32_CRM_CFG_SCLKSEL_SHIFT      (0)
#define HT32_CRM_CFG_SCLKSEL_MASK       (3 << HT32_CRM_CFG_SCLKSEL_SHIFT)
#  define HT32_CRM_CFG_SEL_HICK         (0 << HT32_CRM_CFG_SCLKSEL_SHIFT)
#  define HT32_CRM_CFG_SEL_HEXT         (1 << HT32_CRM_CFG_SCLKSEL_SHIFT)
#  define HT32_CRM_CFG_SEL_PLL          (2 << HT32_CRM_CFG_SCLKSEL_SHIFT)

#define HT32_CRM_CFG_SCLKSTS_SHIFT      (2)
#define HT32_CRM_CFG_SCLKSTS_MASK       (3 << HT32_CRM_CFG_SCLKSTS_SHIFT)
#  define HT32_CRM_CFG_STS_HICK         (0 << HT32_CRM_CFG_SCLKSTS_SHIFT)
#  define HT32_CRM_CFG_STS_HEXT         (1 << HT32_CRM_CFG_SCLKSTS_SHIFT)
#  define HT32_CRM_CFG_STS_PLL          (2 << HT32_CRM_CFG_SCLKSTS_SHIFT)

#define HT32_CRM_CFG_AHBDIV_SHIFT       (4)
#define HT32_CRM_CFG_AHBDIV_MASK        (0x0f << HT32_CRM_CFG_AHBDIV_SHIFT)
#  define HT32_CRM_CFG_AHBDIV_NONE      (0 << HT32_CRM_CFG_AHBDIV_SHIFT)
#  define HT32_CRM_CFG_AHBDIV_2         (8 << HT32_CRM_CFG_AHBDIV_SHIFT)
#  define HT32_CRM_CFG_AHBDIV_4         (9 << HT32_CRM_CFG_AHBDIV_SHIFT)
#  define HT32_CRM_CFG_AHBDIV_8         (10 << HT32_CRM_CFG_AHBDIV_SHIFT)
#  define HT32_CRM_CFG_AHBDIV_16        (11 << HT32_CRM_CFG_AHBDIV_SHIFT)
#  define HT32_CRM_CFG_AHBDIV_64        (12 << HT32_CRM_CFG_AHBDIV_SHIFT)
#  define HT32_CRM_CFG_AHBDIV_128       (13 << HT32_CRM_CFG_AHBDIV_SHIFT)
#  define HT32_CRM_CFG_AHBDIV_256       (14 << HT32_CRM_CFG_AHBDIV_SHIFT)
#  define HT32_CRM_CFG_AHBDIV_512       (15 << HT32_CRM_CFG_AHBDIV_SHIFT)

#define HT32_CRM_CFG_APB1DIV_SHIFT      (10)
#define HT32_CRM_CFG_APB1DIV_MASK       (7 << HT32_CRM_CFG_APB1DIV_SHIFT)
#  define HT32_CRM_CFG_APB1DIV_1        (0 << HT32_CRM_CFG_APB1DIV_SHIFT)
#  define HT32_CRM_CFG_APB1DIV_2        (4 << HT32_CRM_CFG_APB1DIV_SHIFT)
#  define HT32_CRM_CFG_APB1DIV_4        (5 << HT32_CRM_CFG_APB1DIV_SHIFT)
#  define HT32_CRM_CFG_APB1DIV_8        (6 << HT32_CRM_CFG_APB1DIV_SHIFT)
#  define HT32_CRM_CFG_APB1DIV_16       (7 << HT32_CRM_CFG_APB1DIV_SHIFT)
#  define HT32_CRM_CFG_APB1DIV_NONE     HT32_CRM_CFG_APB1DIV_1

#define HT32_CRM_CFG_APB2DIV_SHIFT      (13)
#define HT32_CRM_CFG_APB2DIV_MASK       (7 << HT32_CRM_CFG_APB2DIV_SHIFT)
#  define HT32_CRM_CFG_APB2DIV_1        (0 << HT32_CRM_CFG_APB2DIV_SHIFT)
#  define HT32_CRM_CFG_APB2DIV_2        (4 << HT32_CRM_CFG_APB2DIV_SHIFT)
#  define HT32_CRM_CFG_APB2DIV_4        (5 << HT32_CRM_CFG_APB2DIV_SHIFT)
#  define HT32_CRM_CFG_APB2DIV_8        (6 << HT32_CRM_CFG_APB2DIV_SHIFT)
#  define HT32_CRM_CFG_APB2DIV_16       (7 << HT32_CRM_CFG_APB2DIV_SHIFT)
#  define HT32_CRM_CFG_APB2DIV_NONE     HT32_CRM_CFG_APB2DIV_1

/* Clock misc1 register *****************************************************/

#define HT32_CRM_MISC1_HICKDIV          (1 << 12)
#define HT32_CRM_MISC1_HICKTOUSB        (1 << 13)
#define HT32_CRM_MISC1_HICKTOSCLK       (1 << 14)
#define HT32_CRM_MISC1_PLLCLKTOADC      (1 << 15)

/* Clock misc2 register *****************************************************/

#define HT32_CRM_MISC2_AUTOSTEP_SHIFT   (4)
#define HT32_CRM_MISC2_AUTOSTEP_MASK    (3 << HT32_CRM_MISC2_AUTOSTEP_SHIFT)
#define HT32_CRM_MISC2_AUTOSTEP_ENABLE  (3 << HT32_CRM_MISC2_AUTOSTEP_SHIFT)
#define HT32_CRM_MISC2_HICKDIV_SHIFT    (16)
#define HT32_CRM_MISC2_HICKDIV_MASK     (7 << HT32_CRM_MISC2_HICKDIV_SHIFT)
#  define HT32_CRM_MISC2_HICKDIV_1      (0 << HT32_CRM_MISC2_HICKDIV_SHIFT)
#  define HT32_CRM_MISC2_HICKDIV_2      (1 << HT32_CRM_MISC2_HICKDIV_SHIFT)
#  define HT32_CRM_MISC2_HICKDIV_4      (2 << HT32_CRM_MISC2_HICKDIV_SHIFT)
#  define HT32_CRM_MISC2_HICKDIV_8      (3 << HT32_CRM_MISC2_HICKDIV_SHIFT)
#  define HT32_CRM_MISC2_HICKDIV_16     (4 << HT32_CRM_MISC2_HICKDIV_SHIFT)

/* Peripheral independent clocks register ***********************************/

#define HT32_CRM_PICLKS_USART1SEL_SHIFT (0)
#define HT32_CRM_PICLKS_USART1SEL_MASK  (3 << HT32_CRM_PICLKS_USART1SEL_SHIFT)
#define HT32_CRM_PICLKS_USART2SEL_SHIFT (2)
#define HT32_CRM_PICLKS_USART2SEL_MASK  (3 << HT32_CRM_PICLKS_USART2SEL_SHIFT)
#define HT32_CRM_PICLKS_USART3SEL_SHIFT (4)
#define HT32_CRM_PICLKS_USART3SEL_MASK  (3 << HT32_CRM_PICLKS_USART3SEL_SHIFT)

#define HT32_CRM_PICLKS_USARTSEL_PCLK   0
#define HT32_CRM_PICLKS_USARTSEL_SCLK   1
#define HT32_CRM_PICLKS_USARTSEL_HICK   2
#define HT32_CRM_PICLKS_USARTSEL_LEXT   3

/* Peripheral reset registers ***********************************************/

#define HT32_CRM_APB1RST_TMR2RST       (1 << 0)
#define HT32_CRM_APB1RST_TMR3RST       (1 << 1)
#define HT32_CRM_APB1RST_TMR4RST       (1 << 2)
#define HT32_CRM_APB1RST_TMR6RST       (1 << 4)
#define HT32_CRM_APB1RST_TMR7RST       (1 << 5)
#define HT32_CRM_APB1RST_TMR12RST      (1 << 6)
#define HT32_CRM_APB1RST_TMR13RST      (1 << 7)
#define HT32_CRM_APB1RST_TMR14RST      (1 << 8)
#define HT32_CRM_APB1RST_USART2RST     (1 << 17)
#define HT32_CRM_APB1RST_USART3RST     (1 << 18)

#define HT32_CRM_APB2RST_TMR1RST       (1 << 0)
#define HT32_CRM_APB2RST_USART1RST     (1 << 4)
#define HT32_CRM_APB2RST_TMR9RST       (1 << 16)
#define HT32_CRM_APB2RST_TMR10RST      (1 << 17)
#define HT32_CRM_APB2RST_TMR11RST      (1 << 18)

/* Clock enable registers ***************************************************/

#define HT32_CRM_AHBEN1_GPIOAEN         (1 << 0)
#define HT32_CRM_AHBEN1_GPIOBEN         (1 << 1)
#define HT32_CRM_AHBEN1_GPIOCEN         (1 << 2)
#define HT32_CRM_AHBEN1_GPIODEN         (1 << 3)
#define HT32_CRM_AHBEN1_GPIOEEN         (1 << 4)
#define HT32_CRM_AHBEN1_GPIOFEN         (1 << 5)
#define HT32_CRM_AHBEN1_CRCEN           (1 << 12)
#define HT32_CRM_AHBEN1_DMA1EN          (1 << 22)
#define HT32_CRM_AHBEN1_DMA2EN          (1 << 24)

#define HT32_CRM_AHBEN2_OTGFS1EN        (1 << 7)

#define HT32_CRM_AHBEN3_XMCEN           (1 << 0)

#define HT32_CRM_APB1EN_TMR2EN          (1 << 0)
#define HT32_CRM_APB1EN_TMR3EN          (1 << 1)
#define HT32_CRM_APB1EN_TMR4EN          (1 << 2)
#define HT32_CRM_APB1EN_TMR6EN          (1 << 4)
#define HT32_CRM_APB1EN_TMR7EN          (1 << 5)
#define HT32_CRM_APB1EN_TMR12EN         (1 << 6)
#define HT32_CRM_APB1EN_TMR13EN         (1 << 7)
#define HT32_CRM_APB1EN_TMR14EN         (1 << 8)
#define HT32_CRM_APB1EN_WWDTEN          (1 << 11)
#define HT32_CRM_APB1EN_SPI2EN          (1 << 14)
#define HT32_CRM_APB1EN_SPI3EN          (1 << 15)
#define HT32_CRM_APB1EN_USART2EN        (1 << 17)
#define HT32_CRM_APB1EN_USART3EN        (1 << 18)
#define HT32_CRM_APB1EN_USART4EN        (1 << 19)
#define HT32_CRM_APB1EN_USART5EN        (1 << 20)
#define HT32_CRM_APB1EN_I2C1EN          (1 << 21)
#define HT32_CRM_APB1EN_I2C2EN          (1 << 22)
#define HT32_CRM_APB1EN_I2C3EN          (1 << 23)
#define HT32_CRM_APB1EN_CAN1EN          (1 << 25)
#define HT32_CRM_APB1EN_CAN2EN          (1 << 26)
#define HT32_CRM_APB1EN_PWCEN           (1 << 28)
#define HT32_CRM_APB1EN_DACEN           (1 << 29)
#define HT32_CRM_APB1EN_USART7EN        (1u << 30)
#define HT32_CRM_APB1EN_USART8EN        (1u << 31)

#define HT32_CRM_APB2EN_TMR1EN          (1 << 0)
#define HT32_CRM_APB2EN_USART1EN        (1 << 4)
#define HT32_CRM_APB2EN_USART6EN        (1 << 5)
#define HT32_CRM_APB2EN_ADC1EN          (1 << 8)
#define HT32_CRM_APB2EN_SPI1EN          (1 << 12)
#define HT32_CRM_APB2EN_SCFGEN          (1 << 14)
#define HT32_CRM_APB2EN_TMR9EN          (1 << 16)
#define HT32_CRM_APB2EN_TMR10EN         (1 << 17)
#define HT32_CRM_APB2EN_TMR11EN         (1 << 18)
#define HT32_CRM_APB2EN_ACCEN           (1 << 29)

#endif /* __ARCH_ARM_SRC_HT32F491X3_HARDWARE_HT32F491X3_CRM_H */
