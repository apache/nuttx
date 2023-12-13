/****************************************************************************
 * arch/arm/src/at32/hardware/at32f43xxx_rcc.h
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

#ifndef __ARCH_ARM_SRC_AT32_HARDWARE_AT32F43XXX_RCC_H
#define __ARCH_ARM_SRC_AT32_HARDWARE_AT32F43XXX_RCC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define AT32_CRM_CTRL_OFFSET            0x000 /* Clock control register */
#define AT32_CRM_PLL_CFG_OFFSET         0x004 /* PLL configuration register */
#define AT32_CRM_CFG_OFFSET             0x008 /* Clock configuration register */
#define AT32_CRM_CLKINT_OFFSET          0x00C /* Clock interrupt register */
#define AT32_CRM_AHBRST1_OFFSET         0x010 /* AHB1 peripheral reset register */
#define AT32_CRM_AHBRST2_OFFSET         0x014 /* AHB2 peripheral reset register */
#define AT32_CRM_AHBRST3_OFFSET         0x018 /* AHB3 peripheral reset register */
#define AT32_CRM_APB1RST_OFFSET         0x020 /* APB1 Peripheral reset register */
#define AT32_CRM_APB2RST_OFFSET         0x024 /* APB2 Peripheral reset register */
#define AT32_CRM_AHBEN1_OFFSET          0x030 /* AHB1 Peripheral Clock enable register */
#define AT32_CRM_AHBEN2_OFFSET          0x034 /* AHB2 Peripheral Clock enable register */
#define AT32_CRM_AHBEN3_OFFSET          0x038 /* AHB3 Peripheral Clock enable register */
#define AT32_CRM_APB1EN_OFFSET          0x040 /* APB1 Peripheral Clock enable register */
#define AT32_CRM_APB2EN_OFFSET          0x044 /* APB2 Peripheral Clock enable register */
#define AT32_CRM_AHBLPEN1_OFFSET        0x050 /* RCC AHB1 low power mode peripheral clock enable register */
#define AT32_CRM_AHBLPEN2_OFFSET        0x054 /* RCC AHB2 low power mode peripheral clock enable register */
#define AT32_CRM_AHBLPEN3_OFFSET        0x058 /* RCC AHB3 low power mode peripheral clock enable register */
#define AT32_CRM_APB1LPEN_OFFSET        0x060 /* RCC APB1 low power mode peripheral clock enable register */
#define AT32_CRM_APB2LPEN_OFFSET        0x064 /* RCC APB2 low power mode peripheral clock enable register */
#define AT32_CRM_BPDC_OFFSET            0x070 /* Backup domain control register */
#define AT32_CRM_CTRLSTS_OFFSET         0x074 /* Control/status register */
#define AT32_CRM_MISC1_OFFSET           0x0A0 /* Misc1 register */
#define AT32_CRM_MISC2_OFFSET           0x0A4 /* Misc2 register */

/* Register Addresses *******************************************************/

#define AT32_CRM_CTRL                   (AT32_CRM_BASE+AT32_CRM_CTRL_OFFSET)       
#define AT32_CRM_PLL_CFG                (AT32_CRM_BASE+AT32_CRM_PLL_CFG_OFFSET)   
#define AT32_CRM_CFG                    (AT32_CRM_BASE+AT32_CRM_CFG_OFFSET)   
#define AT32_CRM_CLKINT                 (AT32_CRM_BASE+AT32_CRM_CLKINT_OFFSET)   
#define AT32_CRM_AHBRST1                (AT32_CRM_BASE+AT32_CRM_AHBRST1_OFFSET)   
#define AT32_CRM_AHBRST2                (AT32_CRM_BASE+AT32_CRM_AHBRST2_OFFSET)   
#define AT32_CRM_AHBRST3                (AT32_CRM_BASE+AT32_CRM_AHBRST3_OFFSET)   
#define AT32_CRM_APB1RST                (AT32_CRM_BASE+AT32_CRM_APB1RST_OFFSET)   
#define AT32_CRM_APB2RST                (AT32_CRM_BASE+AT32_CRM_APB2RST_OFFSET)   
#define AT32_CRM_AHBEN1                 (AT32_CRM_BASE+AT32_CRM_AHBEN1_OFFSET)   
#define AT32_CRM_AHBEN2                 (AT32_CRM_BASE+AT32_CRM_AHBEN2_OFFSET)   
#define AT32_CRM_AHBEN3                 (AT32_CRM_BASE+AT32_CRM_AHBEN3_OFFSET)   
#define AT32_CRM_APB1EN                 (AT32_CRM_BASE+AT32_CRM_APB1EN_OFFSET)   
#define AT32_CRM_APB2EN                 (AT32_CRM_BASE+AT32_CRM_APB2EN_OFFSET)   
#define AT32_CRM_AHBLPEN1               (AT32_CRM_BASE+AT32_CRM_AHBLPEN1_OFFSET)   
#define AT32_CRM_AHBLPEN2               (AT32_CRM_BASE+AT32_CRM_AHBLPEN2_OFFSET)   
#define AT32_CRM_AHBLPEN3               (AT32_CRM_BASE+AT32_CRM_AHBLPEN3_OFFSET)   
#define AT32_CRM_APB1LPEN               (AT32_CRM_BASE+AT32_CRM_APB1LPEN_OFFSET)   
#define AT32_CRM_APB2LPEN               (AT32_CRM_BASE+AT32_CRM_APB2LPEN_OFFSET)   
#define AT32_CRM_BPDC                   (AT32_CRM_BASE+AT32_CRM_BPDC_OFFSET)   
#define AT32_CRM_CTRLSTS                (AT32_CRM_BASE+AT32_CRM_CTRLSTS_OFFSET)   
#define AT32_CRM_MISC1                  (AT32_CRM_BASE+AT32_CRM_MISC1_OFFSET)   
#define AT32_CRM_MISC2                  (AT32_CRM_BASE+AT32_CRM_MISC2_OFFSET)   

/* Register Bitfield Definitions ********************************************/

/* Clock control register */

#define CRM_CTRL_HICKEN                 (1 << 0)    /* High speed internal clock enable */
#define CRM_CTRL_HICKSTBL               (1 << 1)    /* High speed internal clock stable */
#define CRM_CTRL_HICKTRIM               (0x3F << 2) /* High speed internal clock trimming */
#define CRM_CTRL_HICKCAL                (0xFF << 8) /* High speed internal clock calibration */
#define CRM_CTRL_HEXTEN                 (1 << 16)   /* High speed external crystal enable */
#define CRM_CTRL_HEXTSTBL               (1 << 17)   /* High speed external crystal stable */
#define CRM_CTRL_HEXTBYPS               (1 << 18)   /* High speed external crystal bypass */
#define CRM_CTRL_CFDEN                  (1 << 19)   /* Clock Failure Detection enable */
#define CRM_CTRL_PLLEN                  (1 << 24)   /* PLL enable */
#define CRM_CTRL_PLLSTBL                (1 << 25)   /* PLL clock stable */

/* PLL configuration register */

#define CRM_PLL_CFG_PLL_MS_SHIFT        (0) /* PLL pre-division, range: 1~15 */
#define CRM_PLL_CFG_PLL_MS_MASK         (15 << CRM_PLL_CFG_PLL_MS_SHIFT) 
#  define CRM_PLL_CFG_PLL_MS(n)         ((n) << CRM_PLL_CFG_PLL_MS_SHIFT) /* n = 1..15 */

#define CRM_PLL_CFG_PLL_NS_SHIFT        (6) /* PLL Multiplication Factor,range: 31~500 */
#define CRM_PLL_CFG_PLL_NS_MASK         (0x1FF << CRM_PLL_CFG_PLL_NS_SHIFT) 
#  define CRM_PLL_CFG_PLL_NS(n)         ((n) << CRM_PLL_CFG_PLL_NS_SHIFT) /* n = 31..500 */

#define CRM_PLL_CFG_PLL_FR_SHIFT        (16) /* PLL post-division */
#define CRM_PLL_CFG_PLL_FR_MASK         (7 << CRM_PLL_CFG_PLL_FR_SHIFT)
#  define CRM_PLL_CFG_PLL_FR_1          (0 << CRM_PLL_CFG_PLL_FR_SHIFT) /* div 1 */
#  define CRM_PLL_CFG_PLL_FR_2          (1 << CRM_PLL_CFG_PLL_FR_SHIFT) /* div 2 */
#  define CRM_PLL_CFG_PLL_FR_4          (2 << CRM_PLL_CFG_PLL_FR_SHIFT) /* div 4 */
#  define CRM_PLL_CFG_PLL_FR_8          (3 << CRM_PLL_CFG_PLL_FR_SHIFT) /* div 8 */
#  define CRM_PLL_CFG_PLL_FR_16         (4 << CRM_PLL_CFG_PLL_FR_SHIFT) /* div 16 */
#  define CRM_PLL_CFG_PLL_FR_32         (5 << CRM_PLL_CFG_PLL_FR_SHIFT) /* div 32 */

#define CRM_PLL_CFG_PLLRCS              (1 << 22) /* PLL reference clock select */

/* Clock configuration register */

#define CRM_CFG_SCLKSEL_SHIFT           (0) /* System clock select */
#define CRM_CFG_SCLKSEL_MASK            (3 << CRM_CFG_SCLKSEL_SHIFT)
#  define CRM_CFG_SEL_HICK              (0 << CRM_CFG_SCLKSEL_SHIFT) /* Select HICK */
#  define CRM_CFG_SEL_HEXT              (1 << CRM_CFG_SCLKSEL_SHIFT) /* Select HEXT */
#  define CRM_CFG_SEL_PLL               (2 << CRM_CFG_SCLKSEL_SHIFT) /* Select PLL */

#define CRM_CFG_SCLKSTS_SHIFT           (2) /* System clock select status */
#define CRM_CFG_SCLKSTSL_MASK           (3 << CRM_CFG_SCLKSTS_SHIFT)
#  define CRM_CFG_STS_HICK              (0 << CRM_CFG_SCLKSTS_SHIFT) /* Select HICK */
#  define CRM_CFG_STS_HEXT              (1 << CRM_CFG_SCLKSTS_SHIFT) /* Select HEXT */
#  define CRM_CFG_STS_PLL               (2 << CRM_CFG_SCLKSTS_SHIFT) /* Select PLL */

#define CRM_CFG_AHBDIV_SHIFT            (4) /* AHB division */
#define CRM_CFG_AHBDIV_MASK             (7 << CRM_CFG_AHBDIV_SHIFT)
#  define CRM_CFG_AHBDIV_NONE           (0 << CRM_CFG_AHBDIV_SHIFT)  /* AHB division None */
#  define CRM_CFG_AHBDIV_2              (8 << CRM_CFG_AHBDIV_SHIFT)  /* AHB division 2 */
#  define CRM_CFG_AHBDIV_4              (9 << CRM_CFG_AHBDIV_SHIFT)  /* AHB division 4 */
#  define CRM_CFG_AHBDIV_8              (10 << CRM_CFG_AHBDIV_SHIFT) /* AHB division 8 */
#  define CRM_CFG_AHBDIV_16             (11 << CRM_CFG_AHBDIV_SHIFT) /* AHB division 16 */
#  define CRM_CFG_AHBDIV_64             (12 << CRM_CFG_AHBDIV_SHIFT) /* AHB division 64 */
#  define CRM_CFG_AHBDIV_128            (13 << CRM_CFG_AHBDIV_SHIFT) /* AHB division 128 */
#  define CRM_CFG_AHBDIV_256            (14 << CRM_CFG_AHBDIV_SHIFT) /* AHB division 256 */
#  define CRM_CFG_AHBDIV_512            (15 << CRM_CFG_AHBDIV_SHIFT) /* AHB division 512 */

#define CRM_CFG_APB1DIV_SHIFT           (10) /* APB1 division */
#define CRM_CFG_APB1DIV_MASK            (7 << CRM_CFG_APB1DIV_SHIFT)
#  define CRM_CFG_APB1DIV_NONE          (3 << CRM_CFG_APB1DIV_SHIFT) /* APB1 division None */
#  define CRM_CFG_APB1DIV_2             (4 << CRM_CFG_APB1DIV_SHIFT) /* APB1 division 2 */
#  define CRM_CFG_APB1DIV_4             (5 << CRM_CFG_APB1DIV_SHIFT) /* APB1 division 4 */
#  define CRM_CFG_APB1DIV_8             (6 << CRM_CFG_APB1DIV_SHIFT) /* APB1 division 8 */
#  define CRM_CFG_APB1DIV_16            (7 << CRM_CFG_APB1DIV_SHIFT) /* APB1 division 16 */

#define CRM_CFG_APB2DIV_SHIFT           (13) /* APB2 division */
#define CRM_CFG_APB2DIV_MASK            (7 << CRM_CFG_APB2DIV_SHIFT)
#  define CRM_CFG_APB2DIV_NONE          (3 << CRM_CFG_APB2DIV_SHIFT) /* APB2 division None */
#  define CRM_CFG_APB2DIV_2             (4 << CRM_CFG_APB2DIV_SHIFT) /* APB2 division 2 */
#  define CRM_CFG_APB2DIV_4             (5 << CRM_CFG_APB2DIV_SHIFT) /* APB2 division 4 */
#  define CRM_CFG_APB2DIV_8             (6 << CRM_CFG_APB2DIV_SHIFT) /* APB2 division 8 */
#  define CRM_CFG_APB2DIV_16            (7 << CRM_CFG_APB2DIV_SHIFT) /* APB2 division 16 */

#define CRM_CFG_ERTCDIV_SHIFT           (16) /* HEXT division for ERTC clock */
#define CRM_CFG_ERTCDIV_MASK            (31 << CRM_CFG_ERTCDIV_SHIFT)
#  define CRM_CFG_ERTCDIV_NONE1         (0 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock None */
#  define CRM_CFG_ERTCDIV_NONE2         (1 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock None */
#  define CRM_CFG_ERTCDIV_2             (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 2 */
#  define CRM_CFG_ERTCDIV_3             (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 3 */
#  define CRM_CFG_ERTCDIV_4             (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 4 */
#  define CRM_CFG_ERTCDIV_5             (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 5 */
#  define CRM_CFG_ERTCDIV_6             (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 6 */
#  define CRM_CFG_ERTCDIV_7             (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 7 */
#  define CRM_CFG_ERTCDIV_8             (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 8 */
#  define CRM_CFG_ERTCDIV_9             (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 9 */
#  define CRM_CFG_ERTCDIV_10            (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 10 */
#  define CRM_CFG_ERTCDIV_11            (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 11 */
#  define CRM_CFG_ERTCDIV_12            (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 12 */
#  define CRM_CFG_ERTCDIV_13            (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 13 */
#  define CRM_CFG_ERTCDIV_14            (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 14 */
#  define CRM_CFG_ERTCDIV_15            (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 15 */
#  define CRM_CFG_ERTCDIV_16            (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 16 */
#  define CRM_CFG_ERTCDIV_17            (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 17 */
#  define CRM_CFG_ERTCDIV_18            (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 18 */
#  define CRM_CFG_ERTCDIV_19            (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 19 */
#  define CRM_CFG_ERTCDIV_20            (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 20 */
#  define CRM_CFG_ERTCDIV_21            (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 21 */
#  define CRM_CFG_ERTCDIV_22            (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 22 */
#  define CRM_CFG_ERTCDIV_23            (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 23 */
#  define CRM_CFG_ERTCDIV_24            (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 24 */
#  define CRM_CFG_ERTCDIV_25            (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 25 */
#  define CRM_CFG_ERTCDIV_26            (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 26 */
#  define CRM_CFG_ERTCDIV_27            (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 27 */
#  define CRM_CFG_ERTCDIV_28            (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 28 */
#  define CRM_CFG_ERTCDIV_29            (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 29 */
#  define CRM_CFG_ERTCDIV_30            (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 30 */
#  define CRM_CFG_ERTCDIV_31            (2 << CRM_CFG_ERTCDIV_SHIFT) /* HEXT division for ERTC clock 31 */

#define CRM_CFG_CLKOUT1_SEL_SHIFT           (21) /* Clock output1 selection */
#define CRM_CFG_CLKOUT1_SEL_MASK            (3 << CRM_CFG_APB1DIV_SHIFT)
#  define CRM_CFG_CLKOUT1_SEL_HICK          (0 << CRM_CFG_APB1DIV_SHIFT) /* HICK output */
#  define CRM_CFG_CLKOUT1_SEL_LEXT          (1 << CRM_CFG_APB1DIV_SHIFT) /* LEXT output */
#  define CRM_CFG_CLKOUT1_SEL_HEXT          (2 << CRM_CFG_APB1DIV_SHIFT) /* HEXT output */
#  define CRM_CFG_CLKOUT1_SEL_PLL           (3 << CRM_CFG_APB1DIV_SHIFT) /* PLL output */

#define CRM_CFG_CLKOUT1DIV1_SHIFT           (24) /* Clock output1 division1 */
#define CRM_CFG_CLKOUT1DIV1_MASK            (7 << CRM_CFG_CLKOUT1DIV1_SHIFT)
#  define CRM_CFG_CLKOUT1DIV1_1             (3 << CRM_CFG_CLKOUT1DIV1_SHIFT) /* CLKOUT1 */
#  define CRM_CFG_CLKOUT1DIV1_1_2           (4 << CRM_CFG_CLKOUT1DIV1_SHIFT) /* CLKOUT1/2 */
#  define CRM_CFG_CLKOUT1DIV1_1_3           (5 << CRM_CFG_CLKOUT1DIV1_SHIFT) /* CLKOUT1/3 */
#  define CRM_CFG_CLKOUT1DIV1_1_4           (6 << CRM_CFG_CLKOUT1DIV1_SHIFT) /* CLKOUT1/4 */
#  define CRM_CFG_CLKOUT1DIV1_1_5           (7 << CRM_CFG_CLKOUT1DIV1_SHIFT) /* CLKOUT1/5 */

#define CRM_CFG_CLKOUT2DIV1_SHIFT           (27) /* Clock output2 division1 */
#define CRM_CFG_CLKOUT2DIV1_MASK            (7 << CRM_CFG_CLKOUT2DIV1_SHIFT)
#  define CRM_CFG_CLKOUT2DIV1_1             (3 << CRM_CFG_CLKOUT2DIV1_SHIFT) /* CLKOUT2 */
#  define CRM_CFG_CLKOUT2DIV1_1_2           (4 << CRM_CFG_CLKOUT2DIV1_SHIFT) /* CLKOUT2/2 */
#  define CRM_CFG_CLKOUT2DIV1_1_3           (5 << CRM_CFG_CLKOUT2DIV1_SHIFT) /* CLKOUT2/3 */
#  define CRM_CFG_CLKOUT2DIV1_1_4           (6 << CRM_CFG_CLKOUT2DIV1_SHIFT) /* CLKOUT2/4 */
#  define CRM_CFG_CLKOUT2DIV1_1_5           (7 << CRM_CFG_CLKOUT2DIV1_SHIFT) /* CLKOUT2/5 */

#define CRM_CFG_CLKOUT2_SEL1_SHIFT           (30) /* clock output2 selecction 1 */
#define CRM_CFG_CLKOUT2_SEL1_MASK            (3 << CRM_CFG_CLKOUT2_SEL1_SHIFT)
#  define CRM_CFG_CLKOUT2_SEL1_SCLK          (0 << CRM_CFG_CLKOUT2_SEL1_SHIFT) /* Output from SCLK */
#  define CRM_CFG_CLKOUT2_SEL1_2             (1 << CRM_CFG_CLKOUT2_SEL1_SHIFT) /* Output determine from CRM_MISC1 */
#  define CRM_CFG_CLKOUT2_SEL1_HEXT          (2 << CRM_CFG_CLKOUT2_SEL1_SHIFT) /* Output from HEXT */
#  define CRM_CFG_CLKOUT2_SEL1_PLL           (3 << CRM_CFG_CLKOUT2_SEL1_SHIFT) /* Output from PLL */

/* Clock interrupt register */

#define CRM_CLKINT_LICKSTBLF                 (1 << 0)  /* LICK stable flag */
#define CRM_CLKINT_LEXTSTBLF                 (1 << 1)  /* LEXT stable flag */
#define CRM_CLKINT_HICKSTBLF                 (1 << 2)  /* HICK stable flag */
#define CRM_CLKINT_HEXTSTBLF                 (1 << 3)  /* HEXT stable flag */
#define CRM_CLKINT_PLLSTBLF                  (1 << 4)  /* PLL stable flag */
#define CRM_CLKINT_CFDF                      (1 << 7)  /* Clock Failure Detection flag */
#define CRM_CLKINT_LICKSTBLIEN               (1 << 8)  /* LICK stable interrupt enable */
#define CRM_CLKINT_LEXTSTBLIEN               (1 << 9)  /* LEXT stable interrupt enable */
#define CRM_CLKINT_HICKSTBLIEN               (1 << 10) /* HICK stable interrupt enable */
#define CRM_CLKINT_HEXTSTBLIEN               (1 << 11) /* HEXT stable interrupt enable */
#define CRM_CLKINT_PLLSTBLIEN                (1 << 12) /* PLL stable interrupt enable */
#define CRM_CLKINT_LICKSTBLFC                (1 << 16) /* LICK stable flag clear */
#define CRM_CLKINT_LEXTSTBLFC                (1 << 17) /* LEXT stable flag clear */
#define CRM_CLKINT_HICKSTBLFC                (1 << 18) /* HICK stable flag clear */
#define CRM_CLKINT_HEXTSTBLFC                (1 << 19) /* HEXT stable flag clear */
#define CRM_CLKINT_PLLSTBLFC                 (1 << 20) /* PLL stable flag clear */
#define CRM_CLKINT_CFDFC                     (1 << 23) /* Clock failure detection interrupt clear */

/* AHB1 peripheral reset register */

#define CRM_AHBRST1_GPIOARST                (1 << 0)  /* IO port A reset */
#define CRM_AHBRST1_GPIOBRST                (1 << 1)  /* IO port B reset */
#define CRM_AHBRST1_GPIOCRST                (1 << 2)  /* IO port C reset */
#define CRM_AHBRST1_GPIODRST                (1 << 3)  /* IO port D reset */
#define CRM_AHBRST1_GPIOERST                (1 << 4)  /* IO port E reset */
#define CRM_AHBRST1_GPIOFRST                (1 << 5)  /* IO port F reset */
#define CRM_AHBRST1_GPIOGRST                (1 << 6)  /* IO port G reset */
#define CRM_AHBRST1_GPIOHRST                (1 << 7)  /* IO port H reset */
#define CRM_AHBRST1_CRCRST                  (1 << 12) /* CRC reset */
#define CRM_AHBRST1_EDMARST                 (1 << 21) /* EDMA reset */
#define CRM_AHBRST1_DMA1RST                 (1 << 22) /* DMA1 reset */
#define CRM_AHBRST1_DMA2RST                 (1 << 24) /* DMA2 reset */
#define CRM_AHBRST1_EMACRST                 (1 << 25) /* EMAC reset */
#define CRM_AHBRST1_OTGFS2RST               (1 << 29) /* OTGFS2 reset */

/* AHB2 peripheral reset register */

#define CRM_AHBRST2_DVPRST                  (1 << 0)  /* DVP reset */
#define CRM_AHBRST2_OTGFS1RST               (1 << 7)  /* OTGFS1 reset */
#define CRM_AHBRST2_SDIO1RST                (1 << 15) /* SDIO1 reset */

/* AHB3 peripheral reset register */

#define CRM_AHBRST3_XMCRST                  (1 << 0) /* XMC reset */
#define CRM_AHBRST3_QSPI1RST                (1 << 0) /* QSPI1 reset */
#define CRM_AHBRST3_QSPI2RST                (1 << 0) /* QSPI2 reset */
#define CRM_AHBRST3_SDIO2RST                (1 << 0) /* SDIO2 reset */

/* APB1 Peripheral reset register */

#define CRM_APB1RST_TMR2RST                 (1 << 0)  /* Timer2 reset */
#define CRM_APB1RST_TMR3RST                 (1 << 1)  /* Timer3 reset */
#define CRM_APB1RST_TMR4RST                 (1 << 2)  /* Timer4 reset */
#define CRM_APB1RST_TMR5RST                 (1 << 3)  /* Timer5 reset */
#define CRM_APB1RST_TMR6RST                 (1 << 4)  /* Timer6 reset */
#define CRM_APB1RST_TMR7RST                 (1 << 5)  /* Timer7 reset */
#define CRM_APB1RST_TMR12RST                (1 << 6)  /* Timer12 reset */
#define CRM_APB1RST_TMR13RST                (1 << 7)  /* Timer13 reset */
#define CRM_APB1RST_TMR14RST                (1 << 8)  /* Timer14 reset */
#define CRM_APB1RST_WWDTRST                 (1 << 11) /* Window watchdog reset */
#define CRM_APB1RST_SPI2RST                 (1 << 14) /* SPI2 reset */
#define CRM_APB1RST_SPI3RST                 (1 << 15) /* SPI3 reset */
#define CRM_APB1RST_USART2RST               (1 << 17) /* USART2 reset */
#define CRM_APB1RST_USART3RST               (1 << 18) /* USART3 reset */
#define CRM_APB1RST_UART4RST                (1 << 19) /* UART4 reset */
#define CRM_APB1RST_UART5RST                (1 << 20) /* UART5 reset */
#define CRM_APB1RST_I2C1RST                 (1 << 21) /* I2C1 reset */
#define CRM_APB1RST_I2C2RST                 (1 << 22) /* I2C2 reset */
#define CRM_APB1RST_I2C3RST                 (1 << 23) /* I2C3 reset */
#define CRM_APB1RST_CAN1RST                 (1 << 25) /* CAN1 reset */
#define CRM_APB1RST_CAN2RST                 (1 << 26) /* CAN2 reset */
#define CRM_APB1RST_PWCRST                  (1 << 28) /* Power interface reset */
#define CRM_APB1RST_DACRST                  (1 << 29) /* DAC interface reset */
#define CRM_APB1RST_UART7RST                (1 << 30) /* UART7 reset */
#define CRM_APB1RST_UART8RST                (1 << 31) /* UART8 reset */

/* APB2 Peripheral reset register */

#define CRM_APB2RST_TMR1RST                 (1 << 0)  /* TMR1 timer reset */
#define CRM_APB2RST_TMR8RST                 (1 << 1)  /* TMR8 timer reset */
#define CRM_APB2RST_USART1RST               (1 << 4)  /* USART1 reset */
#define CRM_APB2RST_USART6RST               (1 << 5)  /* USART6 reset */
#define CRM_APB2RST_ADCRST                  (1 << 8)  /* ADC interface reset */
#define CRM_APB2RST_SPI1RST                 (1 << 12) /* SPI1 reset */
#define CRM_APB2RST_SPI4RST                 (1 << 13) /* SPI4 reset */
#define CRM_APB2RST_SCFGRST                 (1 << 14) /* SCFG reset */
#define CRM_APB2RST_TMR9RST                 (1 << 16) /* Timer9 reset */
#define CRM_APB2RST_TMR10RST                (1 << 17) /* Timer10 reset */
#define CRM_APB2RST_TMR11RST                (1 << 18) /* Timer11 reset */
#define CRM_APB2RST_TMR20RST                (1 << 20) /* Timer20 reset */
#define CRM_APB2RST_ACCRST                  (1 << 29) /* ACC reset */

/* AHB1 Peripheral Clock enable register */

#define CRM_AHB1EN1_GPIOEN(n)               (1 << (n))
#define CRM_AHBEN1_GPIOAEN                  (1 << 0)  /* IO port A clock enable */
#define CRM_AHBEN1_GPIOBEN                  (1 << 1)  /* IO port B clock enable */
#define CRM_AHBEN1_GPIOCEN                  (1 << 2)  /* IO port C clock enable */
#define CRM_AHBEN1_GPIODEN                  (1 << 3)  /* IO port D clock enable */
#define CRM_AHBEN1_GPIOEEN                  (1 << 4)  /* IO port E clock enable */
#define CRM_AHBEN1_GPIOFEN                  (1 << 5)  /* IO port F clock enable */
#define CRM_AHBEN1_GPIOGEN                  (1 << 6)  /* IO port G clock enable */
#define CRM_AHBEN1_GPIOHEN                  (1 << 7)  /* IO port H clock enable */
#define CRM_AHBEN1_CRCEN                    (1 << 12) /* CRC clock enable */
#define CRM_AHBEN1_EDMAEN                   (1 << 21) /* EDMA clock enable */
#define CRM_AHBEN1_DMA1EN                   (1 << 22) /* DMA1 clock enable */
#define CRM_AHBEN1_DMA2EN                   (1 << 24) /* DMA2 clock enable */
#define CRM_AHBEN1_EMACEN                   (1 << 25) /* EMAC clock enable */
#define CRM_AHBEN1_EMACTXEN                 (1 << 26) /* EMAC TX clock enable */
#define CRM_AHBEN1_EMACRXEN                 (1 << 27) /* EMAC RX clock enable */
#define CRM_AHBEN1_EMACPTPEN                (1 << 28) /* EMAC PTP clock enable */
#define CRM_AHBEN1_OTGFS2EN                 (1 << 29) /* OTGFS2 clock enable */

/* AHB2 Peripheral Clock enable register */

#define CRM_AHBEN2_DVPEN                    (1 << 0)  /* DVP clock enable */
#define CRM_AHBEN2_OTGFS1EN                 (1 << 7)  /* OTGFS1 clock enable */
#define CRM_AHBEN2_SDIO1EN                  (1 << 15) /* SDIO1 clock enable */

/* AHB3 Peripheral Clock enable register */

#define CRM_AHBEN3_XMCEN                    (1 << 0)  /* XMC clock enable */
#define CRM_AHBEN3_QSPI1EN                  (1 << 1)  /* QSPI1 clock enable */
#define CRM_AHBEN3_QSPI2EN                  (1 << 14) /* QSPI2 clock enable */
#define CRM_AHBEN3_SDIO2EN                  (1 << 15) /* SDIO2 clock enable */

/* APB1 Peripheral Clock enable register */

#define CRM_APB1EN_TMR2EN                   (1 << 0)  /* Timer2 clock enable */
#define CRM_APB1EN_TMR3EN                   (1 << 1)  /* Timer3 clock enable */
#define CRM_APB1EN_TMR4EN                   (1 << 2)  /* Timer4 clock enable */
#define CRM_APB1EN_TMR5EN                   (1 << 3)  /* Timer5 clock enable */
#define CRM_APB1EN_TMR6EN                   (1 << 4)  /* Timer6 clock enable */
#define CRM_APB1EN_TMR7EN                   (1 << 5)  /* Timer7 clock enable */
#define CRM_APB1EN_TMR12EN                  (1 << 6)  /* Timer12 clock enable */
#define CRM_APB1EN_TMR13EN                  (1 << 7)  /* Timer13 clock enable */
#define CRM_APB1EN_TMR14EN                  (1 << 8)  /* Timer14 clock enable */
#define CRM_APB1EN_WWDTEN                   (1 << 11) /* Window watchdog clock enable */
#define CRM_APB1EN_SPI2EN                   (1 << 14) /* SPI2 clock enable */
#define CRM_APB1EN_SPI3EN                   (1 << 15) /* SPI3 clock enable */
#define CRM_APB1EN_USART2EN                 (1 << 17) /* USART2 clock enable */
#define CRM_APB1EN_USART3EN                 (1 << 18) /* USART3 clock enable */
#define CRM_APB1EN_UART4EN                  (1 << 19) /* UART4 clock enable */
#define CRM_APB1EN_UART5EN                  (1 << 20) /* UART5 clock enable */
#define CRM_APB1EN_I2C1EN                   (1 << 21) /* I2C1 clock enable */
#define CRM_APB1EN_I2C2EN                   (1 << 22) /* I2C2 clock enable */
#define CRM_APB1EN_I2C3EN                   (1 << 23) /* I2C3 clock enable */
#define CRM_APB1EN_CAN1EN                   (1 << 25) /* CAN1 clock enable */
#define CRM_APB1EN_CAN2EN                   (1 << 26) /* CAN2 clock enable */
#define CRM_APB1EN_PWCEN                    (1 << 28) /* Power interface clock enable */
#define CRM_APB1EN_DACEN                    (1 << 29) /* DAC interface clock enable */
#define CRM_APB1EN_UART7EN                  (1 << 30) /* UART7 clock enable */
#define CRM_APB1EN_UART8EN                  (1 << 31) /* UART8 clock enable */

/* APB2 Peripheral Clock enable register */

#define CRM_APB2EN_TMR1EN                   (1 << 0)  /* TMR1 timer clock enable */
#define CRM_APB2EN_TMR8EN                   (1 << 1)  /* TMR8 timer clock enable */
#define CRM_APB2EN_USART1EN                 (1 << 4)  /* USART1 clock enable */
#define CRM_APB2EN_USART6EN                 (1 << 5)  /* USART6 clock enable */
#define CRM_APB2EN_ADC1EN                   (1 << 8)  /* ADC1 interface clock enable */
#define CRM_APB2EN_ADC2EN                   (1 << 9)  /* ADC2 interface clock enable */
#define CRM_APB2EN_ADC3EN                   (1 << 10) /* ADC3 interface clock enable */
#define CRM_APB2EN_SPI1EN                   (1 << 12) /* SPI1 clock enable */
#define CRM_APB2EN_SPI4EN                   (1 << 13) /* SPI4 clock enable */
#define CRM_APB2EN_SCFGEN                   (1 << 14) /* SCFG clock enable */
#define CRM_APB2EN_TMR9EN                   (1 << 16) /* Timer9 clock enable */
#define CRM_APB2EN_TMR10EN                  (1 << 17) /* Timer10 clock enable */
#define CRM_APB2EN_TMR11EN                  (1 << 18) /* Timer11 clock enable */
#define CRM_APB2EN_TMR20EN                  (1 << 20) /* Timer20 clock enable */
#define CRM_APB2EN_ACCEN                    (1 << 29) /* ACC clock enable */

/* RCC AHB1 low power mode peripheral clock enable register */

#define CRM_AHBLPEN1_GPIOALPEN              (1 << 0)  /* IO port A clock enable during sleep mode */
#define CRM_AHBLPEN1_GPIOBLPEN              (1 << 1)  /* IO port B clock enable during sleep mode */
#define CRM_AHBLPEN1_GPIOCLPEN              (1 << 2)  /* IO port C clock enable during sleep mode */
#define CRM_AHBLPEN1_GPIODLPEN              (1 << 3)  /* IO port D clock enable during sleep mode */
#define CRM_AHBLPEN1_GPIOELPEN              (1 << 4)  /* IO port E clock enable during sleep mode */
#define CRM_AHBLPEN1_GPIOFLPEN              (1 << 5)  /* IO port F clock enable during sleep mode */
#define CRM_AHBLPEN1_GPIOGLPEN              (1 << 6)  /* IO port G clock enable during sleep mode */
#define CRM_AHBLPEN1_GPIOHLPEN              (1 << 7)  /* IO port H clock enable during sleep mode */
#define CRM_AHBLPEN1_CRCLPEN                (1 << 12) /* CRC clock enable during sleep mode */
#define CRM_AHBLPEN1_EDMALPEN               (1 << 21) /* EDMA clock enable during sleep mode */
#define CRM_AHBLPEN1_DMA1LPEN               (1 << 22) /* DMA1 clock enable during sleep mode */
#define CRM_AHBLPEN1_DMA2LPEN               (1 << 24) /* DMA2 clock enable during sleep mode */
#define CRM_AHBLPEN1_EMACLPEN               (1 << 25) /* EMAC clock enable during sleep mode */
#define CRM_AHBLPEN1_EMACTXLPEN             (1 << 26) /* EMAC TX clock enable during sleep mode */
#define CRM_AHBLPEN1_EMACRXLPEN             (1 << 27) /* EMAC RX clock enable during sleep mode */
#define CRM_AHBLPEN1_EMACPTPLPEN            (1 << 28) /* EMAC PTP clock enable during sleep mode */
#define CRM_AHBLPEN1_OTGFS2LPEN             (1 << 29) /* OTGFS2 clock enable during sleep mode */

/* RCC AHB2 low power mode peripheral clock enable register */

#define CRM_AHBLPEN2_DVPLPEN                (1 << 0)  /* DVP clock enable during sleep mode */
#define CRM_AHBLPEN2_OTGFS1LPEN             (1 << 7)  /* OTGFS1 clock enable during sleep mode */
#define CRM_AHBLPEN2_SDIO1LPEN              (1 << 15) /* SDIO1 clock enable during sleep mode */

/* RCC AHB3 low power mode Peripheral Clock enable register */

#define CRM_AHBLPEN3_XMCLPEN                (1 << 0)  /* XMC clock enable during sleep mode */
#define CRM_AHBLPEN3_QSPI1LPEN              (1 << 1)  /* QSPI1 clock enable during sleep mode */
#define CRM_AHBLPEN3_QSPI2LPEN              (1 << 14) /* QSPI2 clock enable during sleep mode */
#define CRM_AHBLPEN3_SDIO2LPEN              (1 << 15) /* SDIO2 clock enable during sleep mode */

/* RCC APB1 low power mode peripheral clock enable register */

#define CRM_APB1LPEN_TMR2LPEN               (1 << 0)  /* Timer2 clock enable during sleep mode */
#define CRM_APB1LPEN_TMR3LPEN               (1 << 1)  /* Timer3 clock enable during sleep mode */
#define CRM_APB1LPEN_TMR4LPEN               (1 << 2)  /* Timer4 clock enable during sleep mode */
#define CRM_APB1LPEN_TMR5LPEN               (1 << 3)  /* Timer5 clock enable during sleep mode */
#define CRM_APB1LPEN_TMR6LPEN               (1 << 4)  /* Timer6 clock enable during sleep mode */
#define CRM_APB1LPEN_TMR7LPEN               (1 << 5)  /* Timer7 clock enable during sleep mode */
#define CRM_APB1LPEN_TMR12LPEN              (1 << 6)  /* Timer12 clock enable during sleep mode */
#define CRM_APB1LPEN_TMR13LPEN              (1 << 7)  /* Timer13 clock enable during sleep mode */
#define CRM_APB1LPEN_TMR14LPEN              (1 << 8)  /* Timer14 clock enable during sleep mode */
#define CRM_APB1LPEN_WWDTLPEN               (1 << 11) /* Window watchdog clock enable during sleep mode */
#define CRM_APB1LPEN_SPI2LPEN               (1 << 14) /* SPI2 clock enable during sleep mode */
#define CRM_APB1LPEN_SPI3LPEN               (1 << 15) /* SPI3 clock enable during sleep mode */
#define CRM_APB1LPEN_USART2LPEN             (1 << 17) /* USART2 clock enable during sleep mode */
#define CRM_APB1LPEN_USART3LPEN             (1 << 18) /* USART3 clock enable during sleep mode */
#define CRM_APB1LPEN_UART4LPEN              (1 << 19) /* UART4 clock enable during sleep mode */
#define CRM_APB1LPEN_UART5LPEN              (1 << 20) /* UART5 clock enable during sleep mode */
#define CRM_APB1LPEN_I2C1LPEN               (1 << 21) /* I2C1 clock enable during sleep mode */
#define CRM_APB1LPEN_I2C2LPEN               (1 << 22) /* I2C2 clock enable during sleep mode */
#define CRM_APB1LPEN_I2C3LPEN               (1 << 23) /* I2C3 clock enable during sleep mode */
#define CRM_APB1LPEN_CAN1LPEN               (1 << 25) /* CAN1 clock enable during sleep mode */
#define CRM_APB1LPEN_CAN2LPEN               (1 << 26) /* CAN2 clock enable during sleep mode */
#define CRM_APB1LPEN_PWCLPEN                (1 << 28) /* Power interface clock enable during sleep mode */
#define CRM_APB1LPEN_DACLPEN                (1 << 29) /* DAC interface clock enable during sleep mode */
#define CRM_APB1LPEN_UART7LPEN              (1 << 30) /* UART7 clock enable during sleep mode */
#define CRM_APB1LPEN_UART8LPEN              (1 << 31) /* UART8 clock enable during sleep mode */

/* RCC APB2 low power mode peripheral clock enable register */

#define CRM_APB2LPEN_TMR1LPEN               (1 << 0)  /* TMR1 timer clock enable during sleep mode */
#define CRM_APB2LPEN_TMR8LPEN               (1 << 1)  /* TMR8 timer clock enable during sleep mode */
#define CRM_APB2LPEN_USART1LPEN             (1 << 4)  /* USART1 clock enable during sleep mode */
#define CRM_APB2LPEN_USART6LPEN             (1 << 5)  /* USART6 clock enable during sleep mode */
#define CRM_APB2LPEN_ADC1LPEN               (1 << 8)  /* ADC1 interface clock enable during sleep mode */
#define CRM_APB2LPEN_ADC2LPEN               (1 << 9)  /* ADC2 interface clock enable during sleep mode */
#define CRM_APB2LPEN_ADC3LPEN               (1 << 10) /* ADC3 interface clock enable during sleep mode */
#define CRM_APB2LPEN_SPI1LPEN               (1 << 12) /* SPI1 clock enable during sleep mode */
#define CRM_APB2LPEN_SPI4LPEN               (1 << 13) /* SPI4 clock enable during sleep mode */
#define CRM_APB2LPEN_SCFGLPEN               (1 << 14) /* SCFG clock enable during sleep mode */
#define CRM_APB2LPEN_TMR9LPEN               (1 << 16) /* Timer9 clock enable during sleep mode */
#define CRM_APB2LPEN_TMR10LPEN              (1 << 17) /* Timer10 clock enable during sleep mode */
#define CRM_APB2LPEN_TMR11LPEN              (1 << 18) /* Timer11 clock enable during sleep mode */
#define CRM_APB2LPEN_TMR20LPEN              (1 << 20) /* Timer20 clock enable during sleep mode */
#define CRM_APB2LPEN_ACCLPEN                (1 << 29) /* ACC clock enable during sleep mode */

/* Backup domain control register */

#define CRM_BPDC_LEXTEN                     (1 << 0) /* External low-speed oscillator enable */
#define CRM_BPDC_LEXTSTBL                   (1 << 1) /* External low-speed oscillator stable */
#define CRM_BPDC_LEXTBYPS

#define CRM_BPDC_ERTCSEL_SHIFT              (8) /* ERTC clock source selection */
#define CRM_BPDC_ERTCSEL_MASK               (3 << CRM_BPDC_ERTCSEL_SHIFT)
#  define CRM_BPDC_ERTCSEL_LEXT             (1 << CRM_BPDC_ERTCSEL_SHIFT) /* Select LEXT */
#  define CRM_BPDC_ERTCSEL_LICK             (2 << CRM_BPDC_ERTCSEL_SHIFT) /* Select LICK */
#  define CRM_BPDC_ERTCSEL_HEXT             (3 << CRM_BPDC_ERTCSEL_SHIFT) /* Select HEXT */

#define CRM_BPDC_ERTCEN                     (1 << 15) /* ERTC clock enable */
#define CRM_BPDC_BPDRST                     (1 << 16) /* Battery powered domain software reset */

/* Control/status register */

#define CRM_CTRLSTS_LICKEN                  (1 << 0)  /* LICK enable */
#define CRM_CTRLSTS_LICKSTBL                (1 << 1)  /* LICK stable */
#define CRM_CTRLSTS_RSTFC                   (1 << 24) /* Reset flag clear */
#define CRM_CTRLSTS_NRSTF                   (1 << 26) /* NRST reset flag */
#define CRM_CTRLSTS_PORRSTF                 (1 << 27) /* POR/LVR reset flag */
#define CRM_CTRLSTS_SWRSTF                  (1 << 28) /* Software reset flag */
#define CRM_CTRLSTS_WDTRSTF                 (1 << 29) /* WDT reset flag */
#define CRM_CTRLSTS_WWDTRSTF                (1 << 30) /* WWDT reset flag */
#define CRM_CTRLSTS_LPRSTF                  (1 << 31) /* Low-power reset flag */

/* Misc1 register */

#define CRM_MISC1_HICKCAL_KEY               (255 << 0)/* HICK calibration key */
#define CRM_MISC1_HICKDIV                   (1 << 12) /* HICK 6 divider selection */
#define CRM_MISC1_HICK_TO_USB               (1 << 13) /* USB 48MHz clock source select */
#define CRM_MISC1_HICK_TO_SCLK              (1 << 14) /* HICK as system clock frequency select */

#define CRM_MISC1_CLKOUT2_SEL2_SHIFT        (16) /* Clock output2 sel2 */
#define CRM_MISC1_CLKOUT2_SEL2_MASK         (15 << CRM_MISC1_CLKOUT2_SEL2_SHIFT) 
#  define CRM_MISC1_CLKOUT2_SEL2_USB        (0 << CRM_MISC1_CLKOUT2_SEL2_SHIFT) /* Select USB output */
#  define CRM_MISC1_CLKOUT2_SEL2_ADC        (1 << CRM_MISC1_CLKOUT2_SEL2_SHIFT) /* Select ADC output */
#  define CRM_MISC1_CLKOUT2_SEL2_HICK       (2 << CRM_MISC1_CLKOUT2_SEL2_SHIFT) /* Select HICK output */
#  define CRM_MISC1_CLKOUT2_SEL2_LICK       (3 << CRM_MISC1_CLKOUT2_SEL2_SHIFT) /* Select LICK output */
#  define CRM_MISC1_CLKOUT2_SEL2_LEXT       (4 << CRM_MISC1_CLKOUT2_SEL2_SHIFT) /* Select LEXT output */

#define CRM_MISC1_CLKOUT1DIV2_SHIFT         (24) /* Clock output1 division2 */
#define CRM_MISC1_CLKOUT1DIV2_MASK          (15 << CRM_MISC1_CLKOUT1DIV2_SHIFT)
#  define CRM_MISC1_CLKOUT1DIV2_NONE        (7 << CRM_MISC1_CLKOUT1DIV2_SHIFT)  /* Clock output1 division2 none */
#  define CRM_MISC1_CLKOUT1DIV2_2           (8 << CRM_MISC1_CLKOUT1DIV2_SHIFT)  /* Clock output1 division2 2 */
#  define CRM_MISC1_CLKOUT1DIV2_4           (9 << CRM_MISC1_CLKOUT1DIV2_SHIFT)  /* Clock output1 division2  4*/
#  define CRM_MISC1_CLKOUT1DIV2_8           (10 << CRM_MISC1_CLKOUT1DIV2_SHIFT) /* Clock output1 division2 8 */
#  define CRM_MISC1_CLKOUT1DIV2_16          (11 << CRM_MISC1_CLKOUT1DIV2_SHIFT) /* Clock output1 division2 16 */
#  define CRM_MISC1_CLKOUT1DIV2_64          (12 << CRM_MISC1_CLKOUT1DIV2_SHIFT) /* Clock output1 division2 64 */
#  define CRM_MISC1_CLKOUT1DIV2_128         (13 << CRM_MISC1_CLKOUT1DIV2_SHIFT) /* Clock output1 division2 128 */
#  define CRM_MISC1_CLKOUT1DIV2_256         (14 << CRM_MISC1_CLKOUT1DIV2_SHIFT) /* Clock output1 division2 256 */
#  define CRM_MISC1_CLKOUT1DIV2_512         (15 << CRM_MISC1_CLKOUT1DIV2_SHIFT) /* Clock output1 division2 512 */

#define CRM_MISC1_CLKOUT2DIV2_SHIFT         (28) /* Clock output2 division2 */
#define CRM_MISC1_CLKOUT2DIV2_MASK          (15 << CRM_MISC1_CLKOUT2DIV2_SHIFT)
#  define CRM_MISC1_CLKOUT2DIV2_NONE        (7 << CRM_MISC1_CLKOUT2DIV2_SHIFT)  /* Clock output2 division2 none */
#  define CRM_MISC1_CLKOUT2DIV2_2           (8 << CRM_MISC1_CLKOUT2DIV2_SHIFT)  /* Clock output2 division2 2 */
#  define CRM_MISC1_CLKOUT2DIV2_4           (9 << CRM_MISC1_CLKOUT2DIV2_SHIFT)  /* Clock output2 division2  4*/
#  define CRM_MISC1_CLKOUT2DIV2_8           (10 << CRM_MISC1_CLKOUT2DIV2_SHIFT) /* Clock output2 division2 8 */
#  define CRM_MISC1_CLKOUT2DIV2_16          (11 << CRM_MISC1_CLKOUT2DIV2_SHIFT) /* Clock output2 division2 16 */
#  define CRM_MISC1_CLKOUT2DIV2_64          (12 << CRM_MISC1_CLKOUT2DIV2_SHIFT) /* Clock output2 division2 64 */
#  define CRM_MISC1_CLKOUT2DIV2_128         (13 << CRM_MISC1_CLKOUT2DIV2_SHIFT) /* Clock output2 division2 128 */
#  define CRM_MISC1_CLKOUT2DIV2_256         (14 << CRM_MISC1_CLKOUT2DIV2_SHIFT) /* Clock output2 division2 256 */
#  define CRM_MISC1_CLKOUT2DIV2_512         (15 << CRM_MISC1_CLKOUT2DIV2_SHIFT) /* Clock output2 division2 512 */

/* Misc1 register */

#define CRM_MISC2_AUTO_STEP_EN_SHIFT        (4) /* auto step system clock switch enable */
#define CRM_MISC2_AUTO_STEP_EN_MASK         (3 << CRM_MISC2_AUTO_STEP_EN_SHIFT)
#  define CRM_MISC2_AUTO_STEP_EN_DISABLE    (0 << CRM_MISC2_AUTO_STEP_EN_SHIFT) /* disanle */
#  define CRM_MISC2_AUTO_STEP_EN_ENABLE     (3 << CRM_MISC2_AUTO_STEP_EN_SHIFT) /* enable */

#define CRM_MISC2_CLK1_TO_TMR               (1 << 8) /* CLKOUT1 internal connect to timer 10 channel 1 */
#define CRM_MISC2_EMAC_PPS_SEL              (1 << 9) /* Ethernet pulse width select */

#define CRM_MISC2_USBDIV_SHIFT              (12) /* USB division */
#define CRM_MISC2_USBDIV_MASK               (15 << CRM_MISC2_USBDIV_SHIFT)
#  define CRM_MISC2_USBDIV_1P5              (0 << CRM_MISC2_USBDIV_SHIFT)  /* Div 1.5 */
#  define CRM_MISC2_USBDIV_1P0              (1 << CRM_MISC2_USBDIV_SHIFT)  /* Div 1 */
#  define CRM_MISC2_USBDIV_2P5              (2 << CRM_MISC2_USBDIV_SHIFT)  /* Div 2.5 */
#  define CRM_MISC2_USBDIV_2P0              (3 << CRM_MISC2_USBDIV_SHIFT)  /* Div 2 */
#  define CRM_MISC2_USBDIV_3P5              (4 << CRM_MISC2_USBDIV_SHIFT)  /* Div 3.5 */
#  define CRM_MISC2_USBDIV_3P0              (5 << CRM_MISC2_USBDIV_SHIFT)  /* Div 3 */
#  define CRM_MISC2_USBDIV_4P5              (6 << CRM_MISC2_USBDIV_SHIFT)  /* Div 4.5 */
#  define CRM_MISC2_USBDIV_4P0              (7 << CRM_MISC2_USBDIV_SHIFT)  /* Div 4 */
#  define CRM_MISC2_USBDIV_5P5              (8 << CRM_MISC2_USBDIV_SHIFT)  /* Div 5.5 */
#  define CRM_MISC2_USBDIV_5P0              (9 << CRM_MISC2_USBDIV_SHIFT)  /* Div 5 */
#  define CRM_MISC2_USBDIV_6P5              (10 << CRM_MISC2_USBDIV_SHIFT) /* Div 6.5 */
#  define CRM_MISC2_USBDIV_6P0              (11 << CRM_MISC2_USBDIV_SHIFT) /* Div 6 */
#  define CRM_MISC2_USBDIV_7P0              (12 << CRM_MISC2_USBDIV_SHIFT) /* Div 7 */

#endif /* __ARCH_ARM_SRC_AT32_HARDWARE_AT32F43XXX_RCC_H */
