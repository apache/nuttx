/****************************************************************************
 * arch/arm/src/gd32f4/hardware/gd32f4xx_rcu.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_RCU_H
#define __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_RCU_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define GD32_RCU_CTL_OFFSET            0x0000  /* Control register offset */
#define GD32_RCU_PLL_OFFSET            0x0004  /* PLL register offset */
#define GD32_RCU_CFG0_OFFSET           0x0008  /* Clock configuration register offset */
#define GD32_RCU_INT_OFFSET            0x000c  /* Clock interrupt register offset */
#define GD32_RCU_AHB1RST_OFFSET        0x0010  /* AHB1 reset register offset */
#define GD32_RCU_AHB2RST_OFFSET        0x0014  /* AHB2 reset register offset */
#define GD32_RCU_AHB3RST_OFFSET        0x0018  /* AHB3 reset register offset */
#define GD32_RCU_APB1RST_OFFSET        0x0020  /* APB1 reset register offset */
#define GD32_RCU_APB2RST_OFFSET        0x0024  /* APB2 reset register offset */
#define GD32_RCU_AHB1EN_OFFSET         0x0030  /* AHB1 enable register offset */
#define GD32_RCU_AHB2EN_OFFSET         0x0034  /* AHB2 enable register offset */
#define GD32_RCU_AHB3EN_OFFSET         0x0038  /* AHB3 enable register offset */
#define GD32_RCU_APB1EN_OFFSET         0x0040  /* APB1 enable register offset */
#define GD32_RCU_APB2EN_OFFSET         0x0044  /* APB2 enable register offset */
#define GD32_RCU_AHB1SPEN_OFFSET       0x0050  /* AHB1 sleep mode enable register offset */
#define GD32_RCU_AHB2SPEN_OFFSET       0x0054  /* AHB2 sleep mode enable register offset */
#define GD32_RCU_AHB3SPEN_OFFSET       0x0058  /* AHB3 sleep mode enable register offset */
#define GD32_RCU_APB1SPEN_OFFSET       0x0060  /* APB1 sleep mode enable register offset */
#define GD32_RCU_APB2SPEN_OFFSET       0x0064  /* APB2 sleep mode enable register offset */
#define GD32_RCU_BDCTL_OFFSET          0x0070  /* Backup domain control register offset */
#define GD32_RCU_RSTSCK_OFFSET         0x0074  /* Reset source / clock register offset */
#define GD32_RCU_PLLSSCTL_OFFSET       0x0080  /* PLL clock spread spectrum control register offset*/
#define GD32_RCU_PLLI2S_OFFSET         0x0084  /* PLLI2S register offset */
#define GD32_RCU_PLLSAI_OFFSET         0x0088  /* PLLSAI register offset */
#define GD32_RCU_CFG1_OFFSET           0x008c  /* Clock configuration register 1 */
#define GD32_RCU_ADDCTL_OFFSET         0x00c0  /* Additional clock control register */
#define GD32_RCU_ADDINT_OFFSET         0x00cc  /* Additional clock interrupt register offset */
#define GD32_RCU_ADDAPB1RST_OFFSET     0x00e0  /* APB1 additional reset register offset */
#define GD32_RCU_ADDAPB1EN_OFFSET      0x00e4  /* APB1 additional enable register offset */
#define GD32_RCU_ADDAPB1SPEN_OFFSET    0x00e8  /* APB1 additional sleep mode enable register offset */
#define GD32_RCU_VKEY_OFFSET           0x0100  /* Voltage key register offset */
#define GD32_RCU_DSV_OFFSET            0x0134  /* Deep-sleep mode voltage register offset */

/* Register Addresses *******************************************************/

#define GD32_RCU_CTL                   (GD32_RCU_BASE+GD32_RCU_CTL_OFFSET)          /* Control register */
#define GD32_RCU_PLL                   (GD32_RCU_BASE+GD32_RCU_PLL_OFFSET)          /* PLL register */
#define GD32_RCU_CFG0                  (GD32_RCU_BASE+GD32_RCU_CFG0_OFFSET)         /* Clock configuration register */
#define GD32_RCU_INT                   (GD32_RCU_BASE+GD32_RCU_INT_OFFSET)          /* Clock interrupt register */
#define GD32_RCU_AHB1RST               (GD32_RCU_BASE+GD32_RCU_AHB1RST_OFFSET)      /* AHB1 reset register */
#define GD32_RCU_AHB2RST               (GD32_RCU_BASE+GD32_RCU_AHB2RST_OFFSET)      /* AHB2 reset register */
#define GD32_RCU_AHB3RST               (GD32_RCU_BASE+GD32_RCU_AHB3RST_OFFSET)      /* AHB3 reset register */
#define GD32_RCU_APB1RST               (GD32_RCU_BASE+GD32_RCU_APB1RST_OFFSET)      /* APB1 reset register */
#define GD32_RCU_APB2RST               (GD32_RCU_BASE+GD32_RCU_APB2RST_OFFSET)      /* APB2 reset register */
#define GD32_RCU_AHB1EN                (GD32_RCU_BASE+GD32_RCU_AHB1EN_OFFSET)       /* AHB1 enable register */
#define GD32_RCU_AHB2EN                (GD32_RCU_BASE+GD32_RCU_AHB2EN_OFFSET)       /* AHB2 enable register */
#define GD32_RCU_AHB3EN                (GD32_RCU_BASE+GD32_RCU_AHB3EN_OFFSET)       /* AHB3 enable register */
#define GD32_RCU_APB1EN                (GD32_RCU_BASE+GD32_RCU_APB1EN_OFFSET)       /* APB1 enable register */
#define GD32_RCU_APB2EN                (GD32_RCU_BASE+GD32_RCU_APB2EN_OFFSET)       /* APB2 enable register */
#define GD32_RCU_AHB1SPEN              (GD32_RCU_BASE+GD32_RCU_AHB1SPEN_OFFSET)     /* AHB1 sleep mode enable register */
#define GD32_RCU_AHB2SPEN              (GD32_RCU_BASE+GD32_RCU_AHB2SPEN_OFFSET)     /* AHB2 sleep mode enable register */
#define GD32_RCU_AHB3SPEN              (GD32_RCU_BASE+GD32_RCU_AHB3SPEN_OFFSET)     /* AHB3 sleep mode enable register */
#define GD32_RCU_APB1SPEN              (GD32_RCU_BASE+GD32_RCU_APB1SPEN_OFFSET)     /* APB1 sleep mode enable register */
#define GD32_RCU_APB2SPEN              (GD32_RCU_BASE+GD32_RCU_APB2SPEN_OFFSET)     /* APB2 sleep mode enable register */
#define GD32_RCU_BDCTL                 (GD32_RCU_BASE+GD32_RCU_BDCTL_OFFSET)        /* Backup domain control register */
#define GD32_RCU_RSTSCK                (GD32_RCU_BASE+GD32_RCU_RSTSCK_OFFSET)       /* Reset source / clock register */
#define GD32_RCU_PLLSSCTL              (GD32_RCU_BASE+GD32_RCU_PLLSSCTL_OFFSET)     /* PLL clock spread spectrum control register*/
#define GD32_RCU_PLLI2S                (GD32_RCU_BASE+GD32_RCU_PLLI2S_OFFSET)       /* PLLI2S register */
#define GD32_RCU_PLLSAI                (GD32_RCU_BASE+GD32_RCU_PLLSAI_OFFSET)       /* PLLSAI register */
#define GD32_RCU_CFG1                  (GD32_RCU_BASE+GD32_RCU_CFG1_OFFSET)         /* Clock configuration register 1 */
#define GD32_RCU_ADDCTL                (GD32_RCU_BASE+GD32_RCU_ADDCTL_OFFSET)       /* Additional clock control register */
#define GD32_RCU_ADDINT                (GD32_RCU_BASE+GD32_RCU_ADDINT_OFFSET)       /* Additional clock interrupt register */
#define GD32_RCU_ADDAPB1RST            (GD32_RCU_BASE+GD32_RCU_ADDAPB1RST_OFFSET)   /* APB1 additional reset register */
#define GD32_RCU_ADDAPB1EN             (GD32_RCU_BASE+GD32_RCU_ADDAPB1EN_OFFSET)    /* APB1 additional enable register */
#define GD32_RCU_ADDAPB1SPEN           (GD32_RCU_BASE+GD32_RCU_ADDAPB1SPEN_OFFSET)  /* APB1 additional sleep mode enable register */
#define GD32_RCU_VKEY                  (GD32_RCU_BASE+GD32_RCU_VKEY_OFFSET)         /* Voltage key register */
#define GD32_RCU_DSV                   (GD32_RCU_BASE+GD32_RCU_DSV_OFFSET)          /* Deep-sleep mode voltage register */

/* Register Bitfield Definitions ********************************************/

/* Control register */

#define RCU_CTL_IRC16MEN               (1 << 0)        /* Bit 0: Internal high speed oscillator enable */
#define RCU_CTL_IRC16MSTB              (1 << 1)        /* Bit 1: IRC16M high speed internal oscillator stabilization flag */
#define RCU_CTL_IRC16MADJ_SHIFT        (3)             /* Bits 7-3: High speed internal oscillator clock trim adjust value */
#define RCU_CTL_IRC16MADJ_MASK         (0x1f << RCU_CTL_IRC16MADJ_SHIFT)
#  define CU_CTL_IRC16MADJ(n)          ((n) << RCU_CTL_IRC16MADJ_SHIFT)
#define RCU_CTL_IRC16MCALIB_SHIFT      (8)             /* Bits 15-8: High speed internal oscillator calibration value */
#define RCU_CTL_IRC16MCAL_MASK         (0xff << RCU_CTL_IRC16MCALIB_SHIFT)
#  define RCU_CTL_IRC16MCAL(n)         ((n) << RCU_CTL_IRC16MCALIB_SHIFT)
#define RCU_CTL_HXTALEN                (1 << 16)       /* Bit 16: External high speed oscillator enable */
#define RCU_CTL_HXTALSTB               (1 << 17)       /* Bit 17: External crystal oscillator clock stabilization flag */
#define RCU_CTL_HXTALBPS               (1 << 18)       /* Bit 18: External crystal oscillator clock bypass mode enable */
#define RCU_CTL_CKMEN                  (1 << 19)       /* Bit 19: Clock Security System enable */
#define RCU_CTL_PLLEN                  (1 << 24)       /* Bit 24: PLL enable */
#define RCU_CTL_PLLSTB                 (1 << 25)       /* Bit 25: PLL clock Stabilization flag */
#define RCU_CTL_PLLI2SEN               (1 << 26)       /* Bit 26: PLLI2S enable */
#define RCU_CTL_PLLI2SSTB              (1 << 27)       /* Bit 27: PLLI2S clock Stabilization flag */
#define RCU_CTL_PLLSAIEN               (1 << 28)       /* Bit 28: PLLSAI enable */
#define RCU_CTL_PLLSAISTB              (1 << 29)       /* Bit 29: PLLSAI clock Stabilization flag */

/* PLL register */

#define RCU_PLL_PLLPSC_SHIFT           (0)                                 /* Bits 0-5: The PLL VCO source clock prescale */
#define RCU_PLL_PLLPSC_MASK            (0x3f << RCU_PLL_PLLPSC_SHIFT)
#  define RCU_PLL_PLLPSC(n)            ((n) << RCU_PLL_PLLPSC_SHIFT)       /* CK_PLLSRC/n, n = 2..63 */

#define RCU_PLL_PLLN_SHIFT             (6)                                 /* Bits 6-14: The PLL VCO clock multi factor */
#define RCU_PLL_PLLN_MASK              (0x1ff << RCU_PLL_PLLN_SHIFT)
#  define RCU_PLL_PLLN(n)              ((n) << RCU_PLL_PLLN_SHIFT)         /* CK_PLLVCO = CK_PLLVCOSRC*n, n = 64..500 */

#define RCU_PLL_PLLP_SHIFT             (16)                                /* Bits 16-17: The PLLP output frequency division factor from PLL VCO clock */
#define RCU_PLL_PLLP_MASK              (3 << RCU_PLL_PLLP_SHIFT)
#  define RCU_PLL_PLLP(n)              ((((n)>>1)-1)<< RCU_PLL_PLLP_SHIFT) /* CK_PLLP = CK_PLLVCO/n, n=2,4,6,8 */

#define RCU_PLL_PLLSEL                 (1 << 22)                           /* PLL Clock Source Selection */
#  define RCU_PLL_PLLSEL_IRC16M        (0)
#  define RCU_PLL_PLLSEL_HXTAL         (1 << 22)

#define RCU_PLL_PLLQ_SHIFT             (24)                                /* Bits 24-27: The PLL Q output frequency division factor from PLL VCO clock */
#define RCU_PLL_PLLQ_MASK              (15 << RCU_PLL_PLLQ_SHIFT)
#  define RCU_PLL_PLLQ(n)              ((n) << RCU_PLL_PLLQ_SHIFT)         /* CK_PLLQ = CK_PLLVCO/n, n=2..15 */

/* Clock configuration register 0 */

#define RCU_CFG0_SCS_SHIFT             (0)                     /* Bits 0-1: System clock source select */
#define RCU_CFG0_SCS_MASK              (3 << RCU_CFG0_SCS_SHIFT)
#  define RCU_CFG0_SCS(n)              ((n) << RCU_CFG0_SCS_SHIFT)
#  define RCU_CFG0_SCS_IRC16M          RCU_CFG0_SCS(0)         /* System clock source select IRC16M */
#  define RCU_CFG0_SCS_HXTAL           RCU_CFG0_SCS(1)         /* System clock source select HXTAL */
#  define RCU_CFG0_SCS_PLLP            RCU_CFG0_SCS(2)         /* System clock source select PLLP */

#define RCU_CFG0_SCSS_SHIFT            (2)                     /* Bits 2-3: System clock source select status */
#define RCU_CFG0_SCSS_MASK             (3 << RCU_CFG0_SCSS_SHIFT)
#  define RCU_CFG0_SCSS(n)             ((n) << RCU_CFG0_SCSS_SHIFT)
#  define RCU_CFG0_SCSS_IRC16M         RCU_CFG0_SCSS(0)        /* System clock source select IRC16M */
#  define RCU_CFG0_SCSS_HXTAL          RCU_CFG0_SCSS(1)        /* System clock source select HXTAL */
#  define RCU_CFG0_SCSS_PLLP           RCU_CFG0_SCSS(2)        /* System clock source select PLLP */

#define RCU_CFG0_AHBPSC_SHIFT            (4)                   /* Bits 4-7: AHB prescaler selection */
#define RCU_CFG0_AHBPSC_MASK             (0x0f << RCU_CFG0_AHBPSC_SHIFT)
#  define RCU_CFG0_AHBPSC(n)             ((n) << RCU_CFG0_AHBPSC_SHIFT)
#  define RCU_CFG0_AHBPSC_CKSYS_DIV1     RCU_CFG0_AHBPSC(0)    /* AHB prescaler select CK_SYS, n=0..7 */
#  define RCU_CFG0_AHBPSC_CKSYS_DIV2     RCU_CFG0_AHBPSC(8)    /* AHB prescaler select CK_SYS/2 */
#  define RCU_CFG0_AHBPSC_CKSYS_DIV4     RCU_CFG0_AHBPSC(9)    /* AHB prescaler select CK_SYS/4 */
#  define RCU_CFG0_AHBPSC_CKSYS_DIV8     RCU_CFG0_AHBPSC(10)   /* AHB prescaler select CK_SYS/8 */
#  define RCU_CFG0_AHBPSC_CKSYS_DIV16    RCU_CFG0_AHBPSC(11)   /* AHB prescaler select CK_SYS/16 */
#  define RCU_CFG0_AHBPSC_CKSYS_DIV64    RCU_CFG0_AHBPSC(12)   /* AHB prescaler select CK_SYS/64 */
#  define RCU_CFG0_AHBPSC_CKSYS_DIV128   RCU_CFG0_AHBPSC(13)   /* AHB prescaler select CK_SYS/128 */
#  define RCU_CFG0_AHBPSC_CKSYS_DIV256   RCU_CFG0_AHBPSC(14)   /* AHB prescaler select CK_SYS/256 */
#  define RCU_CFG0_AHBPSC_CKSYS_DIV512   RCU_CFG0_AHBPSC(15)   /* AHB prescaler select CK_SYS/512 */

#define RCU_CFG0_APB1PSC_SHIFT           (10)                  /* Bits 10-12: APB1 prescaler selection */
#define RCU_CFG0_APB1PSC_MASK            (7 << RCU_CFG0_APB1PSC_SHIFT)
#  define RCU_CFG0_APB1PSC(n)            ((n) << RCU_CFG0_APB1PSC_SHIFT)
#  define RCU_CFG0_APB1PSC_CKAHB_DIV1    RCU_CFG0_APB1PSC(0)   /* APB1 prescaler select CK_AHB, n=0..3 */
#  define RCU_CFG0_APB1PSC_CKAHB_DIV2    RCU_CFG0_APB1PSC(4)   /* APB1 prescaler select CK_AHB/2 */
#  define RCU_CFG0_APB1PSC_CKAHB_DIV4    RCU_CFG0_APB1PSC(5)   /* APB1 prescaler select CK_AHB/4 */
#  define RCU_CFG0_APB1PSC_CKAHB_DIV8    RCU_CFG0_APB1PSC(6)   /* APB1 prescaler select CK_AHB/8 */
#  define RCU_CFG0_APB1PSC_CKAHB_DIV16   RCU_CFG0_APB1PSC(7)   /* APB1 prescaler select CK_AHB/16 */

#define RCU_CFG0_APB2PSC_SHIFT           (13)                  /* Bits 13-15: APB2 prescaler selection */
#define RCU_CFG0_APB2PSC_MASK            (7 << RCU_CFG0_APB2PSC_SHIFT)
#  define RCU_CFG0_APB2PSC(n)            ((n) << RCU_CFG0_APB2PSC_SHIFT)
#  define RCU_CFG0_APB2PSC_CKAHB_DIV1    RCU_CFG0_APB2PSC(0)   /* APB2 prescaler select CK_AHB, n=0..3 */
#  define RCU_CFG0_APB2PSC_CKAHB_DIV2    RCU_CFG0_APB2PSC(4)   /* APB2 prescaler select CK_AHB/2 */
#  define RCU_CFG0_APB2PSC_CKAHB_DIV4    RCU_CFG0_APB2PSC(5)   /* APB2 prescaler select CK_AHB/4 */
#  define RCU_CFG0_APB2PSC_CKAHB_DIV8    RCU_CFG0_APB2PSC(6)   /* APB2 prescaler select CK_AHB/8 */
#  define RCU_CFG0_APB2PSC_CKAHB_DIV16   RCU_CFG0_APB2PSC(7)   /* APB2 prescaler select CK_AHB/16 */

#define RCU_CFG0_RTCDIV_SHIFT            (16)                  /* Bits 16-20: RTC clock divider factor  */
#define RCU_CFG0_RTCDIV_MASK             (31 << RCU_CFG0_RTCDIV_SHIFT)
#  define RCU_CFG0_RTCDIV(n)             ((n) << RCU_CFG0_RTCDIV_SHIFT)
#  define RCU_CFG0_RTC_HXTAL_NONE        RCU_CFG0_RTCDIV(0)    /* No clock for RTC, n=0,1 */
#  define RCU_CFG0_RTC_HXTAL_DIV2        RCU_CFG0_RTCDIV(2)    /* RTCDIV clock select CK_HXTAL/2 */
#  define RCU_CFG0_RTC_HXTAL_DIV3        RCU_CFG0_RTCDIV(3)    /* RTCDIV clock select CK_HXTAL/3 */
#  define RCU_CFG0_RTC_HXTAL_DIV4        RCU_CFG0_RTCDIV(4)    /* RTCDIV clock select CK_HXTAL/4 */
#  define RCU_CFG0_RTC_HXTAL_DIV5        RCU_CFG0_RTCDIV(5)    /* RTCDIV clock select CK_HXTAL/5 */
#  define RCU_CFG0_RTC_HXTAL_DIV6        RCU_CFG0_RTCDIV(6)    /* RTCDIV clock select CK_HXTAL/6 */
#  define RCU_CFG0_RTC_HXTAL_DIV7        RCU_CFG0_RTCDIV(7)    /* RTCDIV clock select CK_HXTAL/7 */
#  define RCU_CFG0_RTC_HXTAL_DIV8        RCU_CFG0_RTCDIV(8)    /* RTCDIV clock select CK_HXTAL/8 */
#  define RCU_CFG0_RTC_HXTAL_DIV9        RCU_CFG0_RTCDIV(9)    /* RTCDIV clock select CK_HXTAL/9 */
#  define RCU_CFG0_RTC_HXTAL_DIV10       RCU_CFG0_RTCDIV(10)   /* RTCDIV clock select CK_HXTAL/10 */
#  define RCU_CFG0_RTC_HXTAL_DIV11       RCU_CFG0_RTCDIV(11)   /* RTCDIV clock select CK_HXTAL/11 */
#  define RCU_CFG0_RTC_HXTAL_DIV12       RCU_CFG0_RTCDIV(12)   /* RTCDIV clock select CK_HXTAL/12 */
#  define RCU_CFG0_RTC_HXTAL_DIV13       RCU_CFG0_RTCDIV(13)   /* RTCDIV clock select CK_HXTAL/13 */
#  define RCU_CFG0_RTC_HXTAL_DIV14       RCU_CFG0_RTCDIV(14)   /* RTCDIV clock select CK_HXTAL/14 */
#  define RCU_CFG0_RTC_HXTAL_DIV15       RCU_CFG0_RTCDIV(15)   /* RTCDIV clock select CK_HXTAL/15 */
#  define RCU_CFG0_RTC_HXTAL_DIV16       RCU_CFG0_RTCDIV(16)   /* RTCDIV clock select CK_HXTAL/16 */
#  define RCU_CFG0_RTC_HXTAL_DIV17       RCU_CFG0_RTCDIV(17)   /* RTCDIV clock select CK_HXTAL/17 */
#  define RCU_CFG0_RTC_HXTAL_DIV18       RCU_CFG0_RTCDIV(18)   /* RTCDIV clock select CK_HXTAL/18 */
#  define RCU_CFG0_RTC_HXTAL_DIV19       RCU_CFG0_RTCDIV(19)   /* RTCDIV clock select CK_HXTAL/19 */
#  define RCU_CFG0_RTC_HXTAL_DIV20       RCU_CFG0_RTCDIV(20)   /* RTCDIV clock select CK_HXTAL/20 */
#  define RCU_CFG0_RTC_HXTAL_DIV21       RCU_CFG0_RTCDIV(21)   /* RTCDIV clock select CK_HXTAL/21 */
#  define RCU_CFG0_RTC_HXTAL_DIV22       RCU_CFG0_RTCDIV(22)   /* RTCDIV clock select CK_HXTAL/22 */
#  define RCU_CFG0_RTC_HXTAL_DIV23       RCU_CFG0_RTCDIV(23)   /* RTCDIV clock select CK_HXTAL/23 */
#  define RCU_CFG0_RTC_HXTAL_DIV24       RCU_CFG0_RTCDIV(24)   /* RTCDIV clock select CK_HXTAL/24 */
#  define RCU_CFG0_RTC_HXTAL_DIV25       RCU_CFG0_RTCDIV(25)   /* RTCDIV clock select CK_HXTAL/25 */
#  define RCU_CFG0_RTC_HXTAL_DIV26       RCU_CFG0_RTCDIV(26)   /* RTCDIV clock select CK_HXTAL/26 */
#  define RCU_CFG0_RTC_HXTAL_DIV27       RCU_CFG0_RTCDIV(27)   /* RTCDIV clock select CK_HXTAL/27 */
#  define RCU_CFG0_RTC_HXTAL_DIV28       RCU_CFG0_RTCDIV(28)   /* RTCDIV clock select CK_HXTAL/28 */
#  define RCU_CFG0_RTC_HXTAL_DIV29       RCU_CFG0_RTCDIV(29)   /* RTCDIV clock select CK_HXTAL/29 */
#  define RCU_CFG0_RTC_HXTAL_DIV30       RCU_CFG0_RTCDIV(30)   /* RTCDIV clock select CK_HXTAL/30 */
#  define RCU_CFG0_RTC_HXTAL_DIV31       RCU_CFG0_RTCDIV(31)   /* RTCDIV clock select CK_HXTAL/31 */

#define RCU_CFG0_CKOUT0SEL_SHIFT         (21)                  /* Bits 21-22: CKOUT0 Clock source selection */
#define RCU_CFG0_CKOUT0SEL_MASK          (3 << RCU_CFG0_CKOUT0SEL_SHIFT)
#  define RCU_CFG0_CKOUT0SEL(n)          ((n) << RCU_CFG0_CKOUT0SEL_SHIFT)
#  define RCU_CFG0_CKOUT0SEL_IRC16M      RCU_CFG0_CKOUT0SEL(0) /* Internal 16M RC oscillator clock selected */
#  define RCU_CFG0_CKOUT0SEL_LXTAL       RCU_CFG0_CKOUT0SEL(1) /* Low speed crystal oscillator clock (LXTAL) selected */
#  define RCU_CFG0_CKOUT0SEL_HXTAL       RCU_CFG0_CKOUT0SEL(2) /* High speed crystal oscillator clock (HXTAL) selected */
#  define RCU_CFG0_CKOUT0SEL_PLLP        RCU_CFG0_CKOUT0SEL(3) /* CK_PLLP clock selected */

#define RCU_CFG0_I2SSEL                  (1 << 23)             /* Bit 23: I2S Clock Source Selection */
#  define RCU_I2SSEL_PLLI2S              (0 << 23)             /* PLLI2S output clock selected as I2S source clock */
#  define RCU_I2SSEL_I2S_CKIN            RCU_CFG0_I2SSEL       /* External I2S_CKIN pin selected as I2S source clock */

#define RCU_CFG0_CKOUT0DIV_SHIFT         (24)                  /* Bits 24-26: The CK_OUT0 divider */
#define RCU_CFG0_CKOUT0DIV_MASK          (7 << RCU_CFG0_CKOUT0DIV_SHIFT)
# define RCU_CFG0_CKOUT0DIV(n)           ((n) << RCU_CFG0_CKOUT0DIV_SHIFT)
#  define RCU_CFG0_CKOUT0_DIV1           RCU_CFG0_CKOUT0DIV(0) /* CK_OUT0 is divided by 1, n=0..3 */
#  define RCU_CFG0_CKOUT0_DIV2           RCU_CFG0_CKOUT0DIV(4) /* CK_OUT0 is divided by 2 */
#  define RCU_CFG0_CKOUT0_DIV3           RCU_CFG0_CKOUT0DIV(5) /* CK_OUT0 is divided by 3 */
#  define RCU_CFG0_CKOUT0_DIV4           RCU_CFG0_CKOUT0DIV(6) /* CK_OUT0 is divided by 4 */
#  define RCU_CFG0_CKOUT0_DIV5           RCU_CFG0_CKOUT0DIV(7) /* CK_OUT0 is divided by 5 */

#define RCU_CFG0_CKOUT1DIV_SHIFT         (27)                  /* Bits 27-29: The CK_OUT1 divider */
#define RCU_CFG0_CKOUT1DIV_MASK          (7 << RCU_CFG0_CKOUT1DIV_SHIFT)
# define RCU_CFG0_CKOUT1DIV(n)           ((n) << RCU_CFG0_CKOUT1DIV_SHIFT)
#  define RCU_CFG0_CKOUT1_DIV1           RCU_CFG0_CKOUT1DIV(0) /* CK_OUT1 is divided by 1, n=0..3 */
#  define RCU_CFG0_CKOUT1_DIV2           RCU_CFG0_CKOUT1DIV(4) /* CK_OUT1 is divided by 2 */
#  define RCU_CFG0_CKOUT1_DIV3           RCU_CFG0_CKOUT1DIV(5) /* CK_OUT1 is divided by 3 */
#  define RCU_CFG0_CKOUT1_DIV4           RCU_CFG0_CKOUT1DIV(6) /* CK_OUT1 is divided by 4 */
#  define RCU_CFG0_CKOUT1_DIV5           RCU_CFG0_CKOUT1DIV(7) /* CK_OUT1 is divided by 5 */

#define RCU_CFG0_CKOUT1SEL_SHIFT         (30)                  /* Bits 30-31: CKOUT1 Clock source selection */
#define RCU_CFG0_CKOUT1SEL_MASK          (3 << RCU_CFG0_CKOUT1SEL_SHIFT)
#  define RCU_CFG0_CKOUT1SEL(n)          ((n) << RCU_CFG0_CKOUT1SEL_SHIFT)
#  define RCU_CFG0_CKOUT1SEL_SYSTEMCLOCK RCU_CFG0_CKOUT1SEL(0) /* System clock selected */
#  define RCU_CFG0_CKOUT1SEL_PLLI2SR     RCU_CFG0_CKOUT1SEL(0) /* CK_PLLI2SR clock selected */
#  define RCU_CFG0_CKOUT1SEL_HXTAL       RCU_CFG0_CKOUT1SEL(0) /* High speed crystal oscillator clock (HXTAL) selected */
#  define RCU_CFG0_CKOUT1SEL_PLLp        RCU_CFG0_CKOUT1SEL(0) /* CK_PLLP clock selected */

/* Clock interrupt register */

#define RCU_INT_IRC32KSTBIF              (1 << 0)              /* Bit 0: IRC32K stabilization interrupt flag */
#define RCU_INT_LXTALSTBIF               (1 << 1)              /* Bit 1: LXTAL stabilization interrupt flag */
#define RCU_INT_IRC16MSTBIF              (1 << 2)              /* Bit 2: IRC16M stabilization interrupt flag */
#define RCU_INT_HXTALSTBIF               (1 << 3)              /* Bit 3: HXTAL stabilization interrupt flag */
#define RCU_INT_PLLSTBIF                 (1 << 4)              /* Bit 4: PLL stabilization interrupt flag */
#define RCU_INT_PLLI2SSTBIF              (1 << 5)              /* Bit 5: PLLI2S stabilization interrupt flag */
#define RCU_INT_PLLSAISTBIF              (1 << 6)              /* Bit 6: PLLSAI stabilization interrupt flag */
#define RCU_INT_CKMIF                    (1 << 7)              /* Bit 7: HXTAL clock stuck interrupt flag */
#define RCU_INT_IRC32KSTBIE              (1 << 8)              /* Bit 8: IRC32K stabilization interrupt enable */
#define RCU_INT_LXTALSTBIE               (1 << 9)              /* Bit 9: LXTAL stabilization interrupt enable */
#define RCU_INT_IRC16MSTBIE              (1 << 10)             /* Bit 10: IRC16M stabilization interrupt enable */
#define RCU_INT_HXTALSTBIE               (1 << 11)             /* Bit 11: HXTAL stabilization interrupt enable */
#define RCU_INT_PLLSTBIE                 (1 << 12)             /* Bit 12: PLL stabilization interrupt enable */
#define RCU_INT_PLLI2SSTBIE              (1 << 13)             /* Bit 13: PLLI2S Stabilization Interrupt Enable */
#define RCU_INT_PLLSAISTBIE              (1 << 14)             /* Bit 14: PLLSAI Stabilization Interrupt Enable */
#define RCU_INT_IRC32KSTBIC              (1 << 16)             /* Bit 16: IRC32K Stabilization Interrupt Clear */
#define RCU_INT_LXTALSTBIC               (1 << 17)             /* Bit 17: LXTAL Stabilization Interrupt Clear */
#define RCU_INT_IRC16MSTBIC              (1 << 18)             /* Bit 18: IRC16M Stabilization Interrupt Clear */
#define RCU_INT_HXTALSTBIC               (1 << 19)             /* Bit 19: HXTAL Stabilization Interrupt Clear */
#define RCU_INT_PLLSTBIC                 (1 << 20)             /* Bit 20: PLL stabilization Interrupt Clear */
#define RCU_INT_PLLI2SSTBIC              (1 << 21)             /* Bit 21: PLLI2S stabilization Interrupt Clear */
#define RCU_INT_PLLSAISTBIC              (1 << 22)             /* Bit 22: PLLSAI stabilization Interrupt Clear */
#define RCU_INT_CKMIC                    (1 << 23)             /* Bit 23: HXTAL Clock Stuck Interrupt Clear */

/* AHB1 reset register */

#define RCU_AHB1RST_PARST                (1 << 0)              /* Bit 0: GPIO port A reset */
#define RCU_AHB1RST_PBRST                (1 << 1)              /* Bit 1: GPIO port B reset */
#define RCU_AHB1RST_PCRST                (1 << 2)              /* Bit 2: GPIO port C reset */
#define RCU_AHB1RST_PDRST                (1 << 3)              /* Bit 3: GPIO port D reset */
#define RCU_AHB1RST_PERST                (1 << 4)              /* Bit 4: GPIO port E reset */
#define RCU_AHB1RST_PFRST                (1 << 5)              /* Bit 5: GPIO port F reset */
#define RCU_AHB1RST_PGRST                (1 << 6)              /* Bit 6: GPIO port G reset */
#define RCU_AHB1RST_PHRST                (1 << 7)              /* Bit 7: GPIO port H reset */
#define RCU_AHB1RST_PIRST                (1 << 8)              /* Bit 8: GPIO port I reset */
#define RCU_AHB1RST_CRCRST               (1 << 12)             /* Bit 12: CRC reset */         
#define RCU_AHB1RST_DMA0RST              (1 << 21)             /* Bit 21: DMA0 reset */
#define RCU_AHB1RST_DMA1RST              (1 << 22)             /* Bit 22: DMA1 reset */
#define RCU_AHB1RST_IPARST               (1 << 23)             /* Bit 23: IPA reset */
#define RCU_AHB1RST_ENETRST              (1 << 25)             /* Bit 25: ENET reset */
#define RCU_AHB1RST_USBHSRST             (1 << 29)             /* Bit 29: USBHS reset */

/* AHB2 reset register */

#define RCU_AHB2RST_DCIRST               (1 << 0)              /* Bit 0: DCI reset */
#define RCU_AHB2RST_TRNGRST              (1 << 6)              /* Bit 6: TRNG reset */
#define RCU_AHB2RST_USBFSRST             (1 << 7)              /* Bit 7: USBFS reset */

/* AHB3 reset register */

#define RCU_AHB3RST_EXMCRST              (1 << 0)              /* Bit 0: EXMC reset */

/* APB1 reset register */

#define RCU_APB1RST_TIMER1RST            (1 << 0)                    /* Bit 0: TIMER1 reset */
#define RCU_APB1RST_TIMER2RST            (1 << 1)                    /* Bit 1: TIMER2 reset */
#define RCU_APB1RST_TIMER3RST            (1 << 2)                    /* Bit 2: TIMER3 reset */
#define RCU_APB1RST_TIMER4RST            (1 << 3)                    /* Bit 3: TIMER4 reset */
#define RCU_APB1RST_TIMER5RST            (1 << 4)                    /* Bit 4: TIMER5 reset */
#define RCU_APB1RST_TIMER6RST            (1 << 5)                    /* Bit 5: TIMER6 reset */
#define RCU_APB1RST_TIMER11RST           (1 << 6)                    /* Bit 6: TIMER11 reset */
#define RCU_APB1RST_TIMER12RST           (1 << 7)                    /* Bit 7: TIMER12 reset */
#define RCU_APB1RST_TIMER13RST           (1 << 8)                    /* Bit 8: TIMER13 reset */
#define RCU_APB1RST_WWDGTRST             (1 << 11)                   /* Bit 11: WWDGT reset */
#define RCU_APB1RST_SPI1RST              (1 << 14)                   /* Bit 14: SPI1 reset */
#define RCU_APB1RST_SPI2RST              (1 << 15)                   /* Bit 15: SPI2 reset */
#define RCU_APB1RST_USART1RST            (1 << 17)                   /* Bit 17: USART1 reset */
#define RCU_APB1RST_USART2RST            (1 << 18)                   /* Bit 18: USART2 reset */
#define RCU_APB1RST_UART3RST             (1 << 19)                   /* Bit 19: UART3 reset */
#define RCU_APB1RST_UART4RST             (1 << 20)                   /* Bit 20: UART4 reset */
#define RCU_APB1RST_I2C0RST              (1 << 21)                   /* Bit 21: I2C0 reset */
#define RCU_APB1RST_I2C1RST              (1 << 22)                   /* Bit 22: I2C1 reset */
#define RCU_APB1RST_I2C2RST              (1 << 23)                   /* Bit 23: I2C2 reset */
#define RCU_APB1RST_CAN0RST              (1 << 25)                   /* Bit 25: CAN0 reset */
#define RCU_APB1RST_CAN1RST              (1 << 26)                   /* Bit 26: CAN1 reset */
#define RCU_APB1RST_PMURST               (1 << 28)                   /* Bit 28: PMU reset */
#define RCU_APB1RST_DACRST               (1 << 29)                   /* Bit 29: DAC reset */
#define RCU_APB1RST_UART6RST             (1 << 30)                   /* Bit 30: UART6 reset */
#define RCU_APB1RST_UART7RST             (1 << 31)                   /* Bit 31: UART7 reset */

/* APB2 reset register */

#define RCU_APB2RST_TIMER0RST            (1 << 0)              /* Bit 0: TIMER0 reset */
#define RCU_APB2RST_TIMER7RST            (1 << 1)              /* Bit 1: TIMER7 reset */
#define RCU_APB2RST_USART0RST            (1 << 4)              /* Bit 4: USART0 reset */
#define RCU_APB2RST_USART5RST            (1 << 5)              /* Bit 5: USART5 reset */
#define RCU_APB2RST_ADCRST               (1 << 8)              /* Bit 8: ADC reset */
#define RCU_APB2RST_SDIORST              (1 << 11)             /* Bit 11: SDIO reset */
#define RCU_APB2RST_SPI0RST              (1 << 12)             /* Bit 13: SPI0 reset */
#define RCU_APB2RST_SPI3RST              (1 << 13)             /* Bit 13: SPI3 reset */
#define RCU_APB2RST_SYSCFGRST            (1 << 14)             /* Bit 14: SYSCFG reset */
#define RCU_APB2RST_TIMER8RST            (1 << 16)             /* Bit 16: TIMER8 reset */
#define RCU_APB2RST_TIMER9RST            (1 << 17)             /* Bit 17: TIMER9 reset */
#define RCU_APB2RST_TIMER10RST           (1 << 18)             /* Bit 18: TIMER10 reset */
#define RCU_APB2RST_SPI4RST              (1 << 20)             /* Bit 20: SPI4 reset */
#define RCU_APB2RST_SPI5RST              (1 << 21)             /* Bit 21: SPI5 reset */
#define RCU_APB2RST_TLIRST               (1 << 26)             /* Bit 26: TLI reset */

/* AHB1 enable register */

#define RCU_AHB1EN_PAEN                  (1 << 0)              /* Bit 0: GPIO port A clock enable */
#define RCU_AHB1EN_PBEN                  (1 << 1)              /* Bit 1: GPIO port B clock enable */
#define RCU_AHB1EN_PCEN                  (1 << 2)              /* Bit 2: GPIO port C clock enable */
#define RCU_AHB1EN_PDEN                  (1 << 3)              /* Bit 3: GPIO port D clock enable */
#define RCU_AHB1EN_PEEN                  (1 << 4)              /* Bit 4: GPIO port E clock enable */
#define RCU_AHB1EN_PFEN                  (1 << 5)              /* Bit 5: GPIO port F clock enable */
#define RCU_AHB1EN_PGEN                  (1 << 6)              /* Bit 6: GPIO port G clock enable */
#define RCU_AHB1EN_PHEN                  (1 << 7)              /* Bit 7: GPIO port H clock enable */
#define RCU_AHB1EN_PIEN                  (1 << 8)              /* Bit 10: GPIO port I clock enable */
#define RCU_AHB1EN_CRCEN                 (1 << 12)             /* Bit 12: CRC clock enable */
#define RCU_AHB1EN_BKPSRAMEN             (1 << 18)             /* Bit 18: BKPSRAM clock enable */
#define RCU_AHB1EN_TCMSRAMEN             (1 << 20)             /* Bit 20: TCMSRAM clock enable */
#define RCU_AHB1EN_DMA0EN                (1 << 21)             /* Bit 21: DMA0 clock enable */
#define RCU_AHB1EN_DMA1EN                (1 << 22)             /* Bit 22: DMA1 clock enable */
#define RCU_AHB1EN_IPAEN                 (1 << 23)             /* Bit 23: IPA clock enable */
#define RCU_AHB1EN_ENETEN                (1 << 25)             /* Bit 25: ENET clock enable */
#define RCU_AHB1EN_ENETTXEN              (1 << 26)             /* Bit 26: Ethernet TX clock enable */
#define RCU_AHB1EN_ENETRXEN              (1 << 27)             /* Bit 27: Ethernet RX clock enable */
#define RCU_AHB1EN_ENETPTPEN             (1 << 28)             /* Bit 28: Ethernet PTP clock enable */
#define RCU_AHB1EN_USBHSEN               (1 << 29)             /* Bit 29: USBHS clock enable */
#define RCU_AHB1EN_USBHSULPIEN           (1 << 30)             /* Bit 30: USBHS ULPI clock enable */

/* AHB2 enable register */

#define RCU_AHB2EN_DCIEN                 (1 << 0)              /* Bit 0: DCI clock enable */
#define RCU_AHB2EN_TRNGEN                (1 << 6)              /* Bit 6: TRNG clock enable */
#define RCU_AHB2EN_USBFSEN               (1 << 7)              /* Bit 7: USBFS clock enable */

/* AHB3 enable register */

#define RCU_AHB3EN_EXMCEN                (1 << 0)              /* Bit 0: EXMC clock enable */

/* APB1 enable register */

#define RCU_APB1EN_TIMER1EN              (1 << 0)              /* Bit 0: TIMER1 clock enable */
#define RCU_APB1EN_TIMER2EN              (1 << 1)              /* Bit 1: TIMER2 clock enable */
#define RCU_APB1EN_TIMER3EN              (1 << 2)              /* Bit 2: TIMER3 clock enable */
#define RCU_APB1EN_TIMER4EN              (1 << 3)              /* Bit 3: TIMER4 clock enable */
#define RCU_APB1EN_TIMER5EN              (1 << 4)              /* Bit 4: TIMER5 clock enable */
#define RCU_APB1EN_TIMER6EN              (1 << 5)              /* Bit 5: TIMER6 clock enable */
#define RCU_APB1EN_TIMER11EN             (1 << 6)              /* Bit 6: TIMER11 clock enable */
#define RCU_APB1EN_TIMER12EN             (1 << 7)              /* Bit 7: TIMER12 clock enable */
#define RCU_APB1EN_TIMER13EN             (1 << 8)              /* Bit 8: TIMER13 clock enable */
#define RCU_APB1EN_WWDGTEN               (1 << 11)             /* Bit 11: WWDGT clock enable */
#define RCU_APB1EN_SPI1EN                (1 << 14)             /* Bit 14: SPI1 clock enable */
#define RCU_APB1EN_SPI2EN                (1 << 15)             /* Bit 15: SPI2 clock enable */
#define RCU_APB1EN_USART1EN              (1 << 17)             /* Bit 17: USART1 clock enable */
#define RCU_APB1EN_USART2EN              (1 << 18)             /* Bit 18: USART2 clock enable */
#define RCU_APB1EN_UART3EN               (1 << 19)             /* Bit 19: UART3 clock enable */
#define RCU_APB1EN_UART4EN               (1 << 20)             /* Bit 20: UART4 clock enable */
#define RCU_APB1EN_I2C0EN                (1 << 21)             /* Bit 21: I2C0 clock enable */
#define RCU_APB1EN_I2C1EN                (1 << 22)             /* Bit 22: I2C1 clock enable */
#define RCU_APB1EN_I2C2EN                (1 << 23)             /* Bit 23: I2C2 clock enable */
#define RCU_APB1EN_CAN0EN                (1 << 25)             /* Bit 25: CAN0 clock enable */
#define RCU_APB1EN_CAN1EN                (1 << 26)             /* Bit 26: CAN1 clock enable */
#define RCU_APB1EN_PMUEN                 (1 << 28)             /* Bit 28: PMU clock enable */
#define RCU_APB1EN_DACEN                 (1 << 29)             /* Bit 29: DAC clock enable */
#define RCU_APB1EN_UART6EN               (1 << 30)             /* Bit 30: UART6 clock enable */
#define RCU_APB1EN_UART7EN               (1 << 31)             /* Bit 31: UART7 clock enable */

/* APB2 enable register */

#define RCU_APB2EN_TIMER0EN              (1 << 0)              /* Bit 1: TIMER0 clock enable */
#define RCU_APB2EN_TIMER7EN              (1 << 1)              /* Bit 1: TIMER7 clock enable */
#define RCU_APB2EN_USART0EN              (1 << 4)              /* Bit 4: USART0 clock enable */
#define RCU_APB2EN_USART5EN              (1 << 5)              /* Bit 5: USART5 clock enable */
#define RCU_APB2EN_ADC0EN                (1 << 8)              /* Bit 8: ADC0 clock enable */
#define RCU_APB2EN_ADC1EN                (1 << 9)              /* Bit 9: ADC1 clock enable */
#define RCU_APB2EN_ADC2EN                (1 << 10)             /* Bit 10: ADC2 clock enable */
#define RCU_APB2EN_SDIOEN                (1 << 11)             /* Bit 11: SDIO clock enable */
#define RCU_APB2EN_SPI0EN                (1 << 12)             /* Bit 12: SPI0 clock enable */
#define RCU_APB2EN_SPI3EN                (1 << 13)             /* Bit 13: SPI3 clock enable */
#define RCU_APB2EN_SYSCFGEN              (1 << 14)             /* Bit 14: SYSCFG clock enable */
#define RCU_APB2EN_TIMER8EN              (1 << 16)             /* Bit 16: TIMER8 clock enable */
#define RCU_APB2EN_TIMER9EN              (1 << 17)             /* Bit 17: TIMER9 clock enable */
#define RCU_APB2EN_TIMER10EN             (1 << 18)             /* Bit 18: TIMER10 clock enable */
#define RCU_APB2EN_SPI4EN                (1 << 20)             /* Bit 20: SPI4 clock enable */
#define RCU_APB2EN_SPI5EN                (1 << 21)             /* Bit 21: SPI5 clock enable */
#define RCU_APB2EN_TLIEN                 (1 << 26)             /* Bit 26: TLI clock enable */

/* AHB1 sleep mode enable register */

#define RCU_AHB1SPEN_PASPEN              (1 << 0)              /* Bit 0: GPIO port A clock enable when sleep mode */
#define RCU_AHB1SPEN_PBSPEN              (1 << 1)              /* Bit 1: GPIO port B clock enable when sleep mode */
#define RCU_AHB1SPEN_PCSPEN              (1 << 2)              /* Bit 2: GPIO port C clock enable when sleep mode */
#define RCU_AHB1SPEN_PDSPEN              (1 << 3)              /* Bit 3: GPIO port D clock enable when sleep mode */
#define RCU_AHB1SPEN_PESPEN              (1 << 4)              /* Bit 4: GPIO port E clock enable when sleep mode */
#define RCU_AHB1SPEN_PFSPEN              (1 << 5)              /* Bit 5: GPIO port F clock enable when sleep mode */
#define RCU_AHB1SPEN_PGSPEN              (1 << 6)              /* Bit 6: GPIO port G clock enable when sleep mode */
#define RCU_AHB1SPEN_PHSPEN              (1 << 7)              /* Bit 7: GPIO port H clock enable when sleep mode */
#define RCU_AHB1SPEN_PISPEN              (1 << 8)              /* Bit 8: GPIO port I clock enable when sleep mode */
#define RCU_AHB1SPEN_CRCSPEN             (1 << 12)             /* Bit 12: CRC clock enable when sleep mode */
#define RCU_AHB1SPEN_FMCSPEN             (1 << 15)             /* Bit 15: FMC clock enable when sleep mode */
#define RCU_AHB1SPEN_SRAM0SPEN           (1 << 16)             /* Bit 16: SRAM0 clock enable when sleep mode */
#define RCU_AHB1SPEN_SRAM1SPEN           (1 << 17)             /* Bit 17: SRAM1 clock enable when sleep mode */
#define RCU_AHB1SPEN_BKPSRAMSPEN         (1 << 18)             /* Bit 18: BKPSRAM clock enable when sleep mode */
#define RCU_AHB1SPEN_SRAM2SPEN           (1 << 19)             /* Bit 19: SRAM2 clock enable when sleep mode */
#define RCU_AHB1SPEN_DMA0SPEN            (1 << 21)             /* Bit 21: DMA0 clock when sleep mode enable */
#define RCU_AHB1SPEN_DMA1SPEN            (1 << 22)             /* Bit 22: DMA1 clock when sleep mode enable */
#define RCU_AHB1SPEN_IPASPEN             (1 << 23)             /* Bit 23: IPA clock enable when sleep mode */
#define RCU_AHB1SPEN_ENETSPEN            (1 << 25)             /* Bit 25: ENET clock enable when sleep mode */
#define RCU_AHB1SPEN_ENETTXSPEN          (1 << 26)             /* Bit 26: Ethernet TX clock enable when sleep mode */
#define RCU_AHB1SPEN_ENETRXSPEN          (1 << 27)             /* Bit 27: Ethernet RX clock enable when sleep mode */
#define RCU_AHB1SPEN_ENETPTPSPEN         (1 << 28)             /* Bit 28: Ethernet PTP clock enable when sleep mode */
#define RCU_AHB1SPEN_USBHSSPEN           (1 << 29)             /* Bit 29: USBHS clock enable when sleep mode */
#define RCU_AHB1SPEN_USBHSULPISPEN       (1 << 30)             /* Bit 30: USBHS ULPI clock enable when sleep mode */

/* AHB2 sleep mode enable register */

#define RCU_AHB2SPEN_DCISPEN             (1 << 0)              /* Bit 0: DCI clock enable when sleep mode */
#define RCU_AHB2SPEN_TRNGSPEN            (1 << 6)              /* Bit 6: TRNG clock enable when sleep mode */
#define RCU_AHB2SPEN_USBFSSPEN           (1 << 7)              /* Bit 7: USBFS clock enable when sleep mode */

/* AHB1 sleep mode enable register */

#define RCU_AHB3SPEN_EXMCSPEN            (1 << 0)              /* Bit 0: EXMC clock enable when sleep mode */

/* APB1 sleep mode enable register */

#define RCU_APB1SPEN_TIMER1SPEN          (1 << 0)              /* Bit 0: TIMER1 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER2SPEN          (1 << 1)              /* Bit 1: TIMER2 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER3SPEN          (1 << 2)              /* Bit 2: TIMER3 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER4SPEN          (1 << 3)              /* Bit 3: TIMER4 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER5SPEN          (1 << 4)              /* Bit 4: TIMER5 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER6SPEN          (1 << 5)              /* Bit 5: TIMER6 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER11SPEN         (1 << 6)              /* Bit 6: TIMER11 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER12SPEN         (1 << 7)              /* Bit 7: TIMER12 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER13SPEN         (1 << 8)              /* Bit 8: TIMER13 clock enable when sleep mode */
#define RCU_APB1SPEN_WWDGTSPEN           (1 << 11)             /* Bit 11: WWDGT clock enable when sleep mode */
#define RCU_APB1SPEN_SPI1SPEN            (1 << 14)             /* Bit 15: SPI1 clock enable when sleep mode */
#define RCU_APB1SPEN_SPI2SPEN            (1 << 15)             /* Bit 16: SPI2 clock enable when sleep mode */
#define RCU_APB1SPEN_USART1SPEN          (1 << 17)             /* Bit 17: USART1 clock enable when sleep mode*/
#define RCU_APB1SPEN_USART2SPEN          (1 << 18)             /* Bit 18: USART2 clock enable when sleep mode*/
#define RCU_APB1SPEN_UART3SPEN           (1 << 19)             /* Bit 19: UART3 clock enable when sleep mode*/
#define RCU_APB1SPEN_UART4SPEN           (1 << 20)             /* Bit 20: UART4 clock enable when sleep mode */
#define RCU_APB1SPEN_I2C0SPEN            (1 << 21)             /* Bit 21: I2C0 clock enable when sleep mode */
#define RCU_APB1SPEN_I2C1SPEN            (1 << 22)             /* Bit 22: I2C1 clock enable when sleep mode*/
#define RCU_APB1SPEN_I2C2SPEN            (1 << 23)             /* Bit 23: I2C2 clock enable when sleep mode */
#define RCU_APB1SPEN_CAN0SPEN            (1 << 25)             /* Bit 25: CAN0 clock enable when sleep mode*/
#define RCU_APB1SPEN_CAN1SPEN            (1 << 26)             /* Bit 26: CAN1 clock enable when sleep mode */
#define RCU_APB1SPEN_PMUSPEN             (1 << 28)             /* Bit 28: PMU clock enable when sleep mode */
#define RCU_APB1SPEN_DACSPEN             (1 << 29)             /* Bit 29: DAC clock enable when sleep mode */
#define RCU_APB1SPEN_UART6SPEN           (1 << 30)             /* Bit 0: UART6 clock enable when sleep mode */
#define RCU_APB1SPEN_UART7SPEN           (1 << 31)             /* Bit 31: UART7 clock enable when sleep mode */

/* APB2 sleep mode enable register */

#define RCU_APB2SPEN_TIMER0SPEN          (1 << 0)              /* Bit 0: TIMER0 clock enable when sleep mode */
#define RCU_APB2SPEN_TIMER7SPEN          (1 << 1)              /* Bit 1: TIMER7 clock enable when sleep mode */
#define RCU_APB2SPEN_USART0SPEN          (1 << 4)              /* Bit 4: USART0 clock enable when sleep mode */
#define RCU_APB2SPEN_USART5SPEN          (1 << 5)              /* Bit 5: USART5 clock enable when sleep mode */
#define RCU_APB2SPEN_ADC0SPEN            (1 << 8)              /* Bit 8: ADC0 clock enable when sleep mode */
#define RCU_APB2SPEN_ADC1SPEN            (1 << 9)              /* Bit 9: ADC1 clock enable when sleep mode */
#define RCU_APB2SPEN_ADC2SPEN            (1 << 10)             /* Bit 10: ADC2 clock enable when sleep mode */
#define RCU_APB2SPEN_SDIOSPEN            (1 << 11)             /* Bit 11: SDIO clock enable when sleep mode */
#define RCU_APB2SPEN_SPI0SPEN            (1 << 12)             /* Bit 12: SPI0 clock enable when sleep mode */
#define RCU_APB2SPEN_SPI3SPEN            (1 << 13)             /* Bit 13: SPI3 clock enable when sleep mode */
#define RCU_APB2SPEN_SYSCFGSPEN          (1 << 14)             /* Bit 14: SYSCFG clock enable when sleep mode */
#define RCU_APB2SPEN_TIMER8SPEN          (1 << 16)             /* Bit 16: TIMER8 clock enable when sleep mode */
#define RCU_APB2SPEN_TIMER9SPEN          (1 << 17)             /* Bit 17: TIMER9 clock enable when sleep mode */
#define RCU_APB2SPEN_TIMER10SPEN         (1 << 18)             /* Bit 18: TIMER10 clock enable when sleep mode */
#define RCU_APB2SPEN_SPI4SPEN            (1 << 20)             /* Bit 20: SPI4 clock enable when sleep mode */
#define RCU_APB2SPEN_SPI5SPEN            (1 << 21)             /* Bit 21: SPI5 clock enable when sleep mode */
#define RCU_APB2SPEN_TLISPEN             (1 << 26)             /* Bit 26: TLI clock enable when sleep mode*/

/* Backup domain control register */

#define RCU_BDCTL_LXTALEN                (1 << 0)              /* Bit 0: LXTAL enable */
#define RCU_BDCTL_LXTALSTB               (1 << 1)              /* Bit 1: Low speed crystal oscillator stabilization flag */
#define RCU_BDCTL_LXTALBPS               (1 << 2)              /* Bit 2: LXTAL bypass mode enable */
#define RCU_BDCTL_LXTALDRI               (1 << 3)              /* Bit 3: LXTAL drive capability */
#  define RCU_LXTALDRI_LOWER_DRIVE       (0 << 3)              /* LXTAL drive capability is selected lower */
#  define RCU_LXTALDRI_HIGHER_DRIVE      RCU_BDCTL_LXTALDRI    /* LXTAL drive capability is selected higher */

#define RCU_BDCTL_RTCSRC_SHIFT           (8)                   /* Bits 8:9: RTC clock entry selection */
#define RCU_BDCTL_RTCSRC_MASK            (3 << RCU_BDCR_RTCSEL_SHIFT)
#  define RCU_BDCTL_RTCSRC(n)            ((n) << RCU_BDCR_RTCSEL_SHIFT)
#  define RCU_RTCSRC_NONE                RCU_BDCTL_RTCSRC(0)   /* No clock selected */
#  define RCU_RTCSRC_LXTAL               RCU_BDCTL_RTCSRC(1)   /* RTC source clock select LXTAL */
#  define RCU_RTCSRC_IRC32K              RCU_BDCTL_RTCSRC(2)   /* RTC source clock select IRC32K */
#  define RCU_RTCSRC_HXTAL_DIV_RTCDIV    RCU_BDCTL_RTCSRC(3)   /* RTC source clock select HXTAL/RTCDIV */

#define RCU_BDCTL_RTCEN                  (1 << 15)             /* Bit 15: RTC clock enable */
#define RCU_BDCTL_BKPRST                 (1 << 16)             /* Bit 16: Backup domain reset */

/* Reset source / clock register */

#define RCU_RSTSCK_IRC32KEN              (1 << 0)              /* Bit 0: IRC32K enable */
#define RCU_RSTSCK_IRC32KSTB             (1 << 1)              /* Bit 1: IRC32K stabilization flag */
#define RCU_RSTSCK_RSTFC                 (1 << 24)             /* Bit 24: Reset flag clear */
#define RCU_RSTSCK_BORRSTF               (1 << 25)             /* Bit 25: BOR reset flag */
#define RCU_RSTSCK_EPRSTF                (1 << 26)             /* Bit 26: External pin reset flag */
#define RCU_RSTSCK_PORRSTF               (1 << 27)             /* Bit 27: Power reset flag */
#define RCU_RSTSCK_SWRSTF                (1 << 28)             /* Bit 28: Software reset flag */
#define RCU_RSTSCK_FWDGTRSTF             (1 << 29)             /* Bit 29: Free watchdog timer reset flag */
#define RCU_RSTSCK_WWDGTRSTF             (1 << 30)             /* Bit 30: Window watchdog timer reset flag */
#define RCU_RSTSCK_LPRSTF                (1 << 31)             /* Bit 31: Low-power reset flag */

/* PLL clock spread spectrum control register */

#define RCU_PLLSSCTL_MODCNT_SHIFT        (0)                   /* Bit 0-12: These bits configure PLL spread spectrum modulation 
                                                                *            profile amplitude and frequency. The following criteria
                                                                *            must be met: MODSTEP*MODCNT<=2^15-1 */
#define RCU_PLLSSCTL_MODCNT_MASK         (0x1fff << RCU_PLLSSCTL_MODCNT_SHIFT)
#  define RCU_PLLSSCTL_MODCNT(n)         ((n) << RCU_PLLSSCTL_MODCNT_SHIFT)

#define RCU_PLLSSCTL_MODSTEP_SHIFT       (13)                  /* Bit 13-27: These bits configure PLL spread spectrum modulation 
                                                                *             profile amplitude and frequency. The following criteria
                                                                *             must be met: MODSTEP*MODCNT<=2^15-1 */
#define RCU_PLLSSCTL_MODSTEP_MASK        (0x7fff << RCU_PLLSSCTL_MODSTEP_SHIFT)
#  define RCU_PLLSSCTL_MODSTEP(n)        ((n) << RCU_SSCGR_INCSTEP_SHIFT)

#define RCU_PLLSSCTL_SS_TYPE             (1 << 30)             /* Bit 30: PLL spread spectrum modulation type select */
#define RCU_PLLSSCTL_SSCGON              (1 << 31)             /* Bit 31: PLL spread spectrum modulation enable */

/* PLLI2S register */

#define RCU_PLLI2S_PLLI2SN_SHIFT         (6)                   /* Bits 6-14: The PLLI2S VCO clock multi factor */
#define RCU_PLLI2S_PLLI2SN_MASK          (0x1ff << RCU_PLLI2S_PLLI2SN_SHIFT)
#  define RCU_PLLI2S_PLLI2SN(n)          ((n) << RCU_PLLI2S_PLLI2SN_SHIFT)   /* CK_PLLI2SVCO = CK_PLLI2SVCOSRC*n, n=50..500 */

#define RCU_PLLI2S_PLLI2SR_SHIFT         (28)                  /* Bits 28-30: The PLLI2S R output frequency division factor
                                                                *              from PLLI2S VCO clock */
#define RCU_PLLI2S_PLLI2SR_MASK          (7 << RCU_PLLI2S_PLLI2SR_SHIFT)
#  define RCU_PLLI2S_PLLI2SR(n)          ((n) << RCU_PLLI2S_PLLI2SR_SHIFT)   /* CK_PLLI2SR = CK_PLLI2SVCO/n, n=2..7 */

/* PLLSAI register */

#define RCU_PLLSAI_PLLSAIN_SHIFT         (6)                   /* Bits 6-14: The PLLSAI VCO clock multi factor */
#define RCU_PLLSAI_PLLSAIN_MASK          (0x1ff << RCU_PLLSAI_PLLSAIN_SHIFT)
#  define RCU_PLLSAI_PLLSAIN(n)          ((n) << RCU_PLLSAI_PLLSAIN_SHIFT)          /* n=50..500 */

#define RCU_PLLSAI_PLLSAIP_SHIFT         (16)                  /* Bits 16-17: The PLLSAI P output frequency division factor
                                                                *              from PLLSAI VCO clock */
#define RCU_PLLSAI_PLLSAIP_MASK          (3 << RCU_PLLSAI_PLLSAIP_SHIFT)
#  define RCU_PLLSAI_PLLSAIP(n)          ((((n)>>1)-1) << RCU_PLLSAI_PLLSAIP_SHIFT)  /* n=2,4,6,8 */
#  define RCU_PLLSAI_PLLSAIP_DIV_2       RCU_PLLSAI_PLLSAIP(2)
#  define RCU_PLLSAI_PLLSAIP_DIV_4       RCU_PLLSAI_PLLSAIP(4)
#  define RCU_PLLSAI_PLLSAIP_DIV_6       RCU_PLLSAI_PLLSAIP(6)
#  define RCU_PLLSAI_PLLSAIP_DIV_8       RCU_PLLSAI_PLLSAIP(8)

#define RCU_PLLSAI_PLLSAIR_SHIFT         (28)                  /* Bits 28-30: The PLLSAI R output frequency division factor 
                                                                *              from PLLSAI VCO clock */
#define RCU_PLLSAI_PLLSAIR_MASK          (7 << RCU_PLLSAI_PLLSAIR_SHIFT)
#  define RCU_PLLSAI_PLLSAIR(n)          ((n) << RCU_PLLSAI_PLLSAIR_SHIFT)           /* n=2..7 */

/* Clock configuration register 1 */

#define RCU_CFG1_PLLSAIRDIV_SHIFT        (16)                  /* Bits 16-17: The divider factor from PLLSAIR clock */
#define RCU_CFG1_PLLSAIRDIV_MASK         (3 << RCU_CFG1_PLLSAIRDIV_SHIFT)
#  define RCU_CFG1_PLLSAIRDIV(n)         ((((n)>>1)-1) << RCU_CFG1_PLLSAIRDIV_SHIFT) /* n=2,4,6,8 */
#  define RCU_CFG1_PLLSAIRDIV_DIV_2      RCU_CFG1_PLLSAIRDIV(2)
#  define RCU_CFG1_PLLSAIRDIV_DIV_4      RCU_CFG1_PLLSAIRDIV(4)
#  define RCU_CFG1_PLLSAIRDIV_DIV_6      RCU_CFG1_PLLSAIRDIV(6)
#  define RCU_CFG1_PLLSAIRDIV_DIV_8      RCU_CFG1_PLLSAIRDIV(8)

#define RCU_CFG1_TIMERSEL                (1 << 24)             /* Bit 24: TIMER clock selection */

/* Additional clock control register */

#define RCU_ADDCTL_CK48MSEL              (1 << 0)              /* Bit 0: 48MHz clock selection */
#define RCU_ADDCTL_PLL48MSEL             (1 << 1)              /* Bit 1: PLL48M clock selection */
#define RCU_ADDCTL_IRC48MEN              (1 << 16)             /* Bit 2: Internal 48MHz RC oscillator enable */
#define RCU_ADDCTL_IRC48MSTB             (1 << 17)             /* Bit 3: Internal 48MHz RC oscillator clock stabilization flag */
#define RCU_ADDCTL_IRC48MCAL_SHIFT       (24)                  /* Bit 24-31: Internal 48MHz RC oscillator calibration value register */
#define RCU_ADDCTL_IRC48MCAL_MASK        (ff << 24)
#  define RCU_ADDCTL_IRC48MCAL(n)        ((n) << 24)

/* Additional clock interrupt register */

#define RCU_ADDINT_IRC48MSTBIF           (6)                   /* Bits 6: IRC48M stabilization interrupt flag */
#define RCU_ADDINT_IRC48MSTBIE           (14)                  /* Bit 14: Internal 48 MHz RC oscillator stabilization interrupt enable */
#define RCU_ADDINT_IRC48MSTBIC           (22)                  /* Bit 22: Iinternal 48 MHz RC oscillator stabilization interrupt clear */

/* APB1 additional reset register */

#define RCU_ADDAPB1RST_CTCRST            (1 << 27)             /* Bit 27: CTC reset */
#define RCU_ADDAPB1RST_IREFRST           (1 << 31)             /* Bit 31: IREF reset */

/* APB1 additional enable register */

#define RCU_ADDAPB1EN_CTCEN              (1 << 27)             /* Bit 27: CTC clock enable */
#define RCU_ADDAPB1EN_IREFEN             (1 << 31)             /* Bit 31: IREF interface clock enable */

/* APB1 additional sleep mode enable register */

#define RCU_ADDAPB1SPEN_CTCSPEN          (1 << 27)             /* Bit 27: CTC clock enable during sleep mode */
#define RCU_ADDAPB1SPEN_IREFSPEN         (1 << 31)             /* Bit 31: IREF interface clock enable during sleep mode */

/* Voltage key register */
#define RCU_VKEY_KEY_SHIFT               (0)                   /* Bit 0-31: RCU_DSV key register */
#define RCU_VKEY_KEY_MASK                (ffffffff)
#define RCU_VKEY_UNLOCK                  (0x1A2B3C4D)          /* The voltage key unlock value */

/* Deep-sleep mode voltage register */
#define RCU_DSV_DSLPVS_SHIFT             (0)                   /* Bit 0-2: Deep-sleep mode voltage select */
#define RCU_DSV_DSLPVS_MASK              ((3) << RCU_DSV_DSLPVS_SHIFT)
#define RCU_DSV_DSLPVS(n)                ((n) << RCU_DSV_DSLPVS_SHIFT)
#define RCU_DEEPSLEEP_V_1_2              RCU_DSV_DSLPVS(0)     /* Core voltage is 1.2V in deep-sleep mode */
#define RCU_DEEPSLEEP_V_1_1              RCU_DSV_DSLPVS(1)     /* Core voltage is 1.1V in deep-sleep mode */
#define RCU_DEEPSLEEP_V_1_0              RCU_DSV_DSLPVS(2)     /* Core voltage is 1.0V in deep-sleep mode */
#define RCU_DEEPSLEEP_V_0_9              RCU_DSV_DSLPVS(3)     /* Core voltage is 0.9V in deep-sleep mode */

#endif /* __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_RCU_H */
