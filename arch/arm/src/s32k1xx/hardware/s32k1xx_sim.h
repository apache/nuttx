/****************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k1xx_sim.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_SIM_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_SIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SIM Register Offsets *****************************************************/

#define S32K1XX_SIM_CHIPCTL_OFFSET              0x0004  /* Chip Control register */
#define S32K1XX_SIM_FTMOPT0_OFFSET              0x000c  /* FTM Option Register 0 */
#define S32K1XX_SIM_LPOCLKS_OFFSET              0x0010  /* LPO Clock Select Register */
#define S32K1XX_SIM_ADCOPT_OFFSET               0x0018  /* ADC Options Register */
#define S32K1XX_SIM_FTMOPT1_OFFSET              0x001c  /* FTM Option Register 1 */
#define S32K1XX_SIM_MISCTRL0_OFFSET             0x0020  /* Miscellaneous control register 0 */
#define S32K1XX_SIM_SDID_OFFSET                 0x0024  /* System Device Identification Register */
#define S32K1XX_SIM_PLATCGC_OFFSET              0x0040  /* Platform Clock Gating Control Register */
#define S32K1XX_SIM_FCFG1_OFFSET                0x004c  /* Flash Configuration Register 1 */
#define S32K1XX_SIM_UIDH_OFFSET                 0x0054  /* Unique Identification Register High */
#define S32K1XX_SIM_UIDMH_OFFSET                0x0058  /* Unique Identification Register Mid-High */
#define S32K1XX_SIM_UIDML_OFFSET                0x005c  /* Unique Identification Register Mid Low */
#define S32K1XX_SIM_UIDL_OFFSET                 0x0060  /* Unique Identification Register Low */
#define S32K1XX_SIM_CLKDIV4_OFFSET              0x0068  /* System Clock Divider Register 4 */
#define S32K1XX_SIM_MISCTRL1_OFFSET             0x006c  /* Miscellaneous Control register 1 */

/* SIM Register Addresses ***************************************************/

#define S32K1XX_SIM_CHIPCTL                     (S32K1XX_SIM_BASE + S32K1XX_SIM_CHIPCTL_OFFSET)
#define S32K1XX_SIM_FTMOPT0                     (S32K1XX_SIM_BASE + S32K1XX_SIM_FTMOPT0_OFFSET)
#define S32K1XX_SIM_LPOCLKS                     (S32K1XX_SIM_BASE + S32K1XX_SIM_LPOCLKS_OFFSET)
#define S32K1XX_SIM_ADCOPT                      (S32K1XX_SIM_BASE + S32K1XX_SIM_ADCOPT_OFFSET)
#define S32K1XX_SIM_FTMOPT1                     (S32K1XX_SIM_BASE + S32K1XX_SIM_FTMOPT1_OFFSET)
#define S32K1XX_SIM_MISCTRL0                    (S32K1XX_SIM_BASE + S32K1XX_SIM_MISCTRL0_OFFSET)
#define S32K1XX_SIM_SDID                        (S32K1XX_SIM_BASE + S32K1XX_SIM_SDID_OFFSET)
#define S32K1XX_SIM_PLATCGC                     (S32K1XX_SIM_BASE + S32K1XX_SIM_PLATCGC_OFFSET)
#define S32K1XX_SIM_FCFG1                       (S32K1XX_SIM_BASE + S32K1XX_SIM_FCFG1_OFFSET)
#define S32K1XX_SIM_UIDH                        (S32K1XX_SIM_BASE + S32K1XX_SIM_UIDH_OFFSET)
#define S32K1XX_SIM_UIDMH                       (S32K1XX_SIM_BASE + S32K1XX_SIM_UIDMH_OFFSET)
#define S32K1XX_SIM_UIDML                       (S32K1XX_SIM_BASE + S32K1XX_SIM_UIDML_OFFSET)
#define S32K1XX_SIM_UIDL                        (S32K1XX_SIM_BASE + S32K1XX_SIM_UIDL_OFFSET)
#define S32K1XX_SIM_CLKDIV4                     (S32K1XX_SIM_BASE + S32K1XX_SIM_CLKDIV4_OFFSET)
#define S32K1XX_SIM_MISCTRL1                    (S32K1XX_SIM_BASE + S32K1XX_SIM_MISCTRL1_OFFSET)

/* SIM Register Bitfield Definitions ****************************************/

/* Chip Control register */

#define SIM_CHIPCTL_ADC_INTERLEAVE_EN_SHIFT     (0)      /* Bits 0-3: ADC interleave channel enable */
#define SIM_CHIPCTL_ADC_INTERLEAVE_EN_MASK      (15 << SIM_CHIPCTL_ADC_INTERLEAVE_EN_SHIFT)
#  define SIM_CHIPCTL_ADC_INTERLEAVE_PTB14      (8 << SIM_CHIPCTL_ADC_INTERLEAVE_EN_SHIFT)
#  define SIM_CHIPCTL_ADC_INTERLEAVE_PTB13      (4 << SIM_CHIPCTL_ADC_INTERLEAVE_EN_SHIFT)
#  define SIM_CHIPCTL_ADC_INTERLEAVE_PTB1       (2 << SIM_CHIPCTL_ADC_INTERLEAVE_EN_SHIFT)
#  define SIM_CHIPCTL_ADC_INTERLEAVE_PTB0       (0 << SIM_CHIPCTL_ADC_INTERLEAVE_EN_SHIFT)
#define SIM_CHIPCTL_CLKOUTSEL_SHIFT             (4)      /* Bits 4-7: CLKOUT Select */
#define SIM_CHIPCTL_CLKOUTSEL_MASK              (15 << SIM_CHIPCTL_CLKOUTSEL_SHIFT)
#  define SIM_CHIPCTL_CLKOUTSEL(n)              ((uint32_t)(n) << SIM_CHIPCTL_CLKOUTSEL_SHIFT)
#  define SIM_CHIPCTL_CLKOUTSEL_SCGC_LKOUT      (0 << SIM_CHIPCTL_CLKOUTSEL_SHIFT)
#  define SIM_CHIPCTL_CLKOUTSEL_SOSC_DIV2_CLK   (2 << SIM_CHIPCTL_CLKOUTSEL_SHIFT)
#  define SIM_CHIPCTL_CLKOUTSEL_SIRC_DIV2_CLK   (4 << SIM_CHIPCTL_CLKOUTSEL_SHIFT)
#  define SIM_CHIPCTL_CLKOUTSEL_QSPI_SFIF_CLK_HYP_PREMUX (5 << SIM_CHIPCTL_CLKOUTSEL_SHIFT) /* (S32K148) */
#  define SIM_CHIPCTL_CLKOUTSEL_FIRC_DIV2_CLK   (6 << SIM_CHIPCTL_CLKOUTSEL_SHIFT)
#  define SIM_CHIPCTL_CLKOUTSEL_HCLK            (7 << SIM_CHIPCTL_CLKOUTSEL_SHIFT)
#  define SIM_CHIPCTL_CLKOUTSEL_SPLL_DIV2_CLK   (8 << SIM_CHIPCTL_CLKOUTSEL_SHIFT)
#  define SIM_CHIPCTL_CLKOUTSEL_BUS_CLK         (9 << SIM_CHIPCTL_CLKOUTSEL_SHIFT)
#  define SIM_CHIPCTL_CLKOUTSEL_LPO128K_CLK     (10 << SIM_CHIPCTL_CLKOUTSEL_SHIFT)
#  define SIM_CHIPCTL_CLKOUTSEL_QSPI_MODULE_CLK (11 << SIM_CHIPCTL_CLKOUTSEL_SHIFT) /* (S32K148) */
#  define SIM_CHIPCTL_CLKOUTSEL_LPO_CLK         (12 << SIM_CHIPCTL_CLKOUTSEL_SHIFT)
#  define SIM_CHIPCTL_CLKOUTSEL_QSPI_SFIF_CLK   (13 << SIM_CHIPCTL_CLKOUTSEL_SHIFT) /* (S32K148) */
#  define SIM_CHIPCTL_CLKOUTSEL_RTC_CLK         (14 << SIM_CHIPCTL_CLKOUTSEL_SHIFT)
#  define SIM_CHIPCTL_CLKOUTSEL_QSPI_2xSFIF_CLK (15 << SIM_CHIPCTL_CLKOUTSEL_SHIFT) /* S32K148) */

#define SIM_CHIPCTL_CLKOUTDIV_SHIFT             (8)       /* Bits 8-10:  CLKOUT Divide Ratio */
#define SIM_CHIPCTL_CLKOUTDIV_MASK              (7 << SIM_CHIPCTL_CLKOUTDIV_SHIFT)
#  define SIM_CHIPCTL_CLKOUTDIV(n)              ((uint32_t)((n) - 1) << SIM_CHIPCTL_CLKOUTDIV_SHIFT) /* n=1..8 */

#define SIM_CHIPCTL_CLKOUTEN                    (1 << 11) /* Bit 11: CLKOUT enable */
#define SIM_CHIPCTL_TRACECLK_SEL                (1 << 12) /* Bit 12: Debug trace clock select */
#define SIM_CHIPCTL_PDB_BB_SEL                  (1 << 13) /* Bit 13: PDB back-to-back select */
#define SIM_CHIPCTL_ADC_SUPPLY_SHIFT            (16)      /* Bits 16-18: Supplies monitored on ADC0 channel 0 */
#define SIM_CHIPCTL_ADC_SUPPLY_MASK             (7 << SIM_CHIPCTL_ADC_SUPPLY_SHIFT)
#  define SIM_CHIPCTL_ADC_SUPPLY_VDD            (0 << SIM_CHIPCTL_ADC_SUPPLY_SHIFT) /* 5V input VDD supply */
#  define SIM_CHIPCTL_ADC_SUPPLY_VDDA           (1 << SIM_CHIPCTL_ADC_SUPPLY_SHIFT) /* 5V input analog supply */
#  define SIM_CHIPCTL_ADC_SUPPLY_VREFH          (2 << SIM_CHIPCTL_ADC_SUPPLY_SHIFT) /* ADC Reference Supply */
#  define SIM_CHIPCTL_ADC_SUPPLY_VDD_3V         (3 << SIM_CHIPCTL_ADC_SUPPLY_SHIFT) /* 3.3V Oscillator Regulator Output */
#  define SIM_CHIPCTL_ADC_SUPPLY_VDD_FLASH_3V   (4 << SIM_CHIPCTL_ADC_SUPPLY_SHIFT) /* 3.3V flash regulator output */
#  define SIM_CHIPCTL_ADC_SUPPLY_VDD_LV         (5 << SIM_CHIPCTL_ADC_SUPPLY_SHIFT) /* 1.2V core regulator output */

#define SIM_CHIPCTL_ADC_SUPPLYEN                (1 << 19) /* Bit 19:  Enable supply montorign ADC0 channel 0 */
#define SIM_CHIPCTL_SRAMU_RETEN                 (1 << 20) /* Bit 20: SRAMU retention */
#define SIM_CHIPCTL_SRAML_RETEN                 (1 << 21) /* Bit 21: SRAML retention */

/* FTM Option Register 0 */

#define SIM_FTMOPT0_FTM0FLTxSEL_SHIFT           (0)       /* Bits 0-2: FTM0 Fault X Select */
#define SIM_FTMOPT0_FTM0FLTxSEL_MASK            (7 << SIM_FTMOPT0_FTM0FLTxSEL_SHIFT)
#  define SIM_FTMOPT0_FTM0FLTxSEL_FTM0_FLTx     (0 << SIM_FTMOPT0_FTM0FLTxSEL_SHIFT) /* FTM0_FLTx pin */
#  define SIM_FTMOPT0_FTM0FLTxSEL_TRGMUX_FTM0   (1 << SIM_FTMOPT0_FTM0FLTxSEL_SHIFT) /* TRGMUX_FTM0 out */

#define SIM_FTMOPT0_FTM1FLTxSEL_SHIFT           (4)       /* Bits 4-6: FTM1 Fault X Select */
#define SIM_FTMOPT0_FTM1FLTxSEL_MASK            (7 << SIM_FTMOPT0_FTM1FLTxSEL_SHIFT)
#  define SIM_FTMOPT0_FTM1FLTxSEL_FTM1_FLTx     (0 << SIM_FTMOPT0_FTM1FLTxSEL_SHIFT) /* FTM1_FLTx pin */
#  define SIM_FTMOPT0_FTM1FLTxSEL_TRGMUX_FTM1   (1 << SIM_FTMOPT0_FTM1FLTxSEL_SHIFT) /* TRGMUX_FTM1 out */

#define SIM_FTMOPT0_FTM2FLTxSEL_SHIFT           (8)       /* Bits 8-10: FTM2 Fault X Select */
#define SIM_FTMOPT0_FTM2FLTxSEL_MASK            (7 << SIM_FTMOPT0_FTM2FLTxSEL_SHIFT)
#  define SIM_FTMOPT0_FTM2FLTxSEL_FTM2_FLTx     (0 << SIM_FTMOPT0_FTM2FLTxSEL_SHIFT) /* FTM2_FLTx pin */
#  define SIM_FTMOPT0_FTM2FLTxSEL_TRGMUX_FTM2   (1 << SIM_FTMOPT0_FTM2FLTxSEL_SHIFT) /* TRGMUX_FTM2 out */

#define SIM_FTMOPT0_FTM3FLTxSEL_SHIFT           (0)       /* Bits 0-2: FTM3 Fault X Select */
#define SIM_FTMOPT0_FTM3FLTxSEL_MASK            (7 << SIM_FTMOPT0_FTM3FLTxSEL_SHIFT)
#  define SIM_FTMOPT0_FTM3FLTxSEL_FTM3_FLTx     (0 << SIM_FTMOPT0_FTM3FLTxSEL_SHIFT) /* FTM3_FLTx pin */

#define FTM_CLKSEL_TCLK0                        (0)       /* FTM external clock driven by TCLK0 pin */
#define FTM_CLKSEL_TCLK1                        (1)       /* FTM external clock driven by TCLK1 pin */
#define FTM_CLKSEL_TCLK2                        (2)       /* FTM external clock driven by TCLK2 pin */
#define FTM_CLKSEL_NONE                         (3)       /* No clock input */

#define SIM_FTMOPT0_FTM4CLKSEL_SHIFT            (16)      /* Bits 16-17: FTM4 External Clock Pin Select */
#define SIM_FTMOPT0_FTM4CLKSEL_MASK             (3 << SIM_FTMOPT0_FTM4CLKSEL_SHIFT)
#  define SIM_FTMOPT0_FTM4CLKSEL(clksel)        ((uint32_t)(clksel) << SIM_FTMOPT0_FTM4CLKSEL_SHIFT)
#define SIM_FTMOPT0_FTM5CLKSEL_SHIFT            (18)      /* Bits 18-19: FTM5 External Clock Pin Select */
#define SIM_FTMOPT0_FTM5CLKSEL_MASK             (3 << SIM_FTMOPT0_FTM5CLKSEL_SHIFT)
#  define SIM_FTMOPT0_FTM5CLKSEL(clksel)        ((uint32_t)(clksel) << SIM_FTMOPT0_FTM5CLKSEL_SHIFT)
#define SIM_FTMOPT0_FTM6CLKSEL_SHIFT            (20)      /* Bits 20-21: FTM6 External Clock Pin Select */
#define SIM_FTMOPT0_FTM6CLKSEL_MASK             (3 << SIM_FTMOPT0_FTM6CLKSEL_SHIFT)
#  define SIM_FTMOPT0_FTM6CLKSEL(clksel)        ((uint32_t)(clksel) << SIM_FTMOPT0_FTM6CLKSEL_SHIFT)
#define SIM_FTMOPT0_FTM7CLKSEL_SHIFT            (22)      /* Bits 22-23: FTM7 External Clock Pin Select */
#define SIM_FTMOPT0_FTM7CLKSEL_MASK             (3 << SIM_FTMOPT0_FTM7CLKSEL_SHIFT)
#  define SIM_FTMOPT0_FTM7CLKSEL(clksel)        ((uint32_t)(clksel) << SIM_FTMOPT0_FTM7CLKSEL_SHIFT)
#define SIM_FTMOPT0_FTM0CLKSEL_SHIFT            (24)      /* Bits 24-25: FTM0 External Clock Pin Select */
#define SIM_FTMOPT0_FTM0CLKSEL_MASK             (3 << SIM_FTMOPT0_FTM0CLKSEL_SHIFT)
#  define SIM_FTMOPT0_FTM0CLKSEL(clksel)        ((uint32_t)(clksel) << SIM_FTMOPT0_FTM0CLKSEL_SHIFT)
#define SIM_FTMOPT0_FTM1CLKSEL_SHIFT            (26)      /* Bits 26-27: FTM1 External Clock Pin Select */
#define SIM_FTMOPT0_FTM1CLKSEL_MASK             (3 << SIM_FTMOPT0_FTM1CLKSEL_SHIFT)
#  define SIM_FTMOPT0_FTM1CLKSEL(clksel)        ((uint32_t)(clksel) << SIM_FTMOPT0_FTM1CLKSEL_SHIFT)
#define SIM_FTMOPT0_FTM2CLKSEL_SHIFT            (28)      /* Bits 28-29: FTM2 External Clock Pin Select */
#define SIM_FTMOPT0_FTM2CLKSEL_MASK             (3 << SIM_FTMOPT0_FTM2CLKSEL_SHIFT)
#  define SIM_FTMOPT0_FTM2CLKSEL(clksel)        ((uint32_t)(clksel) << SIM_FTMOPT0_FTM2CLKSEL_SHIFT)
#define SIM_FTMOPT0_FTM3CLKSEL_SHIFT            (30)      /* Bits 30-31: FTM3 External Clock Pin Select */
#define SIM_FTMOPT0_FTM3CLKSEL_MASK             (3 << SIM_FTMOPT0_FTM3CLKSEL_SHIFT)
#  define SIM_FTMOPT0_FTM3CLKSEL(clksel)        ((uint32_t)(clksel) << SIM_FTMOPT0_FTM3CLKSEL_SHIFT)

/* LPO Clock Select Register */

#define SIM_LPOCLKS_LPO1KCLKEN                  (1 << 0)  /* Bit 0:  1kHz LPO_CLK enable */
#define SIM_LPOCLKS_LPO32KCLKEN                 (1 << 1)  /* Bit 1:  32kHz LPO_CLK enable */
#define SIM_LPOCLKS_LPOCLKSEL_SHIFT             (2)       /* Bits 2-3: LPO clock source select */
#define SIM_LPOCLKS_LPOCLKSEL_MASK              (3 << SIM_LPOCLKS_LPOCLKSEL_SHIFT)
#  define SIM_LPOCLKS_LPOCLKSEL(n)              ((uint32_t)(n) << SIM_LPOCLKS_LPOCLKSEL_SHIFT)
#  define SIM_LPOCLKS_LPOCLKSEL_128KHz_LPO_CLK  (0 << SIM_LPOCLKS_LPOCLKSEL_SHIFT) /* 128kHz LPO_CLK */
#  define SIM_LPOCLKS_LPOCLKSEL_NO_CLOCK        (1 << SIM_LPOCLKS_LPOCLKSEL_SHIFT) /* No clock */
#  define SIM_LPOCLKS_LPOCLKSEL_32KHz_LPO_CLK   (2 << SIM_LPOCLKS_LPOCLKSEL_SHIFT) /* 32kHz LPO_CLK derived from 128 kHz LPO_CLK */
#  define SIM_LPOCLKS_LPOCLKSEL_1KHz_LPO_CLK    (3 << SIM_LPOCLKS_LPOCLKSEL_SHIFT) /* 1kHz LPO_CLK derived from 128 kHz LPO_CLK */

#define SIM_LPOCLKS_RTCCLKSEL_SHIFT             (4)       /* Bits 4-5: 32kHz clock source select */
#define SIM_LPOCLKS_RTCCLKSEL_MASK              (3 << SIM_LPOCLKS_RTCCLKSEL_SHIFT)
#  define SIM_LPOCLKS_RTCCLKSEL(n)              ((uint32_t)(n) << SIM_LPOCLKS_RTCCLKSEL_SHIFT)
#  define SIM_LPOCLKS_RTCCLKSEL_SOSCDIV1_CLK    (0 << SIM_LPOCLKS_RTCCLKSEL_SHIFT) /* SOSCDIV1_CLK */
#  define SIM_LPOCLKS_RTCCLKSEL_32KHz_LPO_CLK   (1 << SIM_LPOCLKS_RTCCLKSEL_SHIFT) /* 32KHz LPO_CLK */
#  define SIM_LPOCLKS_RTCCLKSEL_32KHz_RTC_CLKIN (2 << SIM_LPOCLKS_RTCCLKSEL_SHIFT) /* 32KHz RTC_CLKIN clock */
#  define SIM_LPOCLKS_RTCCLKSEL_FIRCDIV1_CLK    (3 << SIM_LPOCLKS_RTCCLKSEL_SHIFT) /* FIRCDIV1_CLK */

/* ADC Options Register */

#define SIM_ADCOPT_ADC0TRGSEL                   (1 << 0)  /* Bit 0:  ADC0 trigger source select */
# define SIM_ADCOPT_ADC0TRGSEL_PDB              (0)       /*         PDB output */
# define SIM_ADCOPT_ADC0TRGSEL_TRGMUX           (1 << 0)  /*         TRGMUX output */
#define SIM_ADCOPT_ADC0SWPRETRG_SHIFT           (1)       /* Bits 1-3: ADC0 software pretrigger sources */
#define SIM_ADCOPT_ADC0SWPRETRG_MASK            (7 << SIM_ADCOPT_ADC0SWPRETRG_SHIFT)
#  define SIM_ADCOPT_ADC0SWPRETRG_DISABLED      (0 << SIM_ADCOPT_ADC0SWPRETRG_SHIFT) /* Software pretrigger disabled */
#  define SIM_ADCOPT_ADC0SWPRETRG_SWPRETRG0     (4 << SIM_ADCOPT_ADC0SWPRETRG_SHIFT) /* Software pretrigger 0 */
#  define SIM_ADCOPT_ADC0SWPRETRG_SWPRETRG1     (5 << SIM_ADCOPT_ADC0SWPRETRG_SHIFT) /* Software pretrigger 1 */
#  define SIM_ADCOPT_ADC0SWPRETRG_SWPRETRG2     (6 << SIM_ADCOPT_ADC0SWPRETRG_SHIFT) /* Software pretrigger 2 */
#  define SIM_ADCOPT_ADC0SWPRETRG_SWPRETRG3     (7 << SIM_ADCOPT_ADC0SWPRETRG_SHIFT) /* Software pretrigger 3 */

#define SIM_ADCOPT_ADC0PRETRGSEL_SHIFT          (4)       /* Bits 4-5:  ADC0 pretrigger source select */
#define SIM_ADCOPT_ADC0PRETRGSEL_MASK           (3 << SIM_ADCOPT_ADC0PRETRGSEL_SHIFT)
#  define SIM_ADCOPT_ADC0PRETRGSEL_PDB          (0 << SIM_ADCOPT_ADC0PRETRGSEL_SHIFT) /* PDB pretrigger */
#  define SIM_ADCOPT_ADC0PRETRGSEL_TRGMUX       (1 << SIM_ADCOPT_ADC0PRETRGSEL_SHIFT) /* TRGMUX pretrigger */
#  define SIM_ADCOPT_ADC0PRETRGSEL_SW           (2 << SIM_ADCOPT_ADC0PRETRGSEL_SHIFT) /* Software pretrigger */

#define SIM_ADCOPT_ADC1TRGSEL                   (1 << 8)  /* Bit 8:  ADC1 trigger source select */
# define SIM_ADCOPT_ADC1TRGSEL_PDB              (0)       /*         PDB output */
# define SIM_ADCOPT_ADC1TRGSEL_TRGMUX           (1 << 8)  /*         TRGMUX output */
#define SIM_ADCOPT_ADC1SWPRETRG_SHIFT           (9)       /* Bits 9-11: ADC1 software pretrigger sources */
#define SIM_ADCOPT_ADC1SWPRETRG_MASK            (7 << SIM_ADCOPT_ADC1SWPRETRG_SHIFT)
#  define SIM_ADCOPT_ADC1SWPRETRG_DISABLED      (0 << SIM_ADCOPT_ADC1SWPRETRG_SHIFT) /* Software pretrigger disabled */
#  define SIM_ADCOPT_ADC1SWPRETRG_SWPRETRG0     (4 << SIM_ADCOPT_ADC1SWPRETRG_SHIFT) /* Software pretrigger 0 */
#  define SIM_ADCOPT_ADC1SWPRETRG_SWPRETRG1     (5 << SIM_ADCOPT_ADC1SWPRETRG_SHIFT) /* Software pretrigger 1 */
#  define SIM_ADCOPT_ADC1SWPRETRG_SWPRETRG2     (6 << SIM_ADCOPT_ADC1SWPRETRG_SHIFT) /* Software pretrigger 2 */
#  define SIM_ADCOPT_ADC1SWPRETRG_SWPRETRG3     (7 << SIM_ADCOPT_ADC1SWPRETRG_SHIFT) /* Software pretrigger 3 */

#define SIM_ADCOPT_ADC1PRETRGSEL_SHIFT          (12)      /* Bits 12-13:  ADC1 pretrigger source select */
#define SIM_ADCOPT_ADC1PRETRGSEL_MASK           (3 << SIM_ADCOPT_ADC1PRETRGSEL_SHIFT)
#  define SIM_ADCOPT_ADC1PRETRGSEL_PDB          (0 << SIM_ADCOPT_ADC1PRETRGSEL_SHIFT) /* PDB pretrigger */
#  define SIM_ADCOPT_ADC1PRETRGSEL_TRGMUX       (1 << SIM_ADCOPT_ADC1PRETRGSEL_SHIFT) /* TRGMUX pretrigger */
#  define SIM_ADCOPT_ADC1PRETRGSEL_SW           (2 << SIM_ADCOPT_ADC1PRETRGSEL_SHIFT) /* Software pretrigger */

/* FTM Option Register 1 */

#define SIM_FTMOPT1_FTM0SYNCBIT                 (1 << 0)  /* Bit 0:  FTM0 Sync Bit */
#define SIM_FTMOPT1_FTM1SYNCBIT                 (1 << 1)  /* Bit 1:  FTM1 Sync Bit */
#define SIM_FTMOPT1_FTM2SYNCBIT                 (1 << 2)  /* Bit 2:  FTM2 Sync Bit */
#define SIM_FTMOPT1_FTM3SYNCBIT                 (1 << 3)  /* Bit 3:  FTM3 Sync Bit */
#define SIM_FTMOPT1_FTM1CH0SEL_SHIFT            (4)       /* Bits 4-5: FTM1 CH0 Select */
#define SIM_FTMOPT1_FTM1CH0SEL_MASK             (3 << SIM_FTMOPT1_FTM1CH0SEL_SHIFT)
#  define SIM_FTMOPT1_FTM1CH0SEL_FTM1_CH0       (0 << SIM_FTMOPT1_FTM1CH0SEL_SHIFT) /* FTM1_CH0 input */
#  define SIM_FTMOPT1_FTM1CH0SEL_CMP0           (1 << SIM_FTMOPT1_FTM1CH0SEL_SHIFT) /* CMP0 output */

#define SIM_FTMOPT1_FTM2CH0SEL_SHIFT            (6)       /* Bits 6-7: FTM2 CH0 Select */
#define SIM_FTMOPT1_FTM2CH0SEL_MASK             (3 << SIM_FTMOPT1_FTM2CH0SEL_SHIFT)
#  define SIM_FTMOPT1_FTM1CH0SEL_FTM2_CH0       (0 << SIM_FTMOPT1_FTM1CH0SEL_SHIFT) /* FTM2_CH0 input */
#  define SIM_FTMOPT1_FTM1CH0SEL_CMP0           (1 << SIM_FTMOPT1_FTM1CH0SEL_SHIFT) /* CMP0 output */

#define SIM_FTMOPT1_FTM2CH1SEL                  (1 << 8)  /* Bit 8:  FTM2 CH1 Select */
#define SIM_FTMOPT1_FTM4SYNCBIT                 (1 << 11) /* Bit 11: FTM4 Sync Bit */
#define SIM_FTMOPT1_FTM5SYNCBIT                 (1 << 12) /* Bit 12: FTM5 Sync Bit */
#define SIM_FTMOPT1_FTM6SYNCBIT                 (1 << 13) /* Bit 13: FTM6 Sync Bit */
#define SIM_FTMOPT1_FTM7SYNCBIT                 (1 << 14) /* Bit 14: FTM7 Sync Bit */
#define SIM_FTMOPT1_FTMGLDOK                    (1 << 15) /* Bit 15: FTM global load enable */
#define SIM_FTMOPT1_FTM0_OUTSEL_SHIFT           (16)      /* Bits 16-23: FTM0 channel modulation select with FTM1_CH1 */
#define SIM_FTMOPT1_FTM0_OUTSEL_MASK            (0xff << SIM_FTMOPT1_FTM0_OUTSEL_SHIFT)
#  define SIM_FTMOPT1_FTM0_OUTSEL_CHAN0         (1 << SIM_FTMOPT1_FTM0_OUTSEL_SHIFT)   /* Modulation with FTM1_CH0 */
#  define SIM_FTMOPT1_FTM0_OUTSEL_CHAN1         (2 << SIM_FTMOPT1_FTM0_OUTSEL_SHIFT)   /* Modulation with FTM1_CH1 */
#  define SIM_FTMOPT1_FTM0_OUTSEL_CHAN2         (4 << SIM_FTMOPT1_FTM0_OUTSEL_SHIFT)   /* Modulation with FTM1_CH2 */
#  define SIM_FTMOPT1_FTM0_OUTSEL_CHAN3         (8 << SIM_FTMOPT1_FTM0_OUTSEL_SHIFT)   /* Modulation with FTM1_CH3 */
#  define SIM_FTMOPT1_FTM0_OUTSEL_CHAN4         (16 << SIM_FTMOPT1_FTM0_OUTSEL_SHIFT)  /* Modulation with FTM1_CH4 */
#  define SIM_FTMOPT1_FTM0_OUTSEL_CHAN5         (32 << SIM_FTMOPT1_FTM0_OUTSEL_SHIFT)  /* Modulation with FTM1_CH5 */
#  define SIM_FTMOPT1_FTM0_OUTSEL_CHAN6         (64 << SIM_FTMOPT1_FTM0_OUTSEL_SHIFT)  /* Modulation with FTM1_CH6 */
#  define SIM_FTMOPT1_FTM0_OUTSEL_CHAN7         (128 << SIM_FTMOPT1_FTM0_OUTSEL_SHIFT) /* Modulation with FTM1_CH7 */

#define SIM_FTMOPT1_FTM3_OUTSEL_SHIFT           (16)      /* Bits 16-23: FTM3 channel modulation select with FTM2_CH1 */
#define SIM_FTMOPT1_FTM3_OUTSEL_MASK            (0xff << SIM_FTMOPT1_FTM3_OUTSEL_SHIFT)
#  define SIM_FTMOPT1_FTM3_OUTSEL_CHAN0         (1 << SIM_FTMOPT1_FTM3_OUTSEL_SHIFT)   /* Modulation with FTM2_CH0 */
#  define SIM_FTMOPT1_FTM3_OUTSEL_CHAN1         (2 << SIM_FTMOPT1_FTM3_OUTSEL_SHIFT)   /* Modulation with FTM2_CH1 */
#  define SIM_FTMOPT1_FTM3_OUTSEL_CHAN2         (4 << SIM_FTMOPT1_FTM3_OUTSEL_SHIFT)   /* Modulation with FTM2_CH2 */
#  define SIM_FTMOPT1_FTM3_OUTSEL_CHAN3         (8 << SIM_FTMOPT1_FTM3_OUTSEL_SHIFT)   /* Modulation with FTM2_CH3 */
#  define SIM_FTMOPT1_FTM3_OUTSEL_CHAN4         (16 << SIM_FTMOPT1_FTM3_OUTSEL_SHIFT)  /* Modulation with FTM2_CH4 */
#  define SIM_FTMOPT1_FTM3_OUTSEL_CHAN5         (32 << SIM_FTMOPT1_FTM3_OUTSEL_SHIFT)  /* Modulation with FTM2_CH5 */
#  define SIM_FTMOPT1_FTM3_OUTSEL_CHAN6         (64 << SIM_FTMOPT1_FTM3_OUTSEL_SHIFT)  /* Modulation with FTM2_CH6 */
#  define SIM_FTMOPT1_FTM3_OUTSEL_CHAN7         (128 << SIM_FTMOPT1_FTM3_OUTSEL_SHIFT) /* Modulation with FTM2_CH7 */

/* Miscellaneous control register 0 */

#define SIM_MISCTRL0_STOP1_MONITOR              (1 << 9)  /* Bit 9:  STOP1 monitor bit */
#define SIM_MISCTRL0_STOP2_MONITOR              (1 << 10) /* Bit 10: STOP2 monitor bit */
#define SIM_MISCTRL0_FTM_GTB_SPLIT_EN           (1 << 14) /* Bit 14: FTM GTB split enable/disable bit */
#define SIM_MISCTRL0_FTM0_OBE_CTRL              (1 << 16) /* Bit 16: FTM0 OBE CTRL bit */
#define SIM_MISCTRL0_FTM1_OBE_CTRL              (1 << 17) /* Bit 17: FTM1 OBE CTRL bit */
#define SIM_MISCTRL0_FTM2_OBE_CTRL              (1 << 18) /* Bit 18: FTM2 OBE CTRL bit */
#define SIM_MISCTRL0_FTM3_OBE_CTRL              (1 << 19) /* Bit 19: FTM3 OBE CTRL bit */
#define SIM_MISCTRL0_FTM4_OBE_CTRL              (1 << 20) /* Bit 20: FTM4 OBE CTRL bit */
#define SIM_MISCTRL0_FTM5_OBE_CTRL              (1 << 21) /* Bit 21: FTM5 OBE CTRL bit */
#define SIM_MISCTRL0_FTM6_OBE_CTRL              (1 << 22) /* Bit 22: FTM6 OBE CTRL bit */
#define SIM_MISCTRL0_FTM7_OBE_CTRL              (1 << 23) /* Bit 23: FTM7 OBE CTRL bit */
#define SIM_MISCTRL0_RMII_CLK_OBE               (1 << 24) /* Bit 24: RMII CLK OBE bit */
#define SIM_MISCTRL0_RMII_CLK_SEL               (1 << 25) /* Bit 25: RMII CLK Select bit */
#define SIM_MISCTRL0_QSPI_CLK_SEL               (1 << 26) /* Bit 26: QSPI CLK Select bit */

/* System Device Identification Register */

#define SIM_SDID_FEATURES_SHIFT                 (0)       /* Bits 0-7: Features */
#define SIM_SDID_FEATURES_MASK                  (0xff << SIM_SDID_FEATURES_SHIFT)
#  define SIM_SDID_FEATURES_SAI                 (1 << 1) /* Bit 1: SAI */
#  define SIM_SDID_FEATURES_ISELED              (1 << 2) /* Bit 2: ISELED */
#  define SIM_SDID_FEATURES_ENET                (1 << 3) /* Bit 3: ENET */
#  define SIM_SDID_FEATURES_QUADSPI             (1 << 4) /* Bit 4: QuadSPI */
#  define SIM_SDID_FEATURES_FLEXIO              (1 << 5) /* Bit 5: FlexIO */
#  define SIM_SDID_FEATURES_ISO_CANFD           (1 << 6) /* Bit 6: ISO CAN-FD */
#  define SIM_SDID_FEATURES_SECURITY            (1 << 7) /* Bit 7: Security */
#define SIM_SDID_PACKAGE_SHIFT                  (8)      /* Bits 8-11: Package */
#define SIM_SDID_PACKAGE_MASK                   (15 << SIM_SDID_PACKAGE_SHIFT)
#  define SIM_SDID_PACKAGE_32QFN                (1 << SIM_SDID_PACKAGE_SHIFT) /* 32 QFN */
#  define SIM_SDID_PACKAGE_48LQFP               (2 << SIM_SDID_PACKAGE_SHIFT) /* 48 LQFP */
#  define SIM_SDID_PACKAGE_64LQFP               (3 << SIM_SDID_PACKAGE_SHIFT) /* 64 LQFP */
#  define SIM_SDID_PACKAGE_100LQFP              (4 << SIM_SDID_PACKAGE_SHIFT) /* 100 LQFP */
#  define SIM_SDID_PACKAGE_144LQFP              (6 << SIM_SDID_PACKAGE_SHIFT) /* 144 LQFP */
#  define SIM_SDID_PACKAGE_176LQFP              (7 << SIM_SDID_PACKAGE_SHIFT) /* 176 LQFP */
#  define SIM_SDID_PACKAGE_100MAPBGA            (8 << SIM_SDID_PACKAGE_SHIFT) /* 100 MAP BGA */

#define SIM_SDID_REVID_SHIFT                    (12)      /* Bits 12-15:  Device revision number */
#define SIM_SDID_REVID_MASK                     (15 << SIM_SDID_REVID_SHIFT)
#define SIM_SDID_RAMSIZE_SHIFT                  (16)      /* Bits 16-19:  RAM size */
#define SIM_SDID_RAMSIZE_MASK                   (15 << SIM_SDID_RAMSIZE_SHIFT)
#  define SIM_SDID_RAMSIZE_17KB                 (15 << SIM_SDID_RAMSIZE_SHIFT) /* S32K116 */
#  define SIM_SDID_RAMSIZE_25KB                 (15 << SIM_SDID_RAMSIZE_SHIFT) /* S32K118 */
#  define SIM_SDID_RAMSIZE_32KB                 (15 << SIM_SDID_RAMSIZE_SHIFT) /* S32K142 */
#  define SIM_SDID_RAMSIZE_48KB                 (13 << SIM_SDID_RAMSIZE_SHIFT) /* S32K144 */
#  define SIM_SDID_RAMSIZE_64KB                 (15 << SIM_SDID_RAMSIZE_SHIFT) /* S32K144 */
#  define SIM_SDID_RAMSIZE_96KB                 (11 << SIM_SDID_RAMSIZE_SHIFT) /* S32K146 */
#  define SIM_SDID_RAMSIZE_128KB                (15 << SIM_SDID_RAMSIZE_SHIFT) /* S32K146 */
#  define SIM_SDID_RAMSIZE_192KB                (11 << SIM_SDID_RAMSIZE_SHIFT) /* S32K148 */
#  define SIM_SDID_RAMSIZE_256KB                (15 << SIM_SDID_RAMSIZE_SHIFT) /* S32K148 */
#  define SIM_SDID_RAMSIZE_256KB                (15 << SIM_SDID_RAMSIZE_SHIFT) /* S32K148 */

#define SIM_SDID_DERIVATE_SHIFT                 (20)      /* Bits 20-23: Derivate */
#define SIM_SDID_DERIVATE_MASK                  (15 << SIM_SDID_DERIVATE_SHIFT)
#define SIM_SDID_SUBSERIES_SHIFT                (24)      /* Bits 24-27: Subseries */
#define SIM_SDID_SUBSERIES_MASK                 (14 << SIM_SDID_SUBSERIES_SHIFT)
#define SIM_SDID_GENERATION_SHIFT               (28)      /* Bits 28-31: S32K product series generation */
#define SIM_SDID_GENERATION_MASK                (15 << SIM_SDID_GENERATION_SHIFT)

/* Platform Clock Gating Control Register */

#define SIM_PLATCGC_CGCMSCM                     (1 << 0)  /* Bit 0:  MSCM Clock Gating Control */
#define SIM_PLATCGC_CGCMPU                      (1 << 1)  /* Bit 1:  MPU Clock Gating Control */
#define SIM_PLATCGC_CGCDMA                      (1 << 2)  /* Bit 2:  DMA Clock Gating Control */
#define SIM_PLATCGC_CGCERM                      (1 << 3)  /* Bit 3:  ERM Clock Gating Control */
#define SIM_PLATCGC_CGCEIM                      (1 << 4)  /* Bit 4:  EIM Clock Gating Control */

/* Flash Configuration Register 1 */

#define SIM_FCFG1_DEPART_SHIFT                  (12)      /* Bits 12-15: FlexNVM partition */
#define SIM_FCFG1_DEPART_MASK                   (15 << SIM_FCFG1_DEPART_SHIFT)
#  define SIM_FCFG1_DEPART(n)                   ((uint32_t)(n) << SIM_FCFG1_DEPART_SHIFT)
#define SIM_FCFG1_EEERAMSIZE_SHIFT              (16)      /* Bits 16-19: EEE SRAM data size */
#define SIM_FCFG1_EEERAMSIZE_MASK               (15 << SIM_FCFG1_EEERAMSIZE_SHIFT)
#  define SIM_FCFG1_EEERAMSIZE_4KB              (2 << SIM_FCFG1_EEERAMSIZE_SHIFT)  /* 4Kb */
#  define SIM_FCFG1_EEERAMSIZE_2KB              (3 << SIM_FCFG1_EEERAMSIZE_SHIFT)  /* 2Kb */
#  define SIM_FCFG1_EEERAMSIZE_1KB              (4 << SIM_FCFG1_EEERAMSIZE_SHIFT)  /* 1Kb */
#  define SIM_FCFG1_EEERAMSIZE_512B             (5 << SIM_FCFG1_EEERAMSIZE_SHIFT)  /* 512 Bytes */
#  define SIM_FCFG1_EEERAMSIZE_256B             (6 << SIM_FCFG1_EEERAMSIZE_SHIFT)  /* 256 Bytes */
#  define SIM_FCFG1_EEERAMSIZE_128B             (7 << SIM_FCFG1_EEERAMSIZE_SHIFT)  /* 128 Bytes */
#  define SIM_FCFG1_EEERAMSIZE_64B              (8 << SIM_FCFG1_EEERAMSIZE_SHIFT)  /* 64 Bytes */
#  define SIM_FCFG1_EEERAMSIZE_32B              (9 << SIM_FCFG1_EEERAMSIZE_SHIFT)  /* 32 Bytes */
#  define SIM_FCFG1_EEERAMSIZE_0B               (15 << SIM_FCFG1_EEERAMSIZE_SHIFT) /*  0 Bytes */

/* Unique Identification Register High (32-bit UIDH[96-127]) */

/* Unique Identification Register Mid-High (32-bit UIDH[64-95]) */

/* Unique Identification Register Mid Low (32-bit UIDH[32-63]) */

/* Unique Identification Register Low (32-bit UIDH[0-31]) */

/* System Clock Divider Register 4 */

#define SIM_CLKDIV4_TRACEFRAC                   (1 << 0)  /* Bit 0:  Trace Clock Divider fraction */
#define SIM_CLKDIV4_TRACEDIV_SHIFT              (1)       /* Bits 1-3:  Trace Clock Divider value */
#define SIM_CLKDIV4_TRACEDIV_MASK               (7 << SIM_CLKDIV4_TRACEDIV_SHIFT)
#  define SIM_CLKDIV4_TRACEDIV(n)               ((uint32_t)((n) - 1) << SIM_CLKDIV4_TRACEDIV_SHIFT) /* n=1..8 */

#define SIM_CLKDIV4_TRACEDIVEN                  (1 << 28) /* Bit 28: Debug Trace Divider control */

/* Miscellaneous Control register 1 */

#define SIM_MISCTRL1_SW_TRG                     (1 << 0)  /* Bit 9:  Software trigger to TRGMUX */

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_SIM_H */
