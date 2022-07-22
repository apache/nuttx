/****************************************************************************
 * arch/arm/src/s32k3xx/s32k3xx_clocknames.h
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

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_S32K3XX_CLOCKNAMES_H
#define __ARCH_ARM_SRC_S32K3XX_CLOCKNAMES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum clock_names_e
{
  /* Main clocks */

  CORE_CLK                      = 0,  /* Core clock */
  AIPS_PLAT_CLK                 = 1,  /* Bus clock */
  AIPS_SLOW_CLK                 = 2,  /* Slow clock */
  HSE_CLK                       = 2,  /* Slow clock */
  DCM_CLK                       = 2,  /* Slow clock */
  LBIST_CLK                     = 2,  /* Slow clock */
  QSPI_MEM_CLK                  = 2,  /* Slow clock */
  SIRC_CLK                      = 3,  /* CLKOUT clock */
  FIRC_CLK                      = 4,  /* CLKOUT clock */
  PLL_PHI0_CLK                  = 7,  /* CLKOUT clock */
  PLL_PHI1_CLK                  = 8,  /* CLKOUT clock */
  SCS_CLK                       = 9,  /* CLKOUT clock */

  SCG_END_OF_CLOCKS             = 18, /* End of SCG clocks */

  /* MC_ME clocks */

  TRGMUX_CLK                    = 32,  /* Trigger Multiplexing Control */
  BCTU_CLK                      = 33,  /* Body Cross Triggering Unit */
  EMIOS0_CLK                    = 34,  /* eMIOS */
  EMIOS1_CLK                    = 35,  /* eMIOS */
  EMIOS2_CLK                    = 36,  /* eMIOS */
  LCU0_CLK                      = 38,  /* LCU */
  LCU1_CLK                      = 39,  /* LCU */
  ADC0_CLK                      = 40,  /* ADC */
  ADC1_CLK                      = 41,  /* ADC */
  ADC2_CLK                      = 42,  /* ADC */
  PIT0_CLK                      = 44,  /* Programmable Interrupt Timer 0 */
  PIT1_CLK                      = 45,  /* Programmable Interrupt Timer 1 */
  MU_A_CLK                      = 46,  /* MU_A */
  MU_B_CLK                      = 47,  /* MU_B */
  EDMA_CLK                      = 131, /* eDMA */
  EDMA_TCD0_CLK                 = 132, /* eDMA TCD0 */
  EDMA_TCD1_CLK                 = 133, /* eDMA TCD1 */
  EDMA_TCD2_CLK                 = 134, /* eDMA TCD2 */
  EDMA_TCD3_CLK                 = 135, /* eDMA TCD3 */
  EDMA_TCD4_CLK                 = 136, /* eDMA TCD4 */
  EDMA_TCD5_CLK                 = 137, /* eDMA TCD5 */
  EDMA_TCD6_CLK                 = 138, /* eDMA TCD6 */
  EDMA_TCD7_CLK                 = 139, /* eDMA TCD7 */
  EDMA_TCD8_CLK                 = 140, /* eDMA TCD8 */
  EDMA_TCD9_CLK                 = 141, /* eDMA TCD9 */
  EDMA_TCD10_CLK                = 142, /* eDMA TCD10 */
  EDMA_TCD11_CLK                = 143, /* eDMA TCD11 */
  SDA_AP_CLK                    = 149, /* SDA-AP */
  EIM_CLK                       = 150, /* EIM */
  ERM_CLK                       = 151, /* ERM */
  MSCM_CLK                      = 152, /* MSCM */
  SWT0_CLK                      = 156, /* Software Watchdog 0 */
  STM0_CLK                      = 157, /* System Timer Module 0 */
  INTM_CLK                      = 159, /* Interrupt Monitor */
  DMAMUX0_CLK                   = 160, /* DMA Channel Multiplexer 0 */
  DMAMUX1_CLK                   = 161, /* DMA Channel Multiplexer 1 */
  RTC_CLK                       = 162, /* Real-time clock */
  WKPU_CLK                      = 173, /* Wakeup Unit */
  CMU_CLK                       = 175, /* CMU 0-5 */
  TSPC_CLK                      = 177, /* Touch Sensing Pin Coupling Controller */
  SXOSC_CLK                     = 178, /* 32 kHz Slow Internal RC Oscillator */
  FXOSC_CLK                     = 181, /* 8-40 MHz Fast External Crystal Oscillator */
  PLL_CLK                       = 184, /* Frequency Modulated Phase-Locked Loop */
  PIT2_CLK                      = 191, /* Programmable Interrupt Timer 2 */
  FLEXCAN0_CLK                  = 193, /* FlexCAN */
  FLEXCAN1_CLK                  = 194, /* FlexCAN */
  FLEXCAN2_CLK                  = 195, /* FlexCAN */
  FLEXCAN3_CLK                  = 196, /* FlexCAN */
  FLEXCAN4_CLK                  = 197, /* FlexCAN */
  FLEXCAN5_CLK                  = 198, /* FlexCAN */
  FLEXIO_CLK                    = 201, /* FlexIO */
  LPUART0_CLK                   = 202, /* UART */
  LPUART1_CLK                   = 203, /* UART */
  LPUART2_CLK                   = 204, /* UART */
  LPUART3_CLK                   = 205, /* UART */
  LPUART4_CLK                   = 206, /* UART */
  LPUART5_CLK                   = 207, /* UART */
  LPUART6_CLK                   = 208, /* UART */
  LPUART7_CLK                   = 209, /* UART */
  LPI2C0_CLK                    = 212, /* I2C */
  LPI2C1_CLK                    = 213, /* I2C */
  LPSPI0_CLK                    = 214, /* SPI */
  LPSPI1_CLK                    = 215, /* SPI */
  LPSPI2_CLK                    = 216, /* SPI */
  LPSPI3_CLK                    = 217, /* SPI */
  SAI0_CLK                      = 219, /* Synchronous Audio Interface 0 */
  LPCMP0_CLK                    = 220, /* Comparator 0 */
  LPCMP1_CLK                    = 221, /* Comparator 1 */
  TMU_CLK                       = 223, /* TempSense */
  CRC_CLK                       = 224, /* CRC */
  FCCU_CLK                      = 225, /* FCCU */
  STCU2_CLK                     = 232, /* Self-Test Control Unit */
  EDMA_TCD12_CLK                = 260, /* eDMA TCD12 */
  EDMA_TCD13_CLK                = 261, /* eDMA TCD13 */
  EDMA_TCD14_CLK                = 262, /* eDMA TCD14 */
  EDMA_TCD15_CLK                = 263, /* eDMA TCD15 */
  EDMA_TCD16_CLK                = 264, /* eDMA TCD16 */
  EDMA_TCD17_CLK                = 265, /* eDMA TCD17 */
  EDMA_TCD18_CLK                = 266, /* eDMA TCD18 */
  EDMA_TCD19_CLK                = 267, /* eDMA TCD19 */
  EDMA_TCD20_CLK                = 268, /* eDMA TCD20 */
  EDMA_TCD21_CLK                = 269, /* eDMA TCD21 */
  EDMA_TCD22_CLK                = 270, /* eDMA TCD22 */
  EDMA_TCD23_CLK                = 271, /* eDMA TCD23 */
  EDMA_TCD24_CLK                = 272, /* eDMA TCD24 */
  EDMA_TCD25_CLK                = 273, /* eDMA TCD25 */
  EDMA_TCD26_CLK                = 274, /* eDMA TCD26 */
  EDMA_TCD27_CLK                = 275, /* eDMA TCD27 */
  EDMA_TCD28_CLK                = 276, /* eDMA TCD28 */
  EDMA_TCD29_CLK                = 277, /* eDMA TCD29 */
  EDMA_TCD30_CLK                = 278, /* eDMA TCD30 */
  EDMA_TCD31_CLK                = 279, /* eDMA TCD31 */
  SEMA42_CLK                    = 280, /* Semaphores2 */
  SWT1_CLK                      = 283, /* Software Watchdog 1 */
  STM1_CLK                      = 285, /* System Timer Module 1 */
  EMAC_CLK                      = 288, /* EMAC */
  LPUART8_CLK                   = 291, /* UART */
  LPUART9_CLK                   = 292, /* UART */
  LPUART10_CLK                  = 293, /* UART */
  LPUART11_CLK                  = 294, /* UART */
  LPUART12_CLK                  = 295, /* UART */
  LPUART13_CLK                  = 296, /* UART */
  LPUART14_CLK                  = 297, /* UART */
  LPUART15_CLK                  = 298, /* UART */
  LPSPI4_CLK                    = 303, /* SPI */
  LPSPI5_CLK                    = 304, /* SPI */
  QSPI_CLK                      = 307, /* QSPI */
  SAI1_CLK                      = 311, /* Synchronous Audio Interface 1 */
  LPCMP2_CLK                    = 314, /* Comparator 2 */
  CM7_0_TCM_CLK                 = 318, /* CM7_0_TCM */
  CM7_1_TCM_CLK                 = 319, /* CM7_1_TCM */
  CLOCK_NAME_COUNT              = 92,  /* The total number of entries */
};

#endif /* __ARCH_ARM_SRC_S32K3XX_CLOCKNAMES_H */
