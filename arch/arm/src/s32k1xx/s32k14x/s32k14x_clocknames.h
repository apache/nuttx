/****************************************************************************
 * arch/arm/src/s32k1xx/s32k14x/s32k14x_clocknames.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_S32K14X_S32K14X_CLOCKNAMES_H
#define __ARCH_ARM_SRC_S32K1XX_S32K14X_S32K14X_CLOCKNAMES_H

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
  BUS_CLK                       = 1,  /* Bus clock */
  SLOW_CLK                      = 2,  /* Slow clock */
  CLKOUT_CLK                    = 3,  /* CLKOUT clock */

  /* Other internal clocks used by peripherals */

  SIRC_CLK                      = 4,  /* SIRC clock */
  FIRC_CLK                      = 5,  /* FIRC clock */
  SOSC_CLK                      = 6,  /* SOSC clock */
  SPLL_CLK                      = 7,  /* SPLL clock */
  RTC_CLKIN_CLK                 = 8,  /* RTC_CLKIN clock */
  SCG_CLKOUT_CLK                = 9,  /* SCG CLK_OUT clock */
  SIRCDIV1_CLK                  = 10, /* SIRCDIV1 functional clock */
  SIRCDIV2_CLK                  = 11, /* SIRCDIV2 functional clock */
  FIRCDIV1_CLK                  = 12, /* FIRCDIV1 functional clock */
  FIRCDIV2_CLK                  = 13, /* FIRCDIV2 functional clock */
  SOSCDIV1_CLK                  = 14, /* SOSCDIV1 functional clock */
  SOSCDIV2_CLK                  = 15, /* SOSCDIV2 functional clock */
  SPLLDIV1_CLK                  = 16, /* SPLLDIV1 functional clock */
  SPLLDIV2_CLK                  = 17, /* SPLLDIV2 functional clock */

  SCG_END_OF_CLOCKS             = 18, /* End of SCG clocks */

  /* SIM clocks */

  SIM_FTM0_CLOCKSEL             = 21, /* FTM0 External Clock Pin Select */
  SIM_FTM1_CLOCKSEL             = 22, /* FTM1 External Clock Pin Select */
  SIM_FTM2_CLOCKSEL             = 23, /* FTM2 External Clock Pin Select */
  SIM_FTM3_CLOCKSEL             = 24, /* FTM3 External Clock Pin Select */
  SIM_FTM4_CLOCKSEL             = 25, /* FTM4 External Clock Pin Select */
  SIM_FTM5_CLOCKSEL             = 26, /* FTM5 External Clock Pin Select */
  SIM_FTM6_CLOCKSEL             = 27, /* FTM6 External Clock Pin Select */
  SIM_FTM7_CLOCKSEL             = 28, /* FTM7 External Clock Pin Select */
  SIM_CLKOUTSELL                = 29, /* CLKOUT Select */
  SIM_RTCCLK_CLK                = 30, /* RTCCLK clock */
  SIM_LPO_CLK                   = 31, /* LPO clock */
  SIM_LPO_1K_CLK                = 32, /* LPO 1KHz clock */
  SIM_LPO_32K_CLK               = 33, /* LPO 32KHz clock */
  SIM_LPO_128K_CLK              = 34, /* LPO 128KHz clock */
  SIM_EIM_CLK                   = 35, /* EIM clock source */
  SIM_ERM_CLK                   = 36, /* ERM clock source */
  SIM_DMA_CLK                   = 37, /* DMA clock source */
  SIM_MPU_CLK                   = 38, /* MPU clock source */
  SIM_MSCM_CLK                  = 39, /* MSCM clock source */
  QSPI_MODULE_SFIF_CLK_HYP      = 40, /* QSPI module SFIF clock source */
  QSPI_MODULE_CLK               = 41, /* QSPI module clock source */
  QSPI_MODULE_CLK_SFIF          = 42, /* QSPI module clock source SFIF */
  QSPI_MODULE_CLK_2XSFIF        = 43, /* QSPI module clock source 2XSFIF */
  SIM_END_OF_CLOCKS             = 44, /* End of SIM clocks */

  CMP0_CLK                      = 45, /* CMP0 clock source */
  CRC0_CLK                      = 46, /* CRC0 clock source */
  DMAMUX0_CLK                   = 47, /* DMAMUX0 clock source */
  EWM0_CLK                      = 48, /* EWM0 clock source */
  PORTA_CLK                     = 49, /* PORTA clock source */
  PORTB_CLK                     = 50, /* PORTB clock source */
  PORTC_CLK                     = 51, /* PORTC clock source */
  PORTD_CLK                     = 52, /* PORTD clock source */
  PORTE_CLK                     = 53, /* PORTE clock source */
  RTC0_CLK                      = 54, /* RTC0 clock source */
  SAI0_CLK                      = 55, /* SAI0 clock source */
  SAI1_CLK                      = 56, /* SAI1 clock source */
  PCC_END_OF_BUS_CLOCKS         = 57, /* End of BUS clocks */
  FLEXCAN0_CLK                  = 58, /* FlexCAN0 clock source */
  FLEXCAN1_CLK                  = 59, /* FlexCAN1 clock source */
  FLEXCAN2_CLK                  = 60, /* FlexCAN2 clock source */
  PDB0_CLK                      = 61, /* PDB0 clock source */
  PDB1_CLK                      = 62, /* PDB1 clock source */
  PCC_END_OF_SYS_CLOCKS         = 63, /* End of SYS clocks */
  FTFC0_CLK                     = 64, /* FTFC0 clock source */
  PCC_END_OF_SLOW_CLOCKS        = 65, /* End of SLOW clocks */
  ENET0_CLK                     = 66, /* ENET0 clock source */
  FTM0_CLK                      = 67, /* FTM0 clock source */
  FTM1_CLK                      = 68, /* FTM1 clock source */
  FTM2_CLK                      = 69, /* FTM2 clock source */
  FTM3_CLK                      = 70, /* FTM3 clock source */
  FTM4_CLK                      = 71, /* FTM4 clock source */
  FTM5_CLK                      = 72, /* FTM5 clock source */
  FTM6_CLK                      = 73, /* FTM6 clock source */
  FTM7_CLK                      = 74, /* FTM7 clock source */
  PCC_END_OF_ASYNCH_DIV1_CLOCKS = 75, /* End of ASYNCH DIV1 clocks */
  ADC0_CLK                      = 76, /* ADC0 clock source */
  ADC1_CLK                      = 77, /* ADC1 clock source */
  FLEXIO0_CLK                   = 78, /* FLEXIO0 clock source */
  LPI2C0_CLK                    = 79, /* LPI2C0 clock source */
  LPI2C1_CLK                    = 80, /* LPI2C1 clock source */
  LPIT0_CLK                     = 81, /* LPIT0 clock source */
  LPSPI0_CLK                    = 82, /* LPSPI0 clock source */
  LPSPI1_CLK                    = 83, /* LPSPI1 clock source */
  LPSPI2_CLK                    = 84, /* LPSPI2 clock source */
  LPTMR0_CLK                    = 85, /* LPTMR0 clock source */
  LPUART0_CLK                   = 86, /* LPUART0 clock source */
  LPUART1_CLK                   = 87, /* LPUART1 clock source */
  LPUART2_CLK                   = 88, /* LPUART2 clock source */
  QSPI0_CLK                     = 89, /* QSPI0 clock source */
  PCC_END_OF_ASYNCH_DIV2_CLOCKS = 90, /* End of ASYNCH DIV2 clocks */
  PCC_END_OF_CLOCKS             = 91, /* End of PCC clocks */
  CLOCK_NAME_COUNT              = 92, /* The total number of entries */
};

#endif /* __ARCH_ARM_SRC_S32K1XX_S32K14X_S32K14X_CLOCKNAMES_H */
