/****************************************************************************
 * arch/arm/src/s32k1xx/s32k14x/s32k14x_clockmapping.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "s32k1xx_periphclocks.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Clock name mappings.
 *
 * Each S32K1xx architecture must provide this array.  This is a constant
 * array storing the mappings between clock names and peripheral clock
 * control indexes.  If there is no peripheral clock control index for a
 * clock name, then the corresponding value is PCC_INVALID_INDEX.
 */

const uint16_t g_clkname_mapping[] =
{
  PCC_INVALID_INDEX,                  /* Core clock                      0  */
  PCC_INVALID_INDEX,                  /* Bus clock                       1  */
  PCC_INVALID_INDEX,                  /* Slow clock                      2  */
  PCC_INVALID_INDEX,                  /* CLKOUT clock                    3  */
  PCC_INVALID_INDEX,                  /* SIRC clock                      4  */
  PCC_INVALID_INDEX,                  /* FIRC clock                      5  */
  PCC_INVALID_INDEX,                  /* SOSC clock                      6  */
  PCC_INVALID_INDEX,                  /* SPLL clock                      7  */
  PCC_INVALID_INDEX,                  /* RTC_CLKIN clock                 8  */
  PCC_INVALID_INDEX,                  /* SCG CLK_OUT clock               9  */
  PCC_INVALID_INDEX,                  /* SIRCDIV1 functional clock       10 */
  PCC_INVALID_INDEX,                  /* SIRCDIV2 functional clock       11 */
  PCC_INVALID_INDEX,                  /* FIRCDIV1 functional clock       12 */
  PCC_INVALID_INDEX,                  /* FIRCDIV2 functional clock       13 */
  PCC_INVALID_INDEX,                  /* SOSCDIV1 functional clock       14 */
  PCC_INVALID_INDEX,                  /* SOSCDIV2 functional clock       15 */
  PCC_INVALID_INDEX,                  /* SPLLDIV1 functional clock       16 */
  PCC_INVALID_INDEX,                  /* SPLLDIV2 functional clock       17 */
  PCC_INVALID_INDEX,                  /* End of SCG clocks               18 */
  PCC_INVALID_INDEX,                  /* No clock entry                  19 */
  PCC_INVALID_INDEX,                  /* No clock entry                  20 */
  PCC_INVALID_INDEX,                  /* FTM0 External Clock Pin Select  21 */
  PCC_INVALID_INDEX,                  /* FTM1 External Clock Pin Select  22 */
  PCC_INVALID_INDEX,                  /* FTM2 External Clock Pin Select  23 */
  PCC_INVALID_INDEX,                  /* FTM3 External Clock Pin Select  24 */
  PCC_INVALID_INDEX,                  /* FTM4 External Clock Pin Select  25 */
  PCC_INVALID_INDEX,                  /* FTM5 External Clock Pin Select  26 */
  PCC_INVALID_INDEX,                  /* FTM6 External Clock Pin Select  27 */
  PCC_INVALID_INDEX,                  /* FTM7 External Clock Pin Select  28 */
  PCC_INVALID_INDEX,                  /* CLKOUT Select                   29 */
  PCC_INVALID_INDEX,                  /* CLK32K clock                    30 */
  PCC_INVALID_INDEX,                  /* LPO clock                       31 */
  PCC_INVALID_INDEX,                  /* LPO 1KHz clock                  32 */
  PCC_INVALID_INDEX,                  /* LPO 32KHz clock                 33 */
  PCC_INVALID_INDEX,                  /* LPO 128KHz clock                34 */
  PCC_INVALID_INDEX,                  /* EIM clock source                35 */
  PCC_INVALID_INDEX,                  /* ERM clock source                36 */
  PCC_INVALID_INDEX,                  /* DMA clock source                37 */
  PCC_INVALID_INDEX,                  /* MPU clock source                38 */
  PCC_INVALID_INDEX,                  /* MSCM clock source               39 */
  PCC_INVALID_INDEX,                  /* QSPI module SFIF clock source   40 */
  PCC_INVALID_INDEX,                  /* QSPI module clock source        41 */
  PCC_INVALID_INDEX,                  /* QSPI module clock source SFIF   42 */
  PCC_INVALID_INDEX,                  /* QSPI module clock source 2XSFIF 43 */
  PCC_INVALID_INDEX,                  /* No clock entry                  44 */
  PCC_CMP0_INDEX,                     /* CMP0 clock source               45 */
  PCC_CRC_INDEX,                      /* CRC clock source                46 */
  PCC_DMAMUX_INDEX,                   /* DMAMUX clock source             47 */
  PCC_EWM_INDEX,                      /* EWM clock source                48 */
  PCC_PORTA_INDEX,                    /* PORTA clock source              49 */
  PCC_PORTB_INDEX,                    /* PORTB clock source              50 */
  PCC_PORTC_INDEX,                    /* PORTC clock source              51 */
  PCC_PORTD_INDEX,                    /* PORTD clock source              52 */
  PCC_PORTE_INDEX,                    /* PORTE clock source              53 */
  PCC_RTC_INDEX,                      /* RTC clock source                54 */
  PCC_SAI0_INDEX,                     /* SAI0 clock source               55 */
  PCC_SAI1_INDEX,                     /* SAI1 clock source               56 */
  PCC_INVALID_INDEX,                  /* End of BUS clocks               57 */
  PCC_FLEXCAN0_INDEX,                 /* FlexCAN0 clock source           58 */
  PCC_FLEXCAN1_INDEX,                 /* FlexCAN1 clock source           59 */
  PCC_FLEXCAN2_INDEX,                 /* FlexCAN2 clock source           60 */
  PCC_PDB0_INDEX,                     /* PDB0 clock source               61 */
  PCC_PDB1_INDEX,                     /* PDB1 clock source               62 */
  PCC_INVALID_INDEX,                  /* End of SYS clocks               63 */
  PCC_FTFC_INDEX,                     /* FTFC clock source               64 */
  PCC_INVALID_INDEX,                  /* End of SLOW clocks              65 */
  PCC_ENET_INDEX,                     /* ENET clock source               66 */
  PCC_FTM0_INDEX,                     /* FTM0 clock source               67 */
  PCC_FTM1_INDEX,                     /* FTM1 clock source               68 */
  PCC_FTM2_INDEX,                     /* FTM2 clock source               69 */
  PCC_FTM3_INDEX,                     /* FTM3 clock source               70 */
  PCC_FTM4_INDEX,                     /* FTM4 clock source               71 */
  PCC_FTM5_INDEX,                     /* FTM5 clock source               72 */
  PCC_FTM6_INDEX,                     /* FTM6 clock source               73 */
  PCC_FTM7_INDEX,                     /* FTM7 clock source               74 */
  PCC_INVALID_INDEX,                  /* End of ASYNCH DIV1 clocks       75 */
  PCC_ADC0_INDEX,                     /* ADC0 clock source               76 */
  PCC_ADC1_INDEX,                     /* ADC1 clock source               77 */
  PCC_FLEXIO_INDEX,                   /* FLEXIO clock source             78 */
  PCC_LPI2C0_INDEX,                   /* LPI2C0 clock source             79 */
  PCC_LPI2C1_INDEX,                   /* LPI2C1 clock source             80 */
  PCC_LPIT_INDEX,                     /* LPIT clock source               81 */
  PCC_LPSPI0_INDEX,                   /* LPSPI0 clock source             82 */
  PCC_LPSPI1_INDEX,                   /* LPSPI1 clock source             83 */
  PCC_LPSPI2_INDEX,                   /* LPSPI2 clock source             84 */
  PCC_LPTMR0_INDEX,                   /* LPTMR0 clock source             85 */
  PCC_LPUART0_INDEX,                  /* LPUART0 clock source            86 */
  PCC_LPUART1_INDEX,                  /* LPUART1 clock source            87 */
  PCC_LPUART2_INDEX,                  /* LPUART2 clock source            88 */
  PCC_QSPI_INDEX,                     /* QSPI clock source               89 */
  PCC_INVALID_INDEX,                  /* End of ASYNCH DIV2 clocks       90 */
  PCC_INVALID_INDEX                   /* End of PCC clocks               91 */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
