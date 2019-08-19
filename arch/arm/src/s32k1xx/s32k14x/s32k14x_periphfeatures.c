/********************************************************************************************************************************************
 * arch/arm/src/s32k1xx/s32k14x/s32k14x_periphfeatures.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ********************************************************************************************************************************************/

/********************************************************************************************************************************************
 * Included Files
 ********************************************************************************************************************************************/

#include <nuttx/config.h>

#include "s32k1xx_periphclocks.h"

/****************************************************************************
 * Public Data
 ********************************************************************************************************************************************/

/* Peripheral Features.
 *
 * Each S32K1xx architecture must provide this array.  This is an array of
 * bit-encoded peripheral clocking features.  See the peripheral instance
 * feature definitions above
 */

const uint8_t g_periph_features[] =
{
  (NO_PERIPHERAL_FEATURE),                                                                            /* Core clock                      0  */
  (NO_PERIPHERAL_FEATURE),                                                                            /* Bus clock                       1  */
  (NO_PERIPHERAL_FEATURE),                                                                            /* Slow clock                      2  */
  (NO_PERIPHERAL_FEATURE),                                                                            /* CLKOUT clock                    3  */
  (NO_PERIPHERAL_FEATURE),                                                                            /* SIRC clock                      4  */
  (NO_PERIPHERAL_FEATURE),                                                                            /* FIRC clock                      5  */
  (NO_PERIPHERAL_FEATURE),                                                                            /* SOSC clock                      6  */
  (NO_PERIPHERAL_FEATURE),                                                                            /* SPLL clock                      7  */
  (NO_PERIPHERAL_FEATURE),                                                                            /* RTC_CLKIN clock                 8  */
  (NO_PERIPHERAL_FEATURE),                                                                            /* SCG CLK_OUT clock               9  */
  (NO_PERIPHERAL_FEATURE),                                                                            /* End of SCG clocks               10 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* No clock entry in clock_names_t 11 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* No clock entry in clock_names_t 12 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* No clock entry in clock_names_t 13 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* No clock entry in clock_names_t 14 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* No clock entry in clock_names_t 15 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* No clock entry in clock_names_t 16 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* No clock entry in clock_names_t 17 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* No clock entry in clock_names_t 18 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* No clock entry in clock_names_t 19 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* No clock entry in clock_names_t 20 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* FTM0 External Clock Pin Select  21 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* FTM1 External Clock Pin Select  22 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* FTM2 External Clock Pin Select  23 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* FTM3 External Clock Pin Select  24 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* FTM4 External Clock Pin Select  25 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* FTM5 External Clock Pin Select  26 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* FTM6 External Clock Pin Select  27 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* FTM7 External Clock Pin Select  28 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* CLKOUT Select                   29 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* CLK32K clock                    30 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* LPO clock                       31 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* LPO 1KHz clock                  32 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* LPO 32KHz clock                 33 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* LPO 128KHz clock                34 */
  (HAS_CLOCK_GATING_IN_SIM | HAS_INT_CLOCK_FROM_SYS_CLOCK),                                           /* EIM clock source                35 */
  (HAS_CLOCK_GATING_IN_SIM | HAS_INT_CLOCK_FROM_SYS_CLOCK),                                           /* ERM clock source                36 */
  (HAS_CLOCK_GATING_IN_SIM | HAS_INT_CLOCK_FROM_SYS_CLOCK),                                           /* DMA clock source                37 */
  (HAS_CLOCK_GATING_IN_SIM | HAS_INT_CLOCK_FROM_SYS_CLOCK),                                           /* MPU clock source                38 */
  (HAS_CLOCK_GATING_IN_SIM | HAS_INT_CLOCK_FROM_SYS_CLOCK),                                           /* MSCM clock source               39 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* QSPI module SFIF clock source   40 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* QSPI module clock source        41 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* QSPI module clock source SFIF   42 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* QSPI module clock source 2XSFIF 43 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* No clock entry in clock_names_t 44 */
  (HAS_INT_CLOCK_FROM_BUS_CLOCK),                                                                     /* CMP0 clock source               45 */
  (HAS_INT_CLOCK_FROM_BUS_CLOCK),                                                                     /* CRC clock source                46 */
  (HAS_INT_CLOCK_FROM_BUS_CLOCK),                                                                     /* DMAMUX clock source             47 */
  (HAS_INT_CLOCK_FROM_BUS_CLOCK),                                                                     /* EWM clock source                48 */
  (HAS_INT_CLOCK_FROM_BUS_CLOCK),                                                                     /* PORTA clock source              49 */
  (HAS_INT_CLOCK_FROM_BUS_CLOCK),                                                                     /* PORTB clock source              50 */
  (HAS_INT_CLOCK_FROM_BUS_CLOCK),                                                                     /* PORTC clock source              51 */
  (HAS_INT_CLOCK_FROM_BUS_CLOCK),                                                                     /* PORTD clock source              52 */
  (HAS_INT_CLOCK_FROM_BUS_CLOCK),                                                                     /* PORTE clock source              53 */
  (HAS_INT_CLOCK_FROM_BUS_CLOCK),                                                                     /* RTC clock source                54 */
  (HAS_INT_CLOCK_FROM_BUS_CLOCK),                                                                     /* SAI0 clock source               55 */
  (HAS_INT_CLOCK_FROM_BUS_CLOCK),                                                                     /* SAI1 clock source               56 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* End of BUS clocks               57 */
  (HAS_INT_CLOCK_FROM_SYS_CLOCK),                                                                     /* FlexCAN0 clock source           58 */
  (HAS_INT_CLOCK_FROM_SYS_CLOCK),                                                                     /* FlexCAN1 clock source           59 */
  (HAS_INT_CLOCK_FROM_SYS_CLOCK),                                                                     /* FlexCAN2 clock source           60 */
  (HAS_INT_CLOCK_FROM_SYS_CLOCK),                                                                     /* PDB0 clock source               61 */
  (HAS_INT_CLOCK_FROM_SYS_CLOCK),                                                                     /* PDB1 clock source               62 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* End of SYS clocks               63 */
  (HAS_INT_CLOCK_FROM_SLOW_CLOCK),                                                                    /* FTFC clock source               64 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* End of SLOW clocks              65 */
  (HAS_MULTIPLIER | HAS_DIVIDER | HAS_PROTOCOL_CLOCK_FROM_ASYNC2 | HAS_INT_CLOCK_FROM_SYS_CLOCK),     /* ENET clock source               66 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC1 | HAS_INT_CLOCK_FROM_SYS_CLOCK),                                    /* FTM0 clock source               67 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC1 | HAS_INT_CLOCK_FROM_SYS_CLOCK),                                    /* FTM1 clock source               68 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC1 | HAS_INT_CLOCK_FROM_SYS_CLOCK),                                    /* FTM2 clock source               69 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC1 | HAS_INT_CLOCK_FROM_SYS_CLOCK),                                    /* FTM3 clock source               70 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC1 | HAS_INT_CLOCK_FROM_SYS_CLOCK),                                    /* FTM4 clock source               71 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC1 | HAS_INT_CLOCK_FROM_SYS_CLOCK),                                    /* FTM5 clock source               72 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC1 | HAS_INT_CLOCK_FROM_SYS_CLOCK),                                    /* FTM6 clock source               73 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC1 | HAS_INT_CLOCK_FROM_SYS_CLOCK),                                    /* FTM7 clock source               74 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* End of ASYNCH DIV1 clocks       75 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC2 | HAS_INT_CLOCK_FROM_BUS_CLOCK),                                    /* ADC0 clock source               76 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC2 | HAS_INT_CLOCK_FROM_BUS_CLOCK),                                    /* ADC1 clock source               77 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC2 | HAS_INT_CLOCK_FROM_BUS_CLOCK),                                    /* FLEXIO clock source             78 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC2 | HAS_INT_CLOCK_FROM_BUS_CLOCK),                                    /* LPI2C0 clock source             79 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC2 | HAS_INT_CLOCK_FROM_BUS_CLOCK),                                    /* LPI2C1 clock source             80 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC2 | HAS_INT_CLOCK_FROM_BUS_CLOCK),                                    /* LPIT clock source               81 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC2 | HAS_INT_CLOCK_FROM_BUS_CLOCK),                                    /* LPSPI0 clock source             82 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC2 | HAS_INT_CLOCK_FROM_BUS_CLOCK),                                    /* LPSPI1 clock source             83 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC2 | HAS_INT_CLOCK_FROM_BUS_CLOCK),                                    /* LPSPI2 clock source             84 */
  (HAS_MULTIPLIER | HAS_DIVIDER | HAS_PROTOCOL_CLOCK_FROM_ASYNC2 | HAS_INT_CLOCK_FROM_BUS_CLOCK),     /* LPTMR0 clock source             85 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC2 | HAS_INT_CLOCK_FROM_BUS_CLOCK),                                    /* LPUART0 clock source            86 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC2 | HAS_INT_CLOCK_FROM_BUS_CLOCK),                                    /* LPUART1 clock source            87 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC2 | HAS_INT_CLOCK_FROM_BUS_CLOCK),                                    /* LPUART2 clock source            88 */
  (HAS_INT_CLOCK_FROM_BUS_CLOCK),                                                                     /* QSPI clock source               89 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* End of ASYNCH DIV2 clocks       90 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* End of PCC clocks               91 */
  };
