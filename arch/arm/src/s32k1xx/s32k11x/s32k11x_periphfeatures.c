/********************************************************************************************************************************************
 * arch/arm/src/s32k1xx/s32k11x/s32k11x_periphfeatures.c
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
  (NO_PERIPHERAL_FEATURE),                                                                            /* No clock entry in clock_names_t 7  */
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
  (NO_PERIPHERAL_FEATURE),                                                                            /* CLKOUT Select                   23 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* CLK32K clock                    24 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* LPO clock                       25 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* LPO 1KHz clock                  26 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* LPO 32KHz clock                 27 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* LPO 128KHz clock                28 */
  (HAS_CLOCK_GATING_IN_SIM | HAS_INT_CLOCK_FROM_SYS_CLOCK),                                           /* EIM clock source                29 */
  (HAS_CLOCK_GATING_IN_SIM | HAS_INT_CLOCK_FROM_SYS_CLOCK),                                           /* ERM clock source                30 */
  (HAS_CLOCK_GATING_IN_SIM | HAS_INT_CLOCK_FROM_SYS_CLOCK),                                           /* DMA clock source                31 */
  (HAS_CLOCK_GATING_IN_SIM | HAS_INT_CLOCK_FROM_SYS_CLOCK),                                           /* MPU clock source                32 */
  (HAS_CLOCK_GATING_IN_SIM | HAS_INT_CLOCK_FROM_SYS_CLOCK),                                           /* MSCM clock source               33 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* No clock entry in clock_names_t 34 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* No clock entry in clock_names_t 35 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* No clock entry in clock_names_t 36 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* No clock entry in clock_names_t 37 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* No clock entry in clock_names_t 38 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* No clock entry in clock_names_t 39 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* No clock entry in clock_names_t 40 */
  (HAS_INT_CLOCK_FROM_BUS_CLOCK),                                                                     /* CMP0 clock source               41 */
  (HAS_INT_CLOCK_FROM_BUS_CLOCK),                                                                     /* CRC clock source                42 */
  (HAS_INT_CLOCK_FROM_BUS_CLOCK),                                                                     /* DMAMUX clock source             43 */
  (HAS_INT_CLOCK_FROM_BUS_CLOCK),                                                                     /* PORTA clock source              44 */
  (HAS_INT_CLOCK_FROM_BUS_CLOCK),                                                                     /* PORTB clock source              45 */
  (HAS_INT_CLOCK_FROM_BUS_CLOCK),                                                                     /* PORTC clock source              46 */
  (HAS_INT_CLOCK_FROM_BUS_CLOCK),                                                                     /* PORTD clock source              47 */
  (HAS_INT_CLOCK_FROM_BUS_CLOCK),                                                                     /* PORTE clock source              48 */
  (HAS_INT_CLOCK_FROM_BUS_CLOCK),                                                                     /* RTC clock source                49 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* End of BUS clocks               50 */
  (HAS_INT_CLOCK_FROM_SYS_CLOCK),                                                                     /* FlexCAN0 clock source           51 */
  (HAS_INT_CLOCK_FROM_SYS_CLOCK),                                                                     /* PDB0 clock source               52 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* End of SYS clocks               53 */
  (HAS_INT_CLOCK_FROM_SLOW_CLOCK),                                                                    /* FTFC clock source               54 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* End of SLOW clocks              55 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC1 | HAS_INT_CLOCK_FROM_SYS_CLOCK),                                    /* FTM0 clock source               56 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC1 | HAS_INT_CLOCK_FROM_SYS_CLOCK),                                    /* FTM1 clock source               57 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* End of ASYNCH DIV1 clocks       58 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC2 | HAS_INT_CLOCK_FROM_BUS_CLOCK),                                    /* ADC0 clock source               59 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC2 | HAS_INT_CLOCK_FROM_BUS_CLOCK),                                    /* FLEXIO clock source             60 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC2 | HAS_INT_CLOCK_FROM_BUS_CLOCK),                                    /* LPI2C0 clock source             61 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC2 | HAS_INT_CLOCK_FROM_BUS_CLOCK),                                    /* LPIT clock source               62 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC2 | HAS_INT_CLOCK_FROM_BUS_CLOCK),                                    /* LPSPI0 clock source             63 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC2 | HAS_INT_CLOCK_FROM_BUS_CLOCK),                                    /* LPSPI1 clock source             64 */
  (HAS_MULTIPLIER | HAS_DIVIDER | HAS_PROTOCOL_CLOCK_FROM_ASYNC2 | HAS_INT_CLOCK_FROM_BUS_CLOCK),     /* LPTMR0 clock source             65 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC2 | HAS_INT_CLOCK_FROM_BUS_CLOCK),                                    /* LPUART0 clock source            66 */
  (HAS_PROTOCOL_CLOCK_FROM_ASYNC2 | HAS_INT_CLOCK_FROM_BUS_CLOCK),                                    /* LPUART1 clock source            67 */
  (NO_PERIPHERAL_FEATURE),                                                                            /* End of ASYNCH DIV2 clocks       68 */
  (NO_PERIPHERAL_FEATURE)                                                                             /* End of PCC clocks               69 */
};
