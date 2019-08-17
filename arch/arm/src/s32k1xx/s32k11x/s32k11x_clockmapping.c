/****************************************************************************
 * arch/arm/src/s32k1xx/s32k11x/s32k11x_clockmapping.c
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
  PCC_INVALID_INDEX,                  /* No clock entry                  7  */
  PCC_INVALID_INDEX,                  /* RTC_CLKIN clock                 8  */
  PCC_INVALID_INDEX,                  /* SCG CLK_OUT clock               9  */
  PCC_INVALID_INDEX,                  /* SIRCDIV1 functional clock       10 */
  PCC_INVALID_INDEX,                  /* SIRCDIV2 functional clock       11 */
  PCC_INVALID_INDEX,                  /* FIRCDIV1 functional clock       12 */
  PCC_INVALID_INDEX,                  /* FIRCDIV2 functional clock       13 */
  PCC_INVALID_INDEX,                  /* SOSCDIV1 functional clock       14 */
  PCC_INVALID_INDEX,                  /* SOSCDIV2 functional clock       15 */
  PCC_INVALID_INDEX,                  /* No clock entry                  16 */
  PCC_INVALID_INDEX,                  /* No clock entry                  17 */
  PCC_INVALID_INDEX,                  /* End of SCG clocks               18 */
  PCC_INVALID_INDEX,                  /* No clock entry                  19 */
  PCC_INVALID_INDEX,                  /* No clock entry                  20 */
  PCC_INVALID_INDEX,                  /* FTM0 External Clock Pin Select  21 */
  PCC_INVALID_INDEX,                  /* FTM1 External Clock Pin Select  22 */
  PCC_INVALID_INDEX,                  /* CLKOUT Select                   23 */
  PCC_INVALID_INDEX,                  /* CLK32K clock                    24 */
  PCC_INVALID_INDEX,                  /* LPO clock                       25 */
  PCC_INVALID_INDEX,                  /* LPO 1KHz clock                  26 */
  PCC_INVALID_INDEX,                  /* LPO 32KHz clock                 27 */
  PCC_INVALID_INDEX,                  /* LPO 128KHz clock                28 */
  PCC_INVALID_INDEX,                  /* EIM clock source                29 */
  PCC_INVALID_INDEX,                  /* ERM clock source                30 */
  PCC_INVALID_INDEX,                  /* DMA clock source                31 */
  PCC_INVALID_INDEX,                  /* MPU clock source                32 */
  PCC_INVALID_INDEX,                  /* MSCM clock source               33 */
  PCC_INVALID_INDEX,                  /* No clock entry                  34 */
  PCC_INVALID_INDEX,                  /* No clock entry                  35 */
  PCC_INVALID_INDEX,                  /* No clock entry                  36 */
  PCC_INVALID_INDEX,                  /* No clock entry                  37 */
  PCC_INVALID_INDEX,                  /* No clock entry                  38 */
  PCC_INVALID_INDEX,                  /* No clock entry                  39 */
  PCC_INVALID_INDEX,                  /* No clock entry                  40 */
  PCC_CMP0_INDEX,                     /* CMP0 clock source               41 */
  PCC_CRC_INDEX,                      /* CRC clock source                42 */
  PCC_DMAMUX_INDEX,                   /* DMAMUX clock source             43 */
  PCC_PORTA_INDEX,                    /* PORTA clock source              44 */
  PCC_PORTB_INDEX,                    /* PORTB clock source              45 */
  PCC_PORTC_INDEX,                    /* PORTC clock source              46 */
  PCC_PORTD_INDEX,                    /* PORTD clock source              47 */
  PCC_PORTE_INDEX,                    /* PORTE clock source              48 */
  PCC_RTC_INDEX,                      /* RTC clock source                49 */
  PCC_INVALID_INDEX,                  /* End of BUS clocks               50 */
  PCC_FLEXCAN0_INDEX,                 /* FlexCAN0 clock source           51 */
  PCC_PDB0_INDEX,                     /* PDB0 clock source               52 */
  PCC_INVALID_INDEX,                  /* End of SYS clocks               53 */
  PCC_FTFC_INDEX,                     /* FTFC clock source               54 */
  PCC_INVALID_INDEX,                  /* End of SLOW clocks              55 */
  PCC_FTM0_INDEX,                     /* FTM0 clock source               56 */
  PCC_FTM1_INDEX,                     /* FTM1 clock source               57 */
  PCC_INVALID_INDEX,                  /* End of ASYNCH DIV1 clocks       58 */
  PCC_ADC0_INDEX,                     /* ADC0 clock source               59 */
  PCC_FLEXIO_INDEX,                   /* FlexIO clock source             60 */
  PCC_LPI2C0_INDEX,                   /* LPI2C0 clock source             61 */
  PCC_LPIT_INDEX,                     /* LPIT clock source               62 */
  PCC_LPSPI0_INDEX,                   /* LPSPI0 clock source             63 */
  PCC_LPSPI1_INDEX,                   /* LPSPI1 clock source             64 */
  PCC_LPTMR0_INDEX,                   /* LPTMR0 clock source             65 */
  PCC_LPUART0_INDEX,                  /* LPUART0 clock source            66 */
  PCC_LPUART1_INDEX,                  /* LPUART1 clock source            67 */
  PCC_INVALID_INDEX,                  /* End of ASYNCH DIV2 clocks       68 */
  PCC_INVALID_INDEX,                  /* End of PCC clocks               69 */
};
